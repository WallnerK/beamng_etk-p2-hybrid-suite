-- etk854hybrid_launchControl.lua - ETK 854 ActiveHybrid - launch control
--
-- Brake + full pedal at a standstill preloads the drivetrain against the brakes
-- (ICE builds boost under load, the rear squats) without creeping; releasing the
-- brake fires the launch. Uncapped, the stall torque would turn the rear wheels
-- through their brakes, so PRELOAD / READY use:
--   * pump torque cap T_ff = MARGIN x C_hold / (stall ratio x cumulativeGearRatio),
--     C_hold = C_R + r_R x min(mu N_R, mu N_F, C_F / r_F): rear brakes plus the
--     rear tyres pushing the body against the front brakes (what squats the rear)
--   * shaft speed target n* = sqrt(T* / c), T* = T_ff x anti-creep trim: the
--     converter turns a held shaft speed into exactly that pump torque
--   * K0 closed, ICE loaded for boost, E-machine never drives (evThrottle 0).
--     GEN_LOAD: 0 pure ICE (ICE torque = pump torque, little boost) | 1 GEN: the
--     ICE makes what the generator can absorb on top of the cap, the generator
--     holds the shaft speed | 2 GEN on petrol engines only
--   * regulator: GEN = ICE torque feed-forward (a throttle PI on shaft speed
--     limit-cycles at graphics rate) + generator speed loop; pure ICE = the speed
--     loop in Nm through the inverse model; both on the frame-mean shaft speed.
--   * DSC hold assist at physics rate: full brakeTorque on every wheel, so the
--     capacity does not depend on the driver's pedal
--   * anti-creep trim on turbine / wheel / body speed. n* is clamped to idle +
--     N_MIN_OVER_IDLE; when the trim needs less, the ICE torque is cut ('trim-floor').
-- FIRE (brake < BRAKE_FIRE, pedal floored): generator and assist off, torqueCoord
-- re-seeded with the shaft torque (launchSeedNm), LAUNCH row for WINDOW_S.
-- GUARD (brake + pedal at a standstill, any mode): the same capacity model on the
-- pedal brake torque as a cap on the coordinator request (launchGuardNm). Instant
-- while the brake is held; while it is released only after GUARD_DWELL_S, so a
-- gamepad trigger cross-over at a normal start stays out of it.
-- With K0 closed the ICE speed is the stall speed (~1400-1550 rpm with the assist).
-- AWD: the stock split-shaft lock stays open while braked at a standstill, so the
-- car holds like RWD; a closed lock (clutchRatio >= AWD_LOCK_R) drives both axles
-- against their own brakes.
--
-- etk854hybrid_launchState (Actions app) IDLE | ARMED | READY | ACTIVE (PRELOAD
-- published as ARMED; the shift logic holds gear 1 in READY), plus launch* diagnostics.
-- ctx: launchArmed / Ready / Active (iceSequencer forces the ICE on), launchHold
-- (torqueCoord hold branch), launchIceThr, launchGenNm, launchPumpCapNm,
-- launchTargetRpm, launchGuardNm (request cap), launchSeedNm (one-shot at FIRE),
-- launchDrop (one-shot torqueCoord reset after an abort).
-- All constants in LC, all state in st (LuaJIT 60-upvalue limit).

local M = {}
M.name = "launchControl"
M.isReal = true

local abs, max, min, sqrt = math.abs, math.max, math.min, math.sqrt
local RPM_TO_AV = 0.104719755
local AV_TO_RPM = 9.549296596

-- ===========================================================
-- Parameters
-- ===========================================================
local LC = {
  -- pedal thresholds (raw throttle / brake)
  BRAKE_ENTER = 0.45, BRAKE_HOLD = 0.30, BRAKE_FIRE = 0.15,   -- ENTER < 0.5: a half-pressed brake enters
  PEDAL_ENTER = 0.90, PEDAL_HOLD = 0.80, PEDAL_HOLD_S = 0.20, PEDAL_FIRE = 0.90,
  PEDAL_REARM = 0.20,                -- after an abort / a launch: lift below this first
  V_ENTER_KMH = 0.5,
  V_OVER_KMH = 1.5,                  -- hold failed: ICE torque 0 for the rest of the hold
  V_DISARM_KMH = 5.0,                -- driven off without a launch
  -- auto-arm (gamepad path): SPORT, DTC / DSC OFF, D/S/M, brake + kickdown for AUTO_S
  AUTO_ARM = true, AUTO_PEDAL = 0.95, AUTO_BRAKE = 0.45, AUTO_S = 0.30,
  ARM_TIMEOUT_S = 60,
  HOLD_MAX_S = 8.0,                  -- UX limit (no converter thermal model)
  READY_TOL_RPM = 100, READY_SETTLE_S = 0.5, READY_MAX_S = 1.5,
  WINDOW_S = 3.0,                    -- LAUNCH window after FIRE
  ACTIVE_ABORT_BRAKE = 0.2, ACTIVE_ABORT_PEDAL = 0.2,
  COOLANT_MAX_C = 112,
  -- capacity model
  MARGIN = 0.75,                     -- share of C_hold that may reach the axle at stall
  GROUND_PATH = true,                -- count the rear tyre -> front brake path
  MU_HOLD = 0.80,                    -- static tyre mu, conservative (spin is caught by the trim)
  AXLE_N_MIN = 1500,                 -- [N] below this: no tyre path
  LOAD_TAU_S = 0.5,                  -- downForce LPF; latched in a hold (the squat moves load)
  LOAD_FALLBACK = 0.80,              -- no downForce yet: 0.8 x m g / 2 per axle
  MASS_FALLBACK_KG = 1900,
  BRAKE_ASSIST = true,               -- DSC hold assist in PRELOAD / READY
  AWD_REAR_SHARE = 0.60,             -- lock clutch closed only
  AWD_LOCK_R = 0.90,                 -- transfercase.clutchRatio above which it counts
  RATIO_MIN = 0.5, RATIO_MAX = 40, RATIO_FALLBACK = 15,   -- conservative (larger = lower cap)
  C_FALLBACK = 0.0075726,            -- c for D 0.30 / S 12
  TR_FALLBACK = 1.7,
  N_MIN_OVER_IDLE = 150,             -- no preload below idle + this ("brake harder")
  N_MAX_DIESEL = 2600, N_MAX_PETROL = 3500,
  N_RAMP_UP = 1500, N_RAMP_DN = 3000,  -- [rpm/s] reference ramp
  -- generator load
  GEN_LOAD = 1,                      -- 0 off / 1 on / 2 petrol only
  -- regen curve ~300 Nm at the hold speed; stageRegenKw / w bounds it per stage
  GEN_MAX_NM = 290, GEN_SOC_MAX = 0.92, GEN_MIN_NM = 30,
  GEN_KP = 0.25,                     -- [Nm/rpm] loop gain KP x 9.55 x dt / J = 0.44
                                     -- (20 fps, lightest shaft J 0.27 kg m^2)
  GEN_KI = 1.0,                      -- [Nm/(rpm*s)]
  GEN_I_MAX = 150,                   -- [Nm] small: the feed-forward does the work
  GEN_ICE_PRED = 2.0,                -- + this x the ICE torque rise per frame (~2 frames lag)
  RPM_TAU_S = 0.01,                  -- short: the frame mean already anti-aliases
  -- ICE torque feed-forward in GEN mode
  GEN_ICE_SHARE = 0.85,              -- target T* + this x Gmax (15 % headroom)
  GEN_ICE_HARD = 1.00,               -- hard cap T* + this x Gmax
  ICE_TQ_KI_UP = 0.006,              -- [1/(Nm*s)] throttle integrator, ICE below target
  ICE_TQ_KI_DN = 0.003,              -- [1/(Nm*s)] ... above it
  ICE_BOOST_RISE = 0.3,              -- [1/s] min. rise while out of reach (a jump to 1 pulses)
  ICE_THR_RISE = 4.0,                -- [1/s] max. hold throttle rise
  GEN_TRIM_E_RPM = 25,               -- generator saturated + shaft this far too fast:
  GEN_TRIM_DN = 300, GEN_TRIM_UP = 100,   -- [Nm/s] ICE target trim (model error)
  -- pure-ICE regulation (GEN off / pack full)
  ICE_KP = 0.30,                     -- [Nm/rpm]
  ICE_KI = 1.5,                      -- [Nm/(rpm*s)]
  ICE_I_MAX = 80,                    -- [Nm]
  ICE_TAU_S = 0.03,                  -- combustion torque LPF
  -- anti-creep trim
  CREEP_TURBINE_RPM = 40,            -- ~0.1 m/s at the wheel in gear 1
  CREEP_WHEEL_MS = 0.083, CREEP_BODY_MS = 0.083,
  HARD_TURBINE_RPM = 150, HARD_WHEEL_MS = 0.30,
  CREEP_TAU_S = 0.05, BODY_TAU_S = 0.10,
  SETTLE_TURBINE_RPM = 20, SETTLE_MS = 0.05, SETTLE_MAX_S = 0.5,
  TRIM_SOFT_RATE = 1.5,              -- [1/s]
  TRIM_HARD_MUL = 0.70, TRIM_HARD_EVERY_S = 0.25, HARD_CUT_S = 0.20,
  TRIM_QUIET_S = 0.5, TRIM_RECOVER = 0.10, TRIM_MIN = 0.30,
  -- generator unload after a hold ends without a fire (ICE at full throttle:
  -- dropping the generator at once would put ~400 Nm on the shaft)
  UNLOAD_MAX_S = 0.8, UNLOAD_DONE_NM = 15,
  UNLOAD_SKIP_PEDAL = 0.50,          -- released with this much pedal: no unload (torque hole)
  -- brake-throttle guard outside the hold
  GUARD_ENABLE = true, GUARD_MARGIN = 0.75,
  GUARD_BRAKE = 0.35, GUARD_PEDAL = 0.30, GUARD_V_KMH = 1.0,
  GUARD_DWELL_S = 0.20,              -- dwell only while the brake is being released:
  GUARD_FALL_RATE = 0.30,            -- ... it fell faster than this [1/s]
  GUARD_FALL_HOLD_S = 0.12,          -- ... within the last this [s]
  GUARD_BRAKE_FALL = 3.0,            -- [1/s] no overlap while the brake falls faster
  BRAKE_RATE_TAU_S = 0.05,
  GUARD_BRAKE_OUT = 0.30, GUARD_PEDAL_OUT = 0.20, GUARD_V_OUT_KMH = 3.0,
  -- self-leveling hold (etk854hybrid_launchLevelHold)
  LEVEL_HOLD_S = 2.0,
  SPRING_TAU_S = 0.5,                -- squat reference LPF
  SOC_MIN_BOOST = 0.10,
}

local PUBLIC = { OFF = "IDLE", ARMED = "ARMED", PRELOAD = "ARMED", READY = "READY",
                 FIRE = "ACTIVE", ABORT = "IDLE" }

local st = {
  phase = "OFF", why = "none", reason = "none", armSrc = "none", armDe = nil,
  armT = 0, autoT = 0, holdT = 0, preT = 0, readyT = 0, pedalLowT = 0, activeT = 0,
  rearmBlock = false, weakMsg = false,
  mode = "-",                        -- "GEN" | "ICE" | "-"
  trim = 1, quietT = 0, hardT = 0, cutT = 0, cut = false, hardN = 0,
  settled = false, settleT = 0,
  turbF = 0, whlF = 0, bodyF = 0, creep = 0,
  capNm = 0, tFf = 0, tStar = 0, nStar = 0, nRef = 0, c = LC.C_FALLBACK, tr = LC.TR_FALLBACK, ratio = 0,
  gI = 0, iI = 0, G = 0, gSat = false,
  iceThr = 0, iceLpf = 0, regenFrac = 0,
  nReg = 0,                          -- regulator shaft speed [rpm]
  thrI = -1,                         -- GEN: ICE throttle integrator (< 0 = seed next frame)
  iceTrim = 0, iceCapNm = 0,         -- GEN: ICE torque target trim / target [Nm]
  -- physics-rate accumulators (wheelsIntermediate)
  accOn = false, accAV = 0, accTq = 0, accN = 0,
  unloadT = 0, unloadCap = 0, assistOn = false,
  guard = false, guardNm = 0, guardDwellT = 0, brakePrev = nil, brakeRate = 0, brakeFallT = 9,
  capR = 0, capGround = 0, floorBind = false,
  nR = 0, nF = 0, massKg = 0,
  levelHoldT = 0,
  springs = nil, springRef = nil, springNow = 0,
  stat = { turbMax = 0, whlMax = 0, bodyMax = 0, trimMin = 1, genSum = 0, genN = 0 },
  -- devices / wheels
  ice = nil, motor = nil, tc = nil, gb = nil, tcase = nil,
  rear = {}, front = {}, all = {}, wheelCount = 0,
}

-- ===========================================================
-- Helpers
-- ===========================================================
local U = require("controller/etk854hybrid/etk854hybrid_util")
local clamp = U.clamp
local clamp01 = U.clamp01NanZero -- NaN -> 0

local function readNumber(data, key, default)
  if type(data) ~= "table" then return default end
  local v = data[key]
  if type(v) == "number" then return v end
  if type(v) == "string" then return tonumber(v) or default end
  if type(v) == "boolean" then return v and 1 or 0 end
  return default
end

local function msg(text, sec)
  if guihooks and guihooks.message then guihooks.message(text, sec or 3, "vehicle.launch") end
end

local function lcLog(fmt, ...)
  log("I", "etk854hybrid.launch", string.format(fmt, ...))
end

-- forward gear: D / S* / M* (the game reports "S1" / "M2")
local function gearFwd(ctx)
  if (ctx.gearSign or 0) <= 0 then return false end
  local g = electrics.values.gear
  if type(g) ~= "string" then return true end
  local c1 = g:sub(1, 1)
  return c1 == "D" or c1 == "S" or c1 == "M"
end

local function gearManualOk(ctx)
  local g = electrics.values.gear
  if type(g) ~= "string" then return false end
  local c1 = g:sub(1, 1)
  if c1 == "S" or c1 == "M" then return true end
  return c1 == "D" and ctx.deMode == "SPORT"
end

local function iceReady(ctx)
  return ctx.iceEngageState == "ON" and ctx.iceCoupled == true
end

local function curves(ctx)
  return ctx.modules and ctx.modules.curves
end

-- ===========================================================
-- Devices / wheels / springs
-- ===========================================================
local function findDevices()
  if not (powertrain and powertrain.getDevice) then return end
  st.ice = powertrain.getDevice("mainEngine")
  st.motor = powertrain.getDevice("integratedMotor")
  st.tc = powertrain.getDevice("torqueConverter")
  st.gb = powertrain.getDevice("gearbox")
  st.tcase = powertrain.getDevice("transfercase")
end

local function findWheels()
  st.rear, st.front, st.all, st.wheelCount = {}, {}, {}, 0
  if not (wheels and wheels.wheels) then return end
  local n = wheels.wheelCount or 0
  st.wheelCount = n
  for i = 0, n - 1 do
    local wd = wheels.wheels[i]
    if wd then
      st.all[#st.all + 1] = wd
      local nm = type(wd.name) == "string" and wd.name or ""
      local c1 = nm:sub(1, 1)
      if c1 == "R" then st.rear[#st.rear + 1] = wd elseif c1 == "F" then st.front[#st.front + 1] = wd end
    end
  end
end

-- self-leveling actuators spring_RR / spring_RL, read only for the squat channel
local function findSprings()
  st.springs = nil
  if not (v and v.data and v.data.beams and obj and obj.getBeamLength) then return end
  local want = { spring_RR = true, spring_RL = true }
  local cids = {}
  for _, b in pairs(v.data.beams) do
    if b.name and want[b.name] and b.cid then cids[#cids + 1] = b.cid end
  end
  if #cids == 2 then st.springs = cids end
end

local function springLen()
  local s = st.springs
  if not s then return 0 end
  return 0.5 * (obj:getBeamLength(s[1]) + obj:getBeamLength(s[2]))
end

-- vehicle mass [kg] from the node weights (load fallback only)
local function findMass()
  st.massKg = 0
  if not (v and v.data and v.data.nodes) then return end
  local m = 0
  for _, n in pairs(v.data.nodes) do
    if type(n) == "table" then m = m + (tonumber(n.nodeWeight) or 0) end
  end
  if m > 300 and m < 10000 then st.massKg = m end
end

-- ===========================================================
-- Capacity model
-- ===========================================================
-- Holding torque of one friction brake [Nm] as wheels.lua applies it:
--   desired = brakeTorque * (min(b, split) + max(b - split, 0) * splitCoef),
--     b = electrics[brakeElectricsName] * usage; parking brake as a floor
--   applied = desired * brakeThermalEfficiency
-- With the hold assist: the full brakeTorque.
local function wheelHoldNm(wd, assist)
  local bt = wd.brakeTorque or 0
  local eff = wd.brakeThermalEfficiency or 1
  if assist then return bt * eff end
  local usage = wd.defaultBrakeInputUsageCoef or 1
  local b = (electrics.values[wd.brakeElectricsName or "brake"] or 0) * usage
  local split = wd.brakeInputSplit or 1
  local d = bt * (min(b, split) + max(b - split, 0) * (wd.brakeSplitCoef or 1))
  local pk = (wd.parkingTorque or 0) * ((input and input.parkingbrake) or 0) * usage
  return max(d, pk) * eff
end

local function axleHoldNm(list, assist)
  local s = 0
  for i = 1, #list do
    local wd = list[i]
    if not wd.isBroken then s = s + wheelHoldNm(wd, assist) end
  end
  return s
end

local function axleRadius(list)
  local s, n = 0, 0
  for i = 1, #list do
    local r = tonumber(list[i].dynamicRadius) or tonumber(list[i].radius)
    if r and r > 0.1 and r < 1 then s, n = s + r, n + 1 end
  end
  return (n > 0) and s / n or 0.34
end

local function axleDownForce(list)
  local s = 0
  for i = 1, #list do s = s + max(0, tonumber(list[i].downForce) or 0) end
  return s
end

-- Normal force per axle [N] from wheels downForce: filtered while free, latched
-- in a hold / guard, static fallback when implausible.
local function updateLoads(dt, latch)
  local fb = LC.LOAD_FALLBACK * 0.5 * (st.massKg > 0 and st.massKg or LC.MASS_FALLBACK_KG) * 9.81
  local nR, nF = axleDownForce(st.rear), axleDownForce(st.front)
  local tot = nR + nF
  local mg = 2 * fb / LC.LOAD_FALLBACK
  if tot < 0.3 * mg or tot > 3 * mg then nR, nF = fb, fb end
  if st.nR <= 0 or st.nF <= 0 then
    st.nR, st.nF = nR, nF
  elseif not latch then
    local a = min(1, dt / LC.LOAD_TAU_S)
    st.nR = st.nR + (nR - st.nR) * a
    st.nF = st.nF + (nF - st.nF) * a
  end
end

-- C_hold at the driven axle [Nm]. RWD, 2WD and AWD with the lock open: rear
-- brakes + tyre path to the front brakes. Sets st.capR / st.capGround.
local function holdCapacity(assist)
  local rear = st.rear
  st.capR, st.capGround = 0, 0
  if #rear == 0 then return 0.5 * axleHoldNm(st.all, assist) end
  local cR = axleHoldNm(rear, assist)
  st.capR = cR
  if #st.front == 0 then return cR end
  local cF = axleHoldNm(st.front, assist)
  local tcase = st.tcase
  if tcase and tcase.mode ~= "disconnected" and (tonumber(tcase.clutchRatio) or 0) >= LC.AWD_LOCK_R then
    -- both axles driven: each brake holds its own share, no tyre path
    return min(cR / LC.AWD_REAR_SHARE, cF / (1 - LC.AWD_REAR_SHARE))
  end
  if LC.GROUND_PATH and st.nR >= LC.AXLE_N_MIN and st.nF >= LC.AXLE_N_MIN then
    local rR, rF = axleRadius(rear), axleRadius(st.front)
    local f = min(LC.MU_HOLD * st.nR, LC.MU_HOLD * st.nF, cF / max(rF, 0.1))
    st.capGround = rR * max(0, f)
  end
  return cR + st.capGround
end

-- Pump torque constant c [Nm/(rad/s)^2] and stall ratio (torqueConverter.lua):
-- at stall kFactor = 0.004 S / (1 + S), pump = kFactor * kFactorCoef * w^2.
local function converterConst()
  local tc = st.tc
  if tc and type(tc.converterStiffness) == "number" and type(tc.kFactorCoef) == "number" then
    local S = tc.converterStiffness
    st.c = 0.004 * S / (1 + S) * tc.kFactorCoef * (tc.impellerClutchCoef or 1)
    st.tr = tonumber(tc.stallTorqueRatio) or LC.TR_FALLBACK
  else
    st.c, st.tr = LC.C_FALLBACK, LC.TR_FALLBACK
  end
  if st.c <= 1e-6 then st.c = LC.C_FALLBACK end
end

-- turbine-to-wheel ratio (gear 1: 13.25 on R6, 14.85 on i4)
local function mechRatio()
  local gb = st.gb
  local r = gb and tonumber(gb.cumulativeGearRatio)
  if r then
    r = abs(r)
    if r >= LC.RATIO_MIN and r <= LC.RATIO_MAX then return r end
  end
  return LC.RATIO_FALLBACK
end

local function nMaxRpm()
  local ice = st.ice
  if ice and ice.requiredEnergyType == "diesel" then return LC.N_MAX_DIESEL end
  return LC.N_MAX_PETROL
end

-- T_ff = margin * C_hold / (TR * i), T* = T_ff * trim, n* = sqrt(T*/c).
-- Returns the untrimmed speed and idle (for "brake harder").
local function computeTargets(ctx, assist, margin)
  converterConst()
  st.ratio = mechRatio()
  st.capNm = holdCapacity(assist)
  st.tFf = margin * st.capNm / max(st.tr * st.ratio, 1)
  st.tStar = st.tFf * st.trim
  local idle = ctx.iceIdleRPM or 800
  local nFf = sqrt(max(st.tFf, 0) / st.c) * AV_TO_RPM
  local nTrim = sqrt(max(st.tStar, 0) / st.c) * AV_TO_RPM
  st.floorBind = nTrim < idle + LC.N_MIN_OVER_IDLE
  st.nStar = clamp(nTrim, idle + LC.N_MIN_OVER_IDLE, nMaxRpm())
  return nFf, idle
end

local function genMax(ctx, n)
  local w = max(n * RPM_TO_AV, 1)
  local cv = curves(ctx)
  local env = (cv and cv.motorRegenTorqueAtRpm and cv.motorRegenTorqueAtRpm(n)) or LC.GEN_MAX_NM
  local g = min(LC.GEN_MAX_NM, (ctx.stageRegenKw or 55) * 1000 / w, env)
  if ctx.evLockoutActive then g = 0 end
  return max(0, g * (ctx.regenAcceptFraction or 1)), env
end

local function genWanted()
  local g = LC.GEN_LOAD
  if g == 2 then
    local ice = st.ice
    return not (ice and ice.requiredEnergyType == "diesel")
  end
  return g == 1
end

local function throttleFor(ctx, netNm, rpm)
  local tc = ctx.modules and ctx.modules.torqueCoord
  if tc and tc.throttleForTorque then return tc.throttleForTorque(netNm, rpm, ctx) end
  local cv = curves(ctx)
  local cap = (cv and cv.iceMaxTorqueAtRpm and cv.iceMaxTorqueAtRpm(rpm)) or 350
  return clamp01(netNm / max(50, cap))
end

-- ===========================================================
-- Anti-creep sensing + trim
-- ===========================================================
-- 0 quiet / 1 soft / 2 hard creep from turbine speed, max |wheelSpeed| of the
-- driven wheels (an undriven front wheel turns as the body pitches) and the
-- forward body speed (the squat is mostly vertical).
local function senseCreep(ctx, dt)
  local tc = st.tc
  local turb = tc and abs(tonumber(tc.outputAV1) or 0) * AV_TO_RPM or 0
  local a = min(1, dt / LC.CREEP_TAU_S)
  st.turbF = st.turbF + (turb - st.turbF) * a
  local wmax = 0
  local all = (electrics.values.etk854hybrid_awdMode == "AWD" or #st.rear == 0) and st.all or st.rear
  for i = 1, #all do
    local s = abs(all[i].wheelSpeed or 0)
    if s > wmax then wmax = s end
  end
  st.whlF = st.whlF + (wmax - st.whlF) * a
  local body = 0
  if obj and obj.getVelocity and obj.getDirectionVector then
    local vel, dir = obj:getVelocity(), obj:getDirectionVector()
    body = abs(vel.x * dir.x + vel.y * dir.y + vel.z * dir.z)
  end
  st.bodyF = st.bodyF + (body - st.bodyF) * min(1, dt / LC.BODY_TAU_S)
  if not st.settled then
    st.settleT = st.settleT + dt
    if (st.turbF < LC.SETTLE_TURBINE_RPM and st.whlF < LC.SETTLE_MS and st.bodyF < LC.SETTLE_MS)
       or st.settleT >= LC.SETTLE_MAX_S then
      st.settled = true
    end
    return 0
  end
  if st.turbF > st.stat.turbMax then st.stat.turbMax = st.turbF end
  if st.whlF > st.stat.whlMax then st.stat.whlMax = st.whlF end
  if st.bodyF > st.stat.bodyMax then st.stat.bodyMax = st.bodyF end
  if st.turbF > LC.HARD_TURBINE_RPM or st.whlF > LC.HARD_WHEEL_MS then return 2 end
  if st.turbF > LC.CREEP_TURBINE_RPM or st.whlF > LC.CREEP_WHEEL_MS or st.bodyF > LC.CREEP_BODY_MS then
    return 1
  end
  return 0
end

-- Fast reduction, slow recovery. A hard creep also cuts the ICE for HARD_CUT_S.
local function updateTrim(dt, creep)
  st.hardT = max(0, st.hardT - dt)
  st.cutT = max(0, st.cutT - dt)
  if creep == 2 then
    st.quietT = 0
    if st.hardT <= 0 then
      st.trim = max(LC.TRIM_MIN, st.trim * LC.TRIM_HARD_MUL)
      st.hardT = LC.TRIM_HARD_EVERY_S
      st.cutT = LC.HARD_CUT_S
      st.hardN = st.hardN + 1
    end
  elseif creep == 1 then
    st.quietT = 0
    st.trim = max(LC.TRIM_MIN, st.trim - LC.TRIM_SOFT_RATE * dt)
  else
    st.quietT = st.quietT + dt
    if st.quietT >= LC.TRIM_QUIET_S then st.trim = min(1, st.trim + LC.TRIM_RECOVER * dt) end
  end
  if st.trim < st.stat.trimMin then st.stat.trimMin = st.trim end
  st.creep = creep
end

local function resetSensing()
  st.settled, st.settleT = false, 0
  st.quietT, st.hardT, st.cutT = 0, 0, 0
  st.creep = 0
end

-- ===========================================================
-- Hold regulator (PRELOAD / READY)
-- ===========================================================
-- Frame means of the physics-rate samples: shaft speed [rpm], ICE combustion
-- torque [Nm]. Boxcar anti-aliasing against the ~70 Hz K0 ringing; falls back
-- to the graphics-frame sample. Consumes the accumulators.
local function frameMeans(ctx)
  local n, comb
  if st.accN > 0 then
    n = st.accAV / st.accN * AV_TO_RPM
    comb = st.accTq / st.accN
  else
    n = abs(ctx.rpmMotor or 0)
    local ice = st.ice
    comb = (ice and tonumber(ice.combustionTorque)) or 0
  end
  st.accAV, st.accTq, st.accN = 0, 0, 0
  return n, comb
end

-- GEN mode, ICE side: torque feed-forward, no speed feedback. A slow integrator
-- on the measured ICE torque finds the throttle whose sustained boost gives
-- capS (the static inverse at the momentary boost pulses 0.3..1); out of reach
-- it rises at least ICE_BOOST_RISE. The hard cap capH (exact inverse, every
-- frame) catches the boost overshoot at the hold entry.
local function genIceThrottle(ctx, dt, n, gmax)
  local capS = st.tStar + LC.GEN_ICE_SHARE * gmax + st.iceTrim
  local capH = st.tStar + LC.GEN_ICE_HARD * gmax + st.iceTrim
  st.iceCapNm = capS
  local reachS = throttleFor(ctx, capS, n)   -- 1 = out of reach at full throttle
  if st.thrI < 0 then st.thrI = min(reachS, st.iceThr) end   -- seed: entry throttle
  local prev = st.thrI
  local err = capS - st.iceLpf
  local t = prev + ((err > 0) and LC.ICE_TQ_KI_UP or LC.ICE_TQ_KI_DN) * err * dt
  if err > 0 and reachS >= 1 then t = max(t, prev + LC.ICE_BOOST_RISE * dt) end
  t = clamp01(t)
  local tH = throttleFor(ctx, capH, n)
  if t > tH then t = tH end
  return t
end

-- GEN mode, generator: G = T_ice,ff - c w_ref^2 + KP e + I (T_ice,ff = ICE
-- torque + GEN_ICE_PRED x its rise). The pump torque damps the shaft itself, so
-- little gain is needed. Saturated with the shaft still too fast = the ICE
-- makes more than modelled: trim its target down (slowly back up).
local function genLoad(dt, e, loadRef, gmax, rise)
  local ff = st.iceLpf + LC.GEN_ICE_PRED * max(0, rise)
  local gp = (ff - loadRef) + LC.GEN_KP * e
  local graw = gp + st.gI
  -- anti-windup
  if not ((graw <= 0 and e < 0) or (graw >= gmax and e > 0)) then
    st.gI = clamp(st.gI + LC.GEN_KI * e * dt, -LC.GEN_I_MAX, LC.GEN_I_MAX)
  end
  local G = clamp(gp + st.gI, 0, gmax)
  st.gSat = G >= gmax - 1
  if st.gSat and e > LC.GEN_TRIM_E_RPM then
    st.iceTrim = max(-(st.tStar + 0.5 * gmax), st.iceTrim - LC.GEN_TRIM_DN * dt)
  elseif st.iceTrim < 0 and (G < (LC.GEN_ICE_SHARE - 0.10) * gmax or e < -LC.GEN_TRIM_E_RPM) then
    st.iceTrim = min(0, st.iceTrim + LC.GEN_TRIM_UP * dt)
  end
  return G
end

-- Pure ICE: torque = pump torque at the reference - PI on the speed error,
-- throttle by the exact inverse (gain independent of the throttle-map slope).
local function iceOnly(ctx, dt, n, e, loadRef)
  local tq = loadRef - LC.ICE_KP * e - st.iI
  st.iceCapNm = tq
  local t = throttleFor(ctx, tq, n)
  if not ((t <= 0 and e > 0) or (t >= 1 and e < 0)) then
    st.iI = clamp(st.iI + LC.ICE_KI * e * dt, -LC.ICE_I_MAX, LC.ICE_I_MAX)
  end
  return t
end

local function regulate(ctx, dt)
  local nS, comb = frameMeans(ctx)
  if st.nReg <= 0 then st.nReg = nS else st.nReg = st.nReg + (nS - st.nReg) * dt / (LC.RPM_TAU_S + dt) end
  local n = st.nReg
  if st.nRef <= 0 then st.nRef = max(n, ctx.iceIdleRPM or 800) end
  if st.nRef < st.nStar then
    st.nRef = min(st.nStar, st.nRef + LC.N_RAMP_UP * dt)
  else
    st.nRef = max(st.nStar, st.nRef - LC.N_RAMP_DN * dt)
  end
  local e = n - st.nRef
  local wRef = st.nRef * RPM_TO_AV
  local loadRef = st.c * wRef * wRef            -- pump torque at the reference
  local prevLpf = st.iceLpf
  st.iceLpf = st.iceLpf + (comb - st.iceLpf) * dt / (LC.ICE_TAU_S + dt)
  local gmax, env = genMax(ctx, n)
  if st.mode == "GEN" and gmax < LC.GEN_MIN_NM then
    -- pack stopped accepting charge: pure-ICE loop
    st.mode, st.iI, st.gI, st.iceTrim = "ICE", 0, 0, 0
  end
  local thr
  if st.mode == "GEN" then
    thr = genIceThrottle(ctx, dt, n, gmax)
    st.G = genLoad(dt, e, loadRef, gmax, st.iceLpf - prevLpf)
    st.stat.genSum, st.stat.genN = st.stat.genSum + st.G, st.stat.genN + 1
  else
    st.G = 0
    thr = iceOnly(ctx, dt, n, e, loadRef)
  end
  if st.cut or st.cutT > 0 then
    thr = 0
  elseif thr > st.iceThr + LC.ICE_THR_RISE * dt then
    thr = st.iceThr + LC.ICE_THR_RISE * dt     -- also the ramp back after a cut
  end
  st.iceThr = thr
  if st.mode == "GEN" then st.thrI = thr end   -- the integrator state IS the output
  st.regenFrac = (env and env > 1) and clamp01(st.G / env) or 0
end

-- Hold ended without a fire: throttle closed, the generator absorbs whatever
-- the ICE still makes above T*.
local function unloadStep(ctx, dt)
  st.unloadT = max(0, st.unloadT - dt)
  local n, comb = frameMeans(ctx)
  st.iceLpf = st.iceLpf + (comb - st.iceLpf) * dt / (LC.ICE_TAU_S + dt)
  local gmax, env = genMax(ctx, n)
  st.iceThr = 0
  st.G = clamp(st.iceLpf - st.unloadCap, 0, gmax)
  st.regenFrac = (env and env > 1) and clamp01(st.G / env) or 0
  if (st.G < 2 and st.iceLpf < st.unloadCap + LC.UNLOAD_DONE_NM) or st.unloadT <= 0 then
    st.unloadT, st.G, st.regenFrac = 0, 0, 0
  end
end

-- ===========================================================
-- Phase changes
-- ===========================================================
local function setPhase(ctx, p, why)
  if st.phase == p then return end
  lcLog("[LC] %s -> %s why=%s v=%.2fkm/h n=%.0f n*=%.0f T*=%.0fNm cap=%.0fNm trim=%.2f G=%.0f boost=%.1fpsi",
    st.phase, p, tostring(why), ctx.speedKmh or 0, abs(ctx.rpmMotor or 0), st.nStar, st.tStar, st.capNm,
    st.trim, st.G, tonumber(electrics.values.turboBoost) or 0)
  st.phase, st.why = p, why or "none"
end

local function holdSummary(ctx, tag)
  local s = st.stat
  lcLog("[LC] hold %s: t=%.2fs mode=%s n*=%.0f nRef=%.0f n=%.0f T*=%.0f Tff=%.0f cap=%.0f i=%.2f trimMin=%.2f hard=%d "
    .. "turbMax=%.0frpm whlMax=%.3fm/s bodyMax=%.3fm/s genAvg=%.0fNm ice=%.0fNm iceCap=%.0fNm iceTrim=%.0fNm "
    .. "thr=%.2f boost=%.1fpsi",
    tag, st.holdT, st.mode, st.nStar, st.nRef, st.nReg, st.tStar, st.tFf, st.capNm, st.ratio,
    s.trimMin, st.hardN, s.turbMax, s.whlMax, s.bodyMax, (s.genN > 0) and s.genSum / s.genN or 0,
    st.iceLpf, st.iceCapNm, st.iceTrim, st.iceThr, tonumber(electrics.values.turboBoost) or 0)
end

local function clearHoldState()
  st.G, st.regenFrac, st.iceThr = 0, 0, 0
  st.gI, st.iI, st.gSat = 0, 0, false
  st.thrI, st.iceTrim, st.iceCapNm, st.nReg = -1, 0, 0, 0
  st.cut = false
end

local function resetArmState()
  st.armT, st.holdT, st.preT, st.readyT, st.pedalLowT = 0, 0, 0, 0, 0
  st.weakMsg = false
end

-- bumpless coordinator seed (launchSeedNm): the full ICE torque the shaft
-- carries once the generator lets go
local function seedFromShaft(ctx)
  local md = st.motor
  local shaft = (md and type(md.etkShaftNm) == "number") and md.etkShaftNm or 0
  ctx.launchSeedNm = max(st.iceLpf, shaft, 0)
end

local function disarm(ctx, why, next)
  clearHoldState()
  resetArmState()
  st.trim = 1
  st.mode = "-"
  setPhase(ctx, next or "OFF", why)
end

-- Leave PRELOAD / READY without a fire (nextPhase ARMED | OFF | ABORT).
-- skipUnload: brake released, pedal above the ICE output -> seed the
-- coordinator like a fire instead of unloading.
local function exitHold(ctx, nextPhase, why, skipUnload)
  holdSummary(ctx, "exit:" .. tostring(why))
  local gen = st.mode == "GEN" and st.G > 1
  if gen and skipUnload then
    seedFromShaft(ctx)
    gen = false
  end
  clearHoldState()
  st.unloadCap = st.tStar            -- unload at the hold's cap
  st.unloadT = gen and LC.UNLOAD_MAX_S or 0
  st.reason = why
  if nextPhase == "ARMED" then
    st.holdT, st.preT, st.readyT, st.pedalLowT = 0, 0, 0, 0
    setPhase(ctx, "ARMED", why)
  else
    if nextPhase == "ABORT" then st.rearmBlock = true end
    resetArmState()
    st.mode = "-"
    setPhase(ctx, nextPhase, why)
  end
end

local function enterPreload(ctx)
  -- a guard trim was sensed against the pedal brake capacity, not this hold:
  -- restart from 1 (st.guard is still last frame's value here)
  if st.guard then
    st.trim = 1
    computeTargets(ctx, LC.BRAKE_ASSIST, LC.MARGIN)
  end
  resetSensing()
  clearHoldState()
  st.holdT, st.preT, st.readyT, st.pedalLowT = 0, 0, 0, 0
  st.nRef = max(abs(ctx.rpmMotor or 0), ctx.iceIdleRPM or 800)
  local ice = st.ice
  st.iceLpf = (ice and tonumber(ice.combustionTorque)) or 0
  -- start from the current throttle: no step, no dip
  st.iceThr = clamp01(tonumber(electrics.values.etk854hybrid_iceThrottle) or 0)
  st.accAV, st.accTq, st.accN = 0, 0, 0      -- entry frame: graphics-frame sample
  st.stat = { turbMax = 0, whlMax = 0, bodyMax = 0, trimMin = st.trim, genSum = 0, genN = 0 }
  st.hardN = 0
  local gmax = genMax(ctx, st.nStar)
  local want = genWanted()
  if want and (ctx.soc or 1) <= LC.GEN_SOC_MAX and gmax >= LC.GEN_MIN_NM then
    st.mode = "GEN"
  else
    st.mode = "ICE"
    if want then msg("LC: reduced turbo boost (battery full)", 3) end
  end
  st.guard, st.unloadT = false, 0
  st.reason = "none"
  setPhase(ctx, "PRELOAD", st.mode)
end

local function fire(ctx)
  holdSummary(ctx, "fire")
  seedFromShaft(ctx)
  clearHoldState()
  st.unloadT, st.assistOn = 0, false
  st.activeT = LC.WINDOW_S
  st.rearmBlock = true
  st.reason = "none"
  resetArmState()
  setPhase(ctx, "FIRE", "brake-release")
  local soc = ctx.soc or 1
  if soc < LC.SOC_MIN_BOOST or ctx.evLockoutActive then
    msg("LAUNCH! (engine only)", 1.5)
  else
    msg("LAUNCH!", 1.5)
  end
end

-- ===========================================================
-- Arming
-- ===========================================================
local function armAllowed(ctx, src)
  if ctx.seqFailsafe then return false, "Launch Control not available (failsafe)" end
  if ctx.evOnly then return false, "Launch Control: not available in MAX EV" end
  if (tonumber(electrics.values.watertemp) or 0) >= LC.COOLANT_MAX_C then
    return false, "Launch Control: engine too hot"
  end
  if not gearFwd(ctx) then return false, "Launch Control: select D, S or M" end
  if src == "manual" and not gearManualOk(ctx) then
    return false, "Launch Control: select SPORT or shift to S/M"
  end
  if (ctx.speedKmh or 0) >= LC.V_ENTER_KMH then return false, "Launch Control: only at a standstill" end
  return true, nil
end

-- src: "manual" (Alt+K / Actions app / QA) or "auto". A manual press while
-- armed switches it off.
function M.arm(ctx, src)
  if not ctx then return false end
  src = src or "manual"
  local p = st.phase
  if p == "FIRE" then return false end
  if src == "manual" and (p == "ARMED" or p == "PRELOAD" or p == "READY") then
    if p ~= "ARMED" then exitHold(ctx, "OFF", "manual-off") else disarm(ctx, "manual-off") end
    msg("Launch Control off", 2)
    return false
  end
  local ok, why = armAllowed(ctx, src)
  if not ok then
    st.reason = why
    if src == "manual" then msg(why, 3) end
    lcLog("[LC] arm rejected (%s): %s gear=%s v=%.2f", src, why, tostring(electrics.values.gear), ctx.speedKmh or 0)
    return false
  end
  resetArmState()
  st.armSrc, st.armDe = src, ctx.deMode
  st.trim = 1
  st.reason = "none"
  setPhase(ctx, "ARMED", src)
  msg("LAUNCH CONTROL - brake + full throttle", 3)
  return true
end

local function autoArmCheck(ctx, dt, pedal, brake, kmh)
  local key = ctx.dscKey
  if LC.AUTO_ARM and not st.rearmBlock and ctx.deMode == "SPORT" and (key == "dtc" or key == "off")
     and gearFwd(ctx) and kmh < LC.V_ENTER_KMH and brake >= LC.AUTO_BRAKE and pedal >= LC.AUTO_PEDAL
     and not ctx.evOnly and not ctx.seqFailsafe then
    st.autoT = st.autoT + dt
    if st.autoT >= LC.AUTO_S then
      st.autoT = 0
      if not M.arm(ctx, "auto") then st.rearmBlock = true end   -- one try per press
    end
  else
    st.autoT = 0
  end
end

-- ===========================================================
-- Per-phase updates
-- ===========================================================
local function updateArmed(ctx, dt, pedal, brake, kmh)
  st.armT = st.armT + dt
  if not gearFwd(ctx) then disarm(ctx, "gear"); return end
  if ctx.evOnly or ctx.seqFailsafe then disarm(ctx, "mode"); return end
  if st.armDe ~= ctx.deMode then disarm(ctx, "de-change"); return end
  if kmh > LC.V_DISARM_KMH then disarm(ctx, "drove-off"); return end
  if st.armT >= LC.ARM_TIMEOUT_S then
    disarm(ctx, "timeout")
    msg("Launch Control off (timeout)", 2)
    return
  end
  if brake >= LC.BRAKE_ENTER and pedal >= LC.PEDAL_ENTER and kmh < LC.V_ENTER_KMH then
    if st.unloadT > 0 or not iceReady(ctx) then return end   -- the guard caps meanwhile
    local nFf, idle = computeTargets(ctx, LC.BRAKE_ASSIST, LC.MARGIN)
    if nFf < idle + LC.N_MIN_OVER_IDLE then
      if not st.weakMsg then
        st.weakMsg = true
        st.reason = "brake-weak"
        msg("LC: brake harder", 2)
      end
      return
    end
    enterPreload(ctx)
    regulate(ctx, dt)                -- outputs from the entry frame on
  else
    st.weakMsg = false
  end
end

local function updateHold(ctx, dt, pedal, brake, kmh, creep)
  st.holdT = st.holdT + dt
  st.preT = st.preT + dt
  -- exits first
  if brake < LC.BRAKE_FIRE then
    -- floored pedal: fire, even early in PRELOAD (no slower map + unload in between)
    if pedal >= LC.PEDAL_FIRE then
      fire(ctx)
    else
      exitHold(ctx, "OFF", "brake-released", pedal >= LC.UNLOAD_SKIP_PEDAL)
    end
    return
  end
  if pedal < LC.PEDAL_HOLD then st.pedalLowT = st.pedalLowT + dt else st.pedalLowT = 0 end
  if st.pedalLowT >= LC.PEDAL_HOLD_S then exitHold(ctx, "ARMED", "pedal-lift"); return end
  if st.holdT >= LC.HOLD_MAX_S then
    exitHold(ctx, "ABORT", "hold-time")
    msg("Launch Control aborted (hold time exceeded)", 3)
    return
  end
  if not gearFwd(ctx) or ctx.evOnly or st.armDe ~= ctx.deMode then exitHold(ctx, "OFF", "mode"); return end
  if not iceReady(ctx) then exitHold(ctx, "ARMED", "ice-lost"); return end
  computeTargets(ctx, LC.BRAKE_ASSIST, LC.MARGIN)
  updateTrim(dt, creep)
  -- filtered wheel / body speeds only after the settle window (they still
  -- carry the ARMED / guard history and would latch the overspeed cut)
  local vMax = st.settled and max(kmh, st.whlF * 3.6, st.bodyF * 3.6) or kmh
  if vMax > LC.V_OVER_KMH and not st.cut then
    st.cut = true
    st.reason = "overspeed"
    lcLog("[LC] hold overspeed %.2fkm/h - ICE torque 0 for the rest of the hold", vMax)
  end
  -- trim below the idle floor: the speed loop cannot follow, cut the ICE torque
  if st.floorBind and st.trim < 1 and not st.cut then
    st.cut = true
    st.reason = "trim-floor"
    msg("LC: hold limit reached - brake harder", 2)
    lcLog("[LC] hold trim floor (trim %.2f, T*=%.0f Nm below the idle + %d rpm floor) - ICE torque 0",
      st.trim, st.tStar, LC.N_MIN_OVER_IDLE)
  end
  regulate(ctx, dt)
  if st.phase == "PRELOAD" then
    local nNow = (st.nReg > 0) and st.nReg or abs(ctx.rpmMotor or 0)
    if abs(nNow - st.nStar) < LC.READY_TOL_RPM then
      st.readyT = st.readyT + dt
    else
      st.readyT = 0
    end
    if st.readyT >= LC.READY_SETTLE_S or st.preT >= LC.READY_MAX_S then
      setPhase(ctx, "READY", (st.readyT >= LC.READY_SETTLE_S) and "settled" or "timeout")
      msg("LC READY - release brake", 2)
    end
  end
end

local function updateFire(ctx, dt, pedal, brake)
  st.activeT = st.activeT - dt
  if brake > LC.ACTIVE_ABORT_BRAKE or pedal < LC.ACTIVE_ABORT_PEDAL then
    st.activeT = 0
    disarm(ctx, "active-abort")
    return
  end
  if st.activeT <= 0 then disarm(ctx, "window-end") end
end

-- Brake-throttle guard: request cap for brake + pedal at a standstill outside the hold.
local function updateGuard(ctx, dt, pedal, brake, kmh, creep)
  local p = st.phase
  if p == "PRELOAD" or p == "READY" or p == "FIRE" then st.guard = false; return end
  local fwd = gearFwd(ctx)
  local on
  if st.guard then
    on = fwd and brake >= LC.GUARD_BRAKE_OUT and pedal >= LC.GUARD_PEDAL_OUT and kmh <= LC.GUARD_V_OUT_KMH
  else
    -- held / rising brake: at once; brake being released: after GUARD_DWELL_S
    local overlap = LC.GUARD_ENABLE and fwd and kmh < LC.GUARD_V_KMH and brake >= LC.GUARD_BRAKE
      and pedal > LC.GUARD_PEDAL and st.brakeRate > -LC.GUARD_BRAKE_FALL
    if overlap then st.guardDwellT = st.guardDwellT + dt else st.guardDwellT = 0 end
    local falling = st.brakeFallT < LC.GUARD_FALL_HOLD_S
    on = (overlap and (not falling or st.guardDwellT >= LC.GUARD_DWELL_S))
      or (fwd and kmh < LC.GUARD_V_KMH and p == "ABORT" and brake >= LC.GUARD_BRAKE_OUT
          and pedal >= LC.GUARD_PEDAL_OUT)
    if on then
      resetSensing()
      lcLog("[LC] guard on (phase %s) brake=%.2f pedal=%.2f v=%.2f dwell=%.2fs falling=%s",
        p, brake, pedal, kmh, st.guardDwellT, tostring(falling))
    end
  end
  if on then st.guardDwellT = 0 end
  st.guard = on and true or false
  if not st.guard then st.guardNm = 0; return end
  computeTargets(ctx, false, LC.GUARD_MARGIN)
  updateTrim(dt, creep)
  st.guardNm = (st.cutT > 0) and 0 or st.tStar
end

-- ===========================================================
-- Main update (master updateGFX, after the ICE sequencer)
-- ===========================================================
function M.updateGFX(ctx, dt)
  if not ctx or not dt or dt <= 0 then return end
  local pedal = ctx.throttleRaw or 0
  local brake = ctx.brakeRaw or 0
  local kmh = ctx.speedKmh or 0
  if st.rearmBlock and pedal < LC.PEDAL_REARM then st.rearmBlock = false end
  -- brake rate and time since the brake last fell (guard dwell)
  local bp = st.brakePrev or brake
  st.brakeRate = st.brakeRate + ((brake - bp) / dt - st.brakeRate) * min(1, dt / LC.BRAKE_RATE_TAU_S)
  if bp - brake > LC.GUARD_FALL_RATE * dt then st.brakeFallT = 0 else st.brakeFallT = st.brakeFallT + dt end
  st.brakePrev = brake

  local p = st.phase
  local holdPrev = p == "PRELOAD" or p == "READY"   -- phase before this frame's transitions
  updateLoads(dt, holdPrev or st.guard)
  local sensing = holdPrev or st.guard
    or (brake >= LC.GUARD_BRAKE_OUT and pedal > LC.GUARD_PEDAL_OUT and kmh < LC.GUARD_V_OUT_KMH)
  local creep = 0
  if sensing then
    creep = senseCreep(ctx, dt)
  else
    st.turbF, st.whlF, st.bodyF, st.creep = 0, 0, 0, 0
  end

  if p == "ABORT" and (brake < LC.GUARD_BRAKE_OUT or pedal < LC.GUARD_PEDAL_OUT) then
    -- pedal lift after an abort, brake still held: the unload has run out and a
    -- normal coordinator ramp would spin up the rear wheels -> hard reset
    if brake >= LC.GUARD_BRAKE_OUT then ctx.launchDrop = true end
    disarm(ctx, "abort-released")
  end
  if st.phase == "OFF" then autoArmCheck(ctx, dt, pedal, brake, kmh) end
  p = st.phase
  if p == "ARMED" then
    updateArmed(ctx, dt, pedal, brake, kmh)
  elseif p == "PRELOAD" or p == "READY" then
    updateHold(ctx, dt, pedal, brake, kmh, creep)
  elseif p == "FIRE" then
    updateFire(ctx, dt, pedal, brake)
  end
  p = st.phase
  local hold = (p == "PRELOAD" or p == "READY")
  if not hold and st.unloadT > 0 then unloadStep(ctx, dt) end
  updateGuard(ctx, dt, pedal, brake, kmh, creep)
  if st.phase == "OFF" and not st.guard and st.unloadT <= 0 then st.trim = 1 end
  local unloading = st.unloadT > 0

  -- physics-rate hook: hold assist (never while moving), frame means for the next frame
  st.assistOn = LC.BRAKE_ASSIST and kmh < LC.V_OVER_KMH
    and (hold or (unloading and brake >= LC.BRAKE_HOLD)) or false
  st.accOn = hold or unloading
  if not st.accOn then st.accAV, st.accTq, st.accN = 0, 0, 0 end

  -- ctx contract
  ctx.launchArmed = (p == "ARMED" or hold)
  ctx.launchReady = (p == "READY")
  ctx.launchActive = (p == "FIRE")
  ctx.launchHold = hold or unloading
  ctx.launchIceThr = st.iceThr
  ctx.launchGenNm = st.G
  ctx.launchPumpCapNm = hold and st.tStar or (unloading and st.unloadCap)
                        or (st.guard and st.guardNm or 0)
  ctx.launchTargetRpm = hold and st.nStar or 0
  ctx.launchGuardNm = (st.guard and not ctx.launchHold) and st.guardNm or nil

  -- squat reference: rear spring length at rest, throttle released
  if st.springs then
    st.springNow = springLen()
    if (p == "OFF" or p == "ARMED") and not st.guard and pedal < 0.1 and kmh < LC.V_ENTER_KMH then
      if st.springRef == nil then st.springRef = st.springNow
      else st.springRef = st.springRef + (st.springNow - st.springRef) * min(1, dt / LC.SPRING_TAU_S) end
    end
  end
  if p ~= "OFF" or st.guard or unloading then
    st.levelHoldT = LC.LEVEL_HOLD_S
  else
    st.levelHoldT = max(0, st.levelHoldT - dt)
  end

  -- channels
  local ev = electrics.values
  ev.etk854hybrid_launchState = PUBLIC[p] or "IDLE"
  ev.etk854hybrid_launchPhase = (p == "OFF" and st.guard) and "GUARD" or p
  ev.etk854hybrid_launchPumpCapNm = ctx.launchPumpCapNm
  ev.etk854hybrid_launchTargetRpm = ctx.launchTargetRpm
  ev.etk854hybrid_launchRefRpm = hold and st.nRef or 0
  ev.etk854hybrid_launchShaftRpm = hold and st.nReg or 0
  ev.etk854hybrid_launchIceCapNm = hold and st.iceCapNm or 0     -- target (GEN) / command (ICE)
  ev.etk854hybrid_launchGenNm = st.G
  ev.etk854hybrid_launchIceThr = (hold or unloading) and st.iceThr or 0
  ev.etk854hybrid_launchTrim = st.trim
  ev.etk854hybrid_launchHoldS = hold and st.holdT or 0
  ev.etk854hybrid_launchBrakeCapNm = (hold or st.guard) and st.capNm or 0
  ev.etk854hybrid_launchRearBrakeNm = (hold or st.guard) and st.capR or 0
  ev.etk854hybrid_launchGroundNm = (hold or st.guard) and st.capGround or 0
  ev.etk854hybrid_launchMode = hold and st.mode or (st.guard and "GUARD" or "-")
  ev.etk854hybrid_launchCreep = st.creep
  ev.etk854hybrid_launchAssist = st.assistOn and 1 or 0
  ev.etk854hybrid_launchUnload = unloading and 1 or 0
  ev.etk854hybrid_launchReason = st.reason
  ev.etk854hybrid_launchLevelHold = (st.levelHoldT > 0) and 1 or 0
  ev.etk854hybrid_launchTurbineRpm = st.turbF
  if st.springs then
    ev.etk854hybrid_launchSpringRmm = st.springNow * 1000
    ev.etk854hybrid_launchSquatMm = st.springRef and (st.springRef - st.springNow) * 1000 or 0
  end
end

-- Last writer of the motor / ICE channels during the hold / unload (after
-- recuperation, anti-kickback and BATTERY CONTROL, which all write evRegen).
function M.applyOutputs(ctx, dt)
  if not (ctx and ctx.launchHold) then return end
  local ev = electrics.values
  ev.etk854hybrid_evThrottle = 0
  ev.etk854hybrid_fillGain = 0
  -- creep floor off (creep.lua cuts too; defence in depth)
  ev.etk854hybrid_creepMaxNm = 0
  ev.etk854hybrid_creepCut = 1
  ev.etk854hybrid_evRegen = st.regenFrac
  ev.etk854hybrid_iceThrottle = ctx.iceOn and st.iceThr or 0
end

-- Physics rate, between wheels.updateWheelVelocities (sets desiredBrakingTorque)
-- and updateWheelTorques: frame-mean samples, and the hold assist raises the
-- desired brake torque, never lowers it (as brakeControl does).
function M.wheelsIntermediate(dt)
  -- shaft speed (= ICE speed, K0 closed) and combustion torque of this step
  if st.accOn then
    local md = st.motor
    local av = md and md.outputAV1
    if type(av) == "number" then
      local ice = st.ice
      local tq = ice and ice.combustionTorque
      st.accAV = st.accAV + ((av < 0) and -av or av)
      st.accTq = st.accTq + ((type(tq) == "number") and tq or 0)
      st.accN = st.accN + 1
    end
  end
  if not st.assistOn then return end
  local all = st.all
  for i = 1, #all do
    local wd = all[i]
    if not wd.isBroken then
      local bt = wd.brakeTorque or 0
      if (wd.desiredBrakingTorque or 0) < bt then wd.desiredBrakingTorque = bt end
    end
  end
end

-- ===========================================================
-- Lifecycle
-- ===========================================================
local function resetState(ctx)
  st.phase, st.why, st.reason, st.armSrc, st.armDe = "OFF", "none", "none", "none", nil
  st.autoT, st.activeT = 0, 0
  resetArmState()
  st.rearmBlock = false
  st.mode = "-"
  st.trim, st.hardN = 1, 0
  resetSensing()
  st.turbF, st.whlF, st.bodyF = 0, 0, 0
  st.capNm, st.tFf, st.tStar, st.nStar, st.nRef, st.ratio = 0, 0, 0, 0, 0, 0
  clearHoldState()
  st.iceLpf = 0
  st.unloadT, st.unloadCap, st.assistOn = 0, 0, false
  st.accOn, st.accAV, st.accTq, st.accN = false, 0, 0, 0
  st.guard, st.guardNm, st.guardDwellT, st.brakePrev, st.brakeRate = false, 0, 0, nil, 0
  st.brakeFallT = 9
  st.capR, st.capGround, st.floorBind = 0, 0, false
  st.nR, st.nF = 0, 0
  st.levelHoldT = 0
  st.springRef, st.springNow = nil, 0
  if ctx then
    ctx.launchArmed, ctx.launchReady, ctx.launchActive, ctx.launchHold = false, false, false, false
    ctx.launchIceThr, ctx.launchGenNm, ctx.launchPumpCapNm, ctx.launchTargetRpm = 0, 0, 0, 0
    ctx.launchGuardNm, ctx.launchSeedNm = nil, nil
  end
  local ev = electrics.values
  ev.etk854hybrid_launchState = "IDLE"
  ev.etk854hybrid_launchPhase = "OFF"
  ev.etk854hybrid_launchPumpCapNm = 0
  ev.etk854hybrid_launchTargetRpm = 0
  ev.etk854hybrid_launchRefRpm = 0
  ev.etk854hybrid_launchShaftRpm = 0
  ev.etk854hybrid_launchIceCapNm = 0
  ev.etk854hybrid_launchGenNm = 0
  ev.etk854hybrid_launchIceThr = 0
  ev.etk854hybrid_launchTrim = 1
  ev.etk854hybrid_launchHoldS = 0
  ev.etk854hybrid_launchBrakeCapNm = 0
  ev.etk854hybrid_launchRearBrakeNm = 0
  ev.etk854hybrid_launchGroundNm = 0
  ev.etk854hybrid_launchMode = "-"
  ev.etk854hybrid_launchCreep = 0
  ev.etk854hybrid_launchAssist = 0
  ev.etk854hybrid_launchUnload = 0
  ev.etk854hybrid_launchReason = "none"
  ev.etk854hybrid_launchLevelHold = 0
  ev.etk854hybrid_launchTurbineRpm = 0
end

function M.init(ctx)
  local jb = ctx and ctx.jbeamData
  -- hybridControl args
  local g = readNumber(jb, "launchGenLoad", LC.GEN_LOAD)
  LC.GEN_LOAD = (g == 0 or g == 2) and g or 1
  LC.BRAKE_ASSIST = readNumber(jb, "launchBrakeAssist", LC.BRAKE_ASSIST and 1 or 0) ~= 0
  LC.GUARD_ENABLE = readNumber(jb, "launchBrakeGuard", LC.GUARD_ENABLE and 1 or 0) ~= 0
  LC.AUTO_ARM = readNumber(jb, "launchAutoArm", LC.AUTO_ARM and 1 or 0) ~= 0
  LC.MARGIN = clamp(readNumber(jb, "launchMargin", LC.MARGIN), 0.40, 0.90)
  resetState(ctx)
  if ctx and ctx.dbg then
    ctx.dbg(1, "launch", string.format("init: gen=%s assist=%s guard=%s autoArm=%s margin=%.2f",
      tostring(LC.GEN_LOAD), tostring(LC.BRAKE_ASSIST), tostring(LC.GUARD_ENABLE),
      tostring(LC.AUTO_ARM), LC.MARGIN))
  end
end

function M.initStage2(ctx)
  findDevices()
  findWheels()
  local ok = pcall(findSprings)
  if not ok then st.springs = nil end
  converterConst()
  local okM = pcall(findMass)
  if not okM then st.massKg = 0 end
  log("I", "etk854hybrid.launch", string.format(
    "[LC] ready: wheels=%d rear=%d front=%d c=%.6f TR=%.2f i=%.2f springs=%s mass=%.0fkg gen=%s(%s) assist=%s guard=%s autoArm=%s margin=%.2f",
    st.wheelCount, #st.rear, #st.front, st.c, st.tr, mechRatio(), tostring(st.springs ~= nil), st.massKg,
    tostring(LC.GEN_LOAD), tostring(genWanted()), tostring(LC.BRAKE_ASSIST), tostring(LC.GUARD_ENABLE),
    tostring(LC.AUTO_ARM), LC.MARGIN))
end

function M.reset(ctx)
  resetState(ctx)
  M.initStage2(ctx)
end

-- Live tuning: M.setParam('MARGIN', 0.7). Existing number / boolean LC keys
-- only; returns the old value; not persistent.
function M.setParam(key, value)
  local old = LC[key]
  if type(old) == "number" and type(value) == "number" then
    LC[key] = value
    return old
  end
  if type(old) == "boolean" and type(value) == "boolean" then
    LC[key] = value
    return old
  end
  return nil
end

function M.getState()
  return {
    phase = st.phase, why = st.why, reason = st.reason, mode = st.mode, armSrc = st.armSrc,
    holdT = st.holdT, trim = st.trim, capNm = st.capNm, tFf = st.tFf, tStar = st.tStar,
    nStar = st.nStar, nRef = st.nRef, c = st.c, tr = st.tr, ratio = st.ratio,
    G = st.G, iceThr = st.iceThr, gSat = st.gSat, cut = st.cut, hardN = st.hardN,
    nReg = st.nReg, iceLpf = st.iceLpf, iceCapNm = st.iceCapNm, iceTrim = st.iceTrim, gI = st.gI, iI = st.iI,
    guard = st.guard, guardNm = st.guardNm, unloadT = st.unloadT, assistOn = st.assistOn,
    brakeFallT = st.brakeFallT, guardDwellT = st.guardDwellT,
    turbF = st.turbF, whlF = st.whlF, bodyF = st.bodyF, settled = st.settled,
    springs = st.springs ~= nil, squatMm = st.springRef and (st.springRef - st.springNow) * 1000 or 0,
    rear = #st.rear, front = #st.front,
  }
end

return M
