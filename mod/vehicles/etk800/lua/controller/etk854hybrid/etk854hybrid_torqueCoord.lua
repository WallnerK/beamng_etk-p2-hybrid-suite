-- etk854hybrid_torqueCoord.lua - ETK 854 ActiveHybrid - drive torque coordinator
--
-- The ONLY place where the driver's drive torque is shaped and split between
-- the combustion engine and the E-machine (P1 spec section 7). Replaces the
-- old allocation block of etk854hybridControl.lua (motor-first / ICE-first
-- branch switch, system cap, iceTarget/iceCap throttle fraction) and the
-- motorLeads() heuristic.
--
-- Chain per frame (the master calls updateGFX after the ICE sequencer):
--   torqueReqNm (pedal map x filtered ceiling, from the master)
--     -> T_req   system cap, reverse limiter, launch, N/P, sail, regen hold
--     -> T       jerk-limited total (the single drive-torque shaper)
--     -> split   planned ICE share iceL (rate-limited), per DE mode
--     -> iceExp  = shareGate g (from the sequencer) * iceL  = what the ICE is
--                  EXPECTED to deliver; 0 while it is not coupled
--     -> motorBase = T - iceExp        -> etk854hybrid_evThrottle
--     -> throttleForTorque(iceExp)      -> etk854hybrid_iceThrottle
--     -> fill channels for the motor device's per-tick fill loop
--        (etk854hybrid_motorShaft.lua motorTorque(): the motor adds
--         fillGain * (iceExpNm - measured ICE/K0 torque) every physics tick,
--         so ICE lag, K0 crank torque and K0 engage torque never reach the
--         shaft - the shaft sees T_cmd).
--
-- Why a jerk limiter instead of the device smoothers: the old path had no
-- VCU limiter at all; every ceiling step (gear change -> rpm -> curve lookup)
-- and every allocation branch flip went straight to the throttles and was
-- only softened by the motor device (10/s rise) and the ICE throttle smoother
-- (15/s) - two different shapers on the two halves of one torque. Now the
-- device smoother only interpolates (jbeam throttleRiseRate 40/s) and the
-- total is shaped exactly once, here.
--
-- Contract: M.isReal, init / initStage2 / updateGFX / reset / getState,
-- filterCeiling(ctx, dt, ceilRaw), throttleForTorque(netNm, rpm, ctx).
-- Launch control inputs (etk854hybrid_launchControl.lua): ctx.launchHold
-- (hold branch: no drive torque, ICE throttle = ctx.launchIceThr, bumpless
-- state), ctx.launchSeedNm (one-shot FIRE seed), ctx.launchGuardNm (request
-- cap of the brake-throttle guard: clamps the total and the ICE share at
-- once; a shadow of the unclamped total is kept and taken over when a short
-- guard ends, P.GUARD_SNAP_MAX_S; the fill may then also take back the
-- motor's drive torque, so the K0 torque of an ICE start - which the
-- sequencer keeps filled under the guard - stays out of the capped
-- converter input), ctx.launchActive (LAUNCH row + envelope).
-- All constants live in P, all state in st (LuaJIT: 60 upvalues/function).

local M = {}
M.name = "torqueCoord"
M.isReal = true

local abs, max, min, sqrt, exp, floor = math.abs, math.max, math.min, math.sqrt, math.exp, math.floor
local RPM_TO_AV = 0.104719755

-- ===========================================================
-- Parameters (P1 spec "TORQUE COORDINATION")
-- ===========================================================
local P = {
  -- Per-mode limiter rows [Nm/s, Nm/s^2]. rup = rup0 * (1 + sportGain*S),
  -- jerk = jerk0 * (1 + jerkGain*S). COMFORT 0 -> 680 Nm in ~0.65 s at S = 0,
  -- ~0.3 s at S = 1. Legacy HYBRID uses the COMFORT row.
  ROWS = {
    COMFORT = { rup0 = 1100, sportGain = 1.2, rdn = 1800, jerk0 = 10000, jerkGain = 1.2,
                lash = 300, iceRise0 = 400, iceRiseGain = 1.5, iceFall = 1200 },
    ECO_PRO = { rup0 = 750, sportGain = 0.8, rdn = 1500, jerk0 = 6000, jerkGain = 0.8,
                lash = 250, iceRise0 = 300, iceRiseGain = 0.0, iceFall = 1200 },
    SPORT   = { rup0 = 3500, sportGain = 0.0, rdn = 4000, jerk0 = 40000, jerkGain = 0.0,
                lash = 800, iceRise0 = 3000, iceRiseGain = 0.0, iceFall = 1200 },
    -- Launch window: effectively a step (launch control is a deliberate
    -- full-envelope request); the jerk term only removes the single-frame
    -- discontinuity.
    LAUNCH  = { rup0 = 8000, sportGain = 0.0, rdn = 8000, jerk0 = 1e6, jerkGain = 0.0,
                lash = 8000, iceRise0 = 8000, iceRiseGain = 0.0, iceFall = 1200 },
  },
  KICK_PEDAL      = 0.90,   -- kickdown: rise never slower than KICK_RUP (ECO PRO
                            -- in AUTO EV: DE_CFG urgentPedal 0.95 held
                            -- urgentDwellS 0.12 s, updateKick)
  KICK_RUP        = 2500,   -- (keeps the QA T14/T18 launches on target)
  -- brake-throttle guard exit (etk854hybrid_launchControl updateGuard): a
  -- guard shorter than this hands over to the unclamped jerk-limited total
  -- it shadowed (no ramp from the cap); a longer one (deliberate two-pedal
  -- hold) ramps from the cap at the row's rates
  GUARD_SNAP_MAX_S = 0.5,
  -- launch FIRE: the ICE throttle stays at 1 this long after the seed
  -- (seedFire, boost-lag vs static curve)
  FIRE_THR_S = 0.3,
  LASH_NM         = 25,     -- driveline lash zone: tip-in from 0 / from regen
  SNAP_NM         = 0.01,
  CEIL_TAU_S      = 0.25,   -- ceiling LPF (gear change -> curve lookup steps)
  ICE_UP_SAT      = 2500,   -- ICE share rise while the E-machine is saturated
  ECO_MOTOR_SHARE = 0.80,   -- ECO PRO: motor carries up to 80 % of its cap
  ECO_SOC_LO      = 0.20,   -- ... faded out between 20 and 35 % SOC
  ECO_SOC_SPAN    = 0.15,
  ECO_SPEED_FADE_KMH = 15,  -- ... and in the last 15 km/h below evTopKmh
  SPLIT_SOC_ICE_FIRST = 0.20,
  OVERRUN_PEDAL   = 0.02,   -- = RGN.THR_FULL in the master (lift-off gate); no longer
                            -- part of the overrun test (see M.updateGFX)
  OVERRUN_NM      = 1,
  FILL_NEG_MAX_NM = 120,    -- motor may absorb this much ICE over-delivery
  FILL_RISE       = 5,      -- fill gain rise [1/s]; drops are instant
  TF_ACTIVE       = 0.99,   -- DSE throttleFactor below this = DSC intervening
  GOV_ACTIVE_THR  = 0.02,   -- ICE idle-governor output above this = governor zone
  TRIM_KI         = 1.0,    -- [1/s] slow ICE torque-model trim (state ON only)
  TRIM_MAX_NM     = 60,
  TRIM_LEAK_TAU_S = 2.0,
  TRIM_DEXP_MAX   = 200,    -- [Nm/s] trim frozen while iceExp is moving
  TRIM_ERR_MAX    = 60,
  COMB_TAU_S      = 0.10,
  DRAG_TAU_S      = 0.10,   -- ICE drag credit LPF (updateRecuperation)
  BISECT_N        = 12,     -- 2^-12 throttle resolution
}

local st = {
  T = 0, rate = 0,          -- jerk-limited total [Nm] and its rate [Nm/s]
  ceil = 0, ceilInit = false,
  iceL = 0,                 -- rate-limited planned ICE share [Nm]
  trim = 0, combLpf = 0, lastIceExp = 0,
  fg = 0,                   -- fill gain actually published
  drag = 0,                 -- ICE drag credit [Nm]
  gOn = false, gT = 0,      -- guard active / its duration [s]
  gFreeT = 0, gFreeRate = 0,  -- shadow total without the guard cap
  kickT = 0, kickOn = false,  -- kickdown pedal held [s] / KICK_RUP active (updateKick)
  ice = nil, motor = nil,
}

-- ===========================================================
-- Helpers
-- ===========================================================
local function clamp(v, lo, hi)
  if v < lo then return lo end
  if v > hi then return hi end
  return v
end

local function clamp01(v)
  if v < 0 then return 0 end
  if v > 1 then return 1 end
  return v
end

local function rampTo(cur, target, upRate, dnRate, dt)
  if cur < target then return min(target, cur + upRate * dt) end
  return max(target, cur - dnRate * dt)
end

-- Copy of mathlib.lua:1262 smoothmin - the exact blend combustionEngine.lua
-- uses for its throttle map (CE:751). Kept local so the inverse below can
-- never drift from, or depend on, the global at module load time.
local function smoothmin(a, b, k)
  k = k or 0.1
  local h = max(min(0.5 + (b - a) / k, 1), 0)
  return h * a + (1 - h) * (b - h * k * 0.5)
end

local function iceCapAtRpm(ctx, rpm)
  local c = ctx and ctx.modules and ctx.modules.curves
  if c and c.iceMaxTorqueAtRpm then return c.iceMaxTorqueAtRpm(rpm) end
  return 350
end

-- ===========================================================
-- Exact inverse of the stock ICE torque model (spec F2, CE:733-803)
--   g    = torqueCurve[floor(rpm)] * intakeAirDensityCoef   (GROSS curve)
--   map  = smoothmin(max(t + t*k*(1-t), 0), 1, (1-t)*0.8),
--          k = maxPowerThrottleMap / (g*FI*av)
--   Tout = g*FI*map
--   load = clamp01(Tout / ((g - fs - fd) * FI))
--   net  = Tout - fs - fd - engineBrakeTorque*(1 - load)   (= CE combustionTorque)
-- net(t) is monotonic (the engine-brake term shrinks as load grows), so a
-- 12-step bisection gives the throttle that yields netNm. The old
-- iceTarget/iceCap fraction over-delivered up to 3x at part load because k
-- is ~2-3 around 2000 rpm.
-- ===========================================================
local function netAt(t, g, FIc, kmap, fs, fd, eb, maxCur)
  local x = t + t * kmap * (1 - t)
  if x < 0 then x = 0 end
  local tout = g * FIc * smoothmin(x, 1, (1 - t) * 0.8)
  local load = clamp01(tout / ((maxCur + 1e-30) * FIc))
  return tout - fs - fd - eb * (1 - load)
end

local function throttleForTorque(netNm, rpm, ctx)
  local ice = st.ice
  rpm = abs(rpm or 0)
  if not ice or type(ice.torqueCurve) ~= "table" or type(ice.friction) ~= "number"
     or type(ice.dynamicFriction) ~= "number" or type(ice.maxPowerThrottleMap) ~= "number" then
    return clamp01(netNm / max(50, iceCapAtRpm(ctx, rpm)))
  end
  local av = rpm * RPM_TO_AV
  local g = (ice.torqueCurve[floor(rpm)] or 0) * (ice.intakeAirDensityCoef or 1)
  if g < 1 then return 0 end
  local FIc = max(0.3, ice.forcedInductionCoef or 1)
  local fs = ice.friction * (ice.wearFrictionCoef or 1) * (ice.damageFrictionCoef or 1)
  local fd = ice.dynamicFriction * (ice.wearDynamicFrictionCoef or 1)
             * (ice.damageDynamicFrictionCoef or 1) * av
  local maxCur = g - fs - fd
  local eb = ice.engineBrakeTorque or 0
  local kmap = ice.maxPowerThrottleMap / (g * FIc * av + 1e-30)
  if netAt(0, g, FIc, kmap, fs, fd, eb, maxCur) >= netNm then return 0 end
  if netAt(1, g, FIc, kmap, fs, fd, eb, maxCur) <= netNm then return 1 end
  local lo, hi = 0, 1
  for _ = 1, P.BISECT_N do
    local mid = (lo + hi) * 0.5
    if netAt(mid, g, FIc, kmap, fs, fd, eb, maxCur) < netNm then lo = mid else hi = mid end
  end
  return hi
end

-- Public form (the sequencer's SYNC feed-forward uses it)
function M.throttleForTorque(netNm, rpm, ctx)
  return throttleForTorque(netNm or 0, rpm or 0, ctx)
end

-- ===========================================================
-- Ceiling filter (spec 2.4): the request ceiling motorCap + icePotential is
-- a curve LOOKUP at the current shaft/ICE rpm, so every gear change stepped
-- the torque request at constant pedal. LPF 0.25 s, frozen while shifting,
-- re-seeded on reset.
-- ===========================================================
function M.filterCeiling(ctx, dt, ceilRaw)
  if not st.ceilInit then
    st.ceil = ceilRaw
    st.ceilInit = true
  elseif not (ctx and ctx.isShifting) then
    st.ceil = st.ceil + (ceilRaw - st.ceil) * (1 - exp(-(dt or 0) / P.CEIL_TAU_S))
  end
  return st.ceil
end

-- ===========================================================
-- Demand chain + jerk-limited total (spec 7.1 / 7.2)
-- ===========================================================
local function limiterRow(ctx)
  if ctx.launchActive then return P.ROWS.LAUNCH end
  return P.ROWS[ctx.deMode or "COMFORT"] or P.ROWS.COMFORT
end

-- Returns the demand and whether the limiter must hard-reset (N / P).
local function demandTorque(ctx)
  if (ctx.gearSign or 0) == 0 then return 0, true end
  -- launch abort released with the brake held (launchControl): drop the
  -- jerk-limited total at once, like N / P, one frame
  if ctx.launchDrop then ctx.launchDrop = false; return 0, true end
  local capSys = ctx.systemTorqueCapNm or 0
  local req = ctx.torqueReqNm or 0
  if ctx.launchActive then
    -- the launch window asks for the combined envelope regardless of the
    -- pedal map (was: torqueReqNm = motorCapNm + iceCapNm in the master)
    req = (ctx.motorCapNm or 0)
          + iceCapAtRpm(ctx, max(ctx.iceRpmDev or ctx.iceRpm or 0, ctx.iceIdleRPM or 800))
  end
  if capSys > 0 and req > capSys then req = capSys end
  -- brake-throttle guard (etk854hybrid_launchControl.lua updateGuard): brake
  -- + pedal at a standstill, converter output capped below what the driven
  -- axle's brakes hold
  local gcap = ctx.launchGuardNm
  if gcap and req > gcap then req = gcap end
  -- reverse limiter moved from the ICE throttle (old C:1603) to the demand
  req = req * (ctx.reverseLimit or 1)
  if ctx.sailSuppressDrive then return 0, false end      -- ramped at rdn (fixes S10)
  -- drive may only restart after the regen has released: clean zero crossing
  if ctx.regenActiveLast then return 0, false end
  if req < 0 then req = 0 end
  return req, false
end

-- Kickdown gesture for the limiter's KICK_RUP, once per frame (runLimiter
-- runs twice under the guard: total + shadow). Kickdown pedal = the Driving
-- Experience's urgent pedal where the split map defines one (ECO PRO map v3:
-- 0.95 - its 0.90..0.95 is the kick zone of the hybrid band, not kickdown),
-- held for its urgentDwellS (0.12 s) - the same gesture as the VCU's urgent
-- start (policyWantsIce ctx.iceUrgentT) and the 8HP kickdown (K.kdDwellEco),
-- so a gamepad stab does not get the kickdown rise either; else P.KICK_PEDAL
-- without a dwell. On ctx.pedalIn like the demand (the raw driver pedal,
-- not the upshift-capped throttle).
local function updateKick(ctx, dt)
  local kp, dwell = P.KICK_PEDAL, 0
  local dc, ec = ctx.deModeCfg, ctx.edriveCfg
  if dc and dc.urgentPedal and ec and ec.deTuned then kp, dwell = dc.urgentPedal, dc.urgentDwellS or 0 end
  if (ctx.pedalIn or 0) >= kp then st.kickT = st.kickT + dt else st.kickT = 0 end
  st.kickOn = st.kickT > 0 and st.kickT >= dwell
end

local function runLimiter(ctx, dt, req, hardReset, row, S)
  if hardReset then
    st.T, st.rate = 0, 0
    return 0
  end
  local rup = row.rup0 * (1 + row.sportGain * S)
  if st.kickOn then rup = max(rup, P.KICK_RUP) end
  local jk = row.jerk0 * (1 + row.jerkGain * S)
  local err = req - st.T
  local aerr = abs(err)
  if aerr < P.SNAP_NM and abs(st.rate) < 1 then
    st.T, st.rate = req, 0
    return st.T
  end
  local R = (err > 0) and rup or row.rdn
  if err > 0 and st.T < P.LASH_NM then R = min(R, row.lash) end
  -- time-optimal approach under a jerk bound: the rate never exceeds what
  -- can still be braked to zero over the remaining error (sqrt(2*J*|err|))
  local rDes = min(R, sqrt(2 * jk * aerr), aerr / max(dt, 1e-4))
  if err < 0 then rDes = -rDes end
  st.rate = st.rate + clamp(rDes - st.rate, -jk * dt, jk * dt)
  local Tn = st.T + st.rate * dt
  if (req - Tn) * err < 0 then       -- crossed the target: snap, no overshoot
    Tn, st.rate = req, 0
  end
  st.T = Tn
  return Tn
end

-- ===========================================================
-- Split (spec 7.3): planned ICE share per DE mode
-- ===========================================================
local function plannedIceShare(ctx, Tb, iceCapNow, motorCapEff)
  local soc = ctx.soc or 1
  local split
  if ctx.chargingActive then
    -- BATTERY CONTROL load-point shift: the ICE carries the drive torque
    -- PLUS the cruise charge torque the motor recuperates (chargeNmLast)
    split = min(iceCapNow, Tb + (ctx.chargeNmLast or 0))
  elseif ctx.deMode == "SPORT" or ctx.legacyIceForce or soc < P.SPLIT_SOC_ICE_FIRST then
    -- ICE carries the base, the motor fills lag / shift recovery and boosts
    -- above the ICE cap (replaces motorLeads)
    split = min(Tb, iceCapNow)
  elseif ctx.deMode == "ECO_PRO" then
    -- EV speed ceiling the policy really uses: in AUTO EV the Driving
    -- Experience's evTopAuto (ECO 120 km/h; the AUTO EV fallback 110
    -- faded the motor budget over 95-110 instead of 105-120 km/h)
    local ec = ctx.edriveCfg
    local top = (ec and ec.deTuned and ctx.deModeCfg and ctx.deModeCfg.evTopAuto)
                or (ec and ec.evTopKmh) or 110
    local budget = P.ECO_MOTOR_SHARE * motorCapEff
      * clamp01((soc - P.ECO_SOC_LO) / P.ECO_SOC_SPAN)
      * clamp01((top - (ctx.speedKmh or 0)) / P.ECO_SPEED_FADE_KMH)
    split = clamp(Tb - budget, 0, iceCapNow)
  else
    split = min(Tb, iceCapNow)     -- COMFORT: ICE-led, softened by iceRise
  end
  -- saturation: whatever the motor cannot deliver goes to the ICE
  split = max(split, min(iceCapNow, Tb - motorCapEff))
  if split < 0 then split = 0 end
  return split
end

-- ===========================================================
-- DSC / governor state (fill + trim gates)
-- ===========================================================
local function dscIntervening(ctx)
  local ev = electrics.values
  -- flags may arrive as 1/0 (electrics.lua converts booleans each frame):
  -- a plain truth test would read a numeric 0 as "DSC intervening"
  local esc, tcs = ev.escActive, ev.tcsActive
  if esc == true or tcs == true or (tonumber(esc) or 0) > 0.5 or (tonumber(tcs) or 0) > 0.5 then return true end
  local ice, motor = st.ice, st.motor
  -- the ICE factor only matters while K0 is closed: a trim on an engine that
  -- idles behind an open K0 changes nothing at the wheels (feedback 2026-10)
  if ice and (ctx.k0Current or 1) >= 0.9 then
    local tf = ev[ice.electricsThrottleFactorName or "throttleFactor"] or ice.throttleFactor or 1
    if tf < P.TF_ACTIVE then return true end
  end
  if motor and (motor.throttleFactor or 1) < P.TF_ACTIVE then return true end
  return false
end

-- Fill gain (spec 7.5). Slip states (K0 cranking / open / engaging) measure
-- the K0 torque, so the fill there is a pure cancellation and stays active in
-- overrun and during regen; locked states measure combustion torque and
-- would otherwise cancel the (regen-credited) engine braking.
local function updateFillGain(ctx, dt, overrun, dsc, slipState)
  local target = ctx.seqFillGain or 0
  if (ctx.gearSign or 0) == 0 or ctx.iceIdleEngage or dsc then target = 0 end
  if (ctx.chargeModeLast or "OFF") ~= "OFF" then target = 0 end
  if not slipState then
    if overrun then target = 0 end
    -- ICE idle governor active (creeping on idle): the governor is a SPEED
    -- controller (CE:706-721); a torque fill against it only ends with the
    -- motor absorbing the governor's torque, so hand over to the governor.
    local ice = st.ice
    if ice and (ice.idleThrottle or 0) > P.GOV_ACTIVE_THR then target = 0 end
  end
  -- Shifts gate the LOCKED fill only (turbo-lag fill would fight the
  -- shift torque intervention). In the slip states the fill is a pure
  -- cancellation of the measured K0 torque (iceExp is 0 there) and has to
  -- stay on: the 8HP may shift during CRANK (rev-cap gear, lug) and a
  -- K0 crank without its cancellation reaches the wheels as a torque dip.
  if ctx.isShifting and not slipState then
    local sfg = electrics.values.etk854hybrid_shiftFillGain
    if type(sfg) == "number" then target = min(target, clamp01(sfg)) else target = 0 end
  end
  if target > st.fg then
    -- instant in the sequencer's slip states: the fill error starts at ~0
    -- there (K0 torque grows from 0); a ramp would only let the crank dip through
    if ctx.seqFillInstant then st.fg = target else st.fg = min(target, st.fg + P.FILL_RISE * dt) end
  else
    st.fg = target
  end
  return st.fg
end

-- Slow ICE torque-model trim (spec 7.4): closes the static gap between the
-- inverted model and ice.combustionTorque (turbo map, air density, wear).
local function updateTrim(ctx, dt, state, iceExp, iceThr, dsc, overridden)
  local ice = st.ice
  local comb = (ice and type(ice.combustionTorque) == "number") and ice.combustionTorque or iceExp
  st.combLpf = st.combLpf + (comb - st.combLpf) * min(1, dt / P.COMB_TAU_S)
  local dExp = (iceExp - st.lastIceExp) / max(dt, 1e-4)
  st.lastIceExp = iceExp
  local govActive = ice and (ice.idleThrottle or 0) > P.GOV_ACTIVE_THR
  if state == "ON" and not overridden and not ctx.isShifting and not dsc and not govActive
     and iceThr > 0.02 and iceThr < 0.98 and abs(dExp) < P.TRIM_DEXP_MAX
     and abs(iceExp - st.combLpf) < P.TRIM_ERR_MAX then
    st.trim = clamp(st.trim + P.TRIM_KI * (iceExp - st.combLpf) * dt, -P.TRIM_MAX_NM, P.TRIM_MAX_NM)
  elseif state ~= "ON" then
    st.trim = st.trim * exp(-dt / P.TRIM_LEAK_TAU_S)
  end
end

-- ICE drag credit for updateRecuperation (spec 7.8): with the ICE coupled in
-- overrun, its fuel-cut drag is subtracted from the regen torque so the
-- selected coast deceleration does not depend on whether K0 is closed.
-- Only while LOCKED: in the slip states the fill already cancels K0 torque.
local function updateDragCredit(ctx, dt, overrun)
  local raw = 0
  if overrun and ctx.iceCoupled then raw = max(0, -(ctx.iceMeasNm or 0)) end
  st.drag = st.drag + (raw - st.drag) * min(1, dt / P.DRAG_TAU_S)
  if raw <= 0 and st.drag < 0.5 then st.drag = 0 end
  ctx.iceDragNm = st.drag
end

-- ===========================================================
-- Launch control hooks (etk854hybrid_launchControl.lua)
-- ===========================================================
-- Hold (ctx.launchHold: PRELOAD / READY and the generator unload after a
-- hold without a fire). The launch module regulates the converter preload
-- itself - ICE throttle ctx.launchIceThr, E-machine as a generator only (its
-- applyOutputs writes etk854hybrid_evRegen last) - so the coordinator
-- publishes no drive torque, no fill and a frozen trim, and keeps its state
-- bumpless: the jerk-limited total follows the measured shaft torque
-- (motorShaft etkShaftNm = K0 + motor torque into the converter) and the ICE
-- share is seeded with the SAME value, so a hold that ends without a fire
-- restarts from the torque the brakes were holding, not from the ~400 Nm the
-- full-throttle ICE makes while the generator absorbs the rest.
local function holdBranch(ctx, dt, ev)
  local motor, ice = st.motor, st.ice
  local shaft = motor and tonumber(motor.etkShaftNm) or st.T
  st.T, st.rate = max(0, shaft), 0
  st.iceL = st.T
  st.fg = 0
  st.drag = 0
  st.gOn = false                     -- a guard before the hold has no shadow to hand over
  local comb = (ice and tonumber(ice.combustionTorque)) or 0
  st.combLpf = st.combLpf + (comb - st.combLpf) * min(1, dt / P.COMB_TAU_S)
  st.lastIceExp = st.combLpf         -- no dExp step for the trim gate afterwards
  local thr = clamp01(ctx.launchIceThr or 0)
  local capFull = ctx.motorCapNm or 0
  local sysCap = ctx.systemTorqueCapNm or 0
  -- what the E-machine could add at the release: its cap, within the system
  -- cap left over by the ICE torque that is already on the shaft
  local avail = capFull
  if sysCap > 0 then avail = min(avail, sysCap - max(0, st.combLpf)) end
  ev.etk854hybrid_evThrottle = 0
  ev.etk854hybrid_iceThrottle = ctx.iceOn and thr or 0
  ev.etk854hybrid_iceExpNm = 0
  ev.etk854hybrid_fillGain = 0
  ev.etk854hybrid_fillPosMaxNm = 0
  ev.etk854hybrid_fillNegMaxNm = 0
  ev.etk854hybrid_evTorqueAvailNm = clamp(avail, 0, max(0, capFull))
  ctx.iceDragNm = 0
  ctx.chargingActive = false
  ctx.fillSlipState = false
  ctx.torqueDeficitNm = 0
  local cap = ctx.launchPumpCapNm or st.T
  ctx.torqueTargetNm = cap
  ctx.torqueTargetBaseNm = cap
  ctx.motorBaseNm, ctx.motorBaseRawNm = 0, 0
  ctx.iceExpNm = st.combLpf
  ctx.iceSplitNm = st.iceL
  ctx.iceThrottleCmd = thr
  ctx.coordOverrun = false
end

-- FIRE (one-shot ctx.launchSeedNm = ICE torque on the shaft when the
-- generator lets go): total and ICE share start there, so the LAUNCH row
-- (8000 Nm/s) ramps from ~400 Nm to the system cap instead of from the ~100 Nm
-- pump cap, and the SPORT split does not pull the full-throttle ICE back
-- first (plannedIceShare min(Tb, iceCap) with iceL above Tb would ramp the
-- share DOWN at rdn = a boost dip at the release). The fill gain starts at
-- the sequencer's value at once (it was 0 in the hold): the turbo-lag fill
-- is exactly what a launch needs in its first 0.2 s.
local function seedFire(ctx, seed)
  local s = max(0, tonumber(seed) or 0)
  local capSys = ctx.systemTorqueCapNm or 0
  if capSys > 0 then s = min(s, capSys) end
  if s > st.T then st.T = s end
  st.rate = 0
  st.iceL = s
  st.fg = clamp01(ctx.seqFillGain or 0)
  -- the full-throttle ICE stays at full throttle through the first frames
  -- of the fire (P.FIRE_THR_S): the inverse model reads the measured, boost-
  -- lagged ~390 Nm at 1413 rpm as ~36 % throttle on the static full-boost
  -- curve - one frame of 0.36 in the fire frame (B58 / i4 launch_hold_full
  -- 2026-09-30 20:20-20:25), a boost dip at the very moment the launch needs it
  st.fireThrT = P.FIRE_THR_S
end

-- ===========================================================
-- Main update (spec 7.1-7.5)
-- ===========================================================
function M.updateGFX(ctx, dt)
  if not ctx then return end
  dt = dt or 0
  if dt <= 0 then return end
  local ev = electrics.values
  if ctx.launchHold then return holdBranch(ctx, dt, ev) end
  if ctx.launchSeedNm then
    seedFire(ctx, ctx.launchSeedNm)
    ctx.launchSeedNm = nil
  end
  -- guard engaged while the total is above it: cut at once, not at rdn
  local gcap = ctx.launchGuardNm
  if gcap then
    if not st.gOn then st.gOn, st.gT, st.gFreeT, st.gFreeRate = true, 0, st.T, st.rate end
    st.gT = st.gT + dt
    if st.T > gcap then st.T = gcap; if st.rate > 0 then st.rate = 0 end end
    if st.iceL > gcap then st.iceL = gcap end
  elseif st.gOn then
    -- guard released: a short one (gamepad trigger cross-over) must not
    -- leave the total clamped below what the pedal map had built meanwhile
    st.gOn = false
    if st.gT <= P.GUARD_SNAP_MAX_S and st.gFreeT > st.T then
      st.T, st.rate = st.gFreeT, max(st.rate, st.gFreeRate)
    end
  end
  local S = ctx.sportIndex or 0
  local row = limiterRow(ctx)
  updateKick(ctx, dt)

  local req, hardReset = demandTorque(ctx)
  local Tb = runLimiter(ctx, dt, req, hardReset, row, S)   -- unscaled total
  if st.gOn then
    -- shadow: the same limiter on the uncapped demand
    ctx.launchGuardNm = nil
    local rf, hr = demandTorque(ctx)
    ctx.launchGuardNm = gcap
    local T0, r0 = st.T, st.rate
    st.T, st.rate = st.gFreeT, st.gFreeRate
    runLimiter(ctx, dt, rf, hr, row, S)
    st.gFreeT, st.gFreeRate = st.T, st.rate
    st.T, st.rate = T0, r0
  end
  local kShift = ctx.kShift or 1
  local Tcmd = Tb * kShift

  local state = ctx.iceEngageState or "OFF"
  local rpmIce = ctx.iceRpmDev or ctx.iceRpm or 0
  local idle = ctx.iceIdleRPM or 800
  local capFull = ctx.motorCapNm or 0
  -- K0 crank reserve (deviation from the spec text, see PLAN_ICE_ENGAGEMENT):
  -- while the E-machine drags the ICE up through K0, its crank torque is
  -- reserved so the driver cannot take the headroom and abort the start.
  local motorCapEff = max(0, capFull - (ctx.iceCrankReserveNm or 0))
  local iceCapNow = 0
  if ctx.iceCoupled or state == "ENGAGE" then iceCapNow = iceCapAtRpm(ctx, max(rpmIce, idle)) end
  -- Overrun = the jerk-limited total has reached zero, NOT the raw pedal.
  -- With the pedal test the ICE share and throttle were dropped the instant
  -- the driver lifted, while the total still ramped down at rdn: the
  -- E-machine had to carry the whole ramp alone, saturated, and the ICE
  -- dragged in fuel cut underneath - shaft torque 471 -> 187 Nm within
  -- 0.1 s against a commanded 457 -> 376 Nm (B58 tip-out at 68 km/h, 4th,
  -- tipin_rolling 2026-09-30). Now the ICE follows the ramp-down (its share
  -- ramps at max(iceFall, rdn), the fill covers the turbo lag) and goes to
  -- fuel cut once the total is out.
  local overrun = Tcmd <= P.OVERRUN_NM

  ctx.chargingActive = (ctx.chargeToTarget == true) and (ctx.soc or 1) < (ctx.batteryTargetSoc or 0)
                       and state == "ON"
  local split = plannedIceShare(ctx, Tb, iceCapNow, motorCapEff)
  if overrun then
    st.iceL = 0
  else
    local motorSat = (Tb - st.iceL) >= motorCapEff - 5
    local up = motorSat and P.ICE_UP_SAT or (row.iceRise0 * (1 + row.iceRiseGain * S))
    st.iceL = rampTo(st.iceL, split, up, max(row.iceFall, row.rdn), dt)
  end

  local relTorque = (state == "RELEASE" and ctx.iceRelPhase == "TORQUE")
  local iceExpRaw = 0
  if state == "BLEND" or state == "ON" or relTorque then
    iceExpRaw = (ctx.iceShareGate or 0) * st.iceL
  end
  local motorBaseRaw = clamp(Tb - iceExpRaw, 0, motorCapEff)
  local iceExp = iceExpRaw * kShift            -- kShift = fast shift factor
  local motorBase = motorBaseRaw * kShift
  ctx.torqueDeficitNm = max(0, Tb - iceExpRaw - motorBaseRaw)

  -- ---- ICE throttle (spec 7.4) ----
  local iceThr
  local ov = ctx.iceThrottleOverride
  if ov ~= nil then
    iceThr = clamp01(ov)
  elseif overrun or state == "OFF" or state == "CRANK" then
    iceThr = 0
  else
    iceThr = throttleForTorque(iceExp + st.trim, rpmIce, ctx)
  end
  if (st.fireThrT or 0) > 0 then
    st.fireThrT = st.fireThrT - dt
    if ctx.launchActive and not overrun and ov == nil then iceThr = 1 end
  end
  local dsc = dscIntervening(ctx)
  updateTrim(ctx, dt, state, iceExp, iceThr, dsc, ov ~= nil)

  -- ---- fill (spec 7.5) ----
  local slipState = (state == "CRANK" or state == "SYNC" or state == "ENGAGE"
                     or (state == "RELEASE" and ctx.iceRelPhase == "OPEN"))
  ctx.fillSlipState = slipState
  local fg = updateFillGain(ctx, dt, overrun, dsc, slipState)
  local avShaft = abs(ctx.rpmMotor or 0) * RPM_TO_AV
  local regenKw = ctx.stageRegenKw or 55
  local sysCap = ctx.systemTorqueCapNm or 0

  ev.etk854hybrid_evThrottle = clamp01(motorBase / max(1, ctx.motorCurveNm or 1))
  ev.etk854hybrid_iceThrottle = ctx.iceOn and iceThr or 0
  ev.etk854hybrid_iceExpNm = iceExp
  ev.etk854hybrid_fillGain = fg
  ev.etk854hybrid_fillPosMaxNm = max(0, capFull - motorBase)
  local negMax = min(P.FILL_NEG_MAX_NM, regenKw * 1000 / max(avShaft, 50)) * (ctx.regenAcceptFraction or 1)
  -- brake-throttle guard: the fill may also take back the motor's own drive
  -- torque (that is no regen), so K0 / ICE torque up to cap + negMax stays
  -- out of the capped converter input (the sequencer keeps the fill on in
  -- ENGAGE under the guard above the idle band, etk854hybrid_iceSequencer
  -- guardOn / guardFill)
  if gcap then negMax = negMax + max(0, motorBase) end
  ev.etk854hybrid_fillNegMaxNm = negMax
  local avail = motorCapEff
  if sysCap > 0 then avail = min(avail, sysCap - max(0, iceExp)) end
  ev.etk854hybrid_evTorqueAvailNm = clamp(avail, 0, max(0, motorCapEff))

  updateDragCredit(ctx, dt, overrun)

  ctx.torqueTargetNm = Tcmd
  ctx.torqueTargetBaseNm = Tb
  ctx.motorBaseNm = motorBase
  ctx.motorBaseRawNm = motorBaseRaw   -- before the shift factor (K0 start headroom)
  ctx.iceExpNm = iceExp
  ctx.iceSplitNm = st.iceL
  ctx.iceThrottleCmd = iceThr
  ctx.coordOverrun = overrun
end

-- ===========================================================
-- Lifecycle
-- ===========================================================
local function resetState()
  st.T, st.rate = 0, 0
  st.ceil, st.ceilInit = 0, false
  st.iceL = 0
  st.trim, st.combLpf, st.lastIceExp = 0, 0, 0
  st.fireThrT = 0
  st.fg = 0
  st.drag = 0
  st.gOn, st.gT, st.gFreeT, st.gFreeRate = false, 0, 0, 0
  st.kickT, st.kickOn = 0, false
end

local function publishIdle()
  local ev = electrics.values
  ev.etk854hybrid_iceExpNm = 0
  ev.etk854hybrid_fillGain = 0
  ev.etk854hybrid_fillPosMaxNm = 0
  ev.etk854hybrid_fillNegMaxNm = 0
  ev.etk854hybrid_evTorqueAvailNm = 0
end

function M.init(ctx)
  resetState()
  publishIdle()
  if ctx then
    ctx.torqueTargetNm, ctx.motorBaseNm, ctx.iceExpNm, ctx.iceDragNm = 0, 0, 0, 0
    ctx.torqueDeficitNm = 0
    if ctx.dbg then ctx.dbg(1, "torqueCoord", "init: jerk-limited total, fill channels, exact ICE inverse") end
  end
end

function M.initStage2(ctx)
  st.ice = powertrain.getDevice("mainEngine")
  st.motor = powertrain.getDevice("integratedMotor")
end

function M.reset(ctx)
  resetState()
  publishIdle()
  M.initStage2(ctx)
  if ctx then ctx.iceDragNm = 0 end
end

function M.updateFixedStep(ctx, dt) end

function M.getState()
  return { T = st.T, rate = st.rate, ceil = st.ceil, iceL = st.iceL, trim = st.trim,
           fillGain = st.fg, dragNm = st.drag }
end

return M
