-- etk854hybrid_torqueCoord.lua - ETK 854 ActiveHybrid - drive torque coordinator
--
-- Shapes the driver's drive torque and splits it between ICE and E-machine.
-- Called by the VCU every frame after the ICE sequencer and launch control.
--
-- Chain per frame:
--   torqueReqNm (pedal map x filtered ceiling)
--     -> T_req   system cap, trailer stability, guard, reverse limit, launch,
--                N/P, sail, regen hold
--     -> T       jerk-limited total (the only shaper; the motor device only
--                interpolates)
--     -> iceL    planned ICE share per DE mode, rate-limited
--     -> iceExp  = shareGate * iceL (0 while the ICE is not coupled)
--     -> motorBase = T - iceExp -> etk854hybrid_evThrottle
--     -> throttleForTorque(iceExp) -> etk854hybrid_iceThrottle
--     -> fill channels: the motor device adds fillGain * (iceExpNm - measured
--        ICE/K0 torque) per physics tick, so ICE lag and K0 crank / engage
--        torque never reach the shaft.
--
-- API: lifecycle + filterCeiling, throttleForTorque(netNm, rpm, ctx).
-- Launch inputs (ctx): launchHold + launchIceThr (hold branch), launchSeedNm
--   (one-shot FIRE seed), launchGuardNm (brake-throttle guard cap), launchDrop
--   (one-frame hard zero), launchActive (LAUNCH row + envelope).
-- Trailer stability (ctx): tsaTorqueMode 1 HOLD / 2 ZERO (one cap per episode of
--   non-zero modes, re-latched on each ZERO entry), tsaTorqueRec (HOLD cap
--   recovery rate [Nm/s], 0 = none), tsaFreeze (no charge load). Constants in
--   P, state in st (LuaJIT 60-upvalue limit).

local M = {}
M.name = "torqueCoord"
M.isReal = true

local abs, max, min, sqrt, exp, floor = math.abs, math.max, math.min, math.sqrt, math.exp, math.floor
local RPM_TO_AV = 0.104719755

-- ===========================================================
-- Parameters
-- ===========================================================
local P = {
  -- Limiter rows [Nm/s, jerk Nm/s^2], S = sportIndex: rise rup0*(1+sportGain*S),
  -- fall rdn, jerk jerk0*(1+jerkGain*S), lash = rise below LASH_NM; iceRise0 /
  -- iceRiseGain / iceFall = ICE share rates. Unknown modes use COMFORT.
  ROWS = {
    COMFORT = { rup0 = 1100, sportGain = 1.2, rdn = 1800, jerk0 = 10000, jerkGain = 1.2,
                lash = 300, iceRise0 = 400, iceRiseGain = 1.5, iceFall = 1200 },
    ECO_PRO = { rup0 = 750, sportGain = 0.8, rdn = 1500, jerk0 = 6000, jerkGain = 0.8,
                lash = 250, iceRise0 = 300, iceRiseGain = 0.0, iceFall = 1200 },
    SPORT   = { rup0 = 3500, sportGain = 0.0, rdn = 4000, jerk0 = 40000, jerkGain = 0.0,
                lash = 800, iceRise0 = 3000, iceRiseGain = 0.0, iceFall = 1200 },
    -- launch: effectively a step, the jerk term only removes the frame step
    LAUNCH  = { rup0 = 8000, sportGain = 0.0, rdn = 8000, jerk0 = 1e6, jerkGain = 0.0,
                lash = 8000, iceRise0 = 8000, iceRiseGain = 0.0, iceFall = 1200 },
  },
  KICK_PEDAL      = 0.90,   -- kickdown pedal if the DE has no urgent pedal
  KICK_RUP        = 2500,   -- [Nm/s] minimum rise during kickdown
  GUARD_SNAP_MAX_S = 0.5,   -- [s] shorter guard: take over its uncapped shadow
  FIRE_THR_S = 0.3,         -- [s] launch FIRE: ICE full throttle after the seed
  LASH_NM         = 25,     -- driveline lash zone
  SNAP_NM         = 0.01,
  CEIL_TAU_S      = 0.25,   -- [s] ceiling LPF
  ICE_UP_SAT      = 2500,   -- [Nm/s] ICE share rise, E-machine saturated
  SAT_MARGIN_NM   = 5,      -- [Nm] E-machine counts as saturated within this of its cap
  ECO_MOTOR_SHARE = 0.80,   -- ECO PRO: motor share of its cap
  ECO_SOC_LO      = 0.20,   -- ... faded out over 20..35 % SOC
  ECO_SOC_SPAN    = 0.15,
  ECO_SPEED_FADE_KMH = 15,  -- ... and over 15 km/h below the EV top speed
  SPLIT_SOC_ICE_FIRST = 0.20,  -- below: ICE carries the base
  OVERRUN_NM      = 1,      -- [Nm] total at or below = overrun
  FILL_NEG_MAX_NM = 120,    -- [Nm] max absorbed ICE over-delivery
  FILL_RISE       = 5,      -- [1/s] fill gain rise; drops are instant
  TF_ACTIVE       = 0.99,   -- throttleFactor below = DSC intervening
  DSE_SRC_STOCK_MASK = 130, -- etk854hybrid_dseSrc bits 2 MTC yaw trim + 128 BC yaw brake (gaugeData)
  K0_CLOSED       = 0.9,    -- K0 ratio from which the ICE throttleFactor counts
  GOV_ACTIVE_THR  = 0.02,   -- idle-governor output above = governor zone
  TRIM_KI         = 1.0,    -- [1/s] ICE torque-model trim (state ON)
  TRIM_THR_MIN    = 0.02,   -- trim only inside this ICE throttle window
  TRIM_THR_MAX    = 0.98,
  TRIM_MAX_NM     = 60,
  TRIM_LEAK_TAU_S = 2.0,    -- [s] trim decay outside ON
  TRIM_DEXP_MAX   = 200,    -- [Nm/s] trim frozen above this iceExp rate
  TRIM_ERR_MAX    = 60,     -- [Nm] trim frozen above this model error
  COMB_TAU_S      = 0.10,
  DRAG_TAU_S      = 0.10,
  BISECT_N        = 12,
}

local st = {
  T = 0, rate = 0,          -- jerk-limited total [Nm], rate [Nm/s]
  ceil = 0, ceilInit = false,
  iceL = 0,                 -- planned ICE share [Nm]
  trim = 0, combLpf = 0, lastIceExp = 0,
  fg = 0,                   -- published fill gain
  drag = 0,                 -- ICE drag credit [Nm]
  gOn = false, gT = 0,      -- guard active / duration [s]
  gFreeT = 0, gFreeRate = 0,  -- uncapped shadow total
  kickT = 0, kickOn = false,
  fireThrT = 0,             -- launch FIRE: full-throttle hold left [s]
  tsaHold = nil,            -- HOLD cap of the running TSA episode [Nm] (tsaCapStep)
  tsaPrev = 0,              -- TSA torque mode of the last frame (ZERO entry from HOLD)
  ice = nil, motor = nil,
}

-- ===========================================================
-- Helpers
-- ===========================================================
local U = require("controller/etk854hybrid/etk854hybrid_util")
local clamp = U.clamp
local clamp01 = U.clamp01

local function rampTo(cur, target, upRate, dnRate, dt)
  if cur < target then return min(target, cur + upRate * dt) end
  return max(target, cur - dnRate * dt)
end

-- local copy of mathlib smoothmin (combustionEngine's throttle map blend)
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

-- What the E-machine can still add: its cap, within the system cap left over
-- by the ICE torque already on the shaft.
local function evTorqueAvail(cap, sysCap, iceNm)
  local avail = cap
  if sysCap > 0 then avail = min(avail, sysCap - max(0, iceNm)) end
  return clamp(avail, 0, max(0, cap))
end

-- ===========================================================
-- Exact inverse of the stock ICE torque model (combustionEngine.lua):
--   g    = torqueCurve[floor(rpm)] * intakeAirDensityCoef   (gross)
--   map  = smoothmin(max(t + t*k*(1-t), 0), 1, (1-t)*0.8),
--          k = maxPowerThrottleMap / (g*FI*av)
--   Tout = g*FI*map;  load = clamp01(Tout / ((g - fs - fd) * FI))
--   net  = Tout - fs - fd - engineBrakeTorque*(1 - load)   (= combustionTorque)
-- net(t) is monotonic -> bisection. A torque/cap fraction would over-deliver
-- up to 3x at part load.
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

-- ICE throttle 0..1 for netNm at rpm (sequencer SYNC, launch control)
function M.throttleForTorque(netNm, rpm, ctx)
  return throttleForTorque(netNm or 0, rpm or 0, ctx)
end

-- ===========================================================
-- Ceiling filter: motorCap + icePotential is an rpm curve lookup, so a gear
-- change would step the request at constant pedal. LPF, frozen while shifting.
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
-- Demand chain + jerk-limited total
-- ===========================================================
local function limiterRow(ctx)
  if ctx.launchActive then return P.ROWS.LAUNCH end
  return P.ROWS[ctx.deMode or "COMFORT"] or P.ROWS.COMFORT
end

-- Driver request after the system cap (launch: the combined envelope).
local function driverReq(ctx)
  local capSys = ctx.systemTorqueCapNm or 0
  local req = ctx.torqueReqNm or 0
  if ctx.launchActive then
    -- combined envelope, regardless of the pedal map
    req = (ctx.motorCapNm or 0)
          + iceCapAtRpm(ctx, max(ctx.iceRpmDev or ctx.iceRpm or 0, ctx.iceIdleRPM or 800))
  end
  if capSys > 0 and req > capSys then req = capSys end
  return req
end

-- TSA HOLD cap, once per frame before the demand. Latched at the total on the
-- episode's first HOLD / ZERO frame; re-latched on every ZERO entry from HOLD
-- at the total or the driver request there, whichever is higher, never above
-- the cap (K2 -> K1 returns to the pre-K2 torque; a ramp-up after an earlier
-- K2 keeps its cap); while HOLD and tsaTorqueRec > 0 (the TSA's settled bend
-- below the K1 band, not speeding up) it rises at that rate towards the driver
-- request. Mode 0 ends the episode.
local function tsaCapStep(ctx, dt)
  local tsm = ctx.tsaTorqueMode or 0
  if tsm ~= 1 and tsm ~= 2 then
    st.tsaHold, st.tsaPrev = nil, 0
    return
  end
  local cap = st.tsaHold
  if cap == nil then
    cap = st.T
  elseif tsm == 2 and st.tsaPrev == 1 then
    cap = min(cap, max(st.T, driverReq(ctx)))
  elseif tsm == 1 and (ctx.tsaTorqueRec or 0) > 0 then
    local req = driverReq(ctx)
    if req > cap then cap = min(req, cap + ctx.tsaTorqueRec * dt) end
  end
  st.tsaHold, st.tsaPrev = cap, tsm
end

-- Returns the demand and whether the limiter must hard-reset (N / P).
-- noGuard: without the brake-throttle guard cap (the guard's shadow).
local function demandTorque(ctx, noGuard)
  if (ctx.gearSign or 0) == 0 then return 0, true end
  -- launch abort, brake held: hard zero like N / P
  if ctx.launchDrop then ctx.launchDrop = false; return 0, true end
  local req = driverReq(ctx)
  -- TSA: 2 ZERO (ramps out at rdn), 1 HOLD (capped at tsaCapStep's cap)
  local tsm = ctx.tsaTorqueMode or 0
  if tsm == 2 then return 0, false end
  if tsm == 1 and st.tsaHold and req > st.tsaHold then req = st.tsaHold end
  -- brake-throttle guard: cap below what the driven axle's brakes hold
  local gcap = (not noGuard) and ctx.launchGuardNm or nil
  if gcap and req > gcap then req = gcap end
  req = req * (ctx.reverseLimit or 1)
  if ctx.sailSuppressDrive then return 0, false end
  -- drive restarts only after the regen has released: clean zero crossing
  if ctx.regenActiveLast then return 0, false end
  if req < 0 then req = 0 end
  return req, false
end

-- Kickdown gesture for KICK_RUP, once per frame (limitStep may run twice).
-- With a DE urgent pedal it must be held for urgentDwellS (same gesture as
-- the urgent start and the shift logic's kickdown, so a gamepad stab gets no
-- kickdown rise); else KICK_PEDAL, no dwell. pedalIn = raw driver pedal.
local function updateKick(ctx, dt)
  local kp, dwell = P.KICK_PEDAL, 0
  local dc, ec = ctx.deModeCfg, ctx.edriveCfg
  if dc and dc.urgentPedal and ec and ec.deTuned then kp, dwell = dc.urgentPedal, dc.urgentDwellS or 0 end
  if (ctx.pedalIn or 0) >= kp then st.kickT = st.kickT + dt else st.kickT = 0 end
  st.kickOn = st.kickT > 0 and st.kickT >= dwell
end

-- One limiter step from (T, rate) towards req; returns the new T, rate.
local function limitStep(T, rate, dt, req, hardReset, row, S)
  if hardReset then return 0, 0 end
  local rup = row.rup0 * (1 + row.sportGain * S)
  if st.kickOn then rup = max(rup, P.KICK_RUP) end
  local jk = row.jerk0 * (1 + row.jerkGain * S)
  local err = req - T
  local aerr = abs(err)
  if aerr < P.SNAP_NM and abs(rate) < 1 then return req, 0 end
  local R = (err > 0) and rup or row.rdn
  if err > 0 and T < P.LASH_NM then R = min(R, row.lash) end
  -- jerk-bounded approach: rate <= sqrt(2*J*|err|), still brakeable to zero
  local rDes = min(R, sqrt(2 * jk * aerr), aerr / max(dt, 1e-4))
  if err < 0 then rDes = -rDes end
  rate = rate + clamp(rDes - rate, -jk * dt, jk * dt)
  local Tn = T + rate * dt
  if (req - Tn) * err < 0 then       -- crossed the target: snap, no overshoot
    Tn, rate = req, 0
  end
  return Tn, rate
end

-- ===========================================================
-- Split: planned ICE share per DE mode
-- ===========================================================
local function plannedIceShare(ctx, Tb, iceCapNow, motorCapEff)
  local soc = ctx.soc or 1
  local split
  if ctx.chargingActive then
    -- BATTERY CONTROL load-point shift: drive torque + charge torque
    split = min(iceCapNow, Tb + (ctx.chargeNmLast or 0))
  elseif ctx.deMode == "SPORT" or ctx.legacyIceForce or soc < P.SPLIT_SOC_ICE_FIRST then
    -- ICE carries the base, the motor fills lag and boosts above the ICE cap
    split = min(Tb, iceCapNow)
  elseif ctx.deMode == "ECO_PRO" then
    -- motor budget, faded out by SOC and below the policy's EV top speed
    local ec = ctx.edriveCfg
    local top = (ec and ec.deTuned and ctx.deModeCfg and ctx.deModeCfg.evTopAuto)
                or (ec and ec.evTopKmh) or 110
    local budget = P.ECO_MOTOR_SHARE * motorCapEff
      * clamp01((soc - P.ECO_SOC_LO) / P.ECO_SOC_SPAN)
      * clamp01((top - (ctx.speedKmh or 0)) / P.ECO_SPEED_FADE_KMH)
    split = clamp(Tb - budget, 0, iceCapNow)
  else
    split = min(Tb, iceCapNow)     -- COMFORT: ICE-led
  end
  -- what the motor cannot deliver goes to the ICE
  split = max(split, min(iceCapNow, Tb - motorCapEff))
  if split < 0 then split = 0 end
  return split
end

-- ===========================================================
-- DSC / governor state (fill + trim gates)
-- ===========================================================
-- Returns (any intervention, tsaOnly). tsaOnly: escActive from the TSA brake
-- stage alone (its lamp is held past the stage to cover the escActive tail);
-- the locked fill may then stay on.
local function dscIntervening(ctx)
  local ev = electrics.values
  -- electrics.lua turns booleans into 1/0: a truth test would read 0 as true
  local esc, tcs = ev.escActive, ev.tcsActive
  local escOn = esc == true or (tonumber(esc) or 0) > 0.5
  local stock = tcs == true or (tonumber(tcs) or 0) > 0.5 or (tonumber(ev.isYCBrakeActive) or 0) > 0.5
    or bit.band(floor(tonumber(ev.etk854hybrid_dseSrc) or 0), P.DSE_SRC_STOCK_MASK) ~= 0
  local ice, motor = st.ice, st.motor
  -- the ICE factor only matters while K0 is closed
  if not stock and ice and (ctx.k0Current or 1) >= P.K0_CLOSED then
    local tf = ev[ice.electricsThrottleFactorName or "throttleFactor"] or ice.throttleFactor or 1
    if tf < P.TF_ACTIVE then stock = true end
  end
  if not stock and motor and (motor.throttleFactor or 1) < P.TF_ACTIVE then stock = true end
  local tsaOnly = escOn and not stock and (tonumber(ev.etk854hybrid_tsaLamp) or 0) > 0.5
  return escOn or stock, tsaOnly
end

-- Fill gain. Slip states (K0 crank / open / engage): pure cancellation of the
-- measured K0 torque, stays on in overrun, regen and shifts. Locked: off in
-- overrun, it would cancel the engine braking.
local function updateFillGain(ctx, dt, overrun, dsc, slipState)
  local target = ctx.seqFillGain or 0
  if (ctx.gearSign or 0) == 0 or ctx.iceIdleEngage or dsc then target = 0 end
  if (ctx.chargeModeLast or "OFF") ~= "OFF" then target = 0 end
  if not slipState then
    -- TSA ZERO: locked fill stays on in overrun, cancels the ICE drag
    if overrun and ctx.tsaTorqueMode ~= 2 then target = 0 end
    -- idle governor is a speed controller: a fill would only fight it
    local ice = st.ice
    if ice and (ice.idleThrottle or 0) > P.GOV_ACTIVE_THR then target = 0 end
  end
  -- shifts gate the locked fill off (it would fight the torque intervention)
  if ctx.isShifting and not slipState then target = 0 end
  if target > st.fg then
    -- instant in slip states: the error starts at ~0, a ramp lets the crank dip through
    if ctx.seqFillInstant then st.fg = target else st.fg = min(target, st.fg + P.FILL_RISE * dt) end
  else
    st.fg = target
  end
  return st.fg
end

-- Slow trim: static gap between the inverse model and ice.combustionTorque
local function updateTrim(ctx, dt, state, iceExp, iceThr, dsc, overridden)
  local ice = st.ice
  local comb = (ice and type(ice.combustionTorque) == "number") and ice.combustionTorque or iceExp
  st.combLpf = st.combLpf + (comb - st.combLpf) * min(1, dt / P.COMB_TAU_S)
  local dExp = (iceExp - st.lastIceExp) / max(dt, 1e-4)
  st.lastIceExp = iceExp
  local govActive = ice and (ice.idleThrottle or 0) > P.GOV_ACTIVE_THR
  if state == "ON" and not overridden and not ctx.isShifting and not dsc and not govActive
     and iceThr > P.TRIM_THR_MIN and iceThr < P.TRIM_THR_MAX and abs(dExp) < P.TRIM_DEXP_MAX
     and abs(iceExp - st.combLpf) < P.TRIM_ERR_MAX then
    st.trim = clamp(st.trim + P.TRIM_KI * (iceExp - st.combLpf) * dt, -P.TRIM_MAX_NM, P.TRIM_MAX_NM)
  elseif state ~= "ON" then
    st.trim = st.trim * exp(-dt / P.TRIM_LEAK_TAU_S)
  end
end

-- ICE drag credit (ctx.iceDragNm): the VCU subtracts the coupled ICE's
-- fuel-cut drag from the regen, so the coast decel does not depend on K0.
-- Coupled only: in the slip states the fill already cancels the K0 torque.
local function updateDragCredit(ctx, dt, overrun)
  local raw = 0
  if overrun and ctx.iceCoupled then raw = max(0, -(ctx.iceMeasNm or 0)) end
  st.drag = st.drag + (raw - st.drag) * min(1, dt / P.DRAG_TAU_S)
  if raw <= 0 and st.drag < 0.5 then st.drag = 0 end
  ctx.iceDragNm = st.drag
end

-- ===========================================================
-- Launch control hooks
-- ===========================================================
-- Hold: launch control drives ICE throttle and generator itself; no drive
-- torque, no fill, trim frozen. Total and ICE share track the measured shaft
-- torque, so a hold without a fire restarts from what the brakes held.
local function holdBranch(ctx, dt, ev)
  local motor, ice = st.motor, st.ice
  local shaft = motor and tonumber(motor.etkShaftNm) or st.T
  st.T, st.rate = max(0, shaft), 0
  st.iceL = st.T
  st.fg = 0
  st.drag = 0
  st.gOn = false                     -- no shadow to hand over after a hold
  local comb = (ice and tonumber(ice.combustionTorque)) or 0
  st.combLpf = st.combLpf + (comb - st.combLpf) * min(1, dt / P.COMB_TAU_S)
  st.lastIceExp = st.combLpf         -- no dExp step for the trim gate
  local thr = clamp01(ctx.launchIceThr or 0)
  local capFull = ctx.motorCapNm or 0
  local sysCap = ctx.systemTorqueCapNm or 0
  ev.etk854hybrid_evThrottle = 0
  ev.etk854hybrid_iceThrottle = ctx.iceOn and thr or 0
  ev.etk854hybrid_iceExpNm = 0
  ev.etk854hybrid_fillGain = 0
  ev.etk854hybrid_fillPosMaxNm = 0
  ev.etk854hybrid_fillNegMaxNm = 0
  -- E-machine headroom at release
  ev.etk854hybrid_evTorqueAvailNm = evTorqueAvail(capFull, sysCap, st.combLpf)
  ctx.iceDragNm = 0
  ctx.chargingActive = false
  ctx.fillSlipState = false
  ctx.torqueDeficitNm = 0
  ctx.torqueTargetNm = ctx.launchPumpCapNm or st.T
  ctx.motorBaseNm, ctx.motorBaseRawNm = 0, 0
  ctx.iceExpNm = st.combLpf
end

-- FIRE: total and ICE share start at the seed (ICE shaft torque at release),
-- so the LAUNCH row ramps up from there and the split does not pull the ICE
-- back first. Fill gain starts at once (turbo-lag fill).
local function seedFire(ctx, seed)
  local s = max(0, tonumber(seed) or 0)
  local capSys = ctx.systemTorqueCapNm or 0
  if capSys > 0 then s = min(s, capSys) end
  if s > st.T then st.T = s end
  st.rate = 0
  st.iceL = s
  st.fg = clamp01(ctx.seqFillGain or 0)
  -- full throttle first: the static inverse would read the boost-lagged seed
  -- as part throttle
  st.fireThrT = P.FIRE_THR_S
end

-- ===========================================================
-- Main update
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
  -- guard below the total: cut at once, not at rdn
  local gcap = ctx.launchGuardNm
  if gcap then
    if not st.gOn then st.gOn, st.gT, st.gFreeT, st.gFreeRate = true, 0, st.T, st.rate end
    st.gT = st.gT + dt
    if st.T > gcap then st.T = gcap; if st.rate > 0 then st.rate = 0 end end
    if st.iceL > gcap then st.iceL = gcap end
  elseif st.gOn then
    -- short guard (gamepad cross-over): resume the shadow total
    st.gOn = false
    if st.gT <= P.GUARD_SNAP_MAX_S and st.gFreeT > st.T then
      st.T, st.rate = st.gFreeT, max(st.rate, st.gFreeRate)
    end
  end
  local S = ctx.sportIndex or 0
  local row = limiterRow(ctx)
  updateKick(ctx, dt)

  tsaCapStep(ctx, dt)
  local req, hardReset = demandTorque(ctx)
  st.T, st.rate = limitStep(st.T, st.rate, dt, req, hardReset, row, S)
  local Tb = st.T                      -- before kShift
  if st.gOn then
    -- shadow: same limiter on the uncapped demand
    local rf, hr = demandTorque(ctx, true)
    st.gFreeT, st.gFreeRate = limitStep(st.gFreeT, st.gFreeRate, dt, rf, hr, row, S)
  end
  local kShift = ctx.kShift or 1       -- shift torque factor 0..1
  local Tcmd = Tb * kShift

  local state = ctx.iceEngageState or "OFF"
  local rpmIce = ctx.iceRpmDev or ctx.iceRpm or 0
  local idle = ctx.iceIdleRPM or 800
  local capFull = ctx.motorCapNm or 0
  -- K0 crank torque is reserved, so the driver cannot starve the start
  local motorCapEff = max(0, capFull - (ctx.iceCrankReserveNm or 0))
  local iceCapNow = 0
  if ctx.iceCoupled or state == "ENGAGE" then iceCapNow = iceCapAtRpm(ctx, max(rpmIce, idle)) end
  -- Overrun = the total has reached zero, not the pedal: the ICE follows the
  -- ramp-down instead of leaving the E-machine alone against its drag.
  local overrun = Tcmd <= P.OVERRUN_NM

  ctx.chargingActive = (ctx.chargeToTarget == true) and (ctx.soc or 1) < (ctx.batteryTargetSoc or 0)
                       and state == "ON" and not ctx.tsaFreeze
  local split = plannedIceShare(ctx, Tb, iceCapNow, motorCapEff)
  if overrun then
    st.iceL = 0
  else
    local motorSat = (Tb - st.iceL) >= motorCapEff - P.SAT_MARGIN_NM
    local up = motorSat and P.ICE_UP_SAT or (row.iceRise0 * (1 + row.iceRiseGain * S))
    st.iceL = rampTo(st.iceL, split, up, max(row.iceFall, row.rdn), dt)
  end

  local relTorque = (state == "RELEASE" and ctx.iceRelPhase == "TORQUE")
  local iceExpRaw = 0
  if state == "BLEND" or state == "ON" or relTorque then
    iceExpRaw = (ctx.iceShareGate or 0) * st.iceL
  end
  local motorBaseRaw = clamp(Tb - iceExpRaw, 0, motorCapEff)
  local iceExp = iceExpRaw * kShift
  local motorBase = motorBaseRaw * kShift
  ctx.torqueDeficitNm = max(0, Tb - iceExpRaw - motorBaseRaw)

  -- ---- ICE throttle ----
  local iceThr
  local ov = ctx.iceThrottleOverride
  if ov ~= nil then
    iceThr = clamp01(ov)
  elseif overrun or state == "OFF" or state == "CRANK" then
    iceThr = 0
  else
    iceThr = throttleForTorque(iceExp + st.trim, rpmIce, ctx)
  end
  if st.fireThrT > 0 then
    st.fireThrT = st.fireThrT - dt
    if ctx.launchActive and not overrun and ov == nil then iceThr = 1 end
  end
  local dsc, dscTsa = dscIntervening(ctx)
  updateTrim(ctx, dt, state, iceExp, iceThr, dsc, ov ~= nil)

  -- ---- fill ----
  local slipState = (state == "CRANK" or state == "SYNC" or state == "ENGAGE"
                     or (state == "RELEASE" and ctx.iceRelPhase == "OPEN"))
  ctx.fillSlipState = slipState
  local fg = updateFillGain(ctx, dt, overrun, dsc and not dscTsa, slipState)
  local avShaft = abs(ctx.rpmMotor or 0) * RPM_TO_AV
  local regenKw = ctx.stageRegenKw or 55
  local sysCap = ctx.systemTorqueCapNm or 0

  ev.etk854hybrid_evThrottle = clamp01(motorBase / max(1, ctx.motorCurveNm or 1))
  ev.etk854hybrid_iceThrottle = ctx.iceOn and iceThr or 0
  ev.etk854hybrid_iceExpNm = iceExp
  ev.etk854hybrid_fillGain = fg
  ev.etk854hybrid_fillPosMaxNm = max(0, capFull - motorBase)
  local negMax = min(P.FILL_NEG_MAX_NM, regenKw * 1000 / max(avShaft, 50)) * (ctx.regenAcceptFraction or 1)
  -- guard: the fill may also take back the motor's own drive torque, so a
  -- start's K0 / ICE torque stays out of the capped converter input
  if gcap then negMax = negMax + max(0, motorBase) end
  ev.etk854hybrid_fillNegMaxNm = negMax
  ev.etk854hybrid_evTorqueAvailNm = evTorqueAvail(motorCapEff, sysCap, iceExp)

  updateDragCredit(ctx, dt, overrun)

  ctx.torqueTargetNm = Tcmd
  ctx.motorBaseNm = motorBase
  ctx.motorBaseRawNm = motorBaseRaw   -- before kShift (K0 start headroom)
  ctx.iceExpNm = iceExp
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
  st.tsaHold, st.tsaPrev = nil, 0
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
           fillGain = st.fg, dragNm = st.drag, tsaHold = st.tsaHold }
end

return M
