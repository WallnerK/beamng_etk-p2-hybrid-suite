-- etk854hybrid_creep.lua - ETK 854 ActiveHybrid - EV creep ("virtual idle")
--
-- With the ICE off the motor device (motorShaft creepFloor) holds the converter
-- pump at the ICE idle speed as a floor, so the car creeps in D / R through the
-- stock converter physics. This module (VCU frame, after torqueCoord, before
-- updateRecuperation) only decides whether the floor acts, its target (idle or
-- HOLD_RPM) and its authority (min(MAX_NM, motorCapNm), faded out above V_FULL_KMH).
-- With the ICE coupled the engine idle governor creeps instead (reason "ice").
-- HOLD: braked at a standstill for HOLD_DWELL_S (or D / R selected with the brake
-- held) -> HOLD_RPM, no drain at a red light. Pedal-independent, so the launch
-- preload / brake-throttle guard never see creep torque.
-- Disarmed: target kept, limit 0 -> the device fades the floor out (no step at
-- the BLEND handover); creepCut = 1 drops it in the same tick (failsafe, key off,
-- launch flags, lockout / deep derate, ABS).
--
-- Channels (etk854hybrid_*): creepTargetRpm, creepMaxNm, creepHold,
--   creepIdleRpm (0 = not armed), bodyKmh (signed, + = forward),
--   creepCut (launchControl also sets it in the hold),
--   creepState 0 OFF | 1 ICE | 2 HOLD | 3 SPINUP | 4 CREEP | 5 READY (not binding),
--   creepReason ok | disabled | failsafe | ignition | PN | arcade | onePedal |
--   launch | lockout | ice | sail | abs | gear | speed | init
-- ctx: creepIdleRpm (recuperation fade, same frame), creepAvail (every gate but
--   "ice" and the speed fade passes; policyWantsIce reads it next frame so the ICE
--   is not kept running while the car merely creeps)
-- Device PI / ramps / fades: integratedMotor in etk854hybrid_powertrain.jbeam.
local M = { name = "creep", isReal = true }

local max, min, abs = math.max, math.min, math.abs
local AV_TO_RPM = 9.549296596

-- Defaults; overridden by the hybridControl args named in ARG.
local CR = {
  ENABLE = 1,             -- 0 = creep only with the ICE coupled
  IDLE_OFFSET_RPM = 0,    -- target = ICE idle + offset
  IDLE_MIN_RPM = 500, IDLE_MAX_RPM = 900,
  MAX_NM = 90,            -- [Nm] shaft: ~40 Nm stall load + spin-up inertia
  MIN_CAP_NM = 20,        -- motorCapNm below this = deep derate, no creep
  V_FULL_KMH = 12, V_ZERO_KMH = 16,
  MAX_GEAR = 2,           -- |gearIndex| above this -> off (M3+, coasting)
  HOLD_BRAKE_IN = 0.15,
  HOLD_BRAKE_OUT = 0.08,  -- ~the brake that just holds the creep torque
  PARKBRAKE_ON = 0.10,    -- = recuperation hardCut threshold
  HOLD_KMH = 0.5, HOLD_EXIT_KMH = 1.5, HOLD_DWELL_S = 0.30,
  HOLD_RPM = 0,
  HILL_GRADE = 0.05,      -- no HOLD uphill above this grade: pump stays at idle as with
                          -- an ICE (0 = rule off)
  HIGH_REVERSE = 1,       -- 1 = creep in R also with regen HIGH (no one-pedal hold in R)
  SPINUP_FRAC = 0.9,      -- SPINUP while pump < this x target
  CREEP_NM_MIN = 0.5,     -- CREEP while the floor adds more than this [Nm]
}
local ARG = {
  ENABLE = "evCreep", IDLE_OFFSET_RPM = "creepIdleOffsetRpm", MAX_NM = "creepMaxNm",
  MIN_CAP_NM = "creepMinCapNm", V_FULL_KMH = "creepVFullKmh", V_ZERO_KMH = "creepVZeroKmh",
  MAX_GEAR = "creepMaxGear", HOLD_BRAKE_IN = "creepHoldBrake", HOLD_BRAKE_OUT = "creepReleaseBrake",
  HOLD_KMH = "creepHoldKmh", HOLD_EXIT_KMH = "creepHoldExitKmh", HOLD_DWELL_S = "creepHoldDwellS",
  HOLD_RPM = "creepHoldRpm", HILL_GRADE = "creepHillGrade", HIGH_REVERSE = "creepHighReverse",
}
-- pristine defaults: a second init() in the same VM must not inherit overrides
local CR_DEF = {}
for k, v in pairs(CR) do CR_DEF[k] = v end

local st = { hold = false, holdT = 0, tgt = 0, wasArmed = false, motor = nil,
             reason = "init", code = 0, avail = false }

-- an undefined jbeam $variable arrives as the literal "$name" -> default
local U = require("controller/etk854hybrid/etk854hybrid_util")
local num = U.num
local function clamp01(x) if x < 0 then return 0 elseif x > 1 then return 1 end return x end

local function publish(ev, tgt, lim, idle, code, reason, hold, body, cut)
  ev.etk854hybrid_creepTargetRpm = tgt
  ev.etk854hybrid_creepMaxNm = lim
  ev.etk854hybrid_creepCut = cut and 1 or 0
  ev.etk854hybrid_creepIdleRpm = idle
  ev.etk854hybrid_creepState = code
  ev.etk854hybrid_creepReason = reason
  ev.etk854hybrid_creepHold = hold and 1 or 0
  ev.etk854hybrid_bodyKmh = body
end

function M.reset(ctx)
  st.hold, st.holdT, st.tgt, st.wasArmed = false, 0, 0, false
  st.reason, st.code, st.avail = "init", 0, false
  if ctx then ctx.creepIdleRpm, ctx.creepAvail = 0, false end
  publish(electrics.values, 0, 0, 0, 0, "init", false, 0, false)
end

function M.init(ctx)
  local jb = (ctx and ctx.jbeamData) or {}
  for k, v in pairs(CR_DEF) do CR[k] = v end
  for k, a in pairs(ARG) do CR[k] = num(jb[a], CR[k]) end
  CR.V_ZERO_KMH = max(CR.V_ZERO_KMH, CR.V_FULL_KMH + 0.5)
  CR.HOLD_BRAKE_OUT = min(CR.HOLD_BRAKE_OUT, CR.HOLD_BRAKE_IN)
  CR.MAX_NM = max(0, CR.MAX_NM)
  M.reset(ctx)
  if ctx and ctx.dbg then
    ctx.dbg(1, "creep", string.format(
      "EV creep %s | maxNm %.0f holdBrake %.2f/%.2f dwell %.2fs hill %.2f highR %d",
      (CR.ENABLE ~= 0) and "ON" or "OFF", CR.MAX_NM, CR.HOLD_BRAKE_IN, CR.HOLD_BRAKE_OUT,
      CR.HOLD_DWELL_S, CR.HILL_GRADE, CR.HIGH_REVERSE))
  end
end

function M.initStage2(ctx)
  st.motor = powertrain.getDevice("integratedMotor")
end

function M.updateGFX(ctx, dt)
  local ev = electrics.values
  local gs = ctx.gearSign or 0
  local kmh = ctx.speedKmh or 0
  local brake = ctx.brakeRaw or 0
  local pb = ev.parkingbrake or 0
  local es = ctx.iceEngageState or "OFF"
  local idle = (ctx.iceIdleRPM or 700) + CR.IDLE_OFFSET_RPM
  idle = min(CR.IDLE_MAX_RPM, max(CR.IDLE_MIN_RPM, idle))
  -- body speed + slope along the travel direction (XYZ getters: no vec3 per frame)
  local body, gradeTravel = 0, 0
  if obj and obj.getDirectionVectorXYZ then
    local dx, dy, dz = obj:getDirectionVectorXYZ()
    local vx, vy, vz = obj:getVelocityXYZ()
    body = (vx * dx + vy * dy + vz * dz) * 3.6
    gradeTravel = dz * gs
  end

  -- arming: the first failing gate names the reason
  local reason = "ok"
  local lvl = ctx.regenLevelOrder and ctx.regenLevelOrder[ctx.regenLevelIdx]
  local ignOff = (ev.ignitionLevel or 2) < 2   -- key off / accessory
  local launchOn = ctx.launchArmed or ctx.launchReady or ctx.launchActive or ctx.launchHold
    or ctx.launchGuardNm
  local lockout = ctx.evLockoutActive or (ctx.motorCapNm or 0) < CR.MIN_CAP_NM
  local absOn = (ev.etk854hybrid_absQualified or 0) > 0.5
  if CR.ENABLE == 0 then reason = "disabled"
  elseif ctx.seqFailsafe then reason = "failsafe"
  elseif ignOff then reason = "ignition"
  elseif gs == 0 then reason = "PN"
  elseif ev.gearboxMode == "arcade" then reason = "arcade"   -- AI + arcade auto-N / auto-brake
  elseif lvl == "HIGH" and (gs > 0 or CR.HIGH_REVERSE == 0) then reason = "onePedal"
  elseif launchOn then reason = "launch"
  elseif lockout then reason = "lockout"
  end
  -- hard cut (any rank): the device drops the floor in the same tick, no fade
  local cut = (ctx.seqFailsafe or ignOff or launchOn or lockout or absOn) and true or false
  -- gates ranked below the ICE gate
  local tail = nil
  if ctx.sailActive then tail = "sail"
  elseif absOn then tail = "abs"
  elseif abs(ev.gearIndex or 0) > CR.MAX_GEAR then tail = "gear"
  end
  if reason == "ok" then
    if ctx.iceCoupled then reason = "ice"                     -- BLEND / ON / RELEASE:TORQUE
    elseif tail then reason = tail end
  end
  -- speed fade excluded: the ICE policy has its own limit
  st.avail = (reason == "ok" or reason == "ice") and tail == nil
  ctx.creepAvail = st.avail
  local vGate = clamp01((CR.V_ZERO_KMH - kmh) / (CR.V_ZERO_KMH - CR.V_FULL_KMH))
  if reason == "ok" and vGate <= 0 then reason = "speed" end
  local armed = reason == "ok"

  -- standstill HOLD
  local braked = brake >= CR.HOLD_BRAKE_IN or pb >= CR.PARKBRAKE_ON
  local released = brake < CR.HOLD_BRAKE_OUT and pb < CR.PARKBRAKE_ON
  local uphill = CR.HILL_GRADE > 0 and gradeTravel > CR.HILL_GRADE
  local seqBusy = es == "CRANK" or es == "SYNC" or es == "ENGAGE"   -- never move the shaft under a rev-match
  local canHold = armed and kmh < CR.HOLD_KMH and braked and not seqBusy and not uphill
  if not armed then
    st.hold, st.holdT = false, 0
  elseif st.hold then
    if released or kmh >= CR.HOLD_EXIT_KMH or uphill then st.hold, st.holdT = false, 0 end
  elseif canHold and not st.wasArmed then
    st.hold = true                                  -- D/R selected with the brake held: no spin-up
  elseif canHold then
    st.holdT = st.holdT + dt
    if st.holdT >= CR.HOLD_DWELL_S then st.hold = true end
  else
    st.holdT = 0
  end
  st.wasArmed = armed

  -- device inputs (disarmed: target kept, limit 0 -> fade-out)
  if armed then st.tgt = st.hold and CR.HOLD_RPM or idle end
  local lim = armed and min(CR.MAX_NM, ctx.motorCapNm or 0) * vGate or 0
  ctx.creepIdleRpm = armed and idle or 0

  local code
  if reason == "ice" then code = 1
  elseif not armed then code = 0
  elseif st.hold then code = 2
  else
    local m = st.motor
    local pump = m and abs(m.outputAV1 or 0) * AV_TO_RPM or 0
    if pump < CR.SPINUP_FRAC * st.tgt then code = 3
    elseif (ev.etk854hybrid_creepNm or 0) > CR.CREEP_NM_MIN then code = 4
    else code = 5 end
  end
  st.reason, st.code = reason, code
  publish(ev, st.tgt, lim, ctx.creepIdleRpm, code, reason, st.hold, body, cut)
end

function M.getState() return st end

return M
