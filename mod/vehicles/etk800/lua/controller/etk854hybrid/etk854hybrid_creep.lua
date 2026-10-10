-- etk854hybrid_creep.lua - ETK 854 ActiveHybrid - EV creep / "virtual idle"
--
-- User feedback round 3: with the engine off the car did not creep in D / R
-- like an automatic ("ganz normal kriechen"), it only crept while the ICE
-- idled on the closed K0. The E-machine now emulates the ICE idle governor:
-- the motor device (etk854hybrid_motorShaft.lua creepFloor, physics rate)
-- holds the converter pump at the ICE idle speed as a FLOOR, so creep torque
-- and creep speed come out of the stock converter physics, exactly as with
-- the engine idling. This module runs once per VCU frame (master updateGFX:
-- after torqueCoord, before updateRecuperation) and decides only
--   * WHETHER the floor may act (arming gates, first failing gate = reason),
--   * at WHICH speed: ICE idle (powerCurves) or the HOLD standby speed,
--   * with WHICH torque authority (min(MAX_NM, motorCapNm), faded out
--     between V_FULL_KMH and V_ZERO_KMH).
-- It never touches torqueReqNm, the jerk limiter, demandComp, the ICE policy
-- or the 8HP request. With the ICE coupled the creep comes from the engine
-- idle governor as before (reason "ice"); the device also fades the floor out
-- on its own whenever K0 torque reaches the shaft.
--
-- HOLD = the 8HP "neutral idle control": braked (brake >= HOLD_BRAKE_IN or
-- parking brake) at a standstill for HOLD_DWELL_S - or immediately when D / R
-- is selected with the brake held - the target drops to HOLD_RPM (0): the
-- E-machine stands still, no drain, no whine at a red light. It is pedal-
-- independent on purpose: brake + pedal (launch preload, brake-throttle
-- guard) never sees creep torque. Exit: brake < HOLD_BRAKE_OUT with the
-- parking brake off, HOLD_EXIT_KMH, uphill, or disarmed. Never entered while
-- the sequencer rev-matches (CRANK / SYNC / ENGAGE). No HOLD uphill in the
-- travel direction above HILL_GRADE (the pump stays at idle, as with an ICE).
--
-- Disarmed: the TARGET is kept and only the authority limit drops to 0, so
-- the device fades the floor out (creepAuthFallRate, on the last armed limit
-- it latches) instead of pulling the setpoint down (no ~40 Nm step at the
-- BLEND handover). Hard cut (creepCut = 1, device authority 0 in the same
-- physics tick, no fade): failsafe, key off, any launch flag (launch control
-- / brake-throttle guard), battery lockout / deep derate, ABS.
--
-- Channels (all etk854hybrid_*):
--   creepTargetRpm  device setpoint [rpm]        creepMaxNm  device limit [Nm]
--   creepIdleRpm    armed virtual idle, 0 = not armed (recuperation fade:
--                   updateRecuperation reads ctx.creepIdleRpm the same frame)
--   creepState      0 OFF | 1 ICE | 2 HOLD | 3 SPINUP (pump < 0.9 x target)
--                   | 4 CREEP (floor adds > 0.5 Nm) | 5 READY (armed, floor
--                   not binding: pedal, regen or rolling above creep speed)
--   creepReason     ok | disabled | failsafe | ignition | PN | arcade |
--                   onePedal | launch | lockout | ice | sail | abs | gear |
--                   speed | init
--   creepHold       1 in HOLD           bodyKmh  signed body speed, + = forward
--   creepCut        1 = hard cut (see above; launchControl.applyOutputs also
--                   writes 1 as the frame's last writer during the hold)
-- ctx outputs:
--   ctx.creepIdleRpm  (= channel creepIdleRpm)
--   ctx.creepAvail    every gate except "ice" and the speed fade passes, i.e.
--                     the E-machine WOULD creep if the ICE were not coupled.
--                     Read by policyWantsIce on the NEXT frame (it runs before
--                     this module): the AUTO standstill hard-off is extended to
--                     the creep regime (< 8 km/h, pedal < 0.04) so the ICE is
--                     not kept running (coupled or IDLE) while the car merely
--                     creeps.
-- Device parameters (PI, ramps, fades) live in the integratedMotor section of
-- etk854hybrid_powertrain.jbeam; this module's parameters are hybridControl
-- arguments (ARG below).
local M = { name = "creep", isReal = true }

local max, min, abs = math.max, math.min, math.abs
local AV_TO_RPM = 9.549296596

-- Defaults; every key can be overridden by the hybridControl jbeam argument
-- named in ARG (etk854hybrid_powertrain.jbeam).
local CR = {
  ENABLE = 1,             -- 0 = v1.0.1 behaviour (creep only with the ICE coupled)
  IDLE_OFFSET_RPM = 0,    -- target = ICE idle (powerCurves: N57 700, B58 / i4 650) + offset
  IDLE_MIN_RPM = 500, IDLE_MAX_RPM = 900,
  MAX_NM = 90,            -- authority limit at the shaft [Nm] (min with motorCapNm,
                          -- which carries derate, evCapScale, reverse limiter, lockout);
                          -- stall load 35-41 Nm plus spin-up inertia
  MIN_CAP_NM = 20,        -- motorCapNm below this = deep derate -> no EV creep
  V_FULL_KMH = 12, V_ZERO_KMH = 16,  -- authority fades out above creep speed
  MAX_GEAR = 2,           -- |gearIndex| above this -> off (M3+ / coasting in a high gear)
  HOLD_BRAKE_IN = 0.15,   -- brake that enters HOLD at a standstill
  HOLD_BRAKE_OUT = 0.08,  -- below this HOLD exits: ~the brake that just holds the
                          -- creep stall torque (843 Nm at the wheel, N57 D1)
  PARKBRAKE_ON = 0.10,    -- = recuperation hardCut threshold
  HOLD_KMH = 0.5, HOLD_EXIT_KMH = 1.5, HOLD_DWELL_S = 0.30,
  HOLD_RPM = 0,           -- standby speed in HOLD (0 = no drain at a red light)
  HILL_GRADE = 0.05,      -- no HOLD uphill in the travel direction (0 = rule off)
  HIGH_REVERSE = 1,       -- 1 = creep in R also with regen level HIGH (no one-pedal
                          -- hold exists in R; 0 = strict: HIGH never creeps)
  SPINUP_FRAC = 0.9,      -- state SPINUP while pump < this x target
  CREEP_NM_MIN = 0.5,     -- state CREEP while the floor adds more than this
}
local ARG = {
  ENABLE = "evCreep", IDLE_OFFSET_RPM = "creepIdleOffsetRpm", MAX_NM = "creepMaxNm",
  MIN_CAP_NM = "creepMinCapNm", V_FULL_KMH = "creepVFullKmh", V_ZERO_KMH = "creepVZeroKmh",
  MAX_GEAR = "creepMaxGear", HOLD_BRAKE_IN = "creepHoldBrake", HOLD_BRAKE_OUT = "creepReleaseBrake",
  HOLD_KMH = "creepHoldKmh", HOLD_EXIT_KMH = "creepHoldExitKmh", HOLD_DWELL_S = "creepHoldDwellS",
  HOLD_RPM = "creepHoldRpm", HILL_GRADE = "creepHillGrade", HIGH_REVERSE = "creepHighReverse",
}
-- pristine defaults: a second init() in the same Lua VM starts from these,
-- not from the previous config's overrides
local CR_DEF = {}
for k, v in pairs(CR) do CR_DEF[k] = v end

local st = { hold = false, holdT = 0, tgt = 0, wasArmed = false, motor = nil,
             reason = "init", code = 0, avail = false }

-- jbeam $variables arrive substituted; a .pc that never defines one (or a
-- typo leaving the literal "$name") falls back to the default
local function num(v, d)
  if type(v) == "number" then return v end
  if type(v) == "string" then return tonumber(v) or d end
  return d
end
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
  -- body speed + slope along the travel direction (one vec read per frame)
  local body, gradeTravel = 0, 0
  if obj and obj.getDirectionVector then
    local d, v = obj:getDirectionVector(), obj:getVelocity()
    body = (v.x * d.x + v.y * d.y + v.z * d.z) * 3.6
    gradeTravel = d.z * gs
  end

  -- ---- arming: first failing gate names the reason ----
  local reason = "ok"
  local lvl = ctx.regenLevelOrder and ctx.regenLevelOrder[ctx.regenLevelIdx]
  if CR.ENABLE == 0 then reason = "disabled"
  elseif ctx.seqFailsafe then reason = "failsafe"
  elseif (ev.ignitionLevel or 2) < 2 then reason = "ignition"   -- key off / accessory
  elseif gs == 0 then reason = "PN"
  elseif ev.gearboxMode == "arcade" then reason = "arcade"   -- AI + arcade auto-N / auto-brake
  elseif lvl == "HIGH" and (gs > 0 or CR.HIGH_REVERSE == 0) then reason = "onePedal"
  elseif ctx.launchArmed or ctx.launchReady or ctx.launchActive or ctx.launchHold
      or ctx.launchGuardNm then reason = "launch"
  elseif ctx.evLockoutActive or (ctx.motorCapNm or 0) < CR.MIN_CAP_NM then reason = "lockout"
  end
  -- hard cut, independent of the reason ranking: the device drops the floor
  -- in the same tick instead of fading it out
  local cut = (ctx.seqFailsafe or (ev.ignitionLevel or 2) < 2
    or ctx.launchArmed or ctx.launchReady or ctx.launchActive or ctx.launchHold or ctx.launchGuardNm
    or ctx.evLockoutActive or (ctx.motorCapNm or 0) < CR.MIN_CAP_NM
    or (ev.etk854hybrid_absQualified or 0) > 0.5) and true or false
  -- gates ranked below the ICE gate (an ICE-coupled frame reports "ice")
  local tail = nil
  if ctx.sailActive then tail = "sail"
  elseif (ev.etk854hybrid_absQualified or 0) > 0.5 then tail = "abs"
  elseif abs(ev.gearIndex or 0) > CR.MAX_GEAR then tail = "gear"
  end
  if reason == "ok" then
    if ctx.iceCoupled then reason = "ice"                     -- BLEND / ON / RELEASE:TORQUE
    elseif tail then reason = tail end
  end
  -- creep regime for the ICE policy: the E-machine would creep if the ICE
  -- were not coupled (speed fade excluded, the policy has its own limit)
  st.avail = (reason == "ok" or reason == "ice") and tail == nil
  ctx.creepAvail = st.avail
  local vGate = clamp01((CR.V_ZERO_KMH - kmh) / (CR.V_ZERO_KMH - CR.V_FULL_KMH))
  if reason == "ok" and vGate <= 0 then reason = "speed" end
  local armed = reason == "ok"

  -- ---- standstill HOLD (8HP neutral-idle control) ----
  -- pedal-independent: brake + pedal at a standstill (launch preload,
  -- brake-throttle guard) never sees creep torque
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

  -- ---- device inputs ----
  -- disarmed: the TARGET is kept, only the authority limit drops to 0, so
  -- the device fades the floor out (5/s, on its latched last limit) instead
  -- of pulling the setpoint down; creepCut = 1 drops it at once
  if armed then st.tgt = st.hold and CR.HOLD_RPM or idle end
  local lim = armed and min(CR.MAX_NM, ctx.motorCapNm or 0) * vGate or 0
  ctx.creepIdleRpm = armed and idle or 0          -- recuperation speed fade (same frame)

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
