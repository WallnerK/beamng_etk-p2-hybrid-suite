-- etk854hybridControl.lua - ETK 854 ActiveHybrid master VCU (controller "hybridControl")
--
-- P2 hybrid supervisor. Owns the mode axes Driving Experience (COMFORT /
-- ECO PRO / SPORT) x EV mode (AUTO EV / MAX EV / BATTERY CONTROL), the pedal
-- map, the ICE on/off policy, E-boost stages + system torque cap, recuperation
-- as a deceleration target (lift-off, blended braking, one-pedal hold, lock-up
-- assist), BATTERY CONTROL charging and the etk854hybrid_* electrics contract
-- (HUD, cluster, 8HP, QA).
--
-- Submodules (controller/etk854hybrid/*, safeRequire + _stub), order: see updateGFX
--   batteryMonitor             SOC bands, EV lockout, cold derate
--   powerCurves                ICE / motor / regen envelopes
--   sailing                    DRIVE -> SAIL_ENTER -> SAIL -> SAIL_EXIT FSM
--   etk854hybrid_sportIndex    driver sportiness index 0..1
--   etk854hybrid_iceSequencer  ICE request arbiter + start/stop FSM; sole owner
--                              of K0, ignition and starter
--   etk854hybrid_torqueCoord   jerk-limited drive torque, ICE/motor split, fill
--   etk854hybrid_launchControl launch control + brake-throttle guard
--   etk854hybrid_creep         EV creep / virtual idle (ctx.creepIdleRpm / creepAvail)
-- Without sequencer or coordinator ctx.seqFailsafe drives ICE on, K0 closed.
--
-- Trailer stability (etk854hybrid_trailerStab) is read as requests only:
-- ctx.tsaTorqueMode / tsaTorqueRec / tsaFreeze, brake = max(driver, request), no lift-off
-- regen and no charge load while it acts (failsafe path: no torque modes).

local M = {}

local floor, abs, max, min, exp = math.floor, math.abs, math.max, math.min, math.exp
local RPM_TO_AV, AV_TO_RPM = 0.104719755, 9.549296596

-- ===========================================================
-- Master Context (shared with submodules)
-- ===========================================================
local ctx = {
  jbeamData = nil,
  debugLevel = 1,

  deMode = "COMFORT",
  deIdx = 1,
  edriveMode = "AUTO_EDRIVE",
  edriveIdx = 1,
  -- legacy 5-mode view (HUD / QA / input actions)
  legacyMode = "AUTO",
  legacyIdx = 1,
  legacyIceForce = false,     -- legacy "HYBRID" preset only

  -- DSE drive-mode key ("dsc"/"dtc"/"off"); dscPending retries the coupling
  -- until the DSE controllers exist
  dscKey = "dsc",
  dscPending = false,

  hudTimer = 0,
  -- arbitrated ICE request (iceSequencer.arbitrate), the policy's hysteresis state
  iceReq = true,

  -- index into regenLevelOrder; 3 = MED: felt lift-off regen without HIGH's
  -- one-pedal stop. OFF is the sailing position.
  regenLevelIdx = 3,
  regenLevelOrder = { "OFF", "LOW", "MED", "HIGH" },
  regenAllowsSail = false,

  -- written by etk854hybrid_launchControl every frame; launchArmed/Ready/Active:
  -- iceSequencer forces the ICE on; launchHold / launchGuardNm: torqueCoord
  launchArmed = false,
  launchHold = false,

  -- trailer stability requests (from electrics every frame): tsaTorqueMode
  -- 0 none / 1 HOLD / 2 ZERO wheel torque; tsaTorqueRec = HOLD cap recovery
  -- rate [Nm/s] (0 = none); tsaFreeze = no ICE start/stop, sail entry, regen
  -- or charge load, gear hold
  tsaTorqueMode = 0,
  tsaTorqueRec = 0,
  tsaFreeze = false,

  -- jbeam tuning variables
  boostStage = 0,
  boostStageOrigin = "tuning",   -- "part" (stage part pins it) | "tuning" ($ebooststage)
  regenStrength = 1.0,
  sailingEnabled = 1,
  -- above the 80 % spawn SOC, so the mode visibly charges a fresh car
  batteryTargetSoc = 0.90,

  modules = {},
}

ctx.dbg = function(tier, tag, msg)
  if tier <= ctx.debugLevel then
    log("I", "etk854hybrid." .. tag, msg)
  end
end

-- ===========================================================
-- Mode axis 1: Driving Experience Control
-- ===========================================================
local DE_ORDER = { "COMFORT", "ECO_PRO", "SPORT" }

-- Pedal map exponent, shaped = pedal ^ gamma: > 1 degressive (ECO PRO),
-- 1 linear, < 1 progressive (SPORT: early bite).
local PEDAL_GAMMA_ECO_PRO = 1.40
local PEDAL_GAMMA_COMFORT = 1.00
local PEDAL_GAMMA_SPORT   = 0.70

-- DE_CFG keys:
--   evCapScale   factor on the E-machine cap (1.0: a weaker motor in ECO
--                would start the ICE earlier than COMFORT)
--   coastScale   factor on the lift-off recuperation
--   iceAlwaysOn  pins the ICE on (policy rule 3)
--   dscMode      DSE calibration "dsc" | "dtc" (more slip before it bites);
--                a driver DSC OFF outranks it
-- AUTO EV only (EDRIVE_CFG.deTuned):
--   pEv/gEv/gHi  split pedal map (mapPedal); pEv set by $evPedalZoneComfort/Eco
--   kickAuto     kick start pedal;  evTopAuto  EV speed ceiling [km/h]
--   plateau/kickGap  pJoin = pEv + plateau (EV MAX plateau end, never starts
--                the ICE), kickJoin = pJoin + kickGap
--   urgentPedal/urgentDwellS  urgent start only after the dwell, a short stab does
--                nothing (also the 8HP kickdown dwell)
--   capTop       1 = band top is min(ceiling, system torque cap)
--   kneeLowFrac  low-SOC knee anchor, fraction of the ceiling
--   legacyRef    publish etk854hybrid_pedalLegacyEq (8HP ICE load axis)
--   dmdWspan     demandComp fade-in span above pJoin
--   siU1/siWRate  sport index demand band end / rate weight (split map: band from pJoin)
--   pEvMin/pEvMax  slider range of pEv (else PEV_MIN/MAX)
--   SPORT dmdP0/dmdWspan: demandComp pedal gate (no split map)
-- ECO PRO defaults: EV zone 0..0.80, plateau to 0.85, kick 0.90, urgent 0.95 after 0.12 s.
local DE_CFG = {
  COMFORT = { pedalGamma = PEDAL_GAMMA_COMFORT, evCapScale = 1.00, coastScale = 1.00,
              iceAlwaysOn = false, dscMode = "dsc",
              pEv = 0.45, gEv = 0.90, gHi = 1.00, kickAuto = 0.70, evTopAuto = 110 },
  ECO_PRO = { pedalGamma = PEDAL_GAMMA_ECO_PRO, evCapScale = 1.00, coastScale = 0.90,
              iceAlwaysOn = false, dscMode = "dsc",
              pEv = 0.80, gEv = 1.20, gHi = 1.00, kickAuto = 0.90, evTopAuto = 120,
              -- pJoin / kickJoin: derived
              plateau = 0.05, kickGap = 0.05, pJoin = 0.85, kickJoin = 0.90,
              urgentPedal = 0.95, urgentDwellS = 0.12, capTop = 1, kneeLowFrac = 0.433,  -- 0.55^1.40
              legacyRef = 1, dmdWspan = 0.10, siU1 = 0.95, siWRate = 0,
              pEvMin = 0.40, pEvMax = 0.85 },
  SPORT   = { pedalGamma = PEDAL_GAMMA_SPORT,   evCapScale = 1.00, coastScale = 1.20,
              iceAlwaysOn = true,  dscMode = "dtc",
              kickAuto = 0.60, evTopAuto = 110,
              dmdP0 = 0.10, dmdWspan = 0.10 },
}
-- default slider range of the EV pedal zone
local PEV_MIN, PEV_MAX = 0.30, 0.65

-- pJoin = pEv + plateau (max 0.92), kickJoin = pJoin + kickGap (max urgent
-- pedal). Rounded to 1e-3: 0.80 + 0.05 + 0.05 = 0.9000000000000001 would miss
-- the 0.90 step. No plateau (COMFORT): pJoin stays nil, band starts at pEv.
local function deriveSplitThresholds(cfg)
  if not (cfg and cfg.plateau and cfg.pEv) then return end
  cfg.pJoin    = math.floor(math.min(cfg.pEv + cfg.plateau, 0.92) * 1000 + 0.5) / 1000
  cfg.kickJoin = math.floor(math.min(cfg.pJoin + (cfg.kickGap or 0.05), cfg.urgentPedal or 0.95) * 1000 + 0.5) / 1000
end

local DSC_LABEL = { dsc = "DSC", dtc = "DTC", off = "OFF" }

-- ===========================================================
-- Mode axis 2: EV mode strategy
-- ===========================================================
local EDRIVE_ORDER = { "AUTO_EDRIVE", "MAX_EDRIVE", "BATTERY_CONTROL" }
local EDRIVE_SHORT = { AUTO_EDRIVE = "AUTO", MAX_EDRIVE = "MAX", BATTERY_CONTROL = "BATT" }

-- demandStart: torque demand may start the ICE (not only the kick pedal).
-- evTopKmh 0 (BATTERY CONTROL): at the target SOC the engine carries the car.
-- deTuned: kick, EV ceiling and split map come from DE_CFG; evTopKmh /
-- iceKickdown are the fallback.
local EDRIVE_CFG = {
  AUTO_EDRIVE     = { evTopKmh = 110, iceKickdown = 0.60, demandStart = true,  chargeToTarget = false,
                      deTuned = true },
  -- evOnly: the ICE never starts; the low-SOC guard in updateGFX falls back to AUTO EV
  MAX_EDRIVE      = { evTopKmh = 9999, iceKickdown = 2.0, demandStart = false, chargeToTarget = false, evOnly = true },
  BATTERY_CONTROL = { evTopKmh = 0,   iceKickdown = 0.0,  demandStart = true,  chargeToTarget = true  },
}

-- Legacy one-dimensional presets (input actions, HUD, QA) -> (DE, EV mode)
local LEGACY_MODE_ORDER = { "AUTO", "HYBRID", "EV", "HOLD", "SPORT" }
local LEGACY_MODE_MAP = {
  AUTO   = { de = "COMFORT", edrive = "AUTO_EDRIVE" },
  -- AUTO EV with the ICE pinned on (QA ICE-power tests)
  HYBRID = { de = "COMFORT", edrive = "AUTO_EDRIVE", iceForce = true },
  EV     = { de = "COMFORT", edrive = "MAX_EDRIVE" },
  HOLD   = { de = "COMFORT", edrive = "BATTERY_CONTROL" },
  SPORT  = { de = "SPORT",   edrive = "AUTO_EDRIVE" },
}

-- ===========================================================
-- E-Boost stages ($ebooststage), stage 0 = stock
-- ===========================================================
-- battKw scales with the stage (13.5 kWh pack: ~7C at stage 0, ~17C at 3).
-- capBonusNm: offset on the system torque cap base, else the cap would eat
-- the extra motor torque at full load (680 base: 680 / 780 / 900 / 1050 Nm).
local BOOST_STAGES = {
  [0] = { torqueNm = 265, powerKw =  83, battKw =  95, regenKw = 55, capBonusNm =   0 },
  [1] = { torqueNm = 320, powerKw = 110, battKw = 125, regenKw = 65, capBonusNm = 100 },
  [2] = { torqueNm = 400, powerKw = 150, battKw = 170, regenKw = 80, capBonusNm = 220 },
  [3] = { torqueNm = 500, powerKw = 200, battKw = 225, regenKw = 95, capBonusNm = 370 },
}
local MOTOR_EFFICIENCY = 0.94   -- matches electricalEfficiency in the jbeam

-- ===========================================================
-- System torque cap
-- ===========================================================
-- A drive-feel limit (full load builds up like a production hybrid), not
-- driveline protection; torqueCoord trims the E-machine share first.
-- cap = base + capBonusNm; base: ECU part section etk854hybridSystemCap.baseNm,
-- else controller arg systemTorqueCapNm, else this. K0 (1150 Nm) and lock-up
-- (1520 Nm) are rated above every cap, so the VCU is the only limiter.
local SYSTEM_TORQUE_CAP_DEFAULT_NM = 680   -- [Nm]

-- ===========================================================
-- ICE engagement thresholds
-- ===========================================================
local SOC_ICE_FORCE          = 0.08   -- below this the ICE runs in every mode
-- MAX EV falls back at the same SOC: the forced ICE charges the pack, so a
-- lower threshold would never be reached.
local BATTERY_LOW_THRESHOLD  = SOC_ICE_FORCE
-- standstill: never idle the ICE at a red light
local AUTO_STANDSTILL_KMH    = 2.5
local AUTO_STANDSTILL_THR    = 0.04
-- with EV creep available (ctx.creepAvail) the same hard-off covers the creep
-- regime (creep 5-7 km/h, regen fade to ~10 km/h)
local AUTO_CREEP_KMH         = 8.0
-- creep hard-off only above this SOC (batteryMonitor LOW1): the ~2-3 kW creep
-- drain would cycle the ICE against the SOC force (rule 1) in crawling traffic
local AUTO_CREEP_MIN_SOC     = SOC_ICE_FORCE + 0.02

-- ICE policy rule 5, scaled by the sportiness index S. Start side on
-- Sst = S x DEF.sStart (ECO 0: S never lowers an ECO start), off side on S.
--   kickOn    = kick * (1 - KICK_ON_CUT * rS)
--   kickOff   = kick * (KICK_OFF_BASE - KICK_OFF_CUT * rS),  rS = clamp01((Sst - S_KICK0) / S_KICK_SPAN)
--   dSport    = load ratio with the K0 crank torque still free (DSPORT_MIN..MAX)
--   demandOn  = DEMAND_ON_BASE -> dSport (smoothstep over Sst)
--   demandOff = min(DEMAND_OFF_MAX, demandOn - DEMAND_OFF_GAP)
-- Demand start per DE (DEF), r = torqueReq / motor cap:
--   deficit   D += (r - 1) * dt while r > rMin, else D -= leak * dt (floor 0);
--             reset while on, at standstill and on a mode change
--   start     D >= Don = onCalm * (1 - smoothstep((Sst - DEF_S0) / DEF_SPAN)),
--             x DEF_TURNING_FRAC while the engine still turns (no crank)
--   immediate r >= rHard -> dSport
--   K0 lead   in the K0 window: D + (r - 1) * lead >= Don (pulls a start
--             forward so a step tip-in keeps its K0 start)
-- Urgent: pedal >= URGENT_PEDAL (AUTO EV: DE urgentPedal held urgentDwellS).
-- Fast: pedal rate >= RATE_URGENT with the pedal above fastPedal (split map:
-- also above kick), latched until it falls back, and r >= fastR. Short
-- debounce but the normal start profile (URGENT would harden every tip-in).
-- RATE_URGENT 0 = off: a keyboard ramps at 3/s and a gamepad steps.
-- Stop, split map: kept while pedal > pEv + EVZONE_KEEP_MARGIN, near the EV
-- ceiling or r > rOff; else released after offCalm..offSport (rising with S),
-- at once on a lift / brake. Otherwise the kickOff / demandOff keep rules.
local ICEPOL = {
  S_KICK0 = 0.25, S_KICK_SPAN = 0.50,
  KICK_ON_CUT = 0.35, KICK_OFF_BASE = 0.55, KICK_OFF_CUT = 0.25,
  DEMAND_ON_BASE = 1.35, DSPORT_MIN = 0.60, DSPORT_MAX = 1.00, DSPORT_RESERVE_NM = 30,
  S_DEMAND0 = 0.30, S_DEMAND_SPAN = 0.40, DEMAND_OFF_MAX = 0.80, DEMAND_OFF_GAP = 0.15,
  -- keep band below the EV ceiling [km/h]; narrow, so a cruise just below it does not hold the ICE
  SPEED_HYST_KMH = 5.0,
  URGENT_PEDAL = 0.85,
  RATE_URGENT = 0,   -- [1/s], controller arg iceRateUrgent (e.g. 2.5 for a pedal / wheel)
  DEF_S0 = 0.15, DEF_SPAN = 0.45, DEF_TURNING_FRAC = 0.3,
  -- DEF: fastPedal/fastR fast rule; rOff/offCalm/offSport (nil = OFF_SPORT_S)
  -- split-map off side; sStart factor on S (nil = 1)
  EVZONE_KEEP_MARGIN = 0.03, OFF_SPORT_S = 4.0, LIFT_PEDAL = 0.02, LIFT_BRAKE = 0.05,
  DEF = {
    COMFORT = { rMin = 1.05, onCalm = 0.45, rHard = 1.8, leak = 0.5, lead = 0.75,
                fastPedal = 0.60, fastR = 1.2, rOff = 0.95, offCalm = 0.6 },
    -- with sStart 0 rHard is not reached below the urgent pedal
    ECO_PRO = { rMin = 1.10, onCalm = 0.75, rHard = 2.4, leak = 0.8, lead = 0.83,
                fastPedal = 0.90, fastR = 1.5, rOff = 0.95, offCalm = 0.5,
                offSport = 1.5, sStart = 0 },
  },
}

-- Pedal map state + constants (one table = one upvalue)
--   KNEE_TAU_S     knee LPF, same as the torqueCoord request ceiling
--   HEALTHY0/SPAN  SOC-side motor health below which the knee slides onto the
--                  low-SOC anchor (a low pack may start the ICE earlier, a cold one not)
local _pm = { kneeF = nil, KNEE_TAU_S = 0.25, HEALTHY0 = 0.5, HEALTHY_SPAN = 0.5,
  -- demandComp: reference rpm per fuel / DE, max factor, fade-in span, share
  -- of the 8HP deliverable torque, factor slew [1/s]. SPORT references offset
  -- SPORT's early light-pedal upshifts.
  DMD_ON = true,
  DMD_NREF = { diesel = { COMFORT = 3000, ECO_PRO = 2500, SPORT = 2600 },
               petrol = { COMFORT = 3600, ECO_PRO = 3000, SPORT = 2950 } },
  DMD_KMAX = 1.8, DMD_WSPAN = 0.15, DMD_CAPF = 0.90, DMD_RATE = 3.0,
  dmdK = 1,
  -- reference ECO map for etk854hybrid_pedalLegacyEq (8HP ICE load axis in ECO)
  LEG = { pEv = 0.55, gEv = 1.10, gHi = 1.30 },
  top = 0 }   -- band top of the last frame

-- ===========================================================
-- Recuperation  (driver selectable: OFF / LOW / MED / HIGH)
-- ===========================================================
-- Specified as a wheel deceleration [m/s^2], converted with the mechanics:
--     motorNm = decel * mass * wheelRadius / (mechRatio * DRIVE_EFF)
--  1. mechRatio = GEARBOX cumulativeGearRatio: the motor's own includes the
--     converter stall ratio, a coasting converter multiplies by 1.0.
--  2. An open converter (below gear 3) passes only ~60-80 Nm of coast torque
--     (kFactor * av^2): LOCKUP_* closes the lock-up while recuperating.
--  3. The regen curve fades at low rpm: ASSIST_* blends in the missing
--     friction brake and latches a standstill hold (one-pedal).
-- Brake pedal = blended braking: regen takes the share it can deliver, the
-- friction command drops by the same amount.
local RGN = {
  -- liftDecel on full lift-off [m/s^2]: OFF free-wheeling (allows sailing),
  -- LOW ~ petrol engine braking, MED ~ hybrid "B", HIGH one-pedal (~0.22 g) to a hold
  LEVEL = {
    OFF  = { liftDecel = 0.00, onePedal = false, allowSail = true  },
    LOW  = { liftDecel = 0.50, onePedal = false, allowSail = false },
    MED  = { liftDecel = 1.20, onePedal = false, allowSail = false },
    HIGH = { liftDecel = 2.20, onePedal = true,  allowSail = false },
  },
  DRIVE_EFF   = 0.92,   -- driveline efficiency motor -> road
  FALLBACK_NM = 180,    -- regen envelope without powerCurves
  -- lift-off gate: full below THR_FULL, zero from THR_OFF. Narrow because
  -- active regen zeroes the drive torque: a wide band would be a dead zone.
  THR_FULL    = 0.02,
  THR_OFF     = 0.05,
  -- brake-pedal share the E-machine may take; tapered out towards a panic stop
  BRAKE_SHARE_MAX      = 1.00,
  BRAKE_SHARE_FULL_END = 0.35,
  BRAKE_SHARE_ZERO_END = 0.80,
  -- speed fade (nothing left below MIN_KMH)
  MIN_KMH     = 0.8,
  TAPER_KMH   = 4.0,
  -- EV creep armed: fade on the gear-1 turbine rpm / virtual idle instead
  -- (~6-7 -> 9-10 km/h), so regen hands over to the creep floor without stopping the car
  CREEP_FADE_LO = 1.10,
  CREEP_FADE_HI = 1.60,
  -- converter lock-up assist
  LOCKUP_MIN_KMH      = 6.0,    -- ICE coupled (stall protection)
  LOCKUP_MIN_EV_KMH   = 1.5,    -- K0 open: no stall risk, one-pedal needs it down to walking pace
  LOCKUP_RATE_UP      = 3.0,
  LOCKUP_RATE_DN      = 6.0,
  LOCKUP_ICE_RPM_FRAC = 1.25,   -- K0 closed: lock only above idle * this
  -- one-pedal friction blend + standstill hold
  ASSIST_RATE_UP   = 2.0,
  ASSIST_RATE_DN   = 4.0,
  ASSIST_MAX       = 0.45,      -- brake coefficient ceiling
  HOLD_KMH         = 1.5,
  HOLD_BRAKE       = 0.30,
  HOLD_RELEASE_THR = 0.05,      -- throttle that releases the hold
  -- output rate limits [Nm/s at the motor], not a fraction: the envelope moves
  -- with shaft speed and gear. 600 Nm/s stays inside the 100 Nm / 100 ms step limit.
  ENGAGE_NM_S  = 600,
  RELEASE_NM_S = 900,
  ACTIVE_THR  = 0.05,           -- above this the drive throttle is zeroed
  -- lift-off regen only once the drive torque is below this (one zero crossing)
  XOVER_NM    = 15,
  -- ABS interlock: regen out when ABS acts with rear over-slip > ABS_REAR_SLIP,
  -- in a hard stop, or with any wheel > ABS_WHEEL_SLIP against virtualAirspeed
  -- (front lock / AWD); held ABS_HOLD_S across ABS pressure cycles
  ABS_REAR_SLIP  = 0.03,
  ABS_HARD_BRAKE = 0.60,
  ABS_WHEEL_SLIP = 0.08,
  ABS_HOLD_S     = 0.30,
  -- rear slip limiter: regen scaled to 0 between REGEN_SLIP_START and ABS_REAR_SLIP
  REGEN_SLIP_START = 0.012,
  SLIP_TAU_S       = 0.05,  -- over-slip LPF (wheel speeds jitter ~1 %)
  MEAS_TAU_S       = 0.05,  -- measured converter torque LPF (blend credit)
  RATIO_MIN      = 0.5,
  RATIO_MAX      = 40.0,
  RATIO_FALLBACK = 4.0,         -- ~ gear 5 x 3.15
  FINAL_DRIVE    = 3.15,        -- ratio reconstruction from the raw gear ratio
}

-- Recuperation state. One table: updateGFX is close to LuaJIT's 60-upvalue limit.
local _rgn = {
  levelName   = "MED",
  lockupCmd   = 0,      -- own lock-up request 0..1
  assistCmd   = 0,      -- own friction brake request 0..1
  holdLatched = false,  -- one-pedal standstill hold
  targetDecel = 0,      -- [m/s^2]
  motorNm     = 0,      -- regen torque at the motor shaft
  ratio       = 4.0,
  mass        = 1900,   -- [kg]
  wheelR      = 0.33,   -- [m]
  brakeNm     = 0,      -- total friction brake torque at brake = 1
  refsOk      = false,
  refsTries   = 0,
  refsRetryT  = 0,
  gearbox     = nil,
  whl         = nil,    -- wheels by name (FL/FR/RL/RR)
  osF         = 0,      -- filtered rear over-slip
  absHoldT    = 0,      -- [s] qualified-ABS hold
  tc          = nil,    -- torque converter device
  tcF         = 0,      -- filtered converter braking torque [Nm]
}

-- ===========================================================
-- HUD / filtering
-- ===========================================================
local HUDC = {
  INTERVAL      = 0.05,   -- 20 Hz
  -- display smoothing [s]; ICE slower to mask driveline torsional ripple
  ICE_POWER_TAU = 0.35,
  EV_POWER_TAU  = 0.12,
  DBG_INTERVAL  = 1.0,    -- periodic diagnostics max 1 Hz
}

-- ===========================================================
-- BATTERY CONTROL charging
-- ===========================================================
local CHARGE_SOC_HYST     = 0.03   -- restart charging this far under the target
local CHG = {
  GAIN            = 0.30,   -- P-gain on ICE torque headroom (~10 kW at highway cruise)
  MAX_KW          = 25,     -- charge power cap
  PARK_RPM        = 2200,   -- Park-Charge rpm setpoint
  -- [Nm], not a regen fraction: the envelope differs per stage, the rpm loop is tuned for this
  PARK_TORQUE_NM  = 80,
  PARK_RPM_KP     = 0.0003,
  PARK_RPM_KI     = 0.00008,
  PARK_RPM_MAX_THR = 0.45,
  PARK_ANTI_WINDUP = 0.40,
  -- ramp time constants [s] (= per-frame 0.18 / 0.15 / 0.90 / 0.85 at 60 fps)
  CRUISE_RISE_TAU_S  = 0.084,
  CRUISE_FALL_TAU_S  = 0.103,
  PARK_I_DECAY_TAU_S = 0.158,
  OFF_DECAY_TAU_S    = 0.103,
}

-- ===========================================================
-- BATTERY CONTROL: maximum-rate charging in NEUTRAL at standstill
-- ===========================================================
-- N at standstill, SOC below target: the engine lifts to a charge load point
-- and the E-machine recuperates at the hardware limit (Park: 80 Nm / 2200 rpm).
-- K0 stays closed in N, so the regen torque brakes the ICE directly: the rpm
-- setpoint sits in the torque plateau with reserve (ICE_LOAD_FRAC), at the
-- speed where full regen torque reaches the stage power budget.
local NCHG = {
  MAX_KMH        = 2.0,    -- "standing"
  ABORT_THR      = 0.05,   -- pedal that aborts
  RPM_MIN        = 2200,   -- never below the Park-Charge point
  RPM_MAX        = 3000,
  RPM_MAXFRAC    = 0.60,   -- of the rev range
  ICE_LOAD_FRAC  = 0.85,   -- share of the ICE curve regen may use
  RPM_RAMP       = 900,    -- setpoint ramp [rpm/s]
  REGEN_RISE     = 0.35,   -- regen fraction per second, engaging
  REGEN_FALL     = 1.20,   -- ... releasing (abort must be quick)
  MAX_THR        = 0.75,   -- rpm PI ceiling
  -- PI only trims around the feed forward (load torque / ICE capacity)
  RPM_KP         = 0.0006,
  RPM_KI         = 0.0002,
  ANTI_WINDUP    = 0.60,
  -- load derate for the MAX_THR ceiling (pumping / friction margin)
  THR_MARGIN     = 0.85,
  STALL_FRAC     = 0.90,   -- below idle * this: dump the load
  STALL_RELEASE  = 4.0,    -- multiplier on REGEN_FALL while sagging
  -- load ceiling: more buys no charge power, and once the shaft is dragged
  -- through 0 rpm the motor (abs(rpm) curve) keeps braking - no way back
  MAX_NM         = 180,
  ENGAGE_FRAC    = 1.05,   -- rpm > idle * this before any load
  RECOVER_RPM    = 100,    -- below this (incl. backwards) -> restart with the starter
}

-- ===========================================================
-- Module-local state
-- ===========================================================
local _simTime = 0
local _icePowerEma = 0
local _evPowerEma = 0
local _chargeRegenFrac = 0
local _parkChargeRpmIntegral = 0
local _dbgLastLogT = 0
local _regenSmoother = nil
local _iceDevice, _motorDevice = nil, nil
local _nChargeRpmSetpoint = 0    -- ramped rpm target, N charge
local _nChargeEngaged = false    -- N charge load latch
local _lcWheelsFn = nil          -- launch DSC hold assist (physics rate)

-- ===========================================================
-- Helpers
-- ===========================================================
local U = require("controller/etk854hybrid/etk854hybrid_util")
local clamp01 = U.clamp01 -- a NaN passes through
local clamp = U.clamp

-- not U.smoothstep: that one maps NaN to 0, this one passes it through
local function smoothstep(t)
  t = clamp01(t)
  return t * t * (3 - 2 * t)
end

-- electrics.lua rewrites boolean channels as 1/0 early in the frame: flags
-- written before that (absActive) read as numbers, later ones (isShifting,
-- escActive) as booleans.
local flag = U.flag

-- an undefined $variable (or a typo) leaves the literal "$name": use the default
local function readNumber(data, key, default)
  if data == nil then return default end
  local v = data[key]
  if type(v) == "number" then return v end
  if type(v) == "string" then
    local n = tonumber(v)
    if n then return n end
  end
  return default
end

local function safeRequire(modPath, label)
  local ok, mod = pcall(require, modPath)
  if ok and type(mod) == "table" then
    ctx.dbg(1, "boot", "loaded " .. (label or modPath))
    return mod
  end
  log("E", "etk854hybrid.boot", "safeRequire FAILED: " .. modPath .. " -> " .. tostring(mod))
  local okStub, stub = pcall(require, "controller/etk854hybrid/_stub")
  if okStub then return stub end
  return {
    init = function() end,
    updateGFX = function() end,
    updateFixedStep = function() end,
    reset = function() end,
  }
end

-- Drive direction: +1 forward, -1 reverse, 0 in P or N (both read gearIndex 0).
-- The gear string is only a fallback for when gearIndex is not a number.
local function getGearSign()
  local idx = electrics.values.gearIndex
  local gearStr = electrics.values.gear
  if type(idx) == "number" then
    if idx > 0 then return 1 end
    if idx < 0 then return -1 end
    return 0
  end
  if gearStr == "R" then return -1 end
  if gearStr == "N" or gearStr == "P" then return 0 end
  return 1
end

-- Mechanical ratio motor -> wheels without the converter: the gearbox's
-- cumulativeGearRatio. The motor's own carries the converter stallTorqueRatio,
-- but a coasting converter multiplies by 1.0 (avRatio >= couplingAVRatio).
local function mechRatioToWheels()
  local gb = _rgn.gearbox or powertrain.getDevice("gearbox")
  _rgn.gearbox = gb
  if gb then
    local r = gb.cumulativeGearRatio
    if type(r) == "number" then
      r = abs(r)
      if r >= RGN.RATIO_MIN and r <= RGN.RATIO_MAX then return r end
    end
    -- during a shift the cumulative value can be 0 or stale
    if type(gb.gearRatio) == "number" and abs(gb.gearRatio) > 0.05 then
      return clamp(abs(gb.gearRatio) * RGN.FINAL_DRIVE, RGN.RATIO_MIN, RGN.RATIO_MAX)
    end
  end
  return RGN.RATIO_FALLBACK
end

-- Rear over-slip (front - rear) / front for the ABS interlock: the E-machine
-- brakes the rear axle only, and absActive also rises at brake onset without
-- any slip. Unknown wheel layout -> 1 (trust the flag).
local function rearOverSlip()
  local w = _rgn.whl
  if not w then
    w = {}
    if wheels and wheels.wheels then
      for i = 0, (wheels.wheelCount or 0) - 1 do
        local wd = wheels.wheels[i]
        if wd and wd.name then w[wd.name] = wd end
      end
    end
    _rgn.whl = w
  end
  local fl, fr, rl, rr = w.FL, w.FR, w.RL, w.RR
  if not (fl and fr and rl and rr) then return 1 end
  local vf = 0.5 * (abs(fl.wheelSpeed or 0) + abs(fr.wheelSpeed or 0))
  if vf < 2 then return 1 end
  local vr = 0.5 * (abs(rl.wheelSpeed or 0) + abs(rr.wheelSpeed or 0))
  return (vf - vr) / vf
end

-- Largest wheel braking slip against virtualAirspeed (0 below 5 m/s, like the ABS)
local function maxWheelSlip()
  local vRef = tonumber(electrics.values.virtualAirspeed) or 0
  if vRef < 5 or not (wheels and wheels.wheels) then return 0 end
  local worst = 0
  for i = 0, (wheels.wheelCount or 0) - 1 do
    local wd = wheels.wheels[i]
    if wd then
      local sl = (vRef - abs(wd.wheelSpeed or vRef)) / vRef
      if sl > worst then worst = sl end
    end
  end
  return worst
end

-- Mass (node weights), wheel radius and total brake torque for the decel model.
-- Guarded: during a rebuild wheels / nodes can be empty, and zeros would
-- disable the one-pedal blend for the session.
local function readVehicleRefs()
  local mass = 0
  if v and v.data and v.data.nodes then
    for _, n in pairs(v.data.nodes) do mass = mass + (n.nodeWeight or 0) end
  end
  if mass > 300 then _rgn.mass = mass end

  local rSum, rCount, brakeNm = 0, 0, 0
  if wheels and wheels.wheels then
    for _, wd in pairs(wheels.wheels) do
      local r = wd.radius or 0
      if r > 0.05 then
        rSum = rSum + r
        rCount = rCount + 1
      end
      local split = wd.brakeInputSplit or 1
      brakeNm = brakeNm + (wd.brakeTorque or 0)
        * (split + (1 - split) * (wd.brakeSplitCoef or 1))
    end
  end
  if rCount > 0 then _rgn.wheelR = rSum / rCount end
  if brakeNm > 100 then _rgn.brakeNm = brakeNm end
  -- sticky once plausible
  _rgn.refsOk = _rgn.refsOk or ((_rgn.brakeNm > 100) and (_rgn.mass > 300))
  log("I", "etk854hybrid.regen", string.format(
    "vehicle refs: mass=%.0fkg wheelR=%.3fm brakeTq=%.0fNm ok=%s",
    _rgn.mass, _rgn.wheelR, _rgn.brakeNm, tostring(_rgn.refsOk)))
end

-- Envelope provider if powerCurves fails to load (the stub has no accessors);
-- petrol-i4 shaped, keeps the torque pipeline arithmetic-safe.
local CURVES_FALLBACK = {
  iceMaxTorqueAtRpm = function(rpm)
    local r = abs(rpm or 0)
    if r < 400 then return 0 end
    if r < 1500 then return 350 * (0.35 + 0.65 * (r - 400) / 1100) end
    if r < 5000 then return 350 end
    if r < 7000 then return 350 * (1 - 0.45 * (r - 5000) / 2000) end
    return 190
  end,
  motorMaxTorqueAtRpm = function(rpm)
    local r = abs(rpm or 0)
    if r <= 3800 then return 500 end
    return max(190, 500 * 3800 / r)
  end,
  motorRegenTorqueAtRpm = function(rpm)
    return RGN.FALLBACK_NM * min(1, abs(rpm or 0) / 600)
  end,
  iceIdleRPM = function() return 800 end,
  iceMaxRPM  = function() return 7000 end,
}

local function curves()
  return ctx.modules.curves
end

-- ===========================================================
-- Envelopes
-- ===========================================================
local function stageCfg()
  return BOOST_STAGES[ctx.boostStage] or BOOST_STAGES[0]
end

-- Reverse speed limiter (the E-machine would reach >110 km/h in R): drive
-- authority (motor cap and ICE throttle) 1 -> 0 between startKmh and fullKmh.
local REV_LIMIT = { startKmh = 30, fullKmh = 36 }
local function reverseLimitFrac()
  if electrics.values.gear ~= "R" then return 1 end
  local kmh = abs(electrics.values.wheelspeed or 0) * 3.6
  if kmh <= REV_LIMIT.startKmh then return 1 end
  if kmh >= REV_LIMIT.fullKmh then return 0 end
  return 1 - (kmh - REV_LIMIT.startKmh) / (REV_LIMIT.fullKmh - REV_LIMIT.startKmh)
end

-- Returns capNm (allowed now) and curveNm (jbeam envelope at this rpm). The
-- device scales its full curve by evThrottle, so the cap becomes a throttle
-- ceiling capNm / curveNm.
local function motorCapAtRpm(rpm, deCfgNow)
  local s = stageCfg()
  local c = curves()
  local curveNm = c and c.motorMaxTorqueAtRpm(rpm) or s.torqueNm
  if curveNm < 1 then curveNm = 1 end

  local capNm = min(curveNm, s.torqueNm)
  local av = abs(rpm or 0) * RPM_TO_AV
  if av > 1.0 then
    capNm = min(capNm, s.powerKw * 1000 / av)
    -- battery discharge budget after motor losses
    capNm = min(capNm, s.battKw * 1000 * MOTOR_EFFICIENCY / av)
  end

  capNm = capNm * (ctx.evPowerFraction or 1.0) * (deCfgNow.evCapScale or 1.0)
  ctx.reverseLimit = reverseLimitFrac()
  capNm = capNm * ctx.reverseLimit
  if ctx.evLockoutActive then capNm = 0 end
  if capNm < 0 then capNm = 0 end
  return capNm, curveNm
end

local function iceCapAtRpm(rpm)
  local c = curves()
  if c then return c.iceMaxTorqueAtRpm(rpm) end
  return 350
end

-- ===========================================================
-- ICE on/off policy
-- ===========================================================
-- The policy only decides whether the ICE is wanted; ignition, starter and K0
-- belong to etk854hybrid_iceSequencer.

-- verdict + reason (etk854hybrid_icePolicyReason)
local function verdict(reason, want)
  ctx.icePolicyReason = reason
  return want
end

-- Load integral (see ICEPOL), returns D [s]; built only while the request is off.
local function updateDeficit(dt, r, def, speedKmh, currentOn)
  local key = (ctx.deIdx or 1) * 10 + (ctx.edriveIdx or 1)   -- DE x EV mode
  -- a mode change also drops the fast-pedal latch
  if ctx.iceDefKey ~= key then ctx.iceFastLatch = false end
  if currentOn or speedKmh < AUTO_STANDSTILL_KMH or ctx.iceDefKey ~= key then
    ctx.iceDefKey = key
    ctx.iceDef = 0
    return 0
  end
  local D = ctx.iceDef or 0
  if r > def.rMin then
    D = D + (r - 1) * dt
  else
    D = max(0, D - def.leak * dt)
  end
  ctx.iceDef = D
  return D
end

-- Mode policy without sailing. currentOn = the arbitrated request (ctx.iceReq).
-- Outputs for the arbiter: ctx.iceForced (forcing rule), iceHardOff (MAX EV /
-- standstill / creep: release at once), iceStartUrgent (URGENT profile),
-- iceStartFast (short debounce), iceLightLoad (split map, pedal in the EV zone,
-- r < rOff: an off counts as a lift). Diagnostics: icePolicyReason, iceDef,
-- iceDefOn, policySportS.
local function policyWantsIce(deCfgNow, edriveCfgNow, currentOn, pedal, speedKmh, torqueReqNm, motorCapNm, soc, dt)
  ctx.iceForced, ctx.iceHardOff, ctx.iceStartUrgent, ctx.iceStartFast = false, false, false, false
  ctx.iceLightLoad = false
  -- urgent dwell timed before the early returns, so it survives a forced verdict
  local urgP = (edriveCfgNow.deTuned and deCfgNow.urgentPedal) or ICEPOL.URGENT_PEDAL
  ctx.iceUrgentT = (pedal >= urgP) and ((ctx.iceUrgentT or 0) + (dt or 0)) or 0
  local S = ctx.sportIndex or 0
  local deTuned = edriveCfgNow.deTuned == true
  local def = ICEPOL.DEF[ctx.deMode] or ICEPOL.DEF.COMFORT
  local splitMap = deCfgNow.pEv ~= nil and deTuned
  local Sst = (splitMap and def.sStart) and S * def.sStart or S
  ctx.policySportS = Sst
  -- 1. battery emergency
  if soc < SOC_ICE_FORCE then
    ctx.iceForced, ctx.iceStartUrgent, ctx.iceFastLatch = true, true, false
    return verdict("soc", true)
  end

  -- 2. BATTERY CONTROL below target (also at standstill: Park-Charge)
  if edriveCfgNow.chargeToTarget then
    if soc < (ctx.batteryTargetSoc - (currentOn and 0 or CHARGE_SOC_HYST)) then
      ctx.iceForced, ctx.iceFastLatch = true, false
      return verdict("charge", true)
    end
  end

  -- 3. SPORT / legacy HYBRID pin the engine on
  if deCfgNow.iceAlwaysOn or ctx.legacyIceForce then
    ctx.iceForced, ctx.iceFastLatch = true, false
    return verdict("pinned", true)
  end

  -- 3b. MAX EV: never (SPORT above still overrides)
  if edriveCfgNow.evOnly then
    ctx.iceHardOff, ctx.iceFastLatch = true, false
    return verdict("evOnly", false)
  end

  -- 4. standstill / EV-creep regime without pedal: off. ctx.creepAvail is
  --    from the previous frame (the creep module runs later).
  if pedal < AUTO_STANDSTILL_THR then
    local still = speedKmh < AUTO_STANDSTILL_KMH
    if still or (ctx.creepAvail and speedKmh < AUTO_CREEP_KMH and soc >= AUTO_CREEP_MIN_SOC) then
      ctx.iceHardOff = true
      ctx.iceDef, ctx.iceFastLatch = 0, false
      return verdict(still and "standstill" or "creep", false)
    end
  end

  -- 5. hysteresis band and load integral (see ICEPOL)
  local kick = (deTuned and (deCfgNow.kickJoin or deCfgNow.kickAuto)) or edriveCfgNow.iceKickdown
  local top  = (deTuned and deCfgNow.evTopAuto) or edriveCfgNow.evTopKmh
  local rS = clamp01((Sst - ICEPOL.S_KICK0) / ICEPOL.S_KICK_SPAN)
  local kickOn  = kick * (1 - ICEPOL.KICK_ON_CUT * rS)
  local kickOff = kick * (ICEPOL.KICK_OFF_BASE - ICEPOL.KICK_OFF_CUT * rS)
  local dSport = clamp((motorCapNm - (ctx.iceCrankNmEst or 120) - ICEPOL.DSPORT_RESERVE_NM)
                       / max(motorCapNm, 1), ICEPOL.DSPORT_MIN, ICEPOL.DSPORT_MAX)
  local sDem = smoothstep((Sst - ICEPOL.S_DEMAND0) / ICEPOL.S_DEMAND_SPAN)
  local demandOn = ICEPOL.DEMAND_ON_BASE - (ICEPOL.DEMAND_ON_BASE - dSport) * sDem
  local demandOff = min(ICEPOL.DEMAND_OFF_MAX, demandOn - ICEPOL.DEMAND_OFF_GAP)
  -- kick > 0: BATTERY CONTROL (kick 0) would call every pedal urgent
  ctx.iceStartUrgent = (kick > 0) and pedal >= urgP
    and ctx.iceUrgentT >= ((deTuned and deCfgNow.urgentDwellS) or 0)
  local r = torqueReqNm / max(motorCapNm, 1)
  -- before the latch: a mode change clears it
  local D = updateDeficit(dt or 0, r, def, speedKmh, currentOn)
  if kick <= 0 or ICEPOL.RATE_URGENT <= 0
     or pedal <= max(def.fastPedal, splitMap and (deCfgNow.kickJoin or 0) or 0) then
    ctx.iceFastLatch = false
  elseif (ctx.pedalRate or 0) >= ICEPOL.RATE_URGENT then
    ctx.iceFastLatch = true
  end
  ctx.iceStartFast = ctx.iceFastLatch == true and r >= def.fastR
  ctx.iceLightLoad = splitMap and pedal <= deCfgNow.pEv and r < def.rOff
  local don = def.onCalm * (1 - smoothstep((Sst - ICEPOL.DEF_S0) / ICEPOL.DEF_SPAN))
  -- engine still turning: re-engaging costs no crank
  if (ctx.iceEngageState or "OFF") ~= "OFF" then don = don * ICEPOL.DEF_TURNING_FRAC end
  ctx.iceDefOn = don
  if currentOn then
    if splitMap then
      -- split-map off side (kickOff / demandOff would sit inside the EV zone
      -- and never let the engine go)
      if pedal >= max(kickOff, deCfgNow.pEv + ICEPOL.EVZONE_KEEP_MARGIN) then
        ctx.iceSurplusT = 0
        return verdict("keep:pedal", true)
      end
      if speedKmh > (top - ICEPOL.SPEED_HYST_KMH) then
        ctx.iceSurplusT = 0
        return verdict("keep:speed", true)
      end
      if r > def.rOff then
        ctx.iceSurplusT = 0
        return verdict("keep:demand", true)
      end
      -- real lift / brake: the arbiter's lift hold decides
      if pedal < ICEPOL.LIFT_PEDAL or (ctx.brakeRaw or 0) > ICEPOL.LIFT_BRAKE then
        return verdict("off", false)
      end
      ctx.iceSurplusT = (ctx.iceSurplusT or 0) + (dt or 0)
      -- on the real S: a sporty driver keeps the engine longer
      local offS = def.offCalm + ((def.offSport or ICEPOL.OFF_SPORT_S) - def.offCalm)
                   * smoothstep((S - ICEPOL.DEF_S0) / ICEPOL.DEF_SPAN)
      if ctx.iceSurplusT < offS then return verdict("keep:surplus", true) end
      return verdict("off", false)
    end
    if pedal >= kickOff then return verdict("keep:pedal", true) end
    if speedKmh > (top - ICEPOL.SPEED_HYST_KMH) then return verdict("keep:speed", true) end
    if edriveCfgNow.demandStart and torqueReqNm > motorCapNm * demandOff then
      return verdict("keep:demand", true)
    end
    return verdict("off", false)
  end
  ctx.iceSurplusT = 0
  if ctx.iceStartUrgent then return verdict("urgent", true) end
  if ctx.iceStartFast then return verdict("fast", true) end
  -- exactly at the kick point counts; BATTERY CONTROL (kick 0) starts on any pedal
  if (kick > 0 and pedal >= kickOn) or pedal > kickOn then return verdict("kick", true) end
  if speedKmh > top then return verdict("speed", true) end
  if not edriveCfgNow.demandStart then return verdict("off", false) end
  if r >= def.rHard - (def.rHard - dSport) * sDem then return verdict("demand", true) end
  if r > def.rMin then
    if D >= don then return verdict("deficit", true) end
    if ctx.iceK0Window and D + (r - 1) * def.lead >= don then return verdict("k0lead", true) end
  end
  return verdict("off", false)
end

-- ===========================================================
-- Pedal map
-- ===========================================================
-- Returns the driver demand T [Nm] (before the N/P gate) and shaped = T /
-- ceiling (etk854hybrid_pedalShaped, the 8HP load axis).
--   split map (AUTO EV, DE with pEv, ICE not pinned):
--     p <= pEv:         T = knee * (p / pEv)^gEv
--     pEv < p < pJoin:  T = knee                              (plateau, ECO)
--     p >= pJoin:       T = knee + (top - knee) * ((p - pJoin) / (1 - pJoin))^gHi
--     knee = motor cap before the redline taper, low-passed, <= top;
--     top = min(ceilF, system cap) with capTop, else ceilF
--   otherwise: T = p^gamma * ceilF
-- Pedal-map view (etk854hybrid_*, map only, no creep floor): pedalIceJoin
-- (pJoin, 0 = no split map), pedalKickOn, pedalUrgent, pedalUrgentDwellS (8HP
-- kickdown dwell), pedalEvFrac (T / knee), pedalBandFrac, pedalZone (0 none |
-- 1 EV | 2 plateau | 3 band | 4 kick..urgent | 5 urgent), pedalLegacyEq (-1 n/a),
-- evZoneOpen (display only).
local function mapPedal(dt, pedalIn, deCfgNow, edriveCfgNow, kneeRawNm, ceilF, shifting)
  if _pm.kneeF == nil then
    _pm.kneeF = kneeRawNm
  elseif not shifting then
    _pm.kneeF = _pm.kneeF + (kneeRawNm - _pm.kneeF) * (1 - exp(-dt / _pm.KNEE_TAU_S))
  end
  local pEv = deCfgNow.pEv
  local split = pEv ~= nil and edriveCfgNow.deTuned == true
                and not (deCfgNow.iceAlwaysOn or ctx.legacyIceForce)
  local T, knee, top = 0, 0, ceilF
  if split and ceilF > 0 then
    -- capTop: the whole band is torque the coordinator really delivers
    local sc = ctx.systemTorqueCapNm or 0
    if (deCfgNow.capTop or 0) > 0 and sc > 0 then top = min(ceilF, sc) end
    -- low SOC: the knee slides onto the low-SOC anchor. Cold derate excluded:
    -- it must not lift the EV zone above the derated cap (ICE start inside it).
    local socFrac = (ctx.evPowerFraction or 1) / max(ctx.evColdDeratingFraction or 1, 0.05)
    local h = clamp01((socFrac - _pm.HEALTHY0) / _pm.HEALTHY_SPAN)
    local kneeLow = (deCfgNow.kneeLowFrac or (pEv ^ deCfgNow.pedalGamma)) * ceilF
    knee = min(top, h * _pm.kneeF + (1 - h) * max(_pm.kneeF, kneeLow))
  end
  -- for M.pedalForTorque
  _pm.split, _pm.cfg, _pm.ceil, _pm.knee, _pm.top = split, deCfgNow, ceilF, knee, top
  local pJ = split and (deCfgNow.pJoin or pEv) or 0
  if pedalIn > 0 and ceilF > 0 then
    if split then
      if pedalIn <= pEv then
        T = knee * (pedalIn / pEv) ^ deCfgNow.gEv
      elseif pedalIn < pJ then
        T = knee
      else
        T = knee + (top - knee) * ((min(pedalIn, 1) - pJ) / (1 - pJ)) ^ deCfgNow.gHi
      end
    else
      T = (pedalIn ^ deCfgNow.pedalGamma) * ceilF
    end
  end
  local shaped = (ceilF > 0) and clamp01(T / ceilF) or 0
  local ev = electrics.values
  ev.etk854hybrid_pedalShaped = shaped
  ev.etk854hybrid_pedalEvEdge = split and pEv or 0
  ctx.pedalJoin = pJ
  local kickP = split and (deCfgNow.kickJoin or deCfgNow.kickAuto) or 0
  local urgP = split and (deCfgNow.urgentPedal or ICEPOL.URGENT_PEDAL) or 0
  ev.etk854hybrid_pedalIceJoin = pJ
  ev.etk854hybrid_pedalKickOn = kickP
  ev.etk854hybrid_pedalUrgent = urgP
  ev.etk854hybrid_pedalUrgentDwellS = (edriveCfgNow.deTuned == true and deCfgNow.urgentDwellS) or 0
  ev.etk854hybrid_pedalEvFrac = (split and knee > 0) and clamp01(T / knee) or 0
  ev.etk854hybrid_pedalBandFrac = (split and T > knee and top > knee) and clamp01((T - knee) / (top - knee)) or 0
  local zone = 0
  if split then
    if pedalIn <= pEv then zone = 1
    elseif pedalIn < pJ then zone = 2
    elseif pedalIn < kickP then zone = 3
    elseif pedalIn < urgP then zone = 4
    else zone = 5 end
  end
  ev.etk854hybrid_pedalZone = zone
  local leq = -1
  if split and deCfgNow.legacyRef then
    local L = _pm.LEG
    if T <= 0 then
      leq = 0
    elseif T <= knee then
      leq = L.pEv * (T / knee) ^ (1 / L.gEv)
    else
      leq = L.pEv + (1 - L.pEv) * clamp01((T - knee) / max(ceilF - knee, 1)) ^ (1 / L.gHi)
    end
  end
  ev.etk854hybrid_pedalLegacyEq = leq
  ev.etk854hybrid_evZoneOpen = (split and (ctx.speedKmh or 0) <= (deCfgNow.evTopAuto or 999)) and 1 or 0
  return T, shaped
end

-- Driver demand as a wheel-torque reference. The pedal map is a shaft torque,
-- so an early upshift would cut wheel torque and lug the engine. With the ICE
-- ON and the pedal in the hybrid band:
--   T = T_map * k,  k = 1 + (clamp(nRef / n_shaft, 1, KMAX) - 1) * w,
--   w = clamp01((pedal - pJoin) / WSPAN)    (SPORT: (pedal - dmdP0) / dmdWspan)
-- Capped at CAPF x the gear's deliverable torque (etk854hybrid_shiftTavNm):
-- the 8HP downshift factors sit above CAPF, so it never forces a downshift;
-- the 8HP plans on the map demand (etk854hybrid_torqueRequestMapNm).
local function demandComp(dt, T, pedalIn, deCfgNow, rpmMotor, gearSign)
  local ev = electrics.values
  local k = 1
  local split = _pm.split
  if _pm.DMD_ON and (split or deCfgNow.dmdP0) and ctx.iceEngageState == "ON" and gearSign > 0 and T > 0 then
    local fam = (_iceDevice and _iceDevice.requiredEnergyType == "diesel") and "diesel" or "petrol"
    local nref = _pm.DMD_NREF[fam][ctx.deMode]
    if nref then
      local w
      if split then
        w = clamp01((pedalIn - (deCfgNow.pJoin or deCfgNow.pEv or 1)) / (deCfgNow.dmdWspan or _pm.DMD_WSPAN))
      else
        w = clamp01((pedalIn - deCfgNow.dmdP0) / (deCfgNow.dmdWspan or _pm.DMD_WSPAN))
      end
      k = 1 + (clamp(nref / max(rpmMotor, 500), 1, _pm.DMD_KMAX) - 1) * w
    end
  end
  local r = _pm.DMD_RATE * dt
  _pm.dmdK = _pm.dmdK + clamp(k - _pm.dmdK, -r, r)
  local out = T
  if _pm.dmdK > 1.001 then
    local tav = ev.etk854hybrid_shiftTavNm
    local cap = (type(tav) == "number" and tav > 0) and _pm.DMD_CAPF * tav or T
    out = min(T * _pm.dmdK, max(T, cap))
  end
  ev.etk854hybrid_dmdComp = _pm.dmdK
  ev.etk854hybrid_torqueRequestMapNm = T
  return out
end

-- ===========================================================
-- Recuperation control
-- ===========================================================
-- Returns regenFrac (0..1 motor command), the envelope at this shaft speed
-- [Nm] and whether regen is meaningfully active. Writes etk854hybrid_evRegen,
-- raises lockupClutchRatio, blends electrics brake / brakelights.
local function updateRecuperation(dt, pedalRaw, brakeRaw, speedKmh, rpmMotor,
                                  gearSign, deCfgNow)
  local ev = electrics.values
  -- lazy, throttled retry (readVehicleRefs sweeps every node)
  if not _rgn.refsOk then
    _rgn.refsRetryT = (_rgn.refsRetryT or 0) - dt
    if _rgn.refsRetryT <= 0 and (_rgn.refsTries or 0) < 10 then
      _rgn.refsRetryT = 1.0
      _rgn.refsTries = (_rgn.refsTries or 0) + 1
      readVehicleRefs()
    end
  end

  local lvl = RGN.LEVEL[_rgn.levelName] or RGN.LEVEL.MED
  local c = curves()
  local envelopeNm = (c and c.motorRegenTorqueAtRpm(rpmMotor)) or RGN.FALLBACK_NM
  if envelopeNm < 1 then envelopeNm = 1 end

  local slipNow = (speedKmh > 10) and rearOverSlip() or 0
  _rgn.osF = _rgn.osF + (slipNow - _rgn.osF) * min(1, dt / RGN.SLIP_TAU_S)
  -- Qualified ABS event: the raw flag also rises on driveline ringing at ~1 %
  -- slip. The 8HP opens the lock-up on etk854hybrid_absQualified only.
  local absRaw = flag(ev.absActive)
  if absRaw and (brakeRaw >= RGN.ABS_HARD_BRAKE or _rgn.osF > RGN.ABS_REAR_SLIP
                 or maxWheelSlip() > RGN.ABS_WHEEL_SLIP) then
    _rgn.absHoldT = RGN.ABS_HOLD_S
  else
    _rgn.absHoldT = max(0, _rgn.absHoldT - dt)
  end
  local absCut = _rgn.absHoldT > 0
  ev.etk854hybrid_absQualified = absCut and 1 or 0
  -- trailer stability acting: no regen (it brakes the car only, the trailer would push)
  local hardCut = ctx.sailSuppressRegen
    or gearSign <= 0
    or (ev.parkingbrake or 0) > 0.1
    or absCut
    or ctx.tsaFreeze == true

  local ratio = mechRatioToWheels()
  _rgn.ratio = ratio
  local decelToWheelNm = _rgn.mass * _rgn.wheelR   -- T = m * a * r

  -- ---- lift-off target -------------------------------------------------
  local liftGate = 0
  if pedalRaw <= RGN.THR_FULL then
    liftGate = 1
  elseif pedalRaw < RGN.THR_OFF then
    liftGate = 1 - (pedalRaw - RGN.THR_FULL) / (RGN.THR_OFF - RGN.THR_FULL)
  end
  local liftDecelFull = lvl.liftDecel * (deCfgNow.coastScale or 1.0) * ctx.regenStrength
  local liftDecel = liftDecelFull * liftGate
  -- no lift-off regen while the drive torque is still ramping out
  local driveOut = (ctx.torqueTargetNm or 0) <= RGN.XOVER_NM
  if not driveOut then liftDecel = 0 end

  -- ---- brake pedal target (blended braking) ----------------------------
  -- No blending with a trailer coupled (trailerAttached / airTrailer): its
  -- brake replays our electrics brake, which after the blend is only the
  -- friction share, so the trailer would push. Lift-off regen still applies.
  local trailerOn = flag(ev.etk854hybrid_trailerAttached) or flag(ev.etk854hybrid_airTrailer)
  ev.etk854hybrid_regenTrailerNoBlend = trailerOn and 1 or 0
  local brakeDemandNm = brakeRaw * _rgn.brakeNm
  local share = 0
  if brakeRaw > 0.005 and not trailerOn then
    if brakeRaw <= RGN.BRAKE_SHARE_FULL_END then
      share = RGN.BRAKE_SHARE_MAX
    elseif brakeRaw < RGN.BRAKE_SHARE_ZERO_END then
      share = RGN.BRAKE_SHARE_MAX * (1 - smoothstep(
        (brakeRaw - RGN.BRAKE_SHARE_FULL_END)
        / (RGN.BRAKE_SHARE_ZERO_END - RGN.BRAKE_SHARE_FULL_END)))
    end
  end
  local brakeRegenAllowNm = brakeDemandNm * share          -- at the wheels
  local brakeDecel = (decelToWheelNm > 1)
    and (brakeRegenAllowNm / decelToWheelNm * ctx.regenStrength) or 0

  local targetDecel = max(liftDecel, brakeDecel)
  -- Speed fade. EV creep armed: on the gear-1 turbine rpm (gear-independent,
  -- no dip at the 2->1 downshift); the friction brake takes over via the
  -- blend credit. Otherwise the fixed fade to MIN_KMH.
  local cIdle = ctx.creepIdleRpm or 0
  local gb1 = _rgn.gearbox                     -- cached by mechRatioToWheels()
  local g1 = gb1 and gb1.gearRatios and gb1.gearRatios[1]
  if cIdle > 0 and type(g1) == "number" and g1 > 0 then
    local nT1 = abs(gb1.outputAV1 or 0) * g1 * AV_TO_RPM
    local lo = RGN.CREEP_FADE_LO * cIdle
    targetDecel = targetDecel * clamp01((nT1 - lo) / ((RGN.CREEP_FADE_HI - RGN.CREEP_FADE_LO) * cIdle))
  elseif speedKmh <= RGN.MIN_KMH then
    targetDecel = 0
  elseif speedKmh < RGN.TAPER_KMH then
    targetDecel = targetDecel * (speedKmh - RGN.MIN_KMH)
                              / (RGN.TAPER_KMH - RGN.MIN_KMH)
  end
  _rgn.targetDecel = hardCut and 0 or targetDecel

  -- ---- deceleration -> motor torque ------------------------------------
  local motorNm = 0
  if not hardCut and targetDecel > 0 and ratio > 0.1 then
    motorNm = targetDecel * decelToWheelNm / (ratio * RGN.DRIVE_EFF)
    motorNm = min(motorNm, envelopeNm)
    local avMotor = rpmMotor * RPM_TO_AV
    if avMotor > 1.0 then
      motorNm = min(motorNm, stageCfg().regenKw * 1000 / avMotor)   -- stage regen budget
    end
    motorNm = motorNm * (ctx.regenAcceptFraction or 1.0)   -- SOC / temperature
    -- a coupled ICE's overrun drag already brakes: the level feels the same
    -- with K0 open or closed
    motorNm = max(0, motorNm - (ctx.iceDragNm or 0))
    -- rear over-braked: scale back through the release ramp (the ABS cut
    -- stays the last resort)
    if _rgn.osF > RGN.REGEN_SLIP_START then
      motorNm = motorNm * clamp01(1 - (_rgn.osF - RGN.REGEN_SLIP_START)
                                      / (RGN.ABS_REAR_SLIP - RGN.REGEN_SLIP_START))
    end
  end

  -- rate limit in Nm; the fraction is derived afterwards
  if absCut and _regenSmoother then
    _regenSmoother:set(0)   -- ABS: out of the loop this frame, no ramp
  end
  local appliedNm = motorNm
  if _regenSmoother then appliedNm = _regenSmoother:get(motorNm, dt) end
  if appliedNm < 0 then appliedNm = 0 end
  if appliedNm > envelopeNm then
    -- envelope fell faster than the release rate (downshift): the limiter
    -- follows, or the torque would jump back once the envelope recovers
    appliedNm = envelopeNm
    if _regenSmoother then _regenSmoother:set(envelopeNm) end
  end
  local regenFrac = clamp01(appliedNm / envelopeNm)
  _rgn.motorNm = appliedNm

  -- ---- converter lock-up assist ----------------------------------------
  local wantLock = 0
  local k0Now = ev.etk854hybrid_k0ClutchRatio or 0
  local lockMinKmh = (k0Now > 0.2) and RGN.LOCKUP_MIN_KMH or RGN.LOCKUP_MIN_EV_KMH
  if appliedNm > 5 and speedKmh > lockMinKmh
     and not flag(ev.isShifting) then
    wantLock = 1
    if k0Now > 0.2 then
      -- ICE coupled: keep a margin over idle or it stalls on the brakes
      local idle = (c and c.iceIdleRPM()) or 800
      if (ev.rpm or 0) < idle * RGN.LOCKUP_ICE_RPM_FRAC then
        wantLock = 0
      end
    end
  end
  local lockRate = (wantLock > _rgn.lockupCmd) and RGN.LOCKUP_RATE_UP
                                              or RGN.LOCKUP_RATE_DN
  _rgn.lockupCmd = clamp01(_rgn.lockupCmd
    + clamp(wantLock - _rgn.lockupCmd, -lockRate * dt, lockRate * dt))
  if _rgn.lockupCmd > 0.01 then
    ev.lockupClutchRatio = max(ev.lockupClutchRatio or 0, _rgn.lockupCmd)
  end

  -- modelled road torque: credited through the lock-up clutch only
  local lockupNow = clamp01(ev.lockupClutchRatio or 0)
  local wheelNm = appliedNm * ratio * RGN.DRIVE_EFF * lockupNow
  -- Blend credit = measured converter braking torque (outputTorque1) x ratio.
  -- The lock-up model collapses when the 8HP opens the lock-up for a
  -- downshift while the converter still transmits most of the regen (rear
  -- over-braking); it is only the fallback without a converter.
  local tcDev = _rgn.tc or powertrain.getDevice("torqueConverter")
  _rgn.tc = tcDev
  local wheelDecelNm
  local tcNm = tcDev and tonumber(tcDev.outputTorque1)
  if tcNm then
    _rgn.tcF = _rgn.tcF + (max(0, -tcNm) - _rgn.tcF) * min(1, dt / RGN.MEAS_TAU_S)
    wheelDecelNm = _rgn.tcF * ratio * RGN.DRIVE_EFF
  else
    wheelDecelNm = wheelNm + (ctx.iceDragNm or 0) * ratio * RGN.DRIVE_EFF * lockupNow
  end

  -- ---- friction brake blending -----------------------------------------
  local newBrakeCoef = brakeRaw
  if _rgn.refsOk and brakeDemandNm > 1 then
    local credit = min(wheelDecelNm, brakeRegenAllowNm)
    if credit > 1 then
      newBrakeCoef = clamp01((brakeDemandNm - credit) / max(1, _rgn.brakeNm))
    end
  end

  -- ---- one-pedal: friction blend into the standstill + hold -------------
  local assistTarget = 0
  local onePedalActive = lvl.onePedal and not hardCut and gearSign > 0
                         and liftGate > 0.5 and brakeRaw < 0.02 and driveOut
  if onePedalActive and _rgn.refsOk then
    local missingNm = liftDecelFull * decelToWheelNm - wheelDecelNm
    if missingNm > 0 then
      assistTarget = clamp(missingNm / _rgn.brakeNm, 0, RGN.ASSIST_MAX)
    end
  end
  if pedalRaw > RGN.HOLD_RELEASE_THR or gearSign <= 0 or not lvl.onePedal then
    _rgn.holdLatched = false
  elseif onePedalActive and speedKmh < RGN.HOLD_KMH then
    _rgn.holdLatched = true
  end
  if _rgn.holdLatched then
    assistTarget = max(assistTarget, RGN.HOLD_BRAKE)
  end
  local aRate = (assistTarget > _rgn.assistCmd) and RGN.ASSIST_RATE_UP
                                               or RGN.ASSIST_RATE_DN
  _rgn.assistCmd = clamp(_rgn.assistCmd
    + clamp(assistTarget - _rgn.assistCmd, -aRate * dt, aRate * dt),
    0, RGN.ASSIST_MAX)
  -- snap to zero only while releasing: at 240 fps one ramp step (0.008) is
  -- below the 0.01 band, so a plain dead band would never let it build
  if assistTarget <= 0 and _rgn.assistCmd < 0.01 then _rgn.assistCmd = 0 end
  newBrakeCoef = max(newBrakeCoef, _rgn.assistCmd)

  -- trailer stability brake request: max(driver, request)
  local tsaB = ev.etk854hybrid_tsaBrakeCmd
  if type(tsaB) == "number" and tsaB == tsaB and tsaB > 0 then newBrakeCoef = max(newBrakeCoef, min(tsaB, 0.35)) end

  if newBrakeCoef ~= brakeRaw then
    ev.brake = newBrakeCoef
  end
  -- Brake lights for regen braking too: 1.3 m/s^2 is the ECE R13-H threshold
  -- for deceleration without the pedal (MED / HIGH light up on lift-off, LOW
  -- not); also for a trailer stability brake request or single-wheel brake.
  if brakeRaw > 0.02 or _rgn.assistCmd > 0.02 or _rgn.targetDecel > 1.3
     or (tonumber(tsaB) or 0) > 0.02 or abs(tonumber(ev.etk854hybrid_tsaWheelNm) or 0) > 200 then
    ev.brakelights = 1
  end

  -- ---- channels --------------------------------------------------------
  ev.etk854hybrid_evRegen        = regenFrac
  ev.etk854hybrid_regenDecelMs2  = _rgn.targetDecel
  ev.etk854hybrid_regenWheelNm   = wheelNm
  ev.etk854hybrid_regenLockup    = _rgn.lockupCmd
  ev.etk854hybrid_regenBrakeCoef = _rgn.assistCmd
  ev.etk854hybrid_onePedalHold   = _rgn.holdLatched and 1 or 0

  return regenFrac, envelopeNm, (regenFrac >= RGN.ACTIVE_THR)
end

-- ===========================================================
-- Mode API
-- ===========================================================

-- DSC / DTC coupling (controller "driveModes"): DE_CFG.dscMode, but a driver's
-- DSC OFF is never re-armed. Returns false while the DSE is not ready (the
-- caller sets ctx.dscPending, updateHud retries).
local function applyDscForDeMode(deName)
  -- Controller init runs in pairs() order: driveModes can exist before the DSE
  -- registered its CMU, and setDriveMode would then throw. Not before the first
  -- updateGFX; any error counts as "not ready yet".
  if not ctx.controllersReady then return false end
  local dm = controller.getController and controller.getController("driveModes")
  if not (dm and dm.setDriveMode and dm.getCurrentDriveModeKey) then
    return false
  end
  local okCur, current = pcall(dm.getCurrentDriveModeKey)
  if not okCur then return false end
  if current == "off" then
    ctx.dscKey = "off"
    return true
  end
  local wanted = (DE_CFG[deName] and DE_CFG[deName].dscMode) or "dsc"
  if current ~= wanted then
    local ok = pcall(dm.setDriveMode, wanted)
    if not ok then return false end
  end
  ctx.dscKey = wanted
  return true
end

-- on ctx to save upvalues in updateHud / publishDefaults
ctx.applyDsc = applyDscForDeMode
ctx.dscLabels = DSC_LABEL

-- 2D state -> legacy name (HUD / QA)
local function refreshLegacyMode()
  local legacy
  if ctx.deMode == "SPORT" then legacy = "SPORT"
  elseif ctx.edriveMode == "BATTERY_CONTROL" then legacy = "HOLD"
  elseif ctx.edriveMode == "MAX_EDRIVE" then legacy = "EV"
  elseif ctx.legacyIceForce then legacy = "HYBRID"
  else legacy = "AUTO" end
  ctx.legacyMode = legacy
  for i, name in ipairs(LEGACY_MODE_ORDER) do
    if name == legacy then ctx.legacyIdx = i break end
  end
end

local function applyModes(deName, edriveName, iceForce)
  if DE_CFG[deName] then
    ctx.deMode = deName
    for i, name in ipairs(DE_ORDER) do
      if name == deName then ctx.deIdx = i break end
    end
  end
  if EDRIVE_CFG[edriveName] then
    ctx.edriveMode = edriveName
    for i, name in ipairs(EDRIVE_ORDER) do
      if name == edriveName then ctx.edriveIdx = i break end
    end
  end
  ctx.legacyIceForce = (iceForce == true)
  refreshLegacyMode()
  -- not ready during M.init: updateHud retries
  if not applyDscForDeMode(ctx.deMode) then
    ctx.dscPending = true
  end
end

local function modeLabel()
  return ctx.deMode .. "/" .. (EDRIVE_SHORT[ctx.edriveMode] or ctx.edriveMode)
end

-- message names (etk854hybrid_mode keeps modeLabel(), the QA checks it)
local MODE_TEXT = { COMFORT = "COMFORT", ECO_PRO = "ECO PRO", SPORT = "SPORT",
  AUTO_EDRIVE = "AUTO EV", MAX_EDRIVE = "MAX EV", BATTERY_CONTROL = "BATTERY CONTROL" }
local function announceMode()
  guihooks.message("ETK 854: " .. (MODE_TEXT[ctx.deMode] or tostring(ctx.deMode)) .. " / "
    .. (MODE_TEXT[ctx.edriveMode] or tostring(ctx.edriveMode)), 3, "vehicle.hybridmode")
  ctx.dbg(1, "mode", "mode = " .. modeLabel() .. " (legacy " .. ctx.legacyMode .. ")")
end

function M.setDeMode(name)
  if not DE_CFG[name] then
    log("W", "etk854hybrid", "Unknown driving experience mode: " .. tostring(name))
    return
  end
  applyModes(name, ctx.edriveMode, ctx.legacyIceForce)
  announceMode()
end

function M.cycleDeMode()
  ctx.deIdx = (ctx.deIdx % #DE_ORDER) + 1
  M.setDeMode(DE_ORDER[ctx.deIdx])
end

function M.setEdriveMode(name)
  if not EDRIVE_CFG[name] then
    log("W", "etk854hybrid", "Unknown EV mode: " .. tostring(name))
    return
  end
  applyModes(ctx.deMode, name, false)
  announceMode()
end

function M.cycleEdriveMode()
  ctx.edriveIdx = (ctx.edriveIdx % #EDRIVE_ORDER) + 1
  M.setEdriveMode(EDRIVE_ORDER[ctx.edriveIdx])
end

function M.setBatteryTargetSoc(value)
  local v = tonumber(value)
  if not v then return end
  if v > 1 then v = v / 100 end          -- accept 0..100 as well as 0..1
  ctx.batteryTargetSoc = clamp(v, 0.10, 1.00)
  guihooks.message(string.format("ETK 854: target SOC %.0f%%", ctx.batteryTargetSoc * 100),
                   3, "vehicle.hybridmode")
end

function M.adjustBatteryTargetSoc(deltaPp)
  local d = (tonumber(deltaPp) or 10) / 100
  M.setBatteryTargetSoc(clamp((ctx.batteryTargetSoc or 0.90) + d, 0.10, 1.00))
end

-- --- legacy 5-mode API (input actions, QA suite, HUD) ---------------------
function M.setMode(modeName)
  local preset = LEGACY_MODE_MAP[modeName]
  if not preset then
    log("W", "etk854hybrid", "Unknown mode: " .. tostring(modeName))
    return
  end
  applyModes(preset.de, preset.edrive, preset.iceForce)
  announceMode()
end

function M.cycleMode()
  ctx.legacyIdx = (ctx.legacyIdx % #LEGACY_MODE_ORDER) + 1
  M.setMode(LEGACY_MODE_ORDER[ctx.legacyIdx])
end

function M.getMode() return modeLabel() end
function M.getLegacyMode() return ctx.legacyMode end
function M.getCtx() return ctx end

-- ===========================================================
-- Regen level (driver-selectable OFF / LOW / MED / HIGH)
-- ===========================================================
-- OFF hands lift-off to the sailing FSM; every other level takes it back.
local function applyRegenLevel(announce)
  local name = ctx.regenLevelOrder[ctx.regenLevelIdx] or "MED"
  local cfg = RGN.LEVEL[name] or RGN.LEVEL.MED
  _rgn.levelName = name
  ctx.regenAllowsSail = (cfg.allowSail == true)
  if not cfg.onePedal then
    _rgn.holdLatched = false
  end
  electrics.values.etk854hybrid_regenLevel = name
  if announce then
    local hint = (name == "OFF") and " (sailing allowed)"
      or (cfg.onePedal and " (One-Pedal)" or "")
    guihooks.message("ETK 854 recuperation: " .. name .. hint, 3, "vehicle.regenlevel")
  end
  ctx.dbg(1, "regen", string.format(
    "level=%s liftDecel=%.2f m/s^2 onePedal=%s sailing=%s",
    name, cfg.liftDecel, tostring(cfg.onePedal == true), tostring(cfg.allowSail == true)))
end

function M.cycleRegenLevel()
  ctx.regenLevelIdx = (ctx.regenLevelIdx % #ctx.regenLevelOrder) + 1
  applyRegenLevel(true)
end

function M.setRegenLevel(levelName)
  for i, name in ipairs(ctx.regenLevelOrder) do
    if name == levelName then
      ctx.regenLevelIdx = i
      applyRegenLevel(true)
      return
    end
  end
  log("W", "etk854hybrid", "Unknown regen level: " .. tostring(levelName))
end

function M.getRegenLevel() return _rgn.levelName end

-- ===========================================================
-- Launch Control (logic in etk854hybrid_launchControl.lua)
-- Manual arm (Alt+K, Actions app, QA): S / M in any DE, or D in SPORT; a
-- second press disarms. SPORT + DTC / DSC OFF also auto-arms on brake +
-- kickdown at a standstill.
-- ===========================================================
function M.armLaunchControl()
  local lm = ctx.modules.launch
  if lm and lm.arm then
    lm.arm(ctx, "manual")
  else
    guihooks.message("Launch Control not available", 3, "vehicle.launch")
  end
end

-- live tuning, not persistent, e.g. M.setLaunchParam('MARGIN', 0.70)
function M.setLaunchParam(key, value)
  local lm = ctx.modules.launch
  if lm and lm.setParam then return lm.setParam(key, value) end
  return nil
end

function M.getLaunchState()
  local lm = ctx.modules.launch
  if lm and lm.getState then return lm.getState() end
  return nil
end

-- ===========================================================
-- Sportiness index, ICE sequencer, QA hooks
-- ===========================================================
function M.getSportIndex() return ctx.sportIndex or 0 end

function M.setSportIndex(value)
  local si = ctx.modules.sportIdx
  if si and si.set then si.set(ctx, value) end
end

function M.resetSportIndex()
  local si = ctx.modules.sportIdx
  if si and si.reset then si.reset(ctx) end
end

function M.getIceEngageState() return ctx.iceEngageState or "OFF" end
function M.getIceSeqStats() return ctx.iceSeqLast end

-- QA only: true / false force the ICE, nil = policy (reset at every test end)
function M.qaForceIce(value)
  if value == nil then ctx.qaIceOverride = nil else ctx.qaIceOverride = (value == true) end
end

-- debug: true = legacy one-frame start (no rev-match, no fill)
function M.setIceSeqPassthrough(on)
  ctx.iceSeqPassthrough = (on == true)
  log("I", "etk854hybrid", "iceSeq passthrough = " .. tostring(ctx.iceSeqPassthrough))
end

-- Inverse of last frame's pedal map: pedal 0..1 for Tnm (QA load tests), nil
-- before the first frame. Split map: the knee returns pEv.
function M.pedalForTorque(Tnm)
  local cfg, ceil = _pm.cfg, _pm.ceil
  if not cfg or not ceil or ceil <= 0 or type(Tnm) ~= "number" then return nil end
  if not _pm.split then
    local T = clamp(Tnm, 0, ceil)
    return clamp01((T / ceil) ^ (1 / max(cfg.pedalGamma or 1, 0.05)))
  end
  local top = _pm.top or ceil
  if top <= 0 then top = ceil end
  local T = clamp(Tnm, 0, top)
  local knee, pEv = _pm.knee or 0, cfg.pEv
  if knee <= 0 then return nil end
  if T <= knee then
    return pEv * (T / knee) ^ (1 / cfg.gEv)
  end
  local span = top - knee
  if span <= 0 then return 1 end
  local pJ = cfg.pJoin or pEv
  return clamp01(pJ + (1 - pJ) * ((T - knee) / span) ^ (1 / cfg.gHi))
end

-- Live tuning of ICE policy / pedal map, not persistent; returns the old value
-- (nil = unknown key). No de: ICEPOL; with de: ICEPOL.DEF[de] if it has the
-- key, else DE_CFG[de] (pJoin / kickJoin re-derived),
-- e.g. M.setIcePolicyParam('onCalm', 0.6, 'COMFORT')
function M.setIcePolicyParam(key, value, de)
  local t = ICEPOL
  if de ~= nil then
    local d = ICEPOL.DEF[de]
    t = (d and type(d[key]) == "number") and d or DE_CFG[de]
  end
  if type(t) ~= "table" or type(t[key]) ~= "number" or type(value) ~= "number" then return nil end
  local old = t[key]
  t[key] = value
  if de and t == DE_CFG[de] then deriveSplitThresholds(t) end
  return old
end

-- ===========================================================
-- Lifecycle
-- ===========================================================
local function publishDefaults()
  electrics.values.etk854hybrid_iceThrottle           = 0
  electrics.values.etk854hybrid_evThrottle            = 0
  electrics.values.etk854hybrid_evRegen               = 0
  -- etk854hybrid_k0ClutchRatio: single writer iceSequencer (failsafe excepted)
  electrics.values.etk854hybrid_mode                  = modeLabel()
  electrics.values.etk854hybrid_deMode                = ctx.deMode
  electrics.values.etk854hybrid_edriveMode            = ctx.edriveMode
  electrics.values.etk854hybrid_modeLegacy            = ctx.legacyMode
  electrics.values.etk854hybrid_phase                 = "IDLE"
  electrics.values.etk854hybrid_iceState              = 1
  electrics.values.etk854hybrid_regenLevel            = ctx.regenLevelOrder[ctx.regenLevelIdx]
  electrics.values.etk854hybrid_tcActive              = 0      -- DSE intervening
  electrics.values.etk854hybrid_dscMode               = ctx.dscLabels[ctx.dscKey] or "DSC"
  electrics.values.etk854hybrid_batterySoc            = 0
  electrics.values.etk854hybrid_icePower              = 0
  electrics.values.etk854hybrid_evPower               = 0
  electrics.values.etk854hybrid_launchState           = "IDLE"
  -- OFF/ARMED/PRELOAD/READY/FIRE/ABORT/GUARD; other launch* channels: launch module
  electrics.values.etk854hybrid_launchPhase           = "OFF"
  electrics.values.etk854hybrid_launchLevelHold       = 0
  electrics.values.etk854hybrid_batteryFraction       = 1
  electrics.values.etk854hybrid_batteryBand           = "NORMAL"
  electrics.values.etk854hybrid_evAvailable           = 1
  electrics.values.etk854hybrid_regenAllowed          = 1
  electrics.values.etk854hybrid_evPowerFraction       = 1
  electrics.values.etk854hybrid_regenAcceptFraction   = 1
  electrics.values.etk854hybrid_absQualified          = 0
  electrics.values.etk854hybrid_boostStage            = ctx.boostStage
  electrics.values.etk854hybrid_evTorqueCapNm         = 0
  electrics.values.etk854hybrid_evTorqueNm            = 0
  electrics.values.etk854hybrid_iceTorqueNm           = 0
  electrics.values.etk854hybrid_iceMaxTorqueNm        = 0
  electrics.values.etk854hybrid_sysTorqueNm           = 0
  electrics.values.etk854hybrid_chargeMode            = "OFF"
  electrics.values.etk854hybrid_torqueRequestNm       = 0
  electrics.values.etk854hybrid_pedalShaped           = 0
  -- ICE policy diagnostics (deficit D / Don [s]) and the pedal-map view
  electrics.values.etk854hybrid_pedalEvEdge           = 0
  electrics.values.etk854hybrid_icePolicyReason       = "none"
  electrics.values.etk854hybrid_iceDeficitS           = 0
  electrics.values.etk854hybrid_iceDeficitOnS         = 0
  electrics.values.etk854hybrid_pedalIceJoin          = 0
  electrics.values.etk854hybrid_pedalKickOn           = 0
  electrics.values.etk854hybrid_pedalUrgent           = 0
  electrics.values.etk854hybrid_pedalUrgentDwellS     = 0
  electrics.values.etk854hybrid_pedalEvFrac           = 0
  electrics.values.etk854hybrid_pedalBandFrac         = 0
  electrics.values.etk854hybrid_pedalZone             = 0
  electrics.values.etk854hybrid_pedalLegacyEq         = -1
  electrics.values.etk854hybrid_evZoneOpen            = 0
  electrics.values.etk854hybrid_policySportS          = 0
  electrics.values.etk854hybrid_regenTorqueNm         = 0
  electrics.values.etk854hybrid_regenRatio            = RGN.RATIO_FALLBACK
  electrics.values.etk854hybrid_regenDecelMs2         = 0
  electrics.values.etk854hybrid_regenWheelNm          = 0
  electrics.values.etk854hybrid_regenLockup           = 0
  electrics.values.etk854hybrid_regenBrakeCoef        = 0
  electrics.values.etk854hybrid_onePedalHold          = 0
  electrics.values.etk854hybrid_targetSoc             = ctx.batteryTargetSoc
  electrics.values.etk854hybrid_sailState             = "DRIVE"
  electrics.values.etk854hybrid_sailActive            = 0
  electrics.values.etk854hybrid_sailTime              = 0
  electrics.values.etk854hybrid_iceRestartActive      = 0
  electrics.values.etk854hybrid_iceRestartMs          = 0
  -- defaults; sequencer / coordinator overwrite them every frame
  electrics.values.etk854hybrid_sportIndex            = 0
  electrics.values.etk854hybrid_evTorqueAvailNm       = 0
  electrics.values.etk854hybrid_iceEngageState        = "ON"
  electrics.values.etk854hybrid_iceLockMs             = 0
  electrics.values.etk854hybrid_iceFireMs             = 0
  electrics.values.etk854hybrid_iceStartType          = "NONE"
  electrics.values.etk854hybrid_iceSeqFault           = "none"
  electrics.values.etk854hybrid_iceSyncBlocked        = 0
  electrics.values.etk854hybrid_iceOffHoldS           = 0
  electrics.values.etk854hybrid_fillGain              = 0
  electrics.values.etk854hybrid_iceExpNm              = 0
  electrics.values.etk854hybrid_fillPosMaxNm          = 0
  electrics.values.etk854hybrid_fillNegMaxNm          = 0
  electrics.values.etk854hybrid_torqueTargetNm        = 0
  electrics.values.etk854hybrid_k0TorqueNm            = 0
  electrics.values.etk854hybrid_k0SlipRpm             = 0
  electrics.values.etk854hybrid_fillNm                = 0
  electrics.values.etk854hybrid_shaftTorqueNm         = 0
  electrics.values.etk854hybrid_iceTorqueEstNm        = 0
  electrics.values.etk854hybrid_fillLocked            = 0
  -- EV creep (creep module; creepNm..tcPumpNm from the motor device);
  -- creepMaxNm 0 = device floor inert
  electrics.values.etk854hybrid_creepTargetRpm        = 0
  electrics.values.etk854hybrid_creepMaxNm            = 0
  electrics.values.etk854hybrid_creepCut              = 0
  electrics.values.etk854hybrid_creepIdleRpm          = 0
  electrics.values.etk854hybrid_creepState            = 0
  electrics.values.etk854hybrid_creepReason           = "init"
  electrics.values.etk854hybrid_creepHold             = 0
  electrics.values.etk854hybrid_bodyKmh               = 0
  electrics.values.etk854hybrid_creepNm               = 0
  electrics.values.etk854hybrid_creepSetRpm           = 0
  electrics.values.etk854hybrid_creepAuth             = 0
  electrics.values.etk854hybrid_pumpRpm               = 0
  electrics.values.etk854hybrid_tcPumpNm              = 0
end

local function resetRuntimeState()
  ctx.hudTimer = 0
  ctx.regenActiveLast = false
  ctx.chargeNmLast = 0
  ctx.chargeModeLast = "OFF"
  ctx.chargingActive = false
  ctx.qaIceOverride = nil
  ctx.iceDragNm = 0
  ctx.torqueTargetNm = 0
  ctx.motorBaseNm = 0
  ctx.iceExpNm = 0
  ctx.launchActive = false
  ctx.failsafeInit = false
  ctx.launchArmed = false
  ctx.launchReady = false
  ctx.launchHold = false
  ctx.launchIceThr = 0
  ctx.launchGuardNm = nil
  ctx.launchSeedNm = nil
  ctx.iceDef, ctx.iceDefOn, ctx.iceDefKey, ctx.iceFastLatch, ctx.iceSurplusT = 0, 0, nil, false, 0
  ctx.iceUrgentT, ctx.policySportS, ctx.pedalJoin = 0, 0, 0
  ctx.icePolicyReason = "none"
  ctx.creepIdleRpm, ctx.creepAvail = 0, false
  _pm.kneeF = nil          -- re-seeded from the first frame
  _pm.dmdK = 1
  _pm.top = 0
  _icePowerEma = 0
  _evPowerEma = 0
  _chargeRegenFrac = 0
  _parkChargeRpmIntegral = 0
  _simTime = 0
  _dbgLastLogT = 0
  _nChargeRpmSetpoint = 0
  _nChargeEngaged = false
  _rgn.lockupCmd = 0
  _rgn.assistCmd = 0
  _rgn.holdLatched = false
  _rgn.targetDecel = 0
  _rgn.motorNm = 0
  _rgn.gearbox = nil
  _rgn.whl = nil
  _rgn.osF = 0
  _rgn.absHoldT = 0
  _rgn.tc = nil
  _rgn.tcF = 0
  -- refsOk kept: mass / radius / brake torque do not change on a reset
  _rgn.refsTries = 0
  _rgn.refsRetryT = 0
  if _regenSmoother and _regenSmoother.set then _regenSmoother:set(0) end
end

-- No M.reset: controller.lua re-runs M.init + M.initSecondStage on a vehicle reset.
function M.init(jbeamData)
  ctx.jbeamData = jbeamData
  ctx.debugLevel = readNumber(jbeamData, "debugLogLevel", 1)

  -- E-Boost stage: a boost-stage part with section etk854hybridBoostStage
  -- (stage >= 0, merged into v.data) pins it, else the $ebooststage slider
  local sliderStage = floor(clamp(readNumber(jbeamData, "eBoostStage", 0), 0, 3) + 0.5)
  local stageSection = v and v.data and v.data.etk854hybridBoostStage
  local partStage = nil
  if type(stageSection) == "table" and type(stageSection.stage) == "number" and stageSection.stage >= 0 then
    partStage = floor(clamp(stageSection.stage, 0, 3) + 0.5)
  end
  ctx.boostStage = partStage or sliderStage
  ctx.boostStageOrigin = partStage and "part" or "tuning"
  ctx.regenStrength = clamp(readNumber(jbeamData, "regenStrength", 1.0), 0.5, 2.0)
  ctx.sailingEnabled = (readNumber(jbeamData, "sailingEnabled", 1) ~= 0) and 1 or 0
  ctx.batteryTargetSoc = clamp(readNumber(jbeamData, "batteryControlTargetSoc", 0.90), 0.10, 1.0)
  -- EV pedal zone (fraction, or the % display value from a .pc)
  for _, pz in ipairs({ { "COMFORT", "evPedalZoneComfort" }, { "ECO_PRO", "evPedalZoneEco" } }) do
    local cfg = DE_CFG[pz[1]]
    local zone = readNumber(jbeamData, pz[2], cfg.pEv)
    if zone > 1 then zone = zone / 100 end
    cfg.pEv = clamp(zone, cfg.pEvMin or PEV_MIN, cfg.pEvMax or PEV_MAX)
    deriveSplitThresholds(cfg)
  end
  ICEPOL.RATE_URGENT = clamp(readNumber(jbeamData, "iceRateUrgent", ICEPOL.RATE_URGENT), 0, 50)
  -- System torque cap = base (ECU section or controller arg) + stage bonus;
  -- 0 disables it, the lower clamp guards against typos
  local capBase = readNumber(jbeamData, "systemTorqueCapNm", SYSTEM_TORQUE_CAP_DEFAULT_NM)
  local capSection = v and v.data and v.data.etk854hybridSystemCap
  if type(capSection) == "table" and type(capSection.baseNm) == "number" then
    capBase = capSection.baseNm
    ctx.systemTorqueCapOrigin = "ecu"
  else
    ctx.systemTorqueCapOrigin = "powertrain"
  end
  ctx.systemTorqueCapBaseNm = capBase
  if capBase > 0 then
    ctx.systemTorqueCapNm = clamp(capBase + (stageCfg().capBonusNm or 0), 150, 5000)
  else
    ctx.systemTorqueCapNm = 0
    ctx.systemTorqueCapBaseNm = 0
  end

  applyModes("COMFORT", "AUTO_EDRIVE", false)
  ctx.regenLevelIdx = 3          -- MED
  applyRegenLevel(false)
  resetRuntimeState()
  -- argument order: newTemporalSmoothing uses the SECOND rate away from zero
  -- (engaging), the FIRST toward zero - opposite to the parameter names
  _regenSmoother = newTemporalSmoothing(RGN.RELEASE_NM_S, RGN.ENGAGE_NM_S)
  publishDefaults()

  ctx.useNewBatteryMonitor = (jbeamData and jbeamData.useNewBatteryMonitor)
  if ctx.useNewBatteryMonitor == nil then ctx.useNewBatteryMonitor = true end

  if ctx.useNewBatteryMonitor then
    ctx.modules.battery = safeRequire("controller/etk854hybrid/batteryMonitor", "batteryMonitor")
    if ctx.modules.battery.init then ctx.modules.battery.init(ctx) end
  end
  ctx.modules.curves = safeRequire("controller/etk854hybrid/powerCurves", "powerCurves")
  if type(ctx.modules.curves.iceMaxTorqueAtRpm) ~= "function" then
    log("E", "etk854hybrid.boot", "powerCurves unusable - using the built-in fallback envelope")
    ctx.modules.curves = CURVES_FALLBACK
  end
  if ctx.modules.curves.init then ctx.modules.curves.init(ctx) end
  ctx.modules.sailing = safeRequire("controller/etk854hybrid/sailing", "sailing")
  if ctx.modules.sailing.init then ctx.modules.sailing.init(ctx) end

  ctx.stageRegenKw = stageCfg().regenKw
  ctx.iceSeqPassthrough = (readNumber(jbeamData, "iceSeqPassthrough", 0) ~= 0)
  ctx.modules.sportIdx = safeRequire("controller/etk854hybrid/etk854hybrid_sportIndex", "sportIndex")
  if ctx.modules.sportIdx.init then ctx.modules.sportIdx.init(ctx) end
  ctx.modules.torqueCoord = safeRequire("controller/etk854hybrid/etk854hybrid_torqueCoord", "torqueCoord")
  if ctx.modules.torqueCoord.init then ctx.modules.torqueCoord.init(ctx) end
  ctx.modules.iceSeq = safeRequire("controller/etk854hybrid/etk854hybrid_iceSequencer", "iceSequencer")
  if ctx.modules.iceSeq.init then ctx.modules.iceSeq.init(ctx) end
  ctx.seqFailsafe = not (ctx.modules.iceSeq.isReal == true and ctx.modules.torqueCoord.isReal == true)
  if ctx.seqFailsafe then
    log("E", "etk854hybrid.boot", "ICE sequencer/torqueCoord unavailable - failsafe ICE on, K0 closed")
  end
  -- launch control after the coordinator (it uses its throttle inverse)
  ctx.modules.launch = safeRequire("controller/etk854hybrid/etk854hybrid_launchControl", "launchControl")
  if ctx.modules.launch.init then ctx.modules.launch.init(ctx) end
  _lcWheelsFn = (type(ctx.modules.launch.wheelsIntermediate) == "function")
                and ctx.modules.launch.wheelsIntermediate or nil
  -- the creep floor itself runs in the motor device; a failed load = no EV creep
  ctx.modules.creep = safeRequire("controller/etk854hybrid/etk854hybrid_creep", "creep")
  if ctx.modules.creep.init then ctx.modules.creep.init(ctx) end

  local s = stageCfg()
  log("I", "etk854hybrid", string.format(
    "ActiveHybrid VCU init | mode=%s | boostStage=%d (%s) (%dNm/%dkW batt=%dkW regen=%dkW) regenStrength=%.2f sailing=%d targetSoc=%.2f sysTqCap=%.0fNm (base %.0f [%s] + stage%d %+.0f)",
    modeLabel(), ctx.boostStage, ctx.boostStageOrigin, s.torqueNm, s.powerKw,
    s.battKw, s.regenKw, ctx.regenStrength,
    ctx.sailingEnabled, ctx.batteryTargetSoc, ctx.systemTorqueCapNm,
    ctx.systemTorqueCapBaseNm, ctx.systemTorqueCapOrigin,
    ctx.boostStage, (s.capBonusNm or 0)))
  local eco = DE_CFG.ECO_PRO
  log("I", "etk854hybrid", string.format(
    "ICE policy | EV pedal zone COMFORT %.0f%% ECO %.0f%% (join %.3f, kick %.3f, urgent %.2f/%.2fs) | kick COMFORT %.2f | EV top %d/%d km/h | rate-urgent %.1f/s",
    DE_CFG.COMFORT.pEv * 100, eco.pEv * 100, eco.pJoin or eco.pEv, eco.kickJoin or eco.kickAuto,
    eco.urgentPedal or ICEPOL.URGENT_PEDAL, eco.urgentDwellS or 0, DE_CFG.COMFORT.kickAuto,
    DE_CFG.COMFORT.evTopAuto, eco.evTopAuto, ICEPOL.RATE_URGENT))
end

function M.initSecondStage(jbeamData)
  _iceDevice = powertrain.getDevice("mainEngine")
  _motorDevice = powertrain.getDevice("integratedMotor")
  readVehicleRefs()

  if ctx.modules.curves and ctx.modules.curves.initStage2 then
    ctx.modules.curves.initStage2(ctx)
  end
  if ctx.useNewBatteryMonitor and ctx.modules.battery and ctx.modules.battery.initStage2 then
    ctx.modules.battery.initStage2(ctx)
  end
  if ctx.modules.sailing and ctx.modules.sailing.initStage2 then
    ctx.modules.sailing.initStage2(ctx)
  end
  -- after powerCurves: the sequencer reads idle/max rpm from it
  for _, key in ipairs({ "sportIdx", "torqueCoord", "iceSeq", "launch", "creep" }) do
    local mod = ctx.modules[key]
    if mod and mod.initStage2 then mod.initStage2(ctx) end
  end
end

-- ===========================================================
-- Failsafe drive (sequencer or coordinator failed to load)
-- ===========================================================
-- ICE running, K0 closed, motor-first split; writes K0 only while the sequencer is missing
local function failsafeDrive(torqueReqNm, motorCapNm, motorCurveNm, rpmIce, iceIdle)
  local ev = electrics.values
  if not ctx.failsafeInit then
    ctx.failsafeInit = true
    if controller.mainController then controller.mainController.setEngineIgnition(true) end
  end
  if _iceDevice and not _iceDevice.isDisabled then _iceDevice.starterDisabled = false end
  ev.etk854hybrid_k0ClutchRatio = 1
  local req = max(0, torqueReqNm)
  local capSys = ctx.systemTorqueCapNm or 0
  if capSys > 0 and req > capSys then req = capSys end
  local motorNm = min(req, motorCapNm)
  local iceNm = (req - motorNm) * (ctx.reverseLimit or 1)
  ev.etk854hybrid_evThrottle = clamp01(motorNm / max(1, motorCurveNm))
  ev.etk854hybrid_iceThrottle = clamp01(iceNm / max(50, iceCapAtRpm(max(rpmIce, iceIdle))))
  ev.etk854hybrid_iceEngageState = "ON"
  ev.etk854hybrid_evTorqueAvailNm = motorCapNm
  ev.etk854hybrid_fillGain = 0
  ctx.iceEngageState, ctx.iceOn, ctx.iceCoupled = "ON", true, true
  ctx.torqueTargetNm, ctx.motorBaseNm, ctx.iceExpNm = req, motorNm, iceNm
  ctx.chargingActive, ctx.iceDragNm, ctx.fillSlipState = false, 0, false
end

-- ===========================================================
-- BATTERY CONTROL charging (own function: updateGFX upvalue budget)
-- ===========================================================
-- Raises the ICE load point and charges the surplus. Only with the ICE ON:
-- the throttle overrides would fight the sequencer's SYNC / ENGAGE speed
-- control, and a charge load on a slipping K0 is pointless.
local function updateBatteryControl(dt, edriveCfgNow, soc, rpmIce, speedKmh, speedMs, pedalRaw,
                                    gearSign, regenFrac, regenEnvelopeNm, iceIdle, iceCeil)
  local ev = electrics.values
  local chargePrevNm = ctx.chargeNmLast or 0   -- load-point shift inside this frame's iceExpNm
  ctx.chargeNmLast = 0
  if not edriveCfgNow.chargeToTarget then
    _nChargeRpmSetpoint = 0
    ev.etk854hybrid_chargeMode = "OFF"
    ctx.chargeModeLast = "OFF"
    _chargeRegenFrac = 0
    return
  end
  local doCharge = (soc < ctx.batteryTargetSoc) and (ctx.iceEngageState == "ON") and not ctx.tsaFreeze
  if not doCharge then
    _nChargeRpmSetpoint = 0
    ev.etk854hybrid_chargeMode = "OFF"
    ctx.chargeModeLast = "OFF"
    _chargeRegenFrac = _chargeRegenFrac * exp(-dt / CHG.OFF_DECAY_TAU_S)
    if _chargeRegenFrac < 0.01 then _chargeRegenFrac = 0 end
    if _chargeRegenFrac > regenFrac then
      ev.etk854hybrid_evRegen = _chargeRegenFrac
    end
    return
  end

  local iceAvNow = rpmIce * RPM_TO_AV
  local iceMaxAtRpm = iceCapAtRpm(max(rpmIce, iceIdle))

  -- Headroom = ICE curve - drive load. iceExpNm already holds last frame's
  -- charge shift, which is removed (else it settles at GAIN/(1+GAIN)); a
  -- measured shaft torque would carry driveline ringing.
  local iceActualT = max(0, (ctx.iceExpNm or 0) - chargePrevNm)

  local headroom = max(0, iceMaxAtRpm - iceActualT)
  local targetRegenNm = headroom * CHG.GAIN
  if iceAvNow > 2 then
    targetRegenNm = min(targetRegenNm, (CHG.MAX_KW * 1000) / iceAvNow)
  end
  local targetRegenFrac = clamp01(targetRegenNm / max(1, regenEnvelopeNm))

  local gearVal = ev.gear
  local isNeutral = (gearVal == "N")
  -- max-rate charging in N (engine ON and SOC already checked in doCharge)
  local nMaxCharge = isNeutral
                 and speedKmh < NCHG.MAX_KMH
                 and pedalRaw < NCHG.ABORT_THR
  -- Park-Charge in P only: an aborted N charge relaxes instead
  local isParked = (not nMaxCharge) and not isNeutral
                   and ((gearVal == "P") or (gearSign == 0 and speedMs < 0.5))

  if nMaxCharge then
    -- rpm setpoint: where the flat regen torque meets the stage power budget
    local stageRegenKw = stageCfg().regenKw
    local c = curves()
    local plateauNm = c and c.motorRegenTorqueAtRpm(NCHG.RPM_MAX) or RGN.FALLBACK_NM
    local rpmForBudget = (plateauNm > 1)
      and (stageRegenKw * 1000 / plateauNm) * AV_TO_RPM
      or NCHG.RPM_MAX
    local rpmTarget = clamp(rpmForBudget, NCHG.RPM_MIN,
                            min(NCHG.RPM_MAX, iceCeil * NCHG.RPM_MAXFRAC))

    if _nChargeRpmSetpoint < iceIdle then
      _nChargeRpmSetpoint = max(rpmIce, iceIdle)
    end
    if _nChargeRpmSetpoint < rpmTarget then
      _nChargeRpmSetpoint = min(rpmTarget, _nChargeRpmSetpoint + NCHG.RPM_RAMP * dt)
    else
      _nChargeRpmSetpoint = max(rpmTarget, _nChargeRpmSetpoint - NCHG.RPM_RAMP * dt)
    end

    -- regen torque: min(regen envelope, ICE curve share, MAX_NM, stage power
    -- budget) x charge acceptance. The ICE only has MAX_THR of its curve to
    -- carry it, else the shaft is dragged through 0 rpm.
    local iceCapNow = max(1, iceMaxAtRpm)
    local loadFrac = min(NCHG.ICE_LOAD_FRAC, NCHG.MAX_THR * NCHG.THR_MARGIN)
    local regenAllowNm = min(regenEnvelopeNm, iceCapNow * loadFrac, NCHG.MAX_NM)
    local avNow = max(1, rpmIce * RPM_TO_AV)
    regenAllowNm = min(regenAllowNm, stageRegenKw * 1000 / avNow)
    regenAllowNm = regenAllowNm * (ctx.regenAcceptFraction or 1.0)

    local targetFrac = clamp01(regenAllowNm / max(1, regenEnvelopeNm))
    -- Engage gate with hysteresis: load only a healthy engine, drop it when it
    -- sags; a dragged-down or backwards engine goes to the sequencer's
    -- emergency path (K0 open, starter restart).
    local sagging = rpmIce < iceIdle * NCHG.STALL_FRAC
    if not _nChargeEngaged then
      if rpmIce > iceIdle * NCHG.ENGAGE_FRAC then
        _nChargeEngaged = true
      else
        targetFrac = 0
      end
    elseif sagging then
      _nChargeEngaged = false
    end
    if sagging then
      targetFrac = 0
      _parkChargeRpmIntegral = 0
      if rpmIce < NCHG.RECOVER_RPM then
        local seq = ctx.modules.iceSeq
        if seq and seq.onExternalStall then
          ctx.dbg(1, "charge", string.format(
            "N/Park charge: engine at %.0f rpm - emergency release + restart", rpmIce))
          seq.onExternalStall(ctx)
        end
      end
    end
    if targetFrac > _chargeRegenFrac then
      _chargeRegenFrac = min(targetFrac, _chargeRegenFrac + NCHG.REGEN_RISE * dt)
    else
      local fall = NCHG.REGEN_FALL * (sagging and NCHG.STALL_RELEASE or 1)
      _chargeRegenFrac = max(targetFrac, _chargeRegenFrac - fall * dt)
    end

    -- rpm loop: feed forward (throttle carrying the regen load) + PI trim
    local ffThrottle = clamp01((_chargeRegenFrac * regenEnvelopeNm) / iceCapNow)
    local rpmError = _nChargeRpmSetpoint - rpmIce
    _parkChargeRpmIntegral = clamp(_parkChargeRpmIntegral + rpmError * dt * NCHG.RPM_KI,
                                   0, NCHG.ANTI_WINDUP)
    local nThrottle = clamp(ffThrottle + rpmError * NCHG.RPM_KP + _parkChargeRpmIntegral,
                            0, NCHG.MAX_THR)
    ev.etk854hybrid_iceThrottle = max(ev.etk854hybrid_iceThrottle or 0, nThrottle)

    if ctx.debugLevel >= 2 and (_simTime - _dbgLastLogT) >= HUDC.DBG_INTERVAL then
      _dbgLastLogT = _simTime
      log("I", "etk854hybrid.charge", string.format(
        "[NCHG] soc=%.4f tgt=%.2f rpm=%.0f/%.0f (goal %.0f) regen=%.2f (%.0fNm) thr=%.2f",
        soc, ctx.batteryTargetSoc, rpmIce, _nChargeRpmSetpoint, rpmTarget,
        _chargeRegenFrac, _chargeRegenFrac * regenEnvelopeNm, nThrottle))
    end
  elseif isParked then
    -- Park-Charge: fixed load, PI to PARK_RPM (the idle governor fights open-loop throttle)
    _nChargeRpmSetpoint = 0
    _chargeRegenFrac = clamp01(CHG.PARK_TORQUE_NM / max(1, regenEnvelopeNm))
    local rpmError = CHG.PARK_RPM - rpmIce
    _parkChargeRpmIntegral = _parkChargeRpmIntegral + rpmError * dt * CHG.PARK_RPM_KI
    _parkChargeRpmIntegral = clamp(_parkChargeRpmIntegral, 0, CHG.PARK_ANTI_WINDUP)
    local parkPiThrottle = clamp(rpmError * CHG.PARK_RPM_KP + _parkChargeRpmIntegral,
                                 0, CHG.PARK_RPM_MAX_THR)
    ev.etk854hybrid_iceThrottle = max(ev.etk854hybrid_iceThrottle or 0, parkPiThrottle)
  else
    _nChargeRpmSetpoint = 0
    local tau = (targetRegenFrac > _chargeRegenFrac) and CHG.CRUISE_RISE_TAU_S or CHG.CRUISE_FALL_TAU_S
    _chargeRegenFrac = _chargeRegenFrac + (1 - exp(-dt / tau)) * (targetRegenFrac - _chargeRegenFrac)
    _parkChargeRpmIntegral = _parkChargeRpmIntegral * exp(-dt / CHG.PARK_I_DECAY_TAU_S)
    if _parkChargeRpmIntegral < 0.001 then _parkChargeRpmIntegral = 0 end
    -- load-point shift: torqueCoord adds this to the ICE share next frame
    ctx.chargeNmLast = _chargeRegenFrac * regenEnvelopeNm
  end
  local mode = nMaxCharge and "NEUTRAL_MAX" or (isParked and "PARK" or "CRUISE")
  ev.etk854hybrid_chargeMode = mode
  ctx.chargeModeLast = mode

  -- driver regen wins if larger
  local combined = max(regenFrac, _chargeRegenFrac)
  ev.etk854hybrid_evRegen = combined
  -- keep the regen rate limiter (Nm) in step
  if _regenSmoother then _regenSmoother:set(combined * regenEnvelopeNm) end

  if ctx.debugLevel >= 2 and (_simTime - _dbgLastLogT) >= HUDC.DBG_INTERVAL then
    _dbgLastLogT = _simTime
    log("I", "etk854hybrid.charge", string.format(
      "[CHG] parked=%s soc=%.4f tgt=%.2f headroom=%.1f regen=%.3f iceRpm=%.0f",
      tostring(isParked), soc, ctx.batteryTargetSoc, headroom, _chargeRegenFrac, rpmIce))
  end
end

-- ===========================================================
-- HUD channels (20 Hz; own function: updateGFX upvalue budget)
-- ===========================================================
local function updateHud(dt, torqueReqNm, motorCapNm, icePotentialNm, regenActive)
  ctx.hudTimer = ctx.hudTimer + dt
  if ctx.hudTimer < HUDC.INTERVAL then return end
  dt = ctx.hudTimer   -- display filters integrate the time since the last HUD tick
  ctx.hudTimer = 0
  local ev = electrics.values
  ev.etk854hybrid_mode         = modeLabel()
  ev.etk854hybrid_deMode       = ctx.deMode
  ev.etk854hybrid_edriveMode   = ctx.edriveMode
  ev.etk854hybrid_modeLegacy   = ctx.legacyMode
  ev.etk854hybrid_iceState     = ctx.iceOn and 1 or 0   -- crank .. release
  ev.etk854hybrid_regenLevel   = ctx.regenLevelOrder[ctx.regenLevelIdx]
  -- deferred DSC coupling, retried until the DSE is ready
  if ctx.dscPending and ctx.applyDsc then
    if ctx.applyDsc(ctx.deMode) then ctx.dscPending = false end
  end
  -- read back: the driver can toggle DSC with its button / radial menu
  local dmc = controller.getController and controller.getController("driveModes")
  if dmc and dmc.getCurrentDriveModeKey then
    ctx.dscKey = dmc.getCurrentDriveModeKey() or ctx.dscKey
  end
  ev.etk854hybrid_dscMode = ctx.dscLabels[ctx.dscKey] or "DSC"
  ev.etk854hybrid_tcActive = (flag(ev.escActive) or flag(ev.tcsActive)) and 1 or 0
  ev.etk854hybrid_boostStage   = ctx.boostStage
  ev.etk854hybrid_targetSoc    = ctx.batteryTargetSoc

  ev.etk854hybrid_torqueRequestNm = torqueReqNm   -- pedal-map channels: mapPedal
  ev.etk854hybrid_evTorqueCapNm   = motorCapNm
  -- commanded split (torqueCoord)
  ev.etk854hybrid_evTorqueNm      = ctx.motorBaseNm or 0
  ev.etk854hybrid_iceTorqueNm     = ctx.iceExpNm or 0
  ev.etk854hybrid_iceMaxTorqueNm  = icePotentialNm
  -- jerk-limited total after the system cap
  ev.etk854hybrid_sysTorqueNm     = ctx.torqueTargetNm or 0
  -- effective cap (0 = off) for the 8HP, whose own fallback can differ
  ev.etk854hybrid_sysTorqueCapNm  = ctx.systemTorqueCapNm or 0
  ev.etk854hybrid_regenTorqueNm   = _rgn.motorNm
  ev.etk854hybrid_regenRatio      = _rgn.ratio
  ev.etk854hybrid_torqueTargetNm  = ctx.torqueTargetNm or 0
  ev.etk854hybrid_k0TorqueNm      = ctx.k0TorqueNm or 0
  ev.etk854hybrid_k0SlipRpm       = ctx.k0SlipRpm or 0
  ev.etk854hybrid_fillNm          = (_motorDevice and _motorDevice.etkFillNm) or 0
  ev.etk854hybrid_shaftTorqueNm   = (_motorDevice and _motorDevice.etkShaftNm) or 0
  ev.etk854hybrid_iceTorqueEstNm  = ctx.iceMeasNm or 0

  if not (ctx.useNewBatteryMonitor and ctx.modules.battery) then
    local battery = energyStorage.getStorage("mainBattery")
    ev.etk854hybrid_batterySoc = battery and (battery.remainingRatio * 100) or 0
  end

  -- HYBRID / ICE only once K0 carries the engine
  local phase
  if ctx.sailActive then phase = "SAIL"
  elseif ctx.chargingActive then phase = "CHARGE"
  elseif regenActive then phase = "REGEN"
  elseif ctx.iceCoupled and (ctx.motorBaseNm or 0) > 1 then phase = "HYBRID"
  elseif ctx.iceCoupled then phase = "ICE"
  else phase = "EV" end
  ev.etk854hybrid_phase = phase

  -- ICE power, dt-corrected EMA (a per-frame alpha barely filters at 240 fps)
  local rawIcePowerKw = 0
  if _iceDevice and ctx.iceOn and not (_iceDevice.isStalled == true) then
    rawIcePowerKw = ((_iceDevice.outputTorque1 or 0) * (_iceDevice.outputAV1 or 0)) / 1000
    if rawIcePowerKw < 0 then rawIcePowerKw = 0 end
  end
  local aIce = dt / (HUDC.ICE_POWER_TAU + dt)
  _icePowerEma = _icePowerEma + aIce * (rawIcePowerKw - _icePowerEma)
  if not ctx.iceOn then _icePowerEma = 0 end
  ev.etk854hybrid_icePower = _icePowerEma

  -- EV power (sign: + drive, - regen)
  local rawEvPowerKw = 0
  if _motorDevice then
    local motorT = _motorDevice.motorTorque or _motorDevice.outputTorque1 or 0
    rawEvPowerKw = (motorT * (_motorDevice.outputAV1 or 0)) / 1000
  end
  local aEv = dt / (HUDC.EV_POWER_TAU + dt)
  _evPowerEma = _evPowerEma + aEv * (rawEvPowerKw - _evPowerEma)
  ev.etk854hybrid_evPower = _evPowerEma
end

-- ===========================================================
-- Main update loop (per graphics frame)
-- ===========================================================
-- Order: inputs -> battery -> motor envelope -> ceiling + pedal map -> sport
-- index -> policy -> sailing -> ICE arbiter + sequencer -> launch -> torque
-- coordinator -> EV creep -> recuperation -> BATTERY CONTROL -> launch hold
-- outputs -> HUD.
-- Reach new functionality through ctx.modules: more than 60 upvalues is a
-- LuaJIT load error that disables the controller.
function M.updateGFX(dt)
  if not ctx.controllersReady then ctx.controllersReady = true; ctx.dscPending = true end
  _simTime = _simTime + dt
  local ev = electrics.values
  local mods = ctx.modules

  local deCfgNow     = DE_CFG[ctx.deMode] or DE_CFG.COMFORT
  local edriveCfgNow = EDRIVE_CFG[ctx.edriveMode] or EDRIVE_CFG.AUTO_EDRIVE

  local pedalRaw = ev.throttle or 0
  local brakeRaw = ev.brake or 0
  local speedMs  = abs(ev.wheelspeed or 0)
  local speedKmh = speedMs * 3.6
  local rpmIce   = ev.rpm or 0
  local gearSign = getGearSign()
  local shifting = flag(ev.isShifting)

  -- Demand pedal: automaticGearbox caps the throttle during upshifts, which
  -- the demand limiter would ramp slowly. Use throttle_input and re-apply
  -- the cap as a fast factor (ctx.kShift); arcade reverse has throttle_input 0.
  -- Policy, sailing, launch keep pedalRaw.
  local pedalIn, kShiftFb = pedalRaw, 1
  local thrIn = ev.throttle_input or 0
  if shifting and thrIn > pedalRaw + 0.01 then
    pedalIn = thrIn
    kShiftFb = pedalRaw / thrIn
  end
  local kP2 = ev.etk854hybrid_shiftTorqueFactor     -- optional, P2 shift logic
  -- else the 8HP upshift torque envelope (1 outside a shift)
  if type(kP2) ~= "number" then kP2 = ev.etk854hybrid_shiftTorqueCoef end
  ctx.pedalIn = pedalIn
  ctx.isShifting = shifting
  ctx.kShift = (type(kP2) == "number") and clamp01(kP2) or kShiftFb

  -- ---------------- batteryMonitor ----------------
  if ctx.useNewBatteryMonitor and mods.battery and mods.battery.updateGFX then
    mods.battery.updateGFX(ctx, dt)
  end
  local soc = ctx.soc
  if soc == nil then
    local battery = energyStorage.getStorage("mainBattery")
    soc = battery and battery.remainingRatio or 1.0
  end

  -- MAX EV is not sustainable on an empty pack
  if ctx.edriveMode == "MAX_EDRIVE" and soc < BATTERY_LOW_THRESHOLD then
    applyModes(ctx.deMode, "AUTO_EDRIVE", false)
    guihooks.message("Battery critical - AUTO EV", 4, "vehicle.hybridmode")
    ctx.dbg(1, "battery", "forced AUTO_EDRIVE from MAX_EDRIVE")
    edriveCfgNow = EDRIVE_CFG.AUTO_EDRIVE
  end

  -- ---------------- Motor envelope ----------------
  local rpmMotor = rpmIce
  if _motorDevice and type(_motorDevice.outputAV1) == "number" then
    rpmMotor = abs(_motorDevice.outputAV1) * AV_TO_RPM
  end
  local motorCapNm, motorCurveNm = motorCapAtRpm(rpmMotor, deCfgNow)
  -- pedal-map knee: the cap before the redline taper (no zero request near the limiter)
  local kneeRawNm = motorCapNm

  -- Redline guard: the ICE limiter only cuts fuel, so with K0 closed the motor
  -- would push the engine past it. Taper the cap over the last ~400 rpm.
  local c = curves()
  local iceIdle = c and c.iceIdleRPM() or 800
  local iceCeil = c and c.iceMaxRPM() or 7000
  if (ev.etk854hybrid_k0ClutchRatio or 0) > 0.2 and rpmIce > iceCeil - 400 then
    motorCapNm = motorCapNm * clamp01((iceCeil - 50 - rpmIce) / 350)
  end
  ctx.rpmMotor, ctx.motorCapNm, ctx.motorCurveNm = rpmMotor, motorCapNm, motorCurveNm

  -- ---------------- Request ceiling + pedal map ----------------
  -- ICE torque once coupled at the shaft speed (else the ceiling collapses to
  -- the motor cap while the ICE is off); torqueCoord low-passes the ceiling
  local iceRefRpm = max(rpmIce, min(rpmMotor, iceCeil))
  if iceRefRpm < iceIdle then iceRefRpm = iceIdle end
  local icePotentialNm = iceCapAtRpm(iceRefRpm)
  -- also keeps sailing's exits from restarting the ICE
  ctx.evOnly = (edriveCfgNow.evOnly == true) and not (deCfgNow.iceAlwaysOn or ctx.legacyIceForce)
  local ceilRaw = motorCapNm + icePotentialNm
  -- MAX EV: full pedal travel on the E-machine
  if ctx.evOnly and ctx.iceEngageState == "OFF" then ceilRaw = motorCapNm end
  local tc = mods.torqueCoord
  local ceilF = (tc and tc.filterCeiling) and tc.filterCeiling(ctx, dt, ceilRaw) or ceilRaw
  local torqueDemNm, pedalShaped = mapPedal(dt, pedalIn, deCfgNow, edriveCfgNow, kneeRawNm, ceilF, shifting)
  torqueDemNm = demandComp(dt, torqueDemNm, pedalIn, deCfgNow, rpmMotor, gearSign)
  local torqueReqNm = (gearSign ~= 0) and torqueDemNm or 0

  -- ---------------- shared context ----------------
  ctx.deModeCfg      = deCfgNow
  ctx.edriveCfg      = edriveCfgNow
  ctx.chargeToTarget = (edriveCfgNow.chargeToTarget == true)
  ctx.throttleRaw    = pedalRaw
  ctx.brakeRaw       = brakeRaw
  ctx.speedKmh       = speedKmh
  ctx.gearSign       = gearSign
  ctx.torqueReqNm    = torqueReqNm
  ctx.pedalShaped    = pedalShaped
  ctx.soc            = soc
  ctx.iceRpm         = rpmIce
  ctx.iceIdleRPM     = iceIdle
  ctx.k0Current      = ev.etk854hybrid_k0ClutchRatio or 1.0
  -- trailer stability requests
  local tsm = ev.etk854hybrid_tsaTorqueMode
  ctx.tsaTorqueMode = (tsm == 1 or tsm == 2) and tsm or 0
  ctx.tsaFreeze = flag(ev.etk854hybrid_tsaFreeze)
  local tsr = tonumber(ev.etk854hybrid_tsaTorqueRec) or 0
  ctx.tsaTorqueRec = (tsr > 0 and tsr < 1e4) and tsr or 0

  -- ---------------- sportiness -> policy -> sailing ----------------
  if mods.sportIdx and mods.sportIdx.updateGFX then mods.sportIdx.updateGFX(ctx, dt) end
  -- K0 start possible now? (K0 lead, arbiter debounce)
  local seqM = mods.iceSeq
  ctx.iceK0Window = (not ctx.seqFailsafe) and seqM ~= nil and seqM.k0WindowOpen ~= nil
                    and seqM.k0WindowOpen(ctx) or false
  ctx.policyWantsIce = policyWantsIce(deCfgNow, edriveCfgNow, ctx.iceReq == true,
                                      pedalRaw, speedKmh, torqueReqNm, motorCapNm, soc, dt)
  ev.etk854hybrid_icePolicyReason = ctx.icePolicyReason or "none"
  ev.etk854hybrid_iceDeficitS     = ctx.iceDef or 0
  ev.etk854hybrid_iceDeficitOnS   = ctx.iceDefOn or 0
  ev.etk854hybrid_policySportS    = ctx.policySportS or 0   -- Sst
  if mods.sailing and mods.sailing.updateGFX then mods.sailing.updateGFX(ctx, dt) end

  -- ---------------- ICE request + start/stop sequence ----------------
  if ctx.seqFailsafe then
    failsafeDrive(torqueReqNm, motorCapNm, motorCurveNm, rpmIce, iceIdle)
  else
    mods.iceSeq.arbitrate(ctx, dt)
    mods.iceSeq.updateGFX(ctx, dt)
  end

  -- ---------------- Launch Control ----------------
  -- sets the ctx.launch* flags the coordinator reads
  local lcm = mods.launch
  if lcm and lcm.updateGFX then lcm.updateGFX(ctx, dt) end

  -- ---------------- Drive torque (jerk limit, split, throttles, fill) ------
  if not ctx.seqFailsafe then mods.torqueCoord.updateGFX(ctx, dt) end

  -- ---------------- EV creep (virtual idle) ----------------
  -- before the recuperation (ctx.creepIdleRpm speed fade)
  if mods.creep and mods.creep.updateGFX then mods.creep.updateGFX(ctx, dt) end

  -- ---------------- Recuperation ----------------
  local regenFrac, regenEnvelopeNm, regenActive =
    updateRecuperation(dt, pedalRaw, brakeRaw, speedKmh, rpmMotor, gearSign, deCfgNow)

  -- Anti-kickback: the device applies regen only at zero drive throttle. The
  -- fill loop stays in the K0 slip states (it only cancels K0 torque there).
  if regenActive then
    ev.etk854hybrid_evThrottle = 0
    if not ctx.fillSlipState then ev.etk854hybrid_fillGain = 0 end
  end
  -- Drive hold for torqueCoord next frame, driver regen only: the cruise
  -- charge keeps regenActive true and must not hold the drive torque at 0.
  local chargeOnly = (ctx.chargeModeLast == "CRUISE") and (_rgn.targetDecel or 0) <= 0
  ctx.regenActiveLast = regenActive and not chargeOnly

  -- ---------------- BATTERY CONTROL charging ----------------
  updateBatteryControl(dt, edriveCfgNow, soc, rpmIce, speedKmh, speedMs, pedalRaw,
                       gearSign, regenFrac, regenEnvelopeNm, iceIdle, iceCeil)

  -- ---------------- Launch hold outputs (last writer) ----------------
  -- the launch generator load must win over every evRegen writer above
  if lcm and lcm.applyOutputs then lcm.applyOutputs(ctx, dt) end

  -- ---------------- HUD channels (20 Hz) ----------------
  updateHud(dt, torqueReqNm, motorCapNm, icePotentialNm, regenActive)
end

-- ===========================================================
-- Physics-rate hook
-- ===========================================================
-- Runs between powertrain.update and wheels.updateWheelTorques; used only by
-- the launch control's DSC hold assist.
function M.updateWheelsIntermediate(dt)
  if _lcWheelsFn then _lcWheelsFn(dt) end
end

return M
