-- etk854hybridControl.lua - ETK 854 ActiveHybrid v2 - Master VCU
--
-- Production-style P2 hybrid supervisor. Owns:
--   * two orthogonal mode axes (Driving Experience x EV mode), like the series car
--   * pedal shaping per Driving Experience mode
--   * the ICE on/off POLICY (thresholds scaled by the driver sportiness index)
--   * configurable E-boost stages ($ebooststage) with torque AND power clamps
--   * gear-aware recuperation with $regenstrength scaling
--   * BATTERY CONTROL charging (cruise headroom + Park-Charge RPM PI)
--   * the etk854hybrid_* electrics channel contract (HUD + QA)
--
-- Submodules (controller/etk854hybrid/*, loaded via safeRequire + _stub):
--   batteryMonitor  SOC bands, EV lockout, cold derate  (unchanged)
--   powerCurves     real ICE/motor/regen envelopes from the powertrain devices
--   sailing         DRIVE -> SAIL_ENTER -> SAIL -> SAIL_EXIT coasting FSM
--   etk854hybrid_sportIndex   driver sportiness index (0..1)
--   etk854hybrid_iceSequencer ICE request arbiter + start/stop FSM: crank,
--                             rev-match with K0 OPEN, K0 engage, torque blend,
--                             release. SOLE owner of K0, ignition, starter.
--   etk854hybrid_torqueCoord  jerk-limited drive torque, ICE/motor split,
--                             exact ICE throttle inverse, motor fill channels
--   etk854hybrid_launchControl launch control (arm / production-style auto-arm, converter
--                             preload against the brake with the E-machine
--                             as a generator load, DSC hold assist, FIRE
--                             window) + brake-throttle guard in every mode
--   etk854hybrid_creep        EV creep / virtual idle: arms the motor device's
--                             idle-speed floor (creepFloor) on the converter
--                             pump, standstill HOLD, ctx.creepIdleRpm for the
--                             recuperation fade, ctx.creepAvail for the policy
-- If the sequencer or the coordinator fails to load, ctx.seqFailsafe keeps
-- the car drivable with the ICE on and K0 closed (failsafeDrive below).

local M = {}

local floor, abs, max, min, exp = math.floor, math.abs, math.max, math.min, math.exp
local RPM_TO_AV = 0.104719755

-- ===========================================================
-- Master Context (shared with submodules)
-- ===========================================================
local ctx = {
  jbeamData = nil,
  debugLevel = 1,

  -- Mode axis 1: Driving Experience Control
  deMode = "COMFORT",
  deIdx = 1,
  -- Mode axis 2: EV mode strategy
  edriveMode = "AUTO_EDRIVE",
  edriveIdx = 1,
  -- Legacy 5-mode compatibility (HUD / QA / input actions)
  legacyMode = "AUTO",
  legacyIdx = 1,
  legacyIceForce = false,     -- set only by the legacy "HYBRID" preset

  -- Stability system (DSE drive-mode axis, part etk854hybrid_DSE_drivemodes).
  -- dscKey mirrors the driveModes controller's current key ("dsc"/"dtc"/"off");
  -- dscPending re-arms the coupling when the mode was set before the DSE
  -- controllers existed (controller init order).
  dscKey = "dsc",
  dscPending = false,

  hudTimer = 0,
  iceOnLastFrame = true,
  -- arbitrated ICE request (etk854hybrid_iceSequencer.arbitrate); the policy
  -- uses it as its hysteresis state
  iceReq = true,

  -- Recuperation selector. The index points into regenLevelOrder; the
  -- deceleration each level asks for lives in RGN.LEVEL further down.
  -- 3 = MED on purpose: enough lift-off recuperation to be felt without the
  -- one-pedal creep-to-stop of HIGH. OFF is the "sailing" position.
  regenLevelIdx = 3,
  regenLevelOrder = { "OFF", "LOW", "MED", "HIGH" },
  regenAllowsSail = false,

  -- Launch control flags, written by etk854hybrid_launchControl every frame
  -- (launchArmed/Ready/Active: iceSequencer forces the ICE on; launchHold:
  -- torqueCoord hold branch; launchGuardNm: torqueCoord request cap)
  launchArmed = false,
  launchHold = false,

  -- jbeam-variable driven tuning (see etk854hybrid_powertrain.jbeam "variables")
  boostStage = 0,
  -- "part" when a fitted E-Boost stage part pins the stage,
  -- "tuning" when the $ebooststage slider decides.
  boostStageOrigin = "tuning",
  regenStrength = 1.0,
  sailingEnabled = 1,
  -- BATTERY CONTROL target. Default is above the 80 % spawn SOC on purpose so
  -- engaging the mode actually charges (series default is "hold current level",
  -- which would make the mode look broken on a freshly spawned car).
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

-- Pedal map: shaped = pedal ^ gamma.
-- gamma > 1 -> degressive (ECO PRO: same pedal travel asks for less torque),
-- gamma = 1 -> linear (COMFORT), gamma < 1 -> progressive (SPORT: early bite).
local PEDAL_GAMMA_ECO_PRO = 1.40
local PEDAL_GAMMA_COMFORT = 1.00
local PEDAL_GAMMA_SPORT   = 0.70

-- coastScale = per-mode factor on the lift-off recuperation the driver
-- selected (RGN.LEVEL[...].liftDecel). ECO PRO glides a little further, SPORT
-- adds bite so lifting off feels like engine braking.
--
-- dscMode = which calibration of the DSE drive-mode axis this driving
-- experience asks for (part etk854hybrid_DSE_drivemodes, controller
-- "driveModes"):
--   "dsc" = full two-stage stability control (early soft torque trim,
--           late hard single-wheel brake stage)
--   "dtc" = Dynamic Traction Control: more slip and more yaw allowed
--           before either stage bites, but still a real safety net.
-- The driver's own DSC button (drive mode "off") always outranks this, see
-- applyDscForDeMode().
--
-- NOTE: the old boolean `tcOff` is gone. It only ever wrote the electrics
-- channel etk854hybrid_tcActive and never touched the DSE, so "SPORT disables
-- traction control" was pure HUD fiction.
--
-- AUTO EV fields (used only while the EV mode is AUTO EV, see
-- EDRIVE_CFG.deTuned):
--   pEv / gEv / gHi  split pedal map with an EV zone (mapPedal below). Pedal
--              0..pEv asks for 0..E-machine torque (curve exponent gEv), the
--              rest of the travel for E-machine..system ceiling (exponent gHi).
--              pEv comes from the tuning variables $evPedalZoneComfort /
--              $evPedalZoneEco (etk854hybrid_powertrain.jbeam), these are the
--              defaults. The old map pedal^gamma x (motor + ICE potential)
--              saturated the N57's 265 Nm E-machine at 0.30 pedal (0.23 on the
--              N57S, 0.38 on the B58; sweeps 2026-09-30), so the EV range of
--              the pedal depended on the engine and was only its first quarter.
--              SPORT has no pEv: it keeps pedal^0.70 over the full ceiling.
--   kickAuto   pedal-position start threshold (was 0.60 for every mode,
--              EDRIVE_CFG.AUTO_EDRIVE.iceKickdown)
--   evTopAuto  EV speed ceiling [km/h]
-- ECO PRO pedal map v3 ("EV zone + EV MAX + hybrid band", feedback
-- round 3, scratch_build/eco_pedal/ecoMapSpec.md). Gamepad drivers sat in the
-- old 0.55 EV zone only up to half trigger and the kick fired at 0.80 (0.52
-- with a sporty index). Now, at a healthy pack (knee = E-machine cap):
--   0    .. pEv 0.80   T = knee * (p / pEv)^gEv           EV zone (gEv 1.20)
--   pEv  .. pJoin 0.85 T = knee                           EV MAX plateau: full
--                      E-machine torque, works as a pressure point, never
--                      starts the ICE (r = 1.0)
--   pJoin .. 1         T = knee + (top - knee) * ((p - pJoin) / (1 - pJoin))^gHi
--                      hybrid band (gHi 1.00), the ICE starts after the load
--                      integral (2-4 s at 0.86-0.89)
--   kickJoin 0.90      kick (start after the arbiter debounce)
--   urgentPedal 0.95   urgent start after urgentDwellS 0.12 s (a 0.08 s stab
--                      does nothing) + 8HP kickdown (same dwell there)
--   top = min(ceilF, system torque cap) with capTop = 1, so the whole band is
--   usable torque (N57 896 Nm ceiling vs 680 Nm cap: the old band wasted its
--   last 30 %). N57 / B58 at 2000 rpm, stage 0: 0.20 -> 50 Nm, 0.50 -> 151,
--   0.80..0.85 -> 265, 0.90 -> 403, 0.95 -> 542, 1.0 -> 680 Nm.
--   plateau / kickGap  derived thresholds pJoin = pEv + plateau (<= 0.92) and
--              kickJoin = pJoin + kickGap (<= urgentPedal), rounded to 1e-3
--              (deriveSplitThresholds: 0.80 + 0.05 + 0.05 is 0.9000000000000001
--              in floats - a pedal at exactly 0.90 would not be a kick)
--   kneeLowFrac  low-SOC knee anchor as a fraction of the ceiling (replaces
--              pEv^pedalGamma, which moved with the zone: 0.55^1.40 = 0.433)
--   legacyRef  publish etk854hybrid_pedalLegacyEq: the round-2 ECO pedal that
--              asked for the same torque (LEG in _pm), the 8HP's ICE load axis
--   dmdWspan   demandComp fade-in span above pJoin (COMFORT: _pm.DMD_WSPAN)
--   siU0/siU1/siWRate  sport index demand band and rate weight in ECO (the
--              index only rises from presses into the band / kickdown zone);
--              only while the split map is live (ctx.pedalJoin > 0, the band
--              then starts at pJoin - siU0 is kept for the param API only)
--   pEvMin/pEvMax  range of the $evPedalZoneEco slider (PEV_MIN/MAX otherwise)
-- SPORT dmdP0 / dmdWspan: demandComp pedal gate in SPORT (no split map; see
-- demandComp and _pm.DMD_NREF).
-- evCapScale ECO PRO 0.85 -> 1.00: the ECO softness comes from the pedal map,
-- not from a weaker E-machine (0.85 capped it at 225 Nm, so ECO PRO started
-- the ICE EARLIER than COMFORT in absolute torque - cruise_100_eco
-- 2026-09-30: evTorqueAvailNm 225, STARTER start at 15 km/h).
local DE_CFG = {
  COMFORT = { pedalGamma = PEDAL_GAMMA_COMFORT, evCapScale = 1.00, coastScale = 1.00,
              iceAlwaysOn = false, dscMode = "dsc", allowSail = true,
              pEv = 0.45, gEv = 0.90, gHi = 1.00, kickAuto = 0.70, evTopAuto = 110 },
  ECO_PRO = { pedalGamma = PEDAL_GAMMA_ECO_PRO, evCapScale = 1.00, coastScale = 0.90,
              iceAlwaysOn = false, dscMode = "dsc", allowSail = true,
              pEv = 0.80, gEv = 1.20, gHi = 1.00, kickAuto = 0.90, evTopAuto = 120,
              -- pJoin / kickJoin: derived, see deriveSplitThresholds
              plateau = 0.05, kickGap = 0.05, pJoin = 0.85, kickJoin = 0.90,
              urgentPedal = 0.95, urgentDwellS = 0.12, capTop = 1, kneeLowFrac = 0.433,  -- 0.55^1.40
              legacyRef = 1, dmdWspan = 0.10, siU0 = 0.85, siU1 = 0.95, siWRate = 0,
              pEvMin = 0.40, pEvMax = 0.85 },
  SPORT   = { pedalGamma = PEDAL_GAMMA_SPORT,   evCapScale = 1.00, coastScale = 1.20,
              iceAlwaysOn = true,  dscMode = "dtc", allowSail = false,
              kickAuto = 0.60, evTopAuto = 110,
              dmdP0 = 0.10, dmdWspan = 0.10 },
}
-- Tuning-slider range of the EV pedal zone (etk854hybrid_powertrain.jbeam);
-- a DE entry may carry its own pEvMin / pEvMax (ECO PRO 0.40..0.85)
local PEV_MIN, PEV_MAX = 0.30, 0.65

-- Split-map thresholds derived from pEv (init, M.setIcePolicyParam): the
-- EV MAX plateau ends at pJoin = pEv + plateau (at most 0.92), the kick
-- point sits kickGap above it (at most the urgent pedal). Rounded once to
-- 1e-3: summed floats miss exact pedal / QA steps (0.80 + 0.05 + 0.05 =
-- 0.9000000000000001 > 0.90). Entries without a plateau (COMFORT) keep
-- pJoin = nil, i.e. the band starts at pEv as before.
local function deriveSplitThresholds(cfg)
  if not (cfg and cfg.plateau and cfg.pEv) then return end
  cfg.pJoin    = math.floor(math.min(cfg.pEv + cfg.plateau, 0.92) * 1000 + 0.5) / 1000
  cfg.kickJoin = math.floor(math.min(cfg.pJoin + (cfg.kickGap or 0.05), cfg.urgentPedal or 0.95) * 1000 + 0.5) / 1000
end

-- HUD label per DSE drive-mode key.
local DSC_LABEL = { dsc = "DSC", dtc = "DTC", off = "OFF" }

-- ===========================================================
-- Mode axis 2: EV mode strategy
-- ===========================================================
local EDRIVE_ORDER = { "AUTO_EDRIVE", "MAX_EDRIVE", "BATTERY_CONTROL" }
local EDRIVE_SHORT = { AUTO_EDRIVE = "AUTO", MAX_EDRIVE = "MAX", BATTERY_CONTROL = "BATT" }

-- demandStart: may the ICE also be started by pure torque demand (not just by
-- the kickdown threshold)? MAX EV deliberately says no - only kickdown or
-- the 140 km/h ceiling bring the engine in.
-- evTopKmh = 0 for BATTERY CONTROL is deliberate: once the target SOC is
-- reached the mode degrades to "engine carries the car, battery is preserved",
-- which is exactly a zero electric speed ceiling.
-- deTuned: kick threshold, EV speed ceiling and the split pedal map come from
-- the Driving Experience (DE_CFG kickAuto / evTopAuto / pEv); evTopKmh and
-- iceKickdown are only the fallback for a DE entry without them.
local EDRIVE_CFG = {
  AUTO_EDRIVE     = { evTopKmh = 110, iceKickdown = 0.60, demandStart = true,  chargeToTarget = false,
                      deTuned = true },
  -- evOnly: the ICE NEVER starts in this mode - no kickdown trigger, no speed
  -- trigger (user request; the reference hybrid would start it at >95% pedal).
  -- Escape hatch stays: the low-SOC guard force-switches to AUTO_EDRIVE.
  MAX_EDRIVE      = { evTopKmh = 9999, iceKickdown = 2.0, demandStart = false, chargeToTarget = false, evOnly = true },
  BATTERY_CONTROL = { evTopKmh = 0,   iceKickdown = 0.0,  demandStart = true,  chargeToTarget = true  },
}

-- Legacy one-dimensional presets. Kept so the existing input actions, the HUD
-- and the QA suite keep working; each maps onto a (DE, EV mode) pair.
local LEGACY_MODE_ORDER = { "AUTO", "HYBRID", "EV", "HOLD", "SPORT" }
local LEGACY_MODE_MAP = {
  AUTO   = { de = "COMFORT", edrive = "AUTO_EDRIVE" },
  -- HYBRID has no series equivalent; it survives as "AUTO EV with the ICE
  -- pinned on", which is what the QA suite relies on for its ICE-power tests.
  HYBRID = { de = "COMFORT", edrive = "AUTO_EDRIVE", iceForce = true },
  EV     = { de = "COMFORT", edrive = "MAX_EDRIVE" },
  HOLD   = { de = "COMFORT", edrive = "BATTERY_CONTROL" },
  SPORT  = { de = "SPORT",   edrive = "AUTO_EDRIVE" },
}

-- ===========================================================
-- E-Boost stages ($ebooststage)
-- Stage 0 = stock calibration figures. Battery discharge budget scales with the
-- stage so the pack C-rate stays plausible (13.5 kWh -> ~7C at stage 0,
-- ~17C at stage 3, i.e. high-performance PHEV cells).
-- ===========================================================
-- capBonusNm: how much the SYSTEM torque cap (below) grows with the stage.
-- A stage is a hardware package - stronger inverter, stronger cooling and, on
-- the mechanical side, the uprated K0 / lock-up clutch the powertrain jbeam is
-- already rated for. Without this the cap would eat the whole extra motor
-- torque at full load and every stage would feel identical above ~60 % pedal,
-- which is exactly what it did before.
-- Stage caps on the default 680 Nm base: 680 / 780 / 900 / 1050 Nm.
-- Kept as an offset rather than an absolute table so an ECU with a higher base
-- (the 500 PS N57S sets 950) gets the same staircase on top of its own base.
local BOOST_STAGES = {
  [0] = { torqueNm = 265, powerKw =  83, battKw =  95, regenKw = 55, capBonusNm =   0 },
  [1] = { torqueNm = 320, powerKw = 110, battKw = 125, regenKw = 65, capBonusNm = 100 },
  [2] = { torqueNm = 400, powerKw = 150, battKw = 170, regenKw = 80, capBonusNm = 220 },
  [3] = { torqueNm = 500, powerKw = 200, battKw = 225, regenKw = 95, capBonusNm = 370 },
}
local MOTOR_EFFICIENCY = 0.94   -- matches electricalEfficiency in the jbeam

-- ===========================================================
-- System torque cap (PLAN_N57_BITURBO backlog B4 / risk R4)
-- ===========================================================
-- The cap is there for a realistic drive feel, not as gearbox or converter
-- protection: the reference hybrid limits its combined system torque in the
-- calibration, so full load builds up like the real car instead of engine and
-- E-machine simply adding up (N57 630 Nm + stage-0 motor 265 Nm = 895 Nm).
-- (Historically, PLAN_N57_BITURBO R4, it also covered the then 700 Nm lock-up
-- clutch; both clutches have since been uprated, see the end of this block.)
-- Cutting order is the production one: the E-machine share is trimmed first so the
-- combustion engine keeps carrying its base load and the cap is felt as
-- "boost tapers off", not as a torque hole.
--
-- The cap is NOT a single number any more. It is assembled once in M.init as
--     base  +  BOOST_STAGES[stage].capBonusNm
-- where the base comes from (in order of precedence)
--     1. the ECU part's custom dict section  etk854hybridSystemCap.baseNm
--        (the 500 PS N57S needs 950 because its engine alone makes 887 Nm)
--     2. the controller argument  systemTorqueCapNm  in the powertrain jbeam
--     3. this constant
-- Both clutches downstream are rated for the highest resulting cap
-- (etk854hybrid_powertrain.jbeam: K0 1150 Nm, lock-up 1520 Nm), so the VCU is
-- the only limiter in the chain - hardware slip would otherwise silently eat
-- the stages instead of delivering them.
local SYSTEM_TORQUE_CAP_DEFAULT_NM = 680   -- series calibration (reference hybrid figure)

-- ===========================================================
-- ICE engagement thresholds
-- ===========================================================
local SOC_ICE_FORCE          = 0.08   -- below this the ICE runs regardless of mode
-- forces MAX EV back to AUTO EV. Was 0.03, but policy rule 1 starts
-- the ICE at SOC_ICE_FORCE and batteryMonitor locks the E-drive out below
-- 5 %: the ICE then charges the pack and 3 % is never reached - the car
-- drove on the engine with "MAX EV" still displayed (sweep 2026-09-30,
-- MAX EV at 4.5 % SOC: ICE ON, mode MAX_EDRIVE for 45 s). The hand-over
-- now happens where the engine is forced on.
local BATTERY_LOW_THRESHOLD  = SOC_ICE_FORCE
-- Standstill: never idle the ICE at a red light.
local AUTO_STANDSTILL_KMH    = 2.5
local AUTO_STANDSTILL_THR    = 0.04
-- ... and, while the E-machine can creep (ctx.creepAvail, etk854hybrid_creep:
-- D / R, no launch / lockout / one-pedal / arcade gate), never keep it running
-- (coupled or decoupled IDLE) for mere creeping: the same hard-off up to the
-- creep regime's speed (EV creep 5-7 km/h, regen fade band up to ~10 km/h).
local AUTO_CREEP_KMH         = 8.0
-- ... but only with an SOC margin above the forced start (rule 1 has no
-- hysteresis): EV creep drains ~2-3 kW, so at SOC_ICE_FORCE the creep
-- hard-off and the SOC force would start / stop the ICE every few seconds in
-- crawling traffic. Below this SOC the creep regime keeps the pre-round-3
-- rule (only the standstill hard-off) - batteryMonitor LOW1 (10 %).
local AUTO_CREEP_MIN_SOC     = SOC_ICE_FORCE + 0.02

-- Policy rule 5 (P1 spec 4.1), scaled by the sportiness index S.
-- START side in ECO PRO (split map): S x DEF.sStart = 0, i.e. the index no
-- longer moves any ECO start threshold (pedal map v3, feedback round 3: after
-- full-trigger presses S ~0.9 took Don to 0 and the kick to 0.52, inside the
-- EV zone). ctx.policySportS / etk854hybrid_policySportS = the S the start
-- side uses (Sst). The OFF side keeps S (offS, capped at DEF.offSport).
--   kickOn  = kick * (1 - 0.35*rS)     kick = DE_CFG.kickJoin (ECO 0.90) or
--                                      kickAuto (COMFORT 0.70 -> 0.455) in
--                                      AUTO EV
--   kickOff = kick * (0.55 - 0.25*rS)  COMFORT 0.385 at S < 0.25 (split map:
--                                      keep:pedal uses max(kickOff, pEv + 0.03))
--   demandOn  = 1.35 -> dSport (smoothstep over S 0.30..0.70), dSport = the
--              load at which the motor still has the K0 crank torque free
--              (clamped 0.60..1.00). Only the OFF threshold is derived from it:
--   demandOff = min(0.80, demandOn - 0.15)     real hysteresis
-- Demand START (user feedback 2026-10: "only when the load integral says
-- so"). The old start needed nothing but torqueReq > 1.35 x motor cap (1.15 x
-- with the K0 window open) plus a 0.05-0.25 s arbiter debounce: a 3 s tip to
-- 0.45 pedal ran the N57 for 5.3 s (tipin_rolling 2026-09-30). Now, with
-- r = torqueReq / motor cap, per Driving Experience (ICEPOL.DEF):
--   deficit D  leaky integral: r > rMin -> D += (r - 1) * dt,
--              else D -= leak * dt (floor 0). Reset while the ICE request is
--              on, at standstill and on a DE / EV mode change.
--   start      D >= Don(S) = onCalm * (1 - smoothstep((S - 0.15) / 0.45))
--              (Don * 0.3 while the engine still turns: IDLE / RELEASE).
--              COMFORT r 1.3 -> 1.5 s, r 1.5 -> 0.9 s; ECO r 1.5 -> 1.5 s,
--              r 2.0 -> 0.75 s; from S 0.60 on only the arbiter debounce
--              (COMFORT; ECO: S is not used here, sStart 0).
--   immediate  r >= rImm = rHard -> dSport (the sporty path: same smoothstep
--              over S 0.30..0.70 that used to move demandOn 1.35 -> dSport)
--   K0 lead    while a K0 start is still possible (ctx.iceK0Window) a start
--              that the integral would justify within `lead` seconds at the
--              present r is pulled forward: D + (r - 1) * lead >= Don. At S = 0
--              and D = 0 that is r >= 1.6 (COMFORT) / 1.9 (ECO), so a real step
--              tip-in keeps its E-machine start, a moderate pedal does not.
-- Urgent (immediate start + URGENT profile): pedal >= URGENT_PEDAL (fixed;
-- was min(0.90, kickOn + 0.25)). ECO PRO in AUTO EV: DE_CFG.urgentPedal
-- 0.95, held for urgentDwellS 0.12 s (ctx.iceUrgentT) - a gamepad stab to
-- full trigger no longer starts the ICE.
-- Fast pedal (immediate start, ctx.iceStartFast): filtered pedal rate
-- (sportIndex st.pr) >= RATE_URGENT [1/s] at a pedal above (strictly) the
-- Driving Experience's fastPedal, latched until the pedal falls back to it,
-- AND a load ratio r >= fastR (the E-machine is clearly short, not 5-30 %).
-- Per DE because ECO PRO's EV zone ended at 0.55: with the old global 0.60 a
-- quick tip-in / gamepad press to 0.61-0.70 in ECO (r 1.2-1.5, N57) started
-- the ICE at once (review 2026-10). With the split map the latch threshold is
-- at least the kick point (ECO 0.90, map v3). The latch is cleared by every early
-- verdict (soc / charge / pinned / evOnly / standstill) and by a DE / EV mode
-- change, so a pedal held through a mode switch cannot fire it without rate.
-- It skips the load integral
-- and gets the short arbiter debounce, but keeps the Driving Experience start
-- profile: the URGENT profile (4x faster blend, harder crank) would change
-- the feel of every quick tip-in to 0.6-0.85 that the user called good.
-- The rate rule is a parameter on purpose: keyboard throttle ramps at 3/s
-- (input.lua:101 smootherKBD), a gamepad trigger almost steps (controller arg
-- iceRateUrgent, 0 disables it; live: M.setIcePolicyParam).
-- COMFORT rHard 2.0 -> 1.8: 2.0 sat exactly at 0.68 pedal on the N57
-- (265 + 631 * 0.23 / 0.55 = 529 Nm, r 1.996), so a ramp to 0.68 waited
-- ~0.95 s for the integral while a step started at once (fast rule). r 1.8
-- is pedal 0.635 on the N57 COMFORT map.
-- Demand STOP with the split map (2026-10): the engine is kept while the
-- pedal is above the EV zone (pEv + EVZONE_KEEP_MARGIN), near the EV speed
-- ceiling, or while r > rOff (0.95); with the demand back inside the
-- E-machine's range the policy lets it go after offCalm (COMFORT 0.6 s,
-- ECO 0.5 s) rising to DEF.offSport or OFF_SPORT_S (ECO 1.5 s, COMFORT 4 s)
-- with S - at once on a real lift or
-- brake (the arbiter's lift hold decides). Without the split map (SPORT,
-- BATTERY CONTROL, legacy) the old keep rules (kickOff / demandOff) apply.
-- The old 2 s "time since last transition" dwell is gone: the sequencer's
-- arbiter debounces the request and holds the ICE after sporty driving.
local ICEPOL = {
  S_KICK0 = 0.25, S_KICK_SPAN = 0.50,
  KICK_ON_CUT = 0.35, KICK_OFF_BASE = 0.55, KICK_OFF_CUT = 0.25,
  DEMAND_ON_BASE = 1.35, DSPORT_MIN = 0.60, DSPORT_MAX = 1.00, DSPORT_RESERVE_NM = 30,
  S_DEMAND0 = 0.30, S_DEMAND_SPAN = 0.40, DEMAND_OFF_MAX = 0.80, DEMAND_OFF_GAP = 0.15,
  -- 10 -> 5: with 10 the ICE stayed on at 100-110 km/h whatever the demand
  -- (cruise_100 2026-09-30: 0.18 pedal = 0.61 x motor cap, ICE ON for 10 s)
  SPEED_HYST_KMH = 5.0,
  URGENT_PEDAL = 0.85,
  -- 0 = rate rule off (default 2026-10, gamepad / keyboard step the pedal);
  -- controller arg iceRateUrgent, e.g. 2.5 for a pedal / wheel
  RATE_URGENT = 0,
  DEF_S0 = 0.15, DEF_SPAN = 0.45, DEF_TURNING_FRAC = 0.3,
  -- fastPedal / fastR: fast-pedal rule per DE (see above). ECO 0.90 = the
  -- kick point of the map v3 (was 0.72 = r ~1.65 on the old N57 ECO map).
  -- rOff / offCalm: off side of the split map (policyWantsIce currentOn):
  -- demand at or below rOff x motor cap for offCalm s (S ~ 0; offSport or
  -- OFF_SPORT_S at S >= 0.60, same smoothstep as Don) lets the engine go;
  -- the pedal keeps it only above the EV zone + EVZONE_KEEP_MARGIN (ECO
  -- 0.83; r <= rOff 0.95 releases it below ~0.77 pedal).
  -- sStart: factor on S on the START side (nil = 1). ECO 0 = the sport index
  -- never lowers an ECO start threshold (map v3).
  -- offSport: off hold at S >= 0.60 (nil = OFF_SPORT_S).
  EVZONE_KEEP_MARGIN = 0.03, OFF_SPORT_S = 4.0, LIFT_PEDAL = 0.02, LIFT_BRAKE = 0.05,
  DEF = {
    COMFORT = { rMin = 1.05, onCalm = 0.45, rHard = 1.8, leak = 0.5, lead = 0.75,
                fastPedal = 0.60, fastR = 1.2, rOff = 0.95, offCalm = 0.6 },
    -- rHard 2.4 stays: with sStart 0 it is never reached below the urgent
    -- pedal (r 2.04 at 0.95 on the N57)
    ECO_PRO = { rMin = 1.10, onCalm = 0.75, rHard = 2.4, leak = 0.8, lead = 0.83,
                fastPedal = 0.90, fastR = 1.5, rOff = 0.95, offCalm = 0.5,
                offSport = 1.5, sStart = 0 },
  },
}

-- Pedal map runtime state + constants (one table = one upvalue for mapPedal)
--   KNEE_TAU_S  knee LPF, the same 0.25 s and shift freeze as the request
--               ceiling (etk854hybrid_torqueCoord.lua filterCeiling, P.CEIL_TAU_S)
--   HEALTHY0/SPAN  E-machine health from the SOC bands and lockout only
--               (ctx.evPowerFraction divided by the cold derate;
--               batteryMonitor.lua:89-95, :180) below which the knee slides
--               onto the old map's torque at pEv - a low pack (SOC 10-20 %)
--               may start the ICE earlier, a cold one may not (see mapPedal)
local _pm = { kneeF = nil, KNEE_TAU_S = 0.25, HEALTHY0 = 0.5, HEALTHY_SPAN = 0.5,
  -- Driver demand as a wheel-torque reference while the ICE is coupled
  -- (demandComp, feedback 2026-10 round 2). Reference engine speed per fuel
  -- and Driving Experience, max. factor, pedal travel above the EV edge over
  -- which it fades in, share of the 8HP's deliverable torque it may use,
  -- factor slew [1/s]. Runtime: dmdK.
  -- SPORT (lead decision round 3): the round-2 SPORT rows upshift 100-600 rpm
  -- earlier, which cut the wheel torque at a light pedal (N57 0.20 pedal
  -- 20 -> 80 km/h 6.8 -> 8.1 s, B58 7.5 -> 8.9 s, shift_table_sport
  -- 2026-09-30). Reference speeds fitted offline on those recordings
  -- (scratch_build/eco_pedal/sport_dmd_fit.py: replayed 20 -> 80 with the
  -- comp gives N57 6.79 s at 2600, B58 7.50 s at 2950 rpm). From 0.40 pedal
  -- on the SPORT rpm is above them, so k stays ~1 there.
  DMD_ON = true,
  DMD_NREF = { diesel = { COMFORT = 3000, ECO_PRO = 2500, SPORT = 2600 },
               petrol = { COMFORT = 3600, ECO_PRO = 3000, SPORT = 2950 } },
  DMD_KMAX = 1.8, DMD_WSPAN = 0.15, DMD_CAPF = 0.90, DMD_RATE = 3.0,
  dmdK = 1,
  -- round-2 ECO map (pEv 0.55 / gEv 1.10 / gHi 1.30 over the full ceiling):
  -- etk854hybrid_pedalLegacyEq = the pedal that asked for the same torque
  -- there - the 8HP's ICE load axis in ECO (etk854hybrid_8hpCore senseDriver)
  LEG = { pEv = 0.55, gEv = 1.10, gHi = 1.30 },
  -- band top of the last frame (min(ceilF, system cap) with capTop, else ceilF)
  top = 0 }

-- ===========================================================
-- Recuperation  (driver selectable: OFF / LOW / MED / HIGH)
-- ===========================================================
-- Everything here is specified as a TARGET DECELERATION at the wheels [m/s^2]
-- and converted into motor torque with the real mechanics:
--
--     motorNm = decel * mass * wheelRadius / (mechRatio * DRIVE_EFF)
--
-- The previous model asked for a fixed 180 Nm at the motor and scaled it with
-- an inverse-gear heuristic. It produced ~0.02 g on lift-off - nothing a driver
-- can feel - and LOW/MED/HIGH scaled exactly that nothing. Three independent
-- problems had to be fixed before recuperation reached the road at all:
--
--  1. RATIO. The heuristic scaled with the motor's cumulativeGearRatio, which
--     INCLUDES the converter stall ratio (1.7). While coasting the converter's
--     torque ratio is 1.0, so every command came out ~1.7x too small. We now
--     take the GEARBOX's cumulativeGearRatio (gear x final drive) - the true
--     mechanical path from the converter output to the wheels.
--  2. CONVERTER. The calibration does not lock the converter below gear 3
--     (torqueConverterLockupMinGear). An open converter transmits coast torque
--     as kFactor * inputAV^2, which saturates around 60-80 Nm at coast-down
--     pump speeds no matter what the E-machine does - so below ~40 km/h the
--     regen torque simply never arrived at the wheels. That is why one-pedal
--     driving did not work at all. RGN.LOCKUP_* closes the lock-up clutch
--     while recuperating, with a stall guard for a still-coupled ICE.
--  3. FADE. The motor's regen curve fades out below regenFadeRPM and the
--     envelope runs out at high gear ratios, so the E-machine alone can never
--     bring the car to a standstill. RGN.ASSIST_* blends in exactly the
--     missing friction brake torque and latches a hold at zero speed - the
--     same mechanism the base game's EV shift logic uses for one-pedal.
--
-- The brake pedal is blended braking in both directions: regen takes over the
-- share of the friction demand it can actually deliver, and the friction
-- command is reduced by that same amount, so the pedal keeps asking for a
-- deceleration instead of for brake pads.
local RGN = {
  -- liftDecel = deceleration target on a full lift-off [m/s^2]
  --   OFF  free-wheeling. This is the position that allows sailing.
  --   LOW  ~ engine braking of a petrol car in a high gear
  --   MED  ~ the "B" selector position of a series hybrid
  --   HIGH ~ one-pedal driving (~0.22 g), carries the car into a standstill
  --          and holds it there (onePedal -> friction blend + hold latch)
  LEVEL = {
    OFF  = { liftDecel = 0.00, onePedal = false, allowSail = true  },
    LOW  = { liftDecel = 0.50, onePedal = false, allowSail = false },
    MED  = { liftDecel = 1.20, onePedal = false, allowSail = false },
    HIGH = { liftDecel = 2.20, onePedal = true,  allowSail = false },
  },
  DRIVE_EFF   = 0.92,   -- driveline efficiency motor -> road
  FALLBACK_NM = 180,    -- regen envelope if powerCurves is unavailable
  -- Lift-off gate: full target below THR_FULL pedal, zero from THR_OFF on.
  -- Deliberately narrow: once regen is meaningfully engaged the master zeroes
  -- the motor DRIVE torque (anti-kickback), so a wide band would leave a dead
  -- zone where the driver holds a light pedal and gets neither drive nor a
  -- clean coast. 5 % is under every creep/hold-speed pedal position.
  THR_FULL    = 0.02,
  THR_OFF     = 0.05,
  -- Brake pedal: share of the friction demand the E-machine may take over.
  -- Tapered out towards a panic stop so emergency braking stays mechanical.
  BRAKE_SHARE_MAX      = 1.00,
  BRAKE_SHARE_FULL_END = 0.35,
  BRAKE_SHARE_ZERO_END = 0.80,
  -- Speed fade: below MIN_KMH the E-machine has nothing left to give.
  MIN_KMH     = 0.8,
  TAPER_KMH   = 4.0,
  -- EV creep armed (etk854hybrid_creep.lua, ctx.creepIdleRpm > 0): the speed
  -- fade sits ABOVE creep speed instead, on the GEAR-1 synchronous turbine
  -- rpm as a multiple of the virtual idle: 0 below LO, full above HI
  -- (N57 7.2 -> 10.5 km/h, B58 6.7 -> 9.7, i4 6.0 -> 8.7; creep speeds 6.4 /
  -- 5.9 / 5.3 km/h). Lift-off and brake regen then hand over to the creep
  -- floor with one torque sign change instead of braking the car to a stop.
  CREEP_FADE_LO = 1.10,
  CREEP_FADE_HI = 1.60,
  -- Converter lock-up assist while recuperating
  LOCKUP_MIN_KMH      = 6.0,    -- with the ICE coupled (stall protection)
  LOCKUP_MIN_EV_KMH   = 1.5,    -- K0 open: an E-machine cannot stall, so the
                                -- clutch may stay closed right down to walking
                                -- pace - that is where one-pedal driving needs
                                -- the torque path most
  LOCKUP_RATE_UP      = 3.0,
  LOCKUP_RATE_DN      = 6.0,
  LOCKUP_ICE_RPM_FRAC = 1.25,   -- K0 closed: only lock above idle * this
  -- One-pedal friction blend + standstill hold
  ASSIST_RATE_UP   = 2.0,
  ASSIST_RATE_DN   = 4.0,
  ASSIST_MAX       = 0.45,      -- brake coefficient ceiling for the blend
  HOLD_KMH         = 1.5,       -- below this the hold latches
  HOLD_BRAKE       = 0.30,
  HOLD_RELEASE_THR = 0.05,      -- throttle that releases the hold
  -- Output rate limits, in Nm/s at the motor shaft. They are deliberately NOT
  -- a fraction of the envelope: the envelope moves with shaft speed (the
  -- maxRegenPower taper) and jumps at every gear change, so a smoothed
  -- FRACTION still lands as a torque step at the wheel - QA T12 measured a
  -- 254 Nm single-frame step through exactly that path.
  -- 600 Nm/s = 60 Nm per 100 ms, inside the "no torque step > 100 Nm/100 ms"
  -- spec target; the release is faster but stays under it as well.
  ENGAGE_NM_S  = 600,
  RELEASE_NM_S = 900,
  ACTIVE_THR  = 0.05,           -- above this we hard-zero the drive throttle
  -- Drive/regen crossover: lift-off recuperation (and the one-pedal friction
  -- blend) only engage once the jerk-limited drive torque (torqueCoord) is
  -- below this, so the shaft torque crosses zero once instead of drive and
  -- regen overlapping on the lift.
  XOVER_NM    = 15,
  -- ABS interlock (see rearOverSlip): the recuperation leaves the loop at once
  -- when the ABS acts AND the rear axle it brakes slips more than the front
  -- by this share, or in a hard stop
  ABS_REAR_SLIP  = 0.03,
  ABS_HARD_BRAKE = 0.60,
  -- ... or when any wheel slips more than this against the ABS reference
  -- speed (electrics.virtualAirspeed, the estimate wheels.lua's own ABS
  -- regulates to ~18 % against) - covers a front lock-up and AWD, where the
  -- rear-vs-front comparison stays blind. A qualified event is held for
  -- ABS_HOLD_S so the cut does not re-arm between two ABS pressure cycles.
  ABS_WHEEL_SLIP = 0.08,
  ABS_HOLD_S     = 0.30,
  -- Rear-axle slip limiter for the recuperation (see updateRecuperation):
  -- above SLIP_START of rear over-slip the regen target is scaled back
  -- linearly to zero at ABS_REAR_SLIP, through the normal release ramp
  REGEN_SLIP_START = 0.012,
  SLIP_TAU_S       = 0.05,  -- over-slip LPF (wheel speeds jitter ~1 % at 50 km/h)
  MEAS_TAU_S       = 0.05,  -- LPF of the measured converter torque (blend credit)
  -- Plausibility window for the ratio read
  RATIO_MIN      = 0.5,
  RATIO_MAX      = 40.0,
  RATIO_FALLBACK = 4.0,         -- ~ gear 5 x 3.15 final drive
  FINAL_DRIVE    = 3.15,        -- only used if the ratio has to be
                                -- reconstructed from the raw gear ratio
}

-- Runtime state of the recuperation block. ONE table on purpose: updateGFX is
-- close to LuaJIT's hard limit of 60 upvalues per function.
local _rgn = {
  levelName   = "MED",
  lockupCmd   = 0,      -- our own converter lock-up request (0..1)
  assistCmd   = 0,      -- our own friction brake request (0..1)
  holdLatched = false,  -- one-pedal standstill hold
  targetDecel = 0,      -- [m/s^2] what the driver asked for
  motorNm     = 0,      -- commanded regen torque at the motor shaft
  wheelNm     = 0,      -- ... after ratio + lock-up, i.e. what reaches the road
  ratio       = 4.0,
  mass        = 1900,   -- [kg]  measured in initSecondStage
  wheelR      = 0.33,   -- [m]
  brakeNm     = 0,      -- total friction brake torque at brake = 1.0
  refsOk      = false,
  refsTries   = 0,
  refsRetryT  = 0,
  gearbox     = nil,
  whl         = nil,    -- wheel data by name (FL/FR/RL/RR) for the ABS interlock
  osF         = 0,      -- filtered rear over-slip
  absHoldT    = 0,      -- [s] qualified-ABS hold (see updateRecuperation)
  tc          = nil,    -- torque converter device (blend credit)
  tcF         = 0,      -- filtered braking torque the converter hands to the gearbox [Nm]
}

-- ===========================================================
-- HUD / filtering
-- ===========================================================
-- One table = one upvalue (updateGFX / updateHud stay far below LuaJIT's 60).
local HUDC = {
  INTERVAL      = 0.05,   -- 20 Hz visual updates
  ICE_POWER_TAU = 0.35,   -- display smoothing time constants (dt-corrected;
  EV_POWER_TAU  = 0.12,   -- ICE slower: masks driveline torsional ripple)
  DBG_INTERVAL  = 1.0,    -- all periodic diagnostics are 1 Hz max
}
-- (LAUNCH_WINDOW_S moved to etk854hybrid_launchControl.lua LC.WINDOW_S)

-- ===========================================================
-- BATTERY CONTROL charging (ported from the proven HOLD logic)
-- ===========================================================
local CHARGE_SOC_HYST     = 0.03   -- restart charging this far under the target
local CHG = {
  -- P-gain on the ICE torque headroom. 0.15 -> 0.30 (2026-09-30): 0.15 gave
  -- 5 kW at 85 km/h in 8th (B58, 1300 rpm, 38 Nm of ~250 Nm headroom), i.e.
  -- +0.6 %/min - the SOC display barely moved. 0.30 takes ~10 kW, the engine
  -- runs at ~55 % load instead of ~45 %, still far from its limit.
  GAIN            = 0.30,
  MAX_KW          = 25,     -- hard cap on charge power taken off the ICE
  PARK_RPM        = 2200,   -- Park-Charge ICE rpm setpoint
  -- Park-Charge regen load expressed in Nm (NOT as a regen fraction): the
  -- regen envelope grew with the boost stages, so a fixed fraction would now
  -- load the idling engine far harder than the value this loop was tuned
  -- against.
  PARK_TORQUE_NM  = 80,
  PARK_RPM_KP     = 0.0003,
  PARK_RPM_KI     = 0.00008,
  PARK_RPM_MAX_THR = 0.45,
  PARK_ANTI_WINDUP = 0.40,
  -- Former PER-FRAME factors (0.18 / 0.15 / 0.90 / 0.85 at 60 fps) as time
  -- constants, so the charge ramps no longer depend on the frame rate:
  -- 1 - exp(-1/60 / tau) reproduces the old factors exactly at 60 fps.
  CRUISE_RISE_TAU_S  = 0.084,
  CRUISE_FALL_TAU_S  = 0.103,
  PARK_I_DECAY_TAU_S = 0.158,
  OFF_DECAY_TAU_S    = 0.103,
}

-- ===========================================================
-- BATTERY CONTROL: maximum-rate charging in NEUTRAL at standstill
-- ===========================================================
-- Driver-facing behaviour: EV mode = BATTERY CONTROL, selector in N, car
-- stopped, SOC below target  ->  the engine lifts to a charging load point and
-- the E-machine recuperates at the highest rate the hardware allows, instead of
-- the gentle 80 Nm / 2200 rpm trickle the Park case uses.
--
-- Physics of this operating point: in N the gearbox output is disconnected, but
-- K0 stays closed, so ICE + K0 + motor + converter pump all turn as one shaft.
-- The regen torque is therefore a pure BRAKE on the engine - the ICE has to
-- produce it plus its own friction.  Two consequences drive the numbers below:
--   1. the rpm setpoint must sit where the ICE curve has ample reserve over the
--      regen torque (NCHG.ICE_LOAD_FRAC), i.e. inside the torque plateau;
--   2. charge POWER, not torque, is what the pack sees - and it rises with rpm,
--      so the ideal setpoint is the speed at which the stage regen power budget
--      is just reached at full regen torque.  Above that, rpm only adds noise
--      and fuel burn for no extra charge.
-- One table, ONE upvalue: updateGFX sits close to LuaJIT's hard limit of 60
-- upvalues per function ("function at line N has more than 60 upvalues" is a
-- LOAD error) - grouping these constants keeps us safely below it.
local NCHG = {
  MAX_KMH        = 2.0,    -- "standing"
  ABORT_THR      = 0.05,   -- any real pedal input aborts
  RPM_MIN        = 2200,   -- never below the Park-Charge point
  RPM_MAX        = 3000,   -- a parked car must not scream
  RPM_MAXFRAC    = 0.60,   -- ... and never past 60 % of the rev range
  ICE_LOAD_FRAC  = 0.85,   -- regen may use 85 % of the ICE curve
  RPM_RAMP       = 900,    -- rpm/s setpoint ramp (no flare-up)
  REGEN_RISE     = 0.35,   -- regen fraction per second, engaging
  REGEN_FALL     = 1.20,   -- ... and releasing (abort must be quick)
  MAX_THR        = 0.75,   -- rpm PI ceiling (Park uses 0.45; this load point
                           -- needs a lot more fuel)
  -- The rpm loop is fed forward with loadTorque/iceCapacity, so the PI only
  -- has to trim; the gains stay moderate despite the much larger load step.
  RPM_KP         = 0.0006,
  RPM_KI         = 0.0002,
  ANTI_WINDUP    = 0.60,
  -- The engine can only use MAX_THR of its curve to PRODUCE the torque the
  -- E-machine brakes it with, so the load point must be derated by the same
  -- ceiling (0.85 = margin for pumping/friction losses at part throttle).
  THR_MARGIN     = 0.85,
  -- Below idle * this the loop is losing the engine -> dump the load at once.
  STALL_FRAC     = 0.90,
  STALL_RELEASE  = 4.0,    -- multiplier on REGEN_FALL while sagging
  -- Hard ceiling on the standstill charge load, independent of the motor's
  -- regen envelope. The envelope grew from 180 to 300 Nm for DRIVING
  -- recuperation; spending that at a stationary load point buys nothing (the
  -- pack is capped by the stage budget long before) and it is exactly what
  -- destabilised this loop - the engine has to PRODUCE every one of those Nm
  -- against the brake, and once the shaft is dragged through zero rpm the
  -- motor device keeps braking in the same direction (it indexes its curve
  -- with abs(rpm)), so there is no way back. 180 Nm is the load point the
  -- 16/16 baseline ran for many runs without a single upset.
  MAX_NM         = 180,
  -- Engage gate: never load an engine that is not running properly, and pull
  -- it back out of a backwards spin with the starter instead of waiting.
  ENGAGE_FRAC    = 1.05,   -- rpm > idle * this before any load is applied
  RECOVER_RPM    = 100,    -- below this (incl. negative) -> crank it
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
local _lastRegenNm = 0
local _nChargeRpmSetpoint = 0    -- ramped ICE rpm target for the N charge point
local _nChargeEngaged = false    -- charge-load enable latch (see NCHG.ENGAGE_FRAC)
-- launch control's physics-rate DSC hold assist (M.updateWheelsIntermediate)
local _lcWheelsFn = nil

-- ===========================================================
-- Helpers
-- ===========================================================
local function clamp01(v)
  if v < 0 then return 0 end
  if v > 1 then return 1 end
  return v
end

local function clamp(v, lo, hi)
  if v < lo then return lo end
  if v > hi then return hi end
  return v
end

local function smoothstep(t)
  t = clamp01(t)
  return t * t * (3 - 2 * t)
end

-- electrics flag: electrics.lua (updateGFX first step) rewrites EVERY boolean
-- channel as 1/0, so a flag written before that point in the frame (wheels.lua
-- absActive) reads as a number, one written by a controller afterwards
-- (isShifting, escActive) as a boolean. `== true` silently missed the number
-- form: the ABS interlock of the recuperation below never fired.
local function flag(v)
  return v == true or (type(v) == "number" and v > 0.5)
end

-- jbeam $variables are substituted before the controller sees them. If a .pc
-- never defines one (or a typo leaves the literal "$name" in place) fall back
-- to the documented default instead of poisoning the arithmetic with a string.
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

-- gearIndex alone cannot tell P from N (both read 0), so cross-check the gear
-- string. Only true Neutral disconnects the driver request.
local function getGearSign()
  local idx = electrics.values.gearIndex
  local gearStr = electrics.values.gear
  if type(idx) == "number" then
    if idx > 0 then return 1 end
    if idx < 0 then return -1 end
    if gearStr == "P" then return 0 end
    return 0
  end
  if gearStr == "R" then return -1 end
  if gearStr == "N" or gearStr == "P" then return 0 end
  return 1
end

-- MECHANICAL ratio from the motor shaft to the wheels, converter EXCLUDED.
-- The gearbox sits directly behind the converter, so ITS cumulativeGearRatio
-- (gear x transfer case x final drive) is exactly the path we need. The
-- motor's own cumulativeGearRatio must NOT be used here: it also carries the
-- converter's stallTorqueRatio, and while coasting the converter multiplies
-- torque by 1.0, not by its stall ratio (see torqueConverter.lua:
-- torqueRatio collapses to 1 as soon as avRatio >= couplingAVRatio).
local function mechRatioToWheels()
  local gb = _rgn.gearbox or powertrain.getDevice("gearbox")
  _rgn.gearbox = gb
  if gb then
    local r = gb.cumulativeGearRatio
    if type(r) == "number" then
      r = abs(r)
      if r >= RGN.RATIO_MIN and r <= RGN.RATIO_MAX then return r end
    end
    -- Fallback: reconstruct from the raw gear ratio (during a shift the
    -- cumulative value can briefly be 0 or stale).
    if type(gb.gearRatio) == "number" and abs(gb.gearRatio) > 0.05 then
      return clamp(abs(gb.gearRatio) * RGN.FINAL_DRIVE, RGN.RATIO_MIN, RGN.RATIO_MAX)
    end
  end
  return RGN.RATIO_FALLBACK
end

-- Rear-axle over-slip for the ABS interlock: (front - rear wheel speed) /
-- front. The E-machine brakes the rear axle only; wheels.lua's absActive
-- alone is no evidence of that - its ABS coefficient dips below 0.9 at the
-- onset of braking and with a lagging virtual airspeed while no wheel slips
-- (QA T12 2026-09-30: flag set from the first frame of a 0.25 brake at
-- 77 km/h with FL/RL equal; cutting on it threw 150-200 Nm out in one frame).
-- Unknown wheel layout -> 1 (trust the flag).
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

-- Largest braking slip of any wheel against the ABS reference speed
-- (wheels.lua virtualAirspeed). 0 below 5 m/s, where wheels.lua's ABS is
-- inactive as well, or when no reference/wheels are known.
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

-- Vehicle constants the deceleration model needs. Node weights are the same
-- source the base-game ESC uses for the vehicle mass; brake torque is summed
-- exactly like the EV shift logic builds its friction brake map.
local function readVehicleRefs()
  -- Every assignment is guarded: a read taken while the vehicle is being
  -- rebuilt (reset, part change) can hand back an empty wheels/nodes table,
  -- and overwriting good calibration data with zeros would silently disable
  -- the one-pedal friction blend for the rest of the session.
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
  -- sticky: once we have a plausible set, keep it
  _rgn.refsOk = _rgn.refsOk or ((_rgn.brakeNm > 100) and (_rgn.mass > 300))
  log("I", "etk854hybrid.regen", string.format(
    "vehicle refs: mass=%.0fkg wheelR=%.3fm brakeTq=%.0fNm ok=%s",
    _rgn.mass, _rgn.wheelR, _rgn.brakeNm, tostring(_rgn.refsOk)))
end

-- If powerCurves fails to load, safeRequire hands back the generic no-op stub,
-- which has none of the curve accessors. This minimal provider keeps the whole
-- torque pipeline arithmetic-safe in that case (petrol-i4 shaped, but the car
-- still drives instead of throwing on a nil call every frame).
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

-- Returns capNm (what the motor may deliver now) and curveNm (what the jbeam
-- envelope offers at this rpm). The device multiplies its FULL curve by the
-- evThrottle channel, so the stage clamp is applied as a throttle ceiling of
-- capNm / curveNm - that is the only way to cap a table-driven motor.
-- Reverse speed limiter. The E-machine sits upstream of the gearbox, so in R
-- it would happily wind the car up to >110 km/h backwards (seen in the
-- arcade/AI test). Real 7er/5er: reverse is electronically limited to
-- ~35 km/h. Ramp the drive authority (motor cap AND ice throttle) from 1 to
-- 0 between REV_LIMIT_START_KMH and REV_LIMIT_FULL_KMH; regen is untouched.
local REV_LIMIT = { startKmh = 30, fullKmh = 36 }
local function reverseLimitFrac()
  if electrics.values.gear ~= "R" then return 1 end
  local kmh = abs(electrics.values.wheelspeed or 0) * 3.6
  if kmh <= REV_LIMIT.startKmh then return 1 end
  if kmh >= REV_LIMIT.fullKmh then return 0 end
  return 1 - (kmh - REV_LIMIT.startKmh) / (REV_LIMIT.fullKmh - REV_LIMIT.startKmh)
end

local function motorCapAtRpm(rpm, deCfgNow)
  local s = stageCfg()
  local c = curves()
  local curveNm = c and c.motorMaxTorqueAtRpm(rpm) or s.torqueNm
  if curveNm < 1 then curveNm = 1 end

  local capNm = min(curveNm, s.torqueNm)
  local av = abs(rpm or 0) * RPM_TO_AV
  if av > 1.0 then
    -- constant-power region of the selected stage
    capNm = min(capNm, s.powerKw * 1000 / av)
    -- battery discharge budget (mechanical side after motor losses)
    capNm = min(capNm, s.battKw * 1000 * MOTOR_EFFICIENCY / av)
  end

  capNm = capNm * (ctx.evPowerFraction or 1.0) * (deCfgNow.evCapScale or 1.0)
  ctx.reverseLimit = reverseLimitFrac()
  capNm = capNm * ctx.reverseLimit
  if ctx.evLockoutActive then capNm = 0 end
  if capNm < 0 then capNm = 0 end
  return capNm, curveNm
end

-- (motorLeads() is gone: the ICE/motor split lives in etk854hybrid_torqueCoord)

local function iceCapAtRpm(rpm)
  local c = curves()
  if c then return c.iceMaxTorqueAtRpm(rpm) end
  return 350
end

-- ===========================================================
-- ICE power control
-- ===========================================================
-- setICEEnabled() / pulseStarter() are gone. Ignition, starter, K0 and the
-- AI/arcade starterDisabled guard belong to etk854hybrid_iceSequencer, which
-- sequences crank -> rev-match -> K0 engage -> torque blend instead of doing
-- all of it in one frame.

-- Policy verdict + its reason (published as etk854hybrid_icePolicyReason so
-- a telemetry run shows WHY the engine was asked for)
local function verdict(reason, want)
  ctx.icePolicyReason = reason
  return want
end

-- Load integral ("Lastintegral", see ICEPOL). Returns the deficit D [s].
-- currentOn = the arbitrated request: D is only built while the ICE is off.
local function updateDeficit(dt, r, def, speedKmh, currentOn)
  local key = (ctx.deIdx or 1) * 10 + (ctx.edriveIdx or 1)   -- DE x EV mode
  -- a mode change also drops the fast-pedal latch (no rate behind it)
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

-- Pure mode policy: what would the ICE do if sailing did not exist?
-- currentOn = the ARBITRATED request (ctx.iceReq), not the engine state.
-- Side outputs for the arbiter (etk854hybrid_iceSequencer.arbitrate):
--   ctx.iceForced      a forcing rule fired (SOC, BATTERY CONTROL, SPORT/HYBRID)
--   ctx.iceHardOff     MAX EV, standstill or EV-creep regime: release at
--                      once (FAST profile)
--   ctx.iceStartUrgent SOC emergency or pedal >= URGENT_PEDAL (URGENT profile)
--   ctx.iceStartFast   fast pedal (short debounce, normal start profile)
-- Diagnostics: ctx.icePolicyReason, ctx.iceDef (D), ctx.iceDefOn (Don),
-- ctx.policySportS (the S the start side uses, Sst).
--   ctx.iceLightLoad   split pedal map, pedal inside the EV zone and r below
--                      demandOff: the arbiter treats an off below LIFT_KMH
--                      as a lift (short hold, etk854hybrid_iceSequencer
--                      arbHoldTime) - easing off onto the E-machine is a lift
local function policyWantsIce(deCfgNow, edriveCfgNow, currentOn, pedal, speedKmh, torqueReqNm, motorCapNm, soc, dt)
  ctx.iceForced, ctx.iceHardOff, ctx.iceStartUrgent, ctx.iceStartFast = false, false, false, false
  ctx.iceLightLoad = false
  -- urgent pedal and its dwell (ECO PRO map v3: 0.95 held 0.12 s; else
  -- URGENT_PEDAL without dwell). Timed before the early returns so a pedal
  -- held through a forced / standstill verdict keeps its age.
  local urgP = (edriveCfgNow.deTuned and deCfgNow.urgentPedal) or ICEPOL.URGENT_PEDAL
  ctx.iceUrgentT = (pedal >= urgP) and ((ctx.iceUrgentT or 0) + (dt or 0)) or 0
  -- sport index on the START side (no side effects, moved up for
  -- ctx.policySportS): ECO PRO's split map takes S x sStart = 0
  local S = ctx.sportIndex or 0
  local deTuned = edriveCfgNow.deTuned == true
  local def = ICEPOL.DEF[ctx.deMode] or ICEPOL.DEF.COMFORT
  local splitMap = deCfgNow.pEv ~= nil and deTuned
  local Sst = (splitMap and def.sStart) and S * def.sStart or S
  ctx.policySportS = Sst
  -- 1. Battery emergency always wins.
  if soc < SOC_ICE_FORCE then
    ctx.iceForced, ctx.iceStartUrgent, ctx.iceFastLatch = true, true, false
    return verdict("soc", true)
  end

  -- 2. BATTERY CONTROL: run the engine until the target SOC is reached.
  --    Deliberately NOT gated by the standstill rule - Park-Charge is a feature.
  if edriveCfgNow.chargeToTarget then
    if soc < (ctx.batteryTargetSoc - (currentOn and 0 or CHARGE_SOC_HYST)) then
      ctx.iceForced, ctx.iceFastLatch = true, false
      return verdict("charge", true)
    end
  end

  -- 3. SPORT / legacy HYBRID pin the engine on.
  if deCfgNow.iceAlwaysOn or ctx.legacyIceForce then
    ctx.iceForced, ctx.iceFastLatch = true, false
    return verdict("pinned", true)
  end

  -- 3b. Pure-EV mode: never start the ICE (SPORT above still overrides;
  -- the low-SOC guard in updateGFX leaves this mode before the pack is flat).
  if edriveCfgNow.evOnly then
    ctx.iceHardOff, ctx.iceFastLatch = true, false
    return verdict("evOnly", false)
  end

  -- 4. Standstill with no intent: engine off. With EV creep available the
  --    same hard-off covers the creep regime (AUTO_CREEP_KMH): the E-machine
  --    creeps, the ICE is not kept for it (reason "creep"). ctx.creepAvail is
  --    the creep module's previous frame (it runs after the policy); SOC,
  --    BATTERY CONTROL, SPORT / HYBRID pin and MAX EV above still win.
  --    The creep branch needs SOC >= AUTO_CREEP_MIN_SOC (no start/stop limit
  --    cycle against rule 1 at 8 % SOC).
  if pedal < AUTO_STANDSTILL_THR then
    local still = speedKmh < AUTO_STANDSTILL_KMH
    if still or (ctx.creepAvail and speedKmh < AUTO_CREEP_KMH and soc >= AUTO_CREEP_MIN_SOC) then
      ctx.iceHardOff = true
      ctx.iceDef, ctx.iceFastLatch = 0, false
      return verdict(still and "standstill" or "creep", false)
    end
  end

  -- 5. Hysteresis band, scaled by the sportiness index (see ICEPOL; the
  --    start side on Sst, the off side on S).
  -- kick: the split map's kick point (ECO kickJoin 0.90), else kickAuto
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
  -- kick > 0 only: BATTERY CONTROL (kick 0) would otherwise call every
  -- quarter pedal "urgent". ECO PRO: the pedal must also have been at/above
  -- urgentPedal for urgentDwellS (stab protection, map v3).
  ctx.iceStartUrgent = (kick > 0) and pedal >= urgP
    and ctx.iceUrgentT >= ((deTuned and deCfgNow.urgentDwellS) or 0)
  local r = torqueReqNm / max(motorCapNm, 1)
  -- before the latch: a DE / EV mode change clears it (updateDeficit)
  local D = updateDeficit(dt or 0, r, def, speedKmh, currentOn)
  -- fast latch: with the split map never below the kick point (ECO 0.90)
  if kick <= 0 or ICEPOL.RATE_URGENT <= 0
     or pedal <= max(def.fastPedal, splitMap and (deCfgNow.kickJoin or 0) or 0) then
    ctx.iceFastLatch = false
  elseif (ctx.pedalRate or 0) >= ICEPOL.RATE_URGENT then
    ctx.iceFastLatch = true
  end
  ctx.iceStartFast = ctx.iceFastLatch == true and r >= def.fastR
  ctx.iceLightLoad = splitMap and pedal <= deCfgNow.pEv and r < def.rOff
  local don = def.onCalm * (1 - smoothstep((Sst - ICEPOL.DEF_S0) / ICEPOL.DEF_SPAN))
  -- engine still turning (IDLE hold / RELEASE): re-engaging it costs no crank
  if (ctx.iceEngageState or "OFF") ~= "OFF" then don = don * ICEPOL.DEF_TURNING_FRAC end
  ctx.iceDefOn = don
  if currentOn then
    if splitMap then
      -- Off side of the load integral (split map only): the engine stays
      -- while the pedal is above the EV zone or the E-machine could not
      -- carry the demand (r > rOff); once the demand is back inside the
      -- E-machine's range for offS(S) it is let go (the arbiter's lift /
      -- light-load hold follows). The old keep rules (pedal >= kickOff 0.385,
      -- demand > 0.80 x cap) sat INSIDE the 0.45 EV zone: after a start at
      -- 0.52 an ease to 0.40 kept the engine on for good (review 2026-10).
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
      -- a real lift / brake is no "demand back in the EV zone": the arbiter's
      -- S-scaled lift hold (etk854hybrid_iceSequencer arbHoldTime) decides
      if pedal < ICEPOL.LIFT_PEDAL or (ctx.brakeRaw or 0) > ICEPOL.LIFT_BRAKE then
        return verdict("off", false)
      end
      ctx.iceSurplusT = (ctx.iceSurplusT or 0) + (dt or 0)
      -- off side on the real S (a sporty driver keeps the engine longer),
      -- capped per DE (ECO offSport 1.5 s)
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
  -- >= (was >): a pedal held exactly at the kick point (COMFORT 0.70 / ECO
  -- 0.90) is a kick; BATTERY CONTROL (kick 0) keeps "any pedal"
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
-- Returns the driver torque demand [Nm] (before the N/P gate) and the
-- normalised demand shaped = demand / ceiling, published every frame as
-- etk854hybrid_pedalShaped (the 8HP reads it as its load axis,
-- etk854hybrid_8hpCore.lua senseDriver; QA X.releasePedal inverts it as
-- ceiling = torqueReq / pedalShaped).
--   split map (AUTO EV, DE entry with pEv, ICE not pinned):
--     p <= pEv:          T = knee * (p / pEv)^gEv                 EV zone
--     pEv < p < pJoin:   T = knee                     EV MAX plateau (ECO)
--     p >= pJoin:        T = knee + (top - knee) * ((p - pJoin) / (1 - pJoin))^gHi
--     knee = E-machine cap (before the redline taper), low-passed like ceilF,
--     never above top. pJoin = pEv without a plateau (COMFORT). top =
--     min(ceilF, system torque cap) with capTop (ECO), else ceilF.
--     N57 COMFORT (265 / 896 Nm): 0.25 -> 156, 0.45 -> 265, 0.60 -> 437,
--     0.85 -> 725 Nm (old linear map: 224 / 403 / 537 / 762).
--     N57 / B58 ECO PRO (map v3, 2000 rpm): 0.20 -> 50, 0.50 -> 151,
--     0.80..0.85 -> 265, 0.90 -> 403, 0.95 -> 542, 1.0 -> 680 Nm.
--   otherwise (SPORT, legacy HYBRID, MAX EV, BATTERY CONTROL):
--     T = p^gamma * ceilF, as before.
-- Also publishes the pedal-map view for cluster / HUD / 8HP / QA (map only:
-- an EV creep floor added elsewhere never shows up here):
--   etk854hybrid_pedalIceJoin   pJoin (0 = no split map), ctx.pedalJoin
--   etk854hybrid_pedalKickOn    kick point, etk854hybrid_pedalUrgent urgent pedal
--   etk854hybrid_pedalUrgentDwellS  urgentDwellS of the policy's urgent start
--                               (deTuned only, else 0) - the ONE source of the
--                               ECO kickdown dwell for the 8HP (8hpCore kdOn)
--   etk854hybrid_pedalEvFrac    T / knee (EV zone fill, 8HP EV power line)
--   etk854hybrid_pedalBandFrac  position of T inside the hybrid band
--   etk854hybrid_pedalZone      0 no split map | 1 p <= pEv | 2 plateau |
--                               3 band below kick | 4 kick..urgent | 5 >= urgent
--   etk854hybrid_pedalLegacyEq  round-2 ECO pedal for the same torque (_pm.LEG,
--                               legacyRef entries only, else -1)
--   etk854hybrid_evZoneOpen     1 while the EV speed ceiling allows EV
--                               (previous frame's speed; display only)
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
    -- band top: the system torque cap instead of the ceiling (capTop), so
    -- the whole band is torque the torque coordinator really delivers
    local sc = ctx.systemTorqueCapNm or 0
    if (deCfgNow.capTop or 0) > 0 and sc > 0 then top = min(ceilF, sc) end
    -- weak E-machine: the knee slides onto the low-SOC anchor
    -- (kneeLowFrac x ceiling; old map: its torque at pEv).
    -- SOC-driven part only (evPowerFraction / cold derate,
    -- batteryMonitor.lua:180): a cold pack below +5 C must not push the
    -- EV zone above the (derated) motor cap - that restarted the ICE at
    -- 0.34-0.45 pedal in winter (review 2026-10). A cold pack keeps the
    -- split map and simply delivers less torque per pedal in the EV zone.
    local socFrac = (ctx.evPowerFraction or 1) / max(ctx.evColdDeratingFraction or 1, 0.05)
    local h = clamp01((socFrac - _pm.HEALTHY0) / _pm.HEALTHY_SPAN)
    local kneeLow = (deCfgNow.kneeLowFrac or (pEv ^ deCfgNow.pedalGamma)) * ceilF
    knee = min(top, h * _pm.kneeF + (1 - h) * max(_pm.kneeF, kneeLow))
  end
  -- inverse map inputs for M.pedalForTorque (QA / tools)
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
  -- pedal-map view (see header)
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

-- Driver demand as a wheel-torque reference (feedback 2026-10 round 2, R2:
-- "viel mehr Drehmoment aus niedrigen Drehzahlen ... wenn das
-- Systemdrehmoment erschoepft ist, dann soll ein niedrigerer Gang eingelegt
-- werden"). The pedal map above is a SHAFT torque: the torque-first 8HP
-- (ECO / COMFORT upshift ~1-2 gears earlier) then simply cut the wheel torque
-- by the ratio step - N57 COMFORT 20->110 km/h at 0.60 pedal 10.8 -> 15.4 s,
-- the engine idling along at 2000 rpm with 250 Nm unused. With the ICE
-- coupled and the pedal above the EV edge (split map, ECO / COMFORT), the
-- request is referred to a reference engine speed instead:
--   T = T_map * k,  k = 1 + (clamp(nRef / n_shaft, 1, KMAX) - 1) * w,
--   w = clamp01((pedal - pJoin) / WSPAN)   (pJoin = pEv in COMFORT; ECO map
--       v3: from the band start 0.85 over dmdWspan 0.10 - a position ramp,
--       deliberately gentler than a torque-equivalent one)
-- i.e. the gear the 8HP picks below nRef gets the wheel torque the pedal
-- would have given at nRef (higher engine load at low rpm, no revving). It
-- is capped at CAPF x the torque the 8HP says this gear delivers
-- (etk854hybrid_shiftTavNm, 8hpCore C.publish), so the compensation never
-- triggers a torque downshift by itself (kDn 0.88-0.98 > CAPF): the 8HP
-- keeps planning on the map demand (etk854hybrid_torqueRequestMapNm) and
-- downshifts only when THAT exceeds what the gear can deliver - "system
-- torque exhausted". The EV pedal zone (w = 0), pure EV (ICE not ON), MAX
-- EV and BATTERY CONTROL in ECO / COMFORT (no split map) and the legacy
-- HYBRID preset are unchanged.
-- SPORT (lead decision round 3): no split map, so the comp runs on the
-- pedal^0.70 map whenever the (pinned) ICE is ON, with its own reference
-- speeds (_pm.DMD_NREF.*.SPORT) and a pedal gate w = clamp01((pedal -
-- dmdP0) / dmdWspan) = 0.10 .. 0.20: steady SPORT cruising (0.004-0.085
-- pedal at 50-130 km/h, shift_table_sport 2026-09-30) keeps k = 1, a 0.20
-- acceleration gets the full reference. From 0.40 pedal on the SPORT rpm
-- sits above nRef and k stays ~1.
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
-- Returns: regenFrac (0..1 command for the motor device), the device envelope
-- at this shaft speed [Nm] and whether we are meaningfully recuperating.
-- Side effects (all deliberate, all released again when regen stops):
--   electrics.values.etk854hybrid_evRegen   motor regen command
--   electrics.values.lockupClutchRatio      raised while recuperating
--   electrics.values.brake / brakelights    blended braking + one-pedal stop
local function updateRecuperation(dt, pedalRaw, brakeRaw, speedKmh, rpmMotor,
                                  gearSign, deCfgNow)
  -- Lazy retry if initSecondStage ran before wheels/nodes were populated.
  -- Throttled: readVehicleRefs sweeps every node, that must not run at 60 Hz.
  if not _rgn.refsOk then
    _rgn.refsRetryT = (_rgn.refsRetryT or 0) - dt
    if _rgn.refsRetryT <= 0 and (_rgn.refsTries or 0) < 10 then
      _rgn.refsRetryT = 1.0
      _rgn.refsTries = (_rgn.refsTries or 0) + 1
      readVehicleRefs()
    end
  end

  local lvl = RGN.LEVEL[_rgn.levelName] or RGN.LEVEL.MED
  local envelopeNm = (curves() and curves().motorRegenTorqueAtRpm(rpmMotor))
                     or RGN.FALLBACK_NM
  if envelopeNm < 1 then envelopeNm = 1 end

  local slipNow = (speedKmh > 10) and rearOverSlip() or 0
  _rgn.osF = _rgn.osF + (slipNow - _rgn.osF) * min(1, dt / RGN.SLIP_TAU_S)
  -- Qualified ABS event: the flag alone also rises on wheels.lua's slip-rate
  -- (derivative) term, e.g. from the rear driveline's torsional ringing under
  -- 300 Nm of regen at a 0.14 brake with ~1 % slip (QA T12 2026-09-30).
  -- Published as etk854hybrid_absQualified: the 8HP opens the converter
  -- lock-up on it - on the raw flag it opened the lock-up under full regen,
  -- the sailing E-machine (K0 open) was pulled from 1870 to 130 rpm within
  -- 150 ms and its regen envelope collapsed: a 230 Nm step.
  local absRaw = flag(electrics.values.absActive)
  if absRaw and (brakeRaw >= RGN.ABS_HARD_BRAKE or _rgn.osF > RGN.ABS_REAR_SLIP
                 or maxWheelSlip() > RGN.ABS_WHEEL_SLIP) then
    _rgn.absHoldT = RGN.ABS_HOLD_S
  else
    _rgn.absHoldT = max(0, _rgn.absHoldT - dt)
  end
  local absCut = _rgn.absHoldT > 0
  electrics.values.etk854hybrid_absQualified = absCut and 1 or 0
  local hardCut = ctx.sailSuppressRegen
    or gearSign <= 0
    or (electrics.values.parkingbrake or 0) > 0.1
    or absCut

  local ratio = mechRatioToWheels()
  _rgn.ratio = ratio
  -- wheel torque <-> deceleration:  F = m * a,  T = F * r
  local decelToWheelNm = _rgn.mass * _rgn.wheelR

  -- ---- lift-off target -------------------------------------------------
  local liftGate = 0
  if pedalRaw <= RGN.THR_FULL then
    liftGate = 1
  elseif pedalRaw < RGN.THR_OFF then
    liftGate = 1 - (pedalRaw - RGN.THR_FULL) / (RGN.THR_OFF - RGN.THR_FULL)
  end
  local liftDecelFull = lvl.liftDecel * (deCfgNow.coastScale or 1.0) * ctx.regenStrength
  local liftDecel = liftDecelFull * liftGate
  -- Drive/regen crossover (see RGN.XOVER_NM): no lift-off regen while the
  -- jerk-limited drive torque is still ramping out.
  local driveOut = (ctx.torqueTargetNm or 0) <= RGN.XOVER_NM
  if not driveOut then liftDecel = 0 end

  -- ---- brake pedal target (blended braking) ----------------------------
  -- Trailer coupled (etk854hybrid_selfLeveling etk854hybrid_trailerAttached,
  -- or an air-coupled trailer, etk854hybrid_airCompressor _airTrailer): NO
  -- blended braking. A coupled trailer's service brake replays OUR electrics
  -- brake (beamstate export of importInputs brake / parkingbrake, e.g.
  -- dolly_single.jbeam:116), and after the blend that is only the friction
  -- share - under regen-only braking the trailer did not brake at all and
  -- pushed the car (PLAN_DRUCKLUFT_ANHAENGER.md section 8). With a trailer
  -- the friction brake follows the pedal 1:1 and the E-machine recuperates on
  -- lift-off only (that decel still adds on top while braking).
  local ev0 = electrics.values
  local trailerOn = flag(ev0.etk854hybrid_trailerAttached) or flag(ev0.etk854hybrid_airTrailer)
  ev0.etk854hybrid_regenTrailerNoBlend = trailerOn and 1 or 0
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
  -- Speed fade. EV creep armed: on the GEAR-1 synchronous turbine rpm (=
  -- vehicle speed, gear-independent -> no regen dip at the 2->1 downshift),
  -- 0 below CREEP_FADE_LO x virtual idle; the friction brake takes over 1:1
  -- through the measured-converter blend credit below. Otherwise (HIGH, R,
  -- P / N, ICE coupled: creepIdleRpm 0) the old fade to MIN_KMH.
  local cIdle = ctx.creepIdleRpm or 0
  local gb1 = _rgn.gearbox                     -- cached by mechRatioToWheels() above
  local g1 = gb1 and gb1.gearRatios and gb1.gearRatios[1]
  if cIdle > 0 and type(g1) == "number" and g1 > 0 then
    local nT1 = abs(gb1.outputAV1 or 0) * g1 * 9.549296596
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
      -- stage recuperation power budget (also protects the pack C-rate)
      motorNm = min(motorNm, stageCfg().regenKw * 1000 / avMotor)
    end
    motorNm = motorNm * (ctx.regenAcceptFraction or 1.0)   -- SOC / temperature
    -- ICE drag credit: with the ICE coupled in overrun its fuel-cut drag
    -- already decelerates the car, so the E-machine only adds the rest and
    -- the selected level feels identical with K0 open or closed
    -- (ctx.iceDragNm from etk854hybrid_torqueCoord, 0 unless LOCKED).
    motorNm = max(0, motorNm - (ctx.iceDragNm or 0))
    -- Rear-axle slip limiter: the E-machine brakes the rear axle only, on top
    -- of the rear friction brakes. When the rear wheels run slower than the
    -- front ones, the rear axle is over-braked and the target is scaled
    -- back - through the release ramp, so the ABS interlock above (a hard
    -- cut) stays the last resort for a real lock-up.
    if _rgn.osF > RGN.REGEN_SLIP_START then
      motorNm = motorNm * clamp01(1 - (_rgn.osF - RGN.REGEN_SLIP_START)
                                      / (RGN.ABS_REAR_SLIP - RGN.REGEN_SLIP_START))
    end
  end

  -- Rate limiting happens HERE, in Nm, and the command fraction is derived
  -- from the limited torque afterwards (see RGN.ENGAGE_NM_S).
  if absCut and _regenSmoother then
    -- ABS is the one case that may not wait for a ramp: if the wheels are
    -- locking, the E-machine has to be out of the loop this frame.
    _regenSmoother:set(0)
  end
  local appliedNm = motorNm
  if _regenSmoother then appliedNm = _regenSmoother:get(motorNm, dt) end
  if appliedNm < 0 then appliedNm = 0 end
  if appliedNm > envelopeNm then
    -- the envelope fell faster than the release rate (downshift -> higher
    -- shaft speed -> power taper): the motor cannot deliver more, and the
    -- limiter state must follow, or the torque would jump back up in one
    -- frame once the envelope recovers
    appliedNm = envelopeNm
    if _regenSmoother then _regenSmoother:set(envelopeNm) end
  end
  local regenFrac = clamp01(appliedNm / envelopeNm)
  _rgn.motorNm = appliedNm
  _lastRegenNm = appliedNm

  -- ---- converter lock-up assist ----------------------------------------
  -- Without this the open converter caps what the E-machine can push back
  -- into the driveline at ~60-80 Nm below gear 3 (see the header note).
  local wantLock = 0
  local k0Now = electrics.values.etk854hybrid_k0ClutchRatio or 0
  local lockMinKmh = (k0Now > 0.2) and RGN.LOCKUP_MIN_KMH or RGN.LOCKUP_MIN_EV_KMH
  if appliedNm > 5 and speedKmh > lockMinKmh
     and not flag(electrics.values.isShifting) then
    wantLock = 1
    if k0Now > 0.2 then
      -- ICE still coupled: a locked converter drags it down, so keep a
      -- healthy margin over idle or the engine stalls on the brakes.
      local idle = (curves() and curves().iceIdleRPM()) or 800
      if (electrics.values.rpm or 0) < idle * RGN.LOCKUP_ICE_RPM_FRAC then
        wantLock = 0
      end
    end
  end
  local lockRate = (wantLock > _rgn.lockupCmd) and RGN.LOCKUP_RATE_UP
                                              or RGN.LOCKUP_RATE_DN
  _rgn.lockupCmd = clamp01(_rgn.lockupCmd
    + clamp(wantLock - _rgn.lockupCmd, -lockRate * dt, lockRate * dt))
  if _rgn.lockupCmd > 0.01 then
    electrics.values.lockupClutchRatio =
      max(electrics.values.lockupClutchRatio or 0, _rgn.lockupCmd)
  end

  -- What actually reaches the road. Credit only through the lock-up clutch:
  -- with an open converter most of the regen torque is burnt as slip, and
  -- over-crediting it would weaken the friction brakes below.
  local lockupNow = clamp01(electrics.values.lockupClutchRatio or 0)
  local wheelNm = appliedNm * ratio * RGN.DRIVE_EFF * lockupNow
  _rgn.wheelNm = wheelNm
  -- The blend credit is the braking torque the converter really hands to the
  -- gearbox (fluid path + lock-up clutch, torqueConverter.lua outputTorque1;
  -- regen plus the drag of a coupled ICE) times the gearbox ratio. The model
  -- above (lock-up ratio as the transmitted share) collapses whenever the
  -- 8HP opens the lock-up for a downshift, while an overrunning converter
  -- still transmits most of the regen: the friction brake came back on top
  -- of it (QA T12 N57 2026-09-30: 0.5 g against a 0.38 g target, rear axle
  -- 3 % over-slip, ABS). Model credit only if the converter is missing.
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
  -- Snap to zero ONLY while releasing. Applying the dead band to the
  -- accumulator itself is a frame-rate trap: at 240 fps one ramp step is
  -- 2.0/s / 240 = 0.008, i.e. smaller than the 0.01 band, so the command was
  -- reset to zero every single frame and the blend could never build. That is
  -- why the one-pedal friction assist never engaged (QA telemetry rgn_brk
  -- flat 0.000 while the hold latch was set).
  if assistTarget <= 0 and _rgn.assistCmd < 0.01 then _rgn.assistCmd = 0 end
  newBrakeCoef = max(newBrakeCoef, _rgn.assistCmd)

  if newBrakeCoef ~= brakeRaw then
    electrics.values.brake = newBrakeCoef
  end
  -- Keep the brake lights honest in BOTH directions: they must stay on when
  -- regen replaced the friction brakes, and come on when the car brakes itself.
  -- 1.3 m/s^2 is the threshold ECE R13-H prescribes for exactly this case
  -- (deceleration without the brake pedal), so MED and HIGH light up on
  -- lift-off and LOW does not - same as the series cars.
  if brakeRaw > 0.02 or _rgn.assistCmd > 0.02 or _rgn.targetDecel > 1.3 then
    electrics.values.brakelights = 1
  end

  -- ---- channels --------------------------------------------------------
  electrics.values.etk854hybrid_evRegen        = regenFrac
  electrics.values.etk854hybrid_regenDecelMs2  = _rgn.targetDecel
  electrics.values.etk854hybrid_regenWheelNm   = wheelNm
  electrics.values.etk854hybrid_regenLockup    = _rgn.lockupCmd
  electrics.values.etk854hybrid_regenBrakeCoef = _rgn.assistCmd
  electrics.values.etk854hybrid_onePedalHold   = _rgn.holdLatched and 1 or 0

  return regenFrac, envelopeNm, (regenFrac >= RGN.ACTIVE_THR)
end

-- ===========================================================
-- Mode API
-- ===========================================================

-- -----------------------------------------------------------
-- DSC / DTC coupling (part etk854hybrid_dse.jbeam, controller "driveModes")
-- ===========================================================
-- The Driving-Experience axis selects the DSC calibration:
--   COMFORT / ECO PRO -> "dsc"
--   SPORT             -> "dtc"
-- Hard rule: a driver who switched DSC OFF stays off. Re-arming stability
-- control behind the driver's back on an unrelated mode change is exactly the
-- behaviour a real production car does not have (and it would be dangerous mid-corner).
--
-- Returns true when the DSE was reachable and the decision was made, false
-- when the driveModes controller does not exist yet (controller init order) -
-- the caller then arms ctx.dscPending and updateGFX retries on the next tick.
local function applyDscForDeMode(deName)
  -- controller.init iterates the controller table with pairs(), so the
  -- init order is hash order: the driveModes controller can already exist
  -- while the DSE supervisors have not registered their CMU yet (fitting the
  -- cluster part changed that order and setDriveMode() then blew up inside
  -- tractionControl.setParameters with CMU == nil, aborting OUR init). Never
  -- touch the DSE before the first updateGFX tick, and treat any error in the
  -- DSE as "not ready yet" so the pending retry keeps going.
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

-- Parked on ctx so updateGFX can reach both without burning upvalues -
-- updateGFX is within a couple of slots of LuaJIT's hard limit of 60.
ctx.applyDsc = applyDscForDeMode
ctx.dscLabels = DSC_LABEL

local function refreshLegacyMode()
  -- Reverse-map the 2D state onto the legacy name so HUD/QA keep a stable view.
  local legacy
  if ctx.deMode == "SPORT" then legacy = "SPORT"
  elseif ctx.edriveMode == "BATTERY_CONTROL" then legacy = "HOLD"
  elseif ctx.edriveMode == "MAX_EDRIVE" then legacy = "EV"
  elseif ctx.legacyIceForce then legacy = "HYBRID"
  else legacy = "AUTO" end
  ctx.legacyMode = legacy
  for i, v in ipairs(LEGACY_MODE_ORDER) do
    if v == legacy then ctx.legacyIdx = i break end
  end
end

local function applyModes(deName, edriveName, iceForce)
  if DE_CFG[deName] then
    ctx.deMode = deName
    for i, v in ipairs(DE_ORDER) do
      if v == deName then ctx.deIdx = i break end
    end
  end
  if EDRIVE_CFG[edriveName] then
    ctx.edriveMode = edriveName
    for i, v in ipairs(EDRIVE_ORDER) do
      if v == edriveName then ctx.edriveIdx = i break end
    end
  end
  ctx.legacyIceForce = (iceForce == true)
  refreshLegacyMode()
  -- Push the matching DSC calibration. During M.init the DSE controllers do
  -- not exist yet, so arm a retry that updateGFX picks up on its first tick.
  if not applyDscForDeMode(ctx.deMode) then
    ctx.dscPending = true
  end
end

local function modeLabel()
  return ctx.deMode .. "/" .. (EDRIVE_SHORT[ctx.edriveMode] or ctx.edriveMode)
end

-- driver-facing names for the mode message (the etk854hybrid_mode channel keeps
-- the short internal label modeLabel(), QA T9 verifies against it)
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
-- Applying a level is more than a scale factor: OFF is the position that
-- hands the lift-off character over to the sailing FSM (true free-wheeling),
-- every other level has to take it back - otherwise the driver selects MED
-- and the car keeps gliding with 0 Nm, which is exactly how the old
-- LOW/MED/HIGH selector managed to feel like it did nothing at all.
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
-- Launch Control
-- The state machine, the converter preload regulator, the DSC hold assist and
-- the brake-throttle guard live in etk854hybrid_launchControl.lua (called from
-- updateGFX / applyOutputs / updateWheelsIntermediate below); ACTIVE is turned
-- into the full-envelope request by etk854hybrid_torqueCoord (LAUNCH row).
-- Manual arm (Alt+K, Actions app, QA T14): gear S / M in any Driving
-- Experience, or D in SPORT; a second press while armed switches it off.
-- In SPORT + DTC / DSC OFF it also arms itself on brake + kickdown at a
-- standstill (the gamepad path).
-- ===========================================================
function M.armLaunchControl()
  local lm = ctx.modules.launch
  if lm and lm.arm then
    lm.arm(ctx, "manual")
  else
    guihooks.message("Launch Control not available", 3, "vehicle.launch")
  end
end

-- Live tuning / diagnostics (console / dev server), not persistent:
--   M.setLaunchParam('MARGIN', 0.70)   M.setLaunchParam('GEN_LOAD', 0)
--   M.setLaunchParam('MU_HOLD', 0.9)   M.setLaunchParam('GROUND_PATH', false)
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
-- P1 API: sportiness index, ICE sequencer, QA hooks
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

-- QA only: true = force the ICE on, false = force it off, nil = policy.
-- The QA suite must hand back nil at every test end and on abort.
function M.qaForceIce(value)
  if value == nil then ctx.qaIceOverride = nil else ctx.qaIceOverride = (value == true) end
end

-- Bisecting aid: true = the sequencer reproduces the legacy one-frame start
-- (ignition + starter + K0 4/s, no rev-match, no fill).
function M.setIceSeqPassthrough(on)
  ctx.iceSeqPassthrough = (on == true)
  log("I", "etk854hybrid", "iceSeq passthrough = " .. tostring(ctx.iceSeqPassthrough))
end

-- Inverse of the pedal map of the last frame: the pedal [0..1] that asks for
-- Tnm at the present knee / ceiling (QA tests that need a given LOAD rather
-- than a given pedal, e.g. T20b "release at 0.85 x motor cap", T19D).
-- nil before the first frame.
-- Split map: a torque at the knee returns pEv (the start of an EV MAX
-- plateau), a band torque is placed between pJoin and 1 against the band top.
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

-- Live tuning of the ICE policy / pedal map (console / dev server), not
-- persistent. Returns the old value, nil for an unknown or non-numeric key.
--   M.setIcePolicyParam('RATE_URGENT', 0)            ICEPOL (global)
--   M.setIcePolicyParam('onCalm', 0.6, 'COMFORT')    ICEPOL.DEF[de]
--   M.setIcePolicyParam('pEv', 0.40, 'COMFORT')      DE_CFG[de] (pedal map, kick, EV top)
-- ECO PRO map v3 knobs: DE_CFG pEv, gEv, gHi, plateau, kickGap, urgentPedal,
-- urgentDwellS, capTop, kneeLowFrac, dmdWspan, siU0, siU1, siWRate (pJoin /
-- kickJoin are re-derived after every DE_CFG write); DEF sStart, onCalm,
-- rMin, offSport, fastPedal. SPORT demandComp gate: DE_CFG dmdP0, dmdWspan.
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
  -- K0 (etk854hybrid_k0ClutchRatio) is published by etk854hybrid_iceSequencer
  -- only - it is the channel's single writer (failsafe branch excepted).
  electrics.values.etk854hybrid_mode                  = modeLabel()
  electrics.values.etk854hybrid_deMode                = ctx.deMode
  electrics.values.etk854hybrid_edriveMode            = ctx.edriveMode
  electrics.values.etk854hybrid_modeLegacy            = ctx.legacyMode
  electrics.values.etk854hybrid_phase                 = "IDLE"
  electrics.values.etk854hybrid_iceState              = 1
  electrics.values.etk854hybrid_regenLevel            = ctx.regenLevelOrder[ctx.regenLevelIdx]
  -- tcActive: 1 while the DSE is ACTUALLY intervening (torque trim or brake
  -- stage). Derived from the game's own escActive/tcsActive channels; it used
  -- to be a static "SPORT turns TC off" flag that had no effect on anything.
  electrics.values.etk854hybrid_tcActive              = 0
  -- Arming state of the stability system: "DSC" | "DTC" | "OFF"
  electrics.values.etk854hybrid_dscMode               = ctx.dscLabels[ctx.dscKey] or "DSC"
  electrics.values.etk854hybrid_batterySoc            = 0
  electrics.values.etk854hybrid_icePower              = 0
  electrics.values.etk854hybrid_evPower               = 0
  electrics.values.etk854hybrid_launchState           = "IDLE"
  -- internal launch phase OFF/ARMED/PRELOAD/READY/FIRE/ABORT/GUARD; the rest
  -- of the etk854hybrid_launch* channels is published by the launch module
  electrics.values.etk854hybrid_launchPhase           = "OFF"
  electrics.values.etk854hybrid_launchLevelHold       = 0
  electrics.values.etk854hybrid_batteryFraction       = 1
  electrics.values.etk854hybrid_batteryBand           = "NORMAL"
  electrics.values.etk854hybrid_evAvailable           = 1
  electrics.values.etk854hybrid_regenAllowed          = 1
  electrics.values.etk854hybrid_evPowerFraction       = 1
  electrics.values.etk854hybrid_regenAcceptFraction   = 1
  electrics.values.etk854hybrid_absQualified          = 0
  -- new in the ActiveHybrid rework
  electrics.values.etk854hybrid_boostStage            = ctx.boostStage
  electrics.values.etk854hybrid_evTorqueCapNm         = 0
  electrics.values.etk854hybrid_evTorqueNm            = 0
  electrics.values.etk854hybrid_iceTorqueNm           = 0
  electrics.values.etk854hybrid_iceMaxTorqueNm        = 0
  electrics.values.etk854hybrid_sysTorqueNm           = 0
  electrics.values.etk854hybrid_chargeMode            = "OFF"
  electrics.values.etk854hybrid_torqueRequestNm       = 0
  electrics.values.etk854hybrid_pedalShaped           = 0
  -- ICE policy diagnostics (feedback 2026-10): EV pedal edge (0 = no split
  -- map), policy reason, load-integral deficit D and its threshold Don [s]
  electrics.values.etk854hybrid_pedalEvEdge           = 0
  electrics.values.etk854hybrid_icePolicyReason       = "none"
  electrics.values.etk854hybrid_iceDeficitS           = 0
  electrics.values.etk854hybrid_iceDeficitOnS         = 0
  -- pedal-map view (mapPedal, ECO PRO map v3) and the start-side sport index
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
  -- P1: ICE engagement / torque coordination / sportiness (contract channels
  -- first, the sequencer and the coordinator overwrite them every frame)
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
  -- EV creep (etk854hybrid_creep.lua: arming / HOLD; the motor device
  -- creepFloor publishes creepNm / creepSetRpm / creepAuth / pumpRpm /
  -- tcPumpNm every frame). creepMaxNm 0 = the device floor is inert.
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
  ctx.iceOnLastFrame = true
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
  ctx.creepIdleRpm, ctx.creepAvail = 0, false   -- EV creep (etk854hybrid_creep.lua)
  _pm.kneeF = nil          -- re-seeded from the first frame's motor cap
  _pm.dmdK = 1
  _pm.top = 0
  _icePowerEma = 0
  _evPowerEma = 0
  _chargeRegenFrac = 0
  _parkChargeRpmIntegral = 0
  _simTime = 0
  _dbgLastLogT = 0
  _lastRegenNm = 0
  _nChargeRpmSetpoint = 0
  _nChargeEngaged = false
  _rgn.lockupCmd = 0
  _rgn.assistCmd = 0
  _rgn.holdLatched = false
  _rgn.targetDecel = 0
  _rgn.motorNm = 0
  _rgn.wheelNm = 0
  _rgn.gearbox = nil
  _rgn.whl = nil
  _rgn.osF = 0
  _rgn.absHoldT = 0
  _rgn.tc = nil
  _rgn.tcF = 0
  -- refsOk deliberately NOT cleared here: mass, wheel radius and brake torque
  -- do not change on a reset, and clearing them would leave the deceleration
  -- model without calibration until the retry loop happens to catch a fully
  -- built vehicle. initSecondStage re-reads them when the config changes.
  _rgn.refsTries = 0
  _rgn.refsRetryT = 0
  if _regenSmoother and _regenSmoother.set then _regenSmoother:set(0) end
end

function M.init(jbeamData)
  ctx.jbeamData = jbeamData
  ctx.debugLevel = readNumber(jbeamData, "debugLogLevel", 1)

  -- jbeam tuning variables ($ebooststage / $regenstrength / $sailingenabled)
  --
  -- E-Boost stage precedence:
  --   1. a part in the "etk854hybrid_boost_stage" slot that merges the custom
  --      section "etk854hybridBoostStage" with stage >= 0  ->  pins the stage
  --   2. otherwise (stage < 0 == "Tuning-Regler" part, or no part at all)
  --      the $ebooststage tuning slider
  -- The section reaches us through the normal jbeam part unification
  -- (slotSystem.lua unifyParts); v.data is already populated when controller
  -- init runs - same pattern as the base-game pyrotechnicCharge controller and
  -- as our own motorShaft device reading v.data[jbeamData.soundConfig].
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
  -- EV pedal zone of the split pedal map ($evPedalZoneComfort 0.30..0.65,
  -- $evPedalZoneEco 0.40..0.85 internally, shown in %). A .pc that stores the
  -- display value (45 instead of 0.45) is accepted as well. ECO PRO's
  -- plateau end / kick point follow the zone (deriveSplitThresholds).
  for _, pz in ipairs({ { "COMFORT", "evPedalZoneComfort" }, { "ECO_PRO", "evPedalZoneEco" } }) do
    local cfg = DE_CFG[pz[1]]
    local v = readNumber(jbeamData, pz[2], cfg.pEv)
    if v > 1 then v = v / 100 end
    cfg.pEv = clamp(v, cfg.pEvMin or PEV_MIN, cfg.pEvMax or PEV_MAX)
    deriveSplitThresholds(cfg)
  end
  -- pedal-rate urgent start (ICEPOL.RATE_URGENT, 1/s); 0 = off (keyboard/gamepad)
  ICEPOL.RATE_URGENT = clamp(readNumber(jbeamData, "iceRateUrgent", ICEPOL.RATE_URGENT), 0, 50)
  -- System torque cap: BASE (engine dependent) + STAGE bonus.
  -- The base comes from the fitted ECU part if it merges the custom dict section
  -- "etk854hybridSystemCap" (same mechanism as the boost-stage section above),
  -- otherwise from the powertrain jbeam's controller argument.
  -- 0 disables the cap entirely (for experimenting with a stronger converter);
  -- the lower bound keeps a typo from strangling the car down to walking pace.
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
    -- The stage bonus rides on top so a paid-for stage is actually delivered
    -- instead of being trimmed away again at full load.
    ctx.systemTorqueCapNm = clamp(capBase + (stageCfg().capBonusNm or 0), 150, 5000)
  else
    ctx.systemTorqueCapNm = 0
    ctx.systemTorqueCapBaseNm = 0
  end

  applyModes("COMFORT", "AUTO_EDRIVE", false)
  ctx.regenLevelIdx = 3          -- MED
  applyRegenLevel(false)
  resetRuntimeState()
  -- NOTE the argument order: newTemporalSmoothing picks self[dif * state >= 0]
  -- (filters.lua), i.e. the SECOND rate applies while the value moves AWAY
  -- from zero (engaging) and the FIRST while it moves back TOWARD zero
  -- (releasing) - the opposite of what the parameter names suggest.
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

  -- P1: sportiness index, drive torque coordinator, ICE sequencer
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
  -- EV creep / virtual idle: arming + HOLD; the floor itself runs in the
  -- motor device (etk854hybrid_motorShaft.lua creepFloor). A failed load
  -- leaves the stub: creepMaxNm stays 0 (publishDefaults), no EV creep.
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
  -- mass / wheel radius / brake torque for the deceleration model
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

M.onInit = M.init

-- ===========================================================
-- Failsafe drive (sequencer or coordinator failed to load)
-- ===========================================================
-- Keeps the car drivable with the ICE running and K0 closed, split like the
-- old motor-first allocation. The ONLY writer of the K0 channel besides the
-- sequencer, and only active while the sequencer is missing.
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
-- BATTERY CONTROL charging (moved out of updateGFX: upvalue budget)
-- ===========================================================
-- Raises the ICE load point and dumps the surplus into the pack. Runs only
-- with the ICE fully engaged (sequencer state ON): the PARK / NEUTRAL
-- throttle overrides would otherwise fight the sequencer's SYNC/ENGAGE speed
-- control, and a CRUISE charge load on a slipping K0 is pointless.
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
  local doCharge = (soc < ctx.batteryTargetSoc) and (ctx.iceEngageState == "ON")
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

  -- Headroom = ICE curve minus the DRIVE torque the engine carries. The
  -- coordinator's expectation iceExpNm (exact throttle inverse + trim) is the
  -- load estimate, but it already contains last frame's load-point shift
  -- (plannedIceShare adds chargeNmLast), so that part is taken out again -
  -- measuring the charge against a load that includes the charge closed a
  -- loop that settled at GAIN/(1+GAIN) of the headroom. The raw
  -- |outputTorque1| EMA that used to be max()ed in on top read the K0
  -- torsional ringing (+-150 Nm before the K0 damping fix) as load and all
  -- but zeroed the headroom (QA T6 2026-09-30: -5..-10 Nm charge torque,
  -- +0.15 pp in 120 s).
  local iceActualT = max(0, (ctx.iceExpNm or 0) - chargePrevNm)

  local headroom = max(0, iceMaxAtRpm - iceActualT)
  local targetRegenNm = headroom * CHG.GAIN
  if iceAvNow > 2 then
    targetRegenNm = min(targetRegenNm, (CHG.MAX_KW * 1000) / iceAvNow)
  end
  local targetRegenFrac = clamp01(targetRegenNm / max(1, regenEnvelopeNm))

  local gearVal = ev.gear
  local isNeutral = (gearVal == "N")
  -- Maximum-rate charging in N: standing still, no pedal input, engine
  -- running, SOC below target (the last two are already in doCharge).
  -- Every one of these is a hard, observable state - shifting out of N,
  -- touching the throttle, rolling away or reaching the target ends it.
  local nMaxCharge = isNeutral
                 and speedKmh < NCHG.MAX_KMH
                 and pedalRaw < NCHG.ABORT_THR
  -- P keeps the gentle legacy Park-Charge. N never falls back into it -
  -- an aborted N charge must relax, not switch to a different forced load.
  local isParked = (not nMaxCharge) and not isNeutral
                   and ((gearVal == "P") or (gearSign == 0 and speedMs < 0.5))

  if nMaxCharge then
    -- --- rpm setpoint -------------------------------------------------
    -- Charge power = regenTorque * av, and the regen torque is flat over
    -- rpm, so the useful ceiling is the speed at which the stage regen
    -- power budget is already met. Revving past it burns fuel for nothing.
    local stageRegenKw = stageCfg().regenKw
    local plateauNm = curves() and curves().motorRegenTorqueAtRpm(NCHG.RPM_MAX)
                      or RGN.FALLBACK_NM
    local rpmForBudget = (plateauNm > 1)
      and (stageRegenKw * 1000 / plateauNm) * 9.549296596
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

    -- --- regen torque: take everything the hardware allows -------------
    -- Limited only by (a) the motor's regen envelope at this shaft speed,
    -- (b) the ICE curve with a reserve - the engine has to PRODUCE this
    -- torque against the brake, (c) the stage charge power budget,
    -- (d) the pack's charge acceptance from batteryMonitor.
    local iceCapNow = max(1, iceMaxAtRpm)
    -- ICE_LOAD_FRAC alone is NOT the binding limit: the engine may only use
    -- NCHG.MAX_THR of its curve to carry this brake torque, so the load has
    -- to be derated by that ceiling too.  QA T7 caught the difference the
    -- hard way when the regen envelope grew from 180 to 300 Nm: on the
    -- 2.0 i4 the loop asked for 0.85 * 340 = 289 Nm while 0.75 throttle can
    -- deliver ~255 - the shaft was dragged through zero into NEGATIVE rpm,
    -- and because the motor device indexes its regen curve with abs(rpm)
    -- the brake torque kept pushing it further backwards. Result: the pack
    -- DISCHARGED at -3.5 pp instead of charging.
    local loadFrac = min(NCHG.ICE_LOAD_FRAC, NCHG.MAX_THR * NCHG.THR_MARGIN)
    local regenAllowNm = min(regenEnvelopeNm, iceCapNow * loadFrac, NCHG.MAX_NM)
    local avNow = max(1, rpmIce * RPM_TO_AV)
    regenAllowNm = min(regenAllowNm, stageRegenKw * 1000 / avNow)
    regenAllowNm = regenAllowNm * (ctx.regenAcceptFraction or 1.0)

    local targetFrac = clamp01(regenAllowNm / max(1, regenEnvelopeNm))
    -- Second line of defence, engine-agnostic: whatever the load model
    -- says, if the engine is actually sagging we take the brake away.
    -- Engage gate with hysteresis: build load only on a healthy engine,
    -- drop it the moment it sags, and hand a dragged-down (or backwards
    -- spinning) engine to the sequencer's emergency path, which opens K0
    -- and restarts it with the starter.
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

    -- --- rpm loop: feed forward + PI trim ------------------------------
    -- The feed forward is the throttle the engine needs just to carry the
    -- regen brake torque; without it the integrator would have to build the
    -- whole ~30-50 % throttle from scratch and the rpm would sag first.
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
    -- Park-Charge: jump straight to a high regen load and hold the ICE at
    -- CHG.PARK_RPM with a PI loop (the engine's own idle governor fights
    -- any open-loop throttle below ~40 %).
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

  -- Brake recuperation must still win if the driver asks for more than the
  -- charge load, so take the larger of the two demands.
  local combined = max(regenFrac, _chargeRegenFrac)
  ev.etk854hybrid_evRegen = combined
  -- keep the driving-regen rate limiter in step with the charge command
  -- (it now holds Nm, not a fraction)
  if _regenSmoother then _regenSmoother:set(combined * regenEnvelopeNm) end

  if ctx.debugLevel >= 2 and (_simTime - _dbgLastLogT) >= HUDC.DBG_INTERVAL then
    _dbgLastLogT = _simTime
    log("I", "etk854hybrid.charge", string.format(
      "[CHG] parked=%s soc=%.4f tgt=%.2f headroom=%.1f regen=%.3f iceRpm=%.0f",
      tostring(isParked), soc, ctx.batteryTargetSoc, headroom, _chargeRegenFrac, rpmIce))
  end
end

-- ===========================================================
-- HUD channels (20 Hz; moved out of updateGFX: upvalue budget)
-- ===========================================================
local function updateHud(dt, torqueReqNm, motorCapNm, icePotentialNm, regenActive)
  ctx.hudTimer = ctx.hudTimer + dt
  if ctx.hudTimer < HUDC.INTERVAL then return end
  -- the display filters below run at the HUD rate, so they must integrate the
  -- time since the last HUD update, not the frame dt (2026-10-10: with the
  -- frame dt the power EMAs were ~INTERVAL/dt times slower than their tau -
  -- at 200 fps the EV power read 1.3 kW 1.5 s after the motor had stopped)
  dt = ctx.hudTimer
  ctx.hudTimer = 0
  local ev = electrics.values
  ev.etk854hybrid_mode         = modeLabel()
  ev.etk854hybrid_deMode       = ctx.deMode
  ev.etk854hybrid_edriveMode   = ctx.edriveMode
  ev.etk854hybrid_modeLegacy   = ctx.legacyMode
  -- 1 while the sequencer runs the engine (crank .. release), as before
  ev.etk854hybrid_iceState     = ctx.iceOn and 1 or 0
  ev.etk854hybrid_regenLevel   = ctx.regenLevelOrder[ctx.regenLevelIdx]
  -- ---- DSC / DTC ----
  -- Deferred coupling: applyModes() may have run before the DSE controllers
  -- existed (M.init runs in the first controller init stage).
  -- Cleared only once the coupling really went through: applyDscForDeMode
  -- returns false while the DSE is not ready yet (controllersReady gate,
  -- CMU not registered) and then keeps retrying every HUD tick.
  if ctx.dscPending and ctx.applyDsc then
    if ctx.applyDsc(ctx.deMode) then ctx.dscPending = false end
  end
  -- Read the arming state back from the DSE every HUD tick: the driver can
  -- toggle DSC with the DSC button / radial menu behind our back.
  local dmc = controller.getController and controller.getController("driveModes")
  if dmc and dmc.getCurrentDriveModeKey then
    ctx.dscKey = dmc.getCurrentDriveModeKey() or ctx.dscKey
  end
  ev.etk854hybrid_dscMode = ctx.dscLabels[ctx.dscKey] or "DSC"
  -- tcActive now means "the DSE is intervening RIGHT NOW" (torque trim or
  -- brake stage), fed by the game's own yawControl/tractionControl channels.
  ev.etk854hybrid_tcActive = (flag(ev.escActive) or flag(ev.tcsActive)) and 1 or 0
  ev.etk854hybrid_boostStage   = ctx.boostStage
  ev.etk854hybrid_targetSoc    = ctx.batteryTargetSoc

  ev.etk854hybrid_torqueRequestNm = torqueReqNm
  -- etk854hybrid_pedalShaped / _pedalEvEdge and the pedal-map view
  -- (_pedalZone, _pedalEvFrac, ...): every frame in mapPedal
  ev.etk854hybrid_evTorqueCapNm   = motorCapNm
  -- commanded split from etk854hybrid_torqueCoord (motor base / expected ICE)
  ev.etk854hybrid_evTorqueNm      = ctx.motorBaseNm or 0
  ev.etk854hybrid_iceTorqueNm     = ctx.iceExpNm or 0
  ev.etk854hybrid_iceMaxTorqueNm  = icePotentialNm
  -- jerk-limited total at the converter input AFTER the system cap. Compare
  -- against torqueRequestNm to see the cap and the limiter working.
  ev.etk854hybrid_sysTorqueNm     = ctx.torqueTargetNm or 0
  -- effective cap (base + stage bonus, 0 = off) for the 8HP program, whose own
  -- fallback (ECU baseNm or 680 + its stage table) can differ from ours
  ev.etk854hybrid_sysTorqueCapNm  = ctx.systemTorqueCapNm or 0
  ev.etk854hybrid_regenTorqueNm   = _lastRegenNm
  ev.etk854hybrid_regenRatio      = _rgn.ratio
  -- P1 diagnostics (spec 7.7)
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

  -- Phase indicator for the HUD. HYBRID/ICE only once K0 actually carries
  -- the engine (crank/sync/engage still count as electric driving).
  local phase
  if ctx.sailActive then phase = "SAIL"
  elseif ctx.chargingActive then phase = "CHARGE"
  elseif regenActive then phase = "REGEN"
  elseif ctx.iceCoupled and (ctx.motorBaseNm or 0) > 1 then phase = "HYBRID"
  elseif ctx.iceCoupled then phase = "ICE"
  else phase = "EV" end
  ev.etk854hybrid_phase = phase

  -- ICE power (EMA filtered, hard-zeroed when off)
  local rawIcePowerKw = 0
  if _iceDevice and ctx.iceOn and not (_iceDevice.isStalled == true) then
    rawIcePowerKw = ((_iceDevice.outputTorque1 or 0) * (_iceDevice.outputAV1 or 0)) / 1000
    if rawIcePowerKw < 0 then rawIcePowerKw = 0 end
  end
  -- dt-corrected smoothing: a fixed per-frame alpha is framerate-dependent
  -- (at 240 fps it barely filters and the driveline's torsional oscillation
  -- at the K0 measuring point shows through as "power flutter")
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
-- Main update loop (~60 Hz)
-- ===========================================================
-- Order (P1 spec section 2): inputs -> battery -> motor envelope -> request
-- ceiling -> sport index -> policy -> sailing -> ICE arbiter + sequencer ->
-- launch -> torque coordinator -> EV creep -> recuperation -> BATTERY CONTROL
-- -> launch hold outputs -> HUD.
-- Every new function is reached through ctx.modules: this function has to
-- stay far below LuaJIT's 60-upvalue limit (a LOAD error, the vehicle would
-- be disabled - prevalidate.py cannot see it).
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

  -- Pedal for the DEMAND path. automaticGearbox.lua:511 CAPS the throttle
  -- during upshifts (min(throttle, throttleCoefWhileShifting)); pushed
  -- through the demand limiter that cap would be ramped out and back in
  -- slowly. Take the raw driver pedal instead (throttle_input, input.lua:677)
  -- and re-apply the cap as a FAST factor on the limited torque (ctx.kShift).
  -- Arcade reverse (brake mapped to throttle) is unaffected: throttle_input
  -- is 0 there. Policy, sailing and launch keep pedalRaw.
  local pedalIn, kShiftFb = pedalRaw, 1
  local thrIn = ev.throttle_input or 0
  if shifting and thrIn > pedalRaw + 0.01 then
    pedalIn = thrIn
    kShiftFb = pedalRaw / thrIn
  end
  local kP2 = ev.etk854hybrid_shiftTorqueFactor     -- optional, from the P2 shift logic
  -- The 8HP program (etk854hybrid_8hpCore.lua C.publish) publishes its upshift
  -- envelope as etk854hybrid_shiftTorqueCoef (1 outside a shift) and no longer
  -- cuts the pedal (legacyPedalCut = false), so without this fallback no
  -- torque intervention would reach the VCU at all.
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

  -- Battery safety: MAX EV is not sustainable on an empty pack.
  if ctx.edriveMode == "MAX_EDRIVE" and soc < BATTERY_LOW_THRESHOLD then
    applyModes(ctx.deMode, "AUTO_EDRIVE", false)
    guihooks.message("Battery critical - AUTO EV", 4, "vehicle.hybridmode")
    ctx.dbg(1, "battery", "forced AUTO_EDRIVE from MAX_EDRIVE")
    edriveCfgNow = EDRIVE_CFG.AUTO_EDRIVE
  end

  -- ---------------- Motor envelope ----------------
  -- The motor rpm is the relevant shaft speed for the motor envelope; it equals
  -- the ICE rpm whenever K0 is closed and is read from the device otherwise.
  local rpmMotor = rpmIce
  if _motorDevice and type(_motorDevice.outputAV1) == "number" then
    rpmMotor = abs(_motorDevice.outputAV1) * 9.549296596
  end
  local motorCapNm, motorCurveNm = motorCapAtRpm(rpmMotor, deCfgNow)
  -- knee of the split pedal map: the cap BEFORE the redline taper below, so a
  -- light pedal near the ICE limiter does not collapse into a zero request
  local kneeRawNm = motorCapNm

  -- E-motor redline guard: the ICE rev limiter only cuts FUEL - with K0 closed
  -- the motor sits on the same shaft and would happily push the engine past
  -- its redline (user report: over-rev under e-boost near the shift point).
  -- Taper the motor cap to zero across the last ~400 rpm before the limiter;
  -- with K0 open (pure EV) the motor's own 10000 rpm envelope applies instead.
  local c = curves()
  local iceIdle = c and c.iceIdleRPM() or 800
  local iceCeil = c and c.iceMaxRPM() or 7000
  if (ev.etk854hybrid_k0ClutchRatio or 0) > 0.2 and rpmIce > iceCeil - 400 then
    motorCapNm = motorCapNm * clamp01((iceCeil - 50 - rpmIce) / 350)
  end
  ctx.rpmMotor, ctx.motorCapNm, ctx.motorCurveNm = rpmMotor, motorCapNm, motorCurveNm

  -- ---------------- Request ceiling + pedal map ----------------
  -- icePotentialNm = what the ICE could deliver once coupled, at the shaft
  -- speed K0 would drag it to (without it the ceiling would collapse to the
  -- motor cap whenever the engine is off and the demand start could never
  -- fire). The ceiling is a curve LOOKUP, so torqueCoord low-passes it.
  local iceRefRpm = max(rpmIce, min(rpmMotor, iceCeil))
  if iceRefRpm < iceIdle then iceRefRpm = iceIdle end
  local icePotentialNm = iceCapAtRpm(iceRefRpm)
  -- pure-EV mode: the sailing module must not restart the ICE either
  -- (its THROTTLE-demand and FAST exits would otherwise bypass the policy)
  ctx.evOnly = (edriveCfgNow.evOnly == true) and not (deCfgNow.iceAlwaysOn or ctx.legacyIceForce)
  local ceilRaw = motorCapNm + icePotentialNm
  -- MAX EV: the full pedal travel maps onto the E-machine
  if ctx.evOnly and ctx.iceEngageState == "OFF" then ceilRaw = motorCapNm end
  local tc = mods.torqueCoord
  local ceilF = (tc and tc.filterCeiling) and tc.filterCeiling(ctx, dt, ceilRaw) or ceilRaw
  -- COMFORT / ECO PRO in AUTO EV: split map with an EV zone (mapPedal)
  local torqueDemNm, pedalShaped = mapPedal(dt, pedalIn, deCfgNow, edriveCfgNow, kneeRawNm, ceilF, shifting)
  -- wheel-torque reference while the ICE is coupled (see demandComp)
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

  -- ---------------- sportiness -> policy -> sailing ----------------
  if mods.sportIdx and mods.sportIdx.updateGFX then mods.sportIdx.updateGFX(ctx, dt) end
  -- K0 start possible right now? (tip-in anticipation + arbiter debounce)
  local seqM = mods.iceSeq
  ctx.iceK0Window = (not ctx.seqFailsafe) and seqM ~= nil and seqM.k0WindowOpen ~= nil
                    and seqM.k0WindowOpen(ctx) or false
  ctx.policyWantsIce = policyWantsIce(deCfgNow, edriveCfgNow, ctx.iceReq == true,
                                      pedalRaw, speedKmh, torqueReqNm, motorCapNm, soc, dt)
  ev.etk854hybrid_icePolicyReason = ctx.icePolicyReason or "none"
  ev.etk854hybrid_iceDeficitS     = ctx.iceDef or 0
  ev.etk854hybrid_iceDeficitOnS   = ctx.iceDefOn or 0
  -- sport index the start side used (ECO PRO map v3: 0, sStart)
  ev.etk854hybrid_policySportS    = ctx.policySportS or 0
  if mods.sailing and mods.sailing.updateGFX then mods.sailing.updateGFX(ctx, dt) end

  -- ---------------- ICE request + start/stop sequence ----------------
  -- Replaces the old commit block (2 s transition dwell), the starter pulse
  -- timer, the starterDisabled guard and the K0 ramp.
  if ctx.seqFailsafe then
    failsafeDrive(torqueReqNm, motorCapNm, motorCurveNm, rpmIce, iceIdle)
  else
    mods.iceSeq.arbitrate(ctx, dt)
    mods.iceSeq.updateGFX(ctx, dt)
  end
  ctx.iceOnLastFrame = ctx.iceOn

  -- ---------------- Launch Control ----------------
  -- etk854hybrid_launchControl: arm / auto-arm, converter preload against
  -- the brake (PRELOAD / READY: ICE full load, E-machine generator, speed
  -- regulated to the brake capacity), FIRE window, brake-throttle guard.
  -- Publishes etk854hybrid_launchState and the ctx.launch* flags the
  -- coordinator below reads.
  local lcm = mods.launch
  if lcm and lcm.updateGFX then lcm.updateGFX(ctx, dt) end

  -- ---------------- Drive torque (jerk limit, split, throttles, fill) ------
  if not ctx.seqFailsafe then mods.torqueCoord.updateGFX(ctx, dt) end

  -- ---------------- EV creep (virtual idle) ----------------
  -- arming, HOLD and the device floor's target / limit (etk854hybrid_creep);
  -- before the recuperation, which reads ctx.creepIdleRpm for its speed fade
  if mods.creep and mods.creep.updateGFX then mods.creep.updateGFX(ctx, dt) end

  -- ---------------- Recuperation ----------------
  -- Deceleration-target model, converter lock-up assist and the one-pedal
  -- friction blend all live in updateRecuperation (see its header).
  local regenFrac, regenEnvelopeNm, regenActive =
    updateRecuperation(dt, pedalRaw, brakeRaw, speedKmh, rpmMotor, gearSign, deCfgNow)

  -- Anti-kickback: the motor device only applies regen when the drive throttle
  -- is zero, so hard-zero it once regen is meaningfully engaged. The fill
  -- loop stays in the K0 slip states (there it only cancels K0 torque and
  -- adds to whatever the device commands, regen included).
  if regenActive then
    ev.etk854hybrid_evThrottle = 0
    if not ctx.fillSlipState then ev.etk854hybrid_fillGain = 0 end
  end
  -- Drive-torque hold for the next frame (torqueCoord: drive restarts only
  -- after the DRIVER's regen has released). The BATTERY CONTROL cruise charge
  -- re-seeds the regen smoother with its own load every frame, so regenActive
  -- is permanently true while charging - that is not driver regen and must
  -- not hold the drive torque at 0 (offline harness: the car stopped).
  local chargeOnly = (ctx.chargeModeLast == "CRUISE") and (_rgn.targetDecel or 0) <= 0
  ctx.regenActiveLast = regenActive and not chargeOnly

  -- ---------------- BATTERY CONTROL charging ----------------
  updateBatteryControl(dt, edriveCfgNow, soc, rpmIce, speedKmh, speedMs, pedalRaw,
                       gearSign, regenFrac, regenEnvelopeNm, iceIdle, iceCeil)

  -- ---------------- Launch hold outputs (last writer) ----------------
  -- Recuperation, anti-kickback and BATTERY CONTROL above all write
  -- etk854hybrid_evRegen; during a launch hold the generator load must win.
  if lcm and lcm.applyOutputs then lcm.applyOutputs(ctx, dt) end

  -- ---------------- HUD channels (20 Hz) ----------------
  updateHud(dt, torqueReqNm, motorCapNm, icePotentialNm, regenActive)
end

-- ===========================================================
-- Reset
-- ===========================================================
function M.onReset()
  applyModes("COMFORT", "AUTO_EDRIVE", false)
  ctx.regenLevelIdx = 3          -- MED
  applyRegenLevel(false)
  resetRuntimeState()
  publishDefaults()

  _iceDevice = powertrain.getDevice("mainEngine")
  _motorDevice = powertrain.getDevice("integratedMotor")

  if ctx.useNewBatteryMonitor and ctx.modules.battery and ctx.modules.battery.reset then
    ctx.modules.battery.reset(ctx)
  end
  if ctx.modules.curves and ctx.modules.curves.reset then ctx.modules.curves.reset(ctx) end
  if ctx.modules.sailing and ctx.modules.sailing.reset then ctx.modules.sailing.reset(ctx) end
  for _, key in ipairs({ "sportIdx", "torqueCoord", "iceSeq", "launch", "creep" }) do
    local mod = ctx.modules[key]
    if mod and mod.reset then mod.reset(ctx) end
  end
end

-- ===========================================================
-- Physics-rate hook
-- ===========================================================
-- main.lua onPhysicsStep runs wheels.updateWheelVelocities -> powertrain.update
-- -> controller.updateWheelsIntermediate -> wheels.updateWheelTorques; the
-- controller loader collects this function (controller.lua:580-582). Only the
-- launch control's DSC hold assist uses it (desiredBrakingTorque raised to
-- the full brakeTorque in PRELOAD / READY); it returns at once otherwise.
function M.updateWheelsIntermediate(dt)
  if _lcWheelsFn then _lcWheelsFn(dt) end
end

return M
