-- ETK 854 ActiveHybrid - etk854hybrid_8hp decision core
-- ============================================================================
-- Shift program for the P2 driveline (ICE 'mainEngine' -> K0 'clutchK0' ->
-- E-machine 'integratedMotor' -> torqueConverter -> 8AT 'gearbox').
-- Programs ECO / COMFORT / SPORT / MANUAL are blended by weight (S.w) and
-- adapted by the sportiness index (S.sEff, capped per program). ECO / COMFORT
-- are torque-first (low maps, demand downshifts from a torque check); SPORT
-- has no overdrive (top gear = direct gear) and a calm relax state. With the
-- ICE decoupled an EV logic (motor operating point, saturation) replaces the
-- ICE maps; ICE starts get their own gear planning (rev cap, engagement floor).
--
-- Called from etk854hybrid_8hp.lua (copy of the base automaticGearbox.lua):
--   decide(dt, M)         gear decision (replaces determineTargetGearIndex)
--   manualOverride(g, M)  kickdown / EV lug protection in M<n>
--   lockup(dt, M)         torque-converter lock-up ratio
--   onShiftStart(g, M)    gear change time + ignition cut
--   publish(dt, M)        HUD / VCU / QA electrics, every GFX frame
--   tempManual, setProgram, getProgram, getDebug, adapt, isTempManual
--
-- Per frame: sense -> program blend + adaptation -> demand integral -> holds
-- and timers -> RULES.run (gates: shifting, K0 freeze, trailer stability,
-- launch, stop, ICE start, sailing; then lug, kickdown, fullUp, paddles,
-- SPORT top gear, ICE-start gear; then RULES.runPartLoad).
--
-- Separate from the base shift logic so no function nears LuaJIT's
-- 60-upvalue limit: tunables in K, PT, MAPS, FAMILY, P; all state in S.
-- GFX rate, variable dt; filters alpha = 1 - exp(-dt / tau). VCU channels
-- are one frame late (hybridControl order 100000, vehicleController 500).
-- Vehicle speeds m/s, rotational speeds rpm.
-- ============================================================================

local C = {}

local abs, min, max, floor, exp, sin = math.abs, math.min, math.max, math.floor, math.exp, math.sin
local PI = math.pi
local AV2RPM = 9.549296596425384
local RPM2AV = 0.104719755

-- program index order; S.w holds one blend weight per program
local PROGS = {"ECO", "COMFORT", "SPORT", "MANUAL"}

-- ----------------------------------------------------------------------------
-- Fixed calibration (not per program)
-- ----------------------------------------------------------------------------
local K = {
  UB = {0, 0.10, 0.20, 0.35, 0.50, 0.70, 0.85, 1.0}, -- map columns on the load axis u
  DE2PROG = {ECO_PRO = "ECO", COMFORT = "COMFORT", SPORT = "SPORT"},
  -- Load axis u (selectAxis):
  --  * EV logic: VCU pedalShaped (request / ceiling); the split pedal map asks
  --    only for motor torque early in the travel, the raw pedal would read
  --    that as far too much load.
  --  * ICE maps: pLoad^gamma; pLoad = raw pedal, in ECO PRO the VCU's linear-
  --    map equivalent (pedalLegacyEq), blended back to raw in the kickdown zone.
  -- Load gates use pLoad, pedal gestures S.pRaw. Gammas = the VCU's
  -- PEDAL_GAMMA_* (keep in sync).
  GAMMA = {ECO_PRO = 1.40, COMFORT = 1.00, SPORT = 0.70},
  -- IDLE = ICE idling with K0 open after sporty driving (re-engages via SYNC)
  ENG = {OFF = true, CRANK = true, SYNC = true, ENGAGE = true, BLEND = true, ON = true, RELEASE = true,
         IDLE = true},
  -- lift-off regen keeps the shaft above this rpm (the 265 Nm stage-0 motor
  -- cannot deliver MED/HIGH decel in top gear)
  REGEN_FLOOR = {OFF = 1200, LOW = 1200, MED = 1500, HIGH = 1800},
  REGEN_FLOOR_BRAKE = 1800,
  REGEN_MAX_RPM = 3200,
  -- per boost stage, from the VCU's BOOST_STAGES (keep in sync): motor corner
  -- speed [rpm] (9549.3 * kW / Nm; stage 3 = curve knee), system cap bonus [Nm]
  EV_CORNER = {[0] = 2990, 3280, 3580, 3500},
  CAP_BONUS = {[0] = 0, 100, 220, 370},
  -- fallbacks without VCU channels / jbeam system-cap section [Nm]
  EV_FALLBACK_NM = 265,
  SYS_CAP_DEFAULT = 680,
  -- reasons counted by the hunting detector (not driver or decel shifts)
  HUNT = {mapUp = true, skipUp = true, evUp = true, mapDn = true, evDn = true,
          torqueDn = true, evSat = true},
  huntPedal = 0.10, huntBrake = 0.02, -- steady driver input for a counted reversal
  -- deceleration downshifts: short re-upshift dwell (PT.tUpAfterPassiveDn)
  PASSIVE = {regenDn = true, coastDn = true, evCoast = true, brakeDn = true, lug = true,
             stop = true, sail = true, sportTop = true},
  -- upshift time (RULES.upTime): calm at load upTimeLoad0, dynamic at
  -- + upTimeLoadSpan; + upTimeStep per skipped gear
  upTimeLoad0 = 0.25, upTimeLoadSpan = 0.45, upTimeStep = 0.06,
  progRamp = 2.0,       -- program blend 0 -> 1 per second
  sRise = 0.6,          -- sEff slew +/s
  sFall = 0.15,         -- sEff slew -/s
  settle = 0.15,        -- post-shift settle before any new decision
  hyst = 150,           -- minimum up/down hysteresis in rpm
  -- skip-upshift reach in calm ECO/COMFORT: g+3 below skipU3, g+2 below skipU2 (uUp)
  skipU3 = 0.30, skipU2 = 0.60,
  upLeadMaxRPM = 250,   -- cap of the rpm gradient compensation (upCandidate)
  -- hunting: 3 reversals in huntWindow s -> dwells x2, dn thresholds -huntDn, for huntHold s
  huntWindow = 10, huntHold = 15, huntDn = 150,
  -- liftUp: pedal gate, max. steps, min. VCU off-hold left (else overlaps the K0 release)
  liftUpPedal = 0.05, liftUpMaxSteps = 3, liftUpRelMargin = 0.2,
  liftUpRelSoonS = 2.0, -- s, release due within this: target the decoupled regen floor
  -- decoupled regen upshift (regenUp): rpm floor, motor envelope share, max. steps
  regenUpMinRPM = 1500, regenUpEnvFrac = 0.90, regenUpMaxSteps = 2,
  brakeDnDecelCoupled = 1.5, -- m/s^2, ECO/COMFORT brake downshift with the ICE coupled
  -- SPORT top gear: back down once it runs <= nUpFull - margin for persistS
  sportTopMarginRPM = 600, sportTopPersistS = 0.5, sportTopPedal = 0.90, sportTopTime = 0.35,
  -- SPORT calm relax (updateHolds): steady low demand for relaxBaseS +
  -- relaxSEffS * sEff; a pedal fall > relaxDrop (pre-corner lift) ends it
  relaxPedal = 0.35, relaxRise = 0.05, relaxRiseExit = 0.12, relaxDrop = 0.10, relaxBrake = 0.05, relaxCornerFrac = 0.7,
  relaxBaseS = 3.0, relaxSEffS = 3.0,
  relaxTopExtra = 1,    -- relaxed SPORT may use the first overdrive
  -- engagement floor only for a demand start (uDn >= this, or no kUp reserve);
  -- otherwise mapUp undoes the iceDn right after the start
  engageFloorU = 0.35,
  tipinPedal = 0.45,    -- tip-in downshift: minimum load pedal (with dpdt >= kRate)
  liftPedal = 0.03,     -- lift-off rules below this raw pedal
  brakeDnPedal = 0.05,  -- brake downshifts only below this raw pedal
  sailDwell = 1.5,      -- s since the last shift before a sail preselect
  stopSpeed = 2.0,      -- m/s
  -- rule gear change times [s]
  tLug = 0.25, tStop = 0.30, tIceDn = 0.30, tRegenDn = 0.30, tCoastDn = 0.35, tBrakeDn = 0.18,
  tSail = 0.40, tPaddleDn = 0.20,
  tDnAfterDn = 0.45,    -- s after a downshift before a map / torque downshift (x2 hunting)
  regenDnHyst = 100,    -- regenDn below the regen floor - this [rpm]
  coastDwellS = 0.8,    -- s since the last shift before coastDn / evCoast
  -- EV lug: K0 open, shaft below evLugRPM at load u above evLugU; evCoast below evCoastRPM
  evLugRPM = 600, evLugU = 0.05, evCoastRPM = 900,
  -- the freeze watchdog restarts on every phase change (one phase <= ~1.2 s);
  -- the total cap only catches a sequencer that loops phases
  freezeTotalCoef = 2.5, -- total freeze cap = coef * freezeWatchdogS
  kdLatchMaxS = 3.0,    -- latched kickdown survives a full ENGAGE + BLEND
  kdDwellEco = 0.12,    -- s, ECO PRO / AUTO EV kickdown dwell (fallback for the VCU channel)
  -- ECO PRO EV power line (updateDemand): evUp rises towards min(evCorner,
  -- startCap - evPowCapMargin) as the motor fill nears full (evPowF0..+Span);
  -- below the start cap, so a kick start needs no iceUp; tau [s], SOC floor
  evPowF0 = 0.85, evPowSpan = 0.15, evPowRiseS = 0.20, evPowFallS = 1.5,
  evPowCapMargin = 250, evPowSoc = 0.20,
  startLead = {OFF = 0.9, CRANK = 0.7, SYNC = 0.35, IDLE = 0.5}, -- s until K0 closes (rpm prediction)
  -- + the torque blend: the K0 freeze lasts through BLEND
  startBlendLead = {ECO = 0.8, COMFORT = 0.6},
  startCapPedal = 0.85, -- load pedal above which no start rev cap and no S.evIce apply
  -- EV saturation band (Treq / motor torque). Where the VCU map scales with
  -- the combined ceiling the demand crosses the motor torque early: up to
  -- evUpSat x motor torque after an upshift counts as unsaturated, the
  -- saturation downshift needs evSat. MAX EV (pedal = motor only) uses *Max.
  evUpSat = 1.12,
  evSat = 1.18,
  evUpSatMax = 0.95,
  evSatMax = 0.97,
  startCapSyncMargin = 400, -- SYNC re-targets to the rev-cap gear only above cap + this
  -- iceStartDown stays this far below the cap: a start out of a lift still
  -- gains rpm in CRANK/SYNC and the SYNC re-check would shift back up
  startDnHeadroom = 500,
  evSatLeadS = 1.5,     -- s, rpm prediction for torqueDownEvIce (request + start lead + blend)
  -- TC / DSC slip holds upshifts (inflated wheel speed), at most tcUpHoldS
  -- per event and below tcUpMaxFrac * nUpFull
  tcUpHoldS = 0.5,
  tcUpMaxFrac = 0.70,
  syncCeilMargin = 500,  -- = the sequencer's SYNC_CEIL_MARGIN_RPM
  -- kickdown landing rpm, checked ahead by the growth over lead (kickTarget)
  kdAccel = 3.0,        -- m/s^2, kickdown acceleration of the car (E + ICE)
  kdHoldLead = 0.5,     -- s the new gear must at least be held (ICE coupled)
  kdStartLead = 1.0,    -- s kickdown -> K0 locked (ICE still to be started)
  pendMaxAge = 0.25,    -- a decided target older than this is not ours any more
  cornerMinSpeed = 8.3, -- m/s (30 km/h)
  -- trailer mode: kUp / kUpShift reserve factor, up-row lift [rpm] (fades with
  -- the adaptation), lug floor lift [rpm]
  trailerKUp = 0.8, trailerUpRPM = 200, trailerLugRPM = 250,
}

-- ----------------------------------------------------------------------------
-- Per-program scalars (MANUAL uses SPORT data); *Calm/*Dyn pairs are blended
-- linearly with sEff.
-- ----------------------------------------------------------------------------
local PT = {
  adaptFloor = {ECO = 0, COMFORT = 0, SPORT = 0.30, MANUAL = 0.30},
  -- torque-first ECO never adapts, COMFORT reaches 'COMFORT+' at most: one
  -- kickdown must not make them sporty for seconds
  adaptGain = {ECO = 0.0, COMFORT = 0.5, SPORT = 0.70, MANUAL = 0.70},
  sEffCap = {ECO = 0.0, COMFORT = 0.35, SPORT = 1.0, MANUAL = 1.0},
  -- uncapped gains, only for the lock-up tip-in slip gate (C.lockup): with the
  -- capped sEff every COMFORT kickdown into 1st-3rd would get the slip
  slipAdaptGain = {ECO = 0.6, COMFORT = 1.0, SPORT = 0.70, MANUAL = 0.70},
  -- torque reserve: a target gear must carry the request within kUp x Tav;
  -- torque downshift once the request exceeds kDn x Tav
  kUpCalm = {ECO = 0.85, COMFORT = 0.75, SPORT = 0.60, MANUAL = 0.60},
  kUpDyn = {ECO = 0.75, COMFORT = 0.62, SPORT = 0.48, MANUAL = 0.48},
  kDnCalm = {ECO = 0.98, COMFORT = 0.95, SPORT = 0.85, MANUAL = 0.85},
  kDnDyn = {ECO = 0.92, COMFORT = 0.88, SPORT = 0.75, MANUAL = 0.75},
  kRate = {ECO = 3.5, COMFORT = 2.5, SPORT = 1.5, MANUAL = 3.0}, -- tip-in pedal rate [1/s], lowered with sEff
  -- corner hold above this lateral acceleration [m/s^2]
  cornerCalm = {ECO = 4.5, COMFORT = 4.0, SPORT = 3.0, MANUAL = 3.0},
  cornerDyn = {ECO = 3.0, COMFORT = 2.5, SPORT = 2.0, MANUAL = 2.0},
  -- upshift ratio-blend time [s]
  upTimeCalm = {ECO = 0.32, COMFORT = 0.30, SPORT = 0.14, MANUAL = 0.14},
  upTimeDyn = {ECO = 0.24, COMFORT = 0.20, SPORT = 0.10, MANUAL = 0.10},
  -- minimum time between two upshifts [s] from the shift start; short, or
  -- every gear overshoots its map point under acceleration
  tUpAfterUp = {ECO = 0.30, COMFORT = 0.35, SPORT = 0.40, MANUAL = 0.40},
  -- re-upshift dwell after a demand downshift [s]
  tUpAfterDnCalm = {ECO = 1.0, COMFORT = 1.2, SPORT = 2.0, MANUAL = 2.0},
  tUpAfterDnDyn = {ECO = 1.0, COMFORT = 2.0, SPORT = 4.0, MANUAL = 4.0},
  -- after a passive downshift; SPORT / MANUAL keep tUpAfterDn
  tUpAfterPassiveDn = {ECO = 0.30, COMFORT = 0.35},
  -- upshift / skip-upshift debounce [s]
  debUp = {ECO = 0.15, COMFORT = 0.20, SPORT = 0.20, MANUAL = 0.20},
  debSkip = {ECO = 0.25, COMFORT = 0.30, SPORT = 0.40, MANUAL = 0.40},
  tdPersist = {ECO = 0.50, COMFORT = 0.30, SPORT = 0.20, MANUAL = 0.20}, -- torque downshift persistence [s]
  -- demand integral: window intW0 + 3.5 * sEff [s], weight intW + 0.5 * sEff
  intW0 = {ECO = 1.0, COMFORT = 1.0, SPORT = 1.5, MANUAL = 1.5},
  intW = {ECO = 0.0, COMFORT = 0.3, SPORT = 0.5, MANUAL = 0.5},
  cornerRel = {ECO = 0.6, COMFORT = 1.0, SPORT = 1.0, MANUAL = 1.0}, -- corner hold release [s] (+ sEff)
  -- coupled overrun upshift delay after the lift [s] (liftUp: ECO/COMFORT only)
  liftUpDelay = {ECO = 0.30, COMFORT = 0.60, SPORT = 0, MANUAL = 0},
  -- decoupled regen upshift above this rpm (SPORT only while relaxed)
  regenUpRPM = {ECO = 2600, COMFORT = 2800, SPORT = 3200, MANUAL = 3200},
  -- minimum time after an upshift before a map / torque downshift [s]
  tDnAfterUpCalm = {ECO = 1.2, COMFORT = 1.2, SPORT = 0.8, MANUAL = 0.8},
  tDnAfterUpDyn = {ECO = 0.8, COMFORT = 0.8, SPORT = 0.8, MANUAL = 0.8},
  -- deepest torque coefficient of the upshift torque intervention
  cMinCalm = {ECO = 0.75, COMFORT = 0.75, SPORT = 0.50, MANUAL = 0.50},
  cMinDyn = {ECO = 0.55, COMFORT = 0.55, SPORT = 0.50, MANUAL = 0.50},
  brakeSpacing = {ECO = 0.30, COMFORT = 0.30, SPORT = 0.25, MANUAL = 0.25}, -- s between brake-downshift steps
  -- downshift times [s]: tip-in, map / torque; kickdown = base + step x (gears - 1)
  dnTimeTip = {ECO = 0.22, COMFORT = 0.20, SPORT = 0.15, MANUAL = 0.15},
  dnTimeDemand = {ECO = 0.26, COMFORT = 0.24, SPORT = 0.18, MANUAL = 0.18},
  kdTimeBase = {ECO = 0.18, COMFORT = 0.18, SPORT = 0.12, MANUAL = 0.12},
  kdTimeStep = {ECO = 0.03, COMFORT = 0.03, SPORT = 0.02, MANUAL = 0.02},
}

-- ----------------------------------------------------------------------------
-- Shift maps [rpm], columns = K.UB on u. "up" rows: POST-shift rpm in gear
-- g+1; "dn" rows: rpm in gear g below which g -> g-1. -1 = full-load column
-- F[g] = nUpFull * R[g+1] / R[g]. Values above the family knee are rescaled
-- at init to the engine's nUpFull. Torque-first ECO / COMFORT: low flat up
-- rows, dn rows near the lug floor (demand downshifts: RULES.torqueDownIce).
-- tools/etk854hybrid_shiftmap_check.py parses this block (keep the format:
-- "name = {" / "}," / "name = {n, n, ...},").
-- ----------------------------------------------------------------------------
-- BEGIN 8HP MAPS
local MAPS = {
  petrol = {
    ECO = {
      upCalm = {1100, 1150, 1250, 1350, 1500, 1750, 2200, -1},
      upDyn = {1150, 1250, 1350, 1500, 1700, 2000, 2600, -1},
      dnCalm = {950, 960, 1000, 1080, 1180, 1350, 1700, 2300},
      dnDyn = {1000, 1050, 1100, 1200, 1350, 1600, 2000, 2800},
    },
    COMFORT = {
      upCalm = {1100, 1150, 1250, 1400, 1600, 1900, 2500, -1},
      upDyn = {1250, 1400, 1600, 1900, 2250, 2800, 3500, -1},
      dnCalm = {950, 980, 1030, 1120, 1250, 1450, 1850, 2600},
      dnDyn = {1050, 1150, 1300, 1550, 1850, 2300, 2900, 3600},
    },
    SPORT = {
      upCalm = {1500, 1650, 1850, 2200, 2600, 3200, 4000, -1},
      upDyn = {2100, 2300, 2600, 3000, 3500, 4200, 4900, -1},
      dnCalm = {1300, 1400, 1600, 1950, 2400, 2900, 3800, 4400},
      dnDyn = {1700, 1850, 2150, 2600, 3200, 3900, 4500, 4900},
    },
  },
  diesel = {
    ECO = {
      upCalm = {1050, 1080, 1120, 1180, 1250, 1400, 1700, -1},
      upDyn = {1100, 1130, 1180, 1250, 1350, 1550, 1900, -1},
      dnCalm = {900, 910, 940, 990, 1050, 1150, 1400, 1900},
      dnDyn = {920, 940, 980, 1050, 1150, 1300, 1600, 2200},
    },
    COMFORT = {
      upCalm = {1050, 1100, 1150, 1230, 1330, 1520, 1900, -1},
      upDyn = {1150, 1250, 1400, 1600, 1850, 2250, 2800, -1},
      dnCalm = {900, 930, 970, 1040, 1120, 1250, 1550, 2100},
      dnDyn = {960, 1000, 1080, 1200, 1350, 1600, 2000, 2600},
    },
    SPORT = {
      upCalm = {1400, 1500, 1650, 1850, 2100, 2600, 3200, -1},
      upDyn = {1800, 1950, 2150, 2450, 2800, 3300, 3900, -1},
      dnCalm = {1150, 1250, 1400, 1700, 1900, 2400, 2900, 3700},
      dnDyn = {1500, 1650, 1850, 2250, 2600, 3100, 3700, 4100},
    },
  },
}
-- END 8HP MAPS

-- Engine-family defaults (petrol = B58, diesel = N57), overridden by the ECU
-- part's vehicleController block "etk854hybrid8hpEngine" (or the powertrain
-- part's default block, see initParams).
--   lugMinRPM       lowest coupled rpm (stall protection, dn threshold floor)
--   coastDnRPM      coupled overrun downshift below this
--   evLugBonusRPM   max. threshold lowering while the E-machine covers the demand
--   turboRampRPM    boost build-up range, lagMin = derate at its start (reserve checks)
--   resvMaxRPM      torque-reserve checks only below this
--   knee / ref      map rescale: ref lands on the engine's nUpFull
--   brakeTargetRPM  [calm, dyn] brake-downshift target; ECO/COMFORT calm =
--                   coupled regen floor under the brake
--   coastTargetRPM  coupled overrun: liftUp target and regen floor
--   startCapRPM     ECO/COMFORT ICE-start rev cap (<= iceStartCapRPM)
--   iceEngageMinRPM ECO/COMFORT lowest engagement rpm of a demand ICE start
--   sportRelaxRPM   relaxed SPORT upshift floor (also into the relaxed overdrive)
local FAMILY = {
  petrol = {lugMinRPM = 950, coastDnRPM = 1050, evLugBonusRPM = 250, turboRampRPM = {1200, 1800},
            lagMin = 0.55, resvMaxRPM = 2400, knee = 3000, ref = 6100,
            brakeTargetRPM = {ECO = {1400, 1700}, COMFORT = {1600, 2000}, SPORT = {2600, 3600}},
            coastTargetRPM = {ECO = 1300, COMFORT = 1500, SPORT = 2200},
            startCapRPM = {ECO = 2800, COMFORT = 3300},
            iceEngageMinRPM = {ECO = 1450, COMFORT = 1550},
            sportRelaxRPM = 2000},
  diesel = {lugMinRPM = 900, coastDnRPM = 950, evLugBonusRPM = 150, turboRampRPM = {1100, 1600},
            lagMin = 0.60, resvMaxRPM = 2100, knee = 2500, ref = 4900,
            brakeTargetRPM = {ECO = {1250, 1450}, COMFORT = {1350, 1650}, SPORT = {2100, 3000}},
            coastTargetRPM = {ECO = 1200, COMFORT = 1300, SPORT = 1800},
            startCapRPM = {ECO = 2400, COMFORT = 2800},
            iceEngageMinRPM = {ECO = 1250, COMFORT = 1350},
            sportRelaxRPM = 1700},
}

local P = {}      -- parameters, rebuilt in init
local S = {}      -- runtime state, rebuilt in init (runs again on every reset)
local D = {}      -- device references and precomputed gear data
local RULES = {}  -- decision rules (fields keep the chunk's local count low)

-- ----------------------------------------------------------------------------
-- Helpers
-- ----------------------------------------------------------------------------
-- electrics values may be nil, bool, NaN or string
local function num(x, def)
  if type(x) == "number" and x == x and x > -1e30 and x < 1e30 then return x end
  return def
end

local function str(x, def)
  if type(x) == "string" and x ~= "" then return x end
  return def
end

local function clamp(x, lo, hi)
  if x < lo then return lo end
  if x > hi then return hi end
  return x
end

local function clamp01(x)
  if x < 0 then return 0 end
  if x > 1 then return 1 end
  return x
end

local function lerp(a, b, t)
  return a + (b - a) * t
end

-- first-order low-pass coefficient for a variable GFX dt
local function alpha(dt, tau)
  if tau <= 0 then return 1 end
  return 1 - exp(-dt / tau)
end

-- base game flags are bool or 0/1 numbers depending on the controller generation
local function truthy(x)
  return x == true or (type(x) == "number" and x > 0.5)
end

-- piecewise-linear row lookup on K.UB; -1 entries stand for the full-load F
local function interpRow(row, u, F)
  local UB = K.UB
  if u <= 0 then
    local v = row[1]
    return (v < 0) and F or v
  end
  for i = 2, 8 do
    local ub = UB[i]
    if u <= ub then
      local a, b = row[i - 1], row[i]
      if a < 0 then a = F end
      if b < 0 then b = F end
      local ua = UB[i - 1]
      return a + (b - a) * (u - ua) / (ub - ua)
    end
  end
  local v = row[8]
  return (v < 0) and F or v
end

-- program-blended scalar: sum over programs of weight * value
local function pv(tbl)
  local w = S.w
  local acc = 0
  for i = 1, 4 do
    local p = PROGS[i]
    local wp = w[p]
    if wp > 0 then acc = acc + wp * (tbl[p] or 0) end
  end
  return acc
end

-- program-blended calm/dyn pair at the current adaptation
local function pvs(calm, dyn)
  return lerp(pv(calm), pv(dyn), S.sEff)
end

-- re-upshift dwell after a passive downshift; SPORT / MANUAL keep tUpAfterDn
local function passiveDwell()
  local w, sEff, pd = S.w, S.sEff, PT.tUpAfterPassiveDn
  local acc = 0
  for i = 1, 4 do
    local p = PROGS[i]
    local wp = w[p]
    if wp > 0 then
      acc = acc + wp * (pd[p] or lerp(PT.tUpAfterDnCalm[p], PT.tUpAfterDnDyn[p], sEff))
    end
  end
  return acc
end

-- program-blended map value (kind 1 = up, 2 = dn) at adaptation sEff;
-- callers pass S.sEffUp for the up rows (0 while SPORT is relaxed)
local function mapVal(kind, u, F, sEff)
  local w, maps = S.w, D.maps
  local acc = 0
  for i = 1, 4 do
    local p = PROGS[i]
    local wp = w[p]
    if wp > 0 then
      local m = maps[p]
      local c, d
      if kind == 1 then c, d = m.upCalm, m.upDyn else c, d = m.dnCalm, m.dnDyn end
      acc = acc + wp * lerp(interpRow(c, u, F), interpRow(d, u, F), sEff)
    end
  end
  return acc
end

-- ----------------------------------------------------------------------------
-- Torque model
-- ----------------------------------------------------------------------------
-- steady-state ICE curve incl. turbo, cached every 25 rpm at init
local function iceCurve(n)
  local tab = D.iceTab
  if not tab then return 0 end
  local x = n / 25
  if x <= 0 then return tab[0] end
  local i = floor(x)
  if i >= D.iceN then return tab[D.iceN] end
  local a = tab[i]
  return a + (tab[i + 1] - a) * (x - i)
end

-- turbo-lag derate for the reserve checks (a part-load upshift lands at low boost)
local function lag(n)
  return clamp((n - P.ramp1) / max(P.ramp2 - P.ramp1, 1), P.lagMin, 1)
end

-- derated steady-state ICE torque (reserve checks)
local function iceShare(n)
  return 0.95 * iceCurve(n) * lag(n)
end

local function iceAvail(n)
  local es = S.engState
  if es ~= "ON" and es ~= "BLEND" and not S.icePending then return 0 end
  return iceShare(n)
end

-- motor torque at shaft speed n, extrapolated from the VCU's availability
-- at nShaft (constant torque below the corner speed, constant power above)
local function evAt(n)
  local ea = S.evAvail
  if ea <= 0 then return 0 end
  local nC = S.evCorner
  local nN = max(S.nShaft, nC)
  return min(ea * nN / nC, ea * nN / max(n, 10))
end

-- what the system can deliver at shaft speed n right now (ICE only if running)
local function Tav(n)
  return min(S.sysCap, iceAvail(n) + evAt(n))
end

-- what the system will deliver once the pending ICE start has coupled
local function TavPend(n)
  return min(S.sysCap, iceShare(n) + evAt(n))
end

-- kickdown potential: the ICE counts because the VCU will start it (not in MAX EV)
local function TavKD(n)
  local ice = 0
  local e = D.engine
  if S.edrive ~= "MAX_EDRIVE" and not (e and e.isDisabled) then ice = 0.95 * iceCurve(n) end
  return min(S.sysCap, ice + evAt(n))
end

-- ----------------------------------------------------------------------------
-- Effective thresholds
-- ----------------------------------------------------------------------------
-- threshold lowering for gear k while the motor alone could carry the
-- (integrated) demand; faded out with low SOC and with sportiness
local function lugBonus(nGear, k)
  local need = max(S.DwUp / D.R[k], 1)
  return P.evLugBonus * clamp01((evAt(nGear) / need - 0.8) / 0.4)
    * clamp01((S.soc - 0.10) / 0.10) * (1 - S.sEff)
end

-- $shift8hpOffset shifts the part-load columns, fades out towards full load
local function offset(u)
  return P.offsetRPM * (1 - clamp01((u - 0.70) / 0.15))
end

-- post-shift rpm threshold for the upshift g -> g+1 (rpm in gear g+1)
local function nUpPost(g, u, bonus)
  local F = D.F[g] or P.nUpFull
  local base = offset(u) - bonus
  local v = mapVal(1, u, F, S.sEffUp) + base + K.trailerUpRPM * S.trailer * (1 - S.sEffUp)
  if S.sportRelax then
    -- calm rows lifted to the relax floor, never above the active threshold
    local act = mapVal(1, u, F, S.sEff) + base + K.trailerUpRPM * S.trailer * (1 - S.sEff)
    v = min(act, max(v, P.sportRelaxRPM))
    -- the relaxed overdrive needs at least the relax floor
    if g >= D.topSport then v = max(v, P.sportRelaxRPM) end
  end
  return min(F, max(S.upFloor, v))
end

-- downshift threshold g -> g-1 (rpm in gear g); the clamp below the g-1 -> g
-- upshift threshold guarantees >= K.hyst of hysteresis
local function nDn(g, u, bonus)
  local v = mapVal(2, u, 0, S.sEff) + offset(u) - bonus - (S.hunt and K.huntDn or 0)
  local up = (g > 1) and nUpPost(g - 1, u, bonus) or 1e9
  return min(max(S.lugMinEff, v), up - K.hyst)
end

-- ----------------------------------------------------------------------------
-- Sensing (every decision frame)
-- ----------------------------------------------------------------------------
local function senseDriver(dt, M)
  local ev = electrics.values
  local de = ev.etk854hybrid_deMode
  if de ~= "ECO_PRO" and de ~= "SPORT" then de = "COMFORT" end
  S.deMode = de
  local ed = ev.etk854hybrid_edriveMode
  if ed ~= "MAX_EDRIVE" and ed ~= "BATTERY_CONTROL" then ed = "AUTO_EDRIVE" end
  S.edrive = ed

  -- M.throttle at decision time = driver pedal (arcade-swapped in reverse)
  local p = clamp01(num(M.throttle, 0))
  S.pRaw = p
  S.brake = clamp01(num(M.brake, 0))
  -- load pedal (K.GAMMA); pedalLegacyEq -1 = not published
  local pl = (de == "ECO_PRO") and num(ev.etk854hybrid_pedalLegacyEq, -1) or -1
  local pLoad = p
  if p > 0 and pl >= 0 then
    pLoad = clamp01(pl)
    local kz = clamp01((p - P.kickdownPedal) / max(1 - P.kickdownPedal, 0.01))
    if kz > 0 and p > pLoad then pLoad = pLoad + (p - pLoad) * kz end
  end
  S.pLoad = pLoad
  S.evFracRaw = clamp01(num(ev.etk854hybrid_pedalEvFrac, 0)) -- motor fill (EV power line)
  -- the two load axes; selectAxis picks one after senseHybrid
  local uPed = (pLoad > 0) and pLoad ^ (K.GAMMA[de] or 1) or 0
  S.uPed = uPed
  local uSh = uPed
  if p > 0 then
    local ps = num(ev.etk854hybrid_pedalShaped, nil)
    if ps ~= nil then uSh = clamp01(ps) end
  end
  S.uShaped = uSh
  S.pSm = S.pSm + (p - S.pSm) * alpha(dt, 0.25)
  local pf = S.pF + (p - S.pF) * alpha(dt, 0.03)
  S.dpdt = S.dpdt + ((pf - S.pF) / dt - S.dpdt) * alpha(dt, 0.08)
  S.pF = pf

  local v = abs(num(ev.wheelspeed, 0))
  S.dvF = S.dvF + ((v - S.vPrev) / dt - S.dvF) * alpha(dt, 0.15)
  S.vPrev = v
  S.v = v
  S.decel = -S.dvF
  -- synchronous rpm rate from the vehicle acceleration (S.n jumps at shifts)
  S.nDotG = (v > 1) and max(0, S.dvF) * S.n / v or 0
  -- lateral acceleration is sensors.gx2 (gy2 is longitudinal)
  local sn = sensors
  local gx = (type(sn) == "table") and num(sn.gx2, 0) or 0
  S.gxS = S.gxS + (abs(gx) - S.gxS) * alpha(dt, 0.3)

  local spd = M.shiftPreventionData
  S.slipUpOK = not (type(spd) == "table" and spd.wheelSlipShiftUp == false)
  S.slipDnOK = not (type(spd) == "table" and spd.wheelSlipShiftDown == false)
end

-- load axis for this frame (K.GAMMA). On a switch the filters restart on the
-- new axis: a step through uSm / uInt would read as a tip-in
local function selectAxis(dt)
  local ice = not S.evLogic
  local u = ice and S.uPed or S.uShaped
  if S.axisIce ~= ice then
    S.axisIce = ice
    S.uSm, S.uInt = u, u
  end
  S.u = u
  S.uSm = S.uSm + (u - S.uSm) * alpha(dt, 0.25)
end

local function senseGearbox()
  local gb, R = D.gearbox, D.R
  local g = gb.gearIndex or 0
  S.g = g
  -- synchronous turbine rpm: converter slip must not move shift points
  S.n = abs(num(gb.outputAV1, 0)) * abs(R[g] or 0) * AV2RPM
  S.nIn = num(gb.inputAV, 0) * AV2RPM
  local tc = D.tc
  S.nShaft = tc and num(tc.inputAV, 0) * AV2RPM or S.nIn
  local e = D.engine
  S.nIce = e and num(e.outputAV1, 0) * AV2RPM or 0
  local mm = tonumber(D.AH.maxGearIndex) or gb.maxGearIndex
  S.modeMax = min(mm, gb.maxGearIndex)
end

local function senseHybrid(dt)
  local ev = electrics.values
  local k0 = clamp01(num(ev.etk854hybrid_k0ClutchRatio, 1))
  S.k0 = k0
  local es = ev.etk854hybrid_iceEngageState
  if type(es) == "string" and K.ENG[es] then
    S.engFallback = false
  else
    -- VCU channel not (yet) published: derive it from ICE state + K0 ratio
    S.engFallback = true
    local iceOn = num(ev.etk854hybrid_iceState, 1) >= 0.5
    if num(ev.etk854hybrid_iceRestartActive, 0) >= 0.5 then es = "ENGAGE"
    elseif iceOn then es = (k0 > 0.95) and "ON" or "ENGAGE"
    elseif k0 > 0.05 then es = "RELEASE"
    else es = "OFF" end
  end
  S.engState = es
  local iceOff = (es == "OFF" or es == "CRANK" or es == "SYNC" or es == "IDLE")
  S.coupled = k0 >= 0.5 and es == "ON"
  -- stall protection must also act while the ICE is dragged mid-transition
  S.lugCoupled = k0 >= 0.5 and not iceOff
  S.isEV = k0 < 0.05 and iceOff
  -- ICE start requested or in progress: plan with the ICE maps + pending torque
  local req = ev.etk854hybrid_iceRequested
  local reqOn = (req == true) or num(req, 0) >= 0.5
  local starting = (es == "CRANK" or es == "SYNC")
  local e = D.engine
  S.icePending = (starting or (reqOn and (es == "OFF" or es == "IDLE"))) and not (e and e.isDisabled)
  S.evLogic = S.isEV and not S.icePending
  if starting then S.startT = S.startT + dt else S.startT = 0 end
  S.startFreeze = starting and S.startT < P.startFreezeS
  S.syncBlocked = num(ev.etk854hybrid_iceSyncBlocked, 0) >= 0.5
  S.lead = (K.startLead[es] or 0) + (K.startBlendLead[S.prog or "COMFORT"] or 0)

  S.sail = str(ev.etk854hybrid_sailState, "DRIVE")
  local ls = str(ev.etk854hybrid_launchState, "IDLE")
  S.launchT = (ls == "ACTIVE") and (S.launchT + dt) or 0
  S.launch = ls
  S.regenLvl = str(ev.etk854hybrid_regenLevel, "MED")
  -- pedal gate: cruise charging also writes evRegen (generator load, not regen)
  local rq = max(0, num(ev.etk854hybrid_regenTorqueNm, 0))
  S.regenTq = rq
  S.regenOn = S.pRaw < 0.05 and S.v > 4.2
    and (num(ev.etk854hybrid_evRegen, 0) > 0.05 or rq > 20)
  -- VCU ICE-off hold: kind none | hold | lift | brake, time left [s]
  S.holdKind = str(ev.etk854hybrid_iceHoldKind, "none")
  S.holdRemain = max(0, num(ev.etk854hybrid_iceOffHoldS, 0))

  local soc = num(ev.etk854hybrid_batteryFraction, nil)
  if soc == nil then
    local pct = num(ev.etk854hybrid_batterySoc, nil)
    soc = pct and pct * 0.01 or 1
  end
  S.soc = clamp01(soc)
  local st = floor(clamp(num(ev.etk854hybrid_boostStage, 0), 0, 3) + 0.5)
  S.stage = st
  S.evCorner = K.EV_CORNER[st] or 2990
  S.charging = ev.etk854hybrid_chargeMode == "CRUISE"
  local tm = ev.etk854hybrid_trailerMode
  S.trailer = (tm == true or num(tm, 0) >= 0.5) and 1 or 0
  -- DSC slip control: the cluster's qualified lamp (raw escActive / tcsActive
  -- rise on any tiny intervention), else the raw flags, else the VCU copy
  local lamp = ev.etk854hybrid_dscLamp
  local esc, tcs = ev.escActive, ev.tcsActive
  if type(lamp) == "number" then
    S.tc = lamp >= 0.5
  elseif esc == nil and tcs == nil then
    S.tc = num(ev.etk854hybrid_tcActive, 0) >= 0.5
  else
    S.tc = truthy(esc) or truthy(tcs)
  end
  if S.tc then S.tcT = S.tcT + dt else S.tcT = 0 end
  S.tsa = num(ev.etk854hybrid_tsaFreeze, 0) >= 0.5 -- trailer stability gear hold
  -- qualified ABS from the VCU: the raw flag rises at ~1 % slip and would
  -- open the lock-up under full regen
  local absQ = ev.etk854hybrid_absQualified
  if absQ ~= nil then S.abs = truthy(absQ) else S.abs = truthy(ev.absActive) end
  S.source = (S.sail == "SAIL") and "SAIL" or (S.isEV and "EV" or "ICE")

  -- K0 freeze: no gear change while K0 engages / releases or sailing moves it
  -- (the sync target would jump mid-slip). SAIL_EXIT only while K0 slips:
  -- with K0 open the car is EV and may kick down. frzT restarts on every
  -- phase change.
  local frozen, why = false, ""
  if es == "ENGAGE" or es == "BLEND" or es == "RELEASE" then frozen, why = true, "K0"
  elseif S.sail == "SAIL_ENTER" then frozen, why = true, "SAILTRANS"
  elseif S.sail == "SAIL_EXIT" and k0 > 0.05 and k0 < 0.95 then frozen, why = true, "SAILTRANS"
  elseif S.engFallback and k0 > 0.05 and k0 < 0.95 then frozen, why = true, "K0" end
  if frozen then
    local phase = why .. "/" .. tostring(es) .. "/" .. tostring(S.sail)
    if phase ~= S.frzPhase then S.frzT = 0 end
    S.frzPhase = phase
    S.frzT = S.frzT + dt
    S.frzTotT = S.frzTotT + dt
  else
    S.frzT, S.frzTotT, S.frzPhase = 0, 0, ""
    S.frzLogged = false
  end
  S.frozen, S.freezeWhy = frozen, why

  S.lugMinEff = max(P.lugMin, 1.30 * P.idle) + K.trailerLugRPM * S.trailer + (S.charging and 150 or 0)
  S.upFloor = S.lugMinEff + 150
end

-- local sportiness index, used only while the VCU's etk854hybrid_sportIndex
-- is nil. Rise tau 1.5 s; decay 6..25 s, growing with sustained sporty
-- driving (history h: 45 s up / 120 s down). Frozen at standstill.
local function sportLocal(dt)
  local L = S.sl
  if S.v < 1 then return L.I end
  -- pedal^gamma: independent of the VCU's EV pedal zone
  local x = max(clamp01((S.uPed - 0.25) / 0.60),
                0.9 * clamp01(max(0, S.dpdt) / 3.0),
                0.8 * clamp01((S.gxS - 2.5) / 4.0),
                (S.v > 8.3) and 0.6 * clamp01((S.decel - 2.5) / 4.0) or 0)
  local tau = (x > L.I) and 1.5 or (6 + 19 * L.h)
  L.I = L.I + (x - L.I) * alpha(dt, tau)
  local hx = clamp01((L.I - 0.45) / 0.55)
  L.h = L.h + (hx - L.h) * alpha(dt, (hx > L.h) and 45 or 120)
  return L.I
end

local function senseTorque(dt)
  local ev = electrics.values
  local ea = num(ev.etk854hybrid_evTorqueAvailNm, nil)
  if ea == nil then ea = num(ev.etk854hybrid_evTorqueCapNm, nil) end
  if ea == nil then ea = K.EV_FALLBACK_NM * clamp01(num(ev.etk854hybrid_evPowerFraction, 1)) end
  local avail = ev.etk854hybrid_evAvailable
  if avail == 0 or avail == false then ea = 0 end
  S.evAvail = max(0, ea)

  local cap = num(ev.etk854hybrid_sysTorqueCapNm, nil)
  if cap == nil then cap = P.sysCapBase + (K.CAP_BONUS[S.stage] or 0) end
  if cap <= 0 then cap = 1e6 end -- VCU: 0 disables the cap
  S.sysCap = cap

  -- VCU pedal-map demand, not the coupled request raised at low rpm (that
  -- one must not trigger torque downshifts); fallback: load-axis model
  local tr = num(ev.etk854hybrid_torqueRequestMapNm, nil) or num(ev.etk854hybrid_torqueRequestNm, nil)
  if tr == nil then
    local ref = clamp(max(S.nIce, min(S.nShaft, P.lim)), P.idle, P.lim)
    tr = S.u * (S.evAvail + iceCurve(ref))
  end
  S.Treq = max(0, tr)
  -- ICE axis: plan with the request of a linear pedal^gamma map; the split
  -- map's lower request would upshift the coupled engine too early
  if S.axisIce and S.uShaped > 0.01 and S.u > S.uShaped then
    S.Treq = S.Treq * S.u / S.uShaped
  end
  S.Dw = S.Treq * abs(D.R[S.g] or 0)
  S.evSat = S.Treq >= ((S.edrive == "MAX_EDRIVE") and K.evSatMax or K.evSat) * evAt(S.n)

  local si = num(ev.etk854hybrid_sportIndex, nil)
  local loc = sportLocal(dt)
  S.sLocal = loc
  if si == nil then
    S.sIdx, S.sportFromLocal = loc, true
  else
    S.sIdx, S.sportFromLocal = clamp01(si), false
  end
end

-- ----------------------------------------------------------------------------
-- Program selection, adaptation, demand integral
-- ----------------------------------------------------------------------------
-- uncapped adaptation target for a gain table (PT.adaptGain / PT.slipAdaptGain)
function RULES.adaptRaw(gain)
  return pv(PT.adaptFloor) + pv(gain) * S.sIdx * P.adaptGainUser
end

local function updateProgram(dt)
  local mode = D.AH.mode
  local prog
  if S.tm.active or (type(mode) == "string" and mode:sub(1, 1) == "M") then prog = "MANUAL"
  elseif mode == "S" then prog = "SPORT"
  elseif S.override then prog = S.override
  else prog = K.DE2PROG[S.deMode] or "COMFORT" end

  local w, wf = S.w, S.wFrom
  if S.prog == nil then
    -- first frame after init: no ramp from an arbitrary default
    for i = 1, 4 do
      local p = PROGS[i]
      w[p] = (p == prog) and 1 or 0
    end
    S.prog, S.lambda = prog, 1
  elseif prog ~= S.prog then
    -- snapshot the current (possibly mid-ramp) blend and ramp to the new one
    for i = 1, 4 do
      local p = PROGS[i]
      wf[p] = w[p]
    end
    S.prog, S.lambda = prog, 0
  end
  if S.lambda < 1 then
    S.lambda = min(1, S.lambda + K.progRamp * dt)
    for i = 1, 4 do
      local p = PROGS[i]
      w[p] = wf[p] + (((p == prog) and 1 or 0) - wf[p]) * S.lambda
    end
  end

  -- adaptation, capped per program; the cap applies to sEff itself, else
  -- SPORT -> COMFORT would show 'COMFORT++' while sEff slews down
  local cap = pv(PT.sEffCap)
  local target = min(cap, clamp01(RULES.adaptRaw(PT.adaptGain)))
  S.sEff = min(cap, clamp01(S.sEff + clamp(target - S.sEff, -K.sFall * dt, K.sRise * dt)))

  -- HUD suffix '' / '+' / '++' with 0.05 hysteresis
  local lvl, se = S.suffixLevel, S.sEff
  if lvl < 2 and se >= 0.66 then lvl = 2
  elseif lvl < 1 and se >= 0.33 then lvl = 1
  elseif lvl == 2 and se < 0.61 then lvl = (se < 0.28) and 0 or 1
  elseif lvl == 1 and se < 0.28 then lvl = 0 end
  S.suffixLevel = lvl
  if prog == "MANUAL" then S.progLabel = "MANUAL"
  else S.progLabel = prog .. ((lvl == 2) and "++" or ((lvl == 1) and "+" or "")) end

  local hm = S.hunt and 2 or 1
  S.kUp = pvs(PT.kUpCalm, PT.kUpDyn) * ((S.trailer > 0) and K.trailerKUp or 1)
  S.kDn = pvs(PT.kDnCalm, PT.kDnDyn)
  S.kRate = pv(PT.kRate) * lerp(1, 0.6, S.sEff)
  S.tUpAfterUp = pv(PT.tUpAfterUp) * hm
  -- re-upshift dwell: held after a demand downshift, brief after a passive one
  S.tUpAfterDn = (S.lastDnPassive and passiveDwell() or pvs(PT.tUpAfterDnCalm, PT.tUpAfterDnDyn)) * hm
  S.tDnAfterUp = pvs(PT.tDnAfterUpCalm, PT.tDnAfterUpDyn) * hm
  S.tDnAfterDn = K.tDnAfterDn * hm
  S.cMin = (mode == "S" or prog == "MANUAL") and 0.5 or pvs(PT.cMinCalm, PT.cMinDyn)
  -- highest selectable gear: SPORT has no overdrive (D.topSport), the gears
  -- above only through fullUp; relaxed SPORT adds K.relaxTopExtra
  S.topGear = (prog == "SPORT") and min(D.topSport, S.modeMax) or S.modeMax
  -- program-blended engine calibration (initParams)
  S.startCap = pv(P.startCap)
  S.engageMin = pv(P.engageMin)
  S.coastTarget = pv(P.coastTarget)
  S.brakeCalmP = pv(P.brakeCalm)
end

-- short demand integral: the window grows with the adaptation and holds the
-- gear after demand peaks (up-axis uUp); the downshift axis uDn stays fast
local function updateDemand(dt)
  local W = pv(PT.intW0) + 3.5 * S.sEff
  local a = alpha(dt, W)
  S.uInt = S.uInt + (S.u - S.uInt) * a
  S.DwInt = S.DwInt + (S.Dw - S.DwInt) * a
  local wI = pv(PT.intW) + 0.5 * S.sEff
  S.uUp = S.uSm + wI * max(0, S.uInt - S.uSm)
  -- ECO/COMFORT on the ICE maps: a rising pedal raises uUp at once (replaces
  -- the PEDAL guard, which would block upshifts while the car gains rpm)
  if (S.prog == "ECO" or S.prog == "COMFORT") and not S.evLogic then S.uUp = max(S.uUp, S.u) end
  S.uDn = max(S.u, S.uSm)
  S.DwUp = max(S.Dw, wI * S.DwInt)

  -- EV maps: gear-independent motor operating point n = 950 + 17 * P_kW,
  -- P evaluated at that point (shaft torque T): n = 950 / (1 - 17 * T / 9549)
  local tq = min(RULES.reqAt(max(S.nShaft, P.idle), S.uUp), S.evAvail)
  local opt = clamp(950 / max(0.05, 1 - 17 * tq / 9549.3), 950, S.evCorner) * (1 + 0.25 * S.sEff)
  local up = lerp(opt, S.evCorner, clamp01((S.uUp - 0.60) / 0.25))
  -- ECO PRO EV power line: uUp stays low in the whole ECO PRO EV zone, so the
  -- motor fill S.evFracRaw raises evUp instead
  if P.evPowLine and S.prog == "ECO" and S.edrive == "AUTO_EDRIVE" and S.evLogic and S.soc >= K.evPowSoc then
    local ut = clamp01((S.evFracRaw - K.evPowF0) / K.evPowSpan)
    S.kPow = S.kPow + (ut - S.kPow) * alpha(dt, (ut > S.kPow) and K.evPowRiseS or K.evPowFallS)
    local full = min(S.evCorner, S.startCap - K.evPowCapMargin)
    if full > up then up = lerp(up, full, S.kPow) end
  else
    S.kPow = 0
  end
  S.evUp = up
  S.evDn = min(max(700, 0.72 * up), up - K.hyst)
  if S.edrive == "MAX_EDRIVE" then
    S.evFloor = 800
    S.nFullEV = min(5500, P.motorMaxRPM - 500)
  else
    S.evFloor = max(900, (S.u >= 0.35) and 1.35 * P.idle or 0)
    S.nFullEV = min(P.nUpFull - 300, P.lim - 400)
  end
end

-- holds, latches, dwell and hold timers
local function updateHolds(dt)
  -- corner hold on |gx2|, released after < 0.7 * thr for cornerRel + sEff s
  local thr = pvs(PT.cornerCalm, PT.cornerDyn)
  if S.gxS > thr and S.v > K.cornerMinSpeed then
    S.cornerHold, S.cornerRelT = true, 0
  elseif S.cornerHold then
    S.cornerRelT = (S.gxS < 0.7 * thr) and (S.cornerRelT + dt) or 0
    if S.cornerRelT >= pv(PT.cornerRel) + S.sEff then S.cornerHold = false end
  end

  -- time since the pedal fell below K.liftUpPedal
  local p = S.pRaw
  if p < K.liftUpPedal then S.liftT = S.liftT + dt else S.liftT = 0 end

  -- lift-off hold (no upshift after a quick lift): drop > 0.30 from a peak
  -- > 0.5 within 0.5 s
  if p >= S.pPeak then
    S.pPeak, S.pPeakT = p, 0
  else
    S.pPeakT = S.pPeakT + dt
    if S.pPeakT > 0.5 then S.pPeak, S.pPeakT = p, 0 end
  end
  if (S.prog == "SPORT" or S.sEff >= 0.45) and S.pPeak > 0.5 and (S.pPeak - p) > 0.30 then
    if S.liftOffT <= 0 then S.liftLow = false end
    S.liftOffT = 1.0 + 3.0 * S.sEff
  elseif S.liftOffT > 0 then
    S.liftOffT = S.liftOffT - dt
  end
  if p <= 0.15 then S.liftLow = true end
  -- cancelled by re-applying the pedal (only after it was really released)
  if S.liftOffT > 0 and S.liftLow and p > 0.15 then S.liftOffT = 0 end

  -- kickdown arming, dwell against pedal stabs (ECO PRO / AUTO EV, shared
  -- with the VCU's urgent ICE start) and latch expiry
  if p < 0.85 then S.kdArmed = true end
  S.kdHoldT = (p >= P.kickdownPedal) and (S.kdHoldT + dt) or 0
  local kdDw = 0
  if S.prog == "ECO" and S.edrive == "AUTO_EDRIVE" then
    kdDw = num(electrics.values.etk854hybrid_pedalUrgentDwellS, K.kdDwellEco)
  end
  S.kdOn = p >= P.kickdownPedal and S.kdArmed and S.kdHoldT >= kdDw
  if S.kdLatch then
    S.kdLatchT = S.kdLatchT + dt
    if S.kdLatchT > K.kdLatchMaxS or p < 0.80 then S.kdLatch = false end
  end

  -- hunting hold expiry
  if S.huntT > 0 then
    S.huntT = S.huntT - dt
    if S.huntT <= 0 then S.hunt = false end
  end

  -- temporary manual exit: 8 s without taps in calm conditions, or stop
  local tm = S.tm
  if tm.active then
    tm.idleT = tm.idleT + dt
    if S.v < K.stopSpeed or D.AH.mode ~= "D"
       or (tm.idleT > 8 and S.gxS < 2 and p < 0.85 and S.brake < 0.3) then
      tm.active, tm.req = false, 0
    end
  end

  S.tSinceShift = S.tSinceShift + dt
  S.tSinceUp = S.tSinceUp + dt
  S.tSinceDn = S.tSinceDn + dt
  S.tSinceBrakeStep = S.tSinceBrakeStep + dt

  -- torque downshift persistence
  if (not S.evLogic) and S.n < P.resvMax and S.Treq > S.kDn * Tav(S.n) then
    S.tdIceT = S.tdIceT + dt
  else
    S.tdIceT = 0
  end
  -- EV saturation with the ICE available: judged against TavPend, else the
  -- car downshifts for EV max power just before the ICE takes over
  local e = D.engine
  S.evIce = S.edrive ~= "MAX_EDRIVE" and (S.prog == "ECO" or S.prog == "COMFORT")
    and S.pLoad <= K.startCapPedal and not S.kdLatch and not (e and e.isDisabled)
  local sat = S.evSat
  if S.evIce then sat = S.Treq > S.kDn * TavPend(S.n) end
  if S.evLogic and sat then S.tdEvT = S.tdEvT + dt else S.tdEvT = 0 end
  -- ICE-start rev cap: ECO/COMFORT, no kickdown (SPORT keeps its gears)
  S.capActive = S.icePending and (S.prog == "ECO" or S.prog == "COMFORT")
    and S.pLoad <= K.startCapPedal and not S.kdLatch
  if S.uLatch >= 0 and S.u < S.uLatch - 0.10 then S.uLatch = -1 end
  S.pending = S.kdLatch or (tm.req ~= 0)

  -- SPORT calm relax; any active input ends it in the same frame. Rise
  -- hysteresis: a cruise-control wobble must not toggle it.
  local dps = p - S.pSm
  local rise = S.sportRelax and K.relaxRiseExit or K.relaxRise
  local calm = p <= K.relaxPedal and dps < rise and dps > -K.relaxDrop and S.brake < K.relaxBrake
    and not S.cornerHold and S.gxS < K.relaxCornerFrac * thr and not S.kdLatch and S.launch == "IDLE"
  S.sportCalmT = calm and (S.sportCalmT + dt) or 0
  S.sportCalm = S.sportCalmT >= K.relaxBaseS + K.relaxSEffS * S.sEff
  S.sportRelax = S.sportCalm and S.prog == "SPORT"
  if S.sportRelax then S.liftOffT = 0 end
  RULES.applyRelax()

  -- SPORT top gear: persistence of "the top gear would run calm enough"
  local R, g, tg = D.R, S.g, S.topGear
  if g > tg and tg >= 1 and R[g] and R[tg] and S.n * R[tg] / R[g] <= P.nUpFull - K.sportTopMarginRPM then
    S.sportTopT = S.sportTopT + dt
  else
    S.sportTopT = 0
  end
end

-- ----------------------------------------------------------------------------
-- Rules
-- ----------------------------------------------------------------------------
-- upshift time (gearbox ratio blend): calm/dyn blended by max(adaptation,
-- load) - the capped ECO/COMFORT adaptation alone keeps every upshift slow
function RULES.upTime(steps)
  local load = clamp01((S.pLoad - K.upTimeLoad0) / K.upTimeLoadSpan)
  return lerp(pv(PT.upTimeCalm), pv(PT.upTimeDyn), max(S.sEff, load)) + K.upTimeStep * max(0, steps - 1)
end

-- kickdown time: base + step per skipped gear
function RULES.kdTime(steps)
  return pv(PT.kdTimeBase) + pv(PT.kdTimeStep) * max(0, steps - 1)
end

-- upshift-side values while SPORT is relaxed: calm up rows (sEffUp = 0),
-- calm reserve kUpShift, top gear + K.relaxTopExtra; the dn side keeps the
-- real sEff. The caller sets S.topGear to the program's top gear first.
function RULES.applyRelax()
  if S.sportRelax then
    S.sEffUp = 0
    S.kUpShift = pv(PT.kUpCalm) * ((S.trailer > 0) and K.trailerKUp or 1)
    S.topGear = min(D.topSport + K.relaxTopExtra, S.modeMax)
  else
    S.sEffUp = S.sEff
    S.kUpShift = S.kUp
  end
end

-- EV lug: K0 open, shaft too slow under load (RULES.lug, C.manualOverride)
function RULES.evLugging()
  return S.isEV and S.n < K.evLugRPM and S.u > K.evLugU
end

-- lug / stall protection (bypasses dwell and freeze)
function RULES.lug()
  local R, g, n = D.R, S.g, S.n
  if g <= 1 then return nil end
  if S.lugCoupled and n < S.lugMinEff then
    for h = g - 1, 1, -1 do
      if n * R[h] / R[g] >= S.upFloor then return h, "lug", K.tLug end
    end
    return 1, "lug", K.tLug
  end
  if RULES.evLugging() then return g - 1, "lug", K.tLug end
  return nil
end

-- best kickdown gear: max wheel torque potential within the rev limit, the
-- higher gear wins within 5 %. Landing rpm checked after the lead (coupled:
-- kdHoldLead; ICE still to start: kdStartLead and the sync ceiling, else
-- iceCeil upshifts again before the ICE pulls).
function RULES.kickTarget()
  local R, g, n = D.R, S.g, S.n
  local evOnly = S.edrive == "MAX_EDRIVE"
  local limit = evOnly and (S.nFullEV - 300) or P.kdMax
  local lead = K.kdHoldLead
  local es = S.engState
  if not evOnly and es ~= "ON" and es ~= "BLEND" and es ~= "RELEASE" then
    limit = min(limit, P.lim - K.syncCeilMargin)
    lead = K.kdStartLead
  end
  local grow = 1 + K.kdAccel * lead / max(S.v, 5)
  local lo = max(1, g - 5)
  local best = 0
  for h = g - 1, lo, -1 do
    local nh = n * R[h] / R[g]
    if nh * grow > limit then break end
    local vh = (evOnly and evAt(nh) or TavKD(nh)) * R[h]
    if vh > best then best = vh end
  end
  if best <= 0 then return nil end
  for h = g - 1, lo, -1 do
    local nh = n * R[h] / R[g]
    if nh * grow > limit then break end
    local vh = (evOnly and evAt(nh) or TavKD(nh)) * R[h]
    if vh >= 0.95 * best then return h end
  end
  return nil
end

-- kickdown (this frame's request, or latched during a K0 freeze)
function RULES.kick()
  if not (S.kdOn or S.kdLatch) then return nil end
  if not S.slipDnOK then
    S.inhibit = "SLIP"
    return nil
  end
  S.kdArmed, S.kdLatch = false, false
  local h = RULES.kickTarget()
  if h and h < S.g then return h, "kickdown", RULES.kdTime(S.g - h) end
  return nil
end

-- rpm in gear k when the K0 freeze ends (S.lead = K0 lock + blend); the
-- acceleration term only with the rev cap active
function RULES.nPred(k)
  local R, g = D.R, S.g
  local ratio = R[k] / R[g]
  local n = S.n * ratio
  if S.capActive then n = n + S.nDotG * ratio * S.lead end
  return n
end

-- ICE-start gear: upshift when the predicted engagement rpm exceeds
-- S.startCap (never below the lug floor); before the sequencer tracks the shaft
function RULES.iceStartGear()
  if not S.capActive then return nil end
  local R, g = D.R, S.g
  local top = S.topGear
  if g >= top or RULES.nPred(g) <= S.startCap then return nil end
  for k = g + 1, top do
    if S.n * R[k] / R[g] < S.upFloor then break end
    if RULES.nPred(k) <= S.startCap then return k, "iceUp", RULES.upTime(k - g) end
  end
  return nil
end

-- engagement floor gate (nPred(g) < S.engageMin): not when the ICE map keeps
-- g anyway (mapUp would undo the downshift); else only for demand (uDn >=
-- engageFloorU, or g lacks the kUp reserve with the ICE in)
function RULES.engageFloorWanted(g)
  local ng = RULES.nPred(g)
  if ng >= nUpPost(g - 1, S.uUp, 0) then return false end
  if S.uDn >= K.engageFloorU then return true end
  return RULES.reqAt(ng, S.uDn) > S.kUp * TavPend(ng)
end

-- ICE start pending in a high EV gear: the demand wants a lower ICE-map gear
-- the EV logic never picks, and the start hold + K0 freeze would keep the
-- gear until the blend ends. Same window as iceStartGear.
function RULES.iceStartDown()
  if not S.icePending then return nil end -- icePending excludes evLogic (senseHybrid)
  local g = S.g
  local lim = S.capActive and (S.startCap - K.startDnHeadroom) or P.kdMax
  -- engagement floor (the low dn rows would let the ICE couple near idle):
  -- highest gear reaching S.engageMin within the cap
  if g > 1 and RULES.nPred(g) < S.engageMin and RULES.engageFloorWanted(g) then
    local h = g - 1
    while h > 1 and RULES.nPred(h) < S.engageMin and RULES.nPred(h - 1) <= lim do h = h - 1 end
    if RULES.nPred(h) <= lim then return h, "iceDn", K.tIceDn end
  end
  local k = RULES.downLoop(S.uDn, false)
  if S.capActive then
    while k < g and RULES.nPred(k) > lim do k = k + 1 end
  end
  if k < g then return k, "iceDn", K.tIceDn end
  return nil
end

-- K0 open during CRANK/SYNC: the shaft must not run past the sequencer's
-- rev-match ceiling (SYNC blocked) or the EV full-load point
function RULES.startCeilUp()
  local g = S.g
  if g >= S.modeMax then return nil end
  local lim = min(P.lim - K.syncCeilMargin, S.evLogic and S.nFullEV or P.nUpFull)
  if S.syncBlocked or S.nIn >= lim then return g + 1 end
  return nil
end

-- full-load upshift at nUpFull (or the rev limiter), ignores holds
function RULES.fullUp()
  local g = S.g
  if g >= S.modeMax then return nil end
  local lim = S.evLogic and S.nFullEV or P.nUpFull
  if S.nIn >= lim then return g + 1 end
  local e = D.engine
  if S.coupled and e and e.revLimiterActive and not e.isTempRevLimiterActive then return g + 1 end
  return nil
end

-- multi-step map downshift loop; returns the target gear (= g for none).
-- EV: gear-independent S.evDn threshold and S.nFullEV limit.
function RULES.downLoop(u, ev)
  local R, g, n = D.R, S.g, S.n
  local lim = ev and S.nFullEV or P.kdMax
  local capped = S.capActive and not ev
  local k = g
  while k > 1 do
    local nk = n * R[k] / R[g]
    local thr = ev and S.evDn or nDn(k, u, lugBonus(nk, k))
    if nk >= thr then break end
    if n * R[k - 1] / R[g] > lim then break end
    if capped and RULES.nPred(k - 1) > S.startCap then break end
    k = k - 1
  end
  return k
end

-- torque-reserve downshift, ICE (the demand downshift of torque-first
-- ECO/COMFORT): request > kDn x Tav for tdPersist, only below resvMax (above
-- it a veto would just be a pedal threshold); target = highest gear carrying
-- the wheel torque with the kUp reserve
function RULES.torqueDownIce()
  local R, g, n = D.R, S.g, S.n
  if n >= P.resvMax or S.tdIceT < pv(PT.tdPersist) or S.u <= S.uLatch + 0.05 then return nil end
  local need = S.Treq * R[g]
  local capped = S.capActive
  for h = g - 1, 1, -1 do
    local nh = n * R[h] / R[g]
    if nh > P.kdMax or (capped and RULES.nPred(h) > S.startCap) then break end
    if need <= S.kUp * Tav(nh) * R[h] then return h end
  end
  if g > 1 then
    local n1 = n * R[g - 1] / R[g]
    if n1 <= P.kdMax and not (capped and RULES.nPred(g - 1) > S.startCap)
       and Tav(n1) * R[g - 1] > 1.05 * Tav(n) * R[g] then return g - 1 end
  end
  return nil
end

-- EV saturation downshift with the ICE available: like torqueDownIce against
-- TavPend, rpm predicted over K.evSatLeadS with the start headroom (else
-- iceStartGear shifts back up). No target: hold until the ICE couples.
function RULES.torqueDownEvIce()
  local R, g, n = D.R, S.g, S.n
  local lim = S.startCap - K.startDnHeadroom
  local acc = S.nDotG * K.evSatLeadS / R[g]
  local need = S.Treq * R[g]
  for h = g - 1, 1, -1 do
    local nh = n * R[h] / R[g]
    if nh > P.kdMax or nh + acc * R[h] > lim then break end
    if need <= S.kUp * TavPend(nh) * R[h] then return h end
  end
  if g > 1 then
    local n1 = n * R[g - 1] / R[g]
    if n1 <= P.kdMax and n1 + acc * R[g - 1] <= lim
       and TavPend(n1) * R[g - 1] > 1.05 * TavPend(n) * R[g] then return g - 1 end
  end
  return nil
end

-- torque-reserve downshift, EV (motor saturated): first gear carrying the
-- request, else the max-power gear, within the EV range. Re-armed only after
-- demand rises another 0.05 (uLatch): stops the saturation cascade.
function RULES.torqueDownEv()
  local R, g, n = D.R, S.g, S.n
  if S.tdEvT < pv(PT.tdPersist) or S.u <= S.uLatch + 0.05 then return nil end
  if S.evIce then return RULES.torqueDownEvIce() end
  local need = S.Treq * R[g] / 0.85
  local limit = S.nFullEV - 300
  -- best: max reachable wheel torque, read only when no gear carries the request
  local target, tv, best = nil, 0, 0
  for h = g - 1, 1, -1 do
    local nh = n * R[h] / R[g]
    if nh > limit then break end
    local vh = evAt(nh) * R[h]
    if vh >= need then
      target, tv = h, vh
      break
    end
    if vh > best then best = vh end
  end
  if not target and best > 0 then
    for h = g - 1, 1, -1 do
      local nh = n * R[h] / R[g]
      if nh > limit then break end
      local vh = evAt(nh) * R[h]
      if vh >= 0.95 * best then
        target, tv = h, vh
        break
      end
    end
  end
  if target and tv >= 1.05 * evAt(n) * R[g] then return target end
  return nil
end

function RULES.torqueDown()
  if S.evLogic then return RULES.torqueDownEv(), "evSat" end
  return RULES.torqueDownIce(), "torqueDn"
end

-- brake downshift, sequential steps towards a target rpm: SPORT (sEff >=
-- 0.55) the calm..dyn brake target; ECO/COMFORT only with the ICE coupled
-- and only to the re-acceleration reserve (calm target)
function RULES.brakeDown()
  local g = S.g
  if S.brake < 0.15 or S.pRaw >= K.brakeDnPedal or S.abs or g <= 2 then return nil end
  local tgt
  if S.prog == "SPORT" or S.sEff >= 0.55 then
    if S.decel < 2.5 then return nil end
    tgt = min(pvs(P.brakeCalm, P.brakeDyn), P.kdMax - 500)
  elseif (S.prog == "ECO" or S.prog == "COMFORT") and S.coupled then
    if S.decel < K.brakeDnDecelCoupled then return nil end
    tgt = S.brakeCalmP
  else
    return nil
  end
  if S.tSinceBrakeStep < pv(PT.brakeSpacing) then return nil end
  local n1 = S.n * D.R[g - 1] / D.R[g]
  if n1 <= tgt then return g - 1 end
  return nil
end

-- overrun upshift with the ICE COUPLED (ECO/COMFORT): the REGEN guard would
-- leave the engine dragged at high rpm after a kickdown (the VCU's drag
-- credit keeps the decel level in the higher gear). Target: highest gear
-- at or above the coast target (the decoupled regen floor if a K0 release is
-- due soon). Not in SPORT (brakeDn would shift back), not with regen OFF
-- (sailing owns the lift), not overlapping the K0 release.
function RULES.liftUp()
  if not S.coupled or S.pRaw >= K.liftUpPedal or S.brake >= 0.02 or S.cornerHold then return nil end
  local p = S.prog
  if not (p == "ECO" or p == "COMFORT") or S.regenLvl == "OFF" then return nil end
  if S.liftT < pv(PT.liftUpDelay) then return nil end
  if S.tSinceUp < S.tUpAfterUp or S.tSinceDn < S.tUpAfterDn then return nil end
  local releasing = S.holdKind ~= "none"
  if releasing and S.holdRemain < RULES.upTime(1) + K.liftUpRelMargin then return nil end
  local R, g, n = D.R, S.g, S.n
  local top = min(S.topGear, g + K.liftUpMaxSteps)
  local fl = S.coastTarget
  if S.regenOn and releasing and S.holdRemain < K.liftUpRelSoonS then
    fl = max(fl, K.REGEN_FLOOR[S.regenLvl] or 1500)
  end
  for k = top, g + 1, -1 do
    if n * R[k] / R[g] >= fl then return k, "liftUp", RULES.upTime(k - g) end
  end
  return nil
end

-- regen upshift with the ICE DECOUPLED (from the REGEN guard): up while the
-- same wheel regen torque stays within regenUpEnvFrac of the motor envelope
-- and above the regen floor; SPORT only while relaxed
function RULES.regenUp()
  if S.coupled or S.brake >= 0.02 then return nil end
  local p = S.prog
  if (p == "SPORT" or p == "MANUAL") and not S.sportRelax then return nil end
  local R, g, n = D.R, S.g, S.n
  if n <= pv(PT.regenUpRPM) then return nil end
  local fl = max(K.REGEN_FLOOR[S.regenLvl] or 1500, K.regenUpMinRPM)
  local top = min(S.topGear, g + K.regenUpMaxSteps)
  local tq = S.regenTq
  for k = top, g + 1, -1 do
    local nk = n * R[k] / R[g]
    if nk >= fl and tq * R[g] / R[k] <= K.regenUpEnvFrac * evAt(nk) then
      return k, "regenUp", RULES.upTime(k - g)
    end
  end
  return nil
end

-- SPORT top-gear target: lowest gear from S.topGear up at <= nUpFull - margin
function RULES.sportTopTarget()
  local g, top = S.g, S.topGear
  if g <= top then return nil end
  local R, n = D.R, S.n
  local lim = P.nUpFull - K.sportTopMarginRPM
  for k = top, g - 1 do
    if n * R[k] / R[g] <= lim then return k end
  end
  return nil
end

-- SPORT top gear: from above it (program switch, top-speed fullUp) back down
-- once it runs calm enough, after the program ramp, not at full load
function RULES.sportTop()
  if S.g <= S.topGear or S.lambda < 1 or S.pRaw >= K.sportTopPedal or S.sportTopT < K.sportTopPersistS
     or not S.slipDnOK then return nil end
  local k = RULES.sportTopTarget()
  if k then return k, "sportTop", K.sportTopTime end
  return nil
end

-- lift-off: regen floor, ICE coast floor, EV coast floor. Coupled regen
-- floor = coast / brake calm target (a dragged engine gains nothing from
-- rpm); decoupled: the efficiency floors K.REGEN_FLOOR*
function RULES.lift()
  local R, g, n = D.R, S.g, S.n
  if g <= 1 then return nil end
  if S.regenOn then
    local brk = S.brake >= 0.15
    local fl
    if S.coupled then
      fl = brk and S.brakeCalmP or S.coastTarget
    else
      fl = brk and K.REGEN_FLOOR_BRAKE or (K.REGEN_FLOOR[S.regenLvl] or 1500)
    end
    if n < fl - K.regenDnHyst and S.tSinceShift >= 0.30 then
      for h = g - 1, 1, -1 do
        local nh = n * R[h] / R[g]
        if nh > K.REGEN_MAX_RPM then break end
        if nh >= fl then return h, "regenDn", K.tRegenDn end
      end
    end
    return nil
  end
  if S.coupled and n < P.coastDn then
    if S.tSinceShift >= K.coastDwellS then return g - 1, "coastDn", K.tCoastDn end
    return nil
  end
  if S.isEV and n < K.evCoastRPM and S.tSinceShift >= K.coastDwellS then return g - 1, "evCoast", K.tCoastDn end
  return nil
end

-- VCU drive request at shaft rpm n (mirrors the VCU's mapPedal): u *
-- min(sysCap, motor + ICE potential), motor only in MAX EV. A shaft torque
-- that follows the pedal in every gear (EV pedal zone: off by a few %).
function RULES.reqAt(n, u)
  local ice = (S.edrive == "MAX_EDRIVE") and 0 or iceCurve(clamp(n, P.idle, P.lim))
  return u * min(S.sysCap, evAt(n) + ice)
end

-- upshift test for gear k at post-shift rpm nk: compares the request AFTER
-- the shift (reqAt on uUp), not the current wheel torque, which would veto
-- every part-load upshift
function RULES.upPass(k, nk)
  local R = D.R
  if S.evLogic then
    if nk < S.evUp or nk < S.evFloor then return false end
    local upSat = (S.edrive == "MAX_EDRIVE") and K.evUpSatMax or K.evUpSat
    if S.evSat or RULES.reqAt(nk, S.uUp) > upSat * evAt(nk) then
      -- saturated (now or after the shift): max-power rule
      return evAt(nk) * R[k] >= 0.98 * evAt(S.n) * R[S.g]
    end
    return true
  end
  if nk < S.upFloor then return false end
  if nk < nUpPost(k - 1, S.uUp, lugBonus(nk, k)) then return false end
  if nk >= P.resvMax then return true end
  return RULES.reqAt(nk, S.uUp) <= S.kUpShift * Tav(nk)
end

-- highest passing upshift target; skips up to g+3 in calm ECO/COMFORT (a
-- demand drop lands in one shift), g+2 in relaxed SPORT
function RULES.upCandidate()
  local R, g, n = D.R, S.g, S.n
  local top = S.topGear
  if g >= top then return nil end
  local kMax = g + 1
  local p = S.prog
  if (p == "ECO" or p == "COMFORT") and S.sEff < 0.5 then
    if S.uUp < K.skipU3 then kMax = g + 3 elseif S.uUp < K.skipU2 then kMax = g + 2 end
  elseif p == "SPORT" and S.sportRelax then
    kMax = g + 2
  end
  if kMax > top then kMax = top end
  -- test at the rpm when the debounce confirms (else every gear overshoots
  -- its map point), capped against wheel-speed noise
  local nEff = n + min(K.upLeadMaxRPM, S.nDotG * pv(PT.debUp))
  for k = kMax, g + 1, -1 do
    if RULES.upPass(k, nEff * R[k] / R[g]) then return k end
  end
  return nil
end

-- map upshift guards; returns the inhibit reason or nil. REGEN: at u = 0 the
-- map lands below the regen floor and would ping-pong with regenDn (overrun
-- upshifts: liftUp / regenUp)
function RULES.upGuard()
  if S.tSinceUp < S.tUpAfterUp or S.tSinceDn < S.tUpAfterDn then return "DWELL" end
  if S.cornerHold then return "CORNER" end
  if S.liftOffT > 0 then return "LIFTOFF" end
  if S.brake > 0.02 then return "BRAKE" end
  if S.regenOn then return "REGEN" end
  if not S.slipUpOK then return "SLIP" end
  if S.tc and S.tcT < K.tcUpHoldS and S.n < K.tcUpMaxFrac * P.nUpFull then return "TC" end
  -- pedal not steady: SPORT/MANUAL either way; ECO/COMFORT only rising on
  -- the EV axis (ICE maps use the raised uUp)
  local dp = S.pRaw - S.pSm
  if S.prog ~= "ECO" and S.prog ~= "COMFORT" then
    dp = abs(dp)
  elseif not S.evLogic then
    dp = 0
  end
  if dp >= 0.08 then return "PEDAL" end
  return nil
end

-- sail preselect: highest gear where the ICE drag-start lands at a usable rpm
function RULES.sailSelect()
  local R, g, n = D.R, S.g, S.n
  local need0 = S.lugMinEff + 250
  local best = nil
  for k = S.topGear, 1, -1 do
    local nk = n * R[k] / R[g]
    local thr = max(need0, (k > 1) and nUpPost(k - 1, 0.20, 0) or 0)
    if nk >= thr then
      best = k
      break
    end
  end
  if best and best ~= g then
    if S.tSinceShift >= K.sailDwell then return best, "sail", K.tSail end
    S.inhibit = "DWELL"
  end
  return nil
end

-- temporary-manual paddle request (only reached while tm.active)
function RULES.paddle()
  local tm = S.tm
  if tm.req == 0 then return nil end
  local R, g, n = D.R, S.g, S.n
  local t = clamp(g + tm.req, 1, S.modeMax)
  tm.req = 0
  while t < g and n * R[t] / R[g] > P.nUpFull - 200 do t = t + 1 end
  if t ~= g then return t, "paddle", (t > g) and RULES.upTime(1) or K.tPaddleDn end
  return nil
end

-- decision chain for D / S: target gear, reason, gear change time, or nil
-- (S.inhibit says why)
function RULES.run(dt)
  local g = S.g
  -- shifting / settle
  if D.gearbox.isShifting or S.tSinceShift < K.settle then
    S.inhibit = "SHIFTING"
    return nil
  end
  -- K0 freeze (with watchdog)
  if S.frozen then
    if S.frzT < P.freezeWatchdogS and S.frzTotT < K.freezeTotalCoef * P.freezeWatchdogS then
      if S.kdOn then
        S.kdLatch, S.kdLatchT, S.kdArmed = true, 0, false
      end
      S.pending = S.kdLatch or (S.tm.req ~= 0)
      local t1, w1, tt1 = RULES.lug()
      if t1 then return t1, w1, tt1 end
      S.inhibit = S.freezeWhy
      return nil
    end
    if not S.frzLogged then
      S.frzLogged = true
      log("W", "etk854hybrid8hp", string.format("[8HP] freeze watchdog %.1fs (total %.1fs) eng=%s k0=%.2f sail=%s",
        S.frzT, S.frzTotT, S.engState, S.k0, S.sail))
    end
    S.inhibit = "WATCHDOG"
  end
  -- trailer stability acting: hold the gear (only lug and fullUp)
  if S.tsa then
    local t1, w1, tt1 = RULES.lug()
    if t1 then return t1, w1, tt1 end
    local t2 = RULES.fullUp()
    if t2 then return t2, "fullUp", RULES.upTime(1) end
    S.inhibit = "TSA"
    return nil
  end
  -- launch control; ARMED at standstill (PRELOAD against the brake) counts
  -- as READY, else fullUp would shift 1 -> 2 on a limiter hit
  if S.launch == "READY" or (S.launch == "ARMED" and S.v < K.stopSpeed) then
    if S.v < K.stopSpeed and g ~= 1 then return 1, "launch", 0.25 end
    S.inhibit = "LAUNCH"
    return nil
  end
  if S.launch == "ACTIVE" then
    local t3 = RULES.fullUp()
    if t3 then return t3, "fullUp", RULES.upTime(1) end
    S.inhibit = "LAUNCH"
    return nil
  end
  -- stop
  if S.v < K.stopSpeed and g > 1 then return 1, "stop", K.tStop end
  -- ICE start (CRANK/SYNC, max startFreezeS): hold the rev-match target;
  -- only lug, kickdown (the sequencer re-targets), the sync ceiling and the
  -- rev-cap gear (CRANK, or SYNC far above the cap)
  if S.startFreeze then
    local t1, w1, tt1 = RULES.lug()
    if t1 then return t1, w1, tt1 end
    local tk, wk, ttk = RULES.kick()
    if tk then return tk, wk, ttk end
    local tf = RULES.startCeilUp()
    if tf then return tf, "iceCeil", RULES.upTime(1) end
    if S.engState == "CRANK" then
      local tc, wc, ttc = RULES.iceStartGear()
      if tc then return tc, wc, ttc end
      tc, wc, ttc = RULES.iceStartDown()
      if tc then return tc, wc, ttc end
    elseif S.engState == "SYNC" and S.capActive and RULES.nPred(S.g) > S.startCap + K.startCapSyncMargin then
      local tc, wc, ttc = RULES.iceStartGear()
      if tc then return tc, wc, ttc end
    end
    S.inhibit = "ICESTART"
    return nil
  end
  -- sailing: kickdown first (must not wait for the restart), else preselect
  if S.sail == "SAIL" then
    local tk, wk, ttk = RULES.kick()
    if tk then return tk, wk, ttk end
    return RULES.sailSelect()
  end

  local tg, why, t = RULES.lug()
  if tg then return tg, why, t end
  tg, why, t = RULES.kick()
  if tg then return tg, why, t end
  -- fullUp also above the SPORT top gear (top-speed exception)
  tg = RULES.fullUp()
  if tg then return tg, "fullUp", RULES.upTime(1) end
  if S.tm.active then return RULES.paddle() end
  tg, why, t = RULES.sportTop()
  if tg then return tg, why, t end
  -- ICE requested, not cranking yet: choose the engagement gear now
  if S.icePending and (S.engState == "OFF" or S.engState == "IDLE") then
    tg, why, t = RULES.iceStartGear()
    if tg then return tg, why, t end
    tg, why, t = RULES.iceStartDown()
    if tg then return tg, why, t end
  end

  return RULES.runPartLoad(dt)
end

-- part-load rules, in priority order
function RULES.runPartLoad(dt)
  local g = S.g
  -- tip-in (bypasses dwell): load on pLoad, rate on the raw pedal
  if not S.cornerHold and S.pLoad >= K.tipinPedal and S.dpdt >= S.kRate and S.slipDnOK then
    local k = RULES.downLoop(S.u, S.evLogic)
    if k < g then return k, "tipin", pv(PT.dnTimeTip) end
    local k2, w2 = RULES.torqueDown()
    if k2 then
      S.uLatch = S.u
      return k2, w2, pv(PT.dnTimeTip)
    end
  end
  -- brake downshift
  local tg = RULES.brakeDown()
  if tg then return tg, "brakeDn", K.tBrakeDn end
  -- overrun upshift, ICE coupled
  local t5, w5, tt5 = RULES.liftUp()
  if t5 then return t5, w5, tt5 end
  -- lift-off
  if S.pRaw < K.liftPedal then
    local t6, w6, tt6 = RULES.lift()
    if t6 then return t6, w6, tt6 end
  end
  -- map / torque downshift
  if not S.cornerHold and S.slipDnOK then
    local k = RULES.downLoop(S.uDn, S.evLogic)
    -- from above the SPORT top gear straight to it (no mapDn + sportTop pair)
    local top = S.topGear
    if k < g and k > top and g > top and S.n * D.R[top] / D.R[g] <= P.kdMax then k = top end
    local k2, w2
    if k >= g then k2, w2 = RULES.torqueDown() end
    if k < g or k2 then
      if S.tSinceUp >= S.tDnAfterUp and S.tSinceDn >= S.tDnAfterDn then
        if k < g then return k, S.evLogic and "evDn" or "mapDn", pv(PT.dnTimeDemand) end
        S.uLatch = S.u
        return k2, w2, pv(PT.dnTimeDemand)
      end
      S.inhibit = "DWELL"
      return nil
    end
  end
  -- map / EV upshift (debounced)
  local k = RULES.upCandidate()
  if not k then
    S.upCand, S.upCandT = nil, 0
    return nil
  end
  local why = RULES.upGuard()
  if why then
    if why == "REGEN" then
      local tr, wr, ttr = RULES.regenUp()
      if tr then return tr, wr, ttr end
    end
    S.inhibit = why
    S.upCand, S.upCandT = nil, 0
    return nil
  end
  -- the debounce runs while ANY candidate persists, its length set by the
  -- first one: the candidate may grow g+1 -> g+3 within frames, a restart
  -- would delay the shift
  if S.upCand then
    S.upCandT = S.upCandT + dt
  else
    S.upCandT, S.upCandDeb = 0, (k > g + 1) and pv(PT.debSkip) or pv(PT.debUp)
  end
  S.upCand = k
  if S.upCandT >= S.upCandDeb then
    S.upCand, S.upCandT = nil, 0
    local reason = S.evLogic and "evUp" or ((k > g + 1) and "skipUp" or "mapUp")
    return k, reason, RULES.upTime(k - g)
  end
  return nil
end

-- hunting detector: >= 3 reversals of autonomous shifts in huntWindow, each
-- with steady driver input (else the driver reversed it, not the program)
function RULES.noteShift(dir, why)
  if not K.HUNT[why] then return end
  local steady = S.brake <= K.huntBrake and S.huntB <= K.huntBrake and abs(S.pRaw - S.huntP) <= K.huntPedal
  if S.lastDir ~= 0 and dir ~= S.lastDir and steady then
    S.rv1, S.rv2, S.rv3 = S.rv2, S.rv3, S.clock
    if S.clock - S.rv1 <= K.huntWindow then
      if not S.hunt then
        log("W", "etk854hybrid8hp", string.format("[8HP] hunt detected g=%d prog=%s s=%.2f u=%.2f",
          S.g, S.prog or "?", S.sEff, S.u))
      end
      S.hunt, S.huntT = true, K.huntHold
    end
  end
  S.lastDir, S.huntP, S.huntB = dir, S.pRaw, S.brake
end

-- keeps the stock shiftDecisionData debug app truthful
local function writeShiftBehavior(M)
  local sb = M.shiftBehavior
  if type(sb) ~= "table" then return end
  local R, g, n = D.R, S.g, S.n
  if g < 1 or not R[g] then return end
  local upAV, dnAV
  if g >= S.topGear then
    upAV = P.nUpFull * RPM2AV
  elseif S.evLogic then
    upAV = S.evUp * R[g] / R[g + 1] * RPM2AV
  else
    local nk = n * R[g + 1] / R[g]
    upAV = nUpPost(g, S.uUp, lugBonus(nk, g + 1)) * R[g] / R[g + 1] * RPM2AV
  end
  if g <= 1 then
    dnAV = 0
  elseif S.evLogic then
    dnAV = S.evDn * RPM2AV
  else
    dnAV = nDn(g, S.uDn, lugBonus(n, g)) * RPM2AV
  end
  sb.shiftUpAV, sb.shiftDownAV = upAV, dnAV
end

-- shift recommendation in M<n> / temp manual: the gear the D program would
-- select now, without timing guards; program fields and the M<n> gear clamp
-- are swapped for the call and restored
local SHW = {ECO = 0, COMFORT = 0, SPORT = 0, MANUAL = 0}
local function shadowPlan()
  local g = S.g
  if g < 1 then return g end
  local prog = S.override or K.DE2PROG[S.deMode] or "COMFORT"
  if prog == "MANUAL" then prog = "SPORT" end
  for i = 1, 4 do
    local p = PROGS[i]
    SHW[p] = (p == prog) and 1 or 0
  end
  local w0, prog0, sEff0, kUp0, kDn0, mm0 = S.w, S.prog, S.sEff, S.kUp, S.kDn, S.modeMax
  local top0, rlx0, sUp0, kUs0 = S.topGear, S.sportRelax, S.sEffUp, S.kUpShift
  S.w, S.prog = SHW, prog
  -- same capped adaptation, top gear and SPORT relax as the D program
  S.sEff = min(pv(PT.sEffCap), clamp01(RULES.adaptRaw(PT.adaptGain)))
  S.kUp = pvs(PT.kUpCalm, PT.kUpDyn) * ((S.trailer > 0) and K.trailerKUp or 1)
  S.kDn = pvs(PT.kDnCalm, PT.kDnDyn)
  local gb = D.gearbox
  S.modeMax = (gb and num(gb.maxGearIndex, nil)) or mm0
  S.topGear = (prog == "SPORT") and min(D.topSport, S.modeMax) or S.modeMax
  S.sportRelax = S.sportCalm and prog == "SPORT"
  RULES.applyRelax()
  local t = RULES.lug() or RULES.fullUp()
  if not t then
    local k = RULES.downLoop(S.uDn, S.evLogic)
    if k < g then t = k else t = RULES.upCandidate() end
    -- SPORT: no overdrive recommended either
    if (t or g) > S.topGear then t = RULES.sportTopTarget() or t end
  end
  S.w, S.prog, S.sEff, S.kUp, S.kDn, S.modeMax = w0, prog0, sEff0, kUp0, kDn0, mm0
  S.topGear, S.sportRelax, S.sEffUp, S.kUpShift = top0, rlx0, sUp0, kUs0
  return t or g
end

-- ----------------------------------------------------------------------------
-- Public API (Hn = hook markers of etk854hybrid_8hp.lua)
-- ----------------------------------------------------------------------------
-- H2: gear decision (updateInGear and updateInGearArcade). S.planned: D / S
-- the target or debounced candidate, M<n> / temp manual shadowPlan(), else
-- the engaged gear.
function C.decide(dt, M)
  dt = clamp(num(dt, 0.016), 0.001, 0.1)
  local gb = D.gearbox
  if not gb then return 0 end
  S.clock = S.clock + dt
  -- a target that never started a shift must not label a later one
  if S.pendTarget and not gb.isShifting and S.clock - S.pendClock > K.pendMaxAge then
    S.pendTarget = nil
  end
  senseGearbox()
  senseDriver(dt, M)
  senseHybrid(dt)
  selectAxis(dt)
  senseTorque(dt)
  updateProgram(dt)
  updateDemand(dt)
  updateHolds(dt)
  S.inhibit = ""
  local g = S.g
  S.target = g
  local mode = D.AH.mode
  -- R / N / P, manual M<n> and gear <= 0 keep the base behaviour (hold)
  if g < 1 or not (mode == "D" or mode == "S") then
    local manual = g >= 1 and type(mode) == "string" and mode:sub(1, 1) == "M"
    S.planned = manual and shadowPlan() or g
    return g
  end

  writeShiftBehavior(M)
  local tgt, why, t = RULES.run(dt)
  if tgt and tgt ~= g then
    tgt = clamp(tgt, 1, S.modeMax)
    if tgt ~= g then
      S.pendTarget, S.pendReason, S.pendTime = tgt, why or "?", t or 0.30
      S.pendClock = S.clock
      S.shiftActive, S.shiftUp, S.shiftTau, S.shiftT, S.shiftWait = true, tgt > g, 0, t or 0.30, 0
      S.lastReason, S.target = why or "?", tgt
      S.upCand, S.upCandT = nil, 0
      if tgt > g then S.uLatch = -1 end
      S.planned = tgt
      return tgt
    end
  end
  if S.tm.active then
    S.planned = shadowPlan()
  else
    S.planned = S.upCand or g
  end
  return g
end

-- H3: M<n> manual mode - kickdown edge and EV lug protection only
function C.manualOverride(gearIndex, M)
  local gb = D.gearbox
  if not gb or gb.isShifting or S.g < 1 then return gearIndex end
  if S.pRaw >= P.kickdownPedal and S.kdArmed then
    S.kdArmed = false
    local h = RULES.kickTarget()
    if h and h < gearIndex then
      S.pendTarget, S.pendReason, S.pendClock = h, "Mkd", S.clock
      S.pendTime = RULES.kdTime(S.g - h)
      S.lastReason, S.target = "Mkd", h
      return h
    end
  end
  if RULES.evLugging() and gearIndex > 1 then
    S.pendTarget, S.pendReason, S.pendTime = gearIndex - 1, "lug", K.tLug
    S.pendClock = S.clock
    S.lastReason, S.target = "lug", gearIndex - 1
    return gearIndex - 1
  end
  return gearIndex
end

-- H4: lock-up ratio: latched lock with hysteresis instead of the base linear
-- ramp; EV thresholds when the ICE is not coupled
function C.lockup(dt, M)
  local sm = D.lockSm
  if not sm then return 0 end
  dt = clamp(num(dt, 0.016), 0.001, 0.1)
  local gb = D.gearbox
  local g = gb.gearIndex or 0
  local mode = D.AH.mode
  local L = 0
  local hard = false
  if g <= 0 or mode == "P" or mode == "N" or mode == "R" then
    S.lockLatched = false
  elseif S.abs or (S.coupled and (S.nIce < 1.15 * P.idle or S.decel > 7)) then
    -- ABS, ICE stall guard, emergency stop: open NOW
    hard = true
    S.lockLatched = false
  elseif (S.brake >= 0.2 and S.v < 4.2) or S.launch == "READY"
      or (S.launch == "ACTIVE" and S.launchT < 1.0) then
    S.lockLatched = false
  elseif g == 1 and g < P.lockupMinGear then
    -- 1st gear: ramp 12 -> 20 km/h below 0.7 load pedal
    local kmh = S.v * 3.6
    L = (kmh >= 12 and S.pLoad < 0.7) and clamp01((kmh - 12) / 8) or 0
  elseif g >= P.lockupMinGear then
    local nOn = S.coupled and P.tcLockupRPM or 800
    local nOff = S.coupled and max(1.30 * P.idle, nOn - 350) or 500
    if S.lockLatched and S.nIn < nOff then S.lockLatched = false end
    if S.lockLatched then
      L = 1
    else
      L = clamp01((S.nIn - (nOn - P.lockupRange)) / P.lockupRange)
      if L >= 1 then S.lockLatched = true end
    end
  end

  if L > 0 then
    local sm_ = mode == "S" or (type(mode) == "string" and mode:sub(1, 1) == "M")
    local sporty = sm_ or S.prog == "SPORT" or S.prog == "MANUAL"
    -- tip-in slip like the base partial lock-up, gated on the uncapped adaptation
    if (S.prog == "ECO" or S.prog == "COMFORT") and g <= 3
       and RULES.adaptRaw(PT.slipAdaptGain) < 0.5 then
      L = L * (1 - max(0, S.pLoad - 0.85) * 1.5)
    end
    if gb.isShiftingUp then
      L = min(L, sporty and 1.0 or lerp(0.6, 1.0, S.sEff))
    elseif gb.isShiftingDown then
      L = min(L, (g <= 1) and 0 or (sporty and 0.6 or 0.3))
    end
    local es = S.engState
    if P.startIsolation < 1 and S.v > 2.78
       and (es == "CRANK" or es == "SYNC" or es == "ENGAGE" or es == "BLEND") then
      L = min(L, P.startIsolation)
    end
  end

  if hard then
    sm:set(0)
    return 0
  end
  -- no blanket brake release: recuperation needs the lock
  return sm:get(clamp01(L), dt)
end

-- H5: shift start; returns gearChangeTime and whether to cut ignition
function C.onShiftStart(newGear, M)
  local gb = D.gearbox
  local old = gb.gearIndex or 0
  local t, why
  if S.pendTarget == newGear and newGear ~= old and S.clock - S.pendClock <= K.pendMaxAge then
    t, why = S.pendTime, S.pendReason
  else
    -- lever/paddle in M, arcade R/N handling, mode clamps
    why = "ext"
    t = (newGear > old and old >= 1) and RULES.upTime(newGear - old) or 0.25
  end
  S.pendTarget = nil
  local mode = D.AH.mode
  local sm_ = mode == "S" or (type(mode) == "string" and mode:sub(1, 1) == "M")
  local up = newGear > old and old >= 1
  if up and sm_ and S.pRaw > 0.3 then t = P.sportGearChangeTime end
  t = max(num(t, 0.30), num(gb.minimumGearChangeTime, 0.10))
  local cut = up and S.pRaw > 0.6 and (sm_ or (S.prog == "SPORT" and S.sEff > 0.6)) and S.k0 > 0.95

  S.shiftActive, S.shiftUp, S.shiftT, S.shiftTau, S.shiftWait = true, up, t, 0, 0
  S.tSinceShift = 0
  if old >= 1 and newGear >= 1 then
    if newGear > old then
      S.tSinceUp = 0
    else
      S.tSinceDn = 0
      S.lastDnPassive = K.PASSIVE[why] == true
    end
    if why == "brakeDn" then S.tSinceBrakeStep = 0 end
    RULES.noteShift((newGear > old) and 1 or -1, why)
    local R = D.R
    local nNew = S.n * R[newGear] / R[old]
    log("I", "etk854hybrid8hp", string.format(
      "[8HP] %s %d->%d rule=%s prog=%s s=%.2f u=%.2f uUp=%.2f n=%d->%d eng=%s k0=%.2f t=%.2f",
      up and "up" or "down", old, newGear, why, S.prog or "?", S.sEff, S.u, S.uUp,
      floor(S.n + 0.5), floor(nNew + 0.5), S.engState, S.k0, t))
  end
  return t, cut
end

-- H8: every GFX frame (in-gear and while-shifting paths)
function C.publish(dt, M)
  dt = clamp(num(dt, 0.016), 0.001, 0.1)
  local ev = electrics.values
  local gb = D.gearbox
  local coef = 1
  if S.shiftActive then
    S.shiftTau = S.shiftTau + dt
    local shifting = gb and gb.isShifting
    if not shifting then S.shiftWait = S.shiftWait + dt else S.shiftWait = 0 end
    -- torque intervention during an upshift under load (applied by the VCU)
    if S.shiftUp and S.pRaw > 0.05 and S.shiftT > 0 then
      coef = 1 - (1 - S.cMin) * sin(PI * clamp01(S.shiftTau / S.shiftT))
    end
    if (S.shiftTau > S.shiftT + 0.05 and not shifting) or S.shiftWait > 0.30 then
      S.shiftActive = false
    end
  end
  ev.etk854hybrid_shiftTorqueCoef = coef
  ev.etk854hybrid_shiftProgram = S.progLabel
  ev.etk854hybrid_shiftAdapt = S.sEff
  ev.etk854hybrid_shiftSource = S.source
  ev.etk854hybrid_shiftReason = S.lastReason
  ev.etk854hybrid_shiftInhibit = S.inhibit
  ev.etk854hybrid_shiftTargetGear = S.target
  ev.etk854hybrid_shiftActive = S.shiftActive and 1 or 0
  ev.etk854hybrid_shiftPending = S.pending and 1 or 0
  ev.etk854hybrid_inputRpm = S.nIn
  ev.etk854hybrid_shiftLoadU = S.u -- load axis the maps are read on
  ev.etk854hybrid_gearPlanned = S.planned or 0 -- cluster shift recommendation
  ev.etk854hybrid_shiftTopGear = S.topGear
  ev.etk854hybrid_shiftRelax = S.sportRelax and 1 or 0
  ev.etk854hybrid_gbRatio = gb and gb.gearRatio or 0 -- momentary ratio (shift-duration telemetry)
  -- Tav in this gear: cap of the VCU's wheel-torque demand reference
  ev.etk854hybrid_shiftTavNm = Tav(S.n)
  -- EV power line: blend weight, EV upshift threshold [post-shift rpm]
  ev.etk854hybrid_shiftEvPowK = S.kPow
  ev.etk854hybrid_shiftEvUpRpm = S.evUp
  if S.sportFromLocal then ev.etk854hybrid_sportIndexLocal = S.sLocal end

  if P.debug >= 1 then
    S.dbgT = S.dbgT + dt
    if S.dbgT >= 1.0 then
      S.dbgT = 0
      log("I", "etk854hybrid8hp", string.format(
        "[8HP] dbg g=%d n=%d nIn=%d u=%.2f uUp=%.2f uDn=%.2f sIdx=%.2f sEff=%.2f prog=%s src=%s eng=%s inh=%s Treq=%d ev=%d cap=%d",
        S.g, floor(S.n), floor(S.nIn), S.u, S.uUp, S.uDn, S.sIdx, S.sEff, S.progLabel, S.source,
        S.engState, S.inhibit, floor(S.Treq), floor(S.evAvail), floor(S.sysCap)))
    end
  end
end

-- H11: temporary manual (D -> M<g> on a paddle tap, auto-return); true when
-- the tap was consumed
function C.tempManual(dir, M)
  if not P.paddleTempManual then return false end
  local gh = M.gearboxHandling
  if type(gh) == "table" and gh.behavior == "arcade" then return false end
  local gb = D.gearbox
  if D.AH.mode ~= "D" or S.v <= K.stopSpeed or (gb.gearIndex or 0) < 1 then return false end
  local tm = S.tm
  local g = gb.gearIndex
  local t = clamp(g + tm.req + dir, 1, S.modeMax)
  if dir < 0 and S.n * D.R[t] / D.R[g] > P.nUpFull - 200 then
    t = g + tm.req -- would over-rev: ignore this tap
  end
  tm.req = t - g
  tm.active, tm.idleT = true, 0
  return true
end

function C.isTempManual()
  return S.tm ~= nil and S.tm.active == true
end

-- H6: adaptation index 0..1 (the shift logic's shiftingAggression)
function C.adapt()
  return S.sEff or 0
end

-- override of the deMode-derived program; nil / "AUTO" = follow it again
-- (lever S / M<n> always win)
function C.setProgram(p)
  if p == "AUTO" or p == "" then p = nil end
  if p == nil or p == "ECO" or p == "COMFORT" or p == "SPORT" or p == "MANUAL" then
    S.override = p
    return true
  end
  return false
end

-- returns HUD label (e.g. "COMFORT+"), active program, override (or nil)
function C.getProgram()
  return S.progLabel, S.prog, S.override
end

-- state snapshot for the QA suite / dev tools (new table per call)
function C.getDebug()
  return {
    g = S.g, n = S.n, nIn = S.nIn, nShaft = S.nShaft, nIce = S.nIce,
    u = S.u, uPed = S.uPed, uUp = S.uUp, uDn = S.uDn, sIdx = S.sIdx, sEff = S.sEff, prog = S.prog,
    label = S.progLabel, source = S.source, engState = S.engState, k0 = S.k0,
    inhibit = S.inhibit, reason = S.lastReason, target = S.target, planned = S.planned,
    Treq = S.Treq, evAvail = S.evAvail, sysCap = S.sysCap, evUp = S.evUp, evDn = S.evDn,
    lugMinEff = S.lugMinEff, upFloor = S.upFloor, hunt = S.hunt, frozen = S.frozen,
    cornerHold = S.cornerHold, liftOffT = S.liftOffT, tempManual = S.tm.active,
    nUpFull = P.nUpFull, kdMax = P.kdMax, family = P.family,
    icePending = S.icePending, evLogic = S.evLogic, capActive = S.capActive, startT = S.startT,
    startFreeze = S.startFreeze, nDotG = S.nDotG,
    topGear = S.topGear, sportRelax = S.sportRelax, sportCalmT = S.sportCalmT, liftT = S.liftT,
    lastDnPassive = S.lastDnPassive, tUpAfterUp = S.tUpAfterUp, tUpAfterDn = S.tUpAfterDn,
    startCap = S.startCap, engageMin = S.engageMin, coastTarget = S.coastTarget, brakeCalm = S.brakeCalmP,
    evIce = S.evIce, holdKind = S.holdKind, holdRemain = S.holdRemain, regenTq = S.regenTq,
    pRaw = S.pRaw, pLoad = S.pLoad, evFracRaw = S.evFracRaw, kPow = S.kPow, evPowLine = P.evPowLine,
    kdOn = S.kdOn, kdHoldT = S.kdHoldT,
  }
end

-- ----------------------------------------------------------------------------
-- Init (on spawn AND on every reset: vehicleController re-inits the shift logic)
-- ----------------------------------------------------------------------------
local function newState()
  return {
    clock = 0,
    g = 0, n = 0, nIn = 0, nShaft = 0, nIce = 0, modeMax = 8,
    pRaw = 0, pLoad = 0, evFracRaw = 0, kPow = 0,
    brake = 0, u = 0, uPed = 0, uShaped = 0, axisIce = nil, uSm = 0, pSm = 0, pF = 0, dpdt = 0,
    v = 0, vPrev = 0, dvF = 0, decel = 0, gxS = 0,
    slipUpOK = true, slipDnOK = true,
    deMode = "COMFORT", edrive = "AUTO_EDRIVE",
    k0 = 1, engState = "ON", engFallback = true, coupled = true, lugCoupled = true, isEV = false,
    icePending = false, evLogic = false, startT = 0, startFreeze = false, syncBlocked = false,
    lead = 0, nDotG = 0, capActive = false, tcT = 0,
    sail = "DRIVE", launch = "IDLE", launchT = 0, regenLvl = "MED", regenOn = false, regenTq = 0,
    holdKind = "none", holdRemain = 0, evIce = false,
    soc = 1, stage = 0, evCorner = K.EV_CORNER[0], charging = false, trailer = 0, tc = false, abs = false,
    tsa = false,
    source = "ICE",
    evAvail = 0, Treq = 0, Dw = 0, sysCap = K.SYS_CAP_DEFAULT, evSat = false,
    sIdx = 0, sLocal = 0, sportFromLocal = true, sl = {I = 0, h = 0},
    prog = nil, override = nil, lambda = 1,
    w = {ECO = 0, COMFORT = 1, SPORT = 0, MANUAL = 0},
    wFrom = {ECO = 0, COMFORT = 1, SPORT = 0, MANUAL = 0},
    sEff = 0, sEffUp = 0, suffixLevel = 0, progLabel = "COMFORT",
    kUp = 0.75, kUpShift = 0.75, kDn = 0.95, kRate = 2.5, cMin = 0.75,
    tUpAfterUp = 0.35, tUpAfterDn = 1.2, tDnAfterUp = 1.2, tDnAfterDn = K.tDnAfterDn, lastDnPassive = false,
    topGear = 8, startCap = 3500, engageMin = 0, coastTarget = 1300, brakeCalmP = 1800,
    sportCalmT = 0, sportCalm = false, sportRelax = false, sportTopT = 0, liftT = 0,
    uInt = 0, DwInt = 0, uUp = 0, uDn = 0, DwUp = 0,
    evUp = 1000, evDn = 750, evFloor = 900, nFullEV = 5000,
    lugMinEff = 950, upFloor = 1100,
    frozen = false, freezeWhy = "", frzT = 0, frzTotT = 0, frzPhase = "", frzLogged = false,
    kdArmed = true, kdLatch = false, kdLatchT = 0, kdHoldT = 0, kdOn = false,
    cornerHold = false, cornerRelT = 0,
    pPeak = 0, pPeakT = 0, liftOffT = 0, liftLow = true,
    hunt = false, huntT = 0, lastDir = 0, rv1 = -1e9, rv2 = -1e9, rv3 = -1e9, huntP = 0, huntB = 0,
    tSinceShift = 99, tSinceUp = 99, tSinceDn = 99, tSinceBrakeStep = 99,
    upCand = nil, upCandT = 0, upCandDeb = 0.20, tdIceT = 0, tdEvT = 0, uLatch = -1, planned = 0,
    pendTarget = nil, pendReason = "", pendTime = 0.30, pendClock = -1e9,
    shiftActive = false, shiftUp = false, shiftT = 0, shiftTau = 0, shiftWait = 0,
    pending = false, inhibit = "", lastReason = "", target = 0,
    lockLatched = false,
    tm = {active = false, req = 0, idleT = 0},
    dbgT = 0,
  }
end

-- highShiftUpRPM may be a scalar or a per-gear table
local function maxEntry(x)
  if type(x) == "table" then
    local m = 0
    for _, v in pairs(x) do
      if type(v) == "number" and v > m then m = v end
    end
    return (m > 0) and m or nil
  end
  return num(x, nil)
end

local function initParams(jb, engine)
  local cfg = (type(jb.etk854hybrid8hp) == "table") and jb.etk854hybrid8hp or {}
  -- engine calibration: ECU part block, else the powertrain part's default
  -- block (i4 on the stock ECU), else FAMILY
  local eng = (type(jb.etk854hybrid8hpEngine) == "table") and jb.etk854hybrid8hpEngine or nil
  P.engCfgSrc = "ecu"
  if not eng then
    eng = (type(jb.etk854hybrid8hpEngineDefault) == "table") and jb.etk854hybrid8hpEngineDefault or {}
    P.engCfgSrc = (next(eng) ~= nil) and "default" or "family"
  end
  local family = eng.family
  if family ~= "petrol" and family ~= "diesel" then
    family = (engine and engine.requiredEnergyType == "diesel") and "diesel" or "petrol"
  end
  local fam = FAMILY[family]
  P.family = family
  P.engCfgFound = P.engCfgSrc == "ecu"

  P.offsetRPM = clamp(num(jb.etk854hybrid8hpOffsetRPM, 0), -300, 400)
  P.adaptGainUser = clamp(num(jb.etk854hybrid8hpAdaptGain, 1.0), 0, 1.5)
  P.paddleTempManual = cfg.paddleTempManual == true
  P.legacyPedalCut = cfg.legacyPedalCut == true
  P.startIsolation = clamp(num(cfg.startIsolation, 1.0), 0, 1)
  P.kickdownPedal = clamp(num(cfg.kickdownPedal, 0.95), 0.80, 1.0)
  P.evPowLine = cfg.evPowerLineEco ~= false and cfg.evPowerLineEco ~= 0 -- default on
  P.freezeWatchdogS = clamp(num(cfg.freezeWatchdogS, 1.5), 0.5, 5)
  -- CRANK/SYNC gear hold, bounded: an unbounded hold deadlocks with the
  -- sequencer's rev ceiling
  P.startFreezeS = clamp(num(cfg.startFreezeS, 1.2), 0.3, 3)
  -- global ICE-start rev cap, upper bound of the per-program P.startCap
  P.startCapRPM = clamp(num(cfg.iceStartCapRPM, 3500), 2000, 6000)
  P.debug = num(cfg.debug, 0)

  P.idle = num(engine and engine.idleRPM, 700)
  P.lim = num(engine and engine.maxRPM, 6000) -- = rev limiter (combustionEngine)
  -- full-load upshift point: engine block nUpFullRPM, else highShiftUpRPM
  local hs = num(eng.nUpFullRPM, nil) or maxEntry(jb.highShiftUpRPM)
  P.nUpFull = min(hs or (P.lim - 200), P.lim - 200)
  P.kdMax = min(P.nUpFull - 300, P.lim - 400)

  P.lugMin = num(eng.lugMinRPM, fam.lugMinRPM)
  P.coastDn = num(eng.coastDnRPM, fam.coastDnRPM)
  P.evLugBonus = num(eng.evLugBonusRPM, fam.evLugBonusRPM)
  local ramp = (type(eng.turboRampRPM) == "table") and eng.turboRampRPM or fam.turboRampRPM
  P.ramp1 = num(ramp[1], fam.turboRampRPM[1])
  P.ramp2 = num(ramp[2], fam.turboRampRPM[2])
  P.lagMin = clamp(num(eng.lagMin, fam.lagMin), 0.1, 1)
  P.resvMax = num(eng.resvMaxRPM, fam.resvMaxRPM)
  local bt = (type(eng.brakeTargetRPM) == "table") and eng.brakeTargetRPM or fam.brakeTargetRPM
  P.brakeCalm, P.brakeDyn = {}, {}
  for _, p in ipairs({"ECO", "COMFORT", "SPORT"}) do
    local row = (type(bt[p]) == "table") and bt[p] or fam.brakeTargetRPM[p]
    P.brakeCalm[p] = num(row[1], fam.brakeTargetRPM[p][1])
    P.brakeDyn[p] = num(row[2], fam.brakeTargetRPM[p][2])
  end
  P.brakeCalm.MANUAL, P.brakeDyn.MANUAL = P.brakeCalm.SPORT, P.brakeDyn.SPORT
  -- per-program torque-first keys; SPORT / MANUAL use the global start cap
  -- and the COMFORT engagement floor
  local sc = (type(eng.startCapRPM) == "table") and eng.startCapRPM or {}
  local em = (type(eng.iceEngageMinRPM) == "table") and eng.iceEngageMinRPM or {}
  local ct = (type(eng.coastTargetRPM) == "table") and eng.coastTargetRPM or {}
  P.startCap, P.engageMin, P.coastTarget = {}, {}, {}
  for _, p in ipairs({"ECO", "COMFORT"}) do
    P.startCap[p] = clamp(num(sc[p], fam.startCapRPM[p]), 2000, P.startCapRPM)
    P.engageMin[p] = clamp(num(em[p], fam.iceEngageMinRPM[p]), 0, 2500)
  end
  P.startCap.SPORT, P.startCap.MANUAL = P.startCapRPM, P.startCapRPM
  P.engageMin.SPORT, P.engageMin.MANUAL = P.engageMin.COMFORT, P.engageMin.COMFORT
  for _, p in ipairs({"ECO", "COMFORT", "SPORT"}) do
    P.coastTarget[p] = clamp(num(ct[p], fam.coastTargetRPM[p]), 900, 3500)
  end
  P.coastTarget.MANUAL = P.coastTarget.SPORT
  P.sportRelaxRPM = clamp(num(eng.sportRelaxRPM, fam.sportRelaxRPM), 1000, 3500)
  P.knee, P.kneeRef = fam.knee, fam.ref

  -- converter lock-up (a range outside 100..600 is clamped + logged)
  P.tcLockupRPM = num(jb.torqueConverterLockupRPM, 1200)
  local rawRange = num(cfg.lockupRangeRPM, num(jb.torqueConverterLockupRange, 250))
  P.lockupRange = clamp(rawRange, 100, 600)
  P.lockupRangeClamped = rawRange ~= P.lockupRange
  P.lockupMinGear = clamp(floor(num(cfg.lockupMinGear, num(jb.torqueConverterLockupMinGear, 2))), 1, 3)
  P.lockEngage = num(jb.torqueConverterLockupRate, 4)
  P.lockRelease = num(jb.torqueConverterLockupInRate, P.lockEngage * 2)
  P.sportGearChangeTime = num(jb.sportGearChangeTime, 0.08)

  local motor = powertrain.getDevice("integratedMotor")
  P.motorMaxRPM = motor and num(motor.maxRPM, nil) or (motor and num(motor.maxAV, 0) * AV2RPM) or 10000
  if P.motorMaxRPM <= 0 then P.motorMaxRPM = 10000 end

  local capSec = v and v.data and v.data.etk854hybridSystemCap
  P.sysCapBase = (type(capSec) == "table" and num(capSec.baseNm, nil)) or K.SYS_CAP_DEFAULT
  return eng
end

-- family maps knee-scaled to this engine's nUpFull; per-ECU override rows
-- are absolute (not scaled)
local function initMaps(eng)
  local src = MAPS[P.family]
  local fac = (P.nUpFull - P.knee) / max(P.kneeRef - P.knee, 1)
  local override = (type(eng.maps) == "table") and eng.maps or {}
  local maps = {}
  for _, p in ipairs({"ECO", "COMFORT", "SPORT"}) do
    local m = {}
    local ov = (type(override[p]) == "table") and override[p] or {}
    for _, key in ipairs({"upCalm", "upDyn", "dnCalm", "dnDyn"}) do
      local row = {}
      local o = ov[key]
      if type(o) == "table" and #o == 8 then
        for i = 1, 8 do row[i] = num(o[i], src[p][key][i]) end
      else
        for i = 1, 8 do
          local x = src[p][key][i]
          if x > P.knee then x = P.knee + (x - P.knee) * fac end
          row[i] = x
        end
      end
      m[key] = row
    end
    maps[p] = m
  end
  maps.MANUAL = maps.SPORT
  D.maps = maps
end

-- invariants asserted once per init (W log, never fatal)
local function checkMaps()
  local bad = 0
  for _, p in ipairs({"ECO", "COMFORT", "SPORT"}) do
    local m = D.maps[p]
    for _, key in ipairs({"upCalm", "upDyn", "dnCalm", "dnDyn"}) do
      local r = m[key]
      for i = 2, 8 do
        if r[i] >= 0 and r[i - 1] >= 0 and r[i] < r[i - 1] then bad = bad + 1 end
      end
    end
    for i = 1, 8 do
      if m.upCalm[i] >= 0 and m.upDyn[i] >= 0 and m.upCalm[i] > m.upDyn[i] then bad = bad + 1 end
      if m.dnCalm[i] > m.dnDyn[i] then bad = bad + 1 end
    end
    for i = 1, 7 do
      if m.upCalm[i] >= 0 and m.dnCalm[i] > m.upCalm[i] - K.hyst then bad = bad + 1 end
      if m.upDyn[i] >= 0 and m.dnDyn[i] > m.upDyn[i] - K.hyst then bad = bad + 1 end
    end
  end
  if bad > 0 then
    log("W", "etk854hybrid8hp", string.format("[8HP] map invariants violated in %d place(s) (family=%s)", bad, P.family))
  end
end

-- torqueData curves are indexed rpm + 1 (combustionEngine getTorqueData)
local function buildIceTable(engine)
  D.iceTab, D.iceN = nil, 0
  local td = engine and engine.torqueData
  local curves = td and td.curves
  local curve = curves and curves[td.finalCurveName or 1]
  local tq = curve and curve.torque
  if type(tq) ~= "table" then return end
  local n = floor(P.lim / 25)
  local tab = {}
  for i = 0, n do
    local x = tq[i * 25 + 1]
    if type(x) ~= "number" then x = (i > 0) and tab[i - 1] or 0 end
    tab[i] = max(0, x)
  end
  D.iceTab, D.iceN = tab, n
end

function C.init(jbeamData, engine, gearbox, torqueConverter, sharedFunctions, automaticHandling)
  local jb = (type(jbeamData) == "table") and jbeamData or {}
  D = {engine = engine, gearbox = gearbox, tc = torqueConverter, AH = automaticHandling,
       R = (gearbox and gearbox.gearRatios) or {}}
  P = {}
  S = newState()
  local eng = initParams(jb, engine)
  initMaps(eng)
  checkMaps()
  buildIceTable(engine)

  -- full-load post-shift rpm per gear (the -1 map columns)
  local R = D.R
  D.F = {}
  local gmax = gearbox and gearbox.maxGearIndex or 0
  for g = 1, gmax - 1 do
    if R[g] and R[g + 1] and R[g] ~= 0 then D.F[g] = P.nUpFull * R[g + 1] / R[g] end
  end
  -- SPORT top gear: highest gear with |R| >= 1 (6th = 1.00)
  D.topSport = gmax
  for g = gmax, 1, -1 do
    if R[g] and abs(R[g]) >= 0.999 then
      D.topSport = g
      break
    end
  end
  -- newTemporalSmoothing(inRate, outRate) uses the SECOND rate moving away
  -- from 0 (engage), the first towards 0 (release)
  D.lockSm = torqueConverter and newTemporalSmoothing(P.lockRelease, P.lockEngage) or nil

  C.P, C.S = P, S
  log("I", "etk854hybrid8hp", string.format(
    "[8HP] init family=%s nFull=%d lim=%d idle=%d lugMin=%d resvMax=%d tcOn=%d paddleTM=%s legacyCut=%s ecuCfg=%s engCfg=%s",
    P.family, floor(P.nUpFull), floor(P.lim), floor(P.idle), floor(P.lugMin), floor(P.resvMax),
    floor(P.tcLockupRPM), tostring(P.paddleTempManual), tostring(P.legacyPedalCut), tostring(P.engCfgFound),
    P.engCfgSrc))
  log("I", "etk854hybrid8hp", string.format(
    "[8HP] init topSport=%d startCap E/C=%d/%d engageMin E/C=%d/%d coast E/C/S=%d/%d/%d brakeCalm E/C/S=%d/%d/%d relax=%d evPowLine=%s",
    D.topSport, floor(P.startCap.ECO), floor(P.startCap.COMFORT), floor(P.engageMin.ECO), floor(P.engageMin.COMFORT),
    floor(P.coastTarget.ECO), floor(P.coastTarget.COMFORT), floor(P.coastTarget.SPORT),
    floor(P.brakeCalm.ECO), floor(P.brakeCalm.COMFORT), floor(P.brakeCalm.SPORT), floor(P.sportRelaxRPM),
    tostring(P.evPowLine)))
  if P.lockupRangeClamped then
    log("W", "etk854hybrid8hp", string.format(
      "[8HP] torqueConverterLockupRange clamped to %d rpm (base-logic value still in jbeam? see CR-J1)", floor(P.lockupRange)))
  end
  if not D.iceTab then
    log("W", "etk854hybrid8hp", "[8HP] no ICE torque curve found - reserve checks use the motor only")
  end
end

C.P, C.S = P, S

return C
