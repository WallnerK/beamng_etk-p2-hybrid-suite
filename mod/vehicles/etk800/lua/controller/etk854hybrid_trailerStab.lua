-- ETK 854 ActiveHybrid - trailer stability (TSA), part of the tow hitch
--
-- Declared with its calibration section by the hitch parts etk854hybrid_towhitch_eu / _hd (towing jbeam);
-- dev switch: hidden jbeam variable $etk854_tsa_enable (default 1) and setEnabled(), no driver switch.
-- Inputs (production stability sensors only): CMU yaw rate, lateral acceleration (sensorHub accelerationX, + = left,
-- negated; readCmu), pitch rate, reference speed; steering,
-- gear, drive / brake state, RAS channels, coupling (selfLeveling channel + own coupler hooks) and a
-- virtual 3-axis load pin from the hitch-carrier beam stresses. No trailer pose, articulation angle or
-- trailer Lua state.
-- Signs: + = right for yaw rates / errors, steering and rear-steer angles, lateral quantities; load pin
-- Fx + = pull, Fy + = trailer pushes the hitch right, Fz + = down load. Units deg/s, deg, N, kg, m/s^2
-- (decelerations positive); km/h on channels, m/s inside formulas.
--
-- Arming: trailer mode on, DSC not OFF, forward gear, CMU normal, yaw control present, no launch control.
-- DSC: sway stages + corner pre-braking; DTC: sway stages only. Coupled without trailer mode: OFF + one
-- hint (the coupling still selects load pin, estimators and trailer thresholds). No trailer-mode provider
-- on the car (nothing writes etk854hybrid_trailerMode): UNAVAILABLE, no hint, no load-pin sampling.
--
-- Sway: r_ref = G(v) * (df - dr_ff) (single track, adapted characteristic speed, dr_ff = rear-steer
-- feed-forward), e = r - r_ref, band-pass 0.3-1.2 Hz (detection) / 0.25-2.5 Hz (control). Schmitt-trigger
-- half-waves are validated by frequency band, driver-band ratio q, steer-rate hold, plausibility (ay or
-- hitch force) and a raw consistency test (a filter rebound has no raw counterpart). Stages:
--   1 silent  rear-steer damping, torque hold, no regen, freeze, gear hold
--   2 lamp    wheel torque 0, anti-phase single front-wheel brake, reduced rear-steer gain
--             (+ event deceleration above the recommended speed)
--   3 lamp    + combination deceleration (VCU brake channel, push limiter), "Reduce speed"
-- Entries / escalations: onHw, stageStep. The actuators hide the sway from the yaw error, so in a stage the
-- machine uses max(yaw error, kFy * lateral hitch force), kFy latched at stage entry. Exits, persistence and
-- the watchdog judge validated sway only (valid yaw-error and in-band trailer-side half-waves): driver
-- corrections do not hold a stage.
-- Corner pre-braking (DSC only): predicted ay vs budget -> K1 torque hold, K2 lamp + torque 0 + deceleration
-- ("Reduce speed" once the prediction without the look-ahead held K2 for k2MsgS or reached k2MsgFrac x the budget);
-- K1 ends early when the driver unwinds; in a settled K1 below the K1 band the hold lets the torque to hold the speed back.
-- Stages 2, 3 and K2 = "DSC intervenes" (yaw control component: escActive, lamp qualifier).
--
-- Outputs: a raise-only addition to the FL / FR desiredBrakingTorque in updateWheelsIntermediate (after the
-- pedal path and its ABS) with a slip limiter (fade 5-8 %, cut above), and the etk854hybrid_tsa* channels
-- (single writer, 100 Hz; no other module's channel is written) for RAS, VCU, torque coordinator, ICE
-- sequencer, sailing, 8HP, cluster, centre screen, HUD:
--   State DISABLED|OFF|DSC_OFF|UNAVAILABLE|STANDBY|READY|ACTIVE|SUSPENDED|FAULT, Stage 0..3, Corner 0..2,
--   Lamp (braking stage, held 0.15 s), YawErrDegS (BP_c), AmpDegS, SwayIdx (amp / a2), FreqHz, RsReqDeg,
--   WheelNm (+ FR, - FL), BrakeCmd 0..0.35, DecelReq, TorqueMode (1 HOLD, 2 ZERO wheel torque), TorqueRec (HOLD
--   cap recovery rate [Nm/s] in a settled K1), Freeze (no ICE start/stop, sail entry, regen or charge load; gear
--   hold), AyPred, AyBudget, VchT, HitchFx/Fy/Fz,
--   DrawbarN, TongueKg (signed), TongueState NONE|OK|LOW|TAIL_HEAVY, MassKg (-1 unknown), MassQ,
--   LoadPin OFF|TARE|UNTARED|OK|INVALID|CONTACT|FAULT, VRecKmh, ReduceSpeed, Flags:
--   1 coupled, 2 trailer mode, 4 sway armed, 8 corner armed, 16 share usable, 32 share disabled, 64 pin
--   valid, 128 half-wave suppressed, 256 reactivation window, 512 wheel brake paused, 1024 push limiter,
--   2048 DTC, 4096 injection / forced stage, 8192 suspended, 16384 tongue valid, 32768 tongue low /
--   tail-heavy, 65536 yaw sign implausible, 131072 event hold, 262144 low-friction memory (corner budget x muLow)
--
-- Load pin: boundary beams = beams with exactly one node named pin.nodePrefix; F = -(sum stress * static
-- unit vector) in the vehicle frame, averaged over the 10 ms step. The tare removes the carrier weight
-- (re-tared at standstill without a trailer, CONTACT if far from it), axComp the carrier inertia in x.
-- Faults: the physics hooks and the 100 Hz core run in pcall; an error latches FAULT (requests 0) until reset.
-- Dev / QA API: exported at the end, pcall-safe; a vehicle reset clears every override except setEnabled.
-- Layout (LuaJIT allows 200 locals in the main chunk): a section keeps its private helpers in a do ... end
-- block and declares outside it only the names other sections use; the API is defined straight into M.

local M = {}
M.type = "auxiliary"
-- after CMU 50, yawControl 60, brakeControl 70, selfLeveling 83; before adaptiveSusp, diffLock, rearSteer, VCU
M.defaultOrder = 84

local abs, min, max, exp, sqrt, floor, sin, pi, huge = math.abs, math.min, math.max, math.exp, math.sqrt, math.floor, math.sin, math.pi, math.huge
local deg, rad = math.deg, math.rad
local sfmt, ssub = string.format, string.sub
local band = bit.band

-- constants read outside the physics-rate loops
local CONST = {
  NAME = "etk854hybrid_trailerStab",
  MSG_CAT = "vehicle.etk854hybrid.tsa",
  MSG_PFX = "Trailer stability: ",
  GROW_TOL = 0.01, -- growth resolution per half-wave (100 Hz peak sampling jitters ~0.01 %)
  OCC_MAX = 60,    -- occupancy buckets (persistOccWinS <= 60)
  MASS_RING = 64   -- mass history (3 s at <= 20 Hz) and step windows (2 x 20 + 24 samples)
}
local TXT = {
  reduce = "Trailer stability: reduce speed",
  cruise = "Trailer stability: cruise control off",
  pin = "Trailer stability: hitch force sensor fault - limited function",
  limited = "Trailer stability: limited function",
  fault = "Trailer stability: fault - function off",
  hint = "Trailer coupled - switch on trailer mode for trailer stability"
}
local HIST = 8          -- half-wave ring
local RATE_N = 5        -- steering-rate window in fixed steps (see refStep)
local AX_LP_HZ = 5      -- kinematic acceleration filter; the mass RLS passes the drawbar force through the same section
local RAD05, RAD2 = rad(0.5), rad(2)
local LIMIT_BLOCK = 3627 -- rasLimit 1+2+8+32+512+1024+2048: freeze, centre, no-increase, override, OFF, DSC OFF, neutral
-- yaw-sign plausibility (yscOK): |r| <= YSC_GAIN * |G*deff| + YSC_R0, CMU body slip <= YSC_BETA [rad]
local YSC_GAIN, YSC_R0, YSC_BETA = 1.25, rad(1), 0.05

local CFG = {} -- calibration (init)
local ST = {}  -- runtime state
local REG = {} -- discovery / registrations
local PIN = {} -- load pin
local EST = {} -- estimators
local F = {}   -- filter states

-- yaw control component (plain table, registered in initSecondStage)
local COMP = {componentOrderYawControl = 15, name = "etk854hybrid_tsaComp"}

-- ---------------------------------------------------------------------------
-- helpers (shared copies: etk854hybrid_util)
-- ---------------------------------------------------------------------------

local U = require("controller/etk854hybrid/etk854hybrid_util")
local num = U.finNum -- finite number (strings via tonumber), else d
local flag = U.flag
local fin = U.fin
local clamp = U.clampMinMax -- min(max(x, a), b)
local lin = U.lin -- clamped linear scale (NaN x -> y0)
local sgn = U.sgn
local sw = U.sw -- switch: boolean as is, number / string >= 0.5
local copy = U.copy

local function warn(msg)
  pcall(log, "W", CONST.NAME, msg)
end

local function info(msg)
  pcall(log, "I", CONST.NAME, msg)
end

-- exact first-order sections on small state tables {y, xp}
local function hp1(s, x, fc, dt)
  local a = exp(-2 * pi * fc * dt)
  local y = a * (s.y + x - s.xp)
  s.y, s.xp = y, x
  return y
end

local function lp1(s, x, fc, dt)
  local a = exp(-2 * pi * fc * dt)
  local y = a * s.y + (1 - a) * x
  s.y = y
  return y
end

local function newF()
  return {y = 0, xp = 0}
end

local function newBp()
  return {h1 = newF(), h2 = newF(), l1 = newF(), l2 = newF()}
end

-- detection band-pass BP_d = gBp * LP(LP(HP(HP(x))))
local function bpd(c, x, dt)
  local f = CFG.filt
  local hp, lp = f.bpdHpHz, f.bpdLpHz
  local y = hp1(c.h1, x, hp, dt)
  y = hp1(c.h2, y, hp, dt)
  y = lp1(c.l1, y, lp, dt)
  y = lp1(c.l2, y, lp, dt)
  return y * REG.gBp
end

local function bpReset(c)
  c.h1.y, c.h1.xp, c.h2.y, c.h2.xp, c.l1.y, c.l2.y = 0, 0, 0, 0, 0, 0
end

local function arr(n, x)
  local t = {}
  for i = 1, n do t[i] = x end
  return t
end

-- queued in-game messages (shown in updateGFX)
local function toast(text)
  local s = ST
  if s.msgN < 4 then
    s.msgN = s.msgN + 1
    s.msgQ[s.msgN] = text
  end
end

-- ---------------------------------------------------------------------------
-- configuration
-- ---------------------------------------------------------------------------

local parseCfg
do
  -- built-in copy of the hitch parts' calibration section (a missing or partial section still works).
  -- Units by key suffix (Kmh, Deg, DegS, Hz, S, Ms2, N, Nm, Kg, M); unsuffixed accelerations / budgets m/s^2,
  -- jerk m/s^3, vch* m/s, masses kg, the rest factors, gains or counts.
  local function defaults()
    return {
      gate = {swayOnKmh = 55, swayOffKmh = 45, cornerOnKmh = 40, cornerOffKmh = 35, stopKmh = 40, swayInDtc = true},
      ref = {frontAngleLinDeg = 33.67, frontAngleCubDeg = 3.63, frontRackFactorRef = 0.14, rasCreditTauS = 0.15,
             maxLatAcc = 9.5, steerRateLpHz = 10, vchDefault = 55},
      filt = {bpdHpHz = 0.3, bpdLpHz = 1.2, bpcHpHz = 0.25, bpcLpHz = 2.5, ampHoldTauS = 1.0,
              rawLpHz = 2.5, rawMeanHz = 0.05},
      det = {hystDegS = 0.5, fMinHz = 0.35, fMaxHz = 1.2, learnBand = 0.3, learnMinHw = 4,
             a1DegS = 1.5, n1 = 2, a2DegS = 2.0, n2 = 3, gNearCrit = 0.7, gNoDecay = 1.0, a3DegS = 3.0, n3 = 2, a3BigDegS = 5.0,
             growth = 1.25, growthRsDeg = 0.4, rawFrac = 0.5, soloS3MinHw = 4,
             exitDegS = 1.0, exitS = 1.5, reactS = 5.0, reactScale = 0.7, lowTongueScale = 0.8,
             s3ExitDegS = 1.5, s3DropKmh = 10, s3MaxS = 8.0,
             persistN = 3, persistWinS = 60.0, persistOccS = 10.0, persistOccWinS = 30, persistDropKmh = 15,
             maxActiveS = 20.0, suspendS = 60.0,
             qMax = 0.6, qMaxPin = 0.6, steerRateDegS = 10.0, steerHoldS = 0.3,
             ayMin = 0.15, fyMinN = 150, fyEarlyN = 600, lampOffHoldS = 0.15},
      ras = {k1 = 0.30, k2 = 0.18, maxDeg = 1.0, signWinS = 3.0, signFrac = -0.2, signMinSum = 0.5,
             signLagS = 0.12, signMinDeg = 0.2},
      wheel = {kNmPerDegS = 1000, dbDegS = 0.5, maxMzNm = 3000, lambda0 = 0.05, lambda1 = 0.08,
               rampNmS = 10000, rampDnNmS = 50000, halfTrackM = 0.78, pauseOnTcs = true},
      decel = {s3Ms2 = 2.5, s2EvtMs2 = 1.0, jerkUp = 10.0, jerkDn = 10.0, brakeMax = 0.35, kp = 0.04, ki = 0.08,
               iMax = 0.15, releasePerS = 2.0, capNoPinMs2 = 2.0, frictionMs2 = 8.0,
               pushMinN = 1200, pushFrac = 0.08, pushKp = 0.5, pushKi = 1.0, pushDecay = 1.0, massDefault = 1500},
      corner = {aB0 = 5.0, noCouplerScale = 1.2, tpS = 0.5, k1Frac = 0.9, k1ExitFrac = 0.8, k1ExitS = 0.5,
                k1UnwindS = 0.2, k1UnwindFrac = 0.8, k2ExitFrac = 0.85, k2Base = 2.5, k2Slope = 2.0, k2Max = 3.0,
                muLow = 0.7, muLowS = 60, muEventMs2 = 4.0,
                muMinKmh = 20, muBlankS = 3.0, muTcTf = 0.9, muQualS = 0.3, muSlip = 0.06,
                massRatioLo = 0.5, massRatioHi = 1.0, massFactorMin = 0.85, massFactorUnknown = 0.9,
                laMaxFrac = 1.5, respTauS = 0.5, laMinHoldS = 0.3, laRateFrac = 0.9, laRateLpHz = 4.0, laHoldS = 0.1,
                laOnce = true, ayMeasCap = 1.7, ayMeasLpHz = 3.0, windFrac = 0.1,
                tqRecNmS = 150, tqRecFrac = 0.9, tqRecDecMs2 = 0.05, tqRecLpHz = 1.0},
      pin = {enabled = true, nodePrefix = "hyb_tw", useTorsion = false, torsionSign = 1, imuComp = false,
             axComp = true, subSample = 2, tareS = 2.0, tareMaxKmh = 0.5, validMinKmh = 5, maxN = 25000,
             implausS = 0.2, implausHoldS = 2.0, contactN = 1500, deformEps = 0.001, checkS = 0.25,
             ctlLpHz = 5.0, displayHz = 1.0},
      est = {massHz = 10, massLambda = 0.9, massExcMin = 4.0, massMin = 0, massMax = 6000,
             massWinS = 0.6, massGapS = 0.8, massStepMs2 = 0.8, massSteadyMs2 = 0.3, massDvKmh = 4, massEvtMin = 2,
             massLpHz = 2.0, massQPublish = 0.5, tongueLpHz = 0.3, tongueLowKg = 25, tongueLowFrac = 0.04,
             vchWindowS = 2.0, vchLambda = 0.9, vchMin = 20, vchMax = 80, keepAfterResetS = 5.0, massQuietS = 2.0},
      rec = {vRecKmh = 100, evtDropKmh = 15, evtHoldS = 600, overKmh = 5, overS = 3, msgRepeatS = 20,
             lowTongueVRecKmh = 80, cruiseCancel = true, k2MsgS = 0.45, k2MsgFrac = 1.05},
      ysc = {minKmh = 20, maxKmh = 80, minDeffDeg = 1.0, minYawDegS = 3.0, maxDdotDegS = 2.0, minAyMs2 = 1.0,
             windowS = 1.0, windows = 3, agreeFrac = 0.9},
      dev = {componentOrder = 15}
    }
  end

  -- keys parsed without the ">= 0" clamp of non-negative defaults
  local SIGNED = {torsionSign = true}

  -- jbeam values of src over def (same shape); non-negative defaults stay >= 0 unless SIGNED
  local function parseInto(def, src, path, issues)
    if src ~= nil and type(src) ~= "table" then issues[#issues + 1] = path .. "(not a table)" end
    if type(src) ~= "table" then src = {} end
    for k, d in pairs(def) do
      local s = src[k]
      local td = type(d)
      if td == "number" then
        local x = num(s, nil)
        if x == nil then
          if s ~= nil then issues[#issues + 1] = path .. k end
          x = d
        end
        if d >= 0 and not SIGNED[k] and x < 0 then
          issues[#issues + 1] = path .. k
          x = 0
        end
        def[k] = x
      elseif td == "boolean" then
        def[k] = sw(s, d)
      elseif td == "string" then
        if type(s) == "string" and s ~= "" then
          def[k] = s
        elseif s ~= nil then
          issues[#issues + 1] = path .. k
        end
      elseif td == "table" then
        parseInto(d, s, path .. k .. ".", issues)
      end
    end
    return def
  end

  local function fixRange(t, k, lo, hi, path, issues)
    local x = t[k]
    if type(x) ~= "number" then return end
    if x < lo or x > hi then
      t[k] = clamp(x, lo, hi)
      issues[#issues + 1] = path .. k
    end
  end

  local function fixInt(t, k, lo, hi, path, issues)
    local x = t[k]
    local y = clamp(floor(x + 0.5), lo, hi)
    if y ~= x then
      t[k] = y
      issues[#issues + 1] = path .. k
    end
  end

  local INT_DET = {"n1", "n2", "n3", "persistN", "soloS3MinHw", "learnMinHw"}

  -- ranges after parsing (out of range -> clamp + listed in issues)
  local function validateCfg(c, issues)
    for sec, t in pairs(c) do
      if type(t) == "table" then
        for k, x in pairs(t) do
          if type(x) == "number" and ssub(k, -2) == "Hz" then
            if k == "rawMeanHz" then
              fixRange(t, k, 0.01, 0.5, sec .. ".", issues)
            else
              fixRange(t, k, 0.01, 20, sec .. ".", issues)
            end
          end
        end
      end
    end
    fixRange(c.decel, "brakeMax", 0, 0.5, "decel.", issues)
    fixRange(c.ras, "k1", 0, 0.6, "ras.", issues)
    fixRange(c.ras, "k2", 0, 0.6, "ras.", issues)
    fixRange(c.ras, "maxDeg", 0, 1.5, "ras.", issues)
    fixRange(c.wheel, "maxMzNm", 0, 3000, "wheel.", issues)
    fixRange(c.ref, "maxLatAcc", 5, 15, "ref.", issues)
    fixRange(c.corner, "aB0", 2, 9, "corner.", issues)
    fixRange(c.corner, "laMaxFrac", 0, 5, "corner.", issues)
    fixRange(c.corner, "respTauS", 0, 2, "corner.", issues)
    fixRange(c.corner, "laMinHoldS", 0, 1, "corner.", issues)
    fixRange(c.corner, "laHoldS", 0, 0.5, "corner.", issues)
    fixRange(c.corner, "laRateFrac", 0, 1, "corner.", issues)
    fixRange(c.corner, "ayMeasCap", 0, 5, "corner.", issues)
    fixRange(c.corner, "windFrac", 0, 1, "corner.", issues)
    fixRange(c.corner, "tqRecNmS", 0, 2000, "corner.", issues)
    fixRange(c.corner, "tqRecFrac", 0, 1, "corner.", issues)
    fixRange(c.corner, "tqRecDecMs2", 0, 2, "corner.", issues)
    fixRange(c.corner, "muBlankS", 0, 10, "corner.", issues)
    fixRange(c.corner, "muTcTf", 0, 1, "corner.", issues)
    fixRange(c.corner, "muQualS", 0, 2, "corner.", issues)
    fixRange(c.corner, "muSlip", 0.01, 0.5, "corner.", issues)
    fixRange(c.corner, "k1UnwindS", 0, 2, "corner.", issues)
    fixRange(c.corner, "k1UnwindFrac", 0, 1, "corner.", issues)
    fixRange(c.est, "massQuietS", 0, 10, "est.", issues)
    fixRange(c.det, "rawFrac", 0, 1, "det.", issues)
    fixRange(c.det, "gNoDecay", 0.5, 2, "det.", issues)
    fixRange(c.det, "lampOffHoldS", 0, 0.5, "det.", issues)
    fixRange(c.rec, "k2MsgS", 0, 2, "rec.", issues)
    fixRange(c.rec, "k2MsgFrac", 1, 3, "rec.", issues)
    fixRange(c.est, "massLambda", 0.5, 1, "est.", issues)
    fixRange(c.est, "massWinS", 0.2, 2, "est.", issues)
    fixRange(c.est, "massGapS", 0, 2.4, "est.", issues)
    fixRange(c.est, "massStepMs2", 0.2, 5, "est.", issues)
    fixRange(c.est, "massSteadyMs2", 0.05, 2, "est.", issues)
    fixRange(c.est, "massDvKmh", 0.5, 20, "est.", issues)
    fixRange(c.est, "vchLambda", 0.5, 1, "est.", issues)
    for i = 1, #INT_DET do fixInt(c.det, INT_DET[i], 1, 8, "det.", issues) end
    fixInt(c.det, "persistOccWinS", 5, CONST.OCC_MAX, "det.", issues)
    fixInt(c.pin, "subSample", 1, 8, "pin.", issues)
    fixInt(c.est, "massEvtMin", 1, 8, "est.", issues)
    fixInt(c.ysc, "windows", 1, 8, "ysc.", issues)
    local d0 = defaults()
    if not (c.det.fMinHz < c.det.fMaxHz) then
      c.det.fMinHz, c.det.fMaxHz = d0.det.fMinHz, d0.det.fMaxHz
      issues[#issues + 1] = "det.fMinHz/fMaxHz"
    end
    if not (c.filt.bpdHpHz < c.filt.bpdLpHz) then
      c.filt.bpdHpHz, c.filt.bpdLpHz = d0.filt.bpdHpHz, d0.filt.bpdLpHz
      issues[#issues + 1] = "filt.bpdHpHz/bpdLpHz"
    end
    if not (c.filt.rawMeanHz < c.filt.rawLpHz) then
      c.filt.rawMeanHz, c.filt.rawLpHz = d0.filt.rawMeanHz, d0.filt.rawLpHz
      issues[#issues + 1] = "filt.rawMeanHz/rawLpHz"
    end
    if not (c.wheel.lambda1 > c.wheel.lambda0) then
      c.wheel.lambda0, c.wheel.lambda1 = d0.wheel.lambda0, d0.wheel.lambda1
      issues[#issues + 1] = "wheel.lambda0/lambda1"
    end
    c.pin.torsionSign = (c.pin.torsionSign < 0) and -1 or 1
  end

  -- calibration = jbeam section over the defaults, then the range checks (problems listed in issues)
  function parseCfg(jd, issues)
    local c = parseInto(defaults(), jd, "", issues)
    validateCfg(c, issues)
    return c
  end
end

-- band-pass makeup gain 1 / (H(f0) * L(f0)) at f0 = sqrt(hp * lp)
local function makeupGain(hp, lp)
  local f0 = sqrt(hp * lp)
  local a, b = (f0 / hp) ^ 2, (f0 / lp) ^ 2
  return 1 / ((a / (1 + a)) * (1 / (1 + b)))
end

-- ---------------------------------------------------------------------------
-- state
-- ---------------------------------------------------------------------------

local function newRegistry()
  return {
    cmu = nil, yc = nil, ycPresent = false, compReg = false, compCount = 0, compOff = false, compErr = nil,
    off = false, objId = nil, whl = nil, rackScale = 1, m1 = 2000, halfTrack = 0.78,
    vch0 = 55, jbeamEnabled = true, gBp = 1, tmProv = false
  }
end

local function newFilters()
  return {
    d = newBp(), m = newBp(), D = newBp(), ay = newBp(), fy = newBp(),
    rl = newF(), rm = newF(), ch = newF(), cl = newF(), ax = newF(), ras = newF(), dd = newF(), ddr = newF(), rq = newF(),
    fx5 = newF(), fx1 = newF(), fx2 = newF(), fx2a = newF(), axm = newF(), crs = newF(), cay = newF(), axr = newF()
  }
end

local function newState()
  return {
    now = 0, lastResetT = -1e9, dscOn = true, enForce = nil, dm = nil, state = "OFF",
    -- inputs
    cmuOK = false, v = 0, kmh = 0, vPrev = nil, r = 0, ay = 0, ayS = 0, q = 0, u = 0, df = 0, slipOK = false,
    fImuX = 0, fImuY = 0, fImuZ = 0, L = 2.81,
    fwdGear = false, dscKey = "dsc", dscOff = false, brakeIn = 0, brakePed = 0, stockYc = false, absQ = false, tcs = false,
    tcBrake = false, launch = false, ccOn = false,
    rasPresent = false, rasState = "", rasSel = "", rasDir = 0, rasFault = "", rasLimit = 0,
    rasFf = 0, rasFb = 0, rasTs = 0, rasAngle = 0,
    trailerMode = false, trailerAttached = false, hookCoupled = false, coupled = false, coupledPrev = false,
    axKin = 0, aMeas = 0, pinValid = false,
    -- gate
    enabledEff = true, armedPre = false, armedBase = false, armedSway = false, armedCorner = false,
    swayLatch = false, cornerLatch = false, suspended = false, suspT = 0,
    -- reference and filters
    G = 0, deff = 0, deffPrev = nil, dDot = 0, drPrev = nil, drDot = 0, Dyaw = 0, eMeas = 0, eDet = 0, ecIn = 0, injOn = false,
    dwX = arr(RATE_N, 0), dwT = arr(RATE_N, 0), dwI = 0,
    yd = 0, raw = 0, ec = 0, yD = 0, yAy = 0, yFy = 0, amp = 0, ampM = 0, swayIdx = 0,
    -- validated sway (calm timer, stage-3 exit, persistence / watchdog evidence): peak hold of the running half-wave
    -- (ampR) and of the valid ones (ampV), last |BP_d| >= exitDegS in the running half-wave (loudT) and validated (swT);
    -- trailer-side half-waves of kFy*BP_d(Fy) (xF): sign, flip time, judged, peak, peak hold, loud time
    ampR = 0, ampV = 0, ampSw = 0, loudT = -1e9, swT = -1e9,
    xF = 0, schF = 0, tflF = 0, seenF = false, pkF = 0, ampRF = 0, loudF = -1e9,
    -- half-waves
    sch = 0, tflip = nil, seen = false, pk = 0, pkD = 0, pkAy = 0, pkFy = 0, rawPk = -1e9,
    hwHold = false, hwInj = false, hwRs = 0, steerHoldT = 0, supp = false, freq = 0,
    hwN = 0, hwA = arr(HIST, 0), hwF = arr(HIST, 0), hwQ = arr(HIST, 0), hwV = arr(HIST, false),
    hwR = arr(HIST, false), hwT = arr(HIST, 0), hwW = arr(HIST, 1), hwS = arr(HIST, 0),
    hwE = arr(HIST, 0), hwFy = arr(HIST, 0), kFy = 0, ampFy = 0, ecB = 0, ecBPrev = nil,
    flN = 0, fl = arr(HIST, 0),
    -- stage machine
    stage = 0, stageEff = 0, why = "", s3Why = "", n2c = 0, calmT = 0, lastExitT = -1e9,
    t3 = 0, vEvt = 0, drop = 10, activeT = 0,
    pe = arr(HIST, -1e9), peI = 0, occ = arr(CONST.OCC_MAX, 0), occB = 0,
    holdT = 0, holdVRec = 0,
    qaForce = nil, qaInj = nil, qaBudget = nil,
    -- corner (aRp response estimate, aR with the look-ahead aLA, laT time since the gate closed, tiT / tiPk turn-in
    -- time / peak rate, laOn look-ahead gate, laWin gate opened in this turn-in, laPk rate peak when its window was
    -- spent (0 = not spent), ayF measured ay, uwPk / uwT episode peak |df| / time unwound, wind how far the corner still
    -- builds 0..1, aPl the prediction without the look-ahead)
    K = 0, kExitT = 0, aPred = 0, aB = 0, aK2 = 0, lowMuT = 0, uwPk = 0, uwT = 0,
    aSS = 0, aRp = 0, aR = 0, aLA = 0, laT = 1e9, tiT = 0, tiPk = 0, laOn = false, laWin = false, laPk = 0, ayF = 0,
    aFloor = 0, wind = 0, aPl = 0,
    -- low-friction memory: blanking left (init / reset / coupling edge), evidence time, time since drive / brake
    -- slip, armed by evidence, last source, friction use and slips (getState)
    muBlankT = (CFG.corner and CFG.corner.muBlankS) or 0, muEvT = 0, muSdT = 1e9, muSbT = 1e9, muEv = false, muSrc = "",
    muUse = 0, muSlD = 0, muSlB = 0,
    -- mass estimator: time without a disturbance
    mQuietT = 0,
    -- actuators
    rasUsable = false, rsReq = 0, wTgtFL = 0, wTgtFR = 0, wCurFL = 0, wCurFR = 0, wIdle = true, wPause = false,
    rDynF = 0.33, rDyn4 = 0.33, sumBT = 0,
    aT = 0, aCut = 0, aCutI = 0, aCmd = 0, bI = 0, brakeCmd = 0, torqueMode = 0, freeze = false,
    -- HOLD cap recovery rate [Nm/s] for the torque coordinator, its deceleration gate (axKin low-passed tqRecLpHz)
    tqRec = 0, axR = 0,
    lampOut = false, lampEdge = false, lampPub = false, lampOffT = -1e9,
    -- guards
    rsDisabled = false, rq = 0, sgS1 = 0, sgS2 = 0, gCnt = 0,
    ysc = 0, yscT = 0, yscN = 0, yscA = 0, yscAg = 0, yscDis = 0,
    -- recommendation, messages, faults (k2T / k2Q: time the running K2 was held without the look-ahead / that K2 asks
    -- to slow down)
    vRec = 100, reduce = false, overT = 0, overLatch = false, stKPrev = false, msgReduceT = -1e9, drawbar = 0,
    k2T = 0, k2Q = false,
    swFault = false, physFault = false, swErr = nil, faultNoted = false,
    msgQ = {}, msgN = 0, msgLimited = false, hintT = 0, hintShown = false
  }
end

local function newPin()
  return {
    state = "OFF", fault = false, discFault = false, why = nil, faultMsg = false,
    nB = 0, bCid = {}, bCx = {}, bCy = {}, bCz = {},
    nT = 0, tCid = {}, tK = {}, tNx = {}, tNy = {}, tNz = {},
    HC = {}, nHC = 0, mh = 0,
    t0x = 0, t0y = 0, t0z = 0, tx = 0, ty = 0, tz = 0, tared = false, deformed = false,
    hookSet = {}, hookN = 0,
    sampling = false, window = false, sub = 0, subN = 2, useT = false,
    ax = 0, ay = 0, az = 0, n = 0, nanRun = 0, total = 0,
    Fx = 0, Fy = 0, Fz = 0, ux = 0, uy = 0, uz = 0, fresh = false,
    invalidT = 0, bigT = 0, contact = false, contactLogged = false,
    tareT = 0, tN = 0, tSx = 0, tSy = 0, tSz = 0, tareReq = false, tareDone = false, healthT = 0
  }
end

local function newEst()
  local md = CFG.decel and CFG.decel.massDefault or 1500
  local e = CFG.est
  local vch = clamp(REG.vch0 or 55, e and e.vchMin or 20, e and e.vchMax or 80)
  return {
    m2 = md, massQ = 0, ex = 0, acc = 0, sw = 0, swm = 0, nEv = 0, blk = 0, mLog = -1,
    rA = arr(CONST.MASS_RING, 0), rY = arr(CONST.MASS_RING, 0), rV = arr(CONST.MASS_RING, 0),
    rOk = arr(CONST.MASS_RING, false), rI = 0, rN = 0,
    mRing = arr(CONST.MASS_RING, md), mI = 0, mN = 0,
    fzS = newF(), tongueKg = 0, tongueState = "NONE", tgT = 0,
    vchT = vch, vTh = 1 / (vch * vch), vP = 1e-6, vN = 0, vwT = 0, vwN = 0, vSv = 0, vSd = 0, vSr = 0,
    has = false
  }
end

local function clearOcc()
  local s = ST
  for i = 1, CONST.OCC_MAX do s.occ[i] = 0 end
end

local function clearPersist()
  local s = ST
  for i = 1, HIST do s.pe[i] = -1e9 end
  clearOcc()
end

-- estimates, learned frequency, event hold, persistence (new coupling / dev API)
local function resetEst()
  local s = ST
  EST = newEst()
  s.flN = 0
  s.holdT, s.holdVRec = 0, 0
  clearPersist()
end

-- published estimates only with a trailer coupled and a working load pin
local function massKnown()
  return ST.coupled and not PIN.fault and EST.massQ >= CFG.est.massQPublish
end

local function tongueEff()
  if ST.coupled and not PIN.fault then return EST.tongueState end
  return "NONE"
end

-- low or negative tongue load (threshold scale, recommended speed, flag 32768)
local function lowTongue(ts)
  return ts == "LOW" or ts == "TAIL_HEAVY"
end

-- ---------------------------------------------------------------------------
-- discovery (init / initSecondStage)
-- ---------------------------------------------------------------------------

local pinDiscover, discover, readStats
do
  local function npos(cid)
    local n = v.data.nodes[cid]
    local p = n and n.pos
    if type(p) ~= "table" and type(p) ~= "cdata" and type(p) ~= "userdata" then error("node " .. tostring(cid) .. " without position") end
    return {num(p.x, 0), num(p.y, 0), num(p.z, 0)}
  end

  local function vsub(a, b)
    return {a[1] - b[1], a[2] - b[2], a[3] - b[3]}
  end

  local function vdot(a, b)
    return a[1] * b[1] + a[2] * b[2] + a[3] * b[3]
  end

  local function vcross(a, b)
    return {a[2] * b[3] - a[3] * b[2], a[3] * b[1] - a[1] * b[3], a[1] * b[2] - a[2] * b[1]}
  end

  local function vnorm(a)
    local l = sqrt(vdot(a, a))
    if not (l > 1e-9) then error("degenerate vector") end
    return {a[1] / l, a[2] / l, a[3] / l}
  end

  -- vehicle frame (fwd, right, up) from refNodes[0]
  local function frameOf()
    local rn = v.data.refNodes and v.data.refNodes[0]
    if type(rn) ~= "table" then error("no reference nodes") end
    local pr = npos(rn.ref)
    local fwd = vnorm(vsub(pr, npos(rn.back)))
    local up = vsub(npos(rn.up), pr)
    local k = vdot(up, fwd)
    up = vnorm({up[1] - k * fwd[1], up[2] - k * fwd[2], up[3] - k * fwd[3]})
    return fwd, vcross(fwd, up), up
  end

  -- hitch node set, couplers, carrier mass, boundary beams, crossing torsionbars
  function pinDiscover()
    local P, c = PIN, CFG.pin
    local nodes = v.data.nodes
    if type(nodes) ~= "table" then error("no nodes") end
    local pre = c.nodePrefix
    local np = #pre
    local H, nH, mh = {}, 0, 0
    for k, n in pairs(nodes) do
      if type(n) == "table" and type(n.name) == "string" and ssub(n.name, 1, np) == pre then
        local cid = n.cid or k
        H[cid] = true
        nH = nH + 1
        mh = mh + num(obj:getNodeMass(cid), 0)
        local tag = n.couplerTag or n.tag
        if (tag == "tow_hitch" or tag == "pintle" or tag == "gooseneck_hitch") and not n.couplerLock and not n.couplerWeld then
          P.HC[cid] = true
          P.nHC = P.nHC + 1
        end
      end
    end
    P.mh = mh
    -- default tare = carrier weight (with imuComp the specific force removes it)
    P.t0x, P.t0y, P.t0z = 0, 0, c.imuComp and 0 or mh * 9.81
    P.tx, P.ty, P.tz = P.t0x, P.t0y, P.t0z
    if nH == 0 then
      P.fault, P.discFault, P.why = true, true, "no hitch nodes"
      return
    end
    local fwd, right, up = frameOf()
    for k, b in pairs(v.data.beams or {}) do
      if type(b) == "table" then
        local in1, in2 = H[b.id1] == true, H[b.id2] == true
        if in1 ~= in2 then
          local ph, pb = npos(in1 and b.id1 or b.id2), npos(in1 and b.id2 or b.id1)
          local u = vnorm(vsub(pb, ph))
          local i = P.nB + 1
          P.nB = i
          P.bCid[i] = b.cid or k
          P.bCx[i], P.bCy[i], P.bCz[i] = vdot(u, fwd), vdot(u, right), vdot(u, up)
        end
      end
    end
    for k, tb in pairs(v.data.torsionbars or {}) do
      if type(tb) == "table" then
        local o1, o2, o3, o4 = not H[tb.id1], not H[tb.id2], not H[tb.id3], not H[tb.id4]
        local nOut = (o1 and 1 or 0) + (o2 and 1 or 0) + (o3 and 1 or 0) + (o4 and 1 or 0)
        if nOut == 1 and (o1 or o4) then
          local p2, p3 = npos(tb.id2), npos(tb.id3)
          local po = npos(o1 and tb.id1 or tb.id4)
          local ax = vsub(p3, p2)
          local cr = vcross(ax, vsub(po, p2))
          local d = sqrt(vdot(cr, cr)) / sqrt(vdot(ax, ax))
          local nv = vnorm(cr)
          local i = P.nT + 1
          P.nT = i
          P.tCid[i] = tb.cid or k
          P.tK[i] = num(tb.spring, 0) * c.torsionSign / max(d, 1e-3)
          P.tNx[i], P.tNy[i], P.tNz[i] = vdot(nv, fwd), vdot(nv, right), vdot(nv, up)
        end
      end
    end
    if P.nB == 0 then
      P.fault, P.discFault, P.why = true, true, "no carrier boundary beams"
    end
  end

  -- front rack (outLimit - inLimit) / 2 of the steering hydros vs the reference
  local function rackScaleOf()
    local hs = v.data.hydros
    local f = 0
    if type(hs) == "table" then
      for _, h in pairs(hs) do
        if type(h) == "table" then
          local src = h.inputSource
          if src == nil or src == "steering" or src == "steering_input" then
            local x = (num(h.outLimit, 1) - num(h.inLimit, 1)) * 0.5
            if x > f then f = x end
          end
        end
      end
    end
    if f > 0.05 and f < 0.5 and CFG.ref.frontRackFactorRef > 0 then return f / CFG.ref.frontRackFactorRef end
    return 1
  end

  local function wheelRefs()
    local w = {}
    local ws = wheels and wheels.wheels
    if type(ws) == "table" then
      for _, wd in pairs(ws) do
        local n = type(wd) == "table" and wd.name
        if n == "FL" or n == "FR" or n == "RL" or n == "RR" then w[n] = wd end
      end
    end
    if w.FL and w.FR and w.RL and w.RR then return w end
    return nil
  end

  local function wheelX(wd)
    local a, b = npos(wd.node1), npos(wd.node2)
    return 0.5 * (a[1] + b[1])
  end

  function discover()
    local g, c = REG, CFG
    local list = controller.getControllersByType("drivingDynamics/CMU")
    local cmu = (type(list) == "table" and list[1]) or controller.getController("CMU")
    local yc = cmu and cmu.getSupervisor and cmu.getSupervisor("yawControl")
    g.cmu, g.yc = cmu or nil, yc or nil
    g.ycPresent = (yc ~= nil and yc ~= false and yc.isActive == true)
    if g.ycPresent and not g.compReg then
      yc.registerComponent(COMP)
      g.compReg = true
      g.compCount = g.compCount + 1
    end
    g.objId = obj:getId()
    local okR, rs = pcall(rackScaleOf)
    g.rackScale = (okR and num(rs, 1)) or 1
    g.whl = wheelRefs()
    local m1 = 0
    for k, n in pairs(v.data.nodes or {}) do
      if type(n) == "table" then m1 = m1 + num(obj:getNodeMass(n.cid or k), 0) end
    end
    if m1 > 300 then g.m1 = m1 end
    local ht = c.wheel.halfTrackM
    if g.whl then
      local okW, x = pcall(function() return abs(wheelX(g.whl.FL) - wheelX(g.whl.FR)) * 0.5 end)
      if okW and type(x) == "number" and x > 0.4 and x < 1.2 then ht = x end
    end
    g.halfTrack = ht
  end

  -- characteristic speed at spawn (vehicleData), else the calibration default
  function readStats()
    local cmu = REG.cmu
    local vd = cmu and cmu.vehicleData
    local stt = vd and vd.vehicleStats
    local x = type(stt) == "table" and num(stt.characteristicSpeed, nil) or nil
    if x == nil or x < 5 or x > 200 then x = CFG.ref.vchDefault end
    REG.vch0 = x
    local e, c = EST, CFG.est
    if e.vN == 0 then
      local vch = clamp(x, c.vchMin, c.vchMax)
      e.vchT, e.vTh = vch, 1 / (vch * vch)
    end
  end
end

-- ---------------------------------------------------------------------------
-- yaw control component (lamp / escActive only in braking stages)
-- ---------------------------------------------------------------------------

do
  local function compAct()
    local s = ST
    return s.lampOut == true and s.dscOn == true and s.dscKey ~= "off" and not s.swFault and not s.physFault
  end

  -- yawControl.updateIsEnabled (every DSC switch); never throws
  COMP.setParameters = function(p)
    if type(p) == "table" and p["yawControl.isEnabled"] ~= nil then ST.dscOn = (p["yawControl.isEnabled"] == true) end
  end

  COMP.actAsYawControl = function(measuredYaw, expectedYaw, yawDifference, bodySlipAngle, frontSlipAngle, rearSlipAngle, dt)
    if REG.compOff then return false end
    local ok, res = pcall(compAct)
    if ok then return res == true end
    REG.compOff = true
    REG.compErr = tostring(res)
    warn("yaw control component disabled after an error: " .. tostring(res))
    return false
  end
end

-- ---------------------------------------------------------------------------
-- load pin
-- ---------------------------------------------------------------------------

-- physics rate (2 kHz), called every subSample-th step; allocation-free
local function pinPhys()
  local obj = obj -- one global lookup per call instead of one per beam
  local P = PIN
  local lx, ly, lz = 0, 0, 0
  local cid, cx, cy, cz = P.bCid, P.bCx, P.bCy, P.bCz
  for i = 1, P.nB do
    local st = obj:getBeamStress(cid[i])
    lx = lx - st * cx[i]
    ly = ly - st * cy[i]
    lz = lz - st * cz[i]
  end
  if P.useT then
    local tc, tk, nx, ny, nz = P.tCid, P.tK, P.tNx, P.tNy, P.tNz
    for i = 1, P.nT do
      local a = obj:getTorsionbarAngle(tc[i]) * tk[i]
      lx = lx + a * nx[i]
      ly = ly + a * ny[i]
      lz = lz + a * nz[i]
    end
  end
  if lx == lx and ly == ly and lz == lz and abs(lx) < huge and abs(ly) < huge and abs(lz) < huge then
    P.ax, P.ay, P.az = P.ax + lx, P.ay + ly, P.az + lz
    P.n = P.n + 1
    P.nanRun = 0
  else
    P.nanRun = P.nanRun + 1
  end
end

-- 1: boxcar mean of the last 10 ms, tare, carrier inertia, implausibility
local function pinFixed(dt)
  local P, c, s = PIN, CFG.pin, ST
  local n = P.n
  P.fresh = false
  if P.nanRun >= 3 then P.invalidT = c.implausHoldS end
  if P.sampling and n > 0 then
    local inv = 1 / n
    local fx, fy, fz = P.ax * inv, P.ay * inv, P.az * inv -- trailer force on the hitch assembly
    local mh = P.mh
    if c.imuComp then fx, fy, fz = fx + mh * s.fImuX, fy + mh * s.fImuY, fz + mh * s.fImuZ end
    local ux, uy, uz = -fx, fy, -fz
    local Fx = ux - P.tx - ((c.axComp and not c.imuComp) and mh * s.axKin or 0)
    local Fy = uy - P.ty
    local Fz = uz - P.tz
    local mag = sqrt(Fx * Fx + Fy * Fy + Fz * Fz)
    if mag == mag and mag < huge then
      P.ux, P.uy, P.uz = ux, uy, uz
      P.Fx, P.Fy, P.Fz = Fx, Fy, Fz
      P.fresh = true
      P.total = P.total + n
      if mag > c.maxN then
        P.bigT = P.bigT + dt
        if P.bigT >= c.implausS then P.invalidT = c.implausHoldS end
      else
        P.bigT = 0
      end
    end
  elseif not P.sampling then
    P.Fx, P.Fy, P.Fz, P.ux, P.uy, P.uz, P.bigT = 0, 0, 0, 0, 0, 0, 0
  end
  P.ax, P.ay, P.az, P.n = 0, 0, 0, 0
  if P.invalidT > 0 then P.invalidT = max(0, P.invalidT - dt) end
end

-- 1: one tare per standstill window without a trailer, after tareS in the window; tare() restarts it
local function pinTareStep(dt)
  local P, c = PIN, CFG.pin
  if not P.window then
    P.tareT, P.tN, P.tSx, P.tSy, P.tSz = 0, 0, 0, 0, 0
    P.contact, P.tareReq, P.tareDone = false, false, false
    return
  end
  if P.tareReq then
    P.tareT, P.tN, P.tSx, P.tSy, P.tSz, P.tareReq, P.tareDone = 0, 0, 0, 0, 0, false, false
  end
  if P.tareDone then return end
  if P.fresh then
    P.tSx, P.tSy, P.tSz = P.tSx + P.ux, P.tSy + P.uy, P.tSz + P.uz
    P.tN = P.tN + 1
  end
  P.tareT = P.tareT + dt
  if P.tareT >= c.tareS and P.tN > 0 then
    local mx, my, mz = P.tSx / P.tN, P.tSy / P.tN, P.tSz / P.tN
    local dx, dy, dz = mx - P.t0x, my - P.t0y, mz - P.t0z
    if sqrt(dx * dx + dy * dy + dz * dz) < c.contactN then
      P.tx, P.ty, P.tz = mx, my, mz
      P.tared, P.contact, P.contactLogged = true, false, false
    else
      P.contact = true
      if not P.contactLogged then
        P.contactLogged = true
        info(sfmt("load pin: tare rejected, %.0f N away from the carrier weight (something rests on the hitch) - previous tare kept", sqrt(dx * dx + dy * dy + dz * dz)))
      end
    end
    P.tareT, P.tN, P.tSx, P.tSy, P.tSz, P.tareDone = 0, 0, 0, 0, 0, true
  end
end

-- 1: broken / deformed boundary beams (every checkS)
local function pinHealth(dt)
  local P, c = PIN, CFG.pin
  P.healthT = P.healthT + dt
  if P.healthT < c.checkS then return end
  P.healthT = 0
  if P.fault or P.nB == 0 then return end
  local broken = false
  for i = 1, P.nB do
    if obj:beamIsBroken(P.bCid[i]) then broken = true break end
  end
  if not broken then
    for i = 1, P.nT do
      if obj:torsionbarIsBroken(P.tCid[i]) then broken = true break end
    end
  end
  if broken then
    P.fault, P.why = true, "boundary beam broken"
    if not P.faultMsg then
      P.faultMsg = true
      toast(TXT.pin)
      warn("load pin FAULT: hitch carrier beam broken - limited function until the vehicle reset")
    end
    return
  end
  if not P.deformed then
    for i = 1, P.nB do
      if num(obj:getBeamDeformation(P.bCid[i]), 0) > c.deformEps then
        P.deformed, P.tared = true, false
        P.tx, P.ty, P.tz = P.t0x, P.t0y, P.t0z
        info("load pin: hitch carrier deformed - tare cleared")
        break
      end
    end
  end
end

-- after readInputs: tare window and sampling for the next 10 ms, pin state, validity, drawbar display
local function pinStateStep(dt)
  local P, c, s = PIN, CFG.pin, ST
  local en = c.enabled and s.enabledEff and P.nB > 0 and not P.fault and not s.physFault and REG.tmProv
  local win = en and not s.coupled and s.kmh < c.tareMaxKmh and abs(s.r) < 0.02
  P.window = win
  P.sampling = en and (s.coupled or win)
  P.subN = c.subSample
  P.useT = c.useTorsion and P.nT > 0
  local st
  if P.fault then
    st = "FAULT"
  elseif not P.sampling then
    st = "OFF"
  elseif P.invalidT > 0 then
    st = "INVALID"
  elseif win then
    st = P.contact and "CONTACT" or (P.tared and "OK" or "TARE")
  elseif P.tared then
    st = "OK"
  else
    st = "UNTARED"
  end
  P.state = st
  s.pinValid = (st == "OK" or st == "UNTARED") and s.coupled and s.kmh >= c.validMinKmh
  s.drawbar = lp1(F.fx1, P.Fx, c.displayHz, dt)
end

-- ---------------------------------------------------------------------------
-- inputs and gating
-- ---------------------------------------------------------------------------

local readInputs
do
  -- DSE drive-mode key (driveModes cached); "off" | "dtc" | "dsc"
  local function dscKeyOf()
    local s = ST
    local dm = s.dm
    if not dm then
      dm = controller.getController("driveModes")
      if dm and dm.getCurrentDriveModeKey then s.dm = dm else dm = nil end
    end
    local key
    if dm then
      local ok, k = pcall(dm.getCurrentDriveModeKey)
      if ok and type(k) == "string" then key = k end
    end
    if not key then
      local lbl = electrics.values.etk854hybrid_dscMode
      key = (lbl == "OFF" and "off") or (lbl == "DTC" and "dtc") or "dsc"
    end
    if key == "off" or key == "dtc" then return key end
    return "dsc"
  end

  -- CMU usable: present, state normal, sensors present and active
  local function cmuSensors()
    local cmu = REG.cmu
    if not cmu or cmu.systemState ~= "normal" then return nil end
    local sh, vd, vs = cmu.sensorHub, cmu.vehicleData, cmu.virtualSensors
    if sh and vd and vs and sh.isActive == true and vd.isActive == true and vs.isActive == true then
      return sh, vd, vs
    end
    return nil
  end

  -- coupling edge: a re-coupling shortly after a reset keeps the estimates (no trailer id is read)
  local function coupleEdge()
    local s = ST
    if s.now - s.lastResetT < CFG.est.keepAfterResetS and EST.has then return end
    resetEst()
    s.hintShown, s.hintT = false, 0
  end

  -- sensorHub axes (measured in game, probe P8 2026-10-17; stock virtualSensors maps them the same way):
  --   accelerationX(Smooth)  lateral, + = towards the vehicle's LEFT (a left turn reads +), no gravity term
  --   accelerationY(Smooth)  longitudinal, + = decelerating (braking), - = speeding up, no gravity term
  --   accelerationZ(Smooth)  vertical incl. gravity, -9.81 upright at rest (CMU isOnRoof)
  --   yawAV(Smooth)          + = clockwise from above = right turn
  local function readCmu(sh, vd, vs)
    local s = ST
    local tw = vs.trustWorthiness
    local vt = vs.virtual
    local v = (num(tw and tw.virtualSpeed, 0) >= 1) and abs(num(vt.speed, 0)) or abs(num(vt.wheelSpeed, 0))
    s.r = num(sh.yawAVSmooth, 0)
    s.ayS = -num(sh.accelerationXSmooth, 0) -- lateral, + = right
    -- small trusted body slip angle = no slide (yaw-sign plausibility only)
    s.slipOK = num(tw and tw.bodySlipAngle, 0) >= 0.5 and abs(num(vt.bodySlipAngle, 0)) <= YSC_BETA
    s.q = num(sh.pitchAVSmooth, 0)
    if CFG.pin.imuComp then
      -- carrier acceleration in the pin frame (+ forward, + right, + up; Z with gravity: +9.81 upright at rest). X / Y are
      -- kinematic (no gravity term), so a slope or roll is not compensated
      s.fImuX, s.fImuY, s.fImuZ = -num(sh.accelerationYSmooth, 0), -num(sh.accelerationXSmooth, 0), -num(sh.accelerationZSmooth, -9.81)
    end
    local stt = vd.vehicleStats
    if type(stt) == "table" then
      local iw = num(stt.invWheelBase, 0)
      if iw > 0.15 and iw < 1 then s.L = 1 / iw end
    end
    return v
  end

  -- 2: inputs ("+" = right)
  function readInputs(dt)
    local s, ev, c = ST, electrics.values, CFG
    local sh, vd, vs = cmuSensors()
    s.cmuOK = sh ~= nil
    local v
    if sh then
      v = readCmu(sh, vd, vs)
    else
      v = abs(num(ev.airspeed, num(ev.wheelspeed, 0)))
      s.r, s.ayS, s.q, s.slipOK = 0, 0, 0, false
    end
    s.v, s.kmh = v, v * 3.6
    s.ay = abs(s.ayS)
    local u = clamp(num(ev.steering_input, 0), -1, 1)
    s.u = u
    local rf = c.ref
    s.df = rad(REG.rackScale * (rf.frontAngleLinDeg * u + rf.frontAngleCubDeg * u * u * u))
    -- rear-axle steering (read only)
    local rs, sel, flt = ev.etk854hybrid_rasState, ev.etk854hybrid_rasSel, ev.etk854hybrid_rasFault
    s.rasPresent = rs ~= nil
    s.rasState = type(rs) == "string" and rs or ""
    s.rasSel = type(sel) == "string" and sel or ""
    s.rasFault = type(flt) == "string" and flt or ""
    s.rasDir = num(ev.etk854hybrid_rasDirCheck, 0)
    s.rasLimit = floor(num(ev.etk854hybrid_rasLimit, 0))
    s.rasFf, s.rasFb = num(ev.etk854hybrid_rasFfDeg, 0), num(ev.etk854hybrid_rasFbDeg, 0)
    s.rasTs, s.rasAngle = num(ev.etk854hybrid_rasTsDeg, 0), num(ev.etk854hybrid_rasAngleDeg, 0)
    -- trailer
    local tm = ev.etk854hybrid_trailerMode
    s.trailerMode = flag(tm)
    -- a trailer-mode provider exists (selfLeveling / adaptiveSusp); latched over resets
    if tm ~= nil then REG.tmProv = true end
    s.trailerAttached = flag(ev.etk854hybrid_trailerAttached)
    s.hookCoupled = PIN.hookN > 0
    s.coupled = s.trailerAttached or s.hookCoupled
    -- gear, stability control, brakes
    local gear = ev.gear
    if type(gear) == "string" then
      s.fwdGear = gear ~= "" and gear ~= "P" and gear ~= "R" and gear ~= "N"
    else
      s.fwdGear = num(gear, 0) > 0
    end
    s.dscKey = dscKeyOf()
    s.dscOff = s.dscKey == "off" or s.dscOn == false
    s.brakeIn = num(ev.brake, 0)
    s.brakePed = num(ev.brake_input, 0)
    s.stockYc = flag(ev.isYCBrakeActive)
    s.absQ = flag(ev.etk854hybrid_absQualified)
    s.tcs = flag(ev.tcsActive)
    s.tcBrake = flag(ev.isTCBrakeActive)
    s.launch = ev.etk854hybrid_launchState == "ACTIVE"
    s.ccOn = flag(ev.cruiseControlActive)
    -- kinematic longitudinal acceleration (+ = speeding up)
    local vp = s.vPrev or v
    s.axKin = lp1(F.ax, (v - vp) / dt, AX_LP_HZ, dt)
    s.vPrev = v
    s.aMeas = -s.axKin
    s.axR = lp1(F.axr, s.axKin, c.corner.tqRecLpHz, dt)
    if s.coupled and not s.coupledPrev then coupleEdge() end
    -- the coupling jerk is no friction evidence (low-friction memory)
    if s.coupled ~= s.coupledPrev then s.muBlankT = c.corner.muBlankS end
    s.coupledPrev = s.coupled
  end
end

local function hwPeaksReset()
  local s = ST
  s.pk, s.pkD, s.pkAy, s.pkFy, s.rawPk = 0, 0, 0, 0, -1e9
  s.hwHold, s.hwInj, s.hwRs = false, false, 0
  s.ampR, s.loudT = 0, -1e9
end

-- effective switch: setEnabled override, else the jbeam value
local function enabledNow()
  local en = ST.enForce
  if en ~= nil then return en end
  return REG.jbeamEnabled
end

-- 3: arming
local function gateStep(dt)
  local s, g = ST, CFG.gate
  local en = enabledNow()
  s.enabledEff = en
  if s.suspended then
    s.suspT = s.suspT - dt
    if s.suspT <= 0 or s.kmh < g.stopKmh then s.suspended = false end
  end
  local pre = en and not s.swFault and not s.physFault and s.cmuOK and REG.ycPresent and not REG.off
    and s.trailerMode and not s.dscOff and s.fwdGear and not s.launch
  s.armedPre = pre
  s.armedBase = pre and not s.suspended
  if s.kmh >= g.swayOnKmh then s.swayLatch = true elseif s.kmh < g.swayOffKmh then s.swayLatch = false end
  if s.kmh >= g.cornerOnKmh then s.cornerLatch = true elseif s.kmh < g.cornerOffKmh then s.cornerLatch = false end
  local keyOK = s.dscKey == "dsc" or (s.dscKey == "dtc" and g.swayInDtc)
  local aw = s.armedBase and keyOK and s.swayLatch
  if aw and not s.armedSway then
    -- the first half-wave after arming is discarded
    s.sch, s.seen, s.tflip = 0, false, nil
    hwPeaksReset()
  elseif not aw and s.armedSway then
    s.freq = 0
  end
  s.armedSway = aw
  s.armedCorner = s.armedBase and s.dscKey == "dsc" and s.cornerLatch
end

local function stateName()
  local s = ST
  if not s.enabledEff then return "DISABLED" end
  if s.swFault or s.physFault then return "FAULT" end
  if not s.cmuOK or not REG.ycPresent or REG.off then return "UNAVAILABLE" end
  if not REG.tmProv then return "UNAVAILABLE" end
  if not s.trailerMode then return "OFF" end
  if s.dscOff then return "DSC_OFF" end
  if s.suspended then return "SUSPENDED" end
  if s.stageEff >= 1 or s.K >= 1 then return "ACTIVE" end
  if s.armedSway then return "READY" end
  return "STANDBY"
end

-- ---------------------------------------------------------------------------
-- reference and filters
-- ---------------------------------------------------------------------------

-- 4: own signed yaw reference, detection / control inputs, steering rate
local function refStep(dt)
  local s, c = ST, CFG.ref
  local v = s.v
  local dr = rad(lp1(F.ras, s.rasFf, 1 / (2 * pi * max(c.rasCreditTauS, 1e-3)), dt)) -- feed-forward share only
  local deff = s.df - dr
  local vch = EST.vchT
  local G = v / (s.L * (1 + v * v / (vch * vch)))
  s.G = G
  local lim = c.maxLatAcc / max(v, 1)
  s.eMeas = deg(s.r - clamp(G * deff, -lim, lim))
  local inj = s.qaInj
  if inj then
    inj.t = inj.t + dt
    if inj.t > inj.dur then
      s.qaInj = nil
      inj = nil
    end
  end
  if inj then
    local x = inj.amp * sin(2 * pi * inj.hz * inj.t)
    s.eDet, s.ecIn, s.injOn = x, inj.ctl * x, true
  else
    s.eDet, s.ecIn, s.injOn = s.eMeas, s.eMeas, false
  end
  -- steering rate over RATE_N steps (50 ms): steering_input is a GFX-rate staircase, a one-step difference
  -- jitters with the frame pattern; 50 ms spans whole frames at 20 / 60 fps for 25 ms of lag
  local wx, wt = s.dwX, s.dwT
  if s.deffPrev == nil then
    for j = 1, RATE_N do wx[j], wt[j] = deff, s.now end
  end
  local j = s.dwI % RATE_N + 1
  local x0, t0 = wx[j], wt[j]
  wx[j], wt[j], s.dwI = deff, s.now, j
  local rate = (s.now - t0 > 1e-6) and deg(deff - x0) / (s.now - t0) or 0
  s.dDot = lp1(F.dd, fin(rate), c.steerRateLpHz, dt)
  -- rear-steer credit rate: dDot + drDot = front road-wheel rate (corner look-ahead in NO DSC)
  s.drDot = lp1(F.ddr, fin(deg(dr - (s.drPrev or dr)) / dt), c.steerRateLpHz, dt)
  s.deffPrev, s.deff, s.drPrev = deff, deff, dr
  s.Dyaw = deg(G * deff)
end

-- 5: BP_d (detection, driver band, ay, lateral hitch force), raw consistency signal, BP_c, amplitude
local function filtStep(dt)
  local s, f = ST, CFG.filt
  local yd = bpd(F.d, s.eDet, dt)
  s.yd = yd
  s.raw = lp1(F.rl, s.eDet, f.rawLpHz, dt) - lp1(F.rm, s.eDet, f.rawMeanHz, dt)
  -- BP_c (wheel brake, published) and the share signal ec: BP_c advanced by the share's realisation lag
  -- (ras.signLagS), so the realised rear angle follows k * BP_c (else ~0.1 s / ~25 deg late at 0.67 Hz)
  local ecB = lp1(F.cl, hp1(F.ch, s.ecIn, f.bpcHpHz, dt), f.bpcLpHz, dt)
  s.ec = ecB + CFG.ras.signLagS * (ecB - (s.ecBPrev or ecB)) / dt
  s.ecB, s.ecBPrev = ecB, ecB
  local dec = exp(-dt / max(f.ampHoldTauS, 1e-3))
  s.ampM = max(abs(bpd(F.m, s.eMeas, dt)), s.ampM * dec)
  s.yD = bpd(F.D, s.Dyaw, dt)
  s.yAy = bpd(F.ay, s.ayS, dt)
  if s.pinValid then
    s.yFy = bpd(F.fy, PIN.Fy, dt)
  else
    s.yFy = 0
    bpReset(F.fy)
  end
  local ya = abs(yd)
  s.amp = max(ya, s.amp * dec)
  s.ampR = max(ya, s.ampR * dec) -- merged into ampV when the half-wave closes valid (hwClose)
  s.ampV = s.ampV * dec
  if ya >= CFG.det.exitDegS then s.loudT = s.now end
  local xf = (s.kFy > 0 and s.pinValid) and s.kFy * s.yFy or 0
  s.xF = xf
  xf = abs(xf)
  s.ampRF = max(xf, s.ampRF * dec)
  if xf > s.pkF then s.pkF = xf end
  if xf >= CFG.det.exitDegS then s.loudF = s.now end
  s.ampFy = max(abs(s.yFy), s.ampFy * dec) -- decays to 0 without a valid pin
  s.swayIdx = clamp(s.amp / max(CFG.det.a2DegS, 1e-3), 0, 2)
end

-- ---------------------------------------------------------------------------
-- half-waves and stage machine
-- ---------------------------------------------------------------------------

local function hwAt(k) -- k = 0 newest
  return (ST.hwN - 1 - k) % HIST + 1
end

-- threshold scale: reactivation window, low / negative tongue load
local function scNow()
  local s, d = ST, CFG.det
  local sc = 1
  if s.now - s.lastExitT < d.reactS then sc = d.reactScale end
  if lowTongue(tongueEff()) then sc = min(sc, d.lowTongueScale) end
  return sc
end

-- learned band (learnMinHw valid half-waves in stage >= 1), running event only: the sway frequency
-- changes with speed and in closed loop
local function freqBand()
  local s, d = ST, CFG.det
  local n = min(s.flN, HIST)
  if s.stage >= 1 and s.flN >= d.learnMinHw and n > 0 then
    local sum = 0
    for i = 1, n do sum = sum + s.fl[i] end
    local fb = sum / n
    return max(d.fMinHz, (1 - d.learnBand) * fb), min(d.fMaxHz, (1 + d.learnBand) * fb), fb
  end
  return d.fMinHz, d.fMaxHz, 0
end

-- trailer-side amplitude [deg/s] of the peak-hold hitch force (0 without)
local function ampTrl()
  local s = ST
  if s.kFy <= 0 or not s.pinValid then return 0 end
  return s.kFy * s.ampFy
end

-- limited function (sign guards, share effect): log + one toast until reset
local function limitedNotice(why)
  local s = ST
  warn(why)
  if not s.msgLimited then
    s.msgLimited = true
    toast(TXT.limited)
  end
end

-- persistence evidence: stage entries within persistWinS, occupancy seconds in the ring
local function persistCounts()
  local s, d = ST, CFG.det
  local cnt, occ = 0, 0
  for i = 1, HIST do
    if s.now - s.pe[i] <= d.persistWinS then cnt = cnt + 1 end
  end
  for i = 1, d.persistOccWinS do occ = occ + s.occ[i] end
  return cnt, occ
end

local hwStep, stageStep
do
  -- consecutive valid half-waves with A >= thr from the newest (weighted: sum of the weights)
  local function trailing(thr, weighted)
    local s = ST
    local n = min(s.hwN, HIST)
    local cnt = 0
    for k = 0, n - 1 do
      local i = (s.hwN - 1 - k) % HIST + 1
      if not s.hwV[i] or s.hwA[i] < thr then break end
      cnt = cnt + (weighted and s.hwW[i] or 1)
    end
    return cnt
  end

  -- trailer-side scale [deg/s per N]: yaw-error / hitch-force peak of the stage-entry half-wave (before any
  -- actuator acts), 0 = none; capped at a1DegS / fyMinN (a stage-1 sway carries >= fyMinN)
  local function kFyOf(i)
    local s, d = ST, CFG.det
    local fy = s.hwFy[i]
    if not s.pinValid or s.hwInj or not s.hwV[i] or fy < d.fyMinN or fy <= 0 then return 0 end
    return min(s.hwA[i] / fy, d.a1DegS / max(d.fyMinN, 1))
  end

  -- stage transition of the computed machine (logs one line)
  local function enter(st, why)
    local s, d = ST, CFG.det
    local from = s.stage
    if st == from then return end
    if from == 0 and st >= 1 and s.qaForce == nil then
      s.peI = s.peI % HIST + 1
      s.pe[s.peI] = s.now
    end
    if from == 0 then
      s.kFy = kFyOf(hwAt(0))
    elseif st == 0 then
      s.kFy = 0
    end
    if st == 3 then
      s.vEvt, s.t3, s.activeT = s.kmh, 0, 0
      s.drop = (why == "PERSIST" or why == "WATCHDOG" or why == "NODECAY") and d.persistDropKmh or d.s3DropKmh
      s.s3Why = why
      if why == "PERSIST" then clearPersist() end
      if s.coupled then
        -- event hold: recommendation below the event speed for evtHoldS
        local r = CFG.rec
        local base = (s.holdT > 0) and s.holdVRec or r.vRecKmh
        s.holdVRec = min(base, 5 * floor((s.vEvt - r.evtDropKmh) / 5))
        s.holdT = r.evtHoldS
      end
    elseif st == 2 then
      s.n2c = 0
    elseif st == 0 then
      s.flN = 0 -- learned band ends with the event
    end
    -- an episode (or its stage-3 part) ends: the occupancy restarts, the entry ring keeps counting
    if why == "CALM" or why == "S3END" or why == "STOP" or why == "SUSPEND" then clearOcc() end
    s.stage, s.why = st, why
    local i = hwAt(0)
    info(sfmt("stage %d->%d A=%.2f f=%.2f v=%.0f why=%s", from, st, s.hwA[i] or 0, s.hwF[i] or 0, s.kmh, why))
  end

  -- near-critical: last 3 valid, last 2 >= a2, E_n / E_(n-2) >= gNearCrit (stage machine amplitude hwE)
  local function nearCrit(a2)
    local s, d = ST, CFG.det
    if min(s.hwN, HIST) < 3 then return false end
    local i0, i1, i2 = hwAt(0), hwAt(1), hwAt(2)
    local E = s.hwE
    return s.hwV[i0] and s.hwV[i1] and s.hwV[i2] and E[i0] >= a2 and E[i1] >= a2
      and E[i0] / max(E[i2], 1e-6) >= d.gNearCrit
  end

  -- the newest n half-waves valid and confirmed by the hitch force (peak >= fyMinN)
  local function pinSway(n)
    local s, d = ST, CFG.det
    if not s.pinValid or min(s.hwN, HIST) < n then return false end
    for k = 0, n - 1 do
      local i = hwAt(k)
      if not s.hwV[i] or s.hwFy[i] < d.fyMinN then return false end
    end
    return true
  end

  -- stage 0 -> 1 NEAR1: pin-confirmed near-critical sway at the reduced threshold a1 * min(sc, reactScale)
  -- (a slowly decaying sway the full a1 would miss)
  local function nearLow(sc)
    local s, d = ST, CFG.det
    if not s.coupled or not pinSway(3) then return false end
    local thr = d.a1DegS * min(sc, d.reactScale)
    local i0, i1, i2 = hwAt(0), hwAt(1), hwAt(2)
    local A = s.hwA
    return A[i0] >= thr and A[i1] >= thr and A[i0] / max(A[i2], 1e-6) >= d.gNearCrit
  end

  -- stage 2 -> 3 NODECAY: last n3 stage-2 half-waves pin-confirmed, newest E_n >= a2 * sc and
  -- E_n / E_(n-2) >= gNoDecay. The stage-2 actuators load the hitch themselves, so a damped sway shrinks
  -- only slowly on the trailer side (~0.8 per period); a self-sustained one grows (> 1.3).
  local function noDecay(sc)
    local s, d = ST, CFG.det
    if not s.coupled or s.qaForce ~= nil or s.kFy <= 0 or s.n2c < d.n3 or not pinSway(d.n3) then return false end
    if min(s.hwN, HIST) < 3 then return false end
    local E = s.hwE
    local i0, i2 = hwAt(0), hwAt(2)
    return E[i0] >= d.a2DegS * sc and E[i0] / max(E[i2], 1e-6) >= d.gNoDecay
  end

  -- amplitude keeps growing although the rear steering damps (stage 1)
  local function growthRule()
    local s, d = ST, CFG.det
    if min(s.hwN, HIST) < 5 then return false end
    local rs = 0
    for k = 0, 2 do
      local i, j = hwAt(k), hwAt(k + 2)
      if not s.hwV[i] or s.hwE[i] < d.growth * s.hwE[j] then return false end
      if s.hwS[i] > rs then rs = s.hwS[i] end
    end
    return rs >= d.growthRsDeg
  end

  -- self-growing sway: the newest n half-waves valid, each > GROW_TOL above the previous, at a steady
  -- frequency (a noise-excited sway grows unevenly and shifts its period)
  local function growing(n)
    local s, d = ST, CFG.det
    if min(s.hwN, HIST) < n then return false end
    local fs = 0
    for k = 0, n - 1 do
      local i = hwAt(k)
      if not s.hwV[i] then return false end
      if k < n - 1 and s.hwA[i] < (1 + CONST.GROW_TOL) * s.hwA[hwAt(k + 1)] then return false end
      fs = fs + s.hwF[i]
    end
    local fm = fs / n
    for k = 0, n - 1 do
      if abs(s.hwF[hwAt(k)] - fm) > d.learnBand * fm then return false end
    end
    return true
  end

  -- stage entries on a half-wave close (armed, not suspended)
  local function onHw()
    local s, d = ST, CFG.det
    local sc = scNow()
    local i0 = hwAt(0)
    local A, valid = s.hwA[i0], s.hwV[i0]
    local a2 = d.a2DegS * sc
    local two = false
    if min(s.hwN, HIST) >= 2 then
      local i1 = hwAt(1)
      two = s.hwV[i1] and s.hwA[i1] >= a2
    end
    local s3ok = s.coupled or trailing(-huge, false) >= d.soloS3MinHw
    if valid and A >= d.a3BigDegS * sc and two and s3ok then
      enter(3, "BIG")
      return
    end
    -- the whole ring grew: above the critical speed, damping would only hide it -> slow down
    if s.coupled and s.qaForce == nil and s.stage <= 2 and growing(HIST) then
      info("sway grows by itself over " .. HIST .. " half-waves: above the critical speed")
      enter(3, "PERSIST")
      return
    end
    if s.stage == 0 then
      if trailing(d.a1DegS * sc, true) >= d.n1 then
        enter(1, "A1")
      elseif s.coupled and growing(d.learnMinHw) and trailing(d.a1DegS * min(sc, d.reactScale), true) >= d.n1 then
        -- growing sway: reduced threshold as in the reactivation window
        enter(1, "GROW")
      elseif nearLow(sc) then
        enter(1, "NEAR1")
      end
    end
    if s.stage <= 1 then
      if trailing(a2, false) >= d.n2 then
        enter(2, "A2")
      elseif nearCrit(a2) then
        enter(2, "NEAR")
      elseif s.stage == 1 and growthRule() then
        enter(2, "GROWTH")
      elseif s.stage == 1 and valid and A >= d.a3DegS * sc and s.hwW[i0] == 2 then
        -- a half-wave at the stage-3 amplitude confirmed as a heavy push by the hitch force: stage 1 is too late
        enter(2, "STRONG")
      end
    elseif s.stage == 2 and valid then
      s.n2c = s.n2c + 1
      if s.n2c >= d.n3 and A >= d.a3DegS * sc and s3ok then
        enter(3, "A3")
      elseif s3ok and noDecay(sc) then
        enter(3, "NODECAY")
      end
    end
  end

  -- share effect: the sway grows while the share acts (2 half-waves by >= growth or 1 by >= growth^2) ->
  -- share off (latched); catches a request-side sign error the realised-share check cannot see
  local function shareEffect(i)
    local s, d = ST, CFG.det
    if s.rsDisabled or s.stage < 1 or s.hwN < 2 then
      s.gCnt = 0
      return
    end
    local j = hwAt(1)
    local A, Ap = s.hwA[i], s.hwA[j]
    if s.hwV[i] and s.hwV[j] and s.hwS[i] >= d.growthRsDeg and A >= d.a2DegS * scNow() and A >= d.growth * Ap then
      s.gCnt = s.gCnt + 1
    else
      s.gCnt = 0
    end
    if s.gCnt >= 2 or (s.gCnt >= 1 and A >= d.growth * d.growth * Ap) then
      s.rsDisabled = true
      limitedNotice("steering share disabled: the sway grows while the share is applied")
    end
  end

  -- closes the running half-wave (record + stage entries)
  local function hwClose()
    local s, d = ST, CFG.det
    local T = s.now - s.tflip
    local f = (T > 1e-6) and 1 / (2 * T) or 0
    local A = s.pk
    local q = s.pkD / max(A, 1e-6)
    local rawOK = s.rawPk >= d.rawFrac * A
    local pinV = s.pinValid
    local early = pinV and s.pkFy >= d.fyEarlyN
    local qLim = early and d.qMaxPin or d.qMax
    local plaus = s.pkAy >= d.ayMin or (pinV and s.pkFy >= d.fyMinN) or s.hwInj
    local lo, hi = freqBand()
    -- in a stage a large half-wave below the band still counts (a growing sway slows down); only in-band
    -- half-waves are learned
    local inBand = f >= lo and f <= hi
    local slow = s.stage >= 1 and f > 0 and f < lo and A >= d.a2DegS * scNow()
    local valid = s.armedSway and not s.suspended and not s.hwHold and (inBand or slow) and q < qLim and plaus and rawOK
    s.supp = q >= qLim or s.hwHold or not rawOK
    s.hwN = s.hwN + 1
    local i = (s.hwN - 1) % HIST + 1
    s.hwA[i], s.hwF[i], s.hwQ[i], s.hwV[i], s.hwR[i], s.hwT[i] = A, f, q, valid and true or false, rawOK, s.now
    s.hwW[i] = (valid and early) and 2 or 1
    s.hwS[i] = s.hwRs
    s.hwFy[i] = pinV and s.pkFy or 0
    -- stage machine amplitude: in a stage the larger of yaw error and trailer side
    local E = A
    if s.stage >= 1 and s.kFy > 0 and pinV and not s.hwInj then E = max(A, s.kFy * s.pkFy) end
    s.hwE[i] = E
    if valid then
      -- validated sway: this half-wave's peak hold and loud time count for the exits (an invalid one never)
      s.ampV = max(s.ampV, s.ampR)
      if s.loudT > s.swT then s.swT = s.loudT end
    end
    if valid and inBand then
      s.freq = f
      if s.stage >= 1 then
        s.flN = s.flN + 1
        s.fl[(s.flN - 1) % HIST + 1] = f
      end
    end
    shareEffect(i)
    if s.armedSway and not s.suspended then onHw() end
  end

  -- 6: Schmitt trigger on BP_d(e_det), half-wave peaks
  function hwStep(dt)
    local s, d = ST, CFG.det
    if abs(s.dDot) > d.steerRateDegS then s.steerHoldT = d.steerHoldS else s.steerHoldT = max(0, s.steerHoldT - dt) end
    local x = s.yd
    local t = abs(x)
    if t > s.pk then s.pk = t end
    t = abs(s.yD)
    if t > s.pkD then s.pkD = t end
    t = abs(s.yAy)
    if t > s.pkAy then s.pkAy = t end
    if s.pinValid then
      t = abs(s.yFy)
      if t > s.pkFy then s.pkFy = t end
    end
    if s.sch ~= 0 then
      t = s.sch * s.raw
      if t > s.rawPk then s.rawPk = t end
    end
    if s.steerHoldT > 0 then s.hwHold = true end
    if s.injOn then s.hwInj = true end
    t = abs(s.rsReq)
    if t > s.hwRs then s.hwRs = t end
    local h = d.hystDegS
    local ns = (x > h) and 1 or ((x < -h) and -1 or s.sch)
    if ns ~= s.sch then
      if s.sch ~= 0 and s.tflip and s.seen then hwClose() end
      if s.sch ~= 0 then s.seen = true end
      s.sch, s.tflip = ns, s.now
      hwPeaksReset()
    end
    -- trailer side kFy*BP_d(Fy) (latched kFy, valid pin): own Schmitt trigger. A half-wave in the sway band (or a
    -- slow one >= a2*sc in a stage) validates its peak hold and loud time; a faster one (the hitch shaken by
    -- driver corrections) is dropped, as is the first one after the latch (no start flip).
    if s.kFy <= 0 or not s.pinValid then
      s.schF, s.seenF, s.pkF, s.ampRF, s.loudF = 0, false, 0, 0, -1e9
      return
    end
    x = s.xF
    ns = (x > h) and 1 or ((x < -h) and -1 or s.schF)
    if ns ~= s.schF then
      if s.schF ~= 0 and s.seenF then
        local T = s.now - s.tflF
        local f = (T > 1e-6) and 1 / (2 * T) or 0
        local lo, hi = freqBand()
        if (f >= lo and f <= hi) or (s.stage >= 1 and f > 0 and f < lo and s.pkF >= d.a2DegS * scNow()) then
          s.ampV = max(s.ampV, s.ampRF)
          if s.loudF > s.swT then s.swT = s.loudF end
        end
      end
      if s.schF ~= 0 then s.seenF = true end
      s.schF, s.tflF, s.pkF, s.ampRF, s.loudF = ns, s.now, 0, 0, -1e9
    end
  end

  -- occupancy ring: persistOccWinS buckets of 1 s with time in stage 1 / 2 (stage 3 brakes already)
  local function occStep(dt)
    local s = ST
    local W = CFG.det.persistOccWinS
    local b = floor(s.now) % W + 1
    if b ~= s.occB then
      s.occ[b] = 0
      s.occB = b
    end
    if (s.stage == 1 or s.stage == 2) and s.qaForce == nil then s.occ[b] = s.occ[b] + dt end
  end

  -- the running half-wave can still close valid: no steer hold, driver-band ratio so far below qLim
  local function hwCand()
    local s, d = ST, CFG.det
    if s.hwHold then return false end
    local qLim = (s.pinValid and s.pkFy >= d.fyEarlyN) and d.qMaxPin or d.qMax
    return s.pkD < qLim * max(s.pk, 1e-6)
  end

  -- sway amplitude that holds stage 3: peak hold of the validated half-waves and of the running ones while they
  -- can still be valid (yaw-error candidate, trailer side)
  local function swayAmp()
    local s = ST
    local a = max(s.ampV, s.ampRF)
    if s.ampR > a and hwCand() then a = s.ampR end
    return a
  end

  -- sway evidence (persistence, watchdog; validated only): peak hold of the validated half-waves, or the newest
  -- valid yaw-error half-wave within one period of its close (between peaks the 1 s peak hold falls to ~0.6 x)
  local function swayEvidence()
    local s = ST
    local e = s.ampV
    if s.hwN > 0 then
      local i = hwAt(0)
      local f = s.hwF[i]
      if s.hwV[i] and f > 0 and s.now - s.hwT[i] <= 1 / f then e = max(e, s.hwE[i]) end
    end
    return e
  end

  -- persistence (coupled only, stage 1 / 2, not while a stage is forced, only while sway >= a2 is present)
  local function persistStep()
    local s, d = ST, CFG.det
    if not s.coupled or s.qaForce ~= nil or (s.stage ~= 1 and s.stage ~= 2) then return end
    local cnt, occ = persistCounts()
    if (cnt >= d.persistN or occ >= d.persistOccS) and swayEvidence() >= d.a2DegS * scNow() then enter(3, "PERSIST") end
  end

  -- watchdog: 20 s in stage >= 2 -> stage 3 while sway is present, else suspend
  local function watchdogStep(dt)
    local s, d = ST, CFG.det
    if s.stage < 2 then
      s.activeT = 0
      return
    end
    s.activeT = s.activeT + dt
    if s.activeT <= d.maxActiveS then return end
    if s.coupled and swayEvidence() >= d.a2DegS * scNow() then
      enter(3, "WATCHDOG")
    else
      enter(0, "SUSPEND")
      s.suspended, s.suspT, s.activeT = true, d.suspendS, 0
      warn(sfmt("watchdog: %.0f s in a braking stage without sway evidence - suspended for %.0f s", d.maxActiveS, d.suspendS))
    end
  end

  -- 7: calm exits, stage-3 hold, persistence, watchdog, stop; forced stage
  function stageStep(dt)
    local s, d = ST, CFG.det
    -- calm = time since the last validated sway >= exitDegS: a valid yaw-error or trailer-side half-wave, or a
    -- running one that can still be valid; driver corrections (invalid half-waves) do not count
    local tq = max(s.swT, s.loudF)
    if s.loudT > tq and hwCand() then tq = s.loudT end
    s.calmT = s.now - tq
    s.ampSw = swayAmp()
    occStep(dt)
    if not s.armedSway or s.kmh < CFG.gate.stopKmh then
      if s.stage ~= 0 then enter(0, "STOP") end
      s.activeT = 0
    else
      if (s.stage == 1 or s.stage == 2) and s.calmT >= d.exitS then
        enter(0, "CALM")
        s.lastExitT = s.now
      end
      if s.stage == 3 then
        s.t3 = s.t3 + dt
        if (s.ampSw < d.s3ExitDegS and s.kmh <= s.vEvt - s.drop) or s.t3 >= d.s3MaxS then
          if s.calmT >= d.exitS then
            enter(0, "S3END")
            s.lastExitT = s.now
          else
            enter(2, "S3END")
          end
        end
      end
      persistStep()
      watchdogStep(dt)
    end
    local fs = s.qaForce
    if fs and s.armedSway then s.stageEff = max(s.stage, fs) else s.stageEff = s.stage end
  end
end

-- ---------------------------------------------------------------------------
-- corner pre-braking (DSC only)
-- ---------------------------------------------------------------------------

-- 8a: ay prediction (every step, so settled on arming). The real response lags the steady state
-- aSS = v * G * d (at most ref.maxLatAcc), which alone would brake on pulses, wiggles and slaloms:
--   aRp         first-order response estimate of aSS (respTauS), signed
--   look-ahead  d + tpS * rate (capped at laMaxFrac * |d|, at most ref.maxLatAcc) lifts aR above aRp while the gate
--               is open, a sustained one-way turn-in: away from centre for >= laMinHoldS, rate still >= laRateFrac *
--               this turn-in's peak (a slalom, lane change or correction slows or reverses the wheel first); aR is
--               aRp again laHoldS after the gate closes (the hold bridges the frame-rate ripple of the rate). laOnce:
--               one window per turn-in once it raised the corner state - when the hold after its gate closed has run
--               out and K >= 1 (tested in every step with the gate closed, also for a K1 reached later through aRp),
--               the window is spent (laPk = the turn-in's peak rate then) and the gate re-opens in the same turn-in
--               only on a new peak above laPk (a rate dip and recovery in a long wind-on lifted the lagging prediction
--               again: K0 -> K2 / K2 -> K1 -> K2 jabs in game, 2026-10-17 (g/h)); a window that raised nothing is not
--               spent
--   floor       measured ay, at most ayMeasCap * |aSS| (near sway resonance it can exceed aSS; a heavy trailer holds a
--               settled bend at ~1.8-2.0 x aSS, 2026-10-17 (k); the cap keeps an ay the steering does not explain, a gust in
--               a held bend, out of K2 below |aSS| = aB / ayMeasCap)
local function predStep(dt, d, rate)
  local s, c = ST, CFG.corner
  local vG = s.v * s.G
  local aMax = CFG.ref.maxLatAcc
  local aSS = clamp(fin(vG * d), -aMax, aMax)
  s.aSS = aSS
  local aRp = aSS
  if c.respTauS > 0 then aRp = fin(s.aRp + (aSS - s.aRp) * (1 - exp(-dt / c.respTauS))) end
  s.aRp = aRp
  local rs = lp1(F.crs, rate, c.laRateLpHz, dt)
  local away = rate * d > 0 and rs * d > 0
  if away then
    s.tiT = s.tiT + dt
    if abs(rs) > s.tiPk then s.tiPk = abs(rs) end
  else
    s.tiT, s.tiPk, s.laWin, s.laPk = 0, 0, false, 0
  end
  local gate = away and s.tiT >= c.laMinHoldS and abs(rs) >= c.laRateFrac * s.tiPk and (s.laPk <= 0 or abs(rs) > s.laPk)
  s.laOn = gate
  if gate then
    local cap = c.laMaxFrac * abs(d)
    s.aLA, s.laT = clamp(fin(vG * (d + clamp(c.tpS * rad(rate), -cap, cap))), -aMax, aMax), 0
    s.laWin, s.laPk = true, 0
  else
    if s.laT < c.laHoldS then s.laT = s.laT + dt end
    if c.laOnce and s.laWin and s.laPk <= 0 and s.laT >= c.laHoldS and s.K >= 1 then s.laPk = s.tiPk end
  end
  local aR = aRp
  if (gate or s.laT < c.laHoldS) and abs(s.aLA) > abs(aR) then aR = s.aLA end
  s.aR = aR
  local ayF = lp1(F.cay, s.ay, c.ayMeasLpHz, dt)
  s.ayF = ayF
  local fl = min(ayF, c.ayMeasCap * abs(aSS))
  s.aFloor = fl
  return max(abs(aR), fl)
end

-- 8: predicted lateral acceleration against the budget. The model's lead over the measured floor (response, look-ahead)
-- counts only while the corner still builds: w = (|aSS| - |aRp|) / (windFrac x aB), 0..1 (the steady state of the current
-- angle runs ahead of the response: a wind-on or a rising speed); once the bend has settled (w 0, also while the speed
-- falls) every K transition follows the measured lateral acceleration (2026-10-17 (j); windFrac 0 or ayMeasCap 0 = the
-- model always counts). K1 -> K0 not while the corner builds (w 1) with the steady state |aSS| at or above the K1 exit
-- level (k1ExitFrac x aB, 2026-10-17 (g/h)); then after aPred < k1ExitFrac x aB for k1ExitS, or early when the driver
-- unwinds: |df| below k1UnwindFrac x its peak of this episode for k1UnwindS (front angle: the rear-steer credit shrinks
-- deff in a held bend), |aSS| below the K1 exit level and aPred below the K1 entry level (only response lag left;
-- re-entry needs aPred > k1Frac x aB again)
local function cornerStep(dt)
  local s, c = ST, CFG.corner
  -- RAS ON NO DSC centres its feed-forward when the K2 lamp comes on: predict with the front road-wheel
  -- angle / rate (with the share's credit K2 would come late)
  local d, rate = s.deff, s.dDot
  if s.rasPresent and s.rasSel == "NODSC" then d, rate = s.df, s.dDot + s.drDot end
  local aPred = predStep(dt, d, rate)
  if not s.armedCorner then
    if s.K ~= 0 then info("corner K" .. s.K .. "->K0 (not armed)") end
    s.K, s.kExitT, s.aPred, s.aB, s.aK2, s.uwPk, s.uwT, s.wind, s.aPl = 0, 0, 0, 0, 0, 0, 0, 0, 0
    return
  end
  local fm = 1
  if s.coupled then
    if massKnown() then
      fm = lin(EST.m2 / max(REG.m1, 1), c.massRatioLo, c.massRatioHi, 1.0, c.massFactorMin)
    else
      fm = c.massFactorUnknown
    end
  end
  local fmu = (s.lowMuT > 0) and c.muLow or 1
  local aB = (s.qaBudget or c.aB0) * fm * fmu * (s.coupled and 1 or c.noCouplerScale)
  -- the corner still builds by windFrac x aB or more: w 1; settled: w 0 (the measured floor alone; without a floor,
  -- ayMeasCap 0, the model keeps counting)
  local wf = c.windFrac * aB
  local gap = abs(s.aSS) - abs(s.aRp)
  local w = (wf > 0 and c.ayMeasCap > 0) and clamp(gap / wf, 0, 1) or 1
  local fl = s.aFloor
  aPred = fl + w * max(0, aPred - fl)
  s.wind, s.aPl = w, fl + w * max(0, abs(s.aRp) - fl)
  local K = s.K
  -- driver unwinds: front road-wheel angle below k1UnwindFrac x its peak of this episode (reset in K0)
  local fa = abs(s.df)
  if K == 0 or fa > s.uwPk then s.uwPk = fa end
  if fa < c.k1UnwindFrac * s.uwPk then s.uwT = s.uwT + dt else s.uwT = 0 end
  if K == 0 and aPred > c.k1Frac * aB then K, s.kExitT = 1, 0 end
  if K == 1 and aPred > aB then K = 2 end
  if K == 2 and aPred < c.k2ExitFrac * aB then K, s.kExitT = 1, 0 end
  local why = ""
  if K == 1 then
    local hiSS = abs(s.aSS) >= c.k1ExitFrac * aB
    if aPred < c.k1ExitFrac * aB and not (hiSS and w >= 1) then
      s.kExitT = s.kExitT + dt
      if s.kExitT >= c.k1ExitS then K = 0 end
    else
      s.kExitT = 0
    end
    if K == 1 and not hiSS and s.uwT > 0 and s.uwT >= c.k1UnwindS and aPred < c.k1Frac * aB then
      K, why = 0, " unwind"
    end
  end
  if K ~= s.K then info(sfmt("corner K%d->K%d aPred=%.2f aB=%.2f v=%.0f%s", s.K, K, aPred, aB, s.kmh, why)) end
  s.K, s.aPred, s.aB = K, aPred, fin(aB)
  s.aK2 = (K == 2) and min(c.k2Max, c.k2Base + c.k2Slope * (aPred - aB)) or 0
end

-- ---------------------------------------------------------------------------
-- estimators
-- ---------------------------------------------------------------------------

local estStep
do
  -- trailer mass from acceleration steps: m_i = (Fx after - Fx before) / (a after - a before) between two quasi-steady
  -- windows of massWinS, massGapS apart (the gap takes the step: torque ramp, pitch, drawbar ringing) at nearly the same
  -- speed, so the trailer resistance (rolling, drag, grade; it rises with speed while a smooth acceleration falls)
  -- cancels. Steps are averaged with weight da^2 and the forgetting massLambda per step; ex = sum of da^2.
  -- Ring i = newest sample: before = the oldest nW samples of the span, after = the newest nW
  local function stepEval(e, c, i, n, nW)
    local sa, sa2, sy, sv, ta, ta2, ty, tv = 0, 0, 0, 0, 0, 0, 0, 0
    for k = 1, nW do
      local jb, ja = (i + k - 1) % n + 1, (i - k) % n + 1
      if not (e.rOk[jb] and e.rOk[ja]) then return end
      local xb, xa = e.rA[jb], e.rA[ja]
      sa, sa2, sy, sv = sa + xb, sa2 + xb * xb, sy + e.rY[jb], sv + e.rV[jb]
      ta, ta2, ty, tv = ta + xa, ta2 + xa * xa, ty + e.rY[ja], tv + e.rV[ja]
    end
    local mb, ma = sa / nW, ta / nW
    local dA, sd2 = ma - mb, c.massSteadyMs2 * c.massSteadyMs2
    if abs(dA) < c.massStepMs2 or sa2 / nW - mb * mb > sd2 or ta2 / nW - ma * ma > sd2
        or abs(tv - sv) / nW > c.massDvKmh then
      return
    end
    local mi = (ty - sy) / nW / dA
    if not (mi >= c.massMin and mi <= c.massMax) then return end
    local w, lam = dA * dA, (e.nEv > 0) and c.massLambda or 0
    e.sw, e.swm = lam * e.sw + w, lam * e.swm + w * mi
    e.m2 = clamp(e.swm / e.sw, c.massMin, c.massMax)
    e.ex, e.nEv, e.blk = e.ex + w, e.nEv + 1, n - 1
  end

  local function massStep(dt)
    local s, e, c = ST, EST, CFG.est
    local y = lp1(F.fx2, lp1(F.fx2a, PIN.Fx, AX_LP_HZ, dt), c.massLpHz, dt)
    local a = lp1(F.axm, s.axKin, c.massLpHz, dt)
    -- disturbances: trailer braking via the brake channel (pedal, one-pedal, TSA), corner state or stage,
    -- sway (pulls the drawbar at 2x its frequency, biases low); a step needs massQuietS before its first window
    -- and none up to its end
    local quiet = s.brakeIn < 0.01 and s.brakePed < 0.01 and s.brakeCmd <= 0 and s.K == 0 and s.stageEff == 0
      and s.amp < CFG.det.exitDegS and s.ampFy < CFG.det.fyMinN
    if quiet then s.mQuietT = s.mQuietT + dt else s.mQuietT = 0 end
    e.acc = e.acc + dt
    if e.acc < 1 / c.massHz - 1e-9 then return end
    e.acc = 0
    local nW = clamp(floor(c.massWinS * c.massHz + 0.5), 2, 20)
    local n = 2 * nW + clamp(floor(c.massGapS * c.massHz + 0.5), 0, 24)
    local i = e.rI % n + 1
    e.rI, e.rN = i, min(e.rN + 1, n)
    -- window samples: straight, no pitching, valid pin (the gap may carry steering corrections and the step's pitch)
    e.rA[i], e.rY[i], e.rV[i] = a, y, s.kmh
    e.rOk[i] = s.pinValid and s.kmh > 10 and abs(s.u) < 0.03 and abs(s.r) < 0.05 and abs(s.q) < 0.02
    if e.blk > 0 then
      e.blk = e.blk - 1
    elseif e.rN >= n and s.mQuietT >= c.massQuietS + n / c.massHz then
      stepEval(e, c, i, n, nW)
    end
    local nH = clamp(floor(3 * c.massHz + 0.5), 1, CONST.MASS_RING)
    e.mI = e.mI % nH + 1
    local old = e.mRing[e.mI]
    e.mRing[e.mI] = e.m2
    e.mN = min(e.mN + 1, nH)
    local stable = e.mN >= nH and abs(e.m2 - old) / max(e.m2, 50) < 0.05
    local q = min(1, e.ex / max(c.massExcMin, 1e-6)) * (stable and 1 or 0.5)
    -- fewer than massEvtMin steps: below the publication threshold (still learning)
    if e.nEv < c.massEvtMin then q = min(q, 0.5 * c.massQPublish) end
    e.massQ = q
    if q > 0 then e.has = true end
    -- one log line at the first publication and after every change by more than 10 %
    if massKnown() and (e.mLog < 0 or abs(e.m2 - e.mLog) > 0.1 * e.mLog) then
      info(sfmt("trailer mass %.0f kg (%d steps, quality %.2f)", e.m2, e.nEv, q))
      e.mLog = e.m2
    end
  end

  -- signed tongue load from the vertical pin force (quiet driving or standstill)
  local function tongueStep(dt)
    local s, e, c, P = ST, EST, CFG.est, PIN
    local pinOK = P.state == "OK" or (P.state == "UNTARED" and not P.deformed)
    local quiet = s.kmh < 0.5 or (abs(s.axKin) < 0.5 and s.ay < 0.5 and abs(s.q) < 0.02)
    if s.coupled and pinOK and quiet then
      lp1(e.fzS, P.Fz, c.tongueLpHz, dt)
      e.tgT = e.tgT + dt
    end
    if e.tgT >= 1 then
      local kg = e.fzS.y / 9.81
      e.tongueKg = kg
      local thr = max(c.tongueLowKg, massKnown() and c.tongueLowFrac * e.m2 or 0)
      e.tongueState = (kg < 0) and "TAIL_HEAVY" or ((kg < thr) and "LOW" or "OK")
      e.has = true
    else
      e.tongueKg, e.tongueState = 0, "NONE"
    end
  end

  -- characteristic speed of the (combination) yaw gain, windowed scalar RLS
  local function vchStep(dt)
    local s, e, c = ST, EST, CFG.est
    local ok = abs(s.deff) >= RAD05 and abs(s.r) >= RAD2 and sgn(s.r) == sgn(s.deff) and s.ay >= 1 and s.ay <= 4
      and abs(s.dDot) < 2 and s.kmh >= 40 and s.brakeIn < 0.01 and abs(s.axKin) < 1 and s.stageEff == 0 and s.K == 0
    if not ok then
      e.vwT, e.vwN, e.vSv, e.vSd, e.vSr = 0, 0, 0, 0, 0
      return
    end
    e.vwT, e.vwN = e.vwT + dt, e.vwN + 1
    e.vSv, e.vSd, e.vSr = e.vSv + s.v, e.vSd + s.deff, e.vSr + s.r
    if e.vwT < c.vchWindowS then return end
    local n = e.vwN
    local vb, db, rb = e.vSv / n, e.vSd / n, e.vSr / n
    e.vwT, e.vwN, e.vSv, e.vSd, e.vSr = 0, 0, 0, 0, 0
    if abs(rb) < 1e-6 then return end
    local y = vb * db / (rb * s.L) - 1
    local phi = vb * vb
    local lam = c.vchLambda
    local P = e.vP
    local k = P * phi / (lam + phi * P * phi)
    local th = e.vTh + k * (y - phi * e.vTh)
    local nP = (P - k * phi * P) / lam
    if th == th and nP == nP and abs(th) < 1 then
      e.vTh, e.vP, e.vN, e.has = th, nP, e.vN + 1, true
    end
    local tmin = 1 / (c.vchMax * c.vchMax)
    e.vchT = (e.vTh > tmin) and clamp(1 / sqrt(e.vTh), c.vchMin, c.vchMax) or c.vchMax
  end

  -- slip of one wheel against its path speed ve (+ drive, - brake), 0 for a skipped wheel
  local function slipOf(wd, ve, inv, skip)
    if skip or not wd then return 0 end
    return (abs(num(wd.wheelSpeed, ve)) - ve) * inv
  end

  -- low-friction memory: lowMuT = muLowS (corner budget x muLow) only on evidence of real low friction that
  -- holds for muQualS (the acceleration lags the wheel slip: at a brake onset on dry road ABS acts before the car
  -- decelerates), at >= muMinKmh and outside muBlankS after init / reset / a coupling edge:
  --   TC   e-machine / ICE throttle factor below muTcTf (gaugeData channels = the device factor, stock TC or YC
  --        trim; the cluster lamp's threshold; without those channels the raw tcsActive) or a TC brake stage, with
  --        drive slip >= muSlip seen within muQualS (the slip makes it friction evidence)
  --   ABS  wheels.lua ABS (driver / TSA brake > 0.1) or the VCU's qualified ABS, brake slip >= muSlip
  -- and friction use sqrt(aCar^2 + ay^2) < muEventMs2. aCar = the car's own tyre force per car mass: the load
  -- pin adds the trailer's share (axKin + Fx / m1); without a valid pin the combination acceleration is scaled
  -- by (m1 + m2) / m1. Slip per wheel against its yaw-corrected path speed; a front wheel the TSA brakes is
  -- left out. One log line per arming (rising edge of the qualified evidence).
  local function muStep(dt)
    local s, c, ev, w = ST, CFG.corner, electrics.values, REG.whl
    local trim
    local tfM, tfI = ev.etk854hybrid_dseMotorTf, ev.etk854hybrid_dseIceTf
    if tfM ~= nil or tfI ~= nil then
      trim = min(num(tfM, 1), num(tfI, 1)) < c.muTcTf
    else
      trim = s.tcs
    end
    local sd, sb, absOn = 0, 0, flag(ev.absActive) or s.absQ
    if w then
      local v = s.v
      local inv, yr = 1 / max(v, 1), s.r * REG.halfTrack
      local a1 = slipOf(w.FL, v + yr, inv, s.wCurFL > 0)
      local a2 = slipOf(w.FR, v - yr, inv, s.wCurFR > 0)
      local a3 = slipOf(w.RL, v + yr, inv, false)
      local a4 = slipOf(w.RR, v - yr, inv, false)
      sd, sb = max(a1, a2, a3, a4), -min(a1, a2, a3, a4)
      absOn = absOn or w.FL.absActive == true or w.FR.absActive == true or w.RL.absActive == true or w.RR.absActive == true
    end
    s.muSlD, s.muSlB = fin(sd), fin(sb)
    if sd >= c.muSlip then s.muSdT = 0 else s.muSdT = s.muSdT + dt end
    if sb >= c.muSlip then s.muSbT = 0 else s.muSbT = s.muSbT + dt end
    local m1 = REG.m1
    local aCar = s.axKin
    if s.pinValid then
      aCar = aCar + F.fx2a.y / m1
    elseif s.coupled then
      aCar = aCar * (m1 + (massKnown() and EST.m2 or CFG.decel.massDefault)) / m1
    end
    local use = fin(sqrt(aCar * aCar + s.ay * s.ay))
    s.muUse = use
    if s.muBlankT > 0 then s.muBlankT = max(0, s.muBlankT - dt) end
    local cand
    if s.kmh >= c.muMinKmh and s.muBlankT <= 0 and use < c.muEventMs2 then
      if (trim or s.tcBrake) and s.muSdT <= c.muQualS then
        cand = s.tcBrake and "TC brake" or "TC"
      elseif absOn and s.brakeIn > 0.1 and s.muSbT <= c.muQualS then
        cand = "ABS"
      end
    end
    if cand then s.muEvT = s.muEvT + dt else s.muEvT = 0 end
    local ok = cand ~= nil and s.muEvT >= c.muQualS
    if ok then
      if not s.muEv then
        info(sfmt("low friction (%s): corner budget x%.2f for %.0f s (was %.0f s left); %.0f km/h aMeas %.2f aCar %.2f ay %.2f "
          .. "slip %.2f/%.2f throttle %.2f brake %.2f", cand, c.muLow, c.muLowS, s.lowMuT, s.kmh, s.aMeas, aCar, s.ay,
          sd, sb, num(ev.throttle, 0), s.brakeIn))
      end
      s.lowMuT, s.muSrc = c.muLowS, cand
    elseif s.lowMuT > 0 then
      s.lowMuT = max(0, s.lowMuT - dt)
    end
    s.muEv = ok
  end

  -- 9: estimators + low-friction memory
  function estStep(dt)
    massStep(dt)
    muStep(dt)
    tongueStep(dt)
    vchStep(dt)
  end
end

-- ---------------------------------------------------------------------------
-- actuator requests
-- ---------------------------------------------------------------------------

local actStep
do
  -- live wheel data (dynamic radius, brake torque)
  local function wheelInfo()
    local s, w = ST, REG.whl
    if not w then
      s.rDynF, s.rDyn4, s.sumBT = 0.33, 0.33, 0
      return
    end
    local a, b = num(w.FL.dynamicRadius, 0.33), num(w.FR.dynamicRadius, 0.33)
    local c, d = num(w.RL.dynamicRadius, 0.33), num(w.RR.dynamicRadius, 0.33)
    s.rDynF, s.rDyn4 = 0.5 * (a + b), 0.25 * (a + b + c + d)
    s.sumBT = num(w.FL.brakeTorque, 0) + num(w.FR.brakeTorque, 0) + num(w.RL.brakeTorque, 0) + num(w.RR.brakeTorque, 0)
  end

  -- rear-steer request (+ excess right yaw -> rear right) from s.ec (filtStep)
  local function rsStep()
    local s, r = ST, CFG.ras
    s.rasUsable = s.rasPresent and s.rasSel == "DSC" and s.rasDir == 1 and s.rasFault == "none"
    local st = s.stageEff
    local k = (st == 1) and r.k1 or ((st >= 2) and r.k2 or 0)
    local req = 0
    if k > 0 and s.rasUsable and not s.rsDisabled and s.ysc ~= -1 and not s.suspended and s.armedSway then
      req = clamp(fin(k * s.ec), -r.maxDeg, r.maxDeg)
    end
    s.rsReq = req
  end

  -- single front-wheel brake target (applied at physics rate)
  local function wheelTargetStep()
    local s, w = ST, CFG.wheel
    local pause = s.stockYc or (w.pauseOnTcs and s.tcs)
    local st = s.stageEff
    s.wPause = st >= 2 and pause
    local T, side = 0, 0
    if st >= 2 and not s.suspended and s.kmh >= CFG.gate.stopKmh and not pause and s.ysc ~= -1 and REG.whl then
      local Mz = min(w.maxMzNm, w.kNmPerDegS * max(0, abs(s.ecB) - w.dbDegS))
      T = fin(Mz / max(REG.halfTrack, 0.3) * s.rDynF)
      side = (s.ecB > 0) and 1 or 2 -- excess right yaw -> brake the left front wheel
    end
    s.wTgtFL = (side == 1) and T or 0
    s.wTgtFR = (side == 2) and T or 0
    if T > 0 then s.wIdle = false end
  end

  -- combination deceleration with push limiter and brake PI
  local function decelStep(dt)
    local s, c = ST, CFG.decel
    local st = s.stageEff
    local fx5 = lp1(F.fx5, PIN.Fx, CFG.pin.ctlLpHz, dt)
    local aEvt = (st == 2 and s.coupled and s.holdT > 0 and s.kmh > s.holdVRec) and c.s2EvtMs2 or 0
    local aT = 0
    if not s.suspended then aT = max((st == 3) and c.s3Ms2 or 0, aEvt, s.aK2) end
    aT = min(aT, sqrt(max(0, c.frictionMs2 * c.frictionMs2 - s.ay * s.ay)))
    if not s.pinValid then aT = min(aT, c.capNoPinMs2) end
    if aT > 0 and s.pinValid then
      local m2 = massKnown() and EST.m2 or c.massDefault
      local smax = max(c.pushMinN, c.pushFrac * m2 * 9.81)
      local ex = (-smax - fx5) / 1000 -- kN beyond the allowed push
      if ex > 0 then s.aCutI = s.aCutI + c.pushKi * ex * dt else s.aCutI = max(0, s.aCutI - c.pushDecay * dt) end
      s.aCutI = min(s.aCutI, aT)
      s.aCut = clamp(c.pushKp * max(ex, 0) + s.aCutI, 0, aT)
    else
      s.aCut, s.aCutI = 0, 0
    end
    s.aT = aT
    local tgt = aT - s.aCut
    local a = s.aCmd
    if tgt > a then a = min(tgt, a + c.jerkUp * dt) else a = max(tgt, a - c.jerkDn * dt) end
    s.aCmd = fin(a)
    if s.aCmd > 0.05 then
      local ff = 0
      if s.sumBT > 100 then ff = clamp(REG.m1 * s.aCmd * s.rDyn4 / s.sumBT, 0, c.brakeMax) end
      local err = s.aCmd - s.aMeas
      local I = clamp(s.bI + c.ki * err * dt, -c.iMax, c.iMax)
      local out = ff + c.kp * err + I
      if (out > c.brakeMax and err > 0) or (out < 0 and err < 0) then I = s.bI end -- anti-windup
      s.bI = I
      s.brakeCmd = fin(clamp(ff + c.kp * err + I, 0, c.brakeMax))
    else
      s.bI = 0
      s.brakeCmd = max(0, s.brakeCmd - c.releasePerS * dt)
    end
  end

  -- torque mode, freeze, lamp (+ hold)
  local function outStep()
    local s = ST
    local st, K = s.stageEff, s.K
    local tm = 0
    if st == 1 or K == 1 then tm = 1 end
    if st >= 2 or K == 2 then tm = 2 end
    if tm == 0 and s.suspended and s.armedPre and s.amp >= CFG.det.a1DegS * scNow() then tm = 1 end
    s.torqueMode = tm
    -- HOLD cap recovery (2026-10-17 (l)): a corner K1 alone (no sway stage, not suspended) in a settled bend (wind < 0.5) with the
    -- prediction and the filtered measured ay below tqRecFrac x aB, while the combination still loses speed (filtered
    -- deceleration above tqRecDecMs2): the coordinator raises its HOLD cap at tqRecNmS towards the driver request, so
    -- the driver gets the torque to hold the speed back; never while speeding up (a cap above the need would carry the
    -- combination into K2)
    local c = CFG.corner
    local lvl = c.tqRecFrac * s.aB
    s.tqRec = (tm == 1 and K == 1 and st == 0 and not s.suspended and s.wind < 0.5 and s.aPred < lvl and s.ayF < lvl
      and -s.axR > c.tqRecDecMs2) and c.tqRecNmS or 0
    s.freeze = st >= 1 or K >= 1
    local lo = st >= 2 or K == 2
    if lo then s.lampOffT = s.now end
    s.lampEdge = lo and not s.lampOut
    s.lampOut = lo
    s.lampPub = lo or (s.now - s.lampOffT < CFG.det.lampOffHoldS)
  end

  -- 10
  function actStep(dt)
    wheelInfo()
    rsStep()
    wheelTargetStep()
    decelStep(dt)
    outStep()
  end
end

-- ---------------------------------------------------------------------------
-- sign guards (the share-effect check runs on every half-wave close)
-- ---------------------------------------------------------------------------

local guardStep
do
  -- the realised rear-steer share must follow the lagged request, else share off (latched)
  local function signGuardStep(dt)
    local s, r = ST, CFG.ras
    s.rq = lp1(F.rq, s.rsReq, 1 / (2 * pi * max(r.signLagS, 1e-3)), dt)
    local dec = exp(-dt / max(r.signWinS, 0.01))
    s.sgS1, s.sgS2 = s.sgS1 * dec, s.sgS2 * dec
    if s.rsDisabled then return end
    if s.stageEff >= 1 and s.rasUsable and abs(s.rq) >= r.signMinDeg and abs(s.rasTs) >= 0.1
        and s.rasState ~= "FAULT" and band(s.rasLimit, LIMIT_BLOCK) == 0 then
      local real = s.rasAngle - s.rasFf - s.rasFb
      s.sgS1 = s.sgS1 + real * s.rq * dt
      s.sgS2 = s.sgS2 + abs(real) * abs(s.rq) * dt
      if s.sgS2 >= r.signMinSum and s.sgS1 / s.sgS2 < r.signFrac then
        s.rsDisabled = true
        limitedNotice("steering share disabled: rear actuator opposes the request")
      end
    end
  end

  -- yaw-sign sample: steady corner, forward gear, DSC, no stock intervention, yaw rate within the steering's
  -- reach, small body slip (a countersteered slide yaws against the steering legitimately)
  local function yscOK()
    local s, y = ST, CFG.ysc
    if not (s.cmuOK and s.enabledEff and s.fwdGear and s.dscKey == "dsc" and not s.dscOff and not s.stockYc
      and not s.tcs and s.slipOK) then
      return false
    end
    local ad, ar = abs(s.deff), abs(s.r)
    return s.kmh >= y.minKmh and s.kmh <= y.maxKmh and ad >= rad(y.minDeffDeg) and ar >= rad(y.minYawDegS)
      and ar <= YSC_GAIN * abs(s.G) * ad + YSC_R0 and abs(s.dDot) < y.maxDdotDegS and s.ay >= y.minAyMs2
      and s.brakeIn < 0.01 and s.stageEff == 0 and s.K == 0
  end

  -- yaw-rate sign plausibility (covers the wheel brake without RAS); only back-to-back windows count
  local function yscStep(dt)
    local s, y = ST, CFG.ysc
    if not yscOK() then
      s.yscT, s.yscN, s.yscA, s.yscAg, s.yscDis = 0, 0, 0, 0, 0
      return
    end
    s.yscT, s.yscN = s.yscT + dt, s.yscN + 1
    if sgn(s.r) == sgn(s.deff) then s.yscA = s.yscA + 1 end
    if s.yscT < y.windowS then return end
    local frac = s.yscA / max(s.yscN, 1)
    if frac >= y.agreeFrac then
      s.yscAg, s.yscDis = s.yscAg + 1, 0
    elseif frac <= 1 - y.agreeFrac then
      s.yscAg, s.yscDis = 0, s.yscDis + 1
    else
      s.yscAg, s.yscDis = 0, 0
    end
    s.yscT, s.yscN, s.yscA = 0, 0, 0
    if s.yscDis >= y.windows and s.ysc ~= -1 then
      s.ysc = -1
      limitedNotice("yaw-rate sign implausible against the steering: wheel brake and steering share off")
    elseif s.yscAg >= y.windows and s.ysc == 0 then
      s.ysc = 1
    end
  end

  -- 11
  function guardStep(dt)
    signGuardStep(dt)
    yscStep(dt)
  end
end

-- ---------------------------------------------------------------------------
-- recommendation, messages, cruise control
-- ---------------------------------------------------------------------------

local function reduceMsg()
  local s = ST
  if s.now - s.msgReduceT >= CFG.rec.msgRepeatS then
    s.msgReduceT = s.now
    toast(TXT.reduce)
  end
end

-- 12: recommended speed, "Reduce speed", trailer-mode hint
local function recStep(dt)
  local s, r = ST, CFG.rec
  if s.holdT > 0 then s.holdT = max(0, s.holdT - dt) end
  local lowT = lowTongue(tongueEff())
  local vRec = r.vRecKmh
  if lowT then vRec = min(vRec, r.lowTongueVRecKmh) end
  if s.holdT > 0 then vRec = min(vRec, s.holdVRec) end
  s.vRec = vRec
  local on = s.trailerMode and s.enabledEff and not s.dscOff and not s.swFault and not s.physFault
  -- K2 asks to slow down (banner, toast) only when it is clearly significant: the prediction without the look-ahead
  -- lift (response while the corner builds, measured floor; cornerStep aPl) held this K2 on its own (>= k2ExitFrac x the
  -- budget) for k2MsgS, or reached k2MsgFrac x the budget; a K2 of the look-ahead alone (turn-in jab, any length) does not ask
  local pl = s.aPl
  if s.K ~= 2 then s.k2T = 0 elseif pl >= CFG.corner.k2ExitFrac * s.aB then s.k2T = s.k2T + dt end
  s.k2Q = s.K == 2 and (s.k2Q or s.k2T >= r.k2MsgS - 1e-6 or pl >= r.k2MsgFrac * s.aB)
  local stK = s.stageEff == 3 or s.k2Q
  if on and s.coupled and s.kmh > vRec + r.overKmh then s.overT = s.overT + dt else s.overT = 0 end
  local was = s.overLatch
  if on and s.overT >= r.overS then s.overLatch = true end
  if s.overLatch and (not on or not s.coupled or s.kmh <= vRec) then s.overLatch = false end
  s.reduce = on and (stK or s.overLatch)
  if on and stK and not s.stKPrev then reduceMsg() end
  if on and s.overLatch and not was and (s.holdT > 0 or lowT) then reduceMsg() end
  s.stKPrev = stK
  -- coupled without trailer mode: one hint per coupling
  if s.coupled and not s.trailerMode and REG.tmProv and s.enabledEff and not s.swFault and not s.physFault then
    s.hintT = s.hintT + dt
    if s.hintT >= 1.0 and not s.hintShown then
      s.hintShown = true
      toast(TXT.hint)
    end
  else
    s.hintT = 0
  end
end

-- stock cruise control off on a braking-stage edge (plain global lookup, never loads the extension)
local function cruiseStep()
  local s = ST
  if not (s.lampEdge and s.ccOn and CFG.rec.cruiseCancel) then return end
  local cc = rawget(_G, "cruiseControl")
  if type(cc) == "table" and type(cc.setEnabled) == "function" then
    if pcall(cc.setEnabled, false) then
      toast(TXT.cruise)
      info("cruise control switched off (braking stage)")
    end
  end
end

-- ---------------------------------------------------------------------------
-- publish
-- ---------------------------------------------------------------------------

local function flagsOf()
  local s = ST
  local b = 0
  if s.coupled then b = b + 1 end
  if s.trailerMode then b = b + 2 end
  if s.armedSway then b = b + 4 end
  if s.armedCorner then b = b + 8 end
  if s.rasUsable then b = b + 16 end
  if s.rsDisabled then b = b + 32 end
  if s.pinValid then b = b + 64 end
  if s.supp then b = b + 128 end
  if s.now - s.lastExitT < CFG.det.reactS then b = b + 256 end
  if s.wPause then b = b + 512 end
  if s.aCut > 0.05 then b = b + 1024 end
  if s.dscKey == "dtc" then b = b + 2048 end
  if s.injOn or s.qaForce ~= nil then b = b + 4096 end
  if s.suspended then b = b + 8192 end
  local ts = tongueEff()
  if ts ~= "NONE" then b = b + 16384 end
  if lowTongue(ts) then b = b + 32768 end
  if s.ysc == -1 then b = b + 65536 end
  if s.holdT > 0 then b = b + 131072 end
  if s.lowMuT > 0 then b = b + 262144 end
  return b
end

-- 13: every channel, finite-checked
local function publish()
  local s, ev, e, P = ST, electrics.values, EST, PIN
  local st = stateName()
  s.state = st
  if st == "DISABLED" or st == "FAULT" then
    s.lampPub, s.lampOffT = false, -1e9
  end
  ev.etk854hybrid_tsaState = st
  ev.etk854hybrid_tsaStage = s.stageEff
  ev.etk854hybrid_tsaCorner = s.K
  ev.etk854hybrid_tsaLamp = s.lampPub and 1 or 0
  ev.etk854hybrid_tsaYawErrDegS = fin(s.ecB)
  ev.etk854hybrid_tsaAmpDegS = fin(s.amp)
  ev.etk854hybrid_tsaSwayIdx = fin(s.swayIdx)
  ev.etk854hybrid_tsaFreqHz = fin(s.freq)
  ev.etk854hybrid_tsaRsReqDeg = fin(s.rsReq)
  ev.etk854hybrid_tsaWheelNm = fin(s.wCurFR - s.wCurFL)
  ev.etk854hybrid_tsaBrakeCmd = fin(s.brakeCmd)
  ev.etk854hybrid_tsaDecelReq = fin(s.aCmd)
  ev.etk854hybrid_tsaTorqueMode = s.torqueMode
  ev.etk854hybrid_tsaTorqueRec = fin(s.tqRec)
  ev.etk854hybrid_tsaFreeze = s.freeze and 1 or 0
  ev.etk854hybrid_tsaAyPred = fin(s.aPred)
  ev.etk854hybrid_tsaAyBudget = fin(s.aB)
  ev.etk854hybrid_tsaVchT = fin(e.vchT)
  ev.etk854hybrid_tsaHitchFx = fin(P.Fx)
  ev.etk854hybrid_tsaHitchFy = fin(P.Fy)
  ev.etk854hybrid_tsaHitchFz = fin(P.Fz)
  ev.etk854hybrid_tsaDrawbarN = fin(s.drawbar)
  local ts = tongueEff()
  ev.etk854hybrid_tsaTongueKg = (ts ~= "NONE") and fin(e.tongueKg) or 0
  ev.etk854hybrid_tsaTongueState = ts
  ev.etk854hybrid_tsaMassKg = massKnown() and fin(e.m2) or -1
  ev.etk854hybrid_tsaMassQ = s.coupled and fin(e.massQ) or 0
  ev.etk854hybrid_tsaLoadPin = P.state
  ev.etk854hybrid_tsaVRecKmh = fin(s.vRec)
  ev.etk854hybrid_tsaReduceSpeed = s.reduce and 1 or 0
  ev.etk854hybrid_tsaFlags = flagsOf()
end

-- all request channels 0 (init, reset, fault)
local function publishOff(state)
  local ev = electrics.values
  ev.etk854hybrid_tsaState = state
  ev.etk854hybrid_tsaStage = 0
  ev.etk854hybrid_tsaCorner = 0
  ev.etk854hybrid_tsaLamp = 0
  ev.etk854hybrid_tsaYawErrDegS = 0
  ev.etk854hybrid_tsaAmpDegS = 0
  ev.etk854hybrid_tsaSwayIdx = 0
  ev.etk854hybrid_tsaFreqHz = 0
  ev.etk854hybrid_tsaRsReqDeg = 0
  ev.etk854hybrid_tsaWheelNm = 0
  ev.etk854hybrid_tsaBrakeCmd = 0
  ev.etk854hybrid_tsaDecelReq = 0
  ev.etk854hybrid_tsaTorqueMode = 0
  ev.etk854hybrid_tsaTorqueRec = 0
  ev.etk854hybrid_tsaFreeze = 0
  ev.etk854hybrid_tsaAyPred = 0
  ev.etk854hybrid_tsaAyBudget = 0
  ev.etk854hybrid_tsaVchT = fin(EST.vchT or 55)
  ev.etk854hybrid_tsaHitchFx = 0
  ev.etk854hybrid_tsaHitchFy = 0
  ev.etk854hybrid_tsaHitchFz = 0
  ev.etk854hybrid_tsaDrawbarN = 0
  ev.etk854hybrid_tsaTongueKg = 0
  ev.etk854hybrid_tsaTongueState = "NONE"
  ev.etk854hybrid_tsaMassKg = -1
  ev.etk854hybrid_tsaMassQ = 0
  ev.etk854hybrid_tsaLoadPin = PIN.fault and "FAULT" or "OFF"
  ev.etk854hybrid_tsaVRecKmh = fin(CFG.rec and CFG.rec.vRecKmh or 100)
  ev.etk854hybrid_tsaReduceSpeed = 0
  ev.etk854hybrid_tsaFlags = 0
end

local function offStateName()
  return enabledNow() and "OFF" or "DISABLED"
end

-- ---------------------------------------------------------------------------
-- core (100 Hz) and fault containment
-- ---------------------------------------------------------------------------

local function core100(dt)
  if not (dt > 0) then return end
  if dt > 0.1 then dt = 0.1 end
  local s = ST
  s.now = s.now + dt
  pinFixed(dt)       -- 1
  pinTareStep(dt)
  pinHealth(dt)
  readInputs(dt)     -- 2
  gateStep(dt)       -- 3
  pinStateStep(dt)
  refStep(dt)        -- 4
  filtStep(dt)       -- 5
  hwStep(dt)         -- 6
  stageStep(dt)      -- 7
  cornerStep(dt)     -- 8
  estStep(dt)        -- 9
  actStep(dt)        -- 10
  guardStep(dt)      -- 11
  recStep(dt)        -- 12
  cruiseStep()
  publish()          -- 13
end

-- all requests 0 at once, state FAULT (every step until the vehicle reset)
local function faultPath()
  local s = ST
  if not s.faultNoted then
    s.faultNoted = true
    pcall(log, "E", CONST.NAME, "trailer stability fault, function off until the vehicle reset: " .. tostring(s.swErr))
    toast(TXT.fault)
  end
  s.stage, s.stageEff, s.K, s.rsReq, s.aCmd, s.aT, s.aCut, s.aCutI, s.bI, s.brakeCmd = 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
  s.wTgtFL, s.wTgtFR, s.wCurFL, s.wCurFR, s.wIdle = 0, 0, 0, 0, true
  s.torqueMode, s.freeze, s.lampOut, s.lampPub, s.lampOffT, s.reduce = 0, false, false, false, -1e9, false
  s.tqRec = 0
  PIN.sampling = false
  publishOff("FAULT")
end

M.updateFixedStep = function(dt)
  local s = ST
  if not (s.swFault or s.physFault) then
    local ok, err = pcall(core100, dt)
    if ok then return end
    s.swFault = true
    s.swErr = tostring(err)
  end
  if not pcall(faultPath) then publishOff("FAULT") end
end

local function physFail(err)
  local s = ST
  s.physFault = true
  s.swErr = "physics: " .. tostring(err)
  s.wTgtFL, s.wTgtFR, s.wCurFL, s.wCurFR, s.wIdle = 0, 0, 0, 0, true
  PIN.sampling = false
end

-- physics rate: load pin sampling (every subSample-th step)
M.update = function(dt)
  local P = PIN
  if P.sampling and not ST.physFault then
    local k = P.sub + 1
    if k < P.subN then
      P.sub = k
      return
    end
    P.sub = 0
    local ok, err = pcall(pinPhys)
    if not ok then physFail(err) end
  end
end

-- one front wheel: slip limiter, ramps, "raise only" addition
local function wheelOne(wd, tgt, cur, vRef, inv, l0, l1, up, dn)
  if wd.isBroken or (wd.absActive and ST.brakeIn > 0.1) then tgt = 0 end
  local lam = (vRef - abs(wd.wheelSpeed)) * inv
  if not (lam <= l1) then
    cur = 0 -- beyond lambda1 (or NaN): immediate cut
  else
    local f = (lam - l0) / (l1 - l0)
    if f < 0 then f = 0 elseif f > 1 then f = 1 end
    tgt = tgt * (1 - f)
    if tgt > cur then cur = min(tgt, cur + up) else cur = max(tgt, cur - dn) end
  end
  if cur > 0 and not wd.isBroken then
    local d = wd.desiredBrakingTorque
    local nd = min(wd.brakeTorque, d + cur)
    if nd > d then wd.desiredBrakingTorque = nd end
  end
  return cur
end

local function wheelPhys(dt)
  local s, w, wh = ST, CFG.wheel, REG.whl
  if not wh then
    s.wCurFL, s.wCurFR, s.wIdle = 0, 0, true
    return
  end
  local vRef = s.v
  local inv = 1 / max(vRef, 1)
  local l0, l1 = w.lambda0, w.lambda1
  local up, dn = w.rampNmS * dt, w.rampDnNmS * dt
  local a = wheelOne(wh.FL, s.wTgtFL, s.wCurFL, vRef, inv, l0, l1, up, dn)
  local b = wheelOne(wh.FR, s.wTgtFR, s.wCurFR, vRef, inv, l0, l1, up, dn)
  s.wCurFL, s.wCurFR = a, b
  if a == 0 and b == 0 and s.wTgtFL == 0 and s.wTgtFR == 0 then s.wIdle = true end
end

-- physics rate, after the pedal path (ABS) and before the brake torques
M.updateWheelsIntermediate = function(dt)
  local s = ST
  if s.wIdle or s.physFault then return end
  local ok, err = pcall(wheelPhys, dt)
  if not ok then physFail(err) end
end

-- this frame's toasts as ONE message: the Messages app keeps one entry per category, a second message in
-- the same frame would replace the first. Same-prefix texts are joined ("Trailer stability: a - b").
local function joinToasts(q, n)
  local pfx = CONST.MSG_PFX
  local np = #pfx
  local txt = q[1]
  for i = 2, n do
    local t, dup = q[i], false
    for j = 1, i - 1 do
      if q[j] == t then dup = true end
    end
    if not dup then
      if ssub(t, 1, np) == pfx and ssub(txt, 1, np) == pfx then
        txt = txt .. " - " .. ssub(t, np + 1)
      else
        txt = txt .. " | " .. t
      end
    end
  end
  return txt
end

M.updateGFX = function(dt)
  local s = ST
  local n = s.msgN
  if n > 0 then
    local txt = s.msgQ[1]
    if n > 1 then
      local ok, j = pcall(joinToasts, s.msgQ, n)
      txt = ok and j or s.msgQ[n]
    end
    for i = 1, n do s.msgQ[i] = nil end
    s.msgN = 0
    pcall(guihooks.message, txt, 5, CONST.MSG_CAT)
  end
end

-- own hitch couplers only; the other object's id only excludes self-coupling
M.onCouplerAttached = function(nodeId, otherId)
  local P = PIN
  if P.HC[nodeId] and otherId ~= nil and otherId ~= REG.objId and not P.hookSet[nodeId] then
    P.hookSet[nodeId] = true
    P.hookN = P.hookN + 1
  end
end

M.onCouplerDetached = function(nodeId, otherId)
  local P = PIN
  if P.hookSet[nodeId] then
    P.hookSet[nodeId] = nil
    P.hookN = max(0, P.hookN - 1)
  end
end

-- ---------------------------------------------------------------------------
-- lifecycle
-- ---------------------------------------------------------------------------

M.init = function(jbeamData)
  local jd = type(jbeamData) == "table" and jbeamData or {}
  local issues = {}
  local c = parseCfg(jd, issues)
  CFG = c
  REG = newRegistry()
  REG.jbeamEnabled = sw(jd.enabled, true)
  REG.gBp = makeupGain(c.filt.bpdHpHz, c.filt.bpdLpHz)
  REG.vch0 = c.ref.vchDefault
  COMP.componentOrderYawControl = c.dev.componentOrder
  F = newFilters()
  ST = newState()
  ST.vRec = c.rec.vRecKmh
  PIN = newPin()
  local ok, err = pcall(pinDiscover)
  if not ok then
    PIN.fault, PIN.discFault, PIN.why = true, true, "discovery error: " .. tostring(err)
  end
  if PIN.discFault then warn("load pin unavailable (" .. tostring(PIN.why) .. ")") end
  EST = newEst()
  if #issues > 0 then
    warn(sfmt("calibration: %d value(s) invalid or out of range, default / clamp used: %s", #issues,
      table.concat(issues, ", ", 1, min(#issues, 16))))
  end
  -- never call another controller here (hash-order init)
  publishOff(offStateName())
end

-- CMU 50 -> yawControl 60 -> us 84 (yawControl sorts components in initLastStage); the component is
-- registered regardless of the jbeam switch (setEnabled)
M.initSecondStage = function(jbeamData)
  local ok, err = pcall(discover)
  if not ok then
    REG.off, REG.ycPresent = true, false
    warn("discovery failed, trailer stability unavailable: " .. tostring(err))
  end
end

M.initLastStage = function(jbeamData)
  local g = REG
  -- CMU all-active shutdown (CMU.initLastStage)
  if g.cmu and (g.cmu.systemState == "offline" or not (g.yc and g.yc.isActive == true)) then g.ycPresent = false end
  pcall(readStats)
  info(sfmt("trailer stability ready: pin=%d/%d mh=%.1f comp=%s enabled=%s", PIN.nB, PIN.nT, PIN.mh,
    tostring(g.compReg), tostring(g.jbeamEnabled)))
  publishOff(offStateName())
end

local function pinReset()
  local P = PIN
  P.ax, P.ay, P.az, P.n, P.sub, P.nanRun = 0, 0, 0, 0, 0, 0
  P.Fx, P.Fy, P.Fz, P.ux, P.uy, P.uz, P.fresh = 0, 0, 0, 0, 0, 0, false
  P.invalidT, P.bigT, P.contact, P.contactLogged = 0, 0, false, false
  P.tareT, P.tN, P.tSx, P.tSy, P.tSz, P.tareReq, P.tareDone = 0, 0, 0, 0, 0, false, false
  P.hookSet, P.hookN = {}, 0
  P.sampling, P.window, P.healthT, P.state = false, false, 0, "OFF"
  if P.deformed then
    -- deformed carrier: the old tare is no longer valid
    P.tared, P.deformed = false, false
    P.tx, P.ty, P.tz = P.t0x, P.t0y, P.t0z
  end
  if P.fault and not P.discFault then
    P.fault, P.faultMsg, P.why = false, false, nil -- re-checked after the reset
  end
end

-- keeps dscOn, setEnabled, the clock, tare (unless deformed), estimators, registrations; clears the rest
M.reset = function(jbeamData)
  local old = ST
  local s = newState()
  s.now, s.dscOn, s.enForce, s.dm = old.now, old.dscOn, old.enForce, old.dm
  s.lastResetT = old.now
  s.hintShown = old.hintShown
  s.vRec = CFG.rec.vRecKmh
  ST = s
  F = newFilters()
  pinReset()
  publishOff(offStateName())
end

M.resetLastStage = function()
  publishOff(offStateName())
end

-- ---------------------------------------------------------------------------
-- dev / QA API (pcall-safe)
-- ---------------------------------------------------------------------------

local function loadPinCore()
  local P = PIN
  return {
    state = P.state, nBeams = P.nB, nTb = P.nT, mh = P.mh, Fx = P.Fx, Fy = P.Fy, Fz = P.Fz,
    tare = {x = P.tx, y = P.ty, z = P.tz}, tared = P.tared, deformed = P.deformed, samples = P.total,
    subSample = CFG.pin.subSample, couplers = P.nHC, hooks = P.hookN, fault = P.fault, why = P.why
  }
end

M.getLoadPin = function()
  local ok, res = pcall(loadPinCore)
  if ok then return res end
  return nil
end

local function hwList()
  local s = ST
  local out = {}
  local n = min(s.hwN, HIST)
  for k = n - 1, 0, -1 do
    local i = hwAt(k)
    out[#out + 1] = {A = s.hwA[i], E = s.hwE[i], fy = s.hwFy[i], f = s.hwF[i], q = s.hwQ[i], valid = s.hwV[i],
                     rawOK = s.hwR[i], t = s.hwT[i], w = s.hwW[i]}
  end
  return out
end

local function stateCore()
  local s, e, g = ST, EST, REG
  local lo, hi, fb = freqBand()
  local cnt, occ = persistCounts()
  local inj = s.qaInj
  return {
    state = s.state, stage = s.stageEff, stageCalc = s.stage, corner = s.K, lamp = s.lampPub, lampOut = s.lampOut,
    enabled = s.enabledEff, jbeamEnabled = g.jbeamEnabled, configOK = true,
    armedSway = s.armedSway, armedCorner = s.armedCorner, dscKey = s.dscKey, dscOn = s.dscOn,
    trailerMode = s.trailerMode, trailerAttached = s.trailerAttached, hookCoupled = s.hookCoupled, coupled = s.coupled,
    v = s.v, kmh = s.kmh, e = s.eDet, eMeas = s.eMeas, ec = s.ecB, ecShare = s.ec, amp = s.amp, freq = s.freq,
    trailerSide = {kFy = s.kFy, amp = ampTrl()}, swayV = {amp = s.ampSw, calmS = s.calmT},
    fLearn = {lo = lo, hi = hi, fBar = fb, n = s.flN}, hw = hwList(), s3Why = s.s3Why, why = s.why,
    persist = {entries = cnt, occS = occ},
    eventHold = {active = s.holdT > 0, vRec = s.holdVRec, leftS = s.holdT},
    rsReq = s.rsReq, rsDisabled = s.rsDisabled, rasUsable = s.rasUsable, ysc = s.ysc,
    wheel = {FL = s.wCurFL, FR = s.wCurFR, tgtFL = s.wTgtFL, tgtFR = s.wTgtFR},
    brakeCmd = s.brakeCmd, decel = s.aCmd, aT = s.aT, aCut = s.aCut, torqueMode = s.torqueMode, freeze = s.freeze,
    torqueRec = s.tqRec, axRec = s.axR,
    aPred = s.aPred, aB = s.aB, vchT = e.vchT, mass = e.m2, massQ = e.massQ, tongue = e.tongueKg,
    pred = {aSS = s.aSS, aRp = s.aRp, aR = s.aR, floor = s.aFloor, ayF = s.ayF, ay = s.ayS, laOn = s.laOn, turnS = s.tiT,
            turnPk = s.tiPk, laSpentPk = s.laPk, unwindS = s.uwT, wind = s.wind, plain = s.aPl},
    massQuietS = s.mQuietT, massSteps = e.nEv,
    lowMuT = s.lowMuT,
    lowMu = {src = s.muSrc, evidence = s.muEv, use = s.muUse, slipDrive = s.muSlD, slipBrake = s.muSlB, dwellS = s.muEvT,
             blankS = s.muBlankT},
    tongueState = tongueEff(), vRec = s.vRec, reduce = s.reduce, flags = flagsOf(),
    suspended = s.suspended, swFault = s.swFault, physFault = s.physFault, swErr = s.swErr,
    qa = {inject = inj and {ampDegS = inj.amp, hz = inj.hz, durS = inj.dur, ctlScale = inj.ctl, t = inj.t} or nil,
          force = s.qaForce, budget = s.qaBudget, eRealAmp = s.ampM},
    component = {registered = g.compReg, count = g.compCount, off = g.compOff, err = g.compErr},
    loadPin = loadPinCore()
  }
end

M.getState = function()
  local ok, res = pcall(stateCore)
  if ok then return res end
  return {error = tostring(res)}
end

M.getParams = function()
  local ok, res = pcall(copy, CFG)
  if ok then return res end
  return nil
end

-- runtime A/B (dev); nil = jbeam value; returns the effective switch
M.setEnabled = function(b)
  local s = ST
  if b == true or b == false then s.enForce = b else s.enForce = nil end
  return enabledNow()
end

-- open-loop replace mode: detection sees amp*sin(2*pi*hz*t), control sees
-- ctlScale times that; plausibility counts as met. nil clears.
M.qaInject = function(t)
  local s = ST
  if type(t) ~= "table" then
    s.qaInj = nil
    return nil
  end
  local amp, hz, dur, ctl = num(t.ampDegS, 0), num(t.hz, 1), num(t.durS, 0), num(t.ctlScale, 0.4)
  if amp == 0 or hz <= 0 or dur <= 0 then
    s.qaInj = nil
    return nil
  end
  s.qaInj = {amp = amp, hz = hz, dur = dur, ctl = ctl, t = 0}
  return true
end

-- stage = max(computed, n) while sway-armed; nil clears
M.qaForceStage = function(n)
  local x = num(n, nil)
  if x == nil or x < 1 then
    ST.qaForce = nil
    return nil
  end
  ST.qaForce = clamp(floor(x + 0.5), 1, 3)
  return ST.qaForce
end

-- corner budget override [m/s^2]; nil clears
M.qaSetBudget = function(aB)
  local x = num(aB, nil)
  if x == nil or x <= 0 then
    ST.qaBudget = nil
    return nil
  end
  ST.qaBudget = x
  return x
end

-- tare request (only inside a tare window: standstill, nothing coupled)
M.tare = function()
  local P = PIN
  if P.window and not P.fault then
    P.tareReq = true
    return true
  end
  return false
end

M.resetEstimators = function()
  local ok = pcall(resetEst)
  return ok
end

return M
