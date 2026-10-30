-- ETK 854 ActiveHybrid - active rear-axle steering (RAS)
--
-- Controller of the part etk854hybrid_suspension_R_ras. Both rear toe links are hydros on one
-- command channel etk854hybrid_rasCmd in [-1, 1] (+ = rear wheels right):
--   ratio_L = toe + F * cmd,  ratio_R = toe - F * cmd    (toe = $toe_RR, F = linkRatioPerUnit)
-- cmd 1 = actuatorDegPerUnit road-wheel deg; cmd 0 is exactly the stock static toe, even if
-- this controller never runs.
--
-- Conventions: + = right for every angle and yaw rate (frontWheelAngle carries the steering
-- input sign); rear angles in road-wheel deg; oversteer <=> sgn(r) * sgn(beta) < 0.
-- Single-track model: r = G(v) * (df - dr), G(v) = v / (L * (1 + v^2 / vch^2)).
--
-- Rear target = ffOut + fbOut + tsOut (only ffOut is credited to the DSC yaw reference):
--   ffOut  feed-forward dr = k(v) * df, k(v) per set NORMAL / SPORT / TRAILER (zero crossing
--          60 / 60 / 40 km/h): k < 0 counter-phase at low speed (smaller turning circle),
--          k > 0 in-phase at speed (lower yaw gain and body slip). Soft limits (softClip),
--          in-phase share on a lagged front angle, set change cross-faded over blendS.
--   fbOut  silent oversteer assist (COMP); dead band + ts.assistDbAdd while tsaStage >= 1
--   tsOut  trailer stability request etk854hybrid_tsaRsReqDeg (selection DSC, dirCheck 1)
--
-- Driver selection (Actions app row R-STEER; kept across reset, serialised across reload;
-- spawn: $etk854_ras_enable 0 -> OFF, else $etk854_ras_assist 1 -> DSC, 0 -> NODSC):
--   OFF    rear wheels straight (also at a standstill), yaw provider answers nil
--   DSC    feed-forward + assist + corrected yaw reference + trailer share. DSC / ABS
--          braking -> NO_INCREASE, except an escActive raised by the trailer stability alone.
--   NODSC  feed-forward + corrected reference; straight while the DSC intervenes
--          (+ dscNeutralHoldS); ABS alone -> NO_INCREASE.
--   DSC OFF (drive mode "off" or yawControl disabled) acts like OFF for every selection.
-- Priority: FAULT > numeric dev override > OFF / DSC OFF > NODSC neutral > launch / CENTRE /
-- NO_INCREASE.
--
-- Safe states: CENTRE in P, ignition off and launch hold; FREEZE below freezeKmh (no tyre
-- wind-up); NO_INCREASE (angle may only shrink) under ABS / DSC braking; FADE of the
-- feed-forward in a confirmed slide; FAULT (broken toe link, Lua error) centres the axle.
-- Target and command are finite-checked before the clamp (a failure raises -> SOFTWARE fault):
-- in LuaJIT math.min(NaN, x) == x, and hydros.lua turns a NaN input into full deflection.
--
-- DSC hooks, registered once per spawn and only if the CMU's yawControl and STMEstimate are
-- active (a lone provider answering nil crashes yawControl):
--   PROV  yaw reference provider (order 10, ahead of STMEstimate 20): STMEstimate's model with
--         df - dr_credit, else counter-phase yaw reads as oversteer. Answers nil (stock) when
--         in doubt; out of service -> feed-forward caps x noProviderScale.
--   COMP  yaw component (order 5): oversteer-only rear angle request, always returns false
--         (no DSC lamp, no escActive).
--
-- Base-game behaviour relied on (0.39.4):
--   * initSecondStage -> hydros.init -> initLastStage: hydros resolved in init, their live
--     mapping checked in initLastStage (liveCheck).
--   * controller.updateGFX runs before hydros.updateGFX: a command acts in the same frame.
--   * Reset: M.reset -> hydros.reset -> resetLastStage. The hydro input survives hydros.reset
--     (0 is written in reset and resetLastStage). With M.reset present initSecondStage is not
--     re-run and yawControl.reset is empty, so the hooks stay registered exactly once.
--   * Functions are cached after every init/reset stage: fault paths branch inside updateGFX.
--   * The CMU resets systemState to "normal" after a shutdown: isActive is checked as well.
--
-- Direction check (rasDirCheck, see dirCheckStep): arms the assist and the trailer share; a
-- negative latch takes PROV out of service. Kept across reset, never a FAULT.
--
-- Electrics (sole writer, every frame), prefix etk854hybrid_:
--   rasCmd -1..1 | rasAngleDeg actual | rasTargetDeg gated target | rasFfDeg / rasFbDeg /
--   rasTsDeg slewed shares [deg] | rasGain blended k (+ = in-phase)
--   rasState     OFF | READY | ACTIVE | HOLD | LIMIT | FAULT   (OFF also for sel OFF / DSC OFF)
--   rasSel       OFF | DSC | NODSC         rasMode  NORMAL | SPORT | TRAILER
--   rasFault     none | BROKEN | SOFTWARE | CONFIG
--   rasLimit     1 FREEZE 2 CENTRE 4 FADE 8 NO_INCREASE 16 REVERSE_FADE 32 OVERRIDE
--                64 HARD_BRAKE 128 HALF_CAPS 256 LAUNCH_ACTIVE 512 SEL_OFF 1024 DSC_OFF
--                2048 DSC_NEUTRAL 4096 TS_ACTIVE
--   rasDse       1 provider reg. 2 component reg. 4 provider answered 8 |fb| >= 0.01 deg
--                16 slide weight < 1 32 hook error 64 DSC disabled 128 replica mismatch
--                256 provider not in service
--   rasDirCheck  1 plausible | 0 open | -1 rear direction implausible | -2 yaw sign implausible
-- Reads: tsaRsReqDeg / tsaLamp / tsaStage (trailerStab), dseSrc (gaugeData, previous frame).
--
-- Driver API: setMode("OFF"|"DSC"|"NODSC"|"ON"), getMode(), cycleMode().
-- Dev / QA API: getState(), setOverride(nil|"OFF"|deg), setYawRefEnabled(b),
-- setAssistEnabled(b), measureWheelAngles(), getParams().

local M = {}
M.type = "auxiliary"
-- after the DSE (CMU 50, yawControl 60, STMEstimate 80) and diffLock 86, before awdMode 90
M.defaultOrder = 87

local abs, min, max, exp, sqrt, atan2, floor = math.abs, math.min, math.max, math.exp, math.sqrt, math.atan2, math.floor
local tan = math.tan
local huge = math.huge
local deg, rad = math.deg, math.rad

local CFG = {} -- resolved configuration (init)
local ST = {}  -- runtime state
local REG = {} -- DSC hook registry

local PROV = {providerOrder = 10, name = "etk854hybrid_rasYawRef"}
local COMP = {componentOrderYawControl = 5, name = "etk854hybrid_rasYawComp"}

-- ---------------------------------------------------------------------------
-- helpers (shared copies: etk854hybrid_util)
-- ---------------------------------------------------------------------------

local U = require("controller/etk854hybrid/etk854hybrid_util")
local num = U.finNum -- finite number (strings via tonumber), else d
local flag = U.flag
local fin = U.fin
local clamp = U.clampMinMax -- min(max(x, a), b)
local lin = U.lin -- clamped linear scale (NaN x -> y0)

-- piecewise linear {kmh = {...}, val = {...}}, held at both ends
local function interp(tbl, x)
  local xs, ys = tbl.kmh, tbl.val
  local n = #xs
  if not (x > xs[1]) then return ys[1] end
  if x >= xs[n] then return ys[n] end
  for i = 2, n do
    if x <= xs[i] then
      local x0 = xs[i - 1]
      return ys[i - 1] + (ys[i] - ys[i - 1]) * (x - x0) / (xs[i] - x0)
    end
  end
  return ys[n]
end

local function lpf(prev, x, tau, dt)
  if tau <= 0 then return x end
  return prev + (x - prev) * (1 - exp(-dt / tau))
end

local function slew(cur, tgt, step)
  return cur + clamp(tgt - cur, -step, step)
end

local function softClip(x, L)
  if not (L > 0) then return 0 end
  return x / (1 + (abs(x) / L) ^ 4) ^ 0.25
end

local sgn = U.sgn
local sw = U.sw -- switch: boolean as is, number / string >= 0.5
local copy = U.copy

local function warn(msg)
  pcall(log, "W", "etk854hybrid_rearSteer", msg)
end

-- copy of the part's controller row and calibration section (fresh table per call)
local function defaults()
  return {
    -- controller row
    enabled = true, assist = false, maxCounterDeg = 3.0, inPhaseGain = 1.0, toeRatio = 0.9975,
    inputSource = "etk854hybrid_rasCmd", hydroNameL = "etk854hybrid_rasToe_RL", hydroNameR = "etk854hybrid_rasToe_RR",
    linkRatioPerUnit = 0.0167, actuatorDegPerUnit = 3.5,
    -- calibration section
    frontAngleLinDeg = 33.67, frontAngleCubDeg = 3.63, frontRackFactorRef = 0.14,
    gain = {
      NORMAL = {kmh = {0, 20, 30, 40, 50, 60, 70, 80, 100, 130}, val = {-0.12, -0.12, -0.11, -0.08, -0.04, 0, 0.04, 0.07, 0.10, 0.12}},
      SPORT = {kmh = {0, 20, 30, 40, 50, 60, 70, 80, 100, 130}, val = {-0.15, -0.15, -0.14, -0.10, -0.05, 0, 0.03, 0.05, 0.07, 0.08}},
      TRAILER = {kmh = {0, 20, 30, 40, 50, 60, 80, 100, 130}, val = {-0.05, -0.05, -0.03, 0, 0.04, 0.08, 0.12, 0.14, 0.15}}
    },
    counterCapDeg = {
      NORMAL = {kmh = {0, 25, 40, 60}, val = {3.0, 3.0, 1.5, 0.5}},
      SPORT = {kmh = {0, 25, 40, 60}, val = {3.0, 3.0, 1.5, 0.5}},
      TRAILER = {kmh = {0, 25, 40}, val = {1.0, 1.0, 0.5}}
    },
    inPhaseCapDeg = {NORMAL = 1.5, SPORT = 1.0, TRAILER = 1.5},
    inPhaseTauS = {NORMAL = 0.12, SPORT = 0.08, TRAILER = 0.15},
    rateDegS = {NORMAL = 15, SPORT = 20, TRAILER = 10},
    frontTauS = 0.03, blendS = 1.0, totalMaxDeg = 3.5, totalRateDegS = 20,
    hardBrake = {minKmh = 30, lo = 0.4, hi = 0.8, scale = 0.5},
    freezeKmh = 1.0, freezeFullKmh = 2.0,
    reverseFadeKmh = 15, reverseMaxKmh = 20,
    centreRateDegS = 3, faultRateDegS = 5, limitRateDegS = 4, limitHoldS = 0.3,
    -- NODSC: straight while the DSC intervenes
    dscNeutralRateDegS = 10, dscNeutralHoldS = 0.5,
    slide = {
      minKmh = 15, beta = 0.065, yawExcess = {dsc = 0.25, dtc = 0.35, off = 0.35},
      csYawMin = 0.15, csFrontMinDeg = 1.0, csOnS = 0.10, excessOnS = 0.05, destabOnS = 0.05,
      offS = 0.8, fadeOutPerS = 8, destabFadeOutPerS = 20, fadeInPerS = 1.0, fadeRateDegS = 20
    },
    yawRef = {
      enabled = true, providerOrder = 10, minDeg = 0.05, tauS = 0.15,
      consDownS = 0.05, consUpS = 0.5, consHoldS = 0.5, noProviderScale = 0.5, refreshS = 1.0
    },
    fb = {
      componentOrder = 5, kn = {dsc = 0.5, dtc = 0.35}, db = {dsc = 0.08, dtc = 0.14},
      kpMaxS = 0.12, maxDeg = 1.5, minKmh = 20, fullKmh = 30, betaGate = 0.06, tauS = 0.08,
      rateDegS = 20, requireDirCheck = true
    },
    dirCheck = {
      minKmh = 8, maxKmh = 30, minSteer = 0.15, minRearDeg = 0.3, minRearFrac = 0.8, minEffect = 0.035,
      settleS = 0.5, maxLatAcc = 3.0, maxSteerRate = 0.5, windowS = 1.0, windows = 3, yawSignFrac = 0.9
    },
    ts = {enabled = true, maxDeg = 1.0, tauS = 0.08, rateDegS = 10, minKmh = 45, assistDbAdd = 0.05},
    faultCheckS = 0.25,
    defaults = {wheelbase = 2.81, charSpeed = 55.0, maxLatAcc = 9.5}
  }
end

-- {kmh, val}: equal lengths, >= 2 points, strictly increasing kmh, finite
local function validCurve(t)
  if type(t) ~= "table" or type(t.kmh) ~= "table" or type(t.val) ~= "table" then return nil end
  local n = #t.kmh
  if n < 2 or #t.val ~= n then return nil end
  local xs, ys = {}, {}
  for i = 1, n do
    local x, y = num(t.kmh[i], nil), num(t.val[i], nil)
    if x == nil or y == nil then return nil end
    if i > 1 and not (x > xs[i - 1]) then return nil end
    xs[i], ys[i] = x, y
  end
  return {kmh = xs, val = ys}
end

-- Overlay the jbeam values src onto the defaults def (same shape). Non-negative defaults
-- stay >= 0; an invalid curve keeps the default (one warning).
local function parseInto(def, src, path)
  if type(src) ~= "table" then src = {} end
  for k, d in pairs(def) do
    local s = src[k]
    local td = type(d)
    if td == "number" then
      local x = num(s, d)
      def[k] = (d >= 0) and max(0, x) or x
    elseif td == "boolean" then
      def[k] = sw(s, d)
    elseif td == "string" then
      if type(s) == "string" and s ~= "" then def[k] = s end
    elseif td == "table" then
      if d.kmh then
        if s ~= nil then
          local c = validCurve(s)
          if c then
            def[k] = c
          else
            warn("invalid table " .. path .. k .. ", default used")
          end
        end
      else
        parseInto(d, s, path .. k .. ".")
      end
    end
  end
  return def
end

local SEL_TEXT = {
  OFF = "Rear-axle steering: OFF",
  DSC = "Rear-axle steering: ON + DSC",
  NODSC = "Rear-axle steering: ON (no DSC steering assist)"
}
local SEL_NEXT = {OFF = "DSC", DSC = "NODSC", NODSC = "OFF"}
local SEL_ALIAS = {OFF = "OFF", DSC = "DSC", NODSC = "NODSC", ON = "DSC"}

-- selection at spawn ($etk854_ras_enable / _assist)
local function defaultSel()
  if not CFG.enabled then return "OFF" end
  return CFG.assist and "DSC" or "NODSC"
end

local function newState()
  local d = CFG.defaults
  local wb = d.wheelbase > 0.5 and d.wheelbase or 2.81
  local vch = d.charSpeed > 1 and d.charSpeed or 55
  local sel = defaultSel()
  return {
    -- enabled: jbeam default only, sel is what acts
    enabled = CFG.enabled, configOK = false, mapFound = false, liveOK = false,
    sel = sel, dscOffActive = false, rasOff = (sel == "OFF"), dscInt = false, dnT = 0, dscNeutral = false,
    ffOn = (sel ~= "OFF"),
    hL = nil, hR = nil, whl = nil, rackScale = 1, wb = wb, invWB = 1 / wb, invSq = 1 / (vch * vch),
    override = nil, yawRefForce = nil, assistForce = nil, handBack = false, dm = nil,
    -- inputs (per frame)
    u = 0, uPrev = nil, uRate = huge, spd = 0, kmh = 0, vFwd = 0, reverse = false,
    park = false, ignOff = false, r = 0, beta = 0, dfMeasRad = 0, deMode = nil, trailer = false,
    dscKey = "dsc", dscOn = true, absOn = false, dscAct = false, tcsOn = false,
    launchHold = false, launchActive = false, brake = 0, cmuOK = false,
    -- calibration set
    mode = "NORMAL", modePrev = "NORMAL", blendW = 1,
    -- feed-forward
    dfModel = 0, dfF = 0, dfIp = 0, k = 0, hb = 1, capScale = 1, counterBranch = false,
    ffBlend = 0, rateFF = 15, ffTargetDeg = 0, ffOut = 0,
    -- slide / FADE
    csT = 0, excT = 0, destabT = 0, fadeHold = 0, fade = false, fadeByCs = false,
    slideConf = false, gFF = 1, consT = 0, conservative = false,
    -- feedback
    fbReqDeg = 0, fbLp = 0, fbOut = 0, fbAllowed = false,
    -- trailer stability share
    tsReq = 0, tsLp = 0, tsOut = 0, tsAllowed = false, tsaLamp = false, tsaStage = 0, stockDsc = false,
    betaGate = false,
    -- total
    targetDeg = 0, cmdDeg = 0, cmdU = 0, creditDeg = 0, drRefG = 0, actDeg = 0,
    niT = 0, noIncrease = false, centre = false, freezeOn = false, limitBits = 0,
    -- faults
    fault = "none", swFault = false, swErr = nil, msgShown = false, healthT = 0, refreshT = 0,
    -- direction check (latch kept across reset)
    dirCheck = 0, dcStreak = 0, dcStreakVal = 0, dcLast = 0, dcRho = 0, dcRhoCp = 0, dcRhoUs = 0, dcFrac = 0,
    dcWindows = 0, dcN = 0, dcT = 0, dcOkT = 0, dcSr = 0, dcSv = 0, dcSf = 0, dcSd = 0, dcSs = 0
  }
end

local function newRegistry()
  return {
    cmu = nil, yc = nil, stm = nil, ycPresent = false,
    provReg = false, compReg = false, provCount = 0, compCount = 0,
    off = false, offWhy = nil, compOff = false, hookErr = nil, maxLat = nil,
    t = 0, calls = 0, compCalls = 0, drRef = 0, w = 1, mismatch = 0, refreshT = -1e9,
    last = nil, lastStm = nil, answered = false
  }
end

local function provInService()
  return REG.provReg and not REG.off and not ((ST.dirCheck or 0) < 0)
end

local function dscOffNow()
  return ST.dscKey == "off" or ST.dscOn == false
end

local function rasOffNow()
  return ST.sel == "OFF" or dscOffNow()
end

local function assistOn()
  if ST.assistForce ~= nil then return ST.assistForce end
  return ST.sel == "DSC"
end

local function faultName()
  if not ST.configOK then return "CONFIG" end
  return ST.fault
end

-- ---------------------------------------------------------------------------
-- faults
-- ---------------------------------------------------------------------------

local function sendFaultMessage()
  guihooks.message("Rear-axle steering fault: rear wheels returned to straight ahead", 5, "vehicle.etk854hybrid.ras")
end

-- one message per fault episode (cleared on reset), one log line per fault
local function faultNotice(what)
  if not ST.msgShown then
    ST.msgShown = true
    pcall(sendFaultMessage)
  end
  warn("fault " .. tostring(what) .. ": rear wheels centred")
end

local function refreshMaxLat()
  local stm = REG.stm
  if not (stm and stm.getConfig) then return end
  local ok, cf = pcall(stm.getConfig)
  if ok and type(cf) == "table" then
    local x = num(cf.maxLateralAcceleration, nil)
    if x then REG.maxLat = x end
  end
end

-- ---------------------------------------------------------------------------
-- DSC yaw reference provider (100 Hz, yawControl.updateFixedStep)
-- ---------------------------------------------------------------------------

local function provCore(dt)
  local c, g, s = CFG, REG, ST
  local yr = c.yawRef
  dt = (type(dt) == "number" and dt > 0 and dt < 0.1) and dt or 0.01
  g.t = g.t + dt
  g.calls = g.calls + 1
  local cr = fin(s.creditDeg)
  g.drRef = g.drRef + (cr - g.drRef) * (1 - exp(-dt / yr.tauS)) -- lagged FF credit [deg]
  if s.conservative then
    g.w = max(0, g.w - dt / yr.consDownS)
  else
    g.w = min(1, g.w + dt / yr.consUpS)
  end
  if s.fault ~= "none" or not s.configOK or s.yawRefForce == false or not g.ycPresent then return nil end
  if rasOffNow() then return nil end
  if (s.dirCheck or 0) < 0 then return nil end -- in doubt: stock
  if abs(g.drRef) < yr.minDeg then return nil end -- centred: bit-identical stock
  local cmu = g.cmu
  if cmu.systemState == "offline" then return nil end
  local vd, vs = cmu.vehicleData, cmu.virtualSensors
  local st = vd.vehicleStats
  -- the arithmetic is exactly STMEstimate.calculateExpectedYaw
  local sp = vs.trustWorthiness.virtualSpeed >= 1 and abs(vs.virtual.speed) or abs(vs.virtual.wheelSpeed)
  local df = vd.frontWheelAngle
  local q = sp / (1 + (sp * sp * st.invSquaredCharacteristicSpeed))
  local xs = (abs(df) * st.invWheelBase) * q                  -- stock
  local xc = (abs(df - rad(g.drRef)) * st.invWheelBase) * q   -- corrected
  if xs ~= xs or xc ~= xc or xs == huge or xc == huge then error("non-finite yaw reference") end
  local cap = g.maxLat / (sp + 1e-30)
  local rStm, rCorr = min(xs, cap), min(xc, cap)
  local real = g.stm.calculateExpectedYaw(dt) -- pure: replica cross-check
  if type(real) ~= "number" or real ~= real or abs(real - rStm) > 1e-6 * max(1, abs(real)) then
    g.mismatch = g.mismatch + 1
    if g.mismatch >= 3 then
      if g.t - g.refreshT > yr.refreshS then
        refreshMaxLat()
        g.refreshT = g.t
        g.mismatch = 0
      else
        g.off = true
        g.offWhy = "replica mismatch"
        warn("yaw reference replica does not match STMEstimate, stock reference used from now on")
      end
    end
    return nil
  end
  g.mismatch = 0
  -- confirmed slide: w -> 0 (exactly stock). Not min(rStm, rCorr): an in-phase credit
  -- (rCorr < rStm) would keep the lowered reference and let the DSC act early.
  local out = rStm + g.w * (rCorr - rStm)
  g.last, g.lastStm, g.answered = out, rStm, true
  return out
end

-- yawControl.updateIsEnabled (every DSC mode change); never throws
PROV.setParameters = function(p)
  if type(p) == "table" and p.isEnabled ~= nil then ST.dscOn = (p.isEnabled == true) end
end

PROV.calculateExpectedYaw = function(speed, dt)
  if REG.off then return nil end
  local ok, res = pcall(provCore, dt)
  if ok then return res end
  REG.off = true
  REG.offWhy = "error: " .. tostring(res)
  warn("yaw reference provider disabled after an error, stock reference used: " .. tostring(res))
  return nil -- STMEstimate answers in the same step
end

-- ---------------------------------------------------------------------------
-- silent yaw component (assist; 100 Hz request, applied in GFX)
-- ---------------------------------------------------------------------------

local function compCore(r, yd)
  local s, c, g = ST, CFG, REG
  g.compCalls = g.compCalls + 1
  local key = s.dscKey
  if not assistOn() or rasOffNow() then
    s.fbReqDeg = 0
    return
  end
  local fb = c.fb
  local vs = g.cmu.virtualSensors
  local sp = vs.trustWorthiness.virtualSpeed >= 1 and abs(vs.virtual.speed) or abs(vs.virtual.wheelSpeed) -- as STMEstimate
  sp = num(sp, 0)
  local G = sp * s.invWB / (1 + sp * sp * s.invSq)
  local kp = min(fb.kpMaxS, fb.kn[key] / max(G, 0.5)) -- loop gain kn
  -- wider dead band while the trailer stability damps
  local e = yd - (fb.db[key] + ((s.tsaStage or 0) >= 1 and c.ts.assistDbAdd or 0))
  if not (e > 0) then -- also catches NaN
    s.fbReqDeg = 0
    return
  end
  -- rear toward the yaw = stabilising (r = G * (df - dr)); no understeer branch
  s.fbReqDeg = fin(((r >= 0) and 1 or -1) * min(deg(kp * e), fb.maxDeg))
end

COMP.setParameters = function(p)
  if type(p) == "table" and p["yawControl.isEnabled"] ~= nil then ST.dscOn = (p["yawControl.isEnabled"] == true) end
end

COMP.actAsYawControl = function(measuredYaw, expectedYaw, yawDifference, bodySlipAngle, frontSlipAngle, rearSlipAngle, dt)
  if not REG.compOff then
    local ok, err = pcall(compCore, measuredYaw, yawDifference)
    if not ok then
      REG.compOff = true
      REG.hookErr = tostring(err)
      ST.fbReqDeg = 0
      warn("yaw component disabled after an error: " .. tostring(err))
    end
  end
  return false
end

-- ---------------------------------------------------------------------------
-- per-frame steps (core)
-- ---------------------------------------------------------------------------

-- 1: BROKEN check every faultCheckS
local function healthStep(dt)
  local s = ST
  if not s.liveOK or s.fault ~= "none" then return end
  s.healthT = s.healthT + dt
  if s.healthT < CFG.faultCheckS then return end
  s.healthT = 0
  if obj:beamIsBroken(s.hL.beamCID) or obj:beamIsBroken(s.hR.beamCID) then
    s.fault = "BROKEN"
    faultNotice("BROKEN (toe link)")
  end
end

-- DSE drive-mode key: "off" | "dtc" | "dsc"
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

local function cmuSensors()
  local cmu = REG.cmu
  if not cmu or cmu.systemState == "offline" then return nil end
  local sh, vd, vs = cmu.sensorHub, cmu.vehicleData, cmu.virtualSensors
  if sh and vd and vs and sh.isActive == true and vd.isActive == true and vs.isActive == true then
    return sh, vd, vs
  end
  return nil
end

-- 2: inputs ("+" = right)
local function readInputs(dt)
  local s, ev = ST, electrics.values
  local u = clamp(num(ev.steering_input, 0), -1, 1)
  s.uRate = s.uPrev and abs(u - s.uPrev) / dt or huge
  s.uPrev, s.u = u, u
  local sp = num(ev.airspeed, nil)
  if sp == nil then sp = num(ev.wheelspeed, 0) end
  sp = abs(sp)
  s.spd, s.kmh = sp, sp * 3.6
  s.vFwd = num(obj:getVelocity():dot(obj:getDirectionVector()), 0)
  local gear = ev.gear
  if type(gear) ~= "string" then gear = "D" end
  s.park = gear == "P"
  s.reverse = gear == "R" or s.vFwd < -0.3
  s.ignOff = num(ev.ignitionLevel, 2) < 2
  local sh, vd, vs = cmuSensors()
  s.cmuOK = sh ~= nil
  if sh then
    s.r = num(sh.yawAVSmooth, 0)
    -- = yawControl's bodySlipAngleAdjusted
    s.beta = num(vs.virtual.bodySlipAngle, 0) * lin(num(vs.trustWorthiness.bodySlipAngle, 0), 0.5, 0.8, 0, 1)
      * (1 - lin(num(vs.virtual.speed, 0), 8, 10, 1, 0))
  else
    s.r = (obj.getYawAngularVelocity and num(obj:getYawAngularVelocity(), 0)) or 0
    s.beta = 0
  end
  s.deMode = ev.etk854hybrid_deMode
  s.trailer = flag(ev.etk854hybrid_trailerMode) or flag(ev.etk854hybrid_trailerAttached)
  s.dscKey = dscKeyOf()
  s.brake = num(ev.brake, 0)
  s.absOn = flag(ev.etk854hybrid_absQualified) or (flag(ev.absActive) and s.brake > 0.1)
  s.tcsOn = flag(ev.tcsActive)
  s.tsReq = num(ev.etk854hybrid_tsaRsReqDeg, 0)
  s.tsaLamp = flag(ev.etk854hybrid_tsaLamp)          -- held 0.15 s past its stage (escActive tail)
  s.tsaStage = num(ev.etk854hybrid_tsaStage, 0)
  local src = num(ev.etk854hybrid_dseSrc, 0)          -- bits 2 MTC yaw, 8 YC brake, 128 BC yaw
  local ycBrake = flag(ev.isYCBrakeActive)
  s.stockDsc = ycBrake or bit.band(floor(src), 138) ~= 0
  local esc = flag(ev.escActive)
  -- not for a trailer-stability-only lamp: must not freeze its own damping
  s.dscAct = ycBrake or (esc and not (s.tsaLamp and not s.stockDsc))
  s.dscInt = ycBrake or esc or s.tcsOn or flag(ev.isTCBrakeActive) or s.tsaLamp -- NODSC neutral
  local ls = ev.etk854hybrid_launchState
  s.launchHold = ls == "ARMED" or ls == "READY"
  s.launchActive = ls == "ACTIVE"
end

-- 2: selection / DSC state; gates the feed-forward (it ramps back at its own rate)
local function selStep(dt)
  local s = ST
  s.dscOffActive = dscOffNow()
  s.rasOff = s.sel == "OFF" or s.dscOffActive
  if s.dscInt then s.dnT = CFG.dscNeutralHoldS else s.dnT = max(0, s.dnT - dt) end
  s.dscNeutral = s.sel == "NODSC" and not s.rasOff and (s.dscInt or s.dnT > 0)
  s.ffOn = not (s.rasOff or s.dscNeutral)
end

-- 2: calibration set and cross-fade over blendS
local function modeStep(dt)
  local s = ST
  local mode = s.trailer and "TRAILER" or (s.deMode == "SPORT" and "SPORT" or "NORMAL")
  if mode ~= s.mode then
    s.modePrev = s.mode
    s.mode = mode
    s.blendW = 0
  end
  s.blendW = min(1, s.blendW + dt / CFG.blendS)
end

-- 3: front road-wheel angle model + filters
local function frontStep(dt)
  local s, c = ST, CFG
  local u = s.u
  s.dfModel = s.rackScale * (c.frontAngleLinDeg * u + c.frontAngleCubDeg * u * u * u)
  local m = nil
  if s.cmuOK then m = num(REG.cmu.vehicleData.frontWheelAngle, nil) end
  s.dfMeasRad = m or rad(s.dfModel)
  s.dfF = lpf(s.dfF, s.dfModel, c.frontTauS, dt)                -- counter-phase
  s.dfIp = lpf(s.dfIp, s.dfF, c.inPhaseTauS[s.mode], dt)        -- in-phase: rear follows the front
end

-- feed-forward law of one calibration set [deg, + right]
local function ffOf(m)
  local s, c = ST, CFG
  local k = interp(c.gain[m], s.kmh)
  if k > 0 then k = k * c.inPhaseGain end
  if k < 0 then
    local L = min(c.maxCounterDeg, interp(c.counterCapDeg[m], s.kmh)) * s.hb * s.capScale
    return softClip(k * s.dfF, L), k
  end
  return softClip(k * s.dfIp, c.inPhaseCapDeg[m] * s.capScale), k
end

-- 4: blended feed-forward, k, reverse fade
local function ffStep()
  local s, c = ST, CFG
  local hbc = c.hardBrake
  s.hb = (s.kmh > hbc.minKmh) and lin(s.brake, hbc.lo, hbc.hi, 1, hbc.scale) or 1
  s.capScale = (REG.ycPresent and not provInService()) and c.yawRef.noProviderScale or 1
  local fNew, kNew = ffOf(s.mode)
  local w = s.blendW
  local ff, k, rate, counter = fNew, kNew, c.rateDegS[s.mode], kNew < 0
  if w < 1 then
    local fOld, kOld = ffOf(s.modePrev)
    ff = fOld * (1 - w) + fNew * w
    k = kOld * (1 - w) + kNew * w
    rate = max(rate, c.rateDegS[s.modePrev])
    counter = counter or kOld < 0
  end
  if s.reverse then ff = ff * lin(s.kmh, c.reverseFadeKmh, c.reverseMaxKmh, 1, 0) end
  s.ffBlend, s.k, s.rateFF, s.counterBranch = ff, k, rate, counter
end

local function slideOff(dt)
  local s = ST
  s.csT, s.excT, s.destabT, s.fadeHold, s.consT = 0, 0, 0, 0, 0
  s.fade, s.fadeByCs, s.slideConf, s.conservative = false, false, false, false
  s.gFF = min(1, s.gFF + CFG.slide.fadeInPerS * dt)
end

-- 5: slide confirmation, FADE, conservative yaw reference
local function slideStep(dt)
  local s, c = ST, CFG
  local sl = c.slide
  if s.reverse or not (s.kmh > sl.minKmh) then return slideOff(dt) end
  local spd, r = s.spd, s.r
  local G = spd * s.invWB / (1 + spd * spd * s.invSq)
  local maxLat = REG.maxLat or c.defaults.maxLatAcc
  local rRef = min(abs(s.dfMeasRad - rad(s.drRefG)) * G, maxLat / (spd + 1e-30))
  local yawExc = abs(r) - rRef > sl.yawExcess[s.dscKey]
  local sr = sgn(r)
  local betaOS = sr * sgn(s.beta) < 0 and abs(s.beta) > sl.beta
  local conf = yawExc or betaOS
  local cs = conf and sr ~= 0 and sgn(s.dfModel) ~= sr and abs(r) > sl.csYawMin and abs(s.dfModel) > sl.csFrontMinDeg
  local destab = conf and abs(s.ffOut) > 0.05 and sgn(s.ffOut) ~= sr
  s.slideConf = conf
  s.csT = cs and (s.csT + dt) or 0
  s.excT = yawExc and (s.excT + dt) or 0
  s.destabT = destab and (s.destabT + dt) or 0
  local cause = s.csT >= sl.csOnS or s.excT >= sl.excessOnS or s.destabT >= sl.destabOnS
  s.fadeHold = cause and sl.offS or max(0, s.fadeHold - dt)
  s.fade = s.fadeHold > 0
  s.fadeByCs = (s.csT >= sl.csOnS) or (s.fade and s.fadeByCs)
  if s.fade then
    s.gFF = max(0, s.gFF - ((s.destabT >= sl.destabOnS) and sl.destabFadeOutPerS or sl.fadeOutPerS) * dt)
  else
    s.gFF = min(1, s.gFF + sl.fadeInPerS * dt)
  end
  s.consT = (conf or s.fade) and c.yawRef.consHoldS or max(0, s.consT - dt)
  s.conservative = s.consT > 0
end

-- 6a: feed-forward slew. While gated, ffOut decays no faster than limitStep centres the
-- command, so a short gate (e.g. the DSC key passing OFF) resumes bumpless.
local function ffSlewStep(dt)
  local s, c = ST, CFG
  s.ffTargetDeg = s.ffOn and s.gFF * s.ffBlend or 0
  local rate = s.fade and c.slide.fadeRateDegS or s.rateFF
  if s.rasOff then
    rate = min(rate, c.centreRateDegS)
  elseif s.dscNeutral then
    rate = min(rate, c.dscNeutralRateDegS)
  end
  s.ffOut = slew(s.ffOut, s.ffTargetDeg, rate * dt)
end

-- gate shared by the feedback shares (assist, trailer stability); reads s.betaGate, set by fbGfxStep
local function shareGate()
  local s, cmu = ST, REG.cmu
  return s.dscOn and s.dscKey ~= "off" and not s.rasOff and not s.dscNeutral and not s.reverse
    and not s.fadeByCs and not s.betaGate and not s.launchHold and not s.launchActive
    and not s.park and not s.ignOff and s.fault == "none" and s.override == nil
    and cmu ~= nil and cmu.systemState == "normal"
end

-- 6b: feedback, GFX part (request comes from COMP at 100 Hz)
local function fbGfxStep(dt)
  local s, c, g = ST, CFG, REG
  local fb = c.fb
  s.betaGate = sgn(s.r) * sgn(s.beta) < 0 and abs(s.beta) > fb.betaGate
  local allowed = assistOn() and g.compReg and not g.compOff
    and (not fb.requireDirCheck or s.dirCheck == 1) and s.kmh >= fb.minKmh and shareGate()
  s.fbAllowed = allowed and true or false
  local tgt = 0
  if s.fbAllowed then tgt = fin(s.fbReqDeg) * lin(s.kmh, fb.minKmh, fb.fullKmh, 0, 1) end
  s.fbLp = lpf(s.fbLp, tgt, fb.tauS, dt)
  s.fbOut = slew(s.fbOut, s.fbLp, fb.rateDegS * dt)
end

-- 6c: trailer stability share
local function tsStep(dt)
  local s, ts = ST, CFG.ts
  -- slide gates of the assist (fadeByCs, betaGate), not the feed-forward FADE
  local ok = ts.enabled and s.sel == "DSC" and s.dirCheck == 1 and s.kmh >= ts.minKmh and shareGate()
  s.tsAllowed = ok and true or false
  local tgt = s.tsAllowed and clamp(fin(s.tsReq), -ts.maxDeg, ts.maxDeg) or 0
  s.tsLp = lpf(s.tsLp, tgt, ts.tauS, dt)
  s.tsOut = slew(s.tsOut, s.tsLp, ts.rateDegS * dt)
end

-- 7: gates and limit states (lowest priority first, later rows override)
local function limitStep(dt)
  local s, c = ST, CFG
  local tm = c.totalMaxDeg
  local raw = s.ffOut + s.fbOut + s.tsOut
  -- finite check before any min/max (see header)
  if raw ~= raw or raw == huge or raw == -huge then error("non-finite rear steer target") end
  local tgt, rate, freeze = clamp(raw, -tm, tm), c.totalRateDegS, true
  local bits = 0
  if s.override == "OFF" then tgt = 0 end
  if s.launchActive then
    tgt = 0
    bits = bits + 256
  end
  s.centre = s.park or s.ignOff or s.launchHold
  if s.centre then
    tgt, rate, freeze = 0, c.centreRateDegS, false
    bits = bits + 2
  end
  if s.absOn or s.dscAct then s.niT = c.limitHoldS else s.niT = max(0, s.niT - dt) end
  s.noIncrease = s.absOn or s.dscAct or s.niT > 0
  local cmd = s.cmdDeg
  if s.noIncrease then
    if tgt * cmd <= 0 then
      tgt = 0
    elseif abs(tgt) > abs(cmd) then
      tgt = cmd
    end
    rate = min(rate, c.limitRateDegS)
    bits = bits + 8
  end
  if s.dscNeutral then
    tgt, rate, freeze = 0, c.dscNeutralRateDegS, false
    bits = bits + 2048
  end
  if s.sel == "OFF" then bits = bits + 512 end
  if s.dscOffActive then bits = bits + 1024 end
  if s.rasOff then tgt, rate, freeze = 0, c.centreRateDegS, false end
  local ov = s.override
  if type(ov) == "number" then tgt, rate, freeze = clamp(ov, -tm, tm), c.rateDegS[s.mode], false end
  if ov ~= nil then bits = bits + 32 end
  if s.fault ~= "none" then tgt, rate, freeze = 0, c.faultRateDegS, false end
  if s.handBack then freeze = false end
  local fs = 1
  if freeze then
    fs = lin(s.kmh, c.freezeKmh, c.freezeFullKmh, 0, 1) -- standstill freeze
    rate = rate * fs
  end
  s.targetDeg = tgt
  s.cmdDeg = slew(cmd, tgt, rate * dt)
  if s.handBack and abs(s.cmdDeg - tgt) < 0.05 then s.handBack = false end
  s.freezeOn = freeze and fs < 1
  if s.freezeOn and abs(s.cmdDeg - tgt) > 0.05 then bits = bits + 1 end
  if s.fade then bits = bits + 4 end
  if s.reverse and s.kmh > c.reverseFadeKmh then bits = bits + 16 end
  if s.hb < 1 and s.counterBranch then bits = bits + 64 end
  if s.capScale < 1 then bits = bits + 128 end
  if abs(s.tsOut) >= 0.01 then bits = bits + 4096 end
  s.limitBits = bits
end

-- 8: actuator command (finite-checked before the clamp)
local function writeCmd()
  local s = ST
  local x = s.cmdDeg / CFG.actuatorDegPerUnit
  if x ~= x or x == huge or x == -huge then error("non-finite rear steer command") end
  s.cmdU = s.configOK and clamp(x, -1, 1) or 0
  electrics.values.etk854hybrid_rasCmd = s.cmdU
end

-- 9: yaw reference credit = commanded angle without the feedback shares fbOut / tsOut
local function creditStep(dt)
  local s, c = ST, CFG
  local tm = c.totalMaxDeg
  local cmd = s.cmdDeg
  local cr = cmd - s.fbOut - s.tsOut
  -- Under NO_INCREASE or the total clamp only part of fbOut / tsOut reaches cmd, so the
  -- feed-forward share lies between cmd - fbOut - tsOut and cmd: take the point nearest ffOut
  -- (the plain difference would credit a blocked feedback and desensitise the DSC).
  if type(s.override) ~= "number" and (s.noIncrease or abs(s.ffOut + s.fbOut + s.tsOut) > tm) then
    cr = clamp(s.ffOut, min(cr, cmd), max(cr, cmd))
  end
  s.creditDeg = clamp(cr, -tm, tm)
  s.drRefG = lpf(s.drRefG, s.creditDeg, c.yawRef.tauS, dt) -- GFX twin of the provider's lag
end

-- One window: rho = mean|r| * L / (mean v * mean tan|df|), rear yaw share D = sum tan|dr| /
-- sum tan|df| (rhoCp = 1 + D), x = rho / rhoUs - 1 -> +1 if x >= D/2, -1 if x <= -D/2, -2 if
-- sgn(r) == sgn(u) in fewer than yawSignFrac of the samples; `windows` equal results latch.
-- The margin scales with D because a plant bias b is multiplicative: a correct mapping reads +1
-- for b >= (1 + D/2) / (1 + D), a reversed one only for b >= (1 + D/2) / (1 - D) (0.983 /
-- 1.054 at D = minEffect).
local function dirCheckEval()
  local s, d = ST, CFG.dirCheck
  local n = s.dcN
  local mr, mv, mf = s.dcSr / n, s.dcSv / n, s.dcSf / n
  local frac = s.dcSs / n
  local res, rho, rhoCp, rhoUs = 0, 0, 0, 0
  if frac < d.yawSignFrac then
    res = -2
  elseif mv > 0 and mf > 0 then
    rho = mr * s.wb / (mv * mf)
    rhoCp = 1 + s.dcSd / s.dcSf
    rhoUs = 1 / (1 + mv * mv * s.invSq)
    if rho >= rhoUs * (1 + 0.5 * (rhoCp - 1)) then
      res = 1
    elseif rho <= rhoUs * (1 - 0.5 * (rhoCp - 1)) then
      res = -1
    end
  end
  s.dcLast, s.dcRho, s.dcRhoCp, s.dcRhoUs, s.dcFrac = res, rho, rhoCp, rhoUs, frac
  s.dcWindows = s.dcWindows + 1
  if res == 0 then
    s.dcStreak, s.dcStreakVal = 0, 0
  elseif res == s.dcStreakVal then
    s.dcStreak = s.dcStreak + 1
  else
    s.dcStreakVal, s.dcStreak = res, 1
  end
  if res ~= 0 and s.dcStreak >= d.windows and s.dirCheck ~= res then
    local wasIn = REG.ycPresent and provInService()
    s.dirCheck = res
    if res == -1 then
      warn("rear steer direction implausible: check the hydro mapping sign")
    elseif res == -2 then
      warn("yaw-rate sign implausible")
    end
    if wasIn and not provInService() then -- once: the provider leaves service
      warn(string.format("direction check %d: DSC yaw reference set to stock, rear steer caps x%.2f",
        res, CFG.yawRef.noProviderScale))
    end
  end
end

-- 10b: direction check sampling: steady counter-phase steering at minKmh..maxKmh, no ABS /
-- DSC / TCS / FADE. The rear angle gate is relative to the target (actuator settled), so the
-- small TRAILER-set angles qualify too.
local function dirCheckStep(dt)
  local s, d = ST, CFG.dirCheck
  local u, a, r = s.u, s.actDeg, s.r
  local tr, tf = tan(abs(rad(a))), tan(abs(s.dfMeasRad))
  local ok = not s.reverse and s.kmh >= d.minKmh and s.kmh <= d.maxKmh and abs(u) >= d.minSteer
    and abs(a) >= max(d.minRearDeg, d.minRearFrac * abs(s.targetDeg)) and sgn(a) == -sgn(u)
    and tr >= d.minEffect * tf and abs(r) * s.spd <= d.maxLatAcc
    and s.uRate <= d.maxSteerRate and not s.absOn and not s.dscAct and not s.tcsOn and not s.fade
  if not ok then
    s.dcN, s.dcT, s.dcOkT = 0, 0, 0 -- discard the window, restart the settle time
    return
  end
  if s.dcOkT < d.settleS then -- yaw build-up after a turn-in: not sampled
    s.dcOkT = s.dcOkT + dt
    return
  end
  if s.dcN == 0 then s.dcSr, s.dcSv, s.dcSf, s.dcSd, s.dcSs = 0, 0, 0, 0, 0 end
  s.dcN = s.dcN + 1
  s.dcT = s.dcT + dt
  s.dcSr = s.dcSr + abs(r)
  s.dcSv = s.dcSv + s.spd
  s.dcSf = s.dcSf + tf
  s.dcSd = s.dcSd + tr
  if sgn(r) == sgn(u) then s.dcSs = s.dcSs + 1 end
  if s.dcT >= d.windowS then
    dirCheckEval()
    s.dcN, s.dcT = 0, 0
  end
end

-- actual rear angle from the hydro states (toe is common-mode)
local function actualDeg()
  local s, c = ST, CFG
  return fin((s.hL.state - s.hR.state) / (2 * c.linkRatioPerUnit) * c.actuatorDegPerUnit)
end

-- 10: actual angle, direction check, maxLat refresh (driveModes)
local function measureStep(dt)
  local s, c = ST, CFG
  if s.liveOK then
    s.actDeg = actualDeg()
  else
    s.actDeg = 0
  end
  dirCheckStep(dt)
  if REG.provReg then
    s.refreshT = s.refreshT + dt
    if s.refreshT >= c.yawRef.refreshS then
      s.refreshT = 0
      refreshMaxLat()
    end
  end
end

local function stateName()
  local s = ST
  if s.fault ~= "none" then return "FAULT" end
  if not s.configOK then return "OFF" end
  if s.rasOff and type(s.override) ~= "number" then return "OFF" end
  if s.fade or s.noIncrease or s.dscNeutral then return "LIMIT" end
  local big = abs(s.cmdDeg) >= 0.05
  if s.freezeOn and big then return "HOLD" end
  if big then return "ACTIVE" end
  return "READY"
end

local function dseBits()
  local g, s = REG, ST
  local b = 0
  if g.provReg then b = b + 1 end
  if g.compReg then b = b + 2 end
  if g.answered then b = b + 4 end
  if abs(fin(s.fbOut)) >= 0.01 then b = b + 8 end
  if g.w < 1 then b = b + 16 end
  if (g.off and g.offWhy ~= "replica mismatch") or g.compOff then b = b + 32 end
  if s.dscOn == false then b = b + 64 end
  if g.off and g.offWhy == "replica mismatch" then b = b + 128 end
  if g.ycPresent and not provInService() then b = b + 256 end
  return b
end

-- 11: every channel except rasCmd (written by writeCmd / the fault paths)
local function publish()
  local s, ev = ST, electrics.values
  ev.etk854hybrid_rasAngleDeg = fin(s.actDeg)
  ev.etk854hybrid_rasTargetDeg = fin(s.targetDeg)
  ev.etk854hybrid_rasFfDeg = fin(s.ffOut)
  ev.etk854hybrid_rasFbDeg = fin(s.fbOut)
  ev.etk854hybrid_rasTsDeg = fin(s.tsOut)
  ev.etk854hybrid_rasGain = fin(s.k)
  ev.etk854hybrid_rasState = stateName()
  ev.etk854hybrid_rasSel = s.sel
  ev.etk854hybrid_rasMode = s.mode
  ev.etk854hybrid_rasFault = faultName()
  ev.etk854hybrid_rasLimit = s.limitBits
  ev.etk854hybrid_rasDse = dseBits()
  ev.etk854hybrid_rasDirCheck = s.dirCheck
  REG.answered = false
end

-- CONFIG: the command stays 0, state OFF / CONFIG
local function publishOff()
  local ev = electrics.values
  ev.etk854hybrid_rasCmd = 0
  ev.etk854hybrid_rasAngleDeg = 0
  ev.etk854hybrid_rasTargetDeg = 0
  ev.etk854hybrid_rasFfDeg = 0
  ev.etk854hybrid_rasFbDeg = 0
  ev.etk854hybrid_rasTsDeg = 0
  ev.etk854hybrid_rasGain = 0
  ev.etk854hybrid_rasState = "OFF"
  ev.etk854hybrid_rasSel = ST.sel
  ev.etk854hybrid_rasMode = ST.mode or "NORMAL"
  ev.etk854hybrid_rasFault = "CONFIG"
  ev.etk854hybrid_rasLimit = 0
  ev.etk854hybrid_rasDse = dseBits()
  ev.etk854hybrid_rasDirCheck = ST.dirCheck or 0
  REG.answered = false
end

local function publishAny()
  if ST.configOK then publish() else publishOff() end
end

local function core(dt)
  dt = min(dt, 0.1)
  if not (dt > 0) then return end
  healthStep(dt)       -- 1
  readInputs(dt)       -- 2
  selStep(dt)
  modeStep(dt)
  frontStep(dt)        -- 3
  ffStep()             -- 4
  slideStep(dt)        -- 5
  ffSlewStep(dt)       -- 6
  fbGfxStep(dt)
  tsStep(dt)
  limitStep(dt)        -- 7
  writeCmd()           -- 8
  creditStep(dt)       -- 9
  measureStep(dt)      -- 10
  publish()            -- 11
end

-- essential fault writes: arithmetic only
local function faultCore(dt)
  local s, c = ST, CFG
  local x = s.cmdDeg
  if x ~= x or abs(x) == huge then x = 0 end
  local h = min(dt, 0.1)
  if not (h > 0) then h = 0 end
  x = slew(x, 0, c.faultRateDegS * h)
  s.cmdDeg = x
  local ev = electrics.values
  ev.etk854hybrid_rasCmd = clamp(x / c.actuatorDegPerUnit, -1, 1)
  ev.etk854hybrid_rasState = "FAULT"
  ev.etk854hybrid_rasFault = s.fault
  ev.etk854hybrid_rasSel = s.sel
end

-- secondary channels in the fault path (a failure here is ignored)
local function faultExtra()
  local s = ST
  s.targetDeg, s.ffOut, s.fbOut, s.k, s.limitBits = 0, 0, 0, 0, 0
  s.tsOut, s.tsLp = 0, 0
  if s.liveOK then s.actDeg = actualDeg() end
  publish()
end

local function faultPath(dt)
  if not pcall(faultCore, dt) then electrics.values.etk854hybrid_rasCmd = 0 end
  pcall(faultExtra)
end

-- ---------------------------------------------------------------------------
-- lifecycle
-- ---------------------------------------------------------------------------

-- exactly 2 hydros on CFG.inputSource, named hydroNameL / hydroNameR
local function resolveHydros()
  local c, s = CFG, ST
  local hs = v.data.hydros
  if type(hs) ~= "table" then return "no hydros in the vehicle" end
  local list = {}
  for _, h in pairs(hs) do
    if type(h) == "table" and h.inputSource == c.inputSource then list[#list + 1] = h end
  end
  if #list ~= 2 then
    return string.format("%d hydros with inputSource %q (need exactly 2)", #list, c.inputSource)
  end
  local beams = v.data.beams
  local hL, hR
  for i = 1, 2 do
    local h = list[i]
    local name = h.name
    if name == nil and type(beams) == "table" and h.beamCID ~= nil then
      local b = beams[h.beamCID]
      name = type(b) == "table" and b.name or nil
    end
    if name == c.hydroNameL then hL = h elseif name == c.hydroNameR then hR = h end
  end
  if not (hL and hR) then
    return string.format("hydro names are not %q / %q", c.hydroNameL, c.hydroNameR)
  end
  if type(hL.beamCID) ~= "number" or type(hR.beamCID) ~= "number" then return "hydro beam cid missing" end
  s.hL, s.hR = hL, hR
  return nil
end

-- front rack: (outLimit - inLimit) / 2 of the steering hydros vs the reference
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
  if f > 0.05 and f < 0.5 and CFG.frontRackFactorRef > 0 then return f / CFG.frontRackFactorRef end
  return 1
end

local function wheelRefs()
  local w = ST.whl
  if w then return w end
  w = {}
  local ws = wheels and wheels.wheels
  if type(ws) == "table" then
    for _, wd in pairs(ws) do
      local n = type(wd) == "table" and wd.name
      if n == "FL" or n == "FR" or n == "RL" or n == "RR" then w[n] = wd end
    end
  end
  if w.FL and w.FR and w.RL and w.RR then
    ST.whl = w
    return w
  end
  return nil
end

local function init(jbeamData)
  local c = parseInto(defaults(), jbeamData, "")
  c.maxCounterDeg = clamp(c.maxCounterDeg, 0, 3.5)
  c.inPhaseGain = clamp(c.inPhaseGain, 0, 1.5)
  c.actuatorDegPerUnit = clamp(c.actuatorDegPerUnit, 2.5, 4.5)
  c.linkRatioPerUnit = clamp(c.linkRatioPerUnit, 0.01, 0.03)
  c.dirCheck.windows = max(1, floor(c.dirCheck.windows + 0.5))
  CFG = c
  PROV.providerOrder = c.yawRef.providerOrder
  COMP.componentOrderYawControl = c.fb.componentOrder
  ST = newState()
  REG = newRegistry()

  local ok, why = pcall(resolveHydros)
  if ok and not why then
    ST.configOK, ST.mapFound = true, true
  else
    ST.configOK, ST.mapFound = false, false
    log("E", "etk854hybrid_rearSteer", "CONFIG: " .. tostring(why) .. " - rear-axle steering disabled, command held at 0")
  end
  local okR, rs = pcall(rackScaleOf)
  ST.rackScale = (okR and num(rs, 1)) or 1
  pcall(wheelRefs) -- optional: only measureWheelAngles needs the wheels
  -- never call another controller here (hash-order init)
  electrics.values.etk854hybrid_rasCmd = 0
  publishAny()
end

-- Discovery + registration here: yawControl sorts its providers / components only in its
-- initLastStage. Registered for every spawn selection, the driver can switch at run time.
local function discover()
  local s, c, g = ST, CFG, REG
  if not s.mapFound then return end
  local list = controller.getControllersByType("drivingDynamics/CMU")
  local cmu = (type(list) == "table" and list[1]) or controller.getController("CMU")
  local yc = cmu and cmu.getSupervisor and cmu.getSupervisor("yawControl")
  local stm = cmu and cmu.getSupervisor and cmu.getSupervisor("yawProviders/STMEstimate")
  g.cmu, g.yc, g.stm = cmu or nil, yc or nil, stm or nil
  g.ycPresent = (yc ~= nil and yc ~= false and yc.isActive == true)
  if g.ycPresent and stm and stm.isActive == true then
    local ok, cf = pcall(stm.getConfig)
    g.maxLat = (ok and type(cf) == "table" and num(cf.maxLateralAcceleration, nil)) or nil
    if g.maxLat and c.yawRef.enabled and not g.provReg then
      yc.registerYawProvider(PROV)
      g.provReg = true
      g.provCount = g.provCount + 1
    end
    if g.maxLat and not g.compReg then -- inert unless assistOn()
      yc.registerComponent(COMP)
      g.compReg = true
      g.compCount = g.compCount + 1
    end
  end
end

local function initSecondStage(jbeamData)
  local ok, err = pcall(discover)
  if not ok then
    REG.off = true
    REG.offWhy = "discovery error: " .. tostring(err)
    warn("DSC hook discovery failed, stock yaw reference kept: " .. tostring(err))
  end
end

local function checkSide(h, slope)
  if h.inputFactor ~= 1 then return "inputFactor " .. tostring(h.inputFactor) end
  local mo, mi, ce = h.multOut, h.multIn, h.center
  if type(mo) ~= "number" or type(mi) ~= "number" or type(ce) ~= "number" or type(h.cOut) ~= "number" then
    return "hydro not initialised"
  end
  if not (abs(mo - mi) < 1e-9) then return "multOut ~= multIn" end
  if not (abs(mo - slope) < 1e-6) then return string.format("slope %.6f, expected %.6f", mo, slope) end
  if not (abs(ce - 1) < 1e-9) then return string.format("center %.9f, expected 1", ce) end
  return nil
end

-- live mapping check after hydros.init
local function liveCheck()
  local c, s = CFG, ST
  local F = c.linkRatioPerUnit
  local e = checkSide(s.hL, F)
  if e then return "left " .. e end
  e = checkSide(s.hR, -F)
  if e then return "right " .. e end
  local T = s.hL.cOut
  if not (abs(T - s.hR.cOut) < 1e-9) then return "toe differs left / right" end
  if not (T >= 0.985 and T <= 1.015) then return string.format("toe %.6f outside [0.985, 1.015]", T) end
  if not (abs(T - c.toeRatio) < 1e-6) then return string.format("toe %.6f ~= toeRatio %.6f", T, c.toeRatio) end
  return nil
end

local function readStats()
  local s = ST
  local cmu = REG.cmu
  local vd = cmu and cmu.vehicleData
  local st = vd and vd.vehicleStats
  if type(st) == "table" then
    local iw, iq = num(st.invWheelBase, 0), num(st.invSquaredCharacteristicSpeed, 0)
    if iw > 0.15 and iw < 1 then
      s.invWB = iw
      s.wb = 1 / iw
    end
    if iq > 1e-7 and iq < 0.04 then s.invSq = iq end
  end
end

local function initLastStage(jbeamData)
  local s, g = ST, REG
  if s.mapFound then
    local ok, why = pcall(liveCheck)
    if ok and not why then
      s.liveOK = true
    else
      s.configOK, s.mapFound, s.liveOK = false, false, false
      log("E", "etk854hybrid_rearSteer", "CONFIG: hydro mapping check failed (" .. tostring(why) .. ") - rear-axle steering disabled, command held at 0")
    end
  end
  pcall(readStats)
  -- the CMU all-active shutdown runs in CMU.initLastStage
  if g.cmu and (g.cmu.systemState == "offline" or not (g.yc and g.yc.isActive == true)) then g.ycPresent = false end
  if s.configOK then
    log("I", "etk854hybrid_rearSteer", string.format(
      "rear-axle steering ready: toe=%.4f F=%.4f deg/unit=%.2f rack=%.3f yawRef=%s comp=%s sel=%s (default %s) mode=%s",
      num(s.hL.cOut, CFG.toeRatio), CFG.linkRatioPerUnit, CFG.actuatorDegPerUnit, s.rackScale,
      tostring(g.provReg), tostring(g.compReg), tostring(s.sel), defaultSel(), s.mode))
  end
  electrics.values.etk854hybrid_rasCmd = 0
  publishAny()
end

local function reset(jbeamData)
  local s, g = ST, REG
  s.cmdDeg, s.cmdU, s.ffOut, s.fbOut, s.fbLp, s.fbReqDeg = 0, 0, 0, 0, 0, 0
  s.dfF, s.dfIp, s.creditDeg, s.drRefG, s.targetDeg, s.actDeg = 0, 0, 0, 0, 0, 0
  s.ffBlend, s.ffTargetDeg, s.k = 0, 0, 0
  s.gFF = 1
  s.csT, s.excT, s.destabT, s.fadeHold, s.consT, s.niT, s.healthT, s.refreshT = 0, 0, 0, 0, 0, 0, 0, 0
  s.fade, s.fadeByCs, s.slideConf, s.conservative = false, false, false, false
  s.noIncrease, s.freezeOn, s.centre, s.fbAllowed, s.limitBits = false, false, false, false, 0
  s.tsReq, s.tsLp, s.tsOut, s.betaGate = 0, 0, 0, false
  s.tsAllowed, s.tsaLamp, s.tsaStage, s.stockDsc = false, false, 0, false -- re-read every frame
  s.fault, s.swFault, s.swErr, s.handBack, s.msgShown = "none", false, nil, false, false
  s.modePrev, s.blendW = s.mode, 1
  s.uPrev, s.uRate = nil, huge
  s.dnT, s.dscNeutral = 0, false
  s.dcN, s.dcT, s.dcOkT = 0, 0, 0 -- the dirCheck latch is a hardware property and stays
  -- kept: sel, override, yawRefForce, assistForce, dirCheck, configOK, mapFound
  g.drRef, g.w, g.mismatch, g.answered = 0, 1, 0, false
  -- kept: registrations, counts, off latches (never re-register)
  electrics.values.etk854hybrid_rasCmd = 0
  publishAny()
end

local function resetLastStage()
  electrics.values.etk854hybrid_rasCmd = 0
  publishAny()
end

local function updateGFX(dt)
  if not ST.configOK then
    publishOff()
    return
  end
  if ST.swFault then
    faultPath(dt)
    return
  end
  local ok, err = pcall(core, dt)
  if not ok then
    ST.swFault = true
    ST.fault = "SOFTWARE"
    ST.swErr = tostring(err)
    faultNotice("SOFTWARE (" .. ST.swErr .. ")")
    faultPath(dt)
  end
end

-- ---------------------------------------------------------------------------
-- dev / QA API
-- ---------------------------------------------------------------------------

local function getState()
  local s, g, c = ST, REG, CFG
  local hL, hR = s.hL, s.hR
  local map = {F = c.linkRatioPerUnit, toe = nil}
  if hL and hR then
    map.toe = hL.cOut
    map.slopeL, map.slopeR = hL.multOut, hR.multOut
    map.center = hL.center
    map.stateL, map.stateR = hL.state, hR.state
    map.cidL, map.cidR = hL.beamCID, hR.beamCID
    if type(hL.state) == "number" and type(hR.state) == "number" then
      map.commonModeErr = (hL.state + hR.state) * 0.5 - num(hL.cOut, c.toeRatio)
    end
  end
  return {
    enabled = s.enabled, configOK = s.configOK, state = s.swFault and "FAULT" or stateName(), fault = faultName(),
    sel = s.sel, dscOffActive = s.dscOffActive, dscNeutral = s.dscNeutral,
    mode = s.mode, modePrev = s.modePrev, blendW = s.blendW, dscKey = s.dscKey, dscOn = s.dscOn,
    kmh = s.kmh, reverse = s.reverse, u = s.u, dfDeg = s.dfModel, k = s.k,
    ffTargetDeg = s.ffTargetDeg, ffOutDeg = s.ffOut, fbReqDeg = s.fbReqDeg, fbOutDeg = s.fbOut,
    targetDeg = s.targetDeg, cmdDeg = s.cmdDeg, actDeg = s.actDeg, cmd = s.cmdU, gFF = s.gFF,
    fade = s.fade, fadeByCs = s.fadeByCs, slideConf = s.slideConf, conservative = s.conservative,
    noIncrease = s.noIncrease, freeze = s.freezeOn, limitBits = s.limitBits, override = s.override,
    handBack = s.handBack, yawRefForce = s.yawRefForce, assistOn = assistOn(), creditDeg = s.creditDeg,
    swErr = s.swErr,
    tsReqDeg = s.tsReq, tsOutDeg = s.tsOut, tsAllowed = s.tsAllowed, stockDsc = s.stockDsc, tsaLamp = s.tsaLamp,
    betaGate = s.betaGate,
    dirCheck = {state = s.dirCheck, streak = s.dcStreak, lastResult = s.dcLast, lastRho = s.dcRho,
                lastRhoCp = s.dcRhoCp, lastRhoUs = s.dcRhoUs, lastYawSignFrac = s.dcFrac, windows = s.dcWindows},
    map = map,
    provider = {registered = g.provReg, count = g.provCount, inService = provInService() and true or false,
                off = g.off, offWhy = g.offWhy, mismatch = g.mismatch, calls = g.calls, w = g.w,
                drRefDeg = g.drRef, last = g.last, lastStm = g.lastStm, maxLat = g.maxLat},
    component = {registered = g.compReg, count = g.compCount, off = g.compOff, calls = g.compCalls}
  }
end

-- nil = automatic (hand-back after a number), "OFF" = target 0, number = fixed angle [deg]
-- (ignores freeze, FADE, NO_INCREASE; FAULT wins). Kept across reset, cleared on reload.
local function setOverride(x)
  local s = ST
  local res = nil
  if type(x) == "string" and x:upper() == "OFF" then
    res = "OFF"
  elseif type(x) == "number" and num(x, nil) then
    res = clamp(x, -CFG.totalMaxDeg, CFG.totalMaxDeg)
  end
  if res == nil and type(s.override) == "number" then s.handBack = true end
  if type(res) == "number" then s.handBack = false end
  s.override = res
  return res
end

-- false: the provider answers nil (stock reference), caps unchanged; nil = jbeam
local function setYawRefEnabled(b)
  if b == true or b == false then ST.yawRefForce = b else ST.yawRefForce = nil end
  return ST.yawRefForce
end

-- feedback A/B (needs the component registered); nil = follow the selection
local function setAssistEnabled(b)
  if b == true or b == false then ST.assistForce = b else ST.assistForce = nil end
  return (assistOn() and REG.compReg) and true or false
end

-- ---------------------------------------------------------------------------
-- driver selection (Actions app row R-STEER)
-- ---------------------------------------------------------------------------

local function selMessage()
  local txt = SEL_TEXT[ST.sel]
  local ok, key = pcall(dscKeyOf) -- fresh: the UI may call between frames
  if (ok and key == "off") or ST.dscOn == false then txt = txt .. " - inactive while DSC OFF" end
  guihooks.message(txt, 3, "vehicle.etk854hybrid.ras")
end

-- "OFF" | "DSC" | "NODSC" | "ON" (= DSC), case-insensitive; anything else is ignored.
-- Returns the selection in force.
local function setMode(x)
  local sel = (type(x) == "string") and SEL_ALIAS[x:upper()] or nil
  if sel and sel ~= ST.sel then
    ST.sel = sel
    pcall(selMessage)
  end
  return ST.sel
end

local function getMode()
  return ST.sel
end

-- OFF -> DSC -> NODSC -> OFF
local function cycleMode()
  return setMode(SEL_NEXT[ST.sel] or "DSC")
end

-- vehicle state serialisation (controller.lua onSerialize / onDeserialize)
local function serialize()
  return {sel = ST.sel}
end

local function deserialize(data)
  if type(data) == "table" and type(data.sel) == "string" and SEL_TEXT[data.sel] then ST.sel = data.sel end
end

-- signed road-wheel angles [deg, + = right]; dev / QA only, not used for control
local function measureCore()
  local w = wheelRefs()
  if not w then error("wheels FL/FR/RL/RR not found") end
  local rn = v.data.refNodes[0]
  local pRef = obj:getNodePosition(rn.ref)
  local fwd = (pRef - obj:getNodePosition(rn.back)):normalized()
  local up = (obj:getNodePosition(rn.up) - pRef):normalized()
  local left = up:cross(fwd):normalized() -- as sensorHub directionVectorLeft
  local out = {}
  for name, wd in pairs(w) do
    local a = obj:getNodePosition(wd.node1) - obj:getNodePosition(wd.node2)
    if a:dot(left) < 0 then a = -a end -- spin axis pointing vehicle-left
    out[name] = deg(atan2(a:dot(fwd), a:dot(left)))
  end
  return out
end

local function measureWheelAngles()
  local ok, res = pcall(measureCore)
  if ok then return res end
  return nil, tostring(res)
end

local function getParams()
  local p = copy(CFG) -- enabled / assist = spawn defaults, not the live state
  p.toe = CFG.toeRatio
  p.F = CFG.linkRatioPerUnit
  p.degPerUnit = CFG.actuatorDegPerUnit
  p.rackScale = ST.rackScale
  p.defaultSel = defaultSel()
  return p
end

M.init = init
M.initSecondStage = initSecondStage
M.initLastStage = initLastStage
M.reset = reset
M.resetLastStage = resetLastStage
M.updateGFX = updateGFX
M.serialize = serialize
M.deserialize = deserialize

M.setMode = setMode
M.getMode = getMode
M.cycleMode = cycleMode

M.getState = getState
M.setOverride = setOverride
M.setYawRefEnabled = setYawRefEnabled
M.setAssistEnabled = setAssistEnabled
M.measureWheelAngles = measureWheelAngles
M.getParams = getParams

return M
