-- ETK 854 ActiveHybrid - active rear-axle steering (RAS)
--
-- Instantiated by the part etk854hybrid_suspension_R_ras
-- (etk854hybrid_suspension_R_ras.jbeam, existing slot etk800_suspension_R).
-- The two toe links of the stock rear axle are hydros that share ONE command
-- channel, etk854hybrid_rasCmd in [-1, 1] (+ = rear wheels steer RIGHT):
--   ratio_L = toe + F * cmd,  ratio_R = toe - F * cmd
-- with toe = $toe_RR and F = linkRatioPerUnit (~3.5 deg road-wheel angle per
-- unit). A command of 0 is exactly the stock static toe, even if this
-- controller never runs.
--
-- Physics (linear single-track model, "+" = RIGHT for every angle and yaw rate):
--   r = G(v) * (df - dr),   G(v) = v / (L * (1 + v^2 / vch^2))
-- df / dr = front / rear road-wheel angle, L wheelbase, vch characteristic
-- speed. The feed-forward law is dr = k(v) * df:
--   k < 0 (counter-phase) below ~60 km/h: the turning centre moves forward,
--     the turning circle shrinks and the car turns in with less steering;
--   k > 0 (in-phase) above: the rear axle builds side force together with
--     the front, yaw gain and body slip drop (calm lane changes).
-- k(v) is a continuous piecewise-linear table per calibration set (NORMAL /
-- SPORT / TRAILER, zero crossing 60 / 60 / 40 km/h), so there is no step.
-- Limits are soft (x / (1 + (|x|/L)^4)^0.25); the in-phase share follows a
-- lagged front angle (front first, rear follows); a set change cross-fades
-- over 1 s.
--
-- Driver selection (Actions app row R-STEER, setMode / cycleMode), kept across
-- a vehicle reset and serialised across a reload; the default at spawn comes
-- from the jbeam ($etk854_ras_enable 0 -> OFF, else $etk854_ras_assist 1 ->
-- DSC, 0 -> NODSC):
--   OFF    rear wheels straight (3 deg/s, also at a standstill), feed-forward
--          and assist off, the DSC yaw provider answers nil (stock reference):
--          the car drives like the model without rear-axle steering.
--   DSC    feed-forward + stability assist (silent yaw component) + corrected
--          DSC yaw reference; DSC / ABS braking -> NO_INCREASE (unchanged).
--   NODSC  feed-forward + corrected reference, no assist. While the DSC
--          intervenes (escActive / tcsActive / isYCBrakeActive /
--          isTCBrakeActive) and for 0.5 s after its last frame the rear wheels
--          return to straight at 10 deg/s (feed-forward gated, freeze
--          ignored), then the normal law and rates resume. ABS alone ->
--          NO_INCREASE as before.
--   DSC OFF (drive mode "off" or yawControl disabled) overrides every
--   selection: exactly like OFF. Priority: FAULT > numeric dev override >
--   OFF / DSC OFF > NODSC neutral > launch / CENTRE / NO_INCREASE.
--
-- Safe states: rear wheels centre in P, with the ignition off and in the
-- launch-control hold (3 deg/s); no motion below 1 km/h (no tyre wind-up);
-- ABS / DSC braking -> the angle may only shrink (NO_INCREASE); a confirmed
-- slide fades the feed-forward to 0 (FADE); a broken toe link or a Lua error
-- centres the command at 5 deg/s (FAULT). The target sum (feed-forward +
-- feedback) and the actuator command (cmdDeg / actuatorDegPerUnit) are
-- finite-checked BEFORE they are clamped: in LuaJIT math.min(NaN, x) == x,
-- and hydros.lua:320 clamps a NaN input to inputInLimit (full deflection)
-- without an error. A failed check raises inside core -> SOFTWARE fault.
--
-- DSC integration (only the game's own extension points, no base file is
-- touched):
--   * Yaw reference provider: a plain table with providerOrder 10, registered
--     with yawControl in initSecondStage. yawControl asks its providers in
--     order every fixed step and takes the first non-nil answer
--     (supervisors/yawControl.lua:47-53); order 10 sorts ahead of
--     STMEstimate's 20 (STMEstimate.lua:8, sorted in yawControl.initLastStage
--     :154-170). The answer is STMEstimate's own model (STMEstimate.lua:23-38)
--     evaluated with (df - dr_credit), dr_credit = the commanded non-feedback
--     rear angle lagged by 0.15 s. Without it the stock reference would take
--     the smaller counter-phase yaw demand for oversteer. It answers nil (=
--     stock STMEstimate in the same step) when the rear is centred, on any
--     fault, when the live replica of STMEstimate disagrees with the real
--     one, with the selection OFF or DSC OFF, or once the direction check has
--     latched a negative result
--     (rasDirCheck -1 rear steer direction / -2 yaw-rate sign implausible).
--     In doubt stock: with a negative dirCheck the provider also counts as
--     not in service (rasDse bit 256), so the feed-forward caps drop to
--     yawRef.noProviderScale (0.5) exactly as without a provider; one log
--     line (W) when that happens. dirCheck itself is unchanged: kept across
--     reset, cleared on reload, never a FAULT; 0 and 1 change nothing here.
--     Principle: in a confirmed slide the DSC uses exactly the stock
--     reference (rear steer is ignored by the reference); outside slides it
--     uses the rear-steer-corrected reference. The answer is
--     rStm + w * (rCorr - rStm); w falls to 0 within 50 ms in a slide and
--     returns to 1 over 0.5 s after a 0.5 s hold.
--   * Silent yaw component (componentOrderYawControl 5): requests an
--     oversteer-only rear angle from yawControl's own yaw difference
--     (yawControl.lua:77-86) and always returns false, so the DSC lamp and
--     escActive stay untouched. Registered whenever yawControl and
--     STMEstimate are usable (independent of the jbeam defaults, so the
--     driver can select DSC at run time); inert unless the selection is DSC
--     (or the dev override setAssistEnabled(true)).
--   * Both are registered only if the CMU's yawControl and STMEstimate exist
--     and are active: a lone provider that returns nil crashes
--     yawControl.lua:70 (measuredYaw * expectedYaw). This controller does not
--     live under drivingDynamics/, so the CMU all-active rule (CMU.lua:363-380)
--     never applies to it.
--
-- Verified against the 0.39.4 base code:
--   * main.lua: controller.initSecondStage (:157) -> hydros.init (:164) ->
--     controller.initLastStage (:191). The hydro tables are resolved from
--     v.data.hydros in init (the same tables hydros.init fills,
--     hydros.lua:624-633); their live mapping (center = 1, multIn = multOut =
--     +/-F, cOut = toe, hydros.lua:650-678) is validated in initLastStage.
--   * main.lua:88/91: controller.updateGFX runs before hydros.updateGFX, so a
--     command is consumed in the same frame.
--   * Reset: controller.resetSecondStage (main.lua:455, our M.reset) ->
--     hydros.reset (:462) -> electrics.resetLastStage (:470) ->
--     controller.resetLastStage (:471). The inputFlex value survives
--     hydros.reset (hydros.lua:300-314, 723-728), so 0 is written in reset AND
--     in resetLastStage.
--   * controller.lua:773-782: with M.reset present, resetSecondStage never
--     re-runs initSecondStage, so the hooks are registered exactly once per
--     spawn (yawControl.reset is empty, :134-135). Every init/reset stage ends
--     with cacheAllControllerFunctions (controller.lua:708-731), so the fault
--     paths branch inside updateGFX instead of swapping M.updateGFX.
--   * The CMU resets systemState to "normal" even after a shutdown
--     (CMU.lua:288-295, 385); yawControl.isActive and the sensors' isActive are
--     checked as well.
--   * Signs: + yaw = right turn (brakeControl.lua:127-129 brakes the front
--     left wheel on oversteer with measuredYaw > 0); frontWheelAngle carries
--     the steering-input sign (vehicleData.lua:103-110); oversteer <=>
--     sign(r) * sign(beta) < 0 (motorTorqueControl.lua:88). Checked at run
--     time by the direction plausibility check (etk854hybrid_rasDirCheck);
--     a negative result hands the DSC back the stock reference (see above).
--
-- Electrics (single writer = this controller, written every frame):
--   etk854hybrid_rasCmd       -1..1 actuator command (input of both hydros)
--   etk854hybrid_rasAngleDeg  actual actuator angle from the hydro states [deg]
--   etk854hybrid_rasTargetDeg gated target before the final slew [deg]
--   etk854hybrid_rasFfDeg     slewed feed-forward share [deg]
--   etk854hybrid_rasFbDeg     slewed feedback share [deg] (0 without the assist)
--   etk854hybrid_rasGain      blended k (+ = in-phase)
--   etk854hybrid_rasState     OFF | READY | ACTIVE | HOLD | LIMIT | FAULT
--                             (OFF also with the selection OFF or DSC OFF,
--                             unless a numeric dev override drives the axle)
--   etk854hybrid_rasSel       OFF | DSC | NODSC driver selection
--   etk854hybrid_rasMode      NORMAL | SPORT | TRAILER
--   etk854hybrid_rasFault     none | BROKEN | SOFTWARE | CONFIG
--   etk854hybrid_rasLimit     bits 1 FREEZE 2 CENTRE 4 FADE 8 NO_INCREASE
--                             16 REVERSE_FADE 32 OVERRIDE 64 HARD_BRAKE
--                             128 HALF_CAPS 256 LAUNCH_ACTIVE
--                             512 SEL_OFF 1024 DSC_OFF 2048 DSC_NEUTRAL
--   etk854hybrid_rasDse       bits 1 provider registered 2 component registered
--                             4 provider answered since the last frame
--                             8 |fb| >= 0.01 deg 16 conservative weight < 1
--                             32 hook error latched 64 DSC disabled
--                             128 replica mismatch latch 256 yaw control
--                             present but provider not in service (off
--                             latch, or dirCheck < 0)
--   etk854hybrid_rasDirCheck  1 | 0 | -1 | -2 direction / yaw-sign plausibility
--                             (< 0: stock DSC reference, caps x noProviderScale)
--
-- Driver API: setMode("OFF"|"DSC"|"NODSC"|"ON"), getMode(), cycleMode().
-- Dev / QA API: getState(), setOverride(nil|"OFF"|deg), setYawRefEnabled(b),
-- setAssistEnabled(b), measureWheelAngles(), getParams().

local M = {}
M.type = "auxiliary"
-- after selfLeveling 83, adaptiveSusp 85, diffLock 86 and the DSE
-- (CMU 50, yawControl 60, STMEstimate 80); before awdMode / cooling 90
M.defaultOrder = 87

local abs, min, max, exp, sqrt, atan2, floor = math.abs, math.min, math.max, math.exp, math.sqrt, math.atan2, math.floor
local tan = math.tan
local huge = math.huge
local deg, rad = math.deg, math.rad

local CFG = {} -- resolved configuration (init)
local ST = {}  -- runtime state
local REG = {} -- DSC hook registry

-- DSC hook tables (plain tables, see header)
local PROV = {providerOrder = 10, name = "etk854hybrid_rasYawRef"}
local COMP = {componentOrderYawControl = 5, name = "etk854hybrid_rasYawComp"}

-- ---------------------------------------------------------------------------
-- helpers
-- ---------------------------------------------------------------------------

-- number if finite (a string goes through tonumber), else d
local function num(x, d)
  if type(x) == "string" then x = tonumber(x) end
  if type(x) == "number" and x == x and x > -huge and x < huge then return x end
  return d
end

local function flag(x)
  return x == true or (type(x) == "number" and x > 0.5)
end

local function fin(x)
  if type(x) == "number" and x == x and x > -huge and x < huge then return x end
  return 0
end

local function clamp(x, a, b)
  return min(max(x, a), b)
end

-- clamped linear scale (NaN x -> y0)
local function lin(x, x0, x1, y0, y1)
  if x1 == x0 then return (x >= x1) and y1 or y0 end
  local t = (x - x0) / (x1 - x0)
  if not (t > 0) then t = 0 elseif t > 1 then t = 1 end
  return y0 + (y1 - y0) * t
end

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

local function sgn(x)
  if x > 0 then return 1 elseif x < 0 then return -1 end
  return 0
end

-- switch: boolean as is, number / string >= 0.5
local function sw(x, d)
  if x == nil then return d end
  if type(x) == "boolean" then return x end
  return num(x, d and 1 or 0) >= 0.5
end

local function copy(t)
  if type(t) ~= "table" then return t end
  local o = {}
  for k, x in pairs(t) do o[k] = copy(x) end
  return o
end

local function warn(msg)
  pcall(log, "W", "etk854hybrid_rearSteer", msg)
end

-- Built-in copy of the part's controller row and calibration section. A
-- vanilla-merged or partial section still works; every call returns a fresh
-- table.
local function defaults()
  return {
    -- controller row
    enabled = true, assist = false, maxCounterDeg = 3.0, inPhaseGain = 1.0, toeRatio = 0.9975,
    inputSource = "etk854hybrid_rasCmd", hydroNameL = "etk854hybrid_rasToe_RL", hydroNameR = "etk854hybrid_rasToe_RR",
    linkRatioPerUnit = 0.0167, actuatorDegPerUnit = 3.5,
    -- calibration section (gain k = rear / front road-wheel angle, + = in-phase)
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
    -- selection NODSC: rear wheels straight while the DSC intervenes (+ hold)
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
      minKmh = 8, maxKmh = 30, minSteer = 0.15, minRearDeg = 1.0, maxLatAcc = 3.0,
      maxSteerRate = 0.5, windowS = 1.0, windows = 3, yawSignFrac = 0.9
    },
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

-- Overwrites the default table def with the jbeam values of src (same
-- shape). Scalars through num (non-negative defaults stay >= 0), switches
-- through sw, curves through validCurve (invalid -> default + one warning).
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

-- driver selection: OFF | DSC | NODSC (messages per selection, cycle order)
local SEL_TEXT = {
  OFF = "Rear-axle steering: OFF",
  DSC = "Rear-axle steering: ON + DSC",
  NODSC = "Rear-axle steering: ON (no DSC steering assist)"
}
local SEL_NEXT = {OFF = "DSC", DSC = "NODSC", NODSC = "OFF"}
local SEL_ALIAS = {OFF = "OFF", DSC = "DSC", NODSC = "NODSC", ON = "DSC"}

-- selection at spawn from the jbeam defaults ($etk854_ras_enable / _assist)
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
    -- enabled = jbeam default only (informational); sel is what acts
    enabled = CFG.enabled, configOK = false, mapFound = false, liveOK = false,
    sel = sel, dscOffActive = false, rasOff = (sel == "OFF"), dscInt = false, dnT = 0, dscNeutral = false,
    ffOn = (sel ~= "OFF"),
    hL = nil, hR = nil, whl = nil, rackScale = 1, wb = wb, invWB = 1 / wb, invSq = 1 / (vch * vch),
    override = nil, yawRefForce = nil, assistForce = nil, handBack = false, dm = nil,
    -- inputs (per frame)
    u = 0, uPrev = nil, uRate = huge, spd = 0, kmh = 0, vFwd = 0, reverse = false, gear = "D",
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
    -- total
    targetDeg = 0, cmdDeg = 0, cmdU = 0, creditDeg = 0, drRefG = 0, actDeg = 0,
    niT = 0, noIncrease = false, centre = false, freezeOn = false, limitBits = 0,
    -- faults
    fault = "none", swFault = false, swErr = nil, msgShown = false, healthT = 0, refreshT = 0,
    -- direction check (latch kept across reset)
    dirCheck = 0, dcStreak = 0, dcStreakVal = 0, dcLast = 0, dcRho = 0, dcRhoCp = 0, dcFrac = 0,
    dcWindows = 0, dcN = 0, dcT = 0, dcSr = 0, dcSv = 0, dcSf = 0, dcSd = 0, dcSs = 0
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

-- registered, no off latch, and no negative direction-check latch (in doubt
-- stock: an implausible rear steer direction or yaw-rate sign never feeds the
-- DSC reference; the caps then drop to noProviderScale, rasDse bit 256)
local function provInService()
  return REG.provReg and not REG.off and not ((ST.dirCheck or 0) < 0)
end

-- DSC OFF in force: drive-mode key "off" or yawControl disabled
local function dscOffNow()
  return ST.dscKey == "off" or ST.dscOn == false
end

-- rear axle held straight: driver selection OFF or DSC OFF
local function rasOffNow()
  return ST.sel == "OFF" or dscOffNow()
end

-- feedback armed: dev override if set, else the selection DSC
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
  if rasOffNow() then return nil end -- selection OFF / DSC OFF: stock
  if (s.dirCheck or 0) < 0 then return nil end -- direction check negative: stock (in doubt stock)
  if abs(g.drRef) < yr.minDeg then return nil end -- stock, bit-identical
  local cmu = g.cmu
  if cmu.systemState == "offline" then return nil end
  local vd, vs = cmu.vehicleData, cmu.virtualSensors
  local st = vd.vehicleStats
  -- the arithmetic is exactly STMEstimate.calculateExpectedYaw (STMEstimate.lua:24-37)
  local sp = vs.trustWorthiness.virtualSpeed >= 1 and abs(vs.virtual.speed) or abs(vs.virtual.wheelSpeed)
  local df = vd.frontWheelAngle
  local q = sp / (1 + (sp * sp * st.invSquaredCharacteristicSpeed))
  local xs = (abs(df) * st.invWheelBase) * q                  -- stock steering term
  local xc = (abs(df - rad(g.drRef)) * st.invWheelBase) * q   -- corrected steering term
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
  -- confirmed slide (conservative): w -> 0 within consDownS, the answer is then
  -- exactly the stock reference rStm (the rear steer is ignored by the
  -- reference); back to the corrected rCorr over consUpS after the hold. No
  -- min(): with an in-phase credit (rCorr < rStm) a min() would keep the
  -- lowered reference in the slide and let the DSC act 20-30 ms early.
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
  return nil -- yawControl falls through to STMEstimate in the same step
end

-- ---------------------------------------------------------------------------
-- silent yaw component (assist; 100 Hz request, applied in GFX)
-- ---------------------------------------------------------------------------

local function compCore(r, yd)
  local s, c, g = ST, CFG, REG
  g.compCalls = g.compCalls + 1
  local key = s.dscKey
  if not assistOn() or rasOffNow() then -- inert unless selection DSC, never with DSC OFF
    s.fbReqDeg = 0
    return
  end
  local fb = c.fb
  local vs = g.cmu.virtualSensors
  local sp = vs.trustWorthiness.virtualSpeed >= 1 and abs(vs.virtual.speed) or abs(vs.virtual.wheelSpeed) -- as STMEstimate.lua:25
  sp = num(sp, 0)
  local G = sp * s.invWB / (1 + sp * sp * s.invSq)
  local kp = min(fb.kpMaxS, fb.kn[key] / max(G, 0.5)) -- loop gain kn
  local e = yd - fb.db[key]
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

-- 1: BROKEN check every faultCheckS (pattern selfLeveling.lua:309-324)
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

-- DSE drive-mode key (driveModes cached as in diffLock); "off" | "dtc" | "dsc"
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

-- CMU usable: present, not offline, sensors present and active
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
  s.gear = gear
  s.park = gear == "P"
  s.reverse = gear == "R" or s.vFwd < -0.3
  s.ignOff = num(ev.ignitionLevel, 2) < 2
  local sh, vd, vs = cmuSensors()
  s.cmuOK = sh ~= nil
  if sh then
    s.r = num(sh.yawAVSmooth, 0)
    -- = yawControl's bodySlipAngleAdjusted (yawControl.lua:56-57, 74-75)
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
  s.dscAct = flag(ev.isYCBrakeActive) or flag(ev.escActive)
  s.tcsOn = flag(ev.tcsActive)
  s.dscInt = s.dscAct or s.tcsOn or flag(ev.isTCBrakeActive) -- "DSC intervenes" (NODSC neutral)
  local ls = ev.etk854hybrid_launchState
  s.launchHold = ls == "ARMED" or ls == "READY"
  s.launchActive = ls == "ACTIVE"
end

-- 2: driver selection and DSC state: OFF / DSC OFF hold the axle straight;
-- NODSC goes straight while the DSC intervenes and dscNeutralHoldS after its
-- last frame; the feed-forward is gated in both cases (it ramps back at its
-- own rate afterwards)
local function selStep(dt)
  local s = ST
  s.dscOffActive = dscOffNow()
  s.rasOff = s.sel == "OFF" or s.dscOffActive
  if s.dscInt then s.dnT = CFG.dscNeutralHoldS else s.dnT = max(0, s.dnT - dt) end
  s.dscNeutral = s.sel == "NODSC" and not s.rasOff and (s.dscInt or s.dnT > 0)
  s.ffOn = not (s.rasOff or s.dscNeutral)
end

-- 2: calibration set and 1 s cross-fade
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
  s.dfF = lpf(s.dfF, s.dfModel, c.frontTauS, dt)                -- counter-phase branch
  s.dfIp = lpf(s.dfIp, s.dfF, c.inPhaseTauS[s.mode], dt)        -- in-phase: front first, rear follows
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

-- 6a: feed-forward slew
-- While gated the feed-forward goes to 0 no faster than the gate centres the
-- command (limitStep), so ffOut stays with cmd and a short gate (DSC key
-- passing OFF, OFF -> ON in the app) resumes bumpless: no swing of cmd
-- toward 0 and back out.
local function ffSlewStep(dt)
  local s, c = ST, CFG
  s.ffTargetDeg = s.ffOn and s.gFF * s.ffBlend or 0 -- gated: selection OFF / DSC OFF / NODSC neutral
  local rate = s.fade and c.slide.fadeRateDegS or s.rateFF
  if s.rasOff then
    rate = min(rate, c.centreRateDegS)
  elseif s.dscNeutral then
    rate = min(rate, c.dscNeutralRateDegS)
  end
  s.ffOut = slew(s.ffOut, s.ffTargetDeg, rate * dt)
end

-- 6b: feedback, GFX part (request comes from COMP at 100 Hz)
local function fbGfxStep(dt)
  local s, c, g = ST, CFG, REG
  local fb = c.fb
  local cmu = g.cmu
  local betaGate = sgn(s.r) * sgn(s.beta) < 0 and abs(s.beta) > fb.betaGate
  local allowed = assistOn() and g.compReg and not g.compOff and s.dscOn and s.dscKey ~= "off"
    and (not fb.requireDirCheck or s.dirCheck == 1) and not s.reverse and s.kmh >= fb.minKmh
    and not s.fadeByCs and not betaGate and not s.launchHold and not s.launchActive
    and not s.park and not s.ignOff and s.fault == "none" and s.override == nil
    and not s.rasOff and not s.dscNeutral
    and cmu ~= nil and cmu.systemState == "normal"
  s.fbAllowed = allowed and true or false
  local tgt = 0
  if s.fbAllowed then tgt = fin(s.fbReqDeg) * lin(s.kmh, fb.minKmh, fb.fullKmh, 0, 1) end
  s.fbLp = lpf(s.fbLp, tgt, fb.tauS, dt)
  s.fbOut = slew(s.fbOut, s.fbLp, fb.rateDegS * dt)
end

-- 7: gates and limit states (lowest priority first, later rows override)
local function limitStep(dt)
  local s, c = ST, CFG
  local tm = c.totalMaxDeg
  local raw = s.ffOut + s.fbOut
  -- finite check BEFORE any min/max (a NaN would be clamped to a full angle)
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
  -- selection NODSC: straight while the DSC intervenes (+ hold), freeze ignored
  if s.dscNeutral then
    tgt, rate, freeze = 0, c.dscNeutralRateDegS, false
    bits = bits + 2048
  end
  -- selection OFF / DSC OFF: straight at the centring rate, freeze ignored
  if s.sel == "OFF" then bits = bits + 512 end
  if s.dscOffActive then bits = bits + 1024 end
  if s.rasOff then tgt, rate, freeze = 0, c.centreRateDegS, false end
  -- a numeric dev override wins over the selection / DSC states (QA), FAULT over everything
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

-- 9: yaw reference credit (commanded angle without the feedback share)
local function creditStep(dt)
  local s, c = ST, CFG
  local tm = c.totalMaxDeg
  local cmd = s.cmdDeg
  local cr = cmd - s.fbOut
  -- NO_INCREASE (or the total clamp) holds or zeroes the command while fbOut
  -- keeps following the request, so only part of fbOut (maybe none) is in
  -- cmd. The feed-forward share then lies between cmd - fbOut (all of it in)
  -- and cmd (none in): take the point nearest ffOut. cmd - fbOut alone would
  -- credit the blocked feedback as a feed-forward angle of the opposite sign
  -- (in-phase: reference above stock) or beyond ffOut (counter-phase), i.e. a
  -- DSC less sensitive than intended. Equals cmd - fbOut whenever all of
  -- fbOut is in cmd (cmd = ffOut + fbOut).
  if type(s.override) ~= "number" and (s.noIncrease or abs(s.ffOut + s.fbOut) > tm) then
    cr = clamp(s.ffOut, min(cr, cmd), max(cr, cmd))
  end
  s.creditDeg = clamp(cr, -tm, tm)
  s.drRefG = lpf(s.drRefG, s.creditDeg, c.yawRef.tauS, dt) -- GFX twin of the provider's lag
end

local function dirCheckEval()
  local s, d = ST, CFG.dirCheck
  local n = s.dcN
  local mr, mv, mf = s.dcSr / n, s.dcSv / n, s.dcSf / n
  local frac = s.dcSs / n
  local res, rho, rhoCp = 0, 0, 0
  if frac < d.yawSignFrac then
    res = -2
  elseif mv > 0 and mf > 0 then
    rho = mr * s.wb / (mv * mf)
    rhoCp = 1 + s.dcSd / s.dcSf
    local rhoUs = 1 / (1 + mv * mv * s.invSq)
    if rho >= rhoUs * (1 + 0.5 * (rhoCp - 1)) then
      res = 1
    elseif rho <= rhoUs * (1 - 0.5 * (rhoCp - 1)) then
      res = -1
    end
  end
  s.dcLast, s.dcRho, s.dcRhoCp, s.dcFrac = res, rho, rhoCp, frac
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

-- 10b: sign-free direction plausibility (diagnostic, feedback arming gate;
-- a negative latch takes the DSC yaw provider out of service)
local function dirCheckStep(dt)
  local s, d = ST, CFG.dirCheck
  local u, a, r = s.u, s.actDeg, s.r
  local ok = not s.reverse and s.kmh >= d.minKmh and s.kmh <= d.maxKmh and abs(u) >= d.minSteer
    and abs(a) >= d.minRearDeg and sgn(a) == -sgn(u) and abs(r) * s.spd <= d.maxLatAcc
    and s.uRate <= d.maxSteerRate and not s.absOn and not s.dscAct and not s.tcsOn and not s.fade
  if not ok then
    s.dcN, s.dcT = 0, 0 -- any interruption discards the window
    return
  end
  if s.dcN == 0 then s.dcSr, s.dcSv, s.dcSf, s.dcSd, s.dcSs = 0, 0, 0, 0, 0 end
  s.dcN = s.dcN + 1
  s.dcT = s.dcT + dt
  s.dcSr = s.dcSr + abs(r)
  s.dcSv = s.dcSv + s.spd
  s.dcSf = s.dcSf + tan(abs(s.dfMeasRad))
  s.dcSd = s.dcSd + tan(abs(rad(a)))
  if sgn(r) == sgn(u) then s.dcSs = s.dcSs + 1 end
  if s.dcT >= d.windowS then
    dirCheckEval()
    s.dcN, s.dcT = 0, 0
  end
end

-- 10: actual actuator angle (static toe and spawn slew are common-mode),
-- direction check, maxLateralAcceleration refresh (driveModes may change it)
local function measureStep(dt)
  local s, c = ST, CFG
  if s.liveOK then
    s.actDeg = fin((s.hL.state - s.hR.state) / (2 * c.linkRatioPerUnit) * c.actuatorDegPerUnit)
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
  if s.rasOff and type(s.override) ~= "number" then return "OFF" end -- selection OFF / DSC OFF
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
  if s.liveOK then
    local c = CFG
    s.actDeg = fin((s.hL.state - s.hR.state) / (2 * c.linkRatioPerUnit) * c.actuatorDegPerUnit)
  end
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

-- discovery + registration (CMU 50 -> yawControl 60 -> STMEstimate 80 -> us 87;
-- yawControl sorts providers / components only in its initLastStage)
-- $etk854_ras_enable / _assist are only the selection at spawn: the hooks are
-- registered regardless, so the driver can switch at run time
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
    if g.maxLat and not g.compReg then -- inert unless assistOn() (selection DSC)
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

-- live mapping check after hydros.init (main.lua:164)
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
  -- CMU all-active shutdown (CMU.lua:363-380) runs in CMU.initLastStage
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
  s.fault, s.swFault, s.swErr, s.handBack, s.msgShown = "none", false, nil, false, false
  s.modePrev, s.blendW = s.mode, 1
  s.uPrev, s.uRate = nil, huge
  s.dnT, s.dscNeutral = 0, false
  s.dcN, s.dcT = 0, 0 -- open window discarded; the latch is a hardware property
  -- kept: sel (driver selection), override, yawRefForce, assistForce, dirCheck, configOK, mapFound
  g.drRef, g.w, g.mismatch, g.answered = 0, 1, 0, false
  -- kept: provReg, compReg, the counts and the off latches (never re-register)
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
    dirCheck = {state = s.dirCheck, streak = s.dcStreak, lastResult = s.dcLast, lastRho = s.dcRho,
                lastRhoCp = s.dcRhoCp, lastYawSignFrac = s.dcFrac, windows = s.dcWindows},
    map = map,
    provider = {registered = g.provReg, count = g.provCount, inService = provInService() and true or false,
                off = g.off, offWhy = g.offWhy, mismatch = g.mismatch, calls = g.calls, w = g.w,
                drRefDeg = g.drRef, last = g.last, lastStm = g.lastStm, maxLat = g.maxLat},
    component = {registered = g.compReg, count = g.compCount, off = g.compOff, calls = g.compCalls}
  }
end

-- nil = automatic (hand-back if a number was active), "OFF" = target 0,
-- number = fixed angle [deg] (ignores freeze, FADE, NO_INCREASE; FAULT wins).
-- Survives a vehicle reset, cleared on reload.
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

-- runtime A/B of the feedback (only effective with the component registered);
-- nil = follow the driver selection (DSC arms it)
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

-- "OFF" | "DSC" | "NODSC" (case-insensitive, "ON" = "DSC"); anything else
-- changes nothing. One message per actual change. Returns the selection in
-- force. Survives a vehicle reset; serialised across a reload.
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

-- signed road-wheel angles [deg, + = wheel heading rotated RIGHT]; dev / QA
-- only, never used for control
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
  local p = copy(CFG)
  p.toe = CFG.toeRatio
  p.F = CFG.linkRatioPerUnit
  p.degPerUnit = CFG.actuatorDegPerUnit
  p.rackScale = ST.rackScale
  p.assist = CFG.assist   -- jbeam default (selection at spawn), not the live state
  p.enabled = CFG.enabled
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
