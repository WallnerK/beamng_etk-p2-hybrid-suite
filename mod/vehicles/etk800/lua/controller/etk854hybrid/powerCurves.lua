-- powerCurves.lua - ETK 854 ActiveHybrid - engine/motor envelope provider
--
-- Replaces the hard-coded "ETK i4 2.0T" torque approximation in the master VCU.
-- Reads the REAL curves off the powertrain devices at initStage2 so the same
-- controller works unchanged on:
--   * i4 2.0 petrol turbo (idle ~800, maxRPM 7000)
--   * I6 3.0 petrol turbo (idle ~800, maxRPM 7000)
--   * I6 3.0 diesel turbo (idle ~750, maxRPM 5500)
-- No petrol-specific constants (no fixed 850 rpm idle threshold, no fixed
-- 5000 rpm peak plateau) survive in the hot path.
--
-- Reads:
--   powertrain.getDevice("mainEngine").torqueData / .idleRPM / .maxRPM
--   powertrain.getDevice("integratedMotor").torqueCurve / .regenCurve / .maxRPM
--
-- Writes: nothing (pure provider). All accessors are safe before initStage2 —
-- they fall back to the analytic envelope below.

local M = {}
M.name = "powerCurves"

local floor, abs, max, min = math.floor, math.abs, math.max, math.min

-- ===========================================================
-- Analytic fallback envelope
-- Only used when a device exposes no usable curve table (would otherwise mean
-- a nil lookup silently zeroing the ICE share of the torque split).
-- Expressed in FRACTIONS of the rev range so it degrades sanely on a diesel.
-- ===========================================================
local FB_ICE_PEAK_NM       = 350
local FB_ICE_IDLE_RPM      = 800
local FB_ICE_MAX_RPM       = 7000
local FB_RISE_END_FRAC     = 0.22   -- rpm/maxRPM where the curve reaches peak
local FB_FALL_START_FRAC   = 0.72   -- rpm/maxRPM where the curve starts dropping
local FB_TORQUE_AT_IDLE    = 0.35   -- fraction of peak available at idle
local FB_TORQUE_AT_MAXRPM  = 0.55   -- fraction of peak left at maxRPM

local FB_MOTOR_PEAK_NM     = 500    -- matches the stage-3 envelope in the jbeam
local FB_MOTOR_CORNER_RPM  = 3800   -- constant-torque -> constant-power corner
local FB_MOTOR_MAX_RPM     = 10000
local FB_MOTOR_REGEN_NM    = 180    -- matches maxRegenTorque in the jbeam

-- ===========================================================
-- Cached state
-- ===========================================================
local state = {
  iceCurve      = nil,   -- [rpm] -> Nm (net, incl. turbo), dense, 0-based
  iceCurveMax   = 0,     -- highest rpm index present in iceCurve
  icePeakNm     = FB_ICE_PEAK_NM,
  icePeakRpm    = 0,
  iceIdleRPM    = FB_ICE_IDLE_RPM,
  iceMaxRPM     = FB_ICE_MAX_RPM,
  iceMaxPowerKw = 0,

  motorCurve    = nil,   -- [rpm] -> Nm, dense, 0-based
  motorCurveMax = 0,
  motorPeakNm   = FB_MOTOR_PEAK_NM,
  motorMaxRPM   = FB_MOTOR_MAX_RPM,

  regenCurve    = nil,   -- [rpm] -> Nm, dense, 0-based
  regenCurveMax = 0,
  regenPeakNm   = FB_MOTOR_REGEN_NM,

  resolved      = false,
}

-- ===========================================================
-- Curve extraction
-- ===========================================================

-- combustionEngine.getTorqueData() returns curves sorted by priority DESC, so
-- entry 1 is the fully boosted variant (turbo/SC) = the real max envelope.
-- Its .torque array is 1-based over rpm (index k holds the value for rpm k-1)
-- and already has friction subtracted, so it can go negative near 0 rpm.
local function extractIceCurve(engine)
  local td = engine and engine.torqueData
  if type(td) ~= "table" or type(td.curves) ~= "table" then return nil, 0 end
  local c = td.curves[td.finalCurveName or 1] or td.curves[1]
  if type(c) ~= "table" or type(c.torque) ~= "table" then return nil, 0 end

  local curve, maxIdx, sawValue = {}, 0, false
  for k, v in pairs(c.torque) do
    if type(k) == "number" and type(v) == "number" then
      local rpm = k - 1
      if rpm >= 0 then
        curve[rpm] = v > 0 and v or 0
        if rpm > maxIdx then maxIdx = rpm end
        sawValue = true
      end
    end
  end
  if not sawValue then return nil, 0 end
  return curve, maxIdx
end

-- Both the motor torqueCurve and regenCurve are produced by createCurve() and
-- are dense 0-based arrays, so they can be used as-is.
local function extractDenseCurve(tbl)
  if type(tbl) ~= "table" then return nil, 0 end
  local maxIdx, count = 0, 0
  for k, v in pairs(tbl) do
    if type(k) == "number" and type(v) == "number" then
      if k > maxIdx then maxIdx = k end
      count = count + 1
    end
  end
  if count < 2 then return nil, 0 end
  return tbl, maxIdx
end

local function curveLookup(curve, maxIdx, rpm, fallbackFn)
  if not curve then return fallbackFn(rpm) end
  local r = floor(abs(rpm or 0))
  if r > maxIdx then r = maxIdx end
  local v = curve[r]
  if v == nil then return fallbackFn(rpm) end
  return v > 0 and v or 0
end

-- ===========================================================
-- Analytic fallbacks (rev-range relative, engine-family agnostic)
-- ===========================================================
local function fallbackIceTorque(rpm)
  local r = abs(rpm or 0)
  local idle, maxR = state.iceIdleRPM, state.iceMaxRPM
  if r < idle * 0.5 then return 0 end
  local frac = r / max(1, maxR)
  local peak = state.icePeakNm
  if frac <= FB_RISE_END_FRAC then
    local t = frac / FB_RISE_END_FRAC
    return peak * (FB_TORQUE_AT_IDLE + (1 - FB_TORQUE_AT_IDLE) * t)
  elseif frac <= FB_FALL_START_FRAC then
    return peak
  elseif frac < 1.0 then
    local t = (frac - FB_FALL_START_FRAC) / (1 - FB_FALL_START_FRAC)
    return peak * (1 - (1 - FB_TORQUE_AT_MAXRPM) * t)
  end
  return peak * FB_TORQUE_AT_MAXRPM
end

local function fallbackMotorTorque(rpm)
  local r = abs(rpm or 0)
  local peak = state.motorPeakNm
  if r <= FB_MOTOR_CORNER_RPM then return peak end
  if r >= state.motorMaxRPM then return peak * FB_MOTOR_CORNER_RPM / state.motorMaxRPM end
  return peak * FB_MOTOR_CORNER_RPM / r
end

local function fallbackRegenTorque(rpm)
  local r = abs(rpm or 0)
  -- Mirrors the device's regenFadeRPM ramp so low-speed regen fades out.
  local fade = min(1, r / 600)
  return state.regenPeakNm * fade
end

-- ===========================================================
-- Public accessors
-- ===========================================================
function M.iceMaxTorqueAtRpm(rpm)
  return curveLookup(state.iceCurve, state.iceCurveMax, rpm, fallbackIceTorque)
end

function M.motorMaxTorqueAtRpm(rpm)
  return curveLookup(state.motorCurve, state.motorCurveMax, rpm, fallbackMotorTorque)
end

function M.motorRegenTorqueAtRpm(rpm)
  return curveLookup(state.regenCurve, state.regenCurveMax, rpm, fallbackRegenTorque)
end

function M.iceIdleRPM()  return state.iceIdleRPM end
function M.iceMaxRPM()   return state.iceMaxRPM end
function M.icePeakNm()   return state.icePeakNm end
function M.motorPeakNm() return state.motorPeakNm end
function M.motorMaxRPM() return state.motorMaxRPM end
function M.isResolved()  return state.resolved end

-- ===========================================================
-- Lifecycle
-- ===========================================================
function M.init(ctx)
  -- Devices do not exist yet at init; keep the fallback envelope until stage 2.
  state.resolved = false
end

function M.initStage2(ctx)
  local engine = powertrain.getDevice("mainEngine")
  if engine then
    if type(engine.idleRPM) == "number" and engine.idleRPM > 0 then
      state.iceIdleRPM = engine.idleRPM
    end
    if type(engine.maxRPM) == "number" and engine.maxRPM > 0 then
      state.iceMaxRPM = engine.maxRPM
    end
    local curve, maxIdx = extractIceCurve(engine)
    state.iceCurve, state.iceCurveMax = curve, maxIdx
    local td = engine.torqueData
    if type(td) == "table" then
      if type(td.maxTorque) == "number" and td.maxTorque > 0 then
        state.icePeakNm  = td.maxTorque
        state.icePeakRpm = td.maxTorqueRPM or 0
      end
      -- torqueData.maxPower is in MECHANICAL hp (1/7120.8 in combustionEngine
      -- = kW x 1.34102), NOT metric PS - convert with 0.7457, else the log
      -- under-reports (N57 read "227 kW" instead of the true 230.4 kW/313 PS).
      if type(td.maxPower) == "number" then state.iceMaxPowerKw = td.maxPower * 0.7457 end
    end
    if type(engine.maxTorqueRating) == "number" and engine.maxTorqueRating > 0
       and state.icePeakNm <= 0 then
      state.icePeakNm = engine.maxTorqueRating
    end
  end

  local motor = powertrain.getDevice("integratedMotor")
  if motor then
    if type(motor.maxRPM) == "number" and motor.maxRPM > 0 then
      state.motorMaxRPM = motor.maxRPM
    end
    state.motorCurve, state.motorCurveMax = extractDenseCurve(motor.torqueCurve)
    state.regenCurve, state.regenCurveMax = extractDenseCurve(motor.regenCurve)
    if type(motor.maxTorque) == "number" and motor.maxTorque > 0 then
      state.motorPeakNm = motor.maxTorque
    end
    if state.regenCurve then
      local peak = 0
      for _, v in pairs(state.regenCurve) do
        if type(v) == "number" and v > peak then peak = v end
      end
      if peak > 0 then state.regenPeakNm = peak end
    end
  end

  state.resolved = (state.iceCurve ~= nil) and (state.motorCurve ~= nil)

  -- one-time init summary; unconditional so every run documents which curve
  -- source is active (FALLBACK here means every ICE decision runs on the
  -- analytic approximation - that must be visible without debugLogLevel)
  log("I", "etk854hybrid.powerCurves", string.format(
    "ICE %.0fNm@%.0f idle=%.0f max=%.0f (%.0fkW) curve=%s | MOTOR %.0fNm max=%.0f curve=%s regenPeak=%.0fNm",
    state.icePeakNm, state.icePeakRpm, state.iceIdleRPM, state.iceMaxRPM,
    state.iceMaxPowerKw, state.iceCurve and "real" or "FALLBACK",
    state.motorPeakNm, state.motorMaxRPM,
    state.motorCurve and "real" or "FALLBACK", state.regenPeakNm))
end

function M.updateGFX(ctx, dt) end
function M.updateFixedStep(ctx, dt) end

function M.reset(ctx)
  -- Devices are rebuilt on reset; re-resolve the curves.
  M.initStage2(ctx)
end

function M.getState()
  return {
    icePeakNm   = state.icePeakNm,
    iceIdleRPM  = state.iceIdleRPM,
    iceMaxRPM   = state.iceMaxRPM,
    motorPeakNm = state.motorPeakNm,
    motorMaxRPM = state.motorMaxRPM,
    regenPeakNm = state.regenPeakNm,
    resolved    = state.resolved,
  }
end

return M
