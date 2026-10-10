-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- ============================================================================
-- ETK 854 ActiveHybrid - etk854hybrid_8hp shift logic
-- Derived from BeamNG.drive 0.39.4
--   lua/vehicle/controller/vehicleController/shiftLogic/automaticGearbox.lua
-- Everything vehicleController calls (vehicleController.lua :205, :478-633,
-- :635-719, :916-926, :1124-1135) is kept with the base semantics; the
-- changes are confined to these hook points, all delegating to
-- etk854hybrid_8hpCore.lua:
--   H1 init: fresh mode lookups (the base never clears existingModeLookup /
--      hShifterModeLookup), core.init, optional temporary-manual paddles
--   H2 determineTargetGearIndex -> core.decide (D / S, arcade and realistic)
--   H3 no throttle cut while shifting (unless legacyPedalCut): the shift
--      logic's throttle IS the hybrid VCU's pedal input
--      (vehicleController.lua:544 -> etk854hybridControl.lua pedalRaw), the
--      torque intervention runs through etk854hybrid_shiftTorqueCoef instead;
--      M<n> manual mode gets core.manualOverride (kickdown / EV lug)
--   H4 updateTorqueConverter -> core.lockup
--   H5 updateWhileShiftingShared: gear change time and ignition cut from the
--      core, NO downshift throttle blip (it would be a full-pedal request to
--      the VCU for one frame)
--   H6 shiftingAggression = adaptation index of the 8HP program
--   H7 getGearName reports M<n> while temporary manual is active
--   H8 core.publish every GFX frame
--   H9 M.setProgram / M.getProgram / M.getDebug
--   H10 lever D <-> S while rolling keeps the gear (no drop to the lowest gear)
-- Loader note: vehicleController.lua:1119 checks FS:fileExists only in the
-- global lua/vehicle tree, so this vehicle-local file is reached through the
-- legacy path controller/shiftLogic-etk854hybrid_8hp.lua (one-line shim next
-- to lua/controller/), which requires this file (main.lua:212 puts the
-- vehicle's lua folder first in package.path).
-- ============================================================================

local M = {}

local max = math.max
local min = math.min
local abs = math.abs

local core = require("controller/vehicleController/shiftLogic/etk854hybrid_8hpCore")

local constants = {rpmToAV = 0.104719755, avToRPM = 9.549296596425384}

local newDesiredGearIndex = 0
local gearbox = nil
local engine = nil
local torqueConverter = nil

local sharedFunctions = nil
local gearboxAvailableLogic = nil
local gearboxLogic = nil

M.gearboxHandling = nil
M.timer = nil
M.timerConstants = nil
M.inputValues = nil
M.shiftPreventionData = nil
M.shiftBehavior = nil
M.smoothedValues = nil

M.currentGearIndex = 0
M.maxGearIndex = 0
M.minGearIndex = 0
M.throttle = 0
M.brake = 0
M.clutchRatio = 1
M.shiftingAggression = 0
M.isArcadeSwitched = false
M.isSportModeActive = false
M.isManualModeActive = false

M.smoothedAvgAVInput = 0
M.rpm = 0
M.idleRPM = 0
M.maxRPM = 0

M.engineThrottle = 0
M.engineLoad = 0
M.engineTorque = 0
M.flywheelTorque = 0
M.gearboxTorque = 0

M.ignition = true
M.isEngineRunning = 0

M.oilTemp = 0
M.waterTemp = 0
M.checkEngine = false

M.energyStorages = {}

local automaticHandling = {
  availableModes = {"P", "R", "N", "D", "S", "1", "2", "3", "4", "5", "6", "7", "8", "9", "M"},
  hShifterModeLookup = {},
  forwardModes = {["D"] = true, ["S"] = true, ["1"] = true, ["2"] = true, ["3"] = true, ["4"] = true, ["5"] = true, ["6"] = true, ["7"] = true, ["8"] = true, ["9"] = true, ["M"] = true},
  availableModeLookup = {},
  existingModeLookup = {},
  modeIndexLookup = {},
  modes = {},
  mode = nil,
  modeIndex = 0,
  maxAllowedGearIndex = 0,
  minAllowedGearIndex = 0,
  autoDownShiftInM = true,
  autoDownShiftInMMinGear = 2,
  defaultForwardMode = "D",
  throttleCoefWhileShifting = 1,
  legacyPedalCut = false
}

-- H4: the lock-up policy lives in the core; only the presence flag stays here
local torqueConverterHandling = {
  hasTorqueConverter = false
}

--shift LEDs are in use if we are in manual control over the gear selection
local function areShiftLEDsInUse()
  return M.gearboxHandling.behavior ~= "arcade" and M.isManualModeActive
end

local function getGearName()
  -- H7: the series car shows M<gear> while the paddles hold a temporary manual gear
  if core.isTempManual() then
    return "M" .. tostring(gearbox.gearIndex)
  end
  local modePrefix = ""
  if automaticHandling.mode == "S" then
    modePrefix = "S"
  elseif string.sub(automaticHandling.mode, 1, 1) == "M" then
    modePrefix = "M"
  end
  return modePrefix ~= "" and modePrefix .. tostring(gearbox.gearIndex) or automaticHandling.mode
end

local function getGearPosition()
  return (automaticHandling.modeIndex - 1) / (#automaticHandling.modes - 1), automaticHandling.modeIndex
end

local function applyGearboxModeRestrictions()
  local manualModeIndex
  if string.sub(automaticHandling.mode, 1, 1) == "M" then
    manualModeIndex = string.sub(automaticHandling.mode, 2)
  end
  local maxGearIndex = gearbox.maxGearIndex
  local minGearIndex = gearbox.minGearIndex
  local numberMode = tonumber(automaticHandling.mode)
  if numberMode then
    maxGearIndex = numberMode
    minGearIndex = 1
  elseif manualModeIndex then
    maxGearIndex = manualModeIndex
    minGearIndex = manualModeIndex
  end

  automaticHandling.maxGearIndex = maxGearIndex
  automaticHandling.minGearIndex = minGearIndex
end

local function applyGearboxMode()
  local autoIndex = automaticHandling.modeIndexLookup[automaticHandling.mode]
  if autoIndex then
    automaticHandling.modeIndex = min(max(autoIndex, 1), #automaticHandling.modes)
    automaticHandling.mode = automaticHandling.modes[automaticHandling.modeIndex]
  end

  if automaticHandling.mode == "P" then
    gearbox:setGearIndex(0)
    gearbox:setMode("park")
  elseif automaticHandling.mode == "N" then
    gearbox:setGearIndex(0)
    gearbox:setMode("neutral")
  else
    gearbox:setMode("drive")
    if automaticHandling.mode == "R" and gearbox.gearIndex > -1 then
      gearbox:setGearIndex(-1)
    elseif automaticHandling.mode ~= "R" and gearbox.gearIndex < 1 then
      gearbox:setGearIndex(1)
    end
  end

  M.isSportModeActive = automaticHandling.mode == "S"
  M.isManualModeActive = string.sub(automaticHandling.mode, 1, 1) == "M"
end

local function gearboxBehaviorChanged(behavior)
  gearboxLogic = gearboxAvailableLogic[behavior]
  M.updateGearboxGFX = gearboxLogic.inGear
  M.shiftUp = gearboxLogic.shiftUp
  M.shiftDown = gearboxLogic.shiftDown
  M.shiftToGearIndex = gearboxLogic.shiftToGearIndex

  if behavior == "arcade" then
    if gearbox.gearIndex > 0 then
      automaticHandling.mode = automaticHandling.defaultForwardMode
      if automaticHandling.mode == "M1" then
        automaticHandling.mode = "M" .. tostring(max(gearbox.gearIndex, 1))
      end
    elseif gearbox.gearIndex < 0 then
      automaticHandling.mode = "R"
    else
      automaticHandling.mode = "N"
    end
  end

  applyGearboxMode()
end

local function setDefaultForwardMode(mode)
  --todo directly set the active mode as well if we are in forward
  automaticHandling.defaultForwardMode = mode
  if automaticHandling.mode == "D" or automaticHandling.mode == "S" or tonumber(automaticHandling.mode) or string.sub(automaticHandling.mode, 1, 1) == "M" then
    if mode == "M1" then --we just shifted into M1
      automaticHandling.mode = "M" .. tostring(max(gearbox.gearIndex, 1))
    else
      automaticHandling.mode = mode
    end
    applyGearboxMode()
    applyGearboxModeRestrictions()
  end
end

-- H10: lever D <-> S while rolling forward keeps the engaged gear and lets the
-- program decide (series car: S drops one or two gears via the SPORT map). The base
-- resets to the LOWEST gear below 0.9 * maxAV here, i.e. an instant multi-gear
-- drop at speed (8th -> 3rd at 80 km/h).
local function keepsForwardGear(previousMode, newMode)
  return (previousMode == "D" or previousMode == "S") and (newMode == "D" or newMode == "S") and gearbox.gearIndex >= 1
end

local function shiftUp()
  if automaticHandling.mode == "N" then
    M.timer.gearChangeDelayTimer = M.timerConstants.gearChangeDelay
  end

  local previousMode = automaticHandling.mode
  automaticHandling.modeIndex = min(automaticHandling.modeIndex + 1, #automaticHandling.modes)
  automaticHandling.mode = automaticHandling.modes[automaticHandling.modeIndex]

  if automaticHandling.mode == "M1" then --we just shifted into M1
    automaticHandling.mode = "M" .. tostring(max(gearbox.gearIndex, 1))
  end

  if M.gearboxHandling.gearboxSafety then
    local gearRatio = 0
    if string.find(automaticHandling.mode, "M") then
      local gearIndex = tonumber(string.sub(automaticHandling.mode, 2))
      gearRatio = gearbox.gearRatios[gearIndex]
    end
    if tonumber(automaticHandling.mode) then
      local gearIndex = tonumber(automaticHandling.mode)
      gearRatio = gearbox.gearRatios[gearIndex]
    end
    if gearbox.outputAV1 * gearRatio > engine.maxAV then
      automaticHandling.mode = previousMode
    end
  end

  if (automaticHandling.mode == "D" or automaticHandling.mode == "R" or automaticHandling.mode == "S") and not keepsForwardGear(previousMode, automaticHandling.mode) then
    local gearIndex = 1
    local tmpEngineAV = gearbox.outputAV1 * gearbox.gearRatios[gearIndex]

    while tmpEngineAV >= engine.maxAV * 0.9 do
      gearIndex = gearIndex + sign(gearIndex)
      tmpEngineAV = gearbox.outputAV1 * (gearbox.gearRatios[gearIndex] or 0)
    end
    gearbox:setGearIndex(gearIndex, 0)
  end

  applyGearboxMode()
  applyGearboxModeRestrictions()
end

local function shiftDown()
  if automaticHandling.mode == "N" then
    M.timer.gearChangeDelayTimer = M.timerConstants.gearChangeDelay
  end

  local previousMode = automaticHandling.mode
  automaticHandling.modeIndex = max(automaticHandling.modeIndex - 1, 1)
  automaticHandling.mode = automaticHandling.modes[automaticHandling.modeIndex]

  if previousMode == "M1" and electrics.values.wheelspeed > 2 and M.gearboxHandling.gearboxSafety then
    --we just tried to downshift past M1, something that is irritating while racing, so we disallow this shift unless we are really slow
    automaticHandling.mode = previousMode
  end

  if M.gearboxHandling.gearboxSafety then
    local gearRatio = 0
    if string.find(automaticHandling.mode, "M") then
      local gearIndex = tonumber(string.sub(automaticHandling.mode, 2))
      gearRatio = gearbox.gearRatios[gearIndex]
    end
    if tonumber(automaticHandling.mode) then
      local gearIndex = tonumber(automaticHandling.mode)
      gearRatio = gearbox.gearRatios[gearIndex]
    end
    if gearbox.outputAV1 * gearRatio > engine.maxAV then
      automaticHandling.mode = previousMode
    end
  end

  if (automaticHandling.mode == "D" or automaticHandling.mode == "R" or automaticHandling.mode == "S") and not keepsForwardGear(previousMode, automaticHandling.mode) then
    local gearIndex = 1
    local tmpEngineAV = gearbox.outputAV1 * gearbox.gearRatios[gearIndex]

    while tmpEngineAV >= engine.maxAV * 0.9 do
      gearIndex = gearIndex + sign(gearIndex)
      tmpEngineAV = gearbox.outputAV1 * (gearbox.gearRatios[gearIndex] or 0)
    end
    gearbox:setGearIndex(gearIndex, 0)
  end

  applyGearboxMode()
  applyGearboxModeRestrictions()
end

-- H1/H11: temporary-manual paddle taps (only installed when the jbeam flag
-- etk854hybrid8hp.paddleTempManual is true). Taps the core does not consume
-- fall through to the normal lever logic, so D -> N -> R at standstill and
-- all non-D modes behave exactly like the base.
local function shiftUpOnDownTempManual()
  if not core.tempManual(1, M) then
    M.shiftUp()
  end
end

local function shiftDownOnDownTempManual()
  if not core.tempManual(-1, M) then
    M.shiftDown()
  end
end

local function shiftToGearIndex(index)
  local desiredMode = automaticHandling.hShifterModeLookup[index]
  if not desiredMode or not automaticHandling.existingModeLookup[desiredMode] then
    if desiredMode and not automaticHandling.existingModeLookup[desiredMode] then
      guihooks.message({txt = "vehicle.vehicleController.cannotShiftAuto", context = {mode = desiredMode}}, 2, "vehicle.shiftLogic.cannotShift")
    end
    desiredMode = "N"
  end
  local previousMode = automaticHandling.mode
  automaticHandling.mode = desiredMode

  if (automaticHandling.mode == "D" or automaticHandling.mode == "R" or automaticHandling.mode == "S" or automaticHandling.mode == "M1") and not keepsForwardGear(previousMode, automaticHandling.mode) then
    local gearIndex = 1
    local tmpEngineAV = gearbox.outputAV1 * gearbox.gearRatios[gearIndex]

    while tmpEngineAV >= engine.maxAV * 0.9 do
      gearIndex = gearIndex + sign(gearIndex)
      tmpEngineAV = gearbox.outputAV1 * (gearbox.gearRatios[gearIndex] or 0)
    end
    gearbox:setGearIndex(gearIndex, 0)
    if automaticHandling.mode == "M1" then --we just shifted into M1
      automaticHandling.mode = "M" .. tostring(max(gearbox.gearIndex, 1))
    end
  end

  applyGearboxMode()
  applyGearboxModeRestrictions()
end

local function updateExposedData()
  M.rpm = engine and (engine.outputAV1 * constants.avToRPM) or 0
  M.smoothedAvgAVInput = sharedFunctions.updateAvgAVSingleDevice("gearbox")
  M.waterTemp = (engine and engine.thermals) and (engine.thermals.coolantTemperature or engine.thermals.oilTemperature) or 0
  M.oilTemp = (engine and engine.thermals) and engine.thermals.oilTemperature or 0
  M.checkEngine = engine and engine.isDisabled or false
  M.ignition = electrics.values.ignitionLevel > 1
  M.engineThrottle = (engine and engine.isDisabled) and 0 or M.throttle
  M.engineLoad = engine and (engine.isDisabled and 0 or engine.instantEngineLoad) or 0
  M.running = engine and not engine.isDisabled or false
  M.engineTorque = engine and engine.combustionTorque or 0
  M.flywheelTorque = engine and engine.outputTorque1 or 0
  M.gearboxTorque = gearbox and gearbox.outputTorque1 or 0
  local running = (engine and engine.starterMaxAV and engine.starterEngagedCoef) and ((engine.outputAV1 > engine.starterMaxAV * 0.8 and engine.starterEngagedCoef <= 0) and 1 or 0) or 1
  -- H12: hybrid READY. The P2 sequencer (etk854hybrid_iceSequencer) switches
  -- the ICE off ON PURPOSE in EV operation and owns every start (the starter
  -- is locked through engine.starterDisabled). vehicleController's
  -- handleStalling (realistic branch, vehicleController.lua:323-346) shows
  -- "engine is off - use the starter" every frame while ignitionLevel >= 2 and
  -- isEngineRunning == 0. The drive system IS running: report 1 while the VCU
  -- is alive (it publishes etk854hybrid_iceEngageState every frame), the key
  -- is on and the engine is healthy. A damaged (isDisabled) engine keeps the
  -- stock semantics. Side effect by intent: electrics.engineRunning (VC:565,
  -- dash battery light / outgauge) now means "READY", like the series car.
  if running == 0 and engine and not engine.isDisabled and (electrics.values.ignitionLevel or 0) >= 2
     and type(electrics.values.etk854hybrid_iceEngageState) == "string" then
    running = 1
  end
  M.isEngineRunning = running
  M.isShifting = gearbox and gearbox.isShifting or false
  M.minGearIndex = gearbox.minGearIndex
  M.maxGearIndex = gearbox.maxGearIndex
  -- H6: the 8HP adaptation index instead of vehicleController's aggression
  M.shiftingAggression = core.adapt()
end

-- H2: the whole gear decision (incl. its own timing) lives in the core; the
-- base gearChangeDelayTimer / throttle-stable gates are gone
local function determineTargetGearIndex(dt)
  return core.decide(dt, M)
end

-- H4
local function updateTorqueConverter(dt)
  if not torqueConverterHandling.hasTorqueConverter then
    return
  end

  electrics.values.lockupClutchRatio = core.lockup(dt, M)
end

-- H5
local function updateWhileShiftingShared(dt)
  local gearChangeTime, cutIgnition = core.onShiftStart(newDesiredGearIndex, M)
  if cutIgnition and engine and engine.cutIgnition then
    engine:cutIgnition(gearChangeTime * 0.5)
  end
  gearbox:setGearIndex(newDesiredGearIndex, gearChangeTime)
  newDesiredGearIndex = 0
  M.timer.gearChangeDelayTimer = M.timerConstants.gearChangeDelay
  M.updateGearboxGFX = gearboxLogic.inGear
  updateExposedData()
  core.publish(dt, M)
end

local function updateInGearArcade(dt)
  M.throttle = M.inputValues.throttle
  M.brake = M.inputValues.brake
  M.isArcadeSwitched = false

  local gearIndex = gearbox.gearIndex
  local engineAV = engine.outputAV1

  -- driving backwards? - only with automatic shift - for obvious reasons ;)
  if (gearIndex < 0 and M.smoothedValues.avgAV <= 0.8) or (gearIndex <= 0 and M.smoothedValues.avgAV < -1) then
    M.throttle, M.brake = M.brake, M.throttle
    M.isArcadeSwitched = true
  end

  --Arcade mode gets a "rev limiter" in case the engine does not have one
  if engineAV > engine.maxAV and not engine.hasRevLimiter then
    local throttleAdjust = min(max((engineAV - engine.maxAV * 1.02) / (engine.maxAV * 0.03), 0), 1)
    M.throttle = min(max(M.throttle - throttleAdjust, 0), 1)
  end

  gearIndex = determineTargetGearIndex(dt)
  --enforce things like L and M modes
  if not M.isManualModeActive then
    gearIndex = min(max(gearIndex, automaticHandling.minGearIndex), automaticHandling.maxGearIndex)
  end
  updateTorqueConverter(dt)

  -- neutral gear handling
  if abs(gearbox.gearIndex) <= 1 and M.timer.neutralSelectionDelayTimer <= 0 then
    if automaticHandling.mode ~= "P" and abs(M.smoothedValues.avgAV) < M.gearboxHandling.arcadeAutoBrakeAVThreshold and M.throttle <= 0 then
      M.brake = max(M.brake, M.gearboxHandling.arcadeAutoBrakeAmount)
    end

    if automaticHandling.mode ~= "N" and abs(M.smoothedValues.avgAV) < M.gearboxHandling.arcadeAutoBrakeAVThreshold and M.smoothedValues.throttle <= 0 then
      gearIndex = 0
      automaticHandling.mode = "N"
      applyGearboxMode()
      applyGearboxModeRestrictions()
    else
      if M.smoothedValues.throttleInput > 0 and M.inputValues.throttle > 0 and M.smoothedValues.brakeInput <= 0 and M.smoothedValues.avgAV > -1 and gearIndex < 1 then
        gearIndex = 1
        M.timer.neutralSelectionDelayTimer = M.timerConstants.neutralSelectionDelay
        automaticHandling.mode = automaticHandling.defaultForwardMode
        applyGearboxMode()
        applyGearboxModeRestrictions()
      end
      -- H9 (feedback round 2): two-pedal standstill (brake held + throttle
      -- past half travel) engages / keeps the forward mode in arcade too. The
      -- base rule above needs brakeInput <= 0, so a braked standstill stayed
      -- in N (then R) and launch control / the brake-throttle guard
      -- (etk854hybrid_launchControl) never saw a forward gear. Not when
      -- rolling backwards (avgAV <= -1).
      if M.smoothedValues.throttleInput > 0.5 and M.inputValues.throttle > 0.5 and M.inputValues.brake > 0.1
         and abs(M.smoothedValues.avgAV) < M.gearboxHandling.arcadeAutoBrakeAVThreshold and gearIndex < 1 then
        gearIndex = 1
        M.timer.neutralSelectionDelayTimer = M.timerConstants.neutralSelectionDelay
        automaticHandling.mode = automaticHandling.defaultForwardMode
        applyGearboxMode()
        applyGearboxModeRestrictions()
      end

      if M.smoothedValues.brakeInput > 0.1 and M.inputValues.brake > 0 and M.smoothedValues.throttleInput <= 0 and M.smoothedValues.avgAV <= 0.15 and gearIndex > -1 then
        gearIndex = -1
        M.timer.neutralSelectionDelayTimer = M.timerConstants.neutralSelectionDelay
        automaticHandling.mode = "R"
        applyGearboxMode()
        applyGearboxModeRestrictions()
      end
    end

    if electrics.values.ignitionLevel ~= 2 and automaticHandling.mode ~= "P" then
      gearIndex = 0
      M.timer.neutralSelectionDelayTimer = M.timerConstants.neutralSelectionDelay
      automaticHandling.mode = "P"
      applyGearboxMode()
      applyGearboxModeRestrictions()
    end
  end

  M.throttle = automaticHandling.mode ~= "N" and M.throttle or 0
  -- H3: no pedal cut unless the VCU cannot apply etk854hybrid_shiftTorqueCoef
  if automaticHandling.legacyPedalCut then
    M.throttle = gearbox.isShiftingUp and min(M.throttle, automaticHandling.throttleCoefWhileShifting) or M.throttle
  end

  if gearbox.gearIndex ~= gearIndex then
    newDesiredGearIndex = gearIndex
    M.updateGearboxGFX = gearboxLogic.whileShifting
  end

  M.currentGearIndex = gearIndex
  updateExposedData()
  core.publish(dt, M)
end

local function updateWhileShiftingArcade(dt)
  M.throttle = M.inputValues.throttle
  M.brake = M.inputValues.brake
  M.isArcadeSwitched = false

  -- old -> wait -> new -> in gear update
  local gearIndex = gearbox.gearIndex
  if (gearIndex < 0 and M.smoothedValues.avgAV <= 0.15) or (gearIndex <= 0 and M.smoothedValues.avgAV < -1) then
    M.throttle, M.brake = M.brake, M.throttle
    M.isArcadeSwitched = true
  end

  updateWhileShiftingShared(dt)
end

local function updateInGear(dt)
  M.throttle = M.inputValues.throttle
  M.brake = M.inputValues.brake
  M.isArcadeSwitched = false

  local gearIndex = determineTargetGearIndex(dt)
  local gearboxInputAV = gearbox.inputAV
  local isManualMode = string.sub(automaticHandling.mode, 1, 1) == "M"

  --enforce things like L and M modes
  gearIndex = min(max(gearIndex, automaticHandling.minGearIndex), automaticHandling.maxGearIndex)
  if isManualMode and gearIndex > automaticHandling.autoDownShiftInMMinGear and gearboxInputAV < engine.idleAV * 1.2 and M.shiftPreventionData.wheelSlipShiftDown and automaticHandling.autoDownShiftInM then
    gearIndex = gearIndex - 1
  end
  -- H3: kickdown / EV lug protection in M<n>; the mode string follows below
  if isManualMode then
    gearIndex = core.manualOverride(gearIndex, M)
  end

  updateTorqueConverter(dt)

  local isSportMode = automaticHandling.mode == "S"

  -- H3: no pedal cut unless the VCU cannot apply etk854hybrid_shiftTorqueCoef
  if automaticHandling.legacyPedalCut then
    M.throttle = (gearbox.isShiftingUp and not isSportMode) and min(M.throttle, automaticHandling.throttleCoefWhileShifting) or M.throttle
  end

  if gearbox.gearIndex ~= gearIndex then
    newDesiredGearIndex = gearIndex
    M.updateGearboxGFX = gearboxLogic.whileShifting
  end

  M.currentGearIndex = gearIndex
  updateExposedData()

  if isManualMode then
    automaticHandling.mode = "M" .. gearIndex
    automaticHandling.modeIndex = automaticHandling.modeIndexLookup[automaticHandling.mode]
    applyGearboxModeRestrictions()
  end
  core.publish(dt, M)
end

local function updateWhileShifting(dt)
  -- old -> wait -> new -> in gear update
  M.throttle = M.inputValues.throttle
  M.brake = M.inputValues.brake
  M.isArcadeSwitched = false

  updateWhileShiftingShared(dt)
end

local function sendTorqueData()
  if engine then
    engine:sendTorqueData()
  end
end

local function init(jbeamData, sharedFunctionTable)
  sharedFunctions = sharedFunctionTable
  engine = powertrain.getDevice("mainEngine")
  gearbox = powertrain.getDevice("gearbox")
  torqueConverter = powertrain.getDevice("torqueConverter")
  newDesiredGearIndex = 0

  M.currentGearIndex = 0
  M.throttle = 0
  M.brake = 0
  M.clutchRatio = 1

  gearboxAvailableLogic = {
    arcade = {
      inGear = updateInGearArcade,
      whileShifting = updateWhileShiftingArcade,
      shiftUp = sharedFunctions.warnCannotShiftSequential,
      shiftDown = sharedFunctions.warnCannotShiftSequential,
      shiftToGearIndex = sharedFunctions.switchToRealisticBehavior
    },
    realistic = {
      inGear = updateInGear,
      whileShifting = updateWhileShifting,
      shiftUp = shiftUp,
      shiftDown = shiftDown,
      shiftToGearIndex = shiftToGearIndex
    }
  }

  automaticHandling.availableModeLookup = {}
  for _, v in pairs(automaticHandling.availableModes) do
    automaticHandling.availableModeLookup[v] = true
  end

  -- H1: rebuild every lookup on each init (init runs again on every reset,
  -- vehicleController.lua:917, while the module stays cached by require)
  automaticHandling.modes = {}
  automaticHandling.modeIndexLookup = {}
  automaticHandling.existingModeLookup = {}
  automaticHandling.hShifterModeLookup = {}
  local modes = jbeamData.automaticModes or "PRNDS21M"
  local modeCount = #modes
  local modeOffset = 0
  local forwardModes = {}
  for i = 1, modeCount do
    local mode = modes:sub(i, i)
    if automaticHandling.availableModeLookup[mode] then
      if automaticHandling.forwardModes[mode] then
        table.insert(forwardModes, mode)
      end
      if mode ~= "M" then
        automaticHandling.modes[i + modeOffset] = mode
        automaticHandling.modeIndexLookup[mode] = i + modeOffset
        automaticHandling.existingModeLookup[mode] = true
      else
        for j = 1, gearbox.maxGearIndex, 1 do
          local manualMode = "M" .. tostring(j)
          local manualModeIndex = i + j - 1
          automaticHandling.modes[manualModeIndex] = manualMode
          automaticHandling.modeIndexLookup[manualMode] = manualModeIndex
          automaticHandling.existingModeLookup[manualMode] = true
          modeOffset = j - 1
        end
      end
    else
      print("unknown auto mode: " .. mode)
    end
  end

  if jbeamData.hShifterModePositions then
    for mode, index in pairs(jbeamData.hShifterModePositions) do
      automaticHandling.hShifterModeLookup[index] = mode
    end
  else
    -- -1 = R, 0 = N, 1 = P, 2 = D, 3 = S (4/5 are "2"/"1", absent in PRNDSM)
    automaticHandling.hShifterModeLookup = {[-1] = "R", [0] = "N", "P", "D", "S", "2", "1", "M1"}
  end

  torqueConverterHandling.hasTorqueConverter = torqueConverter ~= nil

  local defaultMode = jbeamData.defaultAutomaticMode or "N"
  local defaultForwardMode = jbeamData.defaultAutomaticForwardMode or forwardModes[1] or "D"
  --if we want M as default mode, we need to actually use M1, as that's the correct mode name, "M" is just how it's called in jbeam
  if defaultMode == "M" then
    defaultMode = "M1"
  end
  if defaultForwardMode == "M" then
    defaultForwardMode = "M1"
  end
  --Check if we do actually support the requested default mode, if not, default to "D" mode
  if not automaticHandling.existingModeLookup[defaultMode] then
    defaultMode = "D"
  end

  automaticHandling.modeIndex = automaticHandling.modeIndexLookup[defaultMode]
  automaticHandling.defaultForwardMode = defaultForwardMode
  automaticHandling.mode = automaticHandling.modes[automaticHandling.modeIndex]
  automaticHandling.maxGearIndex = gearbox.maxGearIndex
  automaticHandling.minGearIndex = gearbox.minGearIndex
  automaticHandling.autoDownShiftInM = jbeamData.autoDownShiftInM == nil and true or jbeamData.autoDownShiftInM
  automaticHandling.autoDownShiftInMMinGear = jbeamData.autoDownShiftInMMinGear == nil and 2 or jbeamData.autoDownShiftInMMinGear
  automaticHandling.throttleCoefWhileShifting = jbeamData.throttleCoefWhileShifting or 1

  M.maxRPM = engine.maxRPM
  M.idleRPM = engine.idleRPM
  M.maxGearIndex = automaticHandling.maxGearIndex
  M.minGearIndex = abs(automaticHandling.minGearIndex)
  M.energyStorages = sharedFunctions.getEnergyStorages({engine})

  applyGearboxMode()

  -- H1: decision core (re-creates ALL of its state on every init)
  core.init(jbeamData, engine, gearbox, torqueConverter, sharedFunctions, automaticHandling)
  automaticHandling.legacyPedalCut = core.P.legacyPedalCut == true
  -- paddle hooks are looked up at call time (vehicleController.lua:635-660)
  if core.P.paddleTempManual then
    M.shiftUpOnDown = shiftUpOnDownTempManual
    M.shiftDownOnDown = shiftDownOnDownTempManual
  else
    M.shiftUpOnDown = nil
    M.shiftDownOnDown = nil
  end
  M.shiftUpOnUp = nil
  M.shiftDownOnUp = nil
end

local function getState()
  local data = {grb_mde = automaticHandling.mode}

  return tableIsEmpty(data) and nil or data
end

local function setState(data)
  if data.grb_mde then
    automaticHandling.mode = data.grb_mde
    automaticHandling.modeIndex = automaticHandling.modeIndexLookup[automaticHandling.mode]
    applyGearboxMode()
    applyGearboxModeRestrictions()
  end
end

-- H9: program override for other Lua (VCU, UI apps, QA). Reach this module
-- through package.loaded["controller/vehicleController/shiftLogic/etk854hybrid_8hp"]
-- (or the legacy key "controller/shiftLogic-etk854hybrid_8hp", same table).
-- p = nil / "AUTO" (follow the driving experience) | "ECO" | "COMFORT" | "SPORT"
--     -> automatic program, lever S / M<n> return to D (gear kept, H10)
-- p = "MANUAL" -> the M gate: hold the engaged gear as M<g>, the driver
--     shifts (realistic behaviour only; arcade has no sequential shifting)
local function setProgram(p)
  if p == "AUTO" or p == "" then p = nil end
  if p ~= nil and p ~= "ECO" and p ~= "COMFORT" and p ~= "SPORT" and p ~= "MANUAL" then
    return false
  end
  local mode = automaticHandling.mode
  local forward = gearbox ~= nil and gearbox.gearIndex >= 1
  if p == "MANUAL" then
    if M.gearboxHandling and M.gearboxHandling.behavior == "arcade" then
      return false
    end
    if forward and (mode == "D" or mode == "S") then
      automaticHandling.mode = "M" .. tostring(gearbox.gearIndex)
      applyGearboxMode()
      applyGearboxModeRestrictions()
    end
    return true
  end
  core.setProgram(p)
  if forward and (mode == "S" or string.sub(mode, 1, 1) == "M") then
    automaticHandling.mode = "D"
    applyGearboxMode()
    applyGearboxModeRestrictions()
  end
  return true
end

local function getProgram()
  return core.getProgram()
end

local function getDebug()
  return core.getDebug()
end

M.init = init

M.gearboxBehaviorChanged = gearboxBehaviorChanged
M.shiftUp = shiftUp
M.shiftDown = shiftDown
M.shiftToGearIndex = shiftToGearIndex
M.updateGearboxGFX = nop
M.getGearName = getGearName
M.getGearPosition = getGearPosition
M.setDefaultForwardMode = setDefaultForwardMode
M.sendTorqueData = sendTorqueData

M.areShiftLEDsInUse = areShiftLEDsInUse

M.getState = getState
M.setState = setState

M.setProgram = setProgram
M.getProgram = getProgram
M.getDebug = getDebug

return M
