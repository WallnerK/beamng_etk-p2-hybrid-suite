-- ETK 854 ActiveHybrid - xAllrad AWD / 2WD switch
--
-- Instantiated by the part etk854hybrid_transfer_case_AWD
-- (etk854hybrid_transfercase.jbeam). Switches the transfer-case splitShaft
-- between "connected" (xAllrad AWD, stock electronicSplitShaftLock behaviour)
-- and "disconnected" (pure rear drive).
--
-- Mechanism: splitShaft device:setMode (lua/vehicle/powertrain/splitShaft.lua:106-109).
-- In "disconnected" selectUpdates() (:47-58) installs disconnectedUpdateTorque
-- (:42-45): output 2 (front) gets exactly 0 Nm, clutchRatio is ignored. This is
-- the only hard 2WD - electronicSplitShaftLock isEnabled=false alone is not,
-- because its final clamp (electronicSplitShaftLock.lua:98) still honours
-- awdControl's overrideMin (awdControl.lua:38-42, up to 1 on oversteer).
--
-- Reset handling: splitShaft.reset() re-derives the mode from jbeam
-- (splitShaft.lua:179-183) and powertrain.reset() does not replay
-- previousDeviceModes (powertrain.lua:722-748). main.lua onVehicleReset runs
-- powertrain.reset -> controller.reset -> controller.resetSecondStage (calls our
-- M.reset) -> electrics.resetLastStage -> controller.resetLastStage, so the
-- selected mode is re-applied in M.reset and the electrics are re-published in
-- M.resetLastStage. M.reset/M.resetLastStage MUST exist: without them
-- controller.lua:763-793 would call init/initLastStage again and drop the
-- driver's selection back to the jbeam default.
--
-- API (interface contract):
--   M.setMode("AWD"|"2WD"), M.toggle(), M.getMode()
-- Electrics:
--   etk854hybrid_awdMode      string "AWD"|"2WD"
--   etk854hybrid_awdFrontTqNm number, torque currently sent to the front
--                             output (splitShaft outputTorque2) - QA/HUD aid

local M = {}

M.type = "auxiliary"
-- Only GFX-rate logic; order relative to the actuator (80) is irrelevant
-- because the physical switch happens inside the splitShaft device.
M.defaultOrder = 90

local MODE_AWD = "AWD"
local MODE_2WD = "2WD"

-- splitShaft mode strings per driver-facing mode
local SHAFT_MODE = {AWD = "connected", ["2WD"] = "disconnected"}

-- State grouped in one table (keeps the per-function upvalue count low).
local st = {
  shaft = nil, -- splitShaft device
  splitLockName = "electronicSplitShaftLock",
  disableSplitLockIn2WD = false,
  mode = MODE_AWD,
  frontTorqueName = "outputTorque2",
  hasBuiltRadial = false,
  lockParams = {isEnabled = true} -- reused table, no GC churn
}

local function publish()
  electrics.values.etk854hybrid_awdMode = st.mode
end

-- Optional (jbeam disableSplitLockIn2WD, default false): isEnabled=false in 2WD.
-- Default off because the actuator's clutchRatioPID/clutchRatioSmoother only
-- advance under isEnabled (electronicSplitShaftLock.lua:63-95) and would carry
-- a stale ratio into the reconnect. It also does not silence isActing: the
-- overrideMin clamp (:98/:101) runs regardless of isEnabled.
-- electronicSplitShaftLock.setParameters is a no-op without a CMU
-- (electronicSplitShaftLock.lua:205-208), getControllerSafe returns a no-op
-- table when the controller is missing (controller.lua getControllerSafe).
local function applySplitLock()
  if not st.disableSplitLockIn2WD then
    return
  end
  st.lockParams.isEnabled = (st.mode == MODE_AWD)
  controller.getControllerSafe(st.splitLockName).setParameters(st.lockParams)
end

-- Pushes st.mode into the device. Returns false if the shaft cannot follow.
local function applyToShaft()
  local shaft = st.shaft
  if not shaft or not shaft.setMode then
    return false
  end
  -- Front driveshaft missing/torn off: validate() already forced
  -- "disconnected" (splitShaft.lua:111-118); connecting would drive a
  -- non-existent output.
  if shaft.isPhysicallyDisconnected then
    st.mode = MODE_2WD
    return false
  end
  local target = SHAFT_MODE[st.mode]
  if shaft.mode ~= target then
    if target == "connected" then
      -- setMode does not touch shaftAngle (only reset() does,
      -- splitShaft.lua:185-186). The value frozen at disconnect time would
      -- otherwise be released as a torque spike through lockedUpdateTorque
      -- (:20-31). Starting from 0 plus the actuator tracking live through 2WD
      -- (disableSplitLockIn2WD=false) means reconnect sees the current
      -- slip-based clutchRatio, not one frozen at switch time.
      shaft.shaftAngle = 0
    end
    shaft:setMode(target)
  end
  return true
end

local function applyMode(showMessage)
  applyToShaft()
  applySplitLock()
  publish()
  if showMessage then
    local txt = st.mode == MODE_AWD and "ETK 854 xAllrad: all-wheel drive (AWD)" or "ETK 854 xAllrad: rear-wheel drive (2WD)"
    guihooks.message(txt, 3, "vehicle.etk854hybrid.awdMode")
  end
end

local function setMode(mode)
  local m = mode
  if type(m) == "string" then
    m = m:upper()
  end
  if m == "RWD" then
    m = MODE_2WD
  end
  if m ~= MODE_AWD and m ~= MODE_2WD then
    log("W", "etk854hybrid_awdMode.setMode", "Unknown mode: " .. tostring(mode))
    return
  end
  st.mode = m
  applyMode(true)
end

local function toggle()
  setMode(st.mode == MODE_AWD and MODE_2WD or MODE_AWD)
end

local function getMode()
  return st.mode
end

-- Radial menu entry, pattern from lua/vehicle/controller/4wd.lua:100-133.
-- uniqueID is unique to this mod, so no clash with 4wd ("4wdMode") or the DSC
-- driveModes entry ("driveModeNext"..title, driveModes.lua:226).
local function buildRadial()
  if st.hasBuiltRadial or not core_quickAccess then
    return
  end
  st.hasBuiltRadial = true
  core_quickAccess.addEntry({
    level = "/root/playerVehicle/vehicleFeatures/",
    generator = function(entries)
      local isAwd = st.mode == MODE_AWD
      local entry = {
        title = isAwd and "xAllrad: AWD" or "xAllrad: 2WD",
        icon = isAwd and "jointLocked" or "jointUnlocked",
        uniqueID = "etk854hybrid_awdMode",
        onSelect = function()
          controller.getControllerSafe(M.name).toggle()
          return {"reload"}
        end
      }
      if isAwd then
        entry.color = "#ff6600"
      end
      table.insert(entries, entry)
    end
  })
end

local function updateGFX(dt)
  local shaft = st.shaft
  if not shaft then
    return
  end
  -- Adopt external changes (powertrain UI toggleDeviceMode, a stock 4wd
  -- controller, validate() after the front driveshaft broke off) so the HUD
  -- never shows a mode the hardware is not in. Skipped while broken: a broken
  -- splitShaft already runs disconnectedUpdateTorque (splitShaft.lua:55,
  -- onBreak :120-125) regardless of mode, so the selection stays the
  -- driver's and is re-applied after reset (reset :154+ clears isBroken).
  local shaftIsAwd = shaft.mode == "connected"
  if not shaft.isBroken and shaftIsAwd ~= (st.mode == MODE_AWD) then
    st.mode = shaftIsAwd and MODE_AWD or MODE_2WD
    applySplitLock()
    publish()
  end
  electrics.values.etk854hybrid_awdFrontTqNm = shaft[st.frontTorqueName] or 0
end

local function init(jbeamData)
  st.splitLockName = jbeamData.splitLockName or "electronicSplitShaftLock"
  if jbeamData.disableSplitLockIn2WD ~= nil then
    st.disableSplitLockIn2WD = jbeamData.disableSplitLockIn2WD and true or false
  else
    st.disableSplitLockIn2WD = false
  end

  local shaftName = jbeamData.shaftName or "transfercase"
  st.shaft = powertrain.getDevice(shaftName)
  if not st.shaft then
    log("E", "etk854hybrid_awdMode.init", string.format("splitShaft %q not found, AWD/2WD switch disabled", shaftName))
    M.updateGFX = nil
    return
  end
  if not st.shaft.availableModes or #st.shaft.availableModes < 2 then
    -- jbeam "canDisconnect" missing -> setMode("disconnected") would still
    -- work (setMode does not validate), but warn: the part is misconfigured.
    log("W", "etk854hybrid_awdMode.init", string.format("splitShaft %q has no canDisconnect:true", shaftName))
  end
  -- primaryOutputID can be 2 in other layouts (splitShaft.lua:238-244);
  -- the front is always the secondary output.
  st.frontTorqueName = st.shaft.secondaryOutputTorqueName or "outputTorque2"

  st.mode = ((tonumber(jbeamData.start2WD) or 0) >= 0.5) and MODE_2WD or MODE_AWD
  publish()
  buildRadial()
end

-- After initSecondStage: CMU is registered with the actuator, so the
-- setParameters call in applySplitLock takes effect.
local function initLastStage(jbeamData)
  applyMode(false)
end

-- Runs in controller.resetSecondStage, i.e. after powertrain.reset reverted
-- the splitShaft to its jbeam mode. Keeps the driver's selection.
local function reset(jbeamData)
  applyToShaft()
  applySplitLock()
end

-- electrics.reset/resetLastStage may have cleared our channels.
local function resetLastStage()
  publish()
end

-- Survives vehicle state serialization (controller.lua:961-980).
local function serialize()
  return {mode = st.mode}
end

local function deserialize(data)
  if type(data) == "table" and (data.mode == MODE_AWD or data.mode == MODE_2WD) then
    st.mode = data.mode
    applyMode(false)
  end
end

M.init = init
M.initLastStage = initLastStage
M.reset = reset
M.resetLastStage = resetLastStage
M.updateGFX = updateGFX
M.serialize = serialize
M.deserialize = deserialize

M.setMode = setMode
M.toggle = toggle
M.getMode = getMode

return M
