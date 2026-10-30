-- ETK 854 ActiveHybrid - xAllrad AWD / 2WD switch (part
-- etk854hybrid_transfer_case_AWD).
--
-- Sets the transfer-case splitShaft to "connected" (AWD) or "disconnected"
-- (RWD). Only "disconnected" is a hard 2WD (front output exactly 0 Nm);
-- electronicSplitShaftLock isEnabled=false is not, its clamp still honours
-- awdControl's overrideMin.
--
-- Reset: splitShaft.reset() re-derives the mode from jbeam, so M.reset
-- (resetSecondStage) re-applies the selection and M.resetLastStage
-- re-publishes. Both must exist, otherwise controller.lua re-runs init and
-- the selection falls back to the jbeam default.
--
-- API: setMode("AWD" | "2WD", case-insensitive, "RWD" = "2WD"), toggle(), getMode()
-- Publishes: etk854hybrid_awdMode ("AWD" | "2WD"),
--            etk854hybrid_awdFrontTqNm (front output torque [Nm])

local M = {}

M.type = "auxiliary"
M.defaultOrder = 90 -- order irrelevant: the switch happens in the device

local MODE_AWD = "AWD"
local MODE_2WD = "2WD"

local SHAFT_MODE = {AWD = "connected", ["2WD"] = "disconnected"}

local st = {
  shaft = nil, -- splitShaft device
  shaftName = "transfercase",
  splitLockName = "electronicSplitShaftLock",
  disableSplitLockIn2WD = false,
  mode = MODE_AWD,
  frontTorqueName = "outputTorque2",
  hasBuiltRadial = false,
  lockParams = {isEnabled = true} -- reused
}

local function publish()
  electrics.values.etk854hybrid_awdMode = st.mode
end

-- Optional isEnabled=false in 2WD (disableSplitLockIn2WD). Off by default:
-- the actuator's PID/smoother only advance while enabled and would carry a
-- stale clutch ratio into the reconnect. setParameters needs a CMU.
local function applySplitLock()
  if not st.disableSplitLockIn2WD then
    return
  end
  st.lockParams.isEnabled = (st.mode == MODE_AWD)
  controller.getControllerSafe(st.splitLockName).setParameters(st.lockParams)
end

-- Returns false if the shaft cannot follow st.mode.
local function applyToShaft()
  local shaft = st.shaft
  if not shaft or not shaft.setMode then
    return false
  end
  -- Front driveshaft missing/torn off: validate() already forced "disconnected".
  if shaft.isPhysicallyDisconnected then
    st.mode = MODE_2WD
    return false
  end
  local target = SHAFT_MODE[st.mode]
  if shaft.mode ~= target then
    if target == "connected" then
      -- setMode keeps shaftAngle; the angle frozen at disconnect would be
      -- released as a torque spike on reconnect.
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

-- Radial menu entry as in the stock controller/4wd.lua (own uniqueID).
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
  -- Adopt external mode changes (powertrain UI, stock 4wd, validate()) so
  -- the HUD matches the hardware. Not while broken: the selection is kept
  -- and re-applied after the reset that clears isBroken.
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
  st.disableSplitLockIn2WD = jbeamData.disableSplitLockIn2WD and true or false

  local shaftName = jbeamData.shaftName or "transfercase"
  st.shaftName = shaftName
  st.shaft = powertrain.getDevice(shaftName)
  if not st.shaft then
    log("E", "etk854hybrid_awdMode.init", string.format("splitShaft %q not found, AWD/2WD switch disabled", shaftName))
    M.updateGFX = nil
    return
  end
  if not st.shaft.availableModes or #st.shaft.availableModes < 2 then
    -- setMode still works, but the part is misconfigured.
    log("W", "etk854hybrid_awdMode.init", string.format("splitShaft %q has no canDisconnect:true", shaftName))
  end
  -- The front is always the secondary output.
  st.frontTorqueName = st.shaft.secondaryOutputTorqueName or "outputTorque2"

  st.mode = ((tonumber(jbeamData.start2WD) or 0) >= 0.5) and MODE_2WD or MODE_AWD
  publish()
  buildRadial()
end

-- Last stage: the CMU is registered, so setParameters takes effect.
local function initLastStage(jbeamData)
  applyMode(false)
end

-- After powertrain.reset reverted the splitShaft to its jbeam mode. The device
-- is re-fetched in case powertrain.reset fell back to a full init (new tables).
local function reset(jbeamData)
  st.shaft = powertrain.getDevice(st.shaftName) or st.shaft
  applyToShaft()
  applySplitLock()
end

-- electrics.reset may have cleared our channels.
local function resetLastStage()
  publish()
end

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
