-- ETK 854 ActiveHybrid - adaptive damper bridge
--
-- Maps the VCU drive mode (etk854hybrid_deMode) to a step of the stock
-- adaptiveDampers actuators etk854hybrid_damperF/R (ladder p080..p150, 5 % steps).
-- The ladder is walked step by step (default 110 %/s): a single 5 % step is
-- imperceptible and does not excite the solver like a soft -> hard jump.
-- Trailer mode raises the step to at least trailerPercent (a floor).
-- Trailer detection and its channels belong to etk854hybrid_selfLeveling and
-- the trailer API here delegates; without it (stock rear spring) a manual
-- switch here writes trailerMode / levelState (one writer per channel).
--
-- Publishes: etk854hybrid_suspMode (= deMode), etk854hybrid_suspFirmness [%]

local M = {}

M.type = "auxiliary"
M.defaultOrder = 85 -- after adaptiveDampers (80)

local max = math.max
local min = math.min
local floor = math.floor

-- Must match the "modes" rows in etk854hybrid_chassis.jbeam.
local LADDER = {
  minPercent = 80,
  maxPercent = 150,
  stepPercent = 5
}
local LADDER_STEPS = (LADDER.maxPercent - LADDER.minPercent) / LADDER.stepPercent -- 14

local frontName = "etk854hybrid_damperF"
local rearName = "etk854hybrid_damperR"

local modeIndex = {} -- deMode -> ladder index 0..14
local defaultIndex = 4 -- 100 %

local rampIndexPerSecond = 4 -- set in init

local currentIndex = nil -- float, ramps towards the target
local appliedIndex = nil -- last applied integer step
local currentMode = "COMFORT"

local paramTable = {damperMode = "p100"} -- reused

-- One table: LuaJIT 60-upvalue limit.
local T = {
  levelingName = "etk854hybrid_selfLeveling",
  trailerIndex = 7, -- 115 %
  manualTrailer = nil -- driver switch, only without selfLeveling
}

-- getController, not ...Safe: no "Didn't find controller" log per frame
-- with the stock rear spring.
local function getLeveling()
  local c = controller.getController(T.levelingName)
  if c and c.getTrailerMode then
    return c
  end
  return nil
end

local function percentToIndex(percent)
  local p = tonumber(percent) or 100
  p = max(LADDER.minPercent, min(LADDER.maxPercent, p))
  local idx = floor((p - LADDER.minPercent) / LADDER.stepPercent + 0.5)
  return max(0, min(LADDER_STEPS, idx))
end

local function indexToModeName(idx)
  return string.format("p%03d", LADDER.minPercent + idx * LADDER.stepPercent)
end

local function indexToPercent(idx)
  return LADDER.minPercent + idx * LADDER.stepPercent
end

-- getControllerSafe: no-op for an axle without the adaptive part.
local function applyIndex(idx)
  local modeName = indexToModeName(idx)
  paramTable.damperMode = modeName
  controller.getControllerSafe(frontName).setParameters(paramTable)
  controller.getControllerSafe(rearName).setParameters(paramTable)
  appliedIndex = idx
  electrics.values.etk854hybrid_suspFirmness = indexToPercent(idx)
end

local function updateGFX(dt)
  local deMode = electrics.values.etk854hybrid_deMode
  if type(deMode) ~= "string" then
    deMode = currentMode
  end
  currentMode = deMode
  electrics.values.etk854hybrid_suspMode = deMode

  local targetIndex = modeIndex[deMode] or defaultIndex

  local trailer
  if getLeveling() then
    -- selfLeveling (order 83) has published this frame.
    trailer = electrics.values.etk854hybrid_trailerMode == 1
  else
    trailer = T.manualTrailer == true
    electrics.values.etk854hybrid_trailerMode = trailer and 1 or 0
    electrics.values.etk854hybrid_levelState = "OFF"
  end
  if trailer and T.trailerIndex > targetIndex then
    targetIndex = T.trailerIndex
  end

  if currentIndex == nil then
    currentIndex = targetIndex
  elseif currentIndex < targetIndex then
    currentIndex = min(targetIndex, currentIndex + rampIndexPerSecond * dt)
  elseif currentIndex > targetIndex then
    currentIndex = max(targetIndex, currentIndex - rampIndexPerSecond * dt)
  end

  local wanted = floor(currentIndex + 0.5)
  if wanted ~= appliedIndex then
    applyIndex(wanted)
  end
end

local function init(jbeamData)
  frontName = jbeamData.frontDamperController or frontName
  rearName = jbeamData.rearDamperController or rearName

  modeIndex = {
    COMFORT = percentToIndex(jbeamData.comfortPercent or 90),
    ECO_PRO = percentToIndex(jbeamData.ecoPercent or 100),
    SPORT = percentToIndex(jbeamData.sportPercent or 140)
  }
  defaultIndex = modeIndex.COMFORT
  T.trailerIndex = percentToIndex(jbeamData.trailerPercent or 115)
  T.levelingName = jbeamData.levelingController or T.levelingName

  local rampPercent = tonumber(jbeamData.rampPercentPerSecond) or 110
  rampIndexPerSecond = max(0.5, rampPercent / LADDER.stepPercent)

  currentIndex = nil
  appliedIndex = nil
  currentMode = "COMFORT"
  electrics.values.etk854hybrid_suspMode = "COMFORT"
  electrics.values.etk854hybrid_suspFirmness = indexToPercent(defaultIndex)
end

-- A vehicle reset restores the jbeam base damping, but adaptiveDampers.reset()
-- is empty: drop the applied state so the next frame re-applies the step.
-- The manual trailer switch survives.
local function reset()
  currentIndex = nil
  appliedIndex = nil
end

local function trailerMessage(on)
  guihooks.message(on and "Trailer mode ON (firmer damping)" or "Trailer mode OFF", 3, "etk854hybrid_level")
end

-- nil = auto (coupler detection in selfLeveling), true/false = pin.
local function setTrailerMode(b)
  local lev = getLeveling()
  if lev then
    lev.setTrailerMode(b)
    return
  end
  local old = T.manualTrailer == true
  -- Not "b ~= nil and b or nil": that turns false into nil.
  if b == nil then
    T.manualTrailer = nil
  else
    T.manualTrailer = b and true or false
  end
  if (T.manualTrailer == true) ~= old then
    trailerMessage(T.manualTrailer == true)
  end
end

local function toggleTrailerMode()
  local lev = getLeveling()
  if lev then
    lev.toggleTrailerMode()
    return
  end
  T.manualTrailer = not (T.manualTrailer == true)
  trailerMessage(T.manualTrailer)
end

local function getTrailerMode()
  local lev = getLeveling()
  if lev then
    return lev.getTrailerMode()
  end
  return T.manualTrailer == true
end

local function getLevelState()
  local lev = getLeveling()
  if lev and lev.getLevelState then
    return lev.getLevelState()
  end
  return "OFF"
end

M.init = init
M.reset = reset
M.updateGFX = updateGFX
M.setTrailerMode = setTrailerMode
M.toggleTrailerMode = toggleTrailerMode
M.getTrailerMode = getTrailerMode
M.getLevelState = getLevelState

return M
