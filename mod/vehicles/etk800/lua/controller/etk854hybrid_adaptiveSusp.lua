-- ETK 854 ActiveHybrid - Adaptivfahrwerk-Bruecke
--
-- Koppelt die Daempferhaerte an den VCU-Fahrmodus
-- electrics.values.etk854hybrid_deMode ("COMFORT" / "ECO_PRO" / "SPORT").
--
-- Der Controller schreibt NICHT selbst an Beams. Er waehlt lediglich eine
-- Modusstufe des Stock-Actuators
--   lua/vehicle/controller/drivingDynamics/actuators/adaptiveDampers.lua
-- aus, der in etk854hybrid_chassis.jbeam unter den Namen
-- "etk854hybrid_damperF" / "etk854hybrid_damperR" instanziert ist und dort
-- eine Leiter p080..p150 (Daempfung in 5-%-Schritten) mitbringt.
--
-- Weiche Uebergaenge entstehen dadurch, dass die Leiter Stufe fuer Stufe
-- abgefahren wird (Vorgabe 110 %/s, also ca. 0.45 s von 90 % auf 140 %).
-- Jeder Einzelschritt aendert die Daempfung um 5 % - das ist unterhalb der
-- Wahrnehmungsschwelle und regt den Solver nicht an, im Gegensatz zu einem
-- direkten Sprung soft -> hard.
--
-- Publizierte Kanaele:
--   electrics.values.etk854hybrid_suspMode     (String: COMFORT/ECO_PRO/SPORT)
--   electrics.values.etk854hybrid_suspFirmness (Zahl: aktuelle Haerte in %)
--
-- Anhaengermodus (P6): etk854hybrid_trailerMode == 1 hebt die Leiterstufe
-- auf mindestens trailerPercent an (UNTERGRENZE, SPORT bleibt straffer).
-- Erkennung, Niveauregulierung und die Kanaele trailerMode/levelState gehoeren
-- dem Controller etk854hybrid_selfLeveling (Teil etk854hybrid_spring_R); die
-- API hier (setTrailerMode/toggleTrailerMode/getTrailerMode/getLevelState)
-- delegiert nur. Ist die Serienfeder hinten verbaut (kein selfLeveling),
-- fuehrt diese Bruecke einen manuellen Schalter und schreibt die beiden
-- Kanaele selbst -> genau ein Schreiber pro Kanal.

local M = {}

M.type = "auxiliary"
-- adaptiveDampers laeuft mit defaultOrder 80; wir kommen danach dran.
M.defaultOrder = 85

local max = math.max
local min = math.min
local floor = math.floor

-- Leiter, identisch zu den "modes"-Zeilen in etk854hybrid_chassis.jbeam.
local LADDER = {
  minPercent = 80,
  maxPercent = 150,
  stepPercent = 5
}
local LADDER_STEPS = (LADDER.maxPercent - LADDER.minPercent) / LADDER.stepPercent -- 14

local frontName = "etk854hybrid_damperF"
local rearName = "etk854hybrid_damperR"

local modeIndex = {} -- deMode -> Leiterindex 0..14
local defaultIndex = 4 -- 100 %

local rampIndexPerSecond = 4 -- wird aus rampPercentPerSecond berechnet

local currentIndex = nil -- float, faehrt zum Ziel
local appliedIndex = nil -- zuletzt tatsaechlich gesetzte ganzzahlige Stufe
local currentMode = "COMFORT"

local paramTable = {damperMode = "p100"} -- wiederverwendet, kein GC-Druck

-- Anhaenger-Zustand gebuendelt in EINER Tabelle (LuaJIT-Limit 60 Upvalues).
local T = {
  levelingName = "etk854hybrid_selfLeveling",
  trailerIndex = 7, -- 115 %
  manualTrailer = nil -- nur ohne selfLeveling: Fahrer-Schalter
}

-- controller.getController (controller.lua:232) statt ...Safe: liefert nil
-- ohne "Didn't find controller"-Logzeile pro Frame, wenn die Serienfeder
-- hinten verbaut ist. Reiner Tabellenzugriff, daher jeden Frame ok.
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

-- getControllerSafe liefert bei fehlendem Part eine No-Op-Tabelle
-- (lua/vehicle/controller.lua:236) -> kein nil-Check noetig, und die
-- Abstimmung funktioniert auch, wenn nur eine der beiden Achsen das
-- Adaptiv-Teil verbaut hat.
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
    -- selfLeveling (defaultOrder 83) hat diesen Frame schon publiziert.
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

-- adaptiveDampers.reset() ist leer (Zeile 42-43): nach einem Fahrzeug-Reset
-- stellt die Engine die jbeam-Basiswerte wieder her, der Actuator glaubt aber
-- weiter an die alte Stufe. Darum hier den Anwendungszustand verwerfen, damit
-- der naechste Frame die aktuelle Stufe frisch aufsetzt.
-- Der manuelle Anhaenger-Schalter (T.manualTrailer) bleibt ueber den Reset
-- erhalten - Fahrereinstellung.
local function reset()
  currentIndex = nil
  appliedIndex = nil
end

local function trailerMessage(on)
  guihooks.message(on and "Trailer mode ON (firmer damping)" or "Trailer mode OFF", 3, "etk854hybrid_level")
end

-- nil = Automatik (Kupplungserkennung im selfLeveling), true/false = Pin.
local function setTrailerMode(b)
  local lev = getLeveling()
  if lev then
    lev.setTrailerMode(b)
    return
  end
  local old = T.manualTrailer == true
  -- Explizit statt "b ~= nil and b or nil": das wuerde false zu nil machen.
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
