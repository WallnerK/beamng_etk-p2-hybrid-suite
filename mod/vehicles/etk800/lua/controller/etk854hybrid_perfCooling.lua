-- ETK 854 ActiveHybrid - Performance cooling package: ram-air auxiliary
-- coolers + thermal telemetry. Declared by etk854hybrid_radiator_performance;
-- runs after etk854hybrid_cooling (order 91 > 90).
--
-- Aux coolers: the base radiator coefficients are module locals, so the heat
-- is taken directly from engine.thermals.coolantTemperature:
--   Q_aux = (Tc - Tenv) * auxCoolerCoef * thermostat * coolantEfficiency
--           * ram / (airCoefHalf + ram),   ram = ramAirFactor (0.8) * |frontAirflow|
-- Ram air only (no fan, no 0.1 floor): zero at standstill, so fan, wisps and
-- standstill charging match the standard package. engineThermalData is one
-- frame old here.
--
-- Telemetry probe on a car without the part (no effect on the vehicle):
--   controller.loadControllerExternal("etk854hybrid_perfCooling",
--     "etk854hybrid_perfCooling", {auxCoolerCoef = 0})
-- Not on a car with the part (duplicate controller name -> load error); an
-- external load survives a vehicle reset, not a reload.
--
-- Electrics (etk854hybrid_*): auxCoolerKW kW, egtC C, turboTempC C (exhaust +
-- coolant + oil, as the base turbo checks it), turboLimitC C, burnEff 0..1,
-- iceLoad 0..1 (engine.engineLoad), iceCombKW kW (actual crank power net of
-- friction, not the VCU command: combustionTorque * outputAV1, low-pass
-- powerTau), turboHot / coolantHot 0/1 (damageTracker).

local M = {}

M.type = "auxiliary"
M.relevantDevice = nil
M.defaultOrder = 91

local max = math.max
local min = math.min
local abs = math.abs

-- Tunables in one table (LuaJIT 60-upvalue limit), overridable from jbeam.
local CFG = {
  engineName = "mainEngine",
  auxCoolerCoef = 0, -- W/K; 0 = telemetry only (probe)
  -- base-model constants (combustionEngineThermals.lua)
  ramAirFactor = 0.8, -- radiator ram air = ramAirFactor * |frontAirflow|
  airCoefHalf = 15, -- m/s, airCoef = v / (15 + v)
  coolantSpecHeat = 4000, -- J/(kg K)
  minCoolantMass = 1.5, -- kg
  kelvinOffset = 273.15,
  powerTau = 0.1 -- s, low-pass of etk854hybrid_iceCombKW
}

local S = {
  ok = false,
  engine = nil,
  thermals = nil,
  td = nil, -- thermals.debugData.engineThermalData (updated in place)
  turboLimit = 1000,
  auxW = 0,
  combKW = 0
}

local function publish()
  local ev = electrics.values
  local th = S.thermals
  local exh = th.exhaustTemperature or 0
  local oil = th.oilTemperature or 0
  local tc = th.coolantTemperature or 0
  ev.etk854hybrid_auxCoolerKW = S.auxW * 0.001
  ev.etk854hybrid_egtC = exh
  ev.etk854hybrid_turboTempC = exh + tc + oil
  ev.etk854hybrid_turboLimitC = S.turboLimit
  ev.etk854hybrid_burnEff = S.td.engineEfficiency or 0
  ev.etk854hybrid_iceLoad = S.engine.engineLoad or 0
  ev.etk854hybrid_iceCombKW = S.combKW
  local dtr = damageTracker
  ev.etk854hybrid_turboHot = (dtr and dtr.getDamage("engine", "turbochargerHot")) and 1 or 0
  ev.etk854hybrid_coolantHot = (dtr and dtr.getDamage("engine", "coolantOverheating")) and 1 or 0
end

-- true when the base (water-cooled) thermal model is active
local function resolve()
  S.ok = false
  local engine = powertrain.getDevice(CFG.engineName)
  if not engine or not engine.thermals or not engine.thermalsEnabled then
    return false
  end
  local th = engine.thermals
  local td = th.debugData and th.debugData.engineThermalData
  if type(td) ~= "table" or type(th.coolantTemperature) ~= "number" then
    return false
  end
  local ed = v.data[CFG.engineName] or {}
  local tname = type(ed.turbocharger) == "string" and ed.turbocharger or "turbocharger"
  local tsec = v.data[tname]
  S.turboLimit = type(tsec) == "table" and tonumber(tsec.damageThresholdTemperature) or 1000
  S.engine = engine
  S.thermals = th
  S.td = td
  S.ok = true
  return true
end

local function updateGFX(dt)
  if not S.ok or dt <= 0 then
    return
  end
  local th = S.thermals
  local td = S.td
  local tc = th.coolantTemperature
  if type(tc) ~= "number" then
    return
  end
  local tEnv = obj:getEnvTemperature() - CFG.kelvinOffset
  local thermostat = td.thermostatStatus or 0
  local ram = abs(obj:getFrontAirflowSpeed()) * CFG.ramAirFactor

  S.auxW = 0
  if CFG.auxCoolerCoef > 0 and thermostat > 0 and ram > 0 then
    local qa = (tc - tEnv) * CFG.auxCoolerCoef * thermostat * (td.coolantEfficiency or 1) * (ram / (CFG.airCoefHalf + ram))
    if qa > 0 then
      local mass = max(td.coolantMass or 0, CFG.minCoolantMass)
      th.coolantTemperature = max(th.coolantTemperature - qa * dt / (mass * CFG.coolantSpecHeat), tEnv)
      S.auxW = qa
    end
  end

  local engine = S.engine
  local pKW = (engine.combustionTorque or 0) * (engine.outputAV1 or 0) * 0.001
  S.combKW = S.combKW + (pKW - S.combKW) * min(dt / CFG.powerTau, 1)

  publish()
end

local function resetState()
  S.auxW = 0
  S.combKW = 0
  if S.ok then
    publish()
  end
end

local function init(jbeamData)
  jbeamData = jbeamData or {}
  for k, val in pairs(jbeamData) do
    if CFG[k] ~= nil and type(val) == type(CFG[k]) then
      CFG[k] = val
    end
  end
  CFG.auxCoolerCoef = max(CFG.auxCoolerCoef, 0)
  CFG.airCoefHalf = max(CFG.airCoefHalf, 0.1)
  CFG.minCoolantMass = max(CFG.minCoolantMass, 0.1)
  CFG.coolantSpecHeat = max(CFG.coolantSpecHeat, 1)
  CFG.powerTau = max(CFG.powerTau, 0.01)
  if not resolve() then
    log("W", "etk854hybrid_perfCooling", "engine thermals unavailable - aux coolers / thermal telemetry inactive")
  end
  resetState()
end

local function initSecondStage()
  if not S.ok then
    resolve()
    resetState()
  end
end

local function reset()
  resetState()
end

-- State snapshot for dev tools (allocates, not for per-frame use).
local function getState()
  return {
    active = S.ok,
    auxCoolerCoef = CFG.auxCoolerCoef,
    auxW = S.auxW,
    turboLimit = S.turboLimit,
    combKW = S.combKW
  }
end

M.init = init
M.initSecondStage = initSecondStage
M.reset = reset
M.updateGFX = updateGFX
M.getState = getState

return M
