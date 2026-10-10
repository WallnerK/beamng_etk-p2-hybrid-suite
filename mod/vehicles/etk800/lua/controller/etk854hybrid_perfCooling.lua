-- ETK 854 ActiveHybrid - Performance cooling package: auxiliary ram-air
-- coolers + thermal telemetry.
--
-- Declared ONLY by the part etk854hybrid_radiator_performance
-- (etk854hybrid_cooling.jbeam), i.e. only on the Performance configs. The
-- standard cooling controller etk854hybrid_cooling.lua (PWM fan, wisps) is
-- not touched and runs unchanged before this one (defaultOrder 90 < 91).
--
-- Aux coolers (side radiators in the bumper air intakes): the base thermal
-- model (lua/vehicle/powertrain/combustionEngineThermals.lua) knows one
-- coolant radiator; its coefficients are module locals without setters. Like
-- the PWM fan emulation, this module removes its heat directly from the
-- public integrator state engine.thermals.coolantTemperature:
--   Q_aux = (Tc - Tenv) * auxCoolerCoef * thermostat * coolantEfficiency
--           * ram / (airCoefHalf + ram),     ram = 0.8 * |frontAirflow|
-- Same form as the base radiator (:765-772) but WITHOUT the 0.1 no-fan floor
-- and without any fan: ram air only. At standstill Q_aux is 0, so the PWM fan
-- (105 / 101 C), the wisps and the N-charge cooling behave exactly as on the
-- standard package. thermostat / coolantEfficiency / coolantMass are read
-- from thermals.debugData.engineThermalData (one frame old, see
-- etk854hybrid_cooling.lua header), so a closed thermostat or a leaking
-- circuit cuts the aux coolers exactly like the main radiator.
-- auxCoolerCoef 4600 W/K + 5400 W/K main radiator = 10000 W/K at speed.
--
-- Telemetry probe: on a car WITHOUT the Performance radiator this module can
-- be loaded at runtime as a pure telemetry probe (aux coolers off, no effect
-- on the vehicle):
--   controller.loadControllerExternal("etk854hybrid_perfCooling",
--     "etk854hybrid_perfCooling", {auxCoolerCoef = 0})
-- Not on a car that already has the part (duplicate controller name ->
-- load error). An external load survives a vehicle reset, not a reload.
--
-- Published electrics (every frame while the engine thermals are active):
--   etk854hybrid_auxCoolerKW  kW    heat removed by the aux coolers
--   etk854hybrid_egtC         C     exhaust temperature (thermals.exhaustTemperature)
--   etk854hybrid_turboTempC   C     exhaust + coolant + oil, the value the base
--                                   turbocharger compares against its limit
--                                   (turbocharger.lua:200)
--   etk854hybrid_turboLimitC  C     turbocharger damageThresholdTemperature
--                                   (default 1000, turbocharger.lua:441)
--   etk854hybrid_burnEff      0..1  burn efficiency the thermals use this frame
--                                   (engineThermalData.engineEfficiency)
--   etk854hybrid_iceLoad      0..1  engine.engineLoad (the smoothed load the
--                                   thermals index the efficiency table with)
--   etk854hybrid_iceCombKW    kW    combustionTorque * outputAV1, low-pass
--                                   powerTau (net of friction,
--                                   combustionEngine.lua:802) - measured crank
--                                   power, unlike the VCU's commanded ICE torque
--   etk854hybrid_turboHot     0/1   damageTracker engine.turbochargerHot
--   etk854hybrid_coolantHot   0/1   damageTracker engine.coolantOverheating
-- No sound, no particles, no battery draw.

local M = {}

M.type = "auxiliary"
M.relevantDevice = nil
M.defaultOrder = 91

local max = math.max
local min = math.min
local abs = math.abs

-- All tunables in ONE table (LuaJIT 60-upvalue limit); every key can be
-- overridden from jbeam (same type only).
local CFG = {
  engineName = "mainEngine",
  auxCoolerCoef = 0, -- W/K; 0 = telemetry only (probe)
  airCoefHalf = 15, -- combustionEngineThermals.lua:766
  coolantSpecHeat = 4000, -- :152
  minCoolantMass = 1.5, -- :154
  kelvinOffset = 273.15,
  powerTau = 0.1 -- s, low-pass of etk854hybrid_iceCombKW
}

-- Runtime state, also one table.
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

-- Resolves the engine, its thermals module and the turbo limit. Returns true
-- when the base thermal model is active (water cooled).
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
  local ram = abs(obj:getFrontAirflowSpeed()) * 0.8

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
