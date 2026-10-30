-- ETK 854 ActiveHybrid - electric air compressor + trailer air supply
--
-- Part etk854hybrid_load_compressor: trunk piston compressor on a 40 l
-- receiver, pressureTank "etk854hybrid_airTank" (isPrimarySupply) with the
-- hitch nodes hyb_twp / hyb_twb as PTO consumer couplers (missing ones are
-- skipped by the tank).
--
-- Trailer air (stock code only): on coupling the trailer requests our export
-- "pneumaticPTOPressure_<objId>", which the tank writes as min(current, supply)
-- * consumerPressureCoef (stays 0 without isPrimarySupply); the trailer's
-- returned flow is subtracted from our tank. Trailer spring brakes apply
-- below 40 psi supply and release fully at 60 psi (413685 Pa) -> lowWarnBar
-- 4.14. Its service / parking brake follow our electrics.
--
-- Trailer supply protection valve (receiver, bar gauge): fed right at 40 psi
-- the trailer limit-cycles (its spring chambers draw more than we deliver and
-- pin the receiver there), so the supply opens at >= trailerValveOpenBar and
-- closes after trailerValveCloseDelayS below trailerValveCloseBar.
--  * closed = tank.consumerPressureCoef 0, written every frame before the
--    tank's updateGFX (tank reset restores 1; the stock setter only adds a
--    UI message)
--  * with supply 0 the trailer keeps its air and applies its spring brakes;
--    its service brakes still work from its own tank
--  * open 6.0: the mixed pressure lands above the trailer's release point;
--    close 3.4: above the 40 psi band, no chatter while an empty trailer tank
--    charges (mixes to ~3.8); the delay rides out the PTO sync overshoot
--
-- Compressor: HV-electric (EV mode, standstill), isothermal like the stock
-- powertrain/compressor.lua (storedEnergy = p * V):
--   pr      = p_tank,abs / p_env
--   Q_pow   = P_rated * eta_iso / (p_env * ln pr)   free-air flow, power limit
--   Q       = min(Q_max, Q_pow) * spin               Q_max = displacement limit
--   P_el    = clamp(Q_max * p_env * ln pr / eta_iso, idle * P_rated, P_rated) * spin
--   dE_tank = p_env * Q * dt
--   E_batt -= P_el / eta_conv * dt
-- Defaults: ~120 l/min free air at 8 bar, 7.2 -> 8.6 bar in ~27 s.
-- Pressure switch cutIn / cutOut + dryer purge, restartDelayS, relief valve;
-- inhibited below minSoc (until resumeSoc) and without ignition.
-- The trailer's stock air-brake channels (CFG trailer*Electric) are requested
-- the same way and republished.
--
-- Publishes (etk854hybrid_ prefix; bar = gauge):
--   airState OFF | FILL | READY | LOW | NO TANK
--   airInhibit NONE | SWITCH | KEY | SOC | NO HV | FAULT | NO TANK
--   airPressureBar, airSupplyBar (export, 0 while the valve is closed)
--   airPowerW (HV draw), airFlowLpm (free air), airBattWh (since reset)
--   airCompressor, airEnabled, airTrlValve, airRelief, airLeak, airTrailer (0/1)
--   trailerAirBar, trailerSpringBar, trailerServiceBar,
--   trailerSpringCoef (spring-brake share 0..1, 413685 Pa -> 0); -1 = unknown
-- API: setEnabled(on), toggle(), getState(), debugSetPressure(bar), debugVent(barPerS)

local M = {}

M.type = "auxiliary"
M.relevantDevice = nil
M.defaultOrder = 95

local max = math.max
local min = math.min
local mlog = math.log

-- Tunables (one table: LuaJIT 60-upvalue limit); jbeam keys override, type-matched.
local CFG = {
  name = "etk854hybrid_airCompressor",
  tankName = "etk854hybrid_airTank",
  batteryName = "mainBattery",
  enabledOnSpawn = true,
  ratedPowerW = 1500, -- electrical, full load
  isothermalEfficiency = 0.30, -- wire-to-air
  maxFreeAirLpm = 270, -- displacement limit
  idlePowerFraction = 0.30, -- power floor while running
  converterEfficiency = 0.95, -- HV -> motor
  cutInBar = 7.2,
  cutOutBar = 8.6,
  reliefBar = 8.9,
  lowWarnBar = 4.14, -- trailer spring-brake release (60 psi)
  -- trailer supply valve, receiver bar gauge
  trailerValveOpenBar = 6.0,
  trailerValveCloseBar = 3.4,
  trailerValveCloseDelayS = 0.5,
  minSoc = 0.08, -- = VCU SOC_ICE_FORCE
  resumeSoc = 0.10,
  spinUpS = 0.4,
  spinDownS = 1.2,
  restartDelayS = 3.0,
  purgeMinRunS = 1.0,
  soundNode = "hyb_ldc6",
  compressorLoopEvent = "event:>Vehicle>Pneumatics>Air_Compressor",
  purgeEvent = "event:>Vehicle>Pneumatics>Air_Dryer_Purge",
  compressorSoundVolume = 0.5,
  purgeSoundVolume = 0.5,
  soundRpm = 1450, -- stock event pitch = rpm
  trailerTankElectric = "mainAirTank_pressureRelative",
  trailerParkingElectric = "airbrakes_pressure_parking",
  trailerServiceElectric = "airbrakes_pressure_service",
  springReleasePa = 413685,
  pollInterval = 0.5,
  messages = true,
  msgCategory = "vehicle.etk854hybrid.air"
}

local S = {
  objId = 0,
  tank = nil,
  battery = nil,
  couplers = {}, -- own consumer coupler cid -> true
  couplerCount = 0,
  enabled = true,
  demand = false, -- pressure switch closed
  running = false, -- motor commanded on
  spin = 0,
  runTime = 0,
  offTimer = 1e9,
  socInhibit = false,
  inhibit = "NONE",
  state = "NO TANK",
  powerW = 0,
  flowLpm = 0,
  battWh = 0,
  airJ = 0,
  relief = false,
  leak = false,
  pBar = 0,
  ventBarPerS = 0,
  sfx = nil,
  soundNodeId = 0,
  trl = nil, -- {id, node, own}
  trlValve = false, -- supply valve open
  trlLowT = 0, -- s below the closing pressure
  trlMsg = false, -- "blocked" hint pending until the trailer tank channel arrives
  pollT = 0
}

local function message(text)
  if CFG.messages and guihooks and guihooks.message then
    guihooks.message(text, 3, CFG.msgCategory)
  end
end

local function envPressure()
  return powertrain.currentEnvPressure or 101325
end

local function barText(bar)
  return string.format("%.1f", bar or 0)
end

-- ------------------------------------------------------------------ trailer
local function clearTrailerChannels()
  local ev = electrics.values
  ev[CFG.trailerTankElectric] = nil
  ev[CFG.trailerParkingElectric] = nil
  ev[CFG.trailerServiceElectric] = nil
end

local function linkTrailer(own, obj2id, obj2nodeId)
  if S.trl and S.trl.id == obj2id and S.trl.own == own then
    return
  end
  S.trl = {id = obj2id, node = obj2nodeId, own = own}
  clearTrailerChannels()
  -- same request the tank makes for its flow channel
  if beamstate and beamstate.sendExportCouplerData then
    beamstate.sendExportCouplerData(obj2id, obj2nodeId, {electrics = {CFG.trailerTankElectric, CFG.trailerParkingElectric, CFG.trailerServiceElectric}})
  end
  -- hint waits for the trailer tank channel: none for a trailer without air brakes
  S.trlMsg = S.tank ~= nil
end

local function unlinkTrailer()
  S.trlMsg = false
  if S.trl then
    S.trl = nil
    clearTrailerChannels()
  end
end

-- coupled trailer that reports an air tank
local function trailerHasAir()
  return S.trl ~= nil and type(electrics.values[CFG.trailerTankElectric]) == "number"
end

-- Reconcile with beamstate.attachedCouplers (a reset clears it without a
-- detach event).
local function pollCouplers()
  local ac = beamstate and beamstate.attachedCouplers
  if not ac or S.couplerCount == 0 then
    unlinkTrailer()
    return
  end
  for cid in pairs(S.couplers) do
    local a = ac[cid]
    if a and a.obj2id and a.obj2id ~= S.objId then
      linkTrailer(cid, a.obj2id, a.obj2nodeId)
      return
    end
  end
  unlinkTrailer()
end

local function publishTrailer(ev)
  if not S.trl then
    ev.etk854hybrid_airTrailer = 0
    ev.etk854hybrid_trailerAirBar = -1
    ev.etk854hybrid_trailerSpringBar = -1
    ev.etk854hybrid_trailerServiceBar = -1
    ev.etk854hybrid_trailerSpringCoef = -1
    return
  end
  local tp = ev[CFG.trailerTankElectric]
  local sp = ev[CFG.trailerParkingElectric]
  local sv = ev[CFG.trailerServiceElectric]
  ev.etk854hybrid_airTrailer = 1
  if S.trlMsg and type(tp) == "number" then
    S.trlMsg = false
    if not S.trlValve then
      message("Trailer air supply blocked until " .. barText(CFG.trailerValveOpenBar) .. " bar reservoir pressure (now " .. barText(S.pBar) .. " bar)")
    end
  end
  ev.etk854hybrid_trailerAirBar = type(tp) == "number" and tp * 1e-5 or -1
  ev.etk854hybrid_trailerSpringBar = type(sp) == "number" and sp * 1e-5 or -1
  ev.etk854hybrid_trailerServiceBar = type(sv) == "number" and sv * 1e-5 or -1
  if type(sp) == "number" and CFG.springReleasePa > 0 then
    ev.etk854hybrid_trailerSpringCoef = min(max(1 - sp / CFG.springReleasePa, 0), 1)
  else
    ev.etk854hybrid_trailerSpringCoef = -1
  end
end

-- ------------------------------------------------------------------ publish
local function publish()
  local ev = electrics.values
  ev.etk854hybrid_airState = S.state
  ev.etk854hybrid_airPressureBar = S.tank and S.pBar or 0
  ev.etk854hybrid_airCompressor = S.running and 1 or 0
  ev.etk854hybrid_airEnabled = S.enabled and 1 or 0
  ev.etk854hybrid_airInhibit = S.inhibit
  ev.etk854hybrid_airPowerW = S.powerW
  ev.etk854hybrid_airFlowLpm = S.flowLpm
  ev.etk854hybrid_airBattWh = S.battWh
  ev.etk854hybrid_airRelief = S.relief and 1 or 0
  ev.etk854hybrid_airLeak = S.leak and 1 or 0
  local supply = 0
  local tank = S.tank
  if tank and tank.pneumaticPTOConsumerPressureElectricsName then
    local p = ev[tank.pneumaticPTOConsumerPressureElectricsName]
    if type(p) == "number" then
      supply = max(0, (p - envPressure()) * 1e-5)
    end
  end
  ev.etk854hybrid_airSupplyBar = supply
  ev.etk854hybrid_airTrlValve = (tank and S.trlValve) and 1 or 0
  publishTrailer(ev)
end

-- ------------------------------------------------------------------- sound
local function updateSound()
  if not S.sfx then
    return
  end
  local spin = S.spin
  -- as stock compressor.lua: volume, pitch = rpm, colour = 1 while pumping
  obj:setVolumePitchCT(S.sfx, CFG.compressorSoundVolume * spin, CFG.soundRpm * spin, S.running and 1 or 0, 0)
end

local function purge()
  obj:playSFXOnce(CFG.purgeEvent, S.soundNodeId, CFG.purgeSoundVolume, 1)
end

-- ----------------------------------------------------------------- resolve
local function resolve()
  S.tank = nil
  S.couplers = {}
  S.couplerCount = 0
  local tank = energyStorage.getStorage(CFG.tankName)
  if tank and tank.type == "pressureTank" and (tank.capacity or 0) > 0 and type(tank.storedEnergy) == "number" then
    S.tank = tank
    for cid in pairs(tank.pneumaticPTOConsumerCouplerNodeLookup or {}) do
      S.couplers[cid] = true
      S.couplerCount = S.couplerCount + 1
    end
  end
  local bat = energyStorage.getStorage(CFG.batteryName)
  S.battery = (bat and bat.setStoredEnergy and type(bat.storedEnergy) == "number") and bat or nil
  S.soundNodeId = (beamstate.nodeNameMap and beamstate.nodeNameMap[CFG.soundNode]) or 0
end

-- -------------------------------------------------------------- inhibits
local function evalInhibit(tank)
  -- broken receiver / torn hose: onBreak sets the leak rate to maxLeakRate
  S.leak = (tank.currentLeakRate or 0) > 0
  if S.leak and (tank.maxLeakRate or 0) > 0 and tank.currentLeakRate >= tank.maxLeakRate * 0.99 then
    return "FAULT"
  end
  if not S.battery then
    return "NO HV"
  end
  if not S.enabled then
    return "SWITCH"
  end
  if (electrics.values.ignitionLevel or 0) < 2 then
    return "KEY"
  end
  local soc = S.battery.remainingRatio or 1
  if S.socInhibit then
    if soc >= CFG.resumeSoc then
      S.socInhibit = false
    end
  elseif soc < CFG.minSoc then
    S.socInhibit = true
  end
  if S.socInhibit then
    return "SOC"
  end
  return "NONE"
end

-- ------------------------------------------- trailer supply protection valve
local function applyTrailerValve(tank)
  if tank then
    tank.consumerPressureCoef = S.trlValve and 1 or 0
  end
end

local function updateTrailerValve(tank, pBar, dt)
  local open = S.trlValve
  if open then
    if pBar < CFG.trailerValveCloseBar then
      S.trlLowT = S.trlLowT + dt
      if S.trlLowT >= CFG.trailerValveCloseDelayS then
        open = false
      end
    else
      S.trlLowT = 0
    end
  elseif pBar >= CFG.trailerValveOpenBar then
    open = true
  end
  if not open then
    S.trlLowT = 0
  end
  if open ~= S.trlValve then
    S.trlValve = open
    if trailerHasAir() then
      if open then
        message("Trailer air supply open (reservoir " .. barText(pBar) .. " bar)")
      else
        message("Trailer air supply shut off (reservoir " .. barText(pBar) .. " bar) - trailer spring brakes applied")
      end
    end
  end
  applyTrailerValve(tank)
end

-- ----------------------------------------------------------------- update
local function updateGFX(dt)
  if dt <= 0 then
    return
  end

  S.pollT = S.pollT + dt
  if S.pollT >= CFG.pollInterval then
    S.pollT = 0
    pollCouplers()
  end

  local tank = S.tank
  if not tank then
    S.state = "NO TANK"
    S.inhibit = "NO TANK"
    S.running = false
    S.spin = 0
    S.powerW = 0
    S.flowLpm = 0
    publish()
    return
  end

  local env = envPressure()
  local V = tank.capacity
  -- storedEnergy is live; tank.currentPressure lags one frame
  local pAbs = tank.storedEnergy * tank.invCapacity

  -- dev hook: constant vent (simulated consumption / leak)
  if S.ventBarPerS > 0 then
    local e = tank.storedEnergy - S.ventBarPerS * 1e5 * V * dt
    tank.storedEnergy = max(env * V, e)
    pAbs = tank.storedEnergy * tank.invCapacity
  end

  local pBar = (pAbs - env) * 1e-5

  S.inhibit = evalInhibit(tank)
  local allowed = S.inhibit == "NONE"

  -- pressure switch with hysteresis
  if S.demand then
    if pBar >= CFG.cutOutBar then
      S.demand = false
    end
  elseif pBar <= CFG.cutInBar then
    S.demand = true
  end

  local wasRunning = S.running
  local run = allowed and S.demand and (wasRunning or S.offTimer >= CFG.restartDelayS)
  S.running = run
  if run then
    S.runTime = S.runTime + dt
    S.offTimer = 0
  else
    if wasRunning then
      -- dryer purge when the unit stops after a real run
      if S.runTime >= CFG.purgeMinRunS then
        purge()
      end
      S.runTime = 0
    end
    S.offTimer = S.offTimer + dt
  end

  -- motor speed: soft start / run-down
  if run then
    S.spin = min(1, S.spin + dt / S.spinUpS)
  else
    S.spin = max(0, S.spin - dt / S.spinDownS)
  end

  local powerW = 0
  local q = 0
  if run and S.spin > 0 then
    local pr = max(pAbs / env, 1.0001)
    local lnPr = mlog(pr)
    local eta = S.eta
    local qMax = S.qMax
    local pRated = CFG.ratedPowerW
    local qPow = pRated * eta / (env * lnPr)
    q = min(qMax, qPow) * S.spin
    local pNeed = qMax * env * lnPr / eta
    powerW = min(pRated, max(CFG.idlePowerFraction * pRated, pNeed)) * S.spin
    local dE = env * q * dt
    tank.storedEnergy = tank.storedEnergy + dE
    S.airJ = S.airJ + dE
    local battW = powerW / S.convEff
    local bat = S.battery
    bat:setStoredEnergy(max(bat.storedEnergy - battW * dt, 0))
    S.battWh = S.battWh + battW * dt / 3600
    powerW = battW
  end
  S.powerW = powerW
  S.flowLpm = q * 60000

  -- relief valve
  pAbs = tank.storedEnergy * tank.invCapacity
  local reliefAbs = S.reliefPa + env
  S.relief = pAbs > reliefAbs
  if S.relief then
    tank.storedEnergy = reliefAbs * V
    pAbs = reliefAbs
  end
  pBar = (pAbs - env) * 1e-5
  S.pBar = pBar

  -- before the tank's updateGFX: the coef applies to this frame's export
  updateTrailerValve(tank, pBar, dt)

  if pBar < CFG.lowWarnBar then
    S.state = "LOW"
  elseif S.running then
    S.state = "FILL"
  elseif not allowed then
    S.state = "OFF"
  else
    S.state = "READY"
  end

  updateSound()
  publish()
end

-- ---------------------------------------------------------------- couplers
local function onCouplerAttached(nodeId, obj2id, obj2nodeId, attachSpeed, attachEnergy)
  if S.couplers[nodeId] and obj2id and obj2id ~= S.objId then
    linkTrailer(nodeId, obj2id, obj2nodeId)
  end
end

-- beamstate already removed the entry: re-scan (HD combo mount has two couplers)
local function onCouplerDetached(nodeId, obj2id, obj2nodeId, breakForce)
  if S.couplers[nodeId] and S.trl then
    pollCouplers()
  end
end

-- ------------------------------------------------------------------- API
local function setEnabled(on)
  on = on and true or false
  if on == S.enabled then
    return S.enabled
  end
  S.enabled = on
  message(on and "Air compressor ON" or "Air compressor OFF")
  return S.enabled
end

local function toggle()
  return setEnabled(not S.enabled)
end

local function getState()
  local tank = S.tank
  return {
    hasTank = tank ~= nil,
    state = S.state,
    inhibit = S.inhibit,
    enabled = S.enabled,
    demand = S.demand,
    running = S.running,
    spin = S.spin,
    pressureBar = S.pBar,
    powerW = S.powerW,
    flowLpm = S.flowLpm,
    battWh = S.battWh,
    airJ = S.airJ,
    relief = S.relief,
    leak = S.leak,
    couplerCount = S.couplerCount,
    trailerId = S.trl and S.trl.id or nil,
    trailerNode = S.trl and S.trl.node or nil,
    ownNode = S.trl and S.trl.own or nil,
    supplyPressure = tank and tank.supplyPressure or nil,
    trailerValve = S.trlValve,
    trailerValveOpenBar = CFG.trailerValveOpenBar,
    trailerValveCloseBar = CFG.trailerValveCloseBar,
    trailerValveCloseDelayS = CFG.trailerValveCloseDelayS,
    consumerPressureCoef = tank and tank.consumerPressureCoef or nil,
    storedEnergy = tank and tank.storedEnergy or nil,
    capacity = tank and tank.capacity or nil,
    ventBarPerS = S.ventBarPerS,
    soundNode = S.soundNodeId,
    sfxCreated = S.sfx ~= nil
  }
end

-- Dev hooks: set the receiver gauge pressure / vent it at bar/s (0 stops).
local function debugSetPressure(bar)
  bar = tonumber(bar)
  local tank = S.tank
  if not tank or not bar then
    return false
  end
  local env = envPressure()
  tank.storedEnergy = (max(bar, 0) * 1e5 + env) * tank.capacity
  return true
end

local function debugVent(barPerS)
  S.ventBarPerS = max(tonumber(barPerS) or 0, 0)
  return S.ventBarPerS
end

-- ------------------------------------------------------------ lifecycle
local function resetState()
  S.demand = false
  S.running = false
  S.spin = 0
  S.runTime = 0
  S.offTimer = 1e9
  S.socInhibit = false
  S.powerW = 0
  S.flowLpm = 0
  S.battWh = 0
  S.airJ = 0
  S.relief = false
  S.leak = false
  S.ventBarPerS = 0
  S.pollT = 0
  unlinkTrailer()
  local tank = S.tank
  if tank then
    S.pBar = (tank.storedEnergy * tank.invCapacity - envPressure()) * 1e-5
    S.state = S.pBar < CFG.lowWarnBar and "LOW" or "READY"
    S.inhibit = "NONE"
    -- a fresh receiver above the opening pressure starts with the valve open
    S.trlValve = S.pBar >= CFG.trailerValveOpenBar
    S.trlLowT = 0
    applyTrailerValve(tank)
  else
    S.pBar = 0
    S.state = "NO TANK"
    S.inhibit = "NO TANK"
    S.trlValve = false
  end
  if S.sfx then
    obj:setVolumePitchCT(S.sfx, 0, 0, 0, 0)
  end
  publish()
end

local function init(jbeamData)
  jbeamData = jbeamData or {}
  for k, val in pairs(jbeamData) do
    if CFG[k] ~= nil and type(val) == type(CFG[k]) then
      CFG[k] = val
    end
  end
  CFG.cutOutBar = max(CFG.cutOutBar, CFG.cutInBar + 0.2)
  CFG.reliefBar = max(CFG.reliefBar, CFG.cutOutBar + 0.1)
  -- valve: open below the cut-out (or it never opens), close below open
  CFG.trailerValveOpenBar = min(CFG.trailerValveOpenBar, CFG.cutOutBar - 0.1)
  CFG.trailerValveCloseBar = min(CFG.trailerValveCloseBar, CFG.trailerValveOpenBar - 0.5)
  CFG.trailerValveCloseDelayS = max(CFG.trailerValveCloseDelayS, 0)
  CFG.ratedPowerW = max(CFG.ratedPowerW, 0)
  CFG.resumeSoc = max(CFG.resumeSoc, CFG.minSoc)
  -- derived calibration constants (from the final CFG; in S, out of the override loop)
  S.eta = max(CFG.isothermalEfficiency, 0.01)
  S.qMax = CFG.maxFreeAirLpm / 60000 -- m^3/s free air
  S.spinUpS = max(CFG.spinUpS, 0.01)
  S.spinDownS = max(CFG.spinDownS, 0.01)
  S.convEff = max(CFG.converterEfficiency, 0.01)
  S.reliefPa = CFG.reliefBar * 1e5
  S.objId = obj:getId()
  S.enabled = CFG.enabledOnSpawn
  S.sfx = nil
  resolve()
  resetState()
  if not S.tank then
    log("W", "etk854hybrid_airCompressor", "pressure tank '" .. tostring(CFG.tankName) .. "' not found - compressor idle")
  end
end

local function initSounds()
  if not S.tank then
    return
  end
  -- same source setup as the stock compressor.lua initSounds
  S.sfx = obj:createSFXSource2(CFG.compressorLoopEvent, "AudioDefaultLoop3D", "etk854hybridAirCompressor", S.soundNodeId, 1)
  if S.sfx then
    obj:playSFX(S.sfx)
    obj:setVolumePitchCT(S.sfx, 0, 0, 0, 0)
  end
end

-- The driver switch survives a reset; energyStorage.reset restores the receiver.
local function reset()
  resolve()
  resetState()
end

M.init = init
M.initSounds = initSounds
M.reset = reset
M.updateGFX = updateGFX
M.onCouplerAttached = onCouplerAttached
M.onCouplerDetached = onCouplerDetached
M.setEnabled = setEnabled
M.toggle = toggle
M.getState = getState
M.debugSetPressure = debugSetPressure
M.debugVent = debugVent

return M
