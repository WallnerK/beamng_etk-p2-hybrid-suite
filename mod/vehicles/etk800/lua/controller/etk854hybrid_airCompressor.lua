-- ETK 854 ActiveHybrid - electric air compressor + trailer air supply
--
-- Part: etk854hybrid_load_compressor (etk854hybrid_loads.jbeam), the trunk
-- compressor unit of the workshop load (piston compressor on a 40 l air
-- receiver, the part's flexbody is the receiver). The same part declares the
-- pressureTank "etk854hybrid_airTank" (lua/vehicle/energyStorage/
-- pressureTank.lua) with isPrimarySupply = true and our hitch coupler nodes
-- (hyb_twp pintle, hyb_twb ball, etk854hybrid_towing.jbeam) as
-- pneumaticPTOConsumerCouplerNodeNames. Node names that do not exist in the
-- spawned vehicle (no hitch / ball-only mount) are skipped by the tank
-- (pressureTank.lua:360-364, :368-372 check beamstate.nodeNameMap first), the
-- same way the stock md_series / us_semi / wl40 tanks list "pintle" and
-- "fwk2" regardless of which hitch is fitted.
--
-- How a coupled trailer gets its air (stock code only, nothing overridden):
--  * The trailer tank (e.g. dolly.zip vehicles/dolly/dolly.jbeam mainAirTank,
--    pneumaticPTOSupplyCouplerNodeNames ["pintle"]) asks us on coupling to
--    export electrics "pneumaticPTOPressure_<our objectId>"
--    (pressureTank.lua:116-130 -> beamstate.sendExportCouplerData,
--    beamstate.lua:390-393, :472-486, transmitted every frame :721-737).
--  * Our tank writes that channel every frame as min(current, supply) * coef;
--    isPrimarySupply makes supply = current (pressureTank.lua:39-50). WITHOUT
--    isPrimarySupply the exported pressure would stay 0.
--  * The trailer fills its tank with (supply - own) * 1e-6 m^3/(s*Pa) and
--    reports the energy flow back as "pneumaticPTOFlow_<our objectId>", which
--    our tank subtracts (:16-36 trailer side, :52-54 our side).
--  * Trailer airbrakes (lua/vehicle/controller/pneumatics/airbrakes.lua
--    :159-183) FORCE the spring brakes on while the trailer tank's
--    supplyPressure < 275790 Pa (40 psi) + ambient; above that they follow
--    the parking brake command (the 413685 Pa / 60 psi threshold only re-arms
--    the smart parking brake). The spring actuators fill from the trailer
--    tank up to 413685 Pa (:183), and the stock curve is 0 Pa -> full torque,
--    413685 Pa -> 0 (bogie_suspension.jbeam bogie_brakeValve), so a complete
--    release needs >= 60 psi (4.14 bar) in the trailer tank - that is our
--    lowWarnBar. Service and
--    parking brake commands are OUR electrics brake / parkingbrake: the
--    trailer's coupler node carries "importInputs": ["brake","parkingbrake"]
--    (dolly_single.jbeam:116), beamstate exports electrics.values[k] or
--    input[k] (:730-734) and the trailer replays them with input.event
--    (:503-506). Nothing to do on our side, but: our parking brake applied =
--    trailer spring brakes applied.
--
-- Trailer supply protection valve (2026-10, live run 20261009-172556_air_supply):
-- the dolly's auto-apply compares its SUPPLY pressure (= the pressure we
-- export, our receiver) with 40 psi (airbrakes.lua:165). Fed with a supply
-- right at 40 psi it limit-cycles: above it the spring chambers fill from the
-- dolly tank through the brake valve orifice (~0.9 kW pV flow, :187-190,
-- more than the compressor's ~0.35 kW), the PTO sync drags our receiver back
-- below 40 psi within a frame, the chambers exhaust to atmosphere (:191-198)
-- and so on - the receiver stayed pinned at 2.758 bar gauge for 150 s with
-- the compressor at 100 % (reproduced offline with the stock Lua:
-- tools/analyze_air_recovery_sim.py --case legacy). Like the trailer circuit
-- of a truck's multi-circuit protection valve, the trailer supply is now only
-- opened once the receiver has reached trailerValveOpenBar and is closed below
-- trailerValveCloseBar (gauge, receiver pressure). Closed = the exported
-- PTO pressure is 0 via the tank's consumerPressureCoef (pressureTank.lua:48;
-- the field is written directly - the stock setter :185-188 only adds a UI
-- message - every frame BEFORE the tank's own updateGFX, main.lua order
-- controller -> energyStorage; tank reset :227 sets it back to 1, the next
-- frame overwrites it again). With supply 0 the dolly does not sync at all
-- (:25 "supplyPressure > 1"): no flow either way, its tank keeps its air
-- (check-valve behaviour), its spring brakes apply (fail-safe) and its
-- service brakes still work from its own tank. So the dolly is only fed well
-- above the 40 psi band; the receiver recovers alone and the dolly then gets
-- a pressure step above its release point (6.0 bar receiver + ~3.6 bar kept in
-- a 50 l dolly -> ~4.6 bar, offline sim). ONE closing pressure, trailerValveCloseBar 3.4
-- bar, whether the compressor is available or not: 0.6 bar above the band,
-- and low enough that charging a low/empty trailer tank (8.5 bar into an
-- empty 50 l dolly -> ~3.8 bar) does not chatter. The receiver feeds nothing
-- but the trailer (our brakes are hydraulic), so a higher closing pressure
-- while the compressor is off (was 4.5 bar for switch OFF / KEY / SOC / NO
-- HV / FAULT) protected nothing: it only applied the dolly spring brakes
-- earlier - also while driving, 0.5 s after the driver switched the
-- compressor off with the receiver below 4.5 bar (review 2026-10). Offline
-- (--case pumps): 32 instead of 23 full trailer brake applications with the
-- compressor off before the spring brakes apply. The receiver has to stay
-- below the closing pressure for
-- trailerValveCloseDelayS: coupling an empty tank, the stock PTO sync (P
-- controller with one frame dead time) overshoots for a few frames (receiver
-- 8.5 -> 2.9 bar at 60 fps, the dolly tank above the receiver) before it
-- settles at the mixed pressure.
--
-- This controller is the compressor: electrically driven (HV battery via the
-- DC/DC / inverter), so it works in EV mode and at standstill with the ICE
-- off. Model, isothermal like the stock mechanical compressor
-- (lua/vehicle/powertrain/compressor.lua:21-43: tank storedEnergy = p * V =
-- n R T, air drawn in at ambient adds p_env * V_free; pressureTank.lua
-- :58-60, :101-114 keep p = E / V):
--   pr      = p_tank,abs / p_env
--   Q_pow   = P_rated * eta_iso / (p_env * ln pr)   free-air flow, power limit
--   Q       = min(Q_max, Q_pow) * spin               Q_max = displacement limit
--   P_el    = clamp(Q_max * p_env * ln pr / eta_iso, idle * P_rated, P_rated) * spin
--   dE_tank = p_env * Q * dt                         (compressor.lua:30-31, :43)
--   E_batt -= P_el / eta_conv * dt                   (same call as
--             etk854hybrid_cooling.lua drawBattery)
-- Defaults (1.5 kW, eta_iso 0.30, 270 l/min swept): ~120 l/min free air at
-- 8 bar, 7.2 -> 8.6 bar in the 40 l receiver in ~27 s, 0 -> 8.6 bar ~2 min.
-- Pressure switch: on at <= cutInBar, off at >= cutOutBar (gauge), then the
-- air dryer purges (stock purge event, compressor.lua:48-52). No restart for
-- restartDelayS after a stop. Relief valve at reliefBar. Not below minSoc
-- of the HV battery (VCU SOC_ICE_FORCE 0.08, etk854hybridControl.lua:251),
-- resumes at resumeSoc. Needs ignition (ignitionLevel >= 2).
--
-- Trailer telemetry: when a trailer couples at one of the tank's coupler
-- nodes we additionally request its stock air-brake channels
-- (trailerTankElectric / trailerParkingElectric / trailerServiceElectric,
-- defaults = stock names: tank "mainAirTank" -> "<name>_pressureRelative"
-- pressureTank.lua:377, controller "airbrakes" -> "<name>_pressure_parking" /
-- "_pressure_service" airbrakes.lua:408-409, bogie_suspension.jbeam
-- bogie_brakeValve) with the same beamstate.sendExportCouplerData call the
-- tank uses, and republish them under our prefix. Missing channels stay -1.
--
-- Published electrics (every updateGFX):
--   etk854hybrid_airState         OFF | FILL | READY | LOW | NO TANK
--   etk854hybrid_airPressureBar   receiver pressure, bar gauge (0 without tank)
--   etk854hybrid_airCompressor    0/1 motor running
--   etk854hybrid_airEnabled       0/1 driver switch
--   etk854hybrid_airInhibit       NONE | SWITCH | KEY | SOC | NO HV | FAULT | NO TANK
--   etk854hybrid_airPowerW        electrical power drawn from the HV battery [W]
--   etk854hybrid_airFlowLpm       free-air delivery [l/min]
--   etk854hybrid_airBattWh        battery energy drawn since spawn/reset [Wh]
--   etk854hybrid_airSupplyBar     pressure exported to the trailer, bar gauge
--                                 (0 while the trailer supply valve is closed)
--   etk854hybrid_airTrlValve      0/1 trailer supply protection valve open
--   etk854hybrid_airRelief        0/1 relief valve blowing off
--   etk854hybrid_airLeak          0/1 receiver leaking (damage / torn hose)
--   etk854hybrid_airTrailer       0/1 trailer coupled at an air coupling node
--   etk854hybrid_trailerAirBar    trailer tank, bar gauge (-1 = unknown)
--   etk854hybrid_trailerSpringBar trailer spring-brake actuator, bar (-1)
--   etk854hybrid_trailerServiceBar trailer service actuator, bar (-1)
--   etk854hybrid_trailerSpringCoef trailer spring-brake torque share 0..1 from
--                                 the stock bogie_brakeValve curve (0 Pa -> 1,
--                                 413685 Pa -> 0), -1 = unknown
--
-- API (Actions app / console): setEnabled(on), toggle(), getState(),
-- debugSetPressure(bar), debugVent(barPerSecond) (dev hooks, not bound).

local M = {}

M.type = "auxiliary"
M.relevantDevice = nil
M.defaultOrder = 95

local max = math.max
local min = math.min
local mlog = math.log

-- All tunables in ONE table (LuaJIT 60-upvalue limit); every key can be
-- overridden from the jbeam controller row (type-matched).
local CFG = {
  name = "etk854hybrid_airCompressor",
  tankName = "etk854hybrid_airTank",
  batteryName = "mainBattery",
  enabledOnSpawn = true,
  ratedPowerW = 1500, -- electrical input at full load
  isothermalEfficiency = 0.30, -- wire-to-air, isothermal reference
  maxFreeAirLpm = 270, -- swept volume x volumetric efficiency, low back pressure
  idlePowerFraction = 0.30, -- motor + friction floor while running
  converterEfficiency = 0.95, -- HV -> compressor motor supply
  cutInBar = 7.2,
  cutOutBar = 8.6,
  reliefBar = 8.9,
  lowWarnBar = 4.14, -- trailer spring-brake release threshold (60 psi)
  -- trailer supply protection valve, receiver bar GAUGE (header)
  trailerValveOpenBar = 6.0,
  trailerValveCloseBar = 3.4, -- with or without the compressor (header)
  trailerValveCloseDelayS = 0.5, -- below the closing pressure this long
  minSoc = 0.08,
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
  soundRpm = 1450, -- the stock event takes compressor rpm as pitch (compressor.lua:98-100)
  trailerTankElectric = "mainAirTank_pressureRelative",
  trailerParkingElectric = "airbrakes_pressure_parking",
  trailerServiceElectric = "airbrakes_pressure_service",
  springReleasePa = 413685,
  pollInterval = 0.5,
  messages = true,
  msgCategory = "vehicle.etk854hybrid.air"
}

-- Runtime state, one table.
local S = {
  objId = 0,
  tank = nil,
  battery = nil,
  couplers = {}, -- own coupler cid -> true (tank consumer nodes)
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
  trlValve = false, -- trailer supply protection valve open
  trlLowT = 0, -- time below the closing pressure while open
  trlMsg = false, -- "supply closed" hint pending until the trailer's tank channel arrives
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

-- "6,0" (German decimal comma like the other texts of the mod)
local function barText(bar)
  local s = string.format("%.1f", bar or 0)
  return s
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
  -- same request the tank makes for its flow channel (pressureTank.lua:129/:138)
  if beamstate and beamstate.sendExportCouplerData then
    beamstate.sendExportCouplerData(obj2id, obj2nodeId, {electrics = {CFG.trailerTankElectric, CFG.trailerParkingElectric, CFG.trailerServiceElectric}})
  end
  -- the hint waits for the trailer's tank channel (publishTrailer): a trailer
  -- without air brakes on the ball gets no message
  S.trlMsg = S.tank ~= nil
end

local function unlinkTrailer()
  S.trlMsg = false
  if S.trl then
    S.trl = nil
    clearTrailerChannels()
  end
end

-- coupled trailer that reports an air tank (stock channel, -1/nil otherwise)
local function trailerHasAir()
  return S.trl ~= nil and type(electrics.values[CFG.trailerTankElectric]) == "number"
end

-- Reconcile with beamstate.attachedCouplers (beamstate.lua:409, exposed :1591;
-- cleared without a detach event on reset, :1062).
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
  -- same call and argument order as compressor.lua updateSounds (:94-101):
  -- volume, pitch = compressor rpm, colour = 1 while pumping
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
  -- broken receiver / torn hose (pressureTank.lua onBreak :194-197 sets the
  -- leak rate to maxLeakRate, the capacity / 3 per second)
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
-- (header): open at >= trailerValveOpenBar, close below trailerValveCloseBar
-- for trailerValveCloseDelayS; closed = export 0 through the tank's
-- consumerPressureCoef.
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
  -- storedEnergy is the live value (tank.currentPressure lags one frame,
  -- pressureTank.lua:60 runs after the controllers, main.lua:88/:93)
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
      -- dryer purge when the unit stops after a real run (compressor.lua:48-52)
      if S.runTime >= CFG.purgeMinRunS then
        purge()
      end
      S.runTime = 0
    end
    S.offTimer = S.offTimer + dt
  end

  -- motor speed: soft start / run-down
  if run then
    S.spin = min(1, S.spin + dt / max(CFG.spinUpS, 0.01))
  else
    S.spin = max(0, S.spin - dt / max(CFG.spinDownS, 0.01))
  end

  local powerW = 0
  local q = 0
  if run and S.spin > 0 then
    local pr = max(pAbs / env, 1.0001)
    local lnPr = mlog(pr)
    local eta = max(CFG.isothermalEfficiency, 0.01)
    local qMax = CFG.maxFreeAirLpm / 60000 -- m^3/s free air
    local pRated = CFG.ratedPowerW
    local qPow = pRated * eta / (env * lnPr)
    q = min(qMax, qPow) * S.spin
    local pNeed = qMax * env * lnPr / eta
    powerW = min(pRated, max(CFG.idlePowerFraction * pRated, pNeed)) * S.spin
    local dE = env * q * dt
    tank.storedEnergy = tank.storedEnergy + dE
    S.airJ = S.airJ + dE
    local battW = powerW / max(CFG.converterEfficiency, 0.01)
    local bat = S.battery
    bat:setStoredEnergy(max(bat.storedEnergy - battW * dt, 0))
    S.battWh = S.battWh + battW * dt / 3600
    powerW = battW
  end
  S.powerW = powerW
  S.flowLpm = q * 60000

  -- relief valve
  pAbs = tank.storedEnergy * tank.invCapacity
  local reliefAbs = CFG.reliefBar * 1e5 + env
  S.relief = pAbs > reliefAbs
  if S.relief then
    tank.storedEnergy = reliefAbs * V
    pAbs = reliefAbs
  end
  pBar = (pAbs - env) * 1e-5
  S.pBar = pBar

  -- runs before the tank's own updateGFX of this frame (main.lua order), so
  -- the coef applies to this frame's export
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

-- beamstate runs first (main.lua:394-395), so the entry is already gone:
-- re-scan instead of forcing an unlink (the HD combo mount has two couplers)
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

-- Dev hooks (not bound): set the receiver to a gauge pressure / vent it at a
-- constant rate (bar/s of the receiver alone; 0 stops). The coupled trailer
-- tank follows through the stock PTO sync.
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
  -- same source setup as compressor.lua initSounds (:183-189)
  S.sfx = obj:createSFXSource2(CFG.compressorLoopEvent, "AudioDefaultLoop3D", "etk854hybridAirCompressor", S.soundNodeId, 1)
  if S.sfx then
    obj:playSFX(S.sfx)
    obj:setVolumePitchCT(S.sfx, 0, 0, 0, 0)
  end
end

-- the driver switch survives R like a real switch; everything else restarts
-- (energyStorage.reset puts the receiver back to its starting pressure)
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
