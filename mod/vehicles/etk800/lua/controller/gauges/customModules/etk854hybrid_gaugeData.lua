-- ETK 854 ActiveHybrid - instrument cluster data module
-- Custom module of the stock gauges/genericGauges controller. The cluster parts
-- add it with fileName "gauges/customModules/etk854hybrid_gaugeData" and no
-- custom name (genericGauges resolves custom modules by that path).
-- Contract as the stock modules: init / reset / updateGFX (every frame) /
-- setupGaugeData(props, once) / updateGaugeData(m, per UI push while seated).
-- Also getEnergyInfo() (screenData), resetTripComputer(), setOption() (tests).
--
-- Electrics, every frame: etk854hybrid_tachoRpm (rpm / READY 0 / OFF -500),
--   _dscLamp (qualified DSC lamp), _dse* diagnostics, _gearActual, _gearHint,
--   _fuel*, _tripKm, _evAvgKwh, _tripId; stock esc / tcs / dseWarningPulse are
--   rewritten to follow the qualified lamp.
-- customModules per UI push (requested properties only):
--   range: rangeKm (-1 = unknown)
--   scale: iceKw, evKw, regenKw [kW]      gear: gearStyle "number" | "plain"
--   fuel:  fuelL fuelRangeKm instL100 instLh instMode instBar avgL100 tripKm
--          avgKwh [kWh/100 km]
-- Side effect: ICE K0 gate on the traction control group (installIceGate).
--
-- Order: no defaultOrder -> 100000, after CMU / yawControl / tractionControl and
-- before material.updateGFX reads the glowMaps (the telltale rewrite needs both).

local M = {}
M.type = "auxiliary"

local max, min, abs = math.max, math.min, math.abs

-- e-machine limits per boost stage; keep in sync with BOOST_STAGES in etk854hybridControl.lua
local STAGE_EV_KW = {[0] = 83, [1] = 110, [2] = 150, [3] = 200}
local STAGE_REGEN_KW = {[0] = 55, [1] = 65, [2] = 80, [3] = 95}

local HP_TO_KW = 0.7457 -- combustionEngine maxPower is in hp

local J_PER_KWH = 3600000

-- gearActual without a gearbox device; a missing key would arrive as 0 (neutral),
-- this marker lets the page fall back to electrics.gearIndex
local GEAR_UNKNOWN = -99

local cfg = {
  engineName = "mainEngine",
  batteryName = "mainBattery",
  socFloor = 0.10,
  defaultConsumption = 21.0, -- kWh/100 km, start value
  consumptionMin = 12.0,
  consumptionMax = 40.0,
  consumptionDistance = 3000, -- m, averaging distance
  sampleInterval = 1.0, -- s
  tachoReadyRpm = 0,
  tachoOffRpm = -500,
  tachoSweepRate = 1800, -- rpm/s, READY <-> OFF
  iceRpmMin = 150,
  -- DSE / DSC lamp / gear
  motorName = "integratedMotor",
  gearboxName = "gearbox",
  lampTfMax = 0.90, -- throttle factor below = real trim
  lampOnS = 0.25, -- s of trim before the lamp lights
  lampHoldS = 1.0, -- s, after a stock trim / brake stage
  lampHoldTsaS = 0.3, -- s, after a trailer stability braking stage (its own 0.15 s tail comes first)
  lampK0Min = 0.9, -- K0 ratio from which the ICE factor counts
  dseIceK0Gate = true, -- see installIceGate
  dseIceK0GateMin = 0.1, -- K0 ratio below which the ICE TC group sees throttle 0
  dseStockLampQualified = true, -- stock ESC / TCS decals follow the qualified lamp
  gearStyle = "number", -- "number" (D1..D8) | "plain" (D, S<n>, M<n>)
  -- shift recommendation debounce [s]
  gearHintOnS = 0.25, -- show delay D / S (a shift takes ~0.3 s)
  gearHintOnManualS = 0.10, -- show delay M<n>
  gearHintOffS = 0.20, -- clear delay
  -- on-board computer
  fuelTankName = "mainTank",
  defaultFuelConsumption = 8.0, -- l/100 km, start value
  fuelConsumptionMin = 3.0,
  fuelConsumptionMax = 30.0,
  fuelInstTau = 0.6, -- s
  fuelInstLhSpeed = 1.4, -- m/s, below: l/h instead of l/100 km
  fuelBarMaxL100 = 30.0, -- full bar
  fuelBarMaxLh = 10.0,
  tripMinKm = 0.2 -- trip average shown from here
}

-- on-board computer state
local obc = {
  tank = nil,
  retryT = 0,
  lastVol = nil,
  flow = 0, -- l/s filtered
  instL100 = -1,
  instLh = 0,
  instMode = "ev",
  instBar = 0,
  tripFuelL = 0,
  tripDistM = 0,
  tripId = 0,
  fuelL = -1,
  fuelCapL = 0,
  consFuel = 8.0, -- learned l/100 km, ICE running
  sampleT = 0,
  sampleDist = 0,
  sampleFuel = 0
}

local hint = {actual = nil, shown = 0, t = 0}

-- mutable state in one table (upvalue limit)
local st = {
  needle = -500,
  -- estimator
  sampleTimer = 0,
  sampleDist = 0,
  lastEnergy = nil,
  consKwh = 21.0,
  rangeKm = -1,
  usableKwh = 0,
  capKwh = 0,
  hasBattery = false,
  iceKw = 0,
  -- requested properties
  wantRange = false,
  wantScale = false,
  wantGear = false,
  wantFuel = false
}

-- DSE handles (retried at 1 Hz while missing) and DSC lamp timers
local dse = {
  resolved = false,
  retryT = 0,
  mtc = nil, -- motorTorqueControl
  bc = nil, -- brakeControl
  cmu = nil,
  tc = nil, -- tractionControl
  ice = nil,
  em = nil,
  gbx = nil,
  grpIce = nil, -- tractionControl wheelGroups
  grpEm = nil,
  gateDone = false,
  lowT = 0,
  holdT = 0, -- s left of the stock hold
  tsaT = 0, -- s left of the trailer stability hold
  escT = 0, -- s left of the raw escActive hold
  tcsT = 0 -- s left of the raw tcsActive hold
}

local function num(v, d)
  v = tonumber(v)
  if v == nil or v ~= v then
    return d
  end
  return v
end

local function readConfig(jbeamData)
  jbeamData = jbeamData or {}
  cfg.engineName = jbeamData.engineName or cfg.engineName
  cfg.batteryName = jbeamData.batteryName or cfg.batteryName
  cfg.socFloor = num(jbeamData.socFloor, cfg.socFloor)
  cfg.defaultConsumption = num(jbeamData.defaultConsumption, cfg.defaultConsumption)
  cfg.consumptionDistance = max(100, num(jbeamData.consumptionDistance, cfg.consumptionDistance))
  cfg.tachoReadyRpm = num(jbeamData.tachoReadyRpm, cfg.tachoReadyRpm)
  cfg.tachoOffRpm = num(jbeamData.tachoOffRpm, cfg.tachoOffRpm)
  cfg.tachoSweepRate = max(100, num(jbeamData.tachoSweepRate, cfg.tachoSweepRate))
  cfg.iceRpmMin = num(jbeamData.iceRpmMin, cfg.iceRpmMin)
  cfg.motorName = jbeamData.motorName or cfg.motorName
  cfg.gearboxName = jbeamData.gearboxName or cfg.gearboxName
  cfg.lampTfMax = num(jbeamData.lampTfMax, cfg.lampTfMax)
  cfg.lampOnS = max(0, num(jbeamData.lampOnS, cfg.lampOnS))
  cfg.lampHoldS = max(0, num(jbeamData.lampHoldS, cfg.lampHoldS))
  cfg.lampHoldTsaS = max(0, num(jbeamData.lampHoldTsaS, cfg.lampHoldTsaS))
  cfg.lampK0Min = num(jbeamData.lampK0Min, cfg.lampK0Min)
  if jbeamData.dseIceK0Gate ~= nil then
    cfg.dseIceK0Gate = jbeamData.dseIceK0Gate == true
  end
  cfg.dseIceK0GateMin = num(jbeamData.dseIceK0GateMin, cfg.dseIceK0GateMin)
  if jbeamData.dseStockLampQualified ~= nil then
    cfg.dseStockLampQualified = jbeamData.dseStockLampQualified == true
  end
  if jbeamData.gearStyle == "plain" or jbeamData.gearStyle == "number" then
    cfg.gearStyle = jbeamData.gearStyle
  end
  cfg.gearHintOnS = max(0, num(jbeamData.gearHintOnS, cfg.gearHintOnS))
  cfg.gearHintOnManualS = max(0, num(jbeamData.gearHintOnManualS, cfg.gearHintOnManualS))
  cfg.gearHintOffS = max(0, num(jbeamData.gearHintOffS, cfg.gearHintOffS))
  cfg.fuelTankName = jbeamData.fuelTankName or cfg.fuelTankName
  cfg.defaultFuelConsumption = num(jbeamData.defaultFuelConsumption, cfg.defaultFuelConsumption)
  cfg.fuelInstTau = max(0.05, num(jbeamData.fuelInstTau, cfg.fuelInstTau))
  cfg.fuelInstLhSpeed = max(0.1, num(jbeamData.fuelInstLhSpeed, cfg.fuelInstLhSpeed))
  cfg.fuelBarMaxL100 = max(1, num(jbeamData.fuelBarMaxL100, cfg.fuelBarMaxL100))
  cfg.fuelBarMaxLh = max(0.5, num(jbeamData.fuelBarMaxLh, cfg.fuelBarMaxLh))
  cfg.tripMinKm = max(0, num(jbeamData.tripMinKm, cfg.tripMinKm))
end

local function getBattery()
  if not energyStorage or not energyStorage.getStorage then
    return nil
  end
  local b = energyStorage.getStorage(cfg.batteryName)
  if b and (b.energyCapacity or 0) > 0 then
    return b
  end
  return nil
end

local function refreshIcePeak()
  local eng = powertrain and powertrain.getDevice and powertrain.getDevice(cfg.engineName)
  st.iceKw = (eng and num(eng.maxPower, 0) or 0) * HP_TO_KW
end

-- -------------------------------------------------------------- needle
-- Engine rpm while the ICE turns, else READY (ignition on) or OFF. Both stock
-- dial faces print READY at ~0 and OFF at ~-500 rpm; moves sweep at tachoSweepRate.
local function updateNeedle(dt)
  local ev = electrics.values
  local rpm = num(ev.rpm, 0)
  local ign = num(ev.ignitionLevel, 0)
  local target
  if rpm > cfg.iceRpmMin then
    target = rpm
  elseif ign >= 2 then
    target = cfg.tachoReadyRpm
  else
    target = cfg.tachoOffRpm
  end
  local cur = st.needle
  if target > cfg.iceRpmMin then
    -- follow the rpm, but leave the READY/OFF zone at the sweep rate
    if cur < cfg.tachoReadyRpm then
      cur = min(target, cur + cfg.tachoSweepRate * dt)
    else
      cur = target
    end
  else
    local step = cfg.tachoSweepRate * dt
    if cur > cfg.tachoReadyRpm + 1 and target >= cfg.tachoOffRpm then
      -- engine spinning down: follow the falling rpm, never jump
      cur = max(target, min(cur, rpm > 0 and rpm or cur - step * 4))
    else
      cur = cur + max(-step, min(step, target - cur))
    end
  end
  st.needle = cur
  ev.etk854hybrid_tachoRpm = cur
end

-- ----------------------------------------------------------- estimator
-- EV range = (SOC - socFloor) * capacity / consumption * 100; consumption is a
-- distance-weighted average sampled while the ICE is off, kept over R-resets.
-- socFloor = batteryMonitor SOC_LOW1 (end of the EV budget).
local function updateEstimator(dt)
  st.sampleDist = st.sampleDist + abs(num(electrics.values.wheelspeed, 0)) * dt
  st.sampleTimer = st.sampleTimer + dt
  if st.sampleTimer < cfg.sampleInterval then
    return
  end
  st.sampleTimer = 0

  local b = getBattery()
  st.hasBattery = b ~= nil
  if not b then
    st.rangeKm, st.usableKwh, st.capKwh = -1, 0, 0
    st.lastEnergy, st.sampleDist = nil, 0
    return
  end

  local energy = num(b.storedEnergy, 0)
  st.capKwh = b.energyCapacity / J_PER_KWH
  local soc = energy / b.energyCapacity
  st.usableKwh = max(0, soc - cfg.socFloor) * st.capKwh

  local ev = electrics.values
  -- iceState is 1 from crank to release; nil on a non-hybrid vehicle
  local iceOff = num(ev.etk854hybrid_iceState, 1) < 0.5
  local dist = st.sampleDist
  if st.lastEnergy and iceOff and dist > 2 then
    local usedKwh = (st.lastEnergy - energy) / J_PER_KWH
    local sample = usedKwh / (dist / 100000) -- kWh/100 km
    sample = max(-cfg.consumptionMax, min(cfg.consumptionMax * 2, sample))
    local w = min(1, dist / cfg.consumptionDistance)
    st.consKwh = st.consKwh + (sample - st.consKwh) * w
    st.consKwh = max(cfg.consumptionMin, min(cfg.consumptionMax, st.consKwh))
  end
  st.lastEnergy = energy
  st.sampleDist = 0
  st.rangeKm = st.usableKwh / st.consKwh * 100
end

-- ---------------------------------------------------- on-board computer
-- Tank litres come live from storedEnergy (fuelTank updates remainingVolume only
-- after the controllers). Instantaneous = filtered volume drop: l/100 km, l/h
-- below fuelInstLhSpeed, "ev" with the ICE off. Fuel range uses the consumption
-- learned with the ICE running. The trip survives R-resets. -1 = not available.
local MAX_FLOW_LPS = 0.05 -- 180 l/h, above = not consumption

local function tankVolume(t)
  local se, d, e = t.storedEnergy, t.fuelLiquidDensity, t.energyDensity
  if type(se) == "number" and type(d) == "number" and type(e) == "number" and d * e > 0 then
    return se / (d * e)
  end
  return tonumber(t.remainingVolume)
end

local function publishFuel(ev)
  local tripKm = obc.tripDistM * 0.001
  ev.etk854hybrid_fuelL = obc.fuelL
  ev.etk854hybrid_fuelInstL100 = obc.instL100
  ev.etk854hybrid_fuelInstLh = obc.instLh
  ev.etk854hybrid_fuelInstMode = obc.instMode
  ev.etk854hybrid_fuelAvgL100 = (tripKm >= cfg.tripMinKm and tripKm > 0) and (obc.tripFuelL / (tripKm * 0.01)) or -1
  ev.etk854hybrid_tripKm = tripKm
  ev.etk854hybrid_fuelRangeKm = obc.fuelL >= 0 and (obc.fuelL / obc.consFuel * 100) or -1
  ev.etk854hybrid_evAvgKwh = st.consKwh
  ev.etk854hybrid_tripId = obc.tripId
end

local function updateFuel(dt)
  local ev = electrics.values
  local t = obc.tank
  if not t then
    obc.retryT = obc.retryT - dt
    if obc.retryT <= 0 then
      obc.retryT = 1.0
      local s = energyStorage and energyStorage.getStorage and energyStorage.getStorage(cfg.fuelTankName)
      if s and s.type == "fuelTank" then
        obc.tank, t = s, s
      end
    end
  end
  local vol = t and tankVolume(t) or nil
  if not vol then
    obc.fuelL, obc.fuelCapL, obc.lastVol, obc.flow = -1, 0, nil, 0
    obc.instL100, obc.instLh, obc.instMode, obc.instBar = -1, 0, "ev", 0
    publishFuel(ev)
    return
  end
  obc.fuelL = max(0, vol)
  obc.fuelCapL = num(t.capacity, 0)
  local spd = abs(num(ev.wheelspeed, 0))
  local dist = spd * dt
  local used = 0
  if obc.lastVol then
    used = obc.lastVol - vol
    -- a rise (refuel, R refill) or an implausible drop re-baselines
    if used < 0 or used > MAX_FLOW_LPS * dt then
      used = 0
    end
  end
  obc.lastVol = vol
  obc.tripFuelL = obc.tripFuelL + used
  obc.tripDistM = obc.tripDistM + dist

  obc.flow = obc.flow + (used / dt - obc.flow) * min(1, dt / cfg.fuelInstTau)
  local iceOn = num(ev.etk854hybrid_iceState, 1) >= 0.5
  if not iceOn then
    obc.flow = 0
    obc.instMode, obc.instL100, obc.instLh, obc.instBar = "ev", 0, 0, 0
  elseif spd < cfg.fuelInstLhSpeed then
    obc.instMode = "lh"
    obc.instLh = obc.flow * 3600
    obc.instL100 = -1
    obc.instBar = min(1, obc.instLh / cfg.fuelBarMaxLh)
  else
    obc.instMode = "l100"
    obc.instLh = obc.flow * 3600
    obc.instL100 = min(99.9, obc.flow / spd * 100000)
    obc.instBar = min(1, obc.instL100 / cfg.fuelBarMaxL100)
  end

  -- learned consumption with the ICE running (fuel range)
  if iceOn and spd >= cfg.fuelInstLhSpeed then
    obc.sampleDist = obc.sampleDist + dist
    obc.sampleFuel = obc.sampleFuel + used
  end
  obc.sampleT = obc.sampleT + dt
  if obc.sampleT >= cfg.sampleInterval then
    obc.sampleT = 0
    if obc.sampleDist > 2 then
      local sample = obc.sampleFuel / (obc.sampleDist / 100000)
      sample = max(0, min(cfg.fuelConsumptionMax * 2, sample))
      local w = min(1, obc.sampleDist / cfg.consumptionDistance)
      obc.consFuel = obc.consFuel + (sample - obc.consFuel) * w
      obc.consFuel = max(cfg.fuelConsumptionMin, min(cfg.fuelConsumptionMax, obc.consFuel))
    end
    obc.sampleDist, obc.sampleFuel = 0, 0
  end
  publishFuel(ev)
end

-- New trip (fuel average + distance); the learned EV consumption is kept.
local function resetTripComputer()
  obc.tripFuelL, obc.tripDistM = 0, 0
  obc.tripId = obc.tripId + 1
  publishFuel(electrics.values)
  if guihooks and guihooks.message then
    guihooks.message("On-board computer: average consumption reset", 3, "vehicle.etk854hybrid.obc")
  end
  return obc.tripId
end

-- ------------------------------------------------------ DSE diagnostics
-- electrics flags arrive as boolean or number
local function on(v)
  return v == true or (tonumber(v) or 0) > 0.5
end

local function devTf(d)
  local v = d and d.throttleFactor
  if type(v) ~= "number" or v ~= v then
    return 1
  end
  return v
end

local function resolveDse()
  local getC = controller and controller.getController
  if getC then
    dse.mtc = getC("motorTorqueControl")
    dse.bc = getC("brakeControl")
    dse.cmu = getC("CMU")
    dse.tc = getC("tractionControl")
  end
  local getD = powertrain and powertrain.getDevice
  if getD then
    dse.ice = getD(cfg.engineName)
    dse.em = getD(cfg.motorName)
    dse.gbx = getD(cfg.gearboxName)
  end
  dse.grpIce, dse.grpEm = nil, nil
  local tc = dse.tc
  local list = tc and tc.tractionControlledWheels
  if type(list) == "table" then
    for _, e in ipairs(list) do
      local g = type(e) == "table" and e.wheelGroup or nil
      if type(g) == "table" then
        if g.name == cfg.engineName then
          dse.grpIce = g
        elseif g.name == cfg.motorName then
          dse.grpEm = g
        end
      end
    end
  end
  dse.resolved = dse.gbx ~= nil and (dse.tc == nil or dse.grpIce ~= nil or dse.grpEm ~= nil)
end

-- ICE K0 gate: virtualSpeedSlip measures slip while motor.throttle > 0.01, and
-- the engine throttle includes idle, so the ICE TC group would trim and flag
-- tcsActive while the ICE idles behind an open K0. The group's motor reference
-- becomes a read-through proxy whose throttle reads 0 below dseIceK0GateMin
-- (throttleFactor is still written on the real device). Returns false while the
-- TC group does not exist yet.
local function installIceGate()
  if not cfg.dseIceK0Gate then
    return true
  end
  local g = dse.grpIce
  if not g then
    -- no ICE TC group: nothing to gate
    return dse.resolved
  end
  local dev = g.motor
  if type(dev) ~= "table" then
    return false
  end
  if rawget(dev, "etk854hybridK0Gate") then
    return true
  end
  local gateMin = cfg.dseIceK0GateMin
  local proxy = {name = dev.name, etk854hybridK0Gate = true, etk854hybridDevice = dev}
  setmetatable(proxy, {
    __index = function(_, k)
      if k == "throttle" then
        -- nil without the VCU: no gating
        local r = tonumber(electrics.values.etk854hybrid_k0ClutchRatio)
        if r and r < gateMin then
          return 0
        end
      end
      return dev[k]
    end
  })
  g.motor = proxy
  return true
end

local function removeIceGate()
  local g = dse.grpIce
  local p = g and g.motor
  if type(p) == "table" and rawget(p, "etk854hybridK0Gate") then
    g.motor = rawget(p, "etk854hybridDevice") or p
  end
end

-- Stock telltales: yawControl / tractionControl write esc / tcs from the raw
-- didAct every frame. Running after them, rewrite them to CMU.warningLightPulse
-- only while the qualified lamp is on and that supervisor's raw flag is (or was
-- within lampHoldS) on; dseWarningPulse likewise. escActive / tcsActive (read
-- by the VCU and the 8HP), a disabled supervisor's steady 1 and DSC OFF stay
-- untouched. No feedback: the supervisors rewrite esc / tcs every frame.
local function qualifyStockLamps(ev, dt, lampOn)
  if ev.etk854hybrid_dscMode == "OFF" then
    -- keep the stock steady-on telltale
    dse.escT, dse.tcsT = 0, 0
    return
  end
  local cmu = dse.cmu
  local pulse = cmu and num(cmu.warningLightPulse, 0) or 0
  local escAct, tcsAct = on(ev.escActive), on(ev.tcsActive)
  dse.escT = escAct and cfg.lampHoldS or max(0, dse.escT - dt)
  dse.tcsT = tcsAct and cfg.lampHoldS or max(0, dse.tcsT - dt)
  local escRaw, tcsRaw = ev.esc, ev.tcs
  -- nil = not fitted; 1 with the raw flag off = disabled
  if escRaw ~= nil and not (escRaw == 1 and not escAct) then
    ev.esc = (lampOn and (escAct or dse.escT > 0)) and pulse or 0
  end
  if tcsRaw ~= nil and not (tcsRaw == 1 and not tcsAct) then
    ev.tcs = (lampOn and (tcsAct or dse.tcsT > 0)) and pulse or 0
  end
  -- keep the CMU's 1 in a degraded / offline state
  local states = cmu and cmu.systemStates
  if ev.dseWarningPulse ~= nil and states and cmu.systemState == states.normal then
    ev.dseWarningPulse = lampOn and pulse or 0
  end
end

-- DSE diagnostics, qualified DSC lamp, engaged gear. Sampled per frame while the
-- flags change per physics step (duty estimate). etk854hybrid_dseSrc bits:
--   1/2 motorTorqueControl actingAsTC/YC (last wheel group stepped)
--   4/8 TC/YC brake stage   16/32 ICE/e-machine throttleFactor < 1
--   64/128 brakeControl actingAsTC/YC   256 trailer stability brake stage
-- Lamp: min(motorTf, iceTf) < lampTfMax for lampOnS or a DSE brake stage, held
-- lampHoldS; a TSA brake stage, held lampHoldTsaS (each source keeps its own
-- hold, so a short TSA corner jab stays a short flash). iceTf counts only with
-- K0 closed (trimming an ICE that idles behind an open K0 changes nothing at
-- the wheels). The raw escActive / tcsActive react to any trim (even 0.999)
-- and would blink.
local function updateDse(dt)
  local ev = electrics.values
  local motorTf, iceTf = devTf(dse.em), devTf(dse.ice)
  ev.etk854hybrid_dseMotorTf = motorTf
  ev.etk854hybrid_dseIceTf = iceTf

  local tcBrake = on(ev.isTCBrakeActive)
  local ycBrake = on(ev.isYCBrakeActive)
  local src = 0
  local mtc, bc = dse.mtc, dse.bc
  if mtc then
    if mtc.isActingAsTC == true then src = src + 1 end
    if mtc.isActingAsYC == true then src = src + 2 end
  end
  if tcBrake then src = src + 4 end
  if ycBrake then src = src + 8 end
  if iceTf < 1 then src = src + 16 end
  if motorTf < 1 then src = src + 32 end
  if bc then
    if bc.isActingAsTC == true then src = src + 64 end
    if bc.isActingAsYC == true then src = src + 128 end
  end
  local tsaLamp = on(ev.etk854hybrid_tsaLamp)
  if tsaLamp then src = src + 256 end
  ev.etk854hybrid_dseSrc = src

  -- CMU virtual sensors: speed [km/h], body slip angle [rad]
  local vs = dse.cmu and dse.cmu.virtualSensors
  local virt = type(vs) == "table" and vs.virtual or nil
  if type(virt) == "table" then
    ev.etk854hybrid_dseVirtKmh = num(virt.speed, 0) * 3.6
    ev.etk854hybrid_dseBsa = num(virt.bodySlipAngle, 0)
  end
  if dse.grpIce then ev.etk854hybrid_dseSlipIce = num(dse.grpIce.maxSlip, 0) end
  if dse.grpEm then ev.etk854hybrid_dseSlipEm = num(dse.grpEm.maxSlip, 0) end

  local tf = motorTf
  if num(ev.etk854hybrid_k0ClutchRatio, 1) >= cfg.lampK0Min then
    tf = min(tf, iceTf)
  end
  if tf < cfg.lampTfMax then
    dse.lowT = dse.lowT + dt
  else
    dse.lowT = 0
  end
  local stock = dse.lowT >= cfg.lampOnS or tcBrake or ycBrake
  if stock then
    dse.holdT = cfg.lampHoldS
  elseif dse.holdT > 0 then
    dse.holdT = max(0, dse.holdT - dt)
  end
  if tsaLamp then
    dse.tsaT = cfg.lampHoldTsaS
  elseif dse.tsaT > 0 then
    dse.tsaT = max(0, dse.tsaT - dt)
  end
  -- the last clauses keep the lamp correct with zero holds
  local lampOn = dse.holdT > 0 or dse.tsaT > 0 or stock or tsaLamp
  ev.etk854hybrid_dscLamp = lampOn and 1 or 0
  -- raw telltales before the rewrite (telemetry)
  ev.etk854hybrid_dseEscRaw = num(ev.esc, 0)
  ev.etk854hybrid_dseTcsRaw = num(ev.tcs, 0)
  if cfg.dseStockLampQualified then
    qualifyStockLamps(ev, dt, lampOn)
  end

  -- engaged gear (electrics.gearIndex is the 8HP target and runs ahead)
  local gbx = dse.gbx
  if gbx then
    ev.etk854hybrid_gearActual = num(gbx.gearIndex, 0)
  else
    ev.etk854hybrid_gearActual = GEAR_UNKNOWN
  end
end

-- Debounced shift recommendation from etk854hybrid_gearPlanned (8HP target in
-- D / S, shadow D plan in M<n>; 0 = none). Cleared at once on a gear change.
local function updateGearHint(dt)
  local ev = electrics.values
  local actual = num(ev.etk854hybrid_gearActual, 0)
  local planned = num(ev.etk854hybrid_gearPlanned, 0)
  local gs = ev.gear
  local c = type(gs) == "string" and gs:sub(1, 1) or ""
  local manual = c == "M"
  local raw = 0
  if actual >= 1 and planned >= 1 and planned ~= actual and (manual or c == "D" or c == "S") then
    raw = planned
  end
  if actual ~= hint.actual then
    -- a shift spends the recommendation
    hint.actual, hint.shown, hint.t = actual, 0, 0
  end
  if raw == hint.shown then
    hint.t = 0
  else
    hint.t = hint.t + dt
    local need = cfg.gearHintOffS
    if raw ~= 0 then
      need = manual and cfg.gearHintOnManualS or cfg.gearHintOnS
    end
    if hint.t >= need then
      hint.shown, hint.t = raw, 0
    end
  end
  ev.etk854hybrid_gearHint = hint.shown
end

-- ------------------------------------------------------------ contract
-- Transient display state, restarted on spawn and on R (= reset). The learned
-- consumption, the DSE wheel groups / gate proxy (tractionControl.reset only
-- clears slip state) and the trip survive R; re-baselining the tank volume keeps
-- the R refill out of the trip.
local function resetTransient()
  st.sampleTimer, st.sampleDist, st.lastEnergy = 0, 0, nil
  st.needle = cfg.tachoOffRpm
  electrics.values.etk854hybrid_tachoRpm = st.needle
  dse.lowT, dse.holdT, dse.tsaT, dse.escT, dse.tcsT = 0, 0, 0, 0, 0
  electrics.values.etk854hybrid_dscLamp = 0
  hint.actual, hint.shown, hint.t = nil, 0, 0
  electrics.values.etk854hybrid_gearHint = 0
  obc.lastVol, obc.flow = nil, 0
  obc.sampleT, obc.sampleDist, obc.sampleFuel = 0, 0, 0
end

local function init(jbeamData)
  readConfig(jbeamData)
  st.consKwh = cfg.defaultConsumption
  st.rangeKm = -1
  resetTransient()
  dse.resolved, dse.gateDone, dse.retryT = false, false, 0
  electrics.values.etk854hybrid_gearActual = GEAR_UNKNOWN
  -- on-board computer: a spawn starts a new trip
  obc.tank, obc.retryT = nil, 0
  obc.tripFuelL, obc.tripDistM, obc.tripId = 0, 0, 0
  obc.consFuel = cfg.defaultFuelConsumption
  obc.fuelL, obc.fuelCapL = -1, 0
  obc.instL100, obc.instLh, obc.instMode, obc.instBar = -1, 0, "ev", 0
  publishFuel(electrics.values)
end

local function updateGFX(dt)
  if not (dt and dt > 0) then
    return
  end
  updateNeedle(dt)
  updateEstimator(dt)
  updateFuel(dt)
  if not (dse.resolved and dse.gateDone) then
    dse.retryT = dse.retryT - dt
    if dse.retryT <= 0 then
      dse.retryT = 1.0
      resolveDse()
      dse.gateDone = installIceGate()
    end
  end
  updateDse(dt)
  updateGearHint(dt)
end

local function setupGaugeData(props)
  props = props or {}
  st.wantRange = props.range or false
  st.wantScale = props.scale or false
  st.wantGear = props.gear or false
  st.wantFuel = props.fuel or false
  refreshIcePeak()
end

local function updateGaugeData(m)
  if st.wantGear then
    m.gearStyle = cfg.gearStyle
  end
  if st.wantFuel then
    local ev = electrics.values
    m.fuelL = obc.fuelL
    m.fuelRangeKm = ev.etk854hybrid_fuelRangeKm
    m.instL100 = obc.instL100
    m.instLh = obc.instLh
    m.instMode = obc.instMode
    m.instBar = obc.instBar
    m.avgL100 = ev.etk854hybrid_fuelAvgL100
    m.tripKm = obc.tripDistM * 0.001
    m.avgKwh = st.consKwh
  end
  if st.wantRange then
    m.rangeKm = st.hasBattery and st.rangeKm or -1
  end
  if st.wantScale then
    if st.iceKw <= 0 then
      refreshIcePeak()
    end
    local stage = max(0, min(3, math.floor(num(electrics.values.etk854hybrid_boostStage, 0) + 0.5)))
    m.iceKw = st.iceKw
    m.evKw = STAGE_EV_KW[stage]
    m.regenKw = STAGE_REGEN_KW[stage]
  end
end

-- read-only snapshot for other controllers (shared table)
local info = {rangeKm = -1, consKwh = 0, usableKwh = 0, capKwh = 0, iceKw = 0, hasBattery = false,
  fuelL = -1, fuelCapL = 0, fuelRangeKm = -1, instL100 = -1, instLh = 0, instMode = "ev", avgL100 = -1, tripKm = 0}
local function getEnergyInfo()
  info.rangeKm = st.hasBattery and st.rangeKm or -1
  info.consKwh = st.consKwh
  info.usableKwh = st.usableKwh
  info.capKwh = st.capKwh
  info.hasBattery = st.hasBattery
  local ev = electrics.values
  info.fuelL = obc.fuelL
  info.fuelCapL = obc.fuelCapL
  info.fuelRangeKm = ev.etk854hybrid_fuelRangeKm or -1
  info.instL100 = obc.instL100
  info.instLh = obc.instLh
  info.instMode = obc.instMode
  info.avgL100 = ev.etk854hybrid_fuelAvgL100 or -1
  info.tripKm = obc.tripDistM * 0.001
  if st.iceKw <= 0 then
    refreshIcePeak()
  end
  info.iceKw = st.iceKw
  return info
end

-- Runtime option change (tests, not persistent); existing keys of the same type
-- only. dseIceK0Gate false removes the gate proxy. Returns the old value or nil.
local function setOption(key, value)
  local old = cfg[key]
  if old == nil or type(old) ~= type(value) then
    return nil
  end
  cfg[key] = value
  if key == "dseIceK0Gate" then
    if not value then
      removeIceGate()
    end
    dse.gateDone = false
    dse.retryT = 0
  end
  return old
end

M.init = init
M.reset = resetTransient
M.setOption = setOption
M.updateGFX = updateGFX
M.setupGaugeData = setupGaugeData
M.updateGaugeData = updateGaugeData
M.getEnergyInfo = getEnergyInfo
M.resetTripComputer = resetTripComputer

return M
