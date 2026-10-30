-- ETK 854 ActiveHybrid - stepless PWM electric cooling fan
--
-- The base thermal model (combustionEngineThermals.lua) has one binary fan
-- stage; fan airspeed, thermostat and radiator coefficient are module locals
-- without setters.
-- This controller emulates a PWM fan driven by coolant / oil temperature with
-- load pre-control and after-run; the base fan remains as the failsafe stage.
--
-- Emulation: the base fan only raises the radiator face velocity,
--   airCoef = max(v / (15 + v), 0.1)
--   Q_rad   = (Tc - Tenv) * radCoef * thermostat * airCoef * coolantEff
-- The extra Q of the PWM airspeed over the base one (radiatorAirSpeed =
-- max(ram air, base fan)) is removed directly from the coolant integrator
-- engine.thermals.coolantTemperature.
-- engineThermalData is one frame old (controllers update before the
-- powertrain); the delta is zero at speed or with the failsafe fan running,
-- so nothing is counted twice.
--
-- Fan law: duty 0 or dutyMin..1; on at fanOnTemp, linear dutyZeroTemp ->
-- dutyFullTemp, off below fanOffTemp after minOnTime; load pre-control from
-- feedForwardMinTemp. Speed gate: no request while ram air >= the full-duty
-- fan airspeed, unless the coolant override is latched or the oil is hot.
-- The rotor (spin) follows the command with inertia; airflow, power and sound
-- follow the spin, the HUD stage max(command, audible spin).
--
-- Sound: own loop on the base fan event, vol = volumeMax * sqrt(spin); muted
-- while the base failsafe fan plays (volumeMax = radiatorFanVolume).
-- Steam wisps: stock particles at wispEmitNode, pushed away from the radiator
-- top (out through the hood/grille gap); tiers faint 46 / light 46 / heavy 48,
-- none from wispStopTemp or while the stock leak / head-gasket steam runs.
--
-- Electrics (etk854hybrid_*): fanDuty 0..1 (command, failsafe 1), fanStage 0..3
-- (HUD: 1 < 0.6, 2 < 0.9, 3 >= 0.9 / failsafe), fanFailsafe 0/1, fanPowerW W,
-- coolingExtraKW kW, ambientC C, fanSpin 0..1, fanSfx 0/1, wispType 0/46/48,
-- wispCount n (since reset).

local M = {}

M.type = "auxiliary"
M.relevantDevice = nil
M.defaultOrder = 90

local max = math.max
local min = math.min
local abs = math.abs
local sqrt = math.sqrt
local random = math.random

-- Tunables in one table (LuaJIT 60-upvalue limit); defaults mirror
-- etk854hybrid_cooling.jbeam, every key overridable from jbeam.
local CFG = {
  engineName = "mainEngine",
  fanOnTemp = 105.0,
  fanOffTemp = 101.0,
  dutyZeroTemp = 104.0,
  dutyFullTemp = 112.0,
  dutyMin = 0.35,
  minOnTime = 30,
  -- command slew within dutyMin..1
  rampUpPerSecond = 0.15,
  rampDownPerSecond = 0.05,
  -- rotor inertia: 0 -> 0.35 in 2 s, 0.35 -> 0 in 7 s
  spinUpPerSecond = 0.175,
  spinDownPerSecond = 0.05,
  fanAirSpeedMin = 1.7, -- airCoef floor 0.1 == 1.67 m/s
  fanAirSpeedMax = 5.0,
  -- speed gate on ram air (ramAirFactor * |frontAirflow|, m/s); <= 0: fanAirSpeedMax
  speedGateAirSpeed = 0,
  speedGateReleaseAirSpeed = 4.0,
  speedGateOverrideTemp = 110,
  speedGateOverrideHyst = 2.0, -- override release below OverrideTemp - Hyst
  -- load pre-control: low gate in a standstill charge (chargeMode PARK /
  -- NEUTRAL_MAX), drive gate otherwise; feedForwardMinTemp >= fanOnTemp = no-op
  feedForwardLoad = 0.18,
  feedForwardLoadDrive = 0.55,
  feedForwardMaxAirSpeed = 5.0,
  feedForwardDuty = 0.35,
  feedForwardMinTemp = 105.0,
  -- HUD: the rotor counts as running above this spin (still audible, vol ~0.2)
  stageMinSpin = 0.05,
  oilFullFanTemp = 125,
  afterRunMaxTime = 660,
  fanPowerW = 850,
  dcdcEfficiency = 0.90,
  batteryName = "mainBattery",
  drawFromBattery = true,
  fanSoundEvent = "event:>Vehicle>Cooling Fan>Electric_05",
  volumeMax = 0.90,
  pitchMin = 0.75,
  pitchMax = 1.00,
  sfxStopSpin = 0.003, -- loop stopped below this spin
  -- steam wisps (see header)
  wispsEnabled = true,
  wispOnTemp = 110,
  wispHeavyTemp = 115,
  wispStopTemp = 119, -- stock cap steam starts above 120 C
  wispMaxAirspeed = 3.0, -- m/s
  wispRateMin = 0.7, -- particles/s at wispOnTemp
  wispRateMax = 3.5, -- particles/s from wispHeavyTemp
  -- faint tier below wispOnTemp: rotor >= wispFaintMinSpin or after key-off
  wispFaintTemp = 106,
  wispFaintRate = 0.3, -- particles/s
  wispFaintMinSpin = 0.35,
  wispEmitNode = "f15", -- "" = straight up from hyb_rad1
  wispSpeedMin = 0.6, -- m/s
  wispSpeedRand = 0.6,
  wispLightType = 46, -- condensation_exhaust (faintest)
  wispHeavyType = 48, -- steam_light_coolant (stock leak steam)
  wispMinCoolantEff = 0.5, -- no wisps once half the coolant is gone
  -- optional hiss with the heavy wisps (hissVolumeMax 0 = off)
  hissEvent = "event:>Vehicle>Failures>failure_radiator",
  hissVolumeMin = 0.05,
  hissVolumeMax = 0.0,
  -- base-model constants (combustionEngineThermals.lua)
  ramAirFactor = 0.8, -- radiator ram air = ramAirFactor * |frontAirflow|
  coolantSpecHeat = 4000, -- J/(kg K)
  minCoolantMass = 1.5, -- kg
  maxCoolantTemp = 130, -- C
  airCoefHalf = 15, -- m/s, airCoef = v / (15 + v)
  airCoefFloor = 0.1,
  kelvinOffset = 273.15
}

-- Runtime state, also one table.
local S = {
  ok = false,
  engine = nil,
  thermals = nil,
  td = nil, -- thermals.debugData.engineThermalData (updated in place)
  radCoef = 0, -- radiatorArea * radiatorEffectiveness (W/K)
  sfx = nil,
  sfxPlaying = false,
  sfxVol = 0,
  sfxPitch = 0,
  soundNode = nil,
  radTop = nil, -- hyb_rad1 cid
  radBottom = nil, -- hyb_rad2 cid
  wispNode = nil, -- emitter cid (wispEmitNode or hyb_rad1)
  wispRef = nil, -- direction reference cid (particles pushed away from it)
  duty = 0, -- commanded duty
  spin = 0, -- rotor speed
  running = false,
  onTime = 0,
  keyOffTime = 0,
  speedGated = false,
  gateOverride = false,
  powerW = 0,
  extraW = 0,
  wispTick = 0,
  wispType = 0,
  wispRate = 0,
  wispCount = 0,
  hiss = nil,
  hissPlaying = false,
  holdTemp = 0,
  holdLeft = 0
}

local function airCoef(v)
  return max(v / (CFG.airCoefHalf + v), CFG.airCoefFloor)
end

local function publish(duty, failsafe)
  -- stage follows what is heard: the command, or the running-down rotor
  local sd = duty
  if S.spin > CFG.stageMinSpin and S.spin > sd then
    sd = S.spin
  end
  local stage = 0
  if failsafe or sd >= 0.9 then
    stage = 3
  elseif sd >= 0.6 then
    stage = 2
  elseif sd > 0.001 then
    stage = 1
  end
  local ev = electrics.values
  ev.etk854hybrid_fanDuty = failsafe and 1 or duty
  ev.etk854hybrid_fanStage = stage
  ev.etk854hybrid_fanFailsafe = failsafe and 1 or 0
  ev.etk854hybrid_fanPowerW = S.powerW
  ev.etk854hybrid_coolingExtraKW = S.extraW * 0.001
  ev.etk854hybrid_fanSpin = S.spin
  ev.etk854hybrid_fanSfx = S.sfxPlaying and 1 or 0
  ev.etk854hybrid_wispType = S.wispType
  ev.etk854hybrid_wispCount = S.wispCount
end

local function stopSound()
  if S.sfx and S.sfxPlaying then
    obj:stopSFX(S.sfx)
  end
  S.sfxPlaying = false
  S.sfxVol = 0
end

local function stopHiss()
  if S.hiss and S.hissPlaying then
    obj:stopSFX(S.hiss)
  end
  S.hissPlaying = false
end

local function updateSound(spin, failsafe)
  if not S.sfx then
    return
  end
  -- the base failsafe fan plays the same event; stay silent to avoid doubling
  if failsafe or spin <= CFG.sfxStopSpin then
    stopSound()
    return
  end
  local vol = CFG.volumeMax * sqrt(spin)
  local pitch = CFG.pitchMin + (CFG.pitchMax - CFG.pitchMin) * spin
  obj:setVolumePitch(S.sfx, vol, pitch)
  S.sfxVol = vol
  S.sfxPitch = pitch
  if not S.sfxPlaying then
    obj:playSFX(S.sfx)
    S.sfxPlaying = true
  end
end

-- true when the base thermal model is active with a coolant radiator
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
  local radCoef = (tonumber(ed.radiatorArea) or 0) * (tonumber(ed.radiatorEffectiveness) or 0)
  if radCoef <= 0 then
    return false
  end
  S.engine = engine
  S.thermals = th
  S.td = td
  S.radCoef = radCoef
  -- engineGroup "radiator" as the base fan sound uses: hyb_rad1 (top), hyb_rad2 (bottom)
  local rad = ed.radiator
  local radNodes = type(rad) == "table" and rad._engineGroup_nodes or nil
  S.soundNode = (radNodes and radNodes[1]) or engine.engineNodeID
  S.radTop = radNodes and radNodes[1] or nil
  S.radBottom = radNodes and radNodes[2] or nil
  -- wisp emitter: named node away from the radiator top, else top away from bottom
  S.wispNode = S.radTop
  S.wispRef = S.radBottom
  if S.radTop and CFG.wispEmitNode ~= "" then
    for _, n in pairs(v.data.nodes or {}) do
      if n.name == CFG.wispEmitNode and n.cid and n.cid ~= S.radTop then
        S.wispNode = n.cid
        S.wispRef = S.radTop
        break
      end
    end
  end
  S.ok = true
  return true
end

local function drawBattery(powerW, dt)
  if not CFG.drawFromBattery or powerW <= 0 then
    return
  end
  local storage = energyStorage.getStorage(CFG.batteryName)
  if storage and storage.setStoredEnergy and type(storage.storedEnergy) == "number" then
    storage:setStoredEnergy(max(storage.storedEnergy - powerW / CFG.dcdcEfficiency * dt, 0))
  end
end

-- Fan request (0 or dutyMin..1) from temperatures, load, ram air and key state
-- (ignition = ignitionLevel or 2).
local function fanRequest(tc, dt, failsafe, thermostat, ram, ignition)
  local keyOn = ignition >= 2
  if keyOn then
    S.keyOffTime = 0
  else
    S.keyOffTime = S.keyOffTime + dt
  end

  -- speed gate and coolant override, both with hysteresis (kept during failsafe)
  local gateAir = CFG.speedGateAirSpeed > 0 and CFG.speedGateAirSpeed or CFG.fanAirSpeedMax
  if S.speedGated then
    if ram < CFG.speedGateReleaseAirSpeed then
      S.speedGated = false
    end
  elseif ram >= gateAir then
    S.speedGated = true
  end
  if tc >= CFG.speedGateOverrideTemp then
    S.gateOverride = true
  elseif tc < CFG.speedGateOverrideTemp - CFG.speedGateOverrideHyst then
    S.gateOverride = false
  end

  if failsafe then
    return 1
  end
  if not keyOn and S.keyOffTime > CFG.afterRunMaxTime then
    return 0
  end

  local want = 0
  if tc >= CFG.fanOnTemp or (S.running and (tc > CFG.fanOffTemp or S.onTime < CFG.minOnTime)) then
    local x = (tc - CFG.dutyZeroTemp) / (CFG.dutyFullTemp - CFG.dutyZeroTemp)
    want = min(max(x, CFG.dutyMin), 1)
  end

  local oil = S.thermals.oilTemperature
  local oilHot = type(oil) == "number" and oil >= CFG.oilFullFanTemp
  if oilHot then
    want = 1
  end

  -- Pre-control (map-thermostat emulation): running ICE (ignitionCoef 0 = shut
  -- down by the VCU) at high load with little ram air. Low load gate only for
  -- a standstill charge ("CRUISE" is a driving charge).
  local e = S.engine
  if tc >= CFG.feedForwardMinTemp and keyOn and thermostat >= 0.5 and (e.ignitionCoef or 0) > 0 and not e.isStalled then
    local cm = electrics.values.etk854hybrid_chargeMode
    local gate = (cm == "PARK" or cm == "NEUTRAL_MAX") and CFG.feedForwardLoad or CFG.feedForwardLoadDrive
    if (e.engineLoad or 0) >= gate and ram < CFG.feedForwardMaxAirSpeed then
      want = max(want, CFG.feedForwardDuty)
    end
  end

  -- ram air beats the full fan airspeed: the fan adds nothing
  -- (radiatorAirSpeed = max(ram, fan))
  if S.speedGated and not S.gateOverride and not oilHot then
    want = 0
  end

  if want > 0 then
    want = max(want, CFG.dutyMin)
  end
  return want
end

-- Steam wisps + optional hiss; the tick accumulator keeps the density
-- independent of the frame rate.
local function updateWisps(dt, tc, td, ignition)
  S.wispType = 0
  S.wispRate = 0
  if not CFG.wispsEnabled or not S.wispNode or not S.wispRef then
    stopHiss()
    return
  end
  local air = abs(electrics.values.airspeed or 0)
  local faint = false
  local ok = tc >= CFG.wispOnTemp
  if not ok and tc >= CFG.wispFaintTemp then
    -- faint tier: fan working or heat soak after key-off
    faint = S.spin >= CFG.wispFaintMinSpin or ignition < 2
    ok = faint
  end
  ok = ok and tc < CFG.wispStopTemp and air < CFG.wispMaxAirspeed
  if ok then
    -- the stock effects already steam in these cases
    local th = S.thermals
    if th.headGasketBlown or (td.coolantEfficiency or 1) < CFG.wispMinCoolantEff then
      ok = false
    elseif damageTracker and damageTracker.getDamage("engine", "radiatorLeak") then
      ok = false
    end
  end
  if not ok then
    S.wispTick = 0
    stopHiss()
    return
  end

  local span = max(CFG.wispHeavyTemp - CFG.wispOnTemp, 0.1)
  local x = min(max((tc - CFG.wispOnTemp) / span, 0), 1)
  local rate = CFG.wispRateMin + (CFG.wispRateMax - CFG.wispRateMin) * x
  if faint then
    rate = CFG.wispFaintRate
  end
  local heavy = tc >= CFG.wispHeavyTemp
  local ptype = heavy and CFG.wispHeavyType or CFG.wispLightType
  S.wispRate = rate
  S.wispType = ptype
  S.wispTick = min(S.wispTick + rate * dt, 3)
  while S.wispTick >= 1 do
    S.wispTick = S.wispTick - 1
    -- negative velocity = away from the reference node (as the base exhaust smoke)
    obj:addParticleByNodesRelative(S.wispNode, S.wispRef, -(CFG.wispSpeedMin + CFG.wispSpeedRand * random()), ptype, 0, 1)
    S.wispCount = S.wispCount + 1
  end

  if S.hiss then
    if heavy then
      local hx = min(max((tc - CFG.wispHeavyTemp) / max(CFG.wispStopTemp - CFG.wispHeavyTemp, 0.1), 0), 1)
      local vol = CFG.hissVolumeMin + (CFG.hissVolumeMax - CFG.hissVolumeMin) * hx
      obj:setVolumePitch(S.hiss, vol, tc / CFG.maxCoolantTemp)
      if not S.hissPlaying then
        obj:playSFX(S.hiss)
        S.hissPlaying = true
      end
    else
      stopHiss()
    end
  end
end

local function updateGFX(dt)
  if not S.ok or dt <= 0 then
    return
  end
  local th = S.thermals
  local td = S.td
  -- dev hook: pinned coolant temperature (debugSetCoolant with holdS)
  if S.holdLeft > 0 then
    S.holdLeft = S.holdLeft - dt
    th.coolantTemperature = S.holdTemp
  end
  local tc = th.coolantTemperature
  if type(tc) ~= "number" then
    return
  end
  local tEnv = obj:getEnvTemperature() - CFG.kelvinOffset
  local failsafe = td.fanActive == true
  local thermostat = td.thermostatStatus or 0
  local ram = abs(obj:getFrontAirflowSpeed()) * CFG.ramAirFactor

  local ignition = electrics.values.ignitionLevel or 2
  local want = fanRequest(tc, dt, failsafe, thermostat, ram, ignition)
  if want > 0 then
    if not S.running then
      S.running = true
      S.onTime = 0
    end
    S.onTime = S.onTime + dt
  else
    S.running = false
  end

  -- command: 0 or slewed within dutyMin..1 (a PWM fan starts at minimum duty)
  local duty = S.duty
  if want <= 0 then
    duty = 0
  else
    if duty < CFG.dutyMin then
      duty = CFG.dutyMin
    end
    if want > duty then
      duty = min(want, duty + CFG.rampUpPerSecond * dt)
    else
      duty = max(want, duty - CFG.rampDownPerSecond * dt)
    end
  end
  S.duty = duty

  -- Rotor speed follows the command (failsafe = full speed).
  local target = failsafe and 1 or duty
  local spin = S.spin
  if target > spin then
    spin = min(target, spin + CFG.spinUpPerSecond * dt)
  else
    spin = max(target, spin - CFG.spinDownPerSecond * dt)
  end
  S.spin = spin

  local airMax = CFG.fanAirSpeedMax
  local fanAir = 0
  if spin > 0 then
    fanAir = CFG.fanAirSpeedMin + (airMax - CFG.fanAirSpeedMin) * spin
  end

  -- extra heat flow over the base model's (td is one frame old, see header)
  S.extraW = 0
  local air0 = td.radiatorAirSpeed or 0
  if fanAir > air0 and thermostat > 0 then
    local dCoef = airCoef(fanAir) - airCoef(air0)
    local q = (tc - tEnv) * S.radCoef * thermostat * (td.coolantEfficiency or 1) * dCoef
    if q > 0 then
      local mass = max(td.coolantMass or 0, CFG.minCoolantMass)
      th.coolantTemperature = max(tc - q * dt / (mass * CFG.coolantSpecHeat), tEnv)
      S.extraW = q
    end
  end

  -- Fan law: shaft power ~ n^3 ~ airspeed^3.
  local effAir = failsafe and airMax or fanAir
  local r = effAir / airMax
  S.powerW = CFG.fanPowerW * r * r * r
  drawBattery(S.powerW, dt)

  updateSound(spin, failsafe)
  updateWisps(dt, tc, td, ignition)
  electrics.values.etk854hybrid_ambientC = tEnv
  publish(duty, failsafe)
end

local function resetState()
  stopSound()
  stopHiss()
  S.duty = 0
  S.spin = 0
  S.running = false
  S.onTime = 0
  S.keyOffTime = 0
  S.speedGated = false
  S.gateOverride = false
  S.powerW = 0
  S.extraW = 0
  S.wispTick = 0
  S.wispType = 0
  S.wispRate = 0
  S.wispCount = 0
  S.holdLeft = 0
  publish(0, false)
end

local function init(jbeamData)
  jbeamData = jbeamData or {}
  for k, val in pairs(jbeamData) do
    if CFG[k] ~= nil and type(val) == type(CFG[k]) then
      CFG[k] = val
    end
  end
  -- PWM 100 % must equal the base failsafe stage so the hand-over at
  -- radiatorFanTemperature is seamless (delta = 0 while it runs).
  local ed = v.data[CFG.engineName] or {}
  if jbeamData.fanAirSpeedMax == nil and tonumber(ed.radiatorFanMaxAirSpeed) then
    CFG.fanAirSpeedMax = tonumber(ed.radiatorFanMaxAirSpeed)
  end
  CFG.fanAirSpeedMax = max(CFG.fanAirSpeedMax, CFG.fanAirSpeedMin + 0.1)
  CFG.dutyFullTemp = max(CFG.dutyFullTemp, CFG.dutyZeroTemp + 0.5)
  CFG.dutyMin = min(max(CFG.dutyMin, 0.01), 1)
  CFG.spinUpPerSecond = max(CFG.spinUpPerSecond, 0.01)
  CFG.spinDownPerSecond = max(CFG.spinDownPerSecond, 0.01)
  S.sfx = nil
  S.sfxPlaying = false
  S.hiss = nil
  S.hissPlaying = false
  electrics.values.etk854hybrid_ambientC = obj:getEnvTemperature() - CFG.kelvinOffset
  resetState()
  if not resolve() then
    log("W", "etk854hybrid_cooling", "engine thermals unavailable - PWM fan emulation inactive")
  end
end

local function initSecondStage()
  if not S.ok then
    resolve()
  end
end

local function initSounds()
  if not S.ok or not S.soundNode then
    return
  end
  -- same arguments as the base electric fan source
  S.sfx = obj:createSFXSource2(CFG.fanSoundEvent, "AudioDefaultLoop3D", "etk854hybridCoolingFan", S.soundNode, 0)
  S.sfxPlaying = false
  if CFG.wispsEnabled and CFG.hissVolumeMax > 0 and S.radTop then
    S.hiss = obj:createSFXSource2(CFG.hissEvent, "AudioDefaultLoop3D", "etk854hybridCoolingHiss", S.radTop, 0)
  end
  S.hissPlaying = false
end

local function reset()
  resetState()
end

-- Dev/test hook: sets coolant, block and cylinder wall (else the coupling pulls
-- the coolant back within seconds) to tC, clamped -40..maxCoolantTemp; holdS > 0
-- pins the coolant that long. false when the thermals are unavailable.
local function debugSetCoolant(tC, holdS)
  tC = tonumber(tC)
  if not S.ok or not tC then
    return false
  end
  tC = min(max(tC, -40), CFG.maxCoolantTemp)
  local th = S.thermals
  th.coolantTemperature = tC
  if type(th.engineBlockTemperature) == "number" then
    th.engineBlockTemperature = tC
  end
  if type(th.cylinderWallTemperature) == "number" then
    th.cylinderWallTemperature = tC
  end
  holdS = tonumber(holdS) or 0
  S.holdTemp = tC
  S.holdLeft = max(holdS, 0)
  return true
end

-- State snapshot for the QA suite / dev tools (allocates, not for per-frame use).
local function getState()
  return {
    duty = S.duty,
    spin = S.spin,
    running = S.running,
    onTime = S.onTime,
    speedGated = S.speedGated,
    gateOverride = S.gateOverride,
    powerW = S.powerW,
    extraW = S.extraW,
    keyOffTime = S.keyOffTime,
    active = S.ok,
    sfxCreated = S.sfx ~= nil,
    sfxPlaying = S.sfxPlaying,
    sfxVol = S.sfxVol,
    sfxPitch = S.sfxPitch,
    wispType = S.wispType,
    wispRate = S.wispRate,
    wispCount = S.wispCount,
    wispNode = S.wispNode,
    wispRef = S.wispRef,
    hissCreated = S.hiss ~= nil,
    hissPlaying = S.hissPlaying,
    holdLeft = S.holdLeft
  }
end

M.init = init
M.initSecondStage = initSecondStage
M.initSounds = initSounds
M.reset = reset
M.updateGFX = updateGFX
M.getState = getState
M.debugSetCoolant = debugSetCoolant

return M
