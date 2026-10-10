-- ETK 854 ActiveHybrid - electric cooling fan manager (OEM-style PWM fan)
--
-- Why this exists: the base thermal model
--   lua/vehicle/powertrain/combustionEngineThermals.lua (updateWaterCoolingGFX
--   :687-961)
-- knows exactly ONE binary electric fan stage (on at radiatorFanTemperature,
-- off at thermostat * 1.05, :714-731), and its fan airspeed, thermostat and
-- radiator coefficient are module locals without setters. A real production car has
-- a stepless 600-850 W brushless PWM fan commanded by the engine ECU from coolant
-- and oil temperature plus load pre-control, with after-run after key-off.
--
-- Emulation: the fan's ONLY physical effect in the base model is a higher
-- radiator face velocity through
--   airCoef = max(v / (15 + v), 0.1)        (:766)
--   Q_rad   = (Tc - Tenv) * radCoef * thermostat * airCoef * coolantEff (:772)
-- so we compute the extra heat flow our PWM airspeed would add OVER the
-- airspeed the base model used in the PREVIOUS frame (ram air or its own
-- failsafe fan, debugData.radiatorAirSpeed; controller.updateGFX runs before
-- powertrain.updateGFX, main.lua:88/:92, and engineThermalData is refreshed
-- only inside the thermals updateGFX, :917-933) and remove that energy from the
-- integrator state engine.thermals.coolantTemperature (a public module field
-- read every frame at :717/:745/:771/:789). Using the max()-delta means no
-- double counting: at speed, or with the base failsafe fan running, the delta
-- is zero by construction (up to a self-correcting one-frame lag at the
-- failsafe hand-over, below 1 frame x dQ).
--
-- Fan law (2026-09-30 feedback: "later, only when clearly too hot, audible"):
--   command duty 0 or dutyMin..1: on at fanOnTemp, linear dutyZeroTemp ->
--   dutyFullTemp, off below fanOffTemp after minOnTime. Load pre-control only
--   from feedForwardMinTemp (map-thermostat emulation; the jbeam sets it to
--   fanOnTemp, which makes it a no-op). Speed gate: no request while the
--   radiator ram air is >= the full-duty fan airspeed (the fan adds nothing
--   there, :765), unless the coolant override is latched (set at
--   speedGateOverrideTemp, cleared speedGateOverrideHyst below it) or the oil
--   is hot. The rotor speed ("spin") follows the command with inertia;
--   airflow, power and sound follow the spin, the published duty follows the
--   command, the HUD stage follows max(command, audible spin).
--
-- Sound: own loop source on the base fan event (the base fan sound source is a
-- module local without setter, :143/:145/:1876-1881). vol = volumeMax *
-- sqrt(spin), pitch pitchMin..pitchMax. volumeMax should equal
-- mainEngine.radiatorFanVolume so the base failsafe (pitch 1, :719-721) takes
-- over at the same level.
--
-- Steam wisps (optional, wispsEnabled): stock particle types only, emitted
-- with obj:addParticleByNodesRelative (same call as the base radiator steam,
-- :873-878). Emitter: body node wispEmitNode (f15, front crossmember top at
-- x 0, y -1.95, z 0.835, etk800_body_wagon.jbeam:236), pushed away from our
-- radiator top hyb_rad1 (y -1.78, z 0.80), i.e. forward and ~12 deg up towards
-- the hood front edge h4 (y -2.11, z 0.835, etk800_hood.jbeam:96), so the
-- steam leaves through the hood/grille gap instead of rising through the
-- closed hood panel. Fallback without that node: straight up hyb_rad2 ->
-- hyb_rad1. Tiers: faint (type 46, low rate) from wispFaintTemp while the
-- fan runs or after key-off, light 46 from wispOnTemp, heavy 48 from
-- wispHeavyTemp. Handed over to the stock cap steam, which
-- starts above 120 C (:806-812), and suppressed whenever the stock radiator /
-- head-gasket steam already runs (:707-708, :873-885).
--
-- Published electrics:
--   etk854hybrid_fanDuty       0..1  commanded fan duty, 0 or >= dutyMin (failsafe = 1)
--   etk854hybrid_fanStage      0..3  HUD stage of max(duty, spin > stageMinSpin): 0 off,
--                                    1 < 0.60, 2 < 0.90, 3 >= 0.90 / failsafe
--   etk854hybrid_fanFailsafe   0/1   base binary fan (>= radiatorFanTemperature)
--   etk854hybrid_fanPowerW     W     electrical fan power
--   etk854hybrid_coolingExtraKW kW   extra radiator heat flow of the PWM fan
--   etk854hybrid_ambientC      C     environment temperature seen by thermals
--   etk854hybrid_fanSpin       0..1  rotor speed (drives airflow and sound)
--   etk854hybrid_fanSfx        0/1   own fan sound source playing
--   etk854hybrid_wispType      0/46/48 particle type emitted this frame window (0 = none)
--   etk854hybrid_wispCount     n     wisp particles emitted since the last reset

local M = {}

M.type = "auxiliary"
M.relevantDevice = nil
M.defaultOrder = 90

local max = math.max
local min = math.min
local abs = math.abs
local sqrt = math.sqrt
local random = math.random

-- All tunables in ONE table (LuaJIT 60-upvalue limit). Defaults mirror
-- etk854hybrid_cooling.jbeam; every key can be overridden from jbeam.
local CFG = {
  engineName = "mainEngine",
  fanOnTemp = 105.0,
  fanOffTemp = 101.0,
  dutyZeroTemp = 104.0,
  dutyFullTemp = 112.0,
  dutyMin = 0.35,
  minOnTime = 30,
  -- command slew between dutyMin and 1 (PWM controller smoothing)
  rampUpPerSecond = 0.15,
  rampDownPerSecond = 0.05,
  -- rotor inertia: 0 -> 0.35 in 2 s, 0.35 -> 0 in 7 s
  spinUpPerSecond = 0.175,
  spinDownPerSecond = 0.05,
  fanAirSpeedMin = 1.7, -- airCoef floor 0.1 == 1.67 m/s (:766)
  fanAirSpeedMax = 5.0,
  -- Speed gate on the radiator ram air (0.8 * |frontAirflow|, :765).
  -- speedGateAirSpeed <= 0 means "use fanAirSpeedMax".
  speedGateAirSpeed = 0,
  speedGateReleaseAirSpeed = 4.0,
  speedGateOverrideTemp = 110,
  -- override latched at >= speedGateOverrideTemp, released below
  -- speedGateOverrideTemp - speedGateOverrideHyst (no on/off chatter at 110 C)
  speedGateOverrideHyst = 2.0,
  -- Pre-control, two load gates: standstill N-charge (etk854hybrid_chargeMode
  -- PARK/NEUTRAL_MAX, load ~0.23 on the N57S curve) vs. generic high load.
  -- Only active from feedForwardMinTemp on (no temperature-independent fan);
  -- feedForwardMinTemp >= fanOnTemp makes it a no-op (dutyMin is reached
  -- there by the fan law anyway). The speed gate zeroes any request from
  -- fanAirSpeedMax on, so feedForwardMaxAirSpeed above that has no effect.
  feedForwardLoad = 0.18,
  feedForwardLoadDrive = 0.55,
  feedForwardMaxAirSpeed = 5.0,
  feedForwardDuty = 0.35,
  feedForwardMinTemp = 105.0,
  -- HUD stage: the rotor counts as running above this spin (audible,
  -- vol = 0.9 * sqrt(0.05) = 0.2), so the FAN chip stays while it is heard
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
  sfxStopSpin = 0.003, -- below this rotor speed the loop is stopped (vol ~0.05)
  -- steam wisps from the radiator top (stock particle types, see header)
  wispsEnabled = true,
  wispOnTemp = 110,
  wispHeavyTemp = 115,
  wispStopTemp = 119, -- stock cap steam starts above 120 C (:156, :806)
  wispMaxAirspeed = 3.0, -- electrics.airspeed m/s
  wispRateMin = 0.7, -- particles/s at wispOnTemp
  wispRateMax = 3.5, -- particles/s from wispHeavyTemp on
  -- faint tier below wispOnTemp: only while the rotor runs at least
  -- wispFaintMinSpin (fan on) or after key-off (heat soak)
  wispFaintTemp = 106,
  wispFaintRate = 0.3, -- particles/s
  wispFaintMinSpin = 0.35,
  -- emitter node name ("" = straight up from hyb_rad1, direction hyb_rad2 -> hyb_rad1)
  wispEmitNode = "f15",
  wispSpeedMin = 0.6, -- m/s away from hyb_rad1 (or hyb_rad2 in the fallback)
  wispSpeedRand = 0.6,
  wispLightType = 46, -- BNGP_46 condensation_exhaust, faintest
  wispHeavyType = 48, -- BNGP_48 steam_light_coolant (stock rad leak <= 95 C, :875)
  wispMinCoolantEff = 0.5, -- no wisps once half the coolant is gone
  -- optional quiet hiss with the heavy wisps (0 = off, no source created)
  hissEvent = "event:>Vehicle>Failures>failure_radiator", -- same event as base :1872
  hissVolumeMin = 0.05,
  hissVolumeMax = 0.0,
  -- base-model constants (combustionEngineThermals.lua)
  coolantSpecHeat = 4000, -- :152
  minCoolantMass = 1.5, -- :154
  maxCoolantTemp = 130, -- :157
  airCoefHalf = 15, -- :766
  airCoefFloor = 0.1, -- :766
  kelvinOffset = 273.15
}

-- Runtime state, also one table.
local S = {
  ok = false,
  engine = nil,
  thermals = nil,
  td = nil, -- thermals.debugData.engineThermalData (updated in place, :917-957)
  radCoef = 0, -- radiatorArea * radiatorEffectiveness (W/K), :1801
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
  -- While the base failsafe fan runs, the base code plays the same event at
  -- radiatorFanVolume / pitch 1 (:719-723); stay silent to avoid doubling.
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

-- Resolves the engine and its thermals module. Returns true when the base
-- thermal model is active and has a coolant radiator.
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
  -- Same nodes the base fan sound / radiator steam use (nodes.radiator[1..2],
  -- :1815, :876-877, :1877-1881): engineGroup "radiator" = hyb_rad1 (top),
  -- hyb_rad2 (bottom) in jbeam order.
  local rad = ed.radiator
  local radNodes = type(rad) == "table" and rad._engineGroup_nodes or nil
  S.soundNode = (radNodes and radNodes[1]) or engine.engineNodeID
  S.radTop = radNodes and radNodes[1] or nil
  S.radBottom = radNodes and radNodes[2] or nil
  -- wisp emitter: named body node pushed away from the radiator top, else
  -- the radiator top pushed away from the radiator bottom (straight up)
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

-- Engine ECU fan request (0 or dutyMin..1) from temperatures, load, ram air and
-- key state.
local function fanRequest(tc, dt, failsafe, thermostat, ram)
  local ignition = electrics.values.ignitionLevel or 2
  local keyOn = ignition >= 2
  if keyOn then
    S.keyOffTime = 0
  else
    S.keyOffTime = S.keyOffTime + dt
  end

  -- Speed gate with hysteresis (state kept even while the failsafe runs).
  local gateAir = CFG.speedGateAirSpeed > 0 and CFG.speedGateAirSpeed or CFG.fanAirSpeedMax
  if S.speedGated then
    if ram < CFG.speedGateReleaseAirSpeed then
      S.speedGated = false
    end
  elseif ram >= gateAir then
    S.speedGated = true
  end
  -- Coolant override of the gate, latched with hysteresis.
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

  -- Pre-control: a running ICE at high load with little ram air gets the fan
  -- somewhat before fanOnTemp (emulates the map thermostat's lower target at
  -- high load), but never below feedForwardMinTemp. ignitionCoef is 0 while
  -- the VCU has the ICE shut down. Low gate only while the VCU runs a
  -- standstill charge (etk854hybridControl publishes chargeMode
  -- "PARK"/"NEUTRAL_MAX"; "CRUISE" is driving charge).
  local e = S.engine
  if tc >= CFG.feedForwardMinTemp and keyOn and thermostat >= 0.5 and (e.ignitionCoef or 0) > 0 and not e.isStalled then
    local cm = electrics.values.etk854hybrid_chargeMode
    local gate = (cm == "PARK" or cm == "NEUTRAL_MAX") and CFG.feedForwardLoad or CFG.feedForwardLoadDrive
    if (e.engineLoad or 0) >= gate and ram < CFG.feedForwardMaxAirSpeed then
      want = max(want, CFG.feedForwardDuty)
    end
  end

  -- Ram air already beats the fan's full airspeed: the fan adds nothing
  -- (radiatorAirSpeed = max(0.8 * airflow, fanAirSpeed), :765), so stay off
  -- unless the coolant (latched override) or the oil is clearly too hot.
  if S.speedGated and not S.gateOverride and not oilHot then
    want = 0
  end

  if want > 0 then
    want = max(want, CFG.dutyMin)
  end
  return want
end

-- Steam wisps + optional hiss. Rate-limited by a tick accumulator like the
-- base particles (:796-800) so the density does not depend on frame rate.
local function updateWisps(dt, tc, td)
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
    faint = S.spin >= CFG.wispFaintMinSpin or (electrics.values.ignitionLevel or 2) < 2
    ok = faint
  end
  ok = ok and tc < CFG.wispStopTemp and air < CFG.wispMaxAirspeed
  if ok then
    -- the stock effects already steam in these cases (:707-708, :873-885)
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
    -- negative velocity = away from node2 (hyb_rad1 behind -> forward out of
    -- the hood/grille gap; fallback hyb_rad2 below -> upwards); same sign
    -- convention as fire.lua:95 and the exhaust smoke (:447, :504)
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
  local ram = abs(obj:getFrontAirflowSpeed()) * 0.8

  local want = fanRequest(tc, dt, failsafe, thermostat, ram)
  if want > 0 then
    if not S.running then
      S.running = true
      S.onTime = 0
    end
    S.onTime = S.onTime + dt
  else
    S.running = false
  end

  -- Commanded duty: 0, or a slewed value in dutyMin..1 (a PWM fan starts at
  -- its minimum duty, it does not creep up from 0).
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

  -- Extra heat flow over what the base model applied in the previous frame
  -- (td is one frame old here, see header).
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
  updateWisps(dt, tc, td)
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
  -- same arguments as the base electric fan source (:1881)
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

-- Dev/test hook (not bound to any input): sets the coolant temperature to tC
-- through the same public field the PWM emulation writes. Block and cylinder
-- wall are set too, otherwise the 24000 W/K coolant-block coupling (:695)
-- pulls the coolant back within seconds. holdS > 0 pins the coolant value for
-- that many seconds (re-written every frame before the fan logic).
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
