-- ETK 854 ActiveHybrid - instrument cluster data module
-- =========================================================================
-- Target: BeamNG.drive 0.39.4
--
-- A custom module for the base-game controller gauges/genericGauges (the
-- controller named "gauge" that the stock ETK 800 dash declares). It is
-- declared by the parts in etk854hybrid_gauges.jbeam as a controller row
-- with fileName "gauges/customModules/etk854hybrid_gaugeData" and NO custom
-- name: genericGauges resolves custom modules with
--   controller.getController("gauges/customModules/" .. moduleName)
-- (lua/vehicle/controller/gauges/genericGauges.lua:144-153), so the
-- controller name has to equal the fileName.
--
-- The file is found through the vehicle-first package.path
-- (lua/vehicle/main.lua:212), no shim needed. It does NOT shadow any
-- base-game file: the base modules live in lua/vehicle/controller/gauges/
-- customModules/ and none of them is called etk854hybrid_*.
--
-- Contract (same as the stock modules, e.g. electricMotorData.lua):
--   M.type = "auxiliary"
--   init(jbeamData)            controller init, row options below
--   reset()                    R-reset (genericGauges itself is NOT re-set up)
--   updateGFX(dt)              every frame, seated or not
--   setupGaugeData(props, tex) once, props = { <property> = true } from the
--                              displayData.customModules rows of the part
--   updateGaugeData(m, dtPush) every UI push while a player is seated;
--                              m persists between pushes, dtPush is the time
--                              since the previous push, NOT the frame time
--                              (genericGauges.lua:80-92)
--
-- What it provides
--   1. electrics.values.etk854hybrid_tachoRpm (every frame)
--      Drives the rpm needle prop of the etk854hybrid_gauges parts instead of
--      the raw "rpm" channel. The stock ETK dial face already has a READY /
--      OFF bracket below 0 rpm; this channel parks the needle there like a
--      production ActiveHybrid rev counter:
--        ICE turning (rpm > iceRpmMin)       -> engine rpm
--        ICE still, ignition on (level >= 2) -> tachoReadyRpm (0 = READY)
--        ignition off / accessory            -> tachoOffRpm (-500 = OFF)
--      Sweeps between READY and OFF are rate limited (tachoSweepRate).
--      Needle geometry (etk800_gauges.jbeam, rpm prop): petrol 0.03 deg/rpm
--      offset -3500, diesel 0.035 deg/rpm offset -2750. On both faces the
--      READY mark sits at ~0 rpm and the OFF mark at ~-500 rpm (measured on
--      common.zip etk_gauges_*_kmh_b.color.dds), so one value pair fits both.
--   2. customModules.etk854hybrid_gaugeData (per UI push, only what the part
--      asked for in displayData.customModules):
--        range  -> rangeKm    electric range estimate [km], -1 = unknown
--                  consKwh    learned consumption [kWh/100 km]
--        scale  -> iceKw      ICE peak power [kW] (from the engine curve)
--                  evKw       e-machine peak drive power of the boost stage
--                  regenKw    e-machine peak regen power of the boost stage
--        gear   -> gearStyle  "number" | "plain" (row option gearStyle)
--        fuel   -> on-board computer, see item 10
--      Everything else the page needs comes straight from the electrics list
--      in displayData (genericGauges copies electrics.values[k] or 0).
--   3. M.getEnergyInfo() for other controllers (etk854hybrid_screenData uses
--      it so cluster and centre screen show the same range).
--   4. DSE diagnostics (every frame, read-only, nil-safe):
--        etk854hybrid_dseMotorTf   integratedMotor.throttleFactor (1 = no trim)
--        etk854hybrid_dseIceTf     mainEngine.throttleFactor
--        etk854hybrid_dseSrc       bitmask of what the DSE is doing this frame
--            1  motorTorqueControl.isActingAsTC (torque-stage TC; the flag
--               only holds the LAST wheel group stepped, motorTorqueControl.
--               lua:53/:72 - with our list order that is integratedMotor)
--            2  motorTorqueControl.isActingAsYC (torque-stage yaw control)
--            4  electrics.isTCBrakeActive (brake TC, factor > 0.1,
--               brakeControl.lua:97/:236)
--            8  electrics.isYCBrakeActive (brake yaw, factor > 0.3,
--               brakeControl.lua:170/:235)
--           16  mainEngine throttleFactor < 1 (TC or yaw trim on the ICE)
--           32  integratedMotor throttleFactor < 1
--           64  brakeControl.isActingAsTC (any brake factor > 0)
--          128  brakeControl.isActingAsYC (any brake factor > 0)
--        etk854hybrid_dseSlipIce / _dseSlipEm  wheelGroup.maxSlip of the two
--               TC groups (virtualSpeedSlip.lua:53)
--        etk854hybrid_dseBsa       CMU virtualSensors virtual.bodySlipAngle [rad]
--        etk854hybrid_dseVirtKmh   CMU virtualSensors virtual.speed [km/h]
--      All values are sampled once per frame (the flags are written per
--      physics step), so they give a duty estimate, not every 0.5 ms event.
--   5. electrics.values.etk854hybrid_dscLamp (every frame): the QUALIFIED
--      DSC telltale for cluster chip, HUD badge and QA. 1 while
--        min(motorTf, iceTf) < lampTfMax for >= lampOnS continuously
--          (iceTf counts only while K0 is closed, k0ClutchRatio >= lampK0Min:
--          trimming an ICE that idles behind an open K0 changes nothing at
--          the wheels), or
--        a DSE brake stage is active (isTCBrakeActive / isYCBrakeActive),
--      then held for lampHoldS after the last qualifying frame. The raw
--      escActive/tcsActive flags (and etk854hybrid_tcActive in the VCU) turn
--      on after ~0.2 s of ANY didAct, even a 0.999 throttle factor
--      (motorTorqueControl.lua:72/:98, yawControl.lua:90-96,
--      tractionControl.lua:71-77), which is what made the chip blink.
--   6. ICE K0 gate for traction control (row option dseIceK0Gate, default on).
--      virtualSpeedSlip only records slip while wheelGroup.motor.throttle >
--      0.01 (virtualSpeedSlip.lua:41), and combustionEngine device.throttle
--      includes the idle throttle (combustionEngine.lua:737/:801). So the
--      "mainEngine" TC group measured slip - and trimmed / flagged tcsActive -
--      while the ICE only idled behind an open K0 and drove nothing. The
--      gate swaps that group's motor reference (tractionControl.lua:157-164,
--      exposed through tractionControl.tractionControlledWheels) for a
--      read-through proxy whose "throttle" reads 0 while
--      k0ClutchRatio < dseIceK0GateMin. The only readers of wheelGroup.motor
--      are virtualSpeedSlip.lua:41 (.throttle) and motorTorqueControl.lua:54/
--      :58 (.name); both keep working. The proxy does not touch the device;
--      ICE throttleFactor is still written on the real device
--      (motorTorqueControl.lua:43-45). Nothing base-game is overridden.
--   7. electrics.values.etk854hybrid_gearActual (every frame) = the gearbox
--      device's engaged gearIndex. electrics.gearIndex is the 8HP's DECIDED
--      target (etk854hybrid_8hp.lua:506/:559) and runs ahead by the shift
--      time. customModules.etk854hybrid_gaugeData "gear" -> gearStyle
--      ("number" = D1..D8, "plain" = D without number), rendered by the page.
--      GEAR_UNKNOWN (-99) until the gearbox device is resolved: genericGauges
--      turns a missing electrics key into 0 (genericGauges.lua:61), which
--      would read as "neutral", so the page needs an explicit marker to fall
--      back to electrics.gearIndex.
--   8. Qualified STOCK telltales (row option dseStockLampQualified, default
--      on). The dial-face ESC / TCS decals are glowMaps on electrics esc /
--      tcs (etk800.jbeam:62-63), written every frame as
--      floor(isActiveSmoothed) * CMU.warningLightPulse (yawControl.lua:92,
--      tractionControl.lua:73), i.e. on the same raw didAct as escActive /
--      tcsActive. This module has no defaultOrder, so it runs at order
--      100000 (controller.lua:360), after CMU (50), yawControl (60) and
--      tractionControl (65), and before material.updateGFX reads the
--      glowMaps (main.lua:88 vs :112). While a supervisor is enabled, esc /
--      tcs are rewritten to CMU.warningLightPulse only while the qualified
--      lamp is on AND that supervisor's raw flag is on or was on within the
--      last lampHoldS; otherwise 0. dseWarningPulse (UI DSC button blink,
--      CMU.lua:196-197) is rewritten the same way while the CMU state is
--      normal. Untouched: escActive / tcsActive (read by the VCU
--      etk854hybrid_tcActive, the 8HP and QA T18), the steady 1 of a
--      DISABLED supervisor (yawControl.lua:94-96, tractionControl.lua:75-77;
--      recognised as esc/tcs == 1 with the raw flag off) and everything
--      while etk854hybrid_dscMode is "OFF". The supervisors rewrite the raw
--      values every frame before this module runs, so the rewrite does not
--      feed back into the next frame. The raw values are kept for telemetry
--      as etk854hybrid_dseEscRaw / etk854hybrid_dseTcsRaw.
--   9. electrics.values.etk854hybrid_gearHint (every frame): the OEM-style
--      shift recommendation the page / HUD draw next to the engaged gear
--      (triangle + gear). Source: etk854hybrid_gearPlanned from the 8HP core
--      (etk854hybrid_8hpCore.lua C.decide: decided / debounced target in D
--      and S, shadow plan of the D program in M<n> and temporary manual).
--      0 = no recommendation. Shown when planned ~= engaged for gearHintOnS
--      (D / S; a normal shift takes ~0.3 s and moves the engaged gear at its
--      start) or gearHintOnManualS (M<n>), cleared after gearHintOffS or at
--      once when the engaged gear changes.
--  10. On-board computer (every frame, nil-safe: no fuel tank -> fuelL -1):
--        fuel tank    energyStorage cfg.fuelTankName ("mainTank", fuelTank.lua:
--                     remainingVolume = storedEnergy / (density * energyDensity)
--                     :11, capacity in l), read live from storedEnergy
--        instantaneous  fuel flow = tank volume drop per frame (l/s), first-
--                     order filter fuelInstTau; l/100 km = flow / speed * 1e5,
--                     l/h = flow * 3600 below fuelInstLhSpeed; mode "ev" while
--                     the VCU has the ICE off (etk854hybrid_iceState < 0.5)
--        trip average fuel used / distance since the last resetTripComputer()
--                     (or spawn); kept across R-resets (the R refill of the tank
--                     is skipped: any volume RISE re-baselines, no negative use)
--        fuel range   litres / learned consumption WHILE THE ICE RUNS (same
--                     distance-weighted estimator as the electric one, default
--                     defaultFuelConsumption) * 100
--        electric average = the learned consKwh above (the SAME number the
--                     centre screen shows as its ELECTRIC average, so both agree)
--      electrics: etk854hybrid_fuelL, _fuelInstL100 (-1 = n/a), _fuelInstLh,
--        _fuelInstMode ("l100" | "lh" | "ev"), _fuelAvgL100 (-1 until
--        tripMinKm), _tripKm, _fuelRangeKm (-1), _evAvgKwh, _tripId
--      customModules property "fuel" -> fuelL fuelCapL fuelRangeKm instL100
--        instLh instMode instBar(0..1) avgL100 tripKm avgKwh tripId
--      M.resetTripComputer() (Actions app / console) starts a new trip.
--
-- Runtime options (tests): M.setOption(key, value), see below.
--
-- Range estimate
--   usable energy = (SOC - socFloor) * capacity of the battery storage
--   consumption   = distance-weighted average of the battery energy drop per
--                   distance, sampled once per second while the ICE is OFF
--                   (EV / REGEN / SAIL). Starts at defaultConsumption and
--                   keeps the learned value across R-resets.
--   range         = usable / consumption * 100
--   socFloor 0.10 = batteryMonitor SOC_LOW1 (below it the EV budget ends).
--
-- Cost: one table lookup batch per frame for the needle and the DSE
-- channels, one storage read per second, a handful of numbers per UI push.
-- No allocations per frame (the gate proxy is created once). The gate's
-- __index runs once per physics step for the ICE TC group.
-- =========================================================================

local M = {}
M.type = "auxiliary"

local max, min, abs = math.max, math.min, math.abs

-- Boost stage -> e-machine limits. Mirrors BOOST_STAGES in
-- etk854hybridControl.lua (powerKw / regenKw) - keep both in sync.
local STAGE_EV_KW = {[0] = 83, [1] = 110, [2] = 150, [3] = 200}
local STAGE_REGEN_KW = {[0] = 55, [1] = 65, [2] = 80, [3] = 95}

-- combustionEngine device.maxPower is in mechanical hp
-- (torqueToPower = 1 / 7120.9, lua/vehicle/powertrain/combustionEngine.lua:21)
local HP_TO_KW = 0.7457

local J_PER_KWH = 3600000

-- etk854hybrid_gearActual while no gearbox device is resolved (header item 7)
local GEAR_UNKNOWN = -99

local cfg = {
  engineName = "mainEngine",
  batteryName = "mainBattery",
  socFloor = 0.10,
  defaultConsumption = 21.0, -- kWh/100 km, reference-hybrid-like mixed driving
  consumptionMin = 12.0,
  consumptionMax = 40.0,
  consumptionDistance = 3000, -- m, averaging distance of the estimator
  sampleInterval = 1.0, -- s
  tachoReadyRpm = 0,
  tachoOffRpm = -500,
  tachoSweepRate = 1800, -- rpm/s for the READY <-> OFF sweeps
  iceRpmMin = 150,
  -- DSE / DSC lamp / gear
  motorName = "integratedMotor",
  gearboxName = "gearbox",
  lampTfMax = 0.90, -- throttle factor below this counts as a real trim
  lampOnS = 0.25, -- s of continuous trim before the lamp lights
  lampHoldS = 1.0, -- s the lamp stays on after the last qualifying frame
  lampK0Min = 0.9, -- K0 ratio from which the ICE factor counts
  dseIceK0Gate = true,
  dseIceK0GateMin = 0.1, -- K0 ratio below which the ICE TC group sees throttle 0
  dseStockLampQualified = true, -- header item 8
  gearStyle = "number", -- "number" (D1..D8) | "plain" (D, S<n>, M<n>)
  -- shift recommendation debounce (header item 9) [s]
  gearHintOnS = 0.25, -- D / S: planned != engaged this long (a shift takes ~0.3 s)
  gearHintOnManualS = 0.10, -- M<n> / temporary manual
  gearHintOffS = 0.20, -- recommendation gone this long (a shift clears at once)
  -- on-board computer (header item 10)
  fuelTankName = "mainTank",
  defaultFuelConsumption = 8.0, -- l/100 km with the ICE running (R6 hybrid, mixed)
  fuelConsumptionMin = 3.0,
  fuelConsumptionMax = 30.0,
  fuelInstTau = 0.6, -- s, filter of the instantaneous flow
  fuelInstLhSpeed = 1.4, -- m/s (~5 km/h): below this l/h instead of l/100 km
  fuelBarMaxL100 = 30.0, -- full instantaneous bar
  fuelBarMaxLh = 10.0,
  tripMinKm = 0.2 -- trip average shown from this distance on
}

-- on-board computer state (header item 10), one table
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
  consFuel = 8.0, -- learned l/100 km while the ICE runs
  sampleT = 0,
  sampleDist = 0,
  sampleFuel = 0
}

-- shift recommendation display state (header item 9)
local hint = {actual = nil, shown = 0, t = 0}

-- all mutable state in one table (keeps the upvalue count low)
local st = {
  -- needle
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
  -- scale
  iceKw = 0,
  -- requested properties
  wantRange = false,
  wantScale = false,
  wantGear = false,
  wantFuel = false
}

-- DSE handles (resolved on the first frames, retried at 1 Hz while missing)
-- and DSC lamp timers
local dse = {
  resolved = false,
  retryT = 0,
  mtc = nil, -- motorTorqueControl
  bc = nil, -- brakeControl
  cmu = nil, -- CMU (virtualSensors)
  tc = nil, -- tractionControl supervisor
  ice = nil,
  em = nil,
  gbx = nil,
  grpIce = nil, -- tractionControl wheelGroup of the ICE
  grpEm = nil, -- tractionControl wheelGroup of the e-machine
  gateDone = false,
  lowT = 0,
  holdT = 0,
  escT = 0, -- s left since yawControl's raw escActive was last on
  tcsT = 0 -- s left since tractionControl's raw tcsActive was last on
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
    -- engine turning: follow the real rpm without lag, but come out of the
    -- READY/OFF zone at the sweep rate so the needle does not snap
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
-- header item 10. Live volume from storedEnergy (fuelTank.lua:11 derives
-- remainingVolume the same way, but only in its own updateGFX, which runs
-- after the controllers, main.lua:88/:93).
local MAX_FLOW_LPS = 0.05 -- 180 l/h: anything faster is a leak / manipulation, not consumption

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
    -- a RISE (refuel, R refill) or an implausible drop re-baselines
    if used < 0 or used > MAX_FLOW_LPS * dt then
      used = 0
    end
  end
  obc.lastVol = vol
  obc.tripFuelL = obc.tripFuelL + used
  obc.tripDistM = obc.tripDistM + dist

  -- instantaneous (filtered flow)
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

  -- learned consumption while the ICE runs (fuel range), same estimator as
  -- the electric one (updateEstimator)
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

-- Starts a new trip (average fuel consumption + trip distance). The electric
-- average is the learned consumption behind the EV range and is kept.
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
-- true for boolean true or a number > 0.5 (electrics flags arrive as either)
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

-- See header item 6. Returns true when the gate is in place (or not wanted /
-- not applicable), false while the TC group does not exist yet.
local function installIceGate()
  if not cfg.dseIceK0Gate then
    return true
  end
  local g = dse.grpIce
  if not g then
    -- resolved without an ICE TC group (no DSE, or no ICE listed): nothing to gate
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
        -- nil on a vehicle without our VCU: no gating
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

-- Undo installIceGate (runtime option dseIceK0Gate = false, see M.setOption)
local function removeIceGate()
  local g = dse.grpIce
  local p = g and g.motor
  if type(p) == "table" and rawget(p, "etk854hybridK0Gate") then
    g.motor = rawget(p, "etk854hybridDevice") or p
  end
end

-- See header item 8. lampOn = qualified lamp state of this frame.
local function qualifyStockLamps(ev, dt, lampOn)
  if ev.etk854hybrid_dscMode == "OFF" then
    -- DSC OFF: keep the stock steady-on telltale, also during the fade-out
    -- of a raw flag right after the switch
    dse.escT, dse.tcsT = 0, 0
    return
  end
  local cmu = dse.cmu
  local pulse = cmu and num(cmu.warningLightPulse, 0) or 0
  local escAct, tcsAct = on(ev.escActive), on(ev.tcsActive)
  dse.escT = escAct and cfg.lampHoldS or max(0, dse.escT - dt)
  dse.tcsT = tcsAct and cfg.lampHoldS or max(0, dse.tcsT - dt)
  local escRaw, tcsRaw = ev.esc, ev.tcs
  -- nil = supervisor not fitted; 1 with the raw flag off = supervisor disabled
  if escRaw ~= nil and not (escRaw == 1 and not escAct) then
    ev.esc = (lampOn and (escAct or dse.escT > 0)) and pulse or 0
  end
  if tcsRaw ~= nil and not (tcsRaw == 1 and not tcsAct) then
    ev.tcs = (lampOn and (tcsAct or dse.tcsT > 0)) and pulse or 0
  end
  -- CMU.lua:197 writes 1 in a degraded / offline system state; keep that
  local states = cmu and cmu.systemStates
  if ev.dseWarningPulse ~= nil and states and cmu.systemState == states.normal then
    ev.dseWarningPulse = lampOn and pulse or 0
  end
end

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
  ev.etk854hybrid_dseSrc = src

  local vs = dse.cmu and dse.cmu.virtualSensors
  local virt = type(vs) == "table" and vs.virtual or nil
  if type(virt) == "table" then
    ev.etk854hybrid_dseVirtKmh = num(virt.speed, 0) * 3.6
    ev.etk854hybrid_dseBsa = num(virt.bodySlipAngle, 0)
  end
  if dse.grpIce then ev.etk854hybrid_dseSlipIce = num(dse.grpIce.maxSlip, 0) end
  if dse.grpEm then ev.etk854hybrid_dseSlipEm = num(dse.grpEm.maxSlip, 0) end

  -- qualified DSC lamp (header item 5)
  local tf = motorTf
  if num(ev.etk854hybrid_k0ClutchRatio, 1) >= cfg.lampK0Min then
    tf = min(tf, iceTf)
  end
  if tf < cfg.lampTfMax then
    dse.lowT = dse.lowT + dt
  else
    dse.lowT = 0
  end
  if dse.lowT >= cfg.lampOnS or tcBrake or ycBrake then
    dse.holdT = cfg.lampHoldS
  elseif dse.holdT > 0 then
    dse.holdT = max(0, dse.holdT - dt)
  end
  -- the second clause keeps the lamp correct with lampHoldS = 0
  local lampOn = dse.holdT > 0 or dse.lowT >= cfg.lampOnS or tcBrake or ycBrake
  ev.etk854hybrid_dscLamp = lampOn and 1 or 0
  -- raw stock telltale values before item 8 rewrites them (telemetry; the
  -- recorder runs after this module and sees the rewritten esc / tcs)
  ev.etk854hybrid_dseEscRaw = num(ev.esc, 0)
  ev.etk854hybrid_dseTcsRaw = num(ev.tcs, 0)
  if cfg.dseStockLampQualified then
    qualifyStockLamps(ev, dt, lampOn)
  end

  -- engaged gear (header item 7)
  local gbx = dse.gbx
  if gbx then
    ev.etk854hybrid_gearActual = num(gbx.gearIndex, 0)
  else
    ev.etk854hybrid_gearActual = GEAR_UNKNOWN
  end
end

-- See header item 9. Debounced shift recommendation for the page / HUD.
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
    -- a shift happened: the old recommendation is spent, debounce afresh
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
local function init(jbeamData)
  readConfig(jbeamData)
  st.consKwh = cfg.defaultConsumption
  st.needle = cfg.tachoOffRpm
  st.sampleTimer, st.sampleDist, st.lastEnergy = 0, 0, nil
  st.rangeKm = -1
  electrics.values.etk854hybrid_tachoRpm = st.needle
  dse.resolved, dse.gateDone, dse.retryT = false, false, 0
  dse.lowT, dse.holdT, dse.escT, dse.tcsT = 0, 0, 0, 0
  electrics.values.etk854hybrid_dscLamp = 0
  electrics.values.etk854hybrid_gearActual = GEAR_UNKNOWN
  hint.actual, hint.shown, hint.t = nil, 0, 0
  electrics.values.etk854hybrid_gearHint = 0
  -- on-board computer: a spawn starts a new trip
  obc.tank, obc.retryT, obc.lastVol, obc.flow = nil, 0, nil, 0
  obc.tripFuelL, obc.tripDistM, obc.tripId = 0, 0, 0
  obc.consFuel = cfg.defaultFuelConsumption
  obc.sampleT, obc.sampleDist, obc.sampleFuel = 0, 0, 0
  obc.fuelL, obc.fuelCapL = -1, 0
  obc.instL100, obc.instLh, obc.instMode, obc.instBar = -1, 0, "ev", 0
  publishFuel(electrics.values)
end

local function reset()
  -- keep the learned consumption; restart the sampling window
  st.sampleTimer, st.sampleDist, st.lastEnergy = 0, 0, nil
  st.needle = cfg.tachoOffRpm
  electrics.values.etk854hybrid_tachoRpm = st.needle
  -- the DSE wheel groups and the gate proxy survive an R-reset
  -- (tractionControl.reset only clears slip state, tractionControl.lua:124-133)
  dse.lowT, dse.holdT, dse.escT, dse.tcsT = 0, 0, 0, 0
  electrics.values.etk854hybrid_dscLamp = 0
  hint.actual, hint.shown, hint.t = nil, 0, 0
  electrics.values.etk854hybrid_gearHint = 0
  -- on-board computer: the trip survives R; the R refill of the tank must not
  -- count, so the volume is re-baselined (and the fill never counts anyway)
  obc.lastVol, obc.flow = nil, 0
  obc.sampleT, obc.sampleDist, obc.sampleFuel = 0, 0, 0
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
    m.fuelCapL = obc.fuelCapL
    m.fuelRangeKm = ev.etk854hybrid_fuelRangeKm
    m.instL100 = obc.instL100
    m.instLh = obc.instLh
    m.instMode = obc.instMode
    m.instBar = obc.instBar
    m.avgL100 = ev.etk854hybrid_fuelAvgL100
    m.tripKm = obc.tripDistM * 0.001
    m.avgKwh = st.consKwh
    m.tripId = obc.tripId
  end
  if st.wantRange then
    m.rangeKm = st.hasBattery and st.rangeKm or -1
    m.consKwh = st.consKwh
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

-- read-only snapshot for other controllers (no allocation: shared table)
local info = {rangeKm = -1, consKwh = 0, usableKwh = 0, capKwh = 0, iceKw = 0, hasBattery = false,
  fuelL = -1, fuelCapL = 0, fuelRangeKm = -1, instL100 = -1, instLh = 0, instMode = "ev", avgL100 = -1, tripKm = 0}
local function getEnergyInfo()
  info.rangeKm = st.hasBattery and st.rangeKm or -1
  info.consKwh = st.consKwh
  info.usableKwh = st.usableKwh
  info.capKwh = st.capKwh
  info.hasBattery = st.hasBattery
  -- on-board computer (header item 10), same values the cluster shows
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

-- Runtime change of a row option (tests / diagnosis, not persistent):
--   controller.getController("gauges/customModules/etk854hybrid_gaugeData")
--     .setOption("dseStockLampQualified", false)
-- Only existing keys of the same type. dseIceK0Gate false removes the gate
-- proxy at once, true re-installs it on the next frame. Returns the old
-- value, nil when refused.
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
M.reset = reset
M.setOption = setOption
M.updateGFX = updateGFX
M.setupGaugeData = setupGaugeData
M.updateGaugeData = updateGaugeData
M.getEnergyInfo = getEnergyInfo
M.resetTripComputer = resetTripComputer

return M
