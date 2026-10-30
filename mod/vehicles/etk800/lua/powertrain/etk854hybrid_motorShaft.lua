-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
--
-- Licence and credits for this file:
-- * Contains code derived from BeamNG.drive 0.39.4
--   lua/vehicle/powertrain/electricMotor.lua and combustionEngine.lua (bCDDL 1.1).
-- * The inline motor-on-a-shaft device concept and parts of its structure
--   (motorTorque(), motorOutputTorque / motorRegenTorque) are based on
--   motorShaft.lua by NiZhaZi (NZZ) from the
--   "E-CVT Hybrid Vivace" mod, https://github.com/NiZhaZi/Beamng_Hybrid_and_EV_Mod ,
--   used under the MIT License:
--   Copyright (c) 2025 Andrew Zuo
--
--   MIT License
--
--   Permission is hereby granted, free of charge, to any person obtaining a copy
--   of this software and associated documentation files (the "Software"), to deal
--   in the Software without restriction, including without limitation the rights
--   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
--   copies of the Software, and to permit persons to whom the Software is
--   furnished to do so, subject to the following conditions:
--
--   The above copyright notice and this permission notice shall be included in all
--   copies or substantial portions of the Software.
--
--   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
--   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
--   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
--   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
--   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
--   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
--   SOFTWARE.
-- * Modifications and everything else in this file: Deuflhard (ETK 854 ActiveHybrid).
--
-- etk854hybrid_motorShaft.lua
-- Inline P2 E-machine (jbeam device "integratedMotor"):
--   mainEngine -> clutchK0 (parent) -> THIS SHAFT -> torqueConverter (child)
-- i.e. this shaft is the converter pump. The stock electricMotor is a terminal
-- device and crashes the ICE init when chained inline; this device passes the
-- K0 torque through, adds its own, propagates inertia / gear ratio and stubs
-- the ICE starter interface.
--
-- Per physics tick (motorTorque):
--   drive = torque curve x smoothed etk854hybrid_evThrottle, or
--   regen = regen curve x etk854hybrid_evRegen; both trimmed by the DSE throttleFactor
--   + EV creep floor (creepFloor), armed by the VCU via etk854hybrid_creep*
--   + fill = fillGain * (etk854hybrid_iceExpNm - measured K0 / ICE torque), so
--     K0 crank / engage torque and turbo lag never reach the shaft
-- updateTorque integrates the shaft AV: K0 + motor - converter load - friction.
-- Writes per frame: etk854hybrid_creepNm / _creepSetRpm / _creepAuth / _pumpRpm /
--   _tcPumpNm / _motorSoundVol / _motorSoundActive.
-- Device fields read by the VCU: motorTorque, etkFillNm, etkShaftNm,
--   etkShaftRateMax, etkCreepNm. Torque > 0 drives in engine direction.

local M = {}

M.outputPorts = {[1] = true}
-- inline device: must pass both neighbour validators (combustionEngine wants a
-- clutchlike child, torqueConverter an engine parent), else the ICE init crashes
M.deviceCategories = {shaft = true, electricMotor = true, engine = true, clutchlike = true}
M.requiredExternalInertiaOutputs = {1}

local max = math.max
local min = math.min
local abs = math.abs
local floor = math.floor
local clamp = clamp
local sign = sign

local rpmToAV = 0.104719755
local avToRPM = 9.549296596425384
local torqueToPower = 0.0001404345295653085

-- ===========================================================
-- TorqueData (for in-game torque-curve UI)
-- ===========================================================
local function getTorqueData(device)
  local curves = {}
  local maxTorque = 0
  local maxTorqueRPM = 0
  local maxPower = 0
  local maxPowerRPM = 0
  local maxRPM = 0

  local torqueCurve = {}
  local powerCurve = {}

  for k, v in pairs(device.torqueCurve) do
    if type(k) == "number" then
      torqueCurve[k + 1] = v - device.friction - (device.dynamicFriction * k * rpmToAV)
      powerCurve[k + 1] = torqueCurve[k + 1] * k * torqueToPower
      if torqueCurve[k + 1] > maxTorque then
        maxTorque = torqueCurve[k + 1]
        maxTorqueRPM = k + 1
      end
      if powerCurve[k + 1] > maxPower then
        maxPower = powerCurve[k + 1]
        maxPowerRPM = k + 1
      end
      maxRPM = max(maxRPM, k)
    end
  end

  table.insert(curves, 1, { torque = torqueCurve, power = powerCurve, name = "Electric", priority = 10 })

  local dashes = { nil, { 10, 4 }, { 8, 3, 4, 3 }, { 6, 3, 2, 3 }, { 5, 3 } }
  for k, v in ipairs(curves) do
    v.dash = dashes[k]
    v.width = 2
  end

  return {
    maxRPM = maxRPM, curves = curves,
    maxTorque = maxTorque, maxPower = maxPower,
    maxTorqueRPM = maxTorqueRPM, maxPowerRPM = maxPowerRPM,
    finalCurveName = 1, deviceName = device.name,
    vehicleID = obj:getId()
  }
end

local function sendTorqueData(device, data)
  if not data then data = device:getTorqueData() end
  guihooks.trigger("TorqueCurveChanged", data)
end

-- ===========================================================
-- Energy / Storage tracking
-- ===========================================================
local function updateEnergyStorageRatios(device)
  device.energyStorageRatios = {}
  device.energyStorageRegenRatios = {}
  for _, s in pairs(device.registeredEnergyStorages) do
    local storage = energyStorage.getStorage(s)
    if storage then
      device.energyStorageRatios[storage.name] = 1 / max(1, device.storageWithEnergyCounter)
      device.energyStorageRegenRatios[storage.name] = 1 / max(1, device.storageCounter)
    end
  end
end

local function updateEnergyUsage(device)
  if not device.energyStorage then return end

  local hasEnergy = false
  local previousStorageCount = device.storageWithEnergyCounter
  for _, s in pairs(device.registeredEnergyStorages) do
    local storage = energyStorage.getStorage(s)
    if storage then
      local previous = device.previousEnergyLevels[storage.name] or 0
      local storageRatio = device.spentEnergy > 0
        and (device.energyStorageRatios[storage.name] or 1)
        or (device.energyStorageRegenRatios[storage.name] or 1)
      storage.storedEnergy = clamp(storage.storedEnergy - (device.spentEnergy * storageRatio), 0, storage.energyCapacity)
      if previous > 0 and storage.storedEnergy <= 0 then
        device.storageWithEnergyCounter = device.storageWithEnergyCounter - 1
      elseif previous <= 0 and storage.storedEnergy > 0 then
        device.storageWithEnergyCounter = device.storageWithEnergyCounter + 1
      end
      device.previousEnergyLevels[storage.name] = storage.storedEnergy
      hasEnergy = hasEnergy or storage.storedEnergy > 0
    end
  end

  if previousStorageCount ~= device.storageWithEnergyCounter then
    device:updateEnergyStorageRatios()
  end
  device.spentEnergy = 0

  if not hasEnergy and device.hasEnergy then
    device.outputTorqueState = 0
    device.isDisabled = true
  elseif hasEnergy and not device.hasEnergy then
    device.outputTorqueState = 1
    device.isDisabled = false
  end

  device.hasEnergy = hasEnergy
end

local function registerStorage(device, storageName)
  local storage = energyStorage.getStorage(storageName)
  -- register by capacity, not stored energy: a pack empty at spawn must still
  -- register, or the E-machine stays dead after a recharge
  if storage and storage.type == "electricBattery" and (storage.energyCapacity or 0) > 0 then
    device.storageWithEnergyCounter = device.storageWithEnergyCounter + 1
    device.storageCounter = device.storageCounter + 1
    table.insert(device.registeredEnergyStorages, storageName)
    device:updateEnergyStorageRatios()
    device.hasEnergy = true
    device.previousEnergyLevels[storageName] = storage.storedEnergy
  end
end

-- ===========================================================
-- Direction logic
-- ===========================================================
local function computeMotorDirection(device)
  -- "forward": always +1 (P2: the gearbox reverse ratio does the reversing).
  -- "auto": follow the gear; only right for a motor driving an axle directly.
  if device.directionMode == "forward" then
    return 1
  end

  -- gearIndex 0 = N or P: P keeps +1 (the input shaft still turns with the
  -- ICE, regen must work), N gives 0
  local gearIndex = electrics.values.gearIndex
  local gearStr = electrics.values.gear
  if type(gearIndex) == "number" then
    if gearIndex > 0 then return 1
    elseif gearIndex < 0 then return -1
    elseif gearStr == "P" then return 1
    else return 0 end
  end

  if type(gearStr) == "string" then
    if gearStr == "R" then return -1
    elseif gearStr == "N" then return 0
    else return 1 end  -- P / D / S / M
  end

  -- neither field set yet (early init)
  return 1
end

-- ===========================================================
-- EV creep / virtual idle
-- ===========================================================
-- Speed FLOOR on the converter pump (= this shaft), like the stock idle
-- governor on an ICE: motor torque = max(pedal-or-regen torque, PI floor);
-- creep then comes out of the stock converter physics.
-- Armed by the VCU (etk854hybrid_creep.lua):
--   etk854hybrid_creepTargetRpm  setpoint [rpm]
--   etk854hybrid_creepMaxNm      authority limit [Nm]; 0 = disarm (fades out)
--   etk854hybrid_creepCut        1 = hard cut
-- The authority also fades out with the effective K0 torque k0 * (1 - fillGain)
-- and the lock-up clutch (behind a locked converter it would be a cruise control).
-- PI with conditional integration, I in [0, limit]. Two-sided: just above the
-- setpoint the P term lifts a regen torque towards 0, a continuous handover
-- (a one-sided floor chatters). No low-AV taper (full authority at 0 rpm).
-- Returns the new base torque and the torque the floor added (>= 0).
local function creepFloor(device, dt, engineAV, baseNm, throttleFactor, ev)
  if not device.creepOn then return baseNm, 0 end
  local tgt = ev.etk854hybrid_creepTargetRpm or 0
  local limCmd = ev.etk854hybrid_creepMaxNm or 0
  local cut = (ev.etk854hybrid_creepCut or 0) > 0.5
  local k0Eff = (ev.etk854hybrid_k0ClutchRatio or 0) * (1 - clamp(ev.etk854hybrid_fillGain or 0, 0, 1))
  local gK0 = clamp((device.creepK0Hi - k0Eff) / device.creepK0Span, 0, 1)
  local gLu = clamp((device.creepLuHi - (ev.lockupClutchRatio or 0)) / device.creepLuSpan, 0, 1)
  local authT = (limCmd > 0 and not cut) and min(gK0, gLu) or 0
  local a = device.creepAuth
  local wasOff = a <= 0
  if cut then a = 0
  elseif authT > a then a = min(authT, a + device.creepAuthRise * dt)
  else a = max(authT, a - device.creepAuthFall * dt) end
  -- keep the last armed limit: a disarm fades out on it
  if limCmd > 0 then device.creepLimHold = limCmd end
  local limBase = min(device.creepLimHold or 0, device.creepMaxNm) * throttleFactor * device.outputTorqueState
  if limBase < 0 then limBase = 0 end
  local rpm = engineAV * avToRPM
  if a <= 0 or engineAV ~= engineAV or abs(engineAV) > device.maxAV then
    device.creepAuth = 0
    device.creepSet = clamp(rpm, 0, max(tgt, 0))     -- track the shaft: bumpless re-arm
    device.creepI = 0
    device.creepLimHold = 0
    device.etkCreepNm = 0
    return baseNm, 0
  end
  if wasOff then                                      -- bumpless start from the real state
    local tc = device.children and device.children[1]
    device.creepSet = clamp(rpm, 0, max(tgt, 0))
    local seed = clamp(((tc and tc.torqueDiff) or 0) + device.friction, 0, limBase)
    device.creepI = seed
    -- start the authority where the seed (current converter load) fits under
    -- the limit, the rest ramps at creepAuthRise
    if limBase > 0 then a = max(a, min(authT, seed / limBase)) end
  end
  device.creepAuth = a
  local set = device.creepSet
  if tgt > set then
    set = min(tgt, max(set, min(rpm, tgt)) + device.creepRampUp * dt)  -- never ramp below a free shaft
  else
    set = max(tgt, set - device.creepRampDown * dt)
  end
  device.creepSet = set
  local e = set * rpmToAV - engineAV
  local lim = limBase * a
  local kp, i = device.creepKp, device.creepI
  if not (kp * e + i >= lim and e > 0) then i = i + device.creepKi * e * dt end  -- conditional integration
  i = clamp(i, 0, lim)
  device.creepI = i
  local floorNm = min(kp * e + i, lim)                -- two-sided, see above
  local add = 0
  if floorNm > baseNm then add = floorNm - baseNm; baseNm = floorNm end  -- max(): no double counting
  device.etkCreepNm = add
  device.etkCreepAcc = device.etkCreepAcc + add * dt
  device.etkCreepAccT = device.etkCreepAccT + dt
  return baseNm, add
end

-- ===========================================================
-- Motor torque computation
-- ===========================================================
local function motorTorque(device, dt)
  local engineAV = device.outputAV1
  local ev = electrics.values
  -- DSE trim (drivingDynamics writes device.throttleFactor every step); the
  -- min with the electrics factor, so neither can cancel the other
  local throttleFactor = min(ev[device.electricsThrottleFactorName] or 1,
                             device.throttleFactor or 1)
  local rawThrottle = (ev[device.electricsThrottleName] or 0) * throttleFactor
  local throttle = device.throttleSmoother:getUncapped(clamp(rawThrottle, 0, 1), dt)
  device.throttle = throttle

  local motorDirection = device.motorDirection
  local torqueCurve = device.torqueCurve
  local friction = device.friction
  local dynamicFriction = device.dynamicFriction
  local rpm = engineAV * avToRPM * motorDirection
  local torqueRPM = floor(rpm)

  local torque = (torqueCurve[torqueRPM] or (torqueRPM < 0 and torqueCurve[0] or 0))
    * device.outputTorqueState
  torque = torque * throttle * motorDirection
  torque = min(torque, device.maxTorqueLimit)

  -- low-AV taper 0.30 -> 1.0: avoids instability and ICE overrev when the K0
  -- closes against an idling ICE with the motor at peak torque
  local LOW_AV_RAD = 8.0  -- ~76 RPM
  local LOW_AV_FLOOR = 0.30
  local absAV = engineAV >= 0 and engineAV or -engineAV
  if absAV < LOW_AV_RAD then
    local taper = LOW_AV_FLOOR + (1.0 - LOW_AV_FLOOR) * (absAV / LOW_AV_RAD)
    torque = torque * taper
  end

  -- Regen
  local regenThrottle = ev[device.electricsRegenThrottleName] or 0
  local absRPM = abs(torqueRPM)
  local rawRegenTorque = (device.regenCurve[absRPM] or 0)
  local regenTorque = -(min(max(rawRegenTorque * regenThrottle, min(rawRegenTorque, device.minWantedRegenTorque)),
    device.maxWantedRegenTorque) * sign(regenThrottle) * throttleFactor * motorDirection)
  device.regenThrottle = regenThrottle
  device.instantMaxRegenTorque = rawRegenTorque

  local actualTorque = throttle > 0 and torque or regenTorque
  -- creep floor before the fill: the fill acts on top of the floor
  local creepNm
  actualTorque, creepNm = creepFloor(device, dt, engineAV, actualTorque, throttleFactor, ev)
  local driveNm = actualTorque

  -- Fill loop: the VCU publishes the expected combustion-side torque
  -- (etk854hybrid_iceExpNm) and the gain; the motor makes up the difference to
  -- what K0 / ICE deliver this tick. Measurement: K0 torque while slipping
  -- (cancels crank / engage torque), ICE combustionTorque when locked (fills
  -- turbo lag; no inertia term, so no algebraic loop).
  -- A gain <= 1 cancellation, not a feedback loop on slip.
  local fillNm = 0
  local fillGain = ev.etk854hybrid_fillGain or 0
  if fillGain > 0 then
    local k0 = device.parent
    local ice = k0 and k0.parent
    local k0T = (k0 and k0.torqueDiff) or 0
    local comb = (ice and ice.combustionTorque) or k0T
    -- locked = the ICE sequencer's flag; the slip test is only a fallback (it
    -- flickers under the K0 spring ringing and feeds it back into the fill)
    local lockCh = ev.etk854hybrid_fillLocked
    local lockT
    if lockCh ~= nil then
      lockT = (lockCh >= 0.5) and 1 or 0
    else
      local slipAV = ((ice and ice.outputAV1) or engineAV) - engineAV
      lockT = ((ev.etk854hybrid_k0ClutchRatio or 0) >= 0.9 and abs(slipAV) < 5.2) and 1 or 0
    end
    -- 0.1 s crossfade, no step at lock
    local w = device.etkLockW or 0
    w = w + clamp(lockT - w, -10 * dt, 10 * dt)
    device.etkLockW = w
    local meas = w * comb + (1 - w) * k0T
    -- 4 ms LPF against the frictionClutch spring ringing
    local lpf = device.etkIceMeasLpf or meas
    lpf = lpf + (meas - lpf) * min(1, dt / 0.004)
    device.etkIceMeasLpf = lpf
    fillNm = clamp(fillGain * ((ev.etk854hybrid_iceExpNm or 0) - lpf),
                   -(ev.etk854hybrid_fillNegMaxNm or 0), ev.etk854hybrid_fillPosMaxNm or 0)
    if fillNm > 0 then fillNm = fillNm * throttleFactor end   -- DSE trim still wins
    local capT = (torqueCurve[absRPM] or 0) * device.outputTorqueState * throttleFactor
    local capR = rawRegenTorque
    actualTorque = clamp(actualTorque + fillNm * motorDirection, -capR, capT)
  else
    device.etkIceMeasLpf = nil
    device.etkLockW = nil
  end
  device.etkFillNm = fillNm

  -- [HOLD_DBG] regen diagnostic, max 1 line/s (jbeam "debugMotor": true)
  if device.debugMotor then
    if device._dbgRegenLastT == nil then device._dbgRegenLastT = 0 end
    device._dbgRegenLastT = device._dbgRegenLastT + dt
    if device._dbgRegenLastT >= 1.0 then
      device._dbgRegenLastT = 0
      if regenThrottle > 0.01 then
        log("I", "etk854hybrid.motor", string.format(
          "[HOLD_DBG] MOTOR regen: dir=%d rpm=%d regenCmd=%.3f rawRegenTq=%.1f regenTq=%.2f thr=%.3f AV=%.2f",
          motorDirection, torqueRPM, regenThrottle, rawRegenTorque, regenTorque, throttle, engineAV))
      end
    end
  end

  -- creep floor torque counts as drive torque
  device.motorOutputTorque = (creepNm > 0) and max(torque, driveNm) or torque
  device.motorRegenTorque = regenTorque

  local maxCurrentTorque = (torqueCurve[absRPM] or torqueCurve[0]) - friction
    - (dynamicFriction * abs(rpm) * rpmToAV)
  local instantEngineLoad = clamp(actualTorque / (maxCurrentTorque + 1e-30), -1, 1)
  device.instantEngineLoad = instantEngineLoad
  device.engineLoad = device.loadSmoother:getCapped(instantEngineLoad, dt)

  local dtT = dt * actualTorque
  local grossWork = dtT * engineAV
  device.grossWorkPerUpdate = device.grossWorkPerUpdate + grossWork
  device.spentEnergy = device.spentEnergy
    + grossWork / device.electricalEfficiencyTable[floor(abs(device.engineLoad) * 100) * 0.01]
  device.frictionLossPerUpdate = device.frictionLossPerUpdate
    + dt * engineAV * (friction + dynamicFriction * engineAV)

  local avSign = sign(engineAV)
  local frictionTorque = abs(friction * avSign + dynamicFriction * engineAV)
  frictionTorque = min(frictionTorque, abs(engineAV) * device.inertia * 2000) * avSign

  local outT = actualTorque - frictionTorque
  device.motorTorque = outT
  return outT
end

-- ===========================================================
-- Physics update (per fixed-step tick). Self-integrating shaft: updateVelocity
-- mirrors the AV into the K0 output so the K0 computes its slip torque. The
-- ICE AV is never written (that overrides its integration and stalls it).
-- ===========================================================
local function updateVelocity(device, dt)
  device.inputAV = device[device.outputAVName] * device.gearRatio
  device.parent[device.parentOutputAVName] = device.inputAV
end

local function updateTorque(device, dt)
  local engineAV = device.outputAV1
  local motorT = motorTorque(device, dt)

  local k0Torque = device.parent[device.parentOutputTorqueName] or 0

  -- converter load from the last tick
  local child = device.children and device.children[1]
  local downstreamLoad = (child and child.torqueDiff) or 0

  local frictionLoad = (device.friction * clamp(engineAV, -1, 1) + device.dynamicFriction * engineAV)
    * device.wearFrictionCoef * device.damageFrictionCoef

  local netT = k0Torque + motorT - downstreamLoad - frictionLoad

  -- ICE start stats: shaft input torque (30 ms LPF) and its peak rate
  local shIn = k0Torque + motorT
  local shPrev = device.etkShaftNm or shIn
  local shNew = shPrev + (shIn - shPrev) * min(1, dt / 0.03)
  device.etkShaftNm = shNew
  local shRate = abs(shNew - shPrev) / dt
  if shRate > (device.etkShaftRateMax or 0) then device.etkShaftRateMax = shRate end

  local newAV = engineAV + dt * netT / device.inertia
  device.outputAV1 = max(-device.maxAV, min(device.maxAV, newAV))

  -- informational only: the converter computes its own input torque
  device[device.outputTorqueName] = k0Torque + motorT - frictionLoad

  -- back-torque seen by the K0
  device.torqueDiff = downstreamLoad + frictionLoad - motorT
end

-- disconnected (broken or mode): the motor turns its own virtual mass
local function disconnectedUpdateVelocity(device, dt)
  device.inputAV = device.virtualMassAV
  device[device.outputAVName] = device.virtualMassAV / device.gearRatio
end

local function disconnectedUpdateTorque(device, dt)
  local motorT = motorTorque(device, dt)
  device.virtualMassAV = device.virtualMassAV + motorT * device.invCumulativeInertia * dt
  device[device.outputTorqueName] = motorT
  device.torqueDiff = 0
end

local function selectUpdates(device)
  device.velocityUpdate = updateVelocity
  device.torqueUpdate = updateTorque

  if device.isBroken or device.mode == "disconnected" then
    device.velocityUpdate = disconnectedUpdateVelocity
    device.torqueUpdate = disconnectedUpdateTorque
    device.virtualMassAV = device.inputAV
  end
end

-- ===========================================================
-- updateGFX (per frame, NOT per physics tick)
-- ===========================================================
local function updateGFX(device, dt)
  device.motorDirection = computeMotorDirection(device)

  -- torqueDiff must never be nil (a parent reading nil crashes)
  if device.mode == "disconnected" then
    device.torqueDiff = 0
  else
    device.torqueDiff = device[device.outputTorqueName] or 0
  end

  device:updateEnergyUsage()

  device.outputRPM = device.outputAV1 * avToRPM

  -- telemetry: creepNm = mean floor torque over this frame [Nm], creepSetRpm,
  -- creepAuth 0..1, pumpRpm (signed), tcPumpNm = converter pump load [Nm]
  local ev = electrics.values
  local accT = device.etkCreepAccT or 0
  ev.etk854hybrid_creepNm = (accT > 0) and ((device.etkCreepAcc or 0) / accT) or 0
  device.etkCreepAcc, device.etkCreepAccT = 0, 0
  ev.etk854hybrid_creepSetRpm = device.creepSet or 0
  ev.etk854hybrid_creepAuth = device.creepAuth or 0
  ev.etk854hybrid_pumpRpm = device.outputRPM
  local tc = device.children and device.children[1]
  ev.etk854hybrid_tcPumpNm = (tc and tc.torqueDiff) or 0

  device.grossWorkPerUpdate = 0
  device.frictionLossPerUpdate = 0
end

-- ===========================================================
-- E-machine sound: stock electricMotor sound hooks (powertrain.lua calls them
-- on any device that has them). The shaft also turns with the ICE, so the
-- volume follows the electrical activity (drive / regen) over an idle floor.
-- ===========================================================
local function getSoundConfiguration(device)
  return device.soundConfiguration
end

local function updateSounds(device, dt)
  local motorAV = device.outputAV1 or 0
  local rpmRaw = abs(motorAV) * avToRPM
  local rpm = device.soundRPMSmoother:get(rpmRaw, dt)

  -- load magnitude: regen drives the same on-load layer as traction
  local rawLoad = abs(device.instantEngineLoad or 0)
  local engineLoad = clamp(device.soundLoadSmoother:get(rawLoad, dt),
    device.soundMinLoadMix, device.soundMaxLoadMix)

  local driveT = abs(device.motorOutputTorque or 0)
  local regenT = abs(device.motorRegenTorque or 0) * device.soundRegenVolumeCoef
  local activity = clamp(max(driveT, regenT) * device.invSoundReferenceTorque, 0, 1)
  local targetVolume = device.soundIdleVolume + (1 - device.soundIdleVolume) * activity

  -- fade out towards standstill (parked car silent)
  if rpmRaw < device.soundFadeInRPM then
    targetVolume = targetVolume * (rpmRaw * device.invSoundFadeInRPM)
  end
  if device.isDisabled or not device.hasEnergy then
    targetVolume = 0
  end

  local volume = device.soundVolumeSmoother:get(clamp(targetVolume, 0, 1), dt)
  device.engineVolumeCoef = volume

  local fundamentalFreq = sounds.hzToFMODHz(rpm * device.fundamentalFrequencyRPMCoef)
  obj:setEngineSound(device.engineSoundID, rpm, engineLoad, fundamentalFreq, volume)

  -- diagnostic channels (QA + HUD)
  if electrics and electrics.values then
    electrics.values.etk854hybrid_motorSoundVol = volume
    electrics.values.etk854hybrid_motorSoundActive =
      (volume > device.soundActiveVolumeThreshold) and 1 or 0
  end
end

local function initEngineSound(device, soundID, samplePath, engineNodeIDs, offLoadGain, onLoadGain, reference)
  device.soundConfiguration[reference] = device.soundConfiguration[reference] or {}
  device.soundConfiguration[reference].blendFile = samplePath
  obj:queueGameEngineLua(string.format("core_sounds.initEngineSound(%d,%d,%q,%s,%f,%f)",
    objectId, soundID, samplePath, serialize(engineNodeIDs), offLoadGain, onLoadGain))

  if bdebug and bdebug.setNodeDebugText and engineNodeIDs[1] then
    bdebug.setNodeDebugText("Powertrain", engineNodeIDs[1], device.name .. ": " .. samplePath)
  end
end

local function setEngineSoundParameterList(device, soundID, params, reference)
  device.soundConfiguration[reference] = device.soundConfiguration[reference] or {}
  device.soundConfiguration[reference].params =
    tableMergeRecursive(device.soundConfiguration[reference].params or {}, params)
  device.soundConfiguration[reference].soundID = soundID
  obj:queueGameEngineLua(string.format("core_sounds.setEngineSoundParameterList(%d,%d,%s)",
    objectId, soundID, serialize(params)))
end

-- published even without sound: QA tells "sound broken" (0) from "no device" (nil)
local function publishSilentSoundChannels()
  if electrics and electrics.values then
    electrics.values.etk854hybrid_motorSoundVol = 0
    electrics.values.etk854hybrid_motorSoundActive = 0
  end
end

local function initSounds(device, jbeamData)
  publishSilentSoundChannels()

  if sounds.usesOldCustomSounds then
    log("W", "etk854hybrid_motorShaft.initSounds",
      "Old custom engine sounds present - skipping E-motor sound for " .. tostring(device.name))
    return
  end
  if not jbeamData.soundConfig then
    log("W", "etk854hybrid_motorShaft.initSounds",
      "No soundConfig on " .. tostring(device.name) .. " - E-motor stays silent")
    return
  end

  local soundConfig = v.data[jbeamData.soundConfig]
  if not soundConfig then
    log("E", "etk854hybrid_motorShaft.initSounds",
      "Can't find sound config: " .. tostring(jbeamData.soundConfig))
    return
  end

  device.soundConfiguration = {}
  device.engineSoundID = powertrain.getEngineSoundID()

  device.soundRPMSmoother = newTemporalSmoothingNonLinear(
    soundConfig.rpmSmootherInRate or 15, soundConfig.rpmSmootherOutRate or 25)
  device.soundLoadSmoother = newTemporalSmoothingNonLinear(
    soundConfig.loadSmootherInRate or 20, soundConfig.loadSmootherOutRate or 20)
  -- inRate = fall, outRate = rise: fast attack, slow release
  device.soundVolumeSmoother = newTemporalSmoothingNonLinear(
    soundConfig.volumeSmootherInRate or 3, soundConfig.volumeSmootherOutRate or 6)

  device.soundMaxLoadMix = soundConfig.maxLoadMix or 1
  device.soundMinLoadMix = soundConfig.minLoadMix or 0
  local fundamentalFrequencyCylinderCount = soundConfig.fundamentalFrequencyCylinderCount or 6
  device.fundamentalFrequencyRPMCoef = fundamentalFrequencyCylinderCount / 120
  device.engineVolumeCoef = 0

  -- extensions to the stock soundConfig schema
  device.soundIdleVolume = clamp(soundConfig.idleVolume or 0.18, 0, 1)
  local refTorque = soundConfig.referenceTorque or device.maxTorque or 280
  if refTorque <= 0 then refTorque = 280 end
  device.invSoundReferenceTorque = 1 / refTorque
  device.soundRegenVolumeCoef = soundConfig.regenVolumeCoef or 0.85
  device.soundFadeInRPM = max(1, soundConfig.volumeFadeInRPM or 60)
  device.invSoundFadeInRPM = 1 / device.soundFadeInRPM
  device.soundActiveVolumeThreshold = soundConfig.activeVolumeThreshold or 0.02

  local onLoadGain = soundConfig.onLoadGain or 1
  local offLoadGain = soundConfig.offLoadGain or 1

  local sampleName = soundConfig.sampleName
  if not sampleName then
    log("E", "etk854hybrid_motorShaft.initSounds",
      "soundConfig '" .. tostring(jbeamData.soundConfig) .. "' has no sampleName - E-motor stays silent")
    return
  end

  local sampleFolder = soundConfig.sampleFolder or "art/sound/blends/"
  local samplePath = sampleFolder .. sampleName .. ".sfxBlend2D.json"
  device:initEngineSound(device.engineSoundID, samplePath, {device.engineNodeID},
    offLoadGain, onLoadGain, "motor")

  local params = {
    main_gain = soundConfig.mainGain or 0,
    eq_a_freq = sounds.hzToFMODHz(soundConfig.lowCutFreq or 20),
    eq_b_freq = sounds.hzToFMODHz(soundConfig.highCutFreq or 10000),
    eq_c_freq = sounds.hzToFMODHz(soundConfig.eqLowFreq or 500),
    eq_c_gain = soundConfig.eqLowGain or 0,
    eq_c_reso = soundConfig.eqLowWidth or 0,
    eq_d_freq = sounds.hzToFMODHz(soundConfig.eqHighFreq or 2000),
    eq_d_gain = soundConfig.eqHighGain or 0,
    eq_d_reso = soundConfig.eqHighWidth or 0,
    eq_e_gain = soundConfig.eqFundamentalGain or 0,
    eq_e_reso = soundConfig.eqFundamentalWidth or 1,
    onLoadGain = onLoadGain,
    offLoadGain = offLoadGain,
    muffled = 0.5
  }
  device:setEngineSoundParameterList(device.engineSoundID, params, "motor")

  device.updateSounds = updateSounds
  sounds.disableOldEngineSounds()

  log("I", "etk854hybrid_motorShaft.initSounds", string.format(
    "%s: blend=%s node=%s mainGain=%.1f idleVol=%.2f refTq=%.0fNm fadeInRPM=%.0f",
    tostring(device.name), samplePath, tostring(device.engineNodeID),
    params.main_gain, device.soundIdleVolume, refTorque, device.soundFadeInRPM))
end

local function resetSounds(device, jbeamData)
  publishSilentSoundChannels()
  if sounds.usesOldCustomSounds then return end
  if not jbeamData.soundConfig then return end
  if not v.data[jbeamData.soundConfig] then
    log("E", "etk854hybrid_motorShaft.resetSounds",
      "Can't find sound config: " .. tostring(jbeamData.soundConfig))
    return
  end
  if device.soundRPMSmoother then device.soundRPMSmoother:reset() end
  if device.soundLoadSmoother then device.soundLoadSmoother:reset() end
  if device.soundVolumeSmoother then device.soundVolumeSmoother:reset() end
  device.engineVolumeCoef = 0
  sounds.disableOldEngineSounds()
end

-- ===========================================================
-- Inertia & gear-ratio propagation (read by the ICE init through this device)
-- ===========================================================
local function calculateInertia(device)
  local outputInertia
  local cumulativeGearRatio = 1
  local maxCumulativeGearRatio = 1
  if device.children and #device.children > 0 then
    local child = device.children[1]
    outputInertia = child.cumulativeInertia
    cumulativeGearRatio = child.cumulativeGearRatio
    maxCumulativeGearRatio = child.maxCumulativeGearRatio
  else
    outputInertia = device.virtualInertia
  end

  device.cumulativeInertia = outputInertia / device.gearRatio / device.gearRatio
  device.invCumulativeInertia = device.cumulativeInertia > 0 and 1 / device.cumulativeInertia or 0
  device.cumulativeGearRatio = cumulativeGearRatio * device.gearRatio
  device.maxCumulativeGearRatio = maxCumulativeGearRatio * device.gearRatio
end

-- ===========================================================
-- Damage / break / part condition
-- ===========================================================
local function applyDeformGroupDamage(device, damageAmount)
  device.damageFrictionCoef = device.damageFrictionCoef + linearScale(damageAmount, 0, 0.01, 0, 0.1)
end

local function setPartCondition(device, subSystem, odometer, integrity, visual)
  device.wearFrictionCoef = linearScale(odometer, 30000000, 1000000000, 1, 1.5)
  local integrityState = integrity
  if type(integrity) == "number" then
    integrityState = { damageFrictionCoef = linearScale(integrity, 1, 0, 1, 50), isBroken = false }
  end
  device.damageFrictionCoef = integrityState.damageFrictionCoef or 1
  if integrityState.isBroken then device:onBreak() end
end

local function getPartCondition(device)
  local integrityState = { damageFrictionCoef = device.damageFrictionCoef, isBroken = device.isBroken }
  local integrityValue = linearScale(device.damageFrictionCoef, 1, 50, 1, 0)
  if device.isBroken then integrityValue = 0 end
  return integrityValue, integrityState
end

local function onBreak(device)
  device.isBroken = true
  selectUpdates(device)
end

local function setMode(device, mode)
  local isValid = false
  for _, m in ipairs(device.availableModes) do
    if mode == m then isValid = true end
  end
  if not isValid then return end
  device.mode = mode
  if device.isPhysicallyDisconnected then device.mode = "disconnected" end
  selectUpdates(device)
end

local function validate(device)
  if device.isPhysicallyDisconnected then
    device.mode = "disconnected"
    selectUpdates(device)
  end
  return true
end

local function updateSimpleControlButtons(device) end

-- ===========================================================
-- Reset
-- ===========================================================
local function reset(device, jbeamData)
  device.gearRatio = jbeamData.gearRatio or 1
  device.friction = jbeamData.friction or 0
  device.dynamicFriction = jbeamData.dynamicFriction or 0
  device.cumulativeInertia = 1
  device.invCumulativeInertia = 1
  device.cumulativeGearRatio = 1
  device.maxCumulativeGearRatio = 1

  device.inputAV = 0
  device.outputAV1 = 0
  device.outputTorque1 = 0
  device.virtualMassAV = 0
  device.visualShaftAngle = 0

  device.isBroken = false
  device.wearFrictionCoef = 1
  device.damageFrictionCoef = 1

  device.motorTorque = 0
  device.motorOutputTorque = 0
  device.motorRegenTorque = 0
  device.outputRPM = 0
  device.motorDirection = 1

  -- fill loop / diagnostics state
  device.etkFillNm = 0
  device.etkShaftNm = 0
  device.etkShaftRateMax = 0
  device.etkLockW = nil
  device.etkIceMeasLpf = nil

  -- EV creep floor state (parameters stay as built by new())
  device.creepAuth, device.creepSet, device.creepI, device.creepLimHold = 0, 0, 0, 0
  device.etkCreepNm, device.etkCreepAcc, device.etkCreepAccT = 0, 0, 0

  device.throttle = 0
  device.throttleFactor = 1
  device.engineLoad = 0
  device.instantEngineLoad = 0
  device.outputTorqueState = 1
  device.outputAVState = 1
  device.isDisabled = false
  device.frictionLossPerUpdate = 0
  device.grossWorkPerUpdate = 0
  device.spentEnergy = 0
  device.storageWithEnergyCounter = 0
  device.storageCounter = 0
  device.registeredEnergyStorages = {}
  device.previousEnergyLevels = {}
  device.energyStorageRatios = {}
  device.energyStorageRegenRatios = {}
  device.hasEnergy = true

  if device.loadSmoother and device.loadSmoother.reset then device.loadSmoother:reset() end
  if device.throttleSmoother and device.throttleSmoother.reset then device.throttleSmoother:reset() end

  selectUpdates(device)
  return device
end

-- ===========================================================
-- Constructor
-- ===========================================================
local function new(jbeamData)
  local directionMode = jbeamData.directionMode or "auto"
  local k0Hi = jbeamData.creepK0FadeHi or 0.30
  local luHi = jbeamData.creepLockupFadeHi or 0.50
  local device = {
    deviceCategories = shallowcopy(M.deviceCategories),
    outputPorts = shallowcopy(M.outputPorts),
    name = jbeamData.name,
    type = jbeamData.type,
    inputName = jbeamData.inputName,
    inputIndex = jbeamData.inputIndex,
    gearRatio = jbeamData.gearRatio or 1,
    friction = jbeamData.friction or 0,
    dynamicFriction = jbeamData.dynamicFriction or 0,
    wearFrictionCoef = 1,
    damageFrictionCoef = 1,
    cumulativeInertia = 1,
    invCumulativeInertia = 1,
    virtualInertia = 2,
    cumulativeGearRatio = 1,
    maxCumulativeGearRatio = 1,
    -- never physically disconnected: validate() keeps the "connected" mode
    isPhysicallyDisconnected = false,
    inputAV = 0,
    outputAV1 = 0,
    outputTorque1 = 0,
    virtualMassAV = 0,
    visualShaftAngle = 0,
    isBroken = false,

    -- Motor state
    motorTorque = 0,
    motorOutputTorque = 0,
    motorRegenTorque = 0,
    outputRPM = 0,
    motorDirection = 1,
    maxTorqueLimit = math.huge,
    isPropulsed = true,
    torqueDiff = 0,  -- read by the parent, never nil
    -- summed into the ICE inertia by combustionEngine.validate
    additionalEngineInertia = jbeamData.additionalEngineInertia or jbeamData.inertia or 0.12,

    -- Input configuration
    directionMode = directionMode,
    electricsThrottleName = jbeamData.electricsThrottleName or "throttle",
    electricsThrottleFactorName = jbeamData.electricsThrottleFactorName or "throttleFactor",
    electricsRegenThrottleName = jbeamData.electricsRegenThrottleName or "regenThrottle",
    debugMotor = jbeamData.debugMotor == true,

    -- Load / smoothing
    throttle = 0,
    throttleFactor = 1,  -- DSE trim, 1 = no intervention
    inertia = jbeamData.inertia or 0.1,
    instantEngineLoad = 0,
    engineLoad = 0,
    loadSmoother = newTemporalSmoothing(1, 1),
    -- inRate = fall, outRate = rise; fast: only interpolates between VCU frames
    throttleSmoother = newTemporalSmoothing(jbeamData.throttleFallRate or 30,
                                            jbeamData.throttleRiseRate or 10),
    etkFillNm = 0,
    etkShaftNm = 0,
    etkShaftRateMax = 0,

    -- EV creep floor (see creepFloor), "forward" mode only (the floor pushes in
    -- engine direction). PI gains [Nm/(rad/s)], [Nm/(rad/s)/s]; setpoint ramps
    -- [rpm/s]; authority ramps [1/s]; K0 / lock-up fades: full below Lo, zero from Hi.
    creepOn = (jbeamData.creepEnabled ~= false and jbeamData.creepEnabled ~= 0)
              and directionMode == "forward",
    creepKp = jbeamData.creepKp or 15,
    creepKi = jbeamData.creepKi or 150,
    creepRampUp = jbeamData.creepRampUpRpmS or 5000,
    creepRampDown = jbeamData.creepRampDownRpmS or 1500,
    creepMaxNm = jbeamData.creepMaxNm or 90,
    creepAuthRise = jbeamData.creepAuthRiseRate or 20,
    creepAuthFall = jbeamData.creepAuthFallRate or 5,
    creepK0Hi = k0Hi,
    creepK0Span = max(0.01, k0Hi - (jbeamData.creepK0FadeLo or 0.05)),
    creepLuHi = luHi,
    creepLuSpan = max(0.01, luHi - (jbeamData.creepLockupFadeLo or 0.10)),
    creepAuth = 0, creepSet = 0, creepI = 0, creepLimHold = 0,
    etkCreepNm = 0, etkCreepAcc = 0, etkCreepAccT = 0,

    -- Energy
    grossWorkPerUpdate = 0,
    frictionLossPerUpdate = 0,
    spentEnergy = 0,
    storageWithEnergyCounter = 0,
    storageCounter = 0,
    registeredEnergyStorages = {},
    previousEnergyLevels = {},
    energyStorageRatios = {},
    energyStorageRegenRatios = {},
    hasEnergy = true,
    requiredEnergyType = "electricEnergy",
    energyStorage = jbeamData.energyStorage,

    -- Output state
    outputTorqueState = 1,
    outputAVState = 1,
    isDisabled = false,

    -- sound state (set by initSounds; defaults keep updateSounds safe)
    soundConfiguration = {},
    engineSoundID = 0,
    engineVolumeCoef = 0,
    fundamentalFrequencyRPMCoef = 6 / 120,
    soundMinLoadMix = 0,
    soundMaxLoadMix = 1,
    soundIdleVolume = 0.18,
    invSoundReferenceTorque = 1 / 280,
    soundRegenVolumeCoef = 0.85,
    soundFadeInRPM = 60,
    invSoundFadeInRPM = 1 / 60,
    soundActiveVolumeThreshold = 0.02,

    -- ICE starter compat: the stock combustionEngine init walk reads these
    -- (nil = crash); starterMaxAV huge keeps starter comparisons safe
    idleAV = 0,
    idleRPM = 0,
    ignitionCoef = 1,
    isStalled = false,
    starterMaxAV = 1e9,
    invStarterMaxAV = 1e-9,
    starterEngagedCoef = 0,
    starterThrottleKillCoef = 1,
    starterThrottleKillTimer = 0,
    starterDisabled = false,
    stallTimer = 1,
    engineMiscSounds = {},
    combustionTorque = 0,
    nitrousOxideTorque = 0,
    frictionTorque = 0,

    -- Methods
    reset = reset,
    onBreak = onBreak,
    setMode = setMode,
    validate = validate,
    calculateInertia = calculateInertia,
    applyDeformGroupDamage = applyDeformGroupDamage,
    setPartCondition = setPartCondition,
    getPartCondition = getPartCondition,
    updateSimpleControlButtons = updateSimpleControlButtons,
    updateGFX = updateGFX,
    updateEnergyUsage = updateEnergyUsage,
    updateEnergyStorageRatios = updateEnergyStorageRatios,
    registerStorage = registerStorage,
    sendTorqueData = sendTorqueData,
    getTorqueData = getTorqueData,

    -- sound hooks; updateSounds stays nop until initSounds found a blend file
    initSounds = initSounds,
    resetSounds = resetSounds,
    updateSounds = nop,
    initEngineSound = initEngineSound,
    setEngineSoundParameterList = setEngineSoundParameterList,
    getSoundConfiguration = getSoundConfiguration,

    -- ICE starter compat stubs
    activateStarter = nop,
    deactivateStarter = nop,
    setIgnition = nop,
    cutIgnition = nop,
    scaleFriction = function(d, f) d.friction = d.friction * f end,
    scaleOutputTorque = function(d, s) d.outputTorqueState = 1 * s end,
    disable = function(d) d.outputTorqueState = 0; d.isDisabled = true end,
    enable = function(d) d.outputTorqueState = 1; d.isDisabled = false end,
    lockUp = function(d) d.outputTorqueState = 0; d.outputAVState = 0; d.isDisabled = true end,
  }

  -- Resolve output port
  local outputPortIndex = 1
  if jbeamData.outputPortOverride then
    device.outputPorts = {}
    for _, v in pairs(jbeamData.outputPortOverride) do
      device.outputPorts[v] = true
      outputPortIndex = v
    end
  end
  device.outputTorqueName = "outputTorque" .. tostring(outputPortIndex)
  device.outputAVName = "outputAV" .. tostring(outputPortIndex)
  device[device.outputTorqueName] = 0
  device[device.outputAVName] = 0

  -- "disconnected" is selectable only with jbeam canDisconnect
  if jbeamData.canDisconnect then
    device.availableModes = { "connected", "disconnected" }
    device.mode = jbeamData.isDisconnected and "disconnected" or "connected"
  else
    device.availableModes = { "connected" }
    device.mode = "connected"
  end

  -- Parse torque curve
  if not jbeamData.torque then
    log("E", "etk854hybrid_motorShaft.init", "No torque table for " .. tostring(device.name))
  end
  device.maxRPM = 0
  local torqueTable = tableFromHeaderTable(jbeamData.torque or {})
  local points = {}
  for _, v in pairs(torqueTable) do
    table.insert(points, { v.rpm, v.torque })
    device.maxRPM = max(device.maxRPM, v.rpm)
  end
  device.torqueCurve = createCurve(points)
  device.maxAV = device.maxRPM * rpmToAV

  device.torqueData = getTorqueData(device)
  device.maxPower = device.torqueData.maxPower
  device.maxTorque = device.torqueData.maxTorque

  -- regen curve: explicit table, or from the limits (maxRegenPower in kW)
  if jbeamData.regenTorqueCurve then
    local regenTable = tableFromHeaderTable(jbeamData.regenTorqueCurve)
    points = {}
    for _, v in pairs(regenTable) do
      table.insert(points, { v.rpm, v.torque })
    end
    device.regenCurve = createCurve(points)
  else
    local regenFadeRPM = jbeamData.regenFadeRPM or 1000
    local maxRegenPower = jbeamData.maxRegenPower or device.maxPower
    local regenTorqueLimit = jbeamData.maxRegenTorque or device.maxTorque

    device.regenCurve = { [0] = 0 }
    for i = 1, device.maxRPM do
      local fadeCoef = min(1, i / regenFadeRPM)
      local maxRegenTorque = min(regenTorqueLimit, maxRegenPower * 1000 / (i * rpmToAV))
      local maxOverallTorque = device.torqueCurve[i] or 0
      local scaledMaxTorque = fadeCoef * min(maxRegenTorque, maxOverallTorque)
      table.insert(device.regenCurve, scaledMaxTorque)
    end
  end

  device.minWantedRegenTorque = jbeamData.minimumWantedRegenTorque or 0
  device.maxWantedRegenTorque = jbeamData.maximumWantedRegenTorque or device.maxTorque
  device.instantMaxRegenTorque = 0

  -- sound emitter: first of torqueReactionNodes_nodes (cids from the jbeam key
  -- "torqueReactionNodes:"), fallback the ref node
  local trNodes = jbeamData.torqueReactionNodes_nodes
  if type(trNodes) == "table" then
    local allNumbers = true
    for _, n in pairs(trNodes) do
      if type(n) ~= "number" then allNumbers = false end
    end
    if allNumbers then device.torqueReactionNodes = trNodes end
  end
  if not device.torqueReactionNodes then
    device.torqueReactionNodes = {-1, -1, -1}
  end
  device.engineNodeID = device.torqueReactionNodes[1] or -1
  if device.engineNodeID < 0 then
    log("W", "etk854hybrid_motorShaft.init",
      "No torqueReactionNodes on " .. tostring(device.name) .. " - using ref node for the sound emitter")
    device.engineNodeID = v.data.refNodes[0].ref
  end

  -- electrical efficiency over |load| (number = flat), keyed in 0.01 steps
  local eff = jbeamData.electricalEfficiency or 0.94
  local effTable
  if type(eff) == "number" then
    effTable = { { 0, eff }, { 1, eff } }
  else
    effTable = deepcopy(eff)
  end
  local scaled = {}
  for k, v in pairs(effTable) do
    if type(k) == "number" then
      table.insert(scaled, { v[1] * 100, v[2] })
    end
  end
  scaled = createCurve(scaled)
  device.electricalEfficiencyTable = {}
  for k, v in pairs(scaled) do
    device.electricalEfficiencyTable[k * 0.01] = v
  end

  selectUpdates(device)
  return device
end

M.new = new
return M
