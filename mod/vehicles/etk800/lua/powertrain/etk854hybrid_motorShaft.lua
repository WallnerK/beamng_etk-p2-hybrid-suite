-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
--
-- Licence and credits for this file:
-- * Contains code derived from BeamNG.drive 0.39.4
--   lua/vehicle/powertrain/electricMotor.lua and combustionEngine.lua (bCDDL 1.1).
-- * The inline motor-on-a-shaft device concept and parts of its structure
--   (motorTorque(), motorOutputRPM / motorOutputTorque / motorRegenTorque,
--   previousEnergyRatios) are based on motorShaft.lua by NiZhaZi (NZZ) from the
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
-- Custom inline electric motor device for the ETK 854 P2 Hybrid mod.
--
-- v2 NOTE: Verbatim port of the v118-QA-proven device (8/8 autonomous tests PASS).
--          Only change vs. v1: the [HOLD_DBG] regen diagnostic print is gated
--          behind jbeam flag `debugMotor` (default off). Everything else is
--          intentionally byte-identical in logic — see
--          .claude/skills/beamng-troubleshoot/ for the pitfalls behind each line.
--
-- v2.1:    E-machine SOUND. The AVAS pedestrian-warning controller was removed;
--          the motor now uses the stock `electricMotor` engine-sound pipeline
--          (initSounds / updateSounds hooks, sfxBlend2D crossfade) so it whines
--          with RPM and load like the game's own EVs. No speed cutoff.
--          Physics/torque code is untouched. See the SOUND section below.
--
-- v2.2:    P1 MOTOR FILL LOOP (ICE engagement / torque coordination). Per
--          physics tick the motor adds
--              fillGain * (etk854hybrid_iceExpNm - measured ICE/K0 torque)
--          to its commanded torque, so K0 crank torque, K0 engage torque and
--          ICE (turbo) lag never reach the shaft - see motorTorque(). Inert
--          while etk854hybrid_fillGain = 0 (the VCU publishes 0 whenever the
--          fill must not act). Plus shaft-torque diagnostics (etkShaftNm /
--          etkShaftRateMax) and a jbeam-tunable throttle smoother.
--
-- v2.3:    EV CREEP / VIRTUAL IDLE (feedback round 3). creepFloor() holds the
--          converter pump (= this shaft) at the ICE idle speed as a torque
--          FLOOR, max(pedal-or-regen torque, PI floor), like the stock idle
--          governor - armed only by the VCU module etk854hybrid_creep.lua
--          (etk854hybrid_creepTargetRpm / etk854hybrid_creepMaxNm /
--          etk854hybrid_creepCut); a limit of 0 fades the floor out on the
--          last armed limit, creepCut = 1 drops it at once. Publishes
--          etk854hybrid_creepNm / _creepSetRpm
--          / _creepAuth / _pumpRpm / _tcPumpNm once per frame (updateGFX).
--
-- WHY this exists:
--   Stock `electricMotor` is designed as a terminal device (direct-to-wheels in EVs).
--   When chained inline between ICE and gearbox it crashes combustionEngine.lua:1031
--   because it doesn't propagate maxCumulativeGearRatio AND it doesn't stub the
--   starter-loop fields that the ICE init reads from its child.
--
--   This file is a self-contained inline motor: passes parent torque + AV through
--   the chain (like a shaft), adds motor torque additively to the output, propagates
--   inertia/gear-ratio fields correctly, AND stubs the ICE-starter-loop fields so
--   the stock combustionEngine init walk doesn't crash.
--
-- COMPLIANCE: unique device type name `etk854hybrid_motorShaft`, unique filename.
--             No stock file overrides.

local M = {}

M.outputPorts = {[1] = true}
-- This device sits INLINE between ICE and torqueConverter, so it must satisfy
-- BOTH parent and child validators:
--   - combustionEngine.validate (ICE side): requires child.deviceCategories.clutchlike
--   - torqueConverter.validate (downstream): requires parent.deviceCategories.engine
-- Without these flags, validation fails and the powertrain init leaves the
-- ICE's starterMaxAV unset -> downstream crash at combustionEngine.lua:1031.
M.deviceCategories = {shaft = true, electricMotor = true, engine = true, clutchlike = true}
-- Signal to the framework that we need external inertia tracking (we participate
-- in the parent's torque integration via additionalEngineInertia).
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
local psToWatt = 735.499

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
      device.previousEnergyRatios[storage.name] = storage.remainingRatio
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
  -- register by CAPACITY like the stock electricMotor (registerStorage :313):
  -- with storedEnergy > 0 a car spawned with an empty pack never registered it
  -- and the E-machine stayed dead after any recharge; updateEnergyUsage tracks
  -- the energy state (hasEnergy) from here on
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
-- Direction logic (self-contained, no external controller needed)
-- ===========================================================
local function computeMotorDirection(device)
  -- Mode "forward": always +1. CORRECT mode for the P2 inline placement -
  -- the motor sits UPSTREAM of the gearbox, so the reverse gear ratio does
  -- the reversing and the motor must always push in engine direction.
  -- Mode "auto": follow vehicle gear direction. Only correct for motors that
  -- drive an axle DIRECTLY (v1 through-the-road front motor); in the P2
  -- chain it makes the motor fight the ICE in reverse.
  if device.directionMode == "forward" then
    return 1
  end

  -- Default: auto. Read gearIndex (number: 0=N or P, -1=R, 1..n=forward) or gear (string: P/R/N/D/S/M)
  --
  -- v118 fix: `gearIndex == 0` is AMBIGUOUS — both Neutral AND Park return 0.
  -- The earlier `else return 0 end` shortcut killed the string-based Park branch
  -- below, so the v115 P-handling code path was never reached when AT exposed
  -- gearIndex as a number (which it always does). Result: T7 Park-Charge regen
  -- stayed at 0 N·m even with regenCmd=1.0 and motor AV ~100 rad/s.
  -- Fix: check `electrics.values.gear` STRING to distinguish P from N when
  -- gearIndex == 0; only true N returns 0.
  local gearIndex = electrics.values.gearIndex
  local gearStr = electrics.values.gear
  if type(gearIndex) == "number" then
    if gearIndex > 0 then return 1
    elseif gearIndex < 0 then return -1
    elseif gearStr == "P" then return 1  -- Park: motor input shaft still spinning with ICE
    else return 0 end  -- N: truly disconnected
  end

  local gear = electrics.values.gear
  if type(gear) == "string" then
    if gear == "R" then return -1
    elseif gear == "N" then return 0
    -- BUG FIX [v115]: P was mapped to 0 (neutral) which forced motorDirection=0
    -- -> rpm=0 -> regenCurve[0]=0 -> regenTorque=0 -> Park-Charge completely broken.
    -- P is mechanically identical to N from the motor shaft's perspective (gearbox
    -- output is locked, but the INPUT shaft - where the motor sits - is still free
    -- and spinning with the ICE). Return +1 so the regen curve indexes correctly.
    elseif gear == "P" then return 1
    else return 1 end
  end

  -- Fallback if neither field is set yet (early init)
  return 1
end

-- ===========================================================
-- EV CREEP / VIRTUAL IDLE (feedback round 3)
-- ===========================================================
-- A speed FLOOR on the converter pump (= this shaft), the job the stock idle
-- governor does on an ICE (combustionEngine.lua: idle throttle from the idle
-- PI, then throttle = max(idleThrottle, driver throttle)): the motor torque
-- becomes max(pedal-or-regen torque, PI floor). Creep torque and creep speed
-- then come out of the stock converter physics exactly like ICE creep.
-- Armed ONLY by the VCU module etk854hybrid_creep.lua through two channels:
--   etk854hybrid_creepTargetRpm  setpoint (ICE idle, or the HOLD standby rpm)
--   etk854hybrid_creepMaxNm      authority limit; 0 = disarmed: the floor
--                                fades out at creepAuthFall per second on
--                                the LAST armed limit (latched in
--                                device.creepLimHold), so a VCU disarm (e.g.
--                                reason "ice" at the BLEND handover) is no
--                                torque step
--   etk854hybrid_creepCut        1 = hard cut: authority 0 in this tick
--                                (failsafe, key off, launch, lockout, ABS)
-- Device-side interlocks, independent of the VCU (physics rate):
--   * K0 torque that really reaches the shaft, k0 * (1 - fillGain): in SYNC /
--     ENGAGE NORMAL / RELEASE:OPEN the fill cancels the K0 torque, so creep may
--     keep holding the shaft; an IDLE engage (fill 0) or a coupled ICE fades
--     it out between creepK0FadeLo and creepK0FadeHi (= the sequencer's
--     governor handover ratio 0.30)
--   * the lock-up clutch closing (a speed governor behind a locked converter
--     would be a cruise control)
-- PI with conditional integration + anti-windup (I in [0, limit]). Pedal
-- torque and the fill loop below are never reduced. Regen: the floor is
-- two-sided on purpose - within kp*e of the setpoint (e < 0: shaft ABOVE it,
-- I = 0) the P term may lift a regen torque towards 0 (about |regen| / 1.6
-- rpm above the setpoint, ~44 rpm for 69 Nm). That band is the continuous
-- handover from regen to the floor: a one-sided floor (act only when > 0)
-- cannot hold a regen torque at the setpoint and chatters between regen and
-- ~0 Nm every physics tick (offline: same mean, 35 Nm sd). The regen the
-- floor lifts is what a speed floor allows at all (the pump never drops
-- below the creep idle - an ICE idling in gear behaves the same); the
-- creep-aware regen speed fade in updateRecuperation keeps the overlap
-- short, and the friction brake blend credits the measured converter
-- torque. The floor bypasses the low-AV taper on purpose (an idle governor
-- has full authority at 0 rpm). Energy, grossWork, motorTorque and the EV
-- power channel include it automatically.
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
  -- last armed limit: a disarm (limit 0) fades out on it instead of
  -- collapsing lim (= limit x a) to 0 in one tick
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
    -- start the authority where the seed fits under lim (= limBase x a):
    -- the converter load is picked up in this tick (RELEASE:OPEN, where the
    -- fill cancels the K0 torque at once), the rest ramps at creepAuthRise
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
  i = clamp(i, 0, lim)                                -- anti-windup: I in [0, lim]
  device.creepI = i
  local floorNm = min(kp * e + i, lim)                -- two-sided P band: see the header
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
  -- DSE / DSC HOOK -----------------------------------------------------------
  -- drivingDynamics/supervisors/components/motorTorqueControl trims a controlled
  -- powertrain device by writing `device.throttleFactor` (see its updateFixedStep:
  --   motor.throttleFactor = min(tractionControlFactor, yawControlFactor)).
  -- The stock electricMotor honours it via
  --   electrics.values[electricsThrottleFactorName] or device.throttleFactor
  -- We previously fell back to a literal 1, so DSC/TC could NOT trim the
  -- E-machine at all: up to 500 Nm kept pushing while only the brakes fought it.
  -- Taking the MINIMUM of both sources is strictly safer than the stock `or`:
  -- an electrics-driven limit and a DSE trim can never cancel each other out.
  local throttleFactor = min(electrics.values[device.electricsThrottleFactorName] or 1,
                             device.throttleFactor or 1)
  local rawThrottle = (electrics.values[device.electricsThrottleName] or 0) * throttleFactor
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

  -- Low-AV taper: real EV motors smoothly ramp torque from standstill rather
  -- than slamming peak torque at AV=0. Below LOW_AV_RAD/s, scale torque
  -- linearly from 0.30 at AV=0 to 1.0 at LOW_AV_RAD/s.
  -- Prevents simulation instability + ICE-overrev transients when K0 closes
  -- against an idling ICE while motor is at peak torque.
  local LOW_AV_RAD = 8.0  -- ~76 RPM
  local LOW_AV_FLOOR = 0.30
  local absAV = engineAV >= 0 and engineAV or -engineAV
  if absAV < LOW_AV_RAD then
    local taper = LOW_AV_FLOOR + (1.0 - LOW_AV_FLOOR) * (absAV / LOW_AV_RAD)
    torque = torque * taper
  end

  -- Regen
  local regenThrottle = electrics.values[device.electricsRegenThrottleName] or 0
  local rawRegenTorque = (device.regenCurve[abs(torqueRPM)] or 0)
  local regenTorque = -(min(max(rawRegenTorque * regenThrottle, min(rawRegenTorque, device.minWantedRegenTorque)),
    device.maxWantedRegenTorque) * sign(regenThrottle) * throttleFactor * motorDirection)
  device.regenThrottle = regenThrottle
  device.instantMaxRegenTorque = rawRegenTorque

  local actualTorque = throttle > 0 and torque or regenTorque
  -- EV creep floor BEFORE the fill block: the fill keeps cancelling the K0
  -- torque in SYNC / ENGAGE / RELEASE:OPEN on top of whatever creep commands
  local ev = electrics.values
  local creepNm
  actualTorque, creepNm = creepFloor(device, dt, engineAV, actualTorque, throttleFactor, ev)
  local driveNm = actualTorque

  -- P1 MOTOR FILL LOOP (spec 7.6). The VCU (etk854hybrid_torqueCoord)
  -- publishes what the combustion side is EXPECTED to deliver
  -- (etk854hybrid_iceExpNm) and the gain; the motor makes up the difference
  -- to what the K0 / ICE actually deliver THIS tick:
  --   slip states (K0 cranking / open / engaging): measurement = K0 torque
  --     (clutchK0.torqueDiff, frictionClutch.lua:104, computed earlier in the
  --     same tick) -> the crank / engage torque is cancelled at the shaft
  --   locked: measurement = ice.combustionTorque (CE:802; excludes the ICE
  --     inertia term, so there is no algebraic loop and the driveline keeps
  --     its natural inertia feel) -> turbo lag / model error is filled
  -- A gain <= 1 cancellation, not a feedback loop on slip. device.parent is
  -- clutchK0, its parent mainEngine (powertrain chain in the jbeam).
  local fillNm = 0
  local fillGain = ev.etk854hybrid_fillGain or 0
  if fillGain > 0 then
    local k0 = device.parent
    local ice = k0 and k0.parent
    local k0T = (k0 and k0.torqueDiff) or 0
    local comb = (ice and ice.combustionTorque) or k0T
    -- Locked = the sequencer says K0 carries the engine (BLEND / ON / torque
    -- release). The spec's slip test (r >= 0.9 and |slip| < 5.2 rad/s) is
    -- only the fallback: under the stock K0 spring ringing (tens of rpm at
    -- ~70 Hz in the offline harness) it would flicker and feed the ringing
    -- K0 torque back into the fill with ~100 deg phase lag.
    local lockCh = ev.etk854hybrid_fillLocked
    local lockT
    if lockCh ~= nil then
      lockT = (lockCh >= 0.5) and 1 or 0
    else
      local slipAV = ((ice and ice.outputAV1) or engineAV) - engineAV
      lockT = ((ev.etk854hybrid_k0ClutchRatio or 0) >= 0.9 and abs(slipAV) < 5.2) and 1 or 0
    end
    -- 0.1 s crossfade between the two measurements, no step at lock
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
    local capT = (torqueCurve[abs(torqueRPM)] or 0) * device.outputTorqueState * throttleFactor
    local capR = device.regenCurve[abs(torqueRPM)] or 0
    actualTorque = clamp(actualTorque + fillNm * motorDirection, -capR, capT)
  else
    device.etkIceMeasLpf = nil
    device.etkLockW = nil
  end
  device.etkFillNm = fillNm

  -- [HOLD_DBG] Motor regen diagnostic — throttled to 1 Hz (Bug 3 fix).
  -- Previously used frame-counter cap of 60 frames which at 1000 Hz physics = 60ms,
  -- still generating ~17 lines/s. Time-based gate keeps it at ≤1 line/s.
  -- v2: gated behind jbeam flag `debugMotor` (default off) — QA diagnostic only.
  if device.debugMotor then
    if device._dbgRegenLastT == nil then device._dbgRegenLastT = 0 end
    device._dbgRegenLastT = device._dbgRegenLastT + dt
    if device._dbgRegenLastT >= 1.0 then
      device._dbgRegenLastT = 0
      local regenCmd = electrics.values[device.electricsRegenThrottleName] or 0
      if regenCmd > 0.01 then
        log("I", "etk854hybrid.motor", string.format(
          "[HOLD_DBG] MOTOR regen: dir=%d rpm=%d regenCmd=%.3f rawRegenTq=%.1f regenTq=%.2f thr=%.3f AV=%.2f",
          motorDirection, torqueRPM, regenCmd, rawRegenTorque, regenTorque, throttle, engineAV))
      end
    end
  end

  device.motorOutputRPM = torqueRPM
  -- creep floor torque counts as drive torque (sound activity, diagnostics)
  device.motorOutputTorque = (creepNm > 0) and max(torque, driveNm) or torque
  device.motorRegenTorque = regenTorque

  local maxCurrentTorque = (torqueCurve[abs(torqueRPM)] or torqueCurve[0]) - friction
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

  device.motorTorque = (actualTorque - frictionTorque)
  return (actualTorque - frictionTorque)
end

-- ===========================================================
-- Physics update (per fixed-step tick)
--
-- DESIGN: clutchlike pattern (like frictionClutch/torqueConverter), NOT shaft.
-- A shaft writes back to parent.outputAV1, which overwrites the ICE's internally-
-- computed AV and stalls it (ICE.outputAV1 = chain_downstream_AV = 0 at standstill).
-- A clutchlike device just READS parent.outputAV1 and propagates torque additively.
-- This way the ICE's own torque integration drives outputAV1, and our motor's
-- inertia is folded in via `additionalEngineInertia` during combustionEngine.validate.
-- ===========================================================
local function updateVelocity(device, dt)
  -- SELF-INTEGRATING MOTOR (electricMotor-like pattern).
  -- Our outputAV1 is computed in updateTorque via Newton integration on net torque.
  -- Here we just:
  --   1. Set inputAV = outputAV1 (gearRatio=1, hard-coupled shaft)
  --   2. Write back to parent.outputAV1 = our.inputAV  (so parent K0 has slip info)
  -- This allows the K0 clutch above us to compute slip torque between
  -- ICE.outputAV (its inputAV) and our.outputAV1 (its outputAV via this writeback).
  device.inputAV = device[device.outputAVName] * device.gearRatio
  device.parent[device.parentOutputAVName] = device.inputAV
end

local function updateTorque(device, dt)
  -- Self-integrating motor: outputAV1 grows from net torque / inertia.
  -- Inputs:
  --   * Upstream torque (from K0 clutch, through parent.outputTorque1)
  --   * Our motor's own torque (from motorTorque() based on throttle + rpm curve)
  --   * Downstream load (TC.torqueDiff, what the chain below absorbs)
  --   * Friction (shaft losses)
  local engineAV = device.outputAV1
  local motorT = motorTorque(device, dt)

  -- Upstream torque from K0 clutch (or directly from ICE if no K0)
  local k0Torque = device.parent[device.parentOutputTorqueName] or 0

  -- Downstream load (TC.torqueDiff from last tick)
  local downstreamLoad = 0
  if device.children and device.children[1] and device.children[1].torqueDiff then
    downstreamLoad = device.children[1].torqueDiff
  end

  -- Friction
  local frictionLoad = (device.friction * clamp(engineAV, -1, 1) + device.dynamicFriction * engineAV)
    * device.wearFrictionCoef * device.damageFrictionCoef

  -- Net torque on motor shaft. Newton: AV[t+1] = AV[t] + dt * netT / I.
  local netT = k0Torque + motorT - downstreamLoad - frictionLoad

  -- P1 diagnostics: torque K0 + motor put into the shaft (30 ms LPF) and the
  -- largest rate of change since the VCU last reset it (ICE start stats).
  local shIn = k0Torque + motorT
  local shPrev = device.etkShaftNm or shIn
  local shNew = shPrev + (shIn - shPrev) * min(1, dt / 0.03)
  device.etkShaftNm = shNew
  local shRate = abs(shNew - shPrev) / dt
  if shRate > (device.etkShaftRateMax or 0) then device.etkShaftRateMax = shRate end

  -- Integrate AV. Limit to motor's max AV for safety.
  local newAV = engineAV + dt * netT / device.inertia
  device.outputAV1 = math.max(-device.maxAV, math.min(device.maxAV, newAV))

  -- outputTorque1 = forward to downstream (informational; TC computes its own input from kFactor)
  device[device.outputTorqueName] = k0Torque + motorT - frictionLoad

  -- torqueDiff = what K0 feels back from motor side. The K0 sees this as load
  -- through its clutch; combined with K0.inputAV (ICE) and K0.outputAV (motor),
  -- K0 computes its slip-based torque transmission. We send downstreamLoad here
  -- so the K0 propagates correct load back to ICE.
  device.torqueDiff = downstreamLoad + frictionLoad - motorT
end

local function disconnectedUpdateVelocity(device, dt)
  -- Disconnected = K0 clutch open. ICE spins free, our motor on its own virtual mass.
  device.inputAV = device.virtualMassAV
  device[device.outputAVName] = device.virtualMassAV / device.gearRatio
end

local function disconnectedUpdateTorque(device, dt)
  -- ICE decoupled. Don't pass parent torque; only motor torque flows downstream.
  local motorT = motorTorque(device, dt)
  device.virtualMassAV = device.virtualMassAV + motorT * device.invCumulativeInertia * dt
  device[device.outputTorqueName] = motorT
  device.torqueDiff = 0  -- no load on ICE
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
  -- Self-contained direction. No external controller needed.
  device.motorDirection = computeMotorDirection(device)

  -- torqueDiff: the back-torque this device loads on its parent (ICE).
  -- ICE reads `child.torqueDiff` in its torque update; nil here -> crash at
  -- combustionEngine.lua:791. For pass-through shaft, this equals outputTorque.
  if device.mode == "disconnected" then
    device.torqueDiff = 0
  else
    device.torqueDiff = device[device.outputTorqueName] or 0
  end

  device:updateEnergyUsage()

  device.outputRPM = device.outputAV1 * avToRPM

  -- EV creep telemetry, once per frame (the VCU reads it the next frame):
  --   creepNm      frame mean of the torque the floor added [Nm]
  --   creepSetRpm  ramped floor setpoint, creepAuth  device authority 0..1
  --   pumpRpm      this shaft = converter pump [rpm] (THE pump / shaft speed
  --                channel for every package, signed)
  --   tcPumpNm     converter pump load (torqueConverter torqueDiff) [Nm]
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
-- E-MACHINE SOUND  (stock electricMotor hook pattern)
--
-- WHY this lives in the device and not in an auxiliary controller:
--   powertrain.lua calls `device:initSounds(deviceJbeamData[device.name])`
--   for EVERY powertrain device that exposes the method (powertrain.lua:716-718)
--   and `device:updateSounds(dt)` once per GFX frame (powertrain.lua:214-216).
--   The hook is NOT restricted to stock device types, so a custom device gets
--   the exact same engine-sound pipeline the stock `electricMotor` uses
--   (electricMotor.lua:432-500 initSounds, :163-168 updateSounds).
--   That pipeline is the real, rpm-/load-blended EV whine (sfxBlend2D crossfade
--   driven by obj:setEngineSound), NOT a fixed-pitch FMOD one-shot.
--
-- Difference vs. stock: our shaft also spins with the ICE whenever K0 is closed,
-- so a constant full-volume whine would be wrong. We modulate engineVolumeCoef
-- by actual electrical activity (traction torque OR regen torque) on top of a
-- small idle floor, and fade the emitter out below volumeFadeInRPM.
-- There is deliberately NO speed cutoff - this is a motor, not a pedestrian
-- warning device.
-- ===========================================================
local function getSoundConfiguration(device)
  return device.soundConfiguration
end

local function updateSounds(device, dt)
  local motorAV = device.outputAV1 or 0
  local rpmRaw = abs(motorAV) * avToRPM
  local rpm = device.soundRPMSmoother:get(rpmRaw, dt)

  -- Load mix. instantEngineLoad is signed on this device (negative = regen),
  -- the blend only understands magnitude - so recuperation drives the same
  -- "on load" layer as traction does.
  local rawLoad = abs(device.instantEngineLoad or 0)
  local engineLoad = clamp(device.soundLoadSmoother:get(rawLoad, dt),
    device.soundMinLoadMix, device.soundMaxLoadMix)

  -- Activity -> volume coefficient.
  local driveT = abs(device.motorOutputTorque or 0)
  local regenT = abs(device.motorRegenTorque or 0) * device.soundRegenVolumeCoef
  local activity = clamp(max(driveT, regenT) * device.invSoundReferenceTorque, 0, 1)
  local targetVolume = device.soundIdleVolume + (1 - device.soundIdleVolume) * activity

  -- Fade out towards standstill so a parked car is silent.
  if rpmRaw < device.soundFadeInRPM then
    targetVolume = targetVolume * (rpmRaw * device.invSoundFadeInRPM)
  end
  -- Flat battery / disabled motor -> no whine.
  if device.isDisabled or not device.hasEnergy then
    targetVolume = 0
  end

  local volume = device.soundVolumeSmoother:get(clamp(targetVolume, 0, 1), dt)
  device.engineVolumeCoef = volume

  local fundamentalFreq = sounds.hzToFMODHz(rpm * device.fundamentalFrequencyRPMCoef)
  obj:setEngineSound(device.engineSoundID, rpm, engineLoad, fundamentalFreq, volume)

  -- Diagnostic channels (QA suite T15 + HUD).
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

-- Publish the diagnostic channels even when the sound never comes up, so QA can
-- tell "sound config broken" (active=0) apart from "device not in config" (nil).
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
  -- NOTE on newTemporalSmoothingNonLinear(inRate, outRate) (common/filters.lua:351,
  -- :356 `self[dif * st >= 0]`): outRate governs motion AWAY from zero (rise),
  -- inRate motion TOWARDS zero (fall). So "out > in" = fast attack, slow release.
  device.soundVolumeSmoother = newTemporalSmoothingNonLinear(
    soundConfig.volumeSmootherInRate or 3, soundConfig.volumeSmootherOutRate or 6)

  device.soundMaxLoadMix = soundConfig.maxLoadMix or 1
  device.soundMinLoadMix = soundConfig.minLoadMix or 0
  local fundamentalFrequencyCylinderCount = soundConfig.fundamentalFrequencyCylinderCount or 6
  device.fundamentalFrequencyRPMCoef = fundamentalFrequencyCylinderCount / 120
  device.engineVolumeCoef = 0

  -- ETK854 extensions
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
-- Inertia & gear-ratio propagation (CRITICAL for chain)
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
-- Damage / break / part-condition (stubs - we don't model damage in Phase A)
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
  device.motorOutputRPM = 0
  device.motorOutputTorque = 0
  device.motorRegenTorque = 0
  device.outputRPM = 0
  device.motorDirection = 1

  -- P1 fill loop / diagnostics state
  device.etkFillNm = 0
  device.etkShaftNm = 0
  device.etkShaftRateMax = 0
  device.etkLockW = nil
  device.etkIceMeasLpf = nil

  -- EV creep floor state (parameters stay as built by new())
  device.creepAuth, device.creepSet, device.creepI, device.creepLimHold = 0, 0, 0, 0
  device.etkCreepNm, device.etkCreepAcc, device.etkCreepAccT = 0, 0, 0

  device.throttle = 0
  -- DSE trim factor (written by motorTorqueControl). 1 = no intervention.
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
  device.previousEnergyRatios = {}
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
    -- isPhysicallyDisconnected: we're hard-coupled to the ICE shaft, not a
    -- breakable mechanical shaft. Set false so validate() leaves us in
    -- "connected" mode and energy tracking flows normally.
    isPhysicallyDisconnected = false,
    inputAV = 0,
    outputAV1 = 0,
    outputTorque1 = 0,
    virtualMassAV = 0,
    visualShaftAngle = 0,
    isBroken = false,
    nodeCid = jbeamData.node,

    -- Motor state
    motorTorque = 0,
    motorOutputRPM = 0,
    motorOutputTorque = 0,
    motorRegenTorque = 0,
    outputRPM = 0,
    motorDirection = 1,
    maxTorqueLimit = math.huge,
    isPropulsed = true,
    -- torqueDiff: parent (ICE) reads this to know back-torque load. Init to 0.
    torqueDiff = 0,
    -- additionalEngineInertia is summed into ICE.inertia during validate.
    -- Default = our motor's own inertia (hard-coupled, rotates with ICE shaft).
    -- jbeam can override if you want to split it differently.
    additionalEngineInertia = jbeamData.additionalEngineInertia or jbeamData.inertia or 0.12,

    -- Input configuration
    directionMode = jbeamData.directionMode or "auto",
    electricsThrottleName = jbeamData.electricsThrottleName or "throttle",
    electricsThrottleFactorName = jbeamData.electricsThrottleFactorName or "throttleFactor",
    electricsRegenThrottleName = jbeamData.electricsRegenThrottleName or "regenThrottle",
    minWantedRegenTorque = jbeamData.minimumWantedRegenTorque or 0,
    maxWantedRegenTorque = jbeamData.maximumWantedRegenTorque or 200,
    -- v2: debug gate for the [HOLD_DBG] regen diagnostic print (default off;
    -- set "debugMotor": true in the jbeam device row to enable).
    debugMotor = jbeamData.debugMotor == true,

    -- Load / smoothing
    throttle = 0,
    -- DSE trim factor. motorTorqueControl (drivingDynamics) overwrites this
    -- every fixed step for every device listed in tractionControlledMotors /
    -- yawControlledMotors. It MUST exist before the first physics step, else
    -- motorTorque() would read nil on frame 0.
    throttleFactor = 1,
    inertia = jbeamData.inertia or 0.1,
    instantEngineLoad = 0,
    engineLoad = 0,
    loadSmoother = newTemporalSmoothing(1, 1),
    -- newTemporalSmoothing(inRate, outRate): inRate applies toward zero
    -- (fall), outRate away from zero (rise) - filters.lua:390-419. Defaults
    -- = the old fixed 30/10. The powertrain jbeam sets rise 40 / fall 60 so
    -- the device only interpolates between VCU frames: the VCU jerk limiter
    -- (etk854hybrid_torqueCoord) is the single drive-torque shaper.
    throttleSmoother = newTemporalSmoothing(jbeamData.throttleFallRate or 30,
                                            jbeamData.throttleRiseRate or 10),
    etkFillNm = 0,
    etkShaftNm = 0,
    etkShaftRateMax = 0,

    -- EV creep floor (see creepFloor; jbeam keys in the integratedMotor
    -- section of etk854hybrid_powertrain.jbeam). Only in the P2 "forward"
    -- mode: the floor pushes in engine direction, the gearbox reverses.
    --   creepKp / creepKi   PI on the shaft speed [Nm/(rad/s)], [Nm/(rad/s)/s]
    --   creepRampUp/Down    setpoint ramps [rpm/s]
    --   creepMaxNm          device ceiling on the authority limit [Nm]
    --   creepAuthRise/Fall  authority ramps [1/s]
    --   creepK0Hi / Span    effective-K0 fade (full below Lo, zero from Hi)
    --   creepLuHi / Span    lock-up fade (full below Lo, zero from Hi)
    creepOn = (jbeamData.creepEnabled ~= false and jbeamData.creepEnabled ~= 0)
              and ((jbeamData.directionMode or "auto") == "forward"),
    creepKp = jbeamData.creepKp or 15,
    creepKi = jbeamData.creepKi or 150,
    creepRampUp = jbeamData.creepRampUpRpmS or 5000,
    creepRampDown = jbeamData.creepRampDownRpmS or 1500,
    creepMaxNm = jbeamData.creepMaxNm or 90,
    creepAuthRise = jbeamData.creepAuthRiseRate or 20,
    creepAuthFall = jbeamData.creepAuthFallRate or 5,
    creepK0Hi = jbeamData.creepK0FadeHi or 0.30,
    creepK0Span = max(0.01, (jbeamData.creepK0FadeHi or 0.30) - (jbeamData.creepK0FadeLo or 0.05)),
    creepLuHi = jbeamData.creepLockupFadeHi or 0.50,
    creepLuSpan = max(0.01, (jbeamData.creepLockupFadeHi or 0.50) - (jbeamData.creepLockupFadeLo or 0.10)),
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
    previousEnergyRatios = {},
    energyStorageRatios = {},
    energyStorageRegenRatios = {},
    hasEnergy = true,
    requiredEnergyType = "electricEnergy",
    energyStorage = jbeamData.energyStorage,

    -- Output state
    outputTorqueState = 1,
    outputAVState = 1,
    isDisabled = false,

    -- E-machine sound state (filled in by initSounds; defaults keep
    -- updateSounds arithmetic safe even if a soundConfig is half-specified)
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

    -- ICE-starter-loop compat: stock combustionEngine init walks the chain
    -- and reads these fields on its child. nil = crash.
    -- The starter* fields are set high so any stock code that accidentally
    -- dispatches combustionEngine.deactivateStarter on us (due to inheriting
    -- field-name patterns) compares safely instead of crashing on `nil * 1.1`.
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
    maxRPM = jbeamData.maxRPM or 10000,
    maxAV = (jbeamData.maxRPM or 10000) * rpmToAV,

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

    -- E-machine sound hooks. powertrain.lua dispatches initSounds/resetSounds
    -- (:716-718, :777-779) and updateSounds (:214-216) on ANY device that has
    -- them, so this custom device gets the stock EV whine pipeline.
    -- updateSounds starts as nop and is swapped for the real one by initSounds
    -- only once a blend file was actually configured (same as stock).
    initSounds = initSounds,
    resetSounds = resetSounds,
    updateSounds = nop,
    initEngineSound = initEngineSound,
    setEngineSoundParameterList = setEngineSoundParameterList,
    getSoundConfiguration = getSoundConfiguration,

    -- ICE-starter compat stubs (called by stock combustionEngine init walk)
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

  -- Available modes (always allow disconnect for future K0-style logic)
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
  device.maxPowerThrottleMap = device.torqueData.maxPower * psToWatt

  -- Build regen curve from limits (mirror stock electricMotor's algorithm)
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

  device.invEngInertia = 1 / device.inertia
  device.halfInvEngInertia = device.invEngInertia * 0.5

  -- Sound emitter node. The jbeam key `"torqueReactionNodes:"` makes the jbeam
  -- loader translate node NAMES to cids into `torqueReactionNodes_nodes`
  -- (same contract stock electricMotor.lua:587-600 / :700-704 relies on).
  -- The first entry is the emitter position; fall back to the vehicle ref node.
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

  -- Electrical efficiency table (default flat 0.94 if not specified)
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
