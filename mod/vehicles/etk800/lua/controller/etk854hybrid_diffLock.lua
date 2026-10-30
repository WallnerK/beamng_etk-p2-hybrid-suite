-- ETK 854 ActiveHybrid - rear differential lock scheduler (part
-- etk854hybrid_differential_R).
--
-- Schedules the stock LSD of powertrain/differential.lua:
--   lsdTorque = lsdPreload + min(coef,1)*|inputTorque| + activeLockTorque*activeLockCoef
--   coef = lsdLockCoef (drive) | lsdRevLockCoef (coast)
-- inputTorque is the axle torque, so coef 0.5 = 100 % lock under drive.
--
-- No wheel-speed regulation; rates are slews of a capacity. Every GFX frame:
--   1. preset OPEN / SPORT / TRACK / DRIFT from Driving Experience x DSC mode x
--      drivetrain (trailer mode caps at SPORT), slew the LSD fields towards it
--   2. open the lock for ABS, DSC yaw braking and a held standstill
--      (openRateFactor x faster)
--   3. slip HOLD via activeLockCoef in DRIFT and SPORT + DTC, rear-driven only.
--      Slip = rear MEAN vs undriven front: the lock torque does not change the
--      carrier speed, so no lock/unlock limit cycle as with a left/right PID
--   4. stock brake EDL "lockRear" per preset (its inner/outer wheel guess is
--      wrong while countersteering)
--
-- Base game (0.39.4): differential reset() zeroes activeLockCoef; M.reset runs
-- after powertrain.reset, re-fetches the device and snaps to the preset.
-- EDL setParameters needs the CMU (registered in initSecondStage), so the EDL
-- state is sent from updateGFX. absActive needs the driver's brake > 0.1.
--
-- Publishes: etk854hybrid_diffPreset, _diffLock (0..1 = max(2*drive coef, hold)),
--   _diffHold (0..1), _diffAxleSlip, _diffLockNm (transferred, (out2-out1)/2,
--   signed), _diffCapNm (capacity = lsdTorque), _diffHoldElig (0/1),
--   _diffHeld (0/1, opened while held)
-- Dev / QA: setOverride(name|nil), setHoldEnabled(bool|nil), getPresetDef(name),
--   getState(), getPreset()

local M = {}
M.type = "auxiliary"
M.defaultOrder = 86

local abs, min, max = math.abs, math.min, math.max

-- One table: LuaJIT 60-upvalue limit.
local st = {
  ok = false, diff = nil, diffName = "differential_R", edlName = "lockRear",
  edl = nil, edlResolved = false, edlApplied = nil, edlParams = {isEnabled = true},
  presets = {}, preset = "OPEN", override = nil, dscKey = "dsc", dm = nil,
  trailerCap = "SPORT", trailerCapped = false, held = false,
  cur = {preload = 0, power = 0, coast = 0},
  rate = {power = 1.0, coast = 1.0, preload = 300, open = 10, ivPreload = 20, ycPower = 0.05,
          stillMs = 0.7, stillBrake = 0.3},   -- held: < 2.5 km/h + brake/handbrake
  hold = {slip = true, dtc = true, force = nil, lo = 0.06, hi = 0.15, attack = 6.0, release = 2.0,
          hardOpen = 10.0, holdS = 0.25, vMin = 4.0, brakeMax = 0.3, ycLockS = 0.5, hbFreeze = true,
          h = 0, below = 0, s = 0, elig = false, ycT = 99},
  whl = nil
}

local U = require("controller/etk854hybrid/etk854hybrid_util")
local flag = U.flag
local num = U.num -- number as is, numeric string via tonumber, else d

local function bool(v, d)
  if v == nil then return d end
  if type(v) == "boolean" then return v end
  return num(v, d and 1 or 0) >= 0.5
end

local function readPreset(src, fallback)
  src = type(src) == "table" and src or {}
  return {
    preload = max(0, num(src.preload, fallback.preload)),
    power = max(0, min(0.5, num(src.power, fallback.power))),
    coast = max(0, min(0.5, num(src.coast, fallback.coast))),
    edl = bool(src.edl, fallback.edl),
    hold = bool(src.hold, fallback.hold)
  }
end

local DEFAULTS = {
  OPEN = {preload = 0, power = 0.0, coast = 0.0, edl = true, hold = false},
  SPORT = {preload = 60, power = 0.30, coast = 0.10, edl = true, hold = false},
  TRACK = {preload = 100, power = 0.40, coast = 0.20, edl = false, hold = false},
  DRIFT = {preload = 150, power = 0.50, coast = 0.35, edl = false, hold = true}
}

-- DSE drive-mode key "dsc" | "dtc" | "off"; fallback: the VCU's HUD label.
local function dscKey()
  local dm = st.dm
  if not dm then
    dm = controller.getController("driveModes")
    if dm and dm.getCurrentDriveModeKey then st.dm = dm else dm = nil end
  end
  local key = nil
  if dm then
    local ok, k = pcall(dm.getCurrentDriveModeKey)
    if ok and type(k) == "string" then key = k end
  end
  if not key then
    local lbl = electrics.values.etk854hybrid_dscMode
    key = (lbl == "OFF" and "off") or (lbl == "DTC" and "dtc") or "dsc"
  end
  st.dscKey = key
  return key
end

local function isRearDriven()
  -- RWD: no awdMode channel; xAllrad: "2WD"
  return electrics.values.etk854hybrid_awdMode ~= "AWD"
end

-- Driving Experience x DSC x drivetrain -> preset.
local function selectPreset(key)
  st.trailerCapped = false
  if st.override then return st.override end
  local ev = electrics.values
  local sport = ev.etk854hybrid_deMode == "SPORT"
  local p
  if key == "off" then
    p = (sport and isRearDriven()) and "DRIFT" or "TRACK"
  elseif key == "dtc" or sport then
    p = "SPORT"
  else
    p = "OPEN"
  end
  if (p == "TRACK" or p == "DRIFT") and flag(ev.etk854hybrid_trailerMode) and st.presets[st.trailerCap] then
    p = st.trailerCap
    st.trailerCapped = true
  end
  return p
end

-- Slip hold: DRIFT (preset "hold") with slipHold, or SPORT preset + SPORT
-- mode + DTC with dtcHold; rear-driven, no trailer.
local function holdEligible(name, p, key)
  local hd = st.hold
  if st.trailerCapped or flag(electrics.values.etk854hybrid_trailerMode) or not isRearDriven() then
    return false
  end
  local slipOn, dtcOn = hd.slip, hd.dtc
  if hd.force ~= nil then slipOn, dtcOn = hd.force, hd.force end
  if p.hold then return slipOn end
  return name == "SPORT" and dtcOn and key == "dtc" and electrics.values.etk854hybrid_deMode == "SPORT"
end

local function slew(cur, tgt, up, down, dt)
  if tgt > cur then return min(tgt, cur + up * dt) end
  if tgt < cur then return max(tgt, cur - down * dt) end
  return cur
end

local function wheelRefs()
  if st.whl then return st.whl end
  local w = {}
  if wheels and wheels.wheels then
    for i = 0, (wheels.wheelCount or 0) - 1 do
      local wd = wheels.wheels[i]
      if wd and wd.name then w[wd.name] = wd end
    end
  end
  if w.FL and w.FR and w.RL and w.RR then st.whl = w end
  return st.whl
end

-- Slip hold h (0..1) -> activeLockCoef. s = rear MEAN vs undriven front MEAN.
local function updateHold(name, p, key, absOn, ycOn, dt)
  local hd, ev = st.hold, electrics.values
  if ycOn then hd.ycT = 0 else hd.ycT = min(99, hd.ycT + dt) end
  local elig = holdEligible(name, p, key)
  hd.elig = elig
  local hb = num(ev.parkingbrake, 0) >= 0.1
  local drift = name == "DRIFT"
  local hardOpen = absOn or ycOn or num(ev.brake, 0) >= hd.brakeMax or (hb and not drift)
  local s, moving = 0, false
  local w = wheelRefs()
  if w then
    local vf = 0.5 * (abs(w.FL.wheelSpeed or 0) + abs(w.FR.wheelSpeed or 0))
    local vr = 0.5 * (abs(w.RL.wheelSpeed or 0) + abs(w.RR.wheelSpeed or 0))
    s = (vr - vf) / max(vf, 5)
    moving = vf > hd.vMin and num(ev.gearIndex, 0) > 0
  end
  hd.s = s
  if elig and moving and not hardOpen and drift and hb and hd.hbFreeze then
    -- handbrake drift entry slows the rear (s < 0): freeze h
    hd.below = 0
    return
  end
  if not (elig and moving and not hardOpen and hd.ycT >= hd.ycLockS) then
    hd.h = max(0, hd.h - (hardOpen and hd.hardOpen or hd.release) * dt)
    hd.below = 0
    return
  end
  local tgt = max(0, min(1, (s - hd.lo) / (hd.hi - hd.lo)))
  if tgt >= hd.h then
    hd.h = min(tgt, hd.h + hd.attack * dt)
    hd.below = 0
  else
    hd.below = hd.below + dt
    if hd.below > hd.holdS then hd.h = max(tgt, hd.h - hd.release * dt) end
  end
end

local function applyDevice()
  local d, c = st.diff, st.cur
  d.lsdPreload = c.preload
  d.lsdLockCoef = c.power
  d.lsdRevLockCoef = c.coast
  d.activeLockCoef = st.hold.h
end

-- Stock EDL on/off, sent on change only.
local function applyEdl(want)
  if want == st.edlApplied then return end
  if not st.edlResolved then
    st.edlResolved = true
    local c = controller.getController(st.edlName)
    st.edl = (c and c.setParameters) and c or nil
  end
  if st.edl then
    st.edlParams.isEnabled = want
    st.edl.setParameters(st.edlParams)
  end
  st.edlApplied = want
end

local function publish()
  local ev, d, c, hd = electrics.values, st.diff, st.cur, st.hold
  ev.etk854hybrid_diffPreset = st.preset
  ev.etk854hybrid_diffLock = min(1, max(2 * c.power, hd.h))
  ev.etk854hybrid_diffHold = hd.h
  ev.etk854hybrid_diffAxleSlip = hd.s
  ev.etk854hybrid_diffHoldElig = hd.elig and 1 or 0
  ev.etk854hybrid_diffHeld = st.held and 1 or 0
  if d then
    local T = d.inputTorque or 0
    local coef = (T > 0 and c.power) or (T < 0 and c.coast) or 0
    ev.etk854hybrid_diffCapNm = c.preload + min(coef, 1) * abs(T) + (d.activeLockTorque or 0) * hd.h
    ev.etk854hybrid_diffLockNm = 0.5 * ((d.outputTorque2 or 0) - (d.outputTorque1 or 0))
  else
    ev.etk854hybrid_diffCapNm = 0
    ev.etk854hybrid_diffLockNm = 0
  end
end

-- Damaged diff: no lock.
local function brokenOpen()
  local c, hd = st.cur, st.hold
  c.preload, c.power, c.coast = 0, 0, 0
  hd.h, hd.below, hd.elig = 0, 0, false
  st.preset = "OPEN"
  applyDevice()
  publish()
end

local function updateGFX(dt)
  local d = st.diff
  if not d then return end
  if d.isBroken then return brokenOpen() end
  local ev = electrics.values
  local key = dscKey()
  local name = selectPreset(key)
  st.preset = name
  local p = st.presets[name] or st.presets.OPEN
  -- a raw absActive false positive only opens the lock while braking (safe)
  local absOn = flag(ev.etk854hybrid_absQualified) or flag(ev.absActive)
  local ycOn = flag(ev.isYCBrakeActive)
  local r, c = st.rate, st.cur
  local tPre, tPow, tCoast = p.preload, p.power, p.coast
  if absOn or ycOn then
    tPre, tCoast = min(tPre, r.ivPreload), 0
    if ycOn then tPow = min(tPow, r.ycPower) end
  end
  -- Braked standstill / launch hold (ARMED, READY): no lock. The undamped LSD
  -- spring rings between two held rear wheels, which the launch anti-creep
  -- reads as rolling (ICE cut).
  local ls = ev.etk854hybrid_launchState
  local held = ls == "ARMED" or ls == "READY"
  if not held and abs(num(ev.airspeed, 0)) < r.stillMs then
    held = num(ev.brake, 0) >= r.stillBrake or num(ev.parkingbrake, 0) >= 0.1
  end
  if held then tPre, tPow, tCoast = 0, 0, 0 end
  st.held = held
  -- Fast opening only for ABS / yaw braking / held: a re-armed DSC mid-slide
  -- must not release the inner wheel in one frame.
  local k = (absOn or ycOn or held) and r.open or 1
  c.preload = slew(c.preload, tPre, r.preload, r.preload * k, dt)
  c.power = slew(c.power, tPow, r.power, r.power * k, dt)
  c.coast = slew(c.coast, tCoast, r.coast, r.coast * k, dt)
  updateHold(name, p, key, absOn, ycOn, dt)
  applyDevice()
  applyEdl(p.edl)
  publish()
end

local function snapToPreset()
  local key = dscKey()
  st.preset = selectPreset(key)
  local p = st.presets[st.preset] or st.presets.OPEN
  local c, hd = st.cur, st.hold
  c.preload, c.power, c.coast = p.preload, p.power, p.coast
  hd.h, hd.below, hd.s, hd.ycT = 0, 0, 0, 99
  hd.elig = holdEligible(st.preset, p, key)
  st.edlApplied = nil -- re-send from updateGFX
  if st.diff then applyDevice() end
end

local function init(jbeamData)
  st.diffName = jbeamData.differentialName or st.diffName
  st.edlName = jbeamData.edlName or st.edlName
  local src = type(jbeamData.presets) == "table" and jbeamData.presets or {}
  for name, def in pairs(DEFAULTS) do
    st.presets[name] = readPreset(src[name], def)
  end
  local r = st.rate
  r.power = max(0.01, num(jbeamData.rampPowerPerS, r.power))
  r.coast = max(0.01, num(jbeamData.rampCoastPerS, r.coast))
  r.preload = max(1, num(jbeamData.rampPreloadNmPerS, r.preload))
  r.open = max(1, num(jbeamData.openRateFactor, r.open))
  r.ivPreload = max(0, num(jbeamData.interventionPreloadNm, r.ivPreload))
  r.ycPower = max(0, num(jbeamData.ycPowerMax, r.ycPower))
  r.stillMs = max(0, num(jbeamData.stillKmh, r.stillMs * 3.6)) / 3.6
  r.stillBrake = max(0, num(jbeamData.stillBrake, r.stillBrake))
  local hd = st.hold
  hd.slip = bool(jbeamData.slipHold, true)
  hd.dtc = bool(jbeamData.dtcHold, true)
  hd.force = nil
  hd.lo = num(jbeamData.holdSlipLo, hd.lo)
  hd.hi = max(hd.lo + 0.01, num(jbeamData.holdSlipHi, hd.hi))
  hd.attack = max(0.01, num(jbeamData.holdAttackPerS, hd.attack))
  hd.release = max(0.01, num(jbeamData.holdReleasePerS, hd.release))
  hd.hardOpen = max(hd.release, num(jbeamData.holdHardOpenPerS, hd.hardOpen))
  hd.holdS = max(0, num(jbeamData.holdTimeS, hd.holdS))
  hd.vMin = num(jbeamData.holdMinRefSpeed, hd.vMin)
  hd.brakeMax = num(jbeamData.holdBrakeMax, hd.brakeMax)
  hd.ycLockS = max(0, num(jbeamData.holdYcLockoutS, hd.ycLockS))
  hd.hbFreeze = bool(jbeamData.driftHandbrakeFreeze, true)
  local cap = jbeamData.trailerCapPreset
  st.trailerCap = (type(cap) == "string" and st.presets[cap]) and cap or "SPORT"

  st.dm, st.whl, st.override = nil, nil, nil
  st.edl, st.edlResolved, st.edlApplied = nil, false, nil
  st.diff = powertrain.getDevice(st.diffName)
  st.ok = st.diff ~= nil and st.diff.mode == "lsd"
  if not st.ok then
    log("E", "etk854hybrid_diffLock.init", string.format("differential %q missing or not diffType lsd (%s), lock scheduler disabled",
      st.diffName, st.diff and tostring(st.diff.mode) or "nil"))
    st.diff = nil
    M.updateGFX = nil
    return
  end
  snapToPreset()
  st.dm = nil -- driveModes may not be initialised yet
  publish()
end

-- Spawn log line; lockSpring must be the auto value (~1e5), not the default 500.
local function initLastStage()
  local d = st.diff
  if not d then return end
  local p = st.presets.DRIFT
  log("I", "etk854hybrid_diffLock", string.format(
    "rear diff %q mode=%s lockSpring=%.0f lockTorque=%.0f activeLockTorque=%.0f preset=%s | DRIFT %.0f Nm %.2f/%.2f, slipHold=%s dtcHold=%s",
    st.diffName, tostring(d.mode), d.lockSpring or -1, d.lockTorque or -1, d.activeLockTorque or -1, st.preset,
    p.preload, p.power, p.coast, tostring(st.hold.slip), tostring(st.hold.dtc)))
  publish()
end

local function reset()
  if not st.ok then return end
  st.whl = nil
  local d = powertrain.getDevice(st.diffName)
  if d and d.mode == "lsd" then st.diff = d end
  snapToPreset()
end

-- electrics.reset may have cleared our channels.
local function resetLastStage()
  if st.ok then publish() end
end

-- Dev / QA: force a preset, nil = automatic. Beats the trailer cap, slews,
-- survives a vehicle reset.
local function setOverride(name)
  if type(name) == "string" then name = name:upper() end
  st.override = (type(name) == "string" and st.presets[name]) and name or nil
  return st.override
end

-- Dev / QA: force the DRIFT and DTC slip hold on / off, nil = jbeam values.
local function setHoldEnabled(on)
  if on == nil then st.hold.force = nil else st.hold.force = on and true or false end
  return st.hold.force
end

-- QA: the resolved preset table.
local function getPresetDef(name)
  local p = st.presets[name]
  if not p then return nil end
  return {preload = p.preload, power = p.power, coast = p.coast, edl = p.edl, hold = p.hold}
end

local function getState()
  local c, hd, d = st.cur, st.hold, st.diff
  return {
    preset = st.preset, override = st.override, dscKey = st.dscKey, held = st.held == true,
    deMode = electrics.values.etk854hybrid_deMode, trailerCapped = st.trailerCapped,
    preload = c.preload, power = c.power, coast = c.coast,
    hold = hd.h, holdEligible = hd.elig, axleSlip = hd.s, ycT = hd.ycT,
    slipHold = hd.slip, dtcHold = hd.dtc, holdForce = hd.force, edl = st.edlApplied,
    dev = d and {mode = d.mode, lsdPreload = d.lsdPreload, lsdLockCoef = d.lsdLockCoef,
                 lsdRevLockCoef = d.lsdRevLockCoef, activeLockCoef = d.activeLockCoef,
                 activeLockTorque = d.activeLockTorque, lockSpring = d.lockSpring,
                 lockTorque = d.lockTorque, isBroken = d.isBroken or false} or nil
  }
end

M.init = init
M.initLastStage = initLastStage
M.reset = reset
M.resetLastStage = resetLastStage
M.updateGFX = updateGFX
M.setOverride = setOverride
M.setHoldEnabled = setHoldEnabled
M.getPresetDef = getPresetDef
M.getState = getState
M.getPreset = function() return st.preset end

return M
