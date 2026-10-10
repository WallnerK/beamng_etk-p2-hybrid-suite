-- ETK 854 ActiveHybrid - Rear self-leveling + trailer detection
--
-- Analogue of a premium wagon's rear air suspension with level control: under
-- rear load (trunk cargo, trailer tongue load) the rear ride height is slowly
-- restored to the unloaded nominal level. The spring RATE stays unchanged,
-- only the preload grows - exactly what pumping up an air spring does to the
-- static equilibrium.
--
-- Actuator: the two existing named spring beams spring_RR / spring_RL
-- (etk854hybrid_chassis.jbeam, part etk854hybrid_spring_R). Their rest length
-- is set absolutely (drift free) via
--   obj:setPrecompressionRatio(cid, (refLen + offset) / refLen)
-- MEASURED 2026-09-30 (0.39.4): setPrecompressionRatio scales the beam's REF
-- length (obj:getBeamRefLength = the rest length setBeam() received, i.e.
-- spawn distance * beamPrecompression = 0.4297 m here), NOT the spawn node
-- distance. The first version wrote P0 + offset/Lref (P0 = 1.3518): every
-- reset() wrote 1.3518 * 0.4297 = 0.5809 m -> +6.7 kN preload, the rear
-- jacked up ~90 mm onto its rebound stops (hard stop in tension, 13 mm sag
-- for 250 kg), the calibration then saw -80 mm, fell back to the design
-- length and the error stayed negative: the loop never raised.
-- beamstate.lua:749 is consistent with this: those beams are created with
-- ratio 1 (ref = spawn length) and ramped from 1. With the correct base the
-- unladen spring settles at 310.2 mm vs 307.9 mm design (N57S AWD).
-- No new nodes/beams/flexbodies, no air tank, no hydros:
--   * hydros centre their rest ratio at 1 -> the ~1.35 spring precompression
--     would be lost, and hydros.reset returns them to centre.
--   * stock hydraulicSuspension.reset sets bleedCoef=1 -> spring 0 -> the car
--     collapses on every R (hydraulicSuspension.lua:32,154).
--   * pneumatics/autoLevelSuspension needs a pressureTank + PRESSURED beams.
--
-- Sensor: obj:getBeamLength() of the same two beams. The spring sits on the
-- hub node rh1* and is 0.957 vertical -> motion ratio ~1, 1 mm offset ~ 1 mm
-- body lift. L/R average cancels roll and anti-roll-bar effects.
--
-- Trailer: controller onCouplerAttached/onCouplerDetached (main.lua:384-400,
-- controller.lua:196-206) filtered to our own tow-hitch coupler nodes and to
-- foreign objects (obj2id ~= own id rejects tailgate/trunk couplers), plus a
-- 0.5 s reconcile poll over beamstate.attachedCouplers (beamstate.lua:1591,
-- written at :409, cleared without event at :1062).
--
-- Published electrics (every updateGFX, this controller is the only writer
-- while it exists; etk854hybrid_adaptiveSusp takes over trailerMode /
-- levelState only when no leveling instance is present):
--   etk854hybrid_trailerMode      0/1  effective trailer mode (contract)
--   etk854hybrid_trailerAttached  0/1  foreign coupler on our hitch
--   etk854hybrid_levelState       OFF|CALIB|LEVEL|RAISE|LOWER|HOLD|MAX|FAULT
--   etk854hybrid_levelOffsetMm    applied lift [mm]
--   etk854hybrid_levelErrorMm     error the loop levels [mm], + = too low: target -
--                                 filtered height, at standstill after driving minus
--                                 the stop wind-up latched on arrival (levelStep)
--   etk854hybrid_levelLoadKg      estimated extra rear axle load [kg]
--   etk854hybrid_trailerMassKg    trailer total mass [kg], 0 = unknown

local M = {}

M.type = "auxiliary"
-- Before etk854hybrid_adaptiveSusp (85): it reads etk854hybrid_trailerMode in
-- the same frame for its damper floor.
M.defaultOrder = 83

local abs = math.abs
local min = math.min
local max = math.max

-- All tunables in ONE table (LuaJIT: max 60 upvalues per function).
local CFG = {
  name = "etk854hybrid_selfLeveling",
  springBeamNames = {"spring_RR", "spring_RL"},
  enabled = true,
  -- $etk854_height_R: design equilibrium = L0 + h (chassis.jbeam preload
  -- (3993 + 515) / k + h). Measured 2.3 mm off -> the target is LEARNED at
  -- rest, the design length is the plausibility reference and the fallback.
  designHeightOffset = -0.010,
  springRate = 37000, -- $etk854_spring_R, only used for the load estimate
  -- wheel load per spring force (motion ratio incl. the arm bushings),
  -- measured 2026-09-30: 250 kg on the trunk nodes (~300 kg axle, ~1471 N
  -- per wheel) needed +1891 N spring force per side -> 0.78
  loadMotionRatio = 0.78,
  -- measured (N57S AWD): 250 kg on the trunk nodes sag the spring 39 mm and
  -- need ~51 mm of lift (the soft pre-stop below L0 - 35 mm and the arm
  -- bushings share the load, lift/length ratio ~0.7). 55 mm = 2035 N/side;
  -- 4508 + 2035 = 6543 N static << beamDeform 15000.
  maxRaise = 0.055,
  trailerLift = 0.0,
  rateStandstill = 0.004, -- m/s, premium-wagon compressor class (30 mm in ~8-10 s)
  rateDriving = 0.001,
  -- Loop Ki/(s(1+tau s)): tau*ki ~0.25 -> zeta ~1. Never raise ki without lowering tau.
  kiStandstill = 0.5,
  kiDriving = 0.06,
  tauStandstill = 0.5,
  tauDriving = 4.0,
  dbEnterStand = 0.006,
  dbExitStand = 0.002,
  dbEnterDrive = 0.008,
  dbExitDrive = 0.003,
  -- lift bleed: on the way DOWN (body above target) the loop keeps lowering
  -- until the error is inside this band or the lift is gone, so an unloaded
  -- car does not keep a phantom lift of a few mm (exit band 2 mm, ratio 0.6)
  bleedErr = 0.0003,
  standstillSpeed = 0.5, -- m/s wheelspeed
  driveAccelGate = 1.0, -- m/s^2 on sensors.gx2/gy2 (sensors.lua:31-32)
  standAccelGate = 0.5,
  quietTime = 1.0,
  -- Driving: the loop acts only after this long without an accel-gate event
  -- (and without a stand/drive transition). The 4 s height filter needs
  -- ~2.5 tau to forget what it held before - the wound-up standstill height
  -- when the car moves off, a regen or squat phase just under the gate -
  -- and a real load change is levelled at standstill anyway; the driving
  -- loop is only for slow drift (QA 2026-09-30: 1 s let a 15 km/h creep
  -- after a hard stop and 2 s regen coast-downs raise the empty car).
  quietTimeDrive = 10.0,
  calibSettle = 2.0,
  calibTimeout = 6.0,   -- no clean rest by then: provisional design target
  calibSanity = 0.025,  -- measured vs design plausibility window [m]
  calibMinComp = 0.003, -- the learned length must leave the spring compressed
  calibSpeed = 0.3,
  calibConverged = 0.001, -- |h - hF| before a settle sample counts
  applyEpsilon = 0.0001,
  sanityErr = 0.12,
  maxMargin = 0.0005,
  trailerCouplerTags = {tow_hitch = true, pintle = true, gooseneck_hitch = true},
  trailerMassQuery = true,
  massQueryTimeout = 3.0,
  pollInterval = 0.5,
  deformEps = 1e-4, -- beamstate.lua:1491 treats getBeamDeformation() > 0 as deformed
  qaMassNodes = {"r6rr", "r6ll", "r4rr", "r4ll"},
  msgCategory = "etk854hybrid_level"
}

-- All runtime state in ONE table.
local S = {
  state = "OFF",
  springs = {}, -- {cid, P0, Lref, L0, refLen, name}
  springCount = 0,
  targetDesign = 0,
  target = nil,
  calibT = 0,
  calibTotal = 0,
  calibDone = false,    -- a target exists (leveling runs)
  calibLearned = false, -- the target is the measured rest length (final)
  offset = 0,
  applied = 0,
  hF = nil,
  lastErr = 0,
  ctlErr = 0,       -- error the loop acts on (= lastErr minus the stop wind-up)
  stopErr = nil,    -- deviation latched when the car came to rest (see levelStep)
  moved = false,    -- has driven since spawn / reset
  adjusting = false,
  quietTimer = 0,
  wasStand = nil,   -- standstill flag of the previous frame (quietTimer reset)
  pollTimer = 0,
  hitchSet = {}, -- own coupler node cid -> true
  hitchCount = 0,
  objId = 0,
  manualTrailer = nil, -- nil = auto, true/false = driver pin
  trailerAttached = false,
  trailerObjId = nil,
  lastTrailerObjId = nil, -- survives reset(): re-coupling the same trailer keeps the pin
  trailerMode = false,
  lift = 0, -- trailer-mode lift currently in the target (m), excluded from loadKg
  trailerMassKg = 0,
  massQueryT = nil,
  qaNodes = {}, -- {cid, baseMass}
  qaMassKg = 0,
  info = {}
}

local function message(text)
  if guihooks and guihooks.message then
    guihooks.message(text, 3, CFG.msgCategory)
  end
end

local function toBool(val, default)
  if val == nil then
    return default
  end
  if type(val) == "boolean" then
    return val
  end
  local n = tonumber(val)
  if n ~= nil then
    return n ~= 0
  end
  return default
end

local function toSet(list, default)
  if type(list) ~= "table" then
    return default
  end
  local set = {}
  for _, tag in pairs(list) do
    set[tag] = true
  end
  return set
end

-- Absolute write on both springs: rest = refLen + offset, as a ratio of the
-- beam's ref length (see header). offset 0 = the jbeam rest length exactly.
local function setSpringRatios(offset)
  for i = 1, S.springCount do
    local sp = S.springs[i]
    obj:setPrecompressionRatio(sp.cid, (sp.refLen + offset) / sp.refLen)
  end
  S.applied = offset
end

local function applyQaMass()
  local n = #S.qaNodes
  if n == 0 then
    return
  end
  local per = S.qaMassKg / n
  for i = 1, n do
    local q = S.qaNodes[i]
    obj:setNodeMass(q.cid, q.baseMass + per)
  end
end

-- Effective mode = driver pin if set, else coupler state. Explicit if/else:
-- "pin ~= nil and pin or attached" would fall through for pin == false.
local function evalTrailerMode(text, quiet)
  local mode
  if S.manualTrailer ~= nil then
    mode = S.manualTrailer
  else
    mode = S.trailerAttached
  end
  if mode ~= S.trailerMode then
    S.trailerMode = mode
    if not quiet then
      message(text or (mode and "Trailer mode ON" or "Trailer mode OFF"))
    end
  elseif text and not quiet then
    message(text)
  end
end

-- Async total-mass query into the trailer VM (pattern:
-- couplings/containerLockFollow.lua:32-33). The trailer needs no mod code.
local function requestTrailerMass(obj2id)
  if not CFG.trailerMassQuery or not obj2id then
    return
  end
  local cmd = string.format(
    "local m=0 for _,n in pairs(v.data.nodes) do if n.cid then m=m+obj:getNodeMass(n.cid) end end " ..
      "obj:queueObjectLuaCommand(%d, string.format('controller.getControllerSafe(%%q).setTrailerMass(%%.0f)', %q, m))",
    S.objId, CFG.name)
  obj:queueObjectLuaCommand(obj2id, cmd)
  S.massQueryT = 0
end

local function setAttached(attached, obj2id, text)
  if attached == S.trailerAttached then
    return
  end
  S.trailerAttached = attached
  if attached then
    S.trailerObjId = obj2id
    S.trailerMassKg = 0
    if obj2id ~= S.lastTrailerObjId then
      -- New trailer: drop the pin so auto takes over (a driver "off" pin from a
      -- previous trailer must not survive into the next one).
      S.lastTrailerObjId = obj2id
      S.manualTrailer = nil
      evalTrailerMode(text or "Trailer detected - trailer mode ON")
    else
      -- Same trailer re-coupled after R (beamstate.lua:1062 dropped it without
      -- a detach event): keep the driver pin, no message.
      evalTrailerMode(nil, true)
    end
    requestTrailerMass(obj2id)
  else
    S.trailerObjId = nil
    S.lastTrailerObjId = nil -- genuine detach: the next coupling counts as new
    S.trailerMassKg = 0
    S.massQueryT = nil
    -- A manual "on" pin stays on after detaching (driver setting).
    evalTrailerMode(text or "Trailer disconnected")
  end
end

-- Reconcile against beamstate.attachedCouplers (events can be missed across
-- a reset: beamstate.lua:1062 clears the table without onCouplerDetached).
local function pollCouplers()
  local ac = beamstate and beamstate.attachedCouplers
  if not ac or S.hitchCount == 0 then
    if S.trailerAttached then
      setAttached(false)
    end
    return
  end
  local foundId = nil
  for cid in pairs(S.hitchSet) do
    local a = ac[cid]
    if a and a.obj2id and a.obj2id ~= S.objId then
      foundId = a.obj2id
      break
    end
  end
  if foundId and not S.trailerAttached then
    setAttached(true, foundId)
  elseif not foundId and S.trailerAttached then
    setAttached(false)
  end
end

-- Broken or plastically deformed spring -> freeze. Overwriting the rest ratio
-- of a deformed beam would silently erase the crash deformation.
local function checkFault()
  for i = 1, S.springCount do
    local cid = S.springs[i].cid
    local broken = obj:beamIsBroken(cid)
    local deform = obj:getBeamDeformation(cid)
    if broken or (type(deform) == "number" and deform > CFG.deformEps) then
      S.state = "FAULT"
      S.adjusting = false
      message("Self-leveling: fault (spring damaged) - leveling stopped")
      log("W", "etk854hybrid_selfLeveling", string.format("FAULT on %s: broken=%s deformation=%s", S.springs[i].name, tostring(broken), tostring(deform)))
      return
    end
  end
end

-- Axle load = 2 sides * k * dx * motion ratio; offset = compensated part,
-- positive error = rest. The deliberate trailer-mode lift is not load.
local function loadKg()
  return 2 * CFG.springRate * CFG.loadMotionRatio * (max(0, S.offset - S.lift) + max(0, S.ctlErr)) / 9.81
end

local function publish()
  local ev = electrics.values
  ev.etk854hybrid_trailerMode = S.trailerMode and 1 or 0
  ev.etk854hybrid_trailerAttached = S.trailerAttached and 1 or 0
  ev.etk854hybrid_levelState = S.state
  ev.etk854hybrid_levelOffsetMm = S.offset * 1000
  ev.etk854hybrid_levelErrorMm = S.ctlErr * 1000
  ev.etk854hybrid_levelLoadKg = loadKg()
  ev.etk854hybrid_trailerMassKg = S.trailerMassKg
end

-- Nominal level = the settled UNLADEN spring length, learned once per
-- vehicle load (spawn / Ctrl+R) and kept across resets (reset() keeps
-- calibDone / calibLearned / target). Learned only under clean conditions:
-- at rest, quiet, converged, no trailer, no manual trailer pin, no QA mass,
-- offset 0, and within calibSanity of the design length (a car spawned with
-- cargo must not be learned as "normal"). Without a clean rest within
-- calibTimeout (driven off at once) the design length is used provisionally
-- and replaced as soon as a clean rest with offset 0 occurs.
local function updateCalibration(dt, spd, h)
  S.calibTotal = S.calibTotal + dt
  local settled = spd < CFG.calibSpeed and S.quietTimer > 0 and abs(h - S.hF) < CFG.calibConverged
  local clean = not S.trailerAttached and S.manualTrailer ~= true and S.offset == 0 and S.qaMassKg == 0
  if settled and clean then
    S.calibT = S.calibT + dt
  else
    S.calibT = 0
  end
  local refLen = 0.5 * (S.springs[1].refLen + S.springs[2].refLen)
  if S.calibT >= CFG.calibSettle then
    local source
    if abs(S.hF - S.targetDesign) <= CFG.calibSanity and S.hF <= refLen - CFG.calibMinComp then
      S.target = S.hF
      source = "measured"
    else
      S.target = S.targetDesign
      source = "design (measured outside window)"
      log("W", "etk854hybrid_selfLeveling", string.format(
        "measured rest length %.4f m outside +-%.0f mm of design %.4f (ref %.4f) - using the design length",
        S.hF, CFG.calibSanity * 1000, S.targetDesign, refLen))
    end
    if not S.calibDone then S.state = "LEVEL" end
    S.calibDone, S.calibLearned = true, true
    log("I", "etk854hybrid_selfLeveling", string.format(
      "calibrated: target=%.4f m (%s) after %.1fs, design=%.4f m (delta %+.1f mm), ref=%.4f m",
      S.target, source, S.calibTotal, S.targetDesign, (S.hF - S.targetDesign) * 1000, refLen))
  elseif not S.calibDone and S.calibTotal >= CFG.calibTimeout then
    S.target = S.targetDesign
    S.calibDone = true
    S.state = "LEVEL"
    log("I", "etk854hybrid_selfLeveling", string.format(
      "no clean rest within %.0fs - provisional design target %.4f m, learning continues", CFG.calibTimeout, S.target))
  end
end

local function levelStep(dt, standstill)
  local lift = S.trailerMode and CFG.trailerLift or 0
  S.lift = lift
  local err = S.target + lift - S.hF
  S.lastErr = err
  -- Stop wind-up guard. A car braked to a standstill and held there (brake,
  -- P, parking brake) keeps its suspension wound up: tyre and brake
  -- pre-tension between the axles hold the rear 10-15 mm low with NO load
  -- until the wheels roll again (measured 2026-09-30, B58: hard stop -> the
  -- rear spring 12 mm short, the loop raised 16.6 mm and read 107 kg; rolling
  -- a few metres released it). Loading at rest is a change that happens WHILE
  -- standing, so once the car has come to rest after driving, the deviation
  -- present at that moment is latched and only the change from there is
  -- levelled at standstill. Driving clears the latch - the slow driving loop
  -- levels against the plain target, where no wind-up exists.
  -- The wind-up also relaxes while standing (a running engine's vibration
  -- during an N charge, a nudge): with none of our lift applied, a car that
  -- rose by itself takes the latch down with it - never below 0, the plain
  -- target (QA T26 N57 2026-09-30: after a 2-min N charge the stale latch
  -- read the relaxed car as 4.7 mm high and levelled 200 kg to a height
  -- 6 mm short).
  local errCtl = err
  if standstill then
    if S.moved and S.stopErr == nil and S.quietTimer >= CFG.quietTime then S.stopErr = max(0, err) end
    if S.stopErr then
      if S.offset <= CFG.applyEpsilon and err < S.stopErr then S.stopErr = max(0, err) end
      errCtl = err - S.stopErr
    end
  else
    S.stopErr = nil
    S.moved = true
  end
  S.ctlErr = errCtl
  err = errCtl
  local limit = CFG.maxRaise + lift

  local ki, rate, dbIn, dbOut
  if standstill then
    ki, rate, dbIn, dbOut = CFG.kiStandstill, CFG.rateStandstill, CFG.dbEnterStand, CFG.dbExitStand
  else
    ki, rate, dbIn, dbOut = CFG.kiDriving, CFG.rateDriving, CFG.dbEnterDrive, CFG.dbExitDrive
  end

  local d = 0
  local hold = abs(err) > CFG.sanityErr
    or S.quietTimer < (standstill and CFG.quietTime or CFG.quietTimeDrive)
  if not hold then
    local aerr = abs(err)
    if not S.adjusting and aerr > dbIn then
      S.adjusting = true
    elseif S.adjusting and aerr < dbOut and not (err < -CFG.bleedErr and S.offset > 0) then
      S.adjusting = false
    end
    if S.adjusting then
      local step = ki * err * dt
      local lim = rate * dt
      step = max(-lim, min(lim, step))
      -- Never below 0: the car is never lowered under the unloaded level.
      local o = max(0, min(limit, S.offset + step))
      d = o - S.offset
      S.offset = o
    end
  end

  local newState
  if S.offset >= limit - CFG.maxMargin and err > dbIn then
    newState = "MAX"
  elseif hold then
    newState = "HOLD"
  elseif S.adjusting and d > 0 then
    newState = "RAISE"
  elseif S.adjusting and d < 0 then
    newState = "LOWER"
  else
    newState = "LEVEL"
  end
  if newState == "MAX" and S.state ~= "MAX" then
    message("Self-leveling: overload - compensation limit reached")
  end
  S.state = newState

  -- Per-frame step <= 4 mm/s * 16 ms = 0.07 mm (< 3 N) -> no solver kick.
  if abs(S.offset - S.applied) > CFG.applyEpsilon or (S.offset == 0 and S.applied ~= 0) then
    setSpringRatios(S.offset)
  end
end

local function updateGFX(dt)
  S.pollTimer = S.pollTimer + dt
  local doPoll = S.pollTimer >= CFG.pollInterval
  if doPoll then
    S.pollTimer = 0
    pollCouplers()
  end
  if S.massQueryT then
    S.massQueryT = S.massQueryT + dt
    if S.massQueryT > CFG.massQueryTimeout then
      S.massQueryT = nil -- trailerMassKg stays 0 = unknown
    end
  end

  if S.state == "OFF" then
    publish()
    return
  end

  if doPoll and S.state ~= "FAULT" then
    checkFault()
  end

  local spd = abs(electrics.values.wheelspeed or 0)
  local standstill = spd < CFG.standstillSpeed
  local gate = standstill and CFG.standAccelGate or CFG.driveAccelGate
  local quiet = abs(sensors.gx2 or 0) < gate and abs(sensors.gy2 or 0) < gate
  if standstill ~= S.wasStand then
    -- moving off / coming to rest: what the filter holds belongs to the
    -- other regime (wound-up rest height, rolling height), start over
    S.quietTimer = 0
    S.wasStand = standstill
  end
  -- Launch control hold (etk854hybrid_launchControl publishes
  -- etk854hybrid_launchLevelHold = 1 while armed / preloading / READY, in
  -- the brake-throttle guard, during the generator unload and 2 s after):
  -- the converter preload against the brakes squats the rear ON PURPOSE
  -- ("das Auto soll sich hinten senken"). Read as load it would be levelled
  -- up during an 8 s hold, so the loop, the height filter and the stop latch
  -- (quietTimer) are frozen for as long as the flag is set.
  local lh = electrics.values.etk854hybrid_launchLevelHold
  local lcHold = lh == 1 or lh == true
  if lcHold then quiet = false end
  if quiet then
    S.quietTimer = S.quietTimer + dt
  else
    S.quietTimer = 0
  end

  local sp1, sp2 = S.springs[1], S.springs[2]
  local h = 0.5 * (obj:getBeamLength(sp1.cid) + obj:getBeamLength(sp2.cid))
  if S.hF == nil then
    S.hF = h
  elseif quiet then
    -- The height filter only integrates quiet samples. Tracking through a
    -- gated phase let the 4 s filter soak up the squat of every acceleration
    -- (10 mm after 3 s at 0.5 g): 1 s after the lift the loop still read
    -- > 8 mm, raised against a body that was already back up, and since the
    -- car is never lowered below the unladen level the lift only ratcheted
    -- up (QA full runs 2026-09-30: 6.6-6.8 mm lift on an empty car after
    -- 15 min of test driving, read as a phantom ~40 kg load).
    local tau = standstill and CFG.tauStandstill or CFG.tauDriving
    S.hF = S.hF + (h - S.hF) * min(1, dt / tau)
  end

  if S.state == "FAULT" then
    S.lastErr = (S.target or S.targetDesign) - S.hF
    S.ctlErr = S.lastErr
  elseif lcHold then
    -- frozen (see above): keep the applied offset, no calibration sample
    if S.calibDone then S.state = "HOLD" end
  else
    if not S.calibLearned then updateCalibration(dt, spd, h) end
    if S.calibDone then levelStep(dt, standstill) end
  end
  publish()
end

local function onCouplerAttached(nodeId, obj2id, obj2nodeId, attachSpeed, attachEnergy)
  if S.hitchSet[nodeId] and obj2id and obj2id ~= S.objId then
    setAttached(true, obj2id)
  end
end

-- Re-scan instead of forcing false: the HD combo part has two own couplers
-- (hyb_twb tow_hitch + hyb_twp pintle, etk854hybrid_towing.jbeam), releasing
-- one must not drop a trailer still held by the other. beamstate runs first
-- (main.lua:394-395), so its entry is already nil here.
local function onCouplerDetached(nodeId, obj2id, obj2nodeId, breakForce)
  if S.hitchSet[nodeId] and obj2id ~= S.objId and S.trailerAttached then
    pollCouplers()
  end
end

local function findSprings(names)
  local springs = {}
  for i, beamName in ipairs(names) do
    local found = 0
    for _, b in pairs(v.data.beams) do
      if b.name == beamName and b.cid then
        found = found + 1
        if found == 1 then
          local n1, n2 = v.data.nodes[b.id1], v.data.nodes[b.id2]
          local Lref = (n1 and n2) and vec3(n1.pos):distance(vec3(n2.pos)) or 0
          local P0 = tonumber(b.beamPrecompression)
          if P0 == nil and Lref > 0 then
            P0 = obj:getBeamRestLength(b.cid) / Lref
          end
          -- the length setPrecompressionRatio scales (see header); before any
          -- write it equals the rest length
          local refLen = obj:getBeamRefLength(b.cid)
          if type(refLen) ~= "number" or refLen <= 0 then refLen = (P0 or 1) * Lref end
          springs[i] = {cid = b.cid, P0 = P0 or 0, Lref = Lref, L0 = Lref, refLen = refLen, name = beamName}
        end
      end
    end
    if found ~= 1 then
      log("W", "etk854hybrid_selfLeveling", string.format("spring beam '%s' found %d times (need exactly 1)", beamName, found))
      return nil
    end
  end
  return springs
end

local function init(jbeamData)
  jbeamData = jbeamData or {}
  CFG.name = jbeamData.name or CFG.name
  CFG.springBeamNames = type(jbeamData.springBeamNames) == "table" and jbeamData.springBeamNames or {"spring_RR", "spring_RL"}
  CFG.enabled = toBool(jbeamData.enabled, true)
  CFG.designHeightOffset = tonumber(jbeamData.designHeightOffset) or -0.010
  CFG.springRate = tonumber(jbeamData.springRate) or 37000
  CFG.loadMotionRatio = tonumber(jbeamData.loadMotionRatio) or 0.78
  CFG.maxRaise = max(0, tonumber(jbeamData.maxRaise) or 0.055)
  CFG.trailerLift = max(0, tonumber(jbeamData.trailerLift) or 0)
  local rateMm = tonumber(jbeamData.rateStandstillMmps)
  CFG.rateStandstill = rateMm and rateMm / 1000 or (tonumber(jbeamData.rateStandstill) or 0.004)
  CFG.rateDriving = tonumber(jbeamData.rateDriving) or 0.001
  CFG.kiStandstill = tonumber(jbeamData.kiStandstill) or 0.5
  CFG.kiDriving = tonumber(jbeamData.kiDriving) or 0.06
  CFG.tauStandstill = max(0.05, tonumber(jbeamData.tauStandstill) or 0.5)
  CFG.tauDriving = max(0.05, tonumber(jbeamData.tauDriving) or 4.0)
  CFG.dbEnterStand = tonumber(jbeamData.deadbandEnterStand) or 0.006
  CFG.dbExitStand = tonumber(jbeamData.deadbandExitStand) or 0.002
  CFG.dbEnterDrive = tonumber(jbeamData.deadbandEnterDrive) or 0.008
  CFG.dbExitDrive = tonumber(jbeamData.deadbandExitDrive) or 0.003
  CFG.standstillSpeed = tonumber(jbeamData.standstillSpeed) or 0.5
  CFG.driveAccelGate = tonumber(jbeamData.driveAccelGate) or 1.0
  CFG.quietTime = tonumber(jbeamData.quietTime) or 1.0
  CFG.quietTimeDrive = tonumber(jbeamData.quietTimeDrive) or 10.0
  CFG.calibSettle = tonumber(jbeamData.calibSettle) or 2.0
  CFG.calibTimeout = tonumber(jbeamData.calibTimeout) or 6.0
  CFG.calibSanity = tonumber(jbeamData.calibSanity) or tonumber(jbeamData.calibWindow) or 0.025
  CFG.applyEpsilon = tonumber(jbeamData.applyEpsilon) or 0.0001
  CFG.trailerCouplerTags = toSet(jbeamData.trailerCouplerTags, CFG.trailerCouplerTags)
  CFG.trailerMassQuery = toBool(jbeamData.trailerMassQuery, true)
  CFG.pollInterval = tonumber(jbeamData.pollInterval) or 0.5
  if type(jbeamData.qaMassNodes) == "table" then
    CFG.qaMassNodes = jbeamData.qaMassNodes
  end

  S.objId = obj:getId()
  S.offset = 0
  S.applied = 0 -- physics already holds P0 from stage2 -> no write until the loop moves
  S.hF = nil
  S.lastErr = 0
  S.ctlErr = 0
  S.stopErr = nil
  S.moved = false
  S.adjusting = false
  S.quietTimer = 0
  S.wasStand = nil
  S.pollTimer = 0
  S.target = nil
  S.calibT = 0
  S.calibTotal = 0
  S.calibDone = false
  S.calibLearned = false
  S.manualTrailer = nil
  S.trailerAttached = false
  S.trailerObjId = nil
  S.lastTrailerObjId = nil
  S.lift = 0
  S.trailerMode = false
  S.trailerMassKg = 0
  S.massQueryT = nil
  S.qaMassKg = 0

  -- Own hitch nodes. beamstate.couplerCache (beamstate.lua:1086-1090) is built
  -- from the same node fields, but scanning v.data.nodes does not depend on
  -- the beamstate init order. Locked/welded couplers are structural, never a
  -- trailer.
  S.hitchSet = {}
  S.hitchCount = 0
  S.qaNodes = {}
  local qaWanted = toSet(CFG.qaMassNodes, {})
  for _, n in pairs(v.data.nodes) do
    local tag = n.couplerTag or n.tag
    if n.cid and tag and CFG.trailerCouplerTags[tag] and not n.couplerLock and not n.couplerWeld then
      S.hitchSet[n.cid] = true
      S.hitchCount = S.hitchCount + 1
    end
    if n.cid and n.name and qaWanted[n.name] then
      table.insert(S.qaNodes, {cid = n.cid, baseMass = obj:getNodeMass(n.cid)})
    end
  end

  S.springs = {}
  S.springCount = 0
  S.state = "OFF"
  local springs = findSprings(CFG.springBeamNames)
  if springs and #springs == 2 then
    local sane = true
    for i = 1, 2 do
      local sp = springs[i]
      if sp.P0 < 1.0 or sp.P0 > 2.0 or sp.Lref < 0.2 or sp.Lref > 0.5 then
        sane = false
        log("W", "etk854hybrid_selfLeveling", string.format("%s out of sanity range: P0=%.4f Lref=%.4f -> OFF", sp.name, sp.P0, sp.Lref))
      end
    end
    if sane then
      S.springs = springs
      S.springCount = 2
      S.targetDesign = 0.5 * (springs[1].L0 + springs[2].L0) + CFG.designHeightOffset
      if CFG.enabled then
        S.state = "CALIB"
      end
      log("I", "etk854hybrid_selfLeveling", string.format(
        "init: 2 springs, P0=%.4f/%.4f Lref=%.4f/%.4f rest=%.4f/%.4f ref=%.4f/%.4f targetDesign=%.4f hitches=%d qaNodes=%d enabled=%s",
        springs[1].P0, springs[2].P0, springs[1].Lref, springs[2].Lref,
        obj:getBeamRestLength(springs[1].cid), obj:getBeamRestLength(springs[2].cid),
        springs[1].refLen, springs[2].refLen,
        S.targetDesign, S.hitchCount, #S.qaNodes, tostring(CFG.enabled)))
    end
  else
    log("W", "etk854hybrid_selfLeveling", "selfLeveling: spring beams not found -> OFF")
  end
  publish()
end

-- controller.lua:773-776 calls reset(jbeamData) after the physics reset
-- (main.lua:434-471). Whether C++ restores the precompression ratio is
-- unverified -> write ratio 1 (= the jbeam rest length) explicitly. Learned
-- target, calibration and the driver pin survive (geometry is identical after
-- a reset).
local function reset(jbeamData)
  -- FAULT is cleared here too: the reset repairs broken/deformed beams.
  if S.springCount == 2 and S.state ~= "OFF" then
    setSpringRatios(0)
    if S.calibDone then
      S.state = "LEVEL"
    else
      S.state = "CALIB"
      S.calibT = 0
      S.calibTotal = 0
    end
  end
  S.offset = 0
  S.applied = 0
  S.hF = nil
  S.lastErr = 0
  S.ctlErr = 0
  S.stopErr = nil
  S.moved = false
  S.adjusting = false
  S.quietTimer = 0
  S.wasStand = nil
  S.pollTimer = 0
  -- beamstate.lua:1062 clears attachedCouplers without a detach event;
  -- core_trailerRespawn re-couples later and fires onCouplerAttached again.
  S.trailerAttached = false
  S.trailerObjId = nil -- lastTrailerObjId kept on purpose (pin survives R)
  S.lift = 0
  S.trailerMassKg = 0
  S.massQueryT = nil
  if S.manualTrailer ~= nil then
    S.trailerMode = S.manualTrailer
  else
    S.trailerMode = false
  end
  if S.qaMassKg > 0 then
    applyQaMass()
  end
  publish()
end

-- ---------------------------------------------------------------------------
-- Public API (contract: etk854hybrid_adaptiveSusp delegates here)
-- ---------------------------------------------------------------------------

-- nil = back to auto (coupler), true/false = pin.
local function setTrailerMode(b)
  if b == nil then
    S.manualTrailer = nil
  else
    S.manualTrailer = b and true or false
  end
  evalTrailerMode()
end

local function toggleTrailerMode()
  S.manualTrailer = not S.trailerMode
  evalTrailerMode()
end

local function getTrailerMode()
  return S.trailerMode
end

local function getLevelState()
  return S.state
end

local function getLevelInfo()
  local i = S.info
  i.state = S.state
  i.offsetMm = S.offset * 1000
  i.errorMm = S.ctlErr * 1000
  i.rawErrorMm = S.lastErr * 1000
  i.stopErrMm = S.stopErr and S.stopErr * 1000 or nil
  i.loadKg = loadKg()
  i.trailerMode = S.trailerMode
  i.trailerAttached = S.trailerAttached
  i.trailerMassKg = S.trailerMassKg
  i.manualTrailer = S.manualTrailer
  i.targetMm = S.target and S.target * 1000 or nil
  return i
end

-- Callback of the async query in requestTrailerMass (runs in our VM).
local function setTrailerMass(kg)
  if not S.trailerAttached then
    return
  end
  S.trailerMassKg = max(0, tonumber(kg) or 0)
  S.massQueryT = nil
  log("I", "etk854hybrid_selfLeveling", string.format("trailer mass %.0f kg (obj %s)", S.trailerMassKg, tostring(S.trailerObjId)))
end

-- QA stimulus only: spread extra mass over the trunk-floor nodes
-- (r6rr/r6ll y=1.97, r4rr/r4ll y=2.18 -> ~1.2x rear axle share).
local function qaAddRearMass(kg)
  S.qaMassKg = max(0, S.qaMassKg + (tonumber(kg) or 0))
  applyQaMass()
  log("I", "etk854hybrid_selfLeveling", string.format("QA rear mass now %.0f kg on %d nodes", S.qaMassKg, #S.qaNodes))
end

local function qaClearRearMass()
  S.qaMassKg = 0
  applyQaMass()
  log("I", "etk854hybrid_selfLeveling", "QA rear mass cleared")
end

M.init = init
M.reset = reset
M.updateGFX = updateGFX
M.onCouplerAttached = onCouplerAttached
M.onCouplerDetached = onCouplerDetached

M.setTrailerMode = setTrailerMode
M.toggleTrailerMode = toggleTrailerMode
M.getTrailerMode = getTrailerMode
M.getLevelState = getLevelState
M.getLevelInfo = getLevelInfo
M.setTrailerMass = setTrailerMass
M.qaAddRearMass = qaAddRearMass
M.qaClearRearMass = qaClearRearMass

return M
