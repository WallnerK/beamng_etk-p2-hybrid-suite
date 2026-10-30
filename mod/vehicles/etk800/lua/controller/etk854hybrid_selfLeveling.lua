-- ETK 854 ActiveHybrid - rear self-leveling + trailer detection
--
-- Air-spring-like rear level control (part etk854hybrid_spring_R): under rear
-- load the ride height is slowly restored to the learned unladen level by
-- raising the preload of spring_RR / spring_RL; the spring rate is unchanged.
--
-- Actuator: absolute (drift-free) rest-length write
--   obj:setPrecompressionRatio(cid, (refLen + offset) / refLen)
-- The ratio scales the beam's REF length (getBeamRefLength), not the node
-- distance; a node-distance ratio jacks the rear onto its rebound stops.
-- Not hydros (rest ratio centred at 1, loses the ~1.35 precompression),
-- hydraulicSuspension (its reset collapses the spring) or autoLevelSuspension
-- (needs a pressureTank + pressured beams).
-- Sensor: getBeamLength of the same beams (motion ratio ~1); the L/R mean
-- cancels roll.
--
-- Loop: target learned at rest (updateCalibration); integrator with deadband
-- hysteresis and rate limit (levelStep), fast at standstill, slow while
-- driving, frozen above an accel gate; offset >= 0 (never below unladen).
-- Trailer: coupler events on our hitch nodes (not tailgate / trunk couplers)
-- from foreign objects, plus a reconcile poll (a reset clears
-- beamstate.attachedCouplers silently).
--
-- Publishes (adaptiveSusp writes trailerMode / levelState only without us):
--   etk854hybrid_trailerMode (0/1), _trailerAttached (0/1),
--   _levelState OFF|CALIB|LEVEL|RAISE|LOWER|HOLD|MAX|FAULT,
--   _levelOffsetMm (lift), _levelErrorMm (+ = too low, after the stop latch),
--   _levelLoadKg (extra rear axle load), _trailerMassKg (0 = unknown)

local M = {}

M.type = "auxiliary"
-- Before adaptiveSusp (85), which reads trailerMode in the same frame.
M.defaultOrder = 83

local abs = math.abs
local min = math.min
local max = math.max

-- One table: LuaJIT 60-upvalue limit. Defaults; init overrides from jbeam.
local CFG = {
  name = "etk854hybrid_selfLeveling",
  springBeamNames = {"spring_RR", "spring_RL"},
  enabled = true,
  -- $etk854_height_R: design equilibrium = Lref + h; plausibility reference and
  -- fallback for the learned target
  designHeightOffset = -0.010,
  springRate = 37000, -- $etk854_spring_R, load estimate only
  loadMotionRatio = 0.78, -- wheel load per spring force (measured, incl. bushings)
  -- max lift [m]: ~250 kg in the trunk needs ~51 mm; spring force stays far
  -- below beamDeform
  maxRaise = 0.055,
  trailerLift = 0.0,
  rateStandstill = 0.004, -- m/s (30 mm in ~8-10 s)
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
  -- when lowering, keep going until inside this band or no lift (no phantom lift)
  bleedErr = 0.0003,
  standstillSpeed = 0.5, -- m/s wheelspeed
  driveAccelGate = 1.0, -- m/s^2 on sensors.gx2/gy2
  standAccelGate = 0.5,
  quietTime = 1.0,
  -- Driving loop waits this long after a gate event / regime change: the 4 s
  -- height filter needs ~2.5 tau to forget. It only corrects slow drift.
  quietTimeDrive = 10.0,
  calibSettle = 2.0,
  calibTimeout = 6.0,   -- then provisional design target
  calibSanity = 0.025,  -- measured vs design window [m]
  calibMinComp = 0.003, -- learned length must leave the spring compressed
  calibSpeed = 0.3,
  calibConverged = 0.001, -- |h - hF| for a settle sample
  applyEpsilon = 0.0001,
  sanityErr = 0.12,
  maxMargin = 0.0005,
  trailerCouplerTags = {tow_hitch = true, pintle = true, gooseneck_hitch = true},
  trailerMassQuery = true,
  massQueryTimeout = 3.0,
  pollInterval = 0.5,
  deformEps = 1e-4, -- beamstate treats deformation > 0 as deformed
  qaMassNodes = {"r6rr", "r6ll", "r4rr", "r4ll"},
  msgCategory = "etk854hybrid_level"
}

local S = {
  state = "OFF",
  springs = {}, -- {cid, P0, Lref, refLen, name}
  springCount = 0,
  targetDesign = 0,
  target = nil,
  calibT = 0,
  calibTotal = 0,
  calibDone = false,    -- a target exists (leveling runs)
  calibLearned = false, -- target is the measured rest length (final)
  offset = 0,
  applied = 0,
  hF = nil,
  lastErr = 0,
  ctlErr = 0,       -- lastErr minus the stop wind-up
  stopErr = nil,    -- deviation latched on coming to rest
  moved = false,    -- driven since spawn / reset
  adjusting = false,
  quietTimer = 0,
  wasStand = nil,   -- previous frame's standstill flag
  pollTimer = 0,
  hitchSet = {}, -- own coupler node cid -> true
  hitchCount = 0,
  objId = 0,
  manualTrailer = nil, -- nil = auto, true/false = driver pin
  trailerAttached = false,
  trailerObjId = nil,
  lastTrailerObjId = nil, -- survives reset: same trailer keeps the pin
  trailerMode = false,
  lift = 0, -- trailer-mode lift in the target [m], not counted as load
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

-- rest = refLen + offset; offset 0 = the jbeam rest length.
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

-- Driver pin if set, else coupler state ("pin ~= nil and pin or x" fails for false).
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

-- Async total-mass query in the trailer VM (as stock containerLockFollow.lua).
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
      -- New trailer: drop the pin, auto takes over.
      S.lastTrailerObjId = obj2id
      S.manualTrailer = nil
      evalTrailerMode(text or "Trailer detected - trailer mode ON")
    else
      -- Same trailer re-coupled after a reset: keep the pin, no message.
      evalTrailerMode(nil, true)
    end
    requestTrailerMass(obj2id)
  else
    S.trailerObjId = nil
    S.lastTrailerObjId = nil -- next coupling counts as new
    S.trailerMassKg = 0
    S.massQueryT = nil
    -- A manual "on" pin stays on after detaching.
    evalTrailerMode(text or "Trailer disconnected")
  end
end

-- Reconcile with beamstate.attachedCouplers (a reset clears it without a
-- detach event).
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

-- Broken or deformed spring -> freeze: a rest-ratio write would erase the
-- crash deformation.
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

-- Axle load = 2 sides * k * dx * motion ratio, dx = offset (minus the
-- trailer-mode lift) + remaining positive error.
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

-- Target = settled UNLADEN spring length, learned once per vehicle load and
-- kept across resets. Only learned clean (at rest, converged, no trailer / pin
-- / QA mass, offset 0) and within calibSanity of design, so cargo at spawn is
-- not learned as normal. After calibTimeout the design length is used until
-- a clean rest occurs.
local function updateCalibration(dt, spd, h)
  S.calibTotal = S.calibTotal + dt
  local settled = spd < CFG.calibSpeed and S.quietTimer > 0 and abs(h - S.hF) < CFG.calibConverged
  local clean = not S.trailerAttached and S.manualTrailer ~= true and S.offset == 0 and S.qaMassKg == 0
  if settled and clean then
    S.calibT = S.calibT + dt
  else
    S.calibT = 0
  end
  if S.calibT >= CFG.calibSettle then
    local refLen = 0.5 * (S.springs[1].refLen + S.springs[2].refLen)
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
  -- Stop wind-up: braked to rest, tyre / brake pre-tension holds the rear
  -- 10-15 mm low without load until it rolls again. So the deviation on
  -- coming to rest is latched and only the change from there is levelled;
  -- driving clears it. If the wind-up relaxes (no lift applied), the latch
  -- follows down, never below 0.
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
      -- never below the unladen level
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
    -- regime change: the filter holds the other regime's height
    S.quietTimer = 0
    S.wasStand = standstill
  end
  -- Launch-control hold: the converter preload against the brakes squats the
  -- rear on purpose; freeze loop, filter and stop latch while the flag is set.
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
    -- Quiet samples only: tracking gated phases soaks up every squat, and as
    -- the loop never lowers below unladen the lift ratchets into a phantom load.
    local tau = standstill and CFG.tauStandstill or CFG.tauDriving
    S.hF = S.hF + (h - S.hF) * min(1, dt / tau)
  end

  if S.state == "FAULT" then
    S.lastErr = (S.target or S.targetDesign) - S.hF
    S.ctlErr = S.lastErr
  elseif lcHold then
    -- frozen: keep the offset, no calibration sample
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

-- Re-scan: the HD combo part has two couplers, releasing one must not drop a
-- trailer held by the other (beamstate already removed the entry).
local function onCouplerDetached(nodeId, obj2id, obj2nodeId, breakForce)
  if S.hitchSet[nodeId] and obj2id ~= S.objId and S.trailerAttached then
    pollCouplers()
  end
end

-- Per-run state, cleared on spawn (init) and on every vehicle reset.
local function clearRuntimeState()
  S.offset = 0
  S.applied = 0 -- physics holds the jbeam rest length: no write until the loop moves
  S.hF = nil
  S.lastErr = 0
  S.ctlErr = 0
  S.stopErr = nil
  S.moved = false
  S.adjusting = false
  S.quietTimer = 0
  S.wasStand = nil
  S.pollTimer = 0
  -- a reset clears attachedCouplers silently; core_trailerRespawn re-couples later
  S.trailerAttached = false
  S.trailerObjId = nil -- lastTrailerObjId not touched: the pin survives a reset
  S.lift = 0
  S.trailerMassKg = 0
  S.massQueryT = nil
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
          -- the length setPrecompressionRatio scales (= rest length before any write)
          local refLen = obj:getBeamRefLength(b.cid)
          if type(refLen) ~= "number" or refLen <= 0 then refLen = (P0 or 1) * Lref end
          springs[i] = {cid = b.cid, P0 = P0 or 0, Lref = Lref, refLen = refLen, name = beamName}
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
  CFG.springBeamNames = type(jbeamData.springBeamNames) == "table" and jbeamData.springBeamNames or CFG.springBeamNames
  CFG.enabled = toBool(jbeamData.enabled, CFG.enabled)
  CFG.designHeightOffset = tonumber(jbeamData.designHeightOffset) or CFG.designHeightOffset
  CFG.springRate = tonumber(jbeamData.springRate) or CFG.springRate
  CFG.loadMotionRatio = tonumber(jbeamData.loadMotionRatio) or CFG.loadMotionRatio
  CFG.maxRaise = max(0, tonumber(jbeamData.maxRaise) or CFG.maxRaise)
  CFG.trailerLift = max(0, tonumber(jbeamData.trailerLift) or CFG.trailerLift)
  local rateMm = tonumber(jbeamData.rateStandstillMmps)
  CFG.rateStandstill = rateMm and rateMm / 1000 or (tonumber(jbeamData.rateStandstill) or CFG.rateStandstill)
  CFG.rateDriving = tonumber(jbeamData.rateDriving) or CFG.rateDriving
  CFG.kiStandstill = tonumber(jbeamData.kiStandstill) or CFG.kiStandstill
  CFG.kiDriving = tonumber(jbeamData.kiDriving) or CFG.kiDriving
  CFG.tauStandstill = max(0.05, tonumber(jbeamData.tauStandstill) or CFG.tauStandstill)
  CFG.tauDriving = max(0.05, tonumber(jbeamData.tauDriving) or CFG.tauDriving)
  CFG.dbEnterStand = tonumber(jbeamData.deadbandEnterStand) or CFG.dbEnterStand
  CFG.dbExitStand = tonumber(jbeamData.deadbandExitStand) or CFG.dbExitStand
  CFG.dbEnterDrive = tonumber(jbeamData.deadbandEnterDrive) or CFG.dbEnterDrive
  CFG.dbExitDrive = tonumber(jbeamData.deadbandExitDrive) or CFG.dbExitDrive
  CFG.standstillSpeed = tonumber(jbeamData.standstillSpeed) or CFG.standstillSpeed
  CFG.driveAccelGate = tonumber(jbeamData.driveAccelGate) or CFG.driveAccelGate
  CFG.quietTime = tonumber(jbeamData.quietTime) or CFG.quietTime
  CFG.quietTimeDrive = tonumber(jbeamData.quietTimeDrive) or CFG.quietTimeDrive
  CFG.calibSettle = tonumber(jbeamData.calibSettle) or CFG.calibSettle
  CFG.calibTimeout = tonumber(jbeamData.calibTimeout) or CFG.calibTimeout
  CFG.calibSanity = tonumber(jbeamData.calibSanity) or tonumber(jbeamData.calibWindow) or CFG.calibSanity
  CFG.applyEpsilon = tonumber(jbeamData.applyEpsilon) or CFG.applyEpsilon
  CFG.trailerCouplerTags = toSet(jbeamData.trailerCouplerTags, CFG.trailerCouplerTags)
  CFG.trailerMassQuery = toBool(jbeamData.trailerMassQuery, CFG.trailerMassQuery)
  CFG.pollInterval = tonumber(jbeamData.pollInterval) or CFG.pollInterval
  if type(jbeamData.qaMassNodes) == "table" then
    CFG.qaMassNodes = jbeamData.qaMassNodes
  end

  S.objId = obj:getId()
  clearRuntimeState()
  S.target = nil
  S.calibT = 0
  S.calibTotal = 0
  S.calibDone = false
  S.calibLearned = false
  S.manualTrailer = nil
  S.lastTrailerObjId = nil
  S.trailerMode = false
  S.qaMassKg = 0

  -- Own hitch nodes from v.data.nodes (independent of beamstate init order);
  -- locked / welded couplers are structural.
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
      S.targetDesign = 0.5 * (springs[1].Lref + springs[2].Lref) + CFG.designHeightOffset
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

-- After the physics reset; whether it restores the precompression ratio is
-- unverified, so ratio 1 is written explicitly. Learned target and driver
-- pin survive.
local function reset(jbeamData)
  -- also clears FAULT: the reset repairs the beams
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
  clearRuntimeState()
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
-- Public API (etk854hybrid_adaptiveSusp delegates here)
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

-- Callback of requestTrailerMass (runs in our VM).
local function setTrailerMass(kg)
  if not S.trailerAttached then
    return
  end
  S.trailerMassKg = max(0, tonumber(kg) or 0)
  S.massQueryT = nil
  log("I", "etk854hybrid_selfLeveling", string.format("trailer mass %.0f kg (obj %s)", S.trailerMassKg, tostring(S.trailerObjId)))
end

-- QA stimulus: extra mass on the trunk-floor nodes (~1.2x rear axle share).
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
