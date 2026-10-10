-- sailing.lua - ETK 854 ActiveHybrid - coasting ("Segeln") state machine
--
-- Series behaviour it reproduces (reference hybrid / ECO PRO + COMFORT):
--   Pedal 0, brake 0, 25..160 km/h, not SPORT  ->  K0 OPENS, ICE ignition OFF,
--   gearbox stays engaged, E-motor delivers exactly 0 Nm (NO coast regen).
--   The car free-wheels. Touching the brake blends regen in on a ramp; touching
--   the throttle gives instant E-torque and only restarts the ICE when the
--   master policy asks for it (ctx.policyWantsIce: kick pedal, fast pedal,
--   speed ceiling, load integral) or the start is urgent (ctx.iceStartUrgent:
--   kickdown pedal). A moderate demand leaves the sail electrically and the
--   policy's load integral decides later, exactly as without the sail.
--
-- P1 rework: this module no longer touches K0, the starter or the ignition.
-- It only REQUESTS the ICE off/on; etk854hybrid_iceSequencer releases it
-- (FAST: share out, K0 open, ignition off after K0) and restarts it with a
-- rev-matched K0 engage (K0 slip start or starter, sync with K0 open, engage,
-- torque blend). The old drag start (K0 closing at 2.6/s against a stopped
-- engine + a flat 70 Nm feed-forward) is gone. Sailing also does not arm
-- after sporty driving (ctx.sportIndex > SAIL_SPORT_MAX): the engine is kept
-- hot and coupled for the next bend instead.
--
-- Contract (master VCU sets these on ctx BEFORE calling updateGFX):
--   ctx.deMode            "COMFORT" | "ECO_PRO" | "SPORT"
--   ctx.edriveMode        "AUTO_EDRIVE" | "MAX_EDRIVE" | "BATTERY_CONTROL"
--   ctx.sailingEnabled    0/1  ($sailingenabled jbeam variable)
--   ctx.regenAllowsSail   bool (false unless the regen selector is OFF)
--   ctx.throttleRaw       0..1 raw driver pedal
--   ctx.brakeRaw          0..1 raw brake pedal
--   ctx.speedKmh          absolute road speed
--   ctx.gearSign          -1 / 0 / +1
--   ctx.torqueReqNm       driver torque request at the motor shaft [Nm]
--   ctx.motorCapNm        currently available motor torque [Nm] (stage+SOC capped)
--   ctx.soc               0..1
--   ctx.iceOn             sequencer says the ICE participates (state ~= OFF)
--   ctx.iceEngageState    sequencer state OFF|CRANK|SYNC|ENGAGE|BLEND|ON|RELEASE
--   ctx.iceFireMs         request -> fired of the last start (sequencer)
--   ctx.iceRpm            engine rpm
--   ctx.iceIdleRPM        engine idle rpm (from powerCurves - NOT hard coded)
--   ctx.policyWantsIce    what the EV mode/DE policy would do without sailing
--   ctx.iceStartUrgent    the policy calls the pedal urgent (kickdown pedal)
--   ctx.k0Current         current K0 clutch ratio (0 open .. 1 closed)
--   ctx.sportIndex        driver sportiness 0..1 (etk854hybrid_sportIndex)
--
-- Outputs (written to ctx, applied by the master):
--   ctx.sailState         "DRIVE" | "SAIL_ENTER" | "SAIL" | "SAIL_EXIT"
--   ctx.sailActive        bool  (true in SAIL_ENTER + SAIL)
--   ctx.sailSuppressRegen bool  (kill ALL regen - true free-wheeling)
--   ctx.sailSuppressDrive bool  (kill motor drive torque)
--   ctx.sailIceRequest    nil | "off" | "on"   (the sequencer's arbiter obeys)
--   ctx.sailRestartUrgent bool  (restart with the URGENT start profile)
--   ctx.sailK0Override    always nil   } kept for compatibility only; the
--   ctx.sailStarterPulse  always false } sequencer is the sole owner of K0,
--   ctx.sailDragAssistNm  always 0     } starter and ignition
--
-- Publishes channels: etk854hybrid_sailState / _sailActive / _sailTime
-- (etk854hybrid_iceRestartActive / _iceRestartMs moved to the sequencer)

local M = {}
M.name = "sailing"

local max, min, abs = math.max, math.min, math.abs

-- ===========================================================
-- Entry / exit window (target spec section 3)
-- ===========================================================
local SAIL_SPEED_MIN_KMH   = 25.0
local SAIL_SPEED_MAX_KMH   = 160.0
-- Hysteresis band applied to the speed window on the way OUT, so a car sitting
-- exactly on 25.0 km/h does not chatter between SAIL and DRIVE.
local SAIL_SPEED_HYST_KMH  = 3.0
local SAIL_THROTTLE_MAX    = 0.02   -- "pedal released"
local SAIL_BRAKE_MAX       = 0.005  -- "brake released"
-- SOC floor: the K0 drag start needs motor torque, so never sail the battery
-- down to a point where the ICE cannot be restarted electrically. 0.06 -> 0.10
-- (2026-10-10): it sat below the VCU's forced-ICE threshold (SOC_ICE_FORCE
-- 0.08) and a sail-off (hard FAST release) won over the forced start, so a
-- regen-OFF coast could drain the pack further instead of charging it
local SAIL_SOC_MIN         = 0.10

-- Timing
local SAIL_ARM_S           = 0.35   -- entry conditions must hold this long
-- watchdog: force SAIL even if the sequencer's release lags. 1.2 s covers a
-- NORMAL-profile release (torque ramp + K0 open 3/s); sail entry itself
-- asks for the FAST profile (0.25 s + 5/s).
local SAIL_ENTER_MAX_S     = 1.20
local SAIL_MIN_S           = 2.0    -- min dwell for SOFT exits (speed/policy)
local SAIL_REARM_S         = 1.5    -- cooldown after any exit before re-arming
-- No sail after sporty driving: above this index the driver is between two
-- bends, and an engine that has to be restarted costs the next exit.
local SAIL_SPORT_MAX       = 0.40
-- Restart from a sail: SAIL_EXIT waits for the sequencer to reach BLEND/ON
-- (K0 closed onto a matched shaft), at most this long.
local RESTART_TIMEOUT_S    = 2.5

-- Throttle-tip exit: no load shortcut of its own any more. The former
-- EXIT_ICE_LOAD_FRAC (torqueReq > 0.92 x motor cap -> restart through the
-- arbiter bypass) skipped the policy's persistence (load integral) whenever
-- the driver pressed the pedal out of a sail.

-- Debug log throttle
local DBG_INTERVAL_S       = 1.0

-- ===========================================================
-- State
-- ===========================================================
local state = {
  phase          = "DRIVE",
  phaseTimer     = 0,
  armTimer       = 0,
  rearmTimer     = 0,
  sailTime       = 0,
  exitReason     = "",
  k0Cmd          = 1.0,
  restarting     = false,
  restartTimer   = 0,
  restartLatency = 0,
  restartUrgent  = false,
  dbgTimer       = 0,
}

-- ===========================================================
-- Helpers
-- ===========================================================
local function clamp01(v)
  if v < 0 then return 0 end
  if v > 1 then return 1 end
  return v
end

local function rampTo(current, target, rate, dt)
  if current < target then return min(target, current + rate * dt) end
  return max(target, current - rate * dt)
end

-- Sailing is a comfort/efficiency function: it is disabled in SPORT (driver
-- wants engine braking + instant ICE) and in BATTERY_CONTROL below target SOC
-- (the whole point of that mode is to run the ICE and charge).
local function modeAllowsSail(ctx)
  if (ctx.sailingEnabled or 1) == 0 then return false, "DISABLED" end
  -- The recuperation selector owns the lift-off character. Free-wheeling IS
  -- the OFF position of that selector, so any active regen level (LOW / MED /
  -- HIGH) has to win over sailing - otherwise the driver dials in one-pedal
  -- driving and the car keeps gliding with 0 Nm.
  if ctx.regenAllowsSail == false then return false, "REGEN" end
  if ctx.deMode == "SPORT" then return false, "SPORT" end
  if ctx.edriveMode == "BATTERY_CONTROL" and (ctx.soc or 1) < (ctx.batteryTargetSoc or 0.8) then
    return false, "CHARGING"
  end
  return true, ""
end

local function entryConditionsMet(ctx)
  local ok, why = modeAllowsSail(ctx)
  if not ok then return false, why end
  -- entry gate only: a sail in progress is not aborted by a corner
  if (ctx.sportIndex or 0) > SAIL_SPORT_MAX then return false, "SPORTY" end
  if (ctx.gearSign or 1) <= 0 then return false, "GEAR" end
  if (ctx.throttleRaw or 0) > SAIL_THROTTLE_MAX then return false, "THROTTLE" end
  if (ctx.brakeRaw or 0) > SAIL_BRAKE_MAX then return false, "BRAKE" end
  local v = ctx.speedKmh or 0
  if v < SAIL_SPEED_MIN_KMH or v > SAIL_SPEED_MAX_KMH then return false, "SPEED" end
  if (ctx.soc or 1) < SAIL_SOC_MIN then return false, "SOC" end
  return true, ""
end

-- Returns exitReason string or nil. Driver inputs exit immediately; the
-- "soft" reasons (speed window, mode/policy) respect SAIL_MIN_S.
local function checkExit(ctx, dwell)
  if (ctx.throttleRaw or 0) > SAIL_THROTTLE_MAX then return "THROTTLE" end
  if (ctx.brakeRaw or 0) > SAIL_BRAKE_MAX then return "BRAKE" end
  if (ctx.soc or 1) < SAIL_SOC_MIN then return "SOC" end
  if (ctx.gearSign or 1) <= 0 then return "GEAR" end

  local allowed, why = modeAllowsSail(ctx)
  if not allowed then return why end

  if dwell < SAIL_MIN_S then return nil end

  local v = ctx.speedKmh or 0
  if v < (SAIL_SPEED_MIN_KMH - SAIL_SPEED_HYST_KMH) then return "SLOW" end
  if v > (SAIL_SPEED_MAX_KMH + SAIL_SPEED_HYST_KMH) then return "FAST" end
  return nil
end

-- After a sail phase, does the ICE have to come back?
local function needsIceAfterSail(ctx, reason)
  -- pure-EV (MAX EV with evOnly): never restart the ICE from a sail exit;
  -- the demand/FAST shortcuts below would otherwise bypass the master policy
  if ctx.evOnly then return false end
  if ctx.policyWantsIce then return true end
  if reason == "SOC" or reason == "CHARGING" or reason == "SPORT" or reason == "FAST" then
    return true
  end
  if reason == "THROTTLE" then
    -- policyWantsIce is checked above; this frame's policy ran before us
    return ctx.iceStartUrgent == true
  end
  -- BRAKE / SLOW / GEAR: stay electric, the master's normal policy decides later.
  return false
end

local function publish(ctx)
  electrics.values.etk854hybrid_sailState        = state.phase
  electrics.values.etk854hybrid_sailActive       = (state.phase == "SAIL" or state.phase == "SAIL_ENTER") and 1 or 0
  electrics.values.etk854hybrid_sailTime         = state.sailTime
end

local function setCtxOutputs(ctx, suppressRegen, suppressDrive, iceRequest)
  ctx.sailState         = state.phase
  ctx.sailActive        = (state.phase == "SAIL" or state.phase == "SAIL_ENTER")
  ctx.sailSuppressRegen = suppressRegen
  ctx.sailSuppressDrive = suppressDrive
  ctx.sailIceRequest    = iceRequest
  ctx.sailRestartUrgent = (iceRequest == "on") and state.restartUrgent or false
  -- compatibility fields, the sequencer owns K0 / starter / drag torque
  ctx.sailK0Override    = nil
  ctx.sailStarterPulse  = false
  ctx.sailDragAssistNm  = 0
  ctx.sailExitReason    = state.exitReason
end

-- Restart urgency out of a sail: the policy calls the pedal urgent
-- (kickdown pedal), or the battery / speed window forces the engine.
local function restartIsUrgent(ctx, reason)
  if reason == "SOC" or reason == "FAST" then return true end
  if reason == "THROTTLE" then return ctx.iceStartUrgent == true end
  return false
end

local function transition(newPhase)
  state.phase = newPhase
  state.phaseTimer = 0
end

-- ===========================================================
-- Lifecycle
-- ===========================================================
function M.init(ctx)
  state.phase          = "DRIVE"
  state.phaseTimer     = 0
  state.armTimer       = 0
  state.rearmTimer     = 0
  state.sailTime       = 0
  state.exitReason     = ""
  state.k0Cmd          = 1.0
  state.restarting     = false
  state.restartTimer   = 0
  state.restartLatency = 0
  state.restartUrgent  = false
  state.dbgTimer       = 0

  setCtxOutputs(ctx, false, false, nil)
  publish(ctx)

  if ctx and ctx.dbg then
    ctx.dbg(1, "sailing", string.format(
      "init window=%.0f..%.0f km/h minDwell=%.1fs rearm=%.1fs sportMax=%.2f (ICE via sequencer)",
      SAIL_SPEED_MIN_KMH, SAIL_SPEED_MAX_KMH, SAIL_MIN_S, SAIL_REARM_S, SAIL_SPORT_MAX))
  end
end

function M.initStage2(ctx) end

function M.updateGFX(ctx, dt)
  if not ctx or ctx.resetting then return end
  dt = dt or 0
  state.phaseTimer = state.phaseTimer + dt
  if state.rearmTimer > 0 then state.rearmTimer = max(0, state.rearmTimer - dt) end

  local suppressRegen, suppressDrive = false, false
  local iceRequest = nil

  -- ---------------------------------------------------------
  -- DRIVE: watch for the entry window
  -- ---------------------------------------------------------
  if state.phase == "DRIVE" then
    state.sailTime = 0
    state.restarting = false
    state.k0Cmd = ctx.k0Current or 1.0

    local ok = entryConditionsMet(ctx)
    if ok and state.rearmTimer <= 0 then
      state.armTimer = state.armTimer + dt
      if state.armTimer >= SAIL_ARM_S then
        state.armTimer = 0
        state.exitReason = ""
        state.k0Cmd = ctx.k0Current or 1.0
        transition("SAIL_ENTER")
        if ctx.dbg then ctx.dbg(2, "sailing", string.format(
          "SAIL_ENTER v=%.0fkm/h soc=%.2f", ctx.speedKmh or 0, ctx.soc or 0)) end
      end
    else
      state.armTimer = 0
    end

  -- ---------------------------------------------------------
  -- SAIL_ENTER: ask the sequencer for a FAST release (share out, K0 open,
  -- ignition off after K0), already free-wheeling
  -- ---------------------------------------------------------
  elseif state.phase == "SAIL_ENTER" then
    suppressRegen, suppressDrive = true, true
    iceRequest = "off"
    state.k0Cmd = ctx.k0Current or 0

    local abort = checkExit(ctx, SAIL_MIN_S)  -- during entry, any trigger aborts
    if abort then
      state.exitReason = abort
      state.restarting = needsIceAfterSail(ctx, abort)
      state.restartUrgent = state.restarting and restartIsUrgent(ctx, abort)
      state.restartTimer = 0
      -- Release the suppressions in the SAME frame the trigger is seen so the
      -- pedal-to-torque latency stays inside the < 100 ms spec target.
      suppressRegen, suppressDrive = false, false
      if state.restarting then iceRequest = "on" else iceRequest = nil end
      transition("SAIL_EXIT")
    elseif (ctx.iceEngageState or "OFF") == "OFF" or state.phaseTimer >= SAIL_ENTER_MAX_S then
      state.sailTime = 0
      transition("SAIL")
    end

  -- ---------------------------------------------------------
  -- SAIL: true free-wheel. Zero drive torque AND zero regen.
  -- ---------------------------------------------------------
  elseif state.phase == "SAIL" then
    suppressRegen, suppressDrive = true, true
    iceRequest = "off"
    state.k0Cmd = ctx.k0Current or 0
    state.sailTime = state.sailTime + dt

    local reason = checkExit(ctx, state.sailTime)
    if reason then
      state.exitReason = reason
      state.restarting = needsIceAfterSail(ctx, reason)
      state.restartUrgent = state.restarting and restartIsUrgent(ctx, reason)
      state.restartTimer = 0
      -- Same-frame release: the motor answers the pedal / the regen ramp starts
      -- immediately instead of one frame later.
      suppressRegen, suppressDrive = false, false
      if state.restarting then iceRequest = "on" end
      transition("SAIL_EXIT")
      if ctx.dbg then ctx.dbg(2, "sailing", string.format(
        "SAIL_EXIT reason=%s after %.1fs restartICE=%s",
        reason, state.sailTime, tostring(state.restarting))) end
    end

  -- ---------------------------------------------------------
  -- SAIL_EXIT: hand back to DRIVE. Two flavours:
  --   * no ICE needed  -> release the suppressions, electric driving, done.
  --   * ICE needed     -> keep requesting it until the sequencer has it
  --                       coupled (BLEND/ON) - crank, rev-match with K0 open
  --                       and the K0 engage all happen in the sequencer.
  -- ---------------------------------------------------------
  elseif state.phase == "SAIL_EXIT" then
    -- Drive torque and regen are released immediately: the motor answers the
    -- pedal in the same frame the driver touches it (< 100 ms spec target),
    -- and a brake tip goes through the master's regen ramp, not a step.
    suppressRegen, suppressDrive = false, false

    if state.restarting then
      iceRequest = "on"
      state.restartTimer = state.restartTimer + dt
      state.k0Cmd = ctx.k0Current or 0
      local es = ctx.iceEngageState or "OFF"
      local coupled = (es == "BLEND" or es == "ON")
      if coupled or state.restartTimer >= RESTART_TIMEOUT_S then
        -- request -> fired, as the old drag start reported it (QA T11)
        state.restartLatency = (coupled and ctx.iceFireMs and ctx.iceFireMs > 0)
          and (ctx.iceFireMs / 1000) or state.restartTimer
        state.restarting = false
        state.restartUrgent = false
        state.rearmTimer = SAIL_REARM_S
        transition("DRIVE")
        if ctx.dbg then ctx.dbg(1, "sailing", string.format(
          "ICE restart %s: fire %.0f ms, coupled after %.0f ms (rpm=%.0f)",
          coupled and "complete" or "TIMEOUT", state.restartLatency * 1000,
          state.restartTimer * 1000, ctx.iceRpm or 0)) end
      end
    else
      -- Electric exit: nothing to sequence, let the master resume control.
      state.k0Cmd = ctx.k0Current or 0
      state.rearmTimer = SAIL_REARM_S
      transition("DRIVE")
    end
  end

  setCtxOutputs(ctx, suppressRegen, suppressDrive, iceRequest)
  publish(ctx)

  -- 1 Hz diagnostic, only above debugLogLevel 2
  state.dbgTimer = state.dbgTimer + dt
  if (ctx.debugLevel or 0) >= 3 and state.dbgTimer >= DBG_INTERVAL_S then
    state.dbgTimer = 0
    log("I", "etk854hybrid.sailing", string.format(
      "[SAIL] phase=%s t=%.1f v=%.0f thr=%.2f brk=%.2f k0=%.2f ice=%s reason=%s",
      state.phase, state.sailTime, ctx.speedKmh or 0, ctx.throttleRaw or 0,
      ctx.brakeRaw or 0, state.k0Cmd, tostring(ctx.iceOn), state.exitReason))
  end
end

function M.updateFixedStep(ctx, dt) end

function M.reset(ctx)
  M.init(ctx)
end

function M.getState()
  return {
    phase          = state.phase,
    sailTime       = state.sailTime,
    exitReason     = state.exitReason,
    restarting     = state.restarting,
    restartLatency = state.restartLatency,
    k0Cmd          = state.k0Cmd,
    rearmTimer     = state.rearmTimer,
  }
end

-- Exposed for QA / console inspection
M.SAIL_SPEED_MIN_KMH = SAIL_SPEED_MIN_KMH
M.SAIL_SPEED_MAX_KMH = SAIL_SPEED_MAX_KMH
M.SAIL_MIN_S         = SAIL_MIN_S

return M
