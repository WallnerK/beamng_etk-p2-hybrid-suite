-- sailing.lua - ETK 854 ActiveHybrid - sailing (engine-off coasting) state machine
--
-- Not SPORT, pedals released, 25..160 km/h: K0 opens, ICE off, gearbox stays
-- engaged, E-machine at 0 Nm (no coast regen) - the car free-wheels. The brake
-- blends regen in; the throttle gives instant E-torque and restarts the ICE
-- only if the VCU policy wants it or the pedal is urgent (kickdown).
-- Only REQUESTS the ICE off / on; the sequencer owns K0, starter and ignition.
-- No entry after sporty driving: the ICE stays coupled for the next bend.
--
-- Phases: DRIVE -> SAIL_ENTER -> SAIL -> SAIL_EXIT -> DRIVE (ENTER may abort to EXIT)
-- Inputs (ctx): deMode, edriveMode, batteryTargetSoc, evOnly, sailingEnabled,
--   regenAllowsSail, throttleRaw, brakeRaw, speedKmh, gearSign (-1/0/1), soc,
--   k0Current (0 open..1), iceEngageState, iceFireMs, policyWantsIce (VCU ICE
--   decision without sailing), iceStartUrgent, sportIndex (0..1), tsaFreeze
--   (blocks entry only); iceOn, iceRpm (debug log).
-- Outputs (ctx): sailState, sailActive (ENTER + SAIL), sailSuppressRegen,
--   sailSuppressDrive, sailIceRequest (nil | "off" | "on"), sailRestartUrgent.
--   The exit reason is in M.getState().exitReason.
-- Electrics: etk854hybrid_sailState / _sailActive / _sailTime

local M = {}
M.name = "sailing"

local max = math.max

-- ===========================================================
-- Entry / exit window
-- ===========================================================
local SAIL_SPEED_MIN_KMH   = 25.0
local SAIL_SPEED_MAX_KMH   = 160.0
local SAIL_SPEED_HYST_KMH  = 3.0    -- exit hysteresis of the speed window
local SAIL_THROTTLE_MAX    = 0.02
local SAIL_BRAKE_MAX       = 0.005
-- SOC floor: a K0 restart needs motor torque. Keep above the VCU's
-- SOC_ICE_FORCE: a sail-off request wins over the forced start, so a lower
-- floor would let a regen-OFF coast drain the pack.
local SAIL_SOC_MIN         = 0.10

-- Timing [s]
local SAIL_ARM_S           = 0.35   -- entry conditions must hold this long
local SAIL_ENTER_MAX_S     = 1.20   -- watchdog: SAIL anyway (covers a NORMAL release)
local SAIL_MIN_S           = 2.0    -- min sail time before a speed-window exit
local SAIL_REARM_S         = 1.5    -- cooldown after an exit
local SAIL_SPORT_MAX       = 0.40   -- sportIndex above: no entry
local RESTART_TIMEOUT_S    = 2.5    -- SAIL_EXIT waits for BLEND / ON at most

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
-- Off in SPORT (engine braking, instant ICE) and in BATTERY CONTROL below the
-- target SOC (that mode runs the ICE to charge).
local function modeAllowsSail(ctx)
  if (ctx.sailingEnabled or 1) == 0 then return false, "DISABLED" end
  -- free-wheeling is the OFF position of the regen selector: any regen level wins
  if ctx.regenAllowsSail == false then return false, "REGEN" end
  if ctx.deMode == "SPORT" then return false, "SPORT" end
  if ctx.edriveMode == "BATTERY_CONTROL" and (ctx.soc or 1) < (ctx.batteryTargetSoc or 0.8) then
    return false, "CHARGING"
  end
  return true, ""
end

local function entryConditionsMet(ctx)
  if ctx.tsaFreeze then return false, "TSA" end
  local ok, why = modeAllowsSail(ctx)
  if not ok then return false, why end
  -- entry only: a corner does not abort a running sail
  if (ctx.sportIndex or 0) > SAIL_SPORT_MAX then return false, "SPORTY" end
  if (ctx.gearSign or 1) <= 0 then return false, "GEAR" end
  if (ctx.throttleRaw or 0) > SAIL_THROTTLE_MAX then return false, "THROTTLE" end
  if (ctx.brakeRaw or 0) > SAIL_BRAKE_MAX then return false, "BRAKE" end
  local v = ctx.speedKmh or 0
  if v < SAIL_SPEED_MIN_KMH or v > SAIL_SPEED_MAX_KMH then return false, "SPEED" end
  if (ctx.soc or 1) < SAIL_SOC_MIN then return false, "SOC" end
  return true, ""
end

-- Returns the exit reason or nil. Pedals, SOC, gear and mode exit at once;
-- the speed window (with hysteresis) only after the dwell SAIL_MIN_S.
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

-- Does the ICE have to come back after the sail?
local function needsIceAfterSail(ctx, reason)
  -- pure EV: never; the shortcuts below would bypass the VCU policy
  if ctx.evOnly then return false end
  if ctx.policyWantsIce then return true end
  if reason == "SOC" or reason == "CHARGING" or reason == "SPORT" or reason == "FAST" then
    return true
  end
  if reason == "THROTTLE" then
    -- the policy keeps its persistence; only an urgent pedal restarts here
    return ctx.iceStartUrgent == true
  end
  -- BRAKE / SLOW / GEAR: stay electric, the policy decides later
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
end

-- urgent restart: kickdown pedal, or forced by SOC / speed window
local function restartIsUrgent(ctx, reason)
  if reason == "SOC" or reason == "FAST" then return true end
  if reason == "THROTTLE" then return ctx.iceStartUrgent == true end
  return false
end

local function transition(newPhase)
  state.phase = newPhase
  state.phaseTimer = 0
end

-- latch the exit reason and whether (and how urgently) the ICE must restart
local function beginExit(ctx, reason)
  state.exitReason = reason
  state.restarting = needsIceAfterSail(ctx, reason)
  state.restartUrgent = state.restarting and restartIsUrgent(ctx, reason)
  state.restartTimer = 0
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
  if not ctx then return end
  dt = dt or 0
  state.phaseTimer = state.phaseTimer + dt
  if state.rearmTimer > 0 then state.rearmTimer = max(0, state.rearmTimer - dt) end
  -- diagnostic only (debug log / getState)
  state.k0Cmd = ctx.k0Current or ((state.phase == "DRIVE") and 1.0 or 0)

  local suppressRegen, suppressDrive = false, false
  local iceRequest = nil

  -- DRIVE: watch for the entry window
  if state.phase == "DRIVE" then
    state.sailTime = 0
    state.restarting = false

    local ok = entryConditionsMet(ctx)
    if ok and state.rearmTimer <= 0 then
      state.armTimer = state.armTimer + dt
      if state.armTimer >= SAIL_ARM_S then
        state.armTimer = 0
        state.exitReason = ""
        transition("SAIL_ENTER")
        if ctx.dbg then ctx.dbg(2, "sailing", string.format(
          "SAIL_ENTER v=%.0fkm/h soc=%.2f", ctx.speedKmh or 0, ctx.soc or 0)) end
      end
    else
      state.armTimer = 0
    end

  -- SAIL_ENTER: free-wheeling while the sequencer releases the ICE (FAST)
  elseif state.phase == "SAIL_ENTER" then
    suppressRegen, suppressDrive = true, true
    iceRequest = "off"

    local abort = checkExit(ctx, SAIL_MIN_S)  -- any trigger aborts the entry
    if abort then
      beginExit(ctx, abort)
      -- release in the same frame (see SAIL_EXIT)
      suppressRegen, suppressDrive = false, false
      if state.restarting then iceRequest = "on" else iceRequest = nil end
      transition("SAIL_EXIT")
    elseif (ctx.iceEngageState or "OFF") == "OFF" or state.phaseTimer >= SAIL_ENTER_MAX_S then
      state.sailTime = 0
      transition("SAIL")
    end

  -- SAIL: free-wheel, zero drive torque and zero regen
  elseif state.phase == "SAIL" then
    suppressRegen, suppressDrive = true, true
    iceRequest = "off"
    state.sailTime = state.sailTime + dt

    local reason = checkExit(ctx, state.sailTime)
    if reason then
      beginExit(ctx, reason)
      suppressRegen, suppressDrive = false, false
      if state.restarting then iceRequest = "on" end
      transition("SAIL_EXIT")
      if ctx.dbg then ctx.dbg(2, "sailing", string.format(
        "SAIL_EXIT reason=%s after %.1fs restartICE=%s",
        reason, state.sailTime, tostring(state.restarting))) end
    end

  -- SAIL_EXIT: drive and regen released at once (the motor answers the pedal
  -- this frame, a brake tip goes through the VCU's regen ramp). If the ICE is
  -- needed, request it until the sequencer has it coupled (BLEND / ON).
  elseif state.phase == "SAIL_EXIT" then
    suppressRegen, suppressDrive = false, false

    if state.restarting then
      iceRequest = "on"
      state.restartTimer = state.restartTimer + dt
      local es = ctx.iceEngageState or "OFF"
      local coupled = (es == "BLEND" or es == "ON")
      if coupled or state.restartTimer >= RESTART_TIMEOUT_S then
        -- request -> fired, else the time waited
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
      -- electric exit
      state.rearmTimer = SAIL_REARM_S
      transition("DRIVE")
    end
  end

  setCtxOutputs(ctx, suppressRegen, suppressDrive, iceRequest)
  publish(ctx)

  -- 1 Hz diagnostic at debugLevel >= 3
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
