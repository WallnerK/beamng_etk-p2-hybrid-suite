-- batteryMonitor.lua - ETK 854 Hybrid - Phase 2
-- Owner of all battery state and derived capability scaling.
-- Implements PLAN_POWERTRAIN_COORDINATION.md section 3 (SOC bands, EV-Lockout,
-- cold derate) with the module interface from PLAN_SYSTEM_ARCHITECTURE.md.
--
-- Reads:  energyStorage.getStorage("mainBattery"), obj:getEnvironmentTemperature
-- Writes electrics channels:
--   etk854hybrid_batterySoc           (0..100, percent)
--   etk854hybrid_batteryFraction      (0..1)
--   etk854hybrid_batteryBand          ("CRIT"/"LOW1"/"LOW2"/"NORMAL_LO"/"NORMAL"/"TOP1"/"TOP2")
--   etk854hybrid_evAvailable          (0/1 - EV drive power gate)
--   etk854hybrid_regenAllowed         (0/1 - regen acceptance gate)
--   etk854hybrid_evPowerFraction      (0..1 - scale factor for EV drive)
--   etk854hybrid_regenAcceptFraction  (0..1 - scale factor for regen)
-- Writes ctx (consumed by torqueRequest / brakeCoord / iceManager later):
--   ctx.soc, ctx.socPercent, ctx.evPowerFraction, ctx.regenAcceptFraction
--   ctx.batteryBand, ctx.evLockoutActive, ctx.evColdDeratingFraction

local M = {}
M.name = "batteryMonitor"

-- ===========================================================
-- SOC Band Thresholds (per PLAN_POWERTRAIN_COORDINATION 3.1)
-- 0   5    10   15   20             95   98  100 %
-- |   |CRIT|LOW1|LOW2|  NORMAL       |TOP1|TOP2|
-- ===========================================================
local SOC_CRIT      = 0.05
local SOC_LOW1      = 0.10
local SOC_LOW2      = 0.15
local SOC_NORMAL_LO = 0.20
local SOC_TOP1      = 0.95
local SOC_TOP2      = 0.98

-- Lockout: once SOC drops below 5%, suppress EV drive for 30 s.
-- Release only when timer expires AND SOC has recovered above LOW1 (10%).
local EV_LOCKOUT_S       = 30
local EV_LOCKOUT_RELEASE = SOC_LOW1

-- Cold-battery derating proxy (no jbeam thermal model yet).
local COLD_DERATE_HOT_C  =   5    -- >=  5 degC: no derate
local COLD_DERATE_COLD_C = -20    -- <= -20 degC: max derate
local COLD_DERATE_MIN    = 0.5    -- floor at 50% available when frozen

-- ===========================================================
-- Module State (survives hot-require; reset clears it)
-- ===========================================================
local state = {
  soc              = 1.0,
  socPercent       = 100.0,
  band             = "NORMAL",
  evPowerFrac      = 1.0,
  regenAcceptFrac  = 1.0,
  coldDerate       = 1.0,
  evLockoutTimer   = 0,
  evLockoutActive  = false,
  lastSocSign      = "normal",
  coldRefreshTimer = 0,
  initialized      = false,
}

-- ===========================================================
-- Helpers
-- ===========================================================
local function clamp01(t)
  if t < 0 then return 0 end
  if t > 1 then return 1 end
  return t
end

local function lerp(a, b, t)
  return a + (b - a) * clamp01(t)
end

local function classifyBand(soc)
  if soc <= SOC_CRIT       then return "CRIT" end
  if soc <= SOC_LOW1       then return "LOW1" end
  if soc <= SOC_LOW2       then return "LOW2" end
  if soc <= SOC_NORMAL_LO  then return "NORMAL_LO" end
  if soc <  SOC_TOP1       then return "NORMAL" end
  if soc <  SOC_TOP2       then return "TOP1" end
  return "TOP2"
end

-- 0   below 5%
-- ramp 0->0.5 between 5..10%
-- ramp 0.5->0.8 between 10..15%
-- ramp 0.8->1.0 between 15..20%
-- 1.0 above 20%
local function evDrivePowerFraction(soc)
  if soc <= SOC_CRIT      then return 0.0 end
  if soc <= SOC_LOW1      then return lerp(0.0, 0.5, (soc - SOC_CRIT)  / (SOC_LOW1 - SOC_CRIT)) end
  if soc <= SOC_LOW2      then return lerp(0.5, 0.8, (soc - SOC_LOW1)  / (SOC_LOW2 - SOC_LOW1)) end
  if soc <= SOC_NORMAL_LO then return lerp(0.8, 1.0, (soc - SOC_LOW2)  / (SOC_NORMAL_LO - SOC_LOW2)) end
  return 1.0
end

-- 1.0 below 95%, tapers to 0 between 95..98%, 0 above 98%
local function regenAcceptFraction(soc)
  if soc >= SOC_TOP2 then return 0.0 end
  if soc >= SOC_TOP1 then return lerp(1.0, 0.0, (soc - SOC_TOP1) / (SOC_TOP2 - SOC_TOP1)) end
  return 1.0
end

-- Cold-battery derate. pcall-wrapped: getEnvironmentTemperature may not exist
-- in all BeamNG versions or contexts (e.g., during reset).
local function refreshColdDerating()
  local ok, ambK = pcall(function() return obj:getEnvironmentTemperature() end)
  if not ok or type(ambK) ~= "number" then return 1.0 end
  local ambC = ambK - 273.15
  if ambC >= COLD_DERATE_HOT_C  then return 1.0 end
  if ambC <= COLD_DERATE_COLD_C then return COLD_DERATE_MIN end
  return lerp(COLD_DERATE_MIN, 1.0,
              (ambC - COLD_DERATE_COLD_C) / (COLD_DERATE_HOT_C - COLD_DERATE_COLD_C))
end

local function readBatterySoc(fallback)
  local battery = energyStorage.getStorage("mainBattery")
  if battery and battery.remainingRatio then
    return battery.remainingRatio
  end
  return fallback or 1.0
end

-- Lockout state machine.
-- - Latches on first SOC<CRIT crossing (sets 30s timer + "crit" sign)
-- - Releases when timer hits 0 (driver may still see 0 EV power until SOC>CRIT)
-- - Sign returns to "normal" only after timer expired AND SOC>LOW1 (hysteresis)
local function updateLockout(soc, dt)
  if state.evLockoutTimer > 0 then
    state.evLockoutTimer = state.evLockoutTimer - dt
    if state.evLockoutTimer <= 0 then
      state.evLockoutTimer  = 0
      state.evLockoutActive = false
    end
  end

  if soc < SOC_CRIT and state.lastSocSign ~= "crit" then
    state.evLockoutTimer  = EV_LOCKOUT_S
    state.evLockoutActive = true
    state.lastSocSign     = "crit"
    if guihooks and guihooks.message then
      guihooks.message("ETK 854: Battery critical - EV disabled 30s", 5, "vehicle.hybridbattery")
    end
  elseif soc > EV_LOCKOUT_RELEASE and state.lastSocSign == "crit"
         and state.evLockoutTimer <= 0 then
    state.lastSocSign = "normal"
  end
end

-- ===========================================================
-- Channel & ctx publish (single source of truth per write)
-- ===========================================================
local function publish(ctx)
  electrics.values.etk854hybrid_batterySoc          = state.socPercent
  electrics.values.etk854hybrid_batteryFraction     = state.soc
  electrics.values.etk854hybrid_batteryBand         = state.band
  electrics.values.etk854hybrid_evAvailable         = (state.evPowerFrac > 0) and 1 or 0
  electrics.values.etk854hybrid_regenAllowed        = (state.regenAcceptFrac > 0) and 1 or 0
  electrics.values.etk854hybrid_evPowerFraction     = state.evPowerFrac
  electrics.values.etk854hybrid_regenAcceptFraction = state.regenAcceptFrac

  if ctx then
    ctx.soc                    = state.soc
    ctx.socPercent             = state.socPercent
    ctx.evPowerFraction        = state.evPowerFrac
    ctx.regenAcceptFraction    = state.regenAcceptFrac
    ctx.batteryBand            = state.band
    ctx.evLockoutActive        = state.evLockoutActive
    ctx.evColdDeratingFraction = state.coldDerate
  end
end

local function recomputeFromSoc(soc)
  state.soc        = soc
  state.socPercent = soc * 100
  state.band       = classifyBand(soc)

  local base = evDrivePowerFraction(soc)
  if state.evLockoutActive then base = 0 end
  state.evPowerFrac      = base * state.coldDerate
  state.regenAcceptFrac  = regenAcceptFraction(soc)
end

-- ===========================================================
-- Lifecycle (matches _stub.lua interface)
-- ===========================================================
function M.init(ctx)
  -- Read initial SOC; carry through transient nils during spawn.
  local soc = readBatterySoc(1.0)

  state.evLockoutTimer   = 0
  state.evLockoutActive  = (soc < SOC_CRIT)
  state.lastSocSign      = (soc < SOC_CRIT) and "crit" or "normal"
  state.coldRefreshTimer = 0
  state.coldDerate       = refreshColdDerating()

  recomputeFromSoc(soc)
  publish(ctx)
  state.initialized = true

  if ctx and ctx.dbg then
    ctx.dbg(1, "batteryMonitor",
            string.format("init soc=%.3f band=%s evFrac=%.2f regenFrac=%.2f cold=%.2f",
                          state.soc, state.band, state.evPowerFrac,
                          state.regenAcceptFrac, state.coldDerate))
  end
end

function M.updateGFX(ctx, dt)
  if not ctx or ctx.resetting then return end
  dt = dt or 0

  -- 1. SOC read (transient-nil safe)
  local soc = readBatterySoc(state.soc)

  -- 2. Cold derate refresh at ~1 Hz (ambient is slow-varying)
  state.coldRefreshTimer = state.coldRefreshTimer + dt
  if state.coldRefreshTimer >= 1.0 then
    state.coldRefreshTimer = 0
    state.coldDerate = refreshColdDerating()
  end

  -- 3. Lockout state machine
  updateLockout(soc, dt)

  -- 4. Derived bands + scaling
  recomputeFromSoc(soc)

  -- 5. Publish channels + ctx
  publish(ctx)
end

function M.updateFixedStep(ctx, dt)
  -- Battery state does not need physics-rate updates
end

function M.reset(ctx)
  local soc = readBatterySoc(1.0)
  state.evLockoutTimer   = 0
  state.evLockoutActive  = (soc < SOC_CRIT)
  state.lastSocSign      = (soc < SOC_CRIT) and "crit" or "normal"
  state.coldRefreshTimer = 0
  state.coldDerate       = refreshColdDerating()
  recomputeFromSoc(soc)
  publish(ctx)
end

-- ===========================================================
-- Public read accessors (for debug console / other modules)
-- ===========================================================
function M.getState()
  return {
    soc              = state.soc,
    socPercent       = state.socPercent,
    band             = state.band,
    evPowerFraction  = state.evPowerFrac,
    regenAcceptFraction = state.regenAcceptFrac,
    coldDerate       = state.coldDerate,
    evLockoutTimer   = state.evLockoutTimer,
    evLockoutActive  = state.evLockoutActive,
    initialized      = state.initialized,
  }
end

-- Expose pure helpers so other modules / tests can call without duplicating
M._classifyBand          = classifyBand
M._evDrivePowerFraction  = evDrivePowerFraction
M._regenAcceptFraction   = regenAcceptFraction
M.SOC_CRIT      = SOC_CRIT
M.SOC_LOW1      = SOC_LOW1
M.SOC_LOW2      = SOC_LOW2
M.SOC_NORMAL_LO = SOC_NORMAL_LO
M.SOC_TOP1      = SOC_TOP1
M.SOC_TOP2      = SOC_TOP2

return M
