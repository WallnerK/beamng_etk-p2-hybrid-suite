-- batteryMonitor.lua - ETK 854 Hybrid - battery state monitor
-- Battery state and derived scaling: SOC bands, EV lockout below SOC_CRIT,
-- cold derate of EV drive power, regen taper near full.
--
-- Reads:  energyStorage "mainBattery", obj:getEnvironmentTemperature
-- Writes: etk854hybrid_batterySoc [%], _batteryFraction, _batteryBand,
--   _evAvailable / _regenAllowed (0/1), _evPowerFraction / _regenAcceptFraction;
--   ctx.soc, socPercent, evPowerFraction, regenAcceptFraction, batteryBand,
--   evLockoutActive, evColdDeratingFraction

local M = {}
M.name = "batteryMonitor"

-- ===========================================================
-- SOC band thresholds
-- 0   5    10   15   20             95   98  100 %
-- |   |CRIT|LOW1|LOW2|  NORMAL       |TOP1|TOP2|
-- ===========================================================
local SOC_CRIT      = 0.05
local SOC_LOW1      = 0.10
local SOC_LOW2      = 0.15
local SOC_NORMAL_LO = 0.20
local SOC_TOP1      = 0.95
local SOC_TOP2      = 0.98

local EV_LOCKOUT_S       = 30         -- [s] no EV drive after dropping below CRIT
local EV_LOCKOUT_RELEASE = SOC_LOW1   -- re-arm above this SOC

-- cold derate from the ambient temperature (no battery thermal model)
local COLD_DERATE_HOT_C  =   5    -- no derate above
local COLD_DERATE_COLD_C = -20    -- full derate below
local COLD_DERATE_MIN    = 0.5    -- EV power factor at full derate

-- ===========================================================
-- State
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
  critLatched      = false,
  coldRefreshTimer = 0,
  initialized      = false,
}

-- ===========================================================
-- Helpers
-- ===========================================================
local U = require("controller/etk854hybrid/etk854hybrid_util")
local clamp01 = U.clamp01

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

-- 0 at 5 %, linear to 0.5 / 0.8 / 1.0 at 10 / 15 / 20 %
local function evDrivePowerFraction(soc)
  if soc <= SOC_CRIT      then return 0.0 end
  if soc <= SOC_LOW1      then return lerp(0.0, 0.5, (soc - SOC_CRIT)  / (SOC_LOW1 - SOC_CRIT)) end
  if soc <= SOC_LOW2      then return lerp(0.5, 0.8, (soc - SOC_LOW1)  / (SOC_LOW2 - SOC_LOW1)) end
  if soc <= SOC_NORMAL_LO then return lerp(0.8, 1.0, (soc - SOC_LOW2)  / (SOC_NORMAL_LO - SOC_LOW2)) end
  return 1.0
end

-- 1.0 below 95 %, linear to 0 at 98 %
local function regenAcceptFraction(soc)
  if soc >= SOC_TOP2 then return 0.0 end
  if soc >= SOC_TOP1 then return lerp(1.0, 0.0, (soc - SOC_TOP1) / (SOC_TOP2 - SOC_TOP1)) end
  return 1.0
end

-- pcall: getEnvironmentTemperature may be missing (version, reset)
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

-- Lockout latches (critLatched) on the first drop below CRIT, ends with the
-- timer; it can latch again only after the timer AND SOC > EV_LOCKOUT_RELEASE.
local function updateLockout(soc, dt)
  if state.evLockoutTimer > 0 then
    state.evLockoutTimer = state.evLockoutTimer - dt
    if state.evLockoutTimer <= 0 then
      state.evLockoutTimer  = 0
      state.evLockoutActive = false
    end
  end

  if soc < SOC_CRIT and not state.critLatched then
    state.evLockoutTimer  = EV_LOCKOUT_S
    state.evLockoutActive = true
    state.critLatched     = true
    if guihooks and guihooks.message then
      guihooks.message(string.format("ETK 854: Battery critical - EV disabled %ds", EV_LOCKOUT_S),
                       5, "vehicle.hybridbattery")
    end
  elseif soc > EV_LOCKOUT_RELEASE and state.critLatched
         and state.evLockoutTimer <= 0 then
    state.critLatched = false
  end
end

-- ===========================================================
-- Publish
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
-- Lifecycle
-- ===========================================================
-- re-sync lockout, cold derate and the derived values to the current SOC
local function resync(ctx)
  local soc = readBatterySoc(1.0)
  state.evLockoutTimer   = 0
  state.evLockoutActive  = (soc < SOC_CRIT)
  state.critLatched      = (soc < SOC_CRIT)
  state.coldRefreshTimer = 0
  state.coldDerate       = refreshColdDerating()
  recomputeFromSoc(soc)
  publish(ctx)
end

function M.init(ctx)
  resync(ctx)
  state.initialized = true

  if ctx and ctx.dbg then
    ctx.dbg(1, "batteryMonitor",
            string.format("init soc=%.3f band=%s evFrac=%.2f regenFrac=%.2f cold=%.2f",
                          state.soc, state.band, state.evPowerFrac,
                          state.regenAcceptFrac, state.coldDerate))
  end
end

function M.updateGFX(ctx, dt)
  if not ctx then return end
  dt = dt or 0

  -- keeps the last SOC while the storage is missing
  local soc = readBatterySoc(state.soc)

  -- cold derate at ~1 Hz
  state.coldRefreshTimer = state.coldRefreshTimer + dt
  if state.coldRefreshTimer >= 1.0 then
    state.coldRefreshTimer = 0
    state.coldDerate = refreshColdDerating()
  end

  updateLockout(soc, dt)
  recomputeFromSoc(soc)
  publish(ctx)
end

function M.updateFixedStep(ctx, dt)
end

function M.reset(ctx)
  resync(ctx)
end

-- ===========================================================
-- Accessors
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

-- pure helpers for other modules / tests
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
