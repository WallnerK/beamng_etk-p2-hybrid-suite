-- etk854hybrid_sportIndex.lua - ETK 854 ActiveHybrid - driver sportiness index
--
-- Index S in [0, 1] that keeps the ICE on after sporty driving: it lowers the
-- policy's start thresholds, stretches the ICE-off hold and scales the start
-- profile and limiter rise; it never starts the ICE by itself.
--
--   e = max(u_dem, W_RATE*u_rate, W_BRAKE*u_brake, W_LAT*u_lat)   (0 in N / P)
--   S rises toward e with tau 1.2 s and decays with tau 8..90 s set by the memory
--   window W (grows while e > 0.5, shrinks while S < 0.2); decay x4 slower at
--   standstill without pedal (red light between bends), frozen while braking
--   hard or cornering.
--
--   u_dem   smoothstep of the raw pedal over U0..U1 (before the mode map, so
--           mode-neutral); with a split map (ctx.pedalJoin > 0) and siU1 in
--           ctx.deModeCfg (ECO PRO): join..siU1 and rate weight siWRate, so
--           holding the EV zone or tapping the pedal does not count
--   u_rate  filtered pedal rate 1.0..4.0 /s
--   u_brake brake 0.35..0.80 above 30 km/h (W_BRAKE 0: a normal stop must not raise S)
--   u_lat   |sensors.gx2| (smoothed lateral accel) 3..7 m/s^2
--
-- Writes electrics etk854hybrid_sportIndex, _sportWindow (W), _sportExc (e) and
-- ctx.sportIndex, ctx.sportWindow, ctx.pedalRate (1/s, fast-pedal start rule).

local M = {}
M.name = "sportIndex"
M.isReal = true

local abs, max, min, exp = math.abs, math.max, math.min, math.exp

local P = {
  U0 = 0.50, U1 = 0.95,              -- demand band on the raw pedal
  RATE_TAU_S = 0.08,                 -- pedal-rate LPF; no peak hold (stabs would count)
  RATE0 = 1.0, RATE_SPAN = 3.0,
  W_RATE = 0.8,
  BRAKE0 = 0.35, BRAKE1 = 0.80, W_BRAKE = 0.0, BRAKE_MIN_KMH = 30,
  LAT0 = 3.0, LAT1 = 7.0, W_LAT = 0.8,
  TAU_RISE_S = 1.2,
  TAU_MIN_S = 8, TAU_MAX_S = 90,     -- decay tau = min + (max - min) * W
  W_GROW_S = 30, W_SHRINK_S = 180,
  -- W grows on e, not on S: growing on S is a positive feedback that latches the slow decay
  W_GROW_ABOVE = 0.5, W_SHRINK_BELOW = 0.2,
  STANDSTILL_KMH = 5, STANDSTILL_PEDAL = 0.05, STANDSTILL_TAU_MULT = 4,
  FREEZE_BRAKE = 0.3, FREEZE_MIN_KMH = 30,
  DT_MAX = 0.05,
  -- INFO log on band change
  BANDS = { 0.25, 0.50, 0.75 }, BAND_HYST = 0.03,
}

local st = { S = 0, W = 0, pr = 0, lastP = 0, band = 0, e = 0 }

local U = require("controller/etk854hybrid/etk854hybrid_util")
local clamp01 = U.clamp01NanZero -- NaN -> 0
local smoothstep = U.smoothstep -- t through clamp01NanZero

local function publish(ctx)
  electrics.values.etk854hybrid_sportIndex = st.S
  electrics.values.etk854hybrid_sportWindow = st.W
  electrics.values.etk854hybrid_sportExc = st.e
  if ctx then
    ctx.sportIndex = st.S
    ctx.sportWindow = st.W
    ctx.pedalRate = st.pr
  end
end

local function bandOf(s)
  local b = 0
  for i, edge in ipairs(P.BANDS) do
    if s >= edge then b = i end
  end
  return b
end

local function logBand(ctx)
  local b = bandOf(st.S)
  if b == st.band then return end
  -- hysteresis: only accept the new band when clearly inside it
  local edge = (b > st.band) and P.BANDS[b] or P.BANDS[st.band]
  if edge and abs(st.S - edge) < P.BAND_HYST then return end
  st.band = b
  log("I", "etk854hybrid.sportIdx", string.format(
    "[SPORTIDX] band=%d S=%.2f W=%.2f pedal=%.2f v=%.0fkm/h", b, st.S, st.W,
    (ctx and ctx.pedalIn) or 0, (ctx and ctx.speedKmh) or 0))
end

local function lateralAccel()
  if type(sensors) == "table" and type(sensors.gx2) == "number" then return abs(sensors.gx2) end
  return 0
end

function M.updateGFX(ctx, dt)
  if not ctx then return end
  dt = min(max(dt or 0, 0), P.DT_MAX)
  if dt <= 0 then publish(ctx); return end

  local pI = ctx.pedalIn or ctx.throttleRaw or 0
  local v = ctx.speedKmh or 0
  local brake = ctx.brakeRaw or 0
  local aLat = lateralAccel()

  -- split-map band; ctx.pedalJoin is set by the master's mapPedal earlier in the frame
  local dc = ctx.deModeCfg
  local u0, u1, wRate = P.U0, P.U1, P.W_RATE
  local pJ = ctx.pedalJoin or 0
  if dc and dc.siU1 and pJ > 0 then
    u0 = pJ
    u1 = max(dc.siU1, u0 + 0.05)
    wRate = dc.siWRate or P.W_RATE
  end
  local uDem = smoothstep((pI - u0) / (u1 - u0))
  local rateNow = max(0, (pI - st.lastP) / dt)
  st.pr = st.pr + (rateNow - st.pr) * (1 - exp(-dt / P.RATE_TAU_S))
  st.lastP = pI
  local uRate = clamp01((st.pr - P.RATE0) / P.RATE_SPAN)
  local uBrake = 0
  if v > P.BRAKE_MIN_KMH then uBrake = clamp01((brake - P.BRAKE0) / (P.BRAKE1 - P.BRAKE0)) end
  local uLat = clamp01((aLat - P.LAT0) / (P.LAT1 - P.LAT0))

  local e = max(uDem, wRate * uRate, P.W_BRAKE * uBrake, P.W_LAT * uLat)
  if (ctx.gearSign or 1) == 0 then e = 0 end
  st.e = e

  if e > P.W_GROW_ABOVE then
    st.W = min(1, st.W + dt / P.W_GROW_S)
  elseif st.S < P.W_SHRINK_BELOW then
    st.W = max(0, st.W - dt / P.W_SHRINK_S)
  end
  local tauDec = P.TAU_MIN_S + (P.TAU_MAX_S - P.TAU_MIN_S) * st.W
  if v < P.STANDSTILL_KMH and pI < P.STANDSTILL_PEDAL then tauDec = tauDec * P.STANDSTILL_TAU_MULT end

  if e > st.S then
    st.S = st.S + (e - st.S) * (1 - exp(-dt / P.TAU_RISE_S))
  elseif (brake > P.FREEZE_BRAKE and v > P.FREEZE_MIN_KMH) or aLat > P.LAT0 then
    -- freeze: braking into or cornering through a bend
  else
    st.S = st.S + (e - st.S) * (1 - exp(-dt / tauDec))
  end
  st.S = clamp01(st.S)
  publish(ctx)
  logBand(ctx)
end

-- QA / console hook: pin the index (the decay continues from there)
function M.set(ctx, value)
  st.S = clamp01(tonumber(value) or 0)
  st.band = bandOf(st.S)
  publish(ctx)
end

function M.reset(ctx)
  st.S, st.W, st.pr, st.band, st.e = 0, 0, 0, 0, 0
  st.lastP = (ctx and (ctx.pedalIn or ctx.throttleRaw)) or 0
  publish(ctx)
end

function M.init(ctx)
  M.reset(ctx)
  if ctx and ctx.dbg then
    ctx.dbg(1, "sportIdx", string.format("init: tauRise=%gs decay %g..%gs",
      P.TAU_RISE_S, P.TAU_MIN_S, P.TAU_MAX_S))
  end
end

function M.initStage2(ctx) end

function M.getState()
  return { S = st.S, W = st.W, pedalRate = st.pr, e = st.e }
end

return M
