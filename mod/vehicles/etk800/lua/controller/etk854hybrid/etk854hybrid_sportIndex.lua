-- etk854hybrid_sportIndex.lua - ETK 854 ActiveHybrid - driver sportiness index
--
-- User complaint behind this module: "In COMFORT it is very annoying when you
-- spontaneously want to drive sporty - the engine switches off again as soon
-- as no load is requested." A production car solves this with a driver-type
-- detection that stretches the ICE-off delay and lowers the start thresholds
-- after sporty driving. The index S in [0, 1] is exactly that:
--
--   e   = max( u_dem, 0.8*u_rate, W_BRAKE*u_brake, 0.8*u_lat )   (0 in N / P)
--         W_BRAKE = 0 (2026-10): braking no longer RAISES the index, it only
--         freezes it (below). With 0.6 an ordinary stop (brake >= 0.8 above
--         30 km/h) lifted S from ~0 to 0.34-0.44 (cruise_100 / tipin_rolling
--         STOP 2026-09-30) and the ICE then idled decoupled while the car
--         braked to a standstill - user feedback "Motor laeuft oft im
--         Leerlauf mit, obwohl er abgekuppelt ist".
--   S   rises toward e with tau 1.2 s,
--       decays toward e with tau 8..90 s (the window W grows while the
--       driver is ACTIVELY sporty, e > 0.5, and shrinks while S < 0.2 - a long
--       sporty section is remembered much longer than one stab),
--       x4 slower at standstill without pedal (red light between two bends),
--       frozen while braking hard or cornering.
--
--   u_dem   smoothstep of the raw driver pedal between 0.50 and 0.95 - the
--           normalised torque demand BEFORE the mode pedal map, so the index
--           is mode-neutral (pedal 0.60 -> 0.13, pedal 1.0 -> 1).
--           ECO PRO (pedal map v3, DE_CFG siU0 / siU1 / siWRate in
--           etk854hybridControl.lua): the band starts at the map's hybrid
--           join point ctx.pedalJoin (0.85; siU0 without a split map) and ends
--           at siU1 0.95, the rate term is off (siWRate 0) - a gamepad driver
--           holding the EV zone / EV MAX plateau (up to 0.85) or
--           tapping the trigger no longer reads as sporty; only presses into
--           the hybrid band / kickdown zone and cornering raise S in ECO.
--   u_rate  pedal application rate (LPF 0.08 s), 1/s above 1.0, full at 4.0
--   u_brake brake pressure 0.35..0.80 above 30 km/h (late hard braking;
--           weight W_BRAKE = 0, kept for re-tuning)
--   u_lat   |sensors.gx2| 3..7 m/s^2 (lateral; axis verified by QA, W_LAT = 0
--           disables it if the axis turns out swapped). sensors.lua:15-32:
--           gx2 is a 7/s non-linear smoothed acceleration.
--
-- The index never starts the ICE on its own: it only scales the policy
-- thresholds, the ICE-off hold time, the start profile and the limiter rise.
--
-- Publishes (every frame, contract): electrics.values.etk854hybrid_sportIndex,
-- diagnostics etk854hybrid_sportWindow (W) and etk854hybrid_sportExc (the
-- excitation e of the last frame; ECO no-idle analysis, feedback round 3)
-- ctx outputs: ctx.sportIndex, ctx.sportWindow, ctx.pedalRate (the filtered
-- pedal application rate st.pr [1/s]; the master's fast-pedal start rule;
-- kept on the raw pedal in every mode)

local M = {}
M.name = "sportIndex"
M.isReal = true

local abs, max, min, exp = math.abs, math.max, math.min, math.exp

local P = {
  U0 = 0.50, U1 = 0.95,              -- demand band on the raw pedal
  RATE_TAU_S = 0.08,                 -- pedal-rate LPF (no peak-hold on purpose:
  RATE0 = 1.0, RATE_SPAN = 3.0,      --  it made single stabs count as sporty)
  W_RATE = 0.8,
  BRAKE0 = 0.35, BRAKE1 = 0.80, W_BRAKE = 0.0, BRAKE_MIN_KMH = 30,   -- was 0.6, see header
  LAT0 = 3.0, LAT1 = 7.0, W_LAT = 0.8,
  TAU_RISE_S = 1.2,
  TAU_MIN_S = 8, TAU_MAX_S = 90,     -- decay tau = min + (max - min) * W
  W_GROW_S = 30, W_SHRINK_S = 180,
  -- W grows on the instantaneous excitation e, NOT on S: growing on S > 0.5
  -- is a positive feedback (longer tau keeps S above 0.5 longer, which grows
  -- W further) - in the offline harness three kickdowns plus a coast took W
  -- to 1.0 (tau 90 s) and S stayed > 0.5 for a minute of gentle driving.
  W_GROW_ABOVE = 0.5, W_SHRINK_BELOW = 0.2,
  STANDSTILL_KMH = 5, STANDSTILL_PEDAL = 0.05, STANDSTILL_TAU_MULT = 4,
  FREEZE_BRAKE = 0.3, FREEZE_MIN_KMH = 30,
  DT_MAX = 0.05,
  -- log bands (INFO line when the index moves between them, 3 % hysteresis)
  BANDS = { 0.25, 0.50, 0.75 }, BAND_HYST = 0.03,
}

local st = { S = 0, W = 0, pr = 0, lastP = 0, band = 0, e = 0 }

local function clamp01(v)
  if v ~= v then return 0 end        -- NaN guard
  if v < 0 then return 0 end
  if v > 1 then return 1 end
  return v
end

local function smoothstep(t)
  t = clamp01(t)
  return t * t * (3 - 2 * t)
end

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

  -- demand band / rate weight: per Driving Experience when it carries siU1
  -- AND its split pedal map is live (ECO PRO map v3: from the hybrid join
  -- point ctx.pedalJoin, published by the master's mapPedal earlier in the
  -- same frame, to siU1; rate off). ctx.pedalJoin > 0 only with the split
  -- map (AUTO EV, not pinned): ECO PRO + MAX EV / BATTERY CONTROL
  -- keep the default band U0..U1 and W_RATE as before map v3 (review 2026-10)
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
    -- freeze: braking into or cornering through a bend is still sporty driving
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
  if ctx and ctx.dbg then ctx.dbg(1, "sportIdx", "init: tauRise=1.2s decay 8..90s") end
end

function M.initStage2(ctx) end
function M.updateFixedStep(ctx, dt) end

function M.getState()
  return { S = st.S, W = st.W, pedalRate = st.pr, e = st.e }
end

return M
