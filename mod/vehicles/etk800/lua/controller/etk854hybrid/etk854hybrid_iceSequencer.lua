-- etk854hybrid_iceSequencer.lua - ETK 854 ActiveHybrid - ICE start/stop sequencer
--
-- Starts and stops the ICE behind K0 so that the engine speed matches the
-- motor shaft BEFORE K0 closes: K0 closing onto a slow engine lets the
-- frictionClutch slip damping (up to lockTorque) brake the driveline (jerk).
--
-- States (etk854hybrid_iceEngageState):
--   OFF      K0 open, ignition off
--   CRANK    K0: E-machine drags the ICE up through a slipping K0 (capacity
--               control, fill on)
--            STARTER: K0 open, 12 V starter
--   SYNC     K0 open, ICE rev-matched onto shaft + offset (idle-governor
--            reference + inertia feed-forward + integral on the VCU throttle);
--            after the time budget it engages anyway with K0 slip (SLIP/FORCED)
--   ENGAGE   K0 closes, governor handed over at GOV_HANDOVER_R
--   BLEND    K0 locked, ICE share ramped in (smoothstep)
--   ON
--   RELEASE  TORQUE (share out) -> OPEN (K0 opens at zero net ICE torque);
--            ignition cut only after K0 is open
--   IDLE     ICE-off hold spent decoupled (only DEs with ARB.IDLE_DE 1): K0
--            open, engine on its own governor; a request re-engages w/o crank
--   FAULT    internal, published as OFF + etk854hybrid_iceSeqFault
--
-- Per-frame outputs (ctx): r = K0 ratio (k0Ratio), g = ICE torque share gate
-- (iceShareGate), fill = motor fill gain that cancels the K0 torque at the
-- shaft (seqFillGain), override = ICE throttle 0..1 (iceThrottleOverride,
-- nil = torqueCoord throttle). slip = rpmIce - rpmShaft [rpm].
-- CE / VC = stock combustionEngine / vehicleController, VCU / master =
-- hybridControl, DE = Driving Experience.
--
-- Sole writer of etk854hybrid_k0ClutchRatio, ignition / starter,
-- ice.idleAVOverwrite / maxIdleThrottleOverwrite and ice.starterDisabled
-- (except the master's failsafe when this file fails to load).
--
-- M.arbitrate: mode policy -> debounced ctx.iceReq (on-debounce, minimum
-- on/off times, sportiness-scaled ICE-off hold; ctx.tsaFreeze keeps the
-- committed request). Decoupled-ICE diagnostics: per-frame class
-- (etk854hybrid_iceDecoupledWhy) + rate-limited [ICESEQ] decoupled* lines.
--
-- API: init / initStage2 / updateGFX / reset / getState / arbitrate /
-- k0WindowOpen / onExternalStall / getDiag; live levers setParam /
-- setProf / setArb (not persistent). Constants in P / PROF / REL / ARB, state
-- in st (LuaJIT 60-upvalue limit).

local M = {}
M.name = "iceSequencer"
M.isReal = true

local abs, max, min, sqrt, exp = math.abs, math.max, math.min, math.sqrt, math.exp
local RPM_TO_AV = 0.104719755
local AV_TO_RPM = 9.549296596

-- ===========================================================
-- Parameters
-- ===========================================================
local P = {
  -- ALWAYS = warm restart via the E-machine whenever possible, 12 V starter
  -- otherwise; URGENT_ONLY = K0 start only for urgent starts (kickdown,
  -- sail exit, SOC force)
  K0_START_POLICY     = "ALWAYS",
  K0_START_MIN_FRAC   = 1.3,     -- K0 start needs shaft >= max(1.3*idle, idle + 250) rpm,
  K0_START_MIN_ADD    = 250,     -- else K0 cannot drag the ICE to firing speed
  K0_START_RESERVE_NM = 30,      -- motor headroom needed = crankNm + this
  CRANK_HEADROOM_NM   = 10,      -- reserve held back from the driver = crankNm + this
  CRANK_MIN_NM        = 70,
  CRANK_MAX_NM        = 220,
  CRANK_DRAG_FRAC     = 1.0,     -- share of the drag estimate in the crank torque
  -- 1 = K0 start during a downshift (or any shift if urgent); the fill stays
  -- on in the slip states. 0 = 12 V starter during every shift
  K0_START_IN_DOWNSHIFT = 1,
  K0_SHIFT_WAIT_S     = 0.5,     -- non-urgent start waits for an upshift to end
  K0_CAP_DOWN_NMS     = 3000,    -- crank capacity ramp-down after fire
  K0_CAP_KI           = 0.004,   -- [1/(Nm*s)] measured-torque integral on r
  K0_CAP_I_MAX        = 0.2,
  K0_CRANK_RRATE      = 4.0,     -- [1/s]
  K0_CRANK_RMAX       = 0.6,
  CRANK_OPEN_R        = 0.01,    -- K0 counted open after the crank (-> SYNC)
  SLIP_EVENT_J        = 15000,   -- K0 thermal protection: per start ...
  SLIP_BUDGET_J       = 60000,   -- ... and leaky budget (typ. crank 4-6 kJ)
  SLIP_BUDGET_TAU_S   = 120,
  FIRE_FRAC           = 0.90,    -- "fired" = rpm >= 0.90*idle held FIRE_HOLD_S
  FIRE_HOLD_S         = 0.05,
  REFIRE_FRAC         = 0.60,    -- still spinning: ignition-only restart
  STALL_FRAC          = 0.45,    -- rpm < 0.45*idle for STALL_S = stalled
  STALL_S             = 0.2,
  STARTER_ENGAGE_FRAC = 0.78,    -- VC refuses the starter above 0.8*starterMaxAV
  -- warm block: cut the stock starter throttle-kill window (0.5 s, more
  -- when cold) - a warm engine fires on the first compressions
  STARTER_WARM_KILL_S = 0.25,
  STARTER_WARM_BLOCK_C = 40,
  -- crank timeout = throttle-kill window + ADD, clamped to MIN..MAX (MIN
  -- until the crank's second frame)
  STARTER_TIMEOUT_MIN_S = 1.5,
  STARTER_TIMEOUT_MAX_S = 3.0,
  STARTER_TIMEOUT_ADD_S = 1.0,
  RETRY_PAUSE_S       = 0.4,
  MAX_RETRIES         = 2,
  FAULT_LOCKOUT_S     = 5,
  FAULT_WINDOW_S      = 60,
  FAULT_BURST_N       = 3,
  FAULT_BURST_LOCKOUT_S = 30,
  SYNC_LEAD_S         = 0.08,    -- shaft rpm prediction
  -- the reference also follows the shaft's own rate, else it chases an
  -- accelerating low-gear shaft into the rev ceiling
  SYNC_TGTDOT_MAX     = 4000,    -- [rpm/s] clamp of that feed-forward
  SHDOT_TAU_S         = 0.05,
  SLIPDOT_TAU_S       = 0.03,
  SYNC_CEIL_MARGIN_RPM = 500,    -- never rev-match closer than this to iceMaxRPM
  SYNC_BLOCK_GIVEUP_S = 5,
  SYNC_BLOCK_LOCKOUT_S = 3,
  -- approach rate = min(syncRefRate, sqrt(2*SYNC_REF_DECEL*|err|)): the
  -- feed-forward tapers out before the target (a linear ramp overshoots,
  -- throttle smoother and turbo lag keep pushing)
  SYNC_REF_DECEL      = 12000,   -- [rpm/s^2]
  SYNC_KI             = 0.002,   -- [1/(rpm*s)] integral on the VCU throttle
  SYNC_I_MIN          = -0.10,
  SYNC_I_WINDOW_RPM   = 300,     -- integral frozen outside +-300 rpm
  SYNC_SLIPDOT_MAX    = 1500,    -- [rpm/s] slip must be settled, not crossing
  -- SYNC budget = syncMax + min(SYNC_RAMP_MAX_S, |target - rpmIce| / syncRefRate).
  -- After it: |slip| <= SYNC_SLIP_ENGAGE_RPM -> SLIP engage (planned, no
  -- fault), else FORCED after SYNC_HARD_EXTRA_S more (seqFault SYNC_TIMEOUT)
  SYNC_SLIP_ENGAGE_RPM = 400,
  SYNC_HARD_EXTRA_S   = 0.5,
  SYNC_RAMP_MAX_S     = 1.0,
  K0_SLIP_NMS         = 600,     -- SLIP engage: K0 capacity ramp
  SLIP_GOV_RPM        = 60,      -- SLIP/FORCED: governor keeps tracking above this slip
  IDLE_ENGAGE_MARGIN  = 50,      -- shaft below idle - 50: engage at idle (P/N/creep)
  IDLE_BAND_LO        = 60,
  IDLE_BAND_HI        = 150,
  K0_IDLE_ENGAGE_RATE = 1.2,
  K0_FORCED_NMS       = 300,     -- FORCED engage: K0 capacity ramp
  GOV_HANDOVER_R      = 0.3,     -- governor + SYNC throttle released above this
  ENGAGE_ABORT_SLIP_RPM = 350,
  ENGAGE_ABORT_R      = 0.6,
  ENGAGE_ABORT_S      = 0.6,
  ENGAGE_SHIFT_HOLD_R = 0.8,
  ENGAGE_MAX_S        = 1.2,
  ENGAGE_TIMEOUT_RATE = 2.0,
  ENGAGE_DONE_R       = 0.98,    -- ENGAGE -> BLEND at r >= this (lock confirmed or ENGAGE_MAX_S)
  KISS_R              = 0.2,     -- stats: slip when K0 starts to carry
  LOCK_SLIP_RPM       = 30,
  LOCK_CONFIRM_S      = 0.04,
  LOCK_MIN_R          = 0.5,
  K0_HARDEN_RATE      = 8.0,     -- r -> 1 once locked
  URGENT_DEFICIT_NM   = 10,      -- motor saturated -> URGENT blend time
  -- traction control trimming the driven axle (motor or ICE throttleFactor
  -- < BLEND_TC_TF): blend no faster than T_BLEND_TC - the turbo-lagged ICE
  -- trim cannot catch a fast share step on an axle at the slip limit
  BLEND_TC_TF         = 0.97,
  T_BLEND_TC          = 0.90,
  RELEASE_ZERO_NM     = 15,
  RELEASE_ZERO_S      = 0.05,
  RELEASE_TORQUE_TIMEOUT_S = 1.0,
  RELEASE_BLOCK_FRAC  = 0.95,    -- NORMAL release blocked while T_cmd > 0.95*motorCap
  REREQ_ENGAGE_R      = 0.5,     -- re-request in RELEASE:OPEN goes straight to ENGAGE
  REREQ_ENGAGE_SLIP_RPM = 100,   -- at r >= REREQ_ENGAGE_R and |slip| <= this
  K0_OPEN_EPS         = 0.02,
  K0_CLEANUP_RATE     = 8.0,
  RESERVE_RELEASE_NMS = 1500,    -- crank reserve given back to the driver
  T_BLEND_COMFORT_MIN = 0.30,
  U_S0 = 0.30, U_SPAN = 0.50,    -- COMFORT lerps toward URGENT with the index
  K0T_TAU_S           = 0.02,
  MEAS_TAU_S          = 0.05,
  LOCKED_R            = 0.9,
  LOCKED_SLIP_RPM     = 50,
  LOG_BUDGET_PER_S    = 40,
  -- OFF with the ignition switched on externally (VC stall handling, dev rig)
  -- and the engine running: cut the ignition after this long (> FIRE_HOLD_S
  -- and frame-level ignition transients); 0 = disabled
  ORPHAN_IGN_S        = 0.25,
}

-- Start profiles; COMFORT LERP_KEYS move toward URGENT with
-- u = clamp01((sportIndex - U_S0) / U_SPAN). Coupled ~0.6-1.0 s after the request.
local PROF = {
  COMFORT = { crankAlpha = 150, capRate = 1200, crankMaxS = 0.9, syncOffset = 30,
              syncRefRate = 4000, syncThrMax = 0.45, syncTol = 60, syncHold = 0.08,
              syncMax = 0.60, k0LockRate = 4.0, tBlend = 0.80, tRelease = 0.60, k0OpenRate = 3.0 },
  ECO_PRO = { crankAlpha = 130, capRate = 1000, crankMaxS = 1.0, syncOffset = 30,
              syncRefRate = 3500, syncThrMax = 0.40, syncTol = 60, syncHold = 0.08,
              syncMax = 0.70, k0LockRate = 3.5, tBlend = 1.00, tRelease = 0.50, k0OpenRate = 3.0 },
  SPORT   = { crankAlpha = 250, capRate = 2500, crankMaxS = 0.6, syncOffset = 50,
              syncRefRate = 6000, syncThrMax = 0.80, syncTol = 100, syncHold = 0.04,
              syncMax = 0.45, k0LockRate = 6.0, tBlend = 0.25, tRelease = 0.30, k0OpenRate = 5.0 },
  URGENT  = { crankAlpha = 280, capRate = 3000, crankMaxS = 0.6, syncOffset = 50,
              syncRefRate = 7000, syncThrMax = 1.00, syncTol = 120, syncHold = 0.03,
              syncMax = 0.40, k0LockRate = 6.0, tBlend = 0.20, tRelease = 0.30, k0OpenRate = 5.0 },
}
local LERP_KEYS = { crankAlpha = true, capRate = true, syncRefRate = true, syncThrMax = true,
                    k0LockRate = true, tBlend = true, syncMax = true }
-- Release profiles other than NORMAL (NORMAL uses PROF)
local REL = {
  FAST      = { tRelease = 0.25, k0OpenRate = 5.0 },
  EMERGENCY = { tRelease = 0.0,  k0OpenRate = 20.0 },
}
-- Arbiter (M.arbitrate); per-DE tables
local ARB = {
  ON_DEBOUNCE_S = { COMFORT = 0.15, ECO_PRO = 0.25, SPORT = 0.15, BATTERY_CONTROL = 0.50 },
  MIN_ON_S      = { COMFORT = 3.0,  ECO_PRO = 2.0,  SPORT = 3.0 },
  MIN_OFF_S     = { COMFORT = 1.0,  ECO_PRO = 1.5,  SPORT = 1.0 },
  HOLD_MIN_S    = { COMFORT = 2.0,  ECO_PRO = 1.5,  SPORT = 2.0 },
  HOLD_MAX_S    = { COMFORT = 20.0, ECO_PRO = 8.0,  SPORT = 20.0 },
  -- ICE-off hold (arbHoldTime), S = sportiness index:
  --   holdS = base + (H_MID - base) * clamp01((S - H_MID_S0) / H_MID_SPAN)
  --                + (HOLD_MAX - H_MID) * smoothstep((S - H_MAX_S0) / H_MAX_SPAN)
  --   base = 0 on a lift below LIFT_KMH (pedal < LIFT_THR, brake > BRAKE_THR
  --          or ctx.iceLightLoad), HOLD_MIN otherwise
  --   braking below LIFT_KMH with S < BRAKE_S_MAX -> hold 0 (also ends an IDLE;
  --          ECO PRO 1.01 = every brake)
  LIFT_KMH      = { COMFORT = 90.0, ECO_PRO = 100.0, SPORT = 0.0 },
  H_MID_S       = { COMFORT = 4.0,  ECO_PRO = 2.0,  SPORT = 4.0 },
  H_MID_S0 = 0.20, H_MID_SPAN = 0.30, H_MAX_S0 = 0.45, H_MAX_SPAN = 0.35,
  LIFT_THR = 0.02, BRAKE_THR = 0.05,
  BRAKE_S_MAX   = { COMFORT = 0.50, ECO_PRO = 1.01, SPORT = 0.50 },
  -- decoupled idle allowed (1) or never (0); ECO PRO spends its hold coupled
  IDLE_DE       = { COMFORT = 1, ECO_PRO = 0, SPORT = 1 },
  -- abort an unwanted start in SYNC after a real lift / brake this long [s];
  -- 0 = off (a start in progress completes)
  SYNC_ABORT_S  = { COMFORT = 0, ECO_PRO = 0, SPORT = 0 },
  MIN_ON_LIFT_S = 0.5,           -- minimum run time on a lift without decoupled idle
  -- Decoupled idle: once the policy has said "off" for DECOUPLE_S, K0 opens and
  -- the engine idles for the rest of the hold instead of dragging in fuel cut;
  -- a shorter lift stays coupled (no K0 cycling on a pumped pedal). Needs
  -- IDLE_DE 1, >= IDLE_MIN_S hold left and S >= IDLE_S_MIN (IDLE_S_MIN_LIFT on
  -- a lift below LIFT_KMH, so a brisk merge does not idle it decoupled)
  DECOUPLE_S    = { COMFORT = 1.2,  ECO_PRO = 0.8,  SPORT = 1.2 },
  IDLE_MIN_S = 1.0,
  IDLE_S_MIN = 0.45, IDLE_S_MIN_LIFT = 0.60,
  IDLE_W_MIN = 0.12,             -- outside SPORT also sportWindow >= this (not one kickdown)
  -- on-debounce while k0WindowOpen: the tip-in torque ramp would otherwise eat
  -- the crank headroom and send the start to the 12 V starter
  K0_WINDOW_DEBOUNCE_S = { COMFORT = 0.05, ECO_PRO = 0.10, SPORT = 0.05, BATTERY_CONTROL = 0.50 },
}

-- ===========================================================
-- State
-- ===========================================================
local st = {
  ice = nil, k0 = nil, motor = nil, gb = nil,
  state = "OFF", variant = "NONE", prof = "COMFORT", u = 0,
  relProfile = "NORMAL", relPhase = "TORQUE", engageMode = "NORMAL",
  r = 0, g = 0, x = 0, t = 0, now = 0,
  override = 0, fill = 0, reserve = 0, crankNm = 120,
  tReq = 0, startUrgent = false, fired = false, fireHoldT = 0, retries = 0,
  tcap = 0, rI = 0, k0Abort = false, tcapF = 0,
  starterOn = false, crankT = 0, crankFrames = 0, crankTimeout = P.STARTER_TIMEOUT_MIN_S, pauseT = 0, shiftWaitT = 0,
  ref = 0, sI = 0, idleEngage = false, syncBlocked = false,
  syncHoldT = 0, syncT = 0, blockedT = 0, stallT = 0, syncTimeoutLogged = false,
  syncBudget = nil,     -- SYNC time budget, set on the first SYNC frame
  lockT = 0, lockConfirmed = false, abortSlipT = 0, kissLogged = false,
  govArmed = false, engTimeoutLogged = false, zeroT = 0,
  k0T = 0, iceMeas = 0, slipDot = 0, shDot = 0, lastSlip = 0, lastSh = 0,
  eEvent = 0, eBudget = 0,
  onElapsed = 99, offElapsed = 99, lockoutT = 0,
  arbOnT = 0, arbHoldT = 0, holdRemain = 0, holdKind = "none",
  -- arbiter diagnostics of this frame (arbIdleOk -1 = not evaluated)
  arbWant = false, arbIdleOk = -1, minOnLeft = 0, arbSlow = false, arbLift = false, arbBraking = false,
  abortT = 0, orphanT = 0,
  seqFault = "none", faultTimes = {},
  startN = 0, stopN = 0, inStart = false,
  ptStarterT = 0,
  logWinT = 0, logN = 0,
  m = { rpmIce = 0, rpmSh = 0, slip = 0, slipAV = 0, idle = 800, iMax = 7000,
        Iice = 0.5, dragEst = 40 },
  stats = {},
  stop = {},
  -- decoupled-ICE diagnostics (updateDecDiag); tot / cnt survive reset
  dec = { why = "NONE", n = 0, epN = 0, segT0 = 0, epT0 = 0, epDk = "COMFORT", chain = "", chainN = 0,
          dk = "COMFORT", v0 = 0, rpm0 = 0, S0 = 0, W0 = 0, idleOk0 = -1, kind0 = "none", hold0 = 0,
          reason0 = "?", req0 = 0, unwT = 0,
          tot = {}, cnt = {}, win = {}, winN = {}, winAny = false, sumT = 0, lastDk = "COMFORT" },
}

-- ===========================================================
-- Helpers
-- ===========================================================
local U = require("controller/etk854hybrid/etk854hybrid_util")
local clamp = U.clamp
local clamp01 = U.clamp01NanZero -- NaN -> 0

local function rampTo(cur, target, rate, dt)
  if cur < target then return min(target, cur + rate * dt) end
  return max(target, cur - rate * dt)
end

local smoothstep = U.smoothstep -- t through clamp01NanZero

-- Inverse smoothstep (bisection): RELEASE hands g back to BLEND, whose state
-- is the phase x with g = smoothstep(x).
local function invSmoothstep(g)
  g = clamp01(g)
  local lo, hi = 0, 1
  for _ = 1, 12 do
    local mid = 0.5 * (lo + hi)
    if mid * mid * (3 - 2 * mid) < g then lo = mid else hi = mid end
  end
  return 0.5 * (lo + hi)
end

local function pv(key)
  local base = PROF[st.prof] or PROF.COMFORT
  local v = base[key]
  if st.prof == "COMFORT" and LERP_KEYS[key] and st.u > 0 then
    v = v + (PROF.URGENT[key] - v) * st.u
    if key == "tBlend" and v < P.T_BLEND_COMFORT_MIN then v = P.T_BLEND_COMFORT_MIN end
  end
  return v
end

-- Event log with a hard per-second budget (never flood beamng.log).
local function slog(fmt, ...)
  if st.now - st.logWinT >= 1 then st.logWinT = st.now; st.logN = 0 end
  st.logN = st.logN + 1
  if st.logN > P.LOG_BUDGET_PER_S then
    if st.logN == P.LOG_BUDGET_PER_S + 1 then
      log("W", "etk854hybrid.iceSeq", "[ICESEQ] log budget exceeded - suppressed for this second")
    end
    return
  end
  log("I", "etk854hybrid.iceSeq", string.format(fmt, ...))
end

-- Ignition / starter through the vehicleController; edge test on the real
-- device state, so an external ignition change cannot desync us.
local function setIgn(on)
  local ice = st.ice
  if ice and ice.ignitionCoef == (on and 1 or 0) then return end
  local mc = controller.mainController
  if mc and mc.setEngineIgnition then mc.setEngineIgnition(on) end
end

local function setStarter(on)
  local mc = controller.mainController
  if mc and mc.setStarter then mc.setStarter(on) end
  st.starterOn = on
end

-- On every exit of the governor phases. CE reset zeroes idleAVOverwrite but
-- NOT maxIdleThrottleOverwrite.
local function clearIdleOverwrite()
  local ice = st.ice
  if ice then
    ice.idleAVOverwrite = 0
    ice.maxIdleThrottleOverwrite = 0
  end
end

local function starterMaxAV()
  local ice = st.ice
  if ice and type(ice.starterMaxAV) == "number" then return ice.starterMaxAV end
  return st.m.idle * 0.7 * RPM_TO_AV
end

local function crankNmFor(alpha)
  local m = st.m
  return clamp(m.Iice * alpha + m.dragEst * P.CRANK_DRAG_FRAC, P.CRANK_MIN_NM, P.CRANK_MAX_NM)
end

-- Brake-throttle guard of the launch module (brake + pedal at standstill,
-- ctx.launchGuardNm = converter input torque cap, previous frame; not during
-- the launch hold, whose preload loads the converter on purpose). While on,
-- torqueCoord caps motor and ICE share, and the rev-match never leads the
-- shaft (no lead, no syncOffset): a leading reference makes the closing K0
-- push the shaft, which moves the reference again - positive feedback until
-- the brakes slip.
local function guardOn(ctx)
  return ctx.launchGuardNm ~= nil and not ctx.launchHold
end

-- Fill for an IDLE engage under the guard, only with the shaft at or above
-- idle - IDLE_ENGAGE_MARGIN: below it a fill would fight the ICE governor and
-- stall the engine when the guard cuts the cap to 0.
local function guardFill(ctx)
  local m = st.m
  return guardOn(ctx) and m.rpmSh >= m.idle - P.IDLE_ENGAGE_MARGIN
end

-- K0 commanded locked: BLEND / ON / RELEASE:TORQUE (s = st.state)
local function k0LockedState(s)
  return s == "ON" or s == "BLEND" or (s == "RELEASE" and st.relPhase == "TORQUE")
end

-- ===========================================================
-- K0 capacity model = stock frictionClutch under persistent slip (angle on
-- its bound a = maxClutchAngle*r):
--   T = min(LTeff, spring(a) + lockDamp*|slipAV|*r),  spring = a<=1 and a^2*lockSpring or a*lockSpring
-- (runtime device fields; the jbeam keys maxClutchAngle / lockDamp are not read)
-- ===========================================================
local function k0Model()
  local k0 = st.k0
  if k0 and type(k0.maxClutchAngle) == "number" and type(k0.lockSpring) == "number"
     and type(k0.lockDamp) == "number" then
    return k0
  end
  return nil
end

local function k0LockTorqueEff()
  local k0 = st.k0
  if not k0 then return 1150 end
  return (k0.lockTorque or 1150) * (k0.thermalEfficiency or 1)
         * (k0.damageLockTorqueCoef or 1) * (k0.wearLockTorqueCoef or 1)
end

local function k0Cap(r, slipAV)
  local LT = k0LockTorqueEff()
  local k0 = k0Model()
  if not k0 then return LT * r * r end
  local a = k0.maxClutchAngle * r
  local spr = (a <= 1) and (a * a * k0.lockSpring) or (a * k0.lockSpring)
  return min(LT, spr + k0.lockDamp * abs(slipAV) * r)
end

local function k0RatioFor(T, slipAV)
  if T <= 0 then return 0 end
  if not k0Model() then
    local k0 = st.k0
    return clamp01(sqrt(T / max(1, (k0 and k0.lockTorque) or 1150)))
  end
  if k0Cap(1, slipAV) <= T then return 1 end
  local lo, hi = 0, 1
  for _ = 1, 12 do
    local mid = (lo + hi) * 0.5
    if k0Cap(mid, slipAV) < T then lo = mid else hi = mid end
  end
  return hi
end

-- ===========================================================
-- Per-frame measurements
-- ===========================================================
local function measure(ctx, dt)
  local m, ice, k0 = st.m, st.ice, st.k0
  local c = ctx.modules and ctx.modules.curves
  m.idle = (c and c.iceIdleRPM and c.iceIdleRPM()) or ctx.iceIdleRPM or 800
  m.iMax = (c and c.iceMaxRPM and c.iceMaxRPM()) or 7000
  m.rpmIce = (type(ice.outputAV1) == "number") and ice.outputAV1 * AV_TO_RPM or (ctx.iceRpm or 0)
  m.rpmSh = ctx.rpmMotor or m.rpmIce
  -- a shaft at rest reads as a denormal (~1e-323)
  if abs(m.rpmSh) < 1e-3 then m.rpmSh = 0 end
  if abs(m.rpmIce) < 1e-3 then m.rpmIce = 0 end
  local slip = m.rpmIce - m.rpmSh
  m.slip = slip
  m.slipAV = slip * RPM_TO_AV
  if dt > 0 then
    st.slipDot = st.slipDot + ((slip - st.lastSlip) / dt - st.slipDot) * min(1, dt / P.SLIPDOT_TAU_S)
    st.shDot = st.shDot + ((m.rpmSh - st.lastSh) / dt - st.shDot) * min(1, dt / P.SHDOT_TAU_S)
  end
  st.lastSlip, st.lastSh = slip, m.rpmSh
  m.Iice = (type(ice.inertia) == "number") and ice.inertia or 0.5
  -- drag to overcome when cranking (friction + engine brake + dynamic at idle)
  m.dragEst = (ice.friction or 15) * (ice.wearFrictionCoef or 1) + (ice.engineBrakeTorque or 0)
              + (ice.dynamicFriction or 0) * m.idle * RPM_TO_AV
  local k0raw = (k0 and type(k0.torqueDiff) == "number") and k0.torqueDiff or 0
  if dt > 0 then
    st.k0T = st.k0T + (k0raw - st.k0T) * min(1, dt / P.K0T_TAU_S)
    -- slip heat [J], not while commanded locked: the "slip" there is elastic
    -- spring wind-up and would fill the budget in cruise (every start on the
    -- 12 V starter)
    local s = st.state
    local locked = k0LockedState(s)
    local e = locked and 0 or abs(k0raw * m.slipAV) * dt
    st.eEvent = st.eEvent + e
    st.eBudget = st.eBudget * exp(-dt / P.SLIP_BUDGET_TAU_S) + e
    if st.r >= P.LOCKED_R and abs(slip) < P.LOCKED_SLIP_RPM then
      local comb = (type(ice.combustionTorque) == "number") and ice.combustionTorque or st.k0T
      st.iceMeas = st.iceMeas + (comb - st.iceMeas) * min(1, dt / P.MEAS_TAU_S)
    else
      st.iceMeas = st.k0T
    end
  end
  -- shared stall detector
  if m.rpmIce < P.STALL_FRAC * m.idle then st.stallT = st.stallT + dt else st.stallT = 0 end
end

-- ===========================================================
-- Statistics / summary lines
-- ===========================================================
local function resetStartStats()
  local s = st.stats
  s.fireMs, s.syncMs, s.lockMs, s.blendMs = 0, 0, 0, 0
  s.slipAtKiss, s.slipAtLock = nil, nil
  s.maxDev, s.maxDevAch, s.motorSat, s.syncT0, s.blendT0 = 0, 0, false, nil, nil
  s.engageMode = nil
  if st.motor then st.motor.etkShaftRateMax = 0 end
end

local function fmtOpt(v)
  if type(v) ~= "number" then return "na" end
  return string.format("%.0f", v)
end

local function logStartSummary(ctx, fault)
  local s = st.stats
  local total = (st.now - st.tReq) * 1000
  local rateMax = (st.motor and st.motor.etkShaftRateMax) or 0
  if st.motor then st.motor.etkShaftRateMax = 0 end
  log("I", "etk854hybrid.iceSeq", string.format(
    "[ICESEQ] start#%d type=%s prof=%s u=%.2f engage=%s fire=%.0fms sync=%.0fms lock=%.0fms blend=%.0fms total=%.0fms slipAtKiss=%srpm slipAtLock=%srpm eEvent=%.0fJ maxShaftDevNm=%.1f maxShaftDevAchNm=%.1f maxShaftRate=%.0fNm/s motorSat=%d retries=%d fault=%s",
    st.startN, st.variant, st.prof, st.u, s.engageMode or "na", s.fireMs, s.syncMs, s.lockMs, s.blendMs, total,
    fmtOpt(s.slipAtKiss), fmtOpt(s.slipAtLock), st.eEvent, s.maxDev, s.maxDevAch, rateMax,
    s.motorSat and 1 or 0, st.retries, fault or st.seqFault))
  ctx.iceSeqLast = {
    n = st.startN, type = st.variant, prof = st.prof, u = st.u, engageMode = s.engageMode, fireMs = s.fireMs,
    syncMs = s.syncMs, lockMs = s.lockMs, blendMs = s.blendMs, totalMs = total,
    slipAtKiss = s.slipAtKiss, slipAtLock = s.slipAtLock, eEventJ = st.eEvent,
    maxShaftDevNm = s.maxDev, maxShaftDevAchNm = s.maxDevAch, maxShaftRate = rateMax,
    motorSat = s.motorSat,
    retries = st.retries, fault = fault or st.seqFault,
  }
end

local function recordFire(ctx)
  st.fired = true
  local ms = max(1, (st.now - st.tReq) * 1000)
  st.stats.fireMs = ms
  ctx.iceFireMs = ms
  -- iceRestartMs is transient (fire -> coupled; QA and HUD "START"),
  -- iceFireMs persists
  electrics.values.etk854hybrid_iceRestartMs = ms
  electrics.values.etk854hybrid_iceFireMs = ms
  slog("[ICESEQ] fire type=%s rpmIce=%.0f after %.0fms", st.variant, st.m.rpmIce, ms)
end

-- ===========================================================
-- State entry
-- ===========================================================
local function enter(newState, why)
  local m = st.m
  slog("[ICESEQ] %s->%s var=%s prof=%s why=%s rpmIce=%.0f rpmSh=%.0f slip=%.0f r=%.2f g=%.2f dt=%.2fs",
    st.state, newState, st.variant, st.prof, why or "", m.rpmIce, m.rpmSh, m.slip, st.r, st.g, st.t)
  st.state = newState
  st.t = 0
end

local function beginOff(ctx, why)
  enter("OFF", why)
  electrics.values.etk854hybrid_iceRestartMs = 0
  st.offElapsed = 0
  st.g, st.override, st.fill = 0, 0, 0
  clearIdleOverwrite()
  setIgn(false)
  if st.starterOn then setStarter(false) end
  st.startUrgent = false
end

local function beginFault(ctx, reason)
  enter("FAULT", reason)
  if st.starterOn then setStarter(false) end
  setIgn(false)
  clearIdleOverwrite()
  st.seqFault = reason
  st.g, st.override, st.fill = 0, 0, 0
  st.lockoutT = max(st.lockoutT, P.FAULT_LOCKOUT_S)
  local keep = {}
  for _, t in ipairs(st.faultTimes) do
    if st.now - t <= P.FAULT_WINDOW_S then keep[#keep + 1] = t end
  end
  keep[#keep + 1] = st.now
  st.faultTimes = keep
  if #keep >= P.FAULT_BURST_N then
    st.lockoutT = max(st.lockoutT, P.FAULT_BURST_LOCKOUT_S)
    st.faultTimes = {}
    log("E", "etk854hybrid.iceSeq", string.format(
      "[ICESEQ] %d start faults within %.0fs - ICE locked out for %.0fs (last: %s)",
      P.FAULT_BURST_N, P.FAULT_WINDOW_S, P.FAULT_BURST_LOCKOUT_S, reason))
  end
  if guihooks and guihooks.message then
    guihooks.message("Engine start failed", 3, "vehicle.iceSeq")
  end
  logStartSummary(ctx, reason)
  st.inStart = false
  electrics.values.etk854hybrid_iceRestartMs = 0
end

local function beginCrankK0(ctx)
  enter("CRANK", "k0-start")
  st.variant = "K0"
  st.tcap, st.rI = 0, 0
  st.fired, st.fireHoldT, st.k0Abort = false, 0, false
  setIgn(true)
  st.override, st.fill = 0, 1
end

local function beginCrankStarter(ctx, why)
  enter("CRANK", why)
  st.variant = "STARTER"
  st.crankT, st.crankFrames, st.crankTimeout, st.pauseT = 0, 0, P.STARTER_TIMEOUT_MIN_S, 0
  st.fireHoldT = 0
  st.override, st.fill = 0, 0
end

local function beginSync(ctx, why)
  enter("SYNC", why)
  local m, ice = st.m, st.ice
  st.ref = m.rpmIce
  st.sI, st.syncHoldT, st.syncT, st.blockedT = 0, 0, 0, 0
  st.syncTimeoutLogged = false
  st.syncBudget = nil     -- set on the first SYNC frame (needs the target)
  st.stats.syncT0 = st.now
  -- Kill the stock start flare: CE adds idleAVStartOffset (up to 2x idle,
  -- decaying) to every governor target, including our reference. With the
  -- starter off, a smoother reset keeps it at 0.
  if (ice.starterEngagedCoef or 0) <= 0 and ice.idleAVStartOffsetSmoother
     and ice.idleAVStartOffsetSmoother.reset then
    ice.idleAVStartOffsetSmoother:reset()
    ice.idleAVStartOffset = 0
  end
  setIgn(true)
  st.fill = 1
end

local function beginEngage(ctx, mode, why)
  enter("ENGAGE", why .. ":" .. mode)
  st.engageMode = mode
  st.lockT, st.abortSlipT = 0, 0
  st.lockConfirmed, st.kissLogged, st.engTimeoutLogged = false, false, false
  st.govArmed = true
  st.tcapF = k0Cap(st.r, st.m.slipAV)
  if st.stats.syncT0 then st.stats.syncMs = (st.now - st.stats.syncT0) * 1000 end
  st.stats.engageMode = mode
  st.fill = (mode == "IDLE" and not guardFill(ctx)) and 0 or 1
end

local function beginBlend(ctx, x0, why)
  enter("BLEND", why)
  st.x = clamp01(x0 or 0)
  st.g = smoothstep(st.x)
  st.stats.blendT0 = st.now
  st.fill, st.override = 1, nil
end

local function beginOn(ctx)
  enter("ON", "blended")
  st.onElapsed = 0
  st.g, st.r = 1, 1
  if st.stats.blendT0 then st.stats.blendMs = (st.now - st.stats.blendT0) * 1000 end
  if st.inStart then logStartSummary(ctx, nil) end
  st.inStart = false
  electrics.values.etk854hybrid_iceRestartMs = 0
  st.startUrgent = false
end

local function beginRelease(ctx, profile, phase, why)
  enter("RELEASE", why .. ":" .. profile)
  st.relProfile = REL[profile] and profile or "NORMAL"
  st.relPhase = phase or "TORQUE"
  st.zeroT = 0
  st.stopN = st.stopN + 1
  st.stop.t0, st.stop.openT0, st.stop.profile = st.now, nil, st.relProfile
  st.override, st.fill = nil, 1
  if st.relPhase == "OPEN" then st.stop.openT0 = st.now; st.g = 0 end
  if st.relProfile == "EMERGENCY" then
    setIgn(false)
    clearIdleOverwrite()
    st.g, st.relPhase = 0, "OPEN"
    st.stop.openT0 = st.now
  end
end

-- IDLE: entered from RELEASE once K0 is open
local function beginIdle(ctx, why)
  enter("IDLE", why)
  st.g, st.override, st.fill = 0, 0, 0
  clearIdleOverwrite()
  electrics.values.etk854hybrid_iceRestartMs = 0
end

-- ===========================================================
-- OFF -> start request: variant choice
-- ===========================================================
-- lowest shaft speed at which K0 can drag the ICE to firing speed
local function k0MinShaftRpm(idle)
  return max(P.K0_START_MIN_FRAC * idle, idle + P.K0_START_MIN_ADD)
end

-- Returns allowed, reason (logged on CRANK entry: why a STARTER start).
local function k0StartAllowed(ctx)
  local m = st.m
  if P.K0_START_POLICY == "URGENT_ONLY" and not st.startUrgent then return false, "policy" end
  if (ctx.gearSign or 0) == 0 then return false, "N/P" end
  if ctx.isShifting then
    -- urgent: any shift (the kickdown may not be flagged as a downshift yet)
    local gb = st.gb
    local down = gb and gb.isShiftingDown == true
    if not (P.K0_START_IN_DOWNSHIFT > 0 and (down or st.startUrgent)) then return false, "shifting" end
  end
  if ctx.evLockoutActive then return false, "evLockout" end
  if m.rpmSh < k0MinShaftRpm(m.idle) then return false, "shaftSlow" end
  if st.eBudget >= P.SLIP_BUDGET_J then return false, string.format("k0Heat%.0fkJ", st.eBudget / 1000) end
  -- headroom against the DELIVERED motor torque (coordinator ramp), not the
  -- demand (> cap by definition for a demand start); updateReserve then
  -- holds the crank torque back from the driver during CRANK
  local cap = ctx.motorCapNm or 0
  local used = max(0, ctx.motorBaseRawNm or ctx.motorBaseNm or 0)
  st.crankNm = crankNmFor(pv("crankAlpha"))
  if (cap - used) < st.crankNm + P.K0_START_RESERVE_NM then
    return false, string.format("headroom%.0f<%.0fNm", cap - used, st.crankNm + P.K0_START_RESERVE_NM)
  end
  return true, "k0"
end

-- Public: could a K0 start begin THIS frame? (master tip-in anticipation,
-- arbiter short debounce). k0StartAllowed without the policy / shift gates.
local function k0WindowOpen(ctx)
  local m = st.m
  if (ctx.gearSign or 0) == 0 or ctx.evLockoutActive or ctx.isShifting then return false end
  if st.state ~= "OFF" or not st.ice or st.ice.isDisabled then return false end
  if st.eBudget >= P.SLIP_BUDGET_J then return false end
  local rpmSh = ctx.rpmMotor or m.rpmSh
  if rpmSh < k0MinShaftRpm(m.idle) then return false end
  local used = max(0, ctx.motorBaseRawNm or ctx.motorBaseNm or 0)
  return (ctx.motorCapNm or 0) - used >= crankNmFor(pv("crankAlpha")) + P.K0_START_RESERVE_NM
end
M.k0WindowOpen = k0WindowOpen

-- BeamNG quirk: a starter crank withdrawn inside the throttle-kill window
-- leaves CE starterThrottleKillCoef stuck at 0 (deactivateStarter only
-- restores it after the timer ran out), so the engine can never fire again.
-- Only activateStarter or a reset clears it; K0 / REFIRE starts never call
-- activateStarter, so re-arm it on every start.
local function rearmThrottleKill()
  local ice = st.ice
  if not ice or (ice.starterEngagedCoef or 0) > 0 then return end
  if (ice.starterThrottleKillCoef or 1) < 1 then
    ice.starterThrottleKillCoef = 1
    ice.starterThrottleKillTimer = 0
  end
end

-- "Fired" needs more than rpm (K0 drags the ICE to that speed without
-- combustion): CE must be able to produce torque. combustionTorque is no
-- test - a fired but dragged engine shows ~ -friction (governor at ~0).
local function canCombust()
  local ice = st.ice
  if not ice or ice.isDisabled then return false end
  return (ice.ignitionCoef or 0) >= 1 and (ice.starterThrottleKillCoef or 1) >= 1
    and (ice.outputTorqueState or 1) > 0
end

local function openStart()
  rearmThrottleKill()
  st.variant = "NONE"   -- entry logs must not show the last start's variant
  st.tReq = st.now
  st.eEvent, st.retries = 0, 0
  st.seqFault = "none"
  st.fired = false
  st.startN = st.startN + 1
  st.inStart = true
  resetStartStats()
end

local function startRequest(ctx)
  local m = st.m
  openStart()
  if m.rpmIce >= P.REFIRE_FRAC * m.idle then
    st.variant = "REFIRE"
    setIgn(true)
    recordFire(ctx)
    beginSync(ctx, "refire")
  else
    local ok, why = k0StartAllowed(ctx)
    if ok then beginCrankK0(ctx) else beginCrankStarter(ctx, "starter-start:" .. why) end
  end
end

-- ===========================================================
-- States
-- ===========================================================
local function stOff(ctx, dt)
  st.r = rampTo(st.r, 0, P.K0_CLEANUP_RATE, dt)
  st.g, st.override, st.fill = 0, 0, 0
  st.offElapsed = st.offElapsed + dt
  if ctx.iceReq and not st.ice.isDisabled then
    -- non-urgent request during an upshift: wait for its end (K0 start
    -- instead of the 12 V starter) if the shaft is fast enough
    local m = st.m
    local gb = st.gb
    if ctx.isShifting and not st.startUrgent and not (gb and gb.isShiftingDown == true)
       and (ctx.gearSign or 0) ~= 0 and st.shiftWaitT < P.K0_SHIFT_WAIT_S
       and m.rpmSh >= k0MinShaftRpm(m.idle) then
      st.shiftWaitT = st.shiftWaitT + dt
      return
    end
    st.shiftWaitT = 0
    st.orphanT = 0
    startRequest(ctx)
  else
    st.shiftWaitT = 0
    -- orphan ignition (ORPHAN_IGN_S): else an externally started engine would
    -- run decoupled for good
    local ice, m = st.ice, st.m
    if P.ORPHAN_IGN_S > 0 and (ice.ignitionCoef or 0) > 0 and not st.starterOn
       and m.rpmIce >= P.REFIRE_FRAC * m.idle then
      st.orphanT = st.orphanT + dt
      if st.orphanT >= P.ORPHAN_IGN_S then
        slog("[ICESEQ] OFF: ignition on behind the sequencer rpmIce=%.0f for %.2fs - ignition cut",
          m.rpmIce, st.orphanT)
        setIgn(false)
        st.orphanT = 0
      end
    else
      st.orphanT = 0
    end
  end
end

-- CRANK / K0: slip start, model feed-forward (k0RatioFor) + measured-torque
-- integral.
local function stCrankK0(ctx, dt)
  local m = st.m
  st.fill, st.override = 1, 0
  if not ctx.iceReq then
    st.tcap = 0
    st.r = rampTo(st.r, 0, P.K0_CLEANUP_RATE, dt)
    if st.r <= 0 then beginOff(ctx, "withdrawn") end
    return
  end
  if m.rpmIce >= P.FIRE_FRAC * m.idle and canCombust() then st.fireHoldT = st.fireHoldT + dt else st.fireHoldT = 0 end
  if not st.fired and st.fireHoldT >= P.FIRE_HOLD_S then recordFire(ctx) end

  if st.k0Abort then
    st.tcap = 0
    st.r = rampTo(st.r, 0, P.K0_CLEANUP_RATE, dt)
    if st.r <= 0 then
      if st.fired then
        beginSync(ctx, "fired-after-abort")
      elseif (st.ice.outputAV1 or 0) < P.STARTER_ENGAGE_FRAC * starterMaxAV() then
        st.retries = st.retries + 1
        beginCrankStarter(ctx, "k0-abort")
      end
    end
    return
  end

  st.crankNm = crankNmFor(pv("crankAlpha"))
  if not st.fired then
    local headroom = (ctx.motorCapNm or 0) - (ctx.motorBaseRawNm or ctx.motorBaseNm or 0) - P.CRANK_HEADROOM_NM
    local tcapMax = min(st.crankNm, headroom)
    local why = nil
    if tcapMax < P.CRANK_MIN_NM then why = "headroom"
    elseif st.t > pv("crankMaxS") then why = "timeout"
    elseif st.eEvent > P.SLIP_EVENT_J then why = "slip-energy" end
    if why then
      st.k0Abort = true
      slog("[ICESEQ] K0 crank abort (%s) rpmIce=%.0f tcap=%.0f eEvent=%.0fJ", why, m.rpmIce, st.tcap, st.eEvent)
      return
    end
    st.tcap = min(tcapMax, st.tcap + pv("capRate") * dt)
    st.rI = clamp(st.rI + P.K0_CAP_KI * (st.tcap - max(0, -st.k0T)) * dt, -P.K0_CAP_I_MAX, P.K0_CAP_I_MAX)
  else
    st.tcap = max(0, st.tcap - P.K0_CAP_DOWN_NMS * dt)
  end
  local rDes = 0
  if st.tcap > 0 then rDes = clamp(k0RatioFor(st.tcap, m.slipAV) + st.rI, 0, P.K0_CRANK_RMAX) end
  st.r = rampTo(st.r, rDes, P.K0_CRANK_RRATE, dt)
  -- K0 fully open before the rev-match
  if st.fired and st.tcap <= 0 and st.r <= P.CRANK_OPEN_R then
    st.r = 0
    beginSync(ctx, "k0-fired")
  end
end

-- CRANK / STARTER: K0 open.
local function stCrankStarter(ctx, dt)
  local m, ice = st.m, st.ice
  st.override, st.fill = 0, 0
  if not ctx.iceReq then
    if st.starterOn then setStarter(false) end
    st.r = rampTo(st.r, 0, P.K0_CLEANUP_RATE, dt)
    if st.r <= 0 then beginOff(ctx, "withdrawn") end
    return
  end
  if st.r > P.K0_OPEN_EPS then
    st.r = rampTo(st.r, 0, P.K0_CLEANUP_RATE, dt)
    return
  end
  st.r = 0
  if st.pauseT > 0 then st.pauseT = st.pauseT - dt; return end
  -- fired: CE disengages the starter itself above 1.1*starterMaxAV
  local starterEng = (ice.starterEngagedCoef or 0) > 0
  if m.rpmIce >= P.FIRE_FRAC * m.idle and not starterEng and canCombust() then
    st.fireHoldT = st.fireHoldT + dt
  else
    st.fireHoldT = 0
  end
  if st.fireHoldT >= P.FIRE_HOLD_S then
    if st.starterOn then setStarter(false) end
    recordFire(ctx)
    beginSync(ctx, "starter-fired")
    return
  end
  if not st.starterOn then
    -- still spinning down above the starter window: VC would refuse it
    if (ice.outputAV1 or 0) < P.STARTER_ENGAGE_FRAC * starterMaxAV() then
      setIgn(true)
      setStarter(true)
      st.crankT, st.crankFrames, st.crankTimeout = 0, 0, P.STARTER_TIMEOUT_MIN_S
    end
    return
  end
  st.crankT = st.crankT + dt
  st.crankFrames = st.crankFrames + 1
  if st.crankFrames == 2 then
    local th = ice.thermals
    local blockC = th and th.engineBlockTemperature
    if type(blockC) == "number" and blockC >= P.STARTER_WARM_BLOCK_C
       and (ice.starterThrottleKillTimer or 0) > P.STARTER_WARM_KILL_S then
      ice.starterThrottleKillTimer = P.STARTER_WARM_KILL_S
    end
    -- throttle-kill time (longer when cold) plus STARTER_TIMEOUT_ADD_S
    st.crankTimeout = clamp(P.STARTER_TIMEOUT_ADD_S + (ice.starterThrottleKillTimer or 0.5),
                            P.STARTER_TIMEOUT_MIN_S, P.STARTER_TIMEOUT_MAX_S)
  end
  if st.crankT > st.crankTimeout then
    setStarter(false)
    st.retries = st.retries + 1
    if st.retries > P.MAX_RETRIES then beginFault(ctx, "CRANK_FAIL"); return end
    st.pauseT = P.RETRY_PAUSE_S
    slog("[ICESEQ] crank timeout %.2fs rpmIce=%.0f - retry %d", st.crankTimeout, m.rpmIce, st.retries)
  end
end

local function stCrank(ctx, dt)
  if st.variant == "K0" then stCrankK0(ctx, dt) else stCrankStarter(ctx, dt) end
end

-- Rev-match actuator (SYNC and the governor phase of ENGAGE): speed reference
-- -> stock idle governor (idleAVOverwrite) + feed-forward
-- throttleForTorque(Iice * dRef/dt) + integral on the VCU throttle channel
-- (CE uses the max of governor and VCU throttle).
local function syncTrack(ctx, dt)
  local m, ice = st.m, st.ice
  local idle = m.idle
  local idleEngage = m.rpmSh < idle - P.IDLE_ENGAGE_MARGIN
  local guard = guardOn(ctx)
  local off = guard and 0 or pv("syncOffset")
  local ceilRpm = m.iMax - P.SYNC_CEIL_MARGIN_RPM
  local rpmT = idle
  if not idleEngage then
    local lead = st.shDot * P.SYNC_LEAD_S
    if guard and lead > 0 then lead = 0 end
    rpmT = clamp(max(m.rpmSh + lead, idle) + off, idle, ceilRpm)
  end
  local prev = st.ref
  local refErr = rpmT - st.ref
  -- target rate + softly landing approach, never past the target
  local tgtDot = 0
  if not idleEngage and not ctx.isShifting then
    tgtDot = clamp(st.shDot, -P.SYNC_TGTDOT_MAX, P.SYNC_TGTDOT_MAX)
  end
  local approach = min(pv("syncRefRate"), sqrt(2 * P.SYNC_REF_DECEL * abs(refErr)))
  if refErr < 0 then approach = -approach end
  local nr = st.ref + (tgtDot + approach) * dt
  if (rpmT - nr) * refErr < 0 then nr = rpmT end
  st.ref = nr
  local refDotAV = (dt > 0) and ((st.ref - prev) / dt * RPM_TO_AV) or 0
  local thrMax = pv("syncThrMax")
  ice.idleAVOverwrite = st.ref * RPM_TO_AV
  ice.maxIdleThrottleOverwrite = thrMax
  st.govArmed = true
  local tc = ctx.modules and ctx.modules.torqueCoord
  local ff = 0
  if tc and tc.throttleForTorque then ff = tc.throttleForTorque(m.Iice * refDotAV, m.rpmIce, ctx) end
  local err = st.ref - m.rpmIce
  -- integral only once the reference has landed (no wind-up)
  if abs(err) < P.SYNC_I_WINDOW_RPM and abs(refErr) < pv("syncTol") then
    st.sI = clamp(st.sI + P.SYNC_KI * err * dt, P.SYNC_I_MIN, thrMax)
  end
  st.override = clamp(ff + st.sI, 0, thrMax)
  st.idleEngage = idleEngage
  st.syncBlocked = (not idleEngage) and (m.rpmSh + off > ceilRpm)
  return rpmT, idleEngage, st.syncBlocked
end

local function stSync(ctx, dt)
  local m = st.m
  st.fill = 1
  st.r = rampTo(st.r, 0, P.K0_CLEANUP_RATE, dt)
  if not ctx.iceReq then beginOff(ctx, "withdrawn"); return end
  if st.stallT >= P.STALL_S then
    clearIdleOverwrite()
    if st.retries < P.MAX_RETRIES then
      st.retries = st.retries + 1
      beginCrankStarter(ctx, "sync-stall")
    else
      beginFault(ctx, "SYNC_STALL")
    end
    return
  end
  local rpmT, idleEngage, blocked = syncTrack(ctx, dt)
  if blocked then
    st.blockedT = st.blockedT + dt
    if st.blockedT > P.SYNC_BLOCK_GIVEUP_S then
      st.lockoutT = max(st.lockoutT, P.SYNC_BLOCK_LOCKOUT_S)
      slog("[ICESEQ] sync blocked above the rev ceiling for %.1fs - giving up", st.blockedT)
      beginOff(ctx, "sync-blocked")
      return
    end
  else
    st.blockedT = 0
    st.syncT = st.syncT + dt
  end
  local ok
  if idleEngage then
    ok = m.rpmIce >= m.idle - P.IDLE_BAND_LO and m.rpmIce <= m.idle + P.IDLE_BAND_HI
  else
    ok = abs(m.rpmIce - rpmT) <= pv("syncTol") and abs(st.slipDot) <= P.SYNC_SLIPDOT_MAX
         and not ctx.isShifting and not blocked
  end
  if ok then st.syncHoldT = st.syncHoldT + dt else st.syncHoldT = 0 end
  if st.syncHoldT >= pv("syncHold") then
    beginEngage(ctx, idleEngage and "IDLE" or "NORMAL", "matched")
    return
  end
  -- Bounded SYNC (budget see P): an accelerating shaft may never settle; the
  -- rev-up share keeps a large rev-up from timing out while still on the ramp.
  if not st.syncBudget then
    st.syncBudget = pv("syncMax") + min(P.SYNC_RAMP_MAX_S, abs(rpmT - m.rpmIce) / max(1, pv("syncRefRate")))
  end
  local sMax = st.syncBudget
  if st.syncT > sMax and not blocked and not ctx.isShifting then
    if abs(m.slip) <= P.SYNC_SLIP_ENGAGE_RPM then
      beginEngage(ctx, "SLIP", "sync-timeout")
      return
    end
    if not st.syncTimeoutLogged then
      st.syncTimeoutLogged = true
      slog("[ICESEQ] SYNC slow after %.2fs rpmIce=%.0f target=%.0f slip=%.0f", st.syncT, m.rpmIce, rpmT, m.slip)
    end
    if st.syncT > sMax + P.SYNC_HARD_EXTRA_S then
      st.seqFault = "SYNC_TIMEOUT"
      slog("[ICESEQ] SYNC_TIMEOUT rpmIce=%.0f target=%.0f slip=%.0f - FORCED engage", m.rpmIce, rpmT, m.slip)
      beginEngage(ctx, "FORCED", "sync-timeout-hard")
    end
  end
end

-- ENGAGE: K0 closes onto a matched shaft; the fill cancels the K0 torque,
-- the governor holds the ICE on the reference until K0 carries it.
local function stEngage(ctx, dt)
  local m = st.m
  if not ctx.iceReq then
    clearIdleOverwrite()
    beginRelease(ctx, ctx.releaseProfile or "FAST", "OPEN", "withdrawn")
    return
  end
  if st.stallT >= P.STALL_S then beginRelease(ctx, "EMERGENCY", "OPEN", "stall"); return end
  -- IDLE engage: the ICE governor carries the creep torque (no fill, except
  -- guardFill)
  st.fill = (st.engageMode == "IDLE" and not guardFill(ctx)) and 0 or 1
  -- hold-and-rematch on large slip: NORMAL only (IDLE engages below idle)
  local slipBad = st.engageMode == "NORMAL" and abs(m.slip) > P.ENGAGE_ABORT_SLIP_RPM
                  and st.r < P.ENGAGE_ABORT_R
  -- SLIP / FORCED: the governor keeps tracking until the slip is small, so
  -- K0 need not drag the engine
  local slipGov = (st.engageMode == "SLIP" or st.engageMode == "FORCED")
                  and abs(m.slip) > P.SLIP_GOV_RPM and not st.lockConfirmed
  if st.r < P.GOV_HANDOVER_R or slipBad or slipGov then
    syncTrack(ctx, dt)
  else
    if st.govArmed then clearIdleOverwrite(); st.govArmed = false end
    st.override = nil      -- coordinator holds the ICE at zero net torque
  end

  if ctx.isShifting and st.r < P.ENGAGE_SHIFT_HOLD_R and not st.lockConfirmed then
    -- hold r: shaft speed jumps during the ratio change
  elseif st.engageMode == "FORCED" or st.engageMode == "SLIP" then
    local capRate = (st.engageMode == "SLIP") and P.K0_SLIP_NMS or P.K0_FORCED_NMS
    st.tcapF = st.tcapF + capRate * dt
    st.r = max(st.r, min(1, k0RatioFor(st.tcapF, m.slipAV)))
    -- once the slip is small the clutch closes like a normal engage
    if abs(m.slip) <= P.SLIP_GOV_RPM then st.r = min(1, st.r + pv("k0LockRate") * dt) end
    if st.lockConfirmed then st.r = min(1, st.r + P.K0_HARDEN_RATE * dt) end
  elseif slipBad then
    st.abortSlipT = st.abortSlipT + dt
    if st.abortSlipT > P.ENGAGE_ABORT_S then
      st.engageMode = "FORCED"
      st.tcapF = k0Cap(st.r, m.slipAV)
      slog("[ICESEQ] ENGAGE slip %.0frpm persists - FORCED", m.slip)
    end
  else
    local rate = (st.engageMode == "IDLE") and P.K0_IDLE_ENGAGE_RATE or pv("k0LockRate")
    if st.lockConfirmed then rate = max(rate, P.K0_HARDEN_RATE) end
    st.r = min(1, st.r + rate * dt)
  end

  if st.r >= P.KISS_R and not st.kissLogged then
    st.kissLogged = true
    st.stats.slipAtKiss = m.slip
  end
  if abs(m.slip) <= P.LOCK_SLIP_RPM and st.r >= P.LOCK_MIN_R then
    st.lockT = st.lockT + dt
  elseif not st.lockConfirmed then
    st.lockT = 0
  end
  if st.lockT >= P.LOCK_CONFIRM_S then st.lockConfirmed = true end
  if st.t > P.ENGAGE_MAX_S then
    if not st.engTimeoutLogged then
      st.engTimeoutLogged = true
      st.seqFault = "ENGAGE_TIMEOUT"
      slog("[ICESEQ] ENGAGE_TIMEOUT r=%.2f slip=%.0f - hardening", st.r, m.slip)
    end
    st.r = min(1, st.r + P.ENGAGE_TIMEOUT_RATE * dt)
  end
  if st.r >= P.ENGAGE_DONE_R and (st.lockConfirmed or st.t > P.ENGAGE_MAX_S) then
    st.r = 1
    clearIdleOverwrite()
    st.govArmed = false
    st.override = nil
    st.stats.slipAtLock = m.slip
    local ms = (st.now - st.tReq) * 1000
    st.stats.lockMs = ms
    ctx.iceLockMs = ms
    electrics.values.etk854hybrid_iceLockMs = ms
    beginBlend(ctx, 0, "locked")
  end
end

-- BLEND: ICE share in with smoothstep(x); torqueCoord takes the same torque
-- off the motor, the fill covers ICE lag.
local function stBlend(ctx, dt)
  st.r, st.fill, st.override = 1, 1, nil
  if st.stallT >= P.STALL_S or st.ice.isDisabled then
    beginRelease(ctx, "EMERGENCY", "OPEN", "stall")
    return
  end
  if not ctx.iceReq then beginRelease(ctx, ctx.releaseProfile or "NORMAL", "TORQUE", "withdrawn"); return end
  local tb = pv("tBlend")
  if (ctx.torqueDeficitNm or 0) > P.URGENT_DEFICIT_NM then
    tb = PROF.URGENT.tBlend
    st.stats.motorSat = true
  end
  -- traction control active: slower ramp
  local mtf = st.motor and tonumber(st.motor.throttleFactor) or 1
  local itf = tonumber(st.ice.throttleFactor) or 1
  if min(mtf, itf) < P.BLEND_TC_TF then
    tb = max(tb, P.T_BLEND_TC)
  end
  st.x = min(1, st.x + dt / max(tb, 0.05))
  st.g = smoothstep(st.x)
  if st.x >= 1 then beginOn(ctx) end
end

local function stOn(ctx, dt)
  st.r, st.g, st.fill, st.override = 1, 1, 1, nil
  st.onElapsed = st.onElapsed + dt
  ctx.iceReleaseBlocked = false
  if st.stallT >= P.STALL_S or st.ice.isDisabled or st.ice.isStalled then
    beginRelease(ctx, "EMERGENCY", "OPEN", "stall")
    return
  end
  if not ctx.iceReq then
    local prof = ctx.releaseProfile or "NORMAL"
    -- a NORMAL release may not dump torque the motor cannot take over
    if prof == "NORMAL" and (ctx.torqueTargetNm or 0) > P.RELEASE_BLOCK_FRAC * (ctx.motorCapNm or 0) then
      ctx.iceReleaseBlocked = true
      return
    end
    beginRelease(ctx, prof, "TORQUE", "withdrawn")
  end
end

-- RELEASE: TORQUE (share out) -> OPEN (K0 opens) -> ignition off or IDLE.
local function stRelease(ctx, dt)
  local m = st.m
  local prof = st.relProfile
  local overrun = (ctx.torqueTargetNm or 0) <= 1 or (ctx.throttleRaw or 0) < 0.02
  if ctx.iceReq and prof ~= "EMERGENCY" then
    -- still coupled: blend back in (not a new start)
    if st.relPhase == "TORQUE" then beginBlend(ctx, invSmoothstep(st.g), "re-request"); return end
    -- K0 partly open: counts as a restart
    openStart()
    st.variant = "REFIRE"
    recordFire(ctx)
    if st.r >= P.REREQ_ENGAGE_R and abs(m.slip) <= P.REREQ_ENGAGE_SLIP_RPM then beginEngage(ctx, "NORMAL", "re-request"); return end
    beginSync(ctx, "re-request")
    return
  end
  if st.relPhase == "TORQUE" then
    st.fill = 1
    local tRel = (prof == "NORMAL") and pv("tRelease") or REL[prof].tRelease
    if overrun then st.g = 0 else st.g = max(0, st.g - dt / max(tRel, 0.05)) end
    if st.g <= 0 then
      if abs(st.iceMeas) <= P.RELEASE_ZERO_NM then st.zeroT = st.zeroT + dt else st.zeroT = 0 end
      if overrun or st.zeroT >= P.RELEASE_ZERO_S or st.t >= P.RELEASE_TORQUE_TIMEOUT_S then
        st.relPhase = "OPEN"
        st.stop.openT0 = st.now
        slog("[ICESEQ] RELEASE TORQUE->OPEN iceMeas=%.0fNm overrun=%s", st.iceMeas, tostring(overrun))
      end
    end
    return
  end
  -- OPEN: fill on also in overrun - the recuperation drag credit only applies
  -- while locked, so the motor cancels the K0 drag here (no coast-decel bump)
  st.g, st.fill = 0, 1
  local rate = (prof == "NORMAL") and pv("k0OpenRate") or REL[prof].k0OpenRate
  st.r = max(0, st.r - rate * dt)
  if st.r <= P.K0_OPEN_EPS then
    st.r = 0
    local k0Nm = st.k0T
    local torqueMs = ((st.stop.openT0 or st.now) - (st.stop.t0 or st.now)) * 1000
    local openMs = (st.now - (st.stop.openT0 or st.now)) * 1000
    if ctx.iceIdleHold and prof ~= "EMERGENCY" and not st.ice.isDisabled then
      -- ICE-off hold: keep idling decoupled
      beginIdle(ctx, "released-hold")
      log("I", "etk854hybrid.iceSeq", string.format(
        "[ICESEQ] stop#%d prof=%s torque=%.0fms open=%.0fms k0NmAtOpen=%.1f -> IDLE (hold %.1fs)",
        st.stopN, prof, torqueMs, openMs, k0Nm, st.holdRemain))
      return
    end
    beginOff(ctx, "released")
    log("I", "etk854hybrid.iceSeq", string.format(
      "[ICESEQ] stop#%d prof=%s torque=%.0fms open=%.0fms k0NmAtIgnOff=%.1f ignOffAfterK0=1",
      st.stopN, prof, torqueMs, openMs, k0Nm))
  end
end

local function stIdle(ctx, dt)
  st.r = rampTo(st.r, 0, P.K0_CLEANUP_RATE, dt)
  st.g, st.override, st.fill = 0, 0, 0
  st.onElapsed = st.onElapsed + dt   -- engine run time (arbiter minimum)
  local ice = st.ice
  if ice.isDisabled or st.stallT >= P.STALL_S then
    beginOff(ctx, ice.isDisabled and "idle-disabled" or "idle-stall")
    return
  end
  if ctx.iceReq then
    -- engine running: rev-match, no crank
    openStart()
    st.variant = "REFIRE"
    setIgn(true)
    recordFire(ctx)
    beginSync(ctx, "idle-reengage")
    return
  end
  if not ctx.iceIdleHold then beginOff(ctx, "idle-hold-over") end
end

local function stFault(ctx, dt)
  st.r = rampTo(st.r, 0, P.K0_CLEANUP_RATE, dt)
  st.g, st.override, st.fill = 0, 0, 0
  if st.r <= 0 then beginOff(ctx, "fault-cleared") end
end

local STATES = {
  OFF = stOff, CRANK = stCrank, SYNC = stSync, ENGAGE = stEngage,
  BLEND = stBlend, ON = stOn, RELEASE = stRelease, IDLE = stIdle, FAULT = stFault,
}

-- Unsequenced start for bisecting regressions (ctx.iceSeqPassthrough):
-- OFF <-> ON in one frame, K0 ramps at 4/s, no fill.
local function stPassthrough(ctx, dt)
  local m = st.m
  if ctx.iceReq and st.state ~= "ON" then
    setIgn(true)
    if m.rpmIce < 200 then setStarter(true); st.ptStarterT = 0.8 end
    st.variant = "PASSTHROUGH"
    st.state, st.g = "ON", 1
  elseif not ctx.iceReq and st.state ~= "OFF" then
    setIgn(false)
    if st.starterOn then setStarter(false) end
    st.state, st.g = "OFF", 0
  end
  if st.ptStarterT > 0 then
    st.ptStarterT = st.ptStarterT - dt
    if st.ptStarterT <= 0 and st.starterOn then setStarter(false) end
  end
  st.r = rampTo(st.r, (st.state == "ON") and 1 or 0, 4.0, dt)
  st.override, st.fill = nil, 0
end

-- ===========================================================
-- Arbiter: policy -> debounced request ctx.iceReq
-- ===========================================================
local function deKey(ctx)
  local d = ctx.deMode
  if d == "ECO_PRO" or d == "SPORT" then return d end
  return "COMFORT"
end

local function arbWantOn(ctx, dt, urgent, bypass, dk)
  st.arbHoldT, st.holdRemain, st.holdKind = 0, 0, "none"
  st.abortT = 0
  ctx.iceIdleHold = false
  if ctx.iceReq then
    -- escalate a start in progress (kickdown)
    if urgent and (st.state == "CRANK" or st.state == "SYNC" or st.state == "ENGAGE" or st.state == "BLEND") then
      st.startUrgent = true
    end
    return
  end
  if st.state ~= "OFF" and st.state ~= "FAULT" then
    -- engine still turning (release, blocked release, IDLE): take it back
    ctx.iceReq = true
    st.arbOnT = 0
    return
  end
  st.arbOnT = st.arbOnT + dt
  local mk = (ctx.edriveMode == "BATTERY_CONTROL") and "BATTERY_CONTROL" or dk
  local deb = ARB.ON_DEBOUNCE_S[mk]
  -- short debounce while a K0 start is possible or on a fast pedal
  if ctx.iceK0Window or ctx.iceStartFast then deb = min(deb, ARB.K0_WINDOW_DEBOUNCE_S[mk]) end
  if urgent or bypass or (st.arbOnT >= deb and st.offElapsed >= ARB.MIN_OFF_S[dk]) then
    ctx.iceReq = true
    st.arbOnT = 0
    st.startUrgent = urgent
  end
end

-- ICE-off hold of this frame (see ARB): hold time, minimum run time, IDLE
-- allowed, hold kind (telemetry).
local function arbHoldTime(ctx, dk)
  local S = ctx.sportIndex or 0
  local slow = (ctx.speedKmh or 0) < ARB.LIFT_KMH[dk]
  local braking = (ctx.brakeRaw or 0) > ARB.BRAKE_THR
  -- easing off into the EV zone (ctx.iceLightLoad) counts as a lift
  local lift = slow and (braking or (ctx.throttleRaw or 0) < ARB.LIFT_THR
                         or ctx.iceLightLoad == true)
  st.arbSlow, st.arbLift, st.arbBraking = slow, lift, braking      -- arbFlags
  local base = lift and 0 or ARB.HOLD_MIN_S[dk]
  local hMid, hMax = ARB.H_MID_S[dk], ARB.HOLD_MAX_S[dk]
  local holdS = base + (hMid - base) * clamp01((S - ARB.H_MID_S0) / ARB.H_MID_SPAN)
                + (hMax - hMid) * smoothstep((S - ARB.H_MAX_S0) / ARB.H_MAX_SPAN)
  local minOnS = ARB.MIN_ON_S[dk]
  -- "sporty" = high S AND (outside SPORT) sporty window W: one kickdown
  -- lifts S to ~0.9 but W only by ~0.07
  local idleDe = (ARB.IDLE_DE[dk] or 1) > 0
  local idleOk = idleDe and S >= (lift and ARB.IDLE_S_MIN_LIFT or ARB.IDLE_S_MIN)
    and (dk == "SPORT" or (ctx.sportWindow or 0) >= ARB.IDLE_W_MIN)
  local kind = lift and "lift" or "hold"
  if lift and not idleOk then
    holdS = min(holdS, ARB.DECOUPLE_S[dk])
    minOnS = ARB.MIN_ON_LIFT_S
  elseif slow and not idleDe then
    -- no IDLE in this DE, no lift: coupled hold bounded like a lift
    holdS = min(holdS, ARB.DECOUPLE_S[dk])
    minOnS = min(minOnS, ARB.HOLD_MIN_S[dk])
  elseif not slow and not idleOk then
    -- above LIFT_KMH, not sporty: coupled hold and minimum run time capped
    -- to HOLD_MIN_S (less fuel-cut drag)
    minOnS = min(minOnS, ARB.HOLD_MIN_S[dk])
    holdS = min(holdS, ARB.HOLD_MIN_S[dk])
  end
  if lift and braking and S < (ARB.BRAKE_S_MAX[dk] or 0.50) then
    holdS, minOnS, kind = 0, min(minOnS, ARB.MIN_ON_LIFT_S), "brake"
  end
  return holdS, minOnS, idleOk, kind
end

local function arbWantOff(ctx, dt, hard, prof, dk)
  local hadIdle = ctx.iceIdleHold == true
  st.arbOnT = 0
  st.holdKind = "none"
  if hard then
    ctx.iceReq, ctx.iceIdleHold = false, false
    ctx.releaseProfile = prof
    st.holdRemain, st.arbHoldT, st.abortT = 0, 0, 0
    return
  end
  -- engine off or on its way off: nothing to hold
  if not ctx.iceReq and not ctx.iceIdleHold then st.holdRemain, st.abortT = 0, 0; return end
  -- a start in progress completes (the hold counts from BLEND/ON), except the
  -- optional SYNC abort (ARB.SYNC_ABORT_S)
  if ctx.iceReq and st.state ~= "ON" and st.state ~= "BLEND" then
    local ab = ARB.SYNC_ABORT_S[dk] or 0
    local realLift = (ctx.throttleRaw or 0) < ARB.LIFT_THR or (ctx.brakeRaw or 0) > ARB.BRAKE_THR
    if ab > 0 and st.state == "SYNC" and realLift and not ctx.iceForced then
      st.abortT = st.abortT + dt
      if st.abortT >= ab then
        ctx.iceReq, ctx.iceIdleHold, ctx.releaseProfile = false, false, "FAST"
        slog("[ICESEQ] arb sync-abort de=%s after %.2fs lift rpmIce=%.0f v=%.0f",
          dk, st.abortT, st.m.rpmIce, ctx.speedKmh or 0)
        st.abortT = 0
      end
    else
      st.abortT = 0
    end
    return
  end
  st.abortT = 0
  st.arbHoldT = st.arbHoldT + dt
  local holdS, minOnS, idleOk, kind = arbHoldTime(ctx, dk)
  st.holdKind = kind
  st.arbIdleOk = idleOk and 1 or 0
  st.holdRemain = max(0, holdS - st.arbHoldT)
  -- minimum run time is engine time (ON + IDLE), not coupling time
  local minOnLeft = max(0, minOnS - st.onElapsed)
  st.minOnLeft = minOnLeft
  if st.holdRemain <= 0 and minOnLeft <= 0 then
    -- hold over: release (coupled) or ignition off (IDLE)
    ctx.iceReq, ctx.iceIdleHold = false, false
    ctx.releaseProfile = "NORMAL"
  elseif not idleOk then
    -- no decoupled idle: a running IDLE ends (also one inherited on a DE
    -- switch), a coupled hold stays coupled until it runs out
    if not ctx.iceReq then ctx.iceIdleHold = false end
  elseif ctx.iceReq and st.arbHoldT >= ARB.DECOUPLE_S[dk]
      and max(st.holdRemain, minOnLeft) >= ARB.IDLE_MIN_S then
    -- spend the rest of the hold decoupled
    ctx.iceReq, ctx.iceIdleHold = false, true
    ctx.releaseProfile = "NORMAL"
  end
  -- log the cause of every decoupled idle
  if ctx.iceIdleHold and not hadIdle then
    slog("[ICESEQ] arb idle-hold de=%s kind=%s S=%.2f W=%.2f hold=%.1fs minOnLeft=%.1fs v=%.0f thr=%.2f brk=%.2f",
      dk, kind, ctx.sportIndex or 0, ctx.sportWindow or 0, st.holdRemain, minOnLeft,
      ctx.speedKmh or 0, ctx.throttleRaw or 0, ctx.brakeRaw or 0)
  end
end

function M.arbitrate(ctx, dt)
  if not ctx then return end
  dt = dt or 0
  if st.lockoutT > 0 then st.lockoutT = max(0, st.lockoutT - dt) end
  local want = ctx.policyWantsIce == true
  local hard = ctx.iceHardOff == true
  local urgent = ctx.iceStartUrgent == true
  local bypass, prof = false, "NORMAL"
  if ctx.sailIceRequest == "off" then
    want, hard, prof = false, true, "FAST"
  elseif ctx.sailIceRequest == "on" then
    want, bypass = true, true
    urgent = urgent or (ctx.sailRestartUrgent == true)
  end
  if (ctx.launchArmed or ctx.launchReady or ctx.launchActive) and not ctx.evOnly then
    want, urgent = true, true
  end
  -- trailer stability acting: keep the committed request (no start, no stop);
  -- sailing, QA override, lockout and key-off still win
  if ctx.tsaFreeze and ctx.sailIceRequest == nil and ctx.qaIceOverride == nil then
    want, hard, urgent = (ctx.iceReq == true), false, false
  end
  if hard and not ctx.iceForced then prof = "FAST" end   -- standstill, MAX EV
  if ctx.qaIceOverride ~= nil then
    want = (ctx.qaIceOverride == true)
    bypass = true
    hard = not want
  end
  -- lockout and key-off (ignitionLevel < 2) are hard FAST offs even when the
  -- policy forces the ICE (a NORMAL release could be blocked by stOn)
  if st.lockoutT > 0 then want, hard, prof = false, true, "FAST" end
  if (electrics.values.ignitionLevel or 2) < 2 then want, hard, prof = false, true, "FAST" end
  local dk = deKey(ctx)
  -- effective want after every override (decClass "unwanted")
  st.arbWant = want
  st.arbIdleOk, st.minOnLeft, st.arbSlow, st.arbLift, st.arbBraking = -1, 0, false, false, false
  if want then arbWantOn(ctx, dt, urgent, bypass, dk) else arbWantOff(ctx, dt, hard, prof, dk) end
  local ev = electrics.values
  ev.etk854hybrid_iceOffHoldS = st.holdRemain
  ev.etk854hybrid_iceHoldKind = st.holdKind            -- none | hold | lift | brake
  ev.etk854hybrid_arbIdleOk    = st.arbIdleOk          -- 1/0, -1 = not evaluated
  ev.etk854hybrid_arbIdleHold  = ctx.iceIdleHold and 1 or 0
  ev.etk854hybrid_arbHoldT     = st.arbHoldT
  ev.etk854hybrid_arbMinOnLeft = st.minOnLeft
  -- bits: 1 want, 2 hard, 4 urgent, 8 bypass, 16 forced, 32 lightLoad,
  -- 64 slow (< LIFT_KMH), 128 lift, 256 braking, 512 IDLE allowed in this DE
  ev.etk854hybrid_arbFlags = (want and 1 or 0) + (hard and 2 or 0) + (urgent and 4 or 0) + (bypass and 8 or 0)
    + (ctx.iceForced and 16 or 0) + (ctx.iceLightLoad and 32 or 0) + (st.arbSlow and 64 or 0)
    + (st.arbLift and 128 or 0) + (st.arbBraking and 256 or 0) + (((ARB.IDLE_DE[dk] or 1) > 0) and 512 or 0)
  -- committed (debounced) request for the 8HP gear planning; the raw want
  -- flickers at the motor corner speed and would make the gears hunt
  ev.etk854hybrid_iceRequested = ctx.iceReq and 1 or 0
end

-- ===========================================================
-- Decoupled-ICE diagnostics (read-only): per-frame class, segments (class
-- changes) and episodes (NONE -> ... -> NONE), logged as
--   decoupled#N            segment >= DIAG.LOG_MIN_S[CLASS]
--   decoupled-episode#N    episode >= EP_LOG_MIN_S
--   decoupled-summary      per DE window, at most every SUMMARY_S
-- LOG_MIN_S sits just above the normal phase duration: an ordinary start or
-- stop logs nothing.
-- ===========================================================
local DIAG = {
  RPM_FRAC = 0.30,          -- "turning" = rpmIce >= 0.30 x idle
  LOG_MIN_S = { IDLE_HOLD = 0, IDLE_MINON = 0, OFF_IGN_ON = 0, SYNC_BLOCKED = 0, FAULT = 0,
                SYNC_UNWANTED = 0.3, CRANK_UNWANTED = 0.3, RELEASE_OPEN = 0.6, SPINDOWN = 2.0,
                CRANK_K0 = 1.2, CRANK_STARTER = 2.0, SYNC = 1.0, SYNC_IDLE = 1.0, ENGAGE = 1.0, ENGAGE_IDLE = 1.5 },
  EP_LOG_MIN_S = 1.5, SUMMARY_S = 120, CHAIN_MAX = 8,
}

-- Class of a turning but decoupled ICE, NONE otherwise ("UNWANTED" = start
-- completing while the arbiter's effective want is off).
local function decClass()
  local s, m, ice = st.state, st.m, st.ice
  if not ice or m.rpmIce < DIAG.RPM_FRAC * m.idle then return "NONE" end
  if k0LockedState(s) then return "NONE" end
  if s == "ENGAGE" and st.r >= P.LOCK_MIN_R then return "NONE" end
  if s == "IDLE" then return (st.holdRemain > 0) and "IDLE_HOLD" or "IDLE_MINON" end
  if s == "RELEASE" then return "RELEASE_OPEN" end
  if s == "FAULT" then return "FAULT" end
  if s == "CRANK" then
    if not st.arbWant then return "CRANK_UNWANTED" end
    return (st.variant == "K0") and "CRANK_K0" or "CRANK_STARTER"
  end
  if s == "SYNC" then
    if st.syncBlocked then return "SYNC_BLOCKED" end
    if not st.arbWant then return "SYNC_UNWANTED" end
    return st.idleEngage and "SYNC_IDLE" or "SYNC"
  end
  if s == "ENGAGE" then return (st.engageMode == "IDLE") and "ENGAGE_IDLE" or "ENGAGE" end
  if s ~= "OFF" then return "NONE" end                     -- passthrough
  if (ice.ignitionCoef or 0) > 0 and not st.starterOn then return "OFF_IGN_ON" end
  return "SPINDOWN"
end

-- "DE:CLASS=sum s/n" pairs, sorted (DE in which the segment began)
local function fmtWin(win, winN)
  local keys = {}
  for k in pairs(win) do keys[#keys + 1] = k end
  table.sort(keys)
  local parts = {}
  for _, k in ipairs(keys) do
    parts[#parts + 1] = string.format("%s=%.1fs/%d", k, win[k], winN[k] or 0)
  end
  return table.concat(parts, " ")
end

local function updateDecDiag(ctx, dt)
  local d = st.dec
  local cls, dk = decClass(), deKey(ctx)
  if cls ~= d.why then
    if d.why ~= "NONE" then                                     -- segment end
      local dur, key = st.now - d.segT0, d.dk .. ":" .. d.why
      d.tot[key] = (d.tot[key] or 0) + dur
      d.cnt[key] = (d.cnt[key] or 0) + 1
      d.win[key] = (d.win[key] or 0) + dur
      d.winN[key] = (d.winN[key] or 0) + 1
      d.winAny = true
      if dur >= (DIAG.LOG_MIN_S[d.why] or 1.0) then
        slog("[ICESEQ] decoupled#%d %s %.2fs de=%s v=%.0f->%.0fkm/h rpmIce=%.0f->%.0f S=%.2f W=%.2f idleOk=%d hold=%s/%.1fs policy=%s req=%d unwanted=%.2fs next=%s",
          d.n, d.why, dur, d.dk, d.v0, ctx.speedKmh or 0, d.rpm0, st.m.rpmIce, d.S0, d.W0, d.idleOk0,
          d.kind0, d.hold0, d.reason0, d.req0, d.unwT, cls)
      end
    end
    if cls == "NONE" then                                       -- episode end
      local ep = st.now - d.epT0
      if ep >= DIAG.EP_LOG_MIN_S then
        slog("[ICESEQ] decoupled-episode#%d %.2fs de=%s chain=%s", d.epN, ep, d.epDk, d.chain)
      end
    else
      if d.why == "NONE" then                                   -- episode start
        d.epN, d.epT0, d.epDk, d.chain, d.chainN = d.epN + 1, st.now, dk, cls, 1
      elseif d.chainN < DIAG.CHAIN_MAX then
        d.chain, d.chainN = d.chain .. ">" .. cls, d.chainN + 1
      end
      d.n, d.segT0, d.dk, d.v0, d.rpm0 = d.n + 1, st.now, dk, ctx.speedKmh or 0, st.m.rpmIce
      d.S0, d.W0, d.idleOk0 = ctx.sportIndex or 0, ctx.sportWindow or 0, st.arbIdleOk
      d.kind0, d.hold0 = tostring(st.holdKind), st.holdRemain
      d.reason0, d.req0, d.unwT = tostring(ctx.icePolicyReason or "?"), ctx.iceReq and 1 or 0, 0
    end
    d.why = cls
  end
  if cls ~= "NONE" and not st.arbWant then d.unwT = d.unwT + dt end
  d.sumT = d.sumT + dt
  if dk ~= d.lastDk or d.sumT >= DIAG.SUMMARY_S then           -- window summary
    if d.winAny then
      slog("[ICESEQ] decoupled-summary de=%s %.0fs %s", d.lastDk, d.sumT, fmtWin(d.win, d.winN))
    end
    for k in pairs(d.win) do d.win[k] = nil end
    for k in pairs(d.winN) do d.winN[k] = nil end
    d.winAny, d.sumT, d.lastDk = false, 0, dk
  end
end

-- Lifetime totals per "DE:CLASS" since vehicle load:
-- controller.getControllerSafe('hybridControl').getCtx().modules.iceSeq.getDiag()
function M.getDiag()
  local d = st.dec
  return { why = d.why, episodes = d.epN, segments = d.n, tot = d.tot, cnt = d.cnt,
           orphanT = st.orphanT, arbIdleOk = st.arbIdleOk }
end

-- ===========================================================
-- Outputs
-- ===========================================================
local function updateReserve(dt)
  local want = 0
  if st.state == "CRANK" and st.variant == "K0" and not st.fired and not st.k0Abort then
    -- + CRANK_HEADROOM_NM: stCrankK0's abort test subtracts that margin, so
    -- crankNm alone would abort a minimum crank under a saturating demand
    want = st.crankNm + P.CRANK_HEADROOM_NM
  end
  if want >= st.reserve then
    st.reserve = want
  else
    st.reserve = max(want, st.reserve - P.RESERVE_RELEASE_NMS * dt)
  end
end

-- Starter guard: the arcade gearbox auto-cranks a stopped engine unless
-- engine.starterDisabled, so the flag is held in every state except while our
-- own starter is engaged. Not on a damaged engine (CE owns the flag there).
local function applyStarterGuard(ctx)
  local ice = st.ice
  if not ice or ice.isDisabled then return end
  if ctx.iceSeqPassthrough then
    ice.starterDisabled = (st.state == "OFF") and not st.starterOn
  else
    ice.starterDisabled = not st.starterOn
  end
end

local function trackStartStats(ctx)
  local s = st.state
  if st.inStart and (s == "CRANK" or s == "SYNC" or s == "ENGAGE" or s == "BLEND") then
    local motor = st.motor
    if motor and type(motor.etkShaftNm) == "number" then
      local tcmd = ctx.torqueTargetNm or 0
      -- without the EV creep floor (not part of the commanded torque)
      local shaftNm = motor.etkShaftNm - (motor.etkCreepNm or 0)
      local dev = abs(shaftNm - tcmd)
      if dev > st.stats.maxDev then st.stats.maxDev = dev end
      -- QA bound: vs what was available (motor cap + measured ICE/K0 torque)
      local ach = min(tcmd, (ctx.motorCapNm or 0) + st.iceMeas)
      local devA = abs(shaftNm - ach)
      if devA > st.stats.maxDevAch then st.stats.maxDevAch = devA end
    end
    if (ctx.torqueDeficitNm or 0) > P.URGENT_DEFICIT_NM then st.stats.motorSat = true end
  end
end

local function writeOutputs(ctx)
  local s = st.state
  local m = st.m
  ctx.iceEngageState = (s == "FAULT") and "OFF" or s
  ctx.iceOn = (s ~= "OFF" and s ~= "FAULT")
  ctx.iceCoupled = k0LockedState(s)
  ctx.iceShareGate = st.g
  ctx.iceThrottleOverride = st.override
  ctx.seqFillGain = st.fill
  ctx.seqFillInstant = (s == "CRANK" or s == "SYNC" or (s == "ENGAGE" and st.r < P.KISS_R))
  ctx.k0TorqueNm = st.k0T
  ctx.k0SlipRpm = m.slip
  ctx.iceMeasNm = st.iceMeas
  ctx.iceSyncBlocked = (s == "SYNC") and st.syncBlocked or false
  -- torqueCoord: fill 0 for an IDLE engage (see guardFill)
  ctx.iceIdleEngage = (s == "ENGAGE" and st.engageMode == "IDLE" and not guardFill(ctx))
    or (s == "SYNC" and st.idleEngage) or false
  ctx.iceRelPhase = (s == "RELEASE") and st.relPhase or nil
  ctx.k0Ratio = st.r
  ctx.iceRpmDev = m.rpmIce
  ctx.iceSeqFault = st.seqFault
  ctx.iceStartType = st.variant
  ctx.iceCrankReserveNm = st.reserve
  ctx.iceCrankNmEst = crankNmFor(PROF.COMFORT.crankAlpha)
end

local function publish(ctx)
  local ev = electrics.values
  local s = st.state
  ev.etk854hybrid_iceEngageState = ctx.iceEngageState or "OFF"
  ev.etk854hybrid_k0ClutchRatio = st.r
  ev.etk854hybrid_iceRestartActive = (s == "CRANK" or s == "SYNC" or s == "ENGAGE") and 1 or 0
  ev.etk854hybrid_iceStartType = st.variant
  ev.etk854hybrid_iceSeqFault = st.seqFault
  ev.etk854hybrid_iceSyncBlocked = ctx.iceSyncBlocked and 1 or 0
  -- motor fill loop measures combustion torque (1, K0 carries) or K0 torque (0)
  ev.etk854hybrid_fillLocked = (ctx.iceCoupled or (s == "ENGAGE" and st.lockConfirmed)) and 1 or 0
  -- decoupled-ICE diagnostics (shaft speed: etk854hybrid_pumpRpm of the motor)
  local d = st.dec
  local dec = d.why ~= "NONE"
  ev.etk854hybrid_iceDecoupledWhy      = d.why
  ev.etk854hybrid_iceDecoupledS        = dec and (st.now - d.segT0) or 0
  ev.etk854hybrid_iceDecoupledEpS      = dec and (st.now - d.epT0) or 0
  ev.etk854hybrid_iceDecoupledUnwanted = (dec and not st.arbWant) and 1 or 0
  ev.etk854hybrid_iceIgnition          = (st.ice and (st.ice.ignitionCoef or 0) > 0) and 1 or 0
end

function M.updateGFX(ctx, dt)
  if not ctx then return end
  dt = dt or 0
  st.now = st.now + dt
  if not st.ice then
    writeOutputs(ctx)
    publish(ctx)
    return
  end
  measure(ctx, dt)
  st.u = clamp01(((ctx.sportIndex or 0) - P.U_S0) / P.U_SPAN)
  st.prof = st.startUrgent and "URGENT" or (PROF[ctx.deMode] and ctx.deMode or "COMFORT")
  if ctx.iceSeqPassthrough then
    stPassthrough(ctx, dt)
  else
    local fn = STATES[st.state] or stOff
    fn(ctx, dt)
    st.t = st.t + dt
  end
  trackStartStats(ctx)
  updateReserve(dt)
  applyStarterGuard(ctx)
  writeOutputs(ctx)
  updateDecDiag(ctx, dt)
  publish(ctx)
end

-- External stall (master's N-charge recovery): EMERGENCY open -> OFF.
function M.onExternalStall(ctx)
  if st.state == "OFF" or st.state == "FAULT" then return end
  if st.state == "RELEASE" and st.relProfile == "EMERGENCY" then return end
  clearIdleOverwrite()
  beginRelease(ctx, "EMERGENCY", "OPEN", "external-stall")
end

-- ===========================================================
-- Lifecycle
-- ===========================================================
local function resetState(ctx)
  st.ice = powertrain and powertrain.getDevice and powertrain.getDevice("mainEngine") or nil
  st.k0 = powertrain and powertrain.getDevice and powertrain.getDevice("clutchK0") or nil
  st.motor = powertrain and powertrain.getDevice and powertrain.getDevice("integratedMotor") or nil
  st.gb = powertrain and powertrain.getDevice and powertrain.getDevice("gearbox") or nil
  clearIdleOverwrite()
  st.t, st.x, st.override, st.fill, st.reserve = 0, 0, 0, 0, 0
  st.startUrgent, st.fired, st.k0Abort, st.starterOn = false, false, false, false
  st.tcap, st.rI, st.tcapF, st.retries, st.pauseT, st.ptStarterT = 0, 0, 0, 0, 0, 0
  st.shiftWaitT = 0
  st.ref, st.sI, st.syncHoldT, st.syncT, st.blockedT, st.stallT = 0, 0, 0, 0, 0, 0
  st.lockT, st.abortSlipT, st.zeroT = 0, 0, 0
  st.k0T, st.iceMeas, st.slipDot, st.shDot, st.lastSlip, st.lastSh = 0, 0, 0, 0, 0, 0
  st.eEvent, st.eBudget = 0, 0
  st.onElapsed, st.offElapsed, st.lockoutT = 99, 99, 0
  st.arbOnT, st.arbHoldT, st.holdRemain, st.holdKind = 0, 0, 0, "none"
  st.arbWant, st.arbIdleOk, st.minOnLeft = false, -1, 0
  st.arbSlow, st.arbLift, st.arbBraking = false, false, false
  st.abortT, st.orphanT = 0, 0
  -- drop the open diag segment, keep the lifetime totals
  st.dec.why, st.dec.segT0, st.dec.epT0 = "NONE", st.now, st.now
  st.seqFault, st.variant = "none", "NONE"
  st.inStart = false
  st.idleEngage, st.syncBlocked, st.govArmed = false, false, false
  -- Spawn / reset: follow the engine device (CE may spawn it running), not
  -- electrics.engineRunning, which the 8HP reports as hybrid READY (1 in EV)
  local running = true
  if st.ice then
    local c = ctx and ctx.modules and ctx.modules.curves
    local idle = (c and c.iceIdleRPM and c.iceIdleRPM()) or 800
    local rpm = (st.ice.outputAV1 or 0) * AV_TO_RPM
    running = rpm >= 0.5 * idle
    st.m.rpmIce, st.m.idle = rpm, idle
  end
  if running then
    st.state, st.r, st.g = "ON", 1, 1
  else
    st.state, st.r, st.g = "OFF", 0, 0
  end
  if ctx then
    ctx.iceReq = running
    ctx.iceIdleHold = false
    ctx.releaseProfile = "NORMAL"
    ctx.iceReleaseBlocked = false
    ctx.iceFireMs, ctx.iceLockMs = 0, 0
    writeOutputs(ctx)
    publish(ctx)
  end
  electrics.values.etk854hybrid_iceOffHoldS = 0
  electrics.values.etk854hybrid_iceHoldKind = "none"
  electrics.values.etk854hybrid_iceRequested = (ctx and ctx.iceReq) and 1 or 0
  electrics.values.etk854hybrid_arbIdleOk = -1
  electrics.values.etk854hybrid_arbIdleHold = 0
  electrics.values.etk854hybrid_arbHoldT = 0
  electrics.values.etk854hybrid_arbMinOnLeft = 0
  electrics.values.etk854hybrid_arbFlags = 0
end

function M.init(ctx)
  resetState(ctx)
  electrics.values.etk854hybrid_iceRestartMs = 0
  electrics.values.etk854hybrid_iceFireMs = 0
  electrics.values.etk854hybrid_iceLockMs = 0
  if ctx and ctx.dbg then
    ctx.dbg(1, "iceSeq", string.format("init: K0 start policy %s, state %s", P.K0_START_POLICY, st.state))
  end
end

function M.initStage2(ctx)
  resetState(ctx)
  log("I", "etk854hybrid.iceSeq", string.format(
    "[ICESEQ] ready state=%s ice=%s k0=%s motor=%s K0model=%s policy=%s",
    st.state, tostring(st.ice ~= nil), tostring(st.k0 ~= nil), tostring(st.motor ~= nil),
    tostring(k0Model() ~= nil), P.K0_START_POLICY))
end

function M.reset(ctx)
  resetState(ctx)
end

function M.updateFixedStep(ctx, dt) end

-- Live tuning (not persistent, returns the old value / nil):
-- M.setParam('CRANK_DRAG_FRAC', 0.6)
function M.setParam(key, value)
  if type(P[key]) ~= "number" or type(value) ~= "number" then return nil end
  local old = P[key]
  P[key] = value
  return old
end

-- M.setProf('URGENT', 'syncRefRate', 9000)
function M.setProf(prof, key, value)
  local t = PROF[prof]
  if not t or type(t[key]) ~= "number" or type(value) ~= "number" then return nil end
  local old = t[key]
  t[key] = value
  return old
end

-- M.setArb('IDLE_DE', 'ECO_PRO', 1) per-DE entry, M.setArb('IDLE_W_MIN', 0.2) scalar
function M.setArb(key, a, b)
  local t = ARB[key]
  if type(t) == "table" then
    if type(t[a]) ~= "number" or type(b) ~= "number" then return nil end
    local old = t[a]
    t[a] = b
    return old
  end
  if type(t) ~= "number" or type(a) ~= "number" then return nil end
  ARB[key] = a
  return t
end

function M.getState()
  return {
    state = st.state, variant = st.variant, r = st.r, g = st.g, relPhase = st.relPhase,
    engageMode = st.engageMode, starterOn = st.starterOn, retries = st.retries,
    fault = st.seqFault, lockoutT = st.lockoutT, holdRemain = st.holdRemain, holdKind = st.holdKind,
    onElapsed = st.onElapsed, offElapsed = st.offElapsed, eBudget = st.eBudget,
    reserve = st.reserve, starts = st.startN, stops = st.stopN,
    decoupledWhy = st.dec.why, arbIdleOk = st.arbIdleOk, minOnLeft = st.minOnLeft,
  }
end

return M
