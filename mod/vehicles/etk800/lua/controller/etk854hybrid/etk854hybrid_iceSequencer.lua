-- etk854hybrid_iceSequencer.lua - ETK 854 ActiveHybrid - ICE start/stop sequencer
--
-- User requirement behind this module: "The engine must match its speed to
-- the shaft BEFORE the clutch closes onto the driveline." The old master did
-- a start in ONE frame: starter crank (rpm < 200) + K0 closing at 4/s against
-- a stopped engine + the torque request moved to the (not yet running) ICE.
-- The K0 slip-damping term (frictionClutch.lua:104, up to lockTorque) then
-- decelerated the whole driveline = the jerk the driver felt.
--
-- Sequence (published as etk854hybrid_iceEngageState):
--   OFF     K0 open, ignition off
--   CRANK   variant K0      : E-machine drags the ICE up through a slipping
--                             K0 (torque-controlled capacity, the motor fill
--                             loop cancels the crank torque at the shaft)
--           variant STARTER : K0 open, 12 V starter, nothing reaches the shaft
--   SYNC    K0 OPEN, rev-match: ICE speed controlled onto shaft + offset
--           (idle governor with idleAVOverwrite = reference that follows the
--            shaft's own rate + a rate-limited approach, inertia feed-forward
--            + integral on the VCU throttle channel). Bounded: after the
--            profile's syncMax (0.4-0.7 s) it engages anyway with controlled
--            K0 slip (mode SLIP) - an accelerating shaft never "settles"
--   ENGAGE  rpm matched, K0 closes (governor handed over at r >= 0.3)
--           (SYNC / ENGAGE under the launch module's brake-throttle guard:
--            reference never ahead of the shaft, fill on above the idle
--            band - see guardOn / guardFill)
--   BLEND   K0 locked, ICE torque share ramped in (smoothstep, 0.2..1.0 s)
--   ON
--   RELEASE TORQUE: ICE share ramped out -> OPEN: K0 opens with the ICE held
--           at zero net torque -> ignition cut ONLY after K0 is open
--   IDLE    ICE-off hold after sporty driving (sport index >= ARB.IDLE_S_MIN)
--           or for the rest of the minimum run time in such a phase
--           (arbiter ctx.iceIdleHold): K0 OPEN, ignition
--           on, the engine idles on its own governor - no fuel-cut drag on
--           the driveline; a new request re-engages it
--           without a crank (REFIRE: SYNC -> ENGAGE -> BLEND); the end of the
--           hold switches the ignition off (-> OFF).
--           COMFORT / SPORT only (ARB.IDLE_DE): ECO PRO never idles the ICE
--           decoupled, its hold is spent coupled and then released -> OFF
--           (feedback round 3); an IDLE inherited from COMFORT ends on the
--           first ECO PRO arbiter frame
--   (FAULT is internal and published as OFF with etk854hybrid_iceSeqFault)
--
-- Decoupled-ICE diagnostics (feedback round 3, the user plays without a
-- telemetry recording): every frame in which the engine turns (>= DIAG.RPM_FRAC
-- x idle) without carrying the driveline gets a class
-- (etk854hybrid_iceDecoupledWhy, see decClass); long segments, episodes and a
-- per-Driving-Experience summary go to beamng.log as [ICESEQ] decoupled* lines
-- (rate-limited), M.getDiag() returns the lifetime totals.
--
-- The sequencer is the ONLY writer of etk854hybrid_k0ClutchRatio, of the
-- ignition/starter calls, of ice.idleAVOverwrite / maxIdleThrottleOverwrite
-- and of ice.starterDisabled (the master's failsafe branch is the documented
-- exception, active only when this file failed to load).
--
-- M.arbitrate(ctx, dt) turns the mode policy into the debounced request
-- ctx.iceReq (replaces the old 2 s "time since last transition" dwell):
-- on-debounce, minimum on/off times, and a sportiness-scaled ICE-off hold.
--
-- Contract: M.isReal, init / initStage2 / updateGFX / reset / getState,
-- arbitrate(ctx, dt), onExternalStall(ctx), getStats(), getDiag(); live
-- levers setParam / setProf / setArb (not persistent).
-- All constants in P / PROF / REL / ARB, all state in st (LuaJIT: 60
-- upvalues per function - every function here stays far below 30).

local M = {}
M.name = "iceSequencer"
M.isReal = true

local abs, max, min, sqrt, exp = math.abs, math.max, math.min, math.sqrt, math.exp
local RPM_TO_AV = 0.104719755
local AV_TO_RPM = 9.549296596

-- ===========================================================
-- Parameters (P1 spec section 6 "SEQUENCER")
-- ===========================================================
local P = {
  -- 'ALWAYS' = production-style warm restart via the E-machine whenever it can be
  -- done; the 12 V starter only where K0 cannot be used. 'URGENT_ONLY' =
  -- K0 start only for urgent starts (kickdown, sail exit, SOC force).
  K0_START_POLICY     = "ALWAYS",
  K0_START_MIN_FRAC   = 1.3,     -- shaft >= max(1.3*idle, idle + 250) rpm, else
  K0_START_MIN_ADD    = 250,     -- the shaft cannot drag the ICE to firing speed
  K0_START_RESERVE_NM = 30,      -- motor headroom needed = crankNm + this
  CRANK_HEADROOM_NM   = 10,      -- reserve held back from the driver = crankNm + this
  CRANK_MIN_NM        = 70,
  CRANK_MAX_NM        = 220,
  CRANK_DRAG_FRAC     = 1.0,     -- share of the drag estimate put into the crank torque
  -- K0 start while the 8HP DOWNshifts, or through any shift for an urgent
  -- (kickdown) start: the crank torque is cancelled by the motor fill, which
  -- now stays on during shifts in the slip states (torqueCoord
  -- updateFillGain). 0 = starter as before.
  K0_START_IN_DOWNSHIFT = 1,
  K0_SHIFT_WAIT_S     = 0.5,     -- a non-urgent start waits this long for an upshift to end
  K0_CAP_DOWN_NMS     = 3000,    -- crank capacity ramp-down after fire
  K0_CAP_KI           = 0.004,   -- [1/(Nm*s)] measured-torque integral on r
  K0_CAP_I_MAX        = 0.2,
  K0_CRANK_RRATE      = 4.0,     -- [1/s] K0 ratio slew while cranking
  K0_CRANK_RMAX       = 0.6,
  SLIP_EVENT_J        = 15000,   -- K0 thermal protection: per start ...
  SLIP_BUDGET_J       = 60000,   -- ... and leaky budget (typ. crank 4-6 kJ)
  SLIP_BUDGET_TAU_S   = 120,
  FIRE_FRAC           = 0.90,    -- "fired" = rpm >= 0.90*idle held FIRE_HOLD_S
  FIRE_HOLD_S         = 0.05,
  REFIRE_FRAC         = 0.60,    -- still spinning: ignition-only restart
  STALL_FRAC          = 0.45,    -- rpm < 0.45*idle for STALL_S = stalled
  STALL_S             = 0.2,
  STARTER_ENGAGE_FRAC = 0.78,    -- VC:690-698 refuses the starter above 0.8*starterMaxAV
  -- Warm restart through the 12 V starter: the stock engine kills the
  -- throttle for starterThrottleKillTime after the starter engages (CE:999-
  -- 1003, 0.5 s default; the R6 ECUs set 0.30, the i4 keeps 0.5 -> fire
  -- 583 ms). A hybrid's warm engine fires on the first compressions; with a
  -- warm block the kill window is cut to STARTER_WARM_KILL_S. A cold engine
  -- keeps the stock (x2 / x4 cold-scaled) crank.
  STARTER_WARM_KILL_S = 0.25,
  STARTER_WARM_BLOCK_C = 40,
  RETRY_PAUSE_S       = 0.4,
  MAX_RETRIES         = 2,
  FAULT_LOCKOUT_S     = 5,
  FAULT_WINDOW_S      = 60,
  FAULT_BURST_N       = 3,
  FAULT_BURST_LOCKOUT_S = 30,
  SYNC_LEAD_S         = 0.08,    -- shaft rpm prediction (derivative LPF 0.05 s)
  -- The reference follows the TARGET'S OWN RATE (shaft acceleration, clamped)
  -- plus the rate-limited approach below. Without this feed-forward the
  -- reference only closed at syncRefRate - shDot on an accelerating shaft
  -- (cruise_100 2026-09-30: the ICE chased the 1st-gear shaft into the rev
  -- ceiling and SYNC ran for 6 s).
  SYNC_TGTDOT_MAX     = 4000,    -- [rpm/s] clamp of the target-rate feed-forward
  SHDOT_TAU_S         = 0.05,
  SLIPDOT_TAU_S       = 0.03,
  SYNC_CEIL_MARGIN_RPM = 500,    -- never rev-match closer than this to iceMaxRPM
  SYNC_BLOCK_GIVEUP_S = 5,
  SYNC_BLOCK_LOCKOUT_S = 3,
  -- Reference landing: the rev-match reference approaches its target with
  -- rate = min(SYNC_REF_RATE, sqrt(2 * SYNC_REF_DECEL * |error|)), so the
  -- inertia feed-forward tapers out BEFORE the target instead of dropping
  -- to zero at it (a linear ramp overshot ~130 rpm in the offline harness:
  -- ICE throttle smoother + turbo lag keep pushing after the ramp ends).
  SYNC_REF_DECEL      = 12000,   -- [rpm/s^2]
  SYNC_KI             = 0.002,   -- [1/(rpm*s)] integral on the VCU throttle
  SYNC_I_MIN          = -0.10,
  SYNC_I_WINDOW_RPM   = 300,     -- integral frozen outside +-300 rpm
  SYNC_SLIPDOT_MAX    = 1500,    -- [rpm/s] slip must be settled, not crossing
  -- SYNC budget = profile syncMax + rev-up allowance |target - rpmIce| /
  -- syncRefRate at SYNC entry (capped SYNC_RAMP_MAX_S). After the budget:
  -- |slip| <= SYNC_SLIP_ENGAGE_RPM -> SLIP engage (planned strategy, never a
  -- fault); larger slip keeps syncing up to budget + SYNC_HARD_EXTRA_S, then
  -- FORCED - the only path that sets seqFault SYNC_TIMEOUT
  SYNC_SLIP_ENGAGE_RPM = 400,
  SYNC_HARD_EXTRA_S   = 0.5,
  SYNC_RAMP_MAX_S     = 1.0,     -- cap of the rev-up allowance added to syncMax
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
  KISS_R              = 0.2,     -- stats: slip when K0 starts to carry
  LOCK_SLIP_RPM       = 30,
  LOCK_CONFIRM_S      = 0.04,
  LOCK_MIN_R          = 0.5,
  K0_HARDEN_RATE      = 8.0,     -- r -> 1 once locked (no slip left = no torque step)
  URGENT_DEFICIT_NM   = 10,      -- motor saturated -> URGENT blend time
  -- Traction-aware blend: while the DSE traction control trims the driven
  -- axle (E-machine or ICE throttle factor below BLEND_TC_TF, written by
  -- motorTorqueControl on the devices) the ICE share ramps in no faster than
  -- over T_BLEND_TC. The URGENT blend (0.15-0.2 s) put ~400 Nm of crank
  -- torque onto an axle that was already at the slip threshold; the ICE
  -- trim acts through the turbo and is too slow for that step (QA T18 B58
  -- 2026-10: slip 0.06 -> 0.27 within 0.15 s at BLEND end). T18 B58
  -- slipPeakLate over 4 runs: no TC blend ~0.255, 0.6 s ~0.24, 0.9 s ~0.21.
  -- Also slows the kickdown blend on a slipping axle, never a dry start.
  BLEND_TC_TF         = 0.97,
  T_BLEND_TC          = 0.90,
  RELEASE_ZERO_NM     = 15,
  RELEASE_ZERO_S      = 0.05,
  RELEASE_TORQUE_TIMEOUT_S = 1.0,
  RELEASE_BLOCK_FRAC  = 0.95,    -- NORMAL release blocked while T_cmd > 0.95*motorCap
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
  -- OFF with the ignition switched on behind the sequencer (nothing in this
  -- file does that; external writers: the VC stall handling on an ignition
  -- level change, a dev rig) and the engine running (>= REFIRE_FRAC x idle):
  -- the ignition is cut after this long and logged. Longer than FIRE_HOLD_S
  -- and any frame-level ignition transient; 0 disables it (setParam).
  ORPHAN_IGN_S        = 0.25,
}

-- Per start profile. COMFORT values of the LERP keys move toward URGENT
-- with u = clamp01((sportIndex - 0.30) / 0.50).
-- 2026-09-30: syncMax 1.5/1.5/0.9/0.9 -> 0.6/0.7/0.45/0.4 (series car: coupled
-- within ~0.6-1.0 s of the request), faster rev-up and K0 close in COMFORT /
-- ECO PRO (the ICE is matched when K0 closes, so a faster close adds no step).
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
-- Release profiles other than NORMAL (NORMAL uses the start profile table)
local REL = {
  FAST      = { tRelease = 0.25, k0OpenRate = 5.0 },
  EMERGENCY = { tRelease = 0.0,  k0OpenRate = 20.0 },
}
-- Arbiter (spec 4.2)
local ARB = {
  ON_DEBOUNCE_S = { COMFORT = 0.15, ECO_PRO = 0.25, SPORT = 0.15, BATTERY_CONTROL = 0.50 },
  MIN_ON_S      = { COMFORT = 3.0,  ECO_PRO = 2.0,  SPORT = 3.0 },
  MIN_OFF_S     = { COMFORT = 1.0,  ECO_PRO = 1.5,  SPORT = 1.0 },
  HOLD_MIN_S    = { COMFORT = 2.0,  ECO_PRO = 1.5,  SPORT = 2.0 },
  HOLD_MAX_S    = { COMFORT = 20.0, ECO_PRO = 8.0,  SPORT = 20.0 },
  -- ICE-off hold after the policy said "off" (arbHoldTime), scaled by the
  -- sportiness index S (2026-10, user feedback "unter 90 aus dem Gas ...";
  -- the old hold 2 + 18 * smoothstep((S - 0.20) / 0.55) depended on S only):
  --   holdS = base + (H_MID - base) * clamp01((S - H_MID_S0) / H_MID_SPAN)
  --                + (HOLD_MAX - H_MID) * smoothstep((S - H_MAX_S0) / H_MAX_SPAN)
  --   base  = 0 on a LIFT (pedal < LIFT_THR, brake > BRAKE_THR or a pedal
  --           eased into the EV zone, ctx.iceLightLoad) below LIFT_KMH,
  --           HOLD_MIN otherwise (above LIFT_KMH, or a pedal above the EV zone)
  --   minOn above LIFT_KMH without decoupled idle: min(MIN_ON_S, HOLD_MIN_S)
  --   braking below LIFT_KMH with S < BRAKE_S_MAX[DE] -> hold 0, re-evaluated
  --           every frame, so it also ends an IDLE hold in progress. ECO PRO
  --           1.01 = above any reachable S: in ECO every brake ends the hold
  --           (FAHRMODI promise; the old scalar 0.50 never overlapped the
  --           S >= 0.60 lift gate, a sporty ECO stop idled to standstill)
  --   COMFORT lift below 90 km/h: S 0.1 -> 0 s, 0.3 -> 1.2 s, 0.5 -> 4.9 s,
  --           >= 0.8 -> 20 s; above 90 km/h at S < 0.2: 2.0 s as before
  --   ECO PRO (IDLE_DE 0, no decoupled idle): the hold is spent COUPLED, at
  --           most DECOUPLE_S below LIFT_KMH (lift or not) and HOLD_MIN_S
  --           above, then RELEASE -> OFF
  LIFT_KMH      = { COMFORT = 90.0, ECO_PRO = 100.0, SPORT = 0.0 },
  H_MID_S       = { COMFORT = 4.0,  ECO_PRO = 2.0,  SPORT = 4.0 },
  H_MID_S0 = 0.20, H_MID_SPAN = 0.30, H_MAX_S0 = 0.45, H_MAX_SPAN = 0.35,
  LIFT_THR = 0.02, BRAKE_THR = 0.05,
  BRAKE_S_MAX   = { COMFORT = 0.50, ECO_PRO = 1.01, SPORT = 0.50 },
  -- Decoupled idle allowed in this Driving Experience (1) or never (0).
  -- ECO PRO 0 (feedback round 3, gamepad session: 9 of 12 ECO releases went
  -- RELEASE -> IDLE, up to 10.5 s): a full-trigger press takes S past 0.60 in
  -- ~1.1 s and W grows 1/30 per s while e > 0.5 but only shrinks at S < 0.2,
  -- so the sporty-phase gate below stayed open for minutes. ECO PRO =
  -- earliest ICE-off; a decoupled idle burns ~0.9 l/h for nothing.
  -- Live A/B: M.setArb('IDLE_DE', 'ECO_PRO', 1) restores the old behaviour.
  IDLE_DE       = { COMFORT = 1, ECO_PRO = 0, SPORT = 1 },
  -- Optional abort of an unwanted start in SYNC (K0 open, engine fired) after
  -- a real lift / brake lasting this long [s]; 0 = off (default everywhere:
  -- a start in progress completes). Enable per DE only if the live log shows
  -- SYNC_UNWANTED episodes >= 1 s: M.setArb('SYNC_ABORT_S', 'ECO_PRO', 0.4).
  SYNC_ABORT_S  = { COMFORT = 0, ECO_PRO = 0, SPORT = 0 },
  -- minimum run time on a lift below LIFT_KMH while no decoupled idle is
  -- allowed (MIN_ON_S would otherwise keep a fresh start coupled in fuel cut)
  MIN_ON_LIFT_S = 0.5,
  -- Hold = decoupled idle (2026-09-30): once the policy has said "off" for
  -- DECOUPLE_S the K0 opens and the engine idles for the rest of the hold
  -- instead of dragging the car in fuel cut. A shorter lift (gear change,
  -- a breath between two throttle applications, corner entry) keeps it
  -- coupled, so pumping the pedal does not cycle the K0. Only in a DE with
  -- IDLE_DE 1, when at least
  -- IDLE_MIN_S of hold would be left AND S >= IDLE_S_MIN (a genuinely sporty
  -- phase, where the H_MAX part of the hold starts to grow). Below IDLE_S_MIN
  -- a running IDLE ends (-> OFF) and a lift below LIFT_KMH is capped to the
  -- coupled DECOUPLE_S window: an engine idling decoupled while a calm
  -- driver rolls electrically is what the user reported (QA T24A 2026-09-30:
  -- 3 x 1.6 s IDLE at S 0.30, pedal 0.10, 75-82 km/h).
  -- IDLE_S_MIN_LIFT: the same gate for a lift below LIFT_KMH. 0.45 was
  -- reached by any sustained ~0.72 pedal (sportIndex uDem = smoothstep((p -
  -- 0.5) / 0.45)), so an ordinary brisk merge idled the engine decoupled for
  -- ~3.7 s after the lift (review 2026-10). 0.60 needs a sustained >= 0.76
  -- pedal. QA T20a (lift at S 0.92-0.93, S tau ~27 s) keeps IDLE until
  -- S 0.60 is reached ~12 s later, past its 8 s criterion.
  DECOUPLE_S    = { COMFORT = 1.2,  ECO_PRO = 0.8,  SPORT = 1.2 },
  IDLE_MIN_S = 1.0,
  IDLE_S_MIN = 0.45, IDLE_S_MIN_LIFT = 0.60,
  -- ... and outside SPORT a sporty WINDOW (sportIndex W, ctx.sportWindow)
  -- of at least this: a single kickdown is not a sporty phase (CR-VCU1)
  IDLE_W_MIN = 0.12,
  -- On-debounce while a K0 start is possible right now (k0WindowOpen): a
  -- tip-in ramps the delivered torque at 750-2500 Nm/s, so the regular
  -- 0.15 / 0.25 s debounce let the ramp eat the crank headroom and the start
  -- fell back to the 12 V starter.
  K0_WINDOW_DEBOUNCE_S = { COMFORT = 0.05, ECO_PRO = 0.10, SPORT = 0.05, BATTERY_CONTROL = 0.50 },
}

-- ===========================================================
-- State
-- ===========================================================
local st = {
  ice = nil, k0 = nil, motor = nil,
  state = "OFF", variant = "NONE", prof = "COMFORT", u = 0,
  relProfile = "NORMAL", relPhase = "TORQUE", engageMode = "NORMAL",
  r = 0, g = 0, x = 0, t = 0, now = 0,
  override = 0, fill = 0, reserve = 0, crankNm = 120,
  tReq = 0, startUrgent = false, fired = false, fireHoldT = 0, retries = 0,
  tcap = 0, rI = 0, k0Abort = false, tcapF = 0,
  starterOn = false, crankT = 0, crankFrames = 0, crankTimeout = 1.5, pauseT = 0, shiftWaitT = 0,
  ref = 0, sI = 0, rpmT = 0, idleEngage = false, syncBlocked = false,
  syncHoldT = 0, syncT = 0, blockedT = 0, stallT = 0, syncTimeoutLogged = false,
  lockT = 0, lockConfirmed = false, abortSlipT = 0, kissLogged = false,
  govArmed = false, engTimeoutLogged = false, zeroT = 0,
  k0T = 0, iceMeas = 0, slipDot = 0, shDot = 0, lastSlip = 0, lastSh = 0,
  eEvent = 0, eBudget = 0,
  onElapsed = 99, offElapsed = 99, lockoutT = 0,
  arbOnT = 0, arbHoldT = 0, holdRemain = 0, holdKind = "none",
  -- arbiter diagnostics of the current frame (M.arbitrate): the effective
  -- want after every override, IDLE gate 1/0 (-1 = not evaluated), minimum
  -- run time left, lift-rule inputs; SYNC abort timer; orphan ignition timer
  arbWant = false, arbIdleOk = -1, minOnLeft = 0, arbSlow = false, arbLift = false, arbBraking = false,
  abortT = 0, orphanT = 0,
  seqFault = "none", faultTimes = {},
  startN = 0, stopN = 0, inStart = false,
  ptStarterT = 0,
  logWinT = 0, logN = 0,
  m = { rpmIce = 0, rpmSh = 0, slip = 0, slipAV = 0, idle = 800, iMax = 7000,
        Iice = 0.5, dragEst = 40, k0raw = 0 },
  stats = {},
  stop = {},
  -- decoupled-ICE diagnostics (updateDecDiag): current class / segment /
  -- episode, lifetime totals per "DE:CLASS" (survive reset), the summary
  -- window (per DE, at most DIAG.SUMMARY_S)
  dec = { why = "NONE", n = 0, epN = 0, segT0 = 0, epT0 = 0, epDk = "COMFORT", chain = "", chainN = 0,
          dk = "COMFORT", v0 = 0, rpm0 = 0, S0 = 0, W0 = 0, idleOk0 = -1, kind0 = "none", hold0 = 0,
          reason0 = "?", req0 = 0, unwT = 0,
          tot = {}, cnt = {}, win = {}, winN = {}, winAny = false, sumT = 0, lastDk = "COMFORT" },
}

-- ===========================================================
-- Helpers
-- ===========================================================
local function clamp(v, lo, hi)
  if v < lo then return lo end
  if v > hi then return hi end
  return v
end

local function clamp01(v)
  if v ~= v then return 0 end
  if v < 0 then return 0 end
  if v > 1 then return 1 end
  return v
end

local function rampTo(cur, target, rate, dt)
  if cur < target then return min(target, cur + rate * dt) end
  return max(target, cur - rate * dt)
end

local function smoothstep(t)
  t = clamp01(t)
  return t * t * (3 - 2 * t)
end

-- Inverse of smoothstep on [0,1] (monotonic -> bisection). Used when a
-- RELEASE/TORQUE share gate g is handed back to BLEND, whose state is the
-- PHASE x with g = smoothstep(x): passing g itself as x would step g.
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

-- Transition / event log with a hard per-second budget (a misbehaving loop
-- must never flood beamng.log).
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

-- Ignition / starter through the vehicleController (VC:690-713), edge-based
-- on the REAL device state so an external ignition change cannot desync us.
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

-- Every exit of SYNC/ENGAGE, OFF, FAULT, reset and external stall. CE reset
-- zeroes idleAVOverwrite (CE:1333) but NOT maxIdleThrottleOverwrite.
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

-- Brake-throttle guard (etk854hybrid_launchControl updateGuard: brake +
-- pedal at a standstill, ctx.launchGuardNm = cap on the torque into the
-- converter; nil during the launch hold, whose PRELOAD loads the converter
-- on purpose). torqueCoord caps the motor request and the ICE share
-- (BLEND / ON) at the cap; the slip states have to keep the ICE out of it:
-- while the guard is on, the rev-match never runs ahead of the shaft (no
-- SYNC_LEAD_S lead on a rising shaft, no syncOffset) and the motor fill
-- cancels the K0 torque in every ENGAGE mode once the motor has spun the
-- shaft out of the idle band (guardFill; an IDLE engage has fill 0 otherwise,
-- writeOutputs then withholds ctx.iceIdleEngage). The value is the launch
-- module's previous frame (it runs after the sequencer).
-- Why: manual arm in S, brake 1.0, pedal 1.0, B58 (launch_hold_manual_s
-- 2026-09-30 20:33:40): the starter start engaged as IDLE (shaft < idle - 50)
-- while the capped motor (151 Nm) spun the shaft up at ~5000 rpm/s; once the
-- shaft passed idle - 50 the governor reference became shaft + 0.08 s x
-- shDot + 50 rpm, i.e. ~450 rpm ahead with the URGENT governor at full
-- authority. The sticking K0 (slip 0 +- 11 rpm, r 0.12 -> 0.28) passed the
-- ICE torque on (K0 = shaft - motor: +24 -> +88 -> +150 -> +220 Nm), which
-- accelerated the shaft further and moved the reference again: shaft 652 ->
-- 1995 rpm in 0.27 s, converter pump load 334 Nm (= 2.2 x the 154 Nm cap),
-- the brakes slipped (2.75 km/h, turbine 598 rpm) until the governor handed
-- over at r 0.30. With the reference at max(shaft, idle) the ICE carries
-- only its own inertia; the shaft settles where the pump load equals the
-- motor's capped torque, as in pure EV.
local function guardOn(ctx)
  return ctx.launchGuardNm ~= nil and not ctx.launchHold
end

-- Fill of an IDLE engage under the guard: only with the shaft at or above
-- idle - IDLE_ENGAGE_MARGIN (the per-frame idle test of syncTrack). Below
-- it the ICE governor creeps the shaft up to idle as before - the converter
-- load there is the pump load at idle (~40 Nm) - and a fill against the
-- governor (motor in regen) would stall the engine when the guard cuts the
-- cap to 0 (launch anti-creep cut; offline: IDLE engage -> stall -> crank).
local function guardFill(ctx)
  local m = st.m
  return guardOn(ctx) and m.rpmSh >= m.idle - P.IDLE_ENGAGE_MARGIN
end

-- ===========================================================
-- K0 capacity model = stock frictionClutch (FC:82-110, calculateInertia
-- FC:189-194). With persistent slip the clutch angle sits on its bound
-- maxClutchAngle*r, so   T = min(LTeff, spring(a) + lockDamp*|slipAV|*r)
-- with a = maxClutchAngle*r, spring = a<=1 and a^2*lockSpring or a*lockSpring.
-- lockSpring / lockDamp / maxClutchAngle are runtime fields (the jbeam keys
-- maxClutchAngle / lockDamp are NOT read by frictionClutch.lua).
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
  -- a shaft at rest reads as a denormal (~1e-323); keep logs and maths clean
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
  -- drag the crank torque has to overcome (spec 5: friction + engine brake
  -- + dynamic friction at idle speed)
  m.dragEst = (ice.friction or 15) * (ice.wearFrictionCoef or 1) + (ice.engineBrakeTorque or 0)
              + (ice.dynamicFriction or 0) * m.idle * RPM_TO_AV
  local k0raw = (k0 and type(k0.torqueDiff) == "number") and k0.torqueDiff or 0
  m.k0raw = k0raw
  if dt > 0 then
    st.k0T = st.k0T + (k0raw - st.k0T) * min(1, dt / P.K0T_TAU_S)
    -- slip power -> heat [J], only while the sequencer actually slips the
    -- clutch. In BLEND / ON / RELEASE:TORQUE K0 is commanded locked (r = 1,
    -- capacity 1150 Nm above anything the driveline carries); the "slip" read
    -- there is the elastic wind-up of the frictionClutch spring (tens to
    -- hundreds of rpm of ringing at +-200 Nm), energy that is stored and
    -- returned, not dissipated. Integrating |T * slip| of that ringing filled
    -- the 60 kJ budget in cruise and in P/N charging (QA 2026-09-30: 130 kJ
    -- while parked) and silently disabled every K0 start in favour of the
    -- 12 V starter.
    local s = st.state
    local locked = s == "ON" or s == "BLEND" or (s == "RELEASE" and st.relPhase == "TORQUE")
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
  -- stall detector shared by ENGAGE/BLEND/ON/SYNC
  if m.rpmIce < P.STALL_FRAC * m.idle then st.stallT = st.stallT + dt else st.stallT = 0 end
end

-- ===========================================================
-- Statistics / summary lines (spec 5.11)
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
  -- iceRestartMs is TRANSIENT (fire -> coupled): QA T11 polls for a new value
  -- > 0 and the HUD labels the ICE "START" while it is > 0. The persistent
  -- figure for diagnostics / new QA is etk854hybrid_iceFireMs.
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
  st.crankT, st.crankFrames, st.crankTimeout, st.pauseT = 0, 0, 1.5, 0
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
  -- Kill the stock start flare: after a starter start CE adds
  -- idleAVStartOffset (CE:621, idleRPMStartCoef default 2 = up to 2x idle,
  -- decaying at 1/s) to EVERY idle-governor target - including our
  -- rev-match reference. The starter is off here, so resetting the
  -- smoother keeps it at 0 (CE updateGFX feeds it idleAV*coef*0).
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

-- ===========================================================
-- OFF -> start request: variant choice (spec 5.1)
-- ===========================================================
-- Returns allowed, reason (the reason lands in the CRANK entry log line, so
-- every STARTER start says why the E-machine could not do it).
local function k0StartAllowed(ctx)
  local m = st.m
  if P.K0_START_POLICY == "URGENT_ONLY" and not st.startUrgent then return false, "policy" end
  if (ctx.gearSign or 0) == 0 then return false, "N/P" end
  if ctx.isShifting then
    -- urgent (kickdown) starts crank through any shift: the kickdown shift
    -- may not be flagged as a downshift yet in the frame the request lands
    -- (B58S AWD kickdown_ev60 2026-09-30: "starter-start:shifting")
    local gb = st.gb
    local down = gb and gb.isShiftingDown == true
    if not (P.K0_START_IN_DOWNSHIFT > 0 and (down or st.startUrgent)) then return false, "shifting" end
  end
  if ctx.evLockoutActive then return false, "evLockout" end
  if m.rpmSh < max(P.K0_START_MIN_FRAC * m.idle, m.idle + P.K0_START_MIN_ADD) then return false, "shaftSlow" end
  if st.eBudget >= P.SLIP_BUDGET_J then return false, string.format("k0Heat%.0fkJ", st.eBudget / 1000) end
  -- Headroom against the torque the E-machine DELIVERS (the coordinator's
  -- jerk-limited ramp, last frame, before the shift factor), not against the
  -- demand: from the first CRANK frame on updateReserve() holds crankNm +
  -- CRANK_HEADROOM_NM back from the driver (torqueCoord motorCapEff), so a
  -- demand that keeps rising - kickdown, tip-in - can no longer take the
  -- crank torque away and abort the crank; it only pauses the torque build-up
  -- for the ~0.2 s of the crank. The demand-based test sent every rolling
  -- demand start to the 12 V starter, because a demand start means
  -- demand > motor cap by definition (sweep 2026-09-30: 0 K0 starts in 40
  -- rolling starts, "headroom0<119Nm").
  local cap = ctx.motorCapNm or 0
  local used = max(0, ctx.motorBaseRawNm or ctx.motorBaseNm or 0)
  st.crankNm = crankNmFor(pv("crankAlpha"))
  if (cap - used) < st.crankNm + P.K0_START_RESERVE_NM then
    return false, string.format("headroom%.0f<%.0fNm", cap - used, st.crankNm + P.K0_START_RESERVE_NM)
  end
  return true, "k0"
end

-- Public: could a K0 start be begun THIS frame? Used by the master policy
-- (tip-in anticipation) and by the arbiter (skip the on-debounce while the
-- motor still has the crank torque free - the torque ramp closes that
-- window within ~0.1-0.2 s). Same test as the variant choice, without the
-- policy / shifting gates.
local function k0WindowOpen(ctx)
  local m = st.m
  if (ctx.gearSign or 0) == 0 or ctx.evLockoutActive or ctx.isShifting then return false end
  if st.state ~= "OFF" or not st.ice or st.ice.isDisabled then return false end
  if st.eBudget >= P.SLIP_BUDGET_J then return false end
  local rpmSh = ctx.rpmMotor or m.rpmSh
  if rpmSh < max(P.K0_START_MIN_FRAC * m.idle, m.idle + P.K0_START_MIN_ADD) then return false end
  local used = max(0, ctx.motorBaseRawNm or ctx.motorBaseNm or 0)
  return (ctx.motorCapNm or 0) - used >= crankNmFor(pv("crankAlpha")) + P.K0_START_RESERVE_NM
end
M.k0WindowOpen = k0WindowOpen

-- CE deactivateStarter (combustionEngine.lua:1021-1029) restores
-- starterThrottleKillCoef to 1 ONLY if the kill timer already ran out, and
-- setIgnition(0) (CE:1047-1051) zeroes the timer but not the coef. A starter
-- crank withdrawn inside starterThrottleKillTime (0.5 s, x2 petrol / x4
-- diesel cold, CE:1001-1002) therefore leaves the coef stuck at 0 and CE:737
-- multiplies every throttle by it: the engine can no longer fire. Only a new
-- activateStarter (CE:997-1003) or a reset (CE:1351) clears it - our K0 and
-- REFIRE paths never call activateStarter, so re-arm it on every start.
local function rearmThrottleKill()
  local ice = st.ice
  if not ice or (ice.starterEngagedCoef or 0) > 0 then return end
  if (ice.starterThrottleKillCoef or 1) < 1 then
    ice.starterThrottleKillCoef = 1
    ice.starterThrottleKillTimer = 0
  end
end

-- "Fired" needs more than rpm: with K0 dragging the ICE toward a shaft
-- >= 1.3*idle, rpm alone is reached without any combustion. Require that CE
-- can actually produce torque (CE:737 throttle = ... * starterThrottleKillCoef
-- * ignitionCoef; CE:754 * outputTorqueState). combustionTorque itself is not
-- usable: above idle the governor drops idleThrottle to ~0, so a fired but
-- dragged engine legitimately shows combustionTorque ~ -friction (CE:802).
local function canCombust()
  local ice = st.ice
  if not ice or ice.isDisabled then return false end
  return (ice.ignitionCoef or 0) >= 1 and (ice.starterThrottleKillCoef or 1) >= 1
    and (ice.outputTorqueState or 1) > 0
end

local function openStart()
  rearmThrottleKill()
  st.variant = "NONE"   -- the CRANK/SYNC entry log must not show the last start's variant
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
    -- A non-urgent request that lands in an 8HP UPshift waits for the end of
    -- the shift (<= K0_SHIFT_WAIT_S) instead of falling back to the 12 V
    -- starter: 60 km/h in 7th, 0.05 pedal, forced restart in the middle of a
    -- 7->8 upshift -> "starter-start:shifting" (QA T19c N57 2026-09-30).
    -- Only while rolling fast enough for a K0 start; urgent starts and
    -- downshifts (kickdown) go ahead at once.
    local m = st.m
    local gb = st.gb
    if ctx.isShifting and not st.startUrgent and not (gb and gb.isShiftingDown == true)
       and (ctx.gearSign or 0) ~= 0 and st.shiftWaitT < P.K0_SHIFT_WAIT_S
       and m.rpmSh >= max(P.K0_START_MIN_FRAC * m.idle, m.idle + P.K0_START_MIN_ADD) then
      st.shiftWaitT = st.shiftWaitT + dt
      return
    end
    st.shiftWaitT = 0
    st.orphanT = 0
    startRequest(ctx)
  else
    st.shiftWaitT = 0
    -- Orphan ignition (P.ORPHAN_IGN_S): OFF has no exit for an engine whose
    -- ignition was switched on behind the sequencer (only beginOff /
    -- beginFault / the EMERGENCY release cut it) - it would run decoupled
    -- for good. Not while our starter is engaged.
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

-- CRANK / K0: e-machine slip start (spec 5.2). Model feed-forward
-- (k0RatioFor) + measured-torque integral; the motor fill loop cancels the
-- crank torque at the shaft, the master's torqueCoord reserves it.
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
  -- K0 must be OPEN before the rev-match (user requirement beats a drag assist)
  if st.fired and st.tcap <= 0 and st.r <= 0.01 then
    st.r = 0
    beginSync(ctx, "k0-fired")
  end
end

-- CRANK / STARTER (spec 5.3): K0 open, nothing reaches the driveline.
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
  -- fired: CE disengages the starter itself above 1.1*starterMaxAV (CE:445)
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
      st.crankT, st.crankFrames, st.crankTimeout = 0, 0, 1.5
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
    -- activateStarter kills the throttle for starterThrottleKillTime (x2 petrol
    -- / x4 diesel when cold, CE:997-1003): allow that plus one second
    st.crankTimeout = clamp(1.0 + (ice.starterThrottleKillTimer or 0.5), 1.5, 3.0)
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

-- Rev-match actuator shared by SYNC and the governor phase of ENGAGE
-- (spec 5.4): rate-limited speed reference -> stock idle governor
-- (idleAVOverwrite, 100 Hz P-controller CE:706-721, precedent
-- hydraulicsCombustionEngineControl.lua:35-39) + GFX feed-forward
-- throttleForTorque(Iice * dRef/dt) + integral, applied through the VCU
-- throttle channel (CE:737 takes the MAX of governor and VCU throttle).
-- Under the brake-throttle guard (guardOn) no positive lead and no offset.
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
  -- follow the target's own rate (shaft acceleration) + a rate-limited,
  -- softly landing approach; the target is never overshot within a frame
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
  -- integral only for the static error once the reference has landed; while
  -- it moves, the feed-forward carries the acceleration (no wind-up)
  if abs(err) < P.SYNC_I_WINDOW_RPM and abs(refErr) < pv("syncTol") then
    st.sI = clamp(st.sI + P.SYNC_KI * err * dt, P.SYNC_I_MIN, thrMax)
  end
  st.override = clamp(ff + st.sI, 0, thrMax)
  st.rpmT = rpmT
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
  -- Bounded SYNC: an accelerating shaft may never meet the settle test
  -- (tolerance + slip rate + hold), so after syncMax the K0 closes anyway
  -- with a capacity-controlled slip while the governor keeps tracking; the
  -- fill cancels the K0 torque. Not during a ratio change (target jumps),
  -- never while blocked above the rev ceiling (the 8HP upshifts then).
  -- The budget is syncMax (convergence) PLUS the time the reference needs
  -- to rev the engine to the target at the profile's syncRefRate: a starter
  -- start into a kickdown gear (1100 -> ~5500 rpm, 0.65 s at 7000 rpm/s)
  -- used to run into the 0.40 s URGENT timeout while the engine was still
  -- accelerating on the ramp, i.e. before the rev-match could even settle.
  if not st.syncBudget then
    st.syncBudget = pv("syncMax") + min(P.SYNC_RAMP_MAX_S, abs(rpmT - m.rpmIce) / max(1, pv("syncRefRate")))
  end
  local sMax = st.syncBudget
  if st.syncT > sMax and not blocked and not ctx.isShifting then
    -- SLIP engage after the budget is the planned strategy for a target
    -- that keeps moving: no fault, not even when it comes after the slow
    -- phase below. Only the hard fallback (FORCED) is a sequencer fault.
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

-- ENGAGE (spec 5.5): K0 closes onto a matched shaft; the fill cancels the
-- K0 torque at the shaft, the governor keeps the ICE on the reference until
-- the clutch carries it.
local function stEngage(ctx, dt)
  local m = st.m
  if not ctx.iceReq then
    clearIdleOverwrite()
    beginRelease(ctx, ctx.releaseProfile or "FAST", "OPEN", "withdrawn")
    return
  end
  if st.stallT >= P.STALL_S then beginRelease(ctx, "EMERGENCY", "OPEN", "stall"); return end
  -- IDLE engage hands the creep torque to the ICE governor (no fill), except
  -- under the brake-throttle guard once the shaft has left the idle band:
  -- there the fill keeps the K0 torque out of the capped converter input
  -- (guardFill)
  st.fill = (st.engageMode == "IDLE" and not guardFill(ctx)) and 0 or 1
  -- IDLE engage closes onto a shaft BELOW idle by design (P/N/creep), its
  -- large slip is the point - the hold-and-rematch rule is for NORMAL only
  local slipBad = st.engageMode == "NORMAL" and abs(m.slip) > P.ENGAGE_ABORT_SLIP_RPM
                  and st.r < P.ENGAGE_ABORT_R
  -- SLIP / FORCED close on a shaft that is still moving away: the governor
  -- keeps the ICE on the reference until the slip is small, so K0 does not
  -- have to drag the engine (no torque taken from the driveline)
  local slipGov = (st.engageMode == "SLIP" or st.engageMode == "FORCED")
                  and abs(m.slip) > P.SLIP_GOV_RPM and not st.lockConfirmed
  if st.r < P.GOV_HANDOVER_R or slipBad or slipGov then
    syncTrack(ctx, dt)
  else
    if st.govArmed then clearIdleOverwrite(); st.govArmed = false end
    st.override = nil      -- coordinator holds the ICE at zero net torque
  end

  if ctx.isShifting and st.r < P.ENGAGE_SHIFT_HOLD_R and not st.lockConfirmed then
    -- hold r: the shaft speed jumps during the ratio change
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
  if st.r >= 0.98 and (st.lockConfirmed or st.t > P.ENGAGE_MAX_S) then
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

-- BLEND (spec 5.6): ICE share in with smoothstep(g); the motor gives the
-- same torque back through torqueCoord, the fill covers ICE lag.
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
  -- traction control active on the driven axle: slower ICE share ramp
  local mtf = st.motor and tonumber(st.motor.throttleFactor) or 1
  local itf = tonumber(st.ice.throttleFactor) or 1
  if min(mtf, itf) < P.BLEND_TC_TF then
    tb = max(tb, P.T_BLEND_TC)
    st.stats.blendTc = true
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

-- RELEASE (spec 5.8): TORQUE (share out) -> OPEN (K0 opens, ICE at zero net
-- torque or in fuel cut) -> ignition off ONLY after K0 is open (fixes S4).
local function stRelease(ctx, dt)
  local m = st.m
  local prof = st.relProfile
  local overrun = (ctx.torqueTargetNm or 0) <= 1 or (ctx.throttleRaw or 0) < 0.02
  if ctx.iceReq and prof ~= "EMERGENCY" then
    -- still coupled: just blend back in (not a new start)
    if st.relPhase == "TORQUE" then beginBlend(ctx, invSmoothstep(st.g), "re-request"); return end
    -- K0 partly open, ICE running: counts as a (re)start for the statistics
    openStart()
    st.variant = "REFIRE"
    recordFire(ctx)
    if st.r >= 0.5 and abs(m.slip) <= 100 then beginEngage(ctx, "NORMAL", "re-request"); return end
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
  -- OPEN. The fill stays on in overrun as well: the drag credit to the
  -- recuperation only applies while LOCKED, so here the motor cancels the
  -- K0 drag instead (consistent handover, no coast-decel bump).
  st.g, st.fill = 0, 1
  local rate = (prof == "NORMAL") and pv("k0OpenRate") or REL[prof].k0OpenRate
  st.r = max(0, st.r - rate * dt)
  if st.r <= P.K0_OPEN_EPS then
    st.r = 0
    local k0Nm = st.k0T
    local torqueMs = ((st.stop.openT0 or st.now) - (st.stop.t0 or st.now)) * 1000
    local openMs = (st.now - (st.stop.openT0 or st.now)) * 1000
    if ctx.iceIdleHold and prof ~= "EMERGENCY" and not st.ice.isDisabled then
      -- ICE-off hold: decoupled, the engine keeps idling (ignition stays on)
      st.beginIdle(ctx, "released-hold")
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

-- IDLE (ICE-off hold, see the header): enter from RELEASE once K0 is open
local function beginIdle(ctx, why)
  enter("IDLE", why)
  st.g, st.override, st.fill = 0, 0, 0
  clearIdleOverwrite()
  electrics.values.etk854hybrid_iceRestartMs = 0
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
    -- the engine is running: rev-match straight away, no crank
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

st.beginIdle = beginIdle   -- stRelease (defined above) enters IDLE through st

local STATES = {
  OFF = stOff, CRANK = stCrank, SYNC = stSync, ENGAGE = stEngage,
  BLEND = stBlend, ON = stOn, RELEASE = stRelease, IDLE = stIdle, FAULT = stFault,
}

-- Legacy semantics for bisecting regressions (ctx.iceSeqPassthrough):
-- request -> ignition + starter if rpm < 200, K0 ramps at 4/s, state jumps
-- OFF <-> ON, share gate 1, no fill. Exactly what the old master did.
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
-- Arbiter (spec 4.2): policy -> debounced request ctx.iceReq
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
    -- escalate a start already in progress (kickdown during the sequence)
    if urgent and (st.state == "CRANK" or st.state == "SYNC" or st.state == "ENGAGE" or st.state == "BLEND") then
      st.startUrgent = true
    end
    return
  end
  if st.state ~= "OFF" and st.state ~= "FAULT" then
    -- engine still turning (release in progress or blocked): take it back
    ctx.iceReq = true
    st.arbOnT = 0
    return
  end
  st.arbOnT = st.arbOnT + dt
  local mk = (ctx.edriveMode == "BATTERY_CONTROL") and "BATTERY_CONTROL" or dk
  local deb = ARB.ON_DEBOUNCE_S[mk]
  -- short debounce while a K0 start is possible, and for a fast pedal
  -- (master policy ctx.iceStartFast: the driver wants the power now, but
  -- without the URGENT start profile)
  if ctx.iceK0Window or ctx.iceStartFast then deb = min(deb, ARB.K0_WINDOW_DEBOUNCE_S[mk]) end
  if urgent or bypass or (st.arbOnT >= deb and st.offElapsed >= ARB.MIN_OFF_S[dk]) then
    ctx.iceReq = true
    st.arbOnT = 0
    st.startUrgent = urgent
  end
end

-- ICE-off hold for this frame (see ARB): hold time, minimum run time, whether
-- the hold may be spent as a decoupled IDLE, and the hold kind for telemetry.
local function arbHoldTime(ctx, dk)
  local S = ctx.sportIndex or 0
  local slow = (ctx.speedKmh or 0) < ARB.LIFT_KMH[dk]
  local braking = (ctx.brakeRaw or 0) > ARB.BRAKE_THR
  -- a partial lift onto the E-machine (master policy ctx.iceLightLoad: pedal
  -- inside the EV zone, load below demandOff) counts as a lift too, otherwise
  -- easing from 0.52 to 0.25 at 70 km/h kept a fresh start coupled for
  -- HOLD_MIN_S / MIN_ON_S (review 2026-10)
  local lift = slow and (braking or (ctx.throttleRaw or 0) < ARB.LIFT_THR
                         or ctx.iceLightLoad == true)
  st.arbSlow, st.arbLift, st.arbBraking = slow, lift, braking      -- diag (arbFlags)
  local base = lift and 0 or ARB.HOLD_MIN_S[dk]
  local hMid, hMax = ARB.H_MID_S[dk], ARB.HOLD_MAX_S[dk]
  local holdS = base + (hMid - base) * clamp01((S - ARB.H_MID_S0) / ARB.H_MID_SPAN)
                + (hMax - hMid) * smoothstep((S - ARB.H_MAX_S0) / ARB.H_MAX_SPAN)
  local minOnS = ARB.MIN_ON_S[dk]
  -- "sporty" = a high index AND (outside SPORT) a sporty WINDOW: one
  -- kickdown lifts S to ~0.9 for 20+ s but W only by ~0.07 (sportIndex
  -- W_GROW_S 30 s while e > 0.5); a sustained sporty section (QA T20a: three
  -- 3 s full-throttle bursts) reaches W 0.15-0.3 (CR-VCU1, round 2).
  -- Never in a Driving Experience without decoupled idle (ARB.IDLE_DE: ECO PRO).
  local idleDe = (ARB.IDLE_DE[dk] or 1) > 0
  local idleOk = idleDe and S >= (lift and ARB.IDLE_S_MIN_LIFT or ARB.IDLE_S_MIN)
    and (dk == "SPORT" or (ctx.sportWindow or 0) >= ARB.IDLE_W_MIN)
  local kind = lift and "lift" or "hold"
  if lift and not idleOk then
    holdS = min(holdS, ARB.DECOUPLE_S[dk])
    minOnS = ARB.MIN_ON_LIFT_S
  elseif slow and not idleDe then
    -- no decoupled idle in this DE (ECO PRO) and no lift below LIFT_KMH (the
    -- policy let the engine go with the pedal just above the EV zone): the
    -- coupled hold is bounded like a lift, not stretched by S to HOLD_MAX_S
    -- (8 s in fuel cut / light load); minimum run time as above LIFT_KMH
    holdS = min(holdS, ARB.DECOUPLE_S[dk])
    minOnS = min(minOnS, ARB.HOLD_MIN_S[dk])
  elseif not slow and not idleOk then
    -- above LIFT_KMH outside sporty phases the hold stays coupled (no IDLE):
    -- bound the fuel-cut time of a fresh start to the hold, not MIN_ON_S,
    -- and the coupled hold itself to HOLD_MIN_S. CR-VCU1 (round 2): after
    -- a COMFORT kickdown at ~100 km/h the S-scaled hold (up to 20 s) kept
    -- the engine COUPLED for ~12 s in fuel cut / light load; calm = off
    -- after HOLD_MIN_S, sporty = decoupled idle (idleOk branch)
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
  -- nothing to hold: the engine is off or on its way off
  if not ctx.iceReq and not ctx.iceIdleHold then st.holdRemain, st.abortT = 0, 0; return end
  -- a start in progress completes (no abort loops); the hold counts from BLEND/ON.
  -- Optional exception (ARB.SYNC_ABORT_S, default 0 = off): an unwanted start
  -- still in SYNC (fired, K0 open - never in CRANK) on a real lift / brake is
  -- withdrawn after SYNC_ABORT_S; stSync takes its "withdrawn" path to OFF, a
  -- re-request while the engine still spins is a REFIRE without a crank.
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
  -- The minimum run time is ENGINE time (ON and IDLE count, stIdle), not
  -- coupling time: a start just before a lift used to keep the K0 closed for
  -- the rest of the 3 s in fuel cut - in 2nd gear at 3900 rpm that is 1 m/s^2
  -- of engine braking on top of the selected recuperation level (QA T17 LOW
  -- 2026-09-30: 1.01 against a 0.50 target). After sporty driving it is
  -- idled out decoupled; on a calm lift below LIFT_KMH it is MIN_ON_LIFT_S.
  local minOnLeft = max(0, minOnS - st.onElapsed)
  st.minOnLeft = minOnLeft
  if st.holdRemain <= 0 and minOnLeft <= 0 then
    -- hold over: release (still coupled) or ignition off (IDLE)
    ctx.iceReq, ctx.iceIdleHold = false, false
    ctx.releaseProfile = "NORMAL"
  elseif not idleOk then
    -- no decoupled idle outside sporty phases or in a DE without it (ECO
    -- PRO, ARB.IDLE_DE): a running IDLE ends (-> OFF; also one inherited
    -- from COMFORT on a DE switch), a coupled hold stays coupled until it
    -- runs out
    if not ctx.iceReq then ctx.iceIdleHold = false end
  elseif ctx.iceReq and st.arbHoldT >= ARB.DECOUPLE_S[dk]
      and max(st.holdRemain, minOnLeft) >= ARB.IDLE_MIN_S then
    -- hold = decoupled idle: K0 opens, the engine idles for the rest of the
    -- hold (or of its minimum run time), a new request re-engages it without
    -- a crank. Less than IDLE_MIN_S left: stays coupled and is released.
    ctx.iceReq, ctx.iceIdleHold = false, true
    ctx.releaseProfile = "NORMAL"
  end
  -- the cause of every decoupled idle in beamng.log (the user plays without
  -- a telemetry recording); COMFORT / SPORT only since IDLE_DE
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
  if hard and not ctx.iceForced then prof = "FAST" end   -- standstill, MAX EV
  if ctx.qaIceOverride ~= nil then
    want = (ctx.qaIceOverride == true)
    bypass = true
    hard = not want
  end
  -- lockout and key-off are hard offs even when the policy forces the ICE:
  -- they must win over the NORMAL profile chosen above, otherwise stOn's
  -- RELEASE_BLOCK keeps the ICE coupled while the torque demand stays high
  if st.lockoutT > 0 then want, hard, prof = false, true, "FAST" end
  -- key off / accessory (electrics ignitionLevel < 2): the driver switched
  -- the car off - never (re)start the engine behind the key
  if (electrics.values.ignitionLevel or 2) < 2 then want, hard, prof = false, true, "FAST" end
  local dk = deKey(ctx)
  -- diagnostics of this frame: the effective want after every override (the
  -- decoupled classifier's "unwanted"), the hold-rule inputs are only
  -- evaluated by arbHoldTime (-1 / false otherwise)
  st.arbWant = want
  st.arbIdleOk, st.minOnLeft, st.arbSlow, st.arbLift, st.arbBraking = -1, 0, false, false, false
  if want then arbWantOn(ctx, dt, urgent, bypass, dk) else arbWantOff(ctx, dt, hard, prof, dk) end
  local ev = electrics.values
  ev.etk854hybrid_iceOffHoldS = st.holdRemain
  -- which hold rule is running: none | hold | lift | brake (arbHoldTime)
  ev.etk854hybrid_iceHoldKind = st.holdKind
  -- arbiter diagnostics (ECO no-idle analysis, feedback round 3)
  ev.etk854hybrid_arbIdleOk    = st.arbIdleOk          -- 1/0, -1 = not evaluated this frame
  ev.etk854hybrid_arbIdleHold  = ctx.iceIdleHold and 1 or 0
  ev.etk854hybrid_arbHoldT     = st.arbHoldT
  ev.etk854hybrid_arbMinOnLeft = st.minOnLeft
  -- bits: 1 want, 2 hard, 4 urgent, 8 bypass, 16 forced, 32 lightLoad,
  -- 64 slow (< LIFT_KMH), 128 lift, 256 braking, 512 IDLE allowed in this DE
  ev.etk854hybrid_arbFlags = (want and 1 or 0) + (hard and 2 or 0) + (urgent and 4 or 0) + (bypass and 8 or 0)
    + (ctx.iceForced and 16 or 0) + (ctx.iceLightLoad and 32 or 0) + (st.arbSlow and 64 or 0)
    + (st.arbLift and 128 or 0) + (st.arbBraking and 256 or 0) + (((ARB.IDLE_DE[dk] or 1) > 0) and 512 or 0)
  -- The COMMITTED request (after the on-debounce). The 8HP
  -- (etk854hybrid_8hpCore) then plans the gear with the ICE torque that is
  -- about to come instead of downshifting for the E-machine alone
  -- (ice_start_comfort 2026-09-30: 4->2, engaged at 5300 rpm). Not the raw
  -- policy want: at the motor corner speed the demand start flickers with
  -- every gear change (cruise_100: iceUp -> want gone -> evSat downshift ->
  -- want again, 1->2->3->2->3).
  ev.etk854hybrid_iceRequested = ctx.iceReq and 1 or 0
end

-- ===========================================================
-- Decoupled-ICE diagnostics (feedback round 3). Stateless per-frame class
-- (the state machine itself is not touched), segments (class changes) and
-- episodes (NONE -> ... -> NONE) with rate-limited [ICESEQ] log lines:
--   decoupled#N CLASS dur de v S W idleOk hold policy req unwanted next
--                                       (segment >= DIAG.LOG_MIN_S[CLASS])
--   decoupled-episode#N dur de chain=A>B>C   (episode >= EP_LOG_MIN_S)
--   decoupled-summary de=DE Ns CLASS=sum s/n ...  (per DE window, at most
--                                       every SUMMARY_S, only when non-empty)
-- Thresholds: SYNC ECO median 0.65 s / max 0.80 s -> 1.0; K0 crank budget
-- crankMaxS 0.6-1.0 -> 1.2; starter fire ~350 ms, retry timeout 1.5-3 s ->
-- 2.0; RELEASE OPEN 1/3 s -> 0.6; spin-down ECO p90 0.65 s, COMFORT <= 2.0 s
-- -> 2.0; ENGAGE <= ENGAGE_MAX_S 1.2 -> 1.0; IDLE engage at 1.2/s -> 1.5.
-- An ordinary stop or start therefore writes nothing.
-- ===========================================================
local DIAG = {
  RPM_FRAC = 0.30,          -- "turning": rpmIce >= 0.30 x idle (~210 rpm); below that the spin-down is silent
  LOG_MIN_S = { IDLE_HOLD = 0, IDLE_MINON = 0, OFF_IGN_ON = 0, SYNC_BLOCKED = 0, FAULT = 0,
                SYNC_UNWANTED = 0.3, CRANK_UNWANTED = 0.3, RELEASE_OPEN = 0.6, SPINDOWN = 2.0,
                CRANK_K0 = 1.2, CRANK_STARTER = 2.0, SYNC = 1.0, SYNC_IDLE = 1.0, ENGAGE = 1.0, ENGAGE_IDLE = 1.5 },
  EP_LOG_MIN_S = 1.5, SUMMARY_S = 120, CHAIN_MAX = 8,
}

-- NONE | IDLE_HOLD | IDLE_MINON | RELEASE_OPEN | SPINDOWN | OFF_IGN_ON |
-- CRANK_K0 | CRANK_STARTER | CRANK_UNWANTED | SYNC | SYNC_IDLE | SYNC_BLOCKED |
-- SYNC_UNWANTED | ENGAGE | ENGAGE_IDLE | FAULT ("unwanted" = the arbiter's
-- effective want is off while the start completes)
local function decClass()
  local s, m, ice = st.state, st.m, st.ice
  if not ice or m.rpmIce < DIAG.RPM_FRAC * m.idle then return "NONE" end
  if s == "ON" or s == "BLEND" or (s == "RELEASE" and st.relPhase == "TORQUE") then return "NONE" end
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
  if s ~= "OFF" then return "NONE" end                     -- passthrough / unknown state
  if (ice.ignitionCoef or 0) > 0 and not st.starterOn then return "OFF_IGN_ON" end
  return "SPINDOWN"
end

-- "DE:CLASS=sum s/n" pairs of the summary window, sorted (DE = the DE in
-- which the segment began; an IDLE inherited from COMFORT stays COMFORT:)
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
  if dk ~= d.lastDk or d.sumT >= DIAG.SUMMARY_S then           -- per-DE window summary
    if d.winAny then
      slog("[ICESEQ] decoupled-summary de=%s %.0fs %s", d.lastDk, d.sumT, fmtWin(d.win, d.winN))
    end
    for k in pairs(d.win) do d.win[k] = nil end
    for k in pairs(d.winN) do d.winN[k] = nil end
    d.winAny, d.sumT, d.lastDk = false, 0, dk
  end
end

-- Lifetime totals since the vehicle (Lua VM) was loaded, per "DE:CLASS":
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
    -- + CRANK_HEADROOM_NM: stCrankK0 aborts when cap - motorBase - that
    -- margin drops below CRANK_MIN_NM; reserving only crankNm left a 70 Nm
    -- crank (the CRANK_MIN floor) 10 Nm short under a saturating demand
    want = st.crankNm + P.CRANK_HEADROOM_NM
  end
  if want >= st.reserve then
    st.reserve = want
  else
    st.reserve = max(want, st.reserve - P.RESERVE_RELEASE_NMS * dt)
  end
end

-- Starter guard (was C:1468-1478). The AI/arcade gearbox cranks any
-- non-running engine on throttle/brake unless engine.starterDisabled
-- (vehicleController.lua:296-320, the ONLY reader). The sequencer owns every
-- start, so the flag is held in EVERY state except while our own starter is
-- engaged - including CRANK/K0, where an arcade auto-crank would put the
-- starter on the engine while K0 drags it. Never on a damaged engine
-- (disable()/lockUp() own the flag there, CE:264-292).
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
      -- shaft torque without the EV creep floor (etk854hybrid_motorShaft
      -- creepFloor, last tick): creep is no part of the commanded torque, a
      -- start while creeping must not count it as a coordination error
      local shaftNm = motor.etkShaftNm - (motor.etkCreepNm or 0)
      local dev = abs(shaftNm - tcmd)
      if dev > st.stats.maxDev then st.stats.maxDev = dev end
      -- against what was physically available (motor cap + measured ICE/K0
      -- torque, which is negative while K0 drags the engine): the QA T21 bound
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
  ctx.iceCoupled = (s == "BLEND" or s == "ON" or (s == "RELEASE" and st.relPhase == "TORQUE"))
  ctx.iceShareGate = st.g
  ctx.iceThrottleOverride = st.override
  ctx.seqFillGain = st.fill
  ctx.seqFillInstant = (s == "CRANK" or s == "SYNC" or (s == "ENGAGE" and st.r < P.KISS_R))
  ctx.k0TorqueNm = st.k0T
  ctx.k0SlipRpm = m.slip
  ctx.iceMeasNm = st.iceMeas
  ctx.iceSyncBlocked = (s == "SYNC") and st.syncBlocked or false
  -- (read by torqueCoord updateFillGain: fill 0 for an IDLE engage; not under
  -- the brake-throttle guard above the idle band, see guardFill)
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
  -- motor fill loop measurement select (etk854hybrid_motorShaft): 1 = K0
  -- carries the engine -> measure combustion torque, 0 = measure K0 torque
  ev.etk854hybrid_fillLocked = (ctx.iceCoupled or (s == "ENGAGE" and st.lockConfirmed)) and 1 or 0
  -- decoupled-ICE diagnostics (updateDecDiag). The converter pump / motor
  -- shaft speed is etk854hybrid_pumpRpm (motor device, one channel only).
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

-- External stall (master's N-charge recovery): EMERGENCY open -> OFF; the
-- restart then comes through the arbiter like any other start.
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
  -- decoupled diagnostics: the open segment / episode is dropped, the
  -- lifetime totals (st.dec.tot / cnt) survive a reset
  st.dec.why, st.dec.segT0, st.dec.epT0 = "NONE", st.now, st.now
  st.seqFault, st.variant = "none", "NONE"
  st.inStart = false
  st.idleEngage, st.syncBlocked, st.govArmed = false, false, false
  -- Spawn / reset state: follow the engine. CE reset spawns it running at
  -- idle when the vehicle ignition level says so (CE:1302-1314). The engine
  -- DEVICE decides, not electrics.engineRunning: the 8HP shift logic reports
  -- that as hybrid READY (etk854hybrid_8hp.lua H12), i.e. 1 in EV as well.
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

function M.getStats(ctx)
  return (ctx and ctx.iceSeqLast) or nil
end

-- Live tuning (console / dev server): M.setParam('CRANK_DRAG_FRAC', 0.6).
-- Only existing numeric keys of P, returns the old value; not persistent.
function M.setParam(key, value)
  if type(P[key]) ~= "number" or type(value) ~= "number" then return nil end
  local old = P[key]
  P[key] = value
  return old
end

-- Live tuning of a start profile value: M.setProf('URGENT', 'syncRefRate', 9000)
function M.setProf(prof, key, value)
  local t = PROF[prof]
  if not t or type(t[key]) ~= "number" or type(value) ~= "number" then return nil end
  local old = t[key]
  t[key] = value
  return old
end

-- Live tuning of the arbiter (A/B levers, not persistent; returns the old
-- value, nil for an unknown / non-numeric key):
--   M.setArb('IDLE_DE', 'ECO_PRO', 1)        per-DE table entry
--   M.setArb('SYNC_ABORT_S', 'ECO_PRO', 0.4)
--   M.setArb('IDLE_W_MIN', 0.2)              scalar
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
