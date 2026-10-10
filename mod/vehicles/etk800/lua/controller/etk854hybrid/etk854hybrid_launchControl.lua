-- etk854hybrid_launchControl.lua - ETK 854 ActiveHybrid - launch control
--
-- User requirement: "Die Launch Control muss das Ausgabedrehmoment so regeln,
-- dass das Auto nicht ueber die Bremse losschiebt. Erst beim Loslassen der
-- Bremse losfahren. Der E-Motor muss nicht vorgespannt werden, nur der
-- Verbrenner wegen Ladedruck. Dann volle Leistung voraus." Clarification:
-- "Der Wandler soll Druck aufbauen - das Auto soll sich hinten senken und
-- Kraft gegen die Bremse arbeiten."
--
-- Why the old launch pushed through the brake: while armed the normal SPORT
-- pedal map passed the full 680 Nm system cap into the torque converter at
-- stall. Pump torque at stall is c * w^2 (torqueConverter.lua:35-36 with
-- avRatio = 0: kFactor = 0.004 * S / (1 + S)), the output is that times the
-- stall ratio (:38) times gear 1 x axle (13.25 on the R6 cars): ~7000-9000 Nm
-- on a rear axle whose brakes hold ~2700-3000 Nm (wheels.lua:556). The rear
-- wheels turned through their own brakes (QA T14 2026-09-30: 1-8 km/h).
--
-- What this module does instead (PRELOAD / READY):
--   * feed-forward pump torque cap  T_ff = MARGIN x hold capacity C_hold /
--     (stall ratio x gearbox cumulativeGearRatio), where C_hold is what the
--     WHOLE car holds at the driven axle (holdCapacity):
--       C_hold = C_R + r_R x min(mu N_R, mu N_F, C_F / r_F)
--     The rear brakes take the axle torque first (brake and propulsion both
--     act between hub and body, wheels.lua:648-652); the rest turns the rear
--     tyres against the road, the body is pushed forward and held by the
--     front brakes through the front tyres. That tyre pre-tension between the
--     axles is what squats the rear ("Wandler soll Druck aufbauen, hinten
--     senken, Kraft gegen die Bremse"; etk854hybrid_selfLeveling.lua:392-399
--     measured the same wind-up after hard stops: rear 10-15 mm low).
--   * shaft speed target n* = sqrt(T* / c) - the converter itself turns a
--     held shaft speed into exactly that pump torque, so regulating SPEED is
--     exact physics (T* = T_ff x closed-loop anti-creep trim)
--   * K0 stays closed, the ICE runs loaded for boost (turbocharger.lua:178
--     exhaust power ~ (0.1 + 0.8 * load) * throttle^2), the E-machine never
--     drives (evThrottle 0). Generator mode (tuning $etk854hybrid_lcGenLoad):
--       0 = pure ICE: ICE torque = pump torque at the reference (inverse
--           model), speed feedback in Nm, E-machine 0
--       1 = GEN: the ICE runs at the torque the E-machine can absorb as a
--           generator on top of the cap (full throttle while it cannot reach
--           it), the generator holds the shaft speed
--       2 = GEN on petrol engines only. Default 1 since the round-2 in-game
--           check: the offline estimate (N57 pure ICE ~23 psi) did not hold,
--           pure ICE regulated the N57 to ~170 Nm / 13 % throttle at the
--           ~1420 rpm hold and it released at -1.8 psi; with the generator
--           the ICE runs loaded (B58 6.9 psi, 387 Nm at 1413 rpm)
--   * hold regulator (fix of the round-2 open issue, regulate()): it limit-
--     cycled on N57 / N57S / B58S (launch_hold_full 20260930-202657 / -202722
--     / -202609: generator 0 <-> 250 Nm and ICE throttle 0 <-> 1 every 2-3
--     frames, shaft 950-1900 rpm, trim down to 0.45). Cause: once the
--     generator saturated, a throttle PI on the shaft speed took over -
--     ICE_KP 0.002 /rpm through the throttle map slope (~1200-1800 Nm per
--     unit throttle at 10-30 %, combustionEngine.lua:751) is ~2.5-3.5 Nm/rpm
--     at the 30 fps graphics rate, past the discrete stability limit of the
--     ~0.3-0.4 kg m^2 shaft (the pure-ICE hold 20260930-202033 already
--     alternated with period 2 frames). Now (offline lumped model and
--     robustness sweep in scratch_build/launch_fix/):
--       ICE side = torque feed-forward, no speed feedback: the ICE torque is
--         driven to T* + GEN_ICE_SHARE x Gmax by a slow throttle integrator on
--         the measured ICE torque, hard-capped every frame at T* + Gmax by
--         the exact inverse model (torqueCoord throttleForTorque)
--       generator = G = T_ice(feed-forward) - c w_ref^2 + GEN_KP e + I with
--         GEN_KP 0.25 Nm/rpm (was 0.8) and the shaft speed taken as the mean
--         over the physics steps of the frame (wheelsIntermediate) plus a
--         first-order low-pass - the 30 fps sample aliased the K0 ringing
--       pure ICE = the same feedback in the torque domain, linearised by the
--         inverse model (gain independent of the throttle-map slope)
--   * DSC hold assist at physics rate: desiredBrakingTorque = full brakeTorque
--     on every wheel (same pattern as brakeControl.lua:49-56), so the hold
--     capacity does not depend on how hard the driver presses
--   * closed-loop anti-creep trim on turbine / wheel / body speed
-- FIRE (brake < 0.15 with the pedal floored): generator and assist off in the
-- same frame, the torque coordinator is re-seeded with the ICE torque that is
-- already on the shaft (ctx.launchSeedNm) and runs its LAUNCH row for 3 s.
-- GUARD (brake >= 0.35 + pedal > 0.3 at standstill, every mode, also while
-- armed and waiting for the ICE): the same capacity model on the PEDAL brake
-- torque as a plain torque cap on the coordinator request
-- (ctx.launchGuardNm) - production brake override. It engages at once while the
-- brake is held or pressed; only while the brake is being RELEASED (it fell
-- faster than GUARD_FALL_RATE within the last GUARD_FALL_HOLD_S) the overlap
-- must last GUARD_DWELL_S - that keeps the gamepad trigger cross-over at a
-- normal braked start out of it. Round-2 bug: at exactly half brake
-- (launch_hold_half, brake 1 -> 0.5 with the pedal floored in one frame) the
-- 0.2 s dwell (started only once the brake-rate filter had settled) ran
-- past the moment the half brake let go at ~0.2 s; above 1 km/h the guard
-- waits, so it came ~0.85 s late and the car jumped ~1 m (20260930-203037 /
-- -203243: 10 / 5.7 km/h peak). Offline replay of those inputs
-- (scratch_build/launch_fix/guard_replay.py): new logic on at +0.13 s.
--
-- Physics limit (documented for the user): with K0 closed the ICE speed IS
-- the stall speed; the capacity above puts it at ~1450-1550 rpm with the
-- DSC hold assist (R6 steel / carbon, i4 ~1400). xAllrad at a standstill: the
-- stock electronicSplitShaftLock keeps the lock clutch at 0 while braking
-- below avThreshold 1 rad/s (electronicSplitShaftLock.lua:64-79, minimumLock
-- 0), so the front axle is not driven and holds the car through the road
-- exactly like on the RWD cars. Only a closed lock (clutchRatio >= 0.9)
-- drives both axles against their own brakes (no tyre path, AWD branch).
--
-- Effective anti-creep trim range: n* is clamped to idle + N_MIN_OVER_IDLE,
-- so a trim below T(idle + 150) / T_ff cannot lower the pump torque further
-- (~0.32 on the N57 at 700 rpm idle, ~0.38 at 800 rpm). When the clamp binds
-- the hold cuts the ICE torque (reason 'trim-floor') instead of letting the
-- car creep up to the 1.5 km/h overspeed cut.
--
-- Public contract (unchanged for the Actions app and the 8HP):
--   etk854hybrid_launchState IDLE | ARMED | READY | ACTIVE
--   (PRELOAD is published as ARMED; 8hpCore READY holds gear 1)
-- ctx outputs: launchArmed / launchReady / launchActive (iceSequencer:1447
-- forces the ICE on + urgent), launchHold (torqueCoord hold branch),
-- launchIceThr, launchGenNm, launchPumpCapNm, launchTargetRpm,
-- launchGuardNm (torqueCoord request cap), launchSeedNm (one-shot at FIRE).
--
-- Contract: M.isReal, init / initStage2 / reset, arm(ctx, src),
-- updateGFX(ctx, dt), applyOutputs(ctx, dt), wheelsIntermediate(dt),
-- setParam(key, value), getState(). All constants in LC, all state in st
-- (LuaJIT: 60 upvalues per function).

local M = {}
M.name = "launchControl"
M.isReal = true

local abs, max, min, sqrt = math.abs, math.max, math.min, math.sqrt
local RPM_TO_AV = 0.104719755
local AV_TO_RPM = 9.549296596

-- ===========================================================
-- Parameters
-- ===========================================================
local LC = {
  -- pedal thresholds (raw electrics throttle / brake)
  BRAKE_ENTER = 0.45, BRAKE_HOLD = 0.30, BRAKE_FIRE = 0.15,   -- ENTER was 0.50 (exactly
                                     -- the half brake of launch_hold_half)
  PEDAL_ENTER = 0.90, PEDAL_HOLD = 0.80, PEDAL_HOLD_S = 0.20, PEDAL_FIRE = 0.90,
  PEDAL_REARM = 0.20,                -- after an abort / a launch: lift below this first
  V_ENTER_KMH = 0.5,                 -- arm / PRELOAD only at a standstill
  V_OVER_KMH = 1.5,                  -- hold failed: ICE torque 0 for the rest of the hold
  V_DISARM_KMH = 5.0,                -- armed and driven off without a launch
  -- Production-style auto-arm (the gamepad path, no Alt+K there): DE SPORT, DTC or
  -- DSC OFF, D/S/M, standstill, brake + kickdown held AUTO_S
  AUTO_ARM = true, AUTO_PEDAL = 0.95, AUTO_BRAKE = 0.45, AUTO_S = 0.30,   -- BRAKE was 0.50
  ARM_TIMEOUT_S = 60,
  HOLD_MAX_S = 8.0,                  -- UX limit (BeamNG has no converter thermal
                                     -- model, torqueConverter.lua:21-45)
  READY_TOL_RPM = 100, READY_SETTLE_S = 0.5, READY_MAX_S = 1.5,
  WINDOW_S = 3.0,                    -- LAUNCH window (was LAUNCH_WINDOW_S in the master)
  ACTIVE_ABORT_BRAKE = 0.2, ACTIVE_ABORT_PEDAL = 0.2,
  COOLANT_MAX_C = 112,
  -- capacity model
  MARGIN = 0.75,                     -- share of the hold capacity (rear brakes +
                                     -- tyre path) that may reach the axle at stall
  GROUND_PATH = true,                -- count the rear tyre -> body -> front brake path
  MU_HOLD = 0.80,                    -- static tyre-road friction for that path
                                     -- (conservative; rear-tyre spin is caught by
                                     -- the wheel-speed trim, CREEP_WHEEL_MS)
  AXLE_N_MIN = 1500,                 -- [N] less normal force on an axle: no tyre path
  LOAD_TAU_S = 0.5,                  -- LPF of wheels downForce (wheels.lua:310);
                                     -- latched during a hold / guard (the squat
                                     -- itself moves load rearward)
  LOAD_FALLBACK = 0.80,              -- no downForce yet: 0.8 x m g / 2 per axle
  MASS_FALLBACK_KG = 1900,           -- node-weight sum unavailable
  BRAKE_ASSIST = true,               -- DSC hold assist in PRELOAD / READY
  AWD_REAR_SHARE = 0.60,             -- only with the xAllrad lock clutch closed
  AWD_LOCK_R = 0.90,                 -- transfercase.clutchRatio above which it counts
  RATIO_MIN = 0.5, RATIO_MAX = 40, RATIO_FALLBACK = 15,   -- conservative (larger = lower cap)
  C_FALLBACK = 0.0075726,            -- c for D 0.30 / S 12 (torqueConverter.lua:35/181)
  TR_FALLBACK = 1.7,
  N_MIN_OVER_IDLE = 150,             -- no preload below idle + this ("Bremse fester")
  N_MAX_DIESEL = 2600, N_MAX_PETROL = 3500,               -- upper clamps only
  N_RAMP_UP = 1500, N_RAMP_DN = 3000,  -- [rpm/s] shaft speed reference ramp
  -- generator load (E-machine absorbs the ICE torque above the cap). Values
  -- from the offline lumped model + robustness sweep in
  -- scratch_build/launch_fix/ (sim_hold.py, sweep2.py, report.py): 5 engines
  -- x 20/30/60 fps (+-25 % frame jitter) x 40 rpm K0 ringing x turbo lag
  -- 0.1-0.6 s x ICE model error +-3 % x entry 1350/1550 rpm, -1/12 psi:
  -- no limit cycle, pump torque peak <= 1.19 x T*, READY rpm std <= 16 (old
  -- regulator in the same grid: 1.70 x T*, std 231 rpm, 104 G sign flips);
  -- Lua-in-the-loop (lua_harness.py, this file under LuaJIT): entry <= +44 rpm,
  -- re-floor after a lift <= +107 rpm (both < the 1.33 x T* static margin)
  GEN_LOAD = 1,                      -- 0 off / 1 on / 2 petrol only (header)
  -- 250 -> 290 (2026-10-10): the N57 hold ran G = 231 Nm against the 250 cap
  -- and reached only 9.5 psi at the release (T14a floor 10); the motor's
  -- regen curve allows ~300 Nm at the ~1400 rpm hold speed, the stage regen
  -- power (stageRegenKw / w) still bounds it per boost stage
  GEN_MAX_NM = 290, GEN_SOC_MAX = 0.92, GEN_MIN_NM = 30,
  GEN_KP = 0.25,                     -- [Nm/rpm] (was 0.8; 20 fps / i4 0.27 kg m^2:
                                     -- KP x 9.55 x dt / J = 0.44)
  GEN_KI = 1.0,                      -- [Nm/(rpm*s)] (was 4.0)
  GEN_I_MAX = 150,                   -- [Nm] (was 250) - the feed-forward does the work
  GEN_ICE_PRED = 2.0,                -- feed-forward: + this x the RISE of the ICE torque
                                     -- over the last frame (boost build-up: frame mean,
                                     -- LPF and the held command lag ~2 frames)
  RPM_TAU_S = 0.01,                  -- LPF of the frame-mean shaft speed (alpha = dt/(tau+dt));
                                     -- the frame mean is the anti-aliasing filter, a longer
                                     -- tau only adds lag (0.03: +40 % entry overshoot)
  -- ICE torque feed-forward in GEN mode: the ICE makes what the generator can
  -- absorb on top of the cap (boost), never more; replaces the round-2
  -- saturation switch (GEN_SAT_S / _SHARE / UNSAT) and its throttle PI
  GEN_ICE_SHARE = 0.85,              -- ICE torque target T* + this x Gmax (15 % headroom)
  GEN_ICE_HARD = 1.00,               -- hard cap T* + this x Gmax, exact inverse, every frame
  ICE_TQ_KI_UP = 0.006,              -- [1/(Nm*s)] throttle integrator, ICE below the target
  ICE_TQ_KI_DN = 0.003,              -- [1/(Nm*s)] ... above it (the hard cap catches spikes)
  ICE_BOOST_RISE = 0.3,              -- [1/s] min. throttle rise while the target is out of
                                     -- reach even at full throttle on the current boost
                                     -- (i4: full throttle in ~2 s; a jump to 1 pulses at
                                     -- the reachability boundary - offline sweep)
  ICE_THR_RISE = 4.0,                -- [1/s] max. rise of the hold throttle (both modes)
  GEN_TRIM_E_RPM = 25,               -- generator at its limit + shaft this far too fast:
  GEN_TRIM_DN = 300, GEN_TRIM_UP = 100,   -- [Nm/s] ICE target trimmed (model error)
  -- pure-ICE regulation (GEN off / pack full): ICE torque = c w_ref^2 - KP e - I,
  -- throttle by the exact inverse - the loop gain is KP whatever the slope of
  -- the throttle map (round 2: 0.002 /rpm on the throttle = ~2.5 Nm/rpm at
  -- 13 % throttle, period-2 ringing in 20260930-202033)
  ICE_KP = 0.30,                     -- [Nm/rpm] (was 0.002 1/rpm)
  ICE_KI = 1.5,                      -- [Nm/(rpm*s)] (was 0.005 1/(rpm*s))
  ICE_I_MAX = 80,                    -- [Nm] (was 0.40 throttle)
  ICE_TAU_S = 0.03,                  -- LPF of the frame-mean ice.combustionTorque (CE:802)
                                     -- (was 0.05 on the single graphics-frame sample)
  -- anti-creep trim
  CREEP_TURBINE_RPM = 40,            -- = ~0.1 m/s at the wheel in gear 1
  CREEP_WHEEL_MS = 0.083, CREEP_BODY_MS = 0.083,
  HARD_TURBINE_RPM = 150, HARD_WHEEL_MS = 0.30,
  CREEP_TAU_S = 0.05, BODY_TAU_S = 0.10,
  SETTLE_TURBINE_RPM = 20, SETTLE_MS = 0.05, SETTLE_MAX_S = 0.5,
  TRIM_SOFT_RATE = 1.5,              -- [1/s]
  TRIM_HARD_MUL = 0.70, TRIM_HARD_EVERY_S = 0.25, HARD_CUT_S = 0.20,
  TRIM_QUIET_S = 0.5, TRIM_RECOVER = 0.10, TRIM_MIN = 0.30,
  -- generator unload after a hold ends WITHOUT a fire (the ICE is at full
  -- throttle; dropping the generator at once would put ~400 Nm on the shaft)
  UNLOAD_MAX_S = 0.8, UNLOAD_DONE_NM = 15,
  -- ... but not when the brake is released with at least this much pedal:
  -- the driver asks for more than the ICE makes (SPORT map at 0.5: ~0.6 x
  -- ceiling), the unload would only cut a ~150 ms torque hole (offline plant)
  UNLOAD_SKIP_PEDAL = 0.50,
  -- brake-throttle guard outside the hold
  GUARD_ENABLE = true, GUARD_MARGIN = 0.75,
  GUARD_BRAKE = 0.35, GUARD_PEDAL = 0.30, GUARD_V_KMH = 1.0,   -- BRAKE was 0.50
  GUARD_DWELL_S = 0.20,              -- overlap must persist this long, but ONLY while
  GUARD_FALL_RATE = 0.30,            -- ... the brake is being released: it fell faster
  GUARD_FALL_HOLD_S = 0.12,          -- ... than this [1/s] within the last this [s]
                                     -- (the scenario runner holds each cross-over step
                                     -- 0.05-0.13 s; a gamepad release is continuous)
  GUARD_BRAKE_FALL = 3.0,            -- no overlap while the brake falls faster [1/s]
  BRAKE_RATE_TAU_S = 0.05,
  GUARD_BRAKE_OUT = 0.30, GUARD_PEDAL_OUT = 0.20, GUARD_V_OUT_KMH = 3.0,
  -- self-leveling hold (published etk854hybrid_launchLevelHold)
  LEVEL_HOLD_S = 2.0,
  SPRING_TAU_S = 0.5,                -- reference length LPF (throttle released)
  SOC_MIN_BOOST = 0.10,
}

local PUBLIC = { OFF = "IDLE", ARMED = "ARMED", PRELOAD = "ARMED", READY = "READY",
                 FIRE = "ACTIVE", ABORT = "IDLE" }

local st = {
  phase = "OFF", why = "none", reason = "none", armSrc = "none", armDe = nil,
  now = 0, armT = 0, autoT = 0, holdT = 0, preT = 0, readyT = 0, pedalLowT = 0, activeT = 0,
  rearmBlock = false, weakMsg = false,
  mode = "-",                        -- "GEN" | "ICE" | "-"
  trim = 1, quietT = 0, hardT = 0, cutT = 0, cut = false, hardN = 0,
  settled = false, settleT = 0,
  turbF = 0, whlF = 0, bodyF = 0, creep = 0,
  capNm = 0, tFf = 0, tStar = 0, nStar = 0, nRef = 0, c = LC.C_FALLBACK, tr = LC.TR_FALLBACK, ratio = 0,
  gI = 0, iI = 0, G = 0, gSat = false,
  iceThr = 0, iceLpf = 0, regenFrac = 0,
  nReg = 0,                          -- regulator shaft speed [rpm] (frame mean + LPF)
  thrI = -1,                         -- GEN: ICE throttle integrator (< 0 = seed next frame)
  iceTrim = 0, iceCapNm = 0,         -- GEN: ICE torque target trim / target [Nm]
  -- physics-rate accumulators (wheelsIntermediate, only while accOn)
  accOn = false, accAV = 0, accTq = 0, accN = 0,
  unloadT = 0, unloadCap = 0, assistOn = false,
  guard = false, guardNm = 0, guardDwellT = 0, brakePrev = nil, brakeRate = 0, brakeFallT = 9,
  capR = 0, capGround = 0, floorBind = false,
  nR = 0, nF = 0, rR = 0.34, rF = 0.34, massKg = 0,
  levelHoldT = 0,
  springs = nil, springRef = nil, springNow = 0,
  stat = { turbMax = 0, whlMax = 0, bodyMax = 0, trimMin = 1, genSum = 0, genN = 0 },
  -- devices / wheels
  ice = nil, motor = nil, tc = nil, gb = nil, tcase = nil,
  rear = {}, front = {}, all = {}, wheelCount = 0,
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

local function readNumber(data, key, default)
  if type(data) ~= "table" then return default end
  local v = data[key]
  if type(v) == "number" then return v end
  if type(v) == "string" then return tonumber(v) or default end
  if type(v) == "boolean" then return v and 1 or 0 end
  return default
end

local function msg(text, sec)
  if guihooks and guihooks.message then guihooks.message(text, sec or 3, "vehicle.launch") end
end

local function lcLog(fmt, ...)
  log("I", "etk854hybrid.launch", string.format(fmt, ...))
end

-- forward gear (D / S* / M*) - 0.39 reports "S1"/"M2" (beamng-troubleshoot)
local function gearFwd(ctx)
  if (ctx.gearSign or 0) <= 0 then return false end
  local g = electrics.values.gear
  if type(g) ~= "string" then return true end
  local c1 = g:sub(1, 1)
  return c1 == "D" or c1 == "S" or c1 == "M"
end

local function gearManualOk(ctx)
  local g = electrics.values.gear
  if type(g) ~= "string" then return false end
  local c1 = g:sub(1, 1)
  if c1 == "S" or c1 == "M" then return true end
  return c1 == "D" and ctx.deMode == "SPORT"
end

local function iceReady(ctx)
  return ctx.iceEngageState == "ON" and ctx.iceCoupled == true
end

local function curves(ctx)
  return ctx.modules and ctx.modules.curves
end

-- ===========================================================
-- Devices / wheels / springs
-- ===========================================================
local function findDevices()
  if not (powertrain and powertrain.getDevice) then return end
  st.ice = powertrain.getDevice("mainEngine")
  st.motor = powertrain.getDevice("integratedMotor")
  st.tc = powertrain.getDevice("torqueConverter")
  st.gb = powertrain.getDevice("gearbox")
  st.tcase = powertrain.getDevice("transfercase")
end

local function findWheels()
  st.rear, st.front, st.all, st.wheelCount = {}, {}, {}, 0
  if not (wheels and wheels.wheels) then return end
  local n = wheels.wheelCount or 0
  st.wheelCount = n
  for i = 0, n - 1 do
    local wd = wheels.wheels[i]
    if wd then
      st.all[#st.all + 1] = wd
      local nm = type(wd.name) == "string" and wd.name or ""
      local c1 = nm:sub(1, 1)
      if c1 == "R" then st.rear[#st.rear + 1] = wd elseif c1 == "F" then st.front[#st.front + 1] = wd end
    end
  end
end

-- spring_RR / spring_RL (etk854hybrid_chassis.jbeam, the self-leveling
-- actuators) - read-only here, only for the squat channel
local function findSprings()
  st.springs = nil
  if not (v and v.data and v.data.beams and obj and obj.getBeamLength) then return end
  local want = { spring_RR = true, spring_RL = true }
  local cids = {}
  for _, b in pairs(v.data.beams) do
    if b.name and want[b.name] and b.cid then cids[#cids + 1] = b.cid end
  end
  if #cids == 2 then st.springs = cids end
end

local function springLen()
  local s = st.springs
  if not s then return 0 end
  return 0.5 * (obj:getBeamLength(s[1]) + obj:getBeamLength(s[2]))
end

-- vehicle mass [kg] = sum of the jbeam node weights (the load fallback only)
local function findMass()
  st.massKg = 0
  if not (v and v.data and v.data.nodes) then return end
  local m = 0
  for _, n in pairs(v.data.nodes) do
    if type(n) == "table" then m = m + (tonumber(n.nodeWeight) or 0) end
  end
  if m > 300 and m < 10000 then st.massKg = m end
end

-- ===========================================================
-- Capacity model
-- ===========================================================
-- Holding torque of one friction brake [Nm] exactly as wheels.lua applies it:
--   desired = brakeTorque * (min(b, split) + max(b - split, 0) * splitCoef)
--             b = electrics[brakeElectricsName] * defaultBrakeInputUsageCoef
--             (wheels.lua:556, :581, :595)
--   desired = max(desired, parkingTorque * parkingbrake * usage)   (:596)
--   applied = pressureDelay(desired) * brakeThermalEfficiency     (:642)
-- brakeTorque is already scaled by the part condition (:158). With the hold
-- assist the desired torque is the full brakeTorque (wheelsIntermediate).
local function wheelHoldNm(wd, assist)
  local bt = wd.brakeTorque or 0
  local eff = wd.brakeThermalEfficiency or 1
  if assist then return bt * eff end
  local usage = wd.defaultBrakeInputUsageCoef or 1
  local b = (electrics.values[wd.brakeElectricsName or "brake"] or 0) * usage
  local split = wd.brakeInputSplit or 1
  local d = bt * (min(b, split) + max(b - split, 0) * (wd.brakeSplitCoef or 1))
  local pk = (wd.parkingTorque or 0) * ((input and input.parkingbrake) or 0) * usage
  return max(d, pk) * eff
end

local function axleHoldNm(list, assist)
  local s = 0
  for i = 1, #list do
    local wd = list[i]
    if not wd.isBroken then s = s + wheelHoldNm(wd, assist) end
  end
  return s
end

local function axleRadius(list)
  local s, n = 0, 0
  for i = 1, #list do
    local r = tonumber(list[i].dynamicRadius) or tonumber(list[i].radius)
    if r and r > 0.1 and r < 1 then s, n = s + r, n + 1 end
  end
  return (n > 0) and s / n or 0.34
end

local function axleDownForce(list)
  local s = 0
  for i = 1, #list do s = s + max(0, tonumber(list[i].downForce) or 0) end
  return s
end

-- Normal force per axle [N] (wheels downForce, wheels.lua:309-310), low-pass
-- filtered while free, latched during a hold / guard. Falls back to a
-- conservative static split when the wheels report nothing plausible.
local function updateLoads(dt, latch)
  local fb = LC.LOAD_FALLBACK * 0.5 * (st.massKg > 0 and st.massKg or LC.MASS_FALLBACK_KG) * 9.81
  local nR, nF = axleDownForce(st.rear), axleDownForce(st.front)
  local tot = nR + nF
  local mg = 2 * fb / LC.LOAD_FALLBACK
  if tot < 0.3 * mg or tot > 3 * mg then nR, nF = fb, fb end
  if st.nR <= 0 or st.nF <= 0 then
    st.nR, st.nF = nR, nF
  elseif not latch then
    local a = min(1, dt / LC.LOAD_TAU_S)
    st.nR = st.nR + (nR - st.nR) * a
    st.nF = st.nF + (nF - st.nF) * a
  end
  st.rR, st.rF = axleRadius(st.rear), axleRadius(st.front)
end

-- Hold capacity at the DRIVEN axle [Nm] (header). RWD, the xAllrad 2WD mode
-- (splitShaft "disconnected") and xAllrad at a standstill (lock clutch open)
-- put the whole converter output on the rear axle: its brakes take it first,
-- the rest goes through the rear tyres into the body, which the front brakes
-- hold through the front tyres - limited by rear grip, front grip and the
-- front brakes. Sets st.capR / st.capGround for the channels.
local function holdCapacity(assist)
  local rear = st.rear
  st.capR, st.capGround = 0, 0
  if #rear == 0 then return 0.5 * axleHoldNm(st.all, assist) end
  local cR = axleHoldNm(rear, assist)
  st.capR = cR
  if #st.front == 0 then return cR end
  local cF = axleHoldNm(st.front, assist)
  local tcase = st.tcase
  if tcase and tcase.mode ~= "disconnected" and (tonumber(tcase.clutchRatio) or 0) >= LC.AWD_LOCK_R then
    -- both axles driven: each brake holds its own share, no tyre path
    return min(cR / LC.AWD_REAR_SHARE, cF / (1 - LC.AWD_REAR_SHARE))
  end
  if LC.GROUND_PATH and st.nR >= LC.AXLE_N_MIN and st.nF >= LC.AXLE_N_MIN then
    local f = min(LC.MU_HOLD * st.nR, LC.MU_HOLD * st.nF, cF / max(st.rF, 0.1))
    st.capGround = st.rR * max(0, f)
  end
  return cR + st.capGround
end

-- Pump torque constant c [Nm/(rad/s)^2] and stall torque ratio, read from
-- the device: at stall (avRatio 0) kFactor = 0.004 * S / (1 + S)
-- (torqueConverter.lua:35), pump torque = kFactor * kFactorCoef * w^2 (:36),
-- output = pump * stallTorqueRatio (:38), kFactorCoef = 844 * D^5 (:181).
local function converterConst()
  local tc = st.tc
  if tc and type(tc.converterStiffness) == "number" and type(tc.kFactorCoef) == "number" then
    local S = tc.converterStiffness
    st.c = 0.004 * S / (1 + S) * tc.kFactorCoef * (tc.impellerClutchCoef or 1)
    st.tr = tonumber(tc.stallTorqueRatio) or LC.TR_FALLBACK
  else
    st.c, st.tr = LC.C_FALLBACK, LC.TR_FALLBACK
  end
  if st.c <= 1e-6 then st.c = LC.C_FALLBACK end
end

-- gearbox cumulativeGearRatio = gear x final drive (x transfer case): the
-- path from the converter turbine to the wheels (13.25 in gear 1 on the R6
-- cars, 14.85 on the i4 with its 3.15 axle)
local function mechRatio()
  local gb = st.gb
  local r = gb and tonumber(gb.cumulativeGearRatio)
  if r then
    r = abs(r)
    if r >= LC.RATIO_MIN and r <= LC.RATIO_MAX then return r end
  end
  return LC.RATIO_FALLBACK
end

local function nMaxRpm()
  local ice = st.ice
  if ice and ice.requiredEnergyType == "diesel" then return LC.N_MAX_DIESEL end
  return LC.N_MAX_PETROL
end

-- Feed-forward pump torque cap and shaft speed target.
--   T_ff = margin * C_hold / (TR * i_mech),  T* = T_ff * trim,  n* = sqrt(T*/c)
-- Returns the untrimmed target speed (for the "Bremse fester" check).
local function computeTargets(ctx, assist, margin)
  converterConst()
  st.ratio = mechRatio()
  st.capNm = holdCapacity(assist)
  st.tFf = margin * st.capNm / max(st.tr * st.ratio, 1)
  st.tStar = st.tFf * st.trim
  local idle = ctx.iceIdleRPM or 800
  local nFf = sqrt(max(st.tFf, 0) / st.c) * AV_TO_RPM
  local nTrim = sqrt(max(st.tStar, 0) / st.c) * AV_TO_RPM
  st.floorBind = nTrim < idle + LC.N_MIN_OVER_IDLE
  st.nStar = clamp(nTrim, idle + LC.N_MIN_OVER_IDLE, nMaxRpm())
  return nFf, idle
end

local function genMax(ctx, n)
  local w = max(n * RPM_TO_AV, 1)
  local cv = curves(ctx)
  local env = (cv and cv.motorRegenTorqueAtRpm and cv.motorRegenTorqueAtRpm(n)) or LC.GEN_MAX_NM
  local g = min(LC.GEN_MAX_NM, (ctx.stageRegenKw or 55) * 1000 / w, env)
  if ctx.evLockoutActive then g = 0 end
  return max(0, g * (ctx.regenAcceptFraction or 1)), env
end

-- GEN_LOAD 0 / 1 / 2 (2: petrol only, header)
local function genWanted()
  local g = LC.GEN_LOAD
  if g == 2 then
    local ice = st.ice
    return not (ice and ice.requiredEnergyType == "diesel")
  end
  return g == 1 or g == true
end

local function throttleFor(ctx, netNm, rpm)
  local tc = ctx.modules and ctx.modules.torqueCoord
  if tc and tc.throttleForTorque then return tc.throttleForTorque(netNm, rpm, ctx) end
  local cv = curves(ctx)
  local cap = (cv and cv.iceMaxTorqueAtRpm and cv.iceMaxTorqueAtRpm(rpm)) or 350
  return clamp01(netNm / max(50, cap))
end

-- ===========================================================
-- Anti-creep sensing + trim
-- ===========================================================
-- Returns 0 quiet / 1 soft creep / 2 hard creep. Turbine = torque converter
-- output (gearbox input), wheels = largest |wheelSpeed| of the DRIVEN wheels
-- (rear on RWD / xAllrad in 2WD, all four in AWD - 2026-10-10: on the N57 RWD
-- a braked FRONT wheel turned up to 1.5 km/h while the body pitched under the
-- converter preload, the trim fell to its floor and the ICE was cut), body =
-- forward component of the body velocity (a squat is mostly vertical).
local function senseCreep(ctx, dt)
  local tc = st.tc
  local turb = tc and abs(tonumber(tc.outputAV1) or 0) * AV_TO_RPM or 0
  local a = min(1, dt / LC.CREEP_TAU_S)
  st.turbF = st.turbF + (turb - st.turbF) * a
  local wmax = 0
  local all = (electrics.values.etk854hybrid_awdMode == "AWD" or #st.rear == 0) and st.all or st.rear
  for i = 1, #all do
    local s = abs(all[i].wheelSpeed or 0)
    if s > wmax then wmax = s end
  end
  st.whlF = st.whlF + (wmax - st.whlF) * a
  local body = 0
  if obj and obj.getVelocity and obj.getDirectionVector then
    local vel, dir = obj:getVelocity(), obj:getDirectionVector()
    body = abs(vel.x * dir.x + vel.y * dir.y + vel.z * dir.z)
  end
  st.bodyF = st.bodyF + (body - st.bodyF) * min(1, dt / LC.BODY_TAU_S)
  if not st.settled then
    st.settleT = st.settleT + dt
    if (st.turbF < LC.SETTLE_TURBINE_RPM and st.whlF < LC.SETTLE_MS and st.bodyF < LC.SETTLE_MS)
       or st.settleT >= LC.SETTLE_MAX_S then
      st.settled = true
    end
    return 0
  end
  if st.turbF > st.stat.turbMax then st.stat.turbMax = st.turbF end
  if st.whlF > st.stat.whlMax then st.stat.whlMax = st.whlF end
  if st.bodyF > st.stat.bodyMax then st.stat.bodyMax = st.bodyF end
  if st.turbF > LC.HARD_TURBINE_RPM or st.whlF > LC.HARD_WHEEL_MS then return 2 end
  if st.turbF > LC.CREEP_TURBINE_RPM or st.whlF > LC.CREEP_WHEEL_MS or st.bodyF > LC.CREEP_BODY_MS then
    return 1
  end
  return 0
end

-- Fast reduction, slow recovery. A hard creep also cuts the ICE for HARD_CUT_S.
local function updateTrim(dt, creep)
  st.hardT = max(0, st.hardT - dt)
  st.cutT = max(0, st.cutT - dt)
  if creep == 2 then
    st.quietT = 0
    if st.hardT <= 0 then
      st.trim = max(LC.TRIM_MIN, st.trim * LC.TRIM_HARD_MUL)
      st.hardT = LC.TRIM_HARD_EVERY_S
      st.cutT = LC.HARD_CUT_S
      st.hardN = st.hardN + 1
    end
  elseif creep == 1 then
    st.quietT = 0
    st.trim = max(LC.TRIM_MIN, st.trim - LC.TRIM_SOFT_RATE * dt)
  else
    st.quietT = st.quietT + dt
    if st.quietT >= LC.TRIM_QUIET_S then st.trim = min(1, st.trim + LC.TRIM_RECOVER * dt) end
  end
  if st.trim < st.stat.trimMin then st.stat.trimMin = st.trim end
  st.creep = creep
end

local function resetSensing()
  st.settled, st.settleT = false, 0
  st.quietT, st.hardT, st.cutT = 0, 0, 0
  st.creep = 0
end

-- ===========================================================
-- Hold regulator (PRELOAD / READY)
-- ===========================================================
-- Frame means of the physics-rate samples (M.wheelsIntermediate, every
-- physics step while st.accOn): shaft speed [rpm] and ICE combustion torque
-- [Nm] over the last graphics frame - a boxcar anti-aliasing filter against
-- the K0 spring ringing (~70 Hz, etk854hybrid_powertrain.jbeam lockDampRatio
-- note) that a single 30 fps sample folds into a slow fake speed error.
-- Falls back to the graphics-frame sample (entry frame of a hold, hook not
-- wired). Consumes the accumulators.
local function frameMeans(ctx)
  local n, comb
  if st.accN > 0 then
    n = st.accAV / st.accN * AV_TO_RPM
    comb = st.accTq / st.accN
  else
    n = abs(ctx.rpmMotor or 0)
    local ice = st.ice
    comb = (ice and tonumber(ice.combustionTorque)) or 0
  end
  st.accAV, st.accTq, st.accN = 0, 0, 0
  return n, comb
end

-- GEN mode, ICE side: torque feed-forward, NO speed feedback on the
-- throttle. Target capS = T* + GEN_ICE_SHARE x Gmax (+ trim): what the
-- generator absorbs on top of the cap with 15 % headroom. A slow integrator
-- on the measured ICE torque moves the throttle to where the boost that
-- throttle sustains gives exactly capS (turbocharger.lua:178: exhaust power
-- ~ throttle^2) - following the static inverse at the momentary boost
-- instead pulses between ~0.3 and 1 (offline model; consistent with the
-- 0.2 / 0.98 throttle spikes in the round-2 N57 READY data). While capS is
-- out of reach even at full throttle on the current boost (boost still
-- building, i4 30e 338 Nm < 358 Nm) the throttle rises at least
-- ICE_BOOST_RISE. Hard cap
-- every frame: never more than the generator can take (T* + Gmax), exact
-- inverse on the live boost - that catches the boost overshoot at the hold
-- entry that the slow integrator cannot.
local function genIceThrottle(ctx, dt, n, gmax)
  local capS = st.tStar + LC.GEN_ICE_SHARE * gmax + st.iceTrim
  local capH = st.tStar + LC.GEN_ICE_HARD * gmax + st.iceTrim
  st.iceCapNm = capS
  local reachS = throttleFor(ctx, capS, n)   -- 1 = out of reach at full throttle
  if st.thrI < 0 then st.thrI = min(reachS, st.iceThr) end   -- seed: entry throttle
  local prev = st.thrI
  local err = capS - st.iceLpf
  local t = prev + ((err > 0) and LC.ICE_TQ_KI_UP or LC.ICE_TQ_KI_DN) * err * dt
  if err > 0 and reachS >= 1 then t = max(t, prev + LC.ICE_BOOST_RISE * dt) end
  t = clamp01(t)
  local tH = throttleFor(ctx, capH, n)
  if t > tH then t = tH end
  return t
end

-- GEN mode, generator: G = T_ice(feed-forward) - c w_ref^2 + KP e + I. The
-- feed-forward is the frame-mean ICE torque plus GEN_ICE_PRED x its rise over
-- the last frame (boost build-up: the torque of the coming frames). The pump
-- torque c w^2 itself damps the shaft (2 c w ~ 2.3 Nm s/rad at 1400 rpm), so
-- the loop needs little gain: GEN_KP 0.25 Nm/rpm. Generator at its limit and
-- the shaft still too fast = the ICE makes more than the model says: trim the
-- ICE target down (and back up slowly once the generator has headroom).
local function genLoad(dt, e, loadRef, gmax, rise)
  local ff = st.iceLpf + LC.GEN_ICE_PRED * max(0, rise)
  local gp = (ff - loadRef) + LC.GEN_KP * e
  local graw = gp + st.gI
  -- anti-windup: no integration further into a limit
  if not ((graw <= 0 and e < 0) or (graw >= gmax and e > 0)) then
    st.gI = clamp(st.gI + LC.GEN_KI * e * dt, -LC.GEN_I_MAX, LC.GEN_I_MAX)
  end
  local G = clamp(gp + st.gI, 0, gmax)
  st.gSat = G >= gmax - 1
  if st.gSat and e > LC.GEN_TRIM_E_RPM then
    st.iceTrim = max(-(st.tStar + 0.5 * gmax), st.iceTrim - LC.GEN_TRIM_DN * dt)
  elseif st.iceTrim < 0 and (G < (LC.GEN_ICE_SHARE - 0.10) * gmax or e < -LC.GEN_TRIM_E_RPM) then
    st.iceTrim = min(0, st.iceTrim + LC.GEN_TRIM_UP * dt)
  end
  return G
end

-- Pure ICE (GEN off / pack full): ICE torque = pump torque at the reference
-- minus a speed correction in Nm, throttle by the exact inverse model - the
-- loop gain is ICE_KP [Nm/rpm] whatever the throttle-map slope.
local function iceOnly(ctx, dt, n, e, loadRef)
  local tq = loadRef - LC.ICE_KP * e - st.iI
  st.iceCapNm = tq
  local t = throttleFor(ctx, tq, n)
  if not ((t <= 0 and e > 0) or (t >= 1 and e < 0)) then
    st.iI = clamp(st.iI + LC.ICE_KI * e * dt, -LC.ICE_I_MAX, LC.ICE_I_MAX)
  end
  return t
end

local function regulate(ctx, dt)
  local nS, comb = frameMeans(ctx)
  if st.nReg <= 0 then st.nReg = nS else st.nReg = st.nReg + (nS - st.nReg) * dt / (LC.RPM_TAU_S + dt) end
  local n = st.nReg
  if st.nRef <= 0 then st.nRef = max(n, ctx.iceIdleRPM or 800) end
  if st.nRef < st.nStar then
    st.nRef = min(st.nStar, st.nRef + LC.N_RAMP_UP * dt)
  else
    st.nRef = max(st.nStar, st.nRef - LC.N_RAMP_DN * dt)
  end
  local e = n - st.nRef
  local wRef = st.nRef * RPM_TO_AV
  local loadRef = st.c * wRef * wRef            -- pump torque at the reference
  local prevLpf = st.iceLpf
  st.iceLpf = st.iceLpf + (comb - st.iceLpf) * dt / (LC.ICE_TAU_S + dt)
  local gmax, env = genMax(ctx, n)
  if st.mode == "GEN" and gmax < LC.GEN_MIN_NM then
    -- the pack stopped accepting charge: hand over to the pure-ICE loop
    st.mode, st.iI, st.gI, st.iceTrim = "ICE", 0, 0, 0
  end
  local thr
  if st.mode == "GEN" then
    thr = genIceThrottle(ctx, dt, n, gmax)
    st.G = genLoad(dt, e, loadRef, gmax, st.iceLpf - prevLpf)
    st.stat.genSum, st.stat.genN = st.stat.genSum + st.G, st.stat.genN + 1
  else
    st.G = 0
    thr = iceOnly(ctx, dt, n, e, loadRef)
  end
  if st.cut or st.cutT > 0 then
    thr = 0
  elseif thr > st.iceThr + LC.ICE_THR_RISE * dt then
    thr = st.iceThr + LC.ICE_THR_RISE * dt     -- also the ramp back after a cut
  end
  st.iceThr = thr
  if st.mode == "GEN" then st.thrI = thr end   -- the integrator state IS the output
  st.regenFrac = (env and env > 1) and clamp01(st.G / env) or 0
end

-- After a hold that ended WITHOUT a fire: the ICE throttle closes, the
-- generator keeps absorbing whatever the engine still makes above T*, so the
-- shaft never sees the full-throttle ICE torque against the brake.
local function unloadStep(ctx, dt)
  st.unloadT = max(0, st.unloadT - dt)
  local n, comb = frameMeans(ctx)
  st.iceLpf = st.iceLpf + (comb - st.iceLpf) * dt / (LC.ICE_TAU_S + dt)
  local gmax, env = genMax(ctx, n)
  st.iceThr = 0
  st.G = clamp(st.iceLpf - st.unloadCap, 0, gmax)
  st.regenFrac = (env and env > 1) and clamp01(st.G / env) or 0
  if (st.G < 2 and st.iceLpf < st.unloadCap + LC.UNLOAD_DONE_NM) or st.unloadT <= 0 then
    st.unloadT, st.G, st.regenFrac = 0, 0, 0
  end
end

-- ===========================================================
-- Phase changes
-- ===========================================================
local function setPhase(ctx, p, why)
  if st.phase == p then return end
  lcLog("[LC] %s -> %s why=%s v=%.2fkm/h n=%.0f n*=%.0f T*=%.0fNm cap=%.0fNm trim=%.2f G=%.0f boost=%.1fpsi",
    st.phase, p, tostring(why), ctx.speedKmh or 0, abs(ctx.rpmMotor or 0), st.nStar, st.tStar, st.capNm,
    st.trim, st.G, tonumber(electrics.values.turboBoost) or 0)
  st.phase, st.why = p, why or "none"
end

local function holdSummary(ctx, tag)
  local s = st.stat
  lcLog("[LC] hold %s: t=%.2fs mode=%s n*=%.0f nRef=%.0f n=%.0f T*=%.0f Tff=%.0f cap=%.0f i=%.2f trimMin=%.2f hard=%d "
    .. "turbMax=%.0frpm whlMax=%.3fm/s bodyMax=%.3fm/s genAvg=%.0fNm ice=%.0fNm iceCap=%.0fNm iceTrim=%.0fNm "
    .. "thr=%.2f boost=%.1fpsi",
    tag, st.holdT, st.mode, st.nStar, st.nRef, st.nReg, st.tStar, st.tFf, st.capNm, st.ratio,
    s.trimMin, st.hardN, s.turbMax, s.whlMax, s.bodyMax, (s.genN > 0) and s.genSum / s.genN or 0,
    st.iceLpf, st.iceCapNm, st.iceTrim, st.iceThr, tonumber(electrics.values.turboBoost) or 0)
end

local function clearHoldState()
  st.G, st.regenFrac, st.iceThr = 0, 0, 0
  st.gI, st.iI, st.gSat = 0, 0, false
  st.thrI, st.iceTrim, st.iceCapNm, st.nReg = -1, 0, 0, 0
  st.cut = false
end

local function resetArmState()
  st.armT, st.holdT, st.preT, st.readyT, st.pedalLowT = 0, 0, 0, 0, 0
  st.weakMsg = false
end

local function disarm(ctx, why, next)
  clearHoldState()
  resetArmState()
  st.trim = 1
  st.mode = "-"
  setPhase(ctx, next or "OFF", why)
end

-- Leave PRELOAD / READY without a fire. nextPhase: ARMED | OFF | ABORT.
-- skipUnload: the brake is already released and the pedal asks for more than
-- the ICE makes - hand the ICE torque on the shaft to the coordinator like a
-- fire does (bumpless seed) instead of unloading it.
local function exitHold(ctx, nextPhase, why, skipUnload)
  holdSummary(ctx, "exit:" .. tostring(why))
  local gen = st.mode == "GEN" and st.G > 1
  if gen and skipUnload then
    local md = st.motor
    local shaft = (md and type(md.etkShaftNm) == "number") and md.etkShaftNm or 0
    ctx.launchSeedNm = max(st.iceLpf, shaft, 0)
    gen = false
  end
  clearHoldState()
  st.unloadCap = st.tStar            -- the unload keeps the cap the hold ran at
  st.unloadT = gen and LC.UNLOAD_MAX_S or 0
  st.reason = why
  if nextPhase == "ARMED" then
    st.holdT, st.preT, st.readyT, st.pedalLowT = 0, 0, 0, 0
    setPhase(ctx, "ARMED", why)
  else
    if nextPhase == "ABORT" then st.rearmBlock = true end
    resetArmState()
    st.mode = "-"
    setPhase(ctx, nextPhase, why)
  end
end

local function enterPreload(ctx)
  -- A trim the brake-throttle guard wound down while the car waited ARMED
  -- (ICE start, guard on) was sensed against the PEDAL brake capacity and the
  -- start / stop transients, not against this hold (DSC assist, own settle
  -- window): the hold starts from trim 1 like a first entry. st.guard is
  -- still last frame's value here (updateArmed runs before updateGuard); a
  -- re-floor after a pedal lift keeps the hold's own trim (guard off below
  -- GUARD_PEDAL). QA T14a 20261010-115509: guard trim 0.30 seeded the hold
  -- -> T* 48 Nm under the idle + 150 floor, ICE cut, 0 psi at the release.
  if st.guard then
    st.trim = 1
    computeTargets(ctx, LC.BRAKE_ASSIST, LC.MARGIN)
  end
  resetSensing()
  clearHoldState()
  st.holdT, st.preT, st.readyT, st.pedalLowT = 0, 0, 0, 0
  st.nRef = max(abs(ctx.rpmMotor or 0), ctx.iceIdleRPM or 800)
  local ice = st.ice
  st.iceLpf = (ice and tonumber(ice.combustionTorque)) or 0
  -- the throttle the coordinator / guard drove the ICE with: the hold
  -- throttle starts there (ICE_THR_RISE), no step to full throttle and no dip
  st.iceThr = clamp01(tonumber(electrics.values.etk854hybrid_iceThrottle) or 0)
  st.accAV, st.accTq, st.accN = 0, 0, 0      -- entry frame: graphics-frame sample
  st.stat = { turbMax = 0, whlMax = 0, bodyMax = 0, trimMin = st.trim, genSum = 0, genN = 0 }
  st.hardN = 0
  local gmax = genMax(ctx, st.nStar)
  local want = genWanted()
  if want and (ctx.soc or 1) <= LC.GEN_SOC_MAX and gmax >= LC.GEN_MIN_NM then
    st.mode = "GEN"
  else
    st.mode = "ICE"
    if want then msg("LC: reduced turbo boost (battery full)", 3) end
  end
  st.guard, st.unloadT = false, 0
  st.reason = "none"
  setPhase(ctx, "PRELOAD", st.mode)
end

local function fire(ctx)
  holdSummary(ctx, "fire")
  -- The shaft carries the full ICE torque the instant the generator lets go:
  -- that is the bumpless seed for the coordinator's jerk-limited total and
  -- its ICE share (torqueCoord seedFire), not the ~T* the converter saw.
  local md = st.motor
  local shaft = (md and type(md.etkShaftNm) == "number") and md.etkShaftNm or 0
  ctx.launchSeedNm = max(st.iceLpf, shaft, 0)
  clearHoldState()
  st.unloadT, st.assistOn = 0, false
  st.activeT = LC.WINDOW_S
  st.rearmBlock = true
  st.reason = "none"
  resetArmState()
  setPhase(ctx, "FIRE", "brake-release")
  local soc = ctx.soc or 1
  if soc < LC.SOC_MIN_BOOST or ctx.evLockoutActive then
    msg("LAUNCH! (engine only)", 1.5)
  else
    msg("LAUNCH!", 1.5)
  end
end

-- ===========================================================
-- Arming
-- ===========================================================
local function armAllowed(ctx, src)
  if ctx.seqFailsafe then return false, "Launch Control not available (failsafe)" end
  if ctx.evOnly then return false, "Launch Control: not available in MAX EV" end
  if (tonumber(electrics.values.watertemp) or 0) >= LC.COOLANT_MAX_C then
    return false, "Launch Control: engine too hot"
  end
  if not gearFwd(ctx) then return false, "Launch Control: select D, S or M" end
  if src == "manual" and not gearManualOk(ctx) then
    return false, "Launch Control: select SPORT or shift to S/M"
  end
  if (ctx.speedKmh or 0) >= LC.V_ENTER_KMH then return false, "Launch Control: only at a standstill" end
  return true, nil
end

-- src: "manual" (Alt+K / Actions app / QA T14) or "auto" (SPORT + DTC +
-- brake + kickdown). A manual press while armed switches it off again.
function M.arm(ctx, src)
  if not ctx then return false end
  src = src or "manual"
  local p = st.phase
  if p == "FIRE" then return false end
  if src == "manual" and (p == "ARMED" or p == "PRELOAD" or p == "READY") then
    if p ~= "ARMED" then exitHold(ctx, "OFF", "manual-off") else disarm(ctx, "manual-off") end
    msg("Launch Control off", 2)
    return false
  end
  local ok, why = armAllowed(ctx, src)
  if not ok then
    st.reason = why
    if src == "manual" then msg(why, 3) end
    lcLog("[LC] arm rejected (%s): %s gear=%s v=%.2f", src, why, tostring(electrics.values.gear), ctx.speedKmh or 0)
    return false
  end
  resetArmState()
  st.armSrc, st.armDe = src, ctx.deMode
  st.trim = 1
  st.reason = "none"
  setPhase(ctx, "ARMED", src)
  msg("LAUNCH CONTROL - brake + full throttle", 3)
  return true
end

local function autoArmCheck(ctx, dt, pedal, brake, kmh)
  local key = ctx.dscKey
  if LC.AUTO_ARM and not st.rearmBlock and ctx.deMode == "SPORT" and (key == "dtc" or key == "off")
     and gearFwd(ctx) and kmh < LC.V_ENTER_KMH and brake >= LC.AUTO_BRAKE and pedal >= LC.AUTO_PEDAL
     and not ctx.evOnly and not ctx.seqFailsafe then
    st.autoT = st.autoT + dt
    if st.autoT >= LC.AUTO_S then
      st.autoT = 0
      if not M.arm(ctx, "auto") then st.rearmBlock = true end   -- one try per press
    end
  else
    st.autoT = 0
  end
end

-- ===========================================================
-- Per-phase updates
-- ===========================================================
local function updateArmed(ctx, dt, pedal, brake, kmh)
  st.armT = st.armT + dt
  if not gearFwd(ctx) then disarm(ctx, "gear"); return end
  if ctx.evOnly or ctx.seqFailsafe then disarm(ctx, "mode"); return end
  if st.armDe ~= ctx.deMode then disarm(ctx, "de-change"); return end
  if kmh > LC.V_DISARM_KMH then disarm(ctx, "drove-off"); return end
  if st.armT >= LC.ARM_TIMEOUT_S then
    disarm(ctx, "timeout")
    msg("Launch Control off (timeout)", 2)
    return
  end
  if brake >= LC.BRAKE_ENTER and pedal >= LC.PEDAL_ENTER and kmh < LC.V_ENTER_KMH then
    if st.unloadT > 0 or not iceReady(ctx) then return end   -- the guard caps meanwhile
    local nFf, idle = computeTargets(ctx, LC.BRAKE_ASSIST, LC.MARGIN)
    if nFf < idle + LC.N_MIN_OVER_IDLE then
      if not st.weakMsg then
        st.weakMsg = true
        st.reason = "brake-weak"
        msg("LC: brake harder", 2)
      end
      return
    end
    enterPreload(ctx)
    regulate(ctx, dt)                -- outputs from the entry frame on
  else
    st.weakMsg = false
  end
end

local function updateHold(ctx, dt, pedal, brake, kmh, creep)
  st.holdT = st.holdT + dt
  st.preT = st.preT + dt
  -- exits first
  if brake < LC.BRAKE_FIRE then
    -- floored pedal: fire, also within the first 0.3 s of PRELOAD - falling
    -- back to the normal map would give the same demand with the slower
    -- SPORT limiter and the generator unload in between
    if pedal >= LC.PEDAL_FIRE then
      fire(ctx)
    else
      exitHold(ctx, "OFF", "brake-released", pedal >= LC.UNLOAD_SKIP_PEDAL)
    end
    return
  end
  if pedal < LC.PEDAL_HOLD then st.pedalLowT = st.pedalLowT + dt else st.pedalLowT = 0 end
  if st.pedalLowT >= LC.PEDAL_HOLD_S then exitHold(ctx, "ARMED", "pedal-lift"); return end
  if st.holdT >= LC.HOLD_MAX_S then
    exitHold(ctx, "ABORT", "hold-time")
    msg("Launch Control aborted (hold time exceeded)", 3)
    return
  end
  if not gearFwd(ctx) or ctx.evOnly or st.armDe ~= ctx.deMode then exitHold(ctx, "OFF", "mode"); return end
  if not iceReady(ctx) then exitHold(ctx, "ARMED", "ice-lost"); return end
  -- targets, anti-creep, regulator
  computeTargets(ctx, LC.BRAKE_ASSIST, LC.MARGIN)
  updateTrim(dt, creep)
  -- the filtered wheel / body speeds join after the entry settle window
  -- (<= SETTLE_MAX_S): until then they still carry the ARMED / guard window
  -- (enterPreload keeps the filters; T14a 20261010-115509 latched the cut on
  -- a stale 2.08 km/h in the first hold frame at v = 0.13 km/h). The vehicle
  -- speed itself counts from the first frame.
  local vMax = st.settled and max(kmh, st.whlF * 3.6, st.bodyF * 3.6) or kmh
  if vMax > LC.V_OVER_KMH and not st.cut then
    st.cut = true
    st.reason = "overspeed"
    lcLog("[LC] hold overspeed %.2fkm/h - ICE torque 0 for the rest of the hold", vMax)
  end
  -- the trim asks for less than the idle + 150 rpm floor can deliver: the
  -- speed regulator cannot follow, so cut the ICE torque now (header)
  if st.floorBind and st.trim < 1 and not st.cut then
    st.cut = true
    st.reason = "trim-floor"
    msg("LC: hold limit reached - brake harder", 2)
    lcLog("[LC] hold trim floor (trim %.2f, T*=%.0f Nm below the idle + %d rpm floor) - ICE torque 0",
      st.trim, st.tStar, LC.N_MIN_OVER_IDLE)
  end
  regulate(ctx, dt)
  if st.phase == "PRELOAD" then
    local nNow = (st.nReg > 0) and st.nReg or abs(ctx.rpmMotor or 0)   -- regulator speed
    if abs(nNow - st.nStar) < LC.READY_TOL_RPM then
      st.readyT = st.readyT + dt
    else
      st.readyT = 0
    end
    if st.readyT >= LC.READY_SETTLE_S or st.preT >= LC.READY_MAX_S then
      setPhase(ctx, "READY", (st.readyT >= LC.READY_SETTLE_S) and "settled" or "timeout")
      msg("LC READY - release brake", 2)
    end
  end
end

local function updateFire(ctx, dt, pedal, brake)
  st.activeT = st.activeT - dt
  if brake > LC.ACTIVE_ABORT_BRAKE or pedal < LC.ACTIVE_ABORT_PEDAL then
    st.activeT = 0
    disarm(ctx, "active-abort")
    return
  end
  if st.activeT <= 0 then disarm(ctx, "window-end") end
end

-- Brake-throttle guard: torque cap on the coordinator request whenever the
-- driver brakes and presses the pedal at a standstill outside the hold.
local function updateGuard(ctx, dt, pedal, brake, kmh, creep)
  local p = st.phase
  if p == "PRELOAD" or p == "READY" or p == "FIRE" then st.guard = false; return end
  local fwd = gearFwd(ctx)
  local on
  if st.guard then
    on = fwd and brake >= LC.GUARD_BRAKE_OUT and pedal >= LC.GUARD_PEDAL_OUT and kmh <= LC.GUARD_V_OUT_KMH
  else
    -- held or rising brake: at once. Only while the brake is being RELEASED
    -- (it fell faster than GUARD_FALL_RATE within the last GUARD_FALL_HOLD_S)
    -- the overlap has to persist GUARD_DWELL_S - the gamepad trigger cross-
    -- over at a normal braked start (brake decaying while the throttle rises)
    local overlap = LC.GUARD_ENABLE and fwd and kmh < LC.GUARD_V_KMH and brake >= LC.GUARD_BRAKE
      and pedal > LC.GUARD_PEDAL and st.brakeRate > -LC.GUARD_BRAKE_FALL
    if overlap then st.guardDwellT = st.guardDwellT + dt else st.guardDwellT = 0 end
    local falling = st.brakeFallT < LC.GUARD_FALL_HOLD_S
    on = (overlap and (not falling or st.guardDwellT >= LC.GUARD_DWELL_S))
      or (fwd and kmh < LC.GUARD_V_KMH and p == "ABORT" and brake >= LC.GUARD_BRAKE_OUT
          and pedal >= LC.GUARD_PEDAL_OUT)
    if on then
      resetSensing()
      lcLog("[LC] guard on (phase %s) brake=%.2f pedal=%.2f v=%.2f dwell=%.2fs falling=%s",
        p, brake, pedal, kmh, st.guardDwellT, tostring(falling))
    end
  end
  if on then st.guardDwellT = 0 end
  st.guard = on and true or false
  if not st.guard then st.guardNm = 0; return end
  computeTargets(ctx, false, LC.GUARD_MARGIN)
  updateTrim(dt, creep)
  st.guardNm = (st.cutT > 0) and 0 or st.tStar
end

-- ===========================================================
-- Main update (master updateGFX, after the ICE sequencer)
-- ===========================================================
function M.updateGFX(ctx, dt)
  if not ctx or not dt or dt <= 0 then return end
  st.now = st.now + dt
  local pedal = ctx.throttleRaw or 0
  local brake = ctx.brakeRaw or 0
  local kmh = ctx.speedKmh or 0
  if st.rearmBlock and pedal < LC.PEDAL_REARM then st.rearmBlock = false end
  -- brake pedal rate, time since the brake last fell (guard dwell) and axle
  -- loads (latched while holding)
  local bp = st.brakePrev or brake
  st.brakeRate = st.brakeRate + ((brake - bp) / dt - st.brakeRate) * min(1, dt / LC.BRAKE_RATE_TAU_S)
  if bp - brake > LC.GUARD_FALL_RATE * dt then st.brakeFallT = 0 else st.brakeFallT = st.brakeFallT + dt end
  st.brakePrev = brake

  local p = st.phase
  updateLoads(dt, p == "PRELOAD" or p == "READY" or st.guard)
  local sensing = (p == "PRELOAD" or p == "READY") or st.guard
    or (brake >= LC.GUARD_BRAKE_OUT and pedal > LC.GUARD_PEDAL_OUT and kmh < LC.GUARD_V_OUT_KMH)
  local creep = 0
  if sensing then
    creep = senseCreep(ctx, dt)
  else
    st.turbF, st.whlF, st.bodyF, st.creep = 0, 0, 0, 0
  end

  if p == "ABORT" and (brake < LC.GUARD_BRAKE_OUT or pedal < LC.GUARD_PEDAL_OUT) then
    -- the pedal lift after an abort: the hold's unload ran out during the
    -- (still floored) abort window, so the coordinator would ramp the ICE
    -- share down at its normal rate (~0.15 s of full ICE torque into the
    -- converter against the held brakes - QA T14b 2026-10-10: rear wheels to
    -- 9 km/h, body 1 km/h). With the brake still held: hard reset instead.
    if brake >= LC.GUARD_BRAKE_OUT then ctx.launchDrop = true end
    disarm(ctx, "abort-released")
  end
  if st.phase == "OFF" then autoArmCheck(ctx, dt, pedal, brake, kmh) end
  p = st.phase
  if p == "ARMED" then
    updateArmed(ctx, dt, pedal, brake, kmh)
  elseif p == "PRELOAD" or p == "READY" then
    updateHold(ctx, dt, pedal, brake, kmh, creep)
  elseif p == "FIRE" then
    updateFire(ctx, dt, pedal, brake)
  end
  p = st.phase
  local hold = (p == "PRELOAD" or p == "READY")
  if not hold and st.unloadT > 0 then unloadStep(ctx, dt) end
  updateGuard(ctx, dt, pedal, brake, kmh, creep)
  if st.phase == "OFF" and not st.guard and st.unloadT <= 0 then st.trim = 1 end

  -- DSC hold assist flag for the physics-rate hook (never while moving)
  st.assistOn = LC.BRAKE_ASSIST and kmh < LC.V_OVER_KMH
    and (hold or (st.unloadT > 0 and brake >= LC.BRAKE_HOLD)) or false
  -- physics-rate frame means for the regulator / unload of the NEXT frame
  st.accOn = hold or st.unloadT > 0
  if not st.accOn then st.accAV, st.accTq, st.accN = 0, 0, 0 end

  -- ctx contract
  ctx.launchArmed = (p == "ARMED" or hold)
  ctx.launchReady = (p == "READY")
  ctx.launchActive = (p == "FIRE")
  ctx.launchHold = hold or st.unloadT > 0
  ctx.launchIceThr = st.iceThr
  ctx.launchGenNm = st.G
  ctx.launchPumpCapNm = hold and st.tStar or (st.unloadT > 0 and st.unloadCap)
                        or (st.guard and st.guardNm or 0)
  ctx.launchTargetRpm = hold and st.nStar or 0
  ctx.launchGuardNm = (st.guard and not ctx.launchHold) and st.guardNm or nil

  -- squat reference: rear spring length at rest with the throttle released
  if st.springs then
    st.springNow = springLen()
    if (p == "OFF" or p == "ARMED") and not st.guard and pedal < 0.1 and kmh < LC.V_ENTER_KMH then
      if st.springRef == nil then st.springRef = st.springNow
      else st.springRef = st.springRef + (st.springNow - st.springRef) * min(1, dt / LC.SPRING_TAU_S) end
    end
  end
  if p ~= "OFF" or st.guard or st.unloadT > 0 then
    st.levelHoldT = LC.LEVEL_HOLD_S
  else
    st.levelHoldT = max(0, st.levelHoldT - dt)
  end

  -- channels
  local ev = electrics.values
  ev.etk854hybrid_launchState = PUBLIC[p] or "IDLE"
  ev.etk854hybrid_launchPhase = (p == "OFF" and st.guard) and "GUARD" or p
  ev.etk854hybrid_launchPumpCapNm = ctx.launchPumpCapNm
  ev.etk854hybrid_launchTargetRpm = ctx.launchTargetRpm
  ev.etk854hybrid_launchRefRpm = hold and st.nRef or 0
  ev.etk854hybrid_launchShaftRpm = hold and st.nReg or 0          -- regulator speed (frame mean + LPF)
  ev.etk854hybrid_launchIceCapNm = hold and st.iceCapNm or 0     -- ICE torque target (GEN) / command (ICE)
  ev.etk854hybrid_launchGenNm = st.G
  ev.etk854hybrid_launchIceThr = (hold or st.unloadT > 0) and st.iceThr or 0
  ev.etk854hybrid_launchTrim = st.trim
  ev.etk854hybrid_launchHoldS = hold and st.holdT or 0
  ev.etk854hybrid_launchBrakeCapNm = (hold or st.guard) and st.capNm or 0
  ev.etk854hybrid_launchRearBrakeNm = (hold or st.guard) and st.capR or 0
  ev.etk854hybrid_launchGroundNm = (hold or st.guard) and st.capGround or 0
  ev.etk854hybrid_launchMode = hold and st.mode or (st.guard and "GUARD" or "-")
  ev.etk854hybrid_launchCreep = st.creep
  ev.etk854hybrid_launchAssist = st.assistOn and 1 or 0
  ev.etk854hybrid_launchUnload = (st.unloadT > 0) and 1 or 0
  ev.etk854hybrid_launchReason = st.reason
  ev.etk854hybrid_launchLevelHold = (st.levelHoldT > 0) and 1 or 0
  ev.etk854hybrid_launchTurbineRpm = st.turbF
  if st.springs then
    ev.etk854hybrid_launchSpringRmm = st.springNow * 1000
    ev.etk854hybrid_launchSquatMm = st.springRef and (st.springRef - st.springNow) * 1000 or 0
  end
end

-- Last writer of the motor / ICE channels while the hold (or its unload)
-- runs - after recuperation, anti-kickback and BATTERY CONTROL, which all
-- write etk854hybrid_evRegen (master updateGFX order).
function M.applyOutputs(ctx, dt)
  if not (ctx and ctx.launchHold) then return end
  local ev = electrics.values
  ev.etk854hybrid_evThrottle = 0
  ev.etk854hybrid_fillGain = 0
  -- EV creep floor inert (defence in depth: etk854hybrid_creep.lua already
  -- hard-cuts on every launch flag; this is the frame's last writer)
  ev.etk854hybrid_creepMaxNm = 0
  ev.etk854hybrid_creepCut = 1
  ev.etk854hybrid_evRegen = st.regenFrac
  ev.etk854hybrid_iceThrottle = ctx.iceOn and st.iceThr or 0
end

-- Physics rate (master M.updateWheelsIntermediate, main.lua onPhysicsStep:
-- after wheels.updateWheelVelocities has set desiredBrakingTorque, before
-- wheels.updateWheelTorques applies it). Same pattern as the DSE brake stage
-- (brakeControl.lua:49-56): raise, never lower.
function M.wheelsIntermediate(dt)
  -- frame means for the hold regulator (st.accOn: PRELOAD / READY / unload):
  -- motorShaft speed (= ICE speed with K0 closed) and ICE combustion torque,
  -- both already updated by powertrain.update in this physics step
  if st.accOn then
    local md = st.motor
    local av = md and md.outputAV1
    if type(av) == "number" then
      local ice = st.ice
      local tq = ice and ice.combustionTorque
      st.accAV = st.accAV + ((av < 0) and -av or av)
      st.accTq = st.accTq + ((type(tq) == "number") and tq or 0)
      st.accN = st.accN + 1
    end
  end
  if not st.assistOn then return end
  local all = st.all
  for i = 1, #all do
    local wd = all[i]
    if not wd.isBroken then
      local bt = wd.brakeTorque or 0
      if (wd.desiredBrakingTorque or 0) < bt then wd.desiredBrakingTorque = bt end
    end
  end
end

-- ===========================================================
-- Lifecycle
-- ===========================================================
local function resetState(ctx)
  st.phase, st.why, st.reason, st.armSrc, st.armDe = "OFF", "none", "none", "none", nil
  st.autoT, st.activeT = 0, 0
  resetArmState()
  st.rearmBlock = false
  st.mode = "-"
  st.trim, st.hardN = 1, 0
  resetSensing()
  st.turbF, st.whlF, st.bodyF = 0, 0, 0
  st.capNm, st.tFf, st.tStar, st.nStar, st.nRef, st.ratio = 0, 0, 0, 0, 0, 0
  clearHoldState()
  st.iceLpf = 0
  st.unloadT, st.unloadCap, st.assistOn = 0, 0, false
  st.accOn, st.accAV, st.accTq, st.accN = false, 0, 0, 0
  st.guard, st.guardNm, st.guardDwellT, st.brakePrev, st.brakeRate = false, 0, 0, nil, 0
  st.brakeFallT = 9
  st.capR, st.capGround, st.floorBind = 0, 0, false
  st.nR, st.nF = 0, 0
  st.levelHoldT = 0
  st.springRef, st.springNow = nil, 0
  if ctx then
    ctx.launchArmed, ctx.launchReady, ctx.launchActive, ctx.launchHold = false, false, false, false
    ctx.launchIceThr, ctx.launchGenNm, ctx.launchPumpCapNm, ctx.launchTargetRpm = 0, 0, 0, 0
    ctx.launchGuardNm, ctx.launchSeedNm = nil, nil
  end
  local ev = electrics.values
  ev.etk854hybrid_launchState = "IDLE"
  ev.etk854hybrid_launchPhase = "OFF"
  ev.etk854hybrid_launchPumpCapNm = 0
  ev.etk854hybrid_launchTargetRpm = 0
  ev.etk854hybrid_launchRefRpm = 0
  ev.etk854hybrid_launchShaftRpm = 0
  ev.etk854hybrid_launchIceCapNm = 0
  ev.etk854hybrid_launchGenNm = 0
  ev.etk854hybrid_launchIceThr = 0
  ev.etk854hybrid_launchTrim = 1
  ev.etk854hybrid_launchHoldS = 0
  ev.etk854hybrid_launchBrakeCapNm = 0
  ev.etk854hybrid_launchRearBrakeNm = 0
  ev.etk854hybrid_launchGroundNm = 0
  ev.etk854hybrid_launchMode = "-"
  ev.etk854hybrid_launchCreep = 0
  ev.etk854hybrid_launchAssist = 0
  ev.etk854hybrid_launchUnload = 0
  ev.etk854hybrid_launchReason = "none"
  ev.etk854hybrid_launchLevelHold = 0
  ev.etk854hybrid_launchTurbineRpm = 0
end

function M.init(ctx)
  local jb = ctx and ctx.jbeamData
  -- controller args (etk854hybrid_powertrain.jbeam, hybridControl)
  local g = readNumber(jb, "launchGenLoad", LC.GEN_LOAD)
  LC.GEN_LOAD = (g == 0 or g == 2) and g or 1
  LC.BRAKE_ASSIST = readNumber(jb, "launchBrakeAssist", LC.BRAKE_ASSIST and 1 or 0) ~= 0
  LC.GUARD_ENABLE = readNumber(jb, "launchBrakeGuard", LC.GUARD_ENABLE and 1 or 0) ~= 0
  LC.AUTO_ARM = readNumber(jb, "launchAutoArm", LC.AUTO_ARM and 1 or 0) ~= 0
  LC.MARGIN = clamp(readNumber(jb, "launchMargin", LC.MARGIN), 0.40, 0.90)
  resetState(ctx)
  if ctx and ctx.dbg then
    ctx.dbg(1, "launch", string.format("init: gen=%s assist=%s guard=%s autoArm=%s margin=%.2f",
      tostring(LC.GEN_LOAD), tostring(LC.BRAKE_ASSIST), tostring(LC.GUARD_ENABLE),
      tostring(LC.AUTO_ARM), LC.MARGIN))
  end
end

function M.initStage2(ctx)
  findDevices()
  findWheels()
  local ok = pcall(findSprings)
  if not ok then st.springs = nil end
  converterConst()
  local okM = pcall(findMass)
  if not okM then st.massKg = 0 end
  log("I", "etk854hybrid.launch", string.format(
    "[LC] ready: wheels=%d rear=%d front=%d c=%.6f TR=%.2f i=%.2f springs=%s mass=%.0fkg gen=%s(%s) assist=%s guard=%s autoArm=%s margin=%.2f",
    st.wheelCount, #st.rear, #st.front, st.c, st.tr, mechRatio(), tostring(st.springs ~= nil), st.massKg,
    tostring(LC.GEN_LOAD), tostring(genWanted()), tostring(LC.BRAKE_ASSIST), tostring(LC.GUARD_ENABLE),
    tostring(LC.AUTO_ARM), LC.MARGIN))
end

function M.reset(ctx)
  resetState(ctx)
  M.initStage2(ctx)
end

function M.updateFixedStep(ctx, dt) end

-- Live tuning (console / dev server): M.setParam('MARGIN', 0.7). Existing
-- numeric or boolean keys of LC only; returns the old value; not persistent.
function M.setParam(key, value)
  local old = LC[key]
  if type(old) == "number" and type(value) == "number" then
    LC[key] = value
    return old
  end
  if type(old) == "boolean" and type(value) == "boolean" then
    LC[key] = value
    return old
  end
  return nil
end

function M.getState()
  return {
    phase = st.phase, why = st.why, reason = st.reason, mode = st.mode, armSrc = st.armSrc,
    holdT = st.holdT, trim = st.trim, capNm = st.capNm, tFf = st.tFf, tStar = st.tStar,
    nStar = st.nStar, nRef = st.nRef, c = st.c, tr = st.tr, ratio = st.ratio,
    G = st.G, iceThr = st.iceThr, gSat = st.gSat, cut = st.cut, hardN = st.hardN,
    nReg = st.nReg, iceLpf = st.iceLpf, iceCapNm = st.iceCapNm, iceTrim = st.iceTrim, gI = st.gI, iI = st.iI,
    guard = st.guard, guardNm = st.guardNm, unloadT = st.unloadT, assistOn = st.assistOn,
    brakeFallT = st.brakeFallT, guardDwellT = st.guardDwellT,
    turbF = st.turbF, whlF = st.whlF, bodyF = st.bodyF, settled = st.settled,
    springs = st.springs ~= nil, squatMm = st.springRef and (st.springRef - st.springNow) * 1000 or 0,
    rear = #st.rear, front = #st.front,
  }
end

return M
