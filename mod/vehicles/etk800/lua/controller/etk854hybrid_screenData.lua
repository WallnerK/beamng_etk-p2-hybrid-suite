-- ETK 854 ActiveHybrid - centre screen data feed (energy-flow page)
-- =========================================================================
-- Target: BeamNG.drive 0.39.4
--
-- Instantiated by the part etk854hybrid_navi_energyflow
-- (etk854hybrid_gauges.jbeam, slot "gps" of the stock ETK 800 dash).
--
-- The centre screen texture itself is created by the stock controller
-- "beamNavigator" named "etk800_navi" (declared by the dash,
-- etk800_interior.jbeam). The part only overrides that controller's options
-- through a top-level section named "etk800_navi" (controller.lua:499 merges
-- v.data[<controller name>] over the row), so the web view on
-- "@etk800_screen" loads our page at 512x256 instead of the map page. The
-- navigator keeps calling map.updateData() at 15 Hz and the game engine sends
-- map.setData() once; the page provides both as cheap stubs.
--
-- This controller only streams the hybrid state into that existing web view:
--   obj:queueWebViewStreamJS(tag, "etk854hybridFlow", "etk854hybridUpdate(<json>)")
-- (same call htmlTexture.streamJS uses, lua/vehicle/htmlTexture.lua:12-15).
-- It never creates a web view of its own.
--
-- Range / consumption come from the cluster module
-- gauges/customModules/etk854hybrid_gaugeData when that part is fitted, so
-- both screens show the same numbers. Without it the module file is required
-- as a plain library and driven from here (resolved lazily on the first
-- update, after every controller has been initialised).
--
-- jbeam options (controller row):
--   screenTag   web view tag / material emissive tag   default "@etk800_screen"
--   rateHz      push rate while a player is seated     default 15
--
-- Payload (keys kept short, the page tolerates every key missing):
--   soc tgt ph de ed ev ice evNm iceNm k0 ies on sail chg awd awdNm trl lvl
--   fan boost evAv evFr rl op dsc prog spd gear gi ign rng cons cap use
--   iceMax fuel amb wt uL uT uC uV
--   on-board computer (gaugeData header item 10, same numbers as the cluster):
--   fl (fuel l, -1 unknown) fcap (tank l) frng (fuel range km, -1)
--   fi (instantaneous l/100 km, -1 n/a) fh (l/h) fm ("l100" | "lh" | "ev")
--   fa (trip average l/100 km, -1 until tripMinKm) tk (trip km)
-- uL / uT / uC / uV are the game's uiUnitLength / uiUnitTemperature /
-- uiUnitConsumptionRate / uiUnitVolume settings (the same values
-- genericGauges hands the cluster in setup()), re-read every 2 s so a
-- change in the options menu reaches the page without a vehicle reload.
-- =========================================================================

local M = {}
M.type = "auxiliary"

local GAUGE_MODULE = "gauges/customModules/etk854hybrid_gaugeData"

local st = {
  tag = "@etk800_screen",
  interval = 1 / 15,
  timer = 0,
  energy = nil, -- module providing getEnergyInfo()
  ownEnergy = false, -- true when this controller drives the module itself
  resolved = false,
  payload = {},
  unitTimer = 0,
  unitLength = "metric",
  unitTemp = "c",
  unitCons = "metric",
  unitVol = "l"
}

local UNIT_REFRESH_S = 2

local function readUnits()
  if settings and settings.getValue then
    st.unitLength = settings.getValue("uiUnitLength") or "metric"
    st.unitTemp = settings.getValue("uiUnitTemperature") or "c"
    st.unitCons = settings.getValue("uiUnitConsumptionRate") or "metric"
    st.unitVol = settings.getValue("uiUnitVolume") or "l"
  end
end

local function resolveEnergySource()
  st.resolved = true
  local c = controller.getController(GAUGE_MODULE)
  if c and c.getEnergyInfo then
    st.energy, st.ownEnergy = c, false
    return
  end
  local ok, mod = pcall(require, "controller/" .. GAUGE_MODULE)
  if ok and type(mod) == "table" and mod.getEnergyInfo and mod.init then
    mod.init({})
    st.energy, st.ownEnergy = mod, true
  else
    st.energy, st.ownEnergy = nil, false
  end
end

local function str(v)
  if type(v) == "string" and v ~= "" then
    return v
  end
  return nil
end

local function buildPayload()
  local ev = electrics.values
  local p = st.payload
  p.soc = ev.etk854hybrid_batterySoc
  p.tgt = ev.etk854hybrid_targetSoc
  p.ph = str(ev.etk854hybrid_phase)
  p.de = str(ev.etk854hybrid_deMode)
  p.ed = str(ev.etk854hybrid_edriveMode)
  p.ev = ev.etk854hybrid_evPower
  p.ice = ev.etk854hybrid_icePower
  p.evNm = ev.etk854hybrid_evTorqueNm
  p.iceNm = ev.etk854hybrid_iceTorqueNm
  p.k0 = ev.etk854hybrid_k0ClutchRatio
  p.ies = str(ev.etk854hybrid_iceEngageState)
  p.on = ev.etk854hybrid_iceState
  p.sail = ev.etk854hybrid_sailActive
  p.chg = str(ev.etk854hybrid_chargeMode)
  p.awd = str(ev.etk854hybrid_awdMode)
  p.awdNm = ev.etk854hybrid_awdFrontTqNm
  p.trl = ev.etk854hybrid_trailerMode
  p.lvl = str(ev.etk854hybrid_levelState)
  p.fan = ev.etk854hybrid_fanStage
  p.boost = ev.etk854hybrid_boostStage
  p.evAv = ev.etk854hybrid_evAvailable
  p.evFr = ev.etk854hybrid_evPowerFraction
  p.rl = str(ev.etk854hybrid_regenLevel)
  p.op = ev.etk854hybrid_onePedalHold
  p.dsc = str(ev.etk854hybrid_dscMode)
  p.prog = str(ev.etk854hybrid_shiftProgram)
  p.amb = ev.etk854hybrid_ambientC
  p.spd = (ev.wheelspeed or 0) * 3.6
  p.gear = ev.gear
  p.gi = ev.gearIndex
  p.ign = ev.ignitionLevel
  p.fuel = ev.fuel
  p.wt = ev.watertemp
  p.uL = st.unitLength
  p.uT = st.unitTemp
  p.uC = st.unitCons
  p.uV = st.unitVol
  local e = st.energy and st.energy.getEnergyInfo and st.energy.getEnergyInfo()
  if e then
    p.rng = e.rangeKm
    p.cons = e.consKwh
    p.cap = e.capKwh
    p.use = e.usableKwh
    p.iceMax = e.iceKw
    p.fl = e.fuelL
    p.fcap = e.fuelCapL
    p.frng = e.fuelRangeKm
    p.fi = e.instL100
    p.fh = e.instLh
    p.fm = e.instMode
    p.fa = e.avgL100
    p.tk = e.tripKm
  end
  return p
end

local function init(jbeamData)
  jbeamData = jbeamData or {}
  st.tag = jbeamData.screenTag or st.tag
  local hz = tonumber(jbeamData.rateHz) or 15
  if hz < 1 then
    hz = 1
  elseif hz > 60 then
    hz = 60
  end
  st.interval = 1 / hz
  st.timer = 0
  st.resolved = false
  st.unitTimer = 0
  readUnits()
end

local function reset()
  st.timer = 0
  if st.ownEnergy and st.energy and st.energy.reset then
    st.energy.reset()
  end
end

local function updateGFX(dt)
  if not st.resolved then
    resolveEnergySource()
  end
  if st.ownEnergy and st.energy then
    st.energy.updateGFX(dt)
  end
  st.unitTimer = st.unitTimer + dt
  if st.unitTimer >= UNIT_REFRESH_S then
    st.unitTimer = 0
    readUnits()
  end
  st.timer = st.timer + dt
  if st.timer < st.interval or not playerInfo.anyPlayerSeated then
    return
  end
  st.timer = 0
  obj:queueWebViewStreamJS(st.tag, "etk854hybridFlow", "etk854hybridUpdate(" .. jsonEncode(buildPayload()) .. ")")
end

M.init = init
M.reset = reset
M.updateGFX = updateGFX

return M
