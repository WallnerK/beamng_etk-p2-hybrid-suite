-- ETK 854 ActiveHybrid - centre screen data feed (energy flow + trailer page)
-- Controller of the part etk854hybrid_navi_energyflow. The web view belongs to the
-- stock beamNavigator "etk800_navi"; the part only re-targets its options to our
-- page (top-level jbeam section merged over the controller row; the page stubs
-- map.updateData / map.setData). This controller streams the state while a player
-- is seated: queueWebViewStreamJS(tag, "etk854hybridFlow", "etk854hybridUpdate(<json>)").
-- Range, consumption and on-board computer come from the cluster module's
-- getEnergyInfo() so both screens agree; without the cluster part the module is
-- required and driven from here (resolved on the first update, once every
-- controller is initialised).
-- jbeam options: screenTag (default "@etk800_screen"), rateHz (15, 1..60).
--
-- Payload (short keys, all optional on the page):
--   hybrid   soc tgt ph de ed ev ice k0 ies on sail awd awdNm trl lvl
--            boost evAv rl op dsc amb spd ign fuel
--   energy   rng cons iceMax
--   obc      fl fi fh fm ("l100" | "lh" | "ev") fa tk (-1 = unknown)
--   trailer  tsa (nil without the hitch controller: page stays on the energy flow)
--            tsSt tsK tsL tsSw tsFx (N, + = pull)
--            tsTg (kg, signed) tsTs tsM (-1 = learning) tsMq tsLp tsVr tsRs trA
--            rsSel rsMd rsSt lvlMm
--   units    uL uT uC uV uW: game UI units, re-read every UNIT_REFRESH_S
--            (tongue load / mass follow uW; distance, speed, force follow uL)

local M = {}
M.type = "auxiliary"

local GAUGE_MODULE = "gauges/customModules/etk854hybrid_gaugeData"

local st = {
  tag = "@etk800_screen",
  interval = 1 / 15,
  timer = 0,
  energy = nil, -- module providing getEnergyInfo()
  ownEnergy = false, -- driven from here
  resolved = false,
  payload = {},
  unitTimer = 0,
  unitLength = "metric",
  unitTemp = "c",
  unitCons = "metric",
  unitVol = "l",
  unitWeight = "kg"
}

local UNIT_REFRESH_S = 2

local function readUnits()
  if settings and settings.getValue then
    st.unitLength = settings.getValue("uiUnitLength") or "metric"
    st.unitTemp = settings.getValue("uiUnitTemperature") or "c"
    st.unitCons = settings.getValue("uiUnitConsumptionRate") or "metric"
    st.unitVol = settings.getValue("uiUnitVolume") or "l"
    st.unitWeight = settings.getValue("uiUnitWeight") or "kg"
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

local U = require("controller/etk854hybrid/etk854hybrid_util")
local str = U.str -- non-empty string, else nil

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
  p.k0 = ev.etk854hybrid_k0ClutchRatio
  p.ies = str(ev.etk854hybrid_iceEngageState)
  p.on = ev.etk854hybrid_iceState
  p.sail = ev.etk854hybrid_sailActive
  p.awd = str(ev.etk854hybrid_awdMode)
  p.awdNm = ev.etk854hybrid_awdFrontTqNm
  p.trl = ev.etk854hybrid_trailerMode
  p.lvl = str(ev.etk854hybrid_levelState)
  p.boost = ev.etk854hybrid_boostStage
  p.evAv = ev.etk854hybrid_evAvailable
  p.rl = str(ev.etk854hybrid_regenLevel)
  p.op = ev.etk854hybrid_onePedalHold
  p.dsc = str(ev.etk854hybrid_dscMode)
  p.amb = ev.etk854hybrid_ambientC
  p.spd = (ev.wheelspeed or 0) * 3.6
  p.ign = ev.ignitionLevel
  p.fuel = ev.fuel
  p.uL = st.unitLength
  p.uT = st.unitTemp
  p.uC = st.unitCons
  p.uV = st.unitVol
  p.uW = st.unitWeight
  -- trailer page
  p.tsa = str(ev.etk854hybrid_tsaState)
  p.tsSt = ev.etk854hybrid_tsaStage
  p.tsK = ev.etk854hybrid_tsaCorner
  p.tsL = ev.etk854hybrid_tsaLamp
  p.tsSw = ev.etk854hybrid_tsaSwayIdx
  p.tsFx = ev.etk854hybrid_tsaDrawbarN
  p.tsTg = ev.etk854hybrid_tsaTongueKg
  p.tsTs = str(ev.etk854hybrid_tsaTongueState)
  p.tsM = ev.etk854hybrid_tsaMassKg
  p.tsMq = ev.etk854hybrid_tsaMassQ
  p.tsLp = str(ev.etk854hybrid_tsaLoadPin)
  p.tsVr = ev.etk854hybrid_tsaVRecKmh
  p.tsRs = ev.etk854hybrid_tsaReduceSpeed
  local ta, tf = ev.etk854hybrid_trailerAttached, ev.etk854hybrid_tsaFlags
  p.trA = (ta == 1 or ta == true or (type(tf) == "number" and tf % 2 == 1)) and 1 or 0
  p.rsSel = str(ev.etk854hybrid_rasSel)
  p.rsMd = str(ev.etk854hybrid_rasMode)
  p.rsSt = str(ev.etk854hybrid_rasState)
  p.lvlMm = ev.etk854hybrid_levelOffsetMm
  local e = st.energy and st.energy.getEnergyInfo and st.energy.getEnergyInfo()
  if e then
    p.rng = e.rangeKm
    p.cons = e.consKwh
    p.iceMax = e.iceKw
    p.fl = e.fuelL
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
