-- ETK 854 ActiveHybrid - cockpit displays: view selection + centre-screen control panel
-- Target: BeamNG.drive 0.39.4
--
-- Part etk854hybrid_navi_energyflow (slot "gps"). Owns UI state only: the centre-screen view
-- (ENERGY / MAP / TRAILER), the cluster view (AUTO / HYBRID / EFFICIENCY / SPORT / MINIMAL;
-- AUTO follows the Driving Experience) and the MENU overlay, an in-car control panel with every
-- Actions app setting plus the display selections. Vehicle modes are never stored here: every
-- highlight is recomputed from electrics / owner getters, every change goes through the owning
-- controller's setter, looked up at call time (skipped when the part is absent) under pcall.
--
-- Input API (safe any time; ignitionLevel < 1: panel input ignored, menu closed, views settable):
--   touchRay(px,py,pz, dx,dy,dz)  camera ray from the GE trigger action etk854hybrid_cdTouch:
--       origin relative to the vehicle, direction in world axes. Relative because the
--       GE -> vehicle queue lags a frame or more; a world origin would be off by v * lag.
--   touchAt(x, y)                 page px (0..512, 0..256)
--   menu() prev() next() press() back()                         controller keys
--   setCentreView(n) cycleCentreView() setClusterView(n) cycleClusterView()
--   openMenu(tab) closeMenu() getState()                        tests / console
--
-- Electrics: etk854hybrid_cdView (effective centre view), _cdMenu 0|1, _cdTab,
--   _clusterViewSel, _clusterView (AUTO resolved). TRAILER needs etk854hybrid_tsaState;
--   without it ENERGY is shown and the selection is kept.
--
-- Page protocol (etk854hybrid_cockpit_panel.js): obj:queueWebViewJS(screenTag,
--   "etk854hybridPanel(<json>)") with {v = 1, view, menu = 0|1, tab, w, tp}; tab / w only with
--   the menu open, tp = {x, y} while the last touch is younger than K.flashS.
--   Widget {k, x, y, w, h, t, s, c, r, u}: kind hdr|close|tab|lbl|seg|btn|val, rect in page px,
--   text, flags a(ctive) d(im) f(ocus) p(ressed), active colour key, right hint (lbl),
--   subtitle (hdr). The layout is computed here on K.grid; the page only draws.
--   Pushed on change (checked at pushHz and on input) and every keepaliveS (page reloads),
--   only while a player is seated. UI timers use real time: the panel works while paused.
--
-- jbeam options: screenTag, menuTimeoutS (0 = never), keepaliveS, pushHz, clusterViewDefault,
--   centreViewDefault, frameNodes (dash nodes = idRef/idX/idY of the touch trigger),
--   screenTL / screenW / screenH (page corner, width and height vectors in that frame), pageW, pageH.

local M = {}
M.type = "auxiliary"

local floor, max, min, sqrt = math.floor, math.max, math.min, math.sqrt
local sfmt, concat = string.format, table.concat
local clockhp = os.clockhp or os.clock -- real seconds (UI timers)

local LOG_TAG = "etk854hybrid_cockpitPanel"

-- constants (one table: keeps the upvalue count per function low)
local K = {
  tabs = {"DRIVE", "EV", "CHASSIS", "TRAILER", "DISPLAY"},
  tabIdx = {DRIVE = 1, EV = 2, CHASSIS = 3, TRAILER = 4, DISPLAY = 5},
  centreOk = {ENERGY = true, MAP = true, TRAILER = true},
  -- cluster selections in cycle order (also the set of valid names)
  clusterNext = {AUTO = "HYBRID", HYBRID = "EFFICIENCY", EFFICIENCY = "SPORT", SPORT = "MINIMAL", MINIMAL = "AUTO"},
  -- cluster view AUTO per Driving Experience
  autoCluster = {COMFORT = "HYBRID", ECO_PRO = "EFFICIENCY", SPORT = "SPORT"},
  lcLabel = {IDLE = "LAUNCH", ARMED = "LC ARMED", READY = "LC READY", ACTIVE = "LC GO"}, -- as the Actions app
  rgHint = {OFF = "SAILING", HIGH = "ONE-PEDAL"},
  -- the 8HP shift logic is a module, not a controller; vehicleController may load it under either key
  shiftKeys = {"controller/vehicleController/shiftLogic/etk854hybrid_8hp", "controller/shiftLogic-etk854hybrid_8hp"},
  gaugeName = "gauges/customModules/etk854hybrid_gaugeData",
  toastCat = "vehicle.etk854hybrid.display",
  flashS = 0.35, -- pressed flash / touch ring lifetime
  uiDtMax = 0.25, -- largest real-time step per frame (hitches, long pauses)
  touchMargin = 0.02, -- accepted overshoot of a touch beyond the page edge
  hitTol = 2, -- px around interactive widgets; the slot generator keeps a copy (change both)
  -- Fixed slot grid of the control panel (page px). Every interactive widget sits on one slot,
  -- every slot has its own trigger box over the glass (etk854hybrid_gauges.jbeam), so the hover
  -- highlight lights exactly the button under the cursor:
  --   close  hyb_cdClose    closeX, closeY, closeW x closeH
  --   tab i  hyb_cdTab<i>   tabX, tabY + (i-1)*tabPitch, tabW x tabH (i = 1..tabN)
  --   tile   hyb_cdB<r><c>  tileX + (c-1)*colPitch, tileY + (r-1)*rowPitch, tileW x tileH
  --   label of row r (no slot): tileX, lblY + (r-1)*rowPitch, lblW x lblH
  -- Visible rows pack top-down, a row with n options uses tiles 1..n, a val spans tiles; unused
  -- slots hit nothing. The 10 px tab gap is required: the glass is rolled and yawed, so with less
  -- gap a tab's axis-aligned box reaches into the neighbour's hit zone.
  -- tools/etk854hybrid_cockpit_slots.py generates the trigger boxes from this table (plain
  -- "name = integer" pairs only); --check compares table, its copy and the jbeam rows.
  grid = {
    closeX = 432, closeY = 4, closeW = 72, closeH = 22,
    tabX = 8, tabY = 36, tabW = 110, tabH = 33, tabPitch = 43, tabN = 5,
    tileX = 132, tileY = 53, tileW = 71, tileH = 30, colPitch = 75, rowPitch = 53, cols = 5, rows = 4,
    lblY = 38, lblW = 372, lblH = 14
  },
  -- segment rows: t = text (one 71 px tile), a = setter argument = value that lights it,
  -- c = active colour key (one for all or per segment)
  SEG = {
    de = {t = {"COMFORT", "ECO PRO", "SPORT"}, a = {"COMFORT", "ECO_PRO", "SPORT"}, c = "blue"},
    rg = {t = {"OFF", "LOW", "MED", "HIGH"}, a = {"OFF", "LOW", "MED", "HIGH"}, c = "green"},
    sp = {t = {"AUTO", "ECO", "COMF", "SPORT", "MAN"}, a = {"AUTO", "ECO", "COMFORT", "SPORT", "MANUAL"}, c = "blue"},
    ed = {t = {"AUTO EV", "MAX EV", "BATTERY"}, a = {"AUTO_EDRIVE", "MAX_EDRIVE", "BATTERY_CONTROL"}, c = "blue"},
    dsc = {t = {"DSC", "DTC", "OFF"}, a = {"dsc", "dtc", "off"}, c = {"blue", "red", "amber"}},
    awd = {t = {"AWD", "2WD"}, a = {"AWD", "2WD"}, c = "blue"},
    -- "ON - DSC" = Actions app "ON NO DSC" (too wide for a tile)
    ras = {t = {"OFF", "ON + DSC", "ON - DSC"}, a = {"OFF", "DSC", "NODSC"}, c = {"amber", "blue", "lblue"}},
    trl = {t = {"AUTO", "ON", "OFF"}, a = {"AUTO", "ON", "OFF"}, c = "amber"},
    air = {t = {"ON", "OFF"}, a = {"ON", "OFF"}, c = {"green", "amber"}},
    cv = {t = {"ENERGY", "MAP", "TRAILER"}, a = {"ENERGY", "MAP", "TRAILER"}, c = "blue"},
    kv = {t = {"AUTO", "HYBRID", "EFFIC.", "SPORT", "MINIMAL"}, a = {"AUTO", "HYBRID", "EFFICIENCY", "SPORT", "MINIMAL"}, c = "blue"}
  },
  -- IN keys forming the layout signature: common + per tab
  sigCommon = {"tab", "level", "focus", "flash", "view", "csel", "ceff", "tav"},
  sigTab = {
    DRIVE = {"de", "rg", "sp", "spSel", "spHint", "lc", "lcTxt", "lcBtn", "lcBtnS", "lcValS"},
    EV = {"ed", "soc", "tgt", "bc"},
    CHASSIS = {"dm", "dsc", "awd", "rasOn", "ras"},
    TRAILER = {"tpin", "thint", "air", "airOn", "airCol", "airHint"},
    DISPLAY = {"gd", "trip"}
  },
  sig = {} -- per tab: sigCommon + sigTab[tab], filled below
}
for op, d in pairs(K.SEG) do
  d.id = {}
  for j = 1, #d.a do
    d.id[j] = op .. "." .. d.a[j]
  end
end
for tab, list in pairs(K.sigTab) do
  local s = {}
  for j = 1, #K.sigCommon do
    s[#s + 1] = K.sigCommon[j]
  end
  for j = 1, #list do
    s[#s + 1] = list[j]
  end
  K.sig[tab] = s
end

-- jbeam options (defaults)
local CFG = {
  tag = "@etk800_screen",
  menuTimeoutS = 30,
  keepaliveS = 1.0,
  interval = 1 / 15,
  frameNodes = {"dsh2", "dsh2l", "dsh1"},
  TL = {x = 0.11499, y = 0.17887, z = 0.01332},
  W = {x = -0.18240, y = -0.00911, z = 0.02417},
  H = {x = 0.00238, y = -0.08026, z = 0.00452},
  pageW = 512,
  pageH = 256
}

-- UI state (never vehicle modes)
local ST = {
  centreSel = "ENERGY", clusterSel = "AUTO", -- driver selections
  view = "ENERGY", clusterView = "HYBRID", trailerAvail = false, -- effective
  menu = false, tab = 1, level = "tabs", focusId = nil, focusIdx = 1,
  clock = 0, idle = 0, realT = nil, -- clock = UI time (real seconds)
  flashId = nil, flashT = -1e9,
  touchX = 0, touchY = 0, touchT = -1e9,
  pushT = 0, keepT = 0, lastJson = nil, pushSig = nil, layoutSig = nil,
  dirty = false, -- input since the last push check: check now
  pushCount = 0, pushBytes = 0, errN = 0,
  cid = nil, -- {ref, x, y} cids of the frame nodes, nil = touch disabled
  w = {}, -- widget list of the open menu (sent to the page)
  act = {}, -- widget index -> {id, op, arg} for every interactive widget
  nav = {} -- widget indices of the tab content in reading order (cursor)
}
local IN = {} -- inputs of the current layout (strings / numbers only)
local SIG = {} -- reused signature buffer
local P = {v = 1} -- reused payload
local B = {row = 0} -- row counter while building
local GATHER, BUILD, OPS = {}, {}, {}

-- ------------------------------------------------------------------ helpers
local U = require("controller/etk854hybrid/etk854hybrid_util")
local str = U.str -- non-empty string, else nil

local function up(v)
  return type(v) == "string" and v:upper() or ""
end

-- finite number or nil; electrics flags arrive as booleans or 1/0
local function numv(v)
  if v == true then
    return 1
  elseif v == false then
    return 0
  end
  v = tonumber(v)
  if v and v == v and v > -math.huge and v < math.huge then
    return v
  end
  return nil
end

local function flag(v)
  local n = numv(v)
  return n ~= nil and n > 0.5
end

local function flags(on, dim)
  return (on and "a" or "") .. (dim and "d" or "")
end

local function toast(txt)
  if guihooks and guihooks.message then
    guihooks.message(txt, 2, K.toastCat)
  end
end

local function errLog(where, err)
  ST.errN = ST.errN + 1
  if ST.errN <= 5 then
    log("E", LOG_TAG, where .. ": " .. tostring(err))
  end
end

-- name.method(...) of another controller, looked up now, under pcall; true when it ran
local function call(name, method, ...)
  local c = controller.getController(name)
  local f = c and c[method]
  if type(f) ~= "function" then
    return false
  end
  local ok, err = pcall(f, ...)
  if not ok then
    log("W", LOG_TAG, sfmt("%s.%s failed: %s", name, method, tostring(err)))
  end
  return ok
end

local function shiftModule()
  local pl = package.loaded
  local m = pl[K.shiftKeys[1]] or pl[K.shiftKeys[2]]
  if type(m) == "table" and type(m.setProgram) == "function" and type(m.getProgram) == "function" then
    return m
  end
  return nil
end

local function screenOn()
  local ign = numv(electrics.values.ignitionLevel)
  return ign == nil or ign >= 1
end

-- ------------------------------------------------------------------- views
local function effCluster(sel)
  if sel ~= "AUTO" then
    return sel
  end
  return K.autoCluster[electrics.values.etk854hybrid_deMode] or "HYBRID"
end

local function publish()
  local ev = electrics.values
  ST.trailerAvail = type(ev.etk854hybrid_tsaState) == "string"
  ST.view = (ST.centreSel == "TRAILER" and not ST.trailerAvail) and "ENERGY" or ST.centreSel
  ST.clusterView = effCluster(ST.clusterSel)
  ev.etk854hybrid_cdView = ST.view
  ev.etk854hybrid_cdMenu = ST.menu and 1 or 0
  ev.etk854hybrid_cdTab = K.tabs[ST.tab]
  ev.etk854hybrid_clusterViewSel = ST.clusterSel
  ev.etk854hybrid_clusterView = ST.clusterView
end

-- Returns the selection in force (unknown names change nothing); restarts the menu timeout.
local function setCentreView(name)
  local n = type(name) == "string" and name:upper() or nil
  if not (n and K.centreOk[n]) then
    return ST.centreSel
  end
  ST.centreSel = n
  if ST.menu then
    ST.idle = 0
  end
  publish()
  toast("Centre display: " .. n .. (ST.view ~= n and " (not available)" or ""))
  return n
end

-- ENERGY -> MAP -> TRAILER (while available) -> ENERGY, from the effective view
local function cycleCentreView()
  publish()
  local cur = ST.view
  local nxt = "ENERGY"
  if cur == "ENERGY" then
    nxt = "MAP"
  elseif cur == "MAP" and ST.trailerAvail then
    nxt = "TRAILER"
  end
  return setCentreView(nxt)
end

local function setClusterView(name)
  local n = type(name) == "string" and name:upper() or nil
  if not (n and K.clusterNext[n]) then
    return ST.clusterSel
  end
  ST.clusterSel = n
  if ST.menu then
    ST.idle = 0
  end
  publish()
  toast("Instrument cluster: " .. n .. (n == "AUTO" and (" (" .. ST.clusterView .. ")") or ""))
  return n
end

local function cycleClusterView()
  return setClusterView(K.clusterNext[ST.clusterSel] or "AUTO")
end

-- ------------------------------------------------------- setters per action
OPS.de = function(a) call("hybridControl", "setDeMode", a) end
OPS.rg = function(a) call("hybridControl", "setRegenLevel", a) end
OPS.ed = function(a) call("hybridControl", "setEdriveMode", a) end
OPS.soc = function(a) call("hybridControl", "adjustBatteryTargetSoc", a) end
OPS.lc = function() call("hybridControl", "armLaunchControl") end
OPS.sp = function(a)
  local m = shiftModule()
  if m then
    local ok, err = pcall(m.setProgram, a)
    if not ok then
      log("W", LOG_TAG, "8HP setProgram failed: " .. tostring(err))
    end
  end
end
OPS.dsc = function(a) call("driveModes", "setDriveMode", a) end
OPS.awd = function(a) call("etk854hybrid_awdMode", "setMode", a) end
OPS.ras = function(a) call("etk854hybrid_rearSteer", "setMode", a) end
-- AUTO = nil (coupler detection), ON / OFF = pin; self-leveling direct without adaptiveSusp
OPS.trl = function(a)
  local b = nil
  if a == "ON" then
    b = true
  elseif a == "OFF" then
    b = false
  end
  if not call("etk854hybrid_adaptiveSusp", "setTrailerMode", b) then
    call("etk854hybrid_selfLeveling", "setTrailerMode", b)
  end
end
OPS.air = function(a) call("etk854hybrid_airCompressor", "setEnabled", a == "ON") end
OPS.cv = function(a) setCentreView(a) end
OPS.kv = function(a) setClusterView(a) end
OPS.trip = function() call(K.gaugeName, "resetTripComputer") end

-- ------------------------------------------------- layout inputs per tab
-- Every key of K.sigTab[tab] must be set to a string or a number.
GATHER.DRIVE = function(I, ev)
  local de = up(ev.etk854hybrid_deMode)
  I.de = de
  I.rg = up(ev.etk854hybrid_regenLevel)
  I.sp, I.spSel, I.spHint = 0, "AUTO", ""
  local m = shiftModule()
  if m then
    I.sp = 1
    local ok, _, prog, ovr = pcall(m.getProgram)
    if ok then
      -- lever M<n> -> MAN, else the override, else AUTO
      local o = up(ovr)
      I.spSel = (prog == "MANUAL" and "MANUAL") or (o ~= "" and o) or "AUTO"
    end
    I.spHint = (up(ev.etk854hybrid_shiftProgram):gsub("_", " "))
  end
  local ls = str(ev.etk854hybrid_launchState)
  if ls then
    ls = ls:upper()
    -- PRELOAD (state ARMED) and the brake-throttle GUARD only show in the phase
    local ph = up(ev.etk854hybrid_launchPhase)
    local txt = K.lcLabel[ls] or ("LC " .. ls)
    if ph == "PRELOAD" then
      txt = "LC PRELOAD"
    elseif ph == "GUARD" and ls == "IDLE" then
      txt = "LC GUARD"
    end
    local g0 = type(ev.gear) == "string" and ev.gear:sub(1, 1) or ""
    local gearOk = g0 == "S" or g0 == "M" or (g0 == "D" and de == "SPORT")
    local idleDim = ls == "IDLE" and not gearOk
    -- disarm only in ARMED / READY; ACTIVE ignores presses (ARM, dimmed); outside
    -- S/M/D-SPORT dimmed but clickable (the VCU answers with a toast)
    local armed = ls == "ARMED" or ls == "READY"
    I.lc, I.lcTxt = 1, txt
    I.lcBtn = armed and "DISARM" or "ARM"
    I.lcBtnS = flags(armed, idleDim or ls == "ACTIVE")
    I.lcValS = flags(ls ~= "IDLE" or ph == "GUARD", idleDim)
  else
    I.lc, I.lcTxt, I.lcBtn, I.lcBtnS, I.lcValS = 0, "", "", "", ""
  end
end

GATHER.EV = function(I, ev)
  I.ed = up(ev.etk854hybrid_edriveMode)
  local soc = numv(ev.etk854hybrid_batterySoc) -- percent
  if soc and soc <= 1.5 then
    soc = soc * 100
  end
  I.soc = soc and sfmt("SOC %d %%", floor(soc + 0.5)) or "SOC -- %"
  local tgt = numv(ev.etk854hybrid_targetSoc) -- 0..1
  if tgt and tgt > 1.5 then
    tgt = tgt / 100
  end
  I.tgt = tgt and sfmt("%d %%", floor(tgt * 100 + 0.5)) or "-- %"
  I.bc = I.ed == "BATTERY_CONTROL" and 1 or 0
end

GATHER.CHASSIS = function(I, ev)
  I.dm = controller.getController("driveModes") and 1 or 0
  I.dsc = (str(ev.etk854hybrid_dscMode) or "DSC"):lower()
  I.awd = up(ev.etk854hybrid_awdMode)
  local rs = ev.etk854hybrid_rasSel
  I.rasOn = type(rs) == "string" and 1 or 0
  I.ras = up(rs)
end

GATHER.TRAILER = function(I, ev)
  -- pin of the self-leveling: nil = AUTO, true = ON, false = OFF
  local pin
  local lev = controller.getController("etk854hybrid_selfLeveling")
  if lev and type(lev.getLevelInfo) == "function" then
    local ok, info = pcall(lev.getLevelInfo)
    if ok and type(info) == "table" then
      local mt = info.manualTrailer
      pin = (mt == nil and "AUTO") or (mt and "ON") or "OFF"
    end
  end
  local on = flag(ev.etk854hybrid_trailerMode)
  I.tpin = pin or (on and "ON" or "OFF")
  local lvl = str(ev.etk854hybrid_levelState)
  I.thint = (on and "ACTIVE" or "INACTIVE") .. (lvl and (" / " .. lvl:upper()) or "")
  -- compressor: hidden without a receiver ('NO TANK')
  local as = str(ev.etk854hybrid_airState)
  if as and as ~= "NO TANK" then
    as = as:upper()
    local bar = numv(ev.etk854hybrid_airPressureBar)
    I.air = as
    I.airOn = flag(ev.etk854hybrid_airEnabled) and 1 or 0
    -- green idle, light blue pumping, amber below the spring-brake release pressure
    I.airCol = (as == "LOW" and "amber") or (flag(ev.etk854hybrid_airCompressor) and "lblue") or "green"
    I.airHint = (bar and sfmt("%.1f BAR", max(0, bar)) or "-- BAR") .. " / " .. as
  else
    I.air, I.airOn, I.airCol, I.airHint = "", 0, "", ""
  end
end

GATHER.DISPLAY = function(I, ev)
  I.gd = controller.getController(K.gaugeName) and 1 or 0
  local tk = numv(ev.etk854hybrid_tripKm)
  I.trip = tk and sfmt("%.1f KM", max(0, tk)) or ""
end

-- Fills IN for the current tab, returns the layout signature.
local function gather()
  local I = IN
  local tabName = K.tabs[ST.tab]
  I.tab = tabName
  I.level = ST.level
  I.focus = ST.focusId or ""
  I.flash = (ST.flashId and ST.clock - ST.flashT < K.flashS) and ST.flashId or ""
  I.view, I.csel, I.ceff = ST.view, ST.clusterSel, ST.clusterView
  I.tav = ST.trailerAvail and 1 or 0
  GATHER[tabName](I, electrics.values)
  local keys = K.sig[tabName]
  for j = 1, #keys do
    SIG[j] = I[keys[j]]
  end
  return concat(SIG, "|", 1, #keys)
end

-- ------------------------------------------------------------------ layout
local function addW(k, x, y, w, h, t, s, c)
  local wl = ST.w
  local n = #wl + 1
  wl[n] = {k = k, x = x, y = y, w = w, h = h, t = t, s = s or "", c = c}
  return n
end

-- interactive widget; nav = reachable with the controller cursor
local function addI(k, x, y, w, h, t, s, c, id, op, arg, nav)
  local i = addW(k, x, y, w, h, t, s, c)
  ST.act[i] = {id = id, op = op, arg = arg}
  if nav then
    ST.nav[#ST.nav + 1] = i
  end
  return i
end

local function tileX(c)
  return K.grid.tileX + (c - 1) * K.grid.colPitch
end

-- label (+ right hint) of the next grid row; returns its tile y, nil when the grid is full
local function rowLabel(t, r)
  local G = K.grid
  if B.row >= G.rows then
    return nil
  end
  B.row = B.row + 1
  local i = addW("lbl", G.tileX, G.lblY + (B.row - 1) * G.rowPitch, G.lblW, G.lblH, t, "")
  if r and r ~= "" then
    ST.w[i].r = r
  end
  return G.tileY + (B.row - 1) * G.rowPitch
end

local function tileBtn(c, y, t, s, col, id, op, arg)
  return addI("btn", tileX(c), y, K.grid.tileW, K.grid.tileH, t, s, col, id, op, arg, true)
end

-- value field over tile columns c1..c2 (not clickable)
local function valSpan(c1, c2, y, t, s, col)
  local x = tileX(c1)
  return addW("val", x, y, tileX(c2) + K.grid.tileW - x, K.grid.tileH, t, s, col)
end

-- n segments of K.SEG[op] on tiles 1..n; cur lights a segment, colOv overrides its colour
local function segRow(label, hint, op, cur, n, dim, colOv)
  local d, G = K.SEG[op], K.grid
  n = min(n or #d.t, G.cols)
  local y = rowLabel(label, hint)
  if not y then
    return
  end
  for j = 1, n do
    local on = d.a[j] == cur
    local c = (on and colOv) or (type(d.c) == "table" and d.c[j]) or d.c
    addI("seg", tileX(j), y, G.tileW, G.tileH, d.t[j], flags(on, dim), c, d.id[j], op, d.a[j], true)
  end
end

BUILD.DRIVE = function(I)
  segRow("DRIVING MODE", nil, "de", I.de)
  segRow("RECUPERATION", K.rgHint[I.rg], "rg", I.rg)
  if I.sp == 1 then
    segRow("GEARBOX PROGRAM", I.spHint, "sp", I.spSel)
  end
  local y = I.lc == 1 and rowLabel("LAUNCH CONTROL")
  if y then
    tileBtn(1, y, I.lcBtn, I.lcBtnS, "amber", "lc", "lc", nil)
    valSpan(2, K.grid.cols, y, I.lcTxt, I.lcValS, "amber")
  end
end

BUILD.EV = function(I)
  segRow("EV MODE", I.soc, "ed", I.ed)
  local y = rowLabel("BATTERY TARGET", "IN BATTERY CONTROL")
  if y then
    local bc, n = I.bc == 1, K.grid.cols
    -- effective in BATTERY CONTROL only: dimmed otherwise, still clickable
    tileBtn(1, y, "-5", flags(false, not bc), "blue", "soc.-5", "soc", -5)
    valSpan(2, n - 1, y, I.tgt, flags(bc, not bc), "blue")
    tileBtn(n, y, "+5", flags(false, not bc), "blue", "soc.+5", "soc", 5)
  end
end

BUILD.CHASSIS = function(I)
  if I.dm == 1 then
    segRow("STABILITY CONTROL", nil, "dsc", I.dsc)
  end
  if I.awd ~= "" then
    segRow("XALLRAD", nil, "awd", I.awd)
  end
  if I.rasOn == 1 then
    -- DSC OFF disables the rear steering: row dimmed, selection still settable
    local off = I.dsc == "off"
    segRow("REAR-AXLE STEERING", off and "INACTIVE WHILE DSC OFF" or nil, "ras", I.ras, nil, off)
  end
end

BUILD.TRAILER = function(I)
  segRow("TRAILER MODE", I.thint, "trl", I.tpin)
  if I.air ~= "" then
    local on = I.airOn == 1
    segRow("AIR COMPRESSOR", I.airHint, "air", on and "ON" or "OFF", nil, false, on and I.airCol or nil)
  end
end

BUILD.DISPLAY = function(I)
  segRow("CENTRE DISPLAY", nil, "cv", I.view, I.tav == 1 and 3 or 2)
  segRow("INSTRUMENT CLUSTER", I.csel == "AUTO" and ("AUTO: " .. I.ceff) or nil, "kv", I.csel)
  local y = I.gd == 1 and rowLabel("TRIP COMPUTER")
  if y then
    tileBtn(1, y, "RESET", "", "blue", "trip", "trip", nil)
    if I.trip ~= "" then
      valSpan(2, K.grid.cols, y, I.trip, "", "blue")
    end
  end
end

-- cursor position in ST.nav: the focused id, else the old index clamped
local function focusPos()
  local nav, act = ST.nav, ST.act
  for j = 1, #nav do
    if act[nav[j]].id == ST.focusId then
      return j
    end
  end
  return max(1, min(#nav, ST.focusIdx or 1))
end

local function build()
  ST.w, ST.act, ST.nav = {}, {}, {}
  B.row = 0
  local tabName, G = K.tabs[ST.tab], K.grid
  local h = addW("hdr", 0, 0, 512, 30, "SETTINGS")
  ST.w[h].u = tabName
  addI("close", G.closeX, G.closeY, G.closeW, G.closeH, "CLOSE", "", "blue", "close", "close", nil, false)
  for j = 1, min(#K.tabs, G.tabN) do
    addI("tab", G.tabX, G.tabY + (j - 1) * G.tabPitch, G.tabW, G.tabH, K.tabs[j], j == ST.tab and "a" or "", "blue", "tab." .. K.tabs[j], "tab", j, false)
  end
  BUILD[tabName](IN)
  -- a focused widget that vanished (row hidden meanwhile): nearest one
  if ST.level == "items" then
    if #ST.nav == 0 then
      ST.level, ST.focusId = "tabs", nil
    else
      local p = focusPos()
      ST.focusIdx, ST.focusId = p, ST.act[ST.nav[p]].id
    end
  end
  local fl = ST.flashId and ST.clock - ST.flashT < K.flashS
  for i, r in pairs(ST.act) do
    local w = ST.w[i]
    if r.op == "tab" then
      if ST.level == "tabs" and r.arg == ST.tab then
        w.s = w.s .. "f"
      end
    elseif ST.level == "items" and r.id == ST.focusId then
      w.s = w.s .. "f"
    end
    if fl and r.id == ST.flashId then
      w.s = w.s .. "p"
    end
  end
end

-- Rebuilds the widget list when its inputs changed (menu open only).
local function ensureLayout()
  local sig = gather()
  if sig ~= ST.layoutSig then
    build()
    ST.layoutSig = gather() -- build may have moved a lost focus
  end
end

-- -------------------------------------------------------------------- menu
local function closeMenu()
  ST.menu = false
  ST.level, ST.focusId, ST.focusIdx = "tabs", nil, 1
  ST.idle = 0
  publish()
  return true
end

local function openMenu(tabId)
  if not screenOn() then
    return false
  end
  local i = type(tabId) == "string" and K.tabIdx[tabId:upper()] or nil
  if i then
    ST.tab = i
  end
  ST.menu = true
  ST.level, ST.focusId, ST.focusIdx = "tabs", nil, 1
  ST.idle = 0
  publish()
  return true
end

-- false when input must be ignored (screen off closes the menu)
local function inputOk()
  if screenOn() then
    ST.idle = 0
    return true
  end
  if ST.menu then
    closeMenu()
  end
  return false
end

-- level items with the cursor on the first widget of the tab
local function enterItems()
  ensureLayout()
  local first = ST.nav[1]
  if first then
    ST.level, ST.focusId, ST.focusIdx = "items", ST.act[first].id, 1
  else
    ST.level, ST.focusId = "tabs", nil
  end
end

-- runs the action of an interactive content widget, focuses and flashes it
local function activate(i)
  local r = ST.act[i]
  if not r or r.op == "tab" or r.op == "close" then
    return false
  end
  ST.level, ST.focusId = "items", r.id
  for j = 1, #ST.nav do
    if ST.nav[j] == i then
      ST.focusIdx = j
      break
    end
  end
  ST.flashId, ST.flashT = r.id, ST.clock
  local f = OPS[r.op]
  if f then
    f(r.arg)
  end
  return true
end

local function hitTest(x, y)
  local wl, act, tol = ST.w, ST.act, K.hitTol
  for i = #wl, 1, -1 do
    local w = wl[i]
    if act[i] and x >= w.x - tol and x <= w.x + w.w + tol and y >= w.y - tol and y <= w.y + w.h + tol then
      return i
    end
  end
  return nil
end

local function touchAt(x, y)
  x, y = numv(x), numv(y)
  if not (x and y) or not inputOk() then
    return false
  end
  ST.touchX, ST.touchY, ST.touchT = x, y, ST.clock
  if not ST.menu then
    return openMenu(nil)
  end
  ensureLayout()
  local i = hitTest(x, y)
  if not i then
    return false
  end
  local r = ST.act[i]
  if r.op == "close" then
    return closeMenu()
  elseif r.op == "tab" then
    ST.tab = r.arg
    publish()
    enterItems()
    return true
  end
  return activate(i)
end

local function menuKey()
  if ST.menu then
    return closeMenu()
  end
  return inputOk() and openMenu(nil)
end

-- prev / next: tab (level tabs) or cursor (level items), no wrap
local function step(d)
  if not inputOk() then
    return false
  end
  if not ST.menu then
    return openMenu(nil)
  end
  if ST.level == "tabs" then
    ST.tab = max(1, min(#K.tabs, ST.tab + d))
    publish()
    return true
  end
  ensureLayout()
  local nav = ST.nav
  if #nav == 0 then
    ST.level, ST.focusId = "tabs", nil
    return true
  end
  local p = max(1, min(#nav, focusPos() + d))
  ST.focusIdx, ST.focusId = p, ST.act[nav[p]].id
  return true
end

local function keyPrev()
  return step(-1)
end

local function keyNext()
  return step(1)
end

local function press()
  if not inputOk() then
    return false
  end
  if not ST.menu then
    return openMenu(nil)
  end
  if ST.level == "tabs" then
    enterItems()
    return true
  end
  ensureLayout()
  local i = ST.nav[focusPos()]
  return i ~= nil and activate(i)
end

local function back()
  if not inputOk() or not ST.menu then
    return false
  end
  if ST.level == "items" then
    ST.level, ST.focusId = "tabs", nil
    return true
  end
  return closeMenu()
end

-- ------------------------------------------------------------------- touch
local function cross3(ax, ay, az, bx, by, bz)
  return ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx
end

local function dot3(ax, ay, az, bx, by, bz)
  return ax * bx + ay * by + az * bz
end

local function norm3(x, y, z)
  local l = sqrt(x * x + y * y + z * z)
  if not (l > 1e-9) then
    return nil
  end
  return x / l, y / l, z / l
end

-- vehicle-relative node position, world axes (the frame of the touch ray)
local function nodeR(cid)
  local r = obj:getNodePosition(cid)
  return r.x, r.y, r.z
end

-- vehicle-relative ray -> page px, or nil. Frame as core/vehicle/triggerLabelPlacement.lua:
-- nx = norm(X - R), nz = norm(cross(Y - R, nx)), up = cross(nx, nz); page corner
-- O = R + TL.x*nx + TL.y*up + TL.z*nz, axes U / V from W / H likewise;
-- pos + t*dir = O + a*U + b*V solved with Cramer's rule.
local function rayToPage(px, py, pz, dx, dy, dz)
  local c = ST.cid
  local rx, ry, rz = nodeR(c[1])
  local xx, xy, xz = nodeR(c[2])
  local yx, yy, yz = nodeR(c[3])
  local ax, ay, az = norm3(xx - rx, xy - ry, xz - rz)
  if not ax then
    return nil
  end
  local zx, zy, zz = norm3(cross3(yx - rx, yy - ry, yz - rz, ax, ay, az))
  if not zx then
    return nil
  end
  local ux, uy, uz = cross3(ax, ay, az, zx, zy, zz)
  local T, W, H = CFG.TL, CFG.W, CFG.H
  local ox, oy, oz = rx + T.x * ax + T.y * ux + T.z * zx, ry + T.x * ay + T.y * uy + T.z * zy, rz + T.x * az + T.y * uz + T.z * zz
  local Ux, Uy, Uz = W.x * ax + W.y * ux + W.z * zx, W.x * ay + W.y * uy + W.z * zy, W.x * az + W.y * uz + W.z * zz
  local Vx, Vy, Vz = H.x * ax + H.y * ux + H.z * zx, H.x * ay + H.y * uy + H.z * zy, H.x * az + H.y * uz + H.z * zz
  -- a*U + b*V + t*N = Q with N = -dir, Q = pos - O
  local nx, ny, nz = -dx, -dy, -dz
  local qx, qy, qz = px - ox, py - oy, pz - oz
  local dl = sqrt(dx * dx + dy * dy + dz * dz)
  local vnx, vny, vnz = cross3(Vx, Vy, Vz, nx, ny, nz)
  local det = dot3(Ux, Uy, Uz, vnx, vny, vnz)
  local lu = sqrt(Ux * Ux + Uy * Uy + Uz * Uz)
  local lv = sqrt(Vx * Vx + Vy * Vy + Vz * Vz)
  if not (dl > 1e-9) or not (math.abs(det) > 1e-6 * dl * lu * lv) then
    return nil -- no direction or ray (almost) parallel to the glass
  end
  local a = dot3(qx, qy, qz, vnx, vny, vnz) / det
  local b = dot3(Ux, Uy, Uz, cross3(qx, qy, qz, nx, ny, nz)) / det
  local t = dot3(Ux, Uy, Uz, cross3(Vx, Vy, Vz, qx, qy, qz)) / det
  local m = K.touchMargin
  if not (t > 0) or not (a >= -m and a <= 1 + m and b >= -m and b <= 1 + m) then
    return nil
  end
  return a * CFG.pageW, b * CFG.pageH
end

local function touchRay(px, py, pz, dx, dy, dz)
  px, py, pz, dx, dy, dz = numv(px), numv(py), numv(pz), numv(dx), numv(dy), numv(dz)
  if not (px and py and pz and dx and dy and dz) or not ST.cid then
    return false
  end
  local ok, x, y = pcall(rayToPage, px, py, pz, dx, dy, dz)
  if not ok then
    errLog("touchRay", x)
    return false
  end
  if not x then
    return false
  end
  return touchAt(x, y)
end

-- -------------------------------------------------------------- page push
local function push(json)
  obj:queueWebViewJS(CFG.tag, "etk854hybridPanel(" .. json .. ")")
  ST.lastJson = json
  ST.keepT = 0
  ST.pushCount = ST.pushCount + 1
  ST.pushBytes = #json
end

local function tick()
  local tp = ST.clock - ST.touchT < K.flashS
  local sig
  if ST.menu then
    ensureLayout()
    sig = ST.layoutSig
  else
    sig = "C|" .. ST.view
  end
  if tp then
    sig = sig .. sfmt("|T%d,%d", floor(ST.touchX + 0.5), floor(ST.touchY + 0.5))
  end
  if sig ~= ST.pushSig or not ST.lastJson then
    ST.pushSig = sig
    local p = P
    p.view = ST.view
    p.menu = ST.menu and 1 or 0
    p.tab = ST.menu and K.tabs[ST.tab] or nil
    p.w = ST.menu and ST.w or nil
    p.tp = tp and {floor(ST.touchX + 0.5), floor(ST.touchY + 0.5)} or nil
    local json = jsonEncode(p)
    if json ~= ST.lastJson then
      push(json)
      return
    end
  end
  if ST.keepT >= CFG.keepaliveS then
    push(ST.lastJson)
  end
end

-- real seconds since the last frame (dtSim is 0 while paused), clamped; dtSim as fallback
local function uiStep(dtSim)
  local now = clockhp()
  if not (type(now) == "number" and now == now) then
    return max(0, min(K.uiDtMax, dtSim))
  end
  local last = ST.realT
  ST.realT = now
  if not last then
    return max(0, min(K.uiDtMax, dtSim))
  end
  return max(0, min(K.uiDtMax, now - last))
end

local function updateGFX(dtSim)
  local dt = uiStep(numv(dtSim) or 0)
  ST.clock = ST.clock + dt
  ST.keepT = ST.keepT + dt
  publish()
  if ST.menu then
    if not screenOn() then
      closeMenu()
    elseif CFG.menuTimeoutS > 0 then
      ST.idle = ST.idle + dt
      if ST.idle >= CFG.menuTimeoutS then
        closeMenu()
      end
    end
  end
  ST.pushT = ST.pushT + dt
  if ST.pushT >= CFG.interval then
    -- keep the remainder (exact average rate), no catch-up burst after a hitch
    ST.pushT = ST.pushT - CFG.interval
    if ST.pushT > CFG.interval then
      ST.pushT = 0
    end
  elseif not ST.dirty then
    return
  end
  ST.dirty = false
  if not playerInfo.anyPlayerSeated then
    ST.lastJson, ST.pushSig = nil, nil -- push at once when seated again
    return
  end
  local ok, err = pcall(tick)
  if not ok then
    errLog("updateGFX", err)
  end
end

-- --------------------------------------------------------------- lifecycle
local function vec(t, d)
  if type(t) == "table" then
    local x, y, z = numv(t.x), numv(t.y), numv(t.z)
    if x and y and z then
      return {x = x, y = y, z = z}
    end
  end
  return d
end

-- cids of the three frame nodes by name; missing -> touch disabled
local function resolveFrame()
  ST.cid = nil
  local want, got = {}, {}
  for j = 1, 3 do
    want[CFG.frameNodes[j]] = j
  end
  local nodes = v and v.data and v.data.nodes
  if type(nodes) == "table" then
    for k, n in pairs(nodes) do
      if type(n) == "table" and type(n.name) == "string" then
        local j = want[n.name]
        if j and got[j] == nil then
          got[j] = n.cid or k
        end
      end
    end
  end
  if got[1] and got[2] and got[3] then
    ST.cid = got
  else
    log("W", LOG_TAG, "frame nodes " .. concat(CFG.frameNodes, "/") .. " not found, touch disabled")
  end
end

-- UI state back to "menu closed"; view selections are kept
local function clearUi()
  ST.menu = false
  ST.level, ST.focusId, ST.focusIdx = "tabs", nil, 1
  ST.idle = 0
  ST.flashId, ST.flashT, ST.touchT = nil, -1e9, -1e9
  ST.pushT, ST.keepT, ST.dirty = 0, 0, true
  ST.lastJson, ST.pushSig, ST.layoutSig = nil, nil, nil
  ST.w, ST.act, ST.nav = {}, {}, {}
end

local function init(jbeamData)
  local jb = type(jbeamData) == "table" and jbeamData or {}
  CFG.tag = str(jb.screenTag) or "@etk800_screen"
  CFG.menuTimeoutS = max(0, numv(jb.menuTimeoutS) or 30)
  local ka = numv(jb.keepaliveS) or 1.0
  CFG.keepaliveS = ka > 0.1 and ka or 1.0
  local hz = numv(jb.pushHz) or 15
  CFG.interval = 1 / max(1, min(60, hz))
  CFG.TL = vec(jb.screenTL, CFG.TL)
  CFG.W = vec(jb.screenW, CFG.W)
  CFG.H = vec(jb.screenH, CFG.H)
  local pw, ph = numv(jb.pageW), numv(jb.pageH)
  CFG.pageW = (pw and pw > 0) and pw or 512
  CFG.pageH = (ph and ph > 0) and ph or 256
  local fn = jb.frameNodes
  if type(fn) == "table" then
    local b = fn[1] ~= nil and 1 or 0 -- 1-based (json) or 0-based list
    if str(fn[b]) and str(fn[b + 1]) and str(fn[b + 2]) then
      CFG.frameNodes = {fn[b], fn[b + 1], fn[b + 2]}
    end
  end
  local cs = up(jb.clusterViewDefault)
  ST.clusterSel = K.clusterNext[cs] and cs or "AUTO"
  local cv = up(jb.centreViewDefault)
  ST.centreSel = K.centreOk[cv] and cv or "ENERGY"
  ST.tab = 1
  clearUi()
  resolveFrame()
  publish()
end

-- R: close the menu, keep the view selections (M.reset keeps controller.lua from re-running init)
local function reset()
  clearUi()
  publish()
end

-- electrics.reset may have cleared our channels
local function resetLastStage()
  publish()
end

local function serialize()
  return {centre = ST.centreSel, cluster = ST.clusterSel}
end

local function deserialize(d)
  if type(d) ~= "table" then
    return
  end
  if type(d.centre) == "string" and K.centreOk[d.centre] then
    ST.centreSel = d.centre
  end
  if type(d.cluster) == "string" and K.clusterNext[d.cluster] then
    ST.clusterSel = d.cluster
  end
  publish()
end

local function getState()
  return {
    view = ST.view, viewSel = ST.centreSel, trailerAvail = ST.trailerAvail,
    menu = ST.menu and 1 or 0, tab = K.tabs[ST.tab], level = ST.level,
    focus = ST.level == "tabs" and ("tab." .. K.tabs[ST.tab]) or ST.focusId,
    clusterSel = ST.clusterSel, clusterView = ST.clusterView,
    widgets = #ST.w, lastPushBytes = ST.pushBytes, pushCount = ST.pushCount,
    touch = ST.cid ~= nil, lastTouch = {x = ST.touchX, y = ST.touchY, t = ST.touchT}
  }
end

-- public entry points never throw; input = true: the next updateGFX checks the page at
-- once (also while paused) instead of waiting for the push slot
local function guard(name, f, input)
  return function(...)
    if input then
      ST.dirty = true
    end
    local ok, r = pcall(f, ...)
    if ok then
      return r
    end
    errLog(name, r)
    return nil
  end
end

M.init = init
M.reset = reset
M.resetLastStage = resetLastStage
M.updateGFX = updateGFX
M.serialize = serialize
M.deserialize = guard("deserialize", deserialize)

M.touchRay = guard("touchRay", touchRay, true)
M.touchAt = guard("touchAt", touchAt, true)
M.menu = guard("menu", menuKey, true)
M.prev = guard("prev", keyPrev, true)
M.next = guard("next", keyNext, true)
M.press = guard("press", press, true)
M.back = guard("back", back, true)
M.openMenu = guard("openMenu", openMenu, true)
M.closeMenu = guard("closeMenu", closeMenu, true)
M.setCentreView = guard("setCentreView", setCentreView, true)
M.cycleCentreView = guard("cycleCentreView", cycleCentreView, true)
M.setClusterView = guard("setClusterView", setClusterView, true)
M.cycleClusterView = guard("cycleClusterView", cycleClusterView, true)
M.getState = guard("getState", getState)

return M
