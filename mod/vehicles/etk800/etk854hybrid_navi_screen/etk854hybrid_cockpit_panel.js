// ETK 854 ActiveHybrid - centre screen cockpit panel (control panel + MAP view)
// Target: BeamNG.drive 0.39.4, web view "@etk800_screen" (512 x 256).
// Loaded from the <head> of etk854hybrid_navi_screen.html after the stub script, which buffers
// etk854hybridPanel() (q.panel) until this file runs; the DOM is built on DOMContentLoaded.
//
// Input: etk854hybridPanel(p) from etk854hybrid_cockpitPanel.lua on change and as ~1 s keepalive:
//   v 1 | view ENERGY|MAP|TRAILER | menu 1 = panel open | tab | tp [x, y] or {x, y} touch point
//   w  widgets in page px {k x y w h t s c r u}, drawn in list order: k hdr|close|tab|lbl|seg|
//      btn|val; s flags a(ctive) d(immed, still clickable) f(ocus) p(ressed); c active colour
//      blue|lblue|green|amber|red|orange (default blue); r right hint of a lbl; u hdr subtitle.
// Dumb renderer: all state lives in Lua (a page reload recovers via the keepalive); input
// reaches Lua through the touch ray and the controller keys - no DOM listeners.
//
// Layers over the energy-flow page (absolute, 512 x 256):
//   #cpMap    MAP view: stock bngNavigator (loaded on first use), fed with the one-shot
//             map.setData (kept) and the 15 Hz map.updateData (only while visible)
//   #cpMenu   control panel, opaque
//   #cpTouch  touch ring at tp
// window.etk854hybridCdView = view of the latest payload (pins the energy page's trailer page).
// No CSS animations / transitions (they freeze the vehicle texture in 0.39.4), hence no
// navigator.css (bootscreen fade).
// Desktop demo: #demo=menu | menu-ev | menu-chassis | menu-trailer | menu-display | map.
(function () {
  "use strict";

  var C = {
    bg: "#05070a",
    text: "#e6ebf2",
    muted: "#8b95a5",
    faint: "#5d6778",
    dark: "#0b0e13",
    rule: "#1c222c",
    edge: "#2a3140",
    blue: "#2b8bff",
    blueLt: "#8cc8ff",
    amber: "#ffab3d",
    orange: "#ff6b2c",
    green: "#2ad99c",
    red: "#ff4f5e"
  };
  var KEY = { blue: C.blue, lblue: C.blueLt, green: C.green, amber: C.amber, red: C.red, orange: C.orange };
  var NS = "http://www.w3.org/2000/svg";
  var FONT = "font-family:\"Etk854Cond\", \"Roboto Condensed\", \"Arial Narrow\", sans-serif;" +
    "text-rendering:geometricPrecision;-webkit-user-select:none;user-select:none;";
  var LAYER = "position:absolute;left:0;top:0;width:512px;height:256px;margin:0;overflow:hidden;display:none;";
  var NAV_SRC = "/ui/ui-vue/src/services/navigator-class.js";
  var INTERACTIVE = { seg: 1, btn: 1, close: 1, tab: 1 };

  // MAP view: vehicle marker (page px), zoom (page px per metre), pitch (deg)
  var MAP_X = 256, MAP_Y = 178, MAP_SCALE = 1.25, MAP_PITCH = 55;

  // all mutable state in two tables (menu / map)
  var S = { p: null, sig: "", ready: false, view: null, menu: false, wsig: [], wel: [], tp: "", demo: null, errs: {} };
  var M = { data: null, fresh: false, g: null, push: false, ign: null, nav: null, load: 0, shown: false, asked: false };
  var $ = {};

  function logErr(where, e) {
    var t = where + ": " + (e && e.message ? e.message : String(e));
    if (S.errs[t]) { return; }
    S.errs[t] = true;
    try {
      if (typeof beamng !== "undefined" && beamng.sendEngineLua) {
        beamng.sendEngineLua("log('E','etk854hybrid.cockpitPanel'," + JSON.stringify(t) + ")");
      }
    } catch (x) { /* never throw from the error path */ }
  }

  // ------------------------------------------- entry points (buffered now)
  function noteView(p) {
    var v = (p && typeof p.view === "string") ? p.view : "ENERGY";
    window.etk854hybridCdView = v;
    S.view = v;
  }
  window.etk854hybridPanel = function (p) {
    if (p && typeof p === "object") { S.p = p; noteView(p); }
  };
  // payload buffered by the head stub while this file was loading
  var q0 = window.__etk854nq;
  if (q0 && q0.panel) { window.etk854hybridPanel(q0.panel); q0.panel = null; }

  // map.setData: road network, once at navigator init, always kept; map.updateData (x, y,
  // rotation, speed, ignitionLevel): forwarded by syncMap() only while the MAP view is visible
  var map = window.map || (window.map = {});
  var stubSet = map.setData, stubUpd = map.updateData;
  map.setData = function (d) {
    if (typeof stubSet === "function") { try { stubSet(d); } catch (e) { logErr("map.setData", e); } }
    if (d && typeof d === "object") { M.data = d; M.fresh = true; syncMap(); }
  };
  map.updateData = function (g) {
    if (typeof stubUpd === "function") { try { stubUpd(g); } catch (e) { logErr("map.updateData", e); } }
    if (!g || typeof g !== "object") { return; }
    M.g = g;
    M.push = true;
    if (typeof g.ignitionLevel === "number") { M.ign = g.ignitionLevel; }
    syncMap();
  };

  // ---------------------------------------------------------------- helpers
  function num(v) {
    var n = typeof v === "number" ? v : (typeof v === "string" && v !== "" ? +v : NaN);
    return isFinite(n) ? n : null;
  }
  function node(parent, tag, a) {
    var el = document.createElementNS(NS, tag);
    if (a) { for (var k in a) { if (a.hasOwnProperty(k)) { el.setAttribute(k, a[k]); } } }
    if (parent) { parent.appendChild(el); }
    return el;
  }
  function div(parent, id, css) {
    var el = document.createElement("div");
    if (id) { el.id = id; }
    el.setAttribute("style", css);
    parent.appendChild(el);
    return el;
  }
  function layerSvg(id, css) {
    var s = node(document.body, "svg", { id: id, width: "512", height: "256", viewBox: "0 0 512 256", style: css });
    return s;
  }
  function text(parent, x, y, s, fs, ls, fill, bold, anchor) {
    var t = node(parent, "text", { x: x.toFixed(1), y: y.toFixed(1), "font-size": fs, "letter-spacing": ls, fill: fill });
    if (bold) { t.setAttribute("font-weight", "700"); }
    if (anchor) { t.setAttribute("text-anchor", anchor); }
    t.textContent = s;
    t.__ls = ls;
    return t;
  }
  function txt(el, v) {
    if (el && el.__t !== v) { el.__t = v; el.textContent = v; }
  }
  function measure(t) {
    try { return t.getComputedTextLength(); } catch (e) { return 0; }
  }
  // squeeze a text into maxW: first tighter letter-spacing, then glyphs
  function fit(t, maxW) {
    var w = measure(t);
    if (w > maxW && t.__ls > 0.3) { t.setAttribute("letter-spacing", "0.3"); w = measure(t); }
    if (w > maxW && maxW > 0) {
      t.setAttribute("textLength", maxW.toFixed(1));
      t.setAttribute("lengthAdjust", "spacingAndGlyphs");
      w = maxW;
    }
    return w;
  }
  function rgb(hex) {
    var h = String(hex).replace("#", "");
    return [parseInt(h.substr(0, 2), 16), parseInt(h.substr(2, 2), 16), parseInt(h.substr(4, 2), 16)];
  }
  // colour a over colour b with weight t
  function mix(a, b, t) {
    var x = rgb(a), y = rgb(b);
    return "rgb(" + Math.round(y[0] + (x[0] - y[0]) * t) + "," + Math.round(y[1] + (x[1] - y[1]) * t) + "," +
      Math.round(y[2] + (x[2] - y[2]) * t) + ")";
  }
  function clock() {
    var d = new Date();
    return ("0" + d.getHours()).slice(-2) + ":" + ("0" + d.getMinutes()).slice(-2);
  }

  // ---------------------------------------------------------------- widgets
  // header like the energy page: title, blue underline, subtitle, rule
  function drawHdr(b, x, y, w, h, t, u) {
    var tt = text(b, x + 10, y + 19.5, t, 13, 1.6, C.text, true);
    var ul = node(b, "rect", { x: x + 10, y: y + 23.5, width: 0, height: 1.6, fill: C.blue });
    var st = text(b, x + 10, y + 19.5, u, 11, 1.4, C.muted, true);
    node(b, "rect", { x: x + 8, y: y + h - 1, width: Math.max(0, w - 16), height: 1, fill: C.rule });
    return function () {
      var tw = fit(tt, Math.max(40, w * 0.4));
      ul.setAttribute("width", tw.toFixed(1));
      st.setAttribute("x", (x + 10 + tw + 14).toFixed(1));
      fit(st, Math.max(30, w - tw - 124));
    };
  }

  // vertical tab: accent bar + tint when active
  function drawTab(b, x, y, w, h, t, act, prs, col) {
    var fill = prs ? mix(col, C.dark, 0.55) : (act ? mix(col, C.dark, 0.2) : C.dark);
    node(b, "rect", { x: x + 0.6, y: y + 0.6, width: Math.max(0, w - 1.2), height: Math.max(0, h - 1.2), rx: 4,
      fill: fill, stroke: act || prs ? mix(col, C.dark, 0.55) : C.rule, "stroke-width": 1.2 });
    if (act) { node(b, "rect", { x: x + 4, y: y + 7, width: 3, height: Math.max(0, h - 14), rx: 1.5, fill: col }); }
    var tt = text(b, x + 15, y + h / 2 + 4.6, t, 13, 1.4, act || prs ? C.text : C.muted, true);
    return function () { fit(tt, w - 22); };
  }

  // segment / button / CLOSE pill: active = tint, coloured outline, bottom bar; pressed = brighter
  function drawKey(b, x, y, w, h, t, act, prs, col, k) {
    var pill = k === "close";
    var fill = C.dark, stroke = pill ? C.muted : C.edge, sw = 1.2, tc = k === "seg" ? C.muted : C.text;
    if (act) { fill = mix(col, C.dark, 0.24); stroke = col; sw = 1.6; tc = C.text; }
    if (prs) { fill = mix(col, C.dark, 0.5); stroke = mix("#ffffff", col, 0.5); sw = 1.6; tc = "#ffffff"; }
    node(b, "rect", { x: x + sw / 2, y: y + sw / 2, width: Math.max(0, w - sw), height: Math.max(0, h - sw),
      rx: pill ? (h - sw) / 2 : 4, fill: fill, stroke: stroke, "stroke-width": sw });
    var lift = 0;
    if (act && !pill && h >= 24) {
      node(b, "rect", { x: x + 8, y: y + h - 5.5, width: Math.max(0, w - 16), height: 2.4, rx: 1.2, fill: prs ? "#ffffff" : col });
      lift = 1;
    }
    var fs = pill ? 11 : 13;
    var tt = text(b, x + w / 2, y + h / 2 + fs * 0.355 - lift, t, fs, pill ? 1.4 : 1, tc, true, "middle");
    return function () { fit(tt, w - (pill ? 16 : 10)); };
  }

  // value field (not clickable)
  function drawVal(b, x, y, w, h, t, act, col, centre) {
    node(b, "rect", { x: x + 0.5, y: y + 0.5, width: Math.max(0, w - 1), height: Math.max(0, h - 1), rx: 4,
      fill: "#080b10", stroke: C.rule, "stroke-width": 1 });
    var tt = text(b, centre ? x + w / 2 : x + 12, y + h / 2 + 5, t, 14, 1, act ? col : C.text, true, centre ? "middle" : null);
    return function () { fit(tt, w - (centre ? 12 : 22)); };
  }

  // row label (muted) + right-aligned hint
  function drawLbl(b, x, y, w, h, t, r, act, col) {
    var base = y + Math.max(10, h - 3);
    var lt = text(b, x, base, t, 11, 1.2, C.muted, false);
    var rt = r ? text(b, x + w, base, r, 11, 0.8, act ? col : C.text, true, "end") : null;
    return function () {
      var lw = fit(lt, w * 0.62);
      if (rt) { fit(rt, Math.max(30, w - lw - 12)); }
    };
  }

  function widget(w, centre) {
    var k = w.k, s = typeof w.s === "string" ? w.s : "";
    var x = num(w.x) || 0, y = num(w.y) || 0, ww = num(w.w) || 0, hh = num(w.h) || 0;
    var t = (w.t === undefined || w.t === null) ? "" : String(w.t);
    var col = KEY[w.c] || C.blue;
    var act = s.indexOf("a") >= 0, foc = s.indexOf("f") >= 0, prs = s.indexOf("p") >= 0;
    var g = node(null, "g");
    var b = node(g, "g");
    var post = null, rx = 4;
    if (k === "hdr") { post = drawHdr(b, x, y, ww, hh, t, w.u ? String(w.u) : ""); }
    else if (k === "tab") { post = drawTab(b, x, y, ww, hh, t, act, prs, col); }
    else if (k === "seg" || k === "btn" || k === "close") {
      post = drawKey(b, x, y, ww, hh, t, act, prs, col, k);
      if (k === "close") { rx = hh / 2; }
    }
    else if (k === "val") { post = drawVal(b, x, y, ww, hh, t, act, col, centre); }
    else if (k === "lbl") { post = drawLbl(b, x, y, ww, hh, t, w.r ? String(w.r) : "", act, col); }
    // dimmed: shapes fade, text only a little (readable at ~0.5 screen px per page px)
    if (s.indexOf("d") >= 0) {
      for (var c = b.firstChild; c; c = c.nextSibling) {
        c.setAttribute("opacity", c.nodeName === "text" ? "0.7" : "0.45");
      }
    }
    // focus: white outline on the widget edge, outside the dim group
    if (foc && ww > 0 && hh > 0) {
      node(g, "rect", { x: x, y: y, width: ww, height: hh, rx: rx, fill: "none", stroke: "#ffffff", "stroke-width": 2 });
    }
    return { g: g, post: post };
  }

  // a val is centred when an interactive widget follows it in the same row (-5 / value / +5)
  function valCentred(list, w) {
    var x1 = (num(w.x) || 0) + (num(w.w) || 0), y = num(w.y) || 0;
    for (var i = 0; i < list.length; i++) {
      var o = list[i];
      if (o && INTERACTIVE[o.k] && Math.abs((num(o.y) || 0) - y) <= 2 && (num(o.x) || 0) >= x1 - 1) { return true; }
    }
    return false;
  }

  // ------------------------------------------------------------------ menu
  // keyed by list index: only widgets whose JSON changed are rebuilt
  function renderMenu(p) {
    var open = num(p.menu) === 1 || p.menu === true;
    if (open !== S.menu) { S.menu = open; $.menu.style.display = open ? "block" : "none"; }
    if (!open) { return; }
    var list = Array.isArray(p.w) ? p.w : [];
    for (var i = 0; i < list.length; i++) {
      var w = (list[i] && typeof list[i] === "object") ? list[i] : {};
      var centre = w.k === "val" && valCentred(list, w);
      var sig = JSON.stringify(w) + (centre ? "|c" : "");
      if (S.wsig[i] === sig) { continue; }
      var r = widget(w, centre);
      if (S.wel[i]) { $.menuW.replaceChild(r.g, S.wel[i]); } else { $.menuW.appendChild(r.g); }
      S.wel[i] = r.g;
      S.wsig[i] = sig;
      if (r.post) { r.post(); }
    }
    while (S.wel.length > list.length) {
      $.menuW.removeChild(S.wel.pop());
      S.wsig.pop();
    }
  }

  function renderTouch(tp) {
    var x = null, y = null;
    if (tp && typeof tp === "object") {
      x = num(tp.x !== undefined ? tp.x : tp[0]);
      y = num(tp.y !== undefined ? tp.y : tp[1]);
    }
    var sig = (x === null || y === null) ? "" : x.toFixed(1) + "," + y.toFixed(1);
    if (sig === S.tp) { return; }
    S.tp = sig;
    $.touch.style.display = sig ? "block" : "none";
    if (sig) {
      $.ring.setAttribute("cx", x.toFixed(1));
      $.ring.setAttribute("cy", y.toFixed(1));
      $.dot.setAttribute("cx", x.toFixed(1));
      $.dot.setAttribute("cy", y.toFixed(1));
    }
  }

  function receive(p) {
    if (!p || typeof p !== "object") { return; }
    noteView(p);
    S.p = p;
    var sig = "";
    try { sig = JSON.stringify(p); } catch (e) { sig = String(Math.random()); }
    if (sig === S.sig) { return; }   // keepalive with the same content
    S.sig = sig;
    try {
      renderMenu(p);
      renderTouch(p.tp);
    } catch (e) { logErr("render", e); }
    syncMap();
  }

  // --------------------------------------------------------------- MAP view
  // visible = MAP view, panel closed (the opaque panel covers it) and ignition on
  function syncMap() {
    if (!S.ready) { return; }
    var want = S.view === "MAP" && !S.menu && !(M.ign !== null && M.ign < 2);
    if (want !== M.shown) {
      M.shown = want;
      $.map.style.display = want ? "block" : "none";
      M.push = M.g !== null;
      if (want) { mapHeader(); }
    }
    if (!want) { return; }
    if (!M.nav) { navLoad(); }
    if (M.nav) {
      try {
        if (M.fresh && M.data) { M.fresh = false; M.nav.setData(M.data); }
        if (M.push && M.g) { M.push = false; M.nav.updateData(M.g); }
      } catch (e) { logErr("navigator", e); }
    }
    if (!M.data && !M.asked) { askMap(); }
    txt($.mapNote, M.load < 0 ? "MAP UNAVAILABLE" : (M.data ? "" : "NO MAP DATA"));
    txt($.mapClock, clock());
  }

  // the road network arrives once at navigator init: after a page reload ask for it again
  function askMap() {
    M.asked = true;
    try {
      if (typeof beamng !== "undefined" && beamng.sendEngineLua) {
        beamng.sendEngineLua("pcall(function() extensions.ui_uinavi.requestVehicleDashboardMap('@etk800_screen') end)");
      }
    } catch (e) { logErr("askMap", e); }
  }

  function navLoad() {
    if (M.load !== 0) { return; }   // 1 loading, 2 ready, -1 failed
    if (typeof window.bngNavigator === "function") { navCreate(); return; }
    M.load = 1;
    var s = document.createElement("script");
    s.src = NAV_SRC;
    s.onload = function () { M.load = 0; navCreate(); syncMap(); };
    s.onerror = function () { M.load = -1; logErr("navigator", NAV_SRC + " not loaded"); syncMap(); };
    document.head.appendChild(s);
  }

  function navCreate() {
    if (typeof window.bngNavigator !== "function") { M.load = -1; return; }
    try {
      var nav = new window.bngNavigator({
        container: $.mapBox,
        backgroundRgb: [11, 14, 19],
        roadColors: ["#c9d2deFF", "#7d8898FF", "#566070FF"],
        rotate: true,
        scale: MAP_SCALE,
        pitch: MAP_PITCH,
        speedPitch: 3,
        offsetX: MAP_X,
        offsetY: MAP_Y
      });
      var ml = nav.layers && nav.layers.map;
      if (ml) {
        ml.gridColour = "#3a4352";   // page palette
        // routes are off: release the 4096 px route canvas
        if (ml.rcanvas) { ml.rcanvas.width = 1; ml.rcanvas.height = 1; }
      }
      nav.setOpacity(0.45);          // dark terrain, roads keep full contrast
      M.nav = nav;
      M.load = 2;
      M.fresh = !!M.data;
      M.push = !!M.g;
    } catch (e) {
      M.load = -1;
      logErr("navigator", e);
    }
  }

  function mapHeader() {
    var tw = measure($.mapTitle);
    if (tw > 0) {
      $.mapUnder.setAttribute("width", tw.toFixed(1));
      $.mapSub.setAttribute("x", (10 + tw + 14).toFixed(1));
    }
  }

  // ------------------------------------------------------------------ build
  function build() {
    // MAP view: navigator container (transformed by the navigator) + overlay
    $.map = div(document.body, "cpMap", LAYER + "background:" + C.bg + ";z-index:20;");
    var wrap = div($.map, null, "position:absolute;left:0;top:0;width:512px;height:256px;overflow:hidden;");
    $.mapBox = div(wrap, "cpMapBox", "overflow:visible;");
    node($.mapBox, "svg", { style: "overflow:visible;" });
    var ov = node($.map, "svg", { width: "512", height: "256", viewBox: "0 0 512 256",
      style: "position:absolute;left:0;top:0;" + FONT });
    node(ov, "circle", { cx: MAP_X, cy: MAP_Y, r: 16, fill: C.blue, opacity: "0.18" });
    node(ov, "path", { d: "M" + MAP_X + " " + (MAP_Y - 12) + " L" + (MAP_X + 9.5) + " " + (MAP_Y + 11) + " L" + MAP_X + " " +
      (MAP_Y + 6) + " L" + (MAP_X - 9.5) + " " + (MAP_Y + 11) + " Z", fill: C.blue, stroke: C.text, "stroke-width": "1.4",
      "stroke-linejoin": "round" });
    node(ov, "rect", { x: 0, y: 0, width: 512, height: 28, fill: C.bg });
    node(ov, "rect", { x: 8, y: 28, width: 496, height: 1, fill: C.rule });
    $.mapTitle = text(ov, 10, 19, "MAP", 12, 1.6, C.text, true);
    $.mapUnder = node(ov, "rect", { x: 10, y: 23, width: 27, height: 1.6, fill: C.blue });
    $.mapSub = text(ov, 51, 19, "NAVIGATION", 10, 1.2, C.faint, false);
    $.mapClock = text(ov, 502, 19, "", 12, 0, C.text, true, "end");
    $.mapNote = text(ov, 256, 120, "", 11, 1.6, C.faint, true, "middle");

    // control panel (opaque)
    $.menu = layerSvg("cpMenu", LAYER + "z-index:21;" + FONT);
    node($.menu, "rect", { x: 0, y: 0, width: 512, height: 256, fill: C.bg });
    $.menuW = node($.menu, "g");

    // touch ring (no animation; disappears when tp is absent)
    $.touch = layerSvg("cpTouch", LAYER + "z-index:22;");
    $.ring = node($.touch, "circle", { cx: 0, cy: 0, r: 15, fill: "#ffffff", "fill-opacity": "0.08", stroke: "#ffffff",
      "stroke-width": "1.4", "stroke-opacity": "0.85" });
    $.dot = node($.touch, "circle", { cx: 0, cy: 0, r: 1.6, fill: "#ffffff", opacity: "0.9" });
  }

  function refit() {
    // fonts arrived: measure everything again
    S.wsig = [];
    S.sig = "";
    if (S.p) { receive(S.p); }
    if (M.shown) { mapHeader(); }
  }

  function init() {
    try { build(); } catch (e) { logErr("build", e); return; }
    S.ready = true;
    window.etk854hybridPanel = function (p) { stopDemo(); receive(p); };
    if (S.p) { receive(S.p); }
    syncMap();
    try {
      if (document.fonts && document.fonts.ready && document.fonts.ready.then) { document.fonts.ready.then(refit); }
    } catch (e) { /* optional */ }
    startDemo();
  }

  // =================================================================== DEMO
  // Desktop preview only (explicit hash); geometry mirrors K.grid of etk854hybrid_cockpitPanel.lua
  var TABS = ["DRIVE", "EV", "CHASSIS", "TRAILER", "DISPLAY"];
  function demoMenu(tab, level, rows, tp) {
    var w = [{ k: "hdr", x: 0, y: 0, w: 512, h: 30, t: "SETTINGS", u: tab },
      { k: "close", x: 432, y: 4, w: 72, h: 22, t: "CLOSE" }];
    for (var i = 0; i < TABS.length; i++) {
      w.push({ k: "tab", x: 8, y: 36 + i * 43, w: 110, h: 33, t: TABS[i],
        s: TABS[i] === tab ? (level === "tabs" ? "af" : "a") : "" });
    }
    for (var r = 0; r < rows.length; r++) {
      var R = rows[r], y = 38 + r * 53, cells = R.cells, rs = R.s || "";
      w.push({ k: "lbl", x: 132, y: y, w: 372, h: 14, t: R.lbl, r: R.hint });   // Lua never dims a label
      // [x, w] per cell: "pm" = -5 / value over tiles 2-4 / +5, "bv" = button + value over tiles 2-5
      var geo = R.geo === "pm" ? [[132, 71], [207, 221], [432, 71]] : (R.geo === "bv" ? [[132, 71], [207, 296]] : null);
      for (var j = 0; j < cells.length; j++) {
        var c = cells[j];
        w.push({ k: c.k || "seg", x: geo ? geo[j][0] : 132 + j * 75, y: y + 15, w: geo ? geo[j][1] : 71, h: 30,
          t: c.t, s: (c.s || "") + rs, c: c.c || "blue" });
      }
    }
    return { v: 1, view: "ENERGY", menu: 1, tab: tab, w: w, tp: tp };
  }
  var DEMOS = {
    "menu": function () {
      return demoMenu("DRIVE", "items", [
        { lbl: "DRIVING MODE", cells: [{ t: "COMFORT", s: "a" }, { t: "ECO PRO", s: "f" }, { t: "SPORT" }] },
        { lbl: "RECUPERATION", hint: "ONE-PEDAL", cells: [{ t: "OFF", c: "green" }, { t: "LOW", c: "green" },
          { t: "MED", c: "green" }, { t: "HIGH", s: "a", c: "green" }] },
        { lbl: "GEARBOX PROGRAM", hint: "COMFORT", cells: [{ t: "AUTO", s: "a" }, { t: "ECO" }, { t: "COMF" },
          { t: "SPORT" }, { t: "MAN" }] },
        { lbl: "LAUNCH CONTROL", s: "d", geo: "bv", cells: [{ k: "btn", t: "ARM", c: "amber" }, { k: "val", t: "LAUNCH", c: "amber" }] }
      ]);
    },
    "menu-ev": function () {
      return demoMenu("EV", "items", [
        { lbl: "EV MODE", hint: "SOC 64 %", cells: [{ t: "AUTO EV" }, { t: "MAX EV" }, { t: "BATTERY", s: "a" }] },
        { lbl: "BATTERY TARGET", hint: "IN BATTERY CONTROL", geo: "pm", cells: [{ k: "btn", t: "-5" }, { k: "val", t: "80 %" },
          { k: "btn", t: "+5", s: "fp" }] }
      ], [468, 121]);
    },
    "menu-chassis": function () {
      return demoMenu("CHASSIS", "tabs", [
        { lbl: "STABILITY CONTROL", cells: [{ t: "DSC" }, { t: "DTC", c: "red" }, { t: "OFF", s: "a", c: "amber" }] },
        { lbl: "XALLRAD", cells: [{ t: "AWD", s: "a" }, { t: "2WD" }] },
        { lbl: "REAR-AXLE STEERING", hint: "INACTIVE WHILE DSC OFF", s: "d", cells: [{ t: "OFF", c: "amber" },
          { t: "ON + DSC", s: "a" }, { t: "ON - DSC", c: "lblue" }] }
      ]);
    },
    "menu-trailer": function () {
      return demoMenu("TRAILER", "items", [
        { lbl: "TRAILER MODE", hint: "ACTIVE - LEVEL", cells: [{ t: "AUTO", s: "a", c: "amber" }, { t: "ON", c: "amber" },
          { t: "OFF", c: "amber" }] },
        { lbl: "AIR COMPRESSOR", hint: "7.6 BAR", cells: [{ t: "ON", s: "afp", c: "lblue" }, { t: "OFF", c: "amber" }] }
      ], [167, 121]);
    },
    "menu-display": function () {
      return demoMenu("DISPLAY", "items", [
        { lbl: "CENTRE DISPLAY", cells: [{ t: "ENERGY", s: "a" }, { t: "MAP" }, { t: "TRAILER" }] },
        { lbl: "INSTRUMENT CLUSTER", hint: "AUTO: SPORT", cells: [{ t: "AUTO", s: "a" }, { t: "HYBRID" }, { t: "EFFIC." },
          { t: "SPORT" }, { t: "MINIMAL" }] },
        { lbl: "TRIP COMPUTER", geo: "bv", cells: [{ k: "btn", t: "RESET", s: "fp" }, { k: "val", t: "128.4 KM" }] }
      ], [192, 174]);
    }
  };

  // synthetic road network (m): curved main road, street grid, dirt track; the pose follows the main road
  function demoPath(u) {
    return [-700 + 1400 * u, 160 * Math.sin(u * 5.2) - 80 * u];
  }
  function demoRoads() {
    var nodes = {}, i, j, n = 40;
    function link(a, b, dr) {
      nodes[a].links[b] = { drivability: dr, oneWay: false };
      nodes[b].links[a] = { drivability: dr, oneWay: false };
    }
    function add(id, x, y, r) { nodes[id] = { pos: [x, y], radius: r, links: {} }; }
    for (i = 0; i <= n; i++) {
      var p = demoPath(i / n);
      add("m" + i, p[0], p[1], 5);
      if (i > 0) { link("m" + (i - 1), "m" + i, 1); }
    }
    for (i = -3; i <= 3; i++) {
      for (j = -3; j <= 3; j++) { add("g" + i + "_" + j, i * 180 + 40, j * 150 - 30, 3); }
    }
    for (i = -3; i <= 3; i++) {
      for (j = -3; j <= 3; j++) {
        if (i < 3) { link("g" + i + "_" + j, "g" + (i + 1) + "_" + j, 0.6); }
        if (j < 3) { link("g" + i + "_" + j, "g" + i + "_" + (j + 1), 0.6); }
      }
    }
    for (i = 0; i <= 12; i++) {
      add("d" + i, -500 + 60 * i + 30 * Math.sin(i), 380 - 25 * i, 2);
      if (i > 0) { link("d" + (i - 1), "d" + i, 0.05); }
    }
    return { nodes: nodes, terrainTiles: [], time: "12:00" };
  }
  function demoPose(t) {
    var u = 0.42 + ((t * 14 / 1400) % 0.5);
    var a = demoPath(u), b = demoPath(u + 0.002);
    var hdg = Math.atan2(b[0] - a[0], b[1] - a[1]) * 180 / Math.PI;   // clockwise from north
    return { x: a[0], y: a[1], rotation: -hdg, speed: 50, zoom: 1, ignitionLevel: 2 };
  }

  function startDemo() {
    var m = /demo=([a-z-]+)/.exec(String(window.location.hash || ""));
    var name = m ? m[1] : "";
    if (name === "map") {
      window.map.setData(demoRoads());
      receive({ v: 1, view: "MAP", menu: 0 });
      var t0 = Date.now();
      var step = function () { window.map.updateData(demoPose((Date.now() - t0) / 1000)); };
      step();
      S.demo = setInterval(step, 66);
    } else if (DEMOS[name]) {
      receive(DEMOS[name]());
    }
  }
  function stopDemo() {
    if (S.demo) { clearInterval(S.demo); S.demo = null; }
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", init);
  } else {
    init();
  }
})();
