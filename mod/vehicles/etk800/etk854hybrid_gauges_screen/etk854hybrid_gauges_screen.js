// ETK 854 ActiveHybrid - instrument cluster page logic
// Web view "@etk800_gauges_screen" (512 x 256). genericGauges calls
// updateData({electrics, powertrain, customModules}) with the channels of
// gauge.displayData (etk854hybrid_gauges.jbeam). Missing electrics arrive as
// the number 0, so string channels are type-checked. customModules:
//   environmentData        { time "HH:MM", temperatureEnv [C] }
//   etk854hybrid_gaugeData { rangeKm iceKw evKw regenKw gearStyle, on-board
//                            computer: fuelL fuelRangeKm instL100 instLh
//                            instMode instBar avgL100 tripKm avgKwh }
// AIR chips: etk854hybrid_air* / trailerSpringCoef / parkingbrake.
// ECO PRO demand caret: etk854hybrid_pedalZone (1 EV zone | 2 EV MAX plateau |
// 3 hybrid band | 4 kick | 5 kickdown), _pedalEvFrac, _pedalBandFrac,
// _pedalIceJoin (> 0 = split map live), _evZoneOpen, _iceDeficitS.
//
// Layout (texture px, visible through the stock opacity mask):
//   centre column  x 95..221  modes, SOC, EV range, phase, flow glyph
//   bottom strip   y 216..240 odometer, clock, outside temperature
//   rev-counter bowl, centre (335.5, 102.3), r ~136: shift program, gear,
//                  system power, chips, power meter (CHARGE | EV | POWER | BOOST)
//   speedometer sliver x 63..92, y 165..210: instantaneous consumption bar
//   band y ~213..222, x 78..240: tank, fuel range, trip averages
//
// Views HYBRID / EFFICIENCY / SPORT / MINIMAL (etk854hybrid_clusterView, AUTO
// resolved by the cockpit panel; 0 -> HYBRID). Each view owns its elements
// (data-views groups switched by opacity: getComputedTextLength needs laid-out
// text, so never display:none), so no memo (__t, __k, zoneKey, chipKey) holds
// another view's geometry. Shared: header, chips, power meter, bottom strip,
// the HYBRID gear box (MINIMAL) and the toast.
//
// Plain DOM on one inline SVG, every write memoised. No CSS animations (the
// vehicle web view freezes, see the css): animate() runs per data frame.
(function () {
  "use strict";

  // ----------------------------------------------------------------- palette
  var C = {
    text: "#e6ebf2",
    muted: "#8b95a5",
    dim: "#394150",
    dark: "#0b0e13",
    blue: "#2b8bff",
    blueLt: "#8cc8ff",
    amber: "#ffab3d",
    orange: "#ff6b2c",
    green: "#2ad99c",
    red: "#ff4f5e",
    eco: "#35d0b5",
    sport: "#ff5a3c"
  };

  // ------------------------------------------------------ power meter geometry
  // Concentric with the rev-counter ring; degrees from straight down, + = right.
  var ARC = { cx: 335.5, cy: 102.3, r: 122, zoneR: 112.5, a0: -48, a1: 48, s0: 0.2 };
  // demand caret outside the arc, tip towards the centre (track edge r + 4.5,
  // mask fully open to r + 12, cut at ~r + 13.5)
  var DEM = { tip: ARC.r + 5, base: ARC.r + 12.5, half: 4, tauS: 0.08, pulseMs: 1000 };
  // EFFICIENCY: larger caret plus a line across the bar
  var DEM_EF = { tip: ARC.r + 3, base: ARC.r + 12.5, half: 5.5, ln0: ARC.r - 5.5 };

  // e-machine limits per boost stage (fallback without the Lua module);
  // mirrors BOOST_STAGES in etk854hybridControl.lua
  var STAGE_EV_KW = [83, 110, 150, 200];
  var STAGE_REGEN_KW = [55, 65, 80, 95];
  var DEFAULT_ICE_KW = 250;

  var DE_LABEL = { COMFORT: "COMFORT", ECO_PRO: "ECO PRO", SPORT: "SPORT" };
  // shift program that only repeats the driving-experience headline
  var DE_PROG_BASE = { COMFORT: "COMFORT", ECO_PRO: "ECO", SPORT: "SPORT" };
  var DE_COLOR = { COMFORT: C.text, ECO_PRO: C.eco, SPORT: C.sport };
  var ED_LABEL = { AUTO_EDRIVE: "AUTO EV", MAX_EDRIVE: "MAX EV", BATTERY_CONTROL: "BATTERY CONTROL" };
  var ED_COLOR = { AUTO_EDRIVE: C.blueLt, MAX_EDRIVE: C.blue, BATTERY_CONTROL: C.amber };

  var PHASE = {
    READY: { label: "READY", color: C.blueLt },
    EDRIVE: { label: "ELECTRIC", color: C.blue },
    HYBRID: { label: "HYBRID", color: C.amber },
    BOOST: { label: "eBOOST", color: C.orange },
    ENGINE: { label: "ENGINE", color: C.amber },
    SAIL: { label: "SAILING", color: C.blueLt },
    REGEN: { label: "RECUP", color: C.green },
    CHARGE: { label: "CHARGING", color: C.green },
    OFF: { label: "OFF", color: C.muted },
    NONE: { label: "\u2014", color: C.dim }
  };

  var ICE_STARTING = { CRANK: 1, SYNC: 1, ENGAGE: 1 };
  var REGEN_CHIP = { OFF: "REGEN OFF", LOW: "REGEN LOW", HIGH: "REGEN HIGH" };
  var LEVEL_ALERT = { MAX: 1, FAULT: 1 };

  // letter in the data-views attribute
  var VIEW_CODE = { HYBRID: "H", EFFICIENCY: "E", SPORT: "S", MINIMAL: "M" };
  // the toast covers the mid block of every view
  var MID_IDS = ["mid", "efMid", "spMid", "mnMid"];
  // pedalZone 1..5, HUD wording
  var PEDAL_ZONE = ["", "EV", "EV FULL", "HYBRID", "ICE", "KICKDOWN"];
  var DSC_TXT = { DSC: ["DSC", C.blueLt], DTC: ["DTC", C.amber], OFF: ["DSC OFF", C.amber] };

  // ------------------------------------------------------------------ state
  var units = { length: "metric", temp: "c", cons: "metric", vol: "l", press: "bar" };
  var $ = {};
  var ready = false;
  var scale = { max: DEFAULT_ICE_KW + STAGE_EV_KW[0], regen: STAGE_REGEN_KW[0], ice: DEFAULT_ICE_KW, ev: STAGE_EV_KW[0] };
  var prev = null; // last values for toast detection
  var startT = now();
  var toastTimer = null;
  var lastToastKey = "";
  var lastToastT = 0;
  var zoneKey = "";
  var chipKey = ["", ""];
  var dem = { s: null, t: 0, vis: false }; // demand caret: smoothed arc position
  // ECO EV ceiling gate (evZoneOpen), debounced: the VCU compares raw speed,
  // so cruising at the ceiling would flicker the zone
  var evGate = { open: true, since: 0, holdMs: 500 };
  var viewEls = [];   // groups with data-views
  var viewNow = "";
  // EFFICIENCY: smoothed electric consumption; SPORT: peak power hold
  var efCons = { v: 0, t: 0, tauS: 0.6, minSpeed: 1.4 };
  var spPeak = { kw: 0, t: 0, holdMs: 5000 };

  function now() {
    return (window.performance && performance.now) ? performance.now() : Date.now();
  }

  // ------------------------------------------------------------ helpers
  function num(v) {
    if (typeof v === "number") { return isFinite(v) ? v : null; }
    if (typeof v === "boolean") { return v ? 1 : 0; }
    if (typeof v === "string" && v !== "" && isFinite(+v)) { return +v; }
    return null;
  }
  function str(v) {
    return (typeof v === "string" && v !== "") ? v : null;
  }
  function clamp01(v) {
    return v < 0 ? 0 : (v > 1 ? 1 : v);
  }
  function txt(el, v) {
    if (el && el.__t !== v) { el.__t = v; el.textContent = v; }
  }
  function attr(el, k, v) {
    if (!el) { return; }
    var c = el.__a || (el.__a = {});
    if (c[k] !== v) { c[k] = v; el.setAttribute(k, v); }
  }
  // shrink a text element horizontally when it would leave its zone
  function fit(el, maxW) {
    if (!el) { return; }
    el.removeAttribute("textLength");
    el.removeAttribute("lengthAdjust");
    var w = 0;
    try { w = el.getComputedTextLength(); } catch (e) { w = 0; }
    if (w > maxW) {
      el.setAttribute("textLength", maxW);
      el.setAttribute("lengthAdjust", "spacingAndGlyphs");
    }
  }
  function setText(el, v, maxW) {
    if (!el || el.__t === v) { return; }
    el.__t = v;
    el.textContent = v;
    if (maxW) { fit(el, maxW); }
  }
  function thousands(n) {
    return String(n).replace(/\B(?=(\d{3})+(?!\d))/g, "\u2009");
  }

  // --------------------------------------------------------- arc geometry
  function ang(s) {
    return ARC.a0 + (ARC.a1 - ARC.a0) * s;
  }
  function pt(r, deg) {
    var t = deg * Math.PI / 180;
    return [ARC.cx + r * Math.sin(t), ARC.cy + r * Math.cos(t)];
  }
  function arcD(r, s1, s2) {
    if (!(s2 - s1 > 0.003)) { return "M0 0"; }
    var p1 = pt(r, ang(s1));
    var p2 = pt(r, ang(s2));
    return "M" + p1[0].toFixed(2) + " " + p1[1].toFixed(2) +
      " A" + r + " " + r + " 0 0 0 " + p2[0].toFixed(2) + " " + p2[1].toFixed(2);
  }
  function sOf(kw) {
    if (kw >= 0) { return ARC.s0 + (1 - ARC.s0) * clamp01(kw / scale.max); }
    return ARC.s0 - ARC.s0 * clamp01(-kw / scale.regen);
  }

  function buildMeter() {
    attr($.mTrack, "d", arcD(ARC.r, 0, 1));
    attr($.zCharge, "d", arcD(ARC.zoneR, 0, ARC.s0 - 0.006));
    // ticks: zero (bright, long) and quarter steps of the drive range
    var ns = "http://www.w3.org/2000/svg";
    var g = $.mTicks;
    while (g.firstChild) { g.removeChild(g.firstChild); }
    var steps = [ARC.s0, ARC.s0 + (1 - ARC.s0) * 0.25, ARC.s0 + (1 - ARC.s0) * 0.5, ARC.s0 + (1 - ARC.s0) * 0.75];
    for (var i = 0; i < steps.length; i++) {
      var zero = i === 0;
      var a = ang(steps[i]);
      var p1 = pt(zero ? ARC.zoneR - 3 : ARC.r - 6.5, a);
      var p2 = pt(ARC.r + 5.5, a);
      var ln = document.createElementNS(ns, "line");
      ln.setAttribute("x1", p1[0].toFixed(2));
      ln.setAttribute("y1", p1[1].toFixed(2));
      ln.setAttribute("x2", p2[0].toFixed(2));
      ln.setAttribute("y2", p2[1].toFixed(2));
      ln.setAttribute("stroke", zero ? C.text : "#000");
      ln.setAttribute("stroke-width", zero ? "2" : "1.4");
      g.appendChild(ln);
    }
    // "CHARGE" follows the arc (path left -> right, glyph tops inwards); the
    // drive side needs no label, its colours carry EV / POWER / BOOST
    attr($.mLblPath, "d", arcD(ARC.zoneR - 8.5, 0, ARC.s0));
    try {
      $.mLblCPath.setAttributeNS("http://www.w3.org/1999/xlink", "xlink:href", "#mLblPath");
    } catch (e) { /* href alone is enough in current Chromium */ }
  }

  function updateZones(evLimitKw, iceKw) {
    var key = Math.round(evLimitKw) + "|" + Math.round(iceKw) + "|" + Math.round(scale.max);
    if (key === zoneKey) { return; }
    zoneKey = key;
    var sE = sOf(Math.max(0, evLimitKw));
    var sI = Math.max(sE, sOf(iceKw));
    attr($.zEdrive, "d", arcD(ARC.zoneR, ARC.s0 + 0.006, sE));
    attr($.zPower, "d", arcD(ARC.zoneR, sE + 0.006, sI));
    attr($.zBoost, "d", arcD(ARC.zoneR, sI + 0.006, 1));
  }

  function q(v) {
    // quantise bar ends so a steady value does not rewrite the path
    return Math.round(v * 500) / 500;
  }

  // demand caret at arc position s (g = DEM or DEM_EF)
  function caretD(s, g) {
    g = g || DEM;
    var a = ang(s);
    var t = a * Math.PI / 180;
    var tip = pt(g.tip, a);
    var bc = pt(g.base, a);
    var ux = Math.cos(t) * g.half;   // unit tangent of pt(): (cos t, -sin t)
    var uy = -Math.sin(t) * g.half;
    return "M" + tip[0].toFixed(2) + " " + tip[1].toFixed(2) +
      " L" + (bc[0] + ux).toFixed(2) + " " + (bc[1] + uy).toFixed(2) +
      " L" + (bc[0] - ux).toFixed(2) + " " + (bc[1] - uy).toFixed(2) + " Z";
  }
  // radial line at arc position s from radius r0 to r1
  function radialD(s, r0, r1) {
    var p0 = pt(r0, ang(s));
    var p1 = pt(r1, ang(s));
    return "M" + p0[0].toFixed(2) + " " + p0[1].toFixed(2) + " L" + p1[0].toFixed(2) + " " + p1[1].toFixed(2);
  }

  // ---------------------------------------------------------------- toast
  function midOpacity(o) {
    for (var i = 0; i < MID_IDS.length; i++) { attr($[MID_IDS[i]], "opacity", o); }
  }
  function toast(title, sub, color, info, seconds) {
    if (!ready) { return; }
    var key = title + "|" + sub + "|" + (info || "");
    var t = now();
    if (key === lastToastKey && t - lastToastT < 1500) { return; }
    lastToastKey = key;
    lastToastT = t;
    // the column is only 65 px wide around y 88..120
    $.toastTitle.__t = null;
    setText($.toastSub, sub || "", 62);
    setText($.toastTitle, title || "", 58);
    setText($.toastInfo, info || "", 62);
    attr($.toastTitle, "fill", color || C.text);
    attr($.toast, "opacity", "1");
    midOpacity("0");
    if (toastTimer) { clearTimeout(toastTimer); }
    toastTimer = setTimeout(function () {
      toastTimer = null;
      attr($.toast, "opacity", "0");
      midOpacity("1");
    }, Math.max(0.6, Math.min(8, seconds || 1.8)) * 1000);
  }

  function detectChanges(v) {
    if (!prev) {
      prev = v;
      return;
    }
    // no toasts while the page settles after spawn
    if (now() - startT > 2500 && v.hasVcu) {
      if (v.de !== prev.de && v.de) {
        toast(DE_LABEL[v.de] || v.de, "DRIVE MODE", DE_COLOR[v.de] || C.text);
      } else if (v.ed !== prev.ed && v.ed) {
        var info = v.ed === "BATTERY_CONTROL" && v.tgt !== null ? "TARGET " + Math.round(v.tgt * 100) + " %" : "";
        toast(ED_LABEL[v.ed] || v.ed, "EV", ED_COLOR[v.ed] || C.blueLt, info);
      } else if (v.rl !== prev.rl && v.rl) {
        toast(v.rl, "RECUPERATION", v.rl === "OFF" ? C.muted : C.green);
      } else if (v.awd !== prev.awd && v.awd && prev.awd) {
        toast(v.awd, "DRIVETRAIN", v.awd === "AWD" ? C.blue : C.text, v.awd === "AWD" ? "ALL WHEEL DRIVE" : "REAR WHEEL DRIVE");
      } else if (v.trl !== prev.trl) {
        toast(v.trl ? "ON" : "OFF", "TRAILER MODE", v.trl ? C.amber : C.muted);
      } else if (v.dsc !== prev.dsc && v.dsc) {
        toast(v.dsc === "OFF" ? "DSC OFF" : v.dsc, "STABILITY", v.dsc === "DSC" ? C.blueLt : C.amber);
      } else if (v.boost !== prev.boost) {
        toast(v.boost > 0 ? "STAGE " + v.boost : "STOCK", "E-BOOST", v.boost > 0 ? C.orange : C.text);
      }
    }
    prev = v;
  }

  // ----------------------------------------------------------------- gear
  // g      = shift logic gear name: "P" "R" "N" "D", "S<n>", "M<n>"
  // actual = etk854hybrid_gearActual, the engaged gear; GEAR_UNKNOWN without a
  //          gearbox device (a missing channel is 0, so only the marker falls back)
  // target = electrics.gearIndex, the 8HP decision (runs ahead) - fallback only
  // style  = "number": D1..D8 | "plain": D. S/M always numbered; P / R / N and
  //          neutral (index <= 0) never.
  var GEAR_UNKNOWN_MAX = -50; // gaugeData GEAR_UNKNOWN = -99
  function gearLabel(g, actual, target, style) {
    var n = (actual !== null && actual > GEAR_UNKNOWN_MAX) ? actual : target;
    var gn = (n !== null && n >= 1) ? String(Math.round(n)) : "";
    if (typeof g === "string" && g !== "") {
      var G = g.toUpperCase();
      var c = G.charAt(0);
      if (G === "P" || G === "R" || G === "N") { return G; }
      if (c === "D") { return style === "plain" ? "D" : "D" + gn; }
      if (c === "S" || c === "M") { return c + gn; }
      return G.slice(0, 3);
    }
    var gi = num(g);
    return gi === null ? "-" : (gi < 0 ? "R" : (gi === 0 ? "N" : String(gi)));
  }

  // ---------------------------------------------------------------- chips
  function chipList(v, blinkDsc) {
    var list = [];
    if (v.iesStart) { list.push(["START", C.blueLt, false]); }
    if (v.band === "CRIT" || v.band === "LOW1") { list.push(["BATT LOW", C.red, false]); }
    // air warnings: receiver below the 4.14 bar spring-brake release, or an
    // air-coupled trailer holding its spring brakes with our parking brake off
    if (v.air && v.air.low) { list.push(["AIR LOW", C.amber, false, v.air.run]); }
    else if (v.air && v.air.trlBrk) { list.push(["TRL BRK", C.amber, false, v.air.run]); }
    if (v.dsc === "DTC") { list.push(["DTC", C.amber, blinkDsc]); }
    else if (v.dsc === "OFF") { list.push(["DSC OFF", C.amber, blinkDsc]); }
    else if (blinkDsc) { list.push(["DSC", C.amber, true]); }
    if (v.lvlAlert) { list.push(["LEVEL", C.red, false]); }
    if (v.trl) { list.push(["TRAILER", C.amber, false]); }
    if (v.awd === "2WD") { list.push(["2WD", C.text, false]); }
    if (v.op) { list.push(["ONE PEDAL", C.green, false]); }
    else if (REGEN_CHIP[v.rl]) { list.push([REGEN_CHIP[v.rl], v.rl === "OFF" ? C.muted : C.green, false]); }
    if (v.boost > 0) { list.push(["BOOST " + v.boost, C.orange, false]); }
    if (v.fan >= 2) { list.push(["FAN " + v.fan, C.blueLt, false]); }
    // receiver pressure, lowest priority: dot = compressor running, amber
    // outline = protection valve shut (reopens at 6.0 bar)
    if (v.air && !v.air.low && !v.air.trlBrk) {
      list.push([v.air.label, v.air.shut ? C.amber : (v.air.run ? C.blueLt : C.muted), false, v.air.run]);
    }
    return list;
  }
  // per view: MINIMAL keeps warnings + DSC; EFFICIENCY / SPORT drop the chips
  // their column already shows (regen; dsc, boost). Unknown labels = info.
  var CHIP_KIND = {
    "BATT LOW": "warn", "AIR LOW": "warn", "TRL BRK": "warn", LEVEL: "warn",
    DSC: "dsc", DTC: "dsc", "DSC OFF": "dsc",
    "ONE PEDAL": "regen", "REGEN OFF": "regen", "REGEN LOW": "regen", "REGEN HIGH": "regen"
  };
  function chipKind(label) {
    if (CHIP_KIND.hasOwnProperty(label)) { return CHIP_KIND[label]; }
    return label.indexOf("BOOST ") === 0 ? "boost" : "info";
  }
  function chipsFor(view, list) {
    if (view === "HYBRID") { return list; }
    var out = [];
    for (var i = 0; i < list.length; i++) {
      var k = chipKind(list[i][0]);
      var keep = view === "MINIMAL" ? (k === "warn" || k === "dsc") :
        !((view === "EFFICIENCY" && k === "regen") || (view === "SPORT" && (k === "dsc" || k === "boost")));
      if (keep) { out.push(list[i]); }
    }
    return out;
  }
  // slot 0 right of the gear box (left edge x 367), slot 1 left of it (right
  // edge x 313, clear of the CHARGE label)
  var CHIP_MAXW = [52, 38];
  var CHIP_X = [367, 313];
  function setChip(i, c) {
    var g = $["chip" + (i + 1)];
    var box = $["chip" + (i + 1) + "Box"];
    var t = $["chip" + (i + 1) + "Txt"];
    var key = c ? c.join("|") : "";
    if (chipKey[i] === key) { return; }
    chipKey[i] = key;
    var dot = $["chip" + (i + 1) + "Dot"];
    if (!c) {
      attr(g, "opacity", "0");
      attr(dot, "opacity", "0");
      g.__blink = false;
      return;
    }
    var hasDot = !!c[3];
    var pad = hasDot ? 14 : 11; // dot ~3.5 px + gap
    t.__t = null;
    txt(t, c[0]);
    t.removeAttribute("textLength");
    t.removeAttribute("lengthAdjust");
    var w = 0;
    try { w = t.getComputedTextLength(); } catch (e) { w = c[0].length * 5; }
    var bw = Math.min(CHIP_MAXW[i], Math.ceil(w) + pad);
    if (w + pad > CHIP_MAXW[i]) {
      t.setAttribute("textLength", CHIP_MAXW[i] - pad);
      t.setAttribute("lengthAdjust", "spacingAndGlyphs");
    }
    var x0 = i === 0 ? CHIP_X[0] : CHIP_X[1] - bw;
    attr(box, "width", bw);
    attr(box, "x", x0);
    attr(t, "x", (x0 + (hasDot ? (bw - 3) / 2 : bw / 2)).toFixed(1));
    attr(dot, "cx", (x0 + bw - 5.5).toFixed(1));
    attr(dot, "opacity", hasDot ? "1" : "0");
    attr(t, "fill", c[1] === C.muted ? C.muted : C.text);
    attr(box, "stroke", c[1]);
    attr(g, "opacity", "1");
    g.__blink = !!c[2];
  }

  // ---------------------------------------------------------------- flow
  function setLink(el, mode, color) {
    attr(el, "class", "flow " + mode);
    el.__mode = mode;
    if (mode !== "idle") { attr(el, "stroke", color); }
  }

  // ------------------------------------------------------------ animation
  // Script-driven instead of CSS keyframes (see the css), once per data frame:
  // coarse steps, but the page stays live.
  var FLOW_PERIOD_MS = 450;   // one dash period (6 px) per 0.45 s
  var BLINK_PERIOD_MS = 500;  // 0.25 s on / 0.25 s dim
  function animate() {
    var t = now();
    var ph = (t % FLOW_PERIOD_MS) / FLOW_PERIOD_MS;
    var links = [$.lnIceA, $.lnIceB, $.lnBat, $.lnWhl];
    for (var i = 0; i < links.length; i++) {
      var el = links[i];
      if (!el) { continue; }
      var m = el.__mode;
      var off = m === "fwd" ? 6 * (1 - ph) : (m === "rev" ? 6 * ph : 0);
      attr(el, "stroke-dashoffset", off.toFixed(1));
    }
    var lit = (t % BLINK_PERIOD_MS) < BLINK_PERIOD_MS / 2;
    var bl = [$.chip1, $.chip2, $.icIce, $.spDsc];
    for (var j = 0; j < bl.length; j++) {
      var g = bl[j];
      if (!g) { continue; }
      if (g === $.icIce || g === $.spDsc) {
        attr(g, "opacity", g.__blink && !lit ? "0.25" : "1");
      } else if (g.__a && g.__a.opacity !== "0") {
        // visible chip: blink between 1 and 0.25; setChip hides with "0"
        attr(g, "opacity", g.__blink && !lit ? "0.25" : "1");
      }
    }
    // demand carets: hidden / steady / 1 Hz pulse on the EV MAX plateau
    var cr = [$.mDem, $.efDem];
    for (var k = 0; k < cr.length; k++) {
      var md = cr[k];
      if (!md) { continue; }
      var op = "0";
      if (md.__vis) {
        op = md.__pulse ? (0.725 + 0.275 * Math.cos(2 * Math.PI * (t % DEM.pulseMs) / DEM.pulseMs)).toFixed(2) : "1";
      }
      attr(md, "opacity", op);
    }
  }
  function setIcon(el, color, lit) {
    attr(el, "stroke", color);
    attr(el, "fill", lit ? "rgba(255,255,255,0.06)" : C.dark);
  }

  // ----------------------------------------------------- on-board computer
  // gaugeData "fuel". Units from setup() as in the stock gauges_screen.js:
  // consumption "metric" -> l/100 km else US mpg, volume "gal" -> US gal,
  // length -> km / mi. Litres as "L": a condensed "l" reads as "|" here.
  var L_PER_GAL = 3.785411784;
  var KM_PER_MI = 1.609344;
  var KM_TO_MI = 0.621371; // EV range display factor, not 1 / KM_PER_MI (rounds differently)
  var MPG_L100 = 235.214583; // mpg = 235.21 / (l/100 km)
  var OBC_BAR_Y = 207, OBC_BAR_H = 40;
  var obcFitKey = "";
  function mpgOf(l100) {
    return l100 < 0.5 ? "99+" : String(Math.min(99, Math.round(MPG_L100 / l100)));
  }
  function applyObc(gd, on, hasVcu) {
    var fuelL = num(gd.fuelL);
    var show = on && fuelL !== null && fuelL >= 0;
    attr($.obc, "opacity", show ? "1" : "0");
    if (!show) { return; }
    var mpg = units.cons !== "metric";
    var gal = units.vol === "gal";
    var miles = units.length !== "metric";

    // instantaneous: bar + number in the speedometer sliver
    var mode = str(gd.instMode) || "ev";
    var bar = clamp01(num(gd.instBar) || 0);
    var val, unit;
    if (mode === "ev") {
      val = mpg ? "EV" : "0.0";
      unit = mpg ? "" : "EV";
      bar = 0;
    } else if (mode === "lh") {
      var lh = Math.max(0, num(gd.instLh) || 0);
      val = (gal ? lh / L_PER_GAL : lh).toFixed(1);
      unit = gal ? "gal/h" : "L/h";
    } else {
      var l100 = num(gd.instL100);
      val = (l100 === null || l100 < 0) ? "--" : (mpg ? mpgOf(l100) : l100.toFixed(1));
      unit = mpg ? "mpg" : "L/100";
    }
    var ev = mode === "ev";
    setText($.fiVal, val, 16);
    attr($.fiVal, "fill", ev ? C.blue : C.text);
    setText($.fiUnit, unit, 14);
    attr($.fiUnit, "fill", ev ? C.blue : C.muted);
    var h = OBC_BAR_H * bar;
    attr($.fiBar, "height", h.toFixed(1));
    attr($.fiBar, "y", (OBC_BAR_Y - h).toFixed(1));
    attr($.fiBar, "fill", bar > 0.66 ? C.orange : C.amber);
    attr($.fiTrack, "fill", ev ? "#0f2440" : "#161b23");

    // tank + fuel range (left), trip averages fuel / electric (right)
    var vol = gal ? fuelL / L_PER_GAL : fuelL;
    txt($.fuelVal, (gal || vol < 10) ? vol.toFixed(1) : String(Math.round(vol)));
    txt($.fuelUnit, gal ? "gal" : "L");
    var rng = num(gd.fuelRangeKm);
    txt($.frngVal, (rng === null || rng < 0) ? "--" : String(Math.round(miles ? rng / KM_PER_MI : rng)));
    txt($.frngUnit, miles ? "mi" : "km");
    var avg = num(gd.avgL100);
    txt($.avgF, (avg === null || avg < 0) ? "--" : (mpg ? mpgOf(avg) : avg.toFixed(1)));
    txt($.avgFU, mpg ? "mpg" : "L");
    var kwh = num(gd.avgKwh);
    var hasE = hasVcu && kwh !== null && kwh > 0;
    txt($.avgE, hasE ? (miles ? kwh * KM_PER_MI : kwh).toFixed(1) : "");
    txt($.avgEU, hasE ? "kWh" : "");
    txt($.avgPer, (mpg && !hasE) ? "" : (miles ? "/100mi" : "/100km"));
    // re-fit only when the strings changed (getComputedTextLength)
    var key = $.obcL.textContent + "|" + $.obcR.textContent;
    if (key !== obcFitKey) {
      obcFitKey = key;
      fit($.obcL, 74);
      fit($.obcR, 106);
    }
  }

  // ------------------------------------------------------------------ views
  function viewOf(v) {
    return (typeof v === "string" && VIEW_CODE.hasOwnProperty(v)) ? v : "HYBRID";
  }
  function setView(v) {
    if (v === viewNow) { return; }
    viewNow = v;
    var code = VIEW_CODE[v];
    for (var i = 0; i < viewEls.length; i++) {
      var el = viewEls[i];
      attr(el, "opacity", String(el.getAttribute("data-views")).indexOf(code) >= 0 ? "1" : "0");
    }
  }
  // 8HP shift program colour (ECO teal, SPORT red, MANUAL amber)
  function progColor(t) {
    if (t.indexOf("ECO") === 0) { return C.eco; }
    if (t.indexOf("SPORT") === 0) { return C.sport; }
    if (t.indexOf("MANUAL") === 0) { return C.amber; }
    return C.text;
  }

  // gear box geometry: HYBRID (also shown in MINIMAL) and the large SPORT box
  var GEAR_GEO_H = { x: "339", xHint: "333", w: 38, wHint: 29, up: "M354 167.5 L358.5 174 H349.5 Z", dn: "M349.5 167.5 H358.5 L354 174 Z" };
  var GEAR_GEO_S = { x: "340", xHint: "331", w: 40, wHint: 26, up: "M355 165 L360.5 173 H349.5 Z", dn: "M349.5 165 H360.5 L355 173 Z" };
  // gear letter with the optional shift hint (triangle + recommended gear)
  function drawGear(el, hintEl, triEl, txtEl, geo, gl, gCol, showHint, hintG, gActual) {
    var k = gl + (showHint ? "|h" : "");
    if (el.__k !== k) {
      // position / width change: re-fit even with unchanged text
      el.__k = k;
      el.__t = null;
      attr(el, "x", showHint ? geo.xHint : geo.x);
    }
    setText(el, gl, showHint ? geo.wHint : geo.w);
    attr(el, "fill", gCol);
    if (showHint) {
      attr(triEl, "d", hintG > gActual ? geo.up : geo.dn);
      attr(triEl, "fill", gCol);
      txt(txtEl, String(hintG));
      attr(txtEl, "fill", gCol);
    }
    attr(hintEl, "opacity", showHint ? "1" : "0");
  }

  // x = values derived by apply()
  function applyEfficiency(x) {
    var gd = x.gd;
    var miles = x.imperial;
    var mpg = units.cons !== "metric";
    var gal = units.vol === "gal";

    // EV range large, SOC compact (target marker in BATTERY CONTROL)
    txt($.efRngVal, x.rngTxt);
    txt($.efRngUnit, miles ? "mi" : "km");
    attr($.efRngLbl, "fill", x.evUsable ? C.eco : C.dim);
    attr($.efRng, "fill", x.evUsable ? C.text : C.muted);
    txt($.efSocVal, x.socPct === null ? "--" : String(Math.round(x.socPct)));
    attr($.efSocVal, "fill", x.low ? C.red : C.text);
    attr($.efSocBar, "width", x.socPct === null ? "0" : (54 * x.socPct / 100).toFixed(1));
    attr($.efSocBar, "fill", x.low ? C.red : "url(#gSoc)");
    attr($.efSocTgt, "opacity", x.showTgt ? "1" : "0");
    if (x.showTgt) { attr($.efSocTgt, "x", (131.5 + 54 * clamp01(x.tgt) - 1).toFixed(1)); }

    // phase + recuperation level (one-pedal hold first, as in the chips)
    txt($.efPhase, x.P.label);
    attr($.efPhase, "fill", x.P.color);
    setText($.efRegen, !x.hasVcu ? "" : (x.op ? "ONE PEDAL" : (x.rl ? "REGEN " + x.rl : "")), 76);
    attr($.efRegen, "fill", (x.rl === "OFF" && !x.op) ? C.muted : C.green);

    var trip = num(gd.tripKm);
    txt($.efTripVal, (trip === null || trip < 0) ? "--" : (miles ? trip / KM_PER_MI : trip).toFixed(1));
    txt($.efTripUnit, miles ? "mi" : "km");
    var kwh = num(gd.avgKwh);
    txt($.efAvgEVal, (x.hasVcu && kwh !== null && kwh > 0) ? (miles ? kwh * KM_PER_MI : kwh).toFixed(1) : "--");
    txt($.efAvgEUnit, miles ? "kWh/100mi" : "kWh/100km");
    var fuelL = num(gd.fuelL);
    var avg = num(gd.avgL100);
    txt($.efAvgFVal, (fuelL === null || fuelL < 0 || avg === null || avg < 0) ? "--" : (mpg ? mpgOf(avg) : avg.toFixed(1)));
    txt($.efAvgFUnit, mpg ? "mpg" : "L/100km");

    // emphasised demand caret + pedal zone above the gear (HYBRID caret gate)
    var dv = x.demOn;
    if (dv) {
      var sq = q(dem.s);
      var ln = radialD(sq, DEM_EF.ln0, DEM_EF.tip);
      attr($.efDemTri, "d", caretD(sq, DEM_EF));
      attr($.efDemTri, "fill", x.demFill);
      attr($.efDemBg, "d", ln);
      attr($.efDemLn, "d", ln);
      attr($.efDemLn, "stroke", x.demFill);
    }
    $.efDem.__vis = dv;
    $.efDem.__pulse = dv && x.zone === 2;
    setText($.efPedal, dv ? (PEDAL_ZONE[x.zone] || "") : "", 90);
    attr($.efPedal, "fill", x.demFill);

    setText($.efGear, x.gl, 22);
    attr($.efGear, "fill", x.gCol);

    // instantaneous consumption: fuel with the ICE running (l/100 km, l/h near
    // standstill), else battery power / speed (smoothed, < 0 = recuperation)
    var mode = str(gd.instMode) || "ev";
    var val = "--";
    var unit = "";
    var col = C.muted;
    var tc = now();
    if (x.on && mode === "lh") {
      var lh = Math.max(0, num(gd.instLh) || 0);
      val = (gal ? lh / L_PER_GAL : lh).toFixed(1);
      unit = gal ? "gal/h" : "L/h";
      col = C.amber;
    } else if (x.on && mode !== "ev") {
      var l100 = num(gd.instL100);
      if (l100 !== null && l100 >= 0) { val = mpg ? mpgOf(l100) : l100.toFixed(1); col = C.amber; }
      unit = mpg ? "mpg" : "L/100km";
    } else if (x.on) {
      unit = miles ? "kWh/100mi" : "kWh/100km";
      if (x.speed > efCons.minSpeed) {
        var raw = x.ev / (x.speed * 3.6) * 100;
        var dtc = (tc - efCons.t) / 1000;
        efCons.v = (efCons.t === 0 || dtc > 0.5) ? raw : efCons.v + (raw - efCons.v) * (1 - Math.exp(-dtc / efCons.tauS));
        efCons.t = tc;
        var cv = Math.max(-999, Math.min(999, miles ? efCons.v * KM_PER_MI : efCons.v));
        val = Math.abs(cv) < 0.05 ? "0.0" : (Math.abs(cv) >= 99.95 ? String(Math.round(cv)) : cv.toFixed(1));
        col = cv <= -0.05 ? C.green : C.blue;
      }
    }
    if (!(x.on && mode === "ev" && x.speed > efCons.minSpeed)) { efCons.t = 0; }
    setText($.efCons, val, 62);
    attr($.efCons, "fill", col);
    setText($.efConsUnit, unit, 52);
  }

  function applySport(x) {
    // stability mode, blinks (and turns amber) while DSC / DTC acts
    var ds = DSC_TXT.hasOwnProperty(x.dsc) ? DSC_TXT[x.dsc] : null;
    setText($.spDsc, !x.hasVcu ? "NO HYBRID DATA" : (ds ? ds[0] : ""), 80);
    attr($.spDsc, "fill", !x.hasVcu || !ds ? C.muted : (x.blinkDsc ? C.amber : ds[1]));
    $.spDsc.__blink = x.blinkDsc;

    // digital speed (game unit setting)
    setText($.spSpd, String(Math.round(x.speed * (x.imperial ? 2.236936 : 3.6))), 64);
    txt($.spSpdUnit, x.imperial ? "mph" : "km/h");

    // e-boost pips, label lit in phase eBOOST; SOC as the boost reserve
    for (var i = 1; i <= 3; i++) {
      attr($["spPip" + i], "fill", x.hasVcu && x.boost >= i ? C.orange : "#161b23");
    }
    attr($.spBoostLbl, "fill", x.pk === "BOOST" ? C.orange : (x.hasVcu && x.boost > 0 ? C.text : C.muted));
    txt($.spSocVal, x.socPct === null ? "--" : String(Math.round(x.socPct)));
    attr($.spSocVal, "fill", x.low ? C.red : (x.evAvail < 0.5 ? C.muted : C.blue));

    // ICE / EV split, each against its own peak (recuperation: regen peak)
    var iceKw = x.on ? Math.max(0, x.ice) : 0;
    var evKw = x.on ? x.ev : 0;
    attr($.spIceBar, "width", (46 * clamp01(iceKw / scale.ice)).toFixed(1));
    txt($.spIceKw, String(Math.round(iceKw)));
    attr($.spIceKw, "fill", iceKw < 0.5 ? C.muted : C.text);
    var rec = evKw < -0.5;
    attr($.spEvBar, "width", (46 * clamp01(rec ? -evKw / scale.regen : evKw / scale.ev)).toFixed(1));
    attr($.spEvBar, "fill", rec ? C.green : C.blue);
    txt($.spEvKw, String(Math.round(evKw) || 0));
    attr($.spEvKw, "fill", Math.abs(evKw) < 0.5 ? C.muted : (rec ? C.green : C.blue));

    // peak system power, held spPeak.holdMs
    var tp = now();
    var tot = x.on ? Math.max(0, x.total) : 0;
    if (tot >= spPeak.kw) { spPeak.kw = tot; spPeak.t = tp; }
    else if (tp - spPeak.t > spPeak.holdMs) { spPeak.kw = tot; }
    txt($.spPeakVal, String(Math.round(spPeak.kw)));

    // shift program, always (HYBRID hides it when it repeats the headline)
    var prog = str(x.e.etk854hybrid_shiftProgram);
    var progTxt = prog ? prog.toUpperCase().replace(/_/g, " ") : "";
    setText($.spProg, progTxt, 96);
    attr($.spProg, "fill", progColor(progTxt));

    // large gear with the shift hint
    drawGear($.spGear, $.spHint, $.spHintTri, $.spHintTxt, GEAR_GEO_S, x.gl, x.gCol, x.showHint, x.hintG, x.gActual);

    txt($.spKw, String(x.on ? Math.round(x.total) : 0));
    attr($.spKw, "fill", x.kwCol);
  }

  function applyMinimal(x) {
    txt($.mnSocVal, x.socPct === null ? "--" : String(Math.round(x.socPct)));
    attr($.mnSoc, "fill", x.low ? C.red : C.text);
    txt($.mnRngVal, x.rngTxt);
    txt($.mnRngUnit, x.imperial ? "mi" : "km");
    attr($.mnRngLbl, "fill", x.evUsable ? C.blue : C.dim);
    attr($.mnRng, "fill", x.evUsable ? C.text : C.muted);
  }

  // ---------------------------------------------------- HYBRID view blocks
  // Called by apply() in this order with the values it derived.

  // driving experience / EV mode
  function applyHeader(hasVcu, de, ed) {
    setText($.deMode, hasVcu ? (DE_LABEL[de] || (de ? de.replace(/_/g, " ") : "")) : "", 92);
    var deCol = DE_COLOR[de] || C.text;
    attr($.deMode, "fill", deCol);
    attr($.deRule, "fill", deCol);
    setText($.edMode, hasVcu ? (ED_LABEL[ed] || (ed ? ed.replace(/_/g, " ") : "")) : "NO HYBRID DATA", 80);
    attr($.edMode, "fill", hasVcu ? (ED_COLOR[ed] || C.blueLt) : C.muted);
    attr($.edMode, "font-weight", ed === "MAX_EDRIVE" ? "700" : "400");
  }

  function applySoc(socPct, low, showTgt, tgt) {
    txt($.socVal, socPct === null ? "--" : String(Math.round(socPct)));
    attr($.socBar, "width", socPct === null ? "0" : (54 * socPct / 100).toFixed(1));
    attr($.socBar, "fill", low ? C.red : "url(#gSoc)");
    attr($.soc, "fill", low ? C.red : C.text);
    attr($.socTgt, "opacity", showTgt ? "1" : "0");
    if (showTgt) { attr($.socTgt, "x", (131.5 + 54 * clamp01(tgt) - 1).toFixed(1)); }
  }

  function applyRange(rngTxt, imperial, evUsable) {
    txt($.rngVal, rngTxt);
    txt($.rngUnit, imperial ? "mi" : "km");
    attr($.rngLbl, "fill", evUsable ? C.blue : C.dim);
    attr($.rng, "fill", evUsable ? C.text : C.muted);
  }

  function applyPhase(P) {
    txt($.phase, P.label);
    attr($.phase, "fill", P.color);
  }

  // flow glyph: links, icons and the K0 clutch
  function applyFlow(on, sail, ice, ev, total, k0, iceOn, starting, socPct, low, speed) {
    var batOut = on && !sail && ev > 1;
    var batIn = on && !sail && ev < -1;
    var drvOut = on && !sail && total > 1;
    var drvIn = on && !sail && total < -1;
    var iceFlow = on && ice > 1 && k0 > 0.5;

    setLink($.lnBat, batOut ? "fwd" : (batIn ? "rev" : "idle"), batIn ? C.green : C.blue);
    var wheelCol = drvIn ? C.green : (ice > 1 ? (ev > 1 ? C.text : C.amber) : C.blue);
    setLink($.lnWhl, drvOut ? "fwd" : (drvIn ? "rev" : "idle"), wheelCol);
    setLink($.lnIceA, iceFlow ? "fwd" : "idle", C.amber);
    setLink($.lnIceB, iceFlow ? "fwd" : "idle", C.amber);

    var iceCol = starting ? C.blueLt : (on && iceOn ? C.amber : C.dim);
    setIcon($.icIce, iceCol, on && (iceOn || starting));
    $.icIce.__blink = starting;
    var emCol = batIn ? C.green : ((on && Math.abs(ev) > 0.5) ? C.blue : C.dim);
    attr($.icEmCircle, "stroke", emCol);
    attr($.icEmBolt, "fill", emCol);
    var batCol = batIn ? C.green : (batOut ? C.blue : C.dim);
    attr($.icBatBody, "stroke", batCol);
    attr($.icBatNub, "fill", batCol);
    attr($.icBatFill, "width", socPct === null ? "0" : (20 * socPct / 100).toFixed(1));
    attr($.icBatFill, "fill", low ? C.red : (batIn ? C.green : C.blue));
    var whlCol = (drvOut || drvIn) ? wheelCol : (sail && speed > 0.5 ? C.blueLt : C.dim);
    attr($.icWhl, "stroke", whlCol);

    // K0: 1 = closed (engine coupled), 0 = open
    var gap = 1.5 + (1 - k0) * 3;
    var k0Col = !on ? C.dim : (k0 > 0.9 ? (iceFlow ? C.amber : C.muted) : (k0 > 0.1 ? C.amber : C.dim));
    var yA = (178.5 - gap).toFixed(2);
    var yB = (178.5 + gap).toFixed(2);
    attr($.k0A, "y1", yA); attr($.k0A, "y2", yA);
    attr($.k0B, "y1", yB); attr($.k0B, "y2", yB);
    attr($.k0A, "stroke", k0Col);
    attr($.k0B, "stroke", k0Col);
    attr($.lnIceA, "y2", (178.5 - gap - 1.2).toFixed(2));
    attr($.lnIceB, "y1", (178.5 + gap + 1.2).toFixed(2));
  }

  // power meter bars + system power
  function applyMeter(on, ev, total, kwCol) {
    var sT = q(sOf(total));
    if (on && total > 0.5) {
      var sE = q(sOf(Math.max(0, Math.min(ev, total))));
      attr($.mBarE, "d", arcD(ARC.r, ARC.s0, sE));
      attr($.mBarI, "d", arcD(ARC.r, sE, sT));
      attr($.mBarR, "d", "M0 0");
    } else if (on && total < -0.5) {
      attr($.mBarE, "d", "M0 0");
      attr($.mBarI, "d", "M0 0");
      attr($.mBarR, "d", arcD(ARC.r, sT, ARC.s0));
    } else {
      attr($.mBarE, "d", "M0 0");
      attr($.mBarI, "d", "M0 0");
      attr($.mBarR, "d", "M0 0");
    }

    var kwShown = on ? Math.round(total) : 0;
    txt($.kw, String(kwShown));
    attr($.kw, "fill", kwCol);
  }

  // ECO PRO demand caret. zone 1: inside the EV zone by torque / knee; 2: on
  // the edge, pulsing; 3-5: beyond the edge by band position, amber while the
  // load integral (iceDeficitS) builds towards an ICE start. Smooths dem and
  // returns the caret colour.
  function applyDemand(demOn, zone, dEv, dBand, evLimit, iceOn, e, vw) {
    var tDem = now();
    var demFill = C.blueLt;
    if (demOn) {
      var sEdge = sOf(evLimit);
      var sTgt = zone === 1 ? ARC.s0 + (sEdge - ARC.s0) * dEv : (zone === 2 ? sEdge : sEdge + (1 - sEdge) * dBand);
      var dtDem = Math.min(0.5, Math.max(0, (tDem - dem.t) / 1000));
      dem.s = (dem.s === null || !dem.vis) ? sTgt : dem.s + (sTgt - dem.s) * (1 - Math.exp(-dtDem / DEM.tauS));
      attr($.mDem, "d", caretD(q(dem.s)));
      var defS = num(e.etk854hybrid_iceDeficitS) || 0;
      demFill = zone <= 2 ? C.blueLt : ((zone === 3 && defS > 0.02 && !iceOn) ? C.amber : C.text);
      attr($.mDem, "fill", demFill);
    }
    dem.t = tDem;
    dem.vis = demOn;
    if ($.mDem) {
      // the EFFICIENCY view draws its own, larger caret (efDem)
      $.mDem.__vis = demOn && vw !== "EFFICIENCY";
      $.mDem.__pulse = demOn && zone === 2;
    }
    return demFill;
  }

  // shift program + gear box (MINIMAL shows the same box)
  function applyGearBox(progTxt, gl, gCol, showHint, hintG, gActual) {
    setText($.prog, progTxt, 96);
    attr($.prog, "fill", progColor(progTxt));
    drawGear($.gear, $.gHint, $.gHintTri, $.gHintTxt, GEAR_GEO_H, gl, gCol, showHint, hintG, gActual);
  }

  // trailer air supply for the AIR chips, null = no chip. airState OFF | FILL |
  // READY | LOW | NO TANK; missing (0) -> no chip
  function airOf(e) {
    var airSt = str(e.etk854hybrid_airState);
    var air = null;
    if (airSt && airSt !== "NO TANK") {
      var airBar = num(e.etk854hybrid_airPressureBar);
      var spring = num(e.etk854hybrid_trailerSpringCoef);
      var pTxt = airBar === null ? "--" :
        (units.press === "psi" ? String(Math.round(Math.max(0, airBar) * 14.5038)) : Math.max(0, airBar).toFixed(1));
      air = {
        low: airSt === "LOW",
        trlBrk: (num(e.etk854hybrid_airTrailer) || 0) > 0.5 && spring !== null && spring > 0.5 &&
          (num(e.parkingbrake) || 0) < 0.5,
        run: (num(e.etk854hybrid_airCompressor) || 0) > 0.5,
        // protection valve (0 = shut); channel absent -> null = open
        shut: num(e.etk854hybrid_airTrlValve) === 0,
        label: str(e.etk854hybrid_airInhibit) === "SWITCH" ? "AIR OFF" : "AIR " + pTxt
      };
    }
    return air;
  }

  // bottom strip: odometer, clock, outside temperature
  function applyBottom(e, env, imperial) {
    var odoM = num(e.odometer);
    if (odoM !== null && odoM > 0) {
      var odo = Math.min(999999, Math.floor(odoM * (imperial ? 0.000621371 : 0.001)));
      txt($.odo, thousands(odo) + (imperial ? " mi" : " km"));
    } else {
      txt($.odo, "");
    }
    txt($.clock, str(env.time) || "");
    var tc = num(env.temperatureEnv);
    if (tc === null || tc > 99.9 || tc < -99.9) {
      txt($.temp, "");
    } else {
      var tf = units.temp === "f" ? tc * 9 / 5 + 32 : (units.temp === "k" ? tc + 273.15 : tc);
      txt($.temp, tf.toFixed(1) + (units.temp === "f" ? "\u00b0F" : (units.temp === "k" ? " K" : "\u00b0C")));
    }
  }

  // ------------------------------------------------------------ main apply
  function apply(d) {
    if (!ready || !d) { return; }
    var e = d.electrics || {};
    var cm = d.customModules || {};
    var env = cm.environmentData || {};
    var gd = cm.etk854hybrid_gaugeData || {};

    // ---------------- inputs
    var phRaw = str(e.etk854hybrid_phase);
    var hasVcu = phRaw !== null;
    var ign = num(e.ignitionLevel) || 0;
    var ice = hasVcu ? (num(e.etk854hybrid_icePower) || 0) : 0;
    var ev = hasVcu ? (num(e.etk854hybrid_evPower) || 0) : 0;
    var total = ice + ev;
    var iceOn = (num(e.etk854hybrid_iceState) || 0) > 0.5;
    var sail = hasVcu && ((num(e.etk854hybrid_sailActive) || 0) > 0.5 || phRaw === "SAIL");
    var speed = Math.abs(num(e.wheelspeed) || 0);
    var soc = hasVcu ? num(e.etk854hybrid_batterySoc) : null;
    var tgt = num(e.etk854hybrid_targetSoc);
    if (tgt !== null && tgt > 1.5) { tgt = tgt / 100; }
    var de = str(e.etk854hybrid_deMode);
    var ed = str(e.etk854hybrid_edriveMode);
    var boost = Math.max(0, Math.min(3, Math.round(num(e.etk854hybrid_boostStage) || 0)));
    var ies = str(e.etk854hybrid_iceEngageState);
    var k0 = num(e.etk854hybrid_k0ClutchRatio);
    if (!hasVcu) { k0 = 0; } else if (k0 === null) { k0 = iceOn ? 1 : 0; }
    k0 = clamp01(k0);
    var evAvail = num(e.etk854hybrid_evAvailable);
    if (evAvail === null) { evAvail = 1; }
    var evFrac = num(e.etk854hybrid_evPowerFraction);
    if (evFrac === null) { evFrac = 1; }
    var rl = str(e.etk854hybrid_regenLevel);
    var band = str(e.etk854hybrid_batteryBand);
    var dsc = str(e.etk854hybrid_dscMode);
    // qualified DSC lamp (gaugeData); the raw tcActive lights on any DSE didAct
    // and is only a fallback for the demo when dscLamp is left out
    var lampRaw = num(e.etk854hybrid_dscLamp);
    var dseActing = lampRaw !== null ? lampRaw > 0.5 : (num(e.etk854hybrid_tcActive) || 0) > 0.5;
    var awd = str(e.etk854hybrid_awdMode);
    var trl = (num(e.etk854hybrid_trailerMode) || 0) > 0.5;
    var lvl = str(e.etk854hybrid_levelState);
    var fan = Math.round(num(e.etk854hybrid_fanStage) || 0);
    var op = (num(e.etk854hybrid_onePedalHold) || 0) > 0.5;
    var vw = viewOf(e.etk854hybrid_clusterView);

    // ---------------- scale (Lua module, else boost-stage fallback)
    var iceKw = num(gd.iceKw);
    if (!(iceKw > 1)) { iceKw = DEFAULT_ICE_KW; }
    var evKw = num(gd.evKw);
    if (!(evKw > 1)) { evKw = STAGE_EV_KW[boost]; }
    var regenKw = num(gd.regenKw);
    if (!(regenKw > 1)) { regenKw = STAGE_REGEN_KW[boost]; }
    scale.ice = iceKw;
    scale.ev = evKw;
    scale.max = iceKw + evKw;
    scale.regen = regenKw;

    // electric-only capability: stage peak x battery derating (the VCU's
    // DE_CFG.<mode>.evCapScale is 1.00 in every mode)
    var evLimit = evKw * clamp01(evFrac);
    // BATTERY CONTROL never drives electrically while moving (VCU evTopKmh 0)
    var iceForced = de === "SPORT" || ed === "BATTERY_CONTROL";
    if (!hasVcu || evAvail < 0.5 || iceForced) { evLimit = 0; }
    // ECO PRO split map: grey above the ECO EV speed ceiling, only while the
    // map is live (pedalIceJoin > 0; a missing channel is 0)
    var ecoSplit = hasVcu && de === "ECO_PRO" && ed === "AUTO_EDRIVE" && (num(e.etk854hybrid_pedalIceJoin) || 0) > 0;
    var openRaw = (num(e.etk854hybrid_evZoneOpen) || 0) > 0.5;
    var tGate = now();
    if (!ecoSplit || openRaw === evGate.open) {
      evGate.since = 0;
      if (!ecoSplit) { evGate.open = true; }
    } else if (!evGate.since) {
      evGate.since = tGate;
    } else if (tGate - evGate.since >= evGate.holdMs) {
      evGate.open = openRaw;
      evGate.since = 0;
    }
    if (ecoSplit && !evGate.open) { evLimit = 0; }
    updateZones(evLimit, iceKw);

    // ---------------- derived values (pure), shared with the other views
    var socPct = soc === null ? null : Math.max(0, Math.min(100, soc));
    var low = socPct !== null && (socPct < 12 || band === "CRIT" || band === "LOW1");
    var showTgt = ed === "BATTERY_CONTROL" && tgt !== null;
    var rng = num(gd.rangeKm);
    var imperial = units.length !== "metric";
    var rngTxt = (!hasVcu || rng === null || rng < 0) ? "--" : String(Math.round(imperial ? rng * KM_TO_MI : rng));
    var evUsable = hasVcu && evAvail > 0.5;

    // phase
    var pk;
    if (!hasVcu) { pk = "NONE"; }
    else if (ign < 2) { pk = "OFF"; }
    else if (sail) { pk = "SAIL"; }
    else if (phRaw === "CHARGE") { pk = "CHARGE"; }
    else if (phRaw === "REGEN") { pk = "REGEN"; }
    // eBOOST only when the e-machine adds real power, not for torque fill
    else if (phRaw === "HYBRID") { pk = (ice > 10 && ev > Math.max(10, 0.25 * evKw)) ? "BOOST" : "HYBRID"; }
    else if (phRaw === "ICE") { pk = "ENGINE"; }
    else if (phRaw === "EV") { pk = (Math.abs(total) < 1 && speed < 0.5) ? "READY" : "EDRIVE"; }
    else { pk = iceOn ? "ENGINE" : "READY"; }
    var P = PHASE[pk];
    var on = hasVcu && ign >= 2;
    var starting = !!(ies && ICE_STARTING[ies]);
    var kwCol = !on || Math.abs(total) < 0.5 ? C.muted : (total < 0 ? C.green : (ice < 1 ? C.blue : C.text));
    // ECO PRO demand caret: hidden while recuperating, at zero demand and with
    // a grey EV zone
    var zone = Math.round(num(e.etk854hybrid_pedalZone) || 0);
    var dEv = clamp01(num(e.etk854hybrid_pedalEvFrac) || 0);
    var dBand = clamp01(num(e.etk854hybrid_pedalBandFrac) || 0);
    var demOn = on && ecoSplit && zone >= 1 && evLimit > 0 && phRaw !== "REGEN" && total > -0.5 &&
      !(zone === 1 && dEv < 0.01);
    // 8HP shift program (ECO / COMFORT / SPORT with "+" / "++", MANUAL), only
    // when it adds to the driving-experience headline
    var prog = str(e.etk854hybrid_shiftProgram);
    var progTxt = prog ? prog.toUpperCase().replace(/_/g, " ") : "";
    var deBase = DE_PROG_BASE[de];
    if (progTxt === deBase) { progTxt = ""; }
    var gActual = num(e.etk854hybrid_gearActual);
    var gl = gearLabel(e.gear, gActual, num(e.gearIndex), gd.gearStyle);
    var gc0 = gl.charAt(0);
    var gCol = gc0 === "M" || gc0 === "S" ? C.amber : C.text;
    // shift hint (0 = none, debounced in gaugeData), only next to a forward gear
    var hintG = Math.round(num(e.etk854hybrid_gearHint) || 0);
    var showHint = hintG >= 1 && gActual !== null && gActual >= 1 && hintG !== Math.round(gActual) &&
      (gc0 === "D" || gc0 === "S" || gc0 === "M");

    // ---------------- HYBRID view (also the shared parts)
    applyHeader(hasVcu, de, ed);
    applySoc(socPct, low, showTgt, tgt);
    applyRange(rngTxt, imperial, evUsable);
    applyPhase(P);
    applyFlow(on, sail, ice, ev, total, k0, iceOn, starting, socPct, low, speed);
    applyMeter(on, ev, total, kwCol);
    var demFill = applyDemand(demOn, zone, dEv, dBand, evLimit, iceOn, e, vw);
    applyGearBox(progTxt, gl, gCol, showHint, hintG, gActual);
    var air = airOf(e);

    // ---------------- chips
    var blinkDsc = dseActing && on;
    var v = {
      hasVcu: hasVcu, de: de, ed: ed, rl: rl, awd: awd, trl: trl, dsc: dsc, boost: boost,
      tgt: tgt, band: band, op: op, fan: fan, iesStart: starting,
      lvlAlert: !!(lvl && LEVEL_ALERT[lvl]), air: air
    };
    var chips = hasVcu ? chipsFor(vw, chipList(v, blinkDsc)) : [];
    setChip(0, chips[0] || null);
    setChip(1, chips[1] || null);

    applyObc(gd, on, hasVcu);
    applyBottom(e, env, imperial);

    // ---------------- views: HYBRID elements always update (shared parts,
    // toast); the others only while shown, before they become visible
    if (vw !== "HYBRID") {
      var x = {
        e: e, gd: gd, hasVcu: hasVcu, on: on, ice: ice, ev: ev, total: total, speed: speed, imperial: imperial,
        socPct: socPct, low: low, tgt: tgt, showTgt: showTgt, evUsable: evUsable, evAvail: evAvail, rngTxt: rngTxt,
        P: P, pk: pk, rl: rl, op: op, dsc: dsc, blinkDsc: blinkDsc, boost: boost,
        gl: gl, gCol: gCol, gActual: gActual, hintG: hintG, showHint: showHint,
        demOn: demOn, zone: zone, demFill: demFill, kwCol: kwCol
      };
      if (vw === "EFFICIENCY") { applyEfficiency(x); }
      else if (vw === "SPORT") { applySport(x); }
      else { applyMinimal(x); }
    }
    setView(vw);

    detectChanges(v);
    animate();
  }

  // ------------------------------------------------------------ Lua entry
  function doSetup(cfg) {
    cfg = cfg || {};
    if (typeof cfg.uiUnitLength === "string") { units.length = cfg.uiUnitLength; }
    if (typeof cfg.uiUnitTemperature === "string") { units.temp = cfg.uiUnitTemperature.toLowerCase(); }
    if (typeof cfg.uiUnitConsumptionRate === "string") { units.cons = cfg.uiUnitConsumptionRate; }
    if (typeof cfg.uiUnitVolume === "string") { units.vol = cfg.uiUnitVolume; }
    if (typeof cfg.uiUnitPressure === "string") { units.press = cfg.uiUnitPressure; }
  }

  var demo = null; // demo driver, see bottom
  var demoGearStyle = "number"; // #gear=plain in the demo URL
  var demoView = ""; // &view=<VIEW> in the demo URL ("" = channel absent)

  function stopDemo() {
    if (demo) { clearInterval(demo); demo = null; }
  }

  function init() {
    var ids = [
      "deMode", "deRule", "edMode", "mid", "soc", "socVal", "socBar", "socTgt", "rngLbl", "rng", "rngVal", "rngUnit",
      "phase", "toast", "toastSub", "toastTitle", "toastInfo",
      "lnIceA", "lnIceB", "lnBat", "lnWhl", "k0A", "k0B", "icIce", "icEmBolt", "icBatFill", "icWhl",
      "odo", "clock", "temp", "prog", "gear", "gHint", "gHintTri", "gHintTxt", "kw",
      "chip1", "chip1Box", "chip1Txt", "chip1Dot", "chip2", "chip2Box", "chip2Txt", "chip2Dot",
      "obc", "fiTrack", "fiBar", "fiVal", "fiUnit", "obcL", "obcR", "fuelVal", "fuelUnit", "frngVal", "frngUnit",
      "avgF", "avgFU", "avgE", "avgEU", "avgPer",
      "mTrack", "zCharge", "zEdrive", "zPower", "zBoost", "mBarE", "mBarI", "mBarR", "mTicks", "mLblPath", "mLblCPath",
      "mDem",
      // EFFICIENCY / SPORT / MINIMAL views
      "efMid", "efRngLbl", "efRng", "efRngVal", "efRngUnit", "efSocBar", "efSocTgt", "efSocVal", "efPhase", "efRegen",
      "efTripVal", "efTripUnit", "efAvgEVal", "efAvgEUnit", "efAvgFVal", "efAvgFUnit",
      "efPedal", "efGear", "efCons", "efConsUnit", "efDem", "efDemBg", "efDemLn", "efDemTri",
      "spDsc", "spMid", "spSpd", "spSpdUnit", "spBoostLbl", "spPip1", "spPip2", "spPip3", "spSocVal",
      "spIceBar", "spIceKw", "spEvBar", "spEvKw", "spPeakVal", "spProg", "spGear", "spHint", "spHintTri", "spHintTxt", "spKw",
      "mnMid", "mnSoc", "mnSocVal", "mnRngLbl", "mnRng", "mnRngVal", "mnRngUnit"
    ];
    for (var i = 0; i < ids.length; i++) { $[ids[i]] = document.getElementById(ids[i]); }
    $.icEmCircle = document.querySelector("#icEm circle");
    $.icBatBody = document.querySelector("#icBat rect");
    $.icBatNub = document.querySelectorAll("#icBat rect")[1];
    var vl = document.querySelectorAll("#etk-cluster [data-views]");
    for (var vi = 0; vi < vl.length; vi++) { viewEls.push(vl[vi]); }
    buildMeter();
    ready = true;

    window.setup = function (c) { stopDemo(); doSetup(c); };
    window.updateData = function (d) { stopDemo(); apply(d); };
    window.updateMode = function (p) {
      if (p && p.modeName) { toast(String(p.modeName).toUpperCase(), "DRIVE MODE", p.modeColor || C.text); }
    };
    window.messageQueue = function (m) {
      if (m && m.text) { toast(String(m.text), "", m.highlightColor || C.text, "", m.durationSec); }
    };
    window.messageClearById = function () {};
    window.messageClearAll = function () {
      if (toastTimer) { clearTimeout(toastTimer); toastTimer = null; }
      attr($.toast, "opacity", "0");
      midOpacity("1");
    };

    // flush calls that arrived before this script ran
    var bq = window.__etk854q;
    if (bq) {
      if (bq.setup) { doSetup(bq.setup); }
      if (bq.data) { apply(bq.data); }
      if (bq.mode) { window.updateMode(bq.mode); }
      for (var k = 0; k < bq.msgs.length; k++) { window.messageQueue(bq.msgs[k]); }
      window.__etk854q = null;
    }

    // demo only in a desktop browser (in game: local:// and `beamng`)
    var inGame = /^local:/i.test(String(window.location.href)) || typeof beamng !== "undefined";
    var hash = String(window.location.hash || "");
    if ((!inGame || hash.indexOf("demo") >= 0) && !(bq && bq.data)) {
      startDemo(hash);
    }
  }

  // =================================================================== DEMO
  // Desktop preview: same data shape as genericGauges; #demo=<state> pins one.
  function demoFrame(state, t) {
    var E = {
      gear: "D", gearIndex: 3, odometer: 12873400, ignitionLevel: 2, wheelspeed: 14,
      etk854hybrid_batterySoc: 78, etk854hybrid_targetSoc: 0.9, etk854hybrid_batteryBand: "NORMAL",
      etk854hybrid_phase: "EV", etk854hybrid_deMode: "COMFORT", etk854hybrid_edriveMode: "AUTO_EDRIVE",
      etk854hybrid_evPower: 0, etk854hybrid_icePower: 0, etk854hybrid_iceState: 0,
      etk854hybrid_iceEngageState: "OFF", etk854hybrid_k0ClutchRatio: 0,
      etk854hybrid_regenLevel: "MED", etk854hybrid_onePedalHold: 0, etk854hybrid_boostStage: 0,
      etk854hybrid_shiftProgram: "COMFORT", etk854hybrid_evAvailable: 1, etk854hybrid_evPowerFraction: 1,
      etk854hybrid_sailActive: 0, etk854hybrid_chargeMode: "OFF", etk854hybrid_awdMode: 0,
      etk854hybrid_trailerMode: 0, etk854hybrid_levelState: "LEVEL", etk854hybrid_dscMode: "DSC",
      etk854hybrid_tcActive: 0, etk854hybrid_fanStage: 0, etk854hybrid_dscLamp: 0, etk854hybrid_gearActual: 3,
      // trunk compressor not fitted: channels arrive as 0
      etk854hybrid_airState: 0, etk854hybrid_airPressureBar: 0, etk854hybrid_airCompressor: 0,
      etk854hybrid_airInhibit: 0, etk854hybrid_airTrailer: 0, etk854hybrid_trailerSpringCoef: 0, parkingbrake: 0
    };
    // on-board computer defaults: electric driving, trip in progress
    var F = { fuelL: 38.4, fuelCapL: 50, fuelRangeKm: 480, instMode: "ev", instL100: 0, instLh: 0, instBar: 0,
      avgL100: 6.8, tripKm: 128.4, avgKwh: 19.6, tripId: 0 };
    function burn(l100) {
      F.instMode = "l100"; F.instL100 = l100; F.instLh = l100 * E.wheelspeed * 0.036; F.instBar = Math.min(1, l100 / 30);
    }
    // ECO PRO pedal map at pedal p (R6, 2000 rpm, stage 0): knee 265 Nm, band
    // top 680 Nm; ICE off, EV power ~ torque at 2000 rpm
    function ecoMap(p) {
      var knee = 265, top = 680, pEv = 0.80, pJ = 0.85, kick = 0.90, urg = 0.95;
      var T = p <= pEv ? knee * Math.pow(p / pEv, 1.2) : (p < pJ ? knee : knee + (top - knee) * (Math.min(p, 1) - pJ) / (1 - pJ));
      E.etk854hybrid_deMode = "ECO_PRO"; E.etk854hybrid_shiftProgram = "ECO"; E.etk854hybrid_regenLevel = "HIGH";
      E.throttle = p;
      E.etk854hybrid_pedalIceJoin = pJ;
      E.etk854hybrid_pedalZone = p <= pEv ? 1 : (p < pJ ? 2 : (p < kick ? 3 : (p < urg ? 4 : 5)));
      E.etk854hybrid_pedalEvFrac = Math.min(1, T / knee);
      E.etk854hybrid_pedalBandFrac = T > knee ? (T - knee) / (top - knee) : 0;
      E.etk854hybrid_evZoneOpen = 1;
      E.etk854hybrid_evPower = Math.min(83, T * 0.2094);
      // load integral: builds only in the band (r > 1); ECO onCalm ~0.8
      E.etk854hybrid_iceDeficitOnS = 0.8;
      E.etk854hybrid_iceDeficitS = E.etk854hybrid_pedalZone >= 3 ? Math.min(0.79, 0.25 + (p - pJ) * 8) : 0;
    }
    var s = state;
    if (s === "anim") {
      var cyc = t % 44;
      s = cyc < 4 ? "ready" : cyc < 11 ? "ev" : cyc < 15 ? "sail" : cyc < 19 ? "regen" : cyc < 22 ? "start" :
        cyc < 28 ? "boost" : cyc < 32 ? "ice" : cyc < 36 ? "charge" : cyc < 40 ? "sport" : "bc";
    }
    var w = Math.sin(t * 1.3) * 0.5 + 0.5;
    switch (s) {
      case "ready":
        E.gear = "P"; E.gearIndex = 0; E.wheelspeed = 0; E.etk854hybrid_phase = "EV";
        break;
      case "ev":
        E.etk854hybrid_evPower = 35 + 30 * w; E.gearIndex = 3; E.wheelspeed = 13;
        break;
      case "maxed":
        E.etk854hybrid_edriveMode = "MAX_EDRIVE"; E.etk854hybrid_evPower = 70 + 10 * w; E.gearIndex = 5; E.wheelspeed = 24;
        break;
      case "sail":
        E.etk854hybrid_phase = "SAIL"; E.etk854hybrid_sailActive = 1; E.gearIndex = 6; E.wheelspeed = 25;
        break;
      case "regen":
        E.etk854hybrid_phase = "REGEN"; E.etk854hybrid_evPower = -28 - 18 * w; E.gearIndex = 4; E.wheelspeed = 18;
        E.etk854hybrid_batterySoc = 79;
        break;
      case "start":
        E.etk854hybrid_iceEngageState = "SYNC"; E.etk854hybrid_iceState = 1; E.etk854hybrid_k0ClutchRatio = 0.35;
        E.etk854hybrid_evPower = 60; E.gearIndex = 4; E.wheelspeed = 19;
        break;
      case "hybrid":
        E.etk854hybrid_phase = "HYBRID"; E.etk854hybrid_iceState = 1; E.etk854hybrid_iceEngageState = "ON";
        E.etk854hybrid_k0ClutchRatio = 1; E.etk854hybrid_icePower = 95 + 10 * w; E.etk854hybrid_evPower = 18;
        E.gearIndex = 5; E.wheelspeed = 27;
        burn(8.9 + 1.5 * w);
        break;
      case "boost":
        E.etk854hybrid_phase = "HYBRID"; E.etk854hybrid_iceState = 1; E.etk854hybrid_iceEngageState = "ON";
        E.etk854hybrid_k0ClutchRatio = 1; E.etk854hybrid_icePower = 205 + 30 * w; E.etk854hybrid_evPower = 75 + 8 * w;
        E.gear = "S3"; E.gearIndex = 3; E.wheelspeed = 30; E.etk854hybrid_shiftProgram = "SPORT++";
        E.etk854hybrid_deMode = "SPORT";
        burn(26 + 3 * w);
        break;
      case "ice":
        E.etk854hybrid_phase = "ICE"; E.etk854hybrid_iceState = 1; E.etk854hybrid_iceEngageState = "ON";
        E.etk854hybrid_k0ClutchRatio = 1; E.etk854hybrid_icePower = 45 + 10 * w; E.gearIndex = 8; E.wheelspeed = 36;
        E.etk854hybrid_batterySoc = 18; E.etk854hybrid_batteryBand = "NORMAL_LO";
        burn(7.2 + 0.8 * w);
        break;
      case "charge":
        E.etk854hybrid_phase = "CHARGE"; E.etk854hybrid_iceState = 1; E.etk854hybrid_iceEngageState = "ON";
        E.etk854hybrid_k0ClutchRatio = 1; E.etk854hybrid_icePower = 70 + 5 * w; E.etk854hybrid_evPower = -24;
        E.etk854hybrid_edriveMode = "BATTERY_CONTROL"; E.etk854hybrid_batterySoc = 54; E.gearIndex = 7; E.wheelspeed = 30;
        E.etk854hybrid_chargeMode = "CRUISE";
        burn(10.4 + 0.6 * w);
        break;
      case "bc":
        // target reached: engine carries the car, battery is held
        E.etk854hybrid_edriveMode = "BATTERY_CONTROL"; E.etk854hybrid_targetSoc = 0.6; E.etk854hybrid_batterySoc = 62;
        E.etk854hybrid_phase = "ICE"; E.etk854hybrid_iceState = 1; E.etk854hybrid_iceEngageState = "ON";
        E.etk854hybrid_k0ClutchRatio = 1; E.etk854hybrid_icePower = 38 + 8 * w; E.etk854hybrid_evPower = 0;
        E.gearIndex = 6; E.wheelspeed = 22;
        burn(6.1 + 0.7 * w);
        break;
      case "sport":
        E.etk854hybrid_deMode = "SPORT"; E.etk854hybrid_shiftProgram = "SPORT+"; E.etk854hybrid_phase = "ICE";
        E.etk854hybrid_iceState = 1; E.etk854hybrid_iceEngageState = "ON"; E.etk854hybrid_k0ClutchRatio = 1;
        E.etk854hybrid_icePower = 120 + 40 * w; E.etk854hybrid_dscMode = "DTC"; E.gear = "S4"; E.gearIndex = 4;
        E.etk854hybrid_trailerMode = 1; E.wheelspeed = 28;
        burn(15.8 + 4 * w);
        break;
      case "eco":
        // pedal sweep 0.25 .. 0.95 (10 s): caret through zone, plateau, band
        var tri = (t % 10) / 5;
        ecoMap(0.25 + 0.70 * (tri < 1 ? tri : 2 - tri));
        E.gearIndex = 6; E.wheelspeed = 22;
        break;
      case "eco50":
        // EV zone, 79 km/h: 151 Nm, caret ~57 % into the blue zone
        ecoMap(0.50); E.gearIndex = 6; E.wheelspeed = 22;
        break;
      case "eco82":
        // EV MAX plateau: caret pulses on the zone edge
        ecoMap(0.82); E.etk854hybrid_evPower = 72; E.gearIndex = 4; E.wheelspeed = 16.7;
        break;
      case "eco88":
        // hybrid band, ICE still off: the load integral builds (amber caret)
        ecoMap(0.88); E.etk854hybrid_evPower = 83; E.gearIndex = 4; E.wheelspeed = 18;
        break;
      case "off":
        E.ignitionLevel = 1; E.gear = "P"; E.gearIndex = 0; E.wheelspeed = 0;
        break;
      case "nodata":
        for (var k in E) { if (k.indexOf("etk854hybrid_") === 0) { E[k] = 0; } }
        break;
      case "dsc":
        // qualified DSC lamp: the chip blinks
        E.etk854hybrid_dscLamp = 1; E.etk854hybrid_tcActive = 1; E.etk854hybrid_evPower = 60; E.gearIndex = 2; E.wheelspeed = 9;
        break;
      case "manual":
        // M3 revved high: the program recommends 4th (triangle up + 4)
        E.gear = "M3"; E.gearIndex = 3; E.etk854hybrid_shiftProgram = "MANUAL"; E.etk854hybrid_phase = "ICE";
        E.etk854hybrid_iceState = 1; E.etk854hybrid_iceEngageState = "ON"; E.etk854hybrid_k0ClutchRatio = 1;
        E.etk854hybrid_icePower = 90; E.wheelspeed = 22; E.etk854hybrid_gearHint = 4;
        break;
      case "hintdn":
        // D6 while the program plans 4th (e.g. held by an ICE start)
        E.gearIndex = 6; E.wheelspeed = 20; E.etk854hybrid_evPower = 70; E.etk854hybrid_gearHint = 4;
        break;
      case "air":
        // compressor filling, dolly air-coupled (AIR 7.4 + dot)
        E.etk854hybrid_trailerMode = 1; E.etk854hybrid_airState = "FILL"; E.etk854hybrid_airPressureBar = 7.4 + 0.1 * w;
        E.etk854hybrid_airCompressor = 1; E.etk854hybrid_airInhibit = "NONE"; E.etk854hybrid_airTrailer = 1;
        E.etk854hybrid_trailerSpringCoef = 0; E.etk854hybrid_evPower = 22; E.wheelspeed = 8; E.gearIndex = 2;
        break;
      case "airlow":
        // receiver below 4.14 bar: trailer spring brakes not fully released
        E.gear = "P"; E.gearIndex = 0; E.wheelspeed = 0; E.etk854hybrid_trailerMode = 1;
        E.etk854hybrid_airState = "LOW"; E.etk854hybrid_airPressureBar = 3.1 + 0.2 * w; E.etk854hybrid_airCompressor = 1;
        E.etk854hybrid_airInhibit = "NONE"; E.etk854hybrid_airTrailer = 1; E.etk854hybrid_trailerSpringCoef = 0.3;
        break;
      case "trlbrk":
        // receiver fine, trailer still filling its spring actuators
        E.gear = "P"; E.gearIndex = 0; E.wheelspeed = 0; E.etk854hybrid_airState = "READY";
        E.etk854hybrid_airPressureBar = 8.3; E.etk854hybrid_airInhibit = "NONE"; E.etk854hybrid_airTrailer = 1;
        E.etk854hybrid_trailerSpringCoef = 0.8;
        break;
      case "airshut":
        // protection valve still shut: amber AIR outline; parked: no TRL BRK
        E.gear = "P"; E.gearIndex = 0; E.wheelspeed = 0; E.parkingbrake = 1; E.etk854hybrid_trailerMode = 1;
        E.etk854hybrid_airState = "FILL"; E.etk854hybrid_airPressureBar = 5.1 + 0.1 * w; E.etk854hybrid_airCompressor = 1;
        E.etk854hybrid_airInhibit = "NONE"; E.etk854hybrid_airTrailer = 1; E.etk854hybrid_trailerSpringCoef = 1;
        E.etk854hybrid_airTrlValve = 0;
        break;
      case "fuel":
        // engine cruise: instantaneous l/100 km, trip averages, compressor ready
        E.etk854hybrid_phase = "ICE"; E.etk854hybrid_iceState = 1; E.etk854hybrid_iceEngageState = "ON";
        E.etk854hybrid_k0ClutchRatio = 1; E.etk854hybrid_icePower = 32 + 6 * w; E.gearIndex = 8; E.wheelspeed = 36;
        E.etk854hybrid_airState = "READY"; E.etk854hybrid_airPressureBar = 8.4; E.etk854hybrid_airInhibit = "NONE";
        burn(6.3 + 1.2 * w);
        break;
      case "idle":
        // standstill charging with the ICE idling: l/h instead of l/100 km
        E.gear = "P"; E.gearIndex = 0; E.wheelspeed = 0; E.etk854hybrid_phase = "CHARGE"; E.etk854hybrid_iceState = 1;
        E.etk854hybrid_iceEngageState = "ON"; E.etk854hybrid_k0ClutchRatio = 1; E.etk854hybrid_icePower = 18;
        E.etk854hybrid_evPower = -16; E.etk854hybrid_chargeMode = "PARK";
        F.instMode = "lh"; F.instL100 = -1; F.instLh = 1.6 + 0.2 * w; F.instBar = F.instLh / 10;
        break;
      case "imperial":
        // US units (mpg, gal, mi, psi): see startDemo
        E.etk854hybrid_phase = "HYBRID"; E.etk854hybrid_iceState = 1; E.etk854hybrid_iceEngageState = "ON";
        E.etk854hybrid_k0ClutchRatio = 1; E.etk854hybrid_icePower = 70; E.etk854hybrid_evPower = 15;
        E.gearIndex = 7; E.wheelspeed = 29; E.etk854hybrid_airState = "READY"; E.etk854hybrid_airPressureBar = 8.4;
        E.etk854hybrid_airInhibit = "NONE";
        burn(7.8);
        break;
      default:
        break;
    }
    // demo: the engaged gear equals the decided one; P/N have gearbox index 0
    E.etk854hybrid_gearActual = (E.gear === "P" || E.gear === "N") ? 0 : E.gearIndex;
    // AUTO resolves as the cockpit panel does in game
    if (demoView) {
      E.etk854hybrid_clusterView = demoView !== "AUTO" ? demoView :
        ({ ECO_PRO: "EFFICIENCY", SPORT: "SPORT" }[E.etk854hybrid_deMode] || "HYBRID");
    }
    return {
      electrics: E,
      powertrain: {},
      customModules: {
        environmentData: { time: "21:47", temperatureEnv: 17.5 },
        etk854hybrid_gaugeData: {
          rangeKm: state === "nodata" ? -1 : 41.3 * E.etk854hybrid_batterySoc / 78, consKwh: 21,
          iceKw: 250, evKw: 83, regenKw: 55, gearStyle: demoGearStyle,
          fuelL: state === "nodata" ? -1 : F.fuelL, fuelCapL: F.fuelCapL, fuelRangeKm: F.fuelRangeKm,
          instMode: F.instMode, instL100: F.instL100, instLh: F.instLh, instBar: F.instBar,
          avgL100: F.avgL100, tripKm: F.tripKm, avgKwh: F.avgKwh, tripId: F.tripId
        }
      }
    };
  }

  function startDemo(hash) {
    var m = /demo=([a-z0-9]+)/.exec(hash);
    var state = m ? m[1] : "anim";
    demoGearStyle = /gear=plain/.test(hash) ? "plain" : "number";
    var vm = /view=([A-Za-z]+)/.exec(hash);
    demoView = vm ? vm[1].toUpperCase() : "";
    doSetup(state === "imperial"
      ? { uiUnitLength: "imperial", uiUnitTemperature: "f", uiUnitConsumptionRate: "imperial", uiUnitVolume: "gal", uiUnitPressure: "psi" }
      : { uiUnitLength: "metric", uiUnitTemperature: "c", uiUnitConsumptionRate: "metric", uiUnitVolume: "l", uiUnitPressure: "bar" });
    var t0 = now();
    var toasted = false;
    demo = setInterval(function () {
      var t = (now() - t0) / 1000;
      apply(demoFrame(state, t));
      if (state === "toast" && !toasted && t > 0.2) {
        toasted = true;
        toast("MAX EV", "EV", C.blue, "");
      }
    }, 33);
    apply(demoFrame(state, 0));
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", init);
  } else {
    init();
  }
})();
