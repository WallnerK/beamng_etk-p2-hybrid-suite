// ETK 854 ActiveHybrid - centre screen energy-flow page logic
// =========================================================================
// Target: BeamNG.drive 0.39.4, web view "@etk800_screen" (512 x 256)
//
// Input: etk854hybridUpdate(d) at ~15 Hz, d built by
// lua/controller/etk854hybrid_screenData.lua (short keys, any may be missing):
//   soc tgt ph de ed ev ice evNm iceNm k0 ies on sail chg awd awdNm trl lvl
//   fan boost evAv evFr rl op dsc prog spd gear gi ign rng cons cap use
//   iceMax fuel amb wt uL uT uC uV fl fcap frng fi fh fm fa tk
// uL / uT = game unit settings (uiUnitLength / uiUnitTemperature); imperial
// shows range in mi, consumption in kWh/100 mi, and the temperature follows uT.
// On-board computer (etk854hybrid_gaugeData item 10, the same numbers as the
// cluster): fl litres in the tank pictogram, fi / fh / fm instantaneous
// consumption in the ENGINE row, fa trip average + tk trip distance under the
// electric average. uC (uiUnitConsumptionRate) "metric" -> L/100 km, else
// US mpg; uV (uiUnitVolume) "gal" -> US gallons.
// map.updateData(g) from the stock beamNavigator supplies ignitionLevel too.
//
// History: electric / engine / recovered energy integrated from the power
// values between two updates, 30 buckets x 30 s = the last 15 minutes of
// driving (time only runs while updates arrive, i.e. while seated).
// =========================================================================
(function () {
  "use strict";

  var C = {
    text: "#e6ebf2",
    muted: "#8b95a5",
    faint: "#5d6778",
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

  var STAGE_EV_KW = [83, 110, 150, 200];
  var STAGE_REGEN_KW = [55, 65, 80, 95];
  var DE_LABEL = { COMFORT: "COMFORT", ECO_PRO: "ECO PRO", SPORT: "SPORT" };
  var ED_LABEL = { AUTO_EDRIVE: "AUTO EV", MAX_EDRIVE: "MAX EV", BATTERY_CONTROL: "BATTERY CONTROL" };
  var PHASE = {
    READY: ["READY", C.blueLt],
    EDRIVE: ["ELECTRIC", C.blue],
    HYBRID: ["HYBRID", C.amber],
    BOOST: ["eBOOST", C.orange],
    ENGINE: ["ENGINE", C.amber],
    SAIL: ["SAILING", C.blueLt],
    REGEN: ["RECUPERATION", C.green],
    CHARGE: ["CHARGING", C.green]
  };
  var ICE_STARTING = { CRANK: 1, SYNC: 1, ENGAGE: 1 };

  var BUCKETS = 30;
  var BUCKET_S = 30;
  var HX0 = 60, HX1 = 338, HBASE = 232, HUP = 24, HDOWN = 14;

  var $ = {};
  var ready = false;
  var ignNav = null;
  var lastT = 0;
  var hist = [];
  var cur = { ev: 0, ice: 0, rec: 0, t: 0 };
  var histDirty = true;
  var chipSig = "";
  var demo = null;

  function now() {
    return (window.performance && performance.now) ? performance.now() : Date.now();
  }
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
  function fitText(el, maxW) {
    el.removeAttribute("textLength");
    el.removeAttribute("lengthAdjust");
    var w = 0;
    try { w = el.getComputedTextLength(); } catch (e) { w = 0; }
    if (w > maxW) {
      el.setAttribute("textLength", maxW);
      el.setAttribute("lengthAdjust", "spacingAndGlyphs");
    }
    return Math.min(w, maxW);
  }
  function flow(el, mode, color) {
    attr(el, "class", "flow " + mode);
    if (!el.__isFlow) { el.__isFlow = true; flowEls.push(el); }
    el.__mode = mode;
    if (mode !== "idle") { attr(el, "stroke", color); }
  }

  // Script-driven animation instead of CSS keyframes: in BeamNG 0.39.4 a
  // vehicle HTML texture stopped showing DOM updates once an infinite CSS
  // animation ran (the energy-flow dashes start as soon as the car drives),
  // so the screen froze and a blinking chip went on blinking by itself
  // (reproduced on the cluster page 2026-10). Called once per data frame.
  var flowEls = [];
  var blinkEls = [];
  var FLOW_PERIOD_MS = 600;   // one dash period (11 px) per 0.6 s, as the old CSS
  var BLINK_PERIOD_MS = 500;
  function animate() {
    var t = now();
    var ph = (t % FLOW_PERIOD_MS) / FLOW_PERIOD_MS;
    for (var i = 0; i < flowEls.length; i++) {
      var el = flowEls[i];
      var m = el.__mode;
      var off = m === "fwd" ? 11 * (1 - ph) : (m === "rev" ? 11 * ph : 0);
      attr(el, "stroke-dashoffset", off.toFixed(1));
    }
    var lit = (t % BLINK_PERIOD_MS) < BLINK_PERIOD_MS / 2;
    for (var j = 0; j < blinkEls.length; j++) {
      var g = blinkEls[j];
      attr(g, "opacity", g.__blink && !lit ? "0.3" : "1");
    }
  }
  function lit(el, color, on) {
    attr(el, "stroke", color);
    attr(el, "fill", on ? "#101722" : C.dark);
  }
  function kw(v) {
    return Math.round(v) + " kW";
  }
  var KM_TO_MI = 0.621371;
  var L_PER_GAL = 3.785411784;
  var MPG_L100 = 235.214583;
  function mpgOf(l100) {
    return l100 < 0.5 ? "99+" : String(Math.min(99, Math.round(MPG_L100 / l100)));
  }
  function fmtTemp(c, unit) {
    var u = (typeof unit === "string") ? unit.toLowerCase() : "c";
    if (u === "f") { return (c * 9 / 5 + 32).toFixed(1) + "\u00b0F"; }
    if (u === "k") { return (c + 273.15).toFixed(1) + " K"; }
    return c.toFixed(1) + "\u00b0C";
  }

  // ---------------------------------------------------------------- history
  function histPush(ev, ice, dtS) {
    var h = dtS / 3600; // kW * h = kWh
    if (ev > 0) { cur.ev += ev * h; } else { cur.rec += -ev * h; }
    if (ice > 0) { cur.ice += ice * h; }
    cur.t += dtS;
    if (cur.t >= BUCKET_S) {
      hist.push({ ev: cur.ev, ice: cur.ice, rec: cur.rec });
      if (hist.length > BUCKETS - 1) { hist.shift(); }
      cur = { ev: 0, ice: 0, rec: 0, t: 0 };
    }
    histDirty = true;
  }

  function drawHist() {
    if (!histDirty) { return; }
    histDirty = false;
    var all = hist.concat([cur]);
    var maxUp = 0.02, maxDn = 0.02; // kWh per bucket, floor for the scale
    var sumEv = 0, sumIce = 0;
    for (var i = 0; i < all.length; i++) {
      maxUp = Math.max(maxUp, all[i].ev + all[i].ice);
      maxDn = Math.max(maxDn, all[i].rec);
      sumEv += all[i].ev;
      sumIce += all[i].ice;
    }
    var slot = (HX1 - HX0) / BUCKETS;
    var bw = Math.max(2, slot - 2.2);
    var g = $.hist;
    var ns = "http://www.w3.org/2000/svg";
    while (g.firstChild) { g.removeChild(g.firstChild); }
    var off = BUCKETS - all.length;
    for (var j = 0; j < all.length; j++) {
      var b = all[j];
      var x = HX0 + (off + j) * slot + 1.1;
      var hEv = HUP * b.ev / maxUp;
      var hIce = HUP * b.ice / maxUp;
      var hRec = HDOWN * b.rec / maxDn;
      if (hEv > 0.3) { g.appendChild(bar(ns, x, HBASE - hEv, bw, hEv, C.blue)); }
      if (hIce > 0.3) { g.appendChild(bar(ns, x, HBASE - hEv - hIce, bw, hIce, C.amber)); }
      if (hRec > 0.3) { g.appendChild(bar(ns, x, HBASE + 1, bw, hRec, C.green)); }
    }
    var tot = sumEv + sumIce;
    txt($.share, tot > 0.005 ? Math.round(100 * sumEv / tot) + " %" : "--");
  }
  function bar(ns, x, y, w, h, col) {
    var r = document.createElementNS(ns, "rect");
    r.setAttribute("x", x.toFixed(1));
    r.setAttribute("y", y.toFixed(1));
    r.setAttribute("width", w.toFixed(1));
    r.setAttribute("height", h.toFixed(1));
    r.setAttribute("fill", col);
    r.setAttribute("opacity", "0.85");
    return r;
  }

  // ------------------------------------------------------------------ chips
  function drawChips(list) {
    var sig = list.map(function (c) { return c[0] + c[1] + (c[2] ? "!" : ""); }).join("|");
    if (sig === chipSig) { return; }
    chipSig = sig;
    var g = $.chips;
    var ns = "http://www.w3.org/2000/svg";
    while (g.firstChild) { g.removeChild(g.firstChild); }
    blinkEls = $.ice ? [$.ice] : [];
    var x = 352;
    for (var i = 0; i < list.length && i < 4; i++) {
      var c = list[i];
      var grp = document.createElementNS(ns, "g");
      grp.__blink = !!c[2];
      var r = document.createElementNS(ns, "rect");
      var t = document.createElementNS(ns, "text");
      t.setAttribute("y", "189.4");
      t.setAttribute("font-size", "9");
      t.setAttribute("font-weight", "700");
      t.setAttribute("letter-spacing", "0.6");
      t.setAttribute("text-anchor", "middle");
      t.setAttribute("fill", c[1] === C.muted ? C.muted : C.text);
      t.textContent = c[0];
      grp.appendChild(r);
      grp.appendChild(t);
      g.appendChild(grp);
      blinkEls.push(grp);
      var w = fitText(t, 60) + 11;
      if (x + w > 505) { g.removeChild(grp); break; }
      r.setAttribute("x", x);
      r.setAttribute("y", "180");
      r.setAttribute("width", w.toFixed(1));
      r.setAttribute("height", "13");
      r.setAttribute("rx", "6.5");
      r.setAttribute("fill", C.dark);
      r.setAttribute("stroke", c[1]);
      r.setAttribute("stroke-width", "1.2");
      t.setAttribute("x", (x + w / 2).toFixed(1));
      x += w + 4;
    }
  }

  // ------------------------------------------------------------------ apply
  function apply(d) {
    if (!ready || !d) { return; }
    var t = now();
    var dtS = lastT ? Math.min(0.25, Math.max(0, (t - lastT) / 1000)) : 0;
    lastT = t;

    var ph = str(d.ph);
    var hasVcu = ph !== null;
    var ign = num(d.ign);
    if (ign === null) { ign = ignNav === null ? 2 : ignNav; }
    var on = hasVcu && ign >= 2;

    // header
    var dt = new Date();
    txt($.hClock, ("0" + dt.getHours()).slice(-2) + ":" + ("0" + dt.getMinutes()).slice(-2));
    var amb = num(d.amb);
    txt($.hTemp, amb === null ? "" : fmtTemp(amb, d.uT));
    var imperial = typeof d.uL === "string" && d.uL !== "metric";
    txt($.rngUnit, imperial ? "mi" : "km");
    txt($.consUnit, imperial ? "kWh/100 mi" : "kWh/100 km");

    // standby
    attr($.standby, "opacity", on ? "0" : "1");
    txt($.sbTitle, hasVcu ? "HYBRID SYSTEM" : "ENERGY FLOW");
    txt($.sbSub, hasVcu ? "STANDBY" : "NO HYBRID DATA");
    if (!on) { return; }

    var ev = num(d.ev) || 0;
    var ice = num(d.ice) || 0;
    var total = ev + ice;
    var iceOn = (num(d.on) || 0) > 0.5;
    var sail = (num(d.sail) || 0) > 0.5 || ph === "SAIL";
    var k0 = num(d.k0);
    if (k0 === null) { k0 = iceOn ? 1 : 0; }
    k0 = clamp01(k0);
    var soc = num(d.soc);
    var tgt = num(d.tgt);
    if (tgt !== null && tgt > 1.5) { tgt /= 100; }
    var de = str(d.de), ed = str(d.ed);
    var boost = Math.max(0, Math.min(3, Math.round(num(d.boost) || 0)));
    var evPeak = STAGE_EV_KW[boost], regenPeak = STAGE_REGEN_KW[boost];
    var iceMax = num(d.iceMax);
    if (!(iceMax > 1)) { iceMax = 250; }
    var spd = Math.abs(num(d.spd) || 0);
    var ies = str(d.ies);
    var starting = !!(ies && ICE_STARTING[ies]);
    var awd = str(d.awd);

    // phase badge
    var pk;
    if (sail) { pk = "SAIL"; }
    else if (ph === "CHARGE") { pk = "CHARGE"; }
    else if (ph === "REGEN") { pk = "REGEN"; }
    else if (ph === "HYBRID") { pk = (ice > 10 && ev > Math.max(10, 0.25 * evPeak)) ? "BOOST" : "HYBRID"; }
    else if (ph === "ICE") { pk = "ENGINE"; }
    else if (ph === "EV") { pk = (Math.abs(total) < 1 && spd < 2) ? "READY" : "EDRIVE"; }
    else { pk = iceOn ? "ENGINE" : "READY"; }
    var P = PHASE[pk];
    txt($.phTxt, P[0]);
    attr($.phTxt, "fill", P[1]);
    attr($.phBox, "stroke", P[1]);
    txt($.modeTxt, (DE_LABEL[de] || de || "") + (ed ? " \u00b7 " + (ED_LABEL[ed] || ed) : ""));

    // flows
    var batOut = !sail && ev > 1, batIn = !sail && ev < -1;
    var drvOut = !sail && total > 1, drvIn = !sail && total < -1;
    var iceFlow = ice > 1 && k0 > 0.5;
    var driveCol = drvIn ? C.green : (ice > 1 ? (ev > 1 ? C.text : C.amber) : C.blue);
    var dMode = drvOut ? "fwd" : (drvIn ? "rev" : "idle");

    flow($.hvLn, batOut ? "fwd" : (batIn ? "rev" : "idle"), batIn ? C.green : C.blue);
    flow($.fuelLn, iceOn ? "fwd" : "idle", C.amber);
    flow($.iceLn1, iceFlow ? "fwd" : "idle", C.amber);
    flow($.iceLn2, iceFlow ? "fwd" : "idle", C.amber);
    flow($.emLn, dMode, driveCol);
    flow($.rProp, dMode, driveCol);
    flow($.rA1, dMode, driveCol);
    flow($.rA2, dMode, driveCol);

    // front axle: only drawn with the AWD transfer case, lit while it carries torque
    attr($.frontDrive, "opacity", awd ? "1" : "0");
    var frontOn = awd === "AWD" && Math.abs(num(d.awdNm) || 0) > 5 && (drvOut || drvIn);
    var fMode = frontOn ? dMode : "idle";
    flow($.fProp, fMode, driveCol);
    flow($.fA1, fMode, driveCol);
    flow($.fA2, fMode, driveCol);
    attr($.fDiff, "stroke", frontOn ? driveCol : C.dim);
    attr($.wF, "stroke", frontOn ? driveCol : (awd === "AWD" ? C.faint : C.dim));

    // components
    lit($.ice, starting ? C.blueLt : (iceOn ? C.amber : C.dim), iceOn || starting);
    $.ice.__blink = starting;
    if (blinkEls.indexOf($.ice) < 0) { blinkEls.push($.ice); }
    var emCol = batIn ? C.green : (Math.abs(ev) > 0.5 && !sail ? C.blue : C.dim);
    attr($.emCircle, "stroke", emCol);
    attr($.emBolt, "fill", emCol);
    attr($.gbBox, "stroke", (drvOut || drvIn) ? driveCol : C.dim);
    attr($.rDiff, "stroke", (drvOut || drvIn) ? driveCol : C.dim);
    attr($.wR, "stroke", (drvOut || drvIn) ? driveCol : (sail && spd > 2 ? C.blueLt : C.dim));
    attr($.tankBody, "stroke", iceOn ? C.amber : C.dim);
    var fuel = num(d.fuel);
    attr($.tankFill, "width", fuel === null ? "0" : (44 * clamp01(fuel)).toFixed(1));

    var gap = 1.5 + (1 - k0) * 3;
    var k0Col = k0 > 0.9 ? (iceFlow ? C.amber : C.muted) : (k0 > 0.1 ? C.amber : C.dim);
    attr($.k0A, "x1", (116 - gap).toFixed(2)); attr($.k0A, "x2", (116 - gap).toFixed(2));
    attr($.k0B, "x1", (116 + gap).toFixed(2)); attr($.k0B, "x2", (116 + gap).toFixed(2));
    attr($.k0A, "stroke", k0Col);
    attr($.k0B, "stroke", k0Col);
    attr($.iceLn1, "d", "M108 116 H" + (116 - gap - 1.5).toFixed(2));
    attr($.iceLn2, "d", "M" + (116 + gap + 1.5).toFixed(2) + " 116 H127");
    attr($.k0Lbl, "fill", k0 > 0.1 ? C.muted : C.faint);

    // battery
    var socPct = soc === null ? null : Math.max(0, Math.min(100, soc));
    var low = socPct !== null && socPct < 12;
    txt($.socVal, socPct === null ? "--" : String(Math.round(socPct)));
    txt($.battTxt, socPct === null ? "--%" : Math.round(socPct) + "%");
    attr($.soc, "fill", low ? C.red : C.text);
    attr($.socBar, "width", socPct === null ? "0" : (152 * socPct / 100).toFixed(1));
    attr($.socBar, "fill", low ? C.red : "url(#nSoc)");
    attr($.battFill, "width", socPct === null ? "0" : (44 * socPct / 100).toFixed(1));
    attr($.battFill, "fill", low ? C.red : (batIn ? C.green : C.blue));
    attr($.battBody, "stroke", batIn ? C.green : (batOut ? C.blue : C.dim));
    var showTgt = ed === "BATTERY_CONTROL" && tgt !== null;
    attr($.socTgt, "opacity", showTgt ? "1" : "0");
    if (showTgt) { attr($.socTgt, "x", (352 + 152 * clamp01(tgt) - 1).toFixed(1)); }

    var rng = num(d.rng);
    txt($.rngVal, (rng === null || rng < 0) ? "--" : String(Math.round(imperial ? rng * KM_TO_MI : rng)));
    var evAv = num(d.evAv);
    attr($.rngLbl, "fill", evAv === 0 ? C.dim : C.blue);

    // power readouts
    txt($.iceKw, kw(ice));
    attr($.iceKw, "fill", ice > 0.5 ? C.amber : C.muted);
    attr($.iceBar, "width", (152 * clamp01(ice / iceMax)).toFixed(1));
    txt($.evKw, kw(ev));
    attr($.evKw, "fill", ev < -0.5 ? C.green : (ev > 0.5 ? C.blue : C.muted));
    attr($.evBar, "width", (152 * clamp01(ev >= 0 ? ev / evPeak : -ev / regenPeak)).toFixed(1));
    attr($.evBar, "fill", ev < 0 ? C.green : C.blue);
    txt($.sysKw, kw(total));
    attr($.sysKw, "fill", total < -0.5 ? C.green : C.text);

    // chips
    var list = [];
    if (starting) { list.push(["ENGINE START", C.blueLt, true]); }
    var dsc = str(d.dsc);
    if (dsc === "DTC") { list.push(["DTC", C.amber, false]); }
    else if (dsc === "OFF") { list.push(["DSC OFF", C.amber, false]); }
    if (awd) { list.push([awd, awd === "AWD" ? C.blue : C.muted, false]); }
    if ((num(d.trl) || 0) > 0.5) { list.push(["TRAILER", C.amber, false]); }
    var rl = str(d.rl);
    if ((num(d.op) || 0) > 0.5) { list.push(["ONE PEDAL", C.green, false]); }
    // MED is the default recuperation level: not shown, same as the cluster
    else if (rl && rl !== "MED") { list.push(["REGEN " + rl, rl === "OFF" ? C.muted : C.green, false]); }
    if (boost > 0) { list.push(["BOOST " + boost, C.orange, false]); }
    drawChips(list);

    // consumption + history
    var cons = num(d.cons);
    // kWh per 100 mi = kWh per 100 km / 0.621371
    txt($.consVal, cons === null ? "--" : (imperial ? cons / KM_TO_MI : cons).toFixed(1));

    // on-board computer: tank litres, instantaneous in the ENGINE row, trip
    var mpg = typeof d.uC === "string" && d.uC !== "metric";
    var gal = d.uV === "gal";
    var fl = num(d.fl);
    txt($.tankTxt, (fl === null || fl < 0) ? "FUEL" : (gal ? (fl / L_PER_GAL).toFixed(1) + " gal" : Math.round(fl) + " L"));
    attr($.tankTxt, "fill", (fl !== null && fl >= 0 && iceOn) ? C.text : C.muted);
    var fm = str(d.fm);
    var ic = "";
    if (fm === "l100") {
      var fi = num(d.fi);
      if (fi !== null && fi >= 0) { ic = mpg ? mpgOf(fi) + " mpg" : fi.toFixed(1) + " L/100 km"; }
    } else if (fm === "lh") {
      var fh = num(d.fh);
      if (fh !== null) { ic = (gal ? fh / L_PER_GAL : fh).toFixed(1) + (gal ? " gal/h" : " L/h"); }
    }
    if ($.iceCons.__t !== ic) { txt($.iceCons, ic); fitText($.iceCons, 58); }
    var fa = num(d.fa);
    txt($.fuelAvgVal, (fa === null || fa < 0) ? "--" : (mpg ? mpgOf(fa) : fa.toFixed(1)));
    txt($.fuelAvgUnit, mpg ? "mpg" : "L/100 km");
    var tk = num(d.tk);
    txt($.tripVal, tk === null ? "--" : (imperial ? tk * KM_TO_MI : tk).toFixed(1));
    txt($.tripUnit, imperial ? "mi" : "km");
    if (dtS > 0) { histPush(ev, ice, dtS); }
    drawHist();
    animate();
  }

  // ------------------------------------------------------------------- init
  function init() {
    var ids = ["hTemp", "hClock", "phBox", "phTxt", "modeTxt", "wF", "wR", "frontDrive", "fProp", "fA1", "fA2", "fDiff",
      "fuelLn", "hvLn", "iceLn1", "iceLn2", "emLn", "rProp", "rA1", "rA2", "ice", "k0A", "k0B", "k0Lbl", "em", "emBolt",
      "gb", "rDiff", "tank", "tankFill", "batt", "battBody", "battFill", "battTxt", "soc", "socVal", "rngLbl", "rng",
      "rngVal", "rngUnit", "consUnit", "socBar", "socTgt", "iceKw", "iceBar", "evKw", "evBar", "sysKw", "chips", "hist", "consVal", "share",
      "tankTxt", "iceCons", "fuelAvgVal", "fuelAvgUnit", "tripVal", "tripUnit",
      "standby", "sbTitle", "sbSub"];
    for (var i = 0; i < ids.length; i++) { $[ids[i]] = document.getElementById(ids[i]); }
    $.emCircle = document.querySelector("#em circle");
    $.gbBox = document.querySelector("#gb rect");
    $.tankBody = document.querySelector("#tank rect");
    ready = true;

    var q = window.__etk854nq || {};
    ignNav = typeof q.ign === "number" ? q.ign : null;
    window.etk854hybridUpdate = function (d) { stopDemo(); apply(d); };
    window.map.updateData = function (g) {
      if (g && typeof g.ignitionLevel === "number") { ignNav = g.ignitionLevel; }
    };
    if (q.data) { apply(q.data); }
    window.__etk854nq = null;

    // Demo only in a desktop browser: in game the page is served from
    // local:// and the engine injects `beamng`. Real data stops it anyway.
    var inGame = /^local:/i.test(String(window.location.href)) || typeof beamng !== "undefined";
    var hash = String(window.location.hash || "");
    if ((!inGame || hash.indexOf("demo") >= 0) && !q.data) { startDemo(hash); }
  }

  // =================================================================== DEMO
  function demoFrame(state, t) {
    var D = {
      soc: 78, tgt: 0.9, ph: "EV", de: "COMFORT", ed: "AUTO_EDRIVE", ev: 0, ice: 0, k0: 0, ies: "OFF", on: 0,
      sail: 0, chg: "OFF", trl: 0, lvl: "LEVEL", fan: 0, boost: 0, evAv: 1, evFr: 1, rl: "MED", op: 0, dsc: "DSC",
      prog: "COMFORT", spd: 50, gear: "D", gi: 4, ign: 2, rng: 41, cons: 19.6, cap: 13.5, use: 9.2, iceMax: 250,
      fuel: 0.72, amb: 17.5, wt: 88,
      fl: 38.4, fcap: 50, frng: 480, fm: "ev", fi: 0, fh: 0, fa: 6.8, tk: 128.4
    };
    var s = state, w = Math.sin(t * 1.3) * 0.5 + 0.5;
    if (s === "anim") {
      var c = t % 36;
      s = c < 4 ? "ready" : c < 10 ? "ev" : c < 14 ? "sail" : c < 18 ? "regen" : c < 24 ? "boost" : c < 30 ? "charge" : "awd";
    }
    switch (s) {
      case "ready": D.spd = 0; D.gear = "P"; D.gi = 0; break;
      case "ev": D.ev = 38 + 25 * w; break;
      case "sail": D.ph = "SAIL"; D.sail = 1; D.spd = 92; break;
      case "regen": D.ph = "REGEN"; D.ev = -30 - 15 * w; D.spd = 70; break;
      case "boost":
        D.ph = "HYBRID"; D.on = 1; D.ies = "ON"; D.k0 = 1; D.ice = 190 + 30 * w; D.ev = 76; D.de = "SPORT";
        D.prog = "SPORT+"; D.dsc = "DTC"; D.spd = 118; D.fm = "l100"; D.fi = 24.8 + 3 * w; break;
      case "hybrid":
        D.ph = "HYBRID"; D.on = 1; D.ies = "ON"; D.k0 = 1; D.ice = 80 + 10 * w; D.ev = 12; D.spd = 95;
        D.fm = "l100"; D.fi = 8.6 + 1.2 * w; break;
      case "ice":
        D.ph = "ICE"; D.on = 1; D.ies = "ON"; D.k0 = 1; D.ice = 48 + 8 * w; D.soc = 16; D.spd = 130;
        D.fm = "l100"; D.fi = 7.1 + 0.6 * w; break;
      case "charge":
        D.ph = "CHARGE"; D.on = 1; D.ies = "ON"; D.k0 = 1; D.ice = 72 + 6 * w; D.ev = -26; D.ed = "BATTERY_CONTROL";
        D.soc = 55; D.tgt = 0.8; D.chg = "CRUISE"; D.spd = 100; D.fm = "l100"; D.fi = 10.2 + 0.5 * w; break;
      case "start": D.ies = "SYNC"; D.on = 1; D.k0 = 0.4; D.ev = 64; D.spd = 60; break;
      case "awd":
        D.awd = "AWD"; D.awdNm = 140; D.ph = "HYBRID"; D.on = 1; D.ies = "ON"; D.k0 = 1; D.ice = 120 + 20 * w;
        D.ev = 45; D.trl = 1; D.spd = 80; D.fm = "l100"; D.fi = 11.4 + 1.5 * w; break;
      case "imperial":
        D.ev = 30 + 20 * w; D.uL = "imperial"; D.uT = "f"; D.uC = "imperial"; D.uV = "gal"; D.rl = "HIGH"; break;
      case "off": D.ign = 1; break;
      case "nodata": D = { ign: 2, spd: 0 }; break;
      default: break;
    }
    return D;
  }

  function seedHistory() {
    // plausible 15 min so the chart is not empty in the preview
    for (var i = 0; i < BUCKETS - 1; i++) {
      var ph = i % 10;
      hist.push({
        ev: ph < 5 ? 0.12 + 0.05 * Math.sin(i) : 0.02,
        ice: ph >= 5 && ph < 8 ? 0.35 + 0.1 * Math.cos(i) : 0,
        rec: ph === 4 || ph === 8 ? 0.08 : 0.02 + 0.015 * Math.sin(i * 2)
      });
    }
    histDirty = true;
  }

  function startDemo(hash) {
    var m = /demo=([a-z]+)/.exec(hash);
    var state = m ? m[1] : "anim";
    seedHistory();
    var t0 = now();
    demo = setInterval(function () { apply(demoFrame(state, (now() - t0) / 1000)); }, 66);
    apply(demoFrame(state, 0));
  }
  function stopDemo() {
    if (demo) {
      clearInterval(demo);
      demo = null;
      hist = [];
      cur = { ev: 0, ice: 0, rec: 0, t: 0 };
      histDirty = true;
    }
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", init);
  } else {
    init();
  }
})();
