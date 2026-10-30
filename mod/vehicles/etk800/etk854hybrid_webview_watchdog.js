// ETK 854 ActiveHybrid - frame watchdog for the two vehicle web views (BeamNG.drive 0.39.4)
// Loaded by the cluster and centre-screen pages right after their inline head script (stubs).
// After core_vehicles.replaceVehicle the engine stops producing frames for these pages ~0.6 s
// after creation (requestAnimationFrame never fires, the texture keeps its first frame) while
// script and Lua data keep running; reloading the page in the same web view fixes it.
// Stall = a check window with data but no animation frame (paused game or a distant camera
// still produce frames). STALL_CHECKS in a row -> reload, at most MAX_RELOADS per
// RELOAD_WINDOW_MS. window.name carries the cluster's one-shot setup() / updateMode()
// arguments across the reload.
(function () {
  "use strict";
  var KEY = "etk854wd1:";
  var CHECK_MS = 700;
  var STALL_CHECKS = 2;
  var MAX_RELOADS = 3;
  var RELOAD_WINDOW_MS = 60000;
  var DATA_FNS = ["updateData", "etk854hybridUpdate"]; // cluster / centre screen data
  var KEEP_FNS = ["setup", "updateMode"]; // one-shot, replayed after a reload

  var keep = {};
  var hist = [];
  var data = 0;
  var frames = 0;
  var stalls = 0;

  function log(level, msg) {
    try {
      if (typeof beamng !== "undefined" && beamng.sendEngineLua) {
        var t = "[" + String(document.title || location.pathname) + "] " + msg;
        beamng.sendEngineLua("log('" + level + "','etk854hybrid.webview'," + JSON.stringify(t) + ")");
      }
    } catch (e) { /* never throw from the watchdog */ }
  }

  // ---- state handed over by a watchdog reload
  try {
    var n = String(window.name || "");
    if (n.indexOf(KEY) === 0) {
      var s = JSON.parse(n.slice(KEY.length));
      keep = s.keep || {};
      hist = s.hist || [];
    }
  } catch (e) {
    keep = {};
    hist = [];
  }
  window.name = "";
  var q = window.__etk854q; // cluster call buffer from the head script
  if (q) {
    if (keep.setup && !q.setup) { q.setup = keep.setup; }
    if (keep.updateMode && !q.mode) { q.mode = keep.updateMode; }
  }

  // ---- count / remember calls: wraps the head-script stubs and every later window.<name> = f
  function trap(name, onCall) {
    function wrap(f) {
      if (typeof f !== "function") { return f; }
      return function () {
        onCall(arguments);
        return f.apply(this, arguments);
      };
    }
    var cur = wrap(window[name]);
    try {
      Object.defineProperty(window, name, {
        configurable: true,
        enumerable: true,
        get: function () { return cur; },
        set: function (f) { cur = wrap(f); }
      });
    } catch (e) { /* leave this function unwatched */ }
  }
  for (var i = 0; i < DATA_FNS.length; i++) {
    trap(DATA_FNS[i], function () { data++; });
  }
  for (var k = 0; k < KEEP_FNS.length; k++) {
    (function (nm) {
      trap(nm, function (a) { keep[nm] = a[0]; });
    })(KEEP_FNS[k]);
  }

  function reload() {
    var now = Date.now();
    var recent = [];
    for (var j = 0; j < hist.length; j++) {
      if (now - hist[j] < RELOAD_WINDOW_MS) { recent.push(hist[j]); }
    }
    hist = recent;
    if (hist.length >= MAX_RELOADS) { return false; }
    hist.push(now);
    try {
      window.name = KEY + JSON.stringify({ keep: keep, hist: hist });
    } catch (e) {
      window.name = "";
    }
    log("W", "no frames while data arrives - reloading the page (" + hist.length + "/" + MAX_RELOADS + ")");
    location.reload();
    return true;
  }

  function onFrame() { frames++; }
  requestAnimationFrame(onFrame);
  setInterval(function () {
    if (frames > 0) {
      stalls = 0;
    } else if (data > 0) {
      stalls++;
    }
    data = 0;
    frames = 0;
    requestAnimationFrame(onFrame);
    if (stalls >= STALL_CHECKS) {
      stalls = 0;
      reload();
    }
  }, CHECK_MS);
})();
