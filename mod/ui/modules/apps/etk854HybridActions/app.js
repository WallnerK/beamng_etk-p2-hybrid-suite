// ETK 854 ActiveHybrid - Actions (clickable vehicle actions, BeamNG.drive 0.39.4)
// =========================================================================
// Hosting as the HUD app: AngularJS directive, exactly one root element,
// `replace: true` (the host mounts only el[0]); inline styles only (a <style>
// is global in the CEF document). Needs app.json "interactive": "yes" plus
// root pointer-events: auto, otherwise every button is dead.
//
// * Buttons fire bngApi.activeObjectLua() through getControllerSafe(), a no-op
//   when the controller is missing.
// * No local mode state: highlighting mirrors the electrics stream (the VCU
//   may reset its modes silently).
// * Optional features show only while their channel exists; the R-STEER and
//   DISPLAY rows are hidden as a whole while their channel is not a string.
// * Clicks are ignored on a non-ActiveHybrid vehicle (a DSC click must not
//   switch a stock car's driveModes).
// =========================================================================

angular.module('beamng.apps')
.directive('etk854HybridActions', [function () {

  // ---------------------------------------------------------------- palette
  // same as the HUD app
  const C = {
    bgTop:  '#121824',
    bgBot:  '#070a10',
    edge:   'rgba(255,255,255,0.13)',
    text:   '#e9eef6',
    muted:  '#7b8494',
    dim:    '#39404c',
    idleBg: 'rgba(255,255,255,0.045)',
    blue:   '#1e90ff',
    blueLt: '#7cc4ff',
    amber:  '#ffa62b',
    green:  '#24d69a',
    red:    '#ff5563'
  }
  const SOFT = {}
  SOFT[C.blue]   = 'rgba(30,144,255,0.20)'
  SOFT[C.blueLt] = 'rgba(124,196,255,0.16)'
  SOFT[C.amber]  = 'rgba(255,166,43,0.20)'
  SOFT[C.green]  = 'rgba(36,214,154,0.18)'
  SOFT[C.red]    = 'rgba(255,85,99,0.20)'

  const FONT = "'Roboto Condensed','Roboto','Arial Narrow',sans-serif"

  // --------------------------------------------------------------- Lua refs
  const VCU  = "controller.getControllerSafe('hybridControl')"
  const DSE  = "controller.getControllerSafe('driveModes')"
  const AWD  = "controller.getControllerSafe('etk854hybrid_awdMode')"
  const SUSP = "controller.getControllerSafe('etk854hybrid_adaptiveSusp')"
  const AIR  = "controller.getControllerSafe('etk854hybrid_airCompressor')"   // trailer air supply
  const RAS  = "controller.getControllerSafe('etk854hybrid_rearSteer')"      // rear-axle steering
  const CP   = "controller.getControllerSafe('etk854hybrid_cockpitPanel')"   // centre-screen views + panel

  // The shift logic is not a controller. vehicleController loads it as
  // "controller/vehicleController/shiftLogic/<name>" only when the file is in
  // lua/vehicle/, else under the legacy key; a vehicle-folder module may land
  // under either -> look up both. Call and probe tolerate its absence.
  const SHIFT_NAME = 'etk854hybrid_8hp'
  const SHIFT_GET = "(package.loaded['controller/vehicleController/shiftLogic/" + SHIFT_NAME +
    "'] or package.loaded['controller/shiftLogic-" + SHIFT_NAME + "'])"
  const shiftCall = prog =>
    'local m = ' + SHIFT_GET + " if type(m) == 'table' and type(m.setProgram) == 'function' then m.setProgram('" + prog + "') end"
  // the callback form wraps the command as a function argument -> expression
  const SHIFT_PROBE =
    '(function() local m = ' + SHIFT_GET + " return type(m) == 'table' and type(m.setProgram) == 'function' end)()"

  // DISPLAY row tooltips (current state is prefixed at runtime)
  const DP_CL_TIP = 'click: next view (AUTO / HYBRID / EFFICIENCY / SPORT / MINIMAL; AUTO follows the driving experience)'
  const DP_CD_TIP = 'click: next view (ENERGY / MAP / TRAILER; TRAILER only with trailer data)'

  // ------------------------------------------------------------ action rows
  // cell.kind: 'btn' (default, fires cell.lua) or 'val' (read-only readout)
  const ROWS = [
    { id: 'rDe', label: 'DRIVE', cells: [
      { id: 'deC', text: 'COMFORT', lua: VCU + ".setDeMode('COMFORT')", tip: 'Driving Experience COMFORT' },
      { id: 'deE', text: 'ECO PRO', lua: VCU + ".setDeMode('ECO_PRO')", tip: 'Driving Experience ECO PRO' },
      { id: 'deS', text: 'SPORT',   lua: VCU + ".setDeMode('SPORT')",   tip: 'Driving Experience SPORT (ICE always on, DTC)' }
    ] },
    { id: 'rEd', label: 'EV', cells: [
      { id: 'edA', text: 'AUTO',    lua: VCU + ".setEdriveMode('AUTO_EDRIVE')",     tip: 'AUTO EV' },
      { id: 'edM', text: 'MAX',     lua: VCU + ".setEdriveMode('MAX_EDRIVE')",      tip: 'MAX EV - purely electric' },
      { id: 'edB', text: 'BATTERY', lua: VCU + ".setEdriveMode('BATTERY_CONTROL')", tip: 'BATTERY CONTROL - hold / charge to target SOC' }
    ] },
    { id: 'rSoc', label: 'SOC', cells: [
      { id: 'socD', text: '− 5', lua: VCU + '.adjustBatteryTargetSoc(-5)', tip: 'Target SOC -5 pp (effective in BATTERY CONTROL)' },
      { id: 'socV', kind: 'val', text: 'TARGET --%', grow: 2 },
      { id: 'socU', text: '+ 5',      lua: VCU + '.adjustBatteryTargetSoc(5)',  tip: 'Target SOC +5 pp (effective in BATTERY CONTROL)' }
    ] },
    { id: 'rRg', label: 'RECUP', cells: [
      { id: 'rgO', text: 'OFF',  lua: VCU + ".setRegenLevel('OFF')",  tip: 'Recuperation OFF - sailing allowed' },
      { id: 'rgL', text: 'LOW',  lua: VCU + ".setRegenLevel('LOW')",  tip: 'Recuperation LOW' },
      { id: 'rgM', text: 'MED',  lua: VCU + ".setRegenLevel('MED')",  tip: 'Recuperation MED' },
      { id: 'rgH', text: 'HIGH', lua: VCU + ".setRegenLevel('HIGH')", tip: 'Recuperation HIGH - one-pedal' }
    ] },
    { id: 'rDsc', label: 'DSC', cells: [
      { id: 'dsD', text: 'DSC', lua: DSE + ".setDriveMode('dsc')", tip: 'Dynamic Stability Control' },
      { id: 'dsT', text: 'DTC', lua: DSE + ".setDriveMode('dtc')", tip: 'Dynamic Traction Control (SPORT re-selects DTC)' },
      { id: 'dsO', text: 'OFF', lua: DSE + ".setDriveMode('off')", tip: 'DSC OFF - never re-armed by a mode change' }
    ] },
    { id: 'rGb', label: 'GEARBOX', cells: [
      { id: 'spE', text: 'ECO',   lua: shiftCall('ECO'),     tip: 'Shift program ECO' },
      { id: 'spC', text: 'COMF',  lua: shiftCall('COMFORT'), tip: 'Shift program COMFORT' },
      { id: 'spS', text: 'SPORT', lua: shiftCall('SPORT'),   tip: 'Shift program SPORT' },
      { id: 'spM', text: 'MAN',   lua: shiftCall('MANUAL'),  tip: 'Shift program MANUAL' },
      { id: 'spV', kind: 'val', text: '--', grow: 3 },
      { id: 'lc',  text: 'LAUNCH', lua: VCU + '.armLaunchControl()', tip: 'Launch Control: S/M, or D in SPORT; in SPORT/DTC also brake + kickdown at a standstill; press again = off', grow: 1.3 }
    ] },
    { id: 'rCh', label: 'CHASSIS', cells: [
      { id: 'awA', text: 'AWD',     lua: AWD + ".setMode('AWD')",       tip: 'All-wheel drive' },
      { id: 'aw2', text: '2WD',     lua: AWD + ".setMode('2WD')",       tip: 'Rear-wheel drive only' },
      { id: 'trl', text: 'TRAILER', lua: SUSP + '.toggleTrailerMode()', tip: 'Trailer mode (leveling)', grow: 1.3 },
      { id: 'air', text: 'COMPR', lua: AIR + '.toggle()', tip: 'Electric air compressor (trailer air supply) on/off', grow: 1.4 }
    ] },
    // opt: hidden until etk854hybrid_rasSel is a string
    { id: 'rRs', label: 'R-STEER', opt: true, cells: [
      { id: 'rsO', text: 'OFF',       lua: RAS + ".setMode('OFF')",   tip: 'Rear-axle steering off - rear wheels straight, as without the system' },
      { id: 'rsD', text: 'ON + DSC',  lua: RAS + ".setMode('DSC')",   tip: 'Rear-axle steering on with stability assist (default)', grow: 1.35 },
      { id: 'rsN', text: 'ON NO DSC', lua: RAS + ".setMode('NODSC')", tip: 'Rear-axle steering on without steering assist - rear wheels return to straight while DSC intervenes', grow: 1.45 }
    ] },
    // opt: hidden until etk854hybrid_clusterViewSel is a string
    { id: 'rDp', label: 'DISPLAY', opt: true, cells: [
      { id: 'dpC', text: 'CLUSTER --', lua: CP + '.cycleClusterView()', tip: DP_CL_TIP, grow: 1.6 },
      { id: 'dpS', text: 'CENTRE --',  lua: CP + '.cycleCentreView()',  tip: DP_CD_TIP, grow: 1.45 },
      { id: 'dpM', text: 'MENU',       lua: CP + '.menu()',             tip: 'Settings panel on the centre display: open / close', grow: 0.85 }
    ] }
  ]
  const BY_ID = {}
  ROWS.forEach(r => r.cells.forEach(c => { BY_ID[c.id] = c }))

  // [cellId, electrics value that makes it active, optional own colour]
  const DE_BTN  = [['deC', 'COMFORT'], ['deE', 'ECO_PRO'], ['deS', 'SPORT']]
  const ED_BTN  = [['edA', 'AUTO_EDRIVE'], ['edM', 'MAX_EDRIVE'], ['edB', 'BATTERY_CONTROL']]
  const RG_BTN  = [['rgO', 'OFF'], ['rgL', 'LOW'], ['rgM', 'MED'], ['rgH', 'HIGH']]
  // DSC colours as the HUD badge, but OFF lit amber to be unmistakable
  const DSC_BTN = [['dsD', 'DSC', C.blue], ['dsT', 'DTC', C.red], ['dsO', 'OFF', C.amber]]
  const SP_BTN  = [['spE', 'ECO'], ['spC', 'COMFORT'], ['spS', 'SPORT'], ['spM', 'MANUAL']]
  const AWD_BTN = [['awA', 'AWD'], ['aw2', '2WD']]
  const RS_BTN  = [['rsO', 'OFF', C.amber], ['rsD', 'DSC', C.blue], ['rsN', 'NODSC', C.blueLt]]
  const RS_OFF_TIP = ' - inactive while DSC OFF'
  const LC_LABEL = { IDLE: 'LAUNCH', ARMED: 'LC ARMED', READY: 'LC READY', ACTIVE: 'LC GO' }
  // short view names (as on the panel's DISPLAY tab)
  const DP_SHORT = { EFFICIENCY: 'EFFIC.' }

  const num = v => (v === undefined || v === null || v === '' || isNaN(Number(v))) ? null : Number(v)
  const now = () => (window.performance && performance.now) ? performance.now() : Date.now()

  // ---------------------------------------------------------------- markup
  function cellHtml(c) {
    const val = c.kind === 'val'
    const base = 'flex:' + (c.grow || 1) + ' 1 0;min-width:0;display:flex;align-items:center;justify-content:center;' +
      'box-sizing:border-box;border-radius:0.45em;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;' +
      'font-size:0.82em;letter-spacing:0.08em;transition:transform 60ms;'
    const look = val
      ? 'border:1px solid transparent;background:rgba(255,255,255,0.02);color:' + C.muted + ';cursor:default;'
      : 'border:1px solid ' + C.dim + ';background:' + C.idleBg + ';color:' + C.muted + ';cursor:pointer;'
    const attr = val ? 'data-v="' + c.id + '"' : 'data-a="' + c.id + '"'
    return '<div ' + attr + ' title="' + (c.tip || '') + '" style="' + base + look + '">' + c.text + '</div>'
  }
  function rowHtml(r) {
    return '<div data-r="' + r.id + '" style="flex:1 1 0;min-height:0;display:' + (r.opt ? 'none' : 'flex') + ';align-items:stretch;gap:0.35em;">' +
      // nowrap: 'R-STEER' must not break at its hyphen when the font falls back
      '<div style="flex:none;width:4.6em;display:flex;align-items:center;font-size:0.7em;letter-spacing:0.14em;white-space:nowrap;color:' + C.muted + ';">' +
      r.label + '</div>' + r.cells.map(cellHtml).join('') + '</div>'
  }
  const TPL =
    '<div style="width:100%;height:100%;box-sizing:border-box;pointer-events:auto;-webkit-user-select:none;user-select:none;' +
    'overflow:hidden;display:flex;flex-direction:column;gap:0.35em;padding:0.6em;font-size:10px;font-family:' + FONT + ';' +
    'color:' + C.text + ';border-radius:11px;border:1px solid ' + C.edge + ';background:linear-gradient(' + C.bgTop + ',' + C.bgBot + ');">' +
      '<div style="flex:none;display:flex;align-items:center;justify-content:space-between;height:1.3em;' +
      'border-bottom:1px solid rgba(30,144,255,0.35);padding-bottom:0.2em;">' +
        '<div style="font-size:0.85em;letter-spacing:0.26em;color:' + C.muted + ';">ACTIVE<span style="color:' + C.blueLt + ';"> HYBRID</span></div>' +
        '<div data-v="hdr" style="font-size:0.8em;letter-spacing:0.12em;color:' + C.muted + ';white-space:nowrap;">--</div>' +
      '</div>' +
      ROWS.map(rowHtml).join('') +
    '</div>'

  return {
    template: TPL,
    replace: true,
    restrict: 'EA',

    link: function (scope, element) {
      const root = element[0]
      const streamsList = ['electrics']
      StreamsManager.add(streamsList)

      // element-scoped lookup keeps multiple instances independent
      const B = {}
      const found = root.querySelectorAll('[data-a],[data-v],[data-r]')
      for (let i = 0; i < found.length; i++) {
        const el = found[i]
        B[el.getAttribute('data-a') || el.getAttribute('data-v') || el.getAttribute('data-r')] = el
      }

      // ------------------------- memoised DOM writers (ticks at frame rate)
      let memo = {}
      const txt = (id, v) => { const el = B[id]; if (el && el.textContent !== v) el.textContent = v }
      const tip = (id, v) => { const el = B[id]; if (el && el.getAttribute('title') !== v) el.setAttribute('title', v) }
      function vis(id, on) {
        const el = B[id]; if (!el) return
        const k = 'v' + id
        if (memo[k] === on) return
        memo[k] = on
        el.style.display = on ? 'flex' : 'none'
      }
      function look(id, on, col) {
        const el = B[id]; if (!el) return
        const k = 'l' + id, key = on ? col : '-'
        if (memo[k] === key) return
        memo[k] = key
        el.style.borderColor = on ? col : C.dim
        el.style.background  = on ? (SOFT[col] || C.idleBg) : C.idleBg
        el.style.color       = on ? C.text : C.muted
      }
      function fade(id, faded) {
        const el = B[id]; if (!el) return
        const k = 'f' + id
        if (memo[k] === faded) return
        memo[k] = faded
        el.style.opacity = faded ? '0.45' : '1'
      }
      function group(list, cur, col) {
        for (let i = 0; i < list.length; i++) look(list[i][0], cur === list[i][1], list[i][2] || col)
      }

      // ------------------------------------------------ font-size = scale
      // all inner sizes are em -> the root font-size scales the app
      // (290x250 -> 10 px)
      function fit() {
        const w = root.clientWidth, h = root.clientHeight
        if (!w || !h) return
        const fs = Math.max(7, Math.min(16, Math.min(w / 29, h / 25))).toFixed(2) + 'px'
        if (root.style.fontSize !== fs) root.style.fontSize = fs
      }
      let ro = null
      if (typeof ResizeObserver === 'function') { ro = new ResizeObserver(fit); ro.observe(root) }
      scope.$on('app:resized', fit)
      scope.$on('windowResize', fit)
      fit()

      // ------------------------------------------------ shift-program probe
      // null = unknown, else whether setProgram exists; re-probed every 10 s
      // (a Ctrl+R reload with another shift logic fires no VehicleChange)
      let shiftSettable = null, lastProbe = -1e9, probeBusy = false
      function probeShift(t) {
        if (probeBusy && t - lastProbe < 5000) return    // lost callback: retry
        if (t - lastProbe < (shiftSettable === null ? 1500 : 10000)) return
        lastProbe = t
        probeBusy = true
        bngApi.activeObjectLua(SHIFT_PROBE, function (res) {
          probeBusy = false
          shiftSettable = (res === true || res === 'true')
        })
      }

      // ------------------------------------------------------ interaction
      let hybrid = false
      function onClick(ev) {
        const el = ev.target && ev.target.closest ? ev.target.closest('[data-a]') : null
        if (!el || !root.contains(el)) return
        ev.preventDefault(); ev.stopPropagation()
        const a = BY_ID[el.getAttribute('data-a')]
        if (!a) return
        if (!hybrid || !a.lua) return
        bngApi.activeObjectLua(a.lua)
      }
      function onDown(ev) {
        const el = ev.target && ev.target.closest ? ev.target.closest('[data-a]') : null
        if (el) el.style.transform = 'scale(0.95)'
      }
      function onUp() {
        const pressed = root.querySelectorAll('[data-a]')
        for (let i = 0; i < pressed.length; i++) if (pressed[i].style.transform) pressed[i].style.transform = ''
      }
      root.addEventListener('click', onClick)
      root.addEventListener('mousedown', onDown)
      root.addEventListener('mouseup', onUp)
      root.addEventListener('mouseleave', onUp)

      const resetMemo = function () { memo = {}; shiftSettable = null; lastProbe = -1e9; probeBusy = false }
      scope.$on('VehicleFocusChanged', resetMemo)
      scope.$on('VehicleChange', resetMemo)

      scope.$on('$destroy', function () {
        StreamsManager.remove(streamsList)
        if (ro) ro.disconnect()
        root.removeEventListener('click', onClick)
        root.removeEventListener('mousedown', onDown)
        root.removeEventListener('mouseup', onUp)
        root.removeEventListener('mouseleave', onUp)
      })

      // ------------------------------------------------------- stream tick
      let lastFit = 0
      scope.$on('streamsUpdate', function (event, streams) {
        const e = streams && streams.electrics
        if (!e) return
        const t = now()
        if (t - lastFit > 1000) { lastFit = t; fit() }   // no-ResizeObserver fallback

        hybrid = typeof e.etk854hybrid_deMode === 'string'
        if (memo.hybrid !== hybrid) {
          memo.hybrid = hybrid
          root.style.opacity = hybrid ? '1' : '0.4'
        }
        // optional rows before the early return, so a foreign vehicle hides them
        const rasRaw = e.etk854hybrid_rasSel
        const hasRas = typeof rasRaw === 'string'
        vis('rRs', hasRas)
        const cvSelRaw = e.etk854hybrid_clusterViewSel
        const hasDp = typeof cvSelRaw === 'string'
        vis('rDp', hasDp)
        if (!hybrid) { txt('hdr', 'NO ACTIVEHYBRID'); return }

        // ------------------------------------------------ mode groups
        const de  = String(e.etk854hybrid_deMode)
        const ed  = String(e.etk854hybrid_edriveMode || '')
        const rg  = String(e.etk854hybrid_regenLevel || '').toUpperCase()
        const dsc = String(e.etk854hybrid_dscMode || 'DSC').toUpperCase()
        group(DE_BTN, de, C.blue)
        group(ED_BTN, ed, C.blue)
        group(RG_BTN, rg, C.green)
        group(DSC_BTN, dsc)

        // ------------------------------------------------ target SOC
        // targetSoc 0..1, batterySoc percent
        let tgt = num(e.etk854hybrid_targetSoc)
        if (tgt !== null && tgt > 1.5) tgt /= 100
        const bc = ed === 'BATTERY_CONTROL'
        txt('socV', 'TARGET ' + (tgt === null ? '--' : Math.round(tgt * 100)) + '%')
        const sv = B.socV
        if (sv && memo.socBc !== bc) { memo.socBc = bc; sv.style.color = bc ? C.text : C.muted }
        fade('socD', !bc); fade('socU', !bc)

        let soc = num(e.etk854hybrid_batterySoc)
        if (soc !== null && soc <= 1.5) soc *= 100
        txt('hdr', 'SOC ' + (soc === null ? '--' : Math.round(soc)) + '%')

        // ------------------------------------------------ gearbox row
        const spRaw = e.etk854hybrid_shiftProgram
        const spStr = (typeof spRaw === 'string' && spRaw !== '') ? spRaw.toUpperCase() : null
        if (spStr !== null) probeShift(t)
        const settable = spStr !== null && shiftSettable === true
        // leading letter run = base program; an adaptive suffix may follow
        const spBase = spStr === null ? '' : ((spStr.match(/^[A-Z]+/) || [''])[0])
        for (let i = 0; i < SP_BTN.length; i++) {
          vis(SP_BTN[i][0], settable)
          if (settable) look(SP_BTN[i][0], spBase === SP_BTN[i][1] || (SP_BTN[i][1] === 'COMFORT' && spBase === 'COMF'), C.blueLt)
        }
        vis('spV', spStr !== null && !settable)
        if (spStr !== null) {
          if (!settable) txt('spV', spStr.replace(/_/g, ' '))
          tip('spV', 'Shift program: ' + spStr)
        }

        const ls = String(e.etk854hybrid_launchState || 'IDLE').toUpperCase()
        // finer phase: PRELOAD is launchState ARMED, GUARD = brake-throttle guard
        const lph = String(e.etk854hybrid_launchPhase || '').toUpperCase()
        const gear = e.gear
        const g0 = typeof gear === 'string' ? gear.charAt(0) : ''
        const lcGear = g0 === 'S' || g0 === 'M' || (g0 === 'D' && de.toUpperCase() === 'SPORT')
        let lcTxt = LC_LABEL[ls] || ('LC ' + ls)
        if (lph === 'PRELOAD') lcTxt = 'LC PRELOAD'
        else if (lph === 'GUARD' && ls === 'IDLE') lcTxt = 'LC GUARD'
        txt('lc', lcTxt)
        look('lc', ls !== 'IDLE' || lph === 'GUARD', C.amber)
        const trim = num(e.etk854hybrid_launchTrim)
        tip('lc', BY_ID.lc.tip + (lph !== '' && lph !== 'OFF' ? ' - ' + lph + (trim !== null && trim < 0.995 ? ' trim ' + trim.toFixed(2) : '') : ''))
        // only faded: outside S/M/D-SPORT the VCU answers a click with a toast
        fade('lc', !lcGear && ls === 'IDLE')

        // ------------------------------------------------ chassis row
        const awdRaw = e.etk854hybrid_awdMode
        const hasAwd = typeof awdRaw === 'string' && awdRaw !== ''
        vis('awA', hasAwd); vis('aw2', hasAwd)
        if (hasAwd) group(AWD_BTN, awdRaw.toUpperCase(), C.blue)

        const trlRaw = e.etk854hybrid_trailerMode
        const hasTrl = trlRaw !== undefined && trlRaw !== null
        vis('trl', hasTrl)
        if (hasTrl) {
          const trlOn = trlRaw === true || (num(trlRaw) || 0) > 0.5
          look('trl', trlOn, C.amber)
          const lvl = e.etk854hybrid_levelState
          tip('trl', BY_ID.trl.tip + (typeof lvl === 'string' && lvl !== '' ? ' - ' + lvl : ''))
        }

        // compressor: lit = on (green idle / light blue pumping / amber LOW),
        // text = receiver pressure; 'NO TANK' = no receiver
        const airSt = e.etk854hybrid_airState
        const hasAir = typeof airSt === 'string' && airSt !== '' && airSt !== 'NO TANK'
        vis('air', hasAir)
        if (hasAir) {
          const airOn = (num(e.etk854hybrid_airEnabled) || 0) > 0.5
          const airRun = (num(e.etk854hybrid_airCompressor) || 0) > 0.5
          const airBar = num(e.etk854hybrid_airPressureBar)
          const barTxt = airBar === null ? '--' : Math.max(0, airBar).toFixed(1)
          txt('air', airOn ? ('COMPR ' + barTxt) : 'COMPR OFF')
          look('air', airOn, airSt === 'LOW' ? C.amber : (airRun ? C.blueLt : C.green))
          const inh = e.etk854hybrid_airInhibit
          const trlAir = (num(e.etk854hybrid_airTrailer) || 0) > 0.5
          // protection valve: 0 = shut, trailer gets no air until 6.0 bar
          const trlShut = trlAir && num(e.etk854hybrid_airTrlValve) === 0
          tip('air', 'Electric air compressor (trailer air supply) - ' + airSt + ', ' + barTxt + ' bar' +
            (typeof inh === 'string' && inh !== 'NONE' ? ', inhibit ' + inh : '') +
            (trlAir ? (trlShut ? ', trailer air coupled - supply valve shut until 6.0 bar' : ', trailer air coupled') : '') +
            ' - click: on/off')
        }

        // ------------------------------------------------ rear-steer row
        // DSC OFF disables the system: cells faded, selection stays lit and
        // clicks still pass (the controller keeps the choice)
        if (hasRas) {
          group(RS_BTN, rasRaw.toUpperCase())
          const rsDscOff = dsc === 'OFF'
          for (let i = 0; i < RS_BTN.length; i++) {
            const id = RS_BTN[i][0]
            fade(id, rsDscOff)
            tip(id, BY_ID[id].tip + (rsDscOff ? RS_OFF_TIP : ''))
          }
        }

        // ------------------------------------------------ display row
        // CLUSTER / CENTRE cycle and show the selection, lit when not on the
        // default (AUTO / ENERGY); MENU lit while the panel is open
        if (hasDp) {
          const cvSel = cvSelRaw.toUpperCase()
          const cvEff = typeof e.etk854hybrid_clusterView === 'string' ? e.etk854hybrid_clusterView.toUpperCase() : ''
          txt('dpC', 'CLUSTER ' + (DP_SHORT[cvSel] || cvSel || '--'))
          look('dpC', cvSel !== '' && cvSel !== 'AUTO', C.blueLt)
          tip('dpC', 'Instrument cluster: ' + (cvSel || '--') +
            (cvSel === 'AUTO' && cvEff !== '' ? ' (' + cvEff + ')' : '') + ' - ' + DP_CL_TIP)
          const cdView = typeof e.etk854hybrid_cdView === 'string' && e.etk854hybrid_cdView !== ''
            ? e.etk854hybrid_cdView.toUpperCase() : 'ENERGY'
          txt('dpS', 'CENTRE ' + cdView)
          look('dpS', cdView !== 'ENERGY', C.blueLt)
          tip('dpS', 'Centre display: ' + cdView + ' - ' + DP_CD_TIP)
          look('dpM', (num(e.etk854hybrid_cdMenu) || 0) > 0.5, C.blue)
        }
      })
    }
  }
}])
