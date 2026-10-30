// ETK 854 ActiveHybrid - Hybrid Cockpit HUD (BeamNG.drive 0.39.4)
// =========================================================================
// AngularJS directive: 0.39 still mounts app.js apps through the Vue AppHost
// and broadcasts "streamsUpdate"; a Vue app would need the vite build chain.
// Rendering: exactly one root <svg>, `replace: true`; presentation attributes
// only (no <style>, classes or assets), so the global UI CSS cannot interfere.
// The host owns position and size, the viewBox does the scaling.
//
// Data: electrics stream of the VCU (etk854hybridControl.lua) + submodules.
// Chip row (y 55..66): optional channels of other controllers, hidden while
//   nil. Fixed widths (no reflow), overlong labels squeezed; greedy layout
//   left to right, a chip that does not fit is skipped.
// ECO PRO pedal ruler (ECO PRO with pedalIceJoin > 0): pedal 0..1 split into
//   EV | EV MAX plateau | hybrid band (amber fill = load integral towards the
//   ICE start) | kick | kickdown, lit up to the caret (pedal the VCU maps);
//   EV segments grey out above the ECO EV speed ceiling (evZoneOpen 0).
// =========================================================================

angular.module('beamng.apps')
.directive('etk854HybridHud', [function () {

  // ---------------------------------------------------------------- palette
  const C = {
    bgTop:  '#121824',
    bgBot:  '#070a10',
    edge:   'rgba(255,255,255,0.13)',
    text:   '#e9eef6',
    muted:  '#7b8494',
    dim:    '#39404c',
    track:  'rgba(255,255,255,0.075)',
    blue:   '#1e90ff',
    blueLt: '#7cc4ff',
    amber:  '#ffa62b',
    green:  '#24d69a',
    red:    '#ff5563'
  }

  // --------------------------------------------------------------- geometry
  const GX = 62, GY = 128              // power gauge centre
  const R_OUT = 46, R_IN = 35          // outer ring = ICE, inner ring = e-motor
  const LEN_OUT = Math.PI * R_OUT
  const LEN_IN  = Math.PI * R_IN

  // left-to-right (clockwise over the top) and the mirrored variant for regen
  const arcFwd = r => `M ${GX - r} ${GY} A ${r} ${r} 0 0 1 ${GX + r} ${GY}`
  const arcRev = r => `M ${GX + r} ${GY} A ${r} ${r} 0 0 0 ${GX - r} ${GY}`

  const BAR_X = 12, BAR_W = 296, BAR_Y = 42, BAR_H = 8

  const FONT = "'Roboto Condensed','Roboto','Arial Narrow',sans-serif"

  // Power-flow node boxes
  const N_ICE   = { x: 121, y: 136, w:  58, h: 18 }
  const N_EM    = { x: 117, y: 172, w:  66, h: 22 }
  const N_BATT  = { x:  14, y: 172, w:  68, h: 22 }
  const N_WHEEL = { x: 218, y: 172, w:  88, h: 22 }
  const LINK_Y  = N_EM.y + N_EM.h / 2                       // 183

  function node(key, n, label) {
    return `<rect data-k="${key}box" x="${n.x}" y="${n.y}" width="${n.w}" height="${n.h}" rx="5"
                  fill="rgba(255,255,255,0.045)" stroke="${C.dim}" stroke-width="1"/>
            <text data-k="${key}lbl" x="${n.x + n.w / 2}" y="${n.y + n.h / 2 + 3.2}" font-family="${FONT}"
                  font-size="8.5" letter-spacing="1.1" text-anchor="middle" fill="${C.muted}">${label}</text>`
  }

  // one link = two opposing dashed paths + two arrowheads, only one side lit
  function link(key, x1, x2) {
    const a = 6, h = 3.6
    return `<line data-k="${key}bg" x1="${x1}" y1="${LINK_Y}" x2="${x2}" y2="${LINK_Y}"
                  stroke="${C.dim}" stroke-width="1" stroke-dasharray="2 3"/>
            <line data-k="${key}fwd" x1="${x1}" y1="${LINK_Y}" x2="${x2 - a}" y2="${LINK_Y}"
                  stroke="${C.blue}" stroke-width="2.4" stroke-linecap="round"
                  stroke-dasharray="5 6" opacity="0"/>
            <polygon data-k="${key}fwdA" points="${x2},${LINK_Y} ${x2 - a},${LINK_Y - h} ${x2 - a},${LINK_Y + h}"
                  fill="${C.blue}" opacity="0"/>
            <line data-k="${key}rev" x1="${x2}" y1="${LINK_Y}" x2="${x1 + a}" y2="${LINK_Y}"
                  stroke="${C.green}" stroke-width="2.4" stroke-linecap="round"
                  stroke-dasharray="5 6" opacity="0"/>
            <polygon data-k="${key}revA" points="${x1},${LINK_Y} ${x1 + a},${LINK_Y - h} ${x1 + a},${LINK_Y + h}"
                  fill="${C.green}" opacity="0"/>`
  }

  // Status chips. w = fixed width (u) = longest label (~3.9 u/char) + ~4 u
  // padding per side; dot chips need w >= text + 14. The 296 u row holds all
  // but AIR (294 u); with AIR shown the ICE chip is skipped.
  const CHIP_Y = 55, CHIP_H = 11, CHIP_GAP = 4
  const CHIPS = [
    { key: 'cProg', w: 70 },   // shift program + gear, e.g. "SPORT+ · S3▲4"
    { key: 'cAwd',  w: 26 },   // AWD / 2WD
    { key: 'cDiff', w: 46, dot: true, dotFill: C.text },  // LSD nn %, dot = slip hold
    { key: 'cTrl',  w: 48 },   // trailer / leveling / TSA
    { key: 'cAir',  w: 44, dot: true },  // AIR 8.4 / AIR LOW / TRL BRK / AIR OFF
    { key: 'cIce',  w: 54 },   // ICE engage state
    { key: 'cFan',  w: 30 }    // FAN n
  ]
  const DOT_INSET = 6          // dot centre from the chip's right edge
  function chip(c) {
    const tx = c.dot ? BAR_X + (c.w - DOT_INSET + 1) / 2 : BAR_X + c.w / 2
    return `<g data-k="${c.key}" opacity="0">
              <rect data-k="${c.key}box" x="${BAR_X}" y="${CHIP_Y}" width="${c.w}" height="${CHIP_H}" rx="5.5"
                    fill="rgba(255,255,255,0.04)" stroke="${C.dim}" stroke-width="1"/>
              <text data-k="${c.key}txt" x="${tx}" y="${CHIP_Y + 8}" font-family="${FONT}"
                    font-size="7" letter-spacing="0.9" text-anchor="middle" fill="${C.muted}"></text>
              ${c.dot ? `<circle data-k="${c.key}dot" cx="${BAR_X + c.w - DOT_INSET}" cy="${CHIP_Y + CHIP_H / 2}" r="1.9"
                    fill="${c.dotFill || C.blueLt}" opacity="0"/>` : ''}
            </g>`
  }

  // ECO PRO pedal ruler below the power-flow nodes; the status line drops to
  // ST_Y_RULER while it is shown to clear the caret
  const RUL_X = BAR_X, RUL_W = 250, RUL_Y = 198.5, RUL_H = 5
  const ST_Y = 211, ST_Y_RULER = 213.5
  const RUL_SEGS = 5
  function ruler() {
    let s = ''
    for (let i = 0; i < RUL_SEGS; i++) {
      s += `<rect data-k="rulB${i}" x="${RUL_X}" y="${RUL_Y}" width="0" height="${RUL_H}" fill="${C.dim}" opacity="0.3"/>`
    }
    for (let i = 0; i < RUL_SEGS; i++) {
      s += `<rect data-k="rulL${i}" x="${RUL_X}" y="${RUL_Y}" width="0" height="${RUL_H}" fill="${C.dim}"/>`
    }
    return `<g data-k="rul" opacity="0">
              ${s}
              <rect data-k="rulDef" x="${RUL_X}" y="${RUL_Y}" width="0" height="${RUL_H}" fill="${C.amber}"/>
              <text data-k="rulMax" x="${RUL_X}" y="${RUL_Y + RUL_H - 0.9}" font-family="${FONT}" font-size="4.6"
                    font-weight="bold" text-anchor="middle" fill="#0b1220">MAX</text>
              <text data-k="rulKd" x="${RUL_X}" y="${RUL_Y + RUL_H - 0.9}" font-family="${FONT}" font-size="4.6"
                    font-weight="bold" text-anchor="middle" fill="#0b1220">KD</text>
              <line data-k="rulCarB" x1="${RUL_X}" y1="${RUL_Y - 1.6}" x2="${RUL_X}" y2="${RUL_Y + RUL_H + 1.6}"
                    stroke="#05070b" stroke-width="3.4" stroke-linecap="round"/>
              <line data-k="rulCar" x1="${RUL_X}" y1="${RUL_Y - 1.6}" x2="${RUL_X}" y2="${RUL_Y + RUL_H + 1.6}"
                    stroke="${C.text}" stroke-width="1.5" stroke-linecap="round"/>
              <text data-k="rulLbl" x="308" y="${RUL_Y + RUL_H - 0.4}" font-family="${FONT}" font-size="7"
                    letter-spacing="0.9" text-anchor="end" fill="${C.muted}"></text>
            </g>`
  }

  const TPL = `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 320 220"
     preserveAspectRatio="xMidYMid meet"
     style="width:100%;height:100%;display:block;box-sizing:border-box;pointer-events:none;
            -webkit-user-select:none;user-select:none;">
  <defs>
    <linearGradient id="etk854hud-bg" x1="0" y1="0" x2="0" y2="1">
      <stop offset="0" stop-color="${C.bgTop}"/>
      <stop offset="1" stop-color="${C.bgBot}"/>
    </linearGradient>
    <linearGradient id="etk854hud-soc" x1="0" y1="0" x2="1" y2="0">
      <stop offset="0" stop-color="${C.blue}"/>
      <stop offset="1" stop-color="${C.blueLt}"/>
    </linearGradient>
    <linearGradient id="etk854hud-accent" x1="0" y1="0" x2="1" y2="0">
      <stop offset="0"    stop-color="${C.blue}"  stop-opacity="0"/>
      <stop offset="0.45" stop-color="${C.blue}"  stop-opacity="0.85"/>
      <stop offset="1"    stop-color="${C.blueLt}" stop-opacity="0"/>
    </linearGradient>
  </defs>

  <!-- shell -->
  <rect x="0.5" y="0.5" width="319" height="219" rx="11"
        fill="url(#etk854hud-bg)" stroke="${C.edge}" stroke-width="1"/>
  <rect x="12" y="24" width="296" height="1.2" fill="url(#etk854hud-accent)"/>

  <!-- ============================ header ============================ -->
  <text x="12" y="17" font-family="${FONT}" font-size="9" letter-spacing="2.6"
        fill="${C.muted}">ACTIVE<tspan fill="${C.blueLt}"> HYBRID</tspan></text>

  <!-- stability system badge: DSC / DTC / OFF, flashes while intervening -->
  <rect data-k="dscbox" x="152" y="5" width="46" height="16" rx="8"
        fill="rgba(30,144,255,0.12)" stroke="${C.blue}" stroke-width="1"/>
  <text data-k="dsc" x="175" y="16.2" font-family="${FONT}" font-size="8.5"
        letter-spacing="1.6" text-anchor="middle" fill="${C.blueLt}">DSC</text>

  <rect data-k="phasebox" x="206" y="5" width="102" height="16" rx="8"
        fill="rgba(30,144,255,0.12)" stroke="${C.blue}" stroke-width="1"/>
  <text data-k="phase" x="257" y="16.2" font-family="${FONT}" font-size="8.5"
        letter-spacing="1.6" text-anchor="middle" fill="${C.blueLt}">READY</text>

  <!-- ============================ battery =========================== -->
  <text x="12" y="38" font-family="${FONT}" font-size="8" letter-spacing="1.8"
        fill="${C.muted}">BATTERY</text>
  <text data-k="socpct" x="308" y="38.5" font-family="${FONT}" font-size="14"
        font-weight="bold" text-anchor="end" fill="${C.text}">--%</text>

  <rect x="${BAR_X}" y="${BAR_Y}" width="${BAR_W}" height="${BAR_H}" rx="4" fill="${C.track}"/>
  <rect data-k="socbar" x="${BAR_X}" y="${BAR_Y}" width="0" height="${BAR_H}" rx="4"
        fill="url(#etk854hud-soc)"/>
  <rect data-k="socTgt" x="${BAR_X}" y="${BAR_Y - 3}" width="2" height="${BAR_H + 6}"
        fill="${C.text}" opacity="0"/>

  <!-- ====================== optional status chips ==================== -->
  ${CHIPS.map(chip).join('')}

  <!-- ====================== power gauge (rings) ===================== -->
  <path d="${arcFwd(R_OUT)}" fill="none" stroke="${C.track}" stroke-width="7"/>
  <path d="${arcFwd(R_IN)}"  fill="none" stroke="${C.track}" stroke-width="7"/>

  <path data-k="arcIce" d="${arcFwd(R_OUT)}" fill="none" stroke="${C.amber}" stroke-width="7"
        stroke-linecap="butt" stroke-dasharray="${LEN_OUT} ${LEN_OUT}" stroke-dashoffset="${LEN_OUT}"/>
  <path data-k="arcEv"  d="${arcFwd(R_IN)}"  fill="none" stroke="${C.blue}"  stroke-width="7"
        stroke-linecap="butt" stroke-dasharray="${LEN_IN} ${LEN_IN}"  stroke-dashoffset="${LEN_IN}"/>
  <path data-k="arcRegen" d="${arcRev(R_IN)}" fill="none" stroke="${C.green}" stroke-width="7"
        stroke-linecap="butt" stroke-dasharray="${LEN_IN} ${LEN_IN}"  stroke-dashoffset="${LEN_IN}"/>

  <text data-k="total" x="${GX}" y="${GY - 11}" font-family="${FONT}" font-size="23"
        font-weight="bold" text-anchor="middle" fill="${C.text}">0</text>
  <text x="${GX}" y="${GY - 1}" font-family="${FONT}" font-size="7.5" letter-spacing="1.5"
        text-anchor="middle" fill="${C.muted}">kW TOTAL</text>

  <!-- ====================== per-unit readouts ======================= -->
  <circle cx="128" cy="75" r="2.6" fill="${C.amber}"/>
  <text x="136" y="78" font-family="${FONT}" font-size="8" letter-spacing="1.6"
        fill="${C.muted}">ICE</text>
  <text data-k="icekw" x="308" y="79" font-family="${FONT}" font-size="15" font-weight="bold"
        text-anchor="end" fill="${C.amber}">0</text>
  <text data-k="icenm" x="308" y="89" font-family="${FONT}" font-size="7"
        text-anchor="end" fill="${C.muted}">0 Nm</text>

  <circle data-k="emdot" cx="128" cy="103" r="2.6" fill="${C.blue}"/>
  <text x="136" y="106" font-family="${FONT}" font-size="8" letter-spacing="1.6"
        fill="${C.muted}">E-MOTOR</text>
  <text data-k="evkw" x="308" y="107" font-family="${FONT}" font-size="15" font-weight="bold"
        text-anchor="end" fill="${C.blue}">0</text>
  <text data-k="evnm" x="308" y="117" font-family="${FONT}" font-size="7"
        text-anchor="end" fill="${C.muted}">0 Nm</text>

  <circle data-k="recdot" cx="128" cy="129" r="2.6" fill="${C.dim}"/>
  <text x="136" y="132" font-family="${FONT}" font-size="8" letter-spacing="1.6"
        fill="${C.muted}">RECUP</text>
  <text data-k="reckw" x="308" y="133" font-family="${FONT}" font-size="13" font-weight="bold"
        text-anchor="end" fill="${C.dim}">--</text>
  <text data-k="reclvl" x="308" y="143" font-family="${FONT}" font-size="7"
        text-anchor="end" fill="${C.muted}">MED</text>

  <!-- ========================= power flow =========================== -->
  ${link('lb', N_BATT.x + N_BATT.w, N_EM.x)}
  ${link('lw', N_EM.x + N_EM.w, N_WHEEL.x)}

  <!-- K0 clutch link (ICE -> e-motor), drawn as a clutch pack -->
  <line data-k="k0up"   x1="150" y1="${N_ICE.y + N_ICE.h}" x2="150" y2="159"
        stroke="${C.dim}" stroke-width="2"/>
  <line data-k="k0pA"   x1="142" y1="160.5" x2="158" y2="160.5" stroke="${C.dim}" stroke-width="2"/>
  <line data-k="k0pB"   x1="142" y1="165.5" x2="158" y2="165.5" stroke="${C.dim}" stroke-width="2"/>
  <line data-k="k0down" x1="150" y1="167" x2="150" y2="${N_EM.y}"
        stroke="${C.dim}" stroke-width="2"/>
  <text data-k="k0lbl" x="163" y="165.8" font-family="${FONT}" font-size="6.5"
        letter-spacing="0.8" fill="${C.dim}">K0</text>

  ${node('ice',   N_ICE,   'ICE')}
  ${node('em',    N_EM,    'E-MOTOR')}
  ${node('batt',  N_BATT,  'BATTERY')}
  ${node('wheel', N_WHEEL, 'WHEELS')}

  <!-- ======================= ECO PRO pedal ruler ===================== -->
  ${ruler()}

  <!-- ========================= status line ========================== -->
  <text data-k="stLeft" x="12" y="${ST_Y}" font-family="${FONT}" font-size="7.5"
        letter-spacing="1.1" fill="${C.muted}">COMFORT &#183; AUTO EV</text>
  <text data-k="stRight" x="308" y="${ST_Y}" font-family="${FONT}" font-size="7.5"
        letter-spacing="1.1" text-anchor="end" fill="${C.muted}">K0 --</text>
</svg>`

  // ------------------------------------------------------------- constants
  // boost stage -> e-machine drive / regen kW (BOOST_STAGES in the VCU)
  const STAGE_KW    = [83, 110, 150, 200]
  const STAGE_REGEN = [55, 65, 80, 95]
  // ICE ring scale, learned per vehicle (no rated power is published, engines
  // span 185..408 kW): floor ICE_PEAK_FLOOR_KW, instant rise to
  // max(ICE kW, iceMaxTorqueNm x rpm) while running (full-load torque taken
  // at an rpm >= the ICE rpm, so the product stays below the power peak and
  // the scale is found without full throttle), slow decay to the floor,
  // reset on vehicle change.
  const ICE_PEAK_FLOOR_KW = 220
  const ICE_PEAK_TAU      = 600         // s

  const PHASE = {
    EV:      { label: 'ELECTRIC',      color: C.blue,  soft: 'rgba(30,144,255,0.12)' },
    HYBRID:  { label: 'HYBRID BOOST',  color: C.amber, soft: 'rgba(255,166,43,0.12)' },
    ICE:     { label: 'ENGINE',        color: C.amber, soft: 'rgba(255,166,43,0.12)' },
    CHARGE:  { label: 'CHARGING',      color: C.green, soft: 'rgba(36,214,154,0.12)' },
    REGEN:   { label: 'RECUPERATION',  color: C.green, soft: 'rgba(36,214,154,0.12)' },
    SAIL:    { label: 'SAILING',       color: C.blueLt, soft: 'rgba(124,196,255,0.10)' },
    IDLE:    { label: 'READY',         color: C.muted, soft: 'rgba(255,255,255,0.05)' }
  }
  // stability badge, colours as the DSE simpleControlButton (blue / red / grey)
  const DSC_STYLE = {
    DSC: { color: C.blue,  text: C.blueLt, soft: 'rgba(30,144,255,0.12)' },
    DTC: { color: C.red,   text: C.red,    soft: 'rgba(255,85,99,0.12)'  },
    OFF: { color: C.dim,   text: C.muted,  soft: 'rgba(255,255,255,0.04)' }
  }

  const EDRIVE_LABEL = {
    AUTO_EDRIVE:     'AUTO EV',
    MAX_EDRIVE:      'MAX EV',
    BATTERY_CONTROL: 'BATTERY CONTROL'
  }

  // ICE engage state: transitions light blue (as START), running amber
  const ICE_ENGAGE_COLOR = {
    OFF: C.dim, CRANK: C.blueLt, SYNC: C.blueLt, ENGAGE: C.blueLt,
    BLEND: C.amber, ON: C.amber, RELEASE: C.muted
  }
  // Leveling states worth a chip; routine HOLD / CALIB must not pop one (and
  // re-flow the row). Pump direction only with trailer mode on.
  const LEVEL_ALERT   = { MAX: 1, FAULT: 1 }
  const LEVEL_TRAILER = { RAISE: 1, LOWER: 1, MAX: 1, FAULT: 1 }

  // rear diff lock chip colour per preset (etk854hybrid_diffLock.lua)
  const DIFF_COLOR = { SPORT: C.blue, TRACK: C.amber, DRIFT: C.red }

  // ECO PRO pedal ruler: segment colours and zone labels (pedalZone 1..5)
  const RUL_COLOR = [C.blue, C.blueLt, C.text, C.amber, C.red]
  const ZONE_LABEL = ['', 'EV', 'EV FULL', 'HYBRID', 'ICE', 'KICKDOWN']
  const ZONE_COLOR = [C.muted, C.blue, C.blueLt, C.text, C.amber, C.red]

  const clamp01 = v => v < 0 ? 0 : (v > 1 ? 1 : v)
  const num = v => (v === undefined || v === null || isNaN(Number(v))) ? null : Number(v)
  const now = () => (window.performance && performance.now) ? performance.now() : Date.now()

  return {
    template: TPL,
    replace: true,
    restrict: 'EA',

    link: function (scope, element) {
      const streamsList = ['electrics']
      StreamsManager.add(streamsList)
      scope.$on('$destroy', function () { StreamsManager.remove(streamsList) })

      // element-scoped lookup keeps multiple instances independent
      const R = {}
      const found = element[0].querySelectorAll('[data-k]')
      for (let i = 0; i < found.length; i++) R[found[i].getAttribute('data-k')] = found[i]

      // attribute memo: a write equal to the last one is skipped (most values
      // are steady); every attribute set() touches is written only here
      const attrMemo = {}
      const set = (k, a, v) => {
        const e = R[k]; if (!e) return
        const m = attrMemo[k] || (attrMemo[k] = {})
        if (m[a] === v) return
        m[a] = v
        e.setAttribute(a, v)
      }
      const txt  = (k, v)    => { const e = R[k]; if (e && e.textContent !== v) e.textContent = v }
      const show = (k, on)   => set(k, 'opacity', on ? 1 : 0)

      // dash animation phase, advanced from the stream tick (no rAF loop that
      // could outlive the app)
      let dash = 0
      let lastT = now()

      let lastPhase = null

      // learned ICE ring scale (see ICE_PEAK_FLOOR_KW)
      let icePeakKw = ICE_PEAK_FLOOR_KW
      const resetIcePeak = () => { icePeakKw = ICE_PEAK_FLOOR_KW }
      scope.$on('VehicleFocusChanged', resetIcePeak)
      scope.$on('VehicleChange', resetIcePeak)
      // stability-badge memo (avoids touching the DOM 60x/s)
      let lastDsc = null, lastDseActing = null, lastBlink = null

      // engaged gear + shift recommendation for the program chip; '' when the
      // cluster module (etk854hybrid_gearActual) is not fitted
      function hudGear(e) {
        const g = e.gear
        const a = num(e.etk854hybrid_gearActual)
        if (typeof g !== 'string' || g === '' || a === null || a < -50) return ''
        const c = g.charAt(0).toUpperCase()
        if (c === 'P' || c === 'R' || c === 'N') return c
        if (c !== 'D' && c !== 'S' && c !== 'M') return ''
        let s = c + (a >= 1 ? String(Math.round(a)) : '')
        const h = Math.round(num(e.etk854hybrid_gearHint) || 0)
        if (h >= 1 && a >= 1 && h !== Math.round(a)) s += (h > a ? '▲' : '▼') + h
        return s
      }

      // chip memo: layout signature + per-chip text/colour key
      let chipLayout = null
      const chipKey = {}
      const chipDef = {}
      for (let i = 0; i < CHIPS.length; i++) chipDef[CHIPS[i].key] = CHIPS[i]
      // squeeze a label that is wider than its fixed chip (measured once per
      // label change; the rendered width also covers a fallback font)
      function fitChip(key, label) {
        const el = R[key + 'txt'], c = chipDef[key]
        if (!el || !c) return
        el.removeAttribute('textLength')
        el.removeAttribute('lengthAdjust')
        const avail = c.w - (c.dot ? 14 : 8)
        let w = 0
        try { w = el.getComputedTextLength() } catch (err) { w = label.length * 3.9 }
        if (w > avail) {
          el.setAttribute('textLength', avail)
          el.setAttribute('lengthAdjust', 'spacingAndGlyphs')
        }
      }
      function setChip(key, label, color) {
        const k = label === null ? null : (label + '|' + color)
        if (chipKey[key] === k) return
        const lblChanged = label !== null && (chipKey[key] == null || chipKey[key].split('|')[0] !== label)
        chipKey[key] = k
        if (label === null) return
        txt(key + 'txt', label)
        if (lblChanged) fitChip(key, label)
        set(key + 'txt', 'fill', color === C.dim ? C.muted : C.text)
        set(key + 'box', 'stroke', color)
      }

      // ECO PRO pedal ruler
      const rx = p => Math.round((RUL_X + RUL_W * clamp01(p)) * 10) / 10
      let rulOn = null
      // ECO EV ceiling (evZoneOpen) debounced 0.5 s: the VCU compares the
      // raw speed with 120 km/h, cruising at the ceiling would flicker
      const evGate = { open: true, since: 0 }
      function updateRuler(e, de, iceOn) {
        const pJ = num(e.etk854hybrid_pedalIceJoin) || 0
        const on = de === 'ECO_PRO' && pJ > 0
        if (on !== rulOn) {
          rulOn = on
          evGate.open = (num(e.etk854hybrid_evZoneOpen) || 0) > 0.5
          evGate.since = 0
          show('rul', on)
          set('stLeft', 'y', on ? ST_Y_RULER : ST_Y)
          set('stRight', 'y', on ? ST_Y_RULER : ST_Y)
        }
        if (!on) return
        // thresholds (VCU mapPedal): EV edge, join, kick, urgent
        let pE = num(e.etk854hybrid_pedalEvEdge)
        if (!(pE > 0 && pE <= pJ)) pE = Math.max(0, pJ - 0.05)
        let pK = num(e.etk854hybrid_pedalKickOn)
        if (!(pK >= pJ)) pK = pJ
        let pU = num(e.etk854hybrid_pedalUrgent)
        if (!(pU >= pK && pU <= 1)) pU = Math.max(pK, 0.95)
        const openRaw = (num(e.etk854hybrid_evZoneOpen) || 0) > 0.5
        const tG = now()
        if (openRaw === evGate.open) evGate.since = 0
        else if (!evGate.since) evGate.since = tG
        else if (tG - evGate.since >= 500) { evGate.open = openRaw; evGate.since = 0 }
        const evOpen = evGate.open
        const edges = [0, pE, pJ, pK, pU, 1]
        // pedal the VCU maps: throttle, or throttle_input while the gearbox
        // caps the throttle during a shift (etk854hybridControl.lua updateGFX)
        let ped = clamp01(num(e.throttle) || 0)
        const pin = num(e.throttle_input)
        if ((num(e.isShifting) || 0) > 0.5 && pin !== null && pin > ped + 0.01) ped = clamp01(pin)
        for (let i = 0; i < RUL_SEGS; i++) {
          const a = edges[i], b = edges[i + 1]
          const x0 = rx(a), x1 = rx(b)
          const col = (i < 2 && !evOpen) ? C.dim : RUL_COLOR[i]
          // 0.6 u gap between segments keeps the thresholds readable
          const w = Math.max(0, x1 - x0 - (i < RUL_SEGS - 1 ? 0.6 : 0))
          set('rulB' + i, 'x', x0); set('rulB' + i, 'width', w); set('rulB' + i, 'fill', col)
          const lw = ped > a ? Math.min(w, Math.round((rx(ped) - x0) * 10) / 10) : 0
          set('rulL' + i, 'x', x0); set('rulL' + i, 'width', Math.max(0, lw)); set('rulL' + i, 'fill', col)
        }
        // load integral towards the ICE start, drawn into the hybrid band
        const dOn = num(e.etk854hybrid_iceDeficitOnS) || 0
        const dS = num(e.etk854hybrid_iceDeficitS) || 0
        const fr = (dOn > 0 && !iceOn) ? clamp01(dS / dOn) : 0
        const bw = Math.max(0, rx(pK) - rx(pJ) - 0.6)
        set('rulDef', 'x', rx(pJ)); set('rulDef', 'width', Math.round(bw * fr * 10) / 10)
        // segment labels: plateau "MAX" (light blue), kickdown "KD" (red)
        set('rulMax', 'x', Math.round((rx(pE) + rx(pJ)) * 5) / 10)
        set('rulKd', 'x', Math.round((rx(pU) + rx(1)) * 5) / 10)
        const cx = rx(ped)
        set('rulCar', 'x1', cx); set('rulCar', 'x2', cx)
        set('rulCarB', 'x1', cx); set('rulCarB', 'x2', cx)
        // right-hand label from the VCU zone
        const zone = Math.max(0, Math.min(5, Math.round(num(e.etk854hybrid_pedalZone) || 0)))
        let zc = ZONE_COLOR[zone]
        if (zone <= 2 && !evOpen) zc = C.muted
        else if (zone === 3 && dS > 0.02 && !iceOn) zc = C.amber
        txt('rulLbl', ZONE_LABEL[zone])
        set('rulLbl', 'fill', zc)
      }
      function layoutChips() {
        let sig = ''
        for (let i = 0; i < CHIPS.length; i++) sig += chipKey[CHIPS[i].key] === null ? '0' : '1'
        if (sig === chipLayout) return
        chipLayout = sig
        let x = BAR_X
        for (let i = 0; i < CHIPS.length; i++) {
          const c = CHIPS[i]
          // a chip that would run past the SOC bar's right end is hidden
          const on = sig.charAt(i) === '1' && x + c.w <= BAR_X + BAR_W
          show(c.key, on)
          if (!on) continue
          set(c.key + 'box', 'x', x)
          set(c.key + 'txt', 'x', c.dot ? x + (c.w - DOT_INSET + 1) / 2 : x + c.w / 2)
          if (c.dot) set(c.key + 'dot', 'cx', x + c.w - DOT_INSET)
          x += c.w + CHIP_GAP
        }
      }

      // AIR chip (null = no receiver). AIR LOW = below the 4.14 bar spring-brake
      // release; TRL BRK = trailer spring brakes hold with our parking brake
      // off; amber = protection valve shut; dot = compressor running.
      let lastAirDot = null, lastDiffDot = null
      function airChip(e) {
        const st = e.etk854hybrid_airState
        if (typeof st !== 'string' || st === '' || st === 'NO TANK') return null
        const bar = num(e.etk854hybrid_airPressureBar)
        const running = (num(e.etk854hybrid_airCompressor) || 0) > 0.5
        const linked = (num(e.etk854hybrid_airTrailer) || 0) > 0.5
        const spring = num(e.etk854hybrid_trailerSpringCoef)
        const pbOn = (num(e.parkingbrake) || 0) > 0.5
        const shut = num(e.etk854hybrid_airTrlValve) === 0
        if (st === 'LOW') return { label: 'AIR LOW', color: C.amber, running: running }
        if (linked && spring !== null && spring > 0.5 && !pbOn) return { label: 'TRL BRK', color: C.amber, running: running }
        if (e.etk854hybrid_airInhibit === 'SWITCH') return { label: 'AIR OFF', color: C.dim, running: running }
        const barTxt = bar === null ? '--' : Math.max(0, bar).toFixed(1)
        return { label: 'AIR ' + barTxt, color: shut ? C.amber : (running ? C.blueLt : (linked ? C.blue : C.dim)), running: running }
      }

      function setLink(key, dir, color) {
        // dir: 1 = forward (left->right), -1 = reverse, 0 = idle
        show(key + 'fwd',  dir ===  1)
        show(key + 'fwdA', dir ===  1)
        show(key + 'rev',  dir === -1)
        show(key + 'revA', dir === -1)
        if (dir === 1) {
          set(key + 'fwd',  'stroke', color)
          set(key + 'fwdA', 'fill',   color)
          set(key + 'fwd',  'stroke-dashoffset', -dash)
        } else if (dir === -1) {
          set(key + 'rev',  'stroke', color)
          set(key + 'revA', 'fill',   color)
          set(key + 'rev',  'stroke-dashoffset', -dash)
        }
      }

      function setNode(key, active, color) {
        set(key + 'box', 'stroke', active ? color : C.dim)
        set(key + 'box', 'fill',   active ? 'rgba(255,255,255,0.075)' : 'rgba(255,255,255,0.045)')
        set(key + 'lbl', 'fill',   active ? C.text : C.muted)
      }

      // ------------------------------------------------------------- SOC
      // batterySoc is percent; fraction fallback + rescale as a guard
      function updateSoc(e, edrive) {
        let soc = num(e.etk854hybrid_batterySoc)
        if (soc === null) {
          const frac = num(e.etk854hybrid_batteryFraction)
          soc = frac === null ? null : frac * 100
        }
        if (soc !== null && soc <= 1.5) soc *= 100
        const socPct = soc === null ? 0 : Math.max(0, Math.min(100, soc))

        txt('socpct', soc === null ? '--%' : Math.round(socPct) + '%')
        set('socbar', 'width', Math.max(0.001, BAR_W * socPct / 100))
        set('socbar', 'fill', socPct < 12 ? C.red : 'url(#etk854hud-soc)')

        // target SOC marker: only meaningful in BATTERY CONTROL
        let tgt = num(e.etk854hybrid_targetSoc)              // published as 0..1
        if (tgt !== null && tgt > 1.5) tgt /= 100
        const showTgt = (edrive === 'BATTERY_CONTROL') && tgt !== null
        show('socTgt', showTgt)
        if (showTgt) {
          set('socTgt', 'x', Math.min(BAR_X + BAR_W - 2, BAR_X + BAR_W * clamp01(tgt)))
        }
      }

      // ----------------------------------------------------------- rings
      function updateRings(iceKw, evKw, totalKw, boost) {
        const evPeak    = STAGE_KW[boost]
        const regenPeak = STAGE_REGEN[boost]

        const iceFrac = clamp01(iceKw / icePeakKw)
        set('arcIce', 'stroke-dashoffset', LEN_OUT * (1 - iceFrac))

        const evDrive = evKw > 0 ? clamp01(evKw / evPeak) : 0
        const evRegen = evKw < 0 ? clamp01(-evKw / regenPeak) : 0
        set('arcEv',    'stroke-dashoffset', LEN_IN * (1 - evDrive))
        set('arcRegen', 'stroke-dashoffset', LEN_IN * (1 - evRegen))

        const totalStr = String(Math.round(totalKw))
        txt('total', totalStr)
        // keep the number clear of the inner ring as it grows
        set('total', 'font-size', totalStr.length <= 2 ? 24 : (totalStr.length === 3 ? 21 : 18))
        set('total', 'fill', totalKw < -0.5 ? C.green : C.text)
      }

      // ------------------------------------------------------- readouts
      function updateReadouts(e, iceKw, evKw) {
        txt('icekw', String(Math.round(iceKw)))
        txt('evkw',  String(Math.round(evKw)))
        const iceNm = num(e.etk854hybrid_iceTorqueNm) || 0
        const evNm  = num(e.etk854hybrid_evTorqueNm)  || 0
        txt('icenm', Math.round(iceNm) + ' Nm')
        txt('evnm',  Math.round(evNm)  + ' Nm')

        const evNeg = evKw < -0.5
        set('evkw',  'fill', evNeg ? C.green : C.blue)
        set('emdot', 'fill', evNeg ? C.green : C.blue)

        // recuperation: power preferred, torque as the sub-line source
        const regenNm = Math.abs(num(e.etk854hybrid_regenTorqueNm) || 0)
        const recKw = evKw < 0 ? -evKw : 0
        const recActive = recKw > 0.5 || regenNm > 1
        txt('reckw', recActive ? (Math.round(recKw) + ' kW') : '--')
        set('reckw',  'fill', recActive ? C.green : C.dim)
        set('recdot', 'fill', recActive ? C.green : C.dim)

        // selector position (Alt+R): OFF / LOW / MED / HIGH, HIGH = one-pedal
        const regenLvl = String(e.etk854hybrid_regenLevel || 'MED').toUpperCase()
        const opHold = (num(e.etk854hybrid_onePedalHold) || 0) > 0.5
        txt('reclvl', opHold ? (regenLvl + ' HOLD') : regenLvl)
        set('reclvl', 'fill', regenLvl === 'OFF' ? C.dim
                            : (opHold ? C.green : (recActive ? C.green : C.muted)))
      }

      // ---------------------------------------------------------- phase
      function updatePhase(phaseRaw, sail, boost, iceKw, evKw) {
        let pKey = sail ? 'SAIL' : (PHASE[phaseRaw] ? phaseRaw : 'IDLE')
        // "HYBRID" only earns the boost wording when both units actually pull
        const p = PHASE[pKey]
        let pLabel = p.label
        if (pKey === 'HYBRID' && !(iceKw > 1 && evKw > 1)) pLabel = 'HYBRID'
        if (boost > 0 && (pKey === 'HYBRID' || pKey === 'EV')) pLabel += ' S' + boost

        if (pLabel !== lastPhase) {
          lastPhase = pLabel
          txt('phase', pLabel)
          set('phase', 'fill', p.color)
          set('phasebox', 'stroke', p.color)
          set('phasebox', 'fill', p.soft)
        }
      }

      // ------------------------------------------------------ power flow
      // from the numbers, not the phase label, so it matches the readouts;
      // then the K0 clutch pack
      function updateFlow(e, sail, iceKw, evKw, totalKw, iceOn, k0) {
        const iceFlow  = !sail && iceKw > 1
        const battOut  = !sail && evKw >  1       // battery -> e-motor
        const battIn   = !sail && evKw < -1       // e-motor -> battery
        const driveOut = !sail && totalKw >  1    // e-motor -> wheels
        const driveIn  = !sail && totalKw < -1    // wheels  -> e-motor

        setLink('lb', battOut ? 1 : (battIn ? -1 : 0), battIn ? C.green : C.blue)
        setLink('lw', driveOut ? 1 : (driveIn ? -1 : 0), driveIn ? C.green : C.blue)

        const evNeg = evKw < -0.5
        const restarting = (num(e.etk854hybrid_iceRestartMs) || 0) > 0
        setNode('ice',   iceOn || restarting, restarting ? C.blueLt : C.amber)
        setNode('em',    !sail && (Math.abs(evKw) > 0.5), evNeg ? C.green : C.blue)
        setNode('batt',  battOut || battIn, battIn ? C.green : C.blue)
        setNode('wheel', driveOut || driveIn, driveIn ? C.green : C.blue)
        txt('icelbl', restarting ? 'START' : 'ICE')

        const k0Col = k0 > 0.9 ? (iceFlow ? C.amber : C.muted) : (k0 > 0.1 ? C.amber : C.dim)
        set('k0up',   'stroke', k0Col)
        set('k0down', 'stroke', k0 > 0.9 ? k0Col : C.dim)
        set('k0pA',   'stroke', k0Col)
        set('k0pB',   'stroke', k0Col)
        set('k0lbl',  'fill',   k0 > 0.1 ? C.muted : C.dim)
        // plates separate as the clutch opens (2.5 px -> 6 px half-gap)
        const gap = 2.5 + (1 - k0) * 3.5
        set('k0pA', 'y1', 163 - gap); set('k0pA', 'y2', 163 - gap)
        set('k0pB', 'y1', 163 + gap); set('k0pB', 'y2', 163 + gap)
        set('k0up',   'y2', 163 - gap - 1.5)
        set('k0down', 'y1', 163 + gap + 1.5)
      }

      // ------------------------------------------- stability system badge
      // dscMode = arming state; blinks while intervening: dscLamp (qualified,
      // cluster module) or, without that module, the raw tcActive
      function updateDscBadge(e) {
        const dscKey = String(e.etk854hybrid_dscMode || 'DSC').toUpperCase()
        const ds = DSC_STYLE[dscKey] || DSC_STYLE.DSC
        const dscLamp = num(e.etk854hybrid_dscLamp)
        const dseActing = dscLamp !== null
          ? dscLamp > 0.5
          : (num(e.etk854hybrid_tcActive) || 0) > 0.5
        // `dash` runs 0..11 at 26 units/s -> one blink cycle every ~0.42 s
        const blinkOn = dseActing && dash < 5.5
        if (dscKey !== lastDsc || dseActing !== lastDseActing || blinkOn !== lastBlink) {
          lastDsc = dscKey; lastDseActing = dseActing; lastBlink = blinkOn
          txt('dsc', dscKey)
          set('dsc',    'fill',   blinkOn ? C.text : ds.text)
          set('dscbox', 'stroke', blinkOn ? C.amber : ds.color)
          set('dscbox', 'fill',   blinkOn ? 'rgba(255,166,43,0.30)' : ds.soft)
        }
      }

      // --------------------------------------------------- status chips
      // nil channel (part not fitted) -> chip hidden
      function updateChips(e) {
        // Program + gear + recommendation ("MANUAL · M3▲4"); gear needs the
        // cluster module
        const sp = e.etk854hybrid_shiftProgram
        const spTxt = (typeof sp === 'string' && sp !== '') ? sp.toUpperCase().replace(/_/g, ' ').slice(0, 10) : ''
        const gTxt = hudGear(e)
        const progChip = spTxt !== '' && gTxt !== '' ? spTxt + ' · ' + gTxt : (spTxt !== '' ? spTxt : null)
        setChip('cProg', progChip, C.blueLt)

        const awd = e.etk854hybrid_awdMode
        const awdStr = (typeof awd === 'string' && awd !== '') ? awd.toUpperCase() : null
        setChip('cAwd', awdStr, awdStr === 'AWD' ? C.blue : C.dim)

        // rear diff lock in 5 % steps (less DOM churn), hidden while OPEN;
        // dot = slip hold engaged
        const dp = e.etk854hybrid_diffPreset
        const diffOn = typeof dp === 'string' && dp !== '' && dp !== 'OPEN'
        setChip('cDiff', diffOn ? 'LSD ' + (Math.round(20 * clamp01(num(e.etk854hybrid_diffLock) || 0)) * 5) + '%' : null,
                diffOn ? (DIFF_COLOR[dp] || C.blue) : C.dim)
        const diffDot = diffOn && (num(e.etk854hybrid_diffHold) || 0) > 0.5
        if (diffDot !== lastDiffDot) { lastDiffDot = diffDot; show('cDiffdot', diffDot) }

        // TRAILER / TRL <state> / LEVEL <alert> (state cut to fit 48 u);
        // overridden by TSA while sway or corner stage >= 1 (red = TSA lamp)
        // and by TSA FAULT in trailer mode
        const trl = e.etk854hybrid_trailerMode
        const trlOn = trl === true || (num(trl) || 0) > 0.5
        const lvl = typeof e.etk854hybrid_levelState === 'string'
          ? e.etk854hybrid_levelState.toUpperCase() : ''
        const tsSt = num(e.etk854hybrid_tsaStage) || 0, tsK = num(e.etk854hybrid_tsaCorner) || 0
        const tsL = (num(e.etk854hybrid_tsaLamp) || 0) > 0.5
        const tsState = typeof e.etk854hybrid_tsaState === 'string' ? e.etk854hybrid_tsaState : ''
        let trlLabel = trlOn ? (LEVEL_TRAILER[lvl] ? 'TRL ' + lvl.slice(0, 5) : 'TRAILER')
          : (LEVEL_ALERT[lvl] ? 'LEVEL ' + lvl.slice(0, 5) : null)
        let trlColor = C.amber
        if (tsSt >= 1 || tsK >= 1) { trlLabel = 'TSA'; trlColor = tsL ? C.red : C.amber }
        else if (trlOn && tsState === 'FAULT') { trlLabel = 'TSA FAULT'; trlColor = C.red }
        setChip('cTrl', trlLabel, trlColor)

        const air = airChip(e)
        setChip('cAir', air ? air.label : null, air ? air.color : C.dim)
        const airDot = !!(air && air.running)
        if (airDot !== lastAirDot) { lastAirDot = airDot; show('cAirdot', airDot) }

        const ies = e.etk854hybrid_iceEngageState
        const iesStr = (typeof ies === 'string' && ies !== '') ? ies.toUpperCase() : null
        setChip('cIce', iesStr === null ? null : 'ICE ' + iesStr, ICE_ENGAGE_COLOR[iesStr] || C.muted)

        const fan = num(e.etk854hybrid_fanStage)
        setChip('cFan', (fan !== null && fan > 0) ? 'FAN ' + Math.round(fan) : null, C.blueLt)

        layoutChips()
      }

      // ---------------------------------------------------- status line
      function updateStatus(e, edrive, k0, boost, sail) {
        const de = String(e.etk854hybrid_deMode || 'COMFORT').replace('_', ' ')
        txt('stLeft', de + ' · ' + (EDRIVE_LABEL[edrive] || edrive))

        let right = 'K0 ' + (k0 > 0.9 ? 'CLOSED' : (k0 > 0.1 ? 'SLIP' : 'OPEN'))
        if (boost > 0) right += ' · BOOST S' + boost
        const sailState = String(e.etk854hybrid_sailState || '')
        if (sail && sailState && sailState !== 'SAIL') right += ' · ' + sailState
        txt('stRight', right)
      }

      scope.$on('streamsUpdate', function (event, streams) {
        const e = streams && streams.electrics
        if (!e) { return }

        const nowT = now()
        let dt = (nowT - lastT) / 1000
        lastT = nowT
        if (!(dt > 0) || dt > 0.5) dt = 0.016
        dash = (dash + dt * 26) % 11        // 11 = dash(5) + gap(6)

        // ---------------------------------------------------------- inputs
        const boost = Math.max(0, Math.min(3, Math.round(num(e.etk854hybrid_boostStage) || 0)))
        const iceKw = num(e.etk854hybrid_icePower) || 0
        const evKw  = num(e.etk854hybrid_evPower)  || 0
        const totalKw = iceKw + evKw

        // ICE ring scale: running max with slow decay towards the floor
        let iceCandKw = iceKw
        const iceCapNm = num(e.etk854hybrid_iceMaxTorqueNm)
        const iceRpm   = num(e.rpm)
        if (iceKw > 1 && iceCapNm !== null && iceCapNm > 0 && iceRpm !== null && iceRpm > 0) {
          iceCandKw = Math.max(iceCandKw, iceCapNm * iceRpm * Math.PI / 30000)
        }
        if (iceCandKw > icePeakKw) icePeakKw = iceCandKw
        else icePeakKw -= (icePeakKw - ICE_PEAK_FLOOR_KW) * Math.min(1, dt / ICE_PEAK_TAU)

        const edrive = String(e.etk854hybrid_edriveMode || 'AUTO_EDRIVE')
        const phaseRaw = String(e.etk854hybrid_phase || 'IDLE').toUpperCase()
        const sail = (num(e.etk854hybrid_sailActive) || 0) > 0.5 || phaseRaw === 'SAIL'
        const iceOn = (num(e.etk854hybrid_iceState) || 0) > 0.5
        // K0 clutch: 1 = closed (ICE coupled), 0 = open (decoupled)
        const k0Raw = num(e.etk854hybrid_k0ClutchRatio)
        const k0 = clamp01(k0Raw === null ? (iceOn ? 1 : 0) : k0Raw)

        // sections in draw order
        updateSoc(e, edrive)
        updateRings(iceKw, evKw, totalKw, boost)
        updateReadouts(e, iceKw, evKw)
        updatePhase(phaseRaw, sail, boost, iceKw, evKw)
        updateFlow(e, sail, iceKw, evKw, totalKw, iceOn, k0)
        updateDscBadge(e)
        updateChips(e)
        updateRuler(e, String(e.etk854hybrid_deMode || ''), iceOn)
        updateStatus(e, edrive, k0, boost, sail)
      })
    }
  }
}])
