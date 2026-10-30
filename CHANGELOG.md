# Changelog

**1.1.0: rear-axle steering, trailer stability, cockpit panel**

- Optional active rear-axle steering: counter-phase below about 60 km/h (40 km/h in trailer mode), in-phase above, DSC yaw reference corrected, stability assist; Actions app row R-STEER OFF / ON + DSC / ON NO DSC, DSC OFF switches it off
- New models 40e Perf. xAllrad Rear Steer and 35d Perf. xAllrad Rear Steer
- Trailer stability function of the DSC with both tow hitches: sway stages 1-3, corner pre-braking, force-sensing hitch, trailer mass and tongue load estimate, TRAILER page on the centre screen, TSA chip in the HUD
- Trailer-mode DSC retuned after my own drive with a 2.9 t cargo trailer (it held on too long and was a bit too sensitive): eleven corrections
- New model 35d Perf. xAllrad Hauling: rear-axle steering, 3.5 t heavy-duty hitch with ball and pintle hook, trunk air compressor, trailer stability
- Cockpit: centre-screen views ENERGY / MAP / TRAILER, cluster views AUTO / HYBRID / EFFICIENCY / SPORT / MINIMAL, MENU control panel with click zones and rotary-controller keys (Alt+H, Alt+, / Alt+., Alt+J, Alt+Backspace, Alt+V, Alt+C), DISPLAY row in the Actions app
- Watchdog reloads a cockpit page that freezes after a vehicle swap
- Vehicle selector: short model names, short factual descriptions, new thumbnails with badges
- Codebase cleanup without behaviour change; tested on BeamNG.drive 0.40.3

**1.0.0: first public release** (repository edition, includes feedback rounds 2 and 3)

- English in-game texts, neutral naming (xAllrad, EV modes), developer button removed, paint fix, licence headers, fixed file name
- New launch control with generator load and auto-arm; brake-throttle guard in every mode
- EV creep with standstill hold; ECO PRO pedal map v3; no decoupled idle in ECO PRO
- Torque-first shift program, SPORT without overdrive, quicker shifts, no long run-on after one kickdown
- Limited-slip rear differential on every model; bigger turbos and Performance cooling on the Performance models
- Trailer braking 1:1, trailer air supply with protection valve; on-board computer, ECO PRO ruler, LSD and AIR chips
- Fixes: seven key actions now work (incl. Alt+PgUp / PgDn, Alt+G, Alt+T, Alt+W), launch-control fixes (290 Nm generator load in the preload), LSD opens while the car is held at a standstill, sailing only from 10 % charge, e-machine works after an empty-battery spawn, HUD lag at high frame rates

### Internal test builds (0.1.0 to 0.9.1)

- **0.9.1** (Sep 2026): feedback round 1: EV pedal zone, prompt engine stop, gear recommendation, DSC light, audible fan
- **0.9.0** (Sep 2026): release candidate with all three engine lines, Performance and xAllrad models
- **0.8.0** (Sep 2026): optimisation run: rolling K0 starts, shift polish, lift-off ramp, I4 calibration
- **0.7.1** (Sep 2026): instrument cluster and centre screen, energy flow display
- **0.7.0** (Sep 2026): engine start sequencer, 8HP-style shift program, towing, cooling package, Actions app
- **0.6.1** (Aug 2026): DSC / DTC, recuperation levels, B58S Performance, xAllrad models
- **0.6.0** (Aug 2026): adaptive chassis, N57S Performance
- **0.5.3** (Aug 2026): MAX EV, e-boost stages, Hybrid System tuning menu
- **0.5.2** (Aug 2026): N57D30TOP twin-turbo diesel, system torque cap
- **0.5.1** (Aug 2026): e-machine sound, reverse fix, forest green paint
- **0.5.0** (Aug 2026): ActiveHybrid rebuild: B58, two mode axes, sailing, new HUD
- **0.4.0** (Jul 2026): clean rebuild, working key bindings, test rig as a separate part
- **0.3.1** (May 2026): automated in-game test rig
- **0.3.0** (May 2026): HUD and first cluster, pedestrian warning sound, launch control
- **0.2.1** (May 2026): five drive modes, electric driving from standstill
- **0.2.0** (May 2026): own e-machine device with K0 clutch
- **0.1.0** (Jan 2026): first draft: P2 hybrid concept for the ETK 800
