// app.js — canvas renderer, keyboard/touch input, scenario editor.
import { JEANNEAU_36, DEG, KN, stepBoat, initialState, clampThrottle, rpmSetpoint, sumForces, propThrust, shaftToEngine } from './physics.js';
import { WindModel, WIND_PRESETS, mulberry32 } from './wind.js';
import { contactForces, sampleOutline } from './collision.js';

const P = JEANNEAU_36;
const H = 0.01;                           // physics step [s]
// Tolerant lookup: a missing element logs a warning and returns a detached
// stand-in instead of null, so one stale index.html can't abort the module.
const $ = id => document.getElementById(id) ?? (console.warn(`sailsim: no element #${id} — is index.html up to date?`), document.createElement('div'));
const canvas = $('c'), ctx = canvas.getContext('2d');

// ---------------------------------------------------------------- scenario
// world: E to the right, N up, metres.  Optional background image with
// pxPerM; image bottom-left is world (0,0).
function defaultMarina() {
  const docks = [];
  const W = 130, Hm = 90;
  docks.push([[0, 0], [W, 0], [W, 2.5], [0, 2.5]]);                 // main pier along south
  for (let i = 0; i < 9; i++) {                                     // finger piers
    const x = 12 + i * 12;
    docks.push([[x, 2.5], [x + 1.0, 2.5], [x + 1.0, 15], [x, 15]]);
  }
  docks.push([[0, Hm - 2.5], [W, Hm - 2.5], [W, Hm], [0, Hm]]);     // breakwater north
  for (let i = 0; i < 6; i++) {                                     // north fingers
    const x = 25 + i * 14;
    docks.push([[x, Hm - 16], [x + 1.0, Hm - 16], [x + 1.0, Hm - 2.5], [x, Hm - 2.5]]);
  }
  docks.push([[0, 0], [2.5, 0], [2.5, Hm], [0, Hm]]);               // west quay
  return { widthM: W, heightM: Hm, pxPerM: null, image: null, docks, water: [], start: { E: 65, N: 45, psi: 90 } };
}

const sim = {
  scenario: defaultMarina(),
  s: null, t: 0, paused: false,
  wheelCmd: 0, heldSince: 0, throttleIndex: 0,
  wind: new WindModel(WIND_PRESETS.moderate, (Math.random() * 1e9) | 0),
  windVec: { E: 0, N: 0 },
  hullPts: sampleOutline(P.outline),
  sprite: null, spriteLengthM: P.LOA,
  bgImg: null,
  view: { s: 8, cE: 0, cN: 0, follow: false, boatUp: false },
  mode: 'sail',                         // 'sail' | 'scale' | 'dock' | 'place'
  draft: [], undoStack: [], lastContact: 0, hits: 0, trail: [],
  forces: null,
};
window.sim = sim;

// Start-up self-check of the contact pipeline: a box over the bow must
// produce finite, non-zero force.  Fails loudly if a stale collision.js is served.
{
  const probe = contactForces(initialState(0, 0, 0), sim.hullPts, [[[-1, 4.5], [1, 4.5], [1, 7], [-1, 7]]], []);   // box over the bow
  if (!probe.contacts.length || !Number.isFinite(probe.X)) {
    alert('Contact model self-check failed: src/collision.js is stale or mismatched. Hard-reload (Ctrl+Shift+R) or re-copy all files from the zip.');
  }
}
// Console helper: sim.checkContact() reports polygons and current contacts.
sim.checkContact = () => {
  const sc = sim.scenario, c = contactForces(sim.s, sim.hullPts, sc.docks, sc.water);
  const area = poly => Math.abs(poly.reduce((a, [x, y], i) => { const [x2, y2] = poly[(i + 1) % poly.length]; return a + x * y2 - x2 * y; }, 0) / 2);
  console.table([...sc.docks.map((p, i) => ({ kind: 'land', i, vertices: p.length, area_m2: area(p).toFixed(1) })),
                 ...sc.water.map((p, i) => ({ kind: 'water', i, vertices: p.length, area_m2: area(p).toFixed(1) }))]);
  console.log('boat at', sim.s.slice(0, 2).map(x => x.toFixed(1)), 'contacts now', c.contacts.length, 'X Y N', c.X.toFixed(0), c.Y.toFixed(0), c.N.toFixed(0));
  return c;
};

function resetBoat() {
  const st = sim.scenario.start;
  sim.s = initialState(st.E, st.N, st.psi);
  sim.t = 0; sim.throttleIndex = 0; sim.wheelCmd = 0; sim.hits = 0; sim.trail = [];
}
function zoomOnBoat(f) { sim.view.s *= f; sim.view.cE = sim.s[0]; sim.view.cN = sim.s[1]; }
function fitView() {
  const sc = sim.scenario;
  sim.view.s = Math.min(canvas.width / sc.widthM, canvas.height / sc.heightM) * 0.95;
  sim.view.cE = sc.widthM / 2; sim.view.cN = sc.heightM / 2;
}

// ---------------------------------------------------------------- input
const keys = {};
addEventListener('keydown', e => {
  if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
  if (e.repeat) return;
  keys[e.key] = true;
  if (e.key === 'ArrowUp') { sim.throttleIndex = clampThrottle(sim.throttleIndex + 1, P); e.preventDefault(); }
  if (e.key === 'ArrowDown') { sim.throttleIndex = clampThrottle(sim.throttleIndex - 1, P); e.preventDefault(); }
  if (e.key === ' ') { sim.paused = !sim.paused; e.preventDefault(); }
  if (e.key === 'r') resetBoat();
  if (e.key === 'c') sim.s[6] = 0;
  if (e.key === 'n') newRandomWind();
  if (e.key === 'f') fitView();
  if (e.key === 'b') $('bBoatUp').onclick();
  if (e.key === '+' || e.key === '=') zoomOnBoat(1.25);
  if (e.key === '-' || e.key === '_') zoomOnBoat(1 / 1.25);
});
addEventListener('keyup', e => { keys[e.key] = false; });
function bindHold(id, on, off) {
  const el = $(id); const start = e => { e.preventDefault(); on(); }; const end = e => { e.preventDefault(); off(); };
  el.addEventListener('pointerdown', start); el.addEventListener('pointerup', end); el.addEventListener('pointerleave', end);
}
bindHold('bPort', () => keys.z = true, () => keys.z = false);
bindHold('bStbd', () => keys.x = true, () => keys.x = false);
$('bUp').addEventListener('pointerdown', e => { e.preventDefault(); sim.throttleIndex = clampThrottle(sim.throttleIndex + 1, P); });
$('bDown').addEventListener('pointerdown', e => { e.preventDefault(); sim.throttleIndex = clampThrottle(sim.throttleIndex - 1, P); });

function pollWheel(now) {
  const cmd = (keys.z ? -1 : 0) + (keys.x ? 1 : 0);
  if (cmd !== sim.wheelCmd) { sim.wheelCmd = cmd; sim.heldSince = now; }
}

// ---------------------------------------------------------------- physics loop
function extraForces(s) {
  const c = contactForces(s, sim.hullPts, sim.scenario.docks, sim.scenario.water);
  if (c.contacts.length) sim.lastContact = sim.t;
  return c;
}
function physicsStep(now) {
  pollWheel(now);
  sim.wind.step(H);
  const w = sim.wind.state; sim.windVec = { E: w.E, N: w.N };
  const inp = { wheelCmd: sim.wheelCmd, heldTime: (now - sim.heldSince) / 1000, throttleIndex: sim.throttleIndex, wind: sim.windVec, extra: extraForces };
  const before = sim.lastContact;
  sim.s = stepBoat(sim.s, inp, P, H);
  if (sim.lastContact !== before && sim.t - before > 1.0) sim.hits++;
  sim.t += H;
  sim.forces = sumForces(sim.s, inp, P);
  if (Math.round(sim.t / H) % 50 === 0) { sim.trail.push([sim.s[0], sim.s[1]]); if (sim.trail.length > 600) sim.trail.shift(); }
}

// ---------------------------------------------------------------- rendering
// screen rotation applied on top of the map: 0 = north up, -psi = boat up
function viewRot() { return sim.view.boatUp ? -sim.s[2] : 0; }
// world -> canvas *before* the view rotation (the rotation is a ctx transform in draw())
function w2c(E, N) { const v = sim.view; return [canvas.width / 2 + (E - v.cE) * v.s, canvas.height / 2 - (N - v.cN) * v.s]; }
// canvas (as clicked) -> world: undo the view rotation about the centre first
function c2w(x, y) {
  const v = sim.view, th = -viewRot(), cx = canvas.width / 2, cy = canvas.height / 2;
  const dx = x - cx, dy = y - cy, ux = cx + dx * Math.cos(th) - dy * Math.sin(th), uy = cy + dx * Math.sin(th) + dy * Math.cos(th);
  return [v.cE + (ux - cx) / v.s, v.cN - (uy - cy) / v.s];
}

function draw() {
  const sc = sim.scenario, v = sim.view, s = sim.s;
  if (v.follow || v.boatUp) { v.cE = s[0]; v.cN = s[1]; }
  ctx.fillStyle = '#7fa9c3'; ctx.fillRect(0, 0, canvas.width, canvas.height);
  ctx.save(); ctx.translate(canvas.width / 2, canvas.height / 2); ctx.rotate(viewRot()); ctx.translate(-canvas.width / 2, -canvas.height / 2);
  // background image
  if (sim.bgImg && sc.pxPerM) {
    const [x0, y0] = w2c(0, sc.heightM);
    ctx.drawImage(sim.bgImg, x0, y0, sc.widthM * v.s, sc.heightM * v.s);
  } else {
    const [x0, y0] = w2c(0, sc.heightM);
    ctx.fillStyle = '#6f9cb8'; ctx.fillRect(x0, y0, sc.widthM * v.s, sc.heightM * v.s);
    ctx.strokeStyle = 'rgba(255,255,255,0.15)'; ctx.lineWidth = 1;          // 10 m grid
    for (let E = 0; E <= sc.widthM; E += 10) { const [a, b] = w2c(E, 0), [c, d] = w2c(E, sc.heightM); ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(c, d); ctx.stroke(); }
    for (let N = 0; N <= sc.heightM; N += 10) { const [a, b] = w2c(0, N), [c, d] = w2c(sc.widthM, N); ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(c, d); ctx.stroke(); }
  }
  // water polygons: without an image, everything outside them is land
  if (sc.water.length) {
    if (!sim.bgImg) {
      const [x0, y0] = w2c(0, sc.heightM);
      ctx.fillStyle = '#c9bb8e'; ctx.fillRect(x0, y0, sc.widthM * v.s, sc.heightM * v.s);
      for (const poly of sc.water) drawPoly(poly, '#6f9cb8', 'rgba(40,80,120,0.6)');
    } else {
      for (const poly of sc.water) drawPoly(poly, 'rgba(80,160,230,0.10)', 'rgba(40,120,200,0.8)');
    }
  }
  // docks / land
  for (const poly of sc.docks) drawPoly(poly, sim.bgImg ? 'rgba(120,90,60,0.35)' : 'rgba(120,90,60,0.85)', '#3a2a1a');
  if (sim.draft.length) { drawPoly(sim.draft, 'rgba(255,220,0,0.3)', '#ffd400', false); }
  // trail
  if (sim.trail.length > 1) {
    ctx.strokeStyle = 'rgba(255,255,255,0.35)'; ctx.lineWidth = 1; ctx.beginPath();
    sim.trail.forEach(([E, N], i) => { const [x, y] = w2c(E, N); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.stroke();
  }
  drawBoat();
  ctx.restore();
  drawHUD();
}
function drawPoly(poly, fill, stroke, close = true) {
  ctx.beginPath(); poly.forEach(([E, N], i) => { const [x, y] = w2c(E, N); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
  if (close) ctx.closePath();
  ctx.fillStyle = fill; ctx.fill(); ctx.strokeStyle = stroke; ctx.lineWidth = 2; ctx.stroke();
}
function drawBoat() {
  const s = sim.s, v = sim.view;
  const [x, y] = w2c(s[0], s[1]);
  ctx.save(); ctx.translate(x, y); ctx.rotate(s[2]);
  // prop wash
  const T = propThrust(s[3], s[7], P).T;
  if (Math.abs(T) > 50) {
    const L = Math.min(6, Math.abs(T) / 400) * v.s, dir = T > 0 ? 1 : -1;
    ctx.strokeStyle = 'rgba(255,255,255,0.5)'; ctx.lineWidth = 3 * Math.min(1, v.s / 8);
    ctx.beginPath(); ctx.moveTo(0, -P.prop.x * v.s); ctx.lineTo(0, -P.prop.x * v.s + dir * L); ctx.stroke();
  }
  if (sim.sprite) {
    const hpx = sim.spriteLengthM * v.s, wpx = hpx * sim.sprite.width / sim.sprite.height;
    ctx.drawImage(sim.sprite, -wpx / 2, -hpx / 2, wpx, hpx);
  } else {
    ctx.beginPath(); P.outline.forEach(([bx, by], i) => { const px = by * v.s, py = -bx * v.s; i ? ctx.lineTo(px, py) : ctx.moveTo(px, py); });
    ctx.closePath(); ctx.fillStyle = '#f4f0e6'; ctx.fill(); ctx.strokeStyle = '#333'; ctx.lineWidth = 1.5; ctx.stroke();
    ctx.strokeStyle = '#999'; ctx.beginPath(); ctx.moveTo(0, -4.5 * v.s); ctx.lineTo(0, 4.5 * v.s); ctx.stroke();
  }
  // throttle candle from midship: green grows forward (ahead), red grows aft (astern)
  const rpmMax = P.engine.ahead[P.engine.ahead.length - 1];
  const setF = rpmSetpoint(sim.throttleIndex, P) / rpmMax, actF = shaftToEngine(s[7], P) / rpmMax;
  const candle = (f, alpha) => {
    if (Math.abs(f) < 0.01) return;
    const len = 4.0 * Math.abs(f) * v.s, wid = 0.6 * v.s;
    ctx.fillStyle = f > 0 ? `rgba(40,180,70,${alpha})` : `rgba(210,40,40,${alpha})`;
    ctx.fillRect(-wid / 2, f > 0 ? -len : 0, wid, len);            // canvas: -y is forward
  };
  candle(setF, 0.55);                                              // commanded
  candle(actF, 0.9);                                               // actual rpm (lagging)
  ctx.fillStyle = '#222'; ctx.fillRect(-0.35 * v.s, -0.06 * v.s, 0.7 * v.s, 0.12 * v.s);   // midship tick
  // rudder indicator
  ctx.save(); ctx.translate(0, -P.rudder.x * v.s); ctx.rotate(-s[6]);
  ctx.strokeStyle = '#d33'; ctx.lineWidth = 3; ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(0, 0.9 * v.s); ctx.stroke();
  ctx.restore();
  ctx.restore();
  // contact markers
  if (sim.t - sim.lastContact < 0.3) {
    const c = contactForces(s, sim.hullPts, sim.scenario.docks, sim.scenario.water);
    for (const k of c.contacts) { const [cx, cy] = w2c(k.E, k.N); ctx.fillStyle = 'rgba(255,60,0,0.8)'; ctx.beginPath(); ctx.arc(cx, cy, 5, 0, 7); ctx.fill(); }
  }
}
function drawHUD() {
  const s = sim.s, w = sim.wind.state;
  const sog = Math.hypot(s[3], s[4]) / KN;
  const hdg = ((s[2] / DEG) % 360 + 360) % 360;
  const rpm = Math.round(shaftToEngine(s[7], P));
  $('hSpeed').textContent = `${sog.toFixed(1)} kn`;
  $('hHdg').textContent = `${hdg.toFixed(0).padStart(3, '0')}°`;
  $('hRpm').textContent = `${Math.abs(rpm)} rpm ${rpm > 20 ? 'ahead' : rpm < -20 ? 'astern' : 'neutral'} (set ${rpmSetpoint(sim.throttleIndex, P)})`;
  $('hWind').textContent = `${(w.speed / KN).toFixed(1)} kn from ${w.dirDeg.toFixed(0)}°`;
  $('hTime').textContent = `${sim.t.toFixed(0)} s   hits ${sim.hits}${sim.paused ? '   PAUSED' : ''}`;
  const rud = $('hRudder'); rud.style.transform = `translateX(${(s[6] / P.rudder.deltaMax) * 60}px)`;
  $('hRudTxt').textContent = `${(Math.abs(s[6]) / DEG).toFixed(0)}° ${s[6] > 0.005 ? 'stbd' : s[6] < -0.005 ? 'port' : ''}`;
  drawWindsock(canvas.width - 95, 100, w);
}

// Windsock (screen overlay, top-right).  Pole at (ax, ay); the sock streams
// downwind, i.e. toward (dirDeg + 180) rotated by the current view rotation.
// Convention as on a real sock: each of the 5 stripes = 3 kn; a stripe that is
// "inflated" is drawn full-width and straight, limp stripes shrink and sag.
function drawWindsock(ax, ay, w) {
  const kn = w.speed / KN, th = w.dirDeg * DEG + viewRot();
  // compass ring + N marker (N rotates with the view)
  ctx.save(); ctx.translate(ax, ay);
  ctx.strokeStyle = 'rgba(255,255,255,0.6)'; ctx.lineWidth = 1; ctx.beginPath(); ctx.arc(0, 0, 76, 0, 7); ctx.stroke();
  ctx.save(); ctx.rotate(viewRot()); ctx.fillStyle = '#fff'; ctx.font = 'bold 13px system-ui'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  ctx.fillText('N', 0, -64); ctx.restore();
  // sock: rotate so +y points downwind
  ctx.rotate(th);
  ctx.fillStyle = '#444'; ctx.beginPath(); ctx.arc(0, 0, 5, 0, 7); ctx.fill();     // pole
  const inflated = Math.min(5, kn / 3);
  let y = 5;
  for (let i = 0; i < 5; i++) {
    const f = Math.max(0, Math.min(1, inflated - i));        // 0 limp .. 1 fully inflated
    const segLen = 8 + 8 * f, r0 = 14 - i * 1.8, r1 = r0 - 1.8;
    const wig = (1 - f) * 3 * Math.sin(sim.t * 3 + i);       // limp stripes flap a little
    const s0 = 0.55 + 0.45 * f, s1 = 0.55 + 0.45 * Math.max(0, Math.min(1, inflated - i - 1));
    ctx.fillStyle = i % 2 ? '#fff' : '#ff7a1a';
    ctx.beginPath(); ctx.moveTo(-r0 * s0 + wig, y); ctx.lineTo(r0 * s0 + wig, y);
    ctx.lineTo(r1 * s1 + wig, y + segLen); ctx.lineTo(-r1 * s1 + wig, y + segLen); ctx.closePath(); ctx.fill();
    ctx.strokeStyle = 'rgba(0,0,0,0.35)'; ctx.lineWidth = 0.7; ctx.stroke();
    y += segLen;
  }
  ctx.restore();
  ctx.fillStyle = '#fff'; ctx.font = '12px system-ui'; ctx.textAlign = 'center';
  ctx.fillText(`${kn.toFixed(0)} kn from ${w.dirDeg.toFixed(0)}°`, ax, ay + 92);
}

// ---------------------------------------------------------------- editor interactions
let dragging = null;
canvas.addEventListener('pointerdown', e => {
  const [E, N] = c2w(e.offsetX, e.offsetY);
  if (sim.mode === 'dock') { sim.draft.push([E, N]); return; }
  if (sim.mode === 'scale') { sim.draft.push([E, N]); if (sim.draft.length === 2) finishScale(); return; }
  if (sim.mode === 'place') {
    if (!sim.draft.length) { sim.draft.push([E, N]); }
    else { const [E0, N0] = sim.draft[0]; sim.scenario.start = { E: E0, N: N0, psi: Math.atan2(E - E0, N - N0) / DEG }; sim.draft = []; setMode('sail'); resetBoat(); }
    return;
  }
  dragging = { x: e.offsetX, y: e.offsetY, cE: sim.view.cE, cN: sim.view.cN };
});
canvas.addEventListener('pointermove', e => {
  if (!dragging) return;
  sim.view.cE = dragging.cE - (e.offsetX - dragging.x) / sim.view.s;
  sim.view.cN = dragging.cN + (e.offsetY - dragging.y) / sim.view.s; sim.view.follow = false;
});
addEventListener('pointerup', () => dragging = null);
canvas.addEventListener('wheel', e => { e.preventDefault(); sim.view.s *= Math.exp(-e.deltaY * 0.001); }, { passive: false });
canvas.addEventListener('dblclick', () => { if (sim.mode === 'dock') { sim.draft.splice(-2, 2); finishDock('land'); } });

function setMode(m) { sim.mode = m; sim.draft = []; $('modeTxt').textContent = { sail: '', dock: 'Click polygon corners, then finish it as land (dock, quay, shore) or as water (navigable basin; everything outside is land).', scale: 'Click two points a known distance apart.', place: 'Click the boat position, then a point ahead of the bow.' }[m]; }
function polyArea(poly) { return Math.abs(poly.reduce((a, [x, y], i) => { const [x2, y2] = poly[(i + 1) % poly.length]; return a + x * y2 - x2 * y; }, 0) / 2); }
function finishDock(kind = 'land') {
  if (sim.draft.length >= 3 && polyArea(sim.draft) < 0.5) { alert('That polygon has almost no area (points nearly on a line) — it would never be hit. Not added.'); sim.draft = []; return; }
  if (sim.draft.length >= 3) (kind === 'water' ? sim.scenario.water : sim.scenario.docks).push(sim.draft.slice());
  sim.draft = []; sim.undoStack.push(kind);
}
function finishScale() {
  const [[E0, N0], [E1, N1]] = sim.draft; const dWorld = Math.hypot(E1 - E0, N1 - N0);
  const real = parseFloat(prompt('Real distance between the two points, in metres:', '30'));
  if (real > 0) {
    const f = dWorld / real;                                  // current world units per real metre
    const sc = sim.scenario;
    sc.widthM /= f; sc.heightM /= f; sc.pxPerM = sim.bgImg ? sim.bgImg.width / sc.widthM : null;
    sc.docks = sc.docks.map(p => p.map(([E, N]) => [E / f, N / f]));
    sc.water = sc.water.map(p => p.map(([E, N]) => [E / f, N / f]));
    sc.start = { E: sc.start.E / f, N: sc.start.N / f, psi: sc.start.psi };
    resetBoat(); fitView();
  }
  setMode('sail');
}
function newRandomWind() { const rng = mulberry32((Math.random() * 1e9) | 0); sim.wind.set(WindModel.random(rng)); $('windSel').value = 'random'; }

$('windSel').addEventListener('change', e => { if (e.target.value === 'random') newRandomWind(); else sim.wind.set(WIND_PRESETS[e.target.value]); });
$('bRandomWind').onclick = newRandomWind;
$('bReset').onclick = resetBoat;
$('bFit').onclick = fitView;
$('bFollow').onclick = () => sim.view.follow = !sim.view.follow;
$('bBoatUp').onclick = () => { sim.view.boatUp = !sim.view.boatUp; $('bBoatUp').textContent = sim.view.boatUp ? 'North up' : 'Boat up'; };
$('bDock').onclick = () => setMode('dock');
$('bFinishLand').onclick = () => { if (sim.mode === 'dock') finishDock('land'); setMode('sail'); };
$('bFinishWater').onclick = () => { if (sim.mode === 'dock') finishDock('water'); setMode('sail'); };
$('bUndo').onclick = () => {
  if (sim.draft.length) { sim.draft.pop(); return; }
  const k = sim.undoStack.pop(); if (k === 'water') sim.scenario.water.pop(); else sim.scenario.docks.pop();
};
$('bClearDocks').onclick = () => { sim.scenario.docks = []; sim.scenario.water = []; sim.undoStack = []; };
$('bScale').onclick = () => setMode('scale');
$('bPlace').onclick = () => setMode('place');
function loadScenario(sc) {
  sc.water ||= []; sc.docks ||= [];
  sim.scenario = sc; sim.bgImg = null; sim.undoStack = [];
  if (sc.image) { const img = new Image(); img.onload = () => { sim.bgImg = img; }; img.src = sc.image; }
  resetBoat(); fitView();
}
function loadMapImage(img) {
  sim.bgImg = img;
  const sc = sim.scenario; sc.image = img.src; sc.docks = []; sc.water = []; sim.undoStack = [];
  sc.pxPerM = 1;                                 // provisional: 1 px = 1 m until scaled
  sc.widthM = img.width; sc.heightM = img.height; sc.start = { E: img.width / 2, N: img.height / 2, psi: 0 };
  resetBoat(); fitView();
  alert('Map loaded. Use "Set map scale" and click two points whose real distance you know, then draw polygons.');
}
// "Import map": an image (new layout) or a scenario JSON exported earlier (scale + polygons + image)
$('mapFile').addEventListener('change', e => {
  const f = e.target.files[0]; if (!f) return;
  const r = new FileReader();
  if (f.type === 'application/json' || /\.json$/i.test(f.name)) {
    r.onload = () => loadScenario(JSON.parse(r.result)); r.readAsText(f);
  } else {
    r.onload = () => { const img = new Image(); img.onload = () => loadMapImage(img); img.src = r.result; }; r.readAsDataURL(f);
  }
  e.target.value = '';
});
$('loadFile').addEventListener('change', e => {                   // kept for older pages
  const f = e.target.files[0]; if (!f) return;
  const r = new FileReader(); r.onload = () => loadScenario(JSON.parse(r.result)); r.readAsText(f);
});
$('spriteFile').addEventListener('change', e => {
  const f = e.target.files[0]; if (!f) return;
  const img = new Image(); img.onload = () => {
    const L = parseFloat(prompt('Boat length in the image (bow must point up), metres:', P.LOA.toFixed(2)));
    if (L > 0) { sim.sprite = img; sim.spriteLengthM = L; }
  };
  const r = new FileReader(); r.onload = () => img.src = r.result; r.readAsDataURL(f);
});
$('bSave').onclick = () => {
  const blob = new Blob([JSON.stringify(sim.scenario)], { type: 'application/json' });
  const a = document.createElement('a'); a.href = URL.createObjectURL(blob); a.download = 'scenario.json'; a.click();
};
$('loadFile').addEventListener('change', e => {
  const f = e.target.files[0]; if (!f) return;
  const r = new FileReader(); r.onload = () => {
    const sc = JSON.parse(r.result); sim.scenario = sc; sim.bgImg = null;
    if (sc.image) { const img = new Image(); img.onload = () => { sim.bgImg = img; }; img.src = sc.image; }
    resetBoat(); fitView();
  }; r.readAsText(f);
});
// ---------------------------------------------------------------- server-side scenarios
// A static host (GitHub Pages) cannot list a directory, so data/index.json is a
// hand-maintained catalogue: { scenarios: [ { name, file, note } ] }.
async function loadCatalogue() {
  const sel = $('dataSel');
  try {
    const res = await fetch('./data/index.json', { cache: 'no-cache' });
    if (!res.ok) throw new Error(res.status);
    const list = (await res.json()).scenarios || [];
    sel.innerHTML = '<option value="">Load from server…</option>' +
      list.map(x => `<option value="${x.file}">${x.name}</option>`).join('');
    sel.disabled = list.length === 0;
  } catch {
    sel.innerHTML = '<option value="">No data/index.json found</option>';
    sel.disabled = true;
  }
}
$('dataSel').addEventListener('change', async e => {
  const file = e.target.value; if (!file) return;
  try {
    const res = await fetch(`./data/${file}`, { cache: 'no-cache' });
    if (!res.ok) throw new Error(`${res.status} ${res.statusText}`);
    loadScenario(await res.json());
  } catch (err) {
    alert(`Could not load data/${file}: ${err.message}`);
  }
});
loadCatalogue();

$('bPanel').onclick = () => $('panel').classList.toggle('hidden');

// ---------------------------------------------------------------- main loop
function resize() { canvas.width = innerWidth; canvas.height = innerHeight; fitView(); }
addEventListener('resize', resize); resize(); resetBoat();
let last = performance.now(), acc = 0;
function frame(now) {
  let dt = Math.min(0.1, (now - last) / 1000); last = now;
  if (!sim.paused) { acc += dt; while (acc >= H) { physicsStep(now); acc -= H; } }
  draw();
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
