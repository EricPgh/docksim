import { test } from 'node:test';
import assert from 'node:assert/strict';
import { UNION_32 as U, JEANNEAU_36 as J, VESSELS, DEFAULT_VESSEL, KN, DEG, initialState, stepBoat,
         hullForces, propThrust, propWalk, skegForces, keelForces, keelDepth, foilCoeffs, engineToShaft } from '../src/physics.js';

const near = (a, b, tol, m) => assert.ok(Math.abs(a - b) <= tol, `${m}: ${a} vs ${b}`);
const run = (P, idx, T, rud = 0) => { let s = initialState(); s[6] = rud; const h = 0.02, hist = [];
  for (let t = 0; t < T; t += h) { s = stepBoat(s, { wheelCmd: 0, heldTime: 0, throttleIndex: idx }, P, h); hist.push(s); } return { s, hist }; };

test('registry: Union 32 is the default, Jeanneau kept as backup', () => {
  assert.equal(DEFAULT_VESSEL, 'union32');
  assert.equal(VESSELS[DEFAULT_VESSEL], U); assert.equal(VESSELS.jeanneau36, J);
});
test('class data: dimensions match the published Union 32 figures', () => {
  near(U.LWL / 0.3048, 27.25, 0.05, 'LWL ft'); near(U.beam / 0.3048, 10.83, 0.05, 'beam ft');
  near(U.engine.ahead.length, 6, 0, 'throttle steps');
  assert.ok(U.mass > 11500 * 0.4536, 'loaded mass exceeds design displacement');
});
test('wave term: absent for the Jeanneau, steepens Union resistance toward hull speed', () => {
  // R/u^2 must rise with speed (the knee); a pure quadratic would keep it constant
  const c = u => -hullForces(u, 0, 0, U).X / (u * u);
  assert.ok(c(3.3) > 1.3 * c(1.5), `R/u^2 rises: ${c(1.5).toFixed(0)} -> ${c(3.3).toFixed(0)}`);
  near(-hullForces(3.1, 0, 0, J).X, 110 * 3.1 * 3.1 + 60 * 3.1, 1e-9, 'Jeanneau unchanged');
});
test('speed: cruise ~5.5 kn at 2400 rpm, ~6 kn flat out, below hull speed', () => {
  near(run(U, 4, 200).s[3] / KN, 5.4, 0.5, '2400 rpm');
  const top = run(U, 5, 200).s[3] / KN;
  near(top, 6.1, 0.5, '2800 rpm'); assert.ok(top < 7.0, 'below 7.0 kn hull speed');
});
test('bollard thrust ~2.5 kN for 25 hp; equal gear ratios ahead and astern', () => {
  near(propThrust(0, engineToShaft(2800, U), U).T, 2600, 400, 'bollard');
  near(engineToShaft(2000, U), -engineToShaft(-2000, U), 1e-12, 'same shaft speed both ways');
});
test('prop walk: right-hand astern kicks stern to port (bow to stbd); ahead weaker and opposite', () => {
  const a = propWalk(-1000, U), f = propWalk(1000, U);
  assert.ok(a.Y < 0 && a.N > 0, 'astern: stern to port, bow to starboard');
  assert.ok(f.Y > 0 && f.N < 0, 'ahead: stern to starboard');
  assert.ok(Math.abs(a.Y) > 3 * Math.abs(f.Y), 'astern walk dominates');
  assert.deepEqual(propWalk(-1000, J), { Y: 0, N: 0 }, 'Jeanneau has none');
  const L = structuredClone(U); L.prop.walk.hand = -1;
  assert.ok(propWalk(-1000, L).N < 0, 'left-hand prop reverses it');
});
test('backing from rest, wheel centred: bow swings to starboard, then sternway builds', () => {
  const { hist } = run(U, -3, 12);
  const end = hist[hist.length - 1];
  assert.ok(end[2] / DEG > 8 && end[2] / DEG < 30, `heading swing ${(end[2] / DEG).toFixed(1)} deg`);
  assert.ok(end[3] < -0.5, 'sternway');
  near(run(J, -3, 12).s[2], 0, 1e-9, 'Jeanneau backs straight');
});
test('long keel: distributed strips give yaw damping a lumped keel lacks; no skeg', () => {
  const dist = keelForces(0, 0, 0.2, U).N;
  const lump = keelForces(0, 0, 0.2, { ...U, keel: { ...U.keel, nSeg: 0 } }).N;
  assert.ok(dist < 0 && Math.abs(dist) > 10 * Math.abs(lump), `yaw damping ${dist.toFixed(0)} vs lumped ${lump.toFixed(0)} N m`);
  // in pure sway the strips and the lumped keel carry the same area, so the
  // same force to within the strip-quadrature error of the profile (~0.1 %)
  const yd = keelForces(2, 0.3, 0, U).Y, yl = keelForces(2, 0.3, 0, { ...U, keel: { ...U.keel, nSeg: 0 } }).Y;
  assert.ok(Math.abs(yd / yl - 1) < 0.01, `same side force in pure sway: ${yd.toFixed(0)} vs ${yl.toFixed(0)} N`);
  assert.equal(U.skeg, undefined, 'keel-hung rudder, no skeg');
  assert.equal(skegForces(2, 0.3, 0, U).Y, 0);
  near(foilCoeffs(0.1, U.rudder).cl / foilCoeffs(0.1, { ...U.rudder, eff: 1 }).cl, U.rudder.eff, 1e-12, 'flap effectiveness scales lift');
});
test('manoeuvring: without wash the full keel turns much more slowly than the 36i', () => {
  // coasting in neutral at 3 kn, rudder hard over: the full-keel signature
  const coast = P => { let s = [0, 0, 0, 3 * KN, 0, 0, P.rudder.deltaMax, 0];
    for (let t = 0; t < 10; t += 0.02) s = stepBoat(s, { wheelCmd: 0, heldTime: 0, throttleIndex: 0 }, P, 0.02); return s[2]; };
  assert.ok(coast(U) < 0.8 * coast(J), `Union ${(coast(U) / DEG).toFixed(0)} deg vs 36i ${(coast(J) / DEG).toFixed(0)} deg in 10 s`);
});
test('manoeuvring: under power the aperture wash keeps the radius comparable, not smaller', () => {
  const R = P => { const s = run(P, 3, 60, P.rudder.deltaMax).s; return Math.hypot(s[3], s[4]) / s[5]; };
  const ru = R(U), rj = R(J);
  assert.ok(ru >= rj && ru < 2 * rj, `Union ${ru.toFixed(1)} m vs 36i ${rj.toFixed(1)} m`);
});
test('geometry: both mast-step measurements agree once the unequal overhangs are used', () => {
  const FT = 0.3048;
  near(U.stemWLX, U.LWL / 2, 1e-3, 'origin at mid-waterline');
  near(U.stemDeckX - 13.62 * FT, U.mastStepX, 0.005, 'from the deck stem');
  near(U.stemWLX - 10.6 * FT, U.mastStepX, 0.005, 'from the waterline stem');
  const xs = U.outline.map(p => p[0]), hull = xs.filter(x => x <= U.stemDeckX + 1e-9);
  near(Math.max(...hull) - Math.min(...hull), U.LOA, 0.01, 'deck outline spans LOA');
  near(U.stemDeckX - U.stemWLX, 0.92, 0.01, 'bow overhang 0.92 m');
  near(-Math.min(...xs) - U.stemWLX, 0.45, 0.01, 'stern overhang 0.45 m');
  near(Math.max(...xs) - U.stemDeckX, 0.45, 0.01, 'anchor ~0.45 m past the stem');
});
test('keel profile: integrates to keel.area; bite is 12 % of draft; draft is consistent', () => {
  const k = U.keel, dx = (k.xFwd - k.xAft) / k.nSeg;
  let A = 0; for (let i = 0; i < k.nSeg; i++) A += keelDepth(k.profile, k.xFwd - (i + 0.5) * dx) * dx;
  near(A, k.area, 0.05, 'strip sum vs keel.area');
  // the bite is cut 12 % of TOTAL draft above the straight keel bottom line;
  // the stations either side are equally spaced, so the line is their mean
  const canoe = x => U.hull.T0 * (1 - (2 * x / U.LWL) ** 2);
  const bot = x => canoe(x) + keelDepth(k.profile, x);
  const cut = (bot(-2.1) + bot(-2.9)) / 2 - bot(-2.5);
  near(cut / (4.83 * 0.3048), 0.12, 0.01, 'bite = 12 % of draft');
  // canoe body + keel must give the published 1.47 m draft, not more: the canoe
  // body and the keel are separate lateral area and must not double-count
  let draft = 0; for (let x = k.xAft; x <= k.xFwd; x += 0.01) draft = Math.max(draft, canoe(x) + keelDepth(k.profile, x));
  near(draft, 4.83 * 0.3048, 0.01, 'max draft');
  near(k.xFwd - U.mastStepX, 2.72 * 0.3048, 0.005, 'rake starts 2.72 ft forward of the measured mast step');
  assert.equal(keelDepth(k.profile, 2.5), 0, 'no keel forward of the bulkhead');
});
