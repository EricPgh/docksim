import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  JEANNEAU_36 as P, DEG, KN, RHO_W, derived, foilCoeffs, foilForce, hullForces, keelForces,
  propThrust, rudderForces, windForces, rpmSetpoint, clampThrottle, rudderRate,
  derivative, rk4, stepBoat, initialState,
} from '../src/physics.js';
import { WindModel, WIND_PRESETS, mulberry32 } from '../src/wind.js';
import { contactForces, sampleOutline, pointInPolygon } from '../src/collision.js';

const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg}: ${a} vs ${b}`);

// ---------------------------------------------------------------- foil
test('foil: zero lift at zero alpha, antisymmetric, Helmbold slope, period pi', () => {
  const f = P.rudder;
  assert.equal(foilCoeffs(0, f).cl, 0);
  near(foilCoeffs(0.1, f).cl, -foilCoeffs(-0.1, f).cl, 1e-12, 'antisymmetry');
  const slope = 2 * Math.PI * f.ar / (2 + Math.sqrt(f.ar * f.ar + 4));
  near(foilCoeffs(0.05, f).cl / 0.05, slope, 1e-9, 'Helmbold slope');
  near(foilCoeffs(0.1 + Math.PI, f).cl, foilCoeffs(0.1, f).cl, 1e-12, 'reverse flow periodicity');
});
test('foil: stall reduces lift below linear extrapolation; drag never below cd0', () => {
  const f = P.rudder, a = 2 * f.alphaStall;
  assert.ok(foilCoeffs(a, f).cl < foilCoeffs(f.alphaStall, f).cl * 2 * 0.9, 'post-stall lift drops');
  for (let a = -Math.PI; a <= Math.PI; a += 0.01) assert.ok(foilCoeffs(a, f).cd >= f.cd0 - 1e-12);
  near(foilCoeffs(Math.PI / 2, f).cd, f.cn, 1e-9, 'broadside drag = Cn');
});
test('foilForce: sign conventions (ahead, astern, drifting)', () => {
  const F1 = foilForce(2, 0, 10 * DEG, P.rudder);
  assert.ok(F1.Y < 0 && F1.X < 0, 'ahead, TE to stbd -> force to port, drag aft');
  const F2 = foilForce(-1, 0, 10 * DEG, P.rudder);
  assert.ok(F2.Y > 0 && F2.X > 0, 'astern, TE to stbd -> force to stbd, drag forward');
  const F3 = foilForce(2, 0.3, 0, P.keel);
  assert.ok(F3.Y < 0, 'drifting to stbd -> keel lift to port (restoring)');
  assert.deepEqual([foilForce(0, 0, 0.5, P.rudder).X, foilForce(0, 0, 0.5, P.rudder).Y], [0, 0]);
});

// ---------------------------------------------------------------- hull / keel
test('hull: surge resistance calibrated (~1.25 kN at 6 kn) and opposes motion', () => {
  const R = hullForces(3.1, 0, 0, P);
  near(-R.X, 1250, 250, 'resistance at 3.1 m/s');
  assert.ok(hullForces(-1, 0, 0, P).X > 0);
});
test('hull: pure sway -> sway drag, no yaw; pure yaw -> yaw damping, no sway', () => {
  const S = hullForces(0, 0.5, 0, P);
  assert.ok(S.Y < 0); near(S.N, 0, 1e-9, 'symmetric hull, no moment in pure sway');
  const Yw = hullForces(0, 0, 0.2, P);
  assert.ok(Yw.N < 0); near(Yw.Y, 0, 1e-9, 'no net sway force in pure yaw');
  near(hullForces(0, 0.5, 0, P).Y, -hullForces(0, -0.5, 0, P).Y, 1e-9, 'odd in v');
});
test('keel: restoring side force with moment about its position', () => {
  const K = keelForces(2, 0.2, 0, P);
  assert.ok(K.Y < 0); near(K.N, P.keel.x * K.Y, 1e-9, 'moment = x_k Y');
});

// ---------------------------------------------------------------- propeller
test('prop: zero at n=0, n^2 bollard scaling, astern weaker, thrust falls with speed', () => {
  assert.equal(propThrust(1, 0, P).T, 0);
  const T1 = propThrust(0, 10, P).T, T2 = propThrust(0, 20, P).T;
  assert.ok(T1 > 0); near(T2 / T1, 4, 1e-9, 'n^2');
  const Ta = propThrust(0, -10, P).T;
  assert.ok(Ta < 0 && Math.abs(Ta) < T1, 'astern weaker');
  assert.ok(propThrust(2, 10, P).T < T1, 'ahead thrust decreases with advance');
  assert.ok(propThrust(2, -10, P).T < Ta, 'astern thrust while moving ahead is a stronger brake');
  const n2500 = 2500 / 60 / P.prop.gear;
  near(propThrust(0, n2500, P).T, 3000, 600, 'full-throttle bollard thrust ~3 kN');
});

// ---------------------------------------------------------------- rudder
test('rudder: prop wash steers at zero speed ahead, not astern; astern with way reverses sign', () => {
  const T = propThrust(0, 10, P).T;
  const Rw = rudderForces(0, 0, 0, 20 * DEG, T, P);
  assert.ok(Rw.Y < 0 && Rw.N > 0, 'wash + TE stbd -> stern to port, bow to stbd');
  const Ra = rudderForces(0, 0, 0, 20 * DEG, -T, P);
  assert.equal(Ra.Y, 0, 'no slipstream over rudder with astern thrust at rest');
  const Rb = rudderForces(-1, 0, 0, 20 * DEG, 0, P);
  assert.ok(Rb.Y > 0 && Rb.N < 0, 'backing with way: stern goes to starboard');
  assert.ok(rudderForces(2, 0, 0, 20 * DEG, T, P).N > rudderForces(2, 0, 0, 20 * DEG, 0, P).N, 'wash adds rudder moment underway');
});

// ---------------------------------------------------------------- windage
test('wind: head wind slows, beam wind pushes to leeward and blows the bow off, zero apparent -> zero force', () => {
  const s = initialState(0, 0, 0);            // heading north
  const V = 10 * KN;
  const head = windForces(s, 0, -V, P);         // wind from north
  assert.ok(head.X < 0); near(head.Y, 0, 1e-9, 'no side force head-on');
  const east = windForces(s, -V, 0, P);         // wind from east
  assert.ok(east.Y < 0 && east.N < 0, 'from east -> pushed west, bow falls off downwind');
  const moving = [0, 0, 0, V, 0, 0, 0, 0];      // moving north at wind speed, wind from south
  const z = windForces(moving, 0, V, P);
  near(z.X, 0, 1e-9, 'zero apparent'); near(z.Y, 0, 1e-9, 'zero apparent');
  const q = 0.5 * 1.225 * V * V;
  near(-head.X, q * P.windage.cx * P.windage.areaFrontal, 1e-6, 'q Cx A');
});

// ---------------------------------------------------------------- controls
test('controls: throttle steps and wheel rate ramp', () => {
  assert.equal(rpmSetpoint(0, P), 0); assert.equal(rpmSetpoint(1, P), 800);
  assert.equal(rpmSetpoint(-2, P), -1000);
  assert.equal(clampThrottle(99, P), P.engine.ahead.length - 1);
  assert.equal(rudderRate(0, 5, P), 0);
  const D = derived(P);
  near(rudderRate(1, 0, P), D.rudderRateMin, 1e-12, 'initial rate');
  near(rudderRate(-1, 10, P), -D.rudderRateMax, 1e-12, 'held rate');
  const s = initialState(); s[6] = P.rudder.deltaMax;
  assert.equal(derivative(s, { wheelCmd: 1, heldTime: 1, throttleIndex: 0 }, P)[6], 0, 'no rate past the stop');
  assert.ok(derivative(s, { wheelCmd: -1, heldTime: 1, throttleIndex: 0 }, P)[6] < 0, 'can come back');
});

// ---------------------------------------------------------------- integrator
test('rk4: 4th-order convergence on harmonic oscillator, exact-ish on decay', () => {
  const f = ([x, v]) => [v, -x];
  const run = h => { let s = [1, 0]; for (let t = 0; t < 2 * Math.PI - 1e-9; t += h) s = rk4(f, s, h); return s; };
  const err = h => Math.hypot(run(h)[0] - 1, run(h)[1]);
  const e1 = err(2 * Math.PI / 64), e2 = err(2 * Math.PI / 128);
  near(e1 / e2, 16, 3, 'halving h cuts error ~16x');
  let y = [1]; for (let i = 0; i < 100; i++) y = rk4(([x]) => [-x], y, 0.01);
  near(y[0], Math.exp(-1), 1e-9, 'exp decay');
});

// ---------------------------------------------------------------- whole boat
const run = (inpFn, T, s0 = initialState(), h = 0.01) => {
  let s = s0, t = 0, hist = [];
  while (t < T) { s = stepBoat(s, inpFn(t, s), P, h); t += h; hist.push(s); }
  return { s, hist };
};
test('boat: at rest with nothing acting it stays at rest', () => {
  const { s } = run(() => ({ wheelCmd: 0, heldTime: 0, throttleIndex: 0 }), 10);
  for (const x of s) near(x, 0, 1e-12, 'state');
});
test('boat: 2000 rpm ahead reaches ~5 kn; 2500 rpm ~6.5 kn', () => {
  const a = run(() => ({ wheelCmd: 0, heldTime: 0, throttleIndex: 4 }), 150).s;
  near(a[3] / KN, 5.1, 0.8, 'speed at 2000 rpm');
  const b = run(() => ({ wheelCmd: 0, heldTime: 0, throttleIndex: 5 }), 150).s;
  near(b[3] / KN, 6.4, 0.8, 'speed at 2500 rpm');
  near(a[4], 0, 1e-9, 'no sway'); near(a[5], 0, 1e-9, 'no yaw');
});
test('boat: hard-to-starboard from rest with ahead power turns to starboard; steady turn settles', () => {
  const s0 = initialState(); s0[6] = P.rudder.deltaMax;
  const { s, hist } = run(() => ({ wheelCmd: 0, heldTime: 0, throttleIndex: 3 }), 60, s0);
  assert.ok(hist[300][5] > 0, 'yaw rate positive after 3 s (prop wash)');
  assert.ok(s[5] > 0 && s[4] < 0, 'steady: turning stbd, sliding to port (outward)');
  near(s[5], hist[hist.length - 500][5], 0.02, 'settled yaw rate');
  const radius = Math.hypot(s[3], s[4]) / s[5];
  assert.ok(radius > 4 && radius < 20, `plausible turning radius ${radius.toFixed(1)} m (~0.5-2 LOA)`);
  near(s[5] / DEG, 10, 4, 'pivot rate ~10 deg/s at 1500 rpm hard over');
});
test('boat: backing down — no steering until sternway builds, then stern follows the rudder', () => {
  const s0 = initialState(); s0[6] = P.rudder.deltaMax;
  const { s, hist } = run(() => ({ wheelCmd: 0, heldTime: 0, throttleIndex: -3 }), 30, s0);
  near(hist[100][5], 0, 1e-3, 'no yaw at 1 s');
  assert.ok(Math.min(...hist.map(x => x[3])) < -0.5, 'builds sternway'); assert.ok(s[5] < 0, 'TE stbd + sternway -> bow to port');
});
test('boat: beam wind at rest -> leeway and bow blows off', () => {
  const wind = { E: -12 * KN, N: 0 };          // from east, boat heading north
  const { s } = run(() => ({ wheelCmd: 0, heldTime: 0, throttleIndex: 0, wind }), 20);
  assert.ok(s[0] < -1, 'drifted west'); assert.ok(s[2] < -0.05, 'bow fell off to port (downwind)');
});

// ---------------------------------------------------------------- wind model
test('wind model: OU gust variance ~ sigma^2, calm is zero, direction vector convention', () => {
  const w = new WindModel(WIND_PRESETS.calm, 3); w.step(1);
  assert.equal(w.state.speed, 0);
  const cfg = { ...WIND_PRESETS.moderate, puffRate: 0 };
  const g = new WindModel(cfg, 7); let s2 = 0, n = 0;
  for (let i = 0; i < 200000; i++) { g.step(0.5); s2 += g.g * g.g; n++; }
  near(Math.sqrt(s2 / n), cfg.gustSigma, 0.02, 'stationary std');
  const north = new WindModel({ ...WIND_PRESETS.calm, meanKn: 10, dirDeg: 0 }, 1);
  near(north.state.N, -10 * KN, 1e-9, 'northerly blows toward south'); near(north.state.E, 0, 1e-9, '');
  const puffy = new WindModel({ ...WIND_PRESETS.gusty, puffRate: 1 }, 5);
  let seen = false; for (let i = 0; i < 3000; i++) { puffy.step(0.01); if (puffy.puffs.length) seen = true; }
  assert.ok(seen, 'puffs arrive');
});

// ---------------------------------------------------------------- contact
test('contact: penetration pushes hull out of dock, nothing when clear', () => {
  const pts = sampleOutline(P.outline);
  const dock = [[10, -5], [30, -5], [30, 5], [10, 5]];          // wall east of boat
  const clear = contactForces(initialState(0, 0, 90), pts, [dock]);
  assert.deepEqual([clear.X, clear.Y, clear.N], [0, 0, 0]);
  const s = initialState(4.8, 0, 90);                            // heading east, bow 0.27 m into wall
  const c = contactForces(s, pts, [dock]);
  assert.ok(c.contacts.length > 0 && c.X < 0, 'bow pushed back (aft)');
  assert.ok(pointInPolygon(15, 0, dock) && !pointInPolygon(5, 0, dock));
  const side = contactForces([-5, 4.8, 0, 0, 0.5, 0, 0, 0], pts, [[[-20, 3], [0, 3], [0, 8], [-20, 8]]]);
  assert.ok(side.Y < 0, 'stbd side contact while sliding stbd -> pushed to port, damped');
});
test('contact: water polygon — inside is free, poking outside is pushed back in', () => {
  const pts = sampleOutline(P.outline);
  const basin = [[-20, -20], [20, -20], [20, 20], [-20, 20]];
  const inside = contactForces(initialState(0, 0, 0), pts, [], [basin]);
  assert.deepEqual([inside.X, inside.Y, inside.N], [0, 0, 0]);
  const bowOut = contactForces(initialState(0, 15, 0), pts, [], [basin]);     // bow at N=20.47, wall at 20
  assert.ok(bowOut.contacts.length > 0 && bowOut.X < 0, 'bow over the edge is pushed aft');
  const stbdOut = contactForces(initialState(18.5, 0, 0), pts, [], [basin]); // stbd side over E=20
  assert.ok(stbdOut.Y < 0, 'starboard side over the edge is pushed to port');
});
