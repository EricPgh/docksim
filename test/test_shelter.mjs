import { test } from 'node:test';
import assert from 'node:assert/strict';
import { buildShelterField, sampleShelter, cavityLength, serializeShelter, deserializeShelter, SHELTER_DEFAULTS } from '../src/shelter.js';

globalThis.btoa ||= s => Buffer.from(s, 'binary').toString('base64');
globalThis.atob ||= s => Buffer.from(s, 'base64').toString('binary');
const near = (a, b, tol, m) => assert.ok(Math.abs(a - b) <= tol, `${m}: ${a} vs ${b}`);

// one 20 x 20 m building, 10 m tall, centred at (50, 50) in a 100 x 100 m domain
const B = [{ poly: [[40, 40], [60, 40], [60, 60], [40, 60]], height: 10 }];
const bounds = { x0: 0, y0: 0, x1: 100, y1: 100 };
const F = buildShelterField(B, bounds, { dx: 2, nDirs: 8, sorIters: 200 });

test('cavity length: Wilson formula scales with height and crosswind width', () => {
  near(cavityLength(20, 20, 10) / 10, 1.8 * 2 / (Math.pow(2, 0.3) * (1 + 0.48)), 1e-9, 'Lr/H');
  assert.ok(cavityLength(40, 20, 10) > cavityLength(20, 20, 10), 'wider building, longer cavity');
  assert.ok(cavityLength(20, 40, 10) < cavityLength(20, 20, 10), 'longer along-wind, shorter cavity');
  assert.equal(cavityLength(20, 20, 0), 0, 'no height, no cavity');
});
test('field: lee sheltered, windward stagnation, far field recovers', () => {
  // wind from north (0 deg) blows toward the south, so the lee is at lower N
  const lee = sampleShelter(F, 50, 30, 0), near1 = sampleShelter(F, 50, 72, 0);
  const farUp = sampleShelter(F, 50, 96, 0), corner = sampleShelter(F, 10, 10, 0);
  assert.ok(lee.mult < 0.6, `lee sheltered (${lee.mult.toFixed(2)})`);
  assert.ok(lee.turb > 1.3, `lee is gustier (${lee.turb.toFixed(2)})`);
  // the projection produces a stagnation slow-down on the windward centreline...
  assert.ok(near1.mult < 0.9, `windward stagnation (${near1.mult.toFixed(2)})`);
  // ...which decays: further upwind must be closer to free stream than near the face
  assert.ok(farUp.mult > near1.mult, `stagnation decays upwind (${farUp.mult.toFixed(2)} > ${near1.mult.toFixed(2)})`);
  near(corner.mult, 1, 0.15, 'far corner of the domain is free stream');
});
test('field: shelter follows the wind direction, not the map', () => {
  const south = sampleShelter(F, 50, 30, 0);       // wind from N -> lee to the S
  const north = sampleShelter(F, 50, 30, 180);     // wind from S -> that point is now upwind
  assert.ok(south.mult < north.mult - 0.2, 'same point, opposite winds, different shelter');
  const east = sampleShelter(F, 30, 50, 90);       // wind from E -> lee to the W
  assert.ok(east.mult < 0.6, 'lee rotates with direction');
});
test('field: flow accelerates past the sides of the building', () => {
  // wind from the north: the sides are east and west of the building (E<40, E>60)
  let peakN = 0;
  for (let E = 20; E <= 80; E += 1) if (E < 39 || E > 61) peakN = Math.max(peakN, sampleShelter(F, E, 50, 0).mult);
  assert.ok(peakN > 1.1, `side speed-up, wind from N (${peakN.toFixed(2)})`);
  // wind from the east: the sides are now north and south of the building
  let peakE = 0;
  for (let N = 20; N <= 80; N += 1) if (N < 39 || N > 61) peakE = Math.max(peakE, sampleShelter(F, 50, N, 90).mult);
  assert.ok(peakE > 1.1, `side speed-up, wind from E (${peakE.toFixed(2)})`);
});
test('sampling: outside the grid is neutral; interpolation is continuous', () => {
  const out = sampleShelter(F, -50, -50, 0);
  assert.deepEqual([out.mult, out.deflDeg, out.turb], [1, 0, 1]);
  assert.deepEqual(sampleShelter(null, 0, 0, 0), { mult: 1, deflDeg: 0, turb: 1 });
  // Continuity: bilinear sampling means the max step-to-step jump must scale
  // linearly with the step size.  (An absolute threshold would only be testing
  // how steep the cavity edge is, which is a real gradient, not a discontinuity.)
  const maxJump = step => {
    let prev = sampleShelter(F, 50, 20, 0).mult, mx = 0;
    for (let N = 20; N < 80; N += step) { const m = sampleShelter(F, 50, N, 0).mult; mx = Math.max(mx, Math.abs(m - prev)); prev = m; }
    return mx;
  };
  const j1 = maxJump(0.4), j2 = maxJump(0.2);
  near(j1 / j2, 2, 0.35, 'jump scales with step -> continuous');
  let pd = sampleShelter(F, 50, 35, 0).mult, maxD = 0;
  for (let d = 0; d < 360; d += 1) { const m = sampleShelter(F, 50, 35, d).mult; maxD = Math.max(maxD, Math.abs(m - pd)); pd = m; }
  assert.ok(maxD < 0.12, `smooth across direction bins (${maxD.toFixed(3)})`);
});
test('mass conservation: flux across successive transects becomes near-constant', () => {
  // Wind from the north (bin 0) blows south.  Total southward flux across a full
  // east-west transect must be the same at every N if mass is conserved: the
  // flow the building blocks has to reappear beside it.  This is the physical
  // statement of the projection, and unlike a pointwise divergence it is immune
  // to the checkerboard mode a staggered solve leaves at cell centres.
  const fluxSpread = f => {
    const dir = 0, th = 0, fx = -Math.sin(th), fy = -Math.cos(th);
    const rows = [];
    for (let j = 2; j < f.ny - 2; j++) {
      let flux = 0;
      for (let i = 0; i < f.nx; i++) {
        const k = j * f.nx + i; if (f.solid[k]) continue;
        const m = f.speed[dir][k] * f.maxMult / 255, a = f.defl[dir][k] * Math.PI / 180;
        flux += m * Math.cos(a);                       // component along the free stream
      }
      rows.push(flux);
    }
    const mean = rows.reduce((a, b) => a + b) / rows.length;
    return Math.sqrt(rows.reduce((a, b) => a + (b - mean) ** 2, 0) / rows.length) / mean;
  };
  const raw = buildShelterField(B, bounds, { dx: 2, nDirs: 4, sorIters: 0 });
  const sol = buildShelterField(B, bounds, { dx: 2, nDirs: 4, sorIters: 800 });
  const r = fluxSpread(raw), q = fluxSpread(sol);
  assert.ok(q < 0.5 * r, `projection evens the transect flux (${(q * 100).toFixed(1)}% vs ${(r * 100).toFixed(1)}% spread)`);
  assert.ok(q < 0.08, `residual flux variation small (${(q * 100).toFixed(1)}%)`);
});
test('field: interior of a building is zero velocity', () => {
  assert.ok(sampleShelter(F, 50, 50, 0).mult < 0.05, 'inside the footprint');
  assert.ok(sampleShelter(F, 50, 50, 135).mult < 0.05, 'independent of direction');
});
test('serialize: round trip is exact and compact', () => {
  const S = serializeShelter(F), D = deserializeShelter(S);
  assert.equal(D.nx, F.nx); assert.equal(D.speed.length, F.speed.length);
  for (let d = 0; d < F.nDirs; d++) assert.deepEqual([...D.speed[d]], [...F.speed[d]], `speed plane ${d}`);
  for (let d = 0; d < F.nDirs; d++) assert.deepEqual([...D.defl[d]], [...F.defl[d]], `defl plane ${d}`);
  const bytes = JSON.stringify(S).length, cells = F.nx * F.ny * F.nDirs;
  // three byte planes per direction, base64 at 4/3 -> ~4 bytes per cell per direction
  assert.ok(bytes < cells * 5, `compact storage (${bytes} B for ${cells} cells x 3 planes)`);
  near(sampleShelter(D, 50, 30, 0).mult, sampleShelter(F, 50, 30, 0).mult, 1e-12, 'sampling after round trip');
});
test('no obstacles with height -> neutral field everywhere', () => {
  const flat = buildShelterField([{ poly: [[10, 10], [20, 10], [20, 20]], height: 0 }], bounds, { dx: 5, nDirs: 4 });
  for (const E of [15, 50, 90]) for (const N of [15, 50, 90]) near(sampleShelter(flat, E, N, 45).mult, 1, 0.02, `neutral at ${E},${N}`);
});
