// Generates py/fixtures/trajectory.json from the JS physics so test_port.py can
// verify the numpy port, for every vessel.  Run: node py/make_fixture.mjs
import { VESSELS, stepBoat, initialState, KN } from '../src/physics.js';
import { contactForces, sampleOutline } from '../src/collision.js';
import fs from 'fs';

function record(vessel, name, s0, rudderDeg, throttle, wind, docks = [], water = [], steps = 3000) {
  const P = VESSELS[vessel], pts = sampleOutline(P.outline);
  let s = s0; s[6] = rudderDeg * Math.PI / 180;
  const extra = (docks.length || water.length) ? st => contactForces(st, pts, docks, water) : undefined;
  const out = [];
  for (let i = 0; i <= steps; i++) {
    if (i % 100 === 0) out.push([...s]);
    s = stepBoat(s, { wheelCmd: 0, heldTime: 0, throttleIndex: throttle, wind, extra }, P, 0.01);
  }
  return { vessel, name, rudderDeg, throttle, windE: wind.E, windN: wind.N, docks, water, states: out };
}
const wall = [[[15, -10], [25, -10], [25, 10], [15, 10]]], basin = [[[-30, -8], [30, -8], [30, 30], [-30, 30]]];
const cases = [];
for (const v of Object.keys(VESSELS)) {
  cases.push(record(v, 'turn', initialState(0, 0, 0), 20, 3, { E: -10 * KN, N: 0 }, [], [], 4000));
  cases.push(record(v, 'astern', initialState(0, 0, 45), -25, -3, { E: 0, N: -6 * KN }));
  cases.push(record(v, 'walk', initialState(0, 0, 0), 0, -3, { E: 0, N: 0 }));     // rudder centred: pure prop walk
  cases.push(record(v, 'contact', initialState(0, 0, 90), 0, 2, { E: 0, N: -8 * KN }, wall, basin));
}
fs.writeFileSync('py/fixtures/trajectory.json', JSON.stringify({ dt: 0.01, every: 100, cases }));
console.log('wrote', cases.map(c => `${c.vessel}/${c.name}`).join(' '));
