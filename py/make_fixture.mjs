// Generates py/fixtures/trajectory.json from the JS physics so test_port.py can
// verify the numpy port.  Run: node py/make_fixture.mjs
import { JEANNEAU_36 as P, stepBoat, initialState, KN } from '../src/physics.js';
import { contactForces, sampleOutline } from '../src/collision.js';
import fs from 'fs';
const pts = sampleOutline(P.outline);
const cases = [];
// 1: fixed rudder 20 deg stbd, 1500 rpm, wind 10 kn from east, no docks
{ let s = initialState(0, 0, 0); s[6] = 20 * Math.PI / 180; const out = [];
  for (let i = 0; i <= 4000; i++) { if (i % 100 === 0) out.push([...s]); s = stepBoat(s, { wheelCmd: 0, heldTime: 0, throttleIndex: 3, wind: { E: -10 * KN, N: 0 } }, P, 0.01); }
  cases.push({ name: 'turn', rudderDeg: 20, throttle: 3, windE: -10 * KN, windN: 0, docks: [], water: [], states: out }); }
// 2: backing down, rudder 25 deg port, wind 6 kn from north
{ let s = initialState(0, 0, 45); s[6] = -25 * Math.PI / 180; const out = [];
  for (let i = 0; i <= 3000; i++) { if (i % 100 === 0) out.push([...s]); s = stepBoat(s, { wheelCmd: 0, heldTime: 0, throttleIndex: -3, wind: { E: 0, N: -6 * KN } }, P, 0.01); }
  cases.push({ name: 'astern', rudderDeg: -25, throttle: -3, windE: 0, windN: -6 * KN, docks: [], water: [], states: out }); }
// 3: drives into a wall 15 m ahead, then a water polygon boundary on the side
{ const docks = [[[15, -10], [25, -10], [25, 10], [15, 10]]], water = [[[-30, -8], [30, -8], [30, 30], [-30, 30]]];
  let s = initialState(0, 0, 90); const out = [];
  const extra = st => contactForces(st, pts, docks, water);
  for (let i = 0; i <= 3000; i++) { if (i % 100 === 0) out.push([...s]); s = stepBoat(s, { wheelCmd: 0, heldTime: 0, throttleIndex: 2, wind: { E: 0, N: -8 * KN }, extra }, P, 0.01); }
  cases.push({ name: 'contact', rudderDeg: 0, throttle: 2, windE: 0, windN: -8 * KN, docks, water, states: out }); }
fs.writeFileSync('py/fixtures/trajectory.json', JSON.stringify({ dt: 0.01, every: 100, cases }));
console.log('wrote', cases.map(c => c.name + ':' + c.states.length).join(' '));
