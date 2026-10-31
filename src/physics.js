// physics.js — pure, DOM-free.  Runs in Node (tests) and the browser (app).
//
// Conventions (SNAME-style, 3 DOF plan view)
//   body frame : x forward, y to starboard, z down  (so +yaw = bow to starboard)
//   world frame: E east, N north; heading psi measured clockwise from north
//   state s = [xE, yN, psi, u, v, r, delta, n]
//     u,v   : surge/sway speed of CG through the water   [m/s]
//     r     : yaw rate                                    [rad/s]
//     delta : rudder angle, + = trailing edge to starboard [rad]
//     n     : propeller shaft speed, + ahead              [rev/s]
//   Rudder foils:  alpha = delta + atan2(v_in, u_in)  (note the +; see MODEL.md)

export const RHO_W = 1025.0;   // seawater  kg/m^3
export const RHO_A = 1.225;    // air       kg/m^3
export const DEG = Math.PI / 180;
export const KN = 0.514444;    // m/s per knot

// ----------------------------------------------------------------------------
// Parameter set.  Numbers are for the Jeanneau Sun Odyssey 36i (LOA 10.94 m,
// beam 3.59 m, draft 1.94 m, 5700 kg light, Yanmar 3YM30 29 hp on a conventional
// shaft with a KM2P mechanical gearbox and a feathering prop, single spade rudder).  Everything a user might want to tune lives here.
// ----------------------------------------------------------------------------
export const JEANNEAU_36 = {
  name: 'Jeanneau Sun Odyssey 36i',
  LOA: 10.94, LWL: 9.84, beam: 3.59,
  mass: 6200,                    // light ship + fuel, water, crew   [kg]
  kzz: 0.25 * 10.94,             // yaw radius of gyration ~0.25 LOA  [m]
  addedMass: { ax: 0.05, ay: 0.55, az: 0.35, munk: 1.0 }, // fractions of m, Izz
  hull: {
    T0: 0.60,                    // canoe-body draft amidships       [m]
    kQuad: 110,                  // surge resistance  R = kQ u|u| + kL u
    kLin: 60,
    cdCross: 1.0,                // sectional cross-flow drag coeff.
    nStrips: 24,
  },
  keel: { area: 2.0, ar: 2.2, x: -0.3, cd0: 0.008, e: 0.8, alphaStall: 12 * DEG, cn: 1.5 },
  rudder: { area: 0.75, ar: 3.0, x: -4.6, cd0: 0.010, e: 0.8, alphaStall: 15 * DEG, cn: 1.5,
            deltaMax: 35 * DEG, span: 1.5 },
  wheel: { lockToLockTurns: 2.0, rateMin: 90 * DEG, rateMax: 360 * DEG, rampTime: 1.0 },
  prop: { D: 0.42, Kt0: 0.26, J0: 0.85,   // Kt0 lowered with the larger D so bollard thrust stays ~3 kN
          astern: 0.85,                // feathering prop: reverse blade angle is good; rpm loss is in gearAstern
          gearAhead: 2.21, gearAstern: 3.06,   // KM2P mechanical box: taller reduction astern
          x: -4.0,                     // shaft prop sits ~0.6 m ahead of the rudder stock
          wake: 0.05, slipFrac: 0.50, kR: 0.5, tau: 0.8 },
  engine: { ahead: [0, 800, 1000, 1500, 2000, 2500], astern: [0, 800, 1000, 1500, 2000] },
  windage: { areaFrontal: 6.0, areaLateral: 18.0, cx: 0.7, cy: 0.9, xCE: 0.5 },
  // hull outline in body frame, bow first, clockwise (x fwd, y stbd)
  outline: [[5.47, 0], [4.6, 0.75], [3.2, 1.40], [1.2, 1.78], [-1.4, 1.76], [-3.4, 1.55],
            [-4.9, 1.20], [-5.47, 0.95], [-5.47, -0.95], [-4.9, -1.20], [-3.4, -1.55],
            [-1.4, -1.76], [1.2, -1.78], [3.2, -1.40], [4.6, -0.75]],
};

// ----------------------------------------------------------------------------
// Union 32 cutter (Ted Brewer; also sold as Shearwater 32 / East Orient 32).
// Class data: LOA 31.75 ft, LWL 27.25 ft, beam 10.83 ft, 11 500 lb, draft 4.83 ft.
// Volvo Penta MD11D, 2-cyl, 25 hp.  Underbody per the owner: partial keel with
// a raked forefoot and a subtle Brewer bite, keel-hung rudder on the deadwood; the shaft exits the keel into a pocket just forward
// of the rudder; Kiwiprop feathering 3-blade; bow pulpit with anchor, no sprit.
// UNVERIFIED: rated rpm / redline, MS2 gear ratio (2.4:1 assumed, equal both
// ways), prop diameter and set pitch, prop handedness.
// ----------------------------------------------------------------------------
export const UNION_32 = {
  name: 'Union 32 (Brewer cutter)',
  LOA: 9.68, LWL: 8.31, beam: 3.30,
  mass: 5900,                    // 5216 kg design + fuel, water, crew, gear [kg]
  kzz: 0.26 * 9.68,
  addedMass: { ax: 0.06, ay: 0.90, az: 0.70, munk: 1.0 },  // full-keel lateral plane
  hull: {
    T0: 0.52,                    // canoe body only; T0 + max keel depth = 1.47 m draft
    kQuad: 60, kLin: 70, cdCross: 1.0, nStrips: 24,
    kWave: 90,                   // wave-making knee near hull speed (7.0 kn); see MODEL.md
  },
  // Partial keel, subtle Brewer bite.  The forefoot rake starts at the
  // vee-berth / head bulkhead, 2.72 ft (0.83 m) forward of the MEASURED mast
  // step (geometry block below), so the leading edge meets the canoe body at
  // x = 1.753.  About a 45 deg rake down to full 1.47 m draft at x = 0.71; the
  // bottom then runs flat and rises aft in a straight line to the rudder heel.
  // The bite is cut 12 % OF TOTAL DRAFT (0.18 m) above that line at x = -2.5.
  // d = depth BELOW THE CANOE BODY (hull.T0 parabola), so canoe(x) + d(x) is
  // the keel bottom; stations are dense enough to follow the canoe curvature.
  // The last 0.4 m has the 0.45 m-tall prop aperture removed.  Keep keel.area
  // equal to the integral (tested).
  keel: { area: 4.51, ar: 0.4, x: -0.6, cd0: 0.012, e: 0.6, alphaStall: 30 * DEG, cn: 1.3,
          nSeg: 27, xFwd: 1.753, xAft: -3.65,
          profile: [[1.753, 0], [0.71, 0.965], [0.35, 0.954], [0, 0.95], [-0.65, 0.963], [-1.3, 1.001], [-2.1, 0.974], [-2.5, 0.798], [-2.9, 0.985], [-3.2, 1.0], [-3.25, 0.553], [-3.65, 0.581]] },
  // keel-hung: a flap on the keel's trailing edge, no balance area forward of the stock
  rudder: { area: 0.62, ar: 1.7, x: -3.95, cd0: 0.012, e: 0.7, alphaStall: 20 * DEG, cn: 1.3,
            eff: 0.6, deltaMax: 35 * DEG, span: 1.03 },   // 0.6 m chord, canoe to heel
  wheel: { lockToLockTurns: 2.5, rateMin: 90 * DEG, rateMax: 360 * DEG, rampTime: 1.0 },
  prop: { D: 0.38, Kt0: 0.32, J0: 0.85,
          astern: 0.85,          // Kiwiprop feathers to a proper reverse pitch
          gearAhead: 2.4, gearAstern: 2.4,   // MS2 cone-clutch box, assumed equal
          x: -3.45,              // in the aperture, ~0.5 m ahead of the rudder
          wake: 0.25,            // high wake fraction in a full-keel aperture
          slipFrac: 0.65,        // most of the blade height sits in the wash
          kR: 0.50,              // fitted, as for the 36i; see MODEL.md section 5
          tau: 0.9,
          // walk.astern is FITTED, and reduced from the fixed-prop guess: the
          // aperture amplifies walk, the symmetric feathering blades reduce it.
          // Calibrate in the slip.  hand = +1 right-hand, assumed.
          walk: { hand: 1, ahead: 0.03, astern: 0.15 } },
  engine: { ahead: [0, 800, 1200, 1800, 2400, 2800], astern: [0, 800, 1200, 1800, 2400] },
  windage: { areaFrontal: 6.5, areaLateral: 17.0, cx: 0.75, cy: 0.95, xCE: 0.8 },  // long bow overhang
  // Frame: x = 0 at MID-WATERLINE (where the hull strip model lives).  The bow
  // and stern overhangs are NOT equal, so mid-LOA is not the origin.  Owner's
  // measurements, stem to mast step: 13.62 ft at deck (pulpit) level, 10.6 ft at
  // the waterline.  Their difference is the bow overhang, 3.02 ft = 0.92 m; the
  // stern overhang is the rest of LOA - LWL, 0.45 m.  Both measurements put the
  // mast step at x = 0.924 (a unit test checks they agree).
  stemWLX: 4.155, stemDeckX: 5.075, mastStepX: 0.924,
  spriteCentreX: 0.235,          // mid-LOA sits 0.235 m forward of the origin
  // body frame, bow first, starboard side then port.  The first and last points
  // are the anchor on its bow roller, ~0.45 m forward of the deck stem.
  outline: [[5.525, 0.06], [5.075, 0.1], [4.435, 0.7], [3.235, 1.25], [1.435, 1.6], [-0.265, 1.65], [-2.265, 1.55], [-3.765, 1.25], [-4.605, 0.95], [-4.605, -0.95], [-3.765, -1.25], [-2.265, -1.55], [-0.265, -1.65], [1.435, -1.6], [3.235, -1.25], [4.435, -0.7], [5.075, -0.1], [5.525, -0.06]],
};

export const VESSELS = { union32: UNION_32, jeanneau36: JEANNEAU_36 };
export const DEFAULT_VESSEL = 'union32';

export function derived(P) {
  const m = P.mass, Izz = m * P.kzz * P.kzz;
  return {
    m, Izz,
    mx: m * (1 + P.addedMass.ax),
    my: m * (1 + P.addedMass.ay),
    Iz: Izz * (1 + P.addedMass.az),
    rudderRateMin: P.wheel.rateMin * (2 * P.rudder.deltaMax) / (P.wheel.lockToLockTurns * 2 * Math.PI),
    rudderRateMax: P.wheel.rateMax * (2 * P.rudder.deltaMax) / (P.wheel.lockToLockTurns * 2 * Math.PI),
    Ap: Math.PI * P.prop.D * P.prop.D / 4,
  };
}

// ----------------------------------------------------------------------------
// 1. Lifting surface (keel, rudder).  Thin-foil pre-stall, flat plate post-stall,
//    smoothly blended.  Periodic in pi so reversed flow is handled.
// ----------------------------------------------------------------------------
export function foilCoeffs(alpha, f) {
  const a = alpha - Math.PI * Math.round(alpha / Math.PI);   // wrap to (-pi/2, pi/2]
  const clAlpha = 2 * Math.PI * f.ar / (2 + Math.sqrt(f.ar * f.ar + 4)); // Helmbold
  const clLin = clAlpha * a;
  const cdLin = f.cd0 + clLin * clLin / (Math.PI * f.e * f.ar);
  const sa = Math.sin(a), ca = Math.cos(a);
  const clFp = f.cn * sa * ca;          // flat plate: Cn sin a cos a
  const cdFp = f.cn * sa * sa;          //             Cn sin^2 a
  // blend over [alphaStall, 1.5 alphaStall]
  let w = (Math.abs(a) - f.alphaStall) / (0.5 * f.alphaStall);
  w = Math.min(1, Math.max(0, w)); w = w * w * (3 - 2 * w);
  const eff = f.eff ?? 1;               // flap effectiveness (skeg-hung rudders)
  return { cl: eff * ((1 - w) * clLin + w * clFp), cd: (1 - w) * cdLin + w * cdFp, alphaWrapped: a };
}

// Force on a foil in body-frame axes.  (uIn, vIn) = foil velocity through the
// local water.  Returns {X, Y}.  Lift direction convention derived in MODEL.md:
// for u>0, delta>0 (TE to starboard) the force is to port.
export function foilForce(uIn, vIn, delta, f, area = f.area) {
  const V2 = uIn * uIn + vIn * vIn;
  if (V2 < 1e-6) return { X: 0, Y: 0, alpha: 0 };
  const V = Math.sqrt(V2);
  const alpha = delta + Math.atan2(vIn, uIn);
  const { cl, cd } = foilCoeffs(alpha, f);
  const q = 0.5 * RHO_W * area * V2;
  const dx = -uIn / V, dy = -vIn / V;        // unit vector of water flow past the foil
  const nx = -dy, ny = dx;                   // d rotated +90 deg
  return { X: q * (cd * dx + cl * nx), Y: q * (cd * dy + cl * ny), alpha };
}

// ----------------------------------------------------------------------------
// 2. Canoe-body hull: quadratic+linear surge resistance, strip-wise cross-flow
//    drag for sway/yaw (gives yaw damping and sway drag from one model).
// ----------------------------------------------------------------------------
export function hullForces(u, v, r, P) {
  const H = P.hull, L = P.LWL, N = H.nStrips, dx = L / N;
  let X = -(H.kQuad * u * Math.abs(u) + H.kLin * u), Y = 0, Nz = 0;
  if (H.kWave) {                                        // knee near hull speed, Fn ~ 0.4
    const uHull = 1.34 * Math.sqrt(L / 0.3048) * KN;
    X -= H.kWave * u * Math.abs(u) * (u / uHull) ** 4;
  }
  for (let i = 0; i < N; i++) {
    const x = -L / 2 + (i + 0.5) * dx;
    const T = H.T0 * (1 - (2 * x / L) ** 2);            // parabolic draft profile
    const vl = v + x * r;
    const dY = -0.5 * RHO_W * H.cdCross * T * Math.abs(vl) * vl * dx;
    Y += dY; Nz += x * dY;
  }
  return { X, Y, N: Nz };
}

// Linear interpolation of a keel depth profile [[x, d], ...] (x descending).
export function keelDepth(profile, x) {
  if (x >= profile[0][0] || x <= profile[profile.length - 1][0]) return 0;
  for (let i = 0; i < profile.length - 1; i++) {
    const [x0, d0] = profile[i], [x1, d1] = profile[i + 1];
    if (x <= x0 && x >= x1) return d0 + (d1 - d0) * (x0 - x) / (x0 - x1);
  }
  return 0;
}

export function keelForces(u, v, r, P) {
  const k = P.keel;
  if (!k.nSeg) {                                       // concentrated fin keel
    const F = foilForce(u, v + k.x * r, 0, k);
    return { X: F.X, Y: F.Y, N: k.x * F.Y, alpha: F.alpha };
  }
  // Long keel: n strips from xFwd to xAft, each seeing its own inflow v + x_i r.
  // Strip area is d(x_i) dx from a depth profile if one is given (so a Brewer
  // bite is just a stretch of reduced depth), else uniform A/n.  A lumped long
  // keel resists sideslip but gives almost no yaw damping; the distribution is
  // where that damping comes from, weighted by x_i^2 dA_i.
  let X = 0, Y = 0, N = 0;
  const dx = (k.xFwd - k.xAft) / k.nSeg;
  for (let i = 0; i < k.nSeg; i++) {
    const x = k.xFwd - (i + 0.5) * dx;
    const dA = k.profile ? keelDepth(k.profile, x) * dx : k.area / k.nSeg;
    if (dA <= 0) continue;
    const F = foilForce(u, v + x * r, 0, k, dA);
    X += F.X; Y += F.Y; N += x * F.Y;
  }
  return { X, Y, N, alpha: Math.atan2(v, u) };
}

// Fixed skeg ahead of the rudder (optional): same foil model as the keel.
export function skegForces(u, v, r, P) {
  const k = P.skeg;
  if (!k) return { X: 0, Y: 0, N: 0 };
  const F = foilForce(u, v + k.x * r, 0, k);
  return { X: F.X, Y: F.Y, N: k.x * F.Y };
}

// ----------------------------------------------------------------------------
// 3. Propeller.  T = sign(n) rho n^2 D^4 Kt(Ja),  Ja = advance ratio measured
//    in the direction of thrust.  Prop walk, where a vessel has it, is propWalk().
// ----------------------------------------------------------------------------
export function propThrust(u, n, P) {
  const p = P.prop;
  if (Math.abs(n) < 1e-6) return { T: 0, uA: u * (1 - p.wake), J: 0 };
  const uA = u * (1 - p.wake);
  const Ja = Math.sign(n) * uA / (Math.abs(n) * p.D);
  let Kt = p.Kt0 * (1 - Ja / p.J0);
  Kt = Math.min(1.5 * p.Kt0, Math.max(-0.5 * p.Kt0, Kt));
  const T = Math.sign(n) * RHO_W * n * n * p.D ** 4 * Kt * (n < 0 ? p.astern : 1);
  return { T, uA, J: Ja };
}

// Prop walk (optional, P.prop.walk).  Side force proportional to thrust, applied
// at the propeller:  Y = hand * k * T, with k = walk.ahead for T > 0 and
// walk.astern for T < 0.  hand = +1 for a right-hand prop (clockwise seen from
// astern, going ahead): ahead the stern walks to starboard (weakly); astern it
// walks to port (strongly), so the bow swings to starboard.  The coefficients
// are empirical; nothing here derives them.
export function propWalk(T, P) {
  const w = P.prop.walk;
  if (!w || T === 0) return { Y: 0, N: 0 };
  const Y = w.hand * (T > 0 ? w.ahead : w.astern) * T;
  return { Y, N: P.prop.x * Y };
}

// Rudder with propeller slipstream (momentum theory) when thrust is ahead.
export function rudderForces(u, v, r, delta, T, P) {
  const R = P.rudder, p = P.prop, Ap = Math.PI * p.D * p.D / 4;
  const vIn = v + R.x * r;
  const uA = u * (1 - p.wake);
  let uSlip = u;
  if (T > 0) {
    const uW = Math.sqrt(uA * uA + 2 * T / (RHO_W * Ap));  // far-wake velocity, always aft
    uSlip = uA + p.kR * (uW - uA);
  }
  const Fs = foilForce(uSlip, vIn, delta, R, R.area * p.slipFrac);
  const Ff = foilForce(u, vIn, delta, R, R.area * (1 - p.slipFrac));
  const X = Fs.X + Ff.X, Y = Fs.Y + Ff.Y;
  return { X, Y, N: R.x * Y, uSlip };
}

// ----------------------------------------------------------------------------
// 4. Windage.  (wE, wN) = true wind AIR VELOCITY in world frame.
// ----------------------------------------------------------------------------
export function windForces(s, wE, wN, P) {
  const [, , psi, u, v] = s;
  const W = P.windage, sp = Math.sin(psi), cp = Math.cos(psi);
  // boat velocity in world
  const vE = u * sp + v * cp, vN = u * cp - v * sp;
  const aE = wE - vE, aN = wN - vN;               // apparent wind, world
  const wx = aE * sp + aN * cp, wy = aE * cp - aN * sp;   // apparent wind, body
  const Wm = Math.hypot(wx, wy);
  const X = 0.5 * RHO_A * W.cx * W.areaFrontal * Wm * wx;
  const Y = 0.5 * RHO_A * W.cy * W.areaLateral * Wm * wy;
  return { X, Y, N: W.xCE * Y, apparent: { x: wx, y: wy, speed: Wm } };
}

// ----------------------------------------------------------------------------
// 5. Controls
// ----------------------------------------------------------------------------
// engine rpm <-> shaft rev/s through whichever reduction is engaged
export function engineToShaft(rpm, P) { return rpm / 60 / (rpm < 0 ? P.prop.gearAstern : P.prop.gearAhead); }
export function shaftToEngine(n, P) { return n * 60 * (n < 0 ? P.prop.gearAstern : P.prop.gearAhead); }

export function rpmSetpoint(throttleIndex, P) {
  const E = P.engine;
  if (throttleIndex >= 0) return E.ahead[Math.min(throttleIndex, E.ahead.length - 1)];
  return -E.astern[Math.min(-throttleIndex, E.astern.length - 1)];
}
export function clampThrottle(idx, P) {
  return Math.max(-(P.engine.astern.length - 1), Math.min(P.engine.ahead.length - 1, idx));
}
// wheel command: -1 (port / 'z'), 0, +1 (starboard / 'x'); heldTime in s
export function rudderRate(cmd, heldTime, P) {
  if (!cmd) return 0;
  const D = derived(P);
  const f = Math.min(1, heldTime / P.wheel.rampTime);
  return cmd * (D.rudderRateMin + f * (D.rudderRateMax - D.rudderRateMin));
}

// ----------------------------------------------------------------------------
// 6. Equations of motion.  inp = {wheelCmd, heldTime, throttleIndex, wind:{E,N},
//    extra(s)->{X,Y,N}} (extra = contact forces from the app, optional).
// ----------------------------------------------------------------------------
export function sumForces(s, inp, P) {
  const [, , , u, v, r, delta, n] = s;
  const hull = hullForces(u, v, r, P);
  const keel = keelForces(u, v, r, P);
  const skeg = skegForces(u, v, r, P);
  const prop = propThrust(u, n, P);
  const walk = propWalk(prop.T, P);
  const rud = rudderForces(u, v, r, delta, prop.T, P);
  const wind = inp.wind ? windForces(s, inp.wind.E, inp.wind.N, P) : { X: 0, Y: 0, N: 0 };
  const ext = inp.extra ? inp.extra(s) : { X: 0, Y: 0, N: 0 };
  return {
    X: hull.X + keel.X + skeg.X + prop.T + rud.X + wind.X + ext.X,
    Y: hull.Y + keel.Y + skeg.Y + walk.Y + rud.Y + wind.Y + ext.Y,
    N: hull.N + keel.N + skeg.N + walk.N + rud.N + wind.N + ext.N,
    parts: { hull, keel, skeg, prop, walk, rud, wind, ext },
  };
}

export function derivative(s, inp, P) {
  const D = derived(P);
  const [, , psi, u, v, r, delta, n] = s;
  const F = sumForces(s, inp, P);
  const du = (F.X + D.my * v * r) / D.mx;
  const dv = (F.Y - D.mx * u * r) / D.my;
  const dr = (F.N - P.addedMass.munk * (D.my - D.mx) * u * v) / D.Iz;
  const sp = Math.sin(psi), cp = Math.cos(psi);
  let ddelta = rudderRate(inp.wheelCmd, inp.heldTime, P);
  if ((delta >= P.rudder.deltaMax && ddelta > 0) || (delta <= -P.rudder.deltaMax && ddelta < 0)) ddelta = 0;
  const nSet = engineToShaft(rpmSetpoint(inp.throttleIndex, P), P);
  const dn = (nSet - n) / P.prop.tau;
  return [u * sp + v * cp, u * cp - v * sp, r, du, dv, dr, ddelta, dn];
}

// ----------------------------------------------------------------------------
// 7. Integrator (classical RK4, fixed step), generic in the derivative.
// ----------------------------------------------------------------------------
export function rk4(f, s, h) {
  const n = s.length, add = (a, b, c) => a.map((x, i) => x + c * b[i]);
  const k1 = f(s);
  const k2 = f(add(s, k1, h / 2));
  const k3 = f(add(s, k2, h / 2));
  const k4 = f(add(s, k3, h));
  return s.map((x, i) => x + (h / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
}

export function stepBoat(s, inp, P, h) {
  const out = rk4(x => derivative(x, inp, P), s, h);
  out[6] = Math.max(-P.rudder.deltaMax, Math.min(P.rudder.deltaMax, out[6]));
  out[2] = ((out[2] + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI;
  return out;
}

export function initialState(xE = 0, yN = 0, psiDeg = 0) {
  return [xE, yN, psiDeg * DEG, 0, 0, 0, 0, 0];
}
