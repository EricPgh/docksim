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
// beam 3.59 m, draft 1.94 m, 5700 kg light, Yanmar 3YM30 29 hp, single spade
// rudder).  Everything a user might want to tune lives here.
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
  prop: { D: 0.40, Kt0: 0.32, J0: 0.85, astern: 0.65, gear: 2.21, x: -3.6,
          wake: 0.05, slipFrac: 0.45, kR: 0.5, tau: 0.8 },
  engine: { ahead: [0, 800, 1000, 1500, 2000, 2500], astern: [0, 800, 1000, 1500, 2000] },
  windage: { areaFrontal: 6.0, areaLateral: 18.0, cx: 0.7, cy: 0.9, xCE: 0.5 },
  // hull outline in body frame, bow first, clockwise (x fwd, y stbd)
  outline: [[5.47, 0], [4.6, 0.75], [3.2, 1.40], [1.2, 1.78], [-1.4, 1.76], [-3.4, 1.55],
            [-4.9, 1.20], [-5.47, 0.95], [-5.47, -0.95], [-4.9, -1.20], [-3.4, -1.55],
            [-1.4, -1.76], [1.2, -1.78], [3.2, -1.40], [4.6, -0.75]],
};

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
  return { cl: (1 - w) * clLin + w * clFp, cd: (1 - w) * cdLin + w * cdFp, alphaWrapped: a };
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
  for (let i = 0; i < N; i++) {
    const x = -L / 2 + (i + 0.5) * dx;
    const T = H.T0 * (1 - (2 * x / L) ** 2);            // parabolic draft profile
    const vl = v + x * r;
    const dY = -0.5 * RHO_W * H.cdCross * T * Math.abs(vl) * vl * dx;
    Y += dY; Nz += x * dY;
  }
  return { X, Y, N: Nz };
}

export function keelForces(u, v, r, P) {
  const k = P.keel;
  const F = foilForce(u, v + k.x * r, 0, k);
  return { X: F.X, Y: F.Y, N: k.x * F.Y, alpha: F.alpha };
}

// ----------------------------------------------------------------------------
// 3. Propeller.  T = sign(n) rho n^2 D^4 Kt(Ja),  Ja = advance ratio measured
//    in the direction of thrust.  No prop walk (by request).
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
  const prop = propThrust(u, n, P);
  const rud = rudderForces(u, v, r, delta, prop.T, P);
  const wind = inp.wind ? windForces(s, inp.wind.E, inp.wind.N, P) : { X: 0, Y: 0, N: 0 };
  const ext = inp.extra ? inp.extra(s) : { X: 0, Y: 0, N: 0 };
  return {
    X: hull.X + keel.X + prop.T + rud.X + wind.X + ext.X,
    Y: hull.Y + keel.Y + rud.Y + wind.Y + ext.Y,
    N: hull.N + keel.N + rud.N + wind.N + ext.N,
    parts: { hull, keel, prop, rud, wind, ext },
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
  const nSet = rpmSetpoint(inp.throttleIndex, P) / 60 / P.prop.gear;
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
