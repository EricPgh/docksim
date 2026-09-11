// shelter.js — pre-computed local wind modification around buildings/terrain.
//
// Röckle-type diagnostic model (the family behind QUIC-URB): for each wind
// direction bin, empirical flow zones are stamped around every obstacle with a
// stated height, then the field is made approximately divergence-free by a
// variational adjustment.  The result is quantized to bytes, stored in the
// scenario JSON, and sampled at run time as a scalar multiplier + deflection.
//
// What this is NOT: a CFD solution.  The zone parameterisations are empirical
// fits from wind-tunnel work, the flow is depth-averaged so building downwash
// (a genuinely 3-D effect) is only mimicked, and no thermal effects exist.
// Coefficients below are borrowed, not derived; treat them as tunable.
import { pointInPolygon } from './collision.js';

export const SHELTER_DEFAULTS = {
  dx: 2.0,          // grid spacing [m]
  nDirs: 16,        // direction bins (22.5 deg)
  cavityMult: 0.30, // mean speed in the near lee cavity, fraction of free stream
  // (windward stagnation and corner acceleration are NOT stamped: the projection
  //  in stage (a) produces them from the no-penetration condition)
  wakeEnd: 3.0,     // far wake extends to wakeEnd * Lr
  cavityTurb: 2.2,  // gust-intensity multiplier inside the cavity
  sorIters: 400, sorOmega: 1.7,
  maxMult: 2.0,     // quantization ceiling (byte = mult / maxMult * 255)
};

// Wilson/Röckle lee-cavity length.  W crosswind width, L along-wind length,
// H height, all in metres:  Lr/H = 1.8 (W/H) / [ (L/H)^0.3 (1 + 0.24 W/H) ]
export function cavityLength(W, L, H) {
  if (H <= 0) return 0;
  const w = W / H, l = Math.max(0.1, L / H);
  return H * 1.8 * w / (Math.pow(l, 0.3) * (1 + 0.24 * w));
}

// ---------------------------------------------------------------------------
// Field construction
// ---------------------------------------------------------------------------
// obstacles: [{ poly: [[E,N],...], height: metres }]  (height <= 0 is ignored)
export function buildShelterField(obstacles, bounds, opts = {}) {
  const O = { ...SHELTER_DEFAULTS, ...opts };
  const { x0, y0, x1, y1 } = bounds;
  const nx = Math.max(2, Math.ceil((x1 - x0) / O.dx) + 1);
  const ny = Math.max(2, Math.ceil((y1 - y0) / O.dx) + 1);
  const n = nx * ny;
  const solid = new Uint8Array(n);                    // 1 = inside an obstacle
  const built = obstacles.filter(o => o.height > 0 && o.poly && o.poly.length >= 3);

  for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++) {
    const E = x0 + i * O.dx, N = y0 + j * O.dx;
    for (const o of built) if (pointInPolygon(E, N, o.poly)) { solid[j * nx + i] = 1; break; }
  }

  const speed = [], defl = [], turb = [];
  for (let d = 0; d < O.nDirs; d++) {
    const dirDeg = d * 360 / O.nDirs;
    const f = computeDirection(built, dirDeg, { x0, y0, nx, ny, dx: O.dx, solid }, O);
    speed.push(f.speed); defl.push(f.defl); turb.push(f.turb);
  }
  return { x0, y0, dx: O.dx, nx, ny, nDirs: O.nDirs, maxMult: O.maxMult, solid, speed, defl, turb };
}

// Two stages, in this order:
//   (a) INVISCID FLOW-AROUND.  Project the uniform free stream so it does not
//       penetrate the obstacles.  This alone produces the stagnation slow-down
//       on the windward face and the acceleration past the corners, and it
//       conserves mass, because a solid body really is a mass barrier.
//   (b) EMPIRICAL WAKE.  Multiply in the lee cavity / far wake deficit and raise
//       the turbulence there.
// The order matters.  Projecting a field that already contains the wake makes
// the solver read the velocity deficit as a mass SINK and suck flow inward from
// upwind, which is unphysical: a wake is a momentum deficit, not a sink.
function computeDirection(built, dirDeg, G, O) {
  const { x0, y0, nx, ny, dx, solid } = G, n = nx * ny;
  const th = dirDeg * Math.PI / 180;
  const fx = -Math.sin(th), fy = -Math.cos(th);          // unit vector the air moves along
  const gx = -fy, gy = fx;                               // crosswind unit vector
  const u = new Float32Array(n).fill(fx), v = new Float32Array(n).fill(fy);
  const turbF = new Float32Array(n).fill(1);

  // ---- (a) MAC projection for no-penetration -------------------------------
  // Velocities on cell faces, lambda at cell centres: staggering makes the
  // divergence and gradient operators adjoint, so the composed operator is the
  // 5-point Laplacian.  A collocated grid with centred differences decouples odd
  // and even points (checkerboard) and barely reduces divergence.
  // Walls: face velocity zero, Neumann on lambda.  Domain edge: Dirichlet
  // lambda = 0 so flow may enter and leave (an all-Neumann problem would be
  // incompatible, its source not integrating to zero, and SOR would drift).
  if (built.length && O.sorIters > 0) {
    const fluid = k => !solid[k];
    const uf = new Float32Array((nx + 1) * ny), vf = new Float32Array(nx * (ny + 1));
    for (let j = 0; j < ny; j++) for (let i = 0; i <= nx; i++) {
      const L = i - 1 >= 0 ? j * nx + i - 1 : -1, R = i < nx ? j * nx + i : -1;
      const ok = (L < 0 || fluid(L)) && (R < 0 || fluid(R));
      uf[j * (nx + 1) + i] = ok ? fx : 0;
    }
    for (let j = 0; j <= ny; j++) for (let i = 0; i < nx; i++) {
      const D = j - 1 >= 0 ? (j - 1) * nx + i : -1, U2 = j < ny ? j * nx + i : -1;
      const ok = (D < 0 || fluid(D)) && (U2 < 0 || fluid(U2));
      vf[j * nx + i] = ok ? fy : 0;
    }
    const div = new Float32Array(n), lam = new Float32Array(n);
    for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++) {
      const k = j * nx + i; if (solid[k]) continue;
      div[k] = (uf[j * (nx + 1) + i + 1] - uf[j * (nx + 1) + i] + vf[(j + 1) * nx + i] - vf[j * nx + i]) / dx;
    }
    for (let it = 0; it < O.sorIters; it++) {
      for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++) {
        const k = j * nx + i; if (solid[k]) continue;
        let sum = 0, cnt = 0;
        for (const [a, b] of [[i + 1, j], [i - 1, j], [i, j + 1], [i, j - 1]]) {
          if (a < 0 || b < 0 || a >= nx || b >= ny) { cnt++; continue; }   // Dirichlet 0
          const kk = b * nx + a;
          if (solid[kk]) continue;                                        // Neumann: drop the term
          sum += lam[kk]; cnt++;
        }
        if (cnt) lam[k] += O.sorOmega * ((sum + dx * dx * div[k]) / cnt - lam[k]);   // lap(lam) = -div, so +dx^2 div here
      }
    }
    for (let j = 0; j < ny; j++) for (let i = 1; i < nx; i++) {
      const L = j * nx + i - 1, R = j * nx + i;
      if (fluid(L) && fluid(R)) uf[j * (nx + 1) + i] += (lam[R] - lam[L]) / dx;
    }
    for (let j = 1; j < ny; j++) for (let i = 0; i < nx; i++) {
      const D = (j - 1) * nx + i, U2 = j * nx + i;
      if (fluid(D) && fluid(U2)) vf[j * nx + i] += (lam[U2] - lam[D]) / dx;
    }
    for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++) {
      const k = j * nx + i; if (solid[k]) continue;
      u[k] = 0.5 * (uf[j * (nx + 1) + i] + uf[j * (nx + 1) + i + 1]);
      v[k] = 0.5 * (vf[j * nx + i] + vf[(j + 1) * nx + i]);
    }
  }

  // ---- (b) empirical wake deficit ------------------------------------------
  for (const o of built) {
    const H = o.height;
    let sMin = Infinity, sMax = -Infinity, tMin = Infinity, tMax = -Infinity;
    for (const [E, N] of o.poly) {
      const sx = E * fx + N * fy, ty = E * gx + N * gy;
      sMin = Math.min(sMin, sx); sMax = Math.max(sMax, sx);
      tMin = Math.min(tMin, ty); tMax = Math.max(tMax, ty);
    }
    const L = sMax - sMin, W = tMax - tMin, tc = (tMin + tMax) / 2;
    const Lr = cavityLength(W, L, H);
    if (Lr <= 0) continue;
    const far = O.wakeEnd * Lr;
    for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++) {
      const k = j * nx + i; if (solid[k]) continue;
      const E = x0 + i * dx, N = y0 + j * dx;
      const s = E * fx + N * fy, t = (E * gx + N * gy) - tc;
      if (s <= sMax || s > sMax + far) continue;
      const q = (s - sMax) / far, half = (W / 2) * (1 + 0.6 * q);
      if (Math.abs(t) > half) continue;
      const lat = 1 - (Math.abs(t) / half) ** 2;                 // taper to the wake edge
      const near = (s - sMax) <= Lr;
      const base = near ? O.cavityMult
                        : O.cavityMult + (1 - O.cavityMult) * ((s - sMax - Lr) / (far - Lr));
      const m = 1 + (base - 1) * lat;
      u[k] *= m; v[k] *= m;
      const decay = near ? 1 : Math.max(0, 1 - (s - sMax - Lr) / (far - Lr));
      turbF[k] = Math.max(turbF[k], 1 + (O.cavityTurb - 1) * lat * decay);
    }
  }

  // ---- (c) quantize --------------------------------------------------------
  // Solid cells carry zero velocity (no penetration at the wall), so bilinear
  // sampling next to a wall tends to zero rather than blending toward the
  // free-stream value left in the fill.
  for (let k = 0; k < n; k++) if (solid[k]) { u[k] = 0; v[k] = 0; turbF[k] = 1; }
  const speed = new Uint8Array(n), defl = new Int8Array(n), turb = new Uint8Array(n);
  for (let k = 0; k < n; k++) {
    const m = Math.min(O.maxMult, Math.hypot(u[k], v[k]));
    speed[k] = Math.round(m / O.maxMult * 255);
    const a = Math.atan2(u[k] * fy - v[k] * fx, u[k] * fx + v[k] * fy) * 180 / Math.PI;
    defl[k] = Math.max(-90, Math.min(90, Math.round(a)));
    turb[k] = Math.min(255, Math.round(turbF[k] / O.maxMult * 255));
  }
  return { speed, defl, turb };
}

// ---------------------------------------------------------------------------
// Sampling: bilinear in space, linear between the two nearest direction bins.
// Returns { mult, deflDeg, turb }.  mult = 1 outside the field.
// ---------------------------------------------------------------------------
export function sampleShelter(F, E, N, dirDeg) {
  if (!F) return { mult: 1, deflDeg: 0, turb: 1 };
  const gx = (E - F.x0) / F.dx, gy = (N - F.y0) / F.dx;
  if (gx < 0 || gy < 0 || gx > F.nx - 1 || gy > F.ny - 1) return { mult: 1, deflDeg: 0, turb: 1 };
  const i = Math.floor(gx), j = Math.floor(gy), fx = gx - i, fy = gy - j;
  const i1 = Math.min(i + 1, F.nx - 1), j1 = Math.min(j + 1, F.ny - 1);
  const bin = ((dirDeg % 360) + 360) % 360 / (360 / F.nDirs);
  const d0 = Math.floor(bin) % F.nDirs, d1 = (d0 + 1) % F.nDirs, fd = bin - Math.floor(bin);
  const bilin = (arr, scale) => {
    const a = arr[j * F.nx + i] * scale, b = arr[j * F.nx + i1] * scale;
    const c = arr[j1 * F.nx + i] * scale, e = arr[j1 * F.nx + i1] * scale;
    return (a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + e * fx) * fy;
  };
  const s = F.maxMult / 255;
  const mult = bilin(F.speed[d0], s) * (1 - fd) + bilin(F.speed[d1], s) * fd;
  const turb = bilin(F.turb[d0], s) * (1 - fd) + bilin(F.turb[d1], s) * fd;
  const deflDeg = bilin(F.defl[d0], 1) * (1 - fd) + bilin(F.defl[d1], 1) * fd;
  return { mult, deflDeg, turb };
}

// ---------------------------------------------------------------------------
// (De)serialization — base64 of the byte planes, so a field costs ~1 byte per
// cell per direction rather than ~5 characters as JSON numbers.
// ---------------------------------------------------------------------------
const b64 = {
  enc: arr => btoa(String.fromCharCode(...new Uint8Array(arr.buffer, arr.byteOffset, arr.byteLength))),
  dec: (str, Type) => { const bin = atob(str), a = new Uint8Array(bin.length); for (let i = 0; i < bin.length; i++) a[i] = bin.charCodeAt(i); return new Type(a.buffer); },
};
export function serializeShelter(F) {
  if (!F) return null;
  return { x0: F.x0, y0: F.y0, dx: F.dx, nx: F.nx, ny: F.ny, nDirs: F.nDirs, maxMult: F.maxMult,
           solid: b64.enc(F.solid), speed: F.speed.map(b64.enc), defl: F.defl.map(b64.enc), turb: F.turb.map(b64.enc) };
}
export function deserializeShelter(S) {
  if (!S) return null;
  return { ...S, solid: b64.dec(S.solid, Uint8Array), speed: S.speed.map(s => b64.dec(s, Uint8Array)),
           defl: S.defl.map(s => b64.dec(s, Int8Array)), turb: S.turb.map(s => b64.dec(s, Uint8Array)) };
}
