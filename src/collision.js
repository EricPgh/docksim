// collision.js — hull/dock contact as a penalty spring-damper with Coulomb
// friction.  Docks are polygons in world metres.  Hull is sampled along its
// outline; each sample inside a dock polygon gets pushed toward the nearest
// edge.  Adequate for fender-speed contacts (< ~1 m/s).
export const CONTACT = { k: 25000, c: 6000, mu: 0.4, sampleSpacing: 0.3 };

export function pointInPolygon(px, py, poly) {
  let inside = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const [xi, yi] = poly[i], [xj, yj] = poly[j];
    if ((yi > py) !== (yj > py) && px < (xj - xi) * (py - yi) / (yj - yi) + xi) inside = !inside;
  }
  return inside;
}

// nearest point on polygon boundary -> {d, nx, ny} where (nx,ny) points from p to boundary
export function nearestEdge(px, py, poly) {
  let best = { d: Infinity, nx: 0, ny: 0 };
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const [ax, ay] = poly[j], [bx, by] = poly[i];
    const abx = bx - ax, aby = by - ay, L2 = abx * abx + aby * aby || 1e-12;
    let t = ((px - ax) * abx + (py - ay) * aby) / L2; t = Math.max(0, Math.min(1, t));
    const qx = ax + t * abx, qy = ay + t * aby, d = Math.hypot(qx - px, qy - py);
    if (d < best.d) best = { d, nx: (qx - px) / (d || 1e-9), ny: (qy - py) / (d || 1e-9) };
  }
  return best;
}

export function sampleOutline(outline, spacing = CONTACT.sampleSpacing) {
  const pts = [];
  for (let i = 0; i < outline.length; i++) {
    const [x0, y0] = outline[i], [x1, y1] = outline[(i + 1) % outline.length];
    const L = Math.hypot(x1 - x0, y1 - y0), n = Math.max(1, Math.ceil(L / spacing));
    for (let k = 0; k < n; k++) pts.push([x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n]);
  }
  return pts;
}

// Returns body-frame {X, Y, N, contacts[]} for state s.
//   obstacles : polygons whose INSIDE is land/dock (contact when a hull point is inside)
//   water     : polygons whose INSIDE is navigable (contact when a hull point is outside all of them)
// Both use the same push: toward the nearest boundary point of the offending polygon.
export function contactForces(s, hullPts, obstacles, water = [], C = CONTACT) {
  const [xE, yN, psi, u, v, r] = s;
  const sp = Math.sin(psi), cp = Math.cos(psi);
  let X = 0, Y = 0, N = 0; const contacts = [];
  for (const [bx, by] of hullPts) {
    const pE = xE + bx * sp + by * cp, pN = yN + bx * cp - by * sp;
    const hits = [];
    for (const poly of obstacles) if (pointInPolygon(pE, pN, poly)) hits.push(nearestEdge(pE, pN, poly));
    if (water.length && !water.some(poly => pointInPolygon(pE, pN, poly))) {
      let best = null;
      for (const poly of water) { const e = nearestEdge(pE, pN, poly); if (!best || e.d < best.d) best = e; }
      hits.push(best);
    }
    for (const { d, nx, ny } of hits) {
      // velocity of the hull point, body then world
      const vbx = u - r * by, vby = v + r * bx;
      const vE = vbx * sp + vby * cp, vN = vbx * cp - vby * sp;
      const vn = vE * nx + vN * ny;                       // + = moving toward boundary (leaving)
      const Fn = Math.max(0, C.k * d - C.c * vn);
      const tx = -ny, ty = nx, vt = vE * tx + vN * ty;
      const Ft = -Math.sign(vt) * Math.min(C.mu * Fn, C.c * Math.abs(vt));
      const FE = Fn * nx + Ft * tx, FN = Fn * ny + Ft * ty;
      const Fx = FE * sp + FN * cp, Fy = FE * cp - FN * sp;  // world -> body
      X += Fx; Y += Fy; N += bx * Fy - by * Fx;
      contacts.push({ E: pE, N: pN, Fn, depth: d });
    }
  }
  return { X, Y, N, contacts };
}
