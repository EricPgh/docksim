// wind.js — variable wind: mean + Ornstein–Uhlenbeck fluctuations in speed
// and direction + Poisson-arriving puffs.  Direction is meteorological
// ("from"), clockwise from north.  Returns AIR VELOCITY vector (E, N).
import { KN, DEG } from './physics.js';

export function mulberry32(seed) {
  let a = seed >>> 0;
  return function () {
    a = (a + 0x6D2B79F5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
export function gaussian(rng) {
  let u = 0, v = 0;
  while (u === 0) u = rng();
  while (v === 0) v = rng();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

export const WIND_PRESETS = {
  calm:     { meanKn: 0,  dirDeg: 0,   gustSigma: 0,    gustTau: 20, dirSigmaDeg: 0,  dirTau: 60, puffRate: 0,    puffAmp: 0 },
  light:    { meanKn: 5,  dirDeg: 250, gustSigma: 0.15, gustTau: 15, dirSigmaDeg: 8,  dirTau: 40, puffRate: 1/60, puffAmp: 0.4 },
  moderate: { meanKn: 12, dirDeg: 200, gustSigma: 0.20, gustTau: 12, dirSigmaDeg: 10, dirTau: 40, puffRate: 1/40, puffAmp: 0.5 },
  gusty:    { meanKn: 18, dirDeg: 300, gustSigma: 0.30, gustTau: 8,  dirSigmaDeg: 15, dirTau: 30, puffRate: 1/25, puffAmp: 0.7 },
};

export class WindModel {
  constructor(cfg = WIND_PRESETS.moderate, seed = 1) {
    this.rng = mulberry32(seed);
    this.set(cfg);
  }
  set(cfg) {
    this.cfg = { ...cfg };
    this.g = 0;           // OU gust factor  (speed = mean (1+g) + puffs)
    this.d = 0;           // OU direction deviation [rad]
    this.puffs = [];
    this.t = 0;
    this.nextPuff = this._draw();
  }
  static random(rng) {
    const meanKn = Math.round(rng() * 20);
    return { meanKn, dirDeg: Math.round(rng() * 360), gustSigma: 0.1 + 0.25 * rng(),
             gustTau: 8 + 12 * rng(), dirSigmaDeg: 5 + 15 * rng(), dirTau: 30 + 30 * rng(),
             puffRate: rng() / 30, puffAmp: 0.3 + 0.5 * rng() };
  }
  _draw() { const r = this.cfg.puffRate; return r > 0 ? this.t - Math.log(1 - this.rng()) / r : Infinity; }
  // exact OU update: x_{k+1} = x_k e^{-h/tau} + sigma sqrt(1-e^{-2h/tau}) xi
  _ou(x, sigma, tau, h) {
    if (sigma === 0) return 0;
    const e = Math.exp(-h / tau);
    return x * e + sigma * Math.sqrt(1 - e * e) * gaussian(this.rng);
  }
  step(h) {
    const c = this.cfg;
    this.t += h;
    this.g = this._ou(this.g, c.gustSigma, c.gustTau, h);
    this.d = this._ou(this.d, c.dirSigmaDeg * DEG, c.dirTau, h);
    while (this.t >= this.nextPuff) {
      this.puffs.push({ t0: this.nextPuff, dur: 6 + 10 * this.rng(),
                        amp: c.puffAmp * c.meanKn * KN * (0.5 + 0.5 * this.rng()),
                        dd: (this.rng() - 0.5) * 30 * DEG });
      this.nextPuff = this._draw();
    }
    this.puffs = this.puffs.filter(p => this.t < p.t0 + p.dur);
  }
  get state() {
    const c = this.cfg;
    let speed = c.meanKn * KN * (1 + this.g), dir = c.dirDeg * DEG + this.d;
    for (const p of this.puffs) {
      const s = Math.sin(Math.PI * (this.t - p.t0) / p.dur) ** 2;
      speed += p.amp * s; dir += p.dd * s;
    }
    speed = Math.max(0, speed);
    // air moves TOWARD dir+180: E = -sin(dir) V, N = -cos(dir) V
    return { speed, dirDeg: ((dir / DEG) % 360 + 360) % 360, E: -Math.sin(dir) * speed, N: -Math.cos(dir) * speed };
  }
}
