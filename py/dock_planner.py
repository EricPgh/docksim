#!/usr/bin/env python3
"""dock_planner.py — plan a docking manoeuvre under power and render it.

    python3 dock_planner.py scenario.json --wind 12 200 \\
        --start 65 45 90 --goal 54.5 8.5 180 --out docking.mp4

Wind is (speed knots, direction it blows FROM, degrees clockwise from north).
Start/goal are (E metres, N metres, heading degrees) in the scenario frame:
the same frame the app uses, and what "Place boat" writes into the JSON.

Method
------
The plant is the numpy port of the app's physics (sailsim_physics.py, verified
against the JS to 1e-13).  A manoeuvre is K piecewise-constant segments of
(duration, rudder target, throttle step); the rudder follows its target
through the rate-limited wheel and the engine has its rpm lag, exactly as in
the app.  Hundreds of candidate manoeuvres are rolled out at once.

Cost of a manoeuvre = min over t of
    pose error(t) + speed(t) + accumulated penalties(t) + w_T t
so a plan may "arrive" whenever it likes; the argmin defines the end of the
movie.  Penalties: integrated hull penetration into land (heavy), leaving
water polygons, high rpm, gear changes.

Optimiser: cross-entropy method on the segment parameters (derivative-free,
handles the discrete throttle), then refinement rounds with shrinking
variance, then a validation rollout at the fine render time step.  If the
arrival tolerance is not met the horizon and segment count are widened and
the search repeats, warm-started from the best plan so far.  That is the
"iterate as needed".

Output: an mp4 (or gif if ffmpeg is absent), a plan JSON with the segment
schedule, and a plain-language narrative on stdout.
"""
import argparse, base64, json, os, sys, time
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sailsim_physics import (P, DEG, KN, derived, initial_state, rk4_step, rpm_setpoint,
                             contact_forces, sample_outline, shaft_to_engine, prop_thrust)

DMAX = P['rudder']['deltaMax']
THR_MIN, THR_MAX = -(len(P['engine']['astern']) - 1), len(P['engine']['ahead']) - 1


# ----------------------------------------------------------------------------- scenario
def load_scenario(path):
    sc = json.load(open(path))
    docks = [np.array(p, dtype=float) for p in sc.get('docks', [])]
    water = [np.array(p, dtype=float) for p in sc.get('water', [])]
    shelter = decode_shelter(sc['shelter']) if sc.get('shelter') else None
    return dict(docks=docks, water=water, widthM=sc.get('widthM', 100), heightM=sc.get('heightM', 100),
                shelter=shelter, raw=sc)


def decode_shelter(S):
    dec = lambda s, dt: np.frombuffer(base64.b64decode(s), dtype=dt).astype(float)
    return dict(x0=S['x0'], y0=S['y0'], dx=S['dx'], nx=S['nx'], ny=S['ny'], nDirs=S['nDirs'], maxMult=S['maxMult'],
                speed=[dec(s, np.uint8) * S['maxMult'] / 255 for s in S['speed']],
                defl=[dec(s, np.int8) for s in S['defl']])


def sample_shelter(F, E, N, dirDeg):
    """Vectorised bilinear sample of the stored field.  Returns mult, deflDeg arrays."""
    if F is None:
        return np.ones_like(E), np.zeros_like(E)
    gx, gy = (E - F['x0']) / F['dx'], (N - F['y0']) / F['dx']
    inside = (gx >= 0) & (gy >= 0) & (gx <= F['nx'] - 1) & (gy <= F['ny'] - 1)
    gx, gy = np.clip(gx, 0, F['nx'] - 1), np.clip(gy, 0, F['ny'] - 1)
    i, j = np.floor(gx).astype(int), np.floor(gy).astype(int)
    fx, fy = gx - i, gy - j
    i1, j1 = np.minimum(i + 1, F['nx'] - 1), np.minimum(j + 1, F['ny'] - 1)
    b = (dirDeg % 360) / (360 / F['nDirs'])
    d0, d1, fd = int(np.floor(b)) % F['nDirs'], (int(np.floor(b)) + 1) % F['nDirs'], b - np.floor(b)
    def bil(arr):
        nx = F['nx']
        return ((arr[j * nx + i] * (1 - fx) + arr[j * nx + i1] * fx) * (1 - fy)
                + (arr[j1 * nx + i] * (1 - fx) + arr[j1 * nx + i1] * fx) * fy)
    mult = bil(F['speed'][d0]) * (1 - fd) + bil(F['speed'][d1]) * fd
    defl = bil(F['defl'][d0]) * (1 - fd) + bil(F['defl'][d1]) * fd
    return np.where(inside, mult, 1.0), np.where(inside, defl, 0.0)


# ----------------------------------------------------------------------------- rollout
class Rollout:
    """Batched simulation of N manoeuvres given theta (N, K, 3) = [dur, rudder, throttle]."""

    def __init__(self, scen, wind_kn, wind_dir, start, goal, dt, tmax, hull_spacing):
        self.scen, self.dt, self.tmax = scen, dt, tmax
        self.wind_speed, self.wind_dir = wind_kn * KN, wind_dir
        self.start, self.goal = start, goal
        self.hull = sample_outline(P['outline'], hull_spacing)
        self.D = derived(P)
        self.nsteps = int(round(tmax / dt))

    def wind_at(self, S):
        mult, defl = sample_shelter(self.scen['shelter'], S[0], S[1], self.wind_dir)
        d = (self.wind_dir + defl) * DEG
        spd = self.wind_speed * mult
        return -np.sin(d) * spd, -np.cos(d) * spd

    def run(self, theta, record=False):
        N, K, _ = theta.shape
        dur = np.clip(theta[:, :, 0], 1.0, 40.0)
        tgt = np.clip(theta[:, :, 1], -DMAX, DMAX)
        thr = np.clip(np.round(theta[:, :, 2]), THR_MIN, THR_MAX).astype(int)
        cum = np.cumsum(dur, axis=1)                      # (N, K) segment end times
        seg_start = cum - dur
        S = initial_state(N, *self.start)
        idx = np.arange(N)
        T = self.nsteps
        pos_err = np.zeros((T, N)); head_err = np.zeros((T, N)); speed = np.zeros((T, N)); yaw = np.zeros((T, N))
        pen = np.zeros((T, N))
        gear_changes = (np.diff(thr, axis=1) != 0).sum(1).astype(float)
        rpm_pen = (np.abs(thr) >= 4).sum(1).astype(float) * 0.5 + (np.abs(thr) >= 5).sum(1) * 1.0
        hist = np.zeros((T, 8, N)) if record else None
        rr = self.D
        gE, gN, gpsi = self.goal[0], self.goal[1], self.goal[2] * DEG
        for k in range(T):
            t = k * self.dt
            seg = np.minimum((cum <= t).sum(1), K - 1)
            target = tgt[idx, seg]
            t_in = t - seg_start[idx, seg]
            rate = rr['rudderRateMin'] + (rr['rudderRateMax'] - rr['rudderRateMin']) * np.minimum(1.0, t_in / P['wheel']['rampTime'])
            rudder_rate = np.clip((target - S[6]) / 0.15, -rate, rate)
            rpm = rpm_setpoint(thr[idx, seg])
            wE, wN = self.wind_at(S)
            # contact evaluated once per step and held across the RK4 stages
            Xc, Yc, Nc, pk = contact_forces(S, self.hull, self.scen['docks'], self.scen['water'])
            inp = dict(rudder_rate=rudder_rate, rpm_set=rpm, wE=wE, wN=wN)
            if record:
                hist[k] = S
            S = rk4_step(S, inp, self.dt, P, extra=lambda st: (Xc, Yc, Nc))
            pos_err[k] = np.hypot(S[0] - gE, S[1] - gN)
            head_err[k] = np.abs((S[2] - gpsi + np.pi) % (2 * np.pi) - np.pi)
            speed[k] = np.hypot(S[3], S[4]); yaw[k] = np.abs(S[5])
            pen[k] = pk
        cum_pen = np.cumsum(pen, axis=0) * self.dt
        tt = (np.arange(T) * self.dt)[:, None]
        terminal = 4.0 * pos_err ** 2 + 60.0 * head_err ** 2 + 40.0 * speed ** 2 + 30.0 * yaw ** 2
        cost_t = terminal + 3000.0 * cum_pen + 0.02 * tt
        best_k = cost_t.argmin(0)
        cost = cost_t[best_k, idx] + 0.4 * gear_changes + rpm_pen
        arrive = dict(t=best_k * self.dt, pos=pos_err[best_k, idx], head=head_err[best_k, idx] / DEG,
                      speed=speed[best_k, idx], contact=cum_pen[best_k, idx])
        return cost, arrive, hist, (dur, tgt, thr)


# ----------------------------------------------------------------------------- optimiser
def cem(ro, K, pop, iters, mu=None, sigma=None, rng=None, log=print, patience=12):
    rng = rng or np.random.default_rng(0)
    if mu is None:
        mu = np.tile(np.array([8.0, 0.0, 1.0]), (K, 1))
    if sigma is None:
        sigma = np.tile(np.array([5.0, 20 * DEG, 2.0]), (K, 1))
    lo = np.array([1.0, -DMAX, THR_MIN - 0.49]); hi = np.array([40.0, DMAX, THR_MAX + 0.49])
    best, best_theta, best_arr, stale = np.inf, None, None, 0
    n_elite = max(8, pop // 10)
    for it in range(iters):
        theta = rng.normal(mu, sigma, size=(pop, K, 3))
        if best_theta is not None:
            theta[0] = best_theta                              # elitism
        theta = np.clip(theta, lo, hi)
        cost, arr, _, _ = ro.run(theta)
        order = np.argsort(cost)
        elite = theta[order[:n_elite]]
        if cost[order[0]] < best - 1e-6:
            best, best_theta, stale = cost[order[0]], theta[order[0]].copy(), 0
            best_arr = {k: v[order[0]] for k, v in arr.items()}
        else:
            stale += 1
        mu = 0.7 * elite.mean(0) + 0.3 * mu
        sigma = 0.7 * elite.std(0) + 0.3 * sigma
        sigma = np.maximum(sigma, [0.3, 1.5 * DEG, 0.25])
        log(f"  iter {it:2d}  best {best:9.2f}  arrive t={best_arr['t']:5.1f}s  pos {best_arr['pos']:.2f} m  "
            f"hdg {best_arr['head']:.1f} deg  spd {best_arr['speed']:.2f} m/s  contact {best_arr['contact']:.3f}")
        if stale >= patience:
            break
    return best_theta, best, best_arr, mu, sigma


def within(arr, tol):
    return arr['pos'] <= tol['pos'] and arr['head'] <= tol['head'] and arr['speed'] <= tol['speed'] and arr['contact'] <= tol['contact']


def plan(scen, args, log=print):
    rng = np.random.default_rng(args.seed)
    tol = dict(pos=args.tol_pos, head=args.tol_head, speed=args.tol_speed, contact=args.tol_contact)
    K, tmax = args.segments, args.tmax
    mu = sigma = None
    best_theta = None
    for round_ in range(args.max_rounds):
        log(f"\n=== search round {round_ + 1}: K={K} segments, horizon {tmax:.0f} s, dt={args.plan_dt}, pop={args.pop} ===")
        ro = Rollout(scen, args.wind[0], args.wind[1], args.start, args.goal, args.plan_dt, tmax, hull_spacing=0.6)
        if best_theta is not None and best_theta.shape[0] < K:      # warm start: append neutral segments
            pad = np.tile(np.array([6.0, 0.0, 0.0]), (K - best_theta.shape[0], 1))
            mu = np.vstack([best_theta, pad]); sigma = np.tile(np.array([3.0, 12 * DEG, 1.2]), (K, 1))
        theta, cost, arr, mu, sigma = cem(ro, K, args.pop, args.iters, mu, sigma, rng, log)
        log("  refining ...")
        for _ in range(2):
            theta2, cost2, arr2, mu, sigma = cem(ro, K, args.pop, max(8, args.iters // 3), theta, sigma * 0.5, rng, log, patience=6)
            if cost2 < cost:
                theta, cost, arr = theta2, cost2, arr2
        best_theta = theta
        # validation at the render time step with the full hull sampling
        fine = Rollout(scen, args.wind[0], args.wind[1], args.start, args.goal, args.render_dt, tmax, hull_spacing=0.3)
        fcost, farr, hist, sched = fine.run(theta[None], record=True)
        farr = {k: v[0] for k, v in farr.items()}
        log(f"  validation at dt={args.render_dt}: arrive t={farr['t']:.1f}s pos {farr['pos']:.2f} m hdg {farr['head']:.1f} deg "
            f"spd {farr['speed']:.2f} m/s contact {farr['contact']:.3f}")
        if within(farr, tol):
            log("  tolerance met.")
            return theta, farr, hist[:, :, 0], sched, fine
        if round_ + 1 < args.max_rounds:
            log("  tolerance NOT met; widening the search.")
            K += 2; tmax += 30.0
            mu = sigma = None
    log("  giving up after max rounds; returning the best plan found (see numbers above).")
    return theta, farr, hist[:, :, 0], sched, fine


# ----------------------------------------------------------------------------- narrative / export
def narrative(sched, t_end):
    dur, tgt, thr = (x[0] for x in sched)
    lines, t = [], 0.0
    for i, (d, g, h) in enumerate(zip(dur, tgt, thr)):
        if t >= t_end:
            break
        rpm = int(rpm_setpoint(np.array([h]))[0])
        eng = 'neutral' if rpm == 0 else f"{abs(rpm)} rpm {'ahead' if rpm > 0 else 'astern'}"
        rd = g / DEG
        wheel = 'wheel amidships' if abs(rd) < 2 else f"wheel {abs(rd):.0f}° to {'starboard' if rd > 0 else 'port'}"
        last = i == len(dur) - 1
        end = t_end if last else min(t + d, t_end)
        lines.append(f"{t:5.1f}–{end:5.1f} s : {eng:18s} {wheel}" + ("   (held to arrival)" if last else ''))
        t += d
    return lines


def export_plan(path, args, sched, arr):
    dur, tgt, thr = (x[0] for x in sched)
    json.dump(dict(scenario=args.scenario, wind_kn=args.wind[0], wind_from_deg=args.wind[1],
                   start=args.start, goal=args.goal,
                   arrival=dict(t=float(arr['t']), pos_err_m=float(arr['pos']), head_err_deg=float(arr['head']),
                                speed_ms=float(arr['speed']), contact=float(arr['contact'])),
                   segments=[dict(duration_s=float(d), rudder_deg=float(g / DEG), throttle_index=int(h),
                                  rpm=int(rpm_setpoint(np.array([h]))[0])) for d, g, h in zip(dur, tgt, thr)]),
              open(path, 'w'), indent=1)


# ----------------------------------------------------------------------------- movie
def render(scen, hist, t_end, args, ro):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as MPoly
    from matplotlib import animation

    dt = args.render_dt
    n_end = int(min(len(hist), np.ceil((t_end + 3.0) / dt)))     # linger 3 s after arrival
    stride = max(1, int(round(args.speed / (args.fps * dt))))
    frames = list(range(0, n_end, stride))

    W, Hm = scen['widthM'], scen['heightM']
    fig, ax = plt.subplots(figsize=(10, 10 * Hm / W if W >= Hm else 10), dpi=args.dpi)
    ax.set_facecolor('#c9bb8e' if scen['water'] else '#6f9cb8')
    for poly in scen['water']:
        ax.add_patch(MPoly(poly, closed=True, fc='#6f9cb8', ec='#2a5070', lw=1))
    for poly in scen['docks']:
        ax.add_patch(MPoly(poly, closed=True, fc='#7a5a3c', ec='#3a2a1a', lw=1))
    # goal ghost
    gE, gN, gpsi = args.goal[0], args.goal[1], args.goal[2] * DEG
    ax.add_patch(MPoly(body_to_world(P['outline'], gE, gN, gpsi), closed=True, fc='none', ec='#ffd400', lw=1.5, ls='--'))
    ax.set_xlim(0, W); ax.set_ylim(0, Hm); ax.set_aspect('equal'); ax.set_xlabel('E [m]'); ax.set_ylabel('N [m]')
    # wind arrow (free stream)
    wd = args.wind[1] * DEG
    ax.annotate('', xy=(W - 8 - 6 * np.sin(wd), Hm - 8 - 6 * np.cos(wd)), xytext=(W - 8, Hm - 8),
                arrowprops=dict(arrowstyle='-|>', lw=2.5, color='white'))
    ax.text(W - 8, Hm - 16, f"{args.wind[0]:.0f} kn from {args.wind[1]:.0f}°", color='white', ha='center', fontsize=9)

    trail, = ax.plot([], [], color='white', lw=0.8, alpha=0.7)
    boat = MPoly(P['outline'], closed=True, fc='#f4f0e6', ec='#222', lw=1.2); ax.add_patch(boat)
    rudder, = ax.plot([], [], color='#d33', lw=2.5)
    candle = MPoly(np.zeros((4, 2)), closed=True, fc='#28b446', ec='none', alpha=0.9); ax.add_patch(candle)
    wash, = ax.plot([], [], color='white', lw=2, alpha=0.6)
    hud = ax.text(0.01, 0.99, '', transform=ax.transAxes, va='top', ha='left', fontsize=9, family='monospace',
                  bbox=dict(fc='#f4f0e6', ec='#9a8b6c', alpha=0.9))
    ax.set_title(f"Docking plan — {os.path.basename(args.scenario)}", fontsize=11)

    def frame(k):
        s = hist[k]
        E, N, psi, u, v, r, delta, n = s
        boat.set_xy(body_to_world(P['outline'], E, N, psi))
        trail.set_data(hist[:k + 1:5, 0], hist[:k + 1:5, 1])
        rx = P['rudder']['x']
        rud_pts = np.array([[rx, 0], [rx - 0.9 * np.cos(delta), 0.9 * np.sin(delta)]])
        rw = body_to_world(rud_pts, E, N, psi); rudder.set_data(rw[:, 0], rw[:, 1])
        rpm = float(shaft_to_engine(np.array([n]))[0])
        f = rpm / P['engine']['ahead'][-1]
        L = 4.0 * abs(f)
        c = np.array([[-0.3, 0], [0.3, 0], [0.3, L if f > 0 else -L], [-0.3, L if f > 0 else -L]])[:, ::-1]  # (x fwd, y stbd)
        candle.set_xy(body_to_world(c, E, N, psi)); candle.set_facecolor('#28b446' if f >= 0 else '#d22828')
        T, _ = prop_thrust(np.array([u]), np.array([n]))
        if abs(T[0]) > 50:
            Lw = min(6, abs(T[0]) / 400) * (1 if T[0] > 0 else -1)
            wp = body_to_world(np.array([[P['prop']['x'], 0], [P['prop']['x'] - Lw, 0]]), E, N, psi)
            wash.set_data(wp[:, 0], wp[:, 1])
        else:
            wash.set_data([], [])
        hdg = (psi / DEG) % 360
        sog = np.hypot(u, v) / KN
        rd = delta / DEG
        mult, _ = sample_shelter(scen['shelter'], np.array([E]), np.array([N]), args.wind[1])
        hud.set_text(f"t {k * dt:5.1f} s   SOG {sog:4.1f} kn   HDG {hdg:03.0f}°\n"
                     f"rudder {abs(rd):2.0f}° {'stbd' if rd > 1 else 'port' if rd < -1 else '    '}   "
                     f"{abs(rpm):4.0f} rpm {'ahead' if rpm > 20 else 'astern' if rpm < -20 else 'neutral'}\n"
                     f"local wind x{mult[0]:.2f}" + ("   ARRIVED" if k * dt >= t_end else ''))
        return boat, trail, rudder, candle, wash, hud

    ani = animation.FuncAnimation(fig, frame, frames=frames, blit=False)
    out = args.out
    try:
        if out.lower().endswith('.gif'):
            ani.save(out, writer=animation.PillowWriter(fps=args.fps))
        else:
            ani.save(out, writer=animation.FFMpegWriter(fps=args.fps, bitrate=2500))
    except Exception as e:                                        # ffmpeg missing -> gif fallback
        alt = os.path.splitext(out)[0] + '.gif'
        print(f"ffmpeg failed ({e}); writing {alt} instead")
        ani.save(alt, writer=animation.PillowWriter(fps=args.fps)); out = alt
    plt.close(fig)
    return out


def body_to_world(pts, E, N, psi):
    pts = np.asarray(pts, dtype=float)
    sp, cp = np.sin(psi), np.cos(psi)
    return np.column_stack([E + pts[:, 0] * sp + pts[:, 1] * cp, N + pts[:, 0] * cp - pts[:, 1] * sp])


# ----------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('scenario')
    ap.add_argument('--wind', nargs=2, type=float, metavar=('KN', 'FROM_DEG'), required=True)
    ap.add_argument('--start', nargs=3, type=float, metavar=('E', 'N', 'HDG'), required=True)
    ap.add_argument('--goal', nargs=3, type=float, metavar=('E', 'N', 'HDG'), required=True)
    ap.add_argument('--out', default='docking.mp4')
    ap.add_argument('--segments', type=int, default=6)
    ap.add_argument('--pop', type=int, default=160)
    ap.add_argument('--iters', type=int, default=30)
    ap.add_argument('--tmax', type=float, default=90.0)
    ap.add_argument('--max-rounds', type=int, default=3)
    ap.add_argument('--plan-dt', type=float, default=0.1)
    ap.add_argument('--render-dt', type=float, default=0.025)
    ap.add_argument('--tol-pos', type=float, default=0.6, help='m')
    ap.add_argument('--tol-head', type=float, default=6.0, help='deg')
    ap.add_argument('--tol-speed', type=float, default=0.3, help='m/s')
    ap.add_argument('--tol-contact', type=float, default=0.05, help='integrated penetration m*s')
    ap.add_argument('--fps', type=int, default=20)
    ap.add_argument('--speed', type=float, default=1.0, help='playback speed multiplier')
    ap.add_argument('--dpi', type=int, default=90)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--no-movie', action='store_true')
    ap.add_argument('--replay', metavar='PLAN_JSON', help='skip planning; simulate and render this plan (edit it by hand to try variations)')
    args = ap.parse_args()

    scen = load_scenario(args.scenario)
    print(f"scenario {args.scenario}: {len(scen['docks'])} land polygons, {len(scen['water'])} water polygons, "
          f"shelter field {'present' if scen['shelter'] else 'absent'}")
    t0 = time.time()
    if args.replay:
        pj = json.load(open(args.replay))
        theta = np.array([[sg['duration_s'], sg['rudder_deg'] * DEG, sg['throttle_index']] for sg in pj['segments']])[None]
        fine = Rollout(scen, args.wind[0], args.wind[1], args.start, args.goal, args.render_dt, args.tmax, hull_spacing=0.3)
        _, arr, hist, sched = fine.run(theta, record=True)
        arr = {k: v[0] for k, v in arr.items()}; hist = hist[:, :, 0]
        print(f"replaying {args.replay}")
    else:
        theta, arr, hist, sched, fine = plan(scen, args)
        print(f"\nplanning took {time.time() - t0:.0f} s")
    t_end = float(arr['t'])
    print("\nManoeuvre:")
    for line in narrative(sched, t_end):
        print("  " + line)
    print(f"  arrive at t={t_end:.1f} s: {arr['pos']:.2f} m from the mark, {arr['head']:.1f}° off heading, "
          f"{arr['speed'] / KN:.1f} kn, contact {arr['contact']:.3f}")
    plan_path = os.path.splitext(args.out)[0] + '.plan.json'
    export_plan(plan_path, args, sched, arr); print(f"  plan written to {plan_path}")
    if not args.no_movie:
        t1 = time.time()
        out = render(scen, hist, t_end, args, fine)
        print(f"  movie written to {out} ({time.time() - t1:.0f} s)")


if __name__ == '__main__':
    main()
