"""Checks the numpy port against trajectories produced by the JS physics."""
import json, os, sys, numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from sailsim_physics import P, DEG, initial_state, rk4_step, rpm_setpoint, contact_forces, sample_outline

FIX = json.load(open(os.path.join(os.path.dirname(__file__), 'fixtures', 'trajectory.json')))
hull = sample_outline(P['outline'])

def run_case(c):
    S = initial_state(1, 0, 0, 0)
    S[:, 0] = c['states'][0]
    docks = [np.array(p) for p in c['docks']]; water = [np.array(p) for p in c['water']]
    extra = (lambda st: contact_forces(st, hull, docks, water)[:3]) if (docks or water) else None
    inp = dict(rudder_rate=np.zeros(1), rpm_set=rpm_setpoint(np.array([c['throttle']])),
               wE=c['windE'], wN=c['windN'])
    out = [S[:, 0].copy()]
    for i in range(1, len(c['states']) * FIX['every']):
        S = rk4_step(S, inp, FIX['dt'], P, extra)
        if i % FIX['every'] == 0:
            out.append(S[:, 0].copy())
    return np.array(out)

def test_port():
    worst = {}
    for c in FIX['cases']:
        ref = np.array(c['states'])[:, :]
        got = run_case(c)[:len(ref)]
        # position [m], heading [deg], speeds [m/s], yaw rate [deg/s]
        e_pos = np.hypot(got[:, 0] - ref[:, 0], got[:, 1] - ref[:, 1]).max()
        e_psi = (np.abs((got[:, 2] - ref[:, 2] + np.pi) % (2 * np.pi) - np.pi) / DEG).max()
        e_uv = np.abs(got[:, 3:5] - ref[:, 3:5]).max()
        e_r = (np.abs(got[:, 5] - ref[:, 5]) / DEG).max()
        worst[c['name']] = (e_pos, e_psi, e_uv, e_r)
        assert e_pos < 0.02, f"{c['name']}: position drift {e_pos:.4f} m"
        assert e_psi < 0.1, f"{c['name']}: heading drift {e_psi:.4f} deg"
        assert e_uv < 0.005, f"{c['name']}: speed drift {e_uv:.5f} m/s"
    return worst

if __name__ == '__main__':
    for k, (a, b, c, d) in test_port().items():
        print(f"{k:8s} max |dpos| {a:.2e} m   |dpsi| {b:.2e} deg   |du,dv| {c:.2e} m/s   |dr| {d:.2e} deg/s")
    print('port matches JS')
