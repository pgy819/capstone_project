"""Metrics of the current v0.4 geometry (same definitions as explore/recheck v0.3)."""
import json, math, os, sys
import numpy as np, trimesh
import explore_v03 as e, gimbal_print as gp, gimbal_v03 as g
sys.argv = sys.argv[:1]
import importlib
def boresight_limit(selfm, basem):
    src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "recheck_v03.py")).read().split("rows = json.load")[0]
    ns = {}; exec(src, ns); return ns["boresight_limit"](selfm, basem)
def run(h_override=None):
    gp.configure()
    if h_override: g.H_TILT = h_override
    parts, ko, routes = g.build(export=False)
    M = {n: e.mesh(n, v[0]) for n, v in parts.items()}
    PAN = ["turntable_yoke", "horn_coupler", "tilt_servo", "tilt_horn", "tilt_cradle", "camera", "eye_plate"]
    MASS = {"tilt_servo": 55.0, "camera": 5.0, "tilt_horn": 5.0}
    tot, C, Iz = 0.0, np.zeros(3), 0.0
    for n in PAN:
        m = M[n]; mg = MASS.get(n, m.volume / 1000 * 1.24 * 0.40); m.density = mg / m.volume
        com = m.center_mass; Iz += m.moment_inertia[2, 2] + mg * (com[0] ** 2 + com[1] ** 2); tot += mg; C += mg * com
    Iz += 12.0 * 19.8 ** 2; tot += 12.0; C /= tot
    selfm = trimesh.util.concatenate([M[n] for n in ["turntable_yoke", "tilt_servo"]])
    basem = trimesh.util.concatenate([M[n] for n in e.BASE + ["pi5_stack", "lower_deck"]])
    stot, cz = 0.0, 0.0
    for n, m in M.items():
        mg = {"tilt_servo": 55.0, "camera": 5.0, "tilt_horn": 5.0, "pan_servo": 55.0, "pi5_stack": 110.0, "bearing_6806": 26.0, "pan_horn": 5.0}.get(n, m.volume / 1000 * 1.24 * 0.40)
        stot += mg; cz += mg * m.center_mass[2]
    return dict(H_TILT=g.H_TILT, mass_g=round(float(tot), 1), com_r=round(float(np.hypot(C[0], C[1])), 1), com_z=round(float(C[2]), 1),
                Iz=float(Iz * 1e-9), top_z=round(float(max(M[n].bounds[1][2] for n in PAN)), 1),
                boresight_down_deg=boresight_limit(selfm, basem),
                fov_self={t: round(e.fov_blocked(selfm, t), 3) for t in (-30, 0, 30)},
                fov_base={f"{p},{t}": round(e.fov_blocked(basem, t, p), 3) for p in (0, 90, 180) for t in (-30, 0)},
                sys_com_z_above_platform=round(cz / stot - (g.LOWER_TOP - g.DECK_T), 1),
                cables={k: round(g.length(v)) for k, v in routes.items()})
if __name__ == "__main__":
    old = run(79.0); new = run()
    for k in new: print(f"{k:26s} {old[k]}  ->  {new[k]}")
    json.dump({"before_H79": old, "now": new}, open("metrics_v04.json", "w"), indent=1)
