"""Second-pass review of the shortlisted variants: boresight clearance and CoM height."""
import json, math, sys
import numpy as np, trimesh
import explore_v03 as e
import gimbal_v03 as g

def boresight_limit(selfm, basem):
    """Lowest tilt (deg) at which the boresight ray from the lens still clears all parts, worst over pan 0/90/180."""
    worst = -90
    for psi in (0, 90, 180):
        lim = -90
        for th in range(0, -91, -2):
            t, p = math.radians(th), -math.radians(psi)
            d = np.array([math.cos(t), 0, math.sin(t)])
            lens = np.array([g.CAM_BACK_X + g.CAM_PCB + g.CAM_LENS_H - 2.0, 0, 0])
            o = np.array([lens[0] * math.cos(t), 0, lens[0] * math.sin(t) + g.H_TILT])
            P = np.array([[math.cos(p), -math.sin(p), 0], [math.sin(p), math.cos(p), 0], [0, 0, 1]])
            d, o = P @ d, P @ o
            hit = False
            for m in ((selfm, basem) if psi == 0 else (basem,)):   # self turns with pan: only check it in its own frame
                locs, idx, _ = m.ray.intersects_location([o + d * 0.2], [d], multiple_hits=False)
                if len(idx) and np.linalg.norm(locs[0] - o) < 1500:
                    hit = True
            if hit:
                lim = th + 2
                break
        worst = max(worst, lim)
    return worst

rows = json.load(open("explore_v03.json"))
short = [r for r in rows if not r["fail"] and r["wrap_r"] in (36.0, 40.0)]
out = []
for r in short:
    g.configure(r["wrap_r"], r["stop"], r["orient"]); g.H_TILT += r["h_extra"]
    cx, cy, _ = g.tilt_conn_point(); g.HOLE_BUS = (math.degrees(math.atan2(cy, cx)), g.R_MID)
    parts, ko, routes = g.build(export=False)
    M = {n: e.mesh(n, v[0]) for n, v in parts.items()}
    selfm = trimesh.util.concatenate([M[n] for n in ["turntable_yoke", "tilt_servo", "bearing_625"]])
    basem = trimesh.util.concatenate([M[n] for n in e.BASE + ["pi5_stack", "lower_deck"]])
    r["boresight_down_deg"] = boresight_limit(selfm, basem)
    # CoM height of the whole upper assembly + base above the platform interface (lower-deck bottom)
    tot, cz = 0.0, 0.0
    for n, m in M.items():
        mg = e.MASS.get(n, {"pan_servo": 55.0, "pi5_stack": 110.0, "bearing_6806": 26.0, "pan_horn": 5.0}.get(n, m.volume / 1000 * 1.24 * 0.40))
        tot += mg; cz += mg * m.center_mass[2]
    r["sys_mass_g"] = round(tot); r["sys_com_z_above_platform"] = round(cz / tot - (g.LOWER_TOP - g.DECK_T), 1)
    out.append(r)
    print(r["wrap_r"], r["stop"], r["orient"], r["h_extra"], "H", r["H_TILT"], "bore", r["boresight_down_deg"], "sysCoMz", r["sys_com_z_above_platform"], "Iz %.2e" % r["Iz"], "top", r["top_z"], flush=True)
json.dump(out, open("recheck_v03.json", "w"), indent=1)
