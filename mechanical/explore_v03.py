"""Design-space exploration for the v0.3 gimbal: build every variant, check it, score it.

Checks (mesh booleans with manifold3d; the CAD kernel's boolean on rotated cylinders gave false hits):
  static   - any two parts / keep-out volumes overlapping by > 1 mm3
  tilt     - cradle + camera + tilt horn swept -90..90 deg against the turntable and tilt servo
  pan      - rotating group swept to +-160 deg against the base; hard-stop angle found by bisection
  fov      - fraction of the camera's 102 x 67 deg view that hits gimbal parts, at tilt -30/0/+30
  mass     - rotating mass, CoM offset from the pan axis, Iz (PLA at 40 % effective fill)
"""
import itertools
import json
import math
import os
import sys
import tempfile
import time

import cadquery as cq
import numpy as np
import trimesh

import gimbal_v03 as g

TMP = tempfile.mkdtemp()
MASS = {"tilt_servo": 55.0, "camera": 5.0, "bearing_625": 5.0, "tilt_horn": 5.0}
PAN_GROUP = ["turntable_yoke", "horn_coupler", "tilt_servo", "bearing_625", "tilt_horn", "tilt_cradle", "camera"]
TILT_GROUP = ["tilt_horn", "tilt_cradle", "camera"]
BASE = ["top_deck", "bearing_mount", "pillars", "pan_cage", "pan_servo"]


def mesh(name, shape):
    p = os.path.join(TMP, name.replace(" ", "_").replace("/", "_") + ".stl")
    cq.exporters.export(shape, p, tolerance=0.08, angularTolerance=0.15)
    return trimesh.load(p)


def overlap(a, b):
    i = a.intersection(b)
    return 0.0 if i.is_empty else float(i.volume)


def rot(m, deg, axis, point):
    m = m.copy()
    m.apply_transform(trimesh.transformations.rotation_matrix(math.radians(deg), axis, point))
    return m


def fov_blocked(targets, theta_deg, psi_deg=0.0, n_h=41, n_v=27):
    """Fraction of camera rays (102 x 67 deg, pinhole grid) that hit `targets` within 1.5 m.
    The camera is posed at pan psi (right-positive) and tilt theta; targets are in base coordinates."""
    lens = np.array([g.CAM_BACK_X + g.CAM_PCB + g.CAM_LENS_H - 2.0, 0.0, g.H_TILT])   # entrance pupil ~2 mm inside the barrel
    hs = np.tan(np.radians(51.0)) * np.linspace(-1, 1, n_h)
    vs = np.tan(np.radians(33.5)) * np.linspace(-1, 1, n_v)
    d = np.array([[1.0, -h, v] for h in hs for v in vs])          # +y left, image right = -y
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    t = math.radians(theta_deg)                                   # tilt up = rotate about -Y
    R = np.array([[math.cos(t), 0, -math.sin(t)], [0, 1, 0], [math.sin(t), 0, math.cos(t)]])
    d = d @ R.T
    o = R @ (lens - [0, 0, g.H_TILT]) + [0, 0, g.H_TILT]
    p = -math.radians(psi_deg)                                    # pan right = rotate about -Z
    P = np.array([[math.cos(p), -math.sin(p), 0], [math.sin(p), math.cos(p), 0], [0, 0, 1]])
    d = d @ P.T
    o = P @ o
    origins = np.repeat(o[None, :], len(d), axis=0) + d * 0.2
    locs, idx, _ = targets.ray.intersects_location(origins, d, multiple_hits=False)
    hit = np.zeros(len(d), bool)
    if len(idx):
        dist = np.linalg.norm(locs - origins[idx], axis=1)
        hit[idx[dist < 1500]] = True
    return float(hit.mean())


def evaluate(wrap_r, stop, orient, h_extra=0.0):
    g.configure(wrap_r, stop, orient)
    g.H_TILT += h_extra
    cx, cy, _ = g.tilt_conn_point()
    g.HOLE_BUS = (math.degrees(math.atan2(cy, cx)), g.R_MID)
    res = {"wrap_r": wrap_r, "stop": stop, "orient": orient, "h_extra": h_extra, "H_TILT": g.H_TILT, "disc_d": 2 * g.DISC_R,
           "annulus": round(g.ANNULUS, 1), "fail": []}
    if g.ANNULUS < 9.0:
        res["fail"].append(f"cable annulus {g.ANNULUS:.1f} mm < 9 mm")
    parts, ko, routes = g.build(export=False)
    M = {n: mesh(n, v[0]) for n, v in parts.items()}
    K = {n: mesh(n, v) for n, v in ko.items()}
    # static
    allm = {**M, **K}
    stat = []
    for a, b in itertools.combinations(allm, 2):
        if a.startswith("keepout") and b.startswith("keepout"):
            continue
        v = overlap(allm[a], allm[b])
        if v > 1.0:
            stat.append(f"{a} x {b} {v:.0f} mm3")
    res["static"] = stat
    if stat:
        res["fail"].append("static overlap")
    # tilt sweep
    mt = trimesh.boolean.union([M[n] for n in TILT_GROUP])
    fixed = trimesh.boolean.union([M["turntable_yoke"], M["tilt_servo"], K["keepout tilt servo plugs"]])
    tmax = max(overlap(rot(mt, deg, [0, 1, 0], [0, 0, g.H_TILT]), fixed) for deg in range(-90, 91, 15))
    res["tilt_overlap_max"] = round(tmax, 1)
    if tmax > 1.0:
        res["fail"].append("tilt sweep collision")
    # pan sweep and stop
    mp = trimesh.boolean.union([M[n] for n in ["turntable_yoke", "tilt_servo", "tilt_cradle", "camera", "horn_coupler"]]
                               + [K["keepout tilt servo plugs"]])
    mb = trimesh.boolean.union([M[n] for n in ["top_deck", "bearing_mount", "pillars"]] + [K[k] for k in K if "tilt" not in k])
    pmax = max(overlap(rot(mp, s * deg, [0, 0, 1], [0, 0, 0]), mb) for deg in (20, 60, 100, 140, 155) for s in (-1, 1))
    res["pan_overlap_max"] = round(pmax, 1)
    if pmax > 1.0:
        res["fail"].append("pan sweep collision")
    y, bm = M["turntable_yoke"], M["bearing_mount"]
    stop_deg = None
    for deg in range(156, 181, 2):
        if overlap(rot(y, deg, [0, 0, 1], [0, 0, 0]), bm) > 0.05:
            stop_deg = deg
            break
    res["stop_deg"] = stop_deg
    if stop_deg is None:
        res["fail"].append("no hard stop before 180 deg")
    # field of view
    # self = parts that turn with pan (pose-independent in the camera's view); base = fixed parts, depends on psi
    selfm = trimesh.util.concatenate([M[n] for n in ["turntable_yoke", "tilt_servo", "bearing_625", "tilt_cradle", "tilt_horn"]])
    basem = trimesh.util.concatenate([M[n] for n in BASE + ["pi5_stack", "lower_deck"]])
    res["fov_self"] = {t: round(fov_blocked(selfm, t), 3) for t in (-30, 0, 30)}
    res["fov_base"] = {f"{p},{t}": round(fov_blocked(basem, t, p), 3) for p in (0, 90, 180) for t in (-30, 0)}
    # mass properties
    tot, C, Iz = 0.0, np.zeros(3), 0.0
    for n in PAN_GROUP:
        m = M[n]
        mg = MASS.get(n, m.volume / 1000 * 1.24 * 0.40)
        m.density = mg / m.volume
        com = m.center_mass
        Iz += m.moment_inertia[2, 2] + mg * (com[0] ** 2 + com[1] ** 2)
        tot += mg
        C += mg * com
    Iz += 12.0 * 19.8 ** 2                       # 6806 outer race
    tot += 12.0
    C /= tot
    top = max(M[n].bounds[1][2] for n in PAN_GROUP)
    res.update(mass_g=round(float(tot), 1), com_r=round(float(np.hypot(C[0], C[1])), 1), com_z=round(float(C[2]), 1),
               Iz=float(Iz * 1e-9), top_z=round(float(top), 1),
               cables={k: round(g.length(v)) for k, v in routes.items()})
    return res


def score(rows):
    ok = [r for r in rows if not r["fail"]]
    # weights: pan inertia and CoM offset drive servo load/precision; size and height drive Stage-2 load;
    # self-occlusion at level tilt costs search area; base occlusion (worst pan, level tilt) less so.
    keys = {"Iz": 0.30, "com_r": 0.15, "disc_d": 0.10, "top_z": 0.15, "fov_s": 0.15, "fov_b": 0.15}
    for r in rows:
        r["fov_s"] = r["fov_self"][0]
        r["fov_b"] = max(v for k, v in r["fov_base"].items() if k.endswith(",0"))
    lo = {k: min(r[k] for r in ok) for k in keys}
    hi = {k: max(r[k] for r in ok) for k in keys}
    for r in rows:
        r["score"] = None if r["fail"] else round(sum(w * ((r[k] - lo[k]) / (hi[k] - lo[k]) if hi[k] > lo[k] else 0)
                                                         for k, w in keys.items()), 3)
    return sorted(rows, key=lambda r: (r["score"] is None, r["score"] if r["score"] is not None else 9))


if __name__ == "__main__":
    grid = list(itertools.product((32.0, 36.0, 40.0, 44.0), ("rim", "groove"), ("back", "down", "up"), (0.0, 15.0)))
    rows = []
    for i, (w, s, o, h) in enumerate(grid):
        t0 = time.time()
        r = evaluate(w, s, o, h)
        rows.append(r)
        print(f"[{i + 1}/{len(grid)}] R{w:.0f} {s:6s} {o:4s} H{r['H_TILT']:.0f} Iz {r['Iz']:.2e} com {r['com_r']} "
              f"self {r['fov_self']} base {r['fov_base']} stop {r['stop_deg']} fail {r['fail']} ({time.time() - t0:.0f}s)", flush=True)
    rows = score(rows)
    json.dump(rows, open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "explore_v03.json"), "w"), indent=1)
    print("best:", {k: rows[0][k] for k in ("wrap_r", "stop", "orient", "score")})
