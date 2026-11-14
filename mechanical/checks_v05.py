"""v0.5 checks on the print geometry (same methods as explore_v03): static overlaps incl. bolts/nuts, tilt and
pan sweeps, hard stop, field of view, boresight, flexure/fastener clearances. Writes checks_v05.json."""
import itertools
import json
import math
import os

import numpy as np
import trimesh

import explore_v03 as e
import gimbal_print as gp
import gimbal_v03 as g

HERE = os.path.dirname(os.path.abspath(__file__))
# pairs that touch by design (screw in a threaded/insert hole or a horn modelled as a solid disc)
EXPECTED = {("pan_horn", "fasteners_pan"), ("horn_coupler", "pan_horn"), ("pan_servo", "pan_horn"), ("turntable_yoke", "pan_horn"), ("tilt_servo", "tilt_horn"), ("pi5_stack", "fasteners_base"), ("lower_deck", "fasteners_base"), ("top_deck", "fasteners_base"),
            ("bearing_mount", "fasteners_base"), ("horn_coupler", "fasteners_pan"), ("tilt_cradle", "fasteners_tilt"),
            ("turntable_yoke", "tilt_cradle"), ("camera", "fasteners_tilt"),   # self-tapping threads / snap-pivot grip (by design)
            ("turntable_yoke", "fasteners_pan"),
            ("tilt_cradle", "tilt_horn"),   # v0.7: crush ribs of the horn pockets (0.2 mm by design)
            ("tilt_cradle", "camera")}      # v0.7 mount E: 6 crush bumps press 0.1 mm on the board back (by design)   # unsplit yoke has no arm-foot holes; checked on the split parts below


def main():
    gp.configure()
    parts, ko, routes = g.build(export=False)
    M = {n: e.mesh(n, v[0]) for n, v in parts.items()}
    K = {n: e.mesh(n, v) for n, v in ko.items()}
    res = {"static": [], "notes": []}
    allm = {**M, **K}
    for a, b in itertools.combinations(allm, 2):
        if a.startswith("keepout") and b.startswith("keepout"):
            continue
        if (a, b) in EXPECTED or (b, a) in EXPECTED:
            continue
        v = e.overlap(allm[a], allm[b])
        if v > 0.5:
            res["static"].append(f"{a} x {b} {v:.1f} mm3")
    yoke, a, b, c = g.turntable_yoke()
    tt, arm_s, arm_i = gp.split_yoke(yoke, a, b, c)
    for n, sh in (("turntable", tt), ("arm_servo", arm_s), ("arm_idle", arm_i)):
        if sh is None:   # one-piece option has no bottom pillar bolts: check against the top bolts only
            top = None
            for x, y in g.PILLARS:
                f = g.m3(x, y, 0.0, up=False, nut_z=-g.DECK_T - 5.0, nut_rot=g.pillar_slot_dir(x, y))
                top = f if top is None else top.union(f)
            v = e.overlap(e.mesh(n, gp.lower_deck_with_pillars()), e.mesh("top_bolts", top))
            if v > 0.5:
                res["static"].append(f"{n} x top pillar bolts {v:.1f} mm3")
            continue
        v = e.overlap(e.mesh(n, sh), M["fasteners_pan"])
        if n == "turntable" and g.STOP != "axial":   # 4 self-tapping M3 in d2.6 pilots (v0.8.1: through + nut)
            v -= 4 * 4.0 * math.pi * (1.5 ** 2 - g.PILOT_R ** 2)
        if v > 0.5:
            res["static"].append(f"{n} x fasteners_pan {v:.1f} mm3")
    mt = trimesh.boolean.union([M[n] for n in ["tilt_horn", "tilt_cradle", "camera", "fasteners_tilt"] + (["eye_plate"] if "eye_plate" in M else [])])
    fixed = trimesh.boolean.union([M["turntable_yoke"], M["tilt_servo"], K["keepout tilt servo plugs"]])
    # baseline at 0 deg = spline inside the horn (by design); report what rotation adds on top of it
    base0 = e.overlap(mt, fixed)
    res["tilt_overlap_max_mm3"] = round(max(e.overlap(e.rot(mt, d, [0, 1, 0], [0, 0, g.H_TILT]), fixed) for d in range(-90, 91, 10)) - base0, 2)
    mp = trimesh.boolean.union([M[n] for n in ["turntable_yoke", "tilt_servo", "tilt_cradle", "camera", "horn_coupler",
                                               "fasteners_pan", "fasteners_tilt"] if n in M] + [K["keepout tilt servo plugs"]])
    mb = trimesh.boolean.union([M[n] for n in ["top_deck", "bearing_mount", "lower_deck", "fasteners_base", "pan_cage"] if n in M]
                               + [K[k] for k in K if "tilt" not in k])
    res["pan_overlap_max_mm3"] = round(max(e.overlap(e.rot(mp, s * d, [0, 0, 1], [0, 0, 0]), mb)
                                           for d in (15, 45, 90, 135, 150) + (() if g.STOP == "axial" else (180,)) for s in (-1, 1)), 2)
    y, bm = M["turntable_yoke"], M.get("bearing_mount")
    if g.STOP == "axial":
        # v0.8: fixed ribbon (riser, radial leg, first bit up the axis) against everything that turns with pan,
        # 5 deg steps, mesh point test; also against the fixed parts once
        rib = e.mesh("fpc_fixed", g.fpc_fixed())
        pts = rib.sample(20000)
        free = [d for d in range(-180, 181, 5) if not e.rot(mp, d, [0, 0, 1], [0, 0, 0]).contains(pts).any()]
        res["pan_cable_free_deg"] = [min(free), max(free)]
        res["cable_vs_fixed_mm3"] = round(e.overlap(rib, trimesh.boolean.union([M[n] for n in ("pan_cage", "pan_servo", "top_deck", "pan_horn")])), 2)
    res["stop_deg"] = None if g.STOP in ("none", "axial") else next((d for d in range(156, 181, 2) if e.overlap(e.rot(y, d, [0, 0, 1], [0, 0, 0]), bm) > 0.05), None)
    if "horn_coupler" in M:   # radial float of the flexure (bearing version only)
        hub = M["horn_coupler"]
        for dx, dy in ((0.4, 0), (-0.4, 0), (0, 0.4), (0, -0.4)):
            h = hub.copy(); h.apply_translation([dx, dy, 0])
            v = e.overlap(h, trimesh.boolean.union([M["bearing_mount"], M["bearing_6806"]]))
            if v > 0.05:
                res["notes"].append(f"rigid hub offset {dx},{dy}: touches mount/bearing {v:.2f} mm3")
    selfm = trimesh.util.concatenate([M[n] for n in ["turntable_yoke", "tilt_servo", "fasteners_pan"]])
    basem = trimesh.util.concatenate([M[n] for n in e.BASE + ["pi5_stack", "lower_deck"] if n in M])
    res["fov_self"] = {t: round(e.fov_blocked(selfm, t), 4) for t in (-30, 0, 30)}
    res["fov_base"] = {f"{p},{t}": round(e.fov_blocked(basem, t, p), 4) for p in (0, 90, 180) for t in (-30, 0)}
    # parts that tilt with the camera (eye plate, cradle), posed at the same tilt as the rays
    tg = trimesh.util.concatenate([M[n] for n in ("eye_plate", "tilt_cradle") if n in M])
    res["fov_tilt_group"] = {t: round(e.fov_blocked(e.rot(tg, -t, [0, 1, 0], [0, 0, g.H_TILT]), t), 4) for t in (-30, 0, 30)}
    # v0.5.5: the Wide module's real diagonal is about 120 deg; check the tilting parts against a 60 deg cone too
    o = np.array([g.ENTRANCE_PUPIL_X, 0, g.H_TILT])
    th, ph = np.meshgrid(np.radians(np.linspace(0, 60, 50)), np.radians(np.linspace(0, 360, 160)))
    dirs = np.stack([np.cos(th), np.sin(th) * np.cos(ph), np.sin(th) * np.sin(ph)], -1).reshape(-1, 3)
    res["cradle_in_60deg_cone"] = round(float(tg.ray.intersects_any(np.repeat(o[None], len(dirs), 0), dirs).mean()), 4)
    import metrics_v04
    res["boresight_down_deg"] = metrics_v04.boresight_limit(selfm, basem)
    vols = {n: round(M[n].volume / 1000, 2) for n in ("horn_coupler", "turntable_yoke", "bearing_mount", "pan_cage", "top_deck", "lower_deck", "tilt_cradle") if n in M}
    res["volumes_cm3"] = vols
    json.dump(res, open(os.path.join(HERE, "checks_v05.json"), "w"), indent=1)
    for k, v in res.items():
        print(k, v)


if __name__ == "__main__":
    main()
