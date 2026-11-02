"""Print-ready export of the RCWS gimbal (v0.3 geometry, chosen variant) for CATIA and FDM printing.

* The one-piece turntable+yoke of v0.3 is split into three prints so nothing needs support:
    turntable (printed upside down), arm_servo (printed standing on its foot) and arm_idle (printed lying on its flat outer face),
    joined by 2x M3 screws per arm into heat-set inserts in the turntable.
* Every printed part is written twice:
    step/<part>.step      solid in assembly coordinates (CATIA: open each, or open the assembly STEP)
    stl/<part>_print.stl  re-oriented for the build plate, lowest point at z=0
* Printability report: footprint vs bed, height, and downward-facing area steeper than 45 deg above
  the plate (= support needed), per part.
* A fit-test coupon for the 6806 seats (tube and bore) at four allowances.
"""
import json
import math
import os
import tempfile

import cadquery as cq
import numpy as np
import trimesh

import gimbal_v03 as g

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "print_v04")
BED = (220.0, 220.0, 250.0)          # typical FDM bed [VERIFY your printer]
VARIANT = dict(wrap_r=36.0, stop="groove", tilt_orient="down")


def configure():
    g.configure(**VARIANT)
    # bus-cable hole off the arm/gusset footprint (connectors face down at angle ~90 deg)
    g.HOLE_BUS = (135.0, g.R_MID)


def split_yoke(yoke, y_srv_in, y_srv_out, y_idle_in):
    z_top = g.DISC_Z0 + g.DISC_T
    big = 500.0
    turntable = yoke.intersect(g.box(-big, big, -big, big, -big, z_top))
    upper = yoke.cut(g.box(-big, big, -big, big, -big, z_top))
    arm_s = upper.intersect(g.box(-big, big, 0, big, z_top, big))
    arm_i = upper.intersect(g.box(-big, big, -big, 0, z_top, big))
    # bolt-down feet on the inner side of each arm, 2x M3 clearance; inserts in the turntable
    foot_t, foot_w = 6.0, 8.0          # 6 mm foot + 4 mm into the turntable insert = M3x10
    y_idle_out = y_idle_in - g.ARM_T_IDLE
    feet = {"s": (y_srv_in - foot_w, y_srv_out), "i": (y_idle_out, y_idle_in + foot_w)}
    bolts = []
    for key, (y0, y1) in feet.items():
        f = g.box(-g.ARM_W / 2, g.ARM_W / 2, y0, y1, z_top, z_top + foot_t)
        yb = (y_srv_in - foot_w / 2) if key == "s" else (y_idle_in + foot_w / 2)
        for xb in (-9.0, 9.0):
            f = f.cut(g.cyl_z(1.7, z_top - 1, z_top + foot_t + 1, xb, yb))
            f = f.cut(g.cyl_z(3.1, z_top + foot_t - 0.01, z_top + 30, xb, yb))   # head clearance through gussets
            bolts.append((xb, yb))
        if key == "s":
            arm_s = arm_s.union(f).cut(g.cyl_z(3.1, z_top + foot_t - 0.01, z_top + 30, -9, yb)).cut(
                g.cyl_z(3.1, z_top + foot_t - 0.01, z_top + 30, 9, yb))
        else:
            arm_i = arm_i.union(f).cut(g.cyl_z(3.1, z_top + foot_t - 0.01, z_top + 30, -9, yb)).cut(
                g.cyl_z(3.1, z_top + foot_t - 0.01, z_top + 30, 9, yb))
    for xb, yb in bolts:   # M3 heat-set insert (d4.2 x 4.5) from the top face of the turntable
        turntable = turntable.cut(g.cyl_z(2.1, z_top - 4.5, z_top + 1, xb, yb))
    return turntable, arm_s, arm_i


def fit_coupon():
    """6806 seat test at four allowances, left to right: shaft -0.10/-0.05/+0.05/+0.15, bore +0.05/+0.10/+0.20/+0.30 (radius)."""
    out = None
    for i, a in enumerate((-0.10, -0.05, 0.05, 0.15)):
        x = i * 50.0
        tube = g.ring_z(13.5, 15.0 + a, 0, 7.5, x, 0)                    # 6806 inner race (d30)
        base = g.cyl_z(15.0 + 6, 0, 2, x, 0).cut(g.cyl_z(13.5, -1, 3, x, 0))
        b = a + 0.15
        bore = g.box(x - 23, x + 23, 26, 72, 0, 7.5).cut(g.cyl_z(21.0 + b, 0.8, 9, x, 49))   # 6806 OD42, floor 0.8
        piece = tube.union(base).union(bore)
        out = piece if out is None else out.union(piece)
    return out


# build-plate orientation per part: (axis, angle) rotations applied in order
ORIENT = {
    "top_deck": [],
    "lower_deck": [],
    "pillars": [],
    "pan_cage": [],                          # floor on the plate, ears full height
    "bearing_mount": [],
    "horn_coupler": [],
    "turntable": [((1, 0, 0), 180)],         # top face on the plate: housing, lip and groove face up
    "arm_servo": [],                         # standing on its foot: saddle walls vertical, no support (brim)
    "arm_idle": [((1, 0, 0), 90)],           # outer (-Y) face on the plate
    "tilt_cradle": [((0, 1, 0), -90)],       # back plate on the plate, camera standoffs up
    "fit_coupon": [],
    "eye_plate": [((0, 1, 0), -90)],         # back face on the plate, rim and engraving up
}


def to_mesh(shape, tmp, name):
    p = os.path.join(tmp, name + ".stl")
    cq.exporters.export(shape, p, tolerance=0.03, angularTolerance=0.08)
    return trimesh.load(p)


def printability(m):
    lo, hi = m.bounds
    fz = m.face_normals[:, 2]
    cz = m.triangles_center[:, 2]
    over = (fz < -math.cos(math.radians(45))) & (cz > lo[2] + 0.3)
    return {"size_mm": [round(float(v), 1) for v in hi - lo], "fits_bed": bool(all(hi - lo <= np.array(BED))),
            "support_area_mm2": round(float(m.area_faces[over].sum()), 1)}


def main():
    configure()
    parts, keepouts, routes = g.build(export=False)
    yoke, y_srv_in, y_srv_out, y_idle_in = g.turntable_yoke()
    tt, arm_s, arm_i = split_yoke(yoke, y_srv_in, y_srv_out, y_idle_in)
    printed = {n: v[0] for n, v in parts.items() if v[2] and n != "turntable_yoke"}
    printed.update(turntable=tt, arm_servo=arm_s, arm_idle=arm_i)
    bought = {n: v[0] for n, v in parts.items() if not v[2]}

    for d in ("step", "stl", "step_reference"):
        os.makedirs(os.path.join(OUT, d), exist_ok=True)
    tmp = tempfile.mkdtemp()
    report = {}
    assy = cq.Assembly(name="RCWS_gimbal_v04")
    colors = {n: v[1] for n, v in parts.items()}
    colors.update(turntable=(0.20, 0.45, 0.85), arm_servo=(0.25, 0.50, 0.90), arm_idle=(0.25, 0.50, 0.90))
    for name, shape in printed.items():
        assert shape.val().isValid(), name
        cq.exporters.export(shape, os.path.join(OUT, "step", f"{name}.step"))
        assy.add(shape, name=name, color=cq.Color(*colors.get(name, (0.7, 0.7, 0.7))))
        s = shape
        for axis, ang in ORIENT[name]:
            s = s.rotate((0, 0, 0), axis, ang)
        m = to_mesh(s, tmp, name)
        m.apply_translation(-m.bounds[0] * [0, 0, 1] - [m.bounds[0][0] + (m.bounds[1][0] - m.bounds[0][0]) / 2,
                                                       m.bounds[0][1] + (m.bounds[1][1] - m.bounds[0][1]) / 2, 0])
        m.export(os.path.join(OUT, "stl", f"{name}_print.stl"))
        vol = shape.val().Volume() / 1000
        report[name] = {**printability(m), "volume_cm3": round(vol, 1), "pla_g_at_40pct": round(vol * 1.24 * 0.4, 1)}
    for name, shape in bought.items():
        cq.exporters.export(shape, os.path.join(OUT, "step_reference", f"{name}.step"))
        assy.add(shape, name=name, color=cq.Color(*colors[name]))
    assy.export(os.path.join(OUT, "RCWS_gimbal_v04_assembly.step"))
    c = fit_coupon()
    cq.exporters.export(c, os.path.join(OUT, "step", "fit_coupon.step"))
    m = to_mesh(c, tmp, "fit_coupon")
    m.apply_translation(-m.bounds[0])
    m.export(os.path.join(OUT, "stl", "fit_coupon_print.stl"))
    report["fit_coupon"] = {**printability(m), "volume_cm3": round(c.val().Volume() / 1000, 1)}
    json.dump({"variant": VARIANT, "H_TILT": g.H_TILT, "FIT_SHAFT": g.FIT_SHAFT, "FIT_BORE": g.FIT_BORE,
               "bed": BED, "parts": report}, open(os.path.join(OUT, "print_report.json"), "w"), indent=1)
    for k, v in report.items():
        print(f"{k:14s} {v}")
    return printed, parts, keepouts


if __name__ == "__main__":
    main()
