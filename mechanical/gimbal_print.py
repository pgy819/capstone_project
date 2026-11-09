"""Print-ready export of the RCWS gimbal (v0.3 geometry, chosen variant) for CATIA and FDM printing.

* The one-piece turntable+yoke of v0.3 is split into three prints so nothing needs support:
    turntable (printed upside down), arm_servo (printed standing on its foot) and arm_idle (printed lying on its flat outer face),
    joined by 2x M3x10 per arm down into nuts held in radial side slots in the turntable disc (v0.5.1).
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
OUT = os.path.join(HERE, "print_v05")
BED = (220.0, 220.0, 250.0)          # typical FDM bed [VERIFY your printer]
VARIANT = dict(wrap_r=36.0, stop="groove", tilt_orient="down")
H_TILT_KEEP = 79.0


def configure():
    g.configure(**VARIANT)
    # bus-cable hole off the arm/gusset footprint (connectors face down at angle ~90 deg)
    g.HOLE_BUS = (135.0, g.R_MID)
    # Tilt-axis height kept at the v0.3 value. Since the servo plugs moved to the face opposite the horn, 63 mm
    # would also fit (lower CoM) but costs 6 deg of down-look and more FOV occlusion; see metrics_v04.json.
    g.H_TILT = max(g.H_TILT, H_TILT_KEEP)


def split_yoke(yoke, y_srv_in, y_srv_out, y_idle_in):
    z_top = g.DISC_Z0 + g.DISC_T
    big = 500.0
    turntable = yoke.intersect(g.box(-big, big, -big, big, -big, z_top))
    upper = yoke.cut(g.box(-big, big, -big, big, -big, z_top))
    arm_s = upper.intersect(g.box(-big, big, 0, big, z_top, big))
    arm_i = upper.intersect(g.box(-big, big, -big, 0, z_top, big))
    # bolt-down feet on the inner side of each arm, 2x M3 clearance; nuts in the turntable side slots
    foot_t = g.ARM_FOOT_T              # 6 mm foot + 4 mm into the disc nut = M3x10
    feet = g.ARM_FEET()
    bolts = g.ARM_BOLTS()
    for key, (y0, y1) in feet.items():
        f = g.box(-g.ARM_W / 2, g.ARM_W / 2, y0, y1, z_top, z_top + foot_t)
        cut = None
        for xb, yb in bolts:
            if (yb > 0) != (key == "s"):
                continue
            c = g.cyl_z(1.7, z_top - 1, z_top + foot_t + 1, xb, yb).union(
                g.cyl_z(3.1, z_top + foot_t - 0.01, z_top + 40, xb, yb))      # shank + head clearance through gussets
            cut = c if cut is None else cut.union(c)
        if key == "s":
            arm_s = arm_s.union(f).clean().cut(cut).clean()
        else:
            arm_i = arm_i.union(f).clean().cut(cut).clean()
    for a in (arm_s, arm_i):
        assert a.val().Volume() > 1000, "arm boolean failed"
    return turntable, arm_s, arm_i


def lower_deck_with_pillars():
    """Option (user, 11-02): lower deck and the four pillars as one print. Pillars grow out of the deck top
    with a 3 mm cone at the root; the bottom screws are dropped (4 fewer M3x10 + nuts), the top keeps the
    v0.5.1 side nut slot for the M3x10 that comes down through the top deck."""
    out = g.lower_deck()
    for x, y in g.PILLARS:
        out = out.cut(g.cyl_z(1.8, g.LOWER_TOP - g.DECK_T - 1, g.LOWER_TOP + 1, x, y))      # old through hole
        col = g.cyl_z(g.PILLAR_R, g.LOWER_TOP - g.DECK_T, -g.DECK_T, x, y)
        root = (cq.Workplane("XY").workplane(offset=g.LOWER_TOP).center(x, y).circle(g.PILLAR_R + 0.8)
                .workplane(offset=3.0).circle(g.PILLAR_R).loft())
        out = out.union(col).union(root)
    out = out.cut(g.pillars(top=False, bottom=False).cut(g.pillars(top=True, bottom=False)))   # slots only
    for x, y in g.PILLARS:
        out = out.cut(g.cyl_z(1.7, -g.DECK_T - 14.0, 1, x, y))
    return out


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


def check_coupon():
    """v0.5 fit checks before the big prints (about 4 g), left to right:
    M3 nut pockets AF 5.6/5.8/6.0 (pick the one the nut presses into by hand),
    flexure slots 0.6/0.7/0.8 mm beside 1.2 mm blades (the blade must move freely after printing)."""
    base = g.box(0, 70, 0, 22, 0, 3)
    for i, af in enumerate((5.6, 5.8, 6.0)):
        base = base.cut(g.cyl_z(1.7, -1, 4, 7 + i * 11.5, 5)).cut(g.hex_z(af, 3 - 2.6, 4, 7 + i * 11.5, 5))
    # flexure slots: three blades 1.2 x 3 x 12 standing on a shared root, each with its own slot width
    fl = g.box(40, 70, 0, 10, 0, 5)
    x = 41.5
    for gap in (0.6, 0.7, 0.8):
        fl = fl.cut(g.box(x, x + gap, 0, 8.5, -1, 6)).cut(g.box(x + gap + 1.2, x + 2 * gap + 1.2, 0, 8.5, -1, 6))
        x += 2 * gap + 1.2 + 6.0
    return base.cut(g.box(40, 70, 0, 10, -1, 4)).union(fl)


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
    "lower_deck_with_pillars": [],           # option: replaces lower_deck + pillars, deck bottom on the plate
    "eye_plate": [((0, 1, 0), -90)],
    "check_coupon": [],
    "tilt_servo_clamp": [((1, 0, 0), 90)],   # servo-contact face on the plate, nut slots open upward         # back face on the plate, rim and engraving up
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
    printed.update(turntable=tt, arm_servo=arm_s, arm_idle=arm_i, lower_deck_with_pillars=lower_deck_with_pillars())
    bought = {n: v[0] for n, v in parts.items() if not v[2]}

    for d in ("step", "stl", "step_reference"):
        os.makedirs(os.path.join(OUT, d), exist_ok=True)
    tmp = tempfile.mkdtemp()
    report = {}
    assy = cq.Assembly(name="gimbal_v05")
    colors = {n: v[1] for n, v in parts.items()}
    colors["lower_deck_with_pillars"] = (0.80, 0.82, 0.85)
    colors.update(turntable=(0.20, 0.45, 0.85), arm_servo=(0.25, 0.50, 0.90), arm_idle=(0.25, 0.50, 0.90))
    for name, shape in printed.items():
        assert shape.val().isValid(), name
        cq.exporters.export(shape, os.path.join(OUT, "step", f"{name}.step"))
        if name != "lower_deck_with_pillars":
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
    assy.export(os.path.join(OUT, "gimbal_v05_assembly.step"))
    c = fit_coupon()
    cq.exporters.export(c, os.path.join(OUT, "step", "fit_coupon.step"))
    m = to_mesh(c, tmp, "fit_coupon")
    m.apply_translation(-m.bounds[0])
    m.export(os.path.join(OUT, "stl", "fit_coupon_print.stl"))
    report["fit_coupon"] = {**printability(m), "volume_cm3": round(c.val().Volume() / 1000, 1)}
    c = check_coupon()
    cq.exporters.export(c, os.path.join(OUT, "step", "check_coupon.step"))
    m = to_mesh(c, tmp, "check_coupon")
    m.apply_translation(-m.bounds[0])
    m.export(os.path.join(OUT, "stl", "check_coupon_print.stl"))
    report["check_coupon"] = {**printability(m), "volume_cm3": round(c.val().Volume() / 1000, 1)}
    json.dump({"variant": VARIANT, "H_TILT": g.H_TILT, "FIT_SHAFT": g.FIT_SHAFT, "FIT_BORE": g.FIT_BORE,
               "bed": BED, "parts": report}, open(os.path.join(OUT, "print_report.json"), "w"), indent=1)
    for k, v in report.items():
        print(f"{k:14s} {v}")
    return printed, parts, keepouts


if __name__ == "__main__":
    main()
