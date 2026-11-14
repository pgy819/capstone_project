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
VARIANT = dict(wrap_r=36.0, stop="axial", tilt_orient="down")   # v0.8: cable up the pan axis
H_TILT_KEEP = 79.0


def configure():
    g.configure(**VARIANT)
    # bus-cable hole off the arm/gusset footprint (connectors face down at angle ~90 deg)
    if g.STOP != "none":
        g.HOLE_BUS = (135.0, g.R_MID)
    # Tilt-axis height kept at the v0.3 value. Since the servo plugs moved to the face opposite the horn, 63 mm
    # would also fit (lower CoM) but costs 6 deg of down-look and more FOV occlusion; see metrics_v04.json.
    g.H_TILT = max(g.H_TILT, H_TILT_KEEP + (g.DISC_Z0 - 22.0))   # v0.8: the disc rose 6.3 mm, the tilt axis with it


def split_yoke(yoke, y_srv_in, y_srv_out, y_idle_in):
    z_top = g.DISC_Z0 + g.DISC_T
    big = 500.0
    # v0.7 fix: clip to the disc - the tilt-servo channel walls start 0.1 mm below the disc top and left two
    # 0.1 mm slivers hanging outside the disc on the turntable since v0.5.4
    turntable = yoke.intersect(g.box(-big, big, -big, big, -big, z_top)).intersect(g.cyl_z(g.DISC_R + 0.01, -big, z_top + 1))
    upper = yoke.cut(g.box(-big, big, -big, big, -big, z_top))
    ys = 0.0
    arm_s = upper.intersect(g.box(-big, big, ys, big, z_top, big))
    arm_i = upper.intersect(g.box(-big, big, -big, ys, z_top, big))
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
    """v0.5.5: the one-piece deck is now the standard lower deck (kept as an alias for older scripts)."""
    return g.lower_deck()


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
    """v0.6 fit check before the big prints (about 2 g): three M3 pilot holes d2.4/2.6/2.8, 7.5 deep, left to
    right. Pick the one an M3x10 drives into firmly by hand and set PILOT_R = d/2."""
    base = g.box(0, 36, 0, 10, 0, 3)
    for i, d in enumerate((2.4, 2.6, 2.8)):
        base = base.union(g.cyl_z(4.0, 0, 8, 6 + i * 12, 5)).cut(g.cyl_z(d / 2, 0.5, 9, 6 + i * 12, 5))
    return base


def horn_gauge():
    """v0.7 C02 (user, 11-14: horn does not fit, no caliper data). Eight through-pockets D = 20..27 mm with the same
    crush ribs as the turntable post and the cradle hub: push the horn into each; the snug one is HORN_D. A comb of
    slots 3.0..6.0 mm (step 0.5) gives the horn thickness HORN_T. Numbers are engraved next to each."""
    t = 3.0
    out = None
    for i, d in enumerate(range(20, 28)):
        cx, cy = (i % 4) * 33.0, (i // 4) * 35.0
        blk = g.box(cx - 15.5, cx + 15.5, cy - 15.5, cy + 19.0, 0, t)
        keep = g.HORN_D
        g.HORN_D = float(d)
        pk = g.horn_pocket(-1, t + 1, "z").translate((cx, cy, 0))
        g.HORN_D = keep
        txt = cq.Workplane("XY").text(str(d), 4.0, 0.6, halign="center", valign="center").translate((cx, cy + 16.6, t - 0.6))
        blk = blk.cut(pk).cut(txt)
        out = blk if out is None else out.union(blk)
    comb = g.box(-15.5, 117.5, 54.0, 72.0, 0, t)
    for k in range(7):
        w = 3.0 + 0.5 * k
        x = -8 + k * 18.5
        comb = comb.cut(g.box(x - w / 2, x + w / 2, 54.0 - 1, 66.0, -1, t + 1))
        comb = comb.cut(cq.Workplane("XY").text(f"{w:.1f}", 3.5, 0.6, halign="center", valign="center").translate((x, 69.0, t - 0.6)))
    return out.union(comb)


def cam_coupon(v):
    """v0.7 C03-<v>: the camera end of the tilt cradle (web + the stubs of both side walls) built with fixing
    variant v, cut out so the real camera can be tried on it. Variant letter engraved on the wall top."""
    cr = g.tilt_cradle(mount=v)
    x1 = -3.0
    c = cr.intersect(g.box(g.WEB_X0 - 0.01, x1, -(g.CRADLE_HALF_Y + 0.01), g.CRADLE_HALF_Y - 0.01, -100, 300))
    c = c.translate((0, 0, -g.H_TILT))
    txt = cq.Workplane("YZ").text(v, 5.0, 0.6, halign="center", valign="center").rotate((0, 0, 0), (0, 0, 1), 0)
    txt = txt.translate((x1 - 0.6, 0, g.CRADLE_HALF_Z - 4.0))
    return c.cut(txt)


# build-plate orientation per part: (axis, angle) rotations applied in order
ORIENT = {
    "top_deck": [((1, 0, 0), 180)],          # v0.6.1: top face on the plate, pillar pegs standing up
    "lower_deck": [],                        # deck bottom on the plate, pillars standing (one piece, v0.5.5)
    "pan_cage": [],                          # floor on the plate; thin ears carried by 45-degree side ribs
    "bearing_mount": [],
    "horn_coupler": [],
    "turntable": [((1, 0, 0), 180)],         # top face on the plate: housing, lip and groove face up
    "arm_servo": [],                         # standing on its foot: saddle walls vertical, no support (brim)
    "arm_idle": [((1, 0, 0), 90)],           # outer (-Y) face on the plate
    "tilt_cradle": [((1, 0, 0), 180)],        # v0.5.5 H section: top edge on the plate, walls and web vertical
    "fit_coupon": [],
    "eye_plate": [((0, 1, 0), -90)],          # back face on the plate, rim and engraving up
    "check_coupon": [],
    "horn_gauge": [],
    **{f"cam_coupon_{v}": [((0, 1, 0), -90)] for v in "ABCDEF"},   # web back on the plate, walls and lips up
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
    assy = cq.Assembly(name="gimbal_v05")
    colors = {n: v[1] for n, v in parts.items()}
    colors.update(turntable=(0.20, 0.45, 0.85), arm_servo=(0.25, 0.50, 0.90), arm_idle=(0.25, 0.50, 0.90))
    for name, shape in printed.items():
        assert shape.val().isValid(), name
        cq.exporters.export(shape, os.path.join(OUT, "step", f"{name}.step"))
        if True:
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
