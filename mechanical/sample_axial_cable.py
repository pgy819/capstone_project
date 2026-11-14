"""SAMPLE (not in the main model): camera FPC up along the pan axis, out through the middle of the turntable
(user idea, 11-14). Builds a stand-alone turntable variant + the parts around it, for confirmation only.

  * The FPC rises beside the pan servo (side face, y = +17.5) through a deck slot, folds 45 deg so its width
    stands vertical, runs radially in to the axis above the horn floor, folds again and goes straight up the axis
    through a 24 mm hole in the turntable centre. Above the horn floor the axis region is fixed-side cable only.
  * The turntable keeps a full ring on the horn (pocket + crush ribs + 4 radial screw slots, heads counterbored
    flush), but between that ring and the disc only ONE 50-degree sector post stands, opposite the cable. The
    other 310 degrees are open for the radial cable leg -> pan range +-(180 - 25 - margin) = about +-150 deg; the
    sector meeting the cable is the hard end.
  * The cable cup is gone. The disc rises SAMPLE_RAISE above the v0.7 height (16 mm ribbon on edge above the
    screw heads)."""
import math
import os
import cadquery as cq
import gimbal_print as gp
import gimbal_v03 as g

gp.configure()
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "sample_axial")

RIB_W = g.FPC_W                      # ribbon width (stands vertical in the radial leg)
Z_HORN_TOP = g.PAN_HORN_Z + g.HORN_T
FLOOR_TOP = Z_HORN_TOP + 6.0         # 6 mm floor over the horn, screw heads counterbored 3 mm -> flush
LEG_Z0 = FLOOR_TOP + 0.8             # radial ribbon leg bottom edge (0.8 above the floor)
DISC_Z0 = LEG_Z0 + RIB_W + 1.0       # disc underside 1 mm above the ribbon top edge
SAMPLE_RAISE = DISC_Z0 - g.DISC_Z0
DISC_T = 5.0
HOLE_R = 12.0                        # turntable centre hole: ribbon out + screwdriver to the horn screws
SECTOR = 50.0                        # sector post width [deg], centred opposite the cable (270 deg at neutral)
SECTOR_R0 = 9.0                      # sector inner radius (axial ribbon twists inside r < 8)
# v2 (user, 11-14): the cable comes up past the cage's shaft-end face (the Pi side, -X), not its side face.
# The shaft is only 11.2 mm from that end, so this is the shortest way in. The cage ear that sat there (-20, 0)
# moves to two ears at the end corners (-20, +-14.5), leaving the middle free for the 16 mm ribbon.
CABLE_A = 180.0                      # fixed radial leg direction (-X)
RISE_Y = g.SV_SHAFT_FROM_END + 0.3 + 3.0 + 1.0   # ribbon rises 1 mm outside the cage end wall (built along +Y, then turned)
TURN = CABLE_A - 90.0
CAGE_BOLTS_SAMPLE = ((42.0, 11.0), (42.0, -11.0), (-20.0, 14.5), (-20.0, -14.5))


def turntable_sample():
    r = g.HUB_R
    ring = g.cyl_z(r, Z_HORN_TOP - g.HORN_DEPTH, FLOOR_TOP)
    ring = ring.cut(g.horn_pocket(Z_HORN_TOP - g.HORN_DEPTH - 1, Z_HORN_TOP, "z"))
    ring = ring.cut(g.horn_slots(Z_HORN_TOP - 1, FLOOR_TOP + 1, FLOOR_TOP - 3.0, "z"))
    ring = ring.cut(g.cyl_z(3.25, Z_HORN_TOP - 1, FLOOR_TOP + 1))          # spline screw access
    a0, a1 = 270 - SECTOR / 2, 270 + SECTOR / 2
    sec = (cq.Workplane("XY").moveTo(*g.polar(a0, SECTOR_R0)).lineTo(*g.polar(a0, r))
           .threePointArc(g.polar(270, r), g.polar(a1, r)).lineTo(*g.polar(a1, SECTOR_R0))
           .threePointArc(g.polar(270, SECTOR_R0), g.polar(a0, SECTOR_R0)).close()
           .extrude(DISC_Z0 - FLOOR_TOP + 0.02).translate((0, 0, FLOOR_TOP - 0.01)))
    disc = g.cyl_z(g.DISC_R, DISC_Z0, DISC_Z0 + DISC_T).cut(g.cyl_z(HOLE_R, DISC_Z0 - 1, DISC_Z0 + DISC_T + 1))
    sec = sec.rotate((0, 0, 0), (0, 0, 1), TURN)
    out = ring.union(sec).union(disc).clean()
    big = max(out.solids().vals(), key=lambda v: v.Volume())          # drop a zero-volume sliver from the union
    return cq.Workplane("XY").add(big)


def deck_sample():
    d = g.box(-45, 45, -45, 45, -g.DECK_T, 0)
    c = 0.4
    d = d.cut(g.box(-g.SV_SHAFT_FROM_END - c, g.SV_L - g.SV_SHAFT_FROM_END + c, -g.SV_W / 2 - c, g.SV_W / 2 + c, -g.DECK_T - 1, 1))
    slot = g.box(-RIB_W / 2 - 1, RIB_W / 2 + 1, RISE_Y - 0.8, RISE_Y + 2.0, -g.DECK_T - 1, 1).rotate((0, 0, 0), (0, 0, 1), TURN)
    d = d.cut(slot)
    for sx, sy in CAGE_BOLTS_SAMPLE:
        d = d.cut(g.cyl_z(g.PILOT_R, -g.DECK_T - 1, 1, sx, sy))
    return d


def cage_sample():
    keep = g.CAGE_BOLTS
    g.CAGE_BOLTS = CAGE_BOLTS_SAMPLE
    c = g.pan_cage()
    g.CAGE_BOLTS = keep
    return c


def ribbon(psi=0.0, z_top=55.0, t=0.4):
    """Display ribbon: rise beside the servo (width along X), radial leg on edge (+Y -> axis), up the axis.
    psi = pan angle: the axial part above the disc twists with the turntable."""
    rise = g.box(-RIB_W / 2, RIB_W / 2, RISE_Y, RISE_Y + t, -40, LEG_Z0 + RIB_W)
    leg = g.box(-t / 2, t / 2, 0, RISE_Y + t, LEG_Z0, LEG_Z0 + RIB_W)
    up = g.box(-t / 2, t / 2, -RIB_W / 2, RIB_W / 2, LEG_Z0, DISC_Z0)          # fixed side, width along Y
    tw = None
    n = 8
    z0, z1 = DISC_Z0, z_top
    for k in range(n):                                                          # twisting part, stepped
        s = g.box(-t / 2, t / 2, -RIB_W / 2, RIB_W / 2, z0 + (z1 - z0) * k / n, z0 + (z1 - z0) * (k + 1) / n + 0.01)
        s = s.rotate((0, 0, 0), (0, 0, 1), psi * (k + 0.5) / n)
        tw = s if tw is None else tw.union(s)
    return rise.union(leg).union(up).rotate((0, 0, 0), (0, 0, 1), TURN).union(tw.rotate((0, 0, 0), (0, 0, 1), TURN))


def check():
    """Pan range in which the turntable never touches the fixed ribbon (mesh point test, 5 deg steps; the CAD
    boolean misses some hits on coincident faces)."""
    import tempfile
    import numpy as np
    import trimesh
    tt = turntable_sample()
    rib_fixed = ribbon(0, DISC_Z0)
    cg = cage_sample()
    v = cg.intersect(rib_fixed)
    assert not v.solids().vals() or v.val().Volume() < 0.01, "ribbon hits the cage"
    d = tempfile.mkdtemp()
    cq.exporters.export(tt, d + "/t.stl", tolerance=0.02)
    cq.exporters.export(rib_fixed, d + "/r.stl", tolerance=0.02)
    T, pts = trimesh.load(d + "/t.stl"), trimesh.load(d + "/r.stl").sample(20000)
    free = []
    for psi in range(-180, 181, 5):
        M = T.copy()
        M.apply_transform(trimesh.transformations.rotation_matrix(np.radians(psi), [0, 0, 1]))
        if not M.contains(pts).any():
            free.append(psi)
    return min(free), max(free)


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    tt = turntable_sample()
    lo, hi = check()
    print("raise", SAMPLE_RAISE, "disc z", DISC_Z0, "range", lo, hi, "valid", tt.val().isValid(), round(tt.val().Volume() / 1000, 1))
    cq.exporters.export(tt, os.path.join(OUT, "S01_turntable_axial_cable.step"))
    cq.exporters.export(tt.rotate((0, 0, 0), (1, 0, 0), 180), os.path.join(OUT, "S01_turntable_axial_cable_print.stl"), tolerance=0.02, angularTolerance=0.05)
