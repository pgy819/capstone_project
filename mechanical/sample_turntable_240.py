"""SAMPLES (not in the main model, 11-14): turntable with a wide hub joined to the disc over 240 degrees, only the
cable opening (120 deg, behind) left open. Two ways for the camera ribbon to cross the opening:

  case 1 "edge": ribbon stands on edge (thin in plan) -> needs 16 mm of height, turntable tall, wide pan range
  case 2 "flat": ribbon lies flat in a 2 mm slot      -> turntable low, but its 16 mm width eats the opening

Both share the horn ring (pocket + crush ribs + radial slots, heads sunk), the 24 mm centre hole, the arm bolt holes
with nut recesses, and the fixed ribbon path beside the pan cage (x = -16.7). Pan range is measured, not assumed."""
import os
import tempfile

import cadquery as cq
import numpy as np
import trimesh

import gimbal_print as gp
import gimbal_v03 as g

gp.configure()
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sample_turntable_240")

Z0 = g.PAN_HORN_Z + g.HORN_T          # horn top
OPEN = 120.0                          # cable opening, centred behind (180 deg)
HUB_W = 24.0                          # wide hub radius (the part joined to the disc over 240 deg)
R_IN = 12.5                           # inside: screwdriver access to the horn screws through the centre hole
RISE = g.AX_RISE_X                    # 16.7: ribbon riser beside the cage end wall
HW = g.FPC_W / 2


def case_dims(case):
    if case == "edge":
        zf = Z0 + 6.0                 # floor, heads sunk 3
        leg0 = zf + 0.8
        disc0 = leg0 + g.FPC_W + 1.0
    else:
        zf = Z0 + 5.0
        leg0 = zf + 0.6
        disc0 = zf + 2.0
    return zf, leg0, disc0


def sector(r0, r1, a_half, z0, z1, centre=0.0):
    a0, a1 = centre - a_half, centre + a_half
    return (cq.Workplane("XY").moveTo(*g.polar(a0, r0)).lineTo(*g.polar(a0, r1))
            .threePointArc(g.polar(centre, r1), g.polar(a1, r1)).lineTo(*g.polar(a1, r0))
            .threePointArc(g.polar(centre, r0), g.polar(a0, r0)).close()
            .extrude(z1 - z0).translate((0, 0, z0)))


def turntable(case):
    zf, leg0, disc0 = case_dims(case)
    ring = g.cyl_z(g.HUB_R, Z0 - g.HORN_DEPTH, zf)
    ring = ring.cut(g.horn_pocket(Z0 - g.HORN_DEPTH - 1, Z0, "z"))
    ring = ring.cut(g.horn_slots(Z0 - 1, zf + 1, zf - 3.0, "z"))
    ring = ring.cut(g.cyl_z(3.25, Z0 - 1, zf + 1))
    a = (360.0 - OPEN) / 2
    # wide hub over the front 240 deg: from the horn ring level up to the disc
    hub = sector(g.HUB_R - 0.6, HUB_W, a, Z0 - g.HORN_DEPTH, zf + 0.01).union(   # beside the horn ring
        sector(R_IN, HUB_W, a, zf - 0.3, disc0 + 0.01))                             # above the floor (r < 12 open)
    if case == "edge":   # the ends of the 240 deg near the riser stay inside it (r < 16.2), so the riser is not hit early
        for sgn in (1, -1):
            hub = hub.cut(sector(16.2, HUB_W + 1, 12.0, Z0 - 5, disc0 + 1, centre=sgn * (a - 12.0)))
    disc = g.cyl_z(g.DISC_R, disc0, disc0 + g.DISC_T).cut(g.cyl_z(g.AX_HOLE_R, disc0 - 1, disc0 + g.DISC_T + 1))
    t = ring.union(hub).union(disc)
    for x, y in g.ARM_BOLTS():
        t = t.cut(g.cyl_z(1.7, Z0 - 10, disc0 + g.DISC_T + 1, x, y))
        zb = disc0 if np.hypot(x, y) > HUB_W or abs(((np.degrees(np.arctan2(y, x)) - 180) + 180) % 360 - 180) < OPEN / 2 else Z0 - g.HORN_DEPTH
        t = t.cut(g.hex_z(g.NUT_AF + 0.3, zb - 10, zb + 3.0 if zb == disc0 else zb + 3.0, x, y, 0.0))
    out = t.clean()
    return cq.Workplane("XY").add(max(out.solids().vals(), key=lambda v: v.Volume()))


def ribbon(case, t=0.4):
    zf, leg0, disc0 = case_dims(case)
    if case == "edge":
        rise = g.box(-RISE - t, -RISE, -HW, HW, -30, leg0 + g.FPC_W)
        leg = g.box(-RISE, 0, -t / 2, t / 2, leg0, leg0 + g.FPC_W)
        up = g.box(-HW, HW, -t / 2, t / 2, leg0, disc0)
    else:
        rise = g.box(-RISE - t, -RISE, -HW, HW, -30, leg0 + t / 2)
        leg = g.box(-RISE, 0, -HW, HW, leg0 - t / 2, leg0 + t / 2)
        up = g.box(-t, 0, -HW, HW, leg0 - t / 2, disc0)
    return rise.union(leg).union(up)


def pan_range(tt, rib):
    d = tempfile.mkdtemp()
    cq.exporters.export(tt, d + "/t.stl", tolerance=0.02)
    cq.exporters.export(rib, d + "/r.stl", tolerance=0.02)
    T, pts = trimesh.load(d + "/t.stl"), trimesh.load(d + "/r.stl").sample(30000)
    free = []
    for psi in range(-90, 91, 1):
        M = T.copy()
        M.apply_transform(trimesh.transformations.rotation_matrix(np.radians(psi), [0, 0, 1]))
        if not M.contains(pts).any():
            free.append(psi)
    # largest contiguous interval around 0
    lo = hi = 0
    while lo - 1 in free:
        lo -= 1
    while hi + 1 in free:
        hi += 1
    return lo, hi


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    for i, case in enumerate(("edge", "flat"), 1):
        tt = turntable(case)
        zf, leg0, disc0 = case_dims(case)
        lo, hi = pan_range(tt, ribbon(case))
        bb = tt.val().BoundingBox()
        assert tt.val().isValid()
        print(case, "valid", tt.val().isValid(), "height", round(bb.zlen, 1), "disc underside", round(disc0, 1),
              "range", lo, hi, "vol cm3", round(tt.val().Volume() / 1000, 1))
        base = os.path.join(OUT, f"S0{i}_turntable_240_{case}")
        cq.exporters.export(tt, base + ".step")
        cq.exporters.export(tt.rotate((0, 0, 0), (1, 0, 0), 180), base + "_print.stl", tolerance=0.02, angularTolerance=0.05)
