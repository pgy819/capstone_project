"""v1.0 CANDIDATE (user, 11-16): the simplest structure that works, built next to v0.9.2 (which stays the main model).
No new purchases: same servos, Pi stack, camera, M3 / M2.5 / M2 screws and M3 nuts.

Idea: the Pi rides on the pan stage, so the wide camera FPC never crosses the pan joint. Only round wires cross it
(Pi power, Pi-STM32 UART, tilt-servo bus) in a loose loop round the hub, so pan gets its full +-150 deg back and the
turntable is a plain plate again.

  base     : PEDESTAL (one print: foot plate + servo tower) + CLAMP PLATE (4 x M3 from the top; its lip holds the
             servo down). The servo drops in from the top and stands on two rails, the plug hangs between them and
             the cable leaves through a side window. No pillars, no decks, no lap joints.
  pan      : TURNTABLE = horn post (pocket + crush ribs + radial slots, unchanged) + table plate; the two tilt arms
             (unchanged) and a PI BRACKET (vertical plate behind the tilt head, Pi on 5 mm standoffs) bolt to it with
             M3 x 12 + nuts sunk in the table underside. A wire slot next to the post takes the crossing wires.
  tilt     : unchanged (slide-in servo channel, snap pivot, H cradle, slide-in camera).

Run: python gimbal_v10.py  -> gimbal_v100_candidate/ (STEP, print STL, checks json, README) + zip."""
import json
import math
import os
import shutil
import tempfile

import cadquery as cq
import numpy as np
import trimesh

import explore_v03 as e
import gimbal_print as gp
import gimbal_v03 as g

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "gimbal_v100_candidate")

# ----------------------------------------------------------------------------- configuration on top of v0.9.2
gp.configure()
g.configure(wrap_r=36.0, stop="none", tilt_orient="down")   # plain post-on-horn turntable branch
g.NO_LIGHTEN = True
DZ = 14.0 - g.DISC_Z0
g.DISC_Z0 = 14.0                  # 9.5 mm post over the horn: room for the wire loop under the table
g.DISC_T = 6.0
g.DISC_R = 48.0                   # big disc, cut back to the table outline below
g.H_TILT = 57.0 + g.DISC_Z0       # same table-to-tilt-axis height as v0.9 (57)
g.ARM_NUT_POCKET = 3.0
g.PAN_SOFT_LIMIT = 150.0

PAN_RANGE = 150.0
SV0, SV1 = -g.SV_SHAFT_FROM_END, g.SV_L - g.SV_SHAFT_FROM_END     # servo body x range (shaft at x = 0)
SW = g.SV_W / 2
C = 0.3                          # pocket clearance
WALL = 3.0
PLUG_ROOM = 16.0                 # plug + cable bend under the servo (rails carry the servo above it)
Z_SV_BOT = g.SV_TOP - g.SV_H
Z_PLATE_TOP = Z_SV_BOT - 0.1 - PLUG_ROOM
PLATE_T = 4.0
PLATE = 120.0                    # foot plate (square, centred on the pan axis): stable with the Pi swinging round
CLAMP_T = 2.0                    # clamp plate thickness (top at z = 2, the horn post starts at 2.5)
LIP = 1.5                        # clamp lip over the servo top rim [VERIFY: flat rim at the far end of the servo]
BOSS = [(SV0 - C - WALL - 3.0, s * (SW + C + WALL + 2.0)) for s in (1, -1)] + \
       [(SV1 + C + WALL + 3.0, s * (SW + C + WALL + 2.0)) for s in (1, -1)]

TABLE = (-46.0, 26.0, -30.0, 27.0, 6.0)   # x0, x1, y0, y1, corner r: arm feet + bracket flange
WIRE_SLOT = (-24.0, -16.0, -9.5, 9.5)     # crossing wires (a USB-C plug passes edge-on)
BRK_X = -32.0                             # bracket plate rear face (cradle sweeps to about x = -27; >= 4 mm gap)
BRK_T = 3.0
BRK_FLANGE = (-44.0, BRK_X)
BRK_BOLTS = [(-39.0, 24.0), (-39.0, -24.0)]
PI_Z0 = None                              # set in configure_pi()
PI_GAP = 5.0                              # standoffs
PLUG_DOWN = 26.0                          # USB-C plug + bend below the Pi's connector edge


def table_top():
    return g.DISC_Z0 + g.DISC_T


def pi_map(lx, ly):
    """Pi local (lx along 85 from the PCIe end, ly along 56 from the connector edge) -> (y, z) on the bracket."""
    return lx - g.PI_L / 2, table_top() + PLUG_DOWN + ly


X_PCB_BACK = BRK_X - PI_GAP                  # board back face
X_PCB_FRONT = X_PCB_BACK - g.PI_PCB          # component side (faces -X, away from the camera)


# ----------------------------------------------------------------------------- base
def pedestal():
    x0, x1 = SV0 - C - WALL, SV1 + C + WALL
    y0 = SW + C + WALL
    plate = g.box(-PLATE / 2, PLATE / 2, -PLATE / 2, PLATE / 2, Z_PLATE_TOP - PLATE_T, Z_PLATE_TOP).edges("|Z").fillet(8.0)
    tower = g.box(x0, x1, -y0, y0, Z_PLATE_TOP - 0.01, g.SV_TOP)
    pocket = g.box(SV0 - C, SV1 + C, -SW - C, SW + C, Z_PLATE_TOP + 0.01, g.SV_TOP + 1)
    tower = tower.cut(pocket)
    rails = None
    for s in (1, -1):   # servo stands on two rails along the long sides; the plug hangs between them
        r = g.box(SV0 - C, SV1 + C, *sorted((s * (SW + C + 0.01), s * (SW - 4.0))), Z_PLATE_TOP - 0.01, Z_SV_BOT - 0.1)
        rails = r if rails is None else rails.union(r)
    p = plate.union(tower).union(rails)
    for bx, by in BOSS:   # clamp-plate bosses at the tower corners, full height, M3 pilots from the top
        p = p.union(g.cyl_z(4.0, Z_PLATE_TOP - 0.01, g.SV_TOP, bx, by))
        p = p.cut(g.cyl_z(g.PILOT_R, g.SV_TOP - 9.0, g.SV_TOP + 1, bx, by))
    # side window for the servo cable (under the servo, -Y wall), plate hole under the plug
    cx = g.SV_CONN_X
    p = p.cut(g.box(cx - 9.0, cx + 9.0, -y0 - 1, -SW + 4.0, Z_PLATE_TOP + 0.01, Z_SV_BOT - 1.0))
    # foot holes: M3 clearance at the corners (screw the gimbal to a board / stage 2)
    for sx in (1, -1):
        for sy in (1, -1):
            p = p.cut(g.cyl_z(1.7, Z_PLATE_TOP - PLATE_T - 1, Z_PLATE_TOP + 1, sx * (PLATE / 2 - 8), sy * (PLATE / 2 - 8)))
    # root fillet ribs: four 45-degree gussets from the plate to the tower walls (tower is 50 mm tall)
    hz = 12.0
    for sx, xw in ((1, x1), (-1, x0)):
        g_ = (cq.Workplane("XZ").polyline([(0, 0), (sx * hz, 0), (0, hz)]).close().extrude(6.0, both=True)
              .translate((xw, 0, Z_PLATE_TOP - 0.01)))
        p = p.union(g_)
    for sy in (1, -1):
        g_ = (cq.Workplane("YZ").polyline([(0, 0), (sy * hz, 0), (0, hz)]).close().extrude(3.0, both=True)
              .translate((SV1 - 8.0, sy * y0, Z_PLATE_TOP - 0.01)))
        p = p.union(g_)
    return p


def clamp_plate():
    xa, xb = min(b[0] for b in BOSS) - 4.0, max(b[0] for b in BOSS) + 4.0
    ya = max(abs(b[1]) for b in BOSS) + 4.0
    c = g.box(xa, xb, -ya, ya, g.SV_TOP, g.SV_TOP + CLAMP_T).edges("|Z").fillet(4.0)
    opening = g.box(SV0 + LIP, SV1 - LIP, -SW + LIP, SW - LIP, -1, 10).union(g.cyl_z(g.HORN_D / 2 + 1.0, -1, 10))
    c = c.cut(opening)
    for bx, by in BOSS:
        c = c.cut(g.cyl_z(1.7, -1, 10, bx, by))
    return c


# ----------------------------------------------------------------------------- pan stage
def table_prism(z0, z1):
    x0, x1, y0, y1, rc = TABLE
    return g.box(x0, x1, y0, y1, z0, z1).edges("|Z").fillet(rc)


def turntable_and_arms():
    yoke, a, b, c = g.turntable_yoke()
    tt, arm_s, arm_i = gp.split_yoke(yoke, a, b, c)
    tt = tt.intersect(table_prism(-100, 200))
    zt = table_top()
    x0, x1, y0, y1 = WIRE_SLOT
    tt = tt.cut(g.box(x0, x1, y0, y1, g.DISC_Z0 - 1, zt + 1).edges("|Z").fillet(3.0))
    for bx, by in BRK_BOLTS:   # bracket bolts: through + nut sunk in the underside
        tt = tt.cut(g.cyl_z(1.7, g.DISC_Z0 - 1, zt + 1, bx, by))
        tt = tt.cut(g.hex_z(g.NUT_AF + 0.3, g.DISC_Z0 - 1, g.DISC_Z0 + 3.0, bx, by, 0.0))
    for bx, by in g.ARM_BOLTS():   # arm bolts: nut sunk 3 mm (the plain branch cuts no nut recess)
        tt = tt.cut(g.hex_z(g.NUT_AF + 0.3, g.DISC_Z0 - 1, g.DISC_Z0 + 3.0, bx, by, 0.0))
    return tt, arm_s, arm_i


def pi_bracket():
    zt = table_top()
    top = pi_map(0, g.PI_W)[1] + 2.0
    plate = g.box(BRK_X, BRK_X + BRK_T, -g.PI_L / 2 - 2.5, g.PI_L / 2 + 2.5, zt, top)
    flange = g.box(BRK_FLANGE[0], BRK_FLANGE[1] + 0.01, -30.0, 30.0, zt, zt + 4.0)
    b = plate.union(flange)
    for sy in (1, -1):   # two gussets between flange and plate
        gu = (cq.Workplane("XZ").polyline([(0, 0), (-12.0, 0), (0, 14.0)]).close().extrude(1.5, both=True)
              .translate((BRK_X, sy * 28.5, zt + 3.99)))
        b = b.union(gu)
    for bx, by in BRK_BOLTS:
        b = b.cut(g.cyl_z(1.7, zt - 1, zt + 10, bx, by))
    # Pi standoffs (5 mm) with M2.5 pilots, screws from the component side
    for lx, ly in g.PI_HOLES:
        y, z = pi_map(lx, ly)
        b = b.union(g.cyl_x(3.0, X_PCB_BACK, BRK_X + 0.01, z, y))
        b = b.cut(g.cyl_x(g.M25_PILOT_R, X_PCB_BACK - 1, BRK_X + BRK_T - 0.8, z, y))
    # FPC window under the board (camera -> CAM connector) and air slots behind the Pi
    cy, cz = pi_map(48.0, 0.0)
    b = b.cut(g.box(BRK_X - 1, BRK_X + BRK_T + 1, cy - 10.0, cy + 10.0, zt + 6.0, cz - 1.0).edges("|X").fillet(2.0))
    for k in range(6):
        y = -30.0 + k * 12.0
        b = b.cut(g.box(BRK_X - 1, BRK_X + BRK_T + 1, y - 2.0, y + 2.0, cz + 10.0, cz + 46.0).edges("|X").fillet(1.99))
    return b


def pi_model():
    y0, y1 = -g.PI_L / 2, g.PI_L / 2
    z0, z1 = pi_map(0, 0)[1], pi_map(0, g.PI_W)[1]
    pcb = g.box(X_PCB_FRONT, X_PCB_BACK, y0, y1, z0, z1)
    stack = g.box(X_PCB_FRONT - (g.PI_STACK_H - g.PI_PCB), X_PCB_FRONT, y0 + 2, y1 - 2, z0 + 2, z1 - 2)
    return pcb.union(stack)


def fasteners_base_v10():
    """Clamp plate: 4 x M3 x 10 down into the pedestal bosses."""
    out = None
    for bx, by in BOSS:
        f = g.m3(bx, by, g.SV_TOP + CLAMP_T, up=False)
        out = f if out is None else out.union(f)
    return out


def fasteners_v10():
    """Screws/nuts of the new pan-stage parts (bought list unchanged: M3 x 12, M3 nut, M2.5 x 8)."""
    out = None
    zt = table_top()
    for bx, by in BRK_BOLTS:   # bracket: M3 x 12 down through flange (4) + table (6), nut in the recess
        f = g.m3(bx, by, zt + 4.0, up=False, length=12.0, nut_z=g.DISC_Z0 + 3.0 - g.NUT_H, nut_rot=0.0)
        out = f if out is None else out.union(f)
    for lx, ly in g.PI_HOLES:   # Pi: M2.5 x 8 from the component side into the standoffs
        y, z = pi_map(lx, ly)
        out = out.union(g.cyl_x(g.M25_HEAD_D / 2, X_PCB_FRONT - g.M25_HEAD_H, X_PCB_FRONT, z, y)
                        .union(g.cyl_x(1.25, X_PCB_FRONT, X_PCB_FRONT + 8.0, z, y)))
    return out


def routes():
    zt = table_top()
    cy, cz = pi_map(48.0, 4.0)
    zc = g.H_TILT
    fpc = [(g.CRADLE_X0 + 1, 0, zc - g.CRADLE_HALF_Z + 4), (g.CRADLE_X0 - 3, 0, zc - g.CRADLE_HALF_Z - 4),
           (BRK_X + 4, cy, zt + 12), (BRK_X - 2, cy, zt + 12), (X_PCB_FRONT - 3, cy, zt + 14), (X_PCB_FRONT - 3, cy, cz)]
    uy, uz = pi_map(g.PI_PORTS["USB-C"], 0)
    loop = [(*g.polar(180 - 300 * k / 12, 22.0), 8.0) for k in range(13)]
    power = [(X_PCB_FRONT - 4, uy, uz), (X_PCB_FRONT - 4, uy, zt + 4), (-20, -4, zt + 2), (-20, -4, 8.0)] + loop + \
            [(60, 0, 8.0), (60, 0, Z_PLATE_TOP)]
    gy, gz = pi_map(sum(g.PI_GPIO_X) / 2, g.PI_W - 3)
    uart = [(X_PCB_FRONT - 4, gy, gz), (X_PCB_FRONT - 6, gy, zt + 4), (-20, 4, zt + 2), (-20, 4, 8.0)] + loop + \
           [(60, 0, 8.0), (60, 0, Z_PLATE_TOP)]
    cx, cyy, czz = g.tilt_conn_point()
    bus = [(cx, cyy, czz), (0, cyy * 0.5, zt + 6), (-20, 0, zt + 2), (-20, 0, 8.0)] + loop + [(60, 0, 8.0), (60, 0, Z_PLATE_TOP)]
    return {"camera FPC": fpc, "Pi power (USB-C)": power, "UART Pi-STM32": uart, "tilt servo bus": bus}


# ----------------------------------------------------------------------------- assembly, checks, export
def build():
    tt, arm_s, arm_i = turntable_and_arms()
    ts = g.tilt_rot(g.servo_z_up())
    P = {
        "pedestal": (pedestal(), True, "base"),
        "clamp_plate": (clamp_plate(), True, "base"),
        "pan_servo": (g.servo_z_up().translate((0, 0, g.SV_TOP)), False, "base"),
        "pan_horn": (g.cyl_z(g.HORN_D / 2, g.PAN_HORN_Z, g.PAN_HORN_Z + g.HORN_T), False, "pan"),
        "turntable": (tt, True, "pan"),
        "arm_servo": (arm_s, True, "pan"),
        "arm_idle": (arm_i, True, "pan"),
        "pi_bracket": (pi_bracket(), True, "pan"),
        "pi5_stack": (pi_model(), False, "pan"),
        "tilt_servo": (ts, False, "pan"),
        "tilt_horn": (g.cyl_y(g.HORN_D / 2, g.CRADLE_HALF_Y, g.CRADLE_HALF_Y + g.HORN_T, 0, g.H_TILT), False, "tilt"),
        "tilt_cradle": (g.tilt_cradle(), True, "tilt"),
        "camera": (g.camera_module().translate((g.CAM_BACK_X, 0, g.H_TILT)), False, "tilt"),
    }
    _, fpan = g.fasteners()
    P["fasteners_pan"] = (fpan, False, "pan")
    P["fasteners_v10"] = (fasteners_v10(), False, "pan")
    P["fasteners_base"] = (fasteners_base_v10(), False, "base")
    P["fasteners_tilt"] = (g.fasteners_tilt(), False, "tilt")
    return P


EXPECTED = {("pan_servo", "pan_horn"), ("turntable", "pan_horn"), ("tilt_servo", "tilt_horn"), ("tilt_cradle", "tilt_horn"),
            ("tilt_cradle", "camera"), ("turntable", "fasteners_pan"), ("pan_horn", "fasteners_pan"), ("camera", "fasteners_tilt"),
            ("tilt_cradle", "fasteners_tilt"), ("arm_servo", "fasteners_pan"), ("arm_idle", "fasteners_pan"),
            ("pedestal", "fasteners_v10"), ("pi_bracket", "fasteners_v10"), ("pi5_stack", "fasteners_v10"), ("turntable", "fasteners_v10"),
            ("arm_servo", "arm_idle"), ("turntable", "arm_servo"), ("turntable", "arm_idle"),
            ("arm_idle", "tilt_cradle"),                      # snap-pivot grip (by design)
            ("pedestal", "fasteners_base"), ("clamp_plate", "fasteners_base")}


def checks(P):
    M = {n: e.mesh("v10_" + n, v[0]) for n, v in P.items()}
    res = {"static": []}
    names = list(M)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if (a, b) in EXPECTED or (b, a) in EXPECTED:
                continue
            v = e.overlap(M[a], M[b])
            if v > 0.5:
                res["static"].append(f"{a} x {b} {v:.1f} mm3")
    pan = [n for n, v in P.items() if v[2] in ("pan", "tilt") and n != "pan_horn"]
    base = [n for n, v in P.items() if v[2] == "base" and n not in ("pan_servo", "fasteners_base")]
    mp = trimesh.util.concatenate([M[n] for n in pan])
    mb = trimesh.boolean.union([M[n] for n in base])
    res["pan_overlap_max_mm3"] = round(max(e.overlap(e.rot(mp, s * d, [0, 0, 1], [0, 0, 0]), mb)
                                           for d in (30, 60, 90, 120, 150) for s in (-1, 1)), 2)
    tg = trimesh.boolean.union([M[n] for n in ("tilt_cradle", "camera", "tilt_horn", "fasteners_tilt")])
    fixed = trimesh.boolean.union([M[n] for n in ("turntable", "arm_servo", "arm_idle", "tilt_servo", "pi_bracket", "pi5_stack")])
    base0 = e.overlap(tg, fixed)
    res["tilt_overlap_max_mm3"] = round(max(e.overlap(e.rot(tg, d, [0, 1, 0], [0, 0, g.H_TILT]), fixed) for d in range(-90, 91, 10)) - base0, 2)
    # nearest approach of the swinging cradle to the bracket plate
    br = M["pi_bracket"]
    gap = 99.0
    for d in range(-90, 91, 10):
        pts = e.rot(M["tilt_cradle"], d, [0, 1, 0], [0, 0, g.H_TILT]).vertices
        gap = min(gap, float(np.min(pts[:, 0]) - (BRK_X + BRK_T)))
    res["cradle_to_bracket_min_gap_mm"] = round(gap, 1)
    selfm = trimesh.util.concatenate([M[n] for n in ("turntable", "arm_servo", "arm_idle", "tilt_servo", "pi_bracket", "pi5_stack")])
    basem = trimesh.util.concatenate([M[n] for n in ("pedestal", "clamp_plate")])
    res["fov_self"] = {t: round(e.fov_blocked(selfm, t), 4) for t in (-30, 0, 30)}
    res["fov_base"] = {f"{p},{t}": round(e.fov_blocked(basem, t, p), 4) for p in (0, 90, 180) for t in (-30, 0)}
    res["cable_mm"] = {k: round(g.length(v)) for k, v in routes().items()}
    # rotating mass (PLA 60 % infill ~ 0.9 g/cm3 effective; bought parts by catalogue-ish values)
    mass = {"tilt_servo": 55.0, "camera": 5.0, "tilt_horn": 3.0, "pan_horn": 0.0, "pi5_stack": 120.0}
    tot, mom, inz = 0.0, np.zeros(3), 0.0
    for n in pan:
        m = M[n]
        mm = mass.get(n, m.volume * (7.85e-3 if n.startswith("fasteners") else 0.9e-3))
        c = m.center_mass
        tot += mm
        mom += mm * c
        inz += mm * 1e-3 * (c[0] ** 2 + c[1] ** 2) * 1e-6
    com = mom / tot
    res["rotating_mass_g"] = round(tot, 1)
    res["rotating_com_mm"] = [round(float(v), 1) for v in com]
    res["tilt_moment_on_pan_shaft_Nm"] = round(tot * 1e-3 * 9.81 * math.hypot(com[0], com[1]) * 1e-3, 4)
    res["pan_inertia_kgm2_approx"] = float(f"{inz:.2e}")
    res["pan_torque_at_5000degs2_Nm"] = round(inz * math.radians(5000), 4)
    return res


ORIENT = {"pedestal": [], "clamp_plate": [], "turntable": [((1, 0, 0), 180)], "arm_servo": gp.ORIENT["arm_servo"],
          "arm_idle": gp.ORIENT["arm_idle"], "tilt_cradle": gp.ORIENT["tilt_cradle"], "pi_bracket": [((0, 1, 0), 90)]}


def export(P, res):
    if os.path.exists(OUT):
        shutil.rmtree(OUT)
    for d in ("01_CATIA/parts", "02_PRINT"):
        os.makedirs(os.path.join(OUT, d))
    assy = cq.Assembly()
    for n, (sh, printed, grp) in P.items():
        assy.add(sh, name=n)
        if printed:
            cq.exporters.export(sh, os.path.join(OUT, "01_CATIA/parts", n + ".step"))
            s = sh
            for ax, ang in ORIENT[n]:
                s = s.rotate((0, 0, 0), ax, ang)
            bb = s.val().BoundingBox()
            s = s.translate((-(bb.xmin + bb.xmax) / 2, -(bb.ymin + bb.ymax) / 2, -bb.zmin))
            cq.exporters.export(s, os.path.join(OUT, "02_PRINT", n + ".stl"), tolerance=0.02, angularTolerance=0.05)
    assy.save(os.path.join(OUT, "01_CATIA", "gimbal_v100_candidate_assembly.step"))
    json.dump(res, open(os.path.join(OUT, "checks_v10.json"), "w"), indent=1, ensure_ascii=False)
    shutil.copy(os.path.join(HERE, "docs", "V10_CANDIDATE.md"), os.path.join(OUT, "README_v10_후보.md"))
    zp = shutil.make_archive(OUT, "zip", OUT)
    return zp


if __name__ == "__main__":
    P = build()
    for n, (sh, _, _) in P.items():
        assert sh.val().isValid(), n
    res = checks(P)
    for k, v in res.items():
        print(k, v)
    if os.path.exists(os.path.join(HERE, "docs", "V10_CANDIDATE.md")):
        print(export(P, res))
