"""RCWS Stage 1 upper mechanism - 2-axis Pan/Tilt gimbal, draft v0.1 (parametric, CadQuery).

Frame: X = boresight at zero (forward), Y = left, Z = up. Origin = pan axis on the top-deck surface.
Pan axis = Z through origin. Tilt axis = Y at height H_TILT. The camera lens centre sits on the
intersection of both axes so the LOS offset d (parallax, theory doc 1.3) is ~0 by construction.

Every dimension marked [VERIFY] is an assumption to be replaced with a caliper measurement.
Run:  python3 gimbal.py   -> out/ (STEP assembly, STL per printable part, GLB for the viewer)
"""
import json
import math
import os

import cadquery as cq

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")

# ----------------------------------------------------------------------------- parameters [mm]
# Feetech STS3215 (sources disagree: 45.2x24.7x35 vs 40.5x20x38) [VERIFY]
SV_L, SV_W, SV_H = 45.2, 24.7, 35.0      # body: length (along case), width, height (shaft axis)
SV_SHAFT_FROM_END = 11.2                 # output shaft centre from the near end along L [VERIFY]
SV_BOSS_H, SV_BOSS_D = 4.0, 9.0          # output boss above the case [VERIFY]
HORN_D, HORN_T = 24.0, 4.0               # disc horn [VERIFY]

# Raspberry Pi Camera Module 3 Wide (board 25x24, 12.4 high) - hole/lens positions as Camera v2 [VERIFY]
CAM_W, CAM_H, CAM_PCB = 25.0, 24.0, 1.0
CAM_LENS_D, CAM_LENS_H = 8.5, 11.4       # lens barrel above the PCB front face
CAM_LENS_FROM_TOP = 9.46                 # lens centre below the top edge
CAM_HOLES = [(-10.5, -2.0), (10.5, -2.0), (-10.5, -14.5), (10.5, -14.5)]  # (y, z from top edge), d2.2

# Raspberry Pi 5 + Active Cooler + AI HAT+ (Hailo-8) stack envelope [VERIFY stack height]
PI_L, PI_W, PI_PCB = 85.0, 56.0, 1.6
PI_HOLES = [(3.5, 3.5), (61.5, 3.5), (3.5, 52.5), (61.5, 52.5)]  # from the corner near the 40-pin header end
PI_STACK_H = 30.0                        # PCB bottom -> top of AI HAT+ (cooler + 16 mm spacers) [VERIFY]
PI_STANDOFF = 5.0

# Bearing for the passive tilt pivot
BRG_OD, BRG_ID, BRG_W = 16.0, 5.0, 5.0   # 625ZZ

# Layout
DECK_T = 5.0
DECK_X0, DECK_X1, DECK_Y = -94.0, 44.0, 104.0   # top deck extent (pan axis at x=0)
BASE_GAP = 43.0                                  # top-deck underside -> lower-deck top
H_TILT = 52.0                                    # tilt axis height above the top deck
YOKE_T = 5.0
ARM_T_SERVO, ARM_T_IDLE = 4.0, 6.0
ARM_W = 30.0
CRADLE_HALF_Y = 20.0                              # cradle side-wall outer face |y|
CRADLE_WALL = 3.0
CRADLE_X0, CRADLE_X1 = -13.0, 7.0                 # cradle extent along x (around the axis)
CRADLE_HALF_Z = 18.0

# derived heights
SV_TOP = 2.0                                      # pan servo case top above the deck surface
PAN_HORN_Z = SV_TOP + SV_BOSS_H                   # horn bottom
YOKE_Z0 = PAN_HORN_Z + HORN_T                     # yoke plate bottom
LOWER_DECK_TOP = -DECK_T - BASE_GAP
CAM_BACK_X = -7.0                                 # camera PCB back face (lens front ~ +5.4)


def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def cyl_y(r, y0, y1, x=0.0, z=0.0):
    return cq.Workplane("XZ").circle(r).extrude(-(y1 - y0)).translate((x, y0, z))


def cyl_z(r, z0, z1, x=0.0, y=0.0):
    return cq.Workplane("XY").circle(r).extrude(z1 - z0).translate((x, y, z0))


# ----------------------------------------------------------------------------- reference parts (not printed)
def servo_body_z_up():
    """STS3215 with its shaft on +Z at (0,0); case below z=0 (top face at z=0), L along +x."""
    body = box(-SV_SHAFT_FROM_END, SV_L - SV_SHAFT_FROM_END, -SV_W / 2, SV_W / 2, -SV_H, 0)
    body = body.edges("|Z").fillet(2.0)
    boss = cyl_z(SV_BOSS_D / 2, 0, SV_BOSS_H)
    return body.union(boss)


def horn_z_up():
    return cyl_z(HORN_D / 2, 0, HORN_T)


def camera_module():
    """Camera Module 3 Wide, lens on +X, PCB in the YZ plane; lens centre at (0,0,0) in y/z."""
    top = CAM_LENS_FROM_TOP
    pcb = box(0, CAM_PCB, -CAM_W / 2, CAM_W / 2, top - CAM_H, top)
    lens = cq.Workplane("YZ").circle(CAM_LENS_D / 2).extrude(CAM_LENS_H).translate((CAM_PCB, 0, 0))
    lens_block = box(CAM_PCB, CAM_PCB + 3.0, -6, 6, -6, 6)  # sensor housing
    conn = box(-2.5, 0, -10.5, 10.5, top - CAM_H, top - CAM_H + 5.5)  # FPC connector on the back, bottom edge
    return pcb.union(lens_block).union(lens).union(conn)


def pi_stack():
    """Pi 5 + cooler + AI HAT+ as an envelope; PCB bottom at z=0, long side along +y."""
    pcb = box(0, PI_W, 0, PI_L, 0, PI_PCB)
    stack = box(2, PI_W - 2, 2, PI_L - 20, PI_PCB, PI_STACK_H)           # cooler + HAT
    ports = box(1, PI_W - 1, PI_L - 20, PI_L + 2, PI_PCB, PI_PCB + 16)   # USB/Ethernet block
    return pcb.union(stack).union(ports)


# ----------------------------------------------------------------------------- printable parts
def top_deck():
    deck = box(DECK_X0, DECK_X1, -DECK_Y / 2, DECK_Y / 2, -DECK_T, 0)
    deck = deck.edges("|Z").fillet(6.0)
    # pan servo pocket (case passes through, clamped by the cage below)
    c = 0.4
    deck = deck.cut(box(-SV_SHAFT_FROM_END - c, SV_L - SV_SHAFT_FROM_END + c, -SV_W / 2 - c, SV_W / 2 + c, -DECK_T - 1, 1))
    # camera FPC slot next to the pan axis
    deck = deck.cut(box(-24, -16, -11, 11, -DECK_T - 1, 1))
    # corner pillar holes (M3)
    for x, y in pillar_xy():
        deck = deck.cut(cyl_z(1.7, -DECK_T - 1, 1, x, y))
    # lower-platform interface: 4x M4 on 60 mm square around the pan axis [VERIFY with Stage 2 platform]
    return deck


def pillar_xy():
    return [(DECK_X0 + 8, -DECK_Y / 2 + 8), (DECK_X0 + 8, DECK_Y / 2 - 8),
            (DECK_X1 - 8, -DECK_Y / 2 + 8), (DECK_X1 - 8, DECK_Y / 2 - 8)]


def pillars():
    p = None
    for x, y in pillar_xy():
        s = cyl_z(4.0, LOWER_DECK_TOP, -DECK_T, x, y).cut(cyl_z(1.4, LOWER_DECK_TOP - 1, 0, x, y))
        p = s if p is None else p.union(s)
    return p


def lower_deck():
    deck = box(DECK_X0, DECK_X1, -DECK_Y / 2, DECK_Y / 2, LOWER_DECK_TOP - DECK_T, LOWER_DECK_TOP)
    deck = deck.edges("|Z").fillet(6.0)
    for x, y in pillar_xy():
        deck = deck.cut(cyl_z(1.7, LOWER_DECK_TOP - DECK_T - 1, LOWER_DECK_TOP + 1, x, y))
    # Pi standoffs
    px0, py0 = pi_origin()
    for hx, hy in PI_HOLES:
        # Pi is placed with its 56 mm side along x and 85 mm side along y
        x, y = px0 + hy, py0 + hx
        deck = deck.union(cyl_z(3.0, LOWER_DECK_TOP, LOWER_DECK_TOP + PI_STANDOFF, x, y))
        deck = deck.cut(cyl_z(1.25, LOWER_DECK_TOP - DECK_T - 1, LOWER_DECK_TOP + PI_STANDOFF + 1, x, y))
    # ventilation slots under the Pi
    for i in range(5):
        deck = deck.cut(box(px0 + 10 + i * 8, px0 + 14 + i * 8, py0 + 15, py0 + 60, LOWER_DECK_TOP - DECK_T - 1, LOWER_DECK_TOP + 1))
    return deck


def pi_origin():
    """Lower-left corner of the Pi PCB (x, y): 56 mm along x, 85 mm along y, beside the pan servo."""
    return (-SV_SHAFT_FROM_END - 8 - PI_W, -PI_L / 2)


def pan_cage():
    """Sleeve around the pan servo case, screwed to the deck underside."""
    c, t = 0.3, 3.0
    x0, x1 = -SV_SHAFT_FROM_END - c, SV_L - SV_SHAFT_FROM_END + c
    y0, y1 = -SV_W / 2 - c, SV_W / 2 + c
    z0, z1 = SV_TOP - SV_H - t, -DECK_T
    outer = box(x0 - t, x1 + t, y0 - t, y1 + t, z0, z1)
    inner = box(x0, x1, y0, y1, z0 + t, z1 + 1)
    cage = outer.cut(inner)
    flange = box(x0 - t - 8, x1 + t + 8, y0 - t, y1 + t, -DECK_T - 3, -DECK_T)
    flange = flange.cut(box(x0, x1, y0, y1, -DECK_T - 4, 0))
    for x in (x0 - t - 4, x1 + t + 4):
        flange = flange.cut(cyl_z(1.7, -DECK_T - 4, 0, x, 0))
    # cable exit at the far end
    cage = cage.union(flange).cut(box(x1 - 1, x1 + t + 1, -6, 6, z0 + t, z0 + t + 10))
    return cage


def pan_yoke():
    """Rotates with the pan horn: plate + servo arm (+Y) + idle arm (-Y) with 625 bearing pocket."""
    y_srv_in = CRADLE_HALF_Y + HORN_T + 1.0           # inner face of the servo arm
    y_idle_in = -(CRADLE_HALF_Y + 2.0)                # inner face of the idle arm
    y_srv_out = y_srv_in + ARM_T_SERVO
    y_idle_out = y_idle_in - ARM_T_IDLE
    plate = box(-ARM_W / 2, ARM_W / 2, y_idle_out, y_srv_out, YOKE_Z0, YOKE_Z0 + YOKE_T)
    plate = plate.union(cyl_z(ARM_W / 2 + 6, YOKE_Z0, YOKE_Z0 + YOKE_T))
    z_top = H_TILT + 16
    arm_s = box(-ARM_W / 2, ARM_W / 2, y_srv_in, y_srv_out, YOKE_Z0, H_TILT)
    arm_s = arm_s.union(cyl_y(ARM_W / 2, y_srv_in, y_srv_out, 0, H_TILT))
    arm_i = box(-ARM_W / 2, ARM_W / 2, y_idle_out, y_idle_in, YOKE_Z0, H_TILT)
    arm_i = arm_i.union(cyl_y(ARM_W / 2, y_idle_out, y_idle_in, 0, H_TILT))
    yoke = plate.union(arm_s).union(arm_i)
    # tilt servo: boss passes through the servo arm
    yoke = yoke.cut(cyl_y(SV_BOSS_D / 2 + 1.0, y_srv_in - 1, y_srv_out + 1, 0, H_TILT))
    # tilt servo case screws into the arm (4x M2.5) [VERIFY hole pattern]
    for dx, dz in ((-8, 8), (8, 8), (-8, -24), (8, -24)):
        yoke = yoke.cut(cyl_y(1.4, y_srv_in - 1, y_srv_out + 1, dx, H_TILT + dz))
    # 625ZZ pocket (open to the inside) + through hole
    yoke = yoke.cut(cyl_y(BRG_OD / 2 + 0.1, y_idle_in - BRG_W, y_idle_in + 0.1, 0, H_TILT))
    yoke = yoke.cut(cyl_y(BRG_ID / 2 + 2.0, y_idle_out - 1, y_idle_in, 0, H_TILT))
    # pan horn screws (4x M3 on PCD 16) + centre clearance
    yoke = yoke.cut(cyl_z(4.0, YOKE_Z0 - 1, YOKE_Z0 + YOKE_T + 1))
    for a in range(4):
        r = 8.0
        yoke = yoke.cut(cyl_z(1.7, YOKE_Z0 - 1, YOKE_Z0 + YOKE_T + 1, r * math.cos(a * math.pi / 2 + math.pi / 4),
                              r * math.sin(a * math.pi / 2 + math.pi / 4)))
    # FPC window in the plate, behind the axis
    yoke = yoke.cut(box(-14, -8, -11, 11, YOKE_Z0 - 1, YOKE_Z0 + YOKE_T + 1))
    return yoke, y_srv_in, y_srv_out, y_idle_in


def tilt_cradle(y_srv_in):
    """Camera carrier rotating about the tilt axis; side walls at |y| = CRADLE_HALF_Y."""
    zc = H_TILT
    w = CRADLE_WALL
    back = box(CRADLE_X0, CRADLE_X0 + w, -CRADLE_HALF_Y, CRADLE_HALF_Y, zc - CRADLE_HALF_Z, zc + CRADLE_HALF_Z)
    side_p = box(CRADLE_X0, CRADLE_X1, CRADLE_HALF_Y - w, CRADLE_HALF_Y, zc - CRADLE_HALF_Z, zc + CRADLE_HALF_Z)
    side_n = box(CRADLE_X0, CRADLE_X1, -CRADLE_HALF_Y, -CRADLE_HALF_Y + w, zc - CRADLE_HALF_Z, zc + CRADLE_HALF_Z)
    cr = back.union(side_p).union(side_n)
    # horn interface on +Y wall: hub disc meets the tilt horn
    cr = cr.union(cyl_y(HORN_D / 2, CRADLE_HALF_Y - w, CRADLE_HALF_Y, 0, zc))
    for a in range(4):
        r = 8.0
        cr = cr.cut(cyl_y(1.7, CRADLE_HALF_Y - w - 1, CRADLE_HALF_Y + 1, r * math.cos(a * math.pi / 2 + math.pi / 4),
                          zc + r * math.sin(a * math.pi / 2 + math.pi / 4)))
    # stub shaft on -Y wall into the 625ZZ (printed d5, or an M5 shoulder bolt)
    cr = cr.union(cyl_y(BRG_ID / 2, -(CRADLE_HALF_Y + 2.0 + BRG_W), -CRADLE_HALF_Y, 0, zc))
    cr = cr.union(cyl_y(4.0, -(CRADLE_HALF_Y + 1.9), -CRADLE_HALF_Y, 0, zc))  # spacer collar (0.1 gap to the arm; bears on the inner race)
    # camera standoffs on the back plate (lens centre on the axis)
    top = zc + CAM_LENS_FROM_TOP
    for hy, hz in CAM_HOLES:
        z = top + hz
        cr = cr.union(cq.Workplane("YZ").circle(2.2).extrude(CAM_BACK_X - (CRADLE_X0 + w)).translate((CRADLE_X0 + w, hy, z)))
        cr = cr.cut(cq.Workplane("YZ").circle(0.9).extrude(30).translate((CRADLE_X0 - 1, hy, z)))
    # FPC slot through the back plate, bottom edge of the camera
    cr = cr.cut(box(CRADLE_X0 - 1, CRADLE_X0 + w + 1, -11, 11, zc - CRADLE_HALF_Z - 1, top - CAM_H + 3))
    return cr


# ----------------------------------------------------------------------------- assembly
def build():
    os.makedirs(OUT, exist_ok=True)
    yoke, y_srv_in, y_srv_out, y_idle_in = pan_yoke()

    parts = {}
    # printable
    parts["top_deck"] = (top_deck(), (0.80, 0.82, 0.85), True)
    parts["lower_deck"] = (lower_deck(), (0.80, 0.82, 0.85), True)
    parts["pillars"] = (pillars(), (0.55, 0.57, 0.60), True)
    parts["pan_cage"] = (pan_cage(), (0.55, 0.57, 0.60), True)
    parts["pan_yoke"] = (yoke, (0.20, 0.45, 0.85), True)
    parts["tilt_cradle"] = (tilt_cradle(y_srv_in), (0.95, 0.55, 0.15), True)
    # reference (purchased) parts
    pan_servo = servo_body_z_up().translate((0, 0, SV_TOP))
    parts["pan_servo"] = (pan_servo, (0.12, 0.12, 0.13), False)
    parts["pan_horn"] = (horn_z_up().translate((0, 0, PAN_HORN_Z)), (0.70, 0.70, 0.72), False)
    # tilt servo: shaft along -Y at (0, H_TILT); case outside the +Y arm, long side hanging down
    ts = servo_body_z_up().rotate((0, 0, 0), (1, 0, 0), 90)          # shaft +Z -> -Y
    ts = ts.rotate((0, 0, 0), (0, 1, 0), 90)                          # L along +x -> -z
    ts = ts.translate((0, y_srv_out, H_TILT))                          # case top face on the arm's outer face
    parts["tilt_servo"] = (ts, (0.12, 0.12, 0.13), False)
    th = cyl_y(HORN_D / 2, CRADLE_HALF_Y, CRADLE_HALF_Y + HORN_T, 0, H_TILT)
    parts["tilt_horn"] = (th, (0.70, 0.70, 0.72), False)
    brg = cyl_y(BRG_OD / 2, y_idle_in - BRG_W, y_idle_in, 0, H_TILT).cut(cyl_y(BRG_ID / 2, y_idle_in - BRG_W - 1, y_idle_in + 1, 0, H_TILT))
    parts["bearing_625"] = (brg, (0.75, 0.75, 0.78), False)
    parts["camera"] = (camera_module().translate((CAM_BACK_X, 0, H_TILT)), (0.10, 0.45, 0.25), False)
    px0, py0 = pi_origin()
    parts["pi5_stack"] = (pi_stack().translate((px0, py0, LOWER_DECK_TOP + PI_STANDOFF)), (0.15, 0.55, 0.30), False)

    assy = cq.Assembly(name="rcws_gimbal_v0_1")
    for name, (shape, rgb, _) in parts.items():
        assy.add(shape, name=name, color=cq.Color(*rgb))
    assy.save(os.path.join(OUT, "rcws_gimbal_v0_1.step"))
    assy.save(os.path.join(OUT, "rcws_gimbal_v0_1.glb"))
    for name, (shape, _, printable) in parts.items():
        if printable:
            cq.exporters.export(shape, os.path.join(OUT, f"{name}.stl"), tolerance=0.05, angularTolerance=0.1)

    # mass properties (PLA 1.24 g/cm3 for printed parts, catalogue masses for purchased parts)
    report = {}
    for name, (shape, _, printable) in parts.items():
        v = shape.val().Volume() / 1000.0  # cm3
        # printed mass: PLA, ~40 % effective fill (walls + 20 % infill) - rough
        report[name] = {"volume_cm3": round(v, 1), "printed": printable,
                        "mass_g_est": round(v * 1.24 * 0.40, 1) if printable else None}
    with open(os.path.join(OUT, "parts.json"), "w") as f:
        json.dump({"params": {k: v for k, v in globals().items() if k.isupper() and isinstance(v, (int, float))},
                   "parts": report}, f, indent=1)
    return parts


if __name__ == "__main__":
    build()
    print("written to", OUT)
