"""RCWS Stage 1 upper mechanism - 2-axis Pan/Tilt gimbal, draft v0.2 (parametric, CadQuery).

Changes from v0.1
  * Pan support bearing 6806 (30x42x7): the yoke's weight and tipping moment go into the base;
    the pan servo only transmits torque through a pin-and-slot coupler that floats radially
    (no over-constraint between the bearing and the servo shaft).
  * Cable wrap channel around the bearing: camera FPC and the tilt-servo bus cable enter through
    holes in the turntable and wind/unwind in an annulus (clock-spring style) for ±150 deg.
  * Mechanical pan hard stop at about ±171 deg protects the cables; software limit ±150 deg.
  * Tilt servo turned so its body points backward (-X): out of the 102 deg wide-angle view, and
    its connectors face rearward where the cable drops to the turntable.
  * Pi 5 placed with micro-HDMI/USB-C edge on the -X deck edge and USB/Ethernet on the -Y edge,
    20 mm headroom above the AI HAT+ GPIO header for jumper wires; pillars moved off port zones.

Frame: X = boresight at zero (forward), Y = left, Z = up, origin = pan axis on the top-deck surface.
Every value marked [VERIFY] is an assumption to replace with a measurement.
"""
import json
import math
import os

import cadquery as cq

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out_v02")

# ----------------------------------------------------------------------------- purchased parts [mm]
SV_L, SV_W, SV_H = 45.2, 24.7, 35.0      # STS3215 case [VERIFY: sources disagree]
SV_SHAFT_FROM_END = 11.2                 # shaft centre from the near end [VERIFY]
SV_BOSS_H, SV_BOSS_D = 4.0, 9.0          # output boss [VERIFY]
SV_CONN_W, SV_CONN_H, SV_CONN_D = 16.0, 7.0, 6.0   # two 5264-3P sockets on the far end face [VERIFY]
HORN_D, HORN_T = 24.0, 4.0               # disc horn [VERIFY]

CAM_W, CAM_H, CAM_PCB = 25.0, 24.0, 1.0  # Camera Module 3 Wide [VERIFY holes/lens as Camera v2]
CAM_LENS_D, CAM_LENS_H = 8.5, 11.4
CAM_LENS_FROM_TOP = 9.46
CAM_HOLES = [(-10.5, -2.0), (10.5, -2.0), (-10.5, -14.5), (10.5, -14.5)]
FPC_W = 16.0                             # camera-end FPC width (15-pin)

PI_L, PI_W, PI_PCB = 85.0, 56.0, 1.6
PI_HOLES = [(3.5, 3.5), (61.5, 3.5), (3.5, 52.5), (61.5, 52.5)]  # local (x along 85, y along 56) from PCIe-end corner
PI_STACK_H = 30.0                        # PCB bottom -> AI HAT+ top [VERIFY]
PI_STANDOFF = 5.0
PI_PORTS = {                             # local x of port centres on the long edge y=0 [VERIFY]
    "USB-C": 11.2, "HDMI0": 25.8, "HDMI1": 39.2}
PI_GPIO_X = (7.0, 58.0)                  # 40-pin header span along the y=56 edge [VERIFY]

BRG_TILT = (16.0, 5.0, 5.0)              # 625ZZ  OD, ID, W
BRG_PAN = (42.0, 30.0, 7.0)              # 6806 (61806) OD, ID, W

# ----------------------------------------------------------------------------- layout [mm]
DECK_T = 5.0
DECK_X0, DECK_X1, DECK_Y = -100.0, 50.0, 110.0
PILLARS = [(-94.0, 49.0), (44.0, 49.0), (44.0, -49.0), (-30.0, -49.0)]   # kept off the port zones
BASE_GAP = 55.0                          # top-deck underside -> lower-deck top
LOWER_TOP = -DECK_T - BASE_GAP

SV_TOP = 0.0                             # pan servo case top flush with the deck surface (clears the channel floor)
PAN_HORN_Z = SV_TOP + SV_BOSS_H          # 4
TUBE_ID, TUBE_TOP = 27.0, 21.0           # fixed bearing tube (horn d24 turns inside it)
SHOULDER_R, SHOULDER_Z = 17.0, 14.0      # inner race seat
WRAP_R_OUT, WRAP_WALL = 44.0, 3.0        # cable channel outer wall
DISC_Z0, DISC_T, DISC_R = 22.0, 4.0, 56.0
COUPLER_PIN_R, COUPLER_PIN_D = 8.0, 3.0
STOP_R, STOP_HALF_W = 52.0, 6.0          # hard stop at 180 deg (rear)
HOLE_FPC = (180.0, 33.0)                 # turntable cable holes (angle deg, radius)
HOLE_BUS = (128.0, 36.0)
DECK_EXIT = (180.0, 35.0)                # where cables leave the channel downward

H_TILT = 58.0
ARM_W, ARM_T_SERVO, ARM_T_IDLE = 30.0, 4.0, 6.0
CRADLE_HALF_Y, CRADLE_WALL = 16.0, 2.5               # camera 25 wide + 0.5 clearance each side
CRADLE_X0, CRADLE_X1, CRADLE_HALF_Z = -13.0, 7.0, 18.0
CAM_BACK_X = -7.0

PAN_SOFT_LIMIT = 150.0


# ----------------------------------------------------------------------------- helpers
def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def cyl_y(r, y0, y1, x=0.0, z=0.0):
    return cq.Workplane("XZ").circle(r).extrude(-(y1 - y0)).translate((x, y0, z))


def cyl_z(r, z0, z1, x=0.0, y=0.0):
    return cq.Workplane("XY").circle(r).extrude(z1 - z0).translate((x, y, z0))


def ring_z(r_in, r_out, z0, z1, x=0.0, y=0.0):
    return cyl_z(r_out, z0, z1, x, y).cut(cyl_z(r_in, z0 - 1, z1 + 1, x, y))


def polar(a_deg, r):
    a = math.radians(a_deg)
    return r * math.cos(a), r * math.sin(a)


def arc_slot(a_deg, r, tang, rad, z0, z1):
    """Rectangular slot centred at polar (a, r): tang along the tangent, rad along the radius."""
    x, y = polar(a_deg, r)
    return box(-rad / 2, rad / 2, -tang / 2, tang / 2, z0, z1).rotate((0, 0, 0), (0, 0, 1), a_deg).translate((x, y, 0))


# ----------------------------------------------------------------------------- purchased-part envelopes
def servo_z_up():
    """Shaft on +Z at (0,0); case top face at z=0; L along +x; connectors on the far end (+x)."""
    body = box(-SV_SHAFT_FROM_END, SV_L - SV_SHAFT_FROM_END, -SV_W / 2, SV_W / 2, -SV_H, 0).edges("|Z").fillet(2.0)
    boss = cyl_z(SV_BOSS_D / 2, 0, SV_BOSS_H)
    xe = SV_L - SV_SHAFT_FROM_END
    conn = box(xe, xe + 2.0, -SV_CONN_W / 2, SV_CONN_W / 2, -SV_H + 4, -SV_H + 4 + SV_CONN_H)
    return body.union(boss).union(conn)


def servo_conn_keepout_z_up():
    """Plug + bend room in front of the connector face (not printed, used for checks)."""
    xe = SV_L - SV_SHAFT_FROM_END + 2.0
    return box(xe, xe + 14.0, -SV_CONN_W / 2 - 1, SV_CONN_W / 2 + 1, -SV_H + 2, -SV_H + 6 + SV_CONN_H)


def camera_module():
    top = CAM_LENS_FROM_TOP
    pcb = box(0, CAM_PCB, -CAM_W / 2, CAM_W / 2, top - CAM_H, top)
    lens = cq.Workplane("YZ").circle(CAM_LENS_D / 2).extrude(CAM_LENS_H).translate((CAM_PCB, 0, 0))
    housing = box(CAM_PCB, CAM_PCB + 3.0, -6, 6, -6, 6)
    conn = box(-2.5, 0, -10.5, 10.5, top - CAM_H, top - CAM_H + 5.5)
    return pcb.union(housing).union(lens).union(conn)


def pi_origin():
    """Deck coords of the Pi local origin. Local x (85, PCIe->USB) maps to deck -y, local y (56) to +x."""
    return (DECK_X0 + 6.0, PI_L / 2)        # HDMI edge 6 mm inside the -X deck edge


def pi_to_deck(lx, ly):
    ox, oy = pi_origin()
    return ox + ly, oy - lx


def pi_stack():
    ox, oy = pi_origin()
    z0 = LOWER_TOP + PI_STANDOFF
    pcb = box(ox, ox + PI_W, oy - PI_L, oy, z0, z0 + PI_PCB)
    stack = box(ox + 2, ox + PI_W - 2, oy - PI_L + 20, oy - 2, z0 + PI_PCB, z0 + PI_STACK_H)
    usb = box(ox + 1, ox + PI_W - 1, oy - PI_L - 3, oy - PI_L + 20, z0 + PI_PCB, z0 + PI_PCB + 16)
    return pcb.union(stack).union(usb)


def pi_keepouts():
    """Plug and wiring zones that must stay clear (ghost volumes)."""
    ox, oy = pi_origin()
    z0 = LOWER_TOP + PI_STANDOFF + PI_PCB
    zones = {}
    for name, lx in PI_PORTS.items():
        x, y = pi_to_deck(lx, 0)
        w = 9.5 if name == "USB-C" else 11.0
        zones[name] = box(x - 30, x, y - w / 2, y + w / 2, z0, z0 + 7)
    zones["USB/Ethernet"] = box(ox + 1, ox + PI_W - 1, oy - PI_L - 33, oy - PI_L - 3, z0, z0 + 16)
    gx0, gy0 = pi_to_deck(PI_GPIO_X[0], PI_W - 5)
    gx1, gy1 = pi_to_deck(PI_GPIO_X[1], PI_W - 1)
    top = LOWER_TOP + PI_STANDOFF + PI_STACK_H
    zones["GPIO jumpers"] = box(min(gx0, gx1), max(gx0, gx1) + 2, min(gy0, gy1), max(gy0, gy1), top, top + 18)
    return zones


# ----------------------------------------------------------------------------- printed parts: base
def top_deck():
    deck = box(DECK_X0, DECK_X1, -DECK_Y / 2, DECK_Y / 2, -DECK_T, 0).edges("|Z").fillet(6.0)
    c = 0.4
    deck = deck.cut(box(-SV_SHAFT_FROM_END - c, SV_L - SV_SHAFT_FROM_END + c, -SV_W / 2 - c, SV_W / 2 + c, -DECK_T - 1, 1))
    ex, ey = polar(*DECK_EXIT)
    deck = deck.cut(box(ex - 6, ex + 6, ey - (FPC_W + 4) / 2, ey + (FPC_W + 4) / 2, -DECK_T - 1, 1))  # cable drop
    for x, y in PILLARS:
        deck = deck.cut(cyl_z(1.7, -DECK_T - 1, 1, x, y))
    for a in (45, 135, 225, 315):  # bearing-mount screws (M3) on r = 39 inside the channel floor
        x, y = polar(a, 39.0)
        deck = deck.cut(cyl_z(1.7, -DECK_T - 1, 1, x, y))
    # Stage-2 interface: 4x M4 on a 70 mm square around the pan axis [VERIFY]
    for sx in (-35, 35):
        for sy in (-35, 35):
            if abs(sx - ex) < 10 and abs(sy - ey) < 15:
                continue
            deck = deck.cut(cyl_z(2.2, -DECK_T - 1, 1, sx, sy))
    return deck


def bearing_mount():
    """Fixed: tube carrying the 6806 inner race + outer wall of the cable channel + hard-stop block."""
    tube = ring_z(TUBE_ID / 2, BRG_PAN[1] / 2, 0, TUBE_TOP)                        # race fit d30
    shoulder = ring_z(TUBE_ID / 2, SHOULDER_R, 0, SHOULDER_Z)                       # inner-race seat
    floor = ring_z(TUBE_ID / 2, WRAP_R_OUT + WRAP_WALL, 0, 2.0)
    wall = ring_z(WRAP_R_OUT, WRAP_R_OUT + WRAP_WALL, 0, TUBE_TOP)
    m = tube.union(shoulder).union(floor).union(wall)
    # cable drop through the floor (matches the deck slot)
    ex, ey = polar(*DECK_EXIT)
    m = m.cut(box(ex - 6, ex + 6, ey - (FPC_W + 4) / 2, ey + (FPC_W + 4) / 2, -1, 3))
    for a in (45, 135, 225, 315):
        x, y = polar(a, 39.0)
        m = m.cut(cyl_z(1.7, -1, 3, x, y))
    # hard-stop block outside the wall at 180 deg
    blk = box(STOP_R - 4, STOP_R + 4, -STOP_HALF_W, STOP_HALF_W, 0, 15).rotate((0, 0, 0), (0, 0, 1), 180)
    m = m.union(blk).union(box(-(STOP_R - 4), -(WRAP_R_OUT + WRAP_WALL) + 0.5, -STOP_HALF_W, STOP_HALF_W, 0, 6))
    return m


def pillars():
    p = None
    for x, y in PILLARS:
        s = cyl_z(4.0, LOWER_TOP, -DECK_T, x, y).cut(cyl_z(1.4, LOWER_TOP - 1, 0, x, y))
        p = s if p is None else p.union(s)
    return p


def lower_deck():
    deck = box(DECK_X0, DECK_X1, -DECK_Y / 2, DECK_Y / 2, LOWER_TOP - DECK_T, LOWER_TOP).edges("|Z").fillet(6.0)
    for x, y in PILLARS:
        deck = deck.cut(cyl_z(1.7, LOWER_TOP - DECK_T - 1, LOWER_TOP + 1, x, y))
    for lx, ly in PI_HOLES:
        x, y = pi_to_deck(lx, ly)
        deck = deck.union(cyl_z(3.0, LOWER_TOP, LOWER_TOP + PI_STANDOFF, x, y))
        deck = deck.cut(cyl_z(1.25, LOWER_TOP - DECK_T - 1, LOWER_TOP + PI_STANDOFF + 1, x, y))
    ox, oy = pi_origin()
    for i in range(5):  # vents under the cooler
        deck = deck.cut(box(ox + 10 + i * 8, ox + 14 + i * 8, oy - 70, oy - 20, LOWER_TOP - DECK_T - 1, LOWER_TOP + 1))
    # cable tie slots along the +X edge (servo bus / UART to the STM32 off-board)
    for y in (-20, 0, 20):
        deck = deck.cut(box(DECK_X1 - 10, DECK_X1 - 7, y - 2, y + 2, LOWER_TOP - DECK_T - 1, LOWER_TOP + 1))
    return deck


def pan_cage():
    c, t = 0.3, 3.0
    x0, x1 = -SV_SHAFT_FROM_END - c, SV_L - SV_SHAFT_FROM_END + c
    y0, y1 = -SV_W / 2 - c, SV_W / 2 + c
    z0, z1 = SV_TOP - SV_H - t, -DECK_T
    cage = box(x0 - t, x1 + t, y0 - t, y1 + t, z0, z1).cut(box(x0, x1, y0, y1, z0 + t, z1 + 1))
    flange = box(x0 - t - 8, x1 + t + 8, y0 - t, y1 + t, -DECK_T - 3, -DECK_T).cut(box(x0, x1, y0, y1, -DECK_T - 4, 0))
    for x in (x0 - t - 4, x1 + t + 4):
        flange = flange.cut(cyl_z(1.7, -DECK_T - 4, 0, x, 0))
    # open end wall for the two bus connectors (+X)
    cage = cage.union(flange).cut(box(x1 - 1, x1 + t + 1, -SV_CONN_W / 2 - 1.5, SV_CONN_W / 2 + 1.5, z0 + t, z0 + t + SV_CONN_H + 6))
    return cage


# ----------------------------------------------------------------------------- printed parts: rotating
def horn_coupler():
    """Screwed to the pan horn; two pins drive radial slots in the turntable (radial float, no tangential play)."""
    z0, z1 = PAN_HORN_Z + HORN_T, TUBE_TOP - 1.0          # drive cup inside the fixed tube
    c = cyl_z(HORN_D / 2 - 0.5, z0, z1).cut(cyl_z(HORN_D / 2 - 3.0, z0 + 3.0, z1 + 1))
    for s in (-1, 1):
        c = c.union(cyl_z(COUPLER_PIN_D / 2 + 1.5, z1 - 4, z1, s * COUPLER_PIN_R, 0))
        c = c.union(cyl_z(COUPLER_PIN_D / 2, z1, DISC_Z0 + 3.0, s * COUPLER_PIN_R, 0))
    for a in range(4):
        x, y = polar(45 + 90 * a, 8.0)
        c = c.cut(cyl_z(1.7, z0 - 1, z0 + 4, x, y))
    c = c.cut(cyl_z(3.0, z0 - 1, z1 + 1))                 # horn centre screw access
    return c


def turntable_yoke():
    """Turntable disc + bearing housing + arms. Returns (solid, y_srv_in, y_srv_out, y_idle_in)."""
    y_srv_in = CRADLE_HALF_Y + HORN_T + 1.0
    y_idle_in = -(CRADLE_HALF_Y + 2.0)
    y_srv_out = y_srv_in + ARM_T_SERVO
    y_idle_out = y_idle_in - ARM_T_IDLE
    z_disc_top = DISC_Z0 + DISC_T
    disc = cyl_z(DISC_R, DISC_Z0, z_disc_top)
    # outer-race housing hanging below the disc: d42 bore, lip on top of the outer race only
    od, idr, w = BRG_PAN
    housing = ring_z(od / 2, od / 2 + 3.0, TUBE_TOP - w, DISC_Z0)
    lip = ring_z(idr / 2 + 3.5, od / 2 + 0.1, TUBE_TOP, DISC_Z0)  # touches outer race (r>18.5) only
    y = disc.union(housing).union(lip)
    # centre: relief over the tube/inner race, two radial slots for the coupler pins
    y = y.cut(cyl_z(idr / 2 + 3.4, TUBE_TOP - 0.01, DISC_Z0))
    for s in (-1, 1):
        y = y.cut(box(s * COUPLER_PIN_R - 2.0, s * COUPLER_PIN_R + 2.0, -(COUPLER_PIN_D + 0.1) / 2, (COUPLER_PIN_D + 0.1) / 2,
                      DISC_Z0 - 1, DISC_Z0 + 3.2))
    y = y.cut(cyl_z(4.5, DISC_Z0 + 3.1, z_disc_top + 1))  # screwdriver access to the horn screw
    # arms
    arm_s = box(-ARM_W / 2, ARM_W / 2, y_srv_in, y_srv_out, z_disc_top - 0.1, H_TILT).union(
        cyl_y(ARM_W / 2, y_srv_in, y_srv_out, 0, H_TILT))
    arm_i = box(-ARM_W / 2, ARM_W / 2, y_idle_out, y_idle_in, z_disc_top - 0.1, H_TILT).union(
        cyl_y(ARM_W / 2, y_idle_out, y_idle_in, 0, H_TILT))
    # gussets
    for yy0, yy1 in ((y_srv_out, y_srv_out + 6), (y_idle_out - 6, y_idle_out)):
        g = (cq.Workplane("YZ").polyline([(0, 0), (6, 0), (0, 14)]).close().extrude(ARM_W - 6)
             .translate((-(ARM_W - 6) / 2, 0, 0)))
        if yy0 > 0:
            g = g.translate((0, yy0, z_disc_top))
        else:
            g = g.mirror("XZ").translate((0, yy1, z_disc_top))
        y = y.union(g)
    y = y.union(arm_s).union(arm_i)
    # tilt servo boss through the servo arm, case screws [VERIFY pattern], 625 pocket on the idle arm
    y = y.cut(cyl_y(SV_BOSS_D / 2 + 1.0, y_srv_in - 1, y_srv_out + 1, 0, H_TILT))
    for dx, dz in ((8, 8), (8, -8), (-24, 8), (-24, -8)):
        y = y.cut(cyl_y(1.4, y_srv_in - 1, y_srv_out + 1, dx, H_TILT + dz))
    bod, bid, bw = BRG_TILT
    y = y.cut(cyl_y(bod / 2 + 0.1, y_idle_in - bw, y_idle_in + 0.1, 0, H_TILT))
    y = y.cut(cyl_y(bid / 2 + 2.0, y_idle_out - 1, y_idle_in, 0, H_TILT))
    # cable entries into the wrap channel
    y = y.cut(arc_slot(HOLE_FPC[0], HOLE_FPC[1], FPC_W + 3, 6.0, DISC_Z0 - 1, z_disc_top + 1))
    y = y.cut(arc_slot(HOLE_BUS[0], HOLE_BUS[1], 10.0, 7.0, DISC_Z0 - 1, z_disc_top + 1))
    # hard-stop pin at 0 deg (front) on r = STOP_R, reaching down beside the block
    y = y.union(cyl_z(3.0, 9.0, DISC_Z0 + 0.1, STOP_R, 0))
    # lightening pockets in the disc (keep the ring under the arms solid)
    for a in (40, -40, 155, -140, -60, 60):
        if abs(a - HOLE_BUS[0]) < 20 or abs(a - HOLE_FPC[0]) < 20:
            continue
        x, yy = polar(a, 44.0)
        y = y.cut(cyl_z(7.0, DISC_Z0 - 1, z_disc_top + 1, x, yy))
    # cable clip under the tilt servo (bus cable runs down the back of the servo arm)
    clip = box(-24, -18, y_srv_out, y_srv_out + 4, z_disc_top, z_disc_top + 10).cut(
        box(-23, -19, y_srv_out - 1, y_srv_out + 5, z_disc_top + 3, z_disc_top + 7))
    y = y.union(clip)
    return y, y_srv_in, y_srv_out, y_idle_in


def tilt_cradle():
    zc, w = H_TILT, CRADLE_WALL
    back = box(CRADLE_X0, CRADLE_X0 + w, -CRADLE_HALF_Y, CRADLE_HALF_Y, zc - CRADLE_HALF_Z, zc + CRADLE_HALF_Z)
    side_p = box(CRADLE_X0, CRADLE_X1, CRADLE_HALF_Y - w, CRADLE_HALF_Y, zc - CRADLE_HALF_Z, zc + CRADLE_HALF_Z)
    side_n = box(CRADLE_X0, CRADLE_X1, -CRADLE_HALF_Y, -CRADLE_HALF_Y + w, zc - CRADLE_HALF_Z, zc + CRADLE_HALF_Z)
    cr = back.union(side_p).union(side_n)
    cr = cr.union(cyl_y(HORN_D / 2, CRADLE_HALF_Y - w, CRADLE_HALF_Y, 0, zc))
    for a in range(4):
        x, z = polar(45 + 90 * a, 8.0)
        cr = cr.cut(cyl_y(1.7, CRADLE_HALF_Y - w - 1, CRADLE_HALF_Y + 1, x, zc + z))
    bod, bid, bw = BRG_TILT
    cr = cr.union(cyl_y(bid / 2, -(CRADLE_HALF_Y + 2.0 + bw), -CRADLE_HALF_Y, 0, zc))
    cr = cr.union(cyl_y(4.0, -(CRADLE_HALF_Y + 1.9), -CRADLE_HALF_Y, 0, zc))
    top = zc + CAM_LENS_FROM_TOP
    for hy, hz in CAM_HOLES:
        z = top + hz
        cr = cr.union(cq.Workplane("YZ").circle(2.2).extrude(CAM_BACK_X - (CRADLE_X0 + w)).translate((CRADLE_X0 + w, hy, z)))
        cr = cr.cut(cq.Workplane("YZ").circle(0.9).extrude(30).translate((CRADLE_X0 - 1, hy, z)))
    # FPC leaves the camera bottom edge and goes straight back through the back plate
    cr = cr.cut(box(CRADLE_X0 - 1, CRADLE_X0 + w + 1, -(FPC_W + 2) / 2, (FPC_W + 2) / 2, zc - CRADLE_HALF_Z - 1, top - CAM_H + 4))
    return cr


# ----------------------------------------------------------------------------- cable routes (polylines, for length and display)
def cable_routes(y_srv_out):
    zc = H_TILT
    fx, fy = polar(*HOLE_FPC)
    ex, ey = polar(*DECK_EXIT)
    bx, by = polar(*HOLE_BUS)
    pi_cam = pi_to_deck(48.0, 4.0)  # CAM/DISP connector [VERIFY]
    hat_top = LOWER_TOP + PI_STANDOFF + PI_STACK_H
    fpc = [(CRADLE_X0 + 1, 0, zc - CRADLE_HALF_Z + 4), (CRADLE_X0 - 6, 0, zc - CRADLE_HALF_Z - 2),  # out of the cradle
           (CRADLE_X0 - 14, 0, zc - 26), (fx + 3, fy, DISC_Z0 + DISC_T + 2),                          # service loop for tilt
           (fx, fy, DISC_Z0 - 2)]
    wrap_r = WRAP_R_OUT - 4
    fpc += [(*polar(180 + 30 * k, wrap_r), 10) for k in (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12)]  # one turn of slack
    fpc += [(ex, ey, 6), (ex, ey, -DECK_T - 3), (pi_cam[0], ex * 0 + pi_cam[1] * 0 + ey, hat_top + 6),
            (pi_cam[0], pi_cam[1], hat_top + 6), (pi_cam[0], pi_cam[1], LOWER_TOP + PI_STANDOFF + 3)]
    srv_end = (-(SV_L - SV_SHAFT_FROM_END) - 2, y_srv_out + SV_H - 6, zc - 6)
    bus = [srv_end, (-30, y_srv_out + 10, zc - 18), (-21, y_srv_out + 2, DISC_Z0 + DISC_T + 5), (bx, by, DISC_Z0 + DISC_T + 2),
           (bx, by, DISC_Z0 - 2)]
    bus += [(*polar(HOLE_BUS[0] + 30 * k, wrap_r - 3), 6) for k in range(1, 12)]
    pan_conn = (SV_L - SV_SHAFT_FROM_END + 8, 0, SV_TOP - SV_H + 8)
    bus += [(ex + 2, ey + 6, 6), (ex + 2, ey + 6, -DECK_T - 3), (-20, 20, -DECK_T - 6), (pan_conn[0], 18, -DECK_T - 6),
            pan_conn]
    out = [pan_conn, (DECK_X1 - 8, 0, SV_TOP - SV_H + 8), (DECK_X1 - 8, 0, LOWER_TOP + 3), (DECK_X1 + 25, 0, LOWER_TOP + 3)]
    gx, gy = pi_to_deck(sum(PI_GPIO_X) / 2, PI_W - 3)
    uart = [(gx, gy, hat_top + 8), (gx + 8, gy, hat_top + 14), (DECK_X1 - 12, 10, hat_top + 14),
            (DECK_X1 - 8, 10, LOWER_TOP + 3), (DECK_X1 + 25, 10, LOWER_TOP + 3)]
    return {"camera FPC": fpc, "tilt servo bus": bus, "servo bus to STM32": out, "UART to STM32": uart}


def length(poly):
    return sum(math.dist(a, b) for a, b in zip(poly, poly[1:]))


# ----------------------------------------------------------------------------- assembly
def build(export=True):
    yoke, y_srv_in, y_srv_out, y_idle_in = turntable_yoke()
    parts = {}
    parts["top_deck"] = (top_deck(), (0.80, 0.82, 0.85), True, "base")
    parts["bearing_mount"] = (bearing_mount(), (0.55, 0.57, 0.60), True, "base")
    parts["lower_deck"] = (lower_deck(), (0.80, 0.82, 0.85), True, "base")
    parts["pillars"] = (pillars(), (0.55, 0.57, 0.60), True, "base")
    parts["pan_cage"] = (pan_cage(), (0.55, 0.57, 0.60), True, "base")
    parts["pan_servo"] = (servo_z_up().translate((0, 0, SV_TOP)), (0.12, 0.12, 0.13), False, "base")
    parts["pi5_stack"] = (pi_stack(), (0.15, 0.55, 0.30), False, "base")
    od, idr, w = BRG_PAN
    parts["bearing_6806"] = (ring_z(idr / 2, od / 2, TUBE_TOP - w, TUBE_TOP), (0.75, 0.75, 0.78), False, "base")
    parts["pan_horn"] = (cyl_z(HORN_D / 2, PAN_HORN_Z, PAN_HORN_Z + HORN_T), (0.70, 0.70, 0.72), False, "pan")
    parts["horn_coupler"] = (horn_coupler(), (0.95, 0.75, 0.20), True, "pan")
    parts["turntable_yoke"] = (yoke, (0.20, 0.45, 0.85), True, "pan")
    # tilt servo: shaft -Y at (0, H_TILT); case outside the +Y arm; body points backward (-X)
    ts = servo_z_up().rotate((0, 0, 0), (1, 0, 0), 90).rotate((0, 0, 0), (0, 1, 0), 180).translate((0, y_srv_out, H_TILT))
    parts["tilt_servo"] = (ts, (0.12, 0.12, 0.13), False, "pan")
    bod, bid, bw = BRG_TILT
    parts["bearing_625"] = (cyl_y(bod / 2, y_idle_in - bw, y_idle_in, 0, H_TILT).cut(
        cyl_y(bid / 2, y_idle_in - bw - 1, y_idle_in + 1, 0, H_TILT)), (0.75, 0.75, 0.78), False, "pan")
    parts["tilt_horn"] = (cyl_y(HORN_D / 2, CRADLE_HALF_Y, CRADLE_HALF_Y + HORN_T, 0, H_TILT), (0.70, 0.70, 0.72), False, "tilt")
    parts["tilt_cradle"] = (tilt_cradle(), (0.95, 0.55, 0.15), True, "tilt")
    parts["camera"] = (camera_module().translate((CAM_BACK_X, 0, H_TILT)), (0.10, 0.45, 0.25), False, "tilt")

    keepouts = {f"keepout {k}": v for k, v in pi_keepouts().items()}
    keepouts["keepout pan servo plugs"] = servo_conn_keepout_z_up().translate((0, 0, SV_TOP))
    keepouts["keepout tilt servo plugs"] = (servo_conn_keepout_z_up().rotate((0, 0, 0), (1, 0, 0), 90)
                                            .rotate((0, 0, 0), (0, 1, 0), 180).translate((0, y_srv_out, H_TILT)))
    routes = cable_routes(y_srv_out)

    if export:
        os.makedirs(OUT, exist_ok=True)
        assy = cq.Assembly(name="rcws_gimbal_v0_2")
        for name, (shape, rgb, _, _) in parts.items():
            assy.add(shape, name=name, color=cq.Color(*rgb))
        assy.export(os.path.join(OUT, "rcws_gimbal_v0_2.step"))
        for name, (shape, _, printable, _) in parts.items():
            if printable:
                cq.exporters.export(shape, os.path.join(OUT, f"{name}.stl"), tolerance=0.05, angularTolerance=0.1)
        meta = {
            "H_TILT": H_TILT, "pan_soft_limit_deg": PAN_SOFT_LIMIT,
            "cable_lengths_mm": {k: round(length(v)) for k, v in routes.items()},
            "parts": {n: {"printed": p[2], "group": p[3], "volume_cm3": round(p[0].val().Volume() / 1000, 1)} for n, p in parts.items()},
        }
        with open(os.path.join(OUT, "meta.json"), "w") as f:
            json.dump(meta, f, indent=1)
    return parts, keepouts, routes


if __name__ == "__main__":
    build()
    print("written to", OUT)
