"""RCWS Stage 1 upper mechanism - 2-axis Pan/Tilt gimbal, v0.3 design-space generator (parametric, CadQuery).

configure(wrap_r=..., stop="rim"|"groove", tilt_orient="back"|"down"|"up") sets the variant; build() makes it.

v0.5 (11-09, user): cage and bearing mount through-bolted to the deck (M3x10 + nuts in printed hex pockets,
no inserts there); pan drive horn -> rigid hub -> two-stage parallelogram flexure in the turntable (replaces
the pin-and-slot coupler and its creep-prone spring preload); mount floor pilots on the servo output boss so
the 6806 and the servo shaft share one datum; eye pupil widened to r 8.4.
v0.2 notes follow.

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

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out_v03")

# ----------------------------------------------------------------------------- purchased parts [mm]
SV_L, SV_W = 45.2, 24.7                  # STS3215 case (user measured 45 x 24, 11-09)
SV_H = 33.0                              # back face -> shaft face (user, 11-09: 33 at the face, 30 at the ledge)
SV_CAP_H, SV_CAP_INSET = 3.0, 1.0        # shaft-face cap above the ledge; inset per side [VERIFY inset]
SV_SHAFT_FROM_END = 11.2                 # shaft centre from the near end [VERIFY]
SV_BOSS_H, SV_BOSS_D = 2.5, 9.0          # spline above the shaft face (user, 11-09: 2-3 mm) [VERIFY d]
HORN_GAP = 0.5                           # horn underside above the shaft face
SV_CONN_W, SV_CONN_H, SV_CONN_D = 16.0, 7.0, 6.0   # two 5264-3P sockets side by side [VERIFY]
SV_CONN_X = (SV_L - 2 * SV_SHAFT_FROM_END) / 2  # centre of the wide face opposite the horn (user, 11-02) [VERIFY]
SV_PLUG = 14.0                                      # plug + cable bend room out of that face
HORN_D, HORN_T = 24.0, 4.0               # disc horn [VERIFY]

CAM_W, CAM_H, CAM_PCB = 25.0, 24.0, 1.0  # Camera Module 3 Wide [VERIFY holes/lens as Camera v2]
CAM_LENS_D, CAM_LENS_H = 8.5, 11.4
CAM_LENS_FROM_TOP = 9.46
CAM_HOLES = [(-10.5, -2.0), (10.5, -2.0), (-10.5, -14.5), (10.5, -14.5)]
FPC_W = 16.0                             # camera-end FPC width (15-pin)

PI_L, PI_W, PI_PCB = 85.0, 56.0, 1.6
PI_HOLES = [(3.5, 3.5), (61.5, 3.5), (3.5, 52.5), (61.5, 52.5)]  # local (x along 85, y along 56) from PCIe-end corner
PI_STACK_H = 30.0                        # PCB bottom -> AI HAT+ top [VERIFY]
PI_STANDOFF = 0.5                        # v0.6 (user, 11-12): Pi bosses stand 0.5 mm proud (was 0.3)
PI_RECESS = 1.0                          # pocket under the Pi footprint for underside parts (boss + 1.0 clearance) [VERIFY]
LOWER_T = 3.2                            # v0.5.5 (user): lower deck as thin as strength allows (was 5)
PILLAR_PILOT_R, PILLAR_PILOT_L = 1.3, 8.0  # M3x10 from the top deck self-taps into the pillar top (no nut)
# v0.6.1 (user, 11-12): the top deck carries square pegs that drop into sockets in the pillar tops; one M3x10
# goes in sideways through the pillar wall and the peg (self-tapping into the peg). Glue on the pegs is optional
# once nothing needs to be opened again.
PEG_A, PEG_L, PEG_CLR = 6.0, 8.0, 0.1    # square peg side / length below the deck, socket clearance per side
CROSS_DZ = 4.0                           # cross bolt axis this far below the deck underside
PILOT_R = 1.3                            # M3 self-tapping pilot (d2.6) used wherever a nut was hard to place (v0.5.6)
M2_PILOT_R = 0.8                         # M2 self-tapping pilot (d1.6)
# v0.5.3 fastener sizes (user, 11-09): M3 structure, M2.5 Raspberry Pi, M2 camera
M25_CLR, M25_HEAD_D, M25_HEAD_H = 1.4, 4.5, 2.5   # clearance radius, socket head
PI_SCREW_L = 5.0                         # M2.5 x 5 down through the Pi (1.6) self-taps into boss (0.3) + deck (3.2)
M25_NUT_AF, M25_NUT_H, M25_POCKET_AF, M25_POCKET_H = 5.0, 2.0, 5.3, 2.2   # (not used since v0.5.6)
M25_PILOT_R = 1.05                       # M2.5 self-tapping pilot (d2.1) through the 0.3 boss + 3.2 deck
M2_CLR, M2_HEAD_D, M2_HEAD_H = 1.1, 3.8, 2.0
M2_NUT_AF, M2_NUT_H, M2_POCKET_AF, M2_POCKET_H = 4.0, 1.6, 4.3, 1.8
CAM_BOSS = 0.3                           # v0.6 (user, 11-12): camera screw bosses stand 0.3 mm proud of the web (was 0.1)
CAM_SCREW_L = 5.0                        # v0.5.6: M2x5 from the camera front (PCB 1 + boss 0.1) self-taps through the 2.5 web
PI_PORTS = {                             # local x of port centres on the long edge y=0 [VERIFY]
    "USB-C": 11.2, "HDMI0": 25.8, "HDMI1": 39.2}
PI_GPIO_X = (7.0, 58.0)                  # 40-pin header span along the y=56 edge [VERIFY]

BRG_TILT = (16.0, 5.0, 5.0)              # (v0.3 625ZZ, no longer used)
TILT_PIN_D = 6.0                         # printed tilt pivot pin in a preloaded printed bushing (no 625)
BUSH_PRELOAD, BUSH_T = 0.15, 1.2         # bushing tongue interference and thickness [mm]
BRG_PAN = (42.0, 30.0, 7.0)              # 6806 (61806) OD, ID, W

# ----------------------------------------------------------------------------- layout [mm]
DECK_T = 5.0
DECK_X0, DECK_X1, DECK_Y = -100.0, 50.0, 110.0
PILLARS = [(-94.0, 49.0), (44.0, 49.0), (44.0, -49.0), (-30.0, -49.0)]   # kept off the port zones
PILLARS_ROT180 = [(-94.0, -49.0), (-30.0, 49.0), (44.0, 49.0), (44.0, -49.0)]   # v0.6.2: Pi turned, USB end now at +Y
PILLAR_R = 5.6                           # v0.6.1: d11.2 around the square socket (stays inside the r6 deck-corner fillet)
BASE_GAP = 55.0                          # top-deck underside -> lower-deck top
LOWER_TOP = -DECK_T - BASE_GAP

SV_TOP = 0.0                             # pan servo case top flush with the deck surface (clears the channel floor)
PAN_HORN_Z = SV_TOP + HORN_GAP           # horn sits on the spline right above the case (v0.5.2)
TUBE_ID, TUBE_TOP = 27.0, 21.0           # fixed bearing tube (horn d24 turns inside it)
SHOULDER_R, SHOULDER_Z = 17.0, 14.0      # inner race seat
WRAP_R_OUT, WRAP_WALL = 44.0, 3.0        # cable channel outer wall
DISC_Z0, DISC_T, DISC_R = 22.0, 4.0, 56.0
# v0.5 (user, 11-09) pan drive: horn -> rigid hub (4x M3 into the horn) -> 2x M3 -> flexure hub in the turntable.
# Two-stage parallelogram flexure (compliant Oldham): Y float in stage 1, X float in stage 2, both stiff in
# torsion (blades loaded along their length). No sliding contact, no spring preload -> no backlash, no creep.
FLEX_T, FLEX_GAP = 1.2, 0.7              # blade thickness (3 perimeters @0.4) / slot width (FDM-safe) [mm]
FLEX_H2 = 3.0                            # stage-2 blade height at the disc top face (axial compliance) [mm]
FLEX_HUB = (6.5, 4.5)                    # hub half-size x, y
HUB_SCREWS = ((3.5, 0.0), (-3.5, 0.0))   # flexure hub -> rigid hub, M3x10 into nuts in side slots (v0.5.1)
HUB_R = TUBE_ID / 2 - 1.0                # rigid hub radius inside the fixed tube
HORN_SCREW_R = 8.0                       # disc horn hole circle [VERIFY]
STOP_R, STOP_HALF_W = 52.0, 6.0          # hard stop at 180 deg (rear)
HOLE_FPC = (180.0, 33.0)                 # turntable cable holes (angle deg, radius)
HOLE_BUS = (128.0, 36.0)
DECK_EXIT = (180.0, 35.0)                # where cables leave the channel downward

H_TILT = 58.0
ARM_W, ARM_T_SERVO, ARM_T_IDLE = 30.0, 3.5, 6.0
SADDLE_H, SADDLE_CLR = 2.5, 0.2          # torque-reaction pocket around the tilt servo case
SV_CASE_HOLES = ()                       # servo held by the slide-in channel (v0.5.4), case holes not used
HORN_HOLE_R = HORN_D / 2 + 1.0           # U-cut in the servo arm: the horn turns inside the arm plate
# v0.5.4 (user, 11-09): tilt servo drops into a channel from above and sits on a load plate under its lower end
CH_WALL, CH_LIP, CH_SHELF = 3.0, 2.5, 3.0   # side wall, back lip (inward), load plate thickness
CH_CLR = 0.25                            # servo body clearance in the channel
TIE_W, TIE_H = 5.0, 2.0                  # cable-tie slot (one tie over the servo top end and under the plate)
CRADLE_HALF_Y, CRADLE_WALL = 16.0, 2.5               # camera 25 wide + 0.5 clearance each side
Y_SRV_OUT = CRADLE_HALF_Y + HORN_T + HORN_GAP   # tilt servo shaft face = outer face of the servo arm (v0.5.2)
CRADLE_HALF_Z = 18.0
# v0.5.5 (user): H section seen from above: the camera web sits in the middle of the side walls, which run
# the same length behind and in front of it and end at the tilt axis (only the horn hub / pivot boss go further)
CRADLE_X0, CRADLE_X1 = None, 0.0         # set below from the camera position
HUB_DISC_R = 10.8                        # cradle hub disc on the tilt horn: just covers the 4 screw heads (keeps out of the corner rays)
EYE = False                              # eye plate left out for now (user, 11-12); geometry kept in eye_plate()
EYE_X0, EYE_T = 6.0, 1.4                 # plate back face (lens front is at x=5.4) and thickness
EYE_GROOVE_FLOOR = 3.0                   # groove ends this far above the cradle bottom (v0.5.4: plate can move down ~12 mm)
EYE_A, EYE_B = 13.0, 11.0                # almond half-width (max between the cradle grooves) / half-height [mm]
PUPIL_R = 8.4                            # pupil radius on the plate face (v0.4: 7.2; user 11-09 wider)
ENTRANCE_PUPIL_X = 3.4                   # ~2 mm inside the lens barrel; FOV cone apex for clearance
CAM_BACK_X = -7.0
WEB_X1 = CAM_BACK_X - CAM_BOSS           # web front face
WEB_X0 = WEB_X1 - 2.5                    # web back face (CRADLE_WALL)
CRADLE_X0 = 2 * (WEB_X0 + WEB_X1) / 2 - CRADLE_X1     # symmetric about the web

PAN_SOFT_LIMIT = 150.0
HOUSING_R = 24.0                          # outer-race housing outer radius (rotating) - channel inner boundary
STOP = "rim"
TILT_ORIENT = "back"
GROOVE_LUG_HALF = 4.0                     # groove stop: lug half-width on the turntable [mm]
STOP_BLOCK_HALF = 5.0
FIT_SHAFT = -0.05                        # radius allowance on printed pins/tubes that go into a bearing bore
FIT_BORE = 0.15                          # printed bores that take a bearing OD (radius allowance)
PI_WINDOW = None                        # top-deck opening over the Pi (tried 11-02, user reverted: slots in the lower deck instead)
PI_SLOT_W, PI_SLOT_HALF_L = 4.5, 46.0     # long narrow slots in the lower deck under the Pi: cooling + cable pass
MOUNT_TAB_R = 43.0                       # bearing-mount fixing tabs outside the channel wall
MOUNT_TAB_ANGLES = (45, 135, 225, 315)
# v0.5 fasteners (user, 11-09): through-bolts, M3x10 only (one screw size kept) + M3 hex nuts in printed hex
# pockets (captive: no wrench needed). Cage: head under the ear, nut flush in the deck top. Mount: head under
# the deck, nut in the tab top.
NUT_AF, NUT_H = 5.5, 2.4                 # M3 nut across flats / height (ISO 4032)
NUT_POCKET_AF, NUT_POCKET_H = 5.8, 2.6
EAR_CLAMP = 5.0                          # ear material under the deck (5 + deck 5 = M3x10)
CAGE_BOLTS = ((42.0, 11.0), (42.0, -11.0), (-20.0, 0.0))   # outside the mount floor / clear of the shoulder
PAN_BEARING = False                      # v0.6 (user, 11-12): no 6806; the turntable sits on the servo horn directly
BOSS_PILOT = False                       # v0.5.2: horn sits 0.5 mm above the case (2.5 mm spline), no room                        # mount floor locates on the servo output boss -> 6806 coaxial with the shaft
PILOT_CLR = 0.15                         # radius clearance on the boss [VERIFY boss d with a caliper]


def configure(wrap_r=44.0, stop="rim", tilt_orient="back"):
    """Set the variant and derive every dependent dimension."""
    global WRAP_R_OUT, STOP, TILT_ORIENT, DISC_R, DISC_T, STOP_R, HOLE_FPC, HOLE_BUS, DECK_EXIT, H_TILT, R_MID, ANNULUS
    WRAP_R_OUT, STOP, TILT_ORIENT = wrap_r, stop, tilt_orient
    ANNULUS = WRAP_R_OUT - HOUSING_R - 0.5
    R_MID = HOUSING_R + 0.5 + ANNULUS / 2
    if stop == "rim":
        DISC_T = 4.0
        STOP_R = WRAP_R_OUT + WRAP_WALL + 5.0
        DISC_R = STOP_R + 4.0
    else:
        DISC_T = 5.0
        STOP_R = WRAP_R_OUT + WRAP_WALL / 2
        DISC_R = WRAP_R_OUT + WRAP_WALL + 4.0
    HOLE_FPC = (180.0, R_MID)
    DECK_EXIT = (180.0, R_MID)
    z_top = DISC_Z0 + DISC_T
    h = max(z_top + 25.0 + 3.0, 57.0)                # cradle sweep r=22.2 + FPC loop under the cradle
    if tilt_orient == "down":                        # body end 2 mm above the disc (plugs now point outward, +Y)
        h = max(h, z_top + (SV_L - SV_SHAFT_FROM_END) + 2.0)
    H_TILT = h
    cx, cy, _ = tilt_conn_point()
    HOLE_BUS = (math.degrees(math.atan2(cy, cx)), R_MID)


def tilt_tf(p):
    """Map a point from the servo_z_up frame to the assembly (tilt servo on the +Y arm)."""
    x, y, z = p
    x, y, z = x, -z, y                                # rot X +90: shaft +Z -> -Y
    if TILT_ORIENT == "back":
        x, z = -x, -z                                 # rot Y 180: L -> -X
    elif TILT_ORIENT == "down":
        x, z = z, -x                                  # rot Y +90: L -> -Z
    else:
        x, z = -z, x                                  # rot Y -90: L -> +Z
    return x, y + Y_SRV_OUT, z + H_TILT


def tilt_rot(shape):
    s = shape.rotate((0, 0, 0), (1, 0, 0), 90)
    if TILT_ORIENT == "back":
        s = s.rotate((0, 0, 0), (0, 1, 0), 180)
    elif TILT_ORIENT == "down":
        s = s.rotate((0, 0, 0), (0, 1, 0), 90)
    else:
        s = s.rotate((0, 0, 0), (0, 1, 0), -90)
    return s.translate((0, Y_SRV_OUT, H_TILT))


def tilt_conn_point():
    """Centre of the plug zone in front of the tilt servo connectors (assembly coords)."""
    return tilt_tf((SV_CONN_X, 0.0, -SV_H - 2.0 - SV_PLUG / 2))


# ----------------------------------------------------------------------------- helpers
def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def cyl_y(r, y0, y1, x=0.0, z=0.0):
    return cq.Workplane("XZ").circle(r).extrude(-(y1 - y0)).translate((x, y0, z))


def cyl_x(r, x0, x1, z=0.0, y=0.0):
    return cq.Workplane("YZ").circle(r).extrude(x1 - x0).translate((x0, y, z))


def cyl_z(r, z0, z1, x=0.0, y=0.0):
    return cq.Workplane("XY").circle(r).extrude(z1 - z0).translate((x, y, z0))


def ring_z(r_in, r_out, z0, z1, x=0.0, y=0.0):
    return cyl_z(r_out, z0, z1, x, y).cut(cyl_z(r_in, z0 - 1, z1 + 1, x, y))


def hex_z(af, z0, z1, x=0.0, y=0.0, rot=0.0):
    """Hexagonal prism (across flats af) for nut pockets."""
    return (cq.Workplane("XY").polygon(6, af / math.cos(math.pi / 6)).extrude(z1 - z0)
            .rotate((0, 0, 0), (0, 0, 1), rot).translate((x, y, z0)))


def nut_slot(x, y, z_nut, a_deg, length):
    """Side-entry nut trap: M3 nut (flats across the slot) slides in along direction a_deg from `length` mm away
    and stops centred on the bolt at (x, y); nut bottom at z_nut. The closed end is shaped by the nut corner."""
    corner = NUT_AF / math.sqrt(3) + 0.1
    s = box(-corner, length, -NUT_POCKET_AF / 2, NUT_POCKET_AF / 2, z_nut - 0.1, z_nut + NUT_POCKET_H - 0.1)
    return s.rotate((0, 0, 0), (0, 0, 1), a_deg).translate((x, y, 0))


def polar(a_deg, r):
    a = math.radians(a_deg)
    return r * math.cos(a), r * math.sin(a)


def arc_slot(a_deg, r, tang, rad, z0, z1):
    """Rectangular slot centred at polar (a, r): tang along the tangent, rad along the radius."""
    x, y = polar(a_deg, r)
    return box(-rad / 2, rad / 2, -tang / 2, tang / 2, z0, z1).rotate((0, 0, 0), (0, 0, 1), a_deg).translate((x, y, 0))


# ----------------------------------------------------------------------------- purchased-part envelopes
def servo_z_up():
    """Shaft on +Z at (0,0); case top face at z=0; L along +x; connectors on the bottom face (-Z, opposite the horn)."""
    body = box(-SV_SHAFT_FROM_END, SV_L - SV_SHAFT_FROM_END, -SV_W / 2, SV_W / 2, -SV_H, -SV_CAP_H).edges("|Z").fillet(2.0)
    i = SV_CAP_INSET
    body = body.union(box(-SV_SHAFT_FROM_END + i, SV_L - SV_SHAFT_FROM_END - i, -SV_W / 2 + i, SV_W / 2 - i, -SV_CAP_H - 0.01, 0))
    boss = cyl_z(SV_BOSS_D / 2, 0, SV_BOSS_H)
    conn = box(SV_CONN_X - SV_CONN_H / 2, SV_CONN_X + SV_CONN_H / 2, -SV_CONN_W / 2, SV_CONN_W / 2, -SV_H - 2.0, -SV_H + 0.5)
    return body.union(boss).union(conn)


def servo_conn_keepout_z_up():
    """Plug + bend room in front of the connector face (not printed, used for checks)."""
    return box(SV_CONN_X - SV_CONN_H / 2 - 2, SV_CONN_X + SV_CONN_H / 2 + 2, -SV_CONN_W / 2 - 1, SV_CONN_W / 2 + 1,
               -SV_H - 2.0 - SV_PLUG, -SV_H - 2.0)


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


PI_ROT180 = True     # v0.6.2 (user, 11-12): Pi turned 180 deg in the same footprint so its CAM connector (HDMI
                     # edge) faces the deck centre, next to the FPC drop; the camera cable was too short
if PI_ROT180:
    PILLARS = PILLARS_ROT180


def pi_to_deck(lx, ly):
    ox, oy = pi_origin()
    if PI_ROT180:
        lx, ly = PI_L - lx, PI_W - ly
    return ox + ly, oy - lx


def pi_box(lx0, lx1, ly0, ly1, z0, z1):
    """Axis-aligned deck box of a rectangle given in Pi-local coordinates."""
    (xa, ya), (xb, yb) = pi_to_deck(lx0, ly0), pi_to_deck(lx1, ly1)
    return box(min(xa, xb), max(xa, xb), min(ya, yb), max(ya, yb), z0, z1)


def pi_stack():
    ox, oy = pi_origin()
    z0 = LOWER_TOP + PI_STANDOFF
    pcb = pi_box(0, PI_L, 0, PI_W, z0, z0 + PI_PCB)
    stack = pi_box(2, PI_L - 20, 2, PI_W - 2, z0 + PI_PCB, z0 + PI_STACK_H)
    usb = pi_box(PI_L - 20, PI_L + 3, 1, PI_W - 1, z0 + PI_PCB, z0 + PI_PCB + 16)
    return pcb.union(stack).union(usb)


def pi_keepouts():
    """Plug and wiring zones that must stay clear (ghost volumes)."""
    ox, oy = pi_origin()
    z0 = LOWER_TOP + PI_STANDOFF + PI_PCB
    zones = {}
    for name, lx in PI_PORTS.items():
        w = 9.5 if name == "USB-C" else 11.0
        zones[name] = pi_box(lx - w / 2, lx + w / 2, -30, 0, z0, z0 + 7)
    zones["USB/Ethernet"] = pi_box(PI_L + 3, PI_L + 33, 1, PI_W - 1, z0, z0 + 16)
    gx0, gy0 = pi_to_deck(PI_GPIO_X[0], PI_W - 5)
    gx1, gy1 = pi_to_deck(PI_GPIO_X[1], PI_W - 1)
    top = LOWER_TOP + PI_STANDOFF + PI_STACK_H
    zones["GPIO jumpers"] = box(min(gx0, gx1) - 1, max(gx0, gx1) + 1, min(gy0, gy1), max(gy0, gy1), top, top + 18)
    return zones


# ----------------------------------------------------------------------------- printed parts: base
def top_deck():
    deck = box(DECK_X0, DECK_X1, -DECK_Y / 2, DECK_Y / 2, -DECK_T, 0).edges("|Z").fillet(6.0)
    c = 0.4
    deck = deck.cut(box(-SV_SHAFT_FROM_END - c, SV_L - SV_SHAFT_FROM_END + c, -SV_W / 2 - c, SV_W / 2 + c, -DECK_T - 1, 1))
    ex, ey = polar(*DECK_EXIT)
    deck = deck.cut(box(ex - min(6, ANNULUS / 2 - 0.5), ex + min(6, ANNULUS / 2 - 0.5), ey - (FPC_W + 4) / 2, ey + (FPC_W + 4) / 2, -DECK_T - 1, 1))  # cable drop
    for x, y in PILLARS:   # v0.6.1: square peg under the deck, cross pilot hole facing the bolt entry side
        a = pillar_slot_dir(x, y)
        peg = box(x - PEG_A / 2, x + PEG_A / 2, y - PEG_A / 2, y + PEG_A / 2, -DECK_T - PEG_L, -DECK_T + 0.01)
        peg = peg.edges("|Z and <Z").chamfer(0.6) if False else peg
        hole = cyl_x(PILOT_R, -PEG_A, PEG_A, 0.0, 0.0).rotate((0, 0, 0), (0, 0, 1), a).translate((x, y, -DECK_T - CROSS_DZ))
        deck = deck.union(peg).cut(hole)
    for sx, sy in CAGE_BOLTS:       # v0.5.6: cage bolts self-tap 5 mm into the deck (no nut); tip ends at the deck top
        deck = deck.cut(cyl_z(PILOT_R, -DECK_T - 1, 1, sx, sy))
    for a in MOUNT_TAB_ANGLES:     # bearing-mount through-bolts, head under the deck; d3.6 so the boss pilot governs
        x, y = polar(a, MOUNT_TAB_R)
        deck = deck.cut(cyl_z(1.8 if BOSS_PILOT else 1.7, -DECK_T - 1, 1, x, y))
    # open window over the Pi (user, 11-02): cooling air + room for more cables; rim kept clear of the
    # pillar holes, bearing-mount tabs, Stage-2 M4 holes and the FPC drop
    if PI_WINDOW:
        x0, x1, hy = PI_WINDOW
        deck = deck.cut(box(x0, x1, -hy, hy, -DECK_T - 1, 1).edges("|Z").fillet(6.0))
    # Stage-2 interface: 4x M4 on a 70 mm square around the pan axis [VERIFY]
    for sx in (-35, 35):
        for sy in (-35, 35):
            if abs(sx - ex) < 10 and abs(sy - ey) < 15:
                continue
            deck = deck.cut(cyl_z(2.2, -DECK_T - 1, 1, sx, sy))
    return deck


def bearing_mount():
    """Fixed: outer wall of the cable channel + hard-stop block (+ the 6806 tube when PAN_BEARING).
    v0.6: without the bearing this is just the cable channel: floor ring, wall, stop block, 4 fixing tabs."""
    floor = ring_z(TUBE_ID / 2, WRAP_R_OUT + WRAP_WALL, 0, 2.0)
    wall = ring_z(WRAP_R_OUT, WRAP_R_OUT + WRAP_WALL, 0, TUBE_TOP)
    m = floor.union(wall)
    if PAN_BEARING:
        tube = ring_z(TUBE_ID / 2, BRG_PAN[1] / 2 + FIT_SHAFT, 0, TUBE_TOP)            # inner-race fit d30
        shoulder = ring_z(TUBE_ID / 2, SHOULDER_R, 0, SHOULDER_Z)                       # inner-race seat
        m = m.union(tube).union(shoulder)
    # cable drop through the floor (matches the deck slot)
    ex, ey = polar(*DECK_EXIT)
    m = m.cut(box(ex - min(6, ANNULUS / 2 - 0.5), ex + min(6, ANNULUS / 2 - 0.5), ey - (FPC_W + 4) / 2, ey + (FPC_W + 4) / 2, -1, 3))
    for a in MOUNT_TAB_ANGLES:      # fixing tabs outside the wall: M3x10 up through the deck, nut in a pocket on top
        tab = box(MOUNT_TAB_R - 5, MOUNT_TAB_R + 5, -5, 5, 0, 5.0).rotate((0, 0, 0), (0, 0, 1), a)
        x, y = polar(a, MOUNT_TAB_R)
        m = m.union(tab).cut(cyl_z(PILOT_R, -1, 6, x, y))      # v0.5.6: M3x10 up through the deck self-taps into the tab
    if BOSS_PILOT:                  # floor closed down to the servo output boss: same datum as the shaft
        m = m.union(ring_z(SV_BOSS_D / 2 + PILOT_CLR, TUBE_ID / 2 + 0.01, 0, 2.0))
    for sx, sy in CAGE_BOLTS:       # clearance over a cage-bolt tip that lands under the floor
        if math.hypot(sx, sy) < WRAP_R_OUT + WRAP_WALL + 3.5:
            m = m.cut(cyl_z(2.0, -1, 2.05, sx, sy))
    if STOP == "rim":   # block outside the wall at 180 deg, hit by a pin hanging from the disc rim
        blk = box(STOP_R - 4, STOP_R + 4, -STOP_HALF_W, STOP_HALF_W, 0, 15).rotate((0, 0, 0), (0, 0, 1), 180)
        m = m.union(blk).union(box(-(STOP_R - 4), -(WRAP_R_OUT + WRAP_WALL) + 0.5, -STOP_HALF_W, STOP_HALF_W, 0, 6))
    else:               # block on top of the wall at 180 deg, running in a groove under the disc
        m = m.union(box(-(WRAP_R_OUT + WRAP_WALL), -WRAP_R_OUT, -STOP_BLOCK_HALF, STOP_BLOCK_HALF, TUBE_TOP - 1, DISC_Z0 + 3.0))
    return m


def pillar_slot_dir(x, y):
    """Nut slots open toward the nearest deck edge (away from the Pi and cables)."""
    d = {0: DECK_X1 - x, 180: x - DECK_X0, 90: DECK_Y / 2 - y, 270: y + DECK_Y / 2}
    return min(d, key=d.get)


def pillars(top=True, bottom=True):
    """v0.5.1: d10 pillars, through hole d3.4, M3 nut in a side slot 5 mm in from each end (M3x10 through the deck)."""
    p = None
    for x, y in PILLARS:
        a = pillar_slot_dir(x, y)
        s = cyl_z(PILLAR_R, LOWER_TOP, -DECK_T, x, y).cut(cyl_z(1.7, LOWER_TOP - 1, 0, x, y))
        if top:
            s = s.cut(nut_slot(x, y, -DECK_T - 5.0, a, PILLAR_R + 2))
        if bottom:
            s = s.cut(nut_slot(x, y, LOWER_TOP + 5.0 - NUT_H, a, PILLAR_R + 2))
        p = s if p is None else p.union(s)
    return p


def lower_deck():
    """v0.5.5 (user, 11-12): lower deck and the four pillars as one print (deck bottom on the plate). The top deck
    is held by one M3x10 per pillar, driven from above into a pilot hole (self-tapping, no nut). Deck LOWER_T thick.
    Raspberry Pi on PI_STANDOFF bosses: M2.5x5 down through the Pi self-taps into the deck (AI HAT+ spacers stand on the Pi; the
    spacer's male thread into that nut); a 1 mm recess under the board clears its underside parts, the GPIO
    header pins pass through a window."""
    zb = LOWER_TOP - LOWER_T
    deck = box(DECK_X0, DECK_X1, -DECK_Y / 2, DECK_Y / 2, zb, LOWER_TOP).edges("|Z").fillet(6.0)
    # Pi: recess under the footprint (bosses keep a ring of full deck), GPIO pin window, slots
    corners = [pi_to_deck(lx, ly) for lx, ly in ((0, 0), (PI_L, PI_W))]
    (xa, xb), (ya, yb) = sorted((corners[0][0], corners[1][0])), sorted((corners[0][1], corners[1][1]))
    rec = box(xa + 1.5, xb - 1.5, ya + 1.5, yb - 1.5, LOWER_TOP - PI_RECESS, LOWER_TOP + 1)
    for lx, ly in PI_HOLES:
        x, y = pi_to_deck(lx, ly)
        rec = rec.cut(cyl_z(4.5, zb - 1, LOWER_TOP + 2, x, y))
    deck = deck.cut(rec)
    g0, g1 = pi_to_deck(PI_GPIO_X[0] - 1.0, PI_W - 7.5), pi_to_deck(PI_GPIO_X[1] + 1.0, PI_W - 0.5)
    deck = deck.cut(box(min(g0[0], g1[0]), max(g0[0], g1[0]), min(g0[1], g1[1]), max(g0[1], g1[1]), zb - 1, LOWER_TOP + 1))
    ox, oy = pi_origin()
    for i in range(5):  # long narrow slots under the Pi (user, 11-02): air in from below + cables through
        xc = ox + 12 + i * 8
        deck = deck.cut(box(xc - PI_SLOT_W / 2, xc + PI_SLOT_W / 2, -PI_SLOT_HALF_L, PI_SLOT_HALF_L,
                            zb - 1, LOWER_TOP + 1).edges("|Z").fillet(PI_SLOT_W / 2 - 0.01))
    for lx, ly in PI_HOLES:
        x, y = pi_to_deck(lx, ly)
        deck = deck.union(cyl_z(3.0, LOWER_TOP - 0.01, LOWER_TOP + PI_STANDOFF, x, y))
        deck = deck.cut(cyl_z(M25_PILOT_R, zb - 1, LOWER_TOP + PI_STANDOFF + 1, x, y))   # v0.5.6: M2.5 self-taps
    # cable tie slots along the +X edge (servo bus / UART to the STM32 off-board)
    for y in (-20, 0, 20):
        deck = deck.cut(box(DECK_X1 - 10, DECK_X1 - 7, y - 2, y + 2, zb - 1, LOWER_TOP + 1))
    # pillars: grow out of the deck top with a 3 mm root cone, pilot hole at the top for the M3x10
    for x, y in PILLARS:
        col = cyl_z(PILLAR_R, LOWER_TOP - 0.01, -DECK_T, x, y)
        root = (cq.Workplane("XY").workplane(offset=LOWER_TOP).center(x, y).circle(PILLAR_R + 0.3)
                .workplane(offset=3.0).circle(PILLAR_R).loft())
        deck = deck.union(col).union(root)
        # v0.6.1: square socket for the top-deck peg, cross hole for the side bolt (clearance on the entry side,
        # pilot on the far side), small flat on the entry side for the screw head
        sa = PEG_A / 2 + PEG_CLR
        deck = deck.cut(box(x - sa, x + sa, y - sa, y + sa, -DECK_T - PEG_L - 0.5, 0))
        a = pillar_slot_dir(x, y)
        zc = -DECK_T - CROSS_DZ
        loc = lambda sh: sh.rotate((0, 0, 0), (0, 0, 1), a).translate((x, y, zc))
        deck = deck.cut(loc(cyl_x(1.7, 0.0, PILLAR_R + 1)))                     # entry side (+ local x)
        deck = deck.cut(loc(cyl_x(PILOT_R, -PILLAR_R - 1, 0.0)))                  # far side
        deck = deck.cut(loc(box(PILLAR_R - 0.6, PILLAR_R + 2, -3.5, 3.5, -3.5, 3.5)))   # head seat flat
    return deck


def CAGE_EARS(x0, x1, y0, y1, t):
    """Ear footprints around CAGE_BOLTS (4 mm of material around each bolt), merged into the cage walls."""
    ears = []
    for sx, sy in CAGE_BOLTS:
        if sx > x1:      # connector end (+X): ear grows out of the end wall
            ears.append((x1 + t - 0.1, sx + 4.0, sy - 4.75, sy + 4.75))   # head 2.75 + 0.5 + rib 1.5
        else:            # shaft end (-X)
            ears.append((sx - 4.0, x0 - t + 0.1, sy - 5.0, sy + 5.0))
    return ears


def pan_cage():
    c, t = 0.3, 3.0
    x0, x1 = -SV_SHAFT_FROM_END - c, SV_L - SV_SHAFT_FROM_END + c
    y0, y1 = -SV_W / 2 - c, SV_W / 2 + c
    z0, z1 = SV_TOP - SV_H - t, -DECK_T
    cage = box(x0 - t, x1 + t, y0 - t, y1 + t, z0, z1).cut(box(x0, x1, y0, y1, z0 + t, z1 + 1))
    # v0.5.5 (user): ears are thin flanges right under the deck (EAR_CLAMP thick), so the bolt head is reached
    # from below with a plain screwdriver; the cage now prints upside down (deck face on the plate) without support
    flange = None
    for (xa, xb, ya, yb), (sx, sy) in zip(CAGE_EARS(x0, x1, y0, y1, t), CAGE_BOLTS):
        e = box(xa, xb, ya, yb, z1 - EAR_CLAMP, z1)
        # two 1.5 mm ribs at the ear's sides carry the flange at 45 degrees; the head goes in between them
        wall_x = x1 + t if sx > x1 else x0 - t
        ln = abs((xb if sx > x1 else xa) - wall_x)
        for yr in (ya, yb - 1.5):
            pts = [(wall_x, z1 - EAR_CLAMP), ((xb if sx > x1 else xa), z1 - EAR_CLAMP), (wall_x, z1 - EAR_CLAMP - ln)]
            rib = cq.Workplane("XZ").polyline(pts).close().extrude(-1.5).translate((0, yr, 0))
            e = e.union(rib)
        e = e.cut(cyl_z(1.7, z0 - 1, z1 + 1, sx, sy))
        flange = e if flange is None else flange.union(e)
    # floor opening under the two bus connectors (bottom face of the servo)
    cage = cage.union(flange).cut(box(SV_CONN_X - SV_CONN_H / 2 - 2.5, SV_CONN_X + SV_CONN_H / 2 + 2.5,
                                      -SV_CONN_W / 2 - 1.5, SV_CONN_W / 2 + 1.5, z0 - 1, z0 + t + 1))
    return cage


# ----------------------------------------------------------------------------- printed parts: rotating
def horn_coupler():
    """Rigid hub (v0.5): 4x M3x10 down into the horn (6 mm floor, tip ends at the horn underside), 2x heat-set
    nuts in side slots for the flexure hub. Only a 0.7 mm pad under the flexure hub touches the turntable."""
    z0, z1 = PAN_HORN_Z + HORN_T, DISC_Z0 - FLEX_GAP          # horn top .. under the flexure frame
    c = cyl_z(HUB_R, z0, z1)
    hx, hy = FLEX_HUB
    c = c.union(box(-hx + 0.5, hx - 0.5, -hy + 0.5, hy - 0.5, z1 - 0.01, DISC_Z0))   # contact pad under the hub
    for a in range(4):
        x, y = polar(45 + 90 * a, HORN_SCREW_R)
        c = c.cut(cyl_z(1.7, z0 - 1, z1 + 1, x, y)).cut(cyl_z(3.1, z0 + 6.0, z1 + 1, x, y))
    c = c.cut(cyl_z(3.25, z0 - 1, z0 + 4.0))                 # horn spline-screw head
    for x, y in HUB_SCREWS:      # v0.5.6: M3x10 from the flexure hub top self-taps 5 mm into the solid hub (no nut)
        c = c.cut(cyl_z(PILOT_R, DISC_Z0 - 6.0, DISC_Z0 + 1, x, y))
    return c


def flexure_bodies(z0, z1):
    """Two-stage parallelogram flexure in the disc plane. Returns (pocket, bodies) in turntable coords.
    hub H (screwed to the rigid hub) -[stage 1: 2 blades along X, float in Y]- frame M -[stage 2: 2 blades
    along Y, float in X]- disc. Stage-2 blades are FLEX_H2 tall at the top face (on the bed when printed)."""
    g, t = FLEX_GAP, FLEX_T
    hx, hy = FLEX_HUB
    yb0, yb1 = hy + g, hy + g + t                        # stage-1 blade band
    my = yb1 + g + 1.5                                   # frame half-height
    mx0, mx1 = -hx - 1.5 - g - 1.5, hx + g + 1.5         # frame x extent
    px0, px1, py = mx0 - 2 * g - t, mx1 + 2 * g + t, my + g
    pocket = box(px0, px1, -py, py, z0 - 1, z1 + 1)
    hub = box(-hx, hx, -hy, hy, z0, z1).union(box(-hx - 1.5, -hx + 0.01, -yb1, yb1, z0, z1))
    for x, y in HUB_SCREWS:
        hub = hub.cut(cyl_z(1.7, z0 - 1, z1 + 1, x, y))
    b1 = box(-hx - 0.01, hx + g + 0.01, yb0, yb1, z0, z1).union(box(-hx - 0.01, hx + g + 0.01, -yb1, -yb0, z0, z1))
    frame = (box(hx + g, mx1, -my, my, z0, z1)                     # right bar (stage-1 blade ends)
             .union(box(mx0, mx1, my - 1.5, my, z0, z1)).union(box(mx0, mx1, -my, -my + 1.5, z0, z1))
             .union(box(mx0, mx0 + 1.5, -my, my, z0, z1))
             .union(box(px0 + g, px1 - g, -my, -my + 1.5, z0, z1)))    # bottom bar runs out under the stage-2 blades
    b2 = (box(px0 + g, px0 + g + t, -my, py + 0.01, z1 - FLEX_H2, z1)
          .union(box(px1 - g - t, px1 - g, -my, py + 0.01, z1 - FLEX_H2, z1)))
    return pocket, {"hub": hub, "stage1": b1, "frame": frame, "stage2": b2}


ARM_FOOT_T, ARM_FOOT_W = 6.0, 8.0


def servo_channel(z_floor):
    """Tilt-servo seat (v0.5.4): the servo slides down from above (spline down the open U of the arm plate) and
    rests on a load plate under its lower end; side walls with back lips hold it against the arm plate, one cable
    tie over the top end keeps it from lifting. Everything stands on the disc top, so the arm prints upright."""
    x_in = SV_W / 2 + CH_CLR
    x_out = x_in + CH_WALL
    y0, y_back = Y_SRV_OUT - 0.1, Y_SRV_OUT + SV_H + CH_CLR
    y1 = y_back + CH_LIP
    z_bot = H_TILT - (SV_L - SV_SHAFT_FROM_END)          # servo lower end
    z_top_end = H_TILT + SV_SHAFT_FROM_END               # servo upper end
    z_wall = z_top_end + 0.5 + TIE_H + 2.0
    out = box(-x_out, x_out, y0, y1, z_bot - CH_SHELF, z_bot)                    # load plate
    for sg in (1, -1):
        xa, xb = sorted((sg * x_in, sg * x_out))
        w = box(xa, xb, y0, y1, z_floor, z_wall)
        la, lb = sorted((sg * (x_in - CH_LIP), sg * x_in))
        w = w.union(box(la, lb, y_back, y1, z_bot - CH_SHELF, z_wall))           # back lip
        # window with a 45-degree peaked top (no overhang), between the plate and the tie slot
        ya, yb = y0 + 5.0, y_back - 3.0
        half = (yb - ya) / 2
        zb, zt = z_bot + 3.0, z_top_end - 3.0
        pts = [(ya, zb), (yb, zb), (yb, zt - half), ((ya + yb) / 2, zt), (ya, zt - half)]
        w = w.cut(cq.Workplane("YZ").polyline(pts).close().extrude(xb - xa + 2).translate((xa - 1, 0, 0)))
        out = out.union(w)
    ym = (y0 + y_back) / 2
    for zc in (z_top_end + 0.5 + TIE_H / 2, z_bot - CH_SHELF - 1.0 - TIE_H / 2):  # tie: over the top end, under the plate
        out = out.cut(box(-x_out - 1, x_out + 1, ym - TIE_W / 2, ym + TIE_W / 2, zc - TIE_H / 2, zc + TIE_H / 2))
    # front-top corners trimmed with a 45-degree cut: they showed in the view at tilt +30 deg
    cut = (cq.Workplane("YZ").polyline([(y0 - 1, H_TILT + 2.0), (y0 + 10.0, z_wall + 1), (y0 - 1, z_wall + 1)]).close()
           .extrude(2 * x_out + 2).translate((-x_out - 1, 0, 0)))
    return out.cut(cut)


def ARM_FEET():
    """y ranges of the bolt-down feet: servo foot kept clear of the flexure pocket (y > 9.3 + 0.7)."""
    y_srv_out, y_idle_in = Y_SRV_OUT, -(CRADLE_HALF_Y + 2.0)
    return {"s": (max(Y_SRV_OUT - ARM_T_SERVO - ARM_FOOT_W, 10.0), y_srv_out),
            "i": (y_idle_in - ARM_T_IDLE, y_idle_in + ARM_FOOT_W)}


def ARM_BOLTS(*_):
    f = ARM_FEET()
    ys, yi = (f["s"][0] + f["s"][1]) / 2, -(CRADLE_HALF_Y + 2.0) + ARM_FOOT_W / 2
    return [(xb, ys) for xb in (-9.0, 9.0)] + [(xb, yi) for xb in (-9.0, 9.0)]


def ARM_NUT_Z():
    """Nut bottom = tip of an M3x10 seated on the foot top."""
    return DISC_Z0 + DISC_T + ARM_FOOT_T - 10.0


def turntable_yoke():
    """Turntable disc + bearing housing + arms. Returns (solid, y_srv_in, y_srv_out, y_idle_in)."""
    y_srv_out = Y_SRV_OUT
    y_srv_in = y_srv_out - ARM_T_SERVO
    y_idle_in = -(CRADLE_HALF_Y + 2.0)
    y_idle_out = y_idle_in - ARM_T_IDLE
    z_disc_top = DISC_Z0 + DISC_T
    disc = cyl_z(DISC_R, DISC_Z0, z_disc_top)
    if PAN_BEARING:
        # outer-race housing hanging below the disc: d42 bore, lip on top of the outer race only
        od, idr, w = BRG_PAN
        housing = ring_z(od / 2 + FIT_BORE, od / 2 + 3.0, TUBE_TOP - w, DISC_Z0)
        lip = ring_z(idr / 2 + 3.5, od / 2 + 0.1, TUBE_TOP, DISC_Z0)  # touches outer race (r>18.5) only
        y = disc.union(housing).union(lip)
        y = y.cut(cyl_z(idr / 2 + 3.4, TUBE_TOP - 0.01, DISC_Z0))
        pocket, fb = flexure_bodies(DISC_Z0, z_disc_top)   # v0.5 flexure hub (needed only with the bearing)
        y = y.cut(pocket)
        for b in fb.values():
            y = y.union(b)
    else:
        # v0.6 (user): no bearing, no flexure, no separate hub. A post grows down from the disc onto the servo horn
        # and is held by the 4 horn screws (M3x10 in counterbores from the disc top, 6 mm floor on the horn).
        z0 = PAN_HORN_Z + HORN_T
        y = disc.union(cyl_z(HUB_R, z0, DISC_Z0 + 0.01))
        for a in range(4):
            x, yy = polar(45 + 90 * a, HORN_SCREW_R)
            y = y.cut(cyl_z(1.7, z0 - 1, z_disc_top + 1, x, yy)).cut(cyl_z(3.1, z0 + 6.0, z_disc_top + 1, x, yy))
        y = y.cut(cyl_z(3.25, z0 - 1, z0 + 4.0))                 # horn spline-screw head
        y = y.cut(cyl_z(3.0, z0 + 6.0, z_disc_top + 1))          # look-through to the spline screw
    # arm bolts (v0.5.6, user: nut slots too hard): M3x10 down through the foot self-taps 4 mm into a blind
    # pilot hole in the disc (bottom skin stays closed)
    for x, yb in ARM_BOLTS():
        y = y.cut(cyl_z(PILOT_R, z_disc_top - 4.5, z_disc_top + 1, x, yb))
    # arms
    arm_s = box(-ARM_W / 2, ARM_W / 2, y_srv_in, y_srv_out, z_disc_top - 0.1, H_TILT).union(
        cyl_y(ARM_W / 2, y_srv_in, y_srv_out, 0, H_TILT))
    arm_i = box(-ARM_W / 2, ARM_W / 2, y_idle_out, y_idle_in, z_disc_top - 0.1, H_TILT).union(
        cyl_y(ARM_W / 2, y_idle_out, y_idle_in, 0, H_TILT))
    # gussets
    for side in (1, -1):            # gussets on the inner side, next to the bolt-down feet
        g = (cq.Workplane("YZ").polyline([(0, 0), (6, 0), (0, 14)]).close().extrude(ARM_W - 6)
             .translate((-(ARM_W - 6) / 2, 0, 0)))
        if side > 0:
            g = g.mirror("XZ").translate((0, y_srv_in, z_disc_top))
        else:
            g = g.translate((0, y_idle_in, z_disc_top))
        y = y.union(g)
    y = y.union(arm_s).union(arm_i)
    # tilt servo boss through the servo arm, case screws [VERIFY pattern], 625 pocket on the idle arm
    # torque-reaction saddle: the arm grows SADDLE_H outward around the servo body, leaving a pocket that
    # hugs the case sides, so the motor's reaction torque is carried by shape contact, not by the screws
    body_lo, body_hi = [tilt_tf(p) for p in ((-SV_SHAFT_FROM_END, -SV_W / 2, -SV_H), (SV_L - SV_SHAFT_FROM_END, SV_W / 2, 0))]
    bx0, bx1 = sorted((body_lo[0], body_hi[0]))
    bz0, bz1 = sorted((body_lo[2], body_hi[2]))
    saddle = box(-ARM_W / 2, ARM_W / 2, y_srv_out - 0.1, y_srv_out + SADDLE_H, z_disc_top - 0.1, H_TILT).union(
        cyl_y(ARM_W / 2, y_srv_out - 0.1, y_srv_out + SADDLE_H, 0, H_TILT))
    saddle = saddle.cut(box(bx0 - SADDLE_CLR, bx1 + SADDLE_CLR, y_srv_out - 1, y_srv_out + SADDLE_H + 1,
                            -100, bz1 + SADDLE_CLR))                    # open at the bottom (cable side)
    # v0.5.4: replaced by the slide-in channel (side walls + load plate); a saddle cap above the servo would
    # stop it from dropping in from the top
    # v0.5.2: the spline stands only 2.5 mm proud, so the horn has to sit inside the arm plate: U-cut open upward
    # (no overhang when the arm prints standing). The servo shaft face bears on the plate below the horn.
    y = y.cut(cyl_y(HORN_HOLE_R, y_srv_in - 1, y_srv_out + 1, 0, H_TILT))
    y = y.cut(box(-HORN_HOLE_R, HORN_HOLE_R, y_srv_in - 1, y_srv_out + 1, H_TILT, H_TILT + 50))
    y = y.union(servo_channel(z_disc_top - 0.1))
    # case screw pattern defined in the servo frame [VERIFY] and carried through the same transform as the body
    # (v0.4 fix: the old pattern was fixed in assembly coords for the "back" orientation, so with the body
    #  hanging down two holes fell outside the arm and only one side was screwed)
    for sx, sy in SV_CASE_HOLES:
        px, _, pz = tilt_tf((sx, sy, 0))
        y = y.cut(cyl_y(1.4, y_srv_in - 1, y_srv_out + 1, px, pz))
    y = y.cut(tilt_bushing_cut(y_idle_out - 1, y_idle_in + 1))
    # cable entries into the wrap channel
    y = y.cut(arc_slot(HOLE_FPC[0], HOLE_FPC[1], FPC_W + 3, min(6.0, ANNULUS - 2), DISC_Z0 - 1, z_disc_top + 1))
    y = y.cut(arc_slot(HOLE_BUS[0], HOLE_BUS[1], 10.0, min(7.0, ANNULUS - 2), DISC_Z0 - 1, z_disc_top + 1))
    if STOP == "rim":   # pin at 0 deg hanging from the rim beside the fixed block
        y = y.union(cyl_z(3.0, 9.0, DISC_Z0 + 0.1, STOP_R, 0))
    else:               # annular groove over the wall top, interrupted by a lug at 0 deg
        gr = ring_z(WRAP_R_OUT - 0.6, WRAP_R_OUT + WRAP_WALL + 0.6, DISC_Z0 - 1, DISC_Z0 + 3.6)
        lug_a = math.degrees(GROOVE_LUG_HALF / STOP_R)
        lug = (cq.Workplane("XY").moveTo(0, 0).lineTo(*polar(-lug_a, DISC_R + 2)).threePointArc(polar(0, DISC_R + 2), polar(lug_a, DISC_R + 2))
               .close().extrude(10).translate((0, 0, DISC_Z0 - 2)))
        y = y.cut(gr.cut(lug))
    # lightening pockets in the disc (keep the ring under the arms solid)
    pr = min(7.0, ANNULUS / 2 - 1.0)
    for a in (40, -40, 155, -140):          # (+-60 dropped in v0.5.1: arm nut slots run there)
        if abs(((a - HOLE_BUS[0]) + 180) % 360 - 180) < 25 or abs(((a - HOLE_FPC[0]) + 180) % 360 - 180) < 25:
            continue
        x, yy = polar(a, R_MID)
        if abs(yy) < 33 and abs(x) < ARM_W / 2 + pr + 1:    # keep material under the arms
            continue
        y = y.cut(cyl_z(pr, DISC_Z0 - 1, z_disc_top + 1, x, yy))

    return y, y_srv_in, y_srv_out, y_idle_in


def tilt_bushing_cut(y0, y1):
    """v0.5.6 (user: hard to assemble): snap-in pivot. The bore is open to the top through a 4.8 mm throat, so the
    cradle pin is simply pushed down into it from above. Two relief slots turn the throat walls into springy
    fingers (about 16 mm long) that open 0.6 mm each to let the d6 pin pass, then close over it; the bore is
    0.05 mm under the pin radius, so the fingers keep a light grip (no play). The weight sits on solid material
    below the bore."""
    rp = TILT_PIN_D / 2
    cut = cyl_y(rp - 0.05, y0, y1, 0, H_TILT)
    cut = cut.union(box(-(rp - 0.6), rp - 0.6, y0, y1, H_TILT, H_TILT + 40))                # throat
    for sg in (1, -1):                                                                       # finger relief slots
        xa, xb = sorted((sg * (rp + 1.8), sg * (rp + 2.8)))
        cut = cut.union(box(xa, xb, y0, y1, H_TILT - 1.0, H_TILT + 40))
    return cut


def tilt_cradle():
    """v0.5.5 (user, 11-12): H section seen from above. The camera sits on a web in the middle of two side walls,
    which run equally behind and in front of it and stop at the tilt axis, so nothing stands beside the lens.
    The horn hub (servo side) and the pivot boss (other side) are discs around the axis. Camera screws M2x5 from the
    front self-tap into the web (no nuts); bosses stand CAM_BOSS proud of the web."""
    zc, w = H_TILT, CRADLE_WALL
    z0, z1 = zc - CRADLE_HALF_Z, zc + CRADLE_HALF_Z
    web = box(WEB_X0, WEB_X1, -CRADLE_HALF_Y, CRADLE_HALF_Y, z0, z1)
    side_p = box(CRADLE_X0, CRADLE_X1, CRADLE_HALF_Y - w, CRADLE_HALF_Y, z0, z1)
    side_n = box(CRADLE_X0, CRADLE_X1, -CRADLE_HALF_Y, -CRADLE_HALF_Y + w, z0, z1)
    cr = web.union(side_p).union(side_n)
    cr = cr.union(cyl_y(HUB_DISC_R, CRADLE_HALF_Y - w, CRADLE_HALF_Y, 0, zc))            # horn hub (horn screws r8 + head)
    for a in range(4):
        x, z = polar(45 + 90 * a, 8.0)
        cr = cr.cut(cyl_y(1.7, CRADLE_HALF_Y - w - 1, CRADLE_HALF_Y + 1, x, zc + z))
    cr = cr.union(cyl_y(7.0, -CRADLE_HALF_Y, -CRADLE_HALF_Y + w, 0, zc))                  # pivot boss
    cr = cr.union(cyl_y(TILT_PIN_D / 2, -(CRADLE_HALF_Y + 2.0 + ARM_T_IDLE + 1.0), -CRADLE_HALF_Y, 0, zc))  # pivot pin
    cr = cr.union(cyl_y(TILT_PIN_D / 2 + 2.0, -(CRADLE_HALF_Y + 1.9), -CRADLE_HALF_Y, 0, zc))              # thrust collar
    top = zc + CAM_LENS_FROM_TOP
    # relief in the web front for the parts on the camera back, window for its FPC connector
    cr = cr.cut(box(WEB_X1 - 1.0, WEB_X1 + 1, -8.5, 8.5, top - CAM_H + 7.0, top - 1.0))
    cr = cr.cut(box(WEB_X0 - 1, WEB_X1 + 1, -11.0, 11.0, z0 - 1, top - CAM_H + 7.0))
    for hy, hz in CAM_HOLES:
        z = top + hz
        cr = cr.union(cyl_x(2.2, WEB_X1 - 1.01, WEB_X1 + CAM_BOSS, z, hy))
        cr = cr.cut(cyl_x(M2_PILOT_R, WEB_X0 - 1, WEB_X1 + 1, z, hy))   # M2x5 from the camera front self-taps into the web
    return cr


def almond(a, b, x0, x1, zc):
    """Vesica (eye outline) in the YZ plane, half-width a along y, half-height b along z, extruded x0..x1."""
    c = (a * a - b * b) / (2 * b)
    R = b + c
    up = cq.Workplane("YZ").circle(R).extrude(x1 - x0).translate((x0, 0, zc - c))
    dn = cq.Workplane("YZ").circle(R).extrude(x1 - x0).translate((x0, 0, zc + c))
    return up.intersect(dn)


def eye_plate():
    """Decorative eye: almond plate with raised lid rim, engraved iris rings and aperture marks; the lens is the
    pupil. Tongues at the eye corners slide into the cradle grooves. The pupil is a cone from the lens entrance
    pupil so the plate never enters the 102 x 67 deg field of view."""
    zc, x0, x1 = H_TILT, EYE_X0, EYE_X0 + EYE_T
    plate = almond(EYE_A, EYE_B, x0, x1, zc)
    plate = plate.union(box(x0, x1, -(CRADLE_HALF_Y - CRADLE_WALL + 0.7), CRADLE_HALF_Y - CRADLE_WALL + 0.7, zc - 3.0, zc + 3.0))
    rim = almond(EYE_A, EYE_B, x1 - 0.01, x1 + 0.8, zc).cut(almond(EYE_A - 1.6, EYE_B - 1.2, x1 - 1, x1 + 2, zc))
    plate = plate.union(rim)
    # pupil (v0.5 wider): PUPIL_R on the plate face, 61 deg flare toward the lens, straight through the rim zone.
    # Must stay outside the FOV cone from the entrance pupil (half-angle 61 deg = diagonal 54.5 deg + margin).
    t = math.tan(math.radians(61))
    r_back = PUPIL_R - EYE_T * t
    assert r_back >= max((x0 - ENTRANCE_PUPIL_X) * t, CAM_LENS_D / 2 + 0.3), "pupil enters the field of view"
    assert PUPIL_R >= (x1 + 0.8 - ENTRANCE_PUPIL_X) * math.tan(math.radians(56)), "rim zone enters the field of view"
    cone = cq.Workplane("XY").add(cq.Solid.makeCone(r_back, PUPIL_R, EYE_T + 0.01, cq.Vector(x0 - 0.005, 0, zc), cq.Vector(1, 0, 0)))
    plate = plate.cut(cone).cut(cyl_x(PUPIL_R, x1 - 0.01, x1 + 1.0, zc))
    # iris: engraved ring + 8 aperture marks, all recessed (cannot reach into the view)
    ri = PUPIL_R + 0.5
    inner = almond(EYE_A - 2.2, EYE_B - 1.8, x1 - 1, x1 + 1, zc)
    plate = plate.cut(cq.Workplane("YZ").circle(ri + 0.5).circle(ri).extrude(0.5).translate((x1 - 0.5, 0, zc)).intersect(inner))
    for k in range(8):
        a = math.radians(22.5 + 45 * k)
        mark = (box(ri + 1.0, ri + 3.2, -0.3, 0.3, 0, 0.5).rotate((0, 0, 0), (1, 0, 0), 0)
                .translate((0, 0, 0)))
        mark = (cq.Workplane("YZ").rect(1.4, 0.6).extrude(0.5).translate((x1 - 0.5, 0, 0))
                .translate((0, 0, ri + 1.6)).rotate((0, 0, 0), (1, 0, 0), math.degrees(a)).translate((0, 0, zc)))
        plate = plate.cut(mark.intersect(inner))
    return plate


# ----------------------------------------------------------------------------- fasteners (bought, for checks/BOM)
def m3(x, y, z_head, up=True, nut_z=None, length=10.0, nut_rot=None):
    """M3 socket-head screw (head d5.5 x 3) with its seat at z_head, shank `length` toward +Z (up) or -Z."""
    sgn = 1 if up else -1
    head = cyl_z(2.75, z_head - 3.0, z_head) if up else cyl_z(2.75, z_head, z_head + 3.0)
    shank = cyl_z(1.5, *sorted((z_head, z_head + sgn * length)), x=0, y=0)
    out = head.union(shank).translate((x, y, 0))
    if nut_z is not None:
        rot = math.degrees(math.atan2(y, x)) + 30 if nut_rot is None else nut_rot
        out = out.union(hex_z(NUT_AF, nut_z, nut_z + NUT_H, x, y, rot))
    return out


def fasteners():
    """Returns (base, pan): every M3x10 + nut of the v0.5 stacks (screws into inserts carry no nut)."""
    base = None
    for sx, sy in CAGE_BOLTS:            # ear (5) + deck (5); nut seated at the pocket floor
        f = m3(sx, sy, -DECK_T - EAR_CLAMP)
        base = f if base is None else base.union(f)
    for a in MOUNT_TAB_ANGLES:           # deck (5) + tab (5); nut in the tab pocket
        x, y = polar(a, MOUNT_TAB_R)
        base = base.union(m3(x, y, -DECK_T))
    pan = None
    for a in range(4):                   # rigid hub -> horn: 5 mm floor + 4 mm horn, tip 1 mm below the horn
        x, y = polar(45 + 90 * a, HORN_SCREW_R)
        f = m3(x, y, PAN_HORN_Z + HORN_T + 6.0, up=False)
        pan = f if pan is None else pan.union(f)
    for x, y in (HUB_SCREWS if PAN_BEARING else ()):   # flexure hub (DISC_T) -> rigid hub
        pan = pan.union(m3(x, y, DISC_Z0 + DISC_T, up=False))
    for x, y in ARM_BOLTS():   # foot (6) + disc: nut in the disc slot
        pan = pan.union(m3(x, y, DISC_Z0 + DISC_T + ARM_FOOT_T, up=False))
    for x, y in PILLARS:                 # v0.6.1: side bolt through pillar wall + peg (head on the outer flat)
        a = pillar_slot_dir(x, y)
        xh = PILLAR_R - 0.6
        b = cyl_x(2.75, xh, xh + 3.0).union(cyl_x(1.5, xh - 10.0, xh))
        base = base.union(b.rotate((0, 0, 0), (0, 0, 1), a).translate((x, y, -DECK_T - CROSS_DZ)))
    for lx, ly in PI_HOLES:              # M2.5 x 5 down through the Pi into the nut under the lower deck
        x, y = pi_to_deck(lx, ly)
        zt = LOWER_TOP + PI_STANDOFF + PI_PCB
        f = cyl_z(M25_HEAD_D / 2, zt, zt + M25_HEAD_H, x, y).union(cyl_z(1.25, zt - PI_SCREW_L, zt, x, y))
        base = base.union(f)
    return base, pan


def fasteners_tilt():
    """M2 x 5 holding the camera: head on the PCB front corner, self-tapping into the cradle web (no nut)."""
    out = None
    top = H_TILT + CAM_LENS_FROM_TOP
    x_head = CAM_BACK_X + CAM_PCB
    for hy, hz in CAM_HOLES:
        z = top + hz
        f = cyl_x(M2_HEAD_D / 2, x_head, x_head + M2_HEAD_H, z, hy).union(cyl_x(1.0, x_head - CAM_SCREW_L, x_head, z, hy))
        out = f if out is None else out.union(f)
    return out


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
    wrap_r = R_MID + ANNULUS / 4
    fpc += [(*polar(180 + 30 * k, wrap_r), 10) for k in (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12)]  # one turn of slack
    if PI_ROT180:   # CAM connector on the Pi edge facing the drop: straight down, then in under the HAT
        fpc += [(ex, ey, 6), (ex, ey, LOWER_TOP + PI_STANDOFF + 6), (pi_cam[0], pi_cam[1], LOWER_TOP + PI_STANDOFF + 3)]
    else:
        fpc += [(ex, ey, 6), (ex, ey, -DECK_T - 3), (pi_cam[0], ex * 0 + pi_cam[1] * 0 + ey, hat_top + 6),
                (pi_cam[0], pi_cam[1], hat_top + 6), (pi_cam[0], pi_cam[1], LOWER_TOP + PI_STANDOFF + 3)]
    cx, cy, cz = tilt_conn_point()
    bus = [(cx, cy, cz), (0.5 * (cx + bx), 0.5 * (cy + by), 0.5 * (cz + DISC_Z0 + DISC_T) + 4), (bx, by, DISC_Z0 + DISC_T + 2),
           (bx, by, DISC_Z0 - 2)]
    bus += [(*polar(HOLE_BUS[0] + 30 * k, R_MID - ANNULUS / 4), 6) for k in range(1, 12)]
    pan_conn = (SV_CONN_X, 0, SV_TOP - SV_H - 2.0 - SV_PLUG / 2)
    bus += [(ex + 2, ey + 6, 6), (ex + 2, ey + 6, -DECK_T - 3), (-20, 20, -DECK_T - 6), (SV_L - SV_SHAFT_FROM_END + 6, 18, -DECK_T - 6),
            (SV_L - SV_SHAFT_FROM_END + 6, 0, pan_conn[2]), pan_conn]
    out = [pan_conn, (DECK_X1 - 8, 0, pan_conn[2]), (DECK_X1 - 8, 0, LOWER_TOP + 3), (DECK_X1 + 25, 0, LOWER_TOP + 3)]
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
    parts["pan_cage"] = (pan_cage(), (0.55, 0.57, 0.60), True, "base")
    parts["pan_servo"] = (servo_z_up().translate((0, 0, SV_TOP)), (0.12, 0.12, 0.13), False, "base")
    parts["pi5_stack"] = (pi_stack(), (0.15, 0.55, 0.30), False, "base")
    od, idr, w = BRG_PAN
    if PAN_BEARING:
        parts["bearing_6806"] = (ring_z(idr / 2, od / 2, TUBE_TOP - w, TUBE_TOP), (0.75, 0.75, 0.78), False, "base")
    parts["pan_horn"] = (cyl_z(HORN_D / 2, PAN_HORN_Z, PAN_HORN_Z + HORN_T), (0.70, 0.70, 0.72), False, "pan")
    if PAN_BEARING:
        parts["horn_coupler"] = (horn_coupler(), (0.95, 0.75, 0.20), True, "pan")
    parts["turntable_yoke"] = (yoke, (0.20, 0.45, 0.85), True, "pan")
    # tilt servo: shaft -Y at (0, H_TILT); case outside the +Y arm; body points backward (-X)
    ts = tilt_rot(servo_z_up())
    parts["tilt_servo"] = (ts, (0.12, 0.12, 0.13), False, "pan")
    parts["tilt_horn"] = (cyl_y(HORN_D / 2, CRADLE_HALF_Y, CRADLE_HALF_Y + HORN_T, 0, H_TILT), (0.70, 0.70, 0.72), False, "tilt")
    parts["tilt_cradle"] = (tilt_cradle(), (0.95, 0.55, 0.15), True, "tilt")
    if EYE:
        parts["eye_plate"] = (eye_plate(), (0.85, 0.20, 0.10), True, "tilt")
    parts["camera"] = (camera_module().translate((CAM_BACK_X, 0, H_TILT)), (0.10, 0.45, 0.25), False, "tilt")
    fb, fp = fasteners()
    parts["fasteners_base"] = (fb, (0.85, 0.75, 0.30), False, "base")
    parts["fasteners_pan"] = (fp, (0.85, 0.75, 0.30), False, "pan")
    parts["fasteners_tilt"] = (fasteners_tilt(), (0.85, 0.75, 0.30), False, "tilt")

    keepouts = {f"keepout {k}": v for k, v in pi_keepouts().items()}
    keepouts["keepout pan servo plugs"] = servo_conn_keepout_z_up().translate((0, 0, SV_TOP))
    keepouts["keepout tilt servo plugs"] = tilt_rot(servo_conn_keepout_z_up())
    routes = cable_routes(y_srv_out)

    if export:
        os.makedirs(OUT, exist_ok=True)
        assy = cq.Assembly(name="rcws_gimbal_v0_3")
        for name, (shape, rgb, _, _) in parts.items():
            assy.add(shape, name=name, color=cq.Color(*rgb))
        assy.export(os.path.join(OUT, "rcws_gimbal_v0_3.step"))
        for name, (shape, _, printable, _) in parts.items():
            if printable:
                cq.exporters.export(shape, os.path.join(OUT, f"{name}.stl"), tolerance=0.05, angularTolerance=0.1)
        meta = {
            "variant": {"wrap_r": WRAP_R_OUT, "stop": STOP, "tilt_orient": TILT_ORIENT}, "H_TILT": H_TILT, "DISC_R": DISC_R, "pan_soft_limit_deg": PAN_SOFT_LIMIT,
            "cable_lengths_mm": {k: round(length(v)) for k, v in routes.items()},
            "parts": {n: {"printed": p[2], "group": p[3], "volume_cm3": round(p[0].val().Volume() / 1000, 1)} for n, p in parts.items()},
        }
        with open(os.path.join(OUT, "meta.json"), "w") as f:
            json.dump(meta, f, indent=1)
    return parts, keepouts, routes


configure()

if __name__ == "__main__":
    build()
    print("written to", OUT)
