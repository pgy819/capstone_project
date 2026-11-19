"""Stage 2 v0.1 - lower 3 axes: 3-RRS levelling platform under the v0.9.2-S plan B gimbal (user, 11-20).

Purpose: the base is tilted by hand; three STS3215 (ID 3, 4, 5, same bus) keep the platform - and the plan B gimbal
bolted on it - level, with a BNO085 on the platform as the level sensor.

Layout (base frame: origin at the base plate top centre, z up; legs at BETA = 90, 210, 330 deg)
  base plate  d170 x 6, three servo cradles (servo stands on its far end, shaft horizontal along the tangent),
              crank slots, three feet, two bolt-on handles (0 / 180 deg)
  crank       35 mm, on the servo disc horn (crush-rib pocket + 4 radial slots, same as the Stage 1 turntable)
  rod         60 mm, eye on an M3 x 16 pin (self-taps into the crank, two jam nuts), printed ball d16 on top
  platform    ball sockets (cup in the platform boss + screwed retainer ring) at r 70, plate 5 mm, BNO085 pad,
              4 x M3 x 12 down through the plan B lower deck, cable hole
  plan B      the v0.9.2-S plan B gimbal unchanged except 4 screw holes + 1 cable hole in its lower deck

Only the user's screws: M3 x 12 / 16 (+ M3 nuts), M2.5 x 8.
Run: python stage2.py  -> stage2_v01_package/ (+ .zip)
"""
import csv
import json
import math
import os
import shutil
import sys
import tempfile

import cadquery as cq
import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
MECH = os.path.dirname(HERE)
sys.path.insert(0, MECH)
sys.path.insert(0, HERE)
import gimbal_v03 as g          # noqa: E402  helpers + STS3215 / horn dimensions
import s2_kin as kin            # noqa: E402

G = kin.Geo()
VER = "s2v01"
PKG = os.path.join(HERE, "stage2_v01_package")
PLANB = os.path.join(MECH, "gimbal_v092sb_package", "01_CATIA")
BED = (220.0, 220.0, 200.0)

box, cyl_y, cyl_z = g.box, g.cyl_y, g.cyl_z

# ----------------------------------------------------------------------------- servo + cradle (leg frame: x radial, y tangent)
SV_CLR, WALL = 0.3, 3.0
SV_R0, SV_R1 = G.RB - g.SV_W / 2, G.RB + g.SV_W / 2          # servo case across the radius
SV_TOPZ = G.HS + g.SV_SHAFT_FROM_END                         # case top (servo stands on its far end at z = 0)
BAR_T, BAR_Y = 4.0, (-30.0, -6.0)                            # strap over the servo top (middle 4 mm, ends = EAR_T)
BAR_SCREW_Y = -18.0

# ----------------------------------------------------------------------------- bolted joints (user, 11-20)
# Every fixed joint is a protruding ear pair + M3 x 8 bolt + nut. The material a bolt passes through is at most
# JOINT_MAX in total (two EAR_T ears = 4.4); M3 x 8 = 4.4 + nut 2.4 + 1.2 spare. Ears carry 45 deg ribs beside the
# nut, and a raised hex wall (not under the nut) stops the nut turning so one hand drives the bolt.
JOINT_MAX, EAR_T = 4.5, 2.2
EAR_L, EAR_W = 9.0, 12.0          # strap ears: length out of the wall, width
RIB_T = 1.5
CRADLE_H = 1.6                    # hex wall height around the nut
STRAP_PRELOAD = 0.3               # wall ears sit this much below the servo top: the strap presses the servo

# ----------------------------------------------------------------------------- crank / rod stack along the tangent (y)
CR_Y = (2.5, 9.5)            # crank plate (horn 0.5..4.5 sits 2 mm deep in it)
EYE_Y = (9.7, 14.7)          # rod eye
ROD_Y = (9.7, 15.7)          # rod body; ball centre at T_OFF = 12.7
NUT_Y = (14.9, 19.7)         # two jam nuts
CR_HUB_R, CR_TIP_R = g.HORN_D / 2 + g.HORN_CLR + 2.6, 6.5
PIN_CB = (2.5, 5.5)          # M3 x 16 head counterbored into the crank back face, pilot from 5.5
EYE_R, ROD_W = 6.5, 9.0
BALL_R, CUP_CLR = 8.0, 0.15
NECK_R = 3.0
ROD_TOP_BODY = G.L - 12.0    # body -> neck

# ----------------------------------------------------------------------------- platform (platform frame: z 0 = ball centres)
PT_Z = (10.0, 15.0)          # plate
BOSS_R, RING_H = 14.0, 5.0
RING_SCREW = 18.0            # retainer bolts at +-RING_SCREW along the leg tangent (out of the rod's swing), on ears
RING_EAR = (12.0, 24.0, 6.0)  # ear from / to (tangent) and half width (radial)
CABLE_HOLE = (19.0, 2.0, 10.0)                    # platform x, y, r
BNO = (-26.0, 15.0)                               # pad centre (under the plate)
BNO_HOLES = [(sx * 10.16, sy * d) for d in (6.35, 10.16) for sx in (-1, 1) for sy in (-1, 1)]  # Adafruit 4754 / SparkFun

# plan B: platform centre under its centre of mass, deck bottom on the plate top
PB_CX, PB_CY = -27.0, -6.0
PB_DECK_BOT = -63.2
PB_SHIFT = (-PB_CX, -PB_CY, PT_Z[1] - PB_DECK_BOT)
DECK_SCREWS_PB = [(-26.0, 39.0), (13.0, -30.0), (-65.0, -51.0), (-5.0, 12.0)]    # free spots found on the deck (plan B xy)
DECK_SCREWS = [(x + PB_SHIFT[0], y + PB_SHIFT[1]) for x, y in DECK_SCREWS_PB]

# ----------------------------------------------------------------------------- base
BASE_R, BASE_T = 85.0, 6.0
FEET = [(a, 62.0) for a in (60.0, 120.0, 270.0)]
HANDLE_ANGLES = (0.0, 180.0)
HANDLE_SCREWS = [(95.0, -10.0), (95.0, 10.0)]       # (radial, tangent) in the handle frame
HANDLE_Z = (-16.0, -6.0)
HANDLE_EAR = (82.0, 104.0, 22.0)                    # base-plate ear: radial from / to, half width
DECK_CB = (3.25, 1.0)                               # counterbore in the plan B deck top: deck passes 3.2 -> 2.2
BNO_TAB_Z = (PT_Z[0] - 4.0 - EAR_T, PT_Z[0] - 4.0)  # hanging tab the board bolts to (posts at its corners)


def deg(a):
    return math.degrees(a)


def to_leg(shape, i):
    return shape.rotate((0, 0, 0), (0, 0, 1), deg(G.betas()[i]))


def sphere(r, x=0.0, y=0.0, z=0.0):
    return cq.Workplane("XY").sphere(r).translate((x, y, z))


def nut_cradle(x, y, z_face, up=True, af=None):
    """Raised hex wall around a nut seat on the face at z_face (the bolt does not pass through it)."""
    af = af or g.NUT_POCKET_AF
    z0, z1 = (z_face, z_face + CRADLE_H) if up else (z_face - CRADLE_H, z_face)
    return g.hex_z(af + 2.4, z0, z1, x, y).cut(g.hex_z(af, z0 - 1, z1 + 1, x, y))


def ribs_x(x_wall, x_tip, y_list, z_top, up=True):
    """45 deg triangular ribs (RIB_T thick) between a wall face at x_wall and an ear tip at x_tip, under (or over)
    an ear whose face is z_top."""
    out = None
    ln = abs(x_tip - x_wall)
    s = 1 if x_tip > x_wall else -1
    for y in y_list:
        zf = z_top - ln if up else z_top + ln
        pts = [(x_wall, z_top), (x_wall + s * ln, z_top), (x_wall, zf)]
        r = cq.Workplane("XZ").polyline(pts).close().extrude(-RIB_T).translate((0, y, 0))
        out = r if out is None else out.union(r)
    return out


# ============================================================================= bought parts (leg frame)
def servo_leg():
    s = g.servo_z_up().rotate((0, 0, 0), (0, 1, 0), 90).rotate((0, 0, 0), (0, 0, 1), 90)
    return s.translate((G.RB, 0, G.HS))


def horn_leg():
    return cyl_y(g.HORN_D / 2, g.HORN_GAP, g.HORN_GAP + g.HORN_T, G.RB, G.HS)


def bno_board():
    """Adafruit 4754 outline (25.4 x 17.8 x 1.6) + parts 3 mm under it [VERIFY board]."""
    x, y = BNO
    zb = BNO_TAB_Z[0]
    return box(x - 12.7, x + 12.7, y - 8.9, y + 8.9, zb - 1.6, zb).union(
        box(x - 5, x + 5, y - 5, y + 5, zb - 1.6 - 3, zb - 1.6))


# ============================================================================= printed parts
def cradle_leg():
    """Servo cradle in the leg frame (grows from the base plate top): side walls across the radius, 1 mm lips in
    front of the shaft-face ledge, back wall with the connector opening, two strap posts."""
    c, t = SV_CLR, WALL
    r0, r1 = SV_R0 - c, SV_R1 + c
    y0 = -g.SV_H - c                     # back face of the case
    z1 = SV_TOPZ
    k = box(r0 - t, r0, y0 - t, 0, 0, z1).union(box(r1, r1 + t, y0 - t, 0, 0, z1))
    i = g.SV_CAP_INSET
    k = k.union(box(r0, SV_R0 + i - c, -g.SV_CAP_H, 0, 0, z1)).union(box(SV_R1 - i + c, r1, -g.SV_CAP_H, 0, 0, z1))
    back = box(r0 - t, r1 + t, y0 - t, y0, 0, z1)
    zc = G.HS - g.SV_CONN_X                                   # connector centre height
    back = back.cut(box(G.RB - g.SV_CONN_W / 2 - 1.5, G.RB + g.SV_CONN_W / 2 + 1.5, y0 - t - 1, y0 + 1,
                        zc - g.SV_CONN_H / 2 - 2.5, zc + g.SV_CONN_H / 2 + 2.5))
    k = k.union(back)
    # strap ears out of both side walls (top EAR_T below the servo top - preload), ribs underneath, nut below
    ze = z1 - STRAP_PRELOAD
    for xw, xt in ((r0 - t, r0 - t - EAR_L), (r1 + t, r1 + t + EAR_L)):
        e = box(min(xw, xt), max(xw, xt), BAR_SCREW_Y - EAR_W / 2, BAR_SCREW_Y + EAR_W / 2, ze - EAR_T, ze)
        e = e.union(ribs_x(xw, xt, (BAR_SCREW_Y - EAR_W / 2, BAR_SCREW_Y + EAR_W / 2 - RIB_T), ze - EAR_T))
        xh = (xw + xt) / 2 + (1.0 if xt > xw else -1.0)
        e = e.union(nut_cradle(xh, BAR_SCREW_Y, ze - EAR_T, up=False))
        k = k.union(e.cut(cyl_z(1.7, ze - 20, ze + 1, xh, BAR_SCREW_Y)))
    return k


def strap_holes():
    c, t = SV_CLR, WALL
    r0, r1 = SV_R0 - c, SV_R1 + c
    return [(r0 - t - EAR_L / 2 - 1.0, BAR_SCREW_Y), (r1 + t + EAR_L / 2 + 1.0, BAR_SCREW_Y)]


def bar_leg():
    """Strap: EAR_T thick over its whole span (ends on the wall ears), 4 mm in the middle over the servo."""
    r0, r1 = SV_R0 - SV_CLR - WALL - EAR_L, SV_R1 + SV_CLR + WALL + EAR_L
    z0 = SV_TOPZ
    b = box(r0, r1, BAR_SCREW_Y - EAR_W / 2, BAR_SCREW_Y + EAR_W / 2, z0, z0 + EAR_T).edges("|Z").fillet(2.0)
    b = b.union(box(SV_R0 - SV_CLR, SV_R1 + SV_CLR, BAR_Y[0], BAR_Y[1], z0, z0 + BAR_T).edges("|Z").fillet(2.0))
    for x, y in strap_holes():
        b = b.cut(cyl_z(1.7, z0 - 1, z0 + 10, x, y))
    return b


def base_plate():
    b = cyl_z(BASE_R, -BASE_T, 0)
    for i in range(3):
        b = b.union(to_leg(cradle_leg(), i))
        # slot under the crank / rod eye (they dip close to the plate beyond 15 deg)
        b = b.cut(to_leg(box(48.0, 80.0, 1.0, 21.0, -BASE_T - 1, 0.5).edges("|Z").fillet(4.0), i))
    for a, r in FEET:
        x, y = g.polar(a, r)
        b = b.union(cyl_z(6.0, HANDLE_Z[0], -BASE_T + 0.01, x, y))
    for a in HANDLE_ANGLES:
        r0, r1, hw = HANDLE_EAR
        e = box(r0, r1, -hw, hw, -BASE_T, -BASE_T + EAR_T).edges("|Z").fillet(4.0)
        for tt in (-hw + 4.0, -RIB_T / 2, hw - 4.0 - RIB_T):            # ribs from the plate rim onto the ear
            e = e.union(ribs_x(BASE_R - 0.5, BASE_R - 0.5 + (BASE_T - EAR_T), (tt,), -BASE_T + EAR_T, up=False))
        for rr, tt in HANDLE_SCREWS:
            e = e.union(nut_cradle(rr, tt, -BASE_T + EAR_T, up=True))
            e = e.cut(cyl_z(1.7, -BASE_T - 1, 1, rr, tt))
        b = b.union(e.rotate((0, 0, 0), (0, 0, 1), a))
    b = b.cut(cyl_z(12.0, -BASE_T - 1, 1))                    # centre opening: cable slack / hand under the platform
    return b


def handle(a_deg):
    """Grip loop (10 mm) + a thin tongue (EAR_T) under the base-plate ear; bolts up from below, nut on the ear."""
    r0, r1, hw = HANDLE_EAR
    zt = (-BASE_T - EAR_T, -BASE_T)
    tongue = box(r0, r1 + 2.0, -hw, hw, *zt).edges("|Z").fillet(4.0)
    grip = box(r1, r1 + 46.0, -hw - 6.0, hw + 6.0, *HANDLE_Z).edges("|Z").fillet(8.0)
    grip = grip.cut(box(r1 + 8.0, r1 + 34.0, -hw, hw, HANDLE_Z[0] - 1, HANDLE_Z[1] + 1).edges("|Z").fillet(6.0))
    h = tongue.union(grip)
    for rr, tt in HANDLE_SCREWS:
        h = h.cut(cyl_z(1.7, HANDLE_Z[0] - 1, HANDLE_Z[1] + 1, rr, tt))
    return h.rotate((0, 0, 0), (0, 0, 1), a_deg)


def crank_leg0():
    """Crank at alpha = 0 (pointing +x) in the leg frame."""
    y0, y1 = CR_Y
    pts = [(G.RB, G.HS - 11.0), (G.RB + G.A, G.HS - CR_TIP_R + 0.5), (G.RB + G.A, G.HS + CR_TIP_R - 0.5), (G.RB, G.HS + 11.0)]
    arm = cq.Workplane("XZ").polyline(pts).close().extrude(-(y1 - y0)).translate((0, y0, 0))
    k = cyl_y(CR_HUB_R, y0, y1, G.RB, G.HS).union(arm).union(cyl_y(CR_TIP_R, y0, y1, G.RB + G.A, G.HS))
    k = k.cut(g.horn_pocket(y0 - 0.1, g.HORN_GAP + g.HORN_T, "y").translate((G.RB, 0, G.HS)))
    k = k.cut(g.horn_slots(y0 - 1, y1 + 1, g.HORN_GAP + g.HORN_T + 2.0, "y").translate((G.RB, 0, G.HS)))
    k = k.cut(cyl_y(3.5, y0 - 1, y1 + 1, G.RB, G.HS))                         # horn centre screw
    k = k.cut(cyl_y(3.1, y0 - 1, PIN_CB[1], G.RB + G.A, G.HS))                # pin head counterbore
    k = k.cut(cyl_y(g.PILOT_R, PIN_CB[1] - 0.1, y1 + 1, G.RB + G.A, G.HS))    # M3 self-tap pilot
    return k


def pin_leg0():
    x, z = G.RB + G.A, G.HS
    s = cyl_y(2.75, PIN_CB[0], PIN_CB[1], x, z).union(cyl_y(1.5, PIN_CB[1], PIN_CB[1] + 16.0, x, z))
    for k in range(2):
        y = NUT_Y[0] + k * g.NUT_H
        s = s.union(g.hex_z(g.NUT_AF, 0, g.NUT_H, 0, 0).rotate((0, 0, 0), (1, 0, 0), -90).translate((x, y, z)))
    return s


def rod_local():
    """Rod: pin axis along y through the origin, rod along +z, ball centre (0, T_OFF, L). Printed standing up."""
    yc = G.T_OFF
    r = cyl_y(EYE_R, *EYE_Y).union(box(-ROD_W / 2, ROD_W / 2, *ROD_Y, EYE_R - 1.0, ROD_TOP_BODY))
    r = r.union(box(-ROD_W / 2, ROD_W / 2, *ROD_Y, ROD_TOP_BODY - 0.01, ROD_TOP_BODY + 0.01))
    zc = G.L - BALL_R * math.sqrt(0.5)                     # latitude -45 deg
    zn = zc - (BALL_R * math.sqrt(0.5) - NECK_R)           # 45 deg cone down to the neck
    taper = (cq.Workplane("XY").workplane(offset=ROD_TOP_BODY).center(0, yc).rect(ROD_W, ROD_Y[1] - ROD_Y[0])
             .workplane(offset=zn - ROD_TOP_BODY - 0.5).circle(NECK_R).loft())
    r = r.union(taper).union(cyl_z(NECK_R, zn - 0.6, zn + 0.01, 0, yc))
    cone = (cq.Workplane("XY").workplane(offset=zn).center(0, yc).circle(NECK_R)
            .workplane(offset=zc - zn).circle(BALL_R * math.sqrt(0.5)).loft())
    ball = (sphere(BALL_R, 0, yc, G.L).cut(box(-20, 20, yc - 20, yc + 20, G.L - 30, zc))
            .cut(box(-20, 20, yc - 20, yc + 20, G.L + BALL_R - 0.05, G.L + 30)))   # 0.05 flat: clean mesh pole
    r = r.union(cone).union(ball)
    # M3 clearance hole, teardrop pointing +z (the print direction)
    h = 1.65
    tear = (cq.Workplane("XZ").polyline([(-h * 0.7071, h * 0.7071), (0, h * 1.4142), (h * 0.7071, h * 0.7071), (0, 0)])
            .close().extrude(-(EYE_Y[1] - EYE_Y[0] + 2)).translate((0, EYE_Y[0] - 1, 0)))
    r = r.cut(cyl_y(h, EYE_Y[0] - 1, EYE_Y[1] + 1)).cut(tear)
    return r


def ball_local(i):
    return kin.ball_local(G, G.betas()[i])


def at_ball(shape, i):
    """Shape built at a ball centre in leg axes (x radial, y tangent) -> platform frame."""
    p = ball_local(i)
    return to_leg(shape, i).translate((float(p[0]), float(p[1]), 0.0))


def cup_cutter():
    """Ball seat (sphere + clearance) without its top pole (a 0.05 mm flat keeps the exported mesh closed)."""
    rc = BALL_R + CUP_CLR
    return sphere(rc).cut(box(-20, 20, -20, 20, rc - 0.05, 20))


def ring_local():
    r = cyl_z(BOSS_R, -RING_H, 0).cut(cup_cutter())
    # 45 deg chamfer on the lower edge of the opening: the ball's 45 deg print cone and the neck clear it up to
    # 30 deg of rod tilt; the sphere band z 0..-4 still holds the ball (opening r 7.1 < ball r 8)
    zr = -4.0
    rr = math.sqrt((BALL_R + CUP_CLR) ** 2 - zr * zr)
    ch = (cq.Workplane("XY").workplane(offset=-RING_H - 1.0).circle(rr + (RING_H + 1.0 + zr))
          .workplane(offset=RING_H + 1.0 + zr).circle(rr).loft())
    r = r.cut(ch)
    y0, y1, hx = RING_EAR
    for s in (-1, 1):
        e = box(-hx, hx, *sorted((s * y0, s * y1)), -EAR_T, 0).edges("|Z").fillet(2.0)
        r = r.union(e.cut(cup_cutter())).cut(cyl_z(1.7, -RING_H - 1, 1, 0, s * RING_SCREW))
    return r


def boss_ears():
    """Platform side of the retainer joint: ears on the boss bottom, ribs and nut wall on top (open above: the
    ears reach past the plate arm)."""
    y0, y1, hx = RING_EAR
    out = None
    for s in (-1, 1):
        e = box(-hx, hx, *sorted((s * y0, s * y1)), 0, EAR_T).edges("|Z").fillet(2.0)
        for xr in (-hx, hx - RIB_T):
            yw, yt = s * (BOSS_R - 1.0), s * (BOSS_R - 1.0 + 6.0)
            pts = [(yw, EAR_T), (yt, EAR_T), (yw, EAR_T + 6.0)]
            e = e.union(cq.Workplane("YZ").polyline(pts).close().extrude(RIB_T).translate((xr, 0, 0)))
        e = e.union(nut_cradle(0, s * RING_SCREW, EAR_T, up=True))
        e = e.cut(cyl_z(1.7, -1, EAR_T + 3, 0, s * RING_SCREW))
        out = e if out is None else out.union(e)
    return out


def platform():
    p = cyl_z(46.0, *PT_Z)
    for i in range(3):
        b = ball_local(i)
        a = math.degrees(math.atan2(b[1], b[0]))
        ln = float(np.hypot(b[0], b[1]))
        arm = box(0, ln, -14.0, 14.0, *PT_Z).rotate((0, 0, 0), (0, 0, 1), a)
        p = p.union(arm).union(at_ball(cyl_z(BOSS_R, 0, PT_Z[1]), i)).union(at_ball(boss_ears(), i))
    bx, by = BNO
    p = p.union(box(bx - 17, bx + 17, by - 14, by + 14, *BNO_TAB_Z).edges("|Z").fillet(3.0))   # hanging tab
    for sx in (-1, 1):
        for sy in (-1, 1):
            p = p.union(box(bx + sx * 17 - (5 if sx > 0 else 0), bx + sx * 17 + (0 if sx > 0 else 5),
                            by + sy * 14 - (5 if sy > 0 else 0), by + sy * 14 + (0 if sy > 0 else 5),
                            BNO_TAB_Z[1] - 0.01, PT_Z[0] + 0.01))
    # cuts
    for i in range(3):
        p = p.cut(at_ball(cup_cutter(), i))
    web = PT_Z[1] - EAR_T          # deck bolts: plate web EAR_T under the deck, nut in a hex seat from below
    for x, y in DECK_SCREWS:
        p = p.cut(cyl_z(1.7, PT_Z[0] - 1, PT_Z[1] + 1, x, y)).cut(g.hex_z(g.NUT_POCKET_AF, PT_Z[0] - 1, web, x, y))
    for hx, hy in BNO_HOLES:       # M2.5 bolt + nut, board under the tab, nut on top in the gap between the posts
        p = p.cut(cyl_z(g.M25_CLR, BNO_TAB_Z[0] - 1, BNO_TAB_Z[1] + 1, bx + hx, by + hy))
    p = p.cut(cyl_z(CABLE_HOLE[2], PT_Z[0] - 5, PT_Z[1] + 1, CABLE_HOLE[0], CABLE_HOLE[1]))
    return p


# ============================================================================= plan B
PB_PARTS = ["lower_deck", "top_deck", "pan_cage", "driver_frame", "turntable", "arm_servo", "arm_idle", "tilt_cradle"]
PB_REFS = ["pan_servo", "tilt_servo", "pi5_stack", "nucleo_f401re", "fe_urt_1", "camera", "pan_horn", "tilt_horn",
           "fasteners_base", "fasteners_sa", "fasteners_pan", "fasteners_tilt"]


def planb_shape(name):
    sub = "parts" if name in PB_PARTS else "reference"
    return cq.importers.importStep(os.path.join(PLANB, sub, name + ".step"))


def lower_deck_s2():
    """Plan B lower deck + 4 M3 clearance holes (platform screws) + the cable hole over the platform's."""
    d = planb_shape("lower_deck")
    top = PB_DECK_BOT + 3.2
    for x, y in DECK_SCREWS_PB:
        d = d.cut(cyl_z(1.7, PB_DECK_BOT - 1, PB_DECK_BOT + 10, x, y))
        d = d.cut(cyl_z(DECK_CB[0], top - DECK_CB[1], top + 6.0, x, y))       # head seat: the bolt passes 2.2
    cx, cy = CABLE_HOLE[0] - PB_SHIFT[0], CABLE_HOLE[1] - PB_SHIFT[1]
    d = d.cut(cyl_z(CABLE_HOLE[2], PB_DECK_BOT - 1, PB_DECK_BOT + 10, cx, cy))
    return d


# ============================================================================= assembly at a pose
def parts_local():
    """Every part in its own moving frame. groups: base | crank<i> (leg frame, alpha = 0) | rod (rod frame) |
    platform (platform frame)."""
    P = {}
    P["base_plate"] = (base_plate(), True, "base")
    for k, a in enumerate(HANDLE_ANGLES):
        P[f"handle_{k + 1}"] = (handle(a), True, "base")
    for i in range(3):
        P[f"strap_{i + 1}"] = (to_leg(bar_leg(), i), True, "base")
        P[f"servo_{i + 3}"] = (to_leg(servo_leg(), i), False, "base")
        P[f"crank_{i + 1}"] = (crank_leg0(), True, f"crank{i}")
        P[f"horn_{i + 3}"] = (horn_leg(), False, f"crank{i}")
        P[f"pin_{i + 1}"] = (pin_leg0(), False, f"crank{i}")
        P[f"rod_{i + 1}"] = (rod_local(), True, f"rod{i}")
        P[f"ring_{i + 1}"] = (at_ball(ring_local(), i), True, "platform")
    P["platform"] = (platform(), True, "platform")
    P["bno085"] = (bno_board(), False, "platform")
    for n in PB_PARTS + PB_REFS:
        s = lower_deck_s2() if n == "lower_deck" else planb_shape(n)
        P["pb_" + n] = (s.translate(PB_SHIFT), n in PB_PARTS, "platform")
    return P


def frames(q):
    """4x4 transforms of every group for an IK solution q (base frame)."""
    T = {"base": np.eye(4)}
    Tp = np.eye(4)
    Tp[:3, :3] = q["R"]
    Tp[:3, 3] = [q["x"], q["y"], q["z"]]
    T["platform"] = Tp
    for i, b in enumerate(G.betas()):
        Rb = np.eye(4)
        Rb[:3, :3] = kin.rot(0, 0, b)
        a = q["alpha"][i]
        # crank: rotate about the shaft (leg y axis through (RB, HS)) by -alpha (tip up for alpha > 0)
        Ry = np.eye(4)
        Ry[:3, :3] = kin.rot(0, -a, 0)
        Tc = np.eye(4)
        Tc[:3, 3] = [G.RB, 0, G.HS]
        Ti = np.eye(4)
        Ti[:3, 3] = [-G.RB, 0, -G.HS]
        T[f"crank{i}"] = Rb @ Tc @ Ry @ Ti
        # rod: pin at C, pointing at the ball centre, in the leg plane
        Rinv = Rb[:3, :3].T
        C = Rinv @ q["C"][i]
        Pb = Rinv @ q["P"][i]
        th = math.atan2(Pb[0] - C[0], Pb[2] - C[2])
        Tr = np.eye(4)
        Tr[:3, :3] = kin.rot(0, th, 0)
        Tr[:3, 3] = [C[0], 0, C[2]]
        T[f"rod{i}"] = Rb @ Tr
    return T


def to_mesh(shape, tol=0.08):
    p = os.path.join(tempfile.mkdtemp(), "m.stl")
    cq.exporters.export(shape, p, tolerance=tol, angularTolerance=0.15)
    return trimesh.load(p)


def shape_at(shape, T):
    m = cq.Matrix([list(map(float, T[r])) for r in range(3)])
    return cq.Workplane("XY").add(shape.val().transformGeometry(m) if hasattr(shape, "val") else shape.transformGeometry(m))


# ============================================================================= checks
EXPECTED = {("servo", "horn"), ("horn", "crank"), ("crank", "pin"), ("pin", "rod"),
            ("platform", "ring"), ("platform", "pb_lower_deck"), ("platform", "bno085"),
            ("base_plate", "servo"), ("base_plate", "handle"), ("base_plate", "strap")}


def kind(n):
    return n.split("_")[0] if not n.startswith("pb_") else n


FAILED = []


def checks(P, M):
    """Interference of every pair of groups that move against each other over the tilt sweep (platform attitude
    relative to the base = minus the hand tilt). Returns the worst overlap per pair (mm^3)."""
    groups = {}
    for n, (_, _, grp) in P.items():
        groups.setdefault(grp, []).append(n)
    # plan B: only its lowest parts can meet the legs / base
    skip = {n for n in P if n.startswith("pb_") and n not in ("pb_lower_deck", "pb_nucleo_f401re", "pb_pi5_stack")}
    worst = {}
    poses = [(0.0, 0.0)] + [(t, az) for t in (10.0, 15.0, 20.0) for az in range(0, 360, 30)]
    for tilt, az in poses:
        roll, pitch = math.radians(tilt) * math.cos(math.radians(az)), math.radians(tilt) * math.sin(math.radians(az))
        for dz in ((0.0,) if tilt else (-3.0, 0.0, 3.0)):
            q = kin.pose(G, roll, pitch, dz)
            T = frames(q)
            placed = {n: M[n].copy().apply_transform(T[P[n][2]]) for n in P if n not in skip}
            names = list(placed)
            for a in range(len(names)):
                for b in range(a + 1, len(names)):
                    na, nb = names[a], names[b]
                    ga, gb = P[na][2], P[nb][2]
                    if ga == gb:
                        continue
                    if not (ga.startswith(("crank", "rod")) or gb.startswith(("crank", "rod")) or {ga, gb} == {"base", "platform"}):
                        continue
                    A, B = placed[na], placed[nb]
                    if np.any(A.bounds[1] < B.bounds[0]) or np.any(B.bounds[1] < A.bounds[0]):
                        continue
                    try:
                        v = trimesh.boolean.intersection([A, B], engine="manifold").volume
                    except Exception:
                        FAILED.append(f"{na} x {nb}")
                        continue
                    key = f"{na} x {nb}"
                    if v > worst.get(key, (0.0,))[0] + 1e-6:
                        worst[key] = (round(float(v), 3), tilt, az, dz)
    return worst


def joint_lines():
    """Bolt axes of every fixed joint at home: (name, point on the axis (base frame, platform at home), parts)."""
    q0 = kin.pose(G, 0, 0)
    Tp = frames(q0)["platform"]
    out = []
    for i in range(3):
        Rb = kin.rot(0, 0, G.betas()[i])
        for k, (x, y) in enumerate(strap_holes()):
            out.append((f"strap_{i + 1}.{k}", Rb @ np.array([x, y, 0.0]), [f"strap_{i + 1}", "base_plate"]))
        b = ball_local(i)
        for k, s in enumerate((-1, 1)):
            p = np.array(b) + Rb @ np.array([0.0, s * RING_SCREW, 0.0])
            out.append((f"ring_{i + 1}.{k}", (Tp @ np.r_[p, 1.0])[:3], [f"ring_{i + 1}", "platform"]))
    for k, a in enumerate(HANDLE_ANGLES):
        Ra = kin.rot(0, 0, math.radians(a))
        for j, (rr, tt) in enumerate(HANDLE_SCREWS):
            out.append((f"handle_{k + 1}.{j}", Ra @ np.array([rr, tt, 0.0]), ["base_plate", f"handle_{k + 1}"]))
    for j, (x, y) in enumerate(DECK_SCREWS):
        out.append((f"deck.{j}", (Tp @ np.array([x, y, 0.0, 1.0]))[:3], ["platform", "pb_lower_deck"]))
    bx, by = BNO
    for j, (hx, hy) in enumerate(BNO_HOLES[:4]):
        out.append((f"bno.{j}", (Tp @ np.array([bx + hx, by + hy, 0.0, 1.0]))[:3], ["platform", "bno085"]))
    return out


def joint_stacks(P, M):
    """Material a bolt passes through, per joint: ray along z, 2.3 mm beside the axis (inside the head footprint,
    outside the clearance hole), summed over the joint's parts. Must stay <= JOINT_MAX."""
    T = frames(kin.pose(G, 0, 0))
    res = {}
    for name, p, parts in joint_lines():
        tot = 0.0
        for off in ((2.3, 0), (-2.3, 0), (0, 2.3), (0, -2.3)):
            t = 0.0
            for n in parts:
                m = M[n].copy().apply_transform(T[P[n][2]])
                o = np.array([[p[0] + off[0], p[1] + off[1], -200.0]])
                loc, _, _ = m.ray.intersects_location(o, np.array([[0, 0, 1.0]]))
                z = np.sort(loc[:, 2]) if len(loc) else np.array([])
                # keep only the hits within 15 mm of the joint height (other features of a big part do not count)
                z = z[: len(z) // 2 * 2]
                ins = [(z[k], z[k + 1]) for k in range(0, len(z), 2)]
                t += sum(b - a for a, b in ins if abs((a + b) / 2 - near_z(name, p)) < 8.0)
            tot = max(tot, t)
        res[name] = round(float(tot), 2)
    return res


def near_z(name, p):
    return JOINT_Z[name.split(".")[0].split("_")[0]](p)


JOINT_Z = {
    "strap": lambda p: SV_TOPZ,
    "handle": lambda p: -BASE_T,
    "ring": lambda p: G.z0(),
    "deck": lambda p: G.z0() + PT_Z[1],
    "bno": lambda p: G.z0() + BNO_TAB_Z[0],
}


# ============================================================================= export
def print_orient(name, shape):
    """Rotate a printed part into its print pose (flat face on the bed)."""
    if name.startswith("rod"):
        return shape                                       # stands up, eye on the bed
    if name.startswith("crank"):
        return shape.rotate((0, 0, 0), (1, 0, 0), -90)     # back face (y = 2.5) on the bed
    if name.startswith("strap"):
        return shape
    if name == "platform":
        return shape.rotate((0, 0, 0), (1, 0, 0), 180)     # plate top on the bed, bosses up
    if name.startswith("ring"):
        return shape                                       # deck face down: inner sphere narrows upward (<= 45 deg)
    return shape


def bed_fit(shape):
    bb = shape.val().BoundingBox()
    return round(bb.xlen, 1), round(bb.ylen, 1), round(bb.zlen, 1)


def main():
    if os.path.exists(PKG):
        shutil.rmtree(PKG)
    for d in ("01_CATIA/parts", "01_CATIA/reference", "02_PRINT", "03_DOCS"):
        os.makedirs(os.path.join(PKG, d), exist_ok=True)
    P = parts_local()
    q0 = kin.pose(G, 0, 0)
    T0 = frames(q0)
    # assembly STEP at home
    asm = cq.Assembly(name="stage2_v01")
    for n, (s, printed, grp) in P.items():
        placed = shape_at(s, T0[grp])
        asm.add(placed, name=n, color=cq.Color(0.85, 0.55, 0.2) if printed and not n.startswith("pb_") else cq.Color(0.6, 0.6, 0.65))
        sub = "parts" if printed else "reference"
        if not n.startswith("pb_") or n == "pb_lower_deck":
            cq.exporters.export(placed, os.path.join(PKG, "01_CATIA", sub, n + ".step"))
    asm.save(os.path.join(PKG, "01_CATIA", "stage2_v01_assembly.step"))
    # print STLs (one of each kind)
    rows = []
    once = {"crank_1": 3, "rod_1": 3, "ring_1": 3, "strap_1": 3, "handle_1": 2, "base_plate": 1, "platform": 1,
            "pb_lower_deck": 1}
    for n, cnt in once.items():
        s = P[n][0]
        if n.startswith("ring"):
            s = ring_local()
        if n.startswith("strap"):
            s = bar_leg()
        if n.startswith("handle"):
            s = handle(0.0)
        s = print_orient(n, s)
        bb = s.val().BoundingBox()
        s = s.translate((-bb.center.x, -bb.center.y, -bb.zmin))
        fn = {"crank_1": "S01_crank", "rod_1": "S02_rod", "ring_1": "S03_ring", "strap_1": "S04_strap",
              "handle_1": "S05_handle", "base_plate": "S06_base_plate", "platform": "S07_platform",
              "pb_lower_deck": "P02_lower_deck_s2"}[n]
        cq.exporters.export(s, os.path.join(PKG, "02_PRINT", fn + ".stl"), tolerance=0.03, angularTolerance=0.1)
        sz = bed_fit(s)
        rows.append([fn, cnt, sz, all(a <= b for a, b in zip(sz, BED)), round(s.val().Volume() / 1000, 1)])
    with open(os.path.join(PKG, "02_PRINT", "print_list.csv"), "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["file", "qty", "size_mm", "fits_220x220x200", "volume_cm3"])
        w.writerows(rows)
    # meshes for checks / viewer (local frames)
    M = {n: to_mesh(s, 0.05 if n.startswith(("rod", "ring", "platform", "crank")) else 0.1) for n, (s, _, _) in P.items()}
    res = {"kin": json.load(open(os.path.join(HERE, "s2_kin.json"))), "print": rows}
    bad = [n for n, m in M.items() if not m.is_watertight]
    res["non_watertight"] = bad
    res["joint_stack_mm"] = joint_stacks(P, M)
    res["joint_max_mm"] = JOINT_MAX
    res["joint_ok"] = all(v <= JOINT_MAX for v in res["joint_stack_mm"].values())
    res["interference"] = checks(P, M)
    res["failed_pairs"] = sorted(set(FAILED))
    res["unexpected"] = {k: v for k, v in res["interference"].items()
                         if not any({kind(k.split(" x ")[0]), kind(k.split(" x ")[1])} == set(e) for e in EXPECTED)}
    json.dump(res, open(os.path.join(PKG, "03_DOCS", "checks_s2v01.json"), "w"), indent=1, ensure_ascii=False, default=str)
    # meshes for the viewer
    out = os.path.join(HERE, "viewer_meshes")
    os.makedirs(out, exist_ok=True)
    meta = {}
    for n, m in M.items():
        m.export(os.path.join(out, n + ".stl"))
        meta[n] = {"group": P[n][2], "printed": P[n][1]}
    json.dump(meta, open(os.path.join(out, "meta.json"), "w"))
    shutil.make_archive(PKG, "zip", PKG)
    print(json.dumps({k: v for k, v in res.items() if k != "kin"}, indent=1, ensure_ascii=False, default=str))


if __name__ == "__main__":
    main()
