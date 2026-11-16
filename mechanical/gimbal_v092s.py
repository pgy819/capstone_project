"""v0.9.2-S STANDALONE (user, 11-16): v0.9.2 without the lower stage (Stage 2), so it has to stand on a desk by itself
and carry its own electronics. v0.9.2 stays the main model; this file only adds to it.

Changes on top of v0.9.2
  1. FE-URT-1 servo driver on the top deck over the Pi: a separate open DRIVER FRAME (4 standoffs, 2 x M3 x 8 into
     the deck, board on 4 x M2.5 x 8). Kept outside the pan sweep (r 58.4 -> frame edge at r >= 61).
  2. NUCLEO-F401RE under the lower deck. The lower deck could not take it (HDMI plugs in front of the Pi), so a STM32
     TRAY with four legs goes under the lower deck: the tray is the foot of the whole gimbal, the board sits on it
     component side up on 3 standoffs (its real M3 holes, read from ST's MB1136 3D model), and the legs carry the
     lower deck at the four pillar positions (M3 x 12 from below into the pillar roots).
  3. Servo bus daisy chain driver -> Pan -> Tilt. The Pan -> Tilt cable rises behind the camera ribbon (same deck
     slot, widened for the plug), crosses the hub in the ribbon gap, goes up the centre hole, along the table, through
     a window in the servo arm and under the tilt-servo load plate, and up to the plug through a notch in the plate's
     back edge (the "hole in the bottom of the tilt cage").
     The gap over the hub is raised 2 -> 5 mm so a 5264 plug passes it (table and tilt axis +3 mm).
  4. Only the user's screws: M2 x 6, M2.5 x 8, M3 x 8 / 12 / 16 (+ M3 nuts). Cage bolts M3 x 10 -> M3 x 12 (ears 5 -> 7),
     camera stop screws M2 x 5 -> M2 x 6.

Run: python gimbal_v092s.py  -> gimbal_v092s_package/ (+ .zip)"""
import csv
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
VER = "v092s"
PKG = os.path.join(HERE, f"gimbal_{VER}_package")
BED = (220.0, 220.0, 200.0)

# ----------------------------------------------------------------------------- v0.9.2 parameters changed here
g.AX_SLOT = 5.0          # ribbon gap over the hub: 2 -> 5 so the 5264 plug of the Pan -> Tilt cable passes (4.4 thick)
g.EAR_CLAMP = 7.0        # cage ears 5 -> 7: M3 x 12 = ear 7 + deck 5 (there is no M3 x 10)
g.CAGE_BOLT_L = 12.0
g.CAM_SCREW_L = 6.0      # camera stop screws M2 x 6 (there is no M2 x 5)
gp.configure()           # v0.9.2 variant (axial); DISC_Z0 and H_TILT follow AX_SLOT

# ----------------------------------------------------------------------------- new parameters
# FE-URT-1 (catalogue: 55.8 x 36.6 x 11.5, mounting 48.8 x 28.8; hole size not published -> M2.5 passes either)
DRV_L, DRV_W, DRV_H = 55.8, 36.6, 11.5
DRV_HOLE = (28.8 / 2, 48.8 / 2)            # half spacing along x (board width) and y (board length)
DRV_X = -79.5                              # board centre on the deck (long side along y)
FRAME_T, FRAME_BAR, FRAME_SO = 3.0, 8.0, 6.0   # frame base, bar width, standoff height above the base
DRV_DECK_SCREWS = [(DRV_X - DRV_HOLE[0], 0.0), (DRV_X + DRV_HOLE[0], 0.0)]   # on the solid rib between the vent rows
TOP_WIRE_HOLE = (DRV_X, 42.0, 20.0, 8.0)   # driver wires down through the top deck (x, y, w, h)

# NUCLEO-64 (MB1136): from ST's 3D model (NUCLEO-F411RE, same board): PCB x -43.89..26.11, y -2.5..80 (70 x 82.5),
# M3 holes (oval 3.3 x 4.9 along y) at (0, 0), (15.24, 50.8), (-33.02, 52.07); mini USB at the y = 80 edge.
NUC_PCB = (-43.89, 26.11, -2.5, 80.0)
NUC_HOLES = [(0.0, 0.0), (15.24, 50.8), (-33.02, 52.07)]
NUC_BLOCKS = [   # (x0, x1, y0, y1, z0, z1) in board coordinates, PCB from z -0.75 to 0.75
    (-41.91, -36.83, 1.27, 49.53, -8.79, 9.17), (19.05, 24.13, 1.27, 49.53, -8.79, 9.17),     # morpho (pins both sides)
    (14.03, 16.45, 1.02, 48.77, 0.75, 9.29), (-34.23, -31.81, 1.02, 39.62, 0.75, 9.29),       # arduino headers
    (-1.78, 8.12, 72.47, 81.47, 0.75, 4.79),                                                  # mini USB
    (-20.91, -15.91, 57.74, 69.24, 0.75, 4.29), (-20.15, 2.37, 47.8, 53.8, 0.75, 11.49),     # ST-LINK MCU, buttons
    (-22.86, -20.32, 68.57, 78.75, -1.76, 9.17), (-36.32, -34.8, 57.66, 71.88, -1.76, 9.17),  # CN2, CN4 (SWD)
]
NUC_X0 = -17.0          # deck x = NUC_X0 - y_board  (USB end towards the back edge x = -100)
NUC_Y0 = 8.89           # deck y = x_board + NUC_Y0  (board centred on the deck centre line)
NUC_SO = 10.0           # standoffs (bottom morpho pins stand 8.0 below the board)
TRAY_T = 3.0
LEG_H = 40.0            # tray top -> lower-deck underside: standoff 10 + PCB 1.6 + 28 for plugged jumper wires
LEG_R = g.PILLAR_R      # legs continue the pillars (concentric with the r 6 deck-corner fillets)
LEG_SKIN = 4.0          # leg top skin under the deck (M3 x 12 = skin 4 + deck 3.2 + 4.8 into the pillar root)
NUC_PADS = [(-94.5, 31.5), (-94.5, -31.5)]   # unscrewed rests under the USB end corners [VERIFY nothing there]
LOWER_WIRE_HOLES = [(-29.0, 25.0, 14.0, 30.0), (-70.0, 49.5, 20.0, 8.0)]   # STM32 wires up (mirror of the HDMI slot, back)

# Pan -> Tilt bus cable
BUS_T, BUS_W = 1.6, 3.8                      # 3-core servo cable
PLUG = (10.0, 4.5, 6.5)                      # 5264-3P housing w x t x l [VERIFY]
BUS_RISE_X = g.AX_RISE_X + 1.0 + BUS_T / 2   # riser right behind the camera ribbon
ARM_WIN = (11.0, 6.0)                        # window in the servo arm plate under the load plate (w x h)
PLATE_NOTCH = (11.0, 3.5)                    # notch in the back edge of the load plate (w x depth)


def zb_lower():
    return g.LOWER_TOP - g.LOWER_T


def z_tray_top():
    return zb_lower() - LEG_H


def nuc_to_deck(xb, yb):
    return NUC_X0 - yb, xb + NUC_Y0


def nuc_box(x0, x1, y0, y1, z0, z1):
    (xa, ya), (xb, yb) = nuc_to_deck(x0, y0), nuc_to_deck(x1, y1)
    zp = z_tray_top() + NUC_SO + 0.75
    return g.box(min(xa, xb), max(xa, xb), min(ya, yb), max(ya, yb), zp + z0, zp + z1)


# ----------------------------------------------------------------------------- bought-part envelopes
def nucleo_model():
    m = nuc_box(*NUC_PCB, -0.75, 0.75).edges("|Z").fillet(1.0)
    for b in NUC_BLOCKS:
        m = m.union(nuc_box(*b))
    for hx, hy in NUC_HOLES:
        x, y = nuc_to_deck(hx, hy)
        m = m.cut(g.cyl_z(1.6, z_tray_top(), z_tray_top() + 30, x, y))
    return m


def driver_model():
    z0 = FRAME_T + FRAME_SO
    pcb = g.box(DRV_X - DRV_W / 2, DRV_X + DRV_W / 2, -DRV_L / 2, DRV_L / 2, z0, z0 + 1.6)
    top = g.box(DRV_X - DRV_W / 2 + 7, DRV_X + DRV_W / 2 - 7, -DRV_L / 2, DRV_L / 2, z0 + 1.6, z0 + DRV_H - 1.5)   # hole strips kept free
    m = pcb.union(top)
    for sx in (1, -1):
        for sy in (1, -1):
            m = m.cut(g.cyl_z(1.5, z0 - 1, z0 + 2, DRV_X + sx * DRV_HOLE[0], sy * DRV_HOLE[1]))
    return m


# ----------------------------------------------------------------------------- new printed parts
def driver_frame():
    """Open frame on the top deck: two long bars under the board's hole lines, two cross bars, four standoffs.
    Prints flat (base on the plate). The vents under it stay open between the bars."""
    hx, hy = DRV_HOLE
    b = FRAME_BAR / 2
    f = None
    for sx in (1, -1):
        bar = g.box(DRV_X + sx * hx - b, DRV_X + sx * hx + b, -hy - b, hy + b, 0, FRAME_T)
        f = bar if f is None else f.union(bar)
    for sy in (1, -1):
        f = f.union(g.box(DRV_X - hx - b, DRV_X + hx + b, sy * hy - b, sy * hy + b, 0, FRAME_T))
    f = f.edges("|Z").fillet(1.5)
    for sx in (1, -1):
        for sy in (1, -1):
            x, y = DRV_X + sx * hx, sy * hy
            f = f.union(g.cyl_z(3.2, FRAME_T - 0.01, FRAME_T + FRAME_SO, x, y))
            f = f.union(cq.Workplane("XY").workplane(offset=FRAME_T - 0.01).center(x, y).circle(4.5)
                        .workplane(offset=1.3).circle(3.2).loft())
            f = f.cut(g.cyl_z(g.M25_PILOT_R, -1, FRAME_T + FRAME_SO + 1, x, y))       # M2.5 self-taps
    for x, y in DRV_DECK_SCREWS:
        f = f.cut(g.cyl_z(1.7, -1, FRAME_T + 1, x, y))
    return f


def stm32_tray():
    """Foot of the whole gimbal: plate with the deck outline, four hollow legs under the pillars (screw from below),
    three standoffs at the Nucleo's own holes and two rests under the USB end. Prints plate down, legs up."""
    zt = z_tray_top()
    plate = g.box(g.DECK_X0, g.DECK_X1, -g.DECK_Y / 2, g.DECK_Y / 2, zt - TRAY_T, zt).edges("|Z").fillet(6.0)
    zl = zb_lower()
    for x, y in g.PILLARS:
        leg = g.cyl_z(LEG_R, zt - 0.01, zl, x, y)
        root = (cq.Workplane("XY").workplane(offset=zt - 0.01).center(x, y).circle(LEG_R + 3).workplane(offset=3).circle(LEG_R).loft()
                .intersect(g.box(g.DECK_X0 + 0.4, g.DECK_X1 - 0.4, -g.DECK_Y / 2 + 0.4, g.DECK_Y / 2 - 0.4, zt - 1, zt + 4)))
        plate = plate.union(leg, clean=False).union(root, clean=False)
        plate = plate.cut(g.cyl_z(3.2, zt - TRAY_T - 1, zl - LEG_SKIN, x, y))          # head + screwdriver bore
        plate = plate.cut(g.cyl_z(1.7, zl - LEG_SKIN - 1, zl + 1, x, y))
    plate = plate.clean()
    for hx, hy in NUC_HOLES:
        x, y = nuc_to_deck(hx, hy)
        plate = plate.union(g.cyl_z(3.5, zt - 0.01, zt + NUC_SO, x, y))
        plate = plate.union(cq.Workplane("XY").workplane(offset=zt - 0.01).center(x, y).circle(5.0).workplane(offset=1.5).circle(3.5).loft())
        plate = plate.cut(g.cyl_z(g.PILOT_R, zt - 1.0, zt + NUC_SO + 1, x, y))           # M3 self-taps, 1 mm floor kept
    for x, y in NUC_PADS:
        plate = plate.union(g.cyl_z(2.5, zt - 0.01, zt + NUC_SO, x, y))
    return plate


# ----------------------------------------------------------------------------- changes to v0.9.2 parts
def top_deck_mod(deck):
    hw = PLUG[0] / 2 + 0.5
    deck = deck.cut(g.box(-BUS_RISE_X - PLUG[1] / 2 - 0.5, -g.AX_RISE_X + 1.0, -hw, hw, -g.DECK_T - 1, 1))   # plug passes
    x, y, w, h = TOP_WIRE_HOLE
    deck = deck.cut(g.box(x - w / 2, x + w / 2, y - h / 2, y + h / 2, -g.DECK_T - 1, 1).edges("|Z").fillet(h / 2 - 0.01))
    for x, y in DRV_DECK_SCREWS:
        deck = deck.cut(g.cyl_z(g.PILOT_R, -g.DECK_T - 1, 1, x, y))
    return deck


def lower_deck_mod(deck):
    zb = zb_lower()
    for cx, cy, w, h in LOWER_WIRE_HOLES:
        deck = deck.cut(g.box(cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2, zb - 1, g.LOWER_TOP + 1).edges("|Z").fillet(min(w, h) / 2 - 0.01))
    for x, y in g.PILLARS:
        deck = deck.cut(g.cyl_z(g.PILOT_R, zb - 1, zb + 9.0, x, y))
    return deck


def arm_servo_mod(arm):
    """Window through the arm plate (and its gussets) right under the load plate, notch in the plate's back edge."""
    z_bot = g.H_TILT - (g.SV_L - g.SV_SHAFT_FROM_END)
    z_plate = z_bot - g.CH_SHELF
    w, h = ARM_WIN
    z1 = z_plate - 0.8
    win = g.box(-w / 2, w / 2, g.Y_SRV_OUT - g.ARM_T_SERVO - 8.0, g.Y_SRV_OUT + 1.0, z1 - h, z1)
    arm = arm.cut(win.edges("|Y").fillet(1.5))
    y_back = g.Y_SRV_OUT + g.SV_H + g.CH_CLR
    nw, nd = PLATE_NOTCH
    arm = arm.cut(g.box(-nw / 2, nw / 2, y_back + g.CH_LIP - nd, y_back + g.CH_LIP + 1, z_plate - 1, z_bot + 0.01).edges("|Z").fillet(1.0))
    return arm


# ----------------------------------------------------------------------------- fasteners of the new joints
def screw(p, d, length, head_d, head_h, shank_r):
    """Socket-head screw, head seat at p, shank along unit axis d (only +-Z used here)."""
    x, y, z = p
    sg = 1 if d[2] > 0 else -1
    g.note_screw(p, d, length)
    head = g.cyl_z(head_d / 2, *sorted((z, z - sg * head_h)), x, y)
    shank = g.cyl_z(shank_r, *sorted((z, z + sg * length)), x, y)
    return head.union(shank)


def fasteners_sa():
    out = []
    for x, y in DRV_DECK_SCREWS:                                     # frame -> deck: M3 x 8 (3 + 5)
        out.append(screw((x, y, FRAME_T), (0, 0, -1), 8.0, 5.5, 3.0, 1.5))
    z_pcb_top = FRAME_T + FRAME_SO + 1.6
    for sx in (1, -1):                                               # driver -> frame: M2.5 x 8
        for sy in (1, -1):
            out.append(screw((DRV_X + sx * DRV_HOLE[0], sy * DRV_HOLE[1], z_pcb_top), (0, 0, -1), 8.0, g.M25_HEAD_D, g.M25_HEAD_H, 1.25))
    zp = z_tray_top() + NUC_SO + 1.5
    for hx, hy in NUC_HOLES:                                         # Nucleo -> tray: M3 x 12 (1.6 + 10.4)
        x, y = nuc_to_deck(hx, hy)
        out.append(screw((x, y, zp), (0, 0, -1), 12.0, 5.5, 3.0, 1.5))
    zs = zb_lower() - LEG_SKIN
    for x, y in g.PILLARS:                                           # tray legs -> pillar roots: M3 x 12 from below
        out.append(screw((x, y, zs), (0, 0, 1), 12.0, 5.5, 3.0, 1.5))
    f = out[0]
    for s in out[1:]:
        f = f.union(s)
    return f


# ----------------------------------------------------------------------------- cables
def bus_fixed(t=BUS_T):
    """Fixed part of the Pan -> Tilt cable (riser behind the ribbon, leg over the ribbon, first bit up the axis)."""
    hw = BUS_W / 2
    z_leg = g.AX_LEG_Z0 + 0.4
    rise = g.box(-BUS_RISE_X - t / 2, -BUS_RISE_X + t / 2, -hw, hw, -g.DECK_T - 30, z_leg + t)
    leg = g.box(-BUS_RISE_X, 0, -hw, hw, z_leg, z_leg + t)
    return rise.union(leg)


def routes():
    R = g.cable_routes_axial()
    zl = g.AX_LEG_Z0 + 1.2
    pan_conn = (g.SV_CONN_X, 0, g.SV_TOP - g.SV_H - 2.0 - g.SV_PLUG / 2)
    cx, cy, cz = g.tilt_conn_point()
    z_bot = g.H_TILT - (g.SV_L - g.SV_SHAFT_FROM_END)
    z_tab = g.DISC_Z0 + g.DISC_T
    y_back = g.Y_SRV_OUT + g.SV_H + g.CH_CLR
    z_win = z_bot - g.CH_SHELF - 0.8 - ARM_WIN[1] / 2
    R["Pan->Tilt servo bus"] = [pan_conn, (g.SV_CONN_X, -4, -42), (-BUS_RISE_X, -4, -40), (-BUS_RISE_X, 0, -g.DECK_T - 2),
                                (-BUS_RISE_X, 0, zl), (0, 0, zl), (0, 0, z_tab + 2), (0, 12, z_tab + 1.5), (0, 18, z_win),
                                (0, 35, z_bot - g.CH_SHELF - 1.5), (0, y_back + 1.5, z_bot - g.CH_SHELF - 1.0),
                                (0, y_back + 1.5, cz - 6), (cx, cy, cz)]
    del R["tilt servo bus"]
    tx, ty, _, _ = TOP_WIRE_HOLE
    drv_port = (DRV_X, DRV_L / 2, FRAME_T + FRAME_SO + 6)
    R["driver->Pan servo bus"] = [drv_port, (tx, ty, 2), (tx, ty, -g.DECK_T - 3), (tx, 49, -12), (-20, 30, -12),
                                  (g.SV_L - g.SV_SHAFT_FROM_END + 6, 18, -20), (g.SV_L - g.SV_SHAFT_FROM_END + 6, 0, pan_conn[2]), pan_conn]
    nx, ny = nuc_to_deck(-40, 25)
    zh = z_tray_top() + NUC_SO + 12
    lx, ly = LOWER_WIRE_HOLES[1][:2]
    R["STM32<->driver UART"] = [(nx, ny, zh), (lx, ly, zb_lower() - 3), (lx, ly, g.LOWER_TOP + 3), (tx, 49, -12),
                                (tx, ty, -g.DECK_T - 3), (tx, ty, 2), (DRV_X + 10, DRV_L / 2, FRAME_T + FRAME_SO + 4)]
    hat_top = g.LOWER_TOP + g.PI_STANDOFF + g.PI_STACK_H
    gx, gy = g.pi_to_deck(sum(g.PI_GPIO_X) / 2, g.PI_W - 3)
    R["STM32<->Pi UART"] = [(gx, gy, hat_top + 8), (gx + 6, 46, hat_top + 10), (lx, ly, g.LOWER_TOP + 3), (lx, ly, zb_lower() - 3), (nx, ny, zh)]
    ux, uy = nuc_to_deck(3.2, 81.5)
    R["STM32 USB (ST-LINK)"] = [(ux, uy, z_tray_top() + NUC_SO + 3), (g.DECK_X0 - 8, uy, z_tray_top() + NUC_SO + 3),
                                (g.DECK_X0 - 8, 60, g.LOWER_TOP + 10), (-80, 60, g.LOWER_TOP + 12)]
    del R["servo bus to STM32"], R["UART to STM32"]
    return R


# ----------------------------------------------------------------------------- assembly
def build():
    real_chamfer = g.chamfer_holes
    g.chamfer_holes = lambda wp, name="part", mates=(): wp          # chamfer once, at the end, with every screw known
    try:
        parts, keepouts, _ = g.build(export=False)
        yoke, a, b, c = g.turntable_yoke()
        tt, arm_s, arm_i = gp.split_yoke(yoke, a, b, c)
    finally:
        g.chamfer_holes = real_chamfer
    fsa = fasteners_sa()                                               # adds the new screws to g.SCREWS
    P = {}
    P["stm32_tray"] = (stm32_tray(), True, "base")
    P["lower_deck"] = (lower_deck_mod(parts["lower_deck"][0]), True, "base")
    P["top_deck"] = (top_deck_mod(parts["top_deck"][0]), True, "base")
    P["pan_cage"] = (parts["pan_cage"][0], True, "base")
    P["driver_frame"] = (driver_frame(), True, "base")
    P["turntable"] = (tt, True, "pan")
    P["arm_servo"] = (arm_servo_mod(arm_s), True, "pan")
    P["arm_idle"] = (arm_i, True, "pan")
    P["tilt_cradle"] = (parts["tilt_cradle"][0], True, "tilt")
    for n in ("pan_servo", "pi5_stack"):
        P[n] = (parts[n][0], False, "base")
    P["nucleo_f401re"] = (nucleo_model(), False, "base")
    P["fe_urt_1"] = (driver_model(), False, "base")
    for n in ("pan_horn", "tilt_servo"):
        P[n] = (parts[n][0], False, "pan")
    for n in ("tilt_horn", "camera"):
        P[n] = (parts[n][0], False, "tilt")
    P["fasteners_base"] = (parts["fasteners_base"][0], False, "base")
    P["fasteners_sa"] = (fsa, False, "base")
    P["fasteners_pan"] = (parts["fasteners_pan"][0], False, "pan")
    P["fasteners_tilt"] = (parts["fasteners_tilt"][0], False, "tilt")
    horn = parts["pan_horn"][0].val()
    printed = [n for n, v in P.items() if v[1]]
    for n in printed:
        mates = [P[m][0].val() for m in P if m != n and not m.startswith("fasteners")]
        if n == "turntable":
            mates = [P["arm_servo"][0].val(), P["arm_idle"][0].val(), horn]
        P[n] = (g.chamfer_holes(P[n][0], n, mates),) + P[n][1:]
    keepouts["keepout pan servo plugs"] = g.servo_conn_keepout_z_up().translate((0, 0, g.SV_TOP))
    return P, keepouts


EXPECTED = {
    ("pan_servo", "pan_horn"), ("turntable", "pan_horn"), ("tilt_servo", "tilt_horn"), ("tilt_cradle", "tilt_horn"),
    ("tilt_cradle", "camera"), ("arm_idle", "tilt_cradle"), ("turntable", "arm_servo"), ("turntable", "arm_idle"),
    ("pan_horn", "fasteners_pan"), ("turntable", "fasteners_pan"), ("arm_servo", "fasteners_pan"), ("arm_idle", "fasteners_pan"),
    ("camera", "fasteners_tilt"), ("tilt_cradle", "fasteners_tilt"),
    ("pi5_stack", "fasteners_base"), ("lower_deck", "fasteners_base"), ("top_deck", "fasteners_base"), ("pan_cage", "fasteners_base"),
    ("fe_urt_1", "fasteners_sa"), ("driver_frame", "fasteners_sa"), ("top_deck", "fasteners_sa"),
    ("nucleo_f401re", "fasteners_sa"), ("stm32_tray", "fasteners_sa"), ("lower_deck", "fasteners_sa"),
    ("top_deck", "pan_cage"), ("lower_deck", "stm32_tray"), ("top_deck", "driver_frame"), ("driver_frame", "fe_urt_1"),
    ("stm32_tray", "nucleo_f401re"), ("lower_deck", "pi5_stack"),
}
SELF_TAP = {"top_deck", "lower_deck", "stm32_tray", "driver_frame", "tilt_cradle", "pan_cage", "turntable"}


def checks(P, K):
    M = {n: e.mesh("sa_" + n, v[0]) for n, v in P.items()}
    KM = {n: e.mesh("sa_" + n, v) for n, v in K.items()}
    res = {"static": [], "touching_by_design_mm3": {}}
    allm = {**M, **KM}
    names = list(allm)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if a.startswith("keepout") and b.startswith("keepout"):
                continue
            v = e.overlap(allm[a], allm[b])
            if (a, b) in EXPECTED or (b, a) in EXPECTED:
                if v > 0.5:
                    res["touching_by_design_mm3"][f"{a} x {b}"] = round(v, 1)
                continue
            if v > 0.5:
                res["static"].append(f"{a} x {b} {v:.1f} mm3")
    # screws must sit in holes: fastener volume inside a printed part = self-tapping threads only (shank r 1.5 in a
    # pilot of r 1.3 / 1.05) -> report the worst ratio of overlap to the thread ring a pilot would explain
    pan = [n for n, v in P.items() if v[2] in ("pan", "tilt") and n != "pan_horn"]
    base = [n for n, v in P.items() if v[2] == "base" and n not in ("pan_servo",) and not n.startswith("fasteners")]
    mp = trimesh.boolean.union([M[n] for n in pan] + [KM["keepout tilt servo plugs"]])
    mb = trimesh.boolean.union([M[n] for n in base] + [KM[k] for k in KM if "tilt" not in k])
    res["pan_overlap_max_mm3"] = round(max(e.overlap(e.rot(mp, s * d, [0, 0, 1], [0, 0, 0]), mb)
                                           for d in (15, 45, 90, 125, 135) for s in (-1, 1)), 2)
    # clearance from the swept pan group to the new driver frame + board
    drv = trimesh.util.concatenate([M["driver_frame"], M["fe_urt_1"]])
    rr = np.hypot(mp.vertices[:, 0], mp.vertices[:, 1])
    low = mp.vertices[:, 2] < drv.bounds[1][2] + 2
    res["pan_sweep_r_below_driver_top_mm"] = round(float(rr[low].max()), 1)
    res["driver_min_r_mm"] = round(float(np.hypot(drv.vertices[:, 0], drv.vertices[:, 1]).min()), 1)
    # cables crossing the pan joint (camera ribbon + Pan->Tilt bus) against everything that turns
    rib = trimesh.boolean.union([e.mesh("fpc_fixed", g.fpc_fixed()), e.mesh("bus_fixed", bus_fixed())])
    pts = rib.sample(30000)
    free = [d for d in range(-180, 181, 5) if not e.rot(mp, d, [0, 0, 1], [0, 0, 0]).contains(pts).any()]
    res["pan_cable_free_deg"] = [min(free), max(free)]
    res["pan_soft_limit_deg"] = g.PAN_SOFT_LIMIT
    assert min(free) <= -g.PAN_SOFT_LIMIT and max(free) >= g.PAN_SOFT_LIMIT, "cables block the pan range"
    res["cable_vs_fixed_mm3"] = round(e.overlap(rib, trimesh.boolean.union([M[n] for n in ("pan_cage", "pan_servo", "top_deck")])), 2)
    tg = trimesh.boolean.union([M[n] for n in ("tilt_cradle", "camera", "tilt_horn", "fasteners_tilt")])
    fixed = trimesh.boolean.union([M[n] for n in ("turntable", "arm_servo", "arm_idle", "tilt_servo")] + [KM["keepout tilt servo plugs"]])
    base0 = e.overlap(tg, fixed)
    res["tilt_overlap_max_mm3"] = round(max(e.overlap(e.rot(tg, d, [0, 1, 0], [0, 0, g.H_TILT]), fixed) for d in range(-90, 91, 10)) - base0, 2)
    # plug passes: arm window and plate notch against the 5264 housing
    res["arm_window_mm"] = list(ARM_WIN)
    res["plate_notch_mm"] = list(PLATE_NOTCH)
    res["hub_gap_mm"] = g.AX_SLOT
    res["plug_mm"] = list(PLUG)
    R = routes()
    res["cable_mm"] = {k: round(g.length(v)) for k, v in R.items()}
    res["fpc_budget_mm"] = g.FPC_BUDGET
    assert res["cable_mm"]["camera FPC"] <= g.FPC_BUDGET
    selfm = trimesh.util.concatenate([M[n] for n in ("turntable", "arm_servo", "arm_idle", "tilt_servo", "fasteners_pan")])
    basem = trimesh.util.concatenate([M[n] for n in base + ["pi5_stack", "nucleo_f401re", "fe_urt_1"]])
    res["fov_self"] = {t: round(e.fov_blocked(selfm, t), 4) for t in (-30, 0, 30)}
    res["fov_base"] = {f"{p},{t}": round(e.fov_blocked(basem, t, p), 4) for p in (0, 90, 180) for t in (-30, 0)}
    res["heights_mm"] = {"desk_to_tray_top": TRAY_T, "desk_to_lower_deck_bottom": round(TRAY_T + LEG_H, 1),
                         "desk_to_top_deck_top": round(-z_tray_top() + TRAY_T, 1),
                         "desk_to_tilt_axis": round(g.H_TILT - z_tray_top() + TRAY_T, 1), "H_TILT_over_deck": g.H_TILT,
                         "DISC_Z0": g.DISC_Z0}
    res["screws"] = SCREW_TABLE
    res["chamfers"] = {k: v for k, v in g.CHAMFER_LOG.items()}
    res["volumes_cm3"] = {n: round(M[n].volume / 1000, 1) for n, v in P.items() if v[1]}
    return res


SCREW_TABLE = {   # every screw of this variant, all from the user's stock (M2 x 6, M2.5 x 8, M3 x 8/12/16, M3 nuts)
    "M3 x 12": "cage 4 (ear 7 + deck 5), lap joints 4 (+nut), arms 4 (+nut), Nucleo 3, tray legs 4  = 19",
    "M3 x 8": "driver frame -> top deck 2",
    "M3 nut": "lap joints 4, arms 4 = 8",
    "M2.5 x 8": "Pi 4, FE-URT-1 4 = 8",
    "M2 x 6": "camera stop 2 (mount E, optional)",
    "horn screws": "servo accessory screws, unchanged (Pan 4, Tilt 2)",
}

ORIENT = {"stm32_tray": [], "lower_deck": [], "top_deck": [((1, 0, 0), 180)], "pan_cage": [], "driver_frame": [],
          "turntable": [((1, 0, 0), 180)], "arm_servo": gp.ORIENT["arm_servo"], "arm_idle": gp.ORIENT["arm_idle"],
          "tilt_cradle": gp.ORIENT["tilt_cradle"]}
PRINTED = [  # part no, name, Korean, group, orientation, settings, order
    ("P01", "top_deck", "상판 (v0.9.2 + 드라이버 받침 나사 구멍 2, 배선 구멍, Pan→Tilt 선 플러그 통과 홈)", "BASE", "윗면이 바닥(뒤집어서)", "충전 25 %, 벽 3", 2),
    ("P02", "lower_deck", "하판 + 기둥 (v0.9.2 + STM32 배선 구멍 2, 다리 나사 안내 구멍 4)", "BASE", "하판 바닥이 바닥", "충전 25 %, 벽 4", 2),
    ("P03", "stm32_tray", "STM32 받침 (전체 받침대, 다리 4, Nucleo 받침 3 + 쉼 2)", "BASE", "판이 바닥, 다리 세워서", "충전 25 %, 벽 3", 2),
    ("P04", "driver_frame", "FE-URT-1 받침틀", "BASE", "그대로", "충전 40 %, 벽 3", 1),
    ("P05", "pan_cage", "Pan 서보 케이지 (귀 7 mm)", "BASE", "바닥이 바닥", "충전 40 %, 벽 3", 3),
    ("P08", "turntable", "턴테이블 (혼 고리 위 틈 5 mm)", "PAN", "윗면이 바닥(뒤집어서)", "충전 60 %, 벽 4, 서포트: 모든 곳(트리)", 3),
    ("P09", "arm_servo", "서보쪽 팔 (선 통과 창 + 받침판 뒤 홈)", "PAN", "발이 바닥(세워서), 브림", "충전 60 %, 벽 4", 4),
    ("P10", "arm_idle", "부싱쪽 팔", "PAN", "바깥면이 바닥(눕혀서)", "충전 60 %, 벽 4", 4),
    ("P11", "tilt_cradle", "Tilt 크래들 (카메라 E)", "TILT", "위쪽 끝이 바닥(뒤집어서)", "충전 60 %, 벽 4", 4),
]


def export(P, res):
    if os.path.exists(PKG):
        shutil.rmtree(PKG)
    d_cat, d_prt, d_doc = (os.path.join(PKG, s) for s in ("01_CATIA", "02_PRINT", "03_DOCS"))
    for d in (os.path.join(d_cat, "parts"), os.path.join(d_cat, "reference"), d_prt, d_doc):
        os.makedirs(d)
    groups = {k: cq.Assembly(name=k) for k in ("BASE", "PAN", "TILT")}
    report = []
    for n, (sh, printed, grp) in P.items():
        sub = "parts" if printed else "reference"
        f = os.path.join(d_cat, sub, f"{n}.step")
        cq.exporters.export(sh, f)
        back = cq.importers.importStep(f)
        ok = back.val().isValid() and abs(back.val().Volume() - sh.val().Volume()) < 1e-3 * sh.val().Volume() + 1e-3
        report.append({"file": f"01_CATIA/{sub}/{n}.step", "valid": bool(ok)})
        groups[grp.upper()].add(sh, name=n)
    groups["PAN"].add(groups["TILT"], name="TILT")
    root = cq.Assembly(name=f"gimbal_{VER}")
    root.add(groups["BASE"], name="BASE")
    root.add(groups["PAN"], name="PAN")
    root.export(os.path.join(d_cat, f"gimbal_{VER}_assembly.step"))
    tmp = tempfile.mkdtemp()
    with open(os.path.join(d_prt, "print_list.csv"), "w", newline="", encoding="utf-8-sig") as fh:
        w = csv.writer(fh)
        w.writerow(["순서", "부품번호", "파일", "부품", "출력 방향(파일에 이미 적용)", "설정", "크기 x×y×z [mm]"])
        for pn, src, ko, grp, ori, sett, order in sorted(PRINTED, key=lambda r: (r[6], r[0])):
            s = P[src][0]
            for ax, ang in ORIENT[src]:
                s = s.rotate((0, 0, 0), ax, ang)
            p = os.path.join(tmp, src + ".stl")
            cq.exporters.export(s, p, tolerance=0.02, angularTolerance=0.05)
            m = trimesh.load(p)
            lo, hi = m.bounds
            m.apply_translation([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]])
            name = f"{pn}_{src}.stl"
            m.export(os.path.join(d_prt, name))
            size = [round(float(v), 1) for v in m.bounds[1] - m.bounds[0]]
            fits = all(a <= b for a, b in zip(size, BED))
            w.writerow([order, pn, name, ko, ori, sett, " × ".join(map(str, size))])
            report.append({"file": f"02_PRINT/{name}", "watertight": bool(m.is_watertight), "fits_bed": fits, "size_mm": size})
    json.dump(res, open(os.path.join(d_doc, "checks_v092s.json"), "w"), indent=1, ensure_ascii=False)
    json.dump(report, open(os.path.join(d_doc, "package_check.json"), "w"), indent=1, ensure_ascii=False)
    with open(os.path.join(d_doc, "BOM.csv"), "w", newline="", encoding="utf-8-sig") as fh:
        w = csv.writer(fh)
        w.writerow(["품목", "규격", "수량", "쓰임"])
        for row in [
            ("서보", "Feetech STS3215", 2, "Pan ID 1, Tilt ID 2 (혼·동봉 혼 나사)"),
            ("서보 드라이버", "Feetech FE-URT-1", 1, "상판 위 받침틀"),
            ("제어 보드", "NUCLEO-F401RE", 1, "STM32 받침 위"),
            ("나사", "M3 × 12", 19, "케이지 4, 상판-기둥 4, 팔 4, Nucleo 3, 받침 다리 4"),
            ("너트", "M3", 8, "상판-기둥 4, 팔 4"),
            ("나사", "M3 × 8", 2, "드라이버 받침틀 → 상판"),
            ("나사", "M2.5 × 8", 8, "Pi 4, FE-URT-1 4"),
            ("나사", "M2 × 6", 2, "카메라 아래 멈춤 (선택)"),
            ("케이블 타이", "폭 4.8 이하", 1, "Tilt 서보 (받침판 아래 선도 같이 묶임)"),
            ("서보 케이블", "5264-3P (서보 동봉)", 2, f"드라이버→Pan 약 {res['cable_mm']['driver->Pan servo bus']} mm, Pan→Tilt 약 {res['cable_mm']['Pan->Tilt servo bus']} mm"),
            ("카메라 케이블", "300 mm (예산 250)", 1, f"경로 약 {res['cable_mm']['camera FPC']} mm"),
            ("점퍼선", "암-암 / 암-수", "6~8", "STM32 ↔ Pi UART, STM32 ↔ FE-URT-1 UART, GND"),
        ]:
            w.writerow(row)
    shutil.copy(os.path.join(HERE, "docs", "V092S_STANDALONE.md"), os.path.join(PKG, "README_먼저읽기.md"))
    img = os.environ.get("V092S_IMG")      # renders (made outside this script) copied in when given
    if img and os.path.isdir(img):
        os.makedirs(os.path.join(d_doc, "img"))
        for f in sorted(os.listdir(img)):
            if f.startswith("sa_") and f.endswith(".png"):
                shutil.copy(os.path.join(img, f), os.path.join(d_doc, "img", f))
    bad = [r for r in report if not all(v for k, v in r.items() if isinstance(v, bool))]
    zp = shutil.make_archive(PKG, "zip", HERE, os.path.basename(PKG))
    print(f"{len(report)} files checked, problems: {bad if bad else 'none'}")
    return zp


if __name__ == "__main__":
    P, K = build()
    for n, (sh, _, _) in P.items():
        assert sh.val().isValid(), n
    res = checks(P, K)
    for k, v in res.items():
        print(k, v)
    if os.path.exists(os.path.join(HERE, "docs", "V092S_STANDALONE.md")):
        print(export(P, res))
