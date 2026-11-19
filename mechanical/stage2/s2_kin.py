"""Stage 2 (lower 3 axes) - 3-RRS parallel levelling platform: geometry, inverse kinematics, statics.

Frames
  base   : origin at the centre of the base plate top face, z up. Leg i sits at azimuth BETA[i].
           Leg frame: r = radial unit, t = tangent (servo shaft direction, out of the horn face), z.
  platform: origin at the centre of the three ball centres, z = platform normal.

Leg i (R-R-S)
  servo shaft axis  : through S_i = RB r + HS z, along t (crank turns in the r-z plane)
  crank             : length A, angle alpha from horizontal, alpha > 0 = tip up; tip pin at t = T_OFF (rod plane)
  rod               : length L from the crank-tip pin (axis t) to the ball centre
  ball (S joint)    : on the platform at RP r + T_OFF t (platform frame)
The R-R pair keeps the ball centre in the plane t = T_OFF of its leg -> the platform has 3 DOF
(roll, pitch, heave); x, y and yaw follow from the three plane constraints (parasitic motion).

Run: python s2_kin.py  -> prints the design sweep and writes s2_kin.json
"""
import json
import math
import os
from dataclasses import asdict, dataclass

import numpy as np
from scipy.optimize import fsolve

HERE = os.path.dirname(os.path.abspath(__file__))


@dataclass
class Geo:
    RB: float = 35.0        # servo shaft radius [mm]
    HS: float = 34.0        # shaft height above the base plate top (servo stands on its far end: 45.2 - 11.2)
    A: float = 35.0         # crank: shaft -> tip pin
    L: float = 60.0         # rod: tip pin -> ball centre
    RP: float = 70.0        # ball centre radius on the platform (radial part)
    T_OFF: float = 12.7     # ball-centre plane in front of the shaft face (horn 0.5..4.5, crank 2.5..9.5, rod 9.7..15.7)
    BETA0: float = 90.0     # azimuth of leg 0 [deg]

    def betas(self):
        return [math.radians(self.BETA0 + 120.0 * i) for i in range(3)]

    def z0(self):
        """Ball-centre height at home (crank horizontal, rod vertical)."""
        return self.HS + self.L


def rot(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def leg_axes(b):
    r = np.array([math.cos(b), math.sin(b), 0.0])
    t = np.array([-math.sin(b), math.cos(b), 0.0])
    return r, t, np.array([0.0, 0.0, 1.0])


def ball_local(G, b):
    r, t, _ = leg_axes(b)
    return G.RP * r + G.T_OFF * t


def platform_points(G, x, y, z, roll, pitch, yaw):
    R = rot(roll, pitch, yaw)
    p0 = np.array([x, y, z])
    return [p0 + R @ ball_local(G, b) for b in G.betas()], R


def solve_parasitic(G, roll, pitch, z):
    """x, y, yaw so that every ball centre stays in its leg plane."""
    def f(v):
        P, _ = platform_points(G, v[0], v[1], z, roll, pitch, v[2])
        return [float(P[i] @ leg_axes(b)[1] - G.T_OFF) for i, b in enumerate(G.betas())]
    v = fsolve(f, [0.0, 0.0, 0.0], xtol=1e-12)
    return v, max(abs(e) for e in f(v)) < 1e-6


def crank_angle(G, P, b):
    """Crank angle reaching ball centre P (base frame); elbow-down branch (tip below the rod). None if out of reach."""
    r, t, _ = leg_axes(b)
    u = float(P @ r) - G.RB          # radial distance from the shaft
    w = float(P[2]) - G.HS           # height above the shaft
    # (u - A cos a)^2 + (w - A sin a)^2 = L^2  ->  u cos a + w sin a = (u^2 + w^2 + A^2 - L^2) / (2A)
    k = (u * u + w * w + G.A * G.A - G.L * G.L) / (2 * G.A)
    rr = math.hypot(u, w)
    if abs(k) > rr:
        return None
    ph = math.atan2(w, u)
    d = math.acos(k / rr)
    cands = [ph - d, ph + d]
    # the branch continuous with alpha = 0 at home: tip below the line shaft -> ball
    a = min(cands, key=lambda c: abs(math.atan2(math.sin(c), math.cos(c))))
    return math.atan2(math.sin(a), math.cos(a))


def pose(G, roll, pitch, dz=0.0):
    """Full inverse kinematics for a platform attitude (rad) and heave offset from home."""
    z = G.z0() + dz
    v, ok = solve_parasitic(G, roll, pitch, z)
    if not ok:
        return None
    P, R = platform_points(G, v[0], v[1], z, roll, pitch, v[2])
    out = {"x": v[0], "y": v[1], "z": z, "yaw": v[2], "R": R, "P": P, "alpha": [], "C": [], "S": []}
    for i, b in enumerate(G.betas()):
        a = crank_angle(G, P[i], b)
        if a is None:
            return None
        r, t, zz = leg_axes(b)
        S = G.RB * r + G.HS * zz
        C = S + G.T_OFF * t + G.A * (math.cos(a) * r + math.sin(a) * zz)
        out["alpha"].append(a)
        out["C"].append(C)
        out["S"].append(S)
    return out


def leg_metrics(G, q):
    """Per leg: rod tilt from the platform normal (socket cone), crank-rod transmission angle."""
    nz = q["R"][:, 2]
    phi, trans = [], []
    for i, b in enumerate(G.betas()):
        rod = q["P"][i] - q["C"][i]
        u = rod / np.linalg.norm(rod)
        phi.append(math.degrees(math.acos(float(np.clip(u @ nz, -1, 1)))))
        r, t, zz = leg_axes(b)
        crank = q["C"][i] - q["S"][i] - G.T_OFF * t
        c = crank / np.linalg.norm(crank)
        trans.append(math.degrees(math.acos(float(np.clip(c @ u, -1, 1)))))
    return phi, trans


def statics(G, q, mass_kg, cg_local, g_base):
    """Servo torques [N m] and rod forces holding a rigid payload (platform + top) of mass_kg whose CG is cg_local
    (platform frame, mm) against gravity g_base (base frame, m/s^2). Leg i carries s_i along the rod and c_i along
    its t axis at the ball (the only forces an R-R-S leg can take). Returns tau, s, c."""
    Fg = mass_kg * np.asarray(g_base, float)
    p0 = np.array([q["x"], q["y"], q["z"]])
    cg = p0 + q["R"] @ np.asarray(cg_local, float)
    Arows = []
    cols = []
    for i, b in enumerate(G.betas()):
        rod = q["P"][i] - q["C"][i]
        u = rod / np.linalg.norm(rod)
        t = leg_axes(b)[1]
        for d in (u, t):
            m = np.cross((q["P"][i] - cg) / 1000.0, d)
            cols.append(np.concatenate([d, m]))
    Amat = np.array(cols).T                       # 6 x 6: forces of the legs on the platform
    rhs = -np.concatenate([Fg, np.zeros(3)])      # sum = -gravity, moments about the CG = 0
    f = np.linalg.solve(Amat, rhs)
    s, c = f[0::2], f[1::2]
    tau = []
    for i, b in enumerate(G.betas()):
        rod = q["P"][i] - q["C"][i]
        u = rod / np.linalg.norm(rod)
        r, t, zz = leg_axes(b)
        lever = (q["C"][i] - q["S"][i]) / 1000.0
        F_on_crank = -s[i] * u                    # the rod pushes the platform with +s u -> the crank with -s u
        tau.append(float(np.cross(lever, F_on_crank) @ t))
    return np.array(tau), s, c


def sweep(G, tilt_max_deg=20.0, n_az=24, n_r=5, dz_list=(-6.0, 0.0, 6.0), mass=0.80, cg_up=70.0):
    """Every attitude with total tilt <= tilt_max at each heave: reach, crank range, socket cone, transmission,
    parasitic motion, static torque with the base tilted by the same angle (platform kept level in the world)."""
    res = {"reach": True, "alpha_deg": [999, -999], "phi_max": 0.0, "trans": [999, -999], "xy_max": 0.0,
           "yaw_max": 0.0, "tau_max": 0.0, "rod_side_N": 0.0, "fail": []}
    for dz in dz_list:
        for k in range(n_r + 1):
            tilt = math.radians(tilt_max_deg) * k / n_r
            for j in range(n_az if k else 1):
                az = 2 * math.pi * j / n_az
                roll, pitch = tilt * math.cos(az), tilt * math.sin(az)
                q = pose(G, roll, pitch, dz)
                if q is None:
                    res["reach"] = False
                    res["fail"].append((dz, math.degrees(roll), math.degrees(pitch)))
                    continue
                al = [math.degrees(a) for a in q["alpha"]]
                res["alpha_deg"] = [min(res["alpha_deg"][0], min(al)), max(res["alpha_deg"][1], max(al))]
                phi, tr = leg_metrics(G, q)
                res["phi_max"] = max(res["phi_max"], max(phi))
                res["trans"] = [min(res["trans"][0], min(tr)), max(res["trans"][1], max(tr))]
                res["xy_max"] = max(res["xy_max"], math.hypot(q["x"], q["y"]))
                res["yaw_max"] = max(res["yaw_max"], abs(math.degrees(q["yaw"])))
                # base tilted so the platform is level in the world: gravity seen in the base frame = R^T g
                gb = q["R"] @ np.array([0, 0, -9.81])
                tau, s, c = statics(G, q, mass, [0, 0, cg_up], gb)
                res["tau_max"] = max(res["tau_max"], float(np.max(np.abs(tau))))
                res["rod_side_N"] = max(res["rod_side_N"], float(np.max(np.abs(c))))
    return res


def tilt_rate_gain(G, eps=1e-4):
    """Crank angle per platform tilt at home (worst axis): d alpha / d theta."""
    q0 = pose(G, 0, 0)
    worst = 0.0
    for ax in ((eps, 0), (0, eps)):
        q = pose(G, *ax)
        worst = max(worst, max(abs(a - b) for a, b in zip(q["alpha"], q0["alpha"])) / eps)
    return worst


if __name__ == "__main__":
    G = Geo()
    out = {"geo": asdict(G), "dalpha_dtilt": tilt_rate_gain(G)}
    for tilt in (15.0, 20.0):
        r = sweep(G, tilt_max_deg=tilt, dz_list=(-3.0, 0.0, 3.0))
        out[f"tilt{int(tilt)}"] = {k: v for k, v in r.items() if k != "fail"}
        print(tilt, {k: (round(v, 3) if isinstance(v, float) else v) for k, v in out[f"tilt{int(tilt)}"].items()})
    print("d alpha / d tilt", round(out["dalpha_dtilt"], 3))
    json.dump(out, open(os.path.join(HERE, "s2_kin.json"), "w"), indent=1)
