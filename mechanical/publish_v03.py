"""Build the chosen v0.3 variant: STEP/STL, viewer.glb, routes, and the viewer HTML with the exploration table."""
import base64
import json
import math
import os
import sys
import tempfile

import cadquery as cq
import trimesh

import gimbal_v03 as g

HERE = os.path.dirname(os.path.abspath(__file__))
rows = json.load(open(os.path.join(HERE, "final_v03.json")))
best = rows[0] if len(sys.argv) < 2 else json.loads(sys.argv[1])
g.configure(best["wrap_r"], best["stop"], best["orient"])
g.H_TILT += best.get("h_extra", 0.0)
cx, cy, _ = g.tilt_conn_point()
g.HOLE_BUS = (math.degrees(math.atan2(cy, cx)), g.R_MID)
parts, keepouts, routes = g.build(export=True)

scene, tmp = trimesh.Scene(), tempfile.mkdtemp()
for name, shape in [(n, v[0]) for n, v in parts.items()] + list(keepouts.items()):
    p = os.path.join(tmp, name.replace(" ", "_").replace("/", "_") + ".stl")
    cq.exporters.export(shape, p, tolerance=0.08, angularTolerance=0.15)
    scene.add_geometry(trimesh.load(p), node_name=name, geom_name=name)
glb = os.path.join(g.OUT, "viewer.glb")
scene.export(glb)

tpl = open(os.path.join(HERE, "viewer", "template_v03.html")).read()
html = (tpl.replace("__GLB__", base64.b64encode(open(glb, "rb").read()).decode())
        .replace("__HTILT__", str(g.H_TILT))
        .replace("__ROUTES__", json.dumps({k: [list(map(float, p)) for p in v] for k, v in routes.items()}))
        .replace("__EXPLORE__", json.dumps(rows))
        .replace("__BEST__", json.dumps(best)))
open(os.path.join(HERE, "viewer", "rcws_gimbal_viewer.html"), "w").write(html)
print("H_TILT", g.H_TILT, "DISC_R", g.DISC_R, "glb KB", os.path.getsize(glb) // 1024)
