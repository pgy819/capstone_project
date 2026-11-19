"""Stage 2 v0.1 -> viewer/stage2_viewer_s2v01.html (3D, hand-tilt sliders driving the 3-RRS inverse kinematics).
Needs stage2.py to have run (viewer_meshes/ + package checks)."""
import base64
import json
import os
import tempfile
from dataclasses import asdict

import trimesh

import s2_kin as kin

HERE = os.path.dirname(os.path.abspath(__file__))
VIEW = os.path.join(os.path.dirname(HERE), "viewer")
meta = json.load(open(os.path.join(HERE, "viewer_meshes", "meta.json")))
scene = trimesh.Scene()
for name in meta:
    m = trimesh.load(os.path.join(HERE, "viewer_meshes", name + ".stl"))
    if name.startswith("pb_") and len(m.faces) > 4000:
        try:
            m = m.simplify_quadric_decimation(face_count=4000)
        except Exception:
            pass
    scene.add_geometry(m, node_name=name, geom_name=name)
glb = os.path.join(tempfile.mkdtemp(), "s2.glb")
scene.export(glb)
res = json.load(open(os.path.join(HERE, "stage2_v01_package", "03_DOCS", "checks_s2v01.json")))
tpl = open(os.path.join(VIEW, "template_s2v01.html"), encoding="utf-8").read()
html = (tpl.replace("__GLB__", base64.b64encode(open(glb, "rb").read()).decode())
        .replace("__GEO__", json.dumps(asdict(kin.Geo())))
        .replace("__RES__", json.dumps(res, ensure_ascii=False, default=str)))
out = os.path.join(VIEW, "stage2_viewer_s2v01.html")
open(out, "w", encoding="utf-8").write(html)
print(out, os.path.getsize(out) // 1024, "KB")
