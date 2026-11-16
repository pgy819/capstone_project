"""v1.0 candidate -> viewer/gimbal_viewer_v10.html (3D, pose sliders, comparison with v0.9.2)."""
import base64, json, os, tempfile
import cadquery as cq, trimesh
import gimbal_v10 as v
import gimbal_v03 as g
HERE = os.path.dirname(os.path.abspath(__file__))
P = v.build()
scene, tmp = trimesh.Scene(), tempfile.mkdtemp()
for name, (shape, _, _) in P.items():
    p = os.path.join(tmp, name + ".stl")
    fine = name in ("camera", "tilt_cradle")
    cq.exporters.export(shape, p, tolerance=0.03 if fine else 0.08, angularTolerance=0.08 if fine else 0.15)
    scene.add_geometry(trimesh.load(p), node_name=name, geom_name=name)
glb = os.path.join(tmp, "v.glb"); scene.export(glb)
checks = json.load(open(os.path.join(HERE, "gimbal_v100_candidate", "checks_v10.json")))
routes = v.routes()
tpl = open(os.path.join(HERE, "viewer", "template_v10.html")).read()
html = (tpl.replace("__GLB__", base64.b64encode(open(glb, "rb").read()).decode())
        .replace("__HTILT__", str(g.H_TILT))
        .replace("__CHECKS__", json.dumps(checks, ensure_ascii=False))
        .replace("__ROUTES__", json.dumps({k: [list(map(float, p)) for p in pts] for k, pts in routes.items()})))
out = os.path.join(HERE, "viewer", "gimbal_viewer_v10.html")
open(out, "w").write(html)
print(out, os.path.getsize(out) // 1024, "KB")
