"""v0.9.2-S standalone -> viewer/gimbal_viewer_v092s.html (3D, pose sliders, checks, cable routes)."""
import base64, json, os, tempfile
import cadquery as cq, trimesh
import gimbal_v092s as v
import gimbal_v03 as g
HERE = os.path.dirname(os.path.abspath(__file__))
P, _ = v.build()
scene, tmp = trimesh.Scene(), tempfile.mkdtemp()
for name, (shape, _, _) in P.items():
    p = os.path.join(tmp, name + ".stl")
    fine = name in ("camera", "tilt_cradle")
    cq.exporters.export(shape, p, tolerance=0.03 if fine else 0.08, angularTolerance=0.08 if fine else 0.15)
    scene.add_geometry(trimesh.load(p), node_name=name, geom_name=name)
glb = os.path.join(tmp, "v.glb"); scene.export(glb)
checks = json.load(open(os.path.join(v.PKG, "03_DOCS", "checks_v092s.json")))
routes = v.routes()
tpl = open(os.path.join(HERE, "viewer", f"template_{v.VER}.html")).read()
html = (tpl.replace("__GLB__", base64.b64encode(open(glb, "rb").read()).decode())
        .replace("__HTILT__", str(g.H_TILT))
        .replace("__CHECKS__", json.dumps(checks, ensure_ascii=False))
        .replace("__ROUTES__", json.dumps({k: [list(map(float, p)) for p in pts] for k, pts in routes.items()})))
out = os.path.join(HERE, "viewer", f"gimbal_viewer_{v.VER}.html")
open(out, "w").write(html)
print(out, os.path.getsize(out) // 1024, "KB")
