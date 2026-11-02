"""v0.4 print geometry (split yoke, printed bushing, eye plate) -> viewer HTML."""
import base64, json, os, tempfile
import cadquery as cq, trimesh
import gimbal_v03 as g, gimbal_print as gp
HERE = os.path.dirname(os.path.abspath(__file__))
rows = json.load(open(os.path.join(HERE, "final_v03.json")))
best = rows[0]
gp.configure()
parts, keepouts, routes = g.build(export=False)
yoke, a, b, c = g.turntable_yoke()
tt, arm_s, arm_i = gp.split_yoke(yoke, a, b, c)
shapes = {n: v[0] for n, v in parts.items() if n != "turntable_yoke"}
shapes.update(turntable=tt, arm_servo=arm_s, arm_idle=arm_i)
scene, tmp = trimesh.Scene(), tempfile.mkdtemp()
for name, shape in list(shapes.items()) + list(keepouts.items()):
    p = os.path.join(tmp, name.replace(" ", "_").replace("/", "_") + ".stl")
    fine = name in ("eye_plate", "camera", "tilt_cradle")
    cq.exporters.export(shape, p, tolerance=0.03 if fine else 0.08, angularTolerance=0.08 if fine else 0.15)
    scene.add_geometry(trimesh.load(p), node_name=name, geom_name=name)
glb = os.path.join(tmp, "v.glb"); scene.export(glb)
tpl = open(os.path.join(HERE, "viewer", "template_v04.html")).read()
html = (tpl.replace("__GLB__", base64.b64encode(open(glb, "rb").read()).decode())
        .replace("__HTILT__", str(g.H_TILT))
        .replace("__EYEX__", str(g.EYE_X0 + g.EYE_T))
        .replace("__ROUTES__", json.dumps({k: [list(map(float, p)) for p in v] for k, v in routes.items()}))
        .replace("__EXPLORE__", json.dumps(rows)).replace("__BEST__", json.dumps(best)))
open(os.path.join(HERE, "viewer", "rcws_gimbal_viewer_v04.html"), "w").write(html)
print(sorted(shapes), os.path.getsize(glb) // 1024, "KB")
