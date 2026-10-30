"""out_v02/viewer.glb (one named mesh per part + keep-out volumes) and routes.json."""
import json, os, tempfile
import cadquery as cq, numpy as np, trimesh
import gimbal_v02 as g

parts, keepouts, routes = g.build(export=False)
scene = trimesh.Scene(); tmp = tempfile.mkdtemp()
def add(name, shape):
    p = os.path.join(tmp, name.replace(" ", "_").replace("/", "_") + ".stl")
    cq.exporters.export(shape, p, tolerance=0.08, angularTolerance=0.15)
    scene.add_geometry(trimesh.load(p), node_name=name, geom_name=name)
for n, (s, *_rest) in parts.items(): add(n, s)
for n, s in keepouts.items(): add(n, s)
scene.export(os.path.join(g.OUT, "viewer.glb"))
json.dump({k: [list(map(float, p)) for p in v] for k, v in routes.items()}, open(os.path.join(g.OUT, "routes.json"), "w"))
print(os.path.getsize(os.path.join(g.OUT, "viewer.glb")) // 1024, "KB")
