"""Build out/viewer.glb: one named mesh per part, in the gimbal frame (Z up, mm)."""
import os
import tempfile

import cadquery as cq
import numpy as np
import trimesh

import gimbal as g

parts = g.build()
scene = trimesh.Scene()
tmp = tempfile.mkdtemp()
for name, (shape, rgb, printable) in parts.items():
    p = os.path.join(tmp, name + ".stl")
    cq.exporters.export(shape, p, tolerance=0.08, angularTolerance=0.15)
    m = trimesh.load(p)
    m.visual = trimesh.visual.ColorVisuals(m, face_colors=np.array([*(int(c * 255) for c in rgb), 255], dtype=np.uint8))
    scene.add_geometry(m, node_name=name, geom_name=name)
scene.export(os.path.join(g.OUT, "viewer.glb"))
print("viewer.glb", os.path.getsize(os.path.join(g.OUT, "viewer.glb")) // 1024, "KB")
