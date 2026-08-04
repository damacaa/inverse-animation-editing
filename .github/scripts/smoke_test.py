"""End-to-end smoke test for the Python bindings (used in CI).

Builds a minimal 2-node spring scene from JSON, steps it forward, and
verifies the free node falls under gravity.  No scipy needed.
"""
import json
import sys

sys.path.insert(0, "build/python")

import numpy as np

import DifferentiablePhysicsEngine as dpe

scene = {
    "integrationMethod": 2,  # implicit
    "tolerance": 1e-3,
    "printTimes": False,
    "windVel": {"x": 0.0, "y": 0.0, "z": 0.0},
    "objects": [
        {
            "vertPos": [{"x": 0, "y": 1, "z": 0}, {"x": 0, "y": 0, "z": 0}],
            "vertIsFixed": [True, False],
            "vertMass": [1.0, 1.0],
            "springs": [0, 1],
            "springStiffness": [200.0],
            "triangles": [],
            "dragCoefficient": 1.0,
            "damping": 0.5,
            "optimizationSettings": "nG",
        }
    ],
    "colliders": [],
}

info = json.dumps(scene)

init = dpe.initialize(info, "nG")
steps = [init] + dpe.forwardLoop(init.x, init.v, np.array([200.0]), "nG", 0.01, 99, 1)

y = steps[-1].x[4]
assert y < -0.01, f"free node should have fallen, got y={y}"
print(f"smoke test passed: free node at y={y:.6f}")
