"""Experiment 2: effect of the number of simulation steps on the estimate.

The error reported at the end of the optimization is measured on the n steps
used during optimization, and again on a fixed, much longer simulation
(500 frames) to catch parameters that only fit the first instants.

Scenes:
  side  - one edge fixed, no wind        (thesis Table 5.3)
  flag  - one edge fixed, wind           (thesis Tables 5.1 and 5.2)

Both scenes estimate heterogeneous stiffnesses (settings "nL").

NOTE: this sweep is slow.  A full run takes roughly 30+ minutes for "flag"
and several hours for "side" (the thesis run took ~2900 s for 200 steps).
Trim the STEPS list to a few values for a quick check.
"""
import json
import os

import DebugHelper
from Optimizer import Minimize

SCENES = ["side", "flag"]          # "flag_noWind" is available as a no-wind variant
STEPS = list(range(20, 201, 20))   # 20, 40, ..., 200 as in the thesis
DELTA = 0.02
EVAL_FRAMES = 500
OUT_DIR = "../UnityProject/Assets/Files/Experiment2"


def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    for scene in SCENES:
        with open(f"../UnityProject/Assets/Files/{scene}.txt") as f:
            data = json.load(f)

        print(f"\n===== Scene: {scene} (settings: {data['objects'][0]['optimizationSettings']}) =====")

        rows = []
        for n_steps in STEPS:
            data["optimizationIterations"] = n_steps
            data["delta"] = DELTA

            result = Minimize(
                data=json.dumps(data),
                cuadraticErrorFrames=EVAL_FRAMES,
                printResult=False)

            rows.append([
                str(n_steps),
                f"{result.cuadraticError:.2f}",
                f"{result.cuadraticErrorConstant:.2f}",
                f"{result.elapsedTime:.1f}",
            ])

            with open(f"{OUT_DIR}/{scene}_{n_steps}_optimized.txt", "w") as f:
                f.write(result.newData)

        DebugHelper.AddTable(
            "experiment2",
            f"Scene: {scene}",
            ["Steps", "Error (n steps)", f"Error ({EVAL_FRAMES} steps)", "Time (s)"],
            rows)


if __name__ == "__main__":
    main()
