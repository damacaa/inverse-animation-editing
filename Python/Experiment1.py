"""Experiment 1: recover simulation parameters for the three thesis scenes.

For each scene the optimizer estimates the cloth parameters with four
parameterizations, and we report the final error and runtime.

Scenes:
  side     - one edge fixed, no wind
  corners  - four corners fixed, no wind
  flag     - one edge fixed, wind

Parameterizations (settings = mass-mode + stiffness-mode):
  Gn  homogeneous masses        Ln  heterogeneous masses
  nG  homogeneous stiffness     nL  heterogeneous stiffness
"""
import json
import os

import DebugHelper
from Optimizer import Minimize

SCENES = ["side", "corners", "flag"]
SETTINGS = ["Gn", "Ln", "nG", "nL"]
SETTING_NAMES = {
    "Gn": "mass (homogeneous)",
    "Ln": "mass (heterogeneous)",
    "nG": "stiffness (homogeneous)",
    "nL": "stiffness (heterogeneous)",
}
OUT_DIR = "../UnityProject/Assets/Files/Experiment1"


def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    for scene in SCENES:
        with open(f"../UnityProject/Assets/Files/{scene}.txt") as f:
            data = json.load(f)

        print(f"\n===== Scene: {scene} =====")

        rows = []
        for settings in SETTINGS:
            data["objects"][0]["optimizationSettings"] = settings

            result = Minimize(data=json.dumps(data), printResult=False)

            rows.append([
                settings,
                SETTING_NAMES[settings],
                f"{result.cuadraticError:.2f}",
                f"{result.elapsedTime:.1f}",
            ])

            with open(f"{OUT_DIR}/{scene}_{settings}_optimized.txt", "w") as f:
                f.write(result.newData)

        DebugHelper.AddTable(
            "experiment1",
            f"Scene: {scene}",
            ["Settings", "Parameter", "Error", "Time (s)"],
            rows)


if __name__ == "__main__":
    main()
