from unittest import result
from Optimizer import Minimize
import numpy as np
import DebugHelper
import json

files = ["side", ]  # "flag_noWind", "flag"
for f in files:
    sceneData = open(
        f"../UnityProject/Assets/Files/{f}.txt", "r"
    ).read()

    """small = np.arange(10, 101, 10)
    big = np.arange(100, 501, 100)
    values = np.concatenate((small, big))"""

    values = np.arange(50, 51, 10)

    for v in values:

        data_dict = json.loads(sceneData)
        data_dict["optimizationIterations"] = int(v)
        data_dict["delta"] = 0.02
        sceneData = json.dumps(data_dict)

        result = Minimize(
            data=sceneData, cuadraticErrorFrames=500, printResult=False)

        DebugHelper.AddToLog(
            f"{v} {result.cuadraticError} {result.cuadraticErrorConstant} {result.parameterDeviation} {result.elapsedTime}".replace('.', ','), f"test2_{f}_2")

        text_file = open(
            f"../UnityProject/Assets/Files/Test2/{f}_{v}_optimized.txt", "w")
        text_file.write(result.newData)
        text_file.close()
