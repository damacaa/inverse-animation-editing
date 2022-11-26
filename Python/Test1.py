from unittest import result
from Optimizer import Minimize
import numpy as np
import DebugHelper
import json


# All availabe methods for the optimizer
methods = ["CG", "BFGS", "Newton-CG",
           "L-BFGS-B", "TNC", "SLSQP", "trust-constr"]

files = ["side", "corners", "flag"]
settingsOptions = ["Gn", "Ln", "nG", "nL"]

# Read scene file
for f in files:
    DebugHelper.AddToLog("Scene: " + f, "test1")

    sceneData = open(
        f"D:/Projects/MassSpringSimulator/UnityProject/Assets/{f}.txt", "r"
    ).read()

    # Get optimized scene
    for s in settingsOptions:

        data_dict = json.loads(sceneData)
        data_dict["objects"][0]["optimizationSettings"] = s
        sceneData = json.dumps(data_dict)

        result = Minimize(
            data=sceneData, printResult=False)

        DebugHelper.AddToLog(
            f"{s} {result.cuadraticError} {result.elapsedTime}".replace('.', ','), "test1")

        text_file = open(
            f"D:/Projects/MassSpringSimulator/UnityProject/Assets/Files/Test1/{f}_{s}_optimized.txt", "w")
        text_file.write(result.newData)
        text_file.close()


"""small = np.arange(100, 101, 10)
big = np.arange(100, 501, 100)
values = np.concatenate((small, big))
for i in big:
    optimizedSceneData, error, error100, t = Minimize(
        data=sceneData, nSteps=i, h=0.08, printResult=False)
    print(f"{i} {error} {error100} {t}")
    text_file = open(
        "D:/Projects/MassSpringSimulator/UnityProject/Assets/scene_optimizedd"+str(i)+".txt", "w")
    text_file.write(optimizedSceneData)
    text_file.close()"""


# scipy.optimize.show_options(solver="minimize", method="L-BFGS-B", disp=True)

"""
methods2 = ["Nelder-Mead", "Powell", "COBYLA"]
for m in methods:
    print(m)
    Minimize(m)
    print("-" * 60) """


# print("-" * 60)

# res2 = fmin_bfgs(G, p0, fprime=dGdp)
# print(res2)
# print("ERROR: ", abs(desiredMass - res2))
