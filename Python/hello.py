from ast import arg
import numpy as np
import scipy.optimize
import time
from numpy import random
import json

import UnityDLL


def Average(lst):
    return sum(lst) / len(lst)


def g(p, iter, h, m_numDoFs, initialState, targets):
    global lastP
    lastP = p

    steps = []
    current = initialState
    steps.append(initialState)

    for i in range(iter - 1):
        current = UnityDLL.forward(current.x, current.v, p, h)
        steps.append(current)

    g = np.linalg.norm(targets[iter - 1].x - steps[iter - 1].x) ** 2

    return g / iter


def G(p, iter, h, m_numDoFs, initialState, targets, settings):

    current = initialState

    steps = [initialState] + UnityDLL.forwardLoop(
        initialState.x, initialState.v, p, settings, h, iter - 1
    )

    """steps = [initialState]

    for i in range(iter - 1):
        current = UnityDLL.forward(current.x, current.v, p, h)
        steps.append(current)"""

    G = 0
    for i in range(iter):
        G += np.linalg.norm(targets[i].x - steps[i].x) ** 2
    # G = G / iter

    return (100 / (iter / p.size)) * G


def dGdp(p, iter, h, m_numDoFs, initialState, targets, settings):

    steps = []
    current = initialState
    steps.append(initialState)

    """for i in range(iter - 1):
        current = UnityDLL.forward(current.x, current.v, p, h)
        steps.append(current)"""

    steps = [initialState] + UnityDLL.forwardLoop(
        initialState.x, initialState.v, p, settings, h, iter - 1
    )

    _dGdp = np.full(p.size, 0.0)
    _dGdx = []
    _dGdv = []

    for i in range(iter):
        """if i == iter - 1:
            _dGdx.append(2.0 * (targets[iter - 1].x - steps[iter - 1].x))
        else:
            _dGdx.append(np.full(m_numDoFs, 0.0))

        _dGdv.append(np.full(m_numDoFs, 0.0))"""

        _dGdx.append((100 / (iter / p.size)) * 2.0 *
                     (targets[i].x - steps[i].x))
        _dGdv.append(np.full(m_numDoFs, 0.0))

    i = iter - 2
    while i >= 0:
        backward = UnityDLL.backward(
            steps[i].x,
            steps[i].v,
            steps[i + 1].x,
            steps[i + 1].v,
            p,
            settings,
            _dGdx[i + 1],
            _dGdv[i + 1],
            h,
        )

        _dGdp += backward.dGdp
        _dGdx[i] += backward.dGdx
        _dGdv[i] += backward.dGdv

        i -= 1

    # print(x, _dGdp.tolist())

    return _dGdp


def Minimize(method="L-BFGS-B", costFunction=G, jacobian=dGdp, callback=None):

    # scipy.optimize.show_options(solver="minimize", method=method, disp=True)

    data = open(
        "D:/Projects/MassSpringSimulator/UnityProject/Assets/scene.txt", "r"
    ).read()

    data_dict = json.loads(data)

    # PARAMETERS
    iter = 50
    h = 0.03

    masses = []
    stiffnesses = []

    settings = ""

    for o in data_dict["objects"]:

        _settings = o["optimizationSettings"]
        massMode = _settings[0]
        stiffnessMode = _settings[1]

        if massMode == "L":
            masses += o["vertMass"]
        elif massMode == "G":
            masses += [Average(o["vertMass"])]

        if stiffnessMode == "L":
            stiffnesses += o["springStiffness"]
        elif stiffnessMode == "G":
            stiffnesses += [Average(o["springStiffness"])]

        settings += _settings

    desiredMass = np.array(masses)
    desiredStiffness = np.array(stiffnesses)

    desiredParameter = np.concatenate((desiredMass, desiredStiffness), axis=0)

    # desiredParameter = random.rand(6) + 0.5

    print(
        f"{'-'*60}\nSettings: {settings} Iterations: {iter} Timestep: {h} Target parameter: {desiredParameter}\n{'-'*60} "
    )

    # INITIALIZATION
    initialState = UnityDLL.initialize(data, settings)
    m_numDoFs = initialState.x.size

    # CALCULATING TARGET
    targets = [initialState] + UnityDLL.forwardLoop(
        initialState.x,
        initialState.v,
        desiredParameter,
        "nn" * len(data_dict["objects"]),
        h,
        iter - 1,
    )

    p0 = np.full(desiredParameter.size, 1)  # initial parameter value
    args = (iter, h, m_numDoFs, initialState, targets, settings)  # extra info
    bnds = [(0, 10000)] * p0.size  # parameter bounds
    options = {"maxiter": 15000, "maxfun": 100000, "ftol": 0.001}

    # G(desiredParameter, iter, h, m_numDoFs, initialState, targets)
    # dGdp(desiredParameter, iter, h, m_numDoFs, initialState, targets)

    start = time.time()
    res = scipy.optimize.minimize(
        costFunction,
        p0,
        jac=jacobian,  # jacobian "2-point" "3-point" "cs"
        method=method,
        args=args,
        bounds=bnds,
        options=options,
        callback=callback,
    )
    end = time.time()

    print(res)

    # print("RESULT:\n", np.round(res.x, 3))
    # print("ERROR\n: ", abs(desiredParameter - res.x))
    print("Time elapsed:\n", str(round((end - start) * 1000.0, 1)), "ms")

    # WRITING NEW FILE
    offset = 0
    for o in data_dict["objects"]:

        settings = o["optimizationSettings"]
        massMode = settings[0]
        stiffnessMode = settings[1]

        nVerts = len(o["vertMass"])
        nSprings = len(o["springStiffness"])

        if massMode in {"L", "l"}:
            o["vertMass"] = res.x[offset: nVerts + offset].tolist()
            offset += nVerts
        elif massMode in {"G", "g"}:
            o["vertMass"] = [res.x[offset]] * nVerts
            offset += 1

        if stiffnessMode in {"L", "l"}:
            o["springStiffness"] = res.x[offset: nSprings + offset].tolist()
            offset += nSprings
        elif stiffnessMode in {"G", "g"}:
            o["springStiffness"] = [res.x[offset]] * nSprings
            offset += 1

    newData = json.dumps(data_dict)
    text_file = open(
        "D:/Projects/MassSpringSimulator/UnityProject/Assets/scene_optimized.txt", "w"
    )
    n = text_file.write(newData)
    text_file.close()

    return res.x


a = 0


def ShowProgress(p):
    global a
    progress = ["-", "/", "|", "\\"]
    print(progress[a], end="\r")
    a += 1
    if a >= len(progress):
        a = 0


# Minimize(callback=ShowProgress)
print(UnityDLL.test())


""" methods = ["CG", "BFGS", "Newton-CG", "L-BFGS-B", "TNC", "SLSQP", "trust-constr"]
methods2 = ["Nelder-Mead", "Powell", "COBYLA"]
for m in methods:
    print(m)
    Minimize(m)
    print("-" * 60) """


# print("-" * 60)

# res2 = fmin_bfgs(G, p0, fprime=dGdp)
# print(res2)
# print("ERROR: ", abs(desiredMass - res2))

# https://gist.github.com/yuyay/3067185
# python -m pip install "d:/Projects/MassSpringSimulator/UnityDLL"


"""  F  nF
nn
Gn  
Ln
nG
nL
GG  +   -
LL
"""
