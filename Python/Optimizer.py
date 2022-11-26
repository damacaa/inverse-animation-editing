from ast import arg
from asyncio.windows_events import NULL
from cmath import sqrt

from math import fabs
import math
import re
from sre_constants import SUCCESS
from unittest import result
import numpy as np
import scipy.optimize
import time
from numpy import random
import json

import UnityDLL


class OptimizationResult:
    newData = NULL
    elapsedTime = 0
    cost = 0
    cuadraticError = 0
    cuadraticErrorConstant = 0
    parameterDeviation = 0


def Average(lst):
    return sum(lst) / len(lst)


def QuadraticError(estimated, actual, nVerts):
    nSteps = len(estimated)
    G = 0
    for i in range(nSteps):
        G += np.sum(abs(actual[i].x - estimated[i].x) ** 2)

    if math.isnan(G):
        print("It's too much")
        return float("inf")

    return math.sqrt(G/(nVerts*nSteps))


def g(p, nSteps, forwardSubSteps, h, m_numDoFs, initialState, targets):
    print("Deprecated!")
    global lastP
    lastP = p

    steps = []
    current = initialState
    steps.append(initialState)

    for i in range(nSteps - 1):
        current = UnityDLL.forward(current.x, current.v, p, h)
        steps.append(current)

    g = np.sum(targets[nSteps - 1].x - steps[nSteps - 1].x) ** 2

    return g


def G(p, nSteps, forwardSubSteps, h, m_numDoFs, initialState, targets, settings):

    steps = [initialState] + UnityDLL.forwardLoop(
        initialState.x, initialState.v, p, settings, h, nSteps - 1, forwardSubSteps
    )

    G = 0
    history = []
    for i in range(nSteps):  # cambiar a steps
        G += np.sum(abs(targets[i].x - steps[i].x) ** 2)
        history.append(G)
    # G = G / nSteps

    if math.isnan(G):
        print(history)
        G = float("inf")

    return G


def dGdp(p, nSteps, forwardSubSteps, h, m_numDoFs, initialState, targets, settings):

    steps = [initialState] + UnityDLL.forwardLoop(
        initialState.x, initialState.v, p, settings, h, nSteps - 1, forwardSubSteps
    )

    _dGdp = np.full(p.size, 0.0)
    _dGdx = []
    _dGdv = []

    for i in range(nSteps):
        """if i == nSteps - 1:
            _dGdx.append(2.0 * (targets[nSteps - 1].x - steps[nSteps - 1].x))
        else:
            _dGdx.append(np.full(m_numDoFs, 0.0))

        _dGdv.append(np.full(m_numDoFs, 0.0))"""

        _dGdx.append(2.0 * (targets[i].x - steps[i].x))

        # _dGdx.append(2.0 * (targets[i].x - steps[i].x))
        _dGdv.append(np.full(m_numDoFs, 0.0))

    i = nSteps - 2
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

    return _dGdp


def Minimize(data, method="L-BFGS-B", costFunction=G, jacobian=dGdp, cuadraticErrorFrames=0, printResult=True):

    # scipy.optimize.show_options(solver="minimize", method=method, disp=True)

    # read data as json
    data_dict = json.loads(data)

    # PARAMETERS

    nSteps = data_dict["optimizationIterations"]
    forwardSubSteps = data_dict["forwardSubSteps"]
    h = data_dict["delta"]

    p = []

    settings = ""

    for o in data_dict["objects"]:

        _settings = o["optimizationSettings"]
        massMode = _settings[0]
        stiffnessMode = _settings[1]

        if massMode == "L":
            p += o["vertMass"]
        elif massMode == "G":
            p += [Average(o["vertMass"])]

        if stiffnessMode == "L":
            p += o["springStiffness"]
        elif stiffnessMode == "G":
            p += [Average(o["springStiffness"])]

        settings += _settings

    desiredParameter = np.array(p)

    # INITIALIZATION
    initialState = UnityDLL.initialize(data, settings)

    m_numDoFs = initialState.x.size

    # CALCULATING TARGET
    targets = [initialState] + UnityDLL.forwardLoop(
        initialState.x,
        initialState.v,
        np.full(0, 0,),
        "nn" * len(data_dict["objects"]),
        h,
        nSteps - 1,
        forwardSubSteps
    )

    p0 = np.full(desiredParameter.size, 0.0001)  # initial parameter value
    args = (nSteps, forwardSubSteps, h, m_numDoFs, initialState,
            targets, settings)  # extra info
    bnds = [(0.0001, 10000)] * p0.size  # parameter bounds

    # G(desiredParameter, nSteps, h, m_numDoFs, initialState, targets)
    # dGdp(desiredParameter, nSteps, h, m_numDoFs, initialState, targets)

    start = time.time()
    res = scipy.optimize.minimize(
        costFunction,
        p0,
        jac=jacobian,  # jacobian "2-point" "3-point" "cs"
        method=method,
        args=args,
        bounds=bnds,
        callback=ShowProgress,
    )
    end = time.time()

    # WRITING RESULTS TO NEW FILE
    offset = 0
    for o in data_dict["objects"]:

        settings_ = o["optimizationSettings"]
        massMode = settings_[0]
        stiffnessMode = settings_[1]

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

    # Measure time
    elapsedTime = round((end - start), 1)

    # Cost at end
    cost = round(res.fun, 2)

    # Parameter deviation
    parameterDeviation = round(np.sum(
        abs(abs(desiredParameter - res.x)/desiredParameter)*100)/len(res.x), 2)

    # Error with n steps
    steps = [initialState] + UnityDLL.forwardLoop(
        initialState.x, initialState.v, res.x, settings, h, len(
            targets)-1, forwardSubSteps
    )
    cuadraticError = round(QuadraticError(steps, targets, len(res.x)), 2)

    # Error with constant steps
    targets = [initialState] + UnityDLL.forwardLoop(
        initialState.x,
        initialState.v,
        np.full(0, 0,),
        "nn" * len(data_dict["objects"]),
        h,
        cuadraticErrorFrames - 1 if cuadraticErrorFrames > 0 else nSteps - 1,
        forwardSubSteps
    )

    steps = [initialState] + UnityDLL.forwardLoop(
        initialState.x, initialState.v, res.x, settings, h, len(
            targets)-1, forwardSubSteps
    )

    cuadraticErrorConstantSteps = round(
        QuadraticError(steps, targets, len(res.x)), 2)

    # Error with average parameters
    # avg = np.full(1, Average(data_dict["objects"][0]["springStiffness"]),)

    if printResult:
        title = data_dict["title"]

        log = f"{'-'*20}{title}{'-'*20}\n"
        log += f"Settings: {settings}\n"
        log += f"Simulation steps: {nSteps}\n"
        log += f"Substeps: {forwardSubSteps}\n"
        log += f"Timestep: {round(h, 3)}\n"
        log += f"No. parameters: {len(desiredParameter)}\n\n"

        log += f"Success: {res.success}\n"
        log += f"Message: {res.message}\n"
        log += f"Time elapsed: {elapsedTime}s\n\n"

        log += f"Error: {round(res.fun,2)}\n"
        log += f"Cuadratic error: {cost}\n"
        log += f"Fixed cuadratic error: {cost}\n"
        log += f"Parameter deviation: {parameterDeviation}%\n\n"

        # log += f"nfev: {res.nfev}\n"
        # log += f"nit: {res.nit}\n"
        # log += f"njev: {res.njev}\n\n"

        print(log)

        print("Sample:")
        for i in range(min(10, len(desiredParameter))):
            print(f" {round(desiredParameter[i], 4)} --> {round(res.x[i], 4)}")

        log_file = open("C:/debug/log.txt", "a+")
        log_file.write(log)
        log_file.close()

    result = OptimizationResult()
    result.newData = newData
    result.parameterDeviation = parameterDeviation
    result.cost = cost
    result.cuadraticError = cuadraticError
    result.cuadraticErrorConstant = cuadraticErrorConstantSteps
    result.elapsedTime = elapsedTime

    return result


currentProgress = 0


def ShowProgress(p):
    global currentProgress
    progress = ["-", "/", "|", "\\"]
    print(progress[currentProgress], end="\r")
    currentProgress += 1
    if currentProgress >= len(progress):
        currentProgress = 0


# https://gist.github.com/yuyay/3067185
# python -m pip install "d:/Projects/MassSpringSimulator/UnityDLL"
