from ast import arg
import UnityDLL
import numpy as np
import scipy.optimize
import time

lastP = 0
steps = []


def g(p, iter, h, m_numDoFs, initialState, targets):
    global lastP
    lastP = p

    global steps
    steps = []
    current = initialState
    steps.append(initialState)

    for i in range(iter - 1):
        current = UnityDLL.forward(current.x, current.v, p, h)
        steps.append(current)

    g = np.linalg.norm(targets[iter - 1].x - steps[iter - 1].x) ** 2

    return g / iter


def G(p, iter, h, m_numDoFs, initialState, targets):
    global lastP
    lastP = p

    global steps
    steps = []
    current = initialState
    steps.append(initialState)

    for i in range(iter - 1):
        current = UnityDLL.forward(current.x, current.v, p, h)
        steps.append(current)

    G = 0
    for i in range(iter):
        G += np.linalg.norm(targets[i].x - steps[i].x) ** 2
    # G = G / iter

    return G


def dGdp(p, iter, h, m_numDoFs, initialState, targets):
    if p != lastP:
        global steps
        steps = []
        current = initialState
        steps.append(initialState)

        for i in range(iter - 1):
            current = UnityDLL.forward(current.x, current.v, p, h)
            steps.append(current)

    _dGdp = np.full(1, 0.0)
    _dGdx = []
    _dGdv = []

    for i in range(iter):
        """if i == iter - 1:
            _dGdx.append(2.0 * (targets[iter - 1].x - steps[iter - 1].x))
        else:
            _dGdx.append(np.full(m_numDoFs, 0.0))

        _dGdv.append(np.full(m_numDoFs, 0.0))"""

        _dGdx.append(2.0 * (targets[i].x - steps[i].x))
        _dGdv.append(np.full(m_numDoFs, 0.0))

    i = iter - 2
    while i >= 0:
        backward = UnityDLL.backward(
            steps[i].x,
            steps[i].v,
            steps[i + 1].x,
            steps[i + 1].v,
            p,
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


def Minimize(method="L-BFGS-B", costFunction=g, jacobian=dGdp):

    data = open("D:/Projects/MassSpringSimulator/Python/scene.txt", "r").read()

    # PARAMETERS
    iter = 100
    h = 0.01

    desiredParameter = np.full(1, 100)

    print(
        f"{'-'*60}\nIterations: {iter} Timestep: {h} Target parameter: {desiredParameter}\n{'-'*60} "
    )

    # INITIALIZATION
    initialState = UnityDLL.initialize(data)
    m_numDoFs = initialState.x.size

    # CALCULATING TARGET
    current = initialState

    targets = []
    targets.append(initialState)

    for i in range(iter - 1):
        current = UnityDLL.forward(current.x, current.v, desiredParameter, h)
        targets.append(current)

    p0 = np.array([1.2])  # initial parameter value
    args = (iter, h, m_numDoFs, initialState, targets)  # extra info

    start = time.time()

    res = scipy.optimize.minimize(
        costFunction, p0, jac=jacobian, method=method, args=args
    )

    end = time.time()

    # print(res)
    print("RESULT:", res.x)
    print("ERROR: ", np.linalg.norm(desiredParameter - res.x))
    print("Time elapsed: ", str(round((end - start) * 1000.0, 1)), "ms")

    return res.x


Minimize(costFunction=G)


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
