import UnityDLL
import numpy as np
from scipy.optimize import fmin_bfgs

data = open("D:/Projects/MassSpringSimulator/Python/scene.txt", "r").read()

# PARAMETERS
iter = 100
h = 0.1
m_numDoFs = 0

desiredMass = 1.5

# INITIALIZATION
initialState = UnityDLL.initialize(data)
m_numDoFs = initialState.x.size

current = initialState
for i in range(iter):
    current = UnityDLL.forward(current.x, current.v, desiredMass, h)
desired = current


def g(x):
    current = initialState
    for i in range(iter):
        current = UnityDLL.forward(current.x, current.v, x, h)

    g = np.linalg.norm(desired.x - current.x)
    return 1000 * g * g


def dGdp(x):

    steps = []
    current = initialState
    for i in range(iter):
        current = UnityDLL.forward(current.x, current.v, x, h)
        steps.append(current)

    _dGdp = np.full(1, 0.0)
    _dGdx = []
    _dGdv = []

    for i in range(iter):
        if i == iter - 1:
            _dGdx.append(2.0 * (desired.x - steps[iter - 1].x))
        else:
            _dGdx.append(np.full(m_numDoFs, 0.0))

        _dGdv.append(np.full(m_numDoFs, 0.0))

    i = iter - 2
    while i >= 0:
        backward = UnityDLL.bacward(
            steps[i].x,
            steps[i].v,
            steps[i + 1].x,
            steps[i + 1].v,
            x,
            _dGdx[i + 1],
            _dGdv[i + 1],
            h,
        )

        _dGdp += backward.dGdp
        _dGdx[i] += backward.dGdx
        _dGdv[i] += backward.dGdv

        i -= 1

    return _dGdp


print(dGdp(0))

x0 = np.array([1.0])
res = fmin_bfgs(g, x0, fprime=dGdp)

print(res)


# https://gist.github.com/yuyay/3067185
# python -m pip install "d:/Projects/MassSpringSimulator/UnityDLL"
