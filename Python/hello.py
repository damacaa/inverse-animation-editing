import UnityDLL

data = open("D:/Projects/MassSpringSimulator/Python/scene.txt", "r").read()

print(data)


initialState = UnityDLL.initialize(data)

print(initialState.x.tolist(), initialState.v.tolist())

print(UnityDLL.forward(initialState.x, initialState.v, 0.5, 0.01))
# print(backStep.dGdp.tolist())


# https://gist.github.com/yuyay/3067185
# python -m pip install "d:/Projects/MassSpringSimulator/UnityDLL"
