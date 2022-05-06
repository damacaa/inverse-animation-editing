import UnityDLL

print(UnityDLL.add(1,1))

print(UnityDLL.add(2,1))

backStep = UnityDLL.estimate(10, 100, 0.01)

print(backStep.g)
print(backStep.dGdp.tolist())


#https://gist.github.com/yuyay/3067185
#python -m pip install "d:/Projects/MassSpringSimulator/UnityDLL"