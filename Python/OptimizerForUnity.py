# Read scene file
from unittest import result
from Optimizer import Minimize


sceneData = open(
    "../UnityProject/Assets/scene.txt", "r"
).read()

# Get optimized scene
result = Minimize(data=sceneData, printResult=True)

text_file = open(
    "../UnityProject/Assets/scene_optimized.txt", "w")
text_file.write(result.newData)
text_file.close()
