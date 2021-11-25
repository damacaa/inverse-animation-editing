#include "pch.h"
#include "Object.h"
#include <set>
#include <map>



Object::Object(Vector3f pos, Vector3f* vertices, int nVerts, int* _triangles, int nTriangles)
{
	nVertices = nVerts;
	positon = Eigen::Vector3f(
		(float)pos.x,
		(float)pos.y,
		(float)pos.z);

	vertexArray = new Vector3f[nVertices];
	vertexArray2 = new Vector3f[nVertices];

	memcpy(vertexArray, vertices, sizeof(Vector3f) * nVertices);
	memcpy(vertexArray2, vertexArray, sizeof(Vector3f) * nVertices);

	//Create nodes
	nodeArray = new Node[nVertices];
	for (int i = 0; i < nVertices; i++) {
		vertexArray[i] = (Vector3f)vertices[i];
		nodeArray[i].position = Eigen::Vector3f(
			(float)vertexArray[i].x,
			(float)vertexArray[i].y,
			(float)vertexArray[i].z);
		nodeArray[i].id = std::to_string(i);
		nodeArray[i].damping = damping;
	}

	nodeArray[0].locked = true;
	nodeArray[10].locked = true;
	nodeArray[110].locked = true;
	nodeArray[120].locked = true;

	//Creating springs
	int* triangles = new int[nTriangles];
	memcpy(triangles, _triangles, sizeof(int) * nTriangles);

	std::map<std::string, Spring> springMap;

	Spring* newSprings = new Spring[3];
	for (int i = 0; i < nTriangles; i += 3)
	{
		newSprings[0] = Spring(&nodeArray[triangles[i]], &nodeArray[triangles[i + 1]], damping);
		newSprings[1] = Spring(&nodeArray[triangles[i]], &nodeArray[triangles[i + 2]], damping);
		newSprings[2] = Spring(&nodeArray[triangles[i + 1]], &nodeArray[triangles[i + 2]], damping);


		Eigen::Vector3f side1 = nodeArray[triangles[i + 1]].position - nodeArray[triangles[i]].position;
		Eigen::Vector3f side2 = nodeArray[triangles[i + 2]].position - nodeArray[triangles[i]].position;

		float area = 0.5f * side1.cross(side2).norm();

		//float area = 0.1f;

		for (int x = 0; x < 3; x++)
		{//Add the proper mass to each node

			nodeArray[triangles[i + x]].volume += area / 3;

			std::map<std::string, Spring>::iterator it = springMap.find(newSprings[x].id);

			if (it != springMap.end())
			{
				//La arista está en el diccionario
				/*if (useBendingSprings)
				{
					edge.volume += area;
					Edge newEdge = new Edge(edge.other, otherEdge.other, -1, SpringType.Bending);

					if (!edgeDictionary.TryGetValue(newEdge, out otherEdge))
					{
						edgeDictionary.Add(newEdge, newEdge);
					}
				}*/
			}
			else
			{
				//La arista no está en el diccionario
				springMap[newSprings[x].id] = newSprings[x];
			}

			springMap[newSprings[x].id].volume += area;
		}
	}

	nSprings = springMap.size();
	springArray = new Spring[nSprings];

	std::map<std::string, Spring>::iterator it;
	int i = 0;
	for (it = springMap.begin(); it != springMap.end(); ++it) {
		springArray[i] = springMap[it->first];
		i++;
	}


	delete[] newSprings;
	delete[] triangles;
}

Object::~Object()
{
	delete[] vertexArray;
	delete[] vertexArray2;
	delete[] nodeArray;

	vertexArray = 0;
	vertexArray2 = 0;
	nodeArray = 0;
}

void Object::Update(float time, float h)
{
	float newH = h / (float)substeps;

	for (size_t i = 0; i < substeps; i++)
	{

		for (int i = 0; i < nSprings; i++)
		{
			springArray[i].ComputeForces();
		}

		for (int i = 0; i < nVertices; i++) {
			nodeArray[i].Update(time, newH);
		}
	}

	updated = true;
}

Vector3f* Object::GetVertices()
{
	for (int i = 0; i < nVertices; i++) {
		vertexArray[i] = Vector3f(
			(float)nodeArray[i].position.x(),
			(float)nodeArray[i].position.y(),
			(float)nodeArray[i].position.z());
	}

	memcpy(vertexArray2, vertexArray, sizeof(Vector3f) * nVertices); /*Copy data only if vertexArray has been updated.*/

	updated = false;
	
	return vertexArray2;
}
