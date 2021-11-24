#include "pch.h"
#include "Object.h"

Object::Object(Vector3f pos, Vector3f* vertices, int nVerts, int* triangles, int nTriangles)
{
	nVertices = nVerts;
	positon = Eigen::Vector3d(pos.x,
		pos.y,
		pos.z);

	vertexArray = new Vector3f[nVertices];
	vertexArray2 = new Vector3f[nVertices];
	nodeArray = new Node[nVertices];
	//springArray = new Spring[100];

	memcpy(vertexArray, vertices, sizeof(Vector3f) * nVertices);

	//Create nodes
	for (int i = 0; i < nVertices; i++) {
		vertexArray[i] = (Vector3f)vertices[i];
		nodeArray[i].position = Eigen::Vector3d(vertexArray[i].x,
			vertexArray[i].y,
			vertexArray[i].z);
	}

	nodeArray[0].locked = true;
	nodeArray[9].locked = true;
	nodeArray[89].locked = true;
	nodeArray[99].locked = true;

	//Creating springs
	/*//Creating edges
	for (int i = 0; i < nTriangles; i += 3)
	{
		//float area = 0.5f * GetNormal(nodes[triangles[i]].pos, nodes[triangles[i + 1]].pos, nodes[triangles[i + 2]].pos).magnitude;

		//Loop triangles
		int a = 0;
		int b = 0;
		int other = 2;

		for (int x = 0; x < 3; x++)
		{//Add the proper mass to each node
			//nodeArray[triangles[i + x]].volume += area / 3;

			//Loop each edge
			Edge edge = Edge(triangles[i + a], triangles[i + b + 1], triangles[i + other]);

			Edge otherEdge;
			if (edgeDictionary.TryGetValue(edge, out otherEdge))
			{
				//La arista está en el diccionario
				otherEdge.volume += area;
				if (useBendingSprings)
				{
					edge.volume += area;
					Edge newEdge = new Edge(edge.other, otherEdge.other, -1, SpringType.Bending);

					if (!edgeDictionary.TryGetValue(newEdge, out otherEdge))
					{
						edgeDictionary.Add(newEdge, newEdge);
					}
				}
			}
			else
			{
				//La arista no está en el diccionario
				edge.volume += area;
				edgeDictionary.Add(edge, edge);
			}
			if (b == 0) { b = 1; other = 1; }
			else { a = 1; other = 0; }
		}
	}

	//Creating a spring for each edge
	foreach(Edge e in edgeDictionary.Values)
	{
		if (e.springType == SpringType.Traction)
		{
			springs.Add(new Spring(nodes[e.a], nodes[e.b], e.volume, tractionSpringStiffness, damping, e.springType));
		}
		else
		{
			springs.Add(new Spring(nodes[e.a], nodes[e.b], .5f, bendingSpringStiffness, damping, e.springType));
		}
	}*/



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
	for (int i = 0; i < nSprings; i++)
	{
		springArray[i].ComputeForces();
	}

	for (int i = 0; i < nVertices; i++) {
		nodeArray[i].Update(time, h);
		vertexArray[i] = Vector3f(nodeArray[i].position.x(),
			nodeArray[i].position.y(),
			nodeArray[i].position.z());
	}
}

Vector3f* Object::GetVertices()
{
	memcpy(vertexArray2, vertexArray, sizeof(Vector3f) * nVertices); /*Copy data only if vertexArray has been updated.*/
	return vertexArray2;
}
