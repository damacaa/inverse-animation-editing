#include "pch.h"
#include "Object.h"

Object::Object(Vector3f pos)
{
	positon = pos;
	int side = 10;
	nVertices = side * side;

	vertexArray = new Vector3f[nVertices];
	vertexArray2 = new Vector3f[nVertices];
	nodeArray = new Node[nVertices];

	int c = 0;
	for (int i = 0; i < side; i++) {
		for (int j = 0; j < side; j++)
		{
			vertexArray[c].x = pos.x + i;
			vertexArray[c].y = pos.y;
			vertexArray[c].z = pos.z + j;

			nodeArray[c].position = vertexArray[c];
			c++;
		}
	}
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
	for (int i = 0; i < nVertices; i++) {
		nodeArray[i].Update(time, h);
		vertexArray[i] = nodeArray[i].position;
	}
}

Vector3f* Object::GetVertices()
{
	memcpy(vertexArray2, vertexArray, sizeof(Vector3f) * nVertices); /*Copy data only if vertexArray has been updated.*/
	return vertexArray2;
}
