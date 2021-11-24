#include "pch.h"
#include "Object.h"

Object::Object(Vector3f pos, Vector3f* vertices, int nVerts)
{
	positon = pos;
	nVertices = nVerts;

	vertexArray = new Vector3f[nVertices];
	vertexArray2 = new Vector3f[nVertices];
	nodeArray = new Node[nVertices];

	memcpy(vertexArray, vertices, sizeof(Vector3f) * nVertices);

	for (int i = 0; i < nVertices; i++) {
		vertexArray[i] = (Vector3f)vertices[i];
		nodeArray[i].position = vertexArray[i];
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
