#pragma once
#include "Types.h"
#include "Node.h"
class Object
{
public:
	Object(Vector3f pos, Vector3f* vertices, int nVerts);
	~Object();

	int id = -1;

	Vector3f positon;

	Vector3f* vertexArray = 0;
	Vector3f* vertexArray2 = 0;
	int nVertices = 0;

	//int* triangles = 0;
	//int nTriangles = 0;

	Node* nodeArray = 0;

	void Update(float time, float h);

	Vector3f* GetVertices();
};

