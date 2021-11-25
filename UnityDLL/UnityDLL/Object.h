#pragma once
#include "Types.h"
#include "Node.h"
#include "Spring.h"

class Object
{
private:
	Eigen::Vector3f GetNormal(Eigen::Vector3f a, Eigen::Vector3f b, Eigen::Vector3f c);
	
public:
	Object(Vector3f pos, Vector3f* vertices, int nVerts, int* triangles, int nTriangles);
	~Object();

	int id = -1;
	bool updated = false;
	int substeps = 5;
	float damping = 1;

	Eigen::Vector3f positon;

	Vector3f* vertexArray = 0;
	Vector3f* vertexArray2 = 0;
	int nVertices = 0;

	//int* triangles = 0;
	//int nTriangles = 0;

	Node* nodeArray = 0;
	Spring* springArray = 0;
	int nSprings = 0;

	void Update(float time, float h);

	Vector3f* GetVertices();
};

