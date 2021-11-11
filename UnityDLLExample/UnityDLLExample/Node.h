#pragma once
#include "Types.h"
#include <math.h>   
class Node
{
public:
	Node() { this->position = Vector3f(); };

	Node(Vector3f pos) {
		this->position = pos;
	}

	Vector3f position;
	void Update(float time, float h);
};

