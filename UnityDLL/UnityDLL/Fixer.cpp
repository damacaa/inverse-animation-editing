#include "pch.h"
#include "Fixer.h"

bool Fixer::CheckNodeInside(Node* n)
{
	float x = n->position.x();
	float y = n->position.y();
	float z = n->position.z();

	float i = scale.x() / 2.0f;
	float j = scale.y() / 2.0f;
	float k = scale.z() / 2.0f;

	if (x > position.x() + i)
		return false;
	if (x < position.x() - i)
		return false;

	if (y > position.y() + j)
		return false;
	if (y < position.y() - j)
		return false;

	if (z > position.z() + k)
		return false;
	if (z < position.z() - k)
		return false;

	nodesInside.push_back(n);

	return true;
}
