#include "pch.h"
#include "Fixer.h"
#include "Node.h"

bool Fixer::CheckNodeInside(Node* n)
{
	double x = n->position.x();
	double y = n->position.y();
	double z = n->position.z();
	
	double i = scale.x() / 2.0f;
	double j = scale.y() / 2.0f;
	double k = scale.z() / 2.0f;

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
