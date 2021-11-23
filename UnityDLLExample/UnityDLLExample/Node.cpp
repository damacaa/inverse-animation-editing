#include "pch.h"
#include "Node.h"

void Node::Update(float time, float h)
{
	if (!this->locked)
		this->position.y = sinf(this->position.x + this->position.z + time);
}
