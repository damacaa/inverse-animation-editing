#include "pch.h"
#include "Node.h"

void Node::UpdateSymplectic(float h)
{
        ComputeForces();
        acc = force / (volume * density);// nodeMass

        vel += acc * h;
        position += vel * h;

        force = Eigen::Vector3f(0,0,0);
}

void Node::ComputeForces()
{
	force += (volume * density) * Eigen::Vector3f(0, 1, 0) * gravity;// nodeMass
	force += -damping * vel;
}

void Node::Update(float time, float h)
{
	if (this->locked)
		return;

	UpdateSymplectic(h);
}
