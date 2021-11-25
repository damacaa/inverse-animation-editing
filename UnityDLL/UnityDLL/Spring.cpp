#include "pch.h"
#include "Spring.h"

void Spring::ComputeForces()
{
	Eigen::Vector3f u = nodeA->position - nodeB->position;

	length = u.norm();
	u.normalize();

	Eigen::Vector3f force = -(volume / (length0 * length0)) * stiffness * (length - length0) * u;

	force += -damping * u.dot(nodeA->vel - nodeB->vel) * u;

	nodeA->force += force;
	nodeB->force -= force;
}
