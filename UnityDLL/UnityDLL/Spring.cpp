#include "pch.h"
#include "Spring.h"
#include "Node.cpp"

void Spring::ComputeForces()
{
	Eigen::Vector3d u = nodeA->position - nodeB->position;

	length = u.norm();

	u.normalize();

	Eigen::Vector3d force = -(volume / (length0 * length0)) * stiffness * (length - length0) * u;

	force += -damping * u.dot(nodeA->vel - nodeB->vel) * u;

	nodeA->force += force;
	nodeB->force -= force;
}

void Spring::Initialize(float stiffness, float damping)
{
    // TO BE COMPLETED
}

// Update spring state
void Spring::UpdateState()
{
	dir = nodeA->position - nodeB->position;
	length = dir.norm();
	dir.normalize();
}

// Get Force
void Spring::GetForce(Eigen::VectorXd* force)
{
    // TO BE COMPLETED
}

// Get Force Jacobian
void Spring::GetForceJacobian(Eigen::MatrixXd* dFdx, Eigen::MatrixXd* dFdv)
{
    // TO BE COMPLETED
}
