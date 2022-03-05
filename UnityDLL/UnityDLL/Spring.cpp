#include "pch.h"
#include "Spring.h"
#include "Node.h"

Spring::Spring(Node* A, Node* B, float _stiffness, float _damping)
{
	nodeA = A;
	nodeB = B;
	stiffness = _stiffness;
	damping = _damping;

	length = (nodeA->position - nodeB->position).norm();
	length0 = length;

	if (nodeA->id > nodeB->id) {
		id = nodeA->id + nodeB->id;
	}
	else {
		id = nodeB->id + nodeA->id;
	}
}

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
	UpdateState();
	length0 = length;
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
	Eigen::Vector3d f = -stiffness * (length - length0) * dir;

	(*force)[nodeA->index] += f.x();
	(*force)[nodeA->index + 1] += f.y();
	(*force)[nodeA->index + 2] += f.z();

	(*force)[nodeB->index] -= f.x();
	(*force)[nodeB->index + 1] -= f.y();
	(*force)[nodeB->index + 2] -= f.z();
}

// Get Force Jacobian
void Spring::GetForceJacobian(std::vector<T>* derivPos, std::vector<T>* derivVel)
{
	Eigen::Matrix3d dirMat = dir * dir.transpose();

	Eigen::Matrix3d dFadxa = (-stiffness * (length - length0) / length * Eigen::Matrix3d::Identity()) - (stiffness * length0 / length * dirMat);
	Eigen::Matrix3d dFadva = -damping * dirMat;

	for (size_t i = 0; i < 3; i++)
	{
		for (size_t j = 0; j < length; j++)
		{
			derivPos->push_back(T(nodeA->index + i, nodeA->index + i, dFadxa(i, j)));
			derivPos->push_back(T(nodeB->index + i, nodeB->index + i, dFadxa(i, j)));
			derivPos->push_back(T(nodeA->index + i, nodeB->index + i, -dFadxa(i, j)));
			derivPos->push_back(T(nodeB->index + i, nodeA->index + i, -dFadxa(i, j)));

			derivVel->push_back(T(nodeA->index + i, nodeA->index + i, dFadva(i, j)));
			derivVel->push_back(T(nodeB->index + i, nodeB->index + i, dFadva(i, j)));
			derivVel->push_back(T(nodeA->index + i, nodeB->index + i, -dFadva(i, j)));
			derivVel->push_back(T(nodeB->index + i, nodeA->index + i, -dFadva(i, j)));
		}
	}


}

bool Spring::operator==(const Spring& p) const
{
	return (nodeA == p.nodeA && nodeB == p.nodeB) ||
		(nodeA == p.nodeB && nodeB == p.nodeA);
}
