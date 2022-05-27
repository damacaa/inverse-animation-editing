#include "pch.h"
#include "Spring.h"
#include "Node.h"

Spring::Spring(Node* A, Node* B)
{
	nodeA = A;
	nodeB = B;

	length = (nodeA->position - nodeB->position).norm();
	length0 = length;

	if (nodeA->meshId > nodeB->meshId) {
		id = std::to_string(10000 * nodeA->meshId) + std::to_string(nodeB->meshId);
	}
	else {
		id = std::to_string(10000 * nodeB->meshId) + std::to_string(nodeA->meshId);
	}
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
	Eigen::Vector3d f = -stiffness * (length - length0) * dir;//
	//Eigen::Vector3d f = -(volume / (length0 * length0)) * stiffness * (length - length0) * dir;
	f += -damping * dir.dot(nodeA->vel - nodeB->vel) * dir;

	(*force)[nodeA->index] += f.x();
	(*force)[nodeA->index + 1] += f.y();
	(*force)[nodeA->index + 2] += f.z();

	(*force)[nodeB->index] -= f.x();
	(*force)[nodeB->index + 1] -= f.y();
	(*force)[nodeB->index + 2] -= f.z();
}

void Spring::GetdFdstiffness(Eigen::VectorXd* dforce)
{
	Eigen::Vector3d f = -(length - length0) * dir;//

	(*dforce)[nodeA->index] += f.x();
	(*dforce)[nodeA->index + 1] += f.y();
	(*dforce)[nodeA->index + 2] += f.z();

	(*dforce)[nodeB->index] -= f.x();
	(*dforce)[nodeB->index + 1] -= f.y();
	(*dforce)[nodeB->index + 2] -= f.z();
}

void Spring::GetdFdstiffness(std::vector<T>* dforce, int col)
{
	Eigen::Vector3d f = -(length - length0) * dir;


	dforce->push_back(T(nodeA->index, col, f(nodeA->index)));
	dforce->push_back(T(nodeA->index, col, f(nodeA->index + 1)));
	dforce->push_back(T(nodeA->index, col, f(nodeA->index + 2)));

	dforce->push_back(T(nodeB->index, col, -f(nodeB->index)));
	dforce->push_back(T(nodeB->index, col, -f(nodeB->index + 1)));
	dforce->push_back(T(nodeB->index, col, -f(nodeB->index + 2)));
}

// Get Force Jacobian
void Spring::GetForceJacobian(std::vector<T>* derivPos, std::vector<T>* derivVel)
{
	Eigen::Matrix3d dirMat = dir * dir.transpose();

	Eigen::Matrix3d dFadxa = (-stiffness * (length - length0) / length * Eigen::Matrix3d::Identity()) - (stiffness * length0 / length * dirMat);
	Eigen::Matrix3d dFadva = -damping * dirMat;

	//damping = beta * stiffness
	for (size_t i = 0; i < 3; i++)
	{
		for (size_t j = 0; j < 3; j++)
		{
			derivPos->push_back(T(nodeA->index + i, nodeA->index + j, dFadxa(i, j)));
			derivPos->push_back(T(nodeB->index + i, nodeB->index + j, dFadxa(i, j)));
			derivPos->push_back(T(nodeA->index + i, nodeB->index + j, -(dFadxa(i, j))));
			derivPos->push_back(T(nodeB->index + i, nodeA->index + j, -(dFadxa(i, j))));

			derivVel->push_back(T(nodeA->index + i, nodeA->index + j, dFadva(i, j)));
			derivVel->push_back(T(nodeB->index + i, nodeB->index + j, dFadva(i, j)));
			derivVel->push_back(T(nodeA->index + i, nodeB->index + j, -(dFadva(i, j))));
			derivVel->push_back(T(nodeB->index + i, nodeA->index + j, -(dFadva(i, j))));
		}
	}
}

bool Spring::operator==(const Spring& p) const
{
	return (nodeA == p.nodeA && nodeB == p.nodeB) ||
		(nodeA == p.nodeB && nodeB == p.nodeA);
}

void Spring::SetStiffness(float stiffness)
{
	//this->stiffness = stiffness * volume / (length0 * length0);
	this->stiffness = stiffness;
}

void Spring::SetDamping(float beta)
{
	damping = beta * stiffness;
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
