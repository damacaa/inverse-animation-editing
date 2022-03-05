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
	// TO BE COMPLETED
	/* Vector3 u = nodeA.Pos - nodeB.Pos;
		Length = u.magnitude;
		u = u / Length;

		VectorXD U = new DenseVectorXD(3);
		U[0] = u.x;
		U[1] = u.y;
		U[2] = u.z;

		MatrixXD dFadxa = (-Stiffness * (Length - Length0) / Length * DenseMatrixXD.CreateIdentity(3)) - (Stiffness * Length0 / Length * U.OuterProduct(U));

		MatrixXD dFadva = -Damping * U.OuterProduct(U);


		dFdx.SetSubMatrix(nodeA.index, nodeA.index, dFdx.SubMatrix(nodeA.index, 3, nodeA.index, 3) + dFadxa);
		dFdx.SetSubMatrix(nodeB.index, nodeB.index, dFdx.SubMatrix(nodeB.index, 3, nodeB.index, 3) + dFadxa);
		dFdx.SetSubMatrix(nodeA.index, nodeB.index, dFdx.SubMatrix(nodeA.index, 3, nodeB.index, 3) - dFadxa);
		dFdx.SetSubMatrix(nodeB.index, nodeA.index, dFdx.SubMatrix(nodeB.index, 3, nodeA.index, 3) - dFadxa);*/
}

bool Spring::operator==(const Spring& p) const
{
	return (nodeA == p.nodeA && nodeB == p.nodeB) ||
		(nodeA == p.nodeB && nodeB == p.nodeA);
}
