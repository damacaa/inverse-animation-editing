#include "pch.h"
#include "Face.h"
#include "Node.h"
#include "PhysicsManager.h"

Face::Face(Node* A, Node* B, Node* C, double dragCoefficient)
{
	this->A = A;
	this->B = B;
	this->C = C;

	this->dragCoefficient = dragCoefficient;
}

void Face::GetForce(Eigen::VectorXd* force)
{
	Eigen::Vector3d avgVel = (A->vel + B->vel + C->vel) / 3.0;

	Eigen::Vector3d side1 = B->position - A->position;
	Eigen::Vector3d side2 = C->position - A->position;

	normal = side1.cross(side2);
	area = 0.5 * normal.norm();

	Eigen::Vector3d windForce = dragCoefficient * area  * (normal * normal.transpose()) * (PhysicsManager::windVelocity - avgVel) / 3.0;

	(*force)[A->index] += windForce.x();
	(*force)[A->index + 1] += windForce.y();
	(*force)[A->index + 2] += windForce.z();

	(*force)[B->index] += windForce.x();
	(*force)[B->index + 1] += windForce.y();
	(*force)[B->index + 2] += windForce.z();

	(*force)[C->index] += windForce.x();
	(*force)[C->index + 1] += windForce.y();
	(*force)[C->index + 2] += windForce.z();
}

void Face::GetForceJacobian(std::vector<T>* derivPos, std::vector<T>* derivVel)
{
	//Eigen::Vector3d windForce = dragCoefficient * area * normal / 3.0;
	Eigen::Matrix3d dfwindForce = -(1.0 / 3.0) * dragCoefficient * area * normal * normal.transpose();
	//double windForce = dragCoefficient * area;

	for (size_t i = 0; i < 3; i++)
	{
		for (size_t j = 0; j < 3; j++)
		{
			derivVel->push_back(T(A->index + i, A->index + j, dfwindForce(i, j)));
			derivVel->push_back(T(B->index + i, B->index + j, dfwindForce(i, j)));
			derivVel->push_back(T(C->index + i, C->index + j, dfwindForce(i, j)));
		}
	}
}
