#include "pch.h"
#include "Face.h"
#include "Node.h"

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

	Eigen::Vector3d windForce = dragCoefficient * area * normal.dot(wind - avgVel) * normal;

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
	Eigen::Vector3d windForce = dragCoefficient * area * normal;
	//double windForce = dragCoefficient * area;

	derivVel->push_back(T(A->index, A->index, windForce(0)));
	derivVel->push_back(T(A->index + 1, A->index + 1, windForce(1)));
	derivVel->push_back(T(A->index + 2, A->index + 2, windForce(2)));

	derivVel->push_back(T(B->index, B->index, windForce(0)));
	derivVel->push_back(T(B->index + 1, B->index + 1, windForce(1)));
	derivVel->push_back(T(B->index + 2, B->index + 2, windForce(2)));

	derivVel->push_back(T(C->index, C->index, windForce(0)));
	derivVel->push_back(T(C->index + 1, C->index + 1, windForce(1)));
	derivVel->push_back(T(C->index + 2, C->index + 2, windForce(2)));
}
