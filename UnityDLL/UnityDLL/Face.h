#pragma once
#include "Types.h"

class Node;
class Face
{
public:
	Node* A;
	Node* B;
	Node* C;

	double dragCoefficient;
	Eigen::Vector3d wind = Eigen::Vector3d(1, 0, 1);
	Eigen::Vector3d normal = Eigen::Vector3d(0, 1, 0);;
	double area = 0;

	Face() {};
	Face(Node* A, Node* B, Node* C, double dragCoefficient);

	void GetForce(Eigen::VectorXd* force);
	void GetForceJacobian(std::vector<T>* derivPos, std::vector<T>* derivVel);
};

