#pragma once
#include "Types.h"

class Node;
class Spring 
{
private:
	Eigen::Vector3d dir;

public:
	Node* nodeA = nullptr;
	Node* nodeB = nullptr;

	float length0 = 0;
	float length = 0;

	float stiffness = 150.0f;

	float damping = 0;

	Spring() {};

	Spring(Node* A, Node* B);

	void Initialize(float stiffness, float damping);

	void UpdateState();

	void GetForce(Eigen::VectorXd* force);

	void GetdFdstiffness(Eigen::VectorXd* dforce);

	void GetdFdstiffness(std::vector<T>* dforce, int col);

	void GetForceJacobian(std::vector<T>* derivPos, std::vector<T>* derivVel);

	bool operator==(const Spring& p) const;

	void SetStiffness(float stiffness);

	void SetDamping(float damping);
};



