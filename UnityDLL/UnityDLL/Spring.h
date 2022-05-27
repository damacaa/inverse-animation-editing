#pragma once
#include "Types.h"

class Node;
class Spring 
{
private:
	Eigen::Vector3d dir;

public:
	Node* nodeA = 0;
	Node* nodeB = 0;
	Node* oppositeNode = 0;

	float length0 = 0;
	float length = 0;

	float stiffness = 150.0f;

	float volume = 0;
	float damping = 0;

	std::string id = "";

	enum class SpringType {
		traction,
		bending
	};

	SpringType springType = SpringType::traction;

	Spring() {};

	Spring(Node* A, Node* B);

	void ComputeForces();

	void Initialize(float stiffness, float damping);

	void UpdateState();

	void GetForce(Eigen::VectorXd* force);

	void GetdFdstiffness(Eigen::VectorXd* dforce);

	void GetdFdstiffness(std::vector<T>* dforce);

	void GetForceJacobian(std::vector<T>* derivPos, std::vector<T>* derivVel);

	bool operator==(const Spring& p) const;

	void SetStiffness(float stiffness);

	void SetDamping(float damping);
};



