#pragma once
#include "Node.h"
class Spring
{
private:

public:
	Node* nodeA, * nodeB;

	float length0;
	float length;

	float stiffness;

	float volume;
	float damping;

	bool springType;

	Spring() {};

	Spring(Node* A, Node* B, float _volume, float _stiffness, float _damping, bool _springType)
	{
		nodeA = A;
		nodeB = B;

		volume = _volume;
		stiffness = _stiffness;
		damping = _damping;
		springType = _springType;

		length = (nodeA->position - nodeB->position).norm();
		length0 = length;
	}
	void ComputeForces();
};

