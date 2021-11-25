#pragma once
#include "Types.h"
#include "Node.h"
class Spring
{
private:

public:
	Node* nodeA = 0;
	Node* nodeB = 0;

	float length0 = 0;
	float length = 0;

	float stiffness = 100.0f;

	float volume = 0;
	float damping = 0;

	std::string id = "";

	enum class SpringType {
		traction,
		bending
	};

	SpringType springType = SpringType::traction;

	Spring() {};

	Spring(Node* A, Node* B, float _damping)
	{
		nodeA = A;
		nodeB = B;

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

	Spring(Node* A, Node* B, float _volume, float _stiffness, float _damping, SpringType _springType)
	{
		nodeA = A;
		nodeB = B;

		volume = _volume;
		stiffness = _stiffness;
		damping = _damping;
		springType = _springType;

		length = (nodeA->position - nodeB->position).norm();
		length0 = length;

		if (nodeA->id > nodeB->id) {
			id = nodeA->id + nodeB->id;
		}
		else {
			id = nodeB->id + nodeA->id;
		}
	}
	void ComputeForces();

	bool operator==(const Spring& p) const {
		return (nodeA == p.nodeA && nodeB == p.nodeB) ||
			(nodeA == p.nodeB && nodeB == p.nodeA);
	}
};



