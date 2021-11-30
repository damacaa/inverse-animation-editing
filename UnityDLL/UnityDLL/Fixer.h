#pragma once
#include "Node.h"
#include <vector>
class Fixer
{

public:
	Fixer() : position(Eigen::Vector3f(0, 0, 0)), scale(Eigen::Vector3f(1, 1, 1)) {}
	Fixer(Vector3f _position, Vector3f _scale) {
		position = Eigen::Vector3f(_position.x, _position.y, _position.z);
		scale = Eigen::Vector3f(_scale.x, _scale.y, _scale.z);
	}
	~Fixer() { nodesInside.clear(); }

	bool CheckNodeInside(Node* n);

	Eigen::Vector3f position;
	Eigen::Vector3f scale = Eigen::Vector3f(1, 1, 1);

	std::vector<Node*> nodesInside;
};

