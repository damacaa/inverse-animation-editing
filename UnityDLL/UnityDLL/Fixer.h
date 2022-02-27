#pragma once
#include "Node.h"
#include <vector>
class Fixer
{

public:
	Fixer() : position(Eigen::Vector3d(0, 0, 0)), scale(Eigen::Vector3d(1, 1, 1)) {}
	Fixer(Vector3f _position, Vector3f _scale) {
		position = Eigen::Vector3d(_position.x, _position.y, _position.z);
		scale = Eigen::Vector3d(_scale.x, _scale.y, _scale.z);
	}
	~Fixer() { nodesInside.clear(); }

	bool CheckNodeInside(Node* n);

	Eigen::Vector3d position;
	Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1);

	std::vector<Node*> nodesInside;
};

