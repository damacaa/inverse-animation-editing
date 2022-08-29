#pragma once
#include <vector>
#include "Types.h"
class Collider
{
private:
	int type;
	Eigen::Vector3d position;
	Eigen::Vector3d rotation;
	Eigen::Vector3d scale;
	double k = 10.0;
public:
	Collider(int _type, Eigen::Vector3d _pos, Eigen::Vector3d _rot, Eigen::Vector3d _scale) :
		type(_type), position(_pos), rotation(_rot), scale(_scale) {}

	Eigen::Vector3d GetForce(Eigen::Vector3d position);
	Eigen::Matrix3d GetJacobian(Eigen::Vector3d position);
};

