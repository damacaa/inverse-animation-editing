#include "pch.h"
#include "Collider.h"

Eigen::Vector3d Collider::GetForce(Eigen::Vector3d position)
{
	switch (type)
	{
	case 0:
		break;
	case 1: {

		double r = scale.x() / 2.0;
		double distanceToCenter = (position - this->position).norm();
		if (distanceToCenter < r) {
			return 10.0 * (position - this->position).normalized() * (r - distanceToCenter);
		}
		break;
	}
	default:
		break;
	}
	return Eigen::Vector3d();
}
