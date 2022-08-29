#include "pch.h"
#include "Collider.h"

Eigen::Vector3d Collider::GetForce(Eigen::Vector3d position)
{
	switch (type)
	{
	case 0:
		break;
	case 1: {
		//Sphere
		double r = scale.x() / 2.0;
		double distanceToCenter = (position - this->position).norm();
		if (distanceToCenter < r) {
			Eigen::Vector3d normal = (this->position - position).normalized();
			double penetration = r - distanceToCenter;
			return -k * normal * penetration;
		}
		break;
	}
	default:
		break;
	}
	return Eigen::Vector3d(0, 0, 0);
}

Eigen::Matrix3d Collider::GetJacobian(Eigen::Vector3d position)
{
	switch (type)
	{
	case 0:
		break;
	case 1: {
		//Sphere
		double r = scale.x() / 2.0;
		double distanceToCenter = (this->position - position).norm();
		if (distanceToCenter < r) {
			Eigen::Vector3d normal = (this->position - position).normalized();
			double penetration = r - distanceToCenter;
			return -k * normal * normal.transpose();
		}
		break;
	}
	default:
		break;
	}
	return Eigen::Matrix3d().setZero();
}
