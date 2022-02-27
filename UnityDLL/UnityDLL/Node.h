#pragma once
#include "Types.h"
#include <math.h>   
#include <Eigen/Core>
#include <Eigen/Dense>


class Node
{
private:
	inline void UpdateSymplecticNode(float h) {
		ComputeForces();
		acc = force / (volume * density);

		vel += acc * h;
		position += vel * h;

		force = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
	};

	inline void ComputeForces() {
		force += (volume * density) * Eigen::Vector3d(0.0f, gravity, 0.0f);
		force += -damping * vel;
	};

public:
	Node() : isFixed(false) {};

	int index = 0;
	Eigen::Vector3d position = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
	Eigen::Vector3d vel = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
	Eigen::Vector3d acc = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
	Eigen::Vector3d force = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
	float mass = 1;

	float penalty = 0;
	float thickness = 0;
	float damping = 1;
	float gravity = -9.81f;

	float volume = 1;
	float density = 1;

	bool isFixed;

	std::string id = "";

	inline void Initialize(int ind, float mass, float damping) {};
	inline void GetForce(Eigen::VectorXd* force) {};
	inline void GetForceJacobian(Eigen::MatrixXd* dFdx, Eigen::MatrixXd* dFdv) {};
	inline void GetPosition(Eigen::VectorXd* pos) {};
	inline void SetPosition(Eigen::VectorXd* pos) {};
	inline void GetVelocity(Eigen::VectorXd* _vel) {};
	inline void SetVelocity(Eigen::VectorXd* _vel) {};
	inline void GetMass(Eigen::MatrixXd* mass) {};
	inline void GetMassInverse(Eigen::MatrixXd* massInv) {};
	inline void FixVector(Eigen::VectorXd* v) {};
	inline void FixMatrix(Eigen::MatrixXd* M) {};

	inline void Update(float time, float h) {
		if (this->isFixed)
			return;

		ComputeForces();
		acc = force / (volume * density);
		if (this->isFixed)
			return;

		vel += acc * h;
		position += vel * h;
		force = Eigen::Vector3d(0, 0, 0);
	};
};

