#pragma once
#include "Types.h"
#include <math.h>   

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
	int meshId = -1;

	Eigen::Index index = 0;
	Eigen::Vector3d position = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
	Eigen::Vector3d vel = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
	Eigen::Vector3d acc = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
	Eigen::Vector3d force = Eigen::Vector3d(0.0f, 0.0f, 0.0f);

	float penalty = 0;
	float thickness = 0;
	float damping = 1;
	float gravity = -9.81f;

	float mass = 1;
	float volume = 1;
	float density = 1;

	bool isFixed;


	void Initialize(int ind);

	void GetPosition(Eigen::VectorXd* pos);

	void SetPosition(Eigen::VectorXd* pos);

	void GetVelocity(Eigen::VectorXd* _vel);

	void SetVelocity(Eigen::VectorXd* _vel);

	void GetMass(Eigen::MatrixXd* Mass);

	void GetMass(std::vector<T>* massTripletVector);

	void GetMassInverse(Eigen::MatrixXd* massInv);

	void GetMassInv(std::vector<T>* massTripletVector);

	void GetForce(Eigen::VectorXd* force);

	void GetForceJacobian(std::vector<T>* derivPos, std::vector<T>* derivVel);

	void FixVector(Eigen::VectorXd* v);

	void FixMatrix(Eigen::MatrixXd* M);

	void FixMatrix(SpMat* M);

	void UpdateOld(float time, float h);
};

