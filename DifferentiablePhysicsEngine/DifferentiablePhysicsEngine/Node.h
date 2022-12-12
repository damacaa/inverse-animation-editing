#pragma once
#include "Types.h"
#include <math.h>   

class Collider;
class Node
{
private:

public:
	Node() : isFixed(false) {};
	int meshId = -1;

	Eigen::Index index = 0;
	Eigen::Vector3d position = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
	Eigen::Vector3d vel = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
	Eigen::Vector3d acc = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
	Eigen::Vector3d force = Eigen::Vector3d(0.0f, 0.0f, 0.0f);

	double penalty = 0;
	double thickness = 0;
	double damping = 1;
	double gravity = -9.81f;

	double mass = 1;
	double volume = 1;
	double density = 1;

	bool isFixed;

	void Initialize(int ind);

	void GetPosition(Eigen::VectorXd* pos);

	void SetPosition(Eigen::VectorXd* pos);

	void GetVelocity(Eigen::VectorXd* _vel);

	void SetVelocity(Eigen::VectorXd* _vel);

	void GetMass(std::vector<T>* massTripletVector);

	void GetMassInv(std::vector<T>* massTripletVector);

	void GetForce(Eigen::VectorXd* force, std::vector<Collider*> colliders);

	void GetForceJacobian(std::vector<T>* derivPos, std::vector<T>* derivVel, std::vector<Collider*> colliders);

	void SetMass(double _density);

	void SetDamping(float damping);

};

