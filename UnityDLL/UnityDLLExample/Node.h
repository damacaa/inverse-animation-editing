#pragma once
#include "Types.h"
#include <math.h>   
#include <Eigen/Core>


class Node
{
private:
	void UpdateSymplectic(float h);
	void ComputeForces();

public:
	Node() : position(Vector3f()), locked(false) {};
	Node(Vector3f _position) : position(_position), locked(false) {}
	Node(Vector3f _position, bool _locked) : position(_position), locked(_locked) {}

	Eigen::Vector3d position;
	Eigen::Vector3d vel;
	Eigen::Vector3d acc;
	Eigen::Vector3d force;

    float penalty;
    float thickness;
    float damping;
    float gravity;

    float volume = 0;
    float density;


	bool locked;

	void Update(float time, float h);
};

