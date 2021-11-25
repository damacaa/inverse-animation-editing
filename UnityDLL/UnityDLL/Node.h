#pragma once
#include "Types.h"
#include <math.h>   
#include <Eigen/Core>
#include <Eigen/Dense>


class Node
{
private:
	void UpdateSymplectic(float h);
	void ComputeForces();

public:
	Node() : locked(false) {};

	Eigen::Vector3f position = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
	Eigen::Vector3f vel = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
	Eigen::Vector3f acc = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
	Eigen::Vector3f force = Eigen::Vector3f(0.0f, 0.0f, 0.0f);

	float penalty = 0;
	float thickness = 0;
	float damping = 1;
	float gravity = -9.81f;

	float volume = 1;
	float density = 1;

	bool locked;

	std::string id = "";

	void Update(float time, float h);
};

