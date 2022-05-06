#include "pch.h"
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <PhysicsManager.h>

struct ForwardResult {
public:
	Eigen::VectorXd x, v;
};

struct BackwardsResult {
public:
	Eigen::VectorXd dGdp;//Tantos como parametros haya
	Eigen::VectorXd dGdx, dGdv;
};

PhysicsManager physicsManager;

void Initialize(std::string info) {
	physicsManager = PhysicsManager();
}

ForwardResult Forward(Eigen::VectorXd x, Eigen::VectorXd v, float parameter, float h)
{
	physicsManager.SetParam(parameter);

	auto info = physicsManager.Forward(x, v, h);

	ForwardResult result;
	result.x = x;
	result.v = v;

	return result;
}

BackwardsResult Bacwards(Eigen::VectorXd x, Eigen::VectorXd v, Eigen::VectorXd x1, Eigen::VectorXd v1, float parameter, Eigen::VectorXd dGdx1, Eigen::VectorXd dGdv1, float h)
{
	physicsManager.SetParam(parameter);

	auto info = physicsManager.Backwards(x, v, x1, v1, parameter, dGdx1, dGdv1, h);

	BackwardsResult result;
	result.dGdp = info.dGdp;
	result.dGdx = info.dGdx;
	result.dGdv = info.dGdv;

	return result;
}


int a = 0;
int add(int i, int j) {
	a += i;
	return a;
}

BackwardsResult Estimate(float parameter, int iter, float h) {
	PhysicsManager physicsManager = PhysicsManager(Integration::Implicit);

	Vector3f* vertices = new Vector3f[3];
	vertices[0] = Vector3f(0, 0, 0);
	vertices[1] = Vector3f(0, -1, 0);
	vertices[2] = Vector3f(1, -1, 0);

	int* triangles = new int[3];
	triangles[0] = 0;
	triangles[1] = 1;
	triangles[2] = 2;

	physicsManager.AddObject(vertices, 3, triangles, 3, 1, 1);

	BackwardsResult r = BackwardsResult();
	r.g = physicsManager.Estimate(parameter, iter, h, &r.dGdp);

	return r;
}

PYBIND11_MODULE(UnityDLL, m) {
	m.doc() = "pybind11 example plugin"; // optional module docstring

	m.def("add", &add, "A function that adds two numbers");

	m.def("estimate", &Estimate, "A function that calculates some error");


	pybind11::class_<ForwardResult>(m, "ForwardResult")
		.def_readwrite("x", &ForwardResult::x)
		.def_readwrite("v", &ForwardResult::v);

	pybind11::class_<BackwardsResult>(m, "BackwardsResult")
		.def_readwrite("g", &BackwardsResult::g)
		.def_readwrite("dGdp", &BackwardsResult::dGdp);
}