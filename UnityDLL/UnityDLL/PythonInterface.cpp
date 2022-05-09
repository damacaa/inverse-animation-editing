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

ForwardResult Initialize(std::string info)
{
	physicsManager = PhysicsManager(info);
	physicsManager.Start();

	auto sinfo = physicsManager.GetInitialState();

	ForwardResult result;
	result.x = sinfo.x;
	result.v = sinfo.v;

	return result;
}

ForwardResult Forward(Eigen::VectorXd x, Eigen::VectorXd v, float parameter, float h)
{
	physicsManager.SetParam(parameter);

	auto info = physicsManager.Forward(x, v, h);

	ForwardResult result;
	result.x = info.x;
	result.v = info.v;

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



PYBIND11_MODULE(UnityDLL, m) {
	m.doc() = "pybind11 example plugin"; // optional module docstring

	m.def("add", &add, "A function that adds two numbers");

	m.def("initialize", &Initialize, "Initializes simulator");
	m.def("forward", &Forward, "Step forward");
	m.def("bacwards", &Bacwards, "Step backwards");

	pybind11::class_<ForwardResult>(m, "ForwardResult")
		.def_readwrite("x", &ForwardResult::x)
		.def_readwrite("v", &ForwardResult::v);

	pybind11::class_<BackwardsResult>(m, "BackwardsResult")
		.def_readwrite("dGdp", &BackwardsResult::dGdp)
		.def_readwrite("dGdx", &BackwardsResult::dGdx)
		.def_readwrite("dGdv", &BackwardsResult::dGdv);
}