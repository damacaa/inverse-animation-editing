#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <string>
#include <vector>

#include "PhysicsManager.h"

struct ForwardResult {
public:
	Eigen::VectorXd x, v;
};

struct BackwardResult {
public:
	Eigen::VectorXd dGdp;//Tantos como parametros haya
	Eigen::VectorXd dGdx, dGdv;
};

namespace {
	std::string _info;
}

ForwardResult Initialize(std::string info, std::string settings)
{
	_info = info;
	PhysicsManager physicsManager = PhysicsManager(_info);
	physicsManager.Start();

	auto sinfo = physicsManager.GetInitialState();

	ForwardResult result;
	result.x = sinfo.x;
	result.v = sinfo.v;

	return result;
}

ForwardResult Forward(Eigen::VectorXd x, Eigen::VectorXd v, Eigen::VectorXd parameters, std::string settings, float h, int forwardSubSteps)
{
	PhysicsManager physicsManager = PhysicsManager(_info);
	physicsManager.Start();
	physicsManager.SetParam(parameters, settings);

	auto info = physicsManager.Forward(x, v, h, forwardSubSteps);

	ForwardResult result;
	result.x = info.x;
	result.v = info.v;

	return result;
}

std::vector<ForwardResult> ForwardLoop(Eigen::VectorXd x, Eigen::VectorXd v, Eigen::VectorXd parameters, std::string settings, float h, int iter, int forwardSubSteps)
{
	PhysicsManager physicsManager = PhysicsManager(_info);
	physicsManager.Start();
	physicsManager.SetParam(parameters, settings);

	std::vector<ForwardResult> result = std::vector<ForwardResult>(iter);

	PhysicsManager::SimulationInfo info = PhysicsManager::SimulationInfo(x, v);
	for (size_t i = 0; i < iter; i++)
	{
		info = physicsManager.Forward(info.x, info.v, h, forwardSubSteps);

		result[i].x = info.x;
		result[i].v = info.v;
	}


	return result;
}

BackwardResult Backward(Eigen::VectorXd x, Eigen::VectorXd v, Eigen::VectorXd x1, Eigen::VectorXd v1, Eigen::VectorXd parameters, std::string settings, Eigen::VectorXd dGdx1, Eigen::VectorXd dGdv1, float h)
{
	PhysicsManager physicsManager = PhysicsManager(_info);
	physicsManager.Start();
	physicsManager.SetParam(parameters, settings);

	auto info = physicsManager.Backward(x, v, x1, v1, dGdx1, dGdv1, h, settings);

	BackwardResult result;
	result.dGdp = info.dGdp;
	result.dGdx = info.dGdx;
	result.dGdv = info.dGdv;

	return result;
}

PYBIND11_MODULE(DifferentiablePhysicsEngine, m) {
	m.doc() = "Differentiable physics engine for inverse animation editing";

	m.def("initialize", &Initialize, "Initializes simulator");
	m.def("forward", &Forward, "Step forward");
	m.def("forwardLoop", &ForwardLoop, "Take n steps forward");
	m.def("backward", &Backward, "Step backwards");

	pybind11::class_<ForwardResult>(m, "ForwardResult")
		.def_readwrite("x", &ForwardResult::x)
		.def_readwrite("v", &ForwardResult::v);

	pybind11::class_<BackwardResult>(m, "BackwardsResult")
		.def_readwrite("dGdp", &BackwardResult::dGdp)
		.def_readwrite("dGdx", &BackwardResult::dGdx)
		.def_readwrite("dGdv", &BackwardResult::dGdv);
}
