#include "pch.h"
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <vector>

#include <PhysicsManager.h>

struct ForwardResult {
public:
	Eigen::VectorXd x, v;
};

struct BackwardResult {
public:
	Eigen::VectorXd dGdp;//Tantos como parametros haya
	Eigen::VectorXd dGdx, dGdv;
};

std::string _info;

ForwardResult Initialize(std::string info, std::string settings)
{
	_info = info;
	PhysicsManager physicsManager = PhysicsManager(_info);
	physicsManager.Start();

	//physicsManager = physicsManager2;

	auto sinfo = physicsManager.GetInitialState();

	ForwardResult result;
	result.x = sinfo.x;
	result.v = sinfo.v;

	return result;
}

/*ForwardResult Forward(Eigen::VectorXd x, Eigen::VectorXd v, float parameter, float h)
{
	PhysicsManager physicsManager = PhysicsManager(_info);
	physicsManager.Start();
	physicsManager.SetParam(parameter);

	auto info = physicsManager.Forward(x, v, h);

	ForwardResult result;
	result.x = info.x;
	result.v = info.v;

	return result;
}*/

ForwardResult Forward(Eigen::VectorXd x, Eigen::VectorXd v, Eigen::VectorXd parameters, std::string settings, float h)
{
	PhysicsManager physicsManager = PhysicsManager(_info);
	physicsManager.Start();
	physicsManager.SetParam(parameters, settings);

	auto info = physicsManager.Forward(x, v, h);

	ForwardResult result;
	result.x = info.x;
	result.v = info.v;

	return result;
}

std::vector<ForwardResult> ForwardLoop(Eigen::VectorXd x, Eigen::VectorXd v, Eigen::VectorXd parameters, std::string settings, float h, int iter)
{
	PhysicsManager physicsManager = PhysicsManager(_info);
	physicsManager.Start();
	physicsManager.SetParam(parameters, settings);

	std::vector<ForwardResult> result = std::vector<ForwardResult>(iter);

	PhysicsManager::SimulationInfo info = PhysicsManager::SimulationInfo(x, v);
	for (size_t i = 0; i < iter; i++)
	{
		info = physicsManager.Forward(info.x, info.v, h);
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

BackwardResult Estimate(float param, int iter, float h) {
	PhysicsManager physicsManager = PhysicsManager(_info);
	physicsManager.Start();
	Eigen::VectorXd dGdp = Eigen::VectorXd::Constant(1, 0.0);
	float g = physicsManager.Estimate(param, iter, h, &dGdp);

	BackwardResult result;
	result.dGdp = dGdp;
	result.dGdx = Eigen::VectorXd::Constant(1, g);

	return result;
}


int a = 0;
int add(int i, int j) {
	a += i;
	return a;
}

Eigen::VectorXd Test() {
	Eigen::VectorXd a = Eigen::VectorXd::Constant(3, 1);
	Eigen::VectorXd b = Eigen::VectorXd::Constant(2, 2);
	Eigen::VectorXd c = Eigen::VectorXd::Constant(1, 3);

	Eigen::VectorXd A(a.size() + b.size());
	A << a, b;

	Eigen::VectorXd B(A.size() + c.size());
	B << A, c;

	return  B;
}



PYBIND11_MODULE(UnityDLL, m) {
	m.doc() = "pybind11 example plugin"; // optional module docstring

	m.def("add", &add, "A function that adds two numbers");

	m.def("initialize", &Initialize, "Initializes simulator");
	m.def("forward", &Forward, "Step forward");
	m.def("forwardLoop", &ForwardLoop, "Take n steps forward");
	m.def("backward", &Backward, "Step backwards");
	m.def("test", &Test, "Test");

	m.def("estimate", &Estimate, "Estimate for debug purposes");


	pybind11::class_<ForwardResult>(m, "ForwardResult")
		.def_readwrite("x", &ForwardResult::x)
		.def_readwrite("v", &ForwardResult::v);

	pybind11::class_<BackwardResult>(m, "BackwardsResult")
		.def_readwrite("dGdp", &BackwardResult::dGdp)
		.def_readwrite("dGdx", &BackwardResult::dGdx)
		.def_readwrite("dGdv", &BackwardResult::dGdv);
}