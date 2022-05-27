#include "pch.h"
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

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
std::string _settings = "nG";

ForwardResult Initialize(std::string info, std::string settings)
{
	_info = info;
	_settings = settings;
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

ForwardResult Forward(Eigen::VectorXd x, Eigen::VectorXd v, Eigen::VectorXd parameters, float h)
{
	PhysicsManager physicsManager = PhysicsManager(_info);
	physicsManager.Start();
	physicsManager.SetParam(parameters, _settings);

	auto info = physicsManager.Forward(x, v, h);

	ForwardResult result;
	result.x = info.x;
	result.v = info.v;

	return result;
}

BackwardResult Backward(Eigen::VectorXd x, Eigen::VectorXd v, Eigen::VectorXd x1, Eigen::VectorXd v1, Eigen::VectorXd parameters, Eigen::VectorXd dGdx1, Eigen::VectorXd dGdv1, float h)
{
	PhysicsManager physicsManager = PhysicsManager(_info);
	physicsManager.Start();
	physicsManager.SetParam(parameters, _settings);

	auto info = physicsManager.Backward(x, v, x1, v1, dGdx1, dGdv1, h, _settings);

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



PYBIND11_MODULE(UnityDLL, m) {
	m.doc() = "pybind11 example plugin"; // optional module docstring

	m.def("add", &add, "A function that adds two numbers");

	m.def("initialize", &Initialize, "Initializes simulator");
	m.def("forward", &Forward, "Step forward");
	m.def("backward", &Backward, "Step backwards");

	m.def("estimate", &Estimate, "Estimate for debug purposes");


	pybind11::class_<ForwardResult>(m, "ForwardResult")
		.def_readwrite("x", &ForwardResult::x)
		.def_readwrite("v", &ForwardResult::v);

	pybind11::class_<BackwardResult>(m, "BackwardsResult")
		.def_readwrite("dGdp", &BackwardResult::dGdp)
		.def_readwrite("dGdx", &BackwardResult::dGdx)
		.def_readwrite("dGdv", &BackwardResult::dGdv);
}