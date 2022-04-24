#include "pch.h"
#include "PhysicsManager.h"
#include "Object.h"

#include <iostream>
#include <fstream>  
#include <chrono>

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

PhysicsManager::PhysicsManager(Integration _IntegrationMethod)
{
	integrationMethod = _IntegrationMethod;
	SimObjects = std::vector<Object*>();
}

PhysicsManager::~PhysicsManager()
{
	if (SimObjects.size() > 0) {
		std::string description = std::to_string(SimObjects[0]->nVertices) + " vertices and ";
		description += std::to_string(SimObjects[0]->nSprings) + " springs.";
		debugHelper.PrintTimes("SimulationTimes", description);
	}

	for (int i = 0; i < SimObjects.size(); i++)
	{
		delete SimObjects[i];
	}
	SimObjects.clear();

	for (int i = 0; i < Fixers.size(); i++)
	{
		delete Fixers[i];
	}
	Fixers.clear();

	for (size_t i = 0; i < PendingSimObjects.size(); i++)
	{
		delete PendingSimObjects[i];
	}
	PendingSimObjects.clear();

	for (size_t i = 0; i < PendingFixers.size(); i++)
	{
		delete PendingFixers[i];
	}
	PendingFixers.clear();
}

int PhysicsManager::AddObject(Vector3f position, Vector3f* vertices, int nVertices, int* triangles, int nTriangles, float stiffness, float mass)
{
	Object* o = new Object(position, vertices, nVertices, triangles, nTriangles, stiffness, mass);
	o->id = (int)SimObjects.size() + (int)PendingSimObjects.size();
	PendingSimObjects.push_back(o);

	needsRestart = true;
	return o->id;
}

void PhysicsManager::AddFixer(Vector3f position, Vector3f scale)
{
	Fixer* f = new Fixer(position, scale);
	PendingFixers.push_back(f);

	needsRestart = true;
}

void PhysicsManager::Start()
{
	debugHelper = DebugHelper();

	for (size_t i = 0; i < PendingSimObjects.size(); i++)
	{
		SimObjects.push_back(PendingSimObjects[i]);
		for (size_t j = 0; j < Fixers.size(); j++)
		{
			PendingSimObjects[i]->FixnodeArray(Fixers[j]);
		}
	}
	PendingSimObjects.clear();

	for (size_t i = 0; i < PendingFixers.size(); i++)
	{
		Fixers.push_back(PendingFixers[i]);
		for (size_t j = 0; j < SimObjects.size(); j++)
		{
			SimObjects[j]->FixnodeArray(PendingFixers[i]);
		}
	}
	PendingFixers.clear();

	//Parse the simulable objects and initialize their state indices
	m_numDoFs = 0;

	for (size_t i = 0; i < SimObjects.size(); i++)
	{
		// Initialize simulable object
		SimObjects[i]->Initialize(&m_numDoFs);

		// Retrieve pos and vel size
		m_numDoFs += SimObjects[i]->GetNumDoFs();
	}

	Eigen::VectorXd x = Eigen::VectorXd(m_numDoFs);
	Eigen::VectorXd v = Eigen::VectorXd(m_numDoFs);
	for (size_t i = 0; i < SimObjects.size(); i++)
	{
		SimObjects[i]->GetPosition(&x);
		SimObjects[i]->GetVelocity(&v);
	}

	info = SimulationInfo();
	info.x = x;
	info.v = v;

	needsRestart = false;
	initialized = false;
}

void PhysicsManager::UpdatePhysics(float time, float h)
{
	if (Paused)
		return;

	if (needsRestart)
		Start();

	SimulationInfo newSimulationInfo;
	// Select integration method
	switch (integrationMethod)
	{
	case Integration::Explicit:
		break;
	case Integration::Symplectic:
		newSimulationInfo = StepSymplectic(h, info);
		break;
	case Integration::Implicit:
		//Forward
		newSimulationInfo = StepImplicit(h, info);
		break;
	default:
		break;
	}

	UpdateObjects();

	info = newSimulationInfo;
}

PhysicsManager::SimulationInfo PhysicsManager::StepSymplectic(float h, SimulationInfo simulationInfo)
{
	debugHelper.RecordTime("1.Set up");
	Eigen::VectorXd x = simulationInfo.x;
	Eigen::VectorXd v = simulationInfo.v;
	Eigen::VectorXd f = Eigen::VectorXd::Constant(m_numDoFs, 0.0);

	SpMat M(m_numDoFs, m_numDoFs);
	SpMat Minv(m_numDoFs, m_numDoFs);

	std::vector<T> masses = std::vector<T>();
	std::vector<T> massesInv = std::vector<T>();

	for (int i = 0; i < SimObjects.size(); i++)
	{
		SimObjects[i]->SetPosition(&x);
		SimObjects[i]->SetVelocity(&v);

		SimObjects[i]->GetMassInverse(&massesInv);
		SimObjects[i]->GetMass(&masses);
	}

	debugHelper.RecordTime("2.Forces");
	for (int i = 0; i < SimObjects.size(); i++)
		SimObjects[i]->GetForce(&f);

	debugHelper.RecordTime("3.Fixing");
	std::vector<bool> fixedIndices(m_numDoFs); // m_numDoFs/3?
	for (int i = 0; i < SimObjects.size(); i++)
	{
		SimObjects[i]->GetFixedIndices(&fixedIndices);
	}

	for (size_t i = 0; i < m_numDoFs; i++)
	{
		if (fixedIndices[i]) {
			massesInv[i] = T(i, i, 0);
		}
	}

	//Matriz a triplets, fixing y luego vuelve

	debugHelper.RecordTime("4.Calculations");
	Minv.setFromTriplets(massesInv.begin(), massesInv.end());
	v += h * (Minv * f);
	x += h * v;

	debugHelper.Wait();

	SimulationInfo newSimulationInfo;
	newSimulationInfo.x = x;
	newSimulationInfo.v = v;
	newSimulationInfo.parameter = simulationInfo.parameter;

	return newSimulationInfo;
}

PhysicsManager::SimulationInfo PhysicsManager::StepImplicit(float h, SimulationInfo simulationInfo)
{
	debugHelper.RecordTime("1.Set up");
	Eigen::VectorXd x = simulationInfo.x;
	Eigen::VectorXd v = simulationInfo.v;
	Eigen::VectorXd f = Eigen::VectorXd::Constant(m_numDoFs, 0.0);//Forces

	SpMat dFdx(m_numDoFs, m_numDoFs);
	SpMat dFdv(m_numDoFs, m_numDoFs);
	std::vector<T> derivPos = std::vector<T>();
	std::vector<T> derivVel = std::vector<T>();

	SpMat M(m_numDoFs, m_numDoFs);
	SpMat Mi(m_numDoFs, m_numDoFs);
	std::vector<T> masses = std::vector<T>();
	std::vector<T> massesi = std::vector<T>();

	std::vector<bool> fixedIndices(m_numDoFs);

	for (int i = 0; i < SimObjects.size(); i++)
	{
		SimObjects[i]->SetPosition(&x);
		SimObjects[i]->SetVelocity(&v);

		SimObjects[i]->GetMass(&masses);
		//SimObjects[i]->GetMassInverse(&massesi);
		SimObjects[i]->GetFixedIndices(&fixedIndices);
	}

	//FORCES
	debugHelper.RecordTime("2.Calculating forces");
	for (int i = 0; i < SimObjects.size(); i++)
	{
		SimObjects[i]->GetForce(&f);
		SimObjects[i]->GetForceJacobian(&derivPos, &derivVel);
	}

	debugHelper.RecordTime("3.Building matrices from triples");
	//For future reference maybe
	//https://stackoverflow.com/questions/45301305/set-sparsity-pattern-of-eigensparsematrix-without-memory-overhead
	dFdx.setFromTriplets(derivPos.begin(), derivPos.end(), [](const double& a, const double& b) { return a + b; });
	dFdv.setFromTriplets(derivVel.begin(), derivVel.end(), [](const double& a, const double& b) { return a + b; });
	M.setFromTriplets(masses.begin(), masses.end());
	//Mi.setFromTriplets(massesi.begin(), massesi.end());

	//MATRIX OPERATIONS
	debugHelper.RecordTime("4.Calculating A and b");

	SpMat firstPart = M + (-h) * dFdv;

	SpMat A = firstPart + (-h * h) * dFdx;
	Eigen::VectorXd b = firstPart * v + h * f;

	//FIXING
	debugHelper.RecordTime("5.Fixing");
	std::vector<T> tripletsA;
	for (int i = 0; i < A.outerSize(); i++)
		for (typename Eigen::SparseMatrix<double>::InnerIterator it(A, i); it; ++it) {
			if (!fixedIndices[it.row()] && !fixedIndices[it.col()])
				tripletsA.push_back(T(it.row(), it.col(), it.value())); //tripletsA.emplace_back(it.row(), it.col(), it.value());
			else if (it.row() == it.col())
				tripletsA.push_back(T(it.row(), it.col(), 1));//tripletsA.emplace_back(it.row(), it.col(), 1);
		}
	A.setFromTriplets(tripletsA.begin(), tripletsA.end());

	for (size_t i = 0; i < m_numDoFs; i++)
	{
		if (fixedIndices[i]) {
			b[i] = 0;
		}
	}

	//SOLVING
	debugHelper.RecordTime("6.Solving");
	//A.Solve(b, v);
	//Eigen::SimplicialCholesky<SpMat> chol(A);
	//v = chol.solve(b);


	Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::UnitLower | Eigen::UnitUpper> cg;
	cg.compute(A);

	if (initialized) {
		//v = v + h * (f * Mi);
		v = cg.solveWithGuess(b, v);
	}
	else {
		v = cg.solve(b);
		initialized = true;
	}

	x += h * v;

	debugHelper.Wait();

	SimulationInfo newData;
	newData.x = x;
	newData.v = v;
	newData.M = M;
	newData.dFdx = dFdx;
	newData.dFdv = dFdv;

	newData.parameter = simulationInfo.parameter;

	return newData;
}

void PhysicsManager::UpdateObjects()
{
	for (int i = 0; i < SimObjects.size(); i++)
	{
		SimObjects[i]->SetPosition(&info.x);
		SimObjects[i]->SetVelocity(&info.v);
	}
}

float PhysicsManager::Estimate(float parameter, int iter, float h)
{
	//Simulation to define target
	Start();
	SimulationInfo target = info;

	for (size_t i = 0; i < SimObjects.size(); i++)
	{
		SimObjects[i]->SetNodeMass(0.5f);
	}

	for (size_t i = 0; i < iter; i++)
	{
		target = StepImplicit(h, target);
		for (int j = 0; j < SimObjects.size(); j++)
		{
			SimObjects[j]->SetPosition(&target.x);
			SimObjects[j]->SetVelocity(&target.v);
		}
	}


	//Forward
	SimulationInfo newInfo = info;

	//Simulation with given parameter
	for (size_t i = 0; i < SimObjects.size(); i++)
	{
		SimObjects[i]->SetNodeMass(parameter);
	}

	for (int i = 0; i < SimObjects.size(); i++)
	{
		SimObjects[i]->SetPosition(&newInfo.x);
		SimObjects[i]->SetVelocity(&newInfo.v);
	}

	std::vector<SimulationInfo> steps(iter);

	for (size_t i = 0; i < iter; i++)
	{
		//Guardar en simulation info lo mínimo v y x y luego recalcular derivadas
		newInfo = StepImplicit(h, newInfo);
		steps[i] = newInfo;
		for (int j = 0; j < SimObjects.size(); j++)
		{
			SimObjects[j]->SetPosition(&newInfo.x);
			SimObjects[j]->SetVelocity(&newInfo.v);
		}
	}

	//Evaluate g

	//g = sum_i |xi - xi*|^2 //Caso genérico n frames
		//g = |xn - xn*|^2 //Caso frame final

		//g = sum_i (xi - xi*)T (xi - xi*)
		//dg / dxi = 2 (xi - xi*)T

		//dg/dxi = 0, si i != n;  dg/dxn = 2 (xn - xn*)T

	float g = 0;
	for (size_t i = 0; i < info.x.size(); i++)
	{
		g += abs(target.x(i) - steps[iter - 1].x(i));
	}

	//return g * g;

	Eigen::VectorXd dGdp = Eigen::VectorXd::Constant(1, 0.0);//Tantos como parametros haya
	std::vector<Eigen::VectorXd> dGdx(iter);
	std::vector<Eigen::VectorXd> dGdv(iter);

	for (size_t i = 0; i < iter; i++)
	{
		//dGdx poner a 0 todos menos ultimo
		//en ultimo ver mates de arriba
		if (i == iter - 1) {
			dGdx[i] = 2.0 * (target.x - newInfo.x);//?????????
		}
		else
			dGdx[i] = Eigen::VectorXd::Constant(m_numDoFs, 0.0);

		dGdv[i] = Eigen::VectorXd::Constant(m_numDoFs, 0.0);
	}

	for (size_t i = iter - 2; i > 0; i--)
	{
		BackwardStepInfo newInfo = Backwards(steps[i].x, steps[i].v, parameter, dGdx[i + 1], dGdv[i + 1], h, steps[i]);//dGdp, dGdx, dGdv

		//Global
		dGdp += newInfo.dGdp;

		dGdx[i] += newInfo.dGdx;

		dGdv[i] += newInfo.dGdv;
	}

	return g * g;
}

PhysicsManager::BackwardStepInfo PhysicsManager::Backwards(Eigen::VectorXd x1, Eigen::VectorXd v1, float parameter, Eigen::VectorXd dGdx1, Eigen::VectorXd dGdv1, float h, SimulationInfo previous) {
	Eigen::VectorXd dGdp, dGdx, dGdv;//Results

	//SpMat M(m_numDoFs, m_numDoFs);
	SpMat M = previous.M;

	Eigen::VectorXd f1 = Eigen::VectorXd::Constant(m_numDoFs, 0.0);//Forces

	Eigen::VectorXd x = previous.x;
	Eigen::VectorXd v = previous.v;
	SpMat dFdx = previous.dFdx;
	SpMat dFdv = previous.dFdv;

	std::vector<T> derivPos = std::vector<T>();
	std::vector<T> derivVel = std::vector<T>();
	std::vector<T> masses = std::vector<T>();

	//FORCES
	/*for (int i = 0; i < SimObjects.size(); i++)
	{
		SimObjects[i]->SetPosition(&x1);
		SimObjects[i]->SetVelocity(&v1);
		SimObjects[i]->GetMass(&masses);
		SimObjects[i]->GetForce(&f1);
		SimObjects[i]->GetForceJacobian(&derivPos, &derivVel);
	}

	dFdx.setFromTriplets(derivPos.begin(), derivPos.end(), [](const double& a, const double& b) { return a + b; });
	dFdv.setFromTriplets(derivVel.begin(), derivVel.end(), [](const double& a, const double& b) { return a + b; });
	M.setFromTriplets(masses.begin(), masses.end());

	/*SpMat firstPart = M + (-h) * dFdv;

	SpMat Aa = firstPart + (-h * h) * dFdx;
	Eigen::VectorXd bb = firstPart * v1 + h * f1;

	Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::UnitLower | Eigen::UnitUpper> cgg;
	cgg.compute(Aa);

	v = cgg.solveWithGuess(bb, v1);*/



	//Backward
	BackwardStepInfo info = BackwardStepInfo();

	SpMat A = M - (h * dFdv) + ((-h * h) * dFdx);
	Eigen::VectorXd b = (h * dGdx1) + dGdv1;

	Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::UnitLower | Eigen::UnitUpper> cg;
	cg.compute(A);

	Eigen::VectorXd u = cg.solve(b);


	//dGdp: Vector de tantas posiciones como parámetros haya, en este caso 1

	//Eigen::VectorXd c = M * v1 - M * v - h * f1;
	//Eigen::VectorXd dcdp = v1 - v;
	//-u * dcdp.transpose() --> Matriz

	double dcdp = 0;
	for (size_t i = 0; i < m_numDoFs; i++)
	{
		dcdp += v1(i) - v(i);
	}

	//info.dGdp = -u * dcdp;//????????????
	info.dGdp = Eigen::VectorXd::Constant(1, dcdp);//Tantos como parametros haya

	//dGdx

	/*Eigen::VectorXd uT = u.transpose();

	int lhcol = u.cols();
	int lhrow = u.rows();
	int lhcolT = uT.cols();
	int rhrow = dFdx.rows();*/

	//auto a = dFdx * u;
	//auto aa = uT * dFdx;

	//info.dGdx = dGdx1 + (h * u * dFdx);
	info.dGdx = dGdx1 + h * (dFdx * u);

	//dGdv
	info.dGdv = M * u;

	return info;
}

void PhysicsManager::UpdateVertices() {
	for (size_t i = 0; i < SimObjects.size(); i++)
	{
		SimObjects[i]->UpdateVertices();
	}

	Updated = true;
}

Vector3f* PhysicsManager::GetVertices(int* count)
{
	if (!Updated) {
		*count = 0;
		return new Vector3f;
	}

	int totalCount = 0;
	for (size_t i = 0; i < SimObjects.size(); i++)
	{
		totalCount += SimObjects[i]->nVertices;
	}

	Vector3f* result = new Vector3f[totalCount];

	totalCount = 0;
	for (size_t i = 0; i < SimObjects.size(); i++)
	{
		int localCount = SimObjects[i]->nVertices;

		Vector3f* v = SimObjects[i]->GetVertices();
		std::copy(v, v + localCount, result + totalCount);

		totalCount += localCount;
	}

	Updated = false;
	*count = totalCount;
	return result;
}

Vector3f* PhysicsManager::GetVertices(int id, int* count)
{
	if (id >= SimObjects.size() || !SimObjects[id]->updated) {
		*count = 0;
		return new Vector3f;
	}

	Updated = false;
	*count = SimObjects[id]->nVertices;
	return SimObjects[id]->GetVertices();
}