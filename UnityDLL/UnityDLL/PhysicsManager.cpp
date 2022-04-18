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
	std::string description = std::to_string(SimObjects[0]->nVertices) + " vertices and ";
	description += std::to_string(SimObjects[0]->nSprings) + " springs.";
	debugHelper.PrintTimes("SimulationTimes", description);

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
			PendingSimObjects[j]->FixnodeArray(PendingFixers[i]);
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
	Updated = false;

	if (Paused)
		return; // Not simulating

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

	Updated = true;
}

float PhysicsManager::Estimate(float parameter, int iter, float h)
{
	//g = sum_i |xi - xi*|^2 //Caso genérico n frames
		//g = |xn - xn*|^2 //Caso frame final

		//g = sum_i (xi - xi*)T (xi - xi*)
		//dg / dxi = 2 (xi - xi*)T

		//dg/dxi = 0, si i != n;  dg/dxn = 2 (xn - xn*)T

	Start();//Initializes simulation

	std::vector<SimulationInfo> steps(iter);

	//Forward
	steps[0] = StepImplicit(h, info);
	for (size_t i = 1; i < iter; i++)
	{
		SimulationInfo newInfo;
		newInfo = StepImplicit(h, steps[i - 1]);
		steps[i] = newInfo;
		info = newInfo;
	}

	//Guardar en simulation info lo mínimo v y x y luego recalcular derivadas

	std::vector<Eigen::VectorXd> dGdx(iter);
	std::vector<Eigen::VectorXd> dGdv(iter);

	for (size_t i = 0; i < iter; i++)
	{
		//dGdx poner a 0 todos menos ultimo
		//en ultimo ver mates de arriba
		if (i == iter - 1) {
			dGdx[i] = Eigen::VectorXd::Constant(1, 0.0);
		}
		else
			dGdx[i] = Eigen::VectorXd::Constant(1, 0.0);

		dGdv[iter] = Eigen::VectorXd::Constant(1, 0.0);
	}

	Eigen::VectorXd dGdp = Eigen::VectorXd::Constant(1, 0.0);//Tantos como parametros haya
	for (size_t i = iter - 2; i >= 0; i--)
	{
		//Backwards(steps[i + 1].x, steps[i + 1].v, parameter, dGdx[i + 1], dGdv[i + 1]);//dGdp, dGdx, dGdv

		//Backward
		SpMat A = info.M - h * steps[i].dFdv + (-h * h) * steps[i].dFdx;
		Eigen::VectorXd b = h * dGdx[i + 1] + dGdv[i + 1];

		Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::UnitLower | Eigen::UnitUpper> cg;
		cg.compute(A);

		Eigen::VectorXd u = cg.solve(b);

		Eigen::VectorXd c = steps[i + 1].M * steps[i + 1].v - steps[i].M * steps[i].v - h * steps[i].f;//Fuerzas o jacobianas??

		Eigen::VectorXd dcdp = steps[i + 1].v - steps[i].v;

		//Local
		Eigen::VectorXd dGdpLocal = -u * dcdp; //Revisar si hace p escalar

		//Global
		dGdp += dGdpLocal;

		dGdx[i] += dGdx[i + 1] + h * u * steps[i].dFdx;

		dGdv[i] += u * info.M;
	}

	return parameter;
}

void PhysicsManager::Backwards(Eigen::VectorXd x1, Eigen::VectorXd v1, float parameter, Eigen::VectorXd dGdx1, Eigen::VectorXd dGdv1) {
	Eigen::VectorXd dGdx, dGdv, dGdp;

	SpMat dFdx(m_numDoFs, m_numDoFs);
	SpMat dFdv(m_numDoFs, m_numDoFs);
	std::vector<T> derivPos = std::vector<T>();
	std::vector<T> derivVel = std::vector<T>();


	//FORCES
	for (int i = 0; i < SimObjects.size(); i++)
	{
		SimObjects[i]->SetPosition(&x1);//x1 or x?????
		SimObjects[i]->SetVelocity(&v1);
		SimObjects[i]->GetForceJacobian(&derivPos, &derivVel);
	}

	debugHelper.RecordTime("3.Building matrices from triples");
	//For future reference maybe
	//https://stackoverflow.com/questions/45301305/set-sparsity-pattern-of-eigensparsematrix-without-memory-overhead
	dFdx.setFromTriplets(derivPos.begin(), derivPos.end(), [](const double& a, const double& b) { return a + b; });
	dFdv.setFromTriplets(derivVel.begin(), derivVel.end(), [](const double& a, const double& b) { return a + b; });

}

Vector3f* PhysicsManager::GetVertices(int id, int* count)
{
	if (id >= SimObjects.size() || !SimObjects[id]->updated)
		return new Vector3f();

	*count = SimObjects[id]->nVertices;

	return SimObjects[id]->GetVertices();
}