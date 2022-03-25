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
	std::string name = std::to_string(SimObjects[0]->nVertices) + " vertices and ";
	name += std::to_string(SimObjects[0]->nSprings) + " springs.";
	debugHelper.PrintTimes("times", name);

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
	o->id = (int)SimObjects.size();
	SimObjects.push_back(o);

	for (size_t i = 0; i < Fixers.size(); i++)
	{
		o->FixnodeArray(Fixers[i]);
	}

	Start();

	return o->id;
}

void PhysicsManager::AddFixer(Vector3f position, Vector3f scale)
{
	Fixer* f = new Fixer(position, scale);
	Fixers.push_back(f);

	for (size_t i = 0; i < SimObjects.size(); i++)
	{
		SimObjects[i]->FixnodeArray(f);
	}
}

void PhysicsManager::Start()
{
	debugHelper = DebugHelper();

	//Parse the simulable objects and initialize their state indices
	m_numDoFs = 0;

	for (size_t i = 0; i < SimObjects.size(); i++)
	{
		// Initialize simulable object
		SimObjects[i]->Initialize(&m_numDoFs);

		// Retrieve pos and vel size
		m_numDoFs += SimObjects[i]->GetNumDoFs();
	}

	Eigen::VectorXd* x = new Eigen::VectorXd(m_numDoFs);
	Eigen::VectorXd* v = new Eigen::VectorXd(m_numDoFs);
	for (size_t i = 0; i < SimObjects.size(); i++)
	{
		SimObjects[i]->GetPosition(x);
		SimObjects[i]->GetVelocity(v);
	}

	_simulationInfo = SimulationInfo();
	_simulationInfo.x = x;
	_simulationInfo.v = v;

	initialized = false;
}

void PhysicsManager::Update(float time, float h)
{
	Updated = false;

	if (Paused)
		return; // Not simulating

	// Select integration method
	switch (integrationMethod)
	{
	case Integration::Explicit:
		break;
	case Integration::Symplectic:
		StepSymplectic(time, h);
		break;
	case Integration::Implicit:
		_simulationInfo = StepImplicit(time, h, _simulationInfo);
		break;
	default:
		break;
	}

	Updated = true;
}

void PhysicsManager::StepSymplecticOld(float time, float h) {
	for (int i = 0; i < SimObjects.size(); i++)
	{
		SimObjects[i]->Update(time, h);
	}
}

void PhysicsManager::StepSymplectic(float time, float h)
{
	Eigen::VectorXd x = Eigen::VectorXd(m_numDoFs);
	Eigen::VectorXd v = Eigen::VectorXd(m_numDoFs);
	Eigen::VectorXd f = Eigen::VectorXd::Constant(m_numDoFs, 0.0);

	SpMat M(m_numDoFs, m_numDoFs);
	SpMat Minv(m_numDoFs, m_numDoFs);

	SpMat dFdx(m_numDoFs, m_numDoFs);
	SpMat dFdv(m_numDoFs, m_numDoFs);

	std::vector<T> masses = std::vector<T>();
	std::vector<T> massesInv = std::vector<T>();
	std::vector<T> derivPos = std::vector<T>();
	std::vector<T> derivVel = std::vector<T>();

	for (int i = 0; i < SimObjects.size(); i++)
	{
		SimObjects[i]->GetPosition(&x);
		SimObjects[i]->GetVelocity(&v);
		SimObjects[i]->GetForce(&f);

		SimObjects[i]->GetForceJacobian(&derivPos, &derivVel);
		SimObjects[i]->GetMassInverse(&massesInv);
		SimObjects[i]->GetMass(&masses);
	}

	std::vector<bool> fixedIndices(m_numDoFs); // m_numDoFs/3?
	for (int i = 0; i < SimObjects.size(); i++)
	{
		SimObjects[i]->GetFixedIndices(&fixedIndices);
	}

	for (size_t i = 0; i < massesInv.size(); i += 3)
	{
		int id = massesInv[i].col() / 3;
		if (fixedIndices[id]) {
			massesInv[i] = T(massesInv[i].col(), massesInv[i].row(), massesInv[i].value());
		}
	}

	//Matriz a triplets, fixing y luego vuelve

	Minv.setFromTriplets(massesInv.begin(), massesInv.end());
	v += h * (Minv * f);

	for (size_t i = 0; i < fixedIndices.size(); i += 3)
	{
		if (fixedIndices[i]) {
			v[i] = 0;
			v[i + 1] = 0;
			v[i + 2] = 0;
		}
	}

	x += h * v;

	for (int i = 0; i < SimObjects.size(); i++)
	{
		SimObjects[i]->SetPosition(&x);
		SimObjects[i]->SetVelocity(&v);
	}
}

PhysicsManager::SimulationInfo PhysicsManager::StepImplicit(float time, float h, SimulationInfo simulationInfo)
{
	debugHelper.RecordTime("1.Set up");

	Eigen::VectorXd f = Eigen::VectorXd::Constant(m_numDoFs, 0.0);//Forces

	SpMat dFdx(m_numDoFs, m_numDoFs);
	SpMat dFdv(m_numDoFs, m_numDoFs);
	std::vector<T> derivPos = std::vector<T>();
	std::vector<T> derivVel = std::vector<T>();

	SpMat M(m_numDoFs, m_numDoFs);
	SpMat Minv(m_numDoFs, m_numDoFs);
	std::vector<T> masses = std::vector<T>();
	std::vector<T> massesInv = std::vector<T>();

	for (int i = 0; i < SimObjects.size(); i++)
	{
		SimObjects[i]->GetMass(&masses);
		SimObjects[i]->GetMassInverse(&massesInv);
	}

	std::vector<bool> fixedIndices(m_numDoFs);
	for (int i = 0; i < SimObjects.size(); i++)
		SimObjects[i]->GetFixedIndices(&fixedIndices);

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

	//MATRIX OPERATIONS
	debugHelper.RecordTime("4.Calculating A and b");
	SpMat A = M - h * dFdv - h * h * dFdx;
	Eigen::VectorXd b = (M - h * dFdv) * (*simulationInfo.v) + h * f;

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
		//Minv.setFromTriplets(massesInv.begin(), massesInv.end());
		//_v = _v + h * (Minv * f);
		*simulationInfo.v = cg.solveWithGuess(b, *simulationInfo.v);
	}
	else {
		*simulationInfo.v = cg.solve(b);
		initialized = true;
	}

	(*simulationInfo.x) += h * (*simulationInfo.v);

	for (int i = 0; i < SimObjects.size(); i++)
	{
		SimObjects[i]->SetPosition(simulationInfo.x);
		SimObjects[i]->SetVelocity(simulationInfo.v);
	}

	debugHelper.Wait();

	return simulationInfo;
}

Vector3f* PhysicsManager::GetVertices(int id, int* count)
{
	if (id >= SimObjects.size())
		return new Vector3f();

	*count = SimObjects[id]->nVertices;

	return SimObjects[id]->GetVertices();
}