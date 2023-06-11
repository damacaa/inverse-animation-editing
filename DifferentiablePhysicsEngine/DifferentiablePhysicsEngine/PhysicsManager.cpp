#include "pch.h"
#include "PhysicsManager.h"
#include "Object.h"
#include "Collider.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

Eigen::Vector3d PhysicsManager::windVelocity = Eigen::Vector3d(0, 0, 0);

PhysicsManager::PhysicsManager(std::string info)
{
	json js = json::parse(info);

	integrationMethod = (Integration)js["integrationMethod"];
	tolerance = js["tolerance"];
	printTimes = js["printTimes"];

	debugHelper = DebugHelper();
	debugHelper.enabled = printTimes;

	json wind = js["windVel"];
	PhysicsManager::windVelocity = Eigen::Vector3d(wind["x"], wind["y"], wind["z"]);

	for (size_t i = 0; i < js["objects"].size(); i++)
	{
		json obj = js["objects"][i];

		int nVerts = obj["vertPos"].size();
		int nSprings = obj["springs"].size() / 2;
		int nTriangles = obj["triangles"].size();

		Vector3f* vertPos = new Vector3f[nVerts];
		bool* vertIsFixed = new bool[nVerts];
		float* vertMass = new float[nVerts];

		int* springs = new int[(size_t)(nSprings * 2.0)];
		float* springStiffness = new float[nSprings];

		int* triangles = new int[(size_t)obj["triangles"].size()];
		double dragCoefficient = obj["dragCoefficient"];

		float damping = obj["damping"];
		std::string optimizationSettings = obj["optimizationSettings"];

		std::string s = "";
		for (size_t j = 0; j < nVerts; j++)
		{
			json pos = obj["vertPos"][j];

			float x = pos["x"];
			float y = pos["y"];
			float z = pos["z"];

			vertPos[j] = Vector3f(x, y, z);
			vertMass[j] = obj["vertMass"][j];
			vertIsFixed[j] = obj["vertIsFixed"][j];
		}

		for (size_t j = 0; j < nSprings; j++)
		{
			springs[2 * j] = obj["springs"][2 * j];
			springs[(2 * j) + 1] = obj["springs"][(2 * j) + 1];
			springStiffness[j] = obj["springStiffness"][j];
		}

		for (size_t j = 0; j < nTriangles; j++)
		{
			triangles[j] = (int)obj["triangles"][j];
		}


		int id = AddObject(vertPos, vertIsFixed, vertMass, nVerts, springs, springStiffness, nSprings, triangles, nTriangles, dragCoefficient, damping, optimizationSettings);
	}

	for (size_t i = 0; i < js["colliders"].size(); i++)
	{
		json obj = js["colliders"][i];
		Vector3f pos = Vector3f(obj["pos"]["x"], obj["pos"]["y"], obj["pos"]["z"]);
		Vector3f rot = Vector3f(obj["rot"]["x"], obj["rot"]["y"], obj["rot"]["z"]);
		Vector3f scale = Vector3f(obj["scale"]["x"], obj["scale"]["y"], obj["scale"]["z"]);
		int id = AddCollider(obj["type"], pos, rot, scale);
	}
}

PhysicsManager::PhysicsManager(Integration _IntegrationMethod, double tolerance)
{
	integrationMethod = _IntegrationMethod;
	this->tolerance = tolerance;
	SimObjects = std::vector<Object*>();
}

PhysicsManager::~PhysicsManager()
{
	if (printTimes) {
		std::string description = std::to_string(nodeCount) + " vertices and ";
		description += std::to_string(springCount) + " springs.";
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

PhysicsManager::PhysicsManager(const PhysicsManager&)
{
}

int PhysicsManager::AddObject(Vector3f* vertPos, bool* vertIsFixed, float* vertMass, int nVerts,
	int* springs, float* springStiffness, int nSprings, int* triangles, int nTriangles,
	double dragCoefficient, float damping, std::string optimizationSettings)
{
	std::lock_guard<std::mutex> lock(vertexMutex);

	Object* o = new Object(vertPos, vertIsFixed, vertMass, nVerts,
		springs, springStiffness, nSprings, triangles, nTriangles,
		dragCoefficient, damping, optimizationSettings, this);

	o->id = (int)SimObjects.size() + (int)PendingSimObjects.size();
	SimObjects.push_back(o);

	nodeCount += nVerts;
	springCount += nSprings;

	needsRestart = true;
	return o->id;
}

int PhysicsManager::AddCollider(int type, Vector3f pos, Vector3f rot, Vector3f scale)
{
	Eigen::Vector3d _pos = Eigen::Vector3d(pos.x, pos.y, pos.z);
	Eigen::Vector3d _rot = Eigen::Vector3d(rot.x, rot.y, rot.z);
	Eigen::Vector3d _scale = Eigen::Vector3d(scale.x, scale.y, scale.z);

	Colliders.push_back(new Collider(type, _pos, _rot, _scale));

	return Colliders.size() - 1;
}

void PhysicsManager::AddFixer(Vector3f position, Vector3f scale)
{
	std::lock_guard<std::mutex> lock(vertexMutex);

	Fixer* f = new Fixer(position, scale);
	Fixers.push_back(f);

	needsRestart = true;
}

void PhysicsManager::Start()
{
	std::lock_guard<std::mutex> vertexLock(vertexMutex);
	std::lock_guard<std::mutex> objectLock(sceneObjectsMutex);

	for (size_t i = 0; i < PendingSimObjects.size(); i++)
	{
		SimObjects.push_back(PendingSimObjects[i]);
		for (size_t j = 0; j < Fixers.size(); j++)
		{
			PendingSimObjects[i]->Fixnodes(Fixers[j]);
		}
	}
	PendingSimObjects.clear();

	for (size_t i = 0; i < PendingFixers.size(); i++)
	{
		Fixers.push_back(PendingFixers[i]);
		for (size_t j = 0; j < SimObjects.size(); j++)
		{
			SimObjects[j]->Fixnodes(PendingFixers[i]);
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
		// Fill position and velocity vectors
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
		newSimulationInfo = StepExplicit(h, info);
		break;
	case Integration::Symplectic:
		newSimulationInfo = StepSymplectic(h, info);
		break;
	case Integration::Implicit:
		newSimulationInfo = StepImplicit(h, info);
		break;
	default:
		break;
	}

	std::lock_guard<std::mutex> lock(vertexMutex);

	for (int i = 0; i < SimObjects.size(); i++)
	{
		SimObjects[i]->SetPosition(&newSimulationInfo.x);
		SimObjects[i]->UpdateVertices();
		SimObjects[i]->SetVelocity(&newSimulationInfo.v);
	}

	info = newSimulationInfo;
	Updated = true;
}

PhysicsManager::SimulationInfo PhysicsManager::StepExplicit(float h, SimulationInfo simulationInfo)
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
		SimObjects[i]->GetForce(&f, Colliders);

	debugHelper.RecordTime("3.Fixing");
	//Matrix to triplets, fixing and then rebuild matrix
	std::vector<bool> fixedIndices(m_numDoFs);
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
	Minv.setFromTriplets(massesInv.begin(), massesInv.end());

	debugHelper.RecordTime("4.Calculations");
	x += h * info.v;
	v += h * (Minv * f);

	debugHelper.Wait();

	SimulationInfo newSimulationInfo;
	newSimulationInfo.x = x;
	newSimulationInfo.v = v;

	return newSimulationInfo;
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
		SimObjects[i]->GetForce(&f, Colliders);

	debugHelper.RecordTime("3.Fixing");
	//Matrix to triplets, fixing and then rebuild matrix
	std::vector<bool> fixedIndices(m_numDoFs);
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
	Minv.setFromTriplets(massesInv.begin(), massesInv.end());

	debugHelper.RecordTime("4.Calculations");
	v += h * (Minv * f);
	x += h * v;

	debugHelper.Wait();

	SimulationInfo newSimulationInfo;
	newSimulationInfo.x = x;
	newSimulationInfo.v = v;

	return newSimulationInfo;
}

PhysicsManager::SimulationInfo PhysicsManager::StepImplicit(float h, SimulationInfo previousState, int iterations)
{
	debugHelper.RecordTime("1.Set up");
	
	Eigen::VectorXd x = previousState.x;// Node positions
	Eigen::VectorXd v = previousState.v;// Node velocities

	for (int iter = 0; iter < iterations; iter++)
	{

		Eigen::VectorXd f = Eigen::VectorXd::Constant(m_numDoFs, 0.0);//Forces

		SpMat dFdx(m_numDoFs, m_numDoFs);
		SpMat dFdv(m_numDoFs, m_numDoFs);
		std::vector<T> derivPos = std::vector<T>();
		std::vector<T> derivVel = std::vector<T>();

		SpMat M(m_numDoFs, m_numDoFs);
		std::vector<T> masses = std::vector<T>();

		std::vector<bool> fixedIndices(m_numDoFs);

		for (int i = 0; i < SimObjects.size(); i++)
		{
			SimObjects[i]->SetPosition(&x);
			SimObjects[i]->SetVelocity(&v);

			SimObjects[i]->GetMass(&masses);
			SimObjects[i]->GetFixedIndices(&fixedIndices);
		}

		//FORCES
		debugHelper.RecordTime("2.Calculating forces");
		for (int i = 0; i < SimObjects.size(); i++)
		{
			SimObjects[i]->GetForce(&f, Colliders);
			SimObjects[i]->GetForceJacobian(&derivPos, &derivVel, Colliders);
		}

		debugHelper.RecordTime("3.Building matrices from triples");
		//For future reference maybe
		//https://stackoverflow.com/questions/45301305/set-sparsity-pattern-of-eigensparsematrix-without-memory-overhead
		dFdx.setFromTriplets(derivPos.begin(), derivPos.end(), [](const double& a, const double& b) { return a + b; });
		dFdv.setFromTriplets(derivVel.begin(), derivVel.end(), [](const double& a, const double& b) { return a + b; });
		M.setFromTriplets(masses.begin(), masses.end());

		//MATRIX OPERATIONS
		debugHelper.RecordTime("4.Calculating A and b");

		SpMat firstPart = M + (-h) * dFdv;

		SpMat A = firstPart + (-h * h) * dFdx;
		Eigen::VectorXd b;

		//b1 = (M - h dF / dv) v0 + h F0
		//bi + 1 = M v0 + h Fi - h dF / dv vi - h2 dF / dx xi

		if (iter == 0) {
			b = firstPart * v + h * f;
		}
		else {
			b = (M * previousState.v) + (h * f) - (h * dFdv * v) - (h * h * dFdx * x);
		}

		//FIXING
		debugHelper.RecordTime("5.Fixing");

		for (int i = 0; i < A.outerSize(); i++) {
			for (SpMat::InnerIterator j(A, i); j; ++j) {
				if (fixedIndices[i] || fixedIndices[j.row()])
					if (i == j.row())
						j.valueRef() = 1;
					else
						j.valueRef() = 0;
			}
		}

		for (size_t i = 0; i < m_numDoFs; i++)
		{
			if (fixedIndices[i]) {
				b[i] = 0;
			}
		}

		//SOLVING
		debugHelper.RecordTime("6.Solving");

		Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower | Eigen::Upper> cg;//UnitLower UnitUpper
		cg.setTolerance(tolerance);
		cg.compute(A);

		v = cg.solveWithGuess(b, v);
		x = previousState.x + h * v; // xi+1 = x0 + h vi+1
	}

	debugHelper.Wait();

	SimulationInfo newData;
	newData.x = x;
	newData.v = v;

	return newData;
}

Vector3f* PhysicsManager::GetVertices(int* count)
{
	std::lock_guard<std::mutex> lock(vertexMutex);

	if (!Updated) {
		*count = 0;
		return new Vector3f;
	}

	int totalCount = 0;
	for (size_t i = 0; i < SimObjects.size(); i++)
	{
		totalCount += SimObjects[i]->nVerts;
	}

	Vector3f* result = new Vector3f[totalCount];

	totalCount = 0;
	for (size_t i = 0; i < SimObjects.size(); i++)
	{
		int localCount = SimObjects[i]->nVerts;

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
	/*Depending on how Update is being executed, vertexArray might being updated at the same time. To prevent race conditions, we have to wait
	for the lock of vertexArray and copy all data to a second array. Unity/C# will process the data in the second array. In the meantime
	the data in the first array can be updated.*/

	std::lock_guard<std::mutex> lock(vertexMutex);

	if (id >= SimObjects.size() || !SimObjects[id]->updated) {
		*count = 0;
		return new Vector3f;
	}

	Updated = false;
	*count = SimObjects[id]->nVerts;
	return SimObjects[id]->GetVertices();
}

void PhysicsManager::SetParam(Eigen::VectorXd params, std::string settings)
{
	int offset = 0;

	for (int i = 0; i < SimObjects.size(); i++)
	{
		char massMode = settings[2 * i];

		switch (massMode)
		{
		case 'g':
		case 'G':
		{
			SimObjects[i]->SetMass(params(offset));
			offset += 1;
			break;
		}
		case 'l':
		case 'L':
		{
			SimObjects[i]->SetMass(params.segment(offset, SimObjects[i]->nVerts));
			offset += SimObjects[i]->nVerts;
			break;
		}
		default:
			break;
		}

		char stiffnessMode = settings[(2 * i) + 1];

		switch (stiffnessMode)
		{
		case 'g':
		case 'G':
		{
			SimObjects[i]->SetStiffness(params(offset));
			offset += 1;
			break;
		}
		case 'l':
		case 'L':
		{
			SimObjects[i]->SetStiffness(params.segment(offset, SimObjects[i]->nSprings));
			offset += SimObjects[i]->nSprings;
			break;
		}
		default:
			break;
		}
	}
}

PhysicsManager::SimulationInfo PhysicsManager::GetInitialState()
{
	Eigen::VectorXd x = Eigen::VectorXd(m_numDoFs);
	Eigen::VectorXd v = Eigen::VectorXd(m_numDoFs);
	for (size_t i = 0; i < SimObjects.size(); i++)
	{
		SimObjects[i]->GetPosition(&x);
		SimObjects[i]->GetVelocity(&v);
	}

	return SimulationInfo(x, v);
}

PhysicsManager::SimulationInfo PhysicsManager::Forward(Eigen::VectorXd x, Eigen::VectorXd v, float h, int subSteps)
{
	return StepImplicit(h, SimulationInfo(x, v), subSteps);
}

PhysicsManager::BackwardStepInfo PhysicsManager::Backward(Eigen::VectorXd x, Eigen::VectorXd v, Eigen::VectorXd x1, Eigen::VectorXd v1, Eigen::VectorXd dGdx1, Eigen::VectorXd dGdv1, float h, std::string settings)
{
	//Eigen::VectorXd f1 = Eigen::VectorXd::Constant(m_numDoFs, 0.0);//Forces
	Eigen::VectorXd dFdp = Eigen::VectorXd::Constant(m_numDoFs, 0.0);

	//Eigen::VectorXd x = current.x;
	SpMat dFdx(m_numDoFs, m_numDoFs);;
	SpMat dFdv(m_numDoFs, m_numDoFs);;
	SpMat M(m_numDoFs, m_numDoFs);;

	std::vector<T> derivPos = std::vector<T>();
	std::vector<T> derivVel = std::vector<T>();
	std::vector<T> masses = std::vector<T>();
	std::vector<T> dFdpTriplets = std::vector<T>();

	std::vector<bool> fixedIndices(m_numDoFs);

	//FORCES
	int nSprings = 0;
	for (int i = 0; i < SimObjects.size(); i++)
	{
		SimObjects[i]->SetPosition(&x);
		SimObjects[i]->SetVelocity(&v);
		SimObjects[i]->GetMass(&masses);
		//SimObjects[i]->GetForce(&f1);
		SimObjects[i]->GetForceJacobian(&derivPos, &derivVel, Colliders);
		SimObjects[i]->GetFixedIndices(&fixedIndices);

		SimObjects[i]->GetdFdp(&dFdp); //Podría pasar uT a esta función para hacer el cálculo dentro de la función
		SimObjects[i]->GetdFdp(&dFdpTriplets, nSprings);
		nSprings += SimObjects[i]->nSprings;
	}

	dFdx.setFromTriplets(derivPos.begin(), derivPos.end(), [](const double& a, const double& b) { return a + b; });
	dFdv.setFromTriplets(derivVel.begin(), derivVel.end(), [](const double& a, const double& b) { return a + b; });
	M.setFromTriplets(masses.begin(), masses.end());

	SpMat A = M - (h * dFdv) + ((-h * h) * dFdx);
	Eigen::VectorXd b = (h * dGdx1) + dGdv1;

	/*for (int i = 0; i < A.outerSize(); i++) {
		for (SpMat::InnerIterator j(A, i); j; ++j) {
			if (fixedIndices[i] || fixedIndices[j.row()])
				if (i == j.row())
					j.valueRef() = 1;
				else
					j.valueRef() = 0;
		}
	}

	for (size_t i = 0; i < m_numDoFs; i++)
	{
		if (fixedIndices[i]) {
			b[i] = 0;
		}
	}*/

	Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::UnitLower | Eigen::UnitUpper> cg;
	cg.setTolerance(tolerance);
	cg.compute(A);

	Eigen::VectorXd u = cg.solve(b);

	//Backward
	BackwardStepInfo info = BackwardStepInfo();

	// Constraint dinámica --> c = M * v1 - M * v - h * f1
	// Constraint fixing --> S * v1 = 0 ,S matriz de selección (diagonal con 1 donde esté fijado)

	//Constraint dinámica + fixing --> c = M * v1 – M * v – h * (I – ST * S) f1 = 0

	//Derivada constraint dinámica --> dcdp = uT * dGdp
	//Derivada constraint dinámica + fixing --> dcdp = uT * dGdp

	std::vector <Eigen::VectorXd>  ds;

	int paramOffset = 0;
	int vertOffset = 0;
	int springOffset = 0;

	for (int i = 0; i < SimObjects.size(); i++)
	{
		char massMode = settings[(size_t)2 * i];
		char stiffnessMode = settings[(size_t)(2 * i) + 1];

		int nVerts = SimObjects[i]->nVerts;
		int nSprings = SimObjects[i]->nSprings;

		switch (massMode)
		{
		case 'g':
		case 'G':
		{
			// GLOBAL MASS
			Eigen::VectorXd dcdp = v1 - v;
			for (size_t j = vertOffset; j < (size_t)vertOffset + nVerts; j++)
			{
				if (fixedIndices[j]) {
					dcdp(3 * j) = 0;
					dcdp((3 * j) + 1) = 0;
					dcdp((3 * j) + 2) = 0;
				}
			}

			Eigen::VectorXd dGdp_mass = -u.segment((size_t)vertOffset * 3, (size_t)nVerts * 3).transpose() * dcdp.segment((size_t)vertOffset * 3, (size_t)nVerts * 3);

			ds.push_back(dGdp_mass);
			paramOffset += 1;

			break;
		}
		case 'l':
		case 'L':
		{
			// LOCAL MASS
			Eigen::VectorXd dcdp = v1 - v;

			std::vector<T> dcdpTiplets = std::vector<T>();
			//c rows, p columns

			for (size_t j = 0; j < nVerts; j++)
			{
				int idx = j + vertOffset;
				if (!fixedIndices[idx]) {
					dcdpTiplets.push_back(T((3 * j) + 0, j, dcdp((3 * idx) + 0)));
					dcdpTiplets.push_back(T((3 * j) + 1, j, dcdp((3 * idx) + 1)));
					dcdpTiplets.push_back(T((3 * j) + 2, j, dcdp((3 * idx) + 2)));
				}

			}

			SpMat dcdpMat((size_t)nVerts * 3, nVerts);
			dcdpMat.setFromTriplets(dcdpTiplets.begin(), dcdpTiplets.end());

			Eigen::VectorXd dGdp_mass = -u.segment((size_t)vertOffset * 3, (size_t)nVerts * 3).transpose() * dcdpMat; //Vector de tantas posiciones como parámetros haya

			ds.push_back(dGdp_mass);
			paramOffset += nVerts;

			break;
		}
		}

		switch (stiffnessMode)
		{
		case 'g':
		case 'G':
		{
			// GLOBAL STIFFNESS
			Eigen::VectorXd dcdp = h * dFdp;
			Eigen::VectorXd dGdp_stiffness = -u.segment((size_t)vertOffset * 3, (size_t)nVerts * 3).transpose() * dcdp.segment((size_t)vertOffset * 3, (size_t)nVerts * 3);
			ds.push_back(dGdp_stiffness);
			paramOffset += 1;
		}
		break;
		case 'l':
		case 'L':
		{
			// LOCAL STIFFNESS
			SpMat dcdpMat((size_t)nVerts * 3, nSprings);
			std::vector<T> abc = std::vector<T>();

			for (size_t j = 0; j < dFdpTriplets.size(); j++)
			{
				T t = dFdpTriplets[j];

				if (t.col() < springOffset || t.col() >= springOffset + nSprings)
					continue;

				if (t.row() < 3 * vertOffset || t.row() >= 3 * (vertOffset + nVerts))
					continue;

				abc.push_back(T(t.row() - (3 * vertOffset), t.col() - springOffset, t.value()));
			}

			dcdpMat.setFromTriplets(abc.begin(), abc.end());

			Eigen::VectorXd dGdp_stiffness = -u.segment((size_t)vertOffset * 3, (size_t)nVerts * 3).transpose() * dcdpMat; //Vector de tantas posiciones como parámetros haya
			ds.push_back(dGdp_stiffness);

			paramOffset += nSprings;
			break;
		}
		default:
			break;
		}

		vertOffset += nVerts;
		springOffset += nSprings;
	}


	Eigen::VectorXd  dGdpTotal;

	for (size_t i = 0; i < ds.size(); i++)
	{

		Eigen::VectorXd vec_joined(ds[i].size() + dGdpTotal.size());
		vec_joined << dGdpTotal, ds[i];

		dGdpTotal = vec_joined;
	}


	info.dGdp = dGdpTotal;

	info.dGdx = dGdx1 + h * (u.transpose() * dFdx).transpose();

	info.dGdv = M * u;

	return info;
}
