#include "pch.h"
#include "PhysicsManager.h"
#include "Object.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;



PhysicsManager::PhysicsManager(std::string info)
{
	debugHelper = DebugHelper();

	//debugHelper.PrintValue(info, "scene");

	json js = json::parse(info);

	integrationMethod = (Integration)js["integrationMethod"];
	tolerance = js["tolerance"];

	for (size_t i = 0; i < js["objects"].size(); i++)
	{
		json obj = js["objects"][i];

		int nVerts = obj["vertPos"].size();
		int nSprings = obj["springs"].size() / 2;

		Vector3f* vertPos = new Vector3f[nVerts];
		bool* vertIsFixed = new bool[nVerts];
		float vertMass = obj["vertMass"];

		int* springs = new int[(int)nSprings * 2];
		float* springStiffness = new float[nSprings];
		float damping = obj["damping"];

		std::string s = "";
		for (size_t j = 0; j < nVerts; j++)
		{
			json pos = obj["vertPos"][j];

			float x = pos["x"];
			float y = pos["y"];
			float z = pos["z"];

			vertPos[j] = Vector3f(x, y, z);

			vertIsFixed[j] = obj["vertIsFixed"][j];
		}

		for (size_t j = 0; j < nSprings; j++)
		{
			springs[2 * j] = obj["springs"][2 * j];
			springs[(2 * j) + 1] = obj["springs"][(2 * j) + 1];
			springStiffness[j] = obj["springStiffness"][j];
		}

		int id = AddObject(vertPos, vertIsFixed, vertMass, nVerts, springs, springStiffness, nSprings, damping);
		//Vector3f* vertPos, float vertMass, int nVerts, int* springs, float* springStiffness, int nSprings, float damping
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
	if (SimObjects.size() > 0) {
		std::string description = std::to_string(SimObjects[0]->nVerts) + " vertices and ";
		description += std::to_string(SimObjects[0]->nSprings) + " springs.";
		//debugHelper.PrintTimes("SimulationTimes", description);
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

int PhysicsManager::AddObject(Vector3f* vertices, int nVertices, int* triangles, int nTriangles, float stiffness, float mass)
{
	Object* o = new Object(vertices, nVertices, triangles, nTriangles, stiffness, mass);
	o->id = (int)SimObjects.size() + (int)PendingSimObjects.size();
	PendingSimObjects.push_back(o);

	needsRestart = true;
	return o->id;
}

int PhysicsManager::AddObject(Vector3f* vertPos, bool* vertIsFixed, float vertMass, int nVerts, int* springs, float* springStiffness, int nSprings, float damping)
{
	Object* o = new Object(vertPos, vertIsFixed, vertMass, nVerts, springs, springStiffness, nSprings, damping);
	o->id = (int)SimObjects.size() + (int)PendingSimObjects.size();
	SimObjects.push_back(o);

	std::string s = "";

	for (size_t i = 0; i < nVerts; i++)
	{
		s += std::to_string(vertPos[i].x) + ", " + std::to_string(vertPos[i].y) + ", " + std::to_string(vertPos[i].z) + "\n";
		s += vertIsFixed[i] + "\n";
		s += std::to_string(vertMass) + "\n";
		s += std::to_string(nVerts) + "\n\n";

		s += std::to_string(o->_nodes[i].position.x()) + ", " + std::to_string(o->_nodes[i].position.y()) + ", " + std::to_string(o->_nodes[i].position.z()) + "\n";
	}

	//debugHelper.PrintValue(s, "objectInit");

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
		/*Object* o = SimObjects[i];
		std::string s = "";
		for (size_t j = 0; j < o->nVerts; j++)
		{
			//s += std::to_string(o->GetVertices()[0].x);
			s += std::to_string(o->_nodes[j].position.x()) + ", " + std::to_string(o->_nodes[j].position.y()) + ", " + std::to_string(o->_nodes[j].position.z()) + "\n";
		}

		debugHelper.PrintValue(s, "objStart" + std::to_string(i));*/

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
	//newSimulationInfo.parameter = simulationInfo.parameter;

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
	std::vector<T> masses = std::vector<T>();

	//SpMat Mi(m_numDoFs, m_numDoFs);
	//std::vector<T> massesi = std::vector<T>();

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

	//OLD FIXING
	/*std::vector<T> tripletsA;
	for (int i = 0; i < A.outerSize(); i++)
		for (typename Eigen::SparseMatrix<double>::InnerIterator it(A, i); it; ++it) {
			if (!fixedIndices[it.row()] && !fixedIndices[it.col()])
				tripletsA.push_back(T(it.row(), it.col(), it.value())); //tripletsA.emplace_back(it.row(), it.col(), it.value());
			else if (it.row() == it.col())
				tripletsA.push_back(T(it.row(), it.col(), 1));//tripletsA.emplace_back(it.row(), it.col(), 1);
		}
	A.setFromTriplets(tripletsA.begin(), tripletsA.end());*/

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
	//A.Solve(b, v);
	//Eigen::SimplicialCholesky<SpMat> chol(A);
	//v = chol.solve(b);


	Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower | Eigen::Upper> cg;//UnitLower UnitUpper
	cg.setTolerance(tolerance);
	cg.compute(A);

	//v = v + h * (f * Mi);
	v = cg.solveWithGuess(b, v);

	x += h * v;

	debugHelper.Wait();

	SimulationInfo newData;
	newData.x = x;
	newData.v = v;

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

float PhysicsManager::Estimate(float parameter, int iter, float h, Eigen::VectorXd* _dGdp)
{
	//Simulation to define target
	Start();
	SimulationInfo target = info;

	for (size_t i = 0; i < SimObjects.size(); i++)
	{
		SimObjects[i]->SetMass(0.5f);
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
	SimulationInfo forwardResult = info;

	//Simulation with given parameter
	for (size_t i = 0; i < SimObjects.size(); i++)
	{
		SimObjects[i]->SetMass(parameter);
	}

	for (int i = 0; i < SimObjects.size(); i++)
	{
		SimObjects[i]->SetPosition(&forwardResult.x);
		SimObjects[i]->SetVelocity(&forwardResult.v);
	}

	std::vector<SimulationInfo> steps(iter);

	for (size_t i = 0; i < iter; i++)
	{
		//Guardar en simulation info lo mínimo v y x y luego recalcular derivadas
		forwardResult = StepImplicit(h, forwardResult);
		steps[i] = forwardResult;
		for (int j = 0; j < SimObjects.size(); j++)
		{
			SimObjects[j]->SetPosition(&forwardResult.x);
			SimObjects[j]->SetVelocity(&forwardResult.v);
		}
	}

	//Evaluate g

	//g = sum_i |xi - xi*|^2 //Caso genérico n frames
	//g = |xn - xn*|^2 //Caso frame final

	//g = sum_i (xi - xi*)T (xi - xi*)
	//dg / dxi = 2 (xi - xi*)T

	//forwardResult = steps[iter - 1]
	float g = (target.x - forwardResult.x).squaredNorm();

	//return g * g;

	Eigen::VectorXd dGdp = Eigen::VectorXd::Constant(1, 0.0);//Tantos como parametros haya
	std::vector<Eigen::VectorXd> dGdx(iter);
	std::vector<Eigen::VectorXd> dGdv(iter);

	for (size_t i = 0; i < iter; i++)
	{
		//dg/dxi = 0, si i != n;  dg/dxn = 2 (xn - xn*)T
		if (i == iter - 1) {
			dGdx[i] = 2.0 * (target.x - forwardResult.x);
		}
		else
			dGdx[i] = Eigen::VectorXd::Constant(m_numDoFs, 0.0);

		dGdv[i] = Eigen::VectorXd::Constant(m_numDoFs, 0.0);
	}

	for (int i = iter - 2; i >= 0; i--)
	{
		BackwardStepInfo backStepResult = Backward(steps[i].x, steps[i].v, steps[i + 1].x, steps[i + 1].v, dGdx[i + 1], dGdv[i + 1], h, "LL");//dGdp, dGdx, dGdv

		//Global
		dGdp += backStepResult.dGdp;

		dGdx[i] += backStepResult.dGdx;

		dGdv[i] += backStepResult.dGdv;
	}

	std::string sep = "\n----------------------------------------\n";

	Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");

	std::stringstream ss;
	ss.str("Back propagation results");
	ss << "dGdp: " << dGdp.format(CommaInitFmt) << sep;
	ss << "dGdx: " << dGdx[0].format(CommaInitFmt) << sep;
	ss << "dGdv: " << dGdv[0].format(CommaInitFmt) << sep;
	std::string result = ss.str();

	debugHelper.PrintValue(result, "backstep");

	*_dGdp = dGdp;
	return g;
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
	if (id >= SimObjects.size() || !SimObjects[id]->updated) {
		*count = 0;
		return new Vector3f;
	}

	Updated = false;
	*count = SimObjects[id]->nVerts;
	return SimObjects[id]->GetVertices();
}

void PhysicsManager::SetParam(float param)
{
	for (size_t i = 0; i < SimObjects.size(); i++)
	{
		//SimObjects[i]->SetMass(param);
		SimObjects[i]->SetStiffness(param);
	}
}

void PhysicsManager::SetParam(Eigen::VectorXd params, std::string settings)
{
	Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");
	std::stringstream ss;
	ss << params.format(CommaInitFmt);
	std::string result = ss.str();

	debugHelper.PrintValue(result, "params");

	char massMode = settings[0];
	char stiffnessMode = settings[1];

	int offset = 0;

	switch (massMode)
	{
	case 'g':
	case 'G':
	{
		SimObjects[0]->SetMass(params(0));
		offset = 1;
		break;
	}
	case 'l':
	case 'L':
	{
		SimObjects[0]->SetMass(params.segment(offset, (params.count()  - offset)));
		offset = SimObjects[0]->nVerts;
		break;
	}
	default:
		break;
	}

	switch (stiffnessMode)
	{
	case 'g':
	case 'G':
	{
		SimObjects[0]->SetStiffness(params(offset));
		offset += 1;
		break;
	}
	case 'l':
	case 'L':
	{
		SimObjects[0]->SetStiffness(params);//.head(offset)
		offset += SimObjects[0]->nSprings;
		break;
	}
	default:
		break;
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

PhysicsManager::SimulationInfo PhysicsManager::Forward(Eigen::VectorXd x, Eigen::VectorXd v, float h)
{
	//Updates springs?
	return StepImplicit(h, SimulationInfo(x, v));
}

PhysicsManager::BackwardStepInfo PhysicsManager::Backward(Eigen::VectorXd x, Eigen::VectorXd v, Eigen::VectorXd x1, Eigen::VectorXd v1, Eigen::VectorXd dGdx1, Eigen::VectorXd dGdv1, float h, std::string settings)
{
	std::string sep = "\n----------------------------------------\n";
	Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");
	std::stringstream ss;
	ss.str("Back propagation results\n");

	char massMode = settings[0];
	char stiffnessMode = settings[1];

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

	//FORCES
	for (int i = 0; i < SimObjects.size(); i++)
	{
		SimObjects[i]->SetPosition(&x);
		SimObjects[i]->SetVelocity(&v);
		SimObjects[i]->GetMass(&masses);
		//SimObjects[i]->GetForce(&f1);
		SimObjects[i]->GetForceJacobian(&derivPos, &derivVel);

		SimObjects[i]->GetdFdp(&dFdp); //Podría pasar uT a esta función para hacer el cálculo dentro de la función
		SimObjects[i]->GetdFdp(&dFdpTriplets);
	}

	dFdx.setFromTriplets(derivPos.begin(), derivPos.end(), [](const double& a, const double& b) { return a + b; });
	dFdv.setFromTriplets(derivVel.begin(), derivVel.end(), [](const double& a, const double& b) { return a + b; });
	M.setFromTriplets(masses.begin(), masses.end());

	SpMat A = M - (h * dFdv) + ((-h * h) * dFdx);
	Eigen::VectorXd b = (h * dGdx1) + dGdv1;

	Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::UnitLower | Eigen::UnitUpper> cg;
	cg.setTolerance(tolerance);
	cg.compute(A);

	Eigen::VectorXd u = cg.solve(b);

	//Backward
	BackwardStepInfo info = BackwardStepInfo();

	//dcdp --> c = M * v1 - M * v - h * f1
	//dGdp


	Eigen::VectorXd dGdp1;
	switch (massMode)
	{
	case 'g':
	case 'G':
	{
		// GLOBAL MASS
		Eigen::VectorXd dcdp = v1 - v;
		dGdp1 = -u.transpose() * dcdp;
		break;
	}
	case 'l':
	case 'L':
	{
		// LOCAL MASS
		Eigen::VectorXd dcdp = v1 - v;

		std::vector<T> dcdpTiplets = std::vector<T>();
		//c filas p columnas
		int j = 0;
		for (size_t i = 0; i < m_numDoFs; i += 3)
		{
			dcdpTiplets.push_back(T(i, j, dcdp(i)));
			dcdpTiplets.push_back(T(i + 1, j, dcdp(i + 1)));
			dcdpTiplets.push_back(T(i + 2, j, dcdp(i + 2)));
			j++;
		}

		SpMat dcdpMat(m_numDoFs, m_numDoFs / 3);
		dcdpMat.setFromTriplets(dcdpTiplets.begin(), dcdpTiplets.end());


		dGdp1 = -u.transpose() * dcdpMat; //Vector de tantas posiciones como parámetros haya
		break;
	}
	default:
		dGdp1 = Eigen::VectorXd::Constant(0, 0);
		break;
	}

	Eigen::VectorXd dGdp2;
	switch (stiffnessMode)
	{
	case 'g':
	case 'G':
	{
		// GLOBAL STIFFNESS
		Eigen::VectorXd dcdp = h * dFdp;
		dGdp2 = -u.transpose() * dcdp;

		ss << "dFdp: " << dFdp.format(CommaInitFmt) << sep;
	}
	break;
	case 'l':
	case 'L':
	{
		// LOCAL STIFFNESS
		/*igen::VectorXd dcdp = h * dFdp;//Stiffness

		std::vector<T> dcdpTiplets = std::vector<T>();
		//c filas p columnas
		int j = 0;
		for (size_t i = 0; i < m_numDoFs; i += 3)
		{
			dcdpTiplets.push_back(T(i, j, dcdp(i)));
			dcdpTiplets.push_back(T(i + 1, j, dcdp(i + 1)));
			dcdpTiplets.push_back(T(i + 2, j, dcdp(i + 2)));
			j++;
		}*/

		SpMat dcdpMat(m_numDoFs, SimObjects[0]->nSprings);
		dcdpMat.setFromTriplets(dFdpTriplets.begin(), dFdpTriplets.end());

		dGdp2 = -u.transpose() * dcdpMat; //Vector de tantas posiciones como parámetros haya

		ss << "u: " << u.format(CommaInitFmt) << sep;
		ss << "dcdpMat:\n" << Eigen::MatrixXd(dcdpMat) << sep;
		ss << "dGdp2:\n" << dGdp2.format(CommaInitFmt) << sep;
		//dGdp2 = -u.transpose() * Eigen::VectorXd::Constant(6, 0);
		break;
	}
	default:
		dGdp2 = Eigen::VectorXd::Constant(0, 0);
		break;
	}

	Eigen::VectorXd vec_joined(dGdp1.size() + dGdp2.size());
	vec_joined << dGdp1, dGdp2;

	info.dGdp = vec_joined;

	//Eigen::VectorXd dcdp = v1 - v; //Mass


	//info.dGdp = Eigen::VectorXd::Constant(1, (-u.transpose() * dcdp)(0));//old



	/*ss << "dcdp: " << dcdp.format(CommaInitFmt) << sep;
	ss << "d:\n" << Eigen::MatrixXd(dcdpMat) << sep;
	ss << "u: " << u.format(CommaInitFmt) << sep;
	ss << "dGdp: " << info.dGdp.format(CommaInitFmt) << sep;*/
	std::string result = ss.str();
	debugHelper.PrintValue(result, "backstep");

	//dGdx
	info.dGdx = dGdx1 + h * (u.transpose() * dFdx).transpose();

	//dGdv
	info.dGdv = M * u;

	return info;
}
