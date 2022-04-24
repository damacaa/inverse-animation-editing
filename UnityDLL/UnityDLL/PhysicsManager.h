#pragma once 
#include <vector>
#include "Types.h"
#include "DebugHelper.h"

class Object;
class Fixer;

enum class Integration
{
	Explicit = 0,
	Symplectic = 1,
	Implicit = 2,
};

class PhysicsManager
{
private:

	struct SimulationInfo {
		double parameter;
		Eigen::VectorXd x, v, f;
		SpMat M, dFdx, dFdv;
	};

	struct BackwardStepInfo {
		Eigen::VectorXd dGdp;//Tantos como parametros haya
		Eigen::VectorXd dGdx, dGdv;
	};

	bool Paused = false;
	float TimeStep;
	Vector3f Gravity;
	Integration integrationMethod = Integration::Implicit;

	std::vector<Object*> SimObjects;
	std::vector<Object*> PendingSimObjects;
	std::vector<Fixer*> Fixers;
	std::vector<Fixer*> PendingFixers;
	bool needsRestart = true;
	int m_numDoFs;

	bool initialized = false;
	SimulationInfo info;

	DebugHelper debugHelper;

public:
	bool Updated = false;

	PhysicsManager(Integration _IntegrationMethod);
	~PhysicsManager();

	int AddObject(Vector3f position, Vector3f* vertices, int nVertices, int* triangles, int nTriangles, float stiffness, float mass);

	void AddFixer(Vector3f position, Vector3f scale);

	void Start();

	void UpdatePhysics(float time, float h);

	void UpdateObjects();

	float Estimate(float parameter, int iter, float h);

	PhysicsManager::BackwardStepInfo Backwards(Eigen::VectorXd x1, Eigen::VectorXd v1, float parameter, Eigen::VectorXd dGdx1, Eigen::VectorXd dGdv1, float h, SimulationInfo previous);

	SimulationInfo StepSymplectic(float h, SimulationInfo simulationInfo);

	SimulationInfo StepImplicit(float h, SimulationInfo simulationInfo);

	Vector3f* GetVertices(int id, int* count);
};


