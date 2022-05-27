#pragma once 
#include <vector>
#include "Types.h"
#include "DebugHelper.h"


class Object;
class Fixer;



class PhysicsManager
{
public:
	bool Updated = false;

	enum class Integration
	{
		Explicit = 0,
		Symplectic = 1,
		Implicit = 2,
	};

private:

	struct SimulationInfo {
		SimulationInfo() {}
		SimulationInfo(Eigen::VectorXd x, Eigen::VectorXd v) {
			this->x = x;
			this->v = v;
		}
		Eigen::VectorXd x, v;
	};

	struct BackwardStepInfo {
		Eigen::VectorXd dGdp;//Tantos como parametros haya
		Eigen::VectorXd dGdx, dGdv;
	};

	bool Paused = false;
	float TimeStep;
	double tolerance = 1e-2;
	Vector3f Gravity;
	PhysicsManager::Integration integrationMethod = PhysicsManager::Integration::Implicit;

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

	PhysicsManager(std::string info);
	PhysicsManager(PhysicsManager::Integration _IntegrationMethod = PhysicsManager::Integration::Implicit, double tolerance = 1e-2);
	~PhysicsManager();

	int AddObject(Vector3f* vertices, int nVertices, int* triangles, int nTriangles, float stiffness, float mass);

	int AddObject(Vector3f* vertPos, bool* vertIsFixed, float vertMass, int nVerts, int* springs, float* springStiffness, int nSprings, float damping);

	void AddFixer(Vector3f position, Vector3f scale);

	void Start();

	void UpdatePhysics(float time, float h);

	void UpdateObjects();

	float Estimate(float parameter, int iter, float h, Eigen::VectorXd* _dGdp);

	SimulationInfo StepSymplectic(float h, SimulationInfo simulationInfo);

	SimulationInfo StepImplicit(float h, SimulationInfo simulationInfo);

	void UpdateVertices();

	Vector3f* GetVertices(int* count);

	Vector3f* GetVertices(int id, int* count);

	void SetParam(float param);

	void SetParam(Eigen::VectorXd params, std::string settings);

	SimulationInfo GetInitialState();

	SimulationInfo Forward(Eigen::VectorXd x, Eigen::VectorXd v, float h);

	PhysicsManager::BackwardStepInfo Backward(Eigen::VectorXd x, Eigen::VectorXd v, Eigen::VectorXd x1, Eigen::VectorXd v1, Eigen::VectorXd dGdx1, Eigen::VectorXd dGdv1, float h, std::string settings);

	
};


