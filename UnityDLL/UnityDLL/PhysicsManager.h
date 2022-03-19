#pragma once 
#include <vector>
#include "Types.h"

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
		Eigen::VectorXd* x;
		Eigen::VectorXd* v;
		double parameter;
	};

	bool Paused = false;
	float TimeStep;
	Vector3f Gravity;
	Integration integrationMethod = Integration::Implicit;

	std::vector<Object*> SimObjects;
	std::vector<Fixer*> Fixers;
	int m_numDoFs;

	bool initialized = false;
	SimulationInfo _simulationInfo;

public:
	bool Updated = false;

	PhysicsManager(Integration _IntegrationMethod);
	~PhysicsManager();

	int AddObject(Vector3f position, Vector3f* vertices, int nVertices, int* triangles, int nTriangles, float stiffness, float mass);
	void AddFixer(Vector3f position, Vector3f scale);

	void Start();
	void Update(float time, float h);

	void StepSymplecticOld(float time, float h);

	void StepSymplecticDense(float time, float h);

	void StepSymplecticSparse(float time, float h);

	SimulationInfo StepImplicit(float time, float h, SimulationInfo simulationInfo);

	Vector3f* GetVertices(int id, int* count);

};


