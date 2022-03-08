#pragma once 
#include <vector>
#include "Types.h"

class Object;
class Fixer;
class PhysicsManager
{
private:
	enum class Integration
	{
		Explicit = 0,
		Symplectic = 1,
		Implicit = 2,
	};

	bool Paused = false;
	float TimeStep;
	Vector3f Gravity;
	Integration IntegrationMethod = PhysicsManager::Integration::Implicit;

	std::vector<Object*> SimObjects;
	std::vector<Fixer*> Fixers;
	int m_numDoFs;

	bool initialized = false;
	Eigen::VectorXd _v;
	float vDecrease = 1.0f;

public:
	bool Updated = false;

	PhysicsManager() {
		SimObjects = std::vector<Object*>();
	}
	~PhysicsManager();

	int AddObject(Vector3f position, Vector3f* vertices, int nVertices, int* triangles, int nTriangles, float stiffness, float mass);
	void AddFixer(Vector3f position, Vector3f scale);

	void Start();
	void Update(float time, float h);

	void StepSymplecticOld(float time, float h);

	void StepSymplecticDense(float time, float h);

	void StepSymplecticSparse(float time, float h);

	void StepImplicit(float time, float h);

	Vector3f* GetVertices(int id, int* count);

};

