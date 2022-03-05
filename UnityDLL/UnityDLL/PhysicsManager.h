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

	std::vector<Fixer*> Fixers;
	int m_numDoFs;

public:
	bool Updated = false;
	std::vector<Object*> SimObjects;


	PhysicsManager() {
		SimObjects = std::vector<Object*>();
	}
	~PhysicsManager();

	int AddObject(Vector3f position, Vector3f* vertices, int nVertices, int* triangles, int nTriangles);
	void AddFixer(Vector3f position, Vector3f scale);

	void Start();
	void Update(float time, float h);

	void StepSymplecticOld(float time, float h);

	void StepSymplecticDense(float time, float h);

	void StepSymplecticSparse(float time, float h);


	void StepImplicit(float time, float h);

	Vector3f* GetVertices(int id, int* count);

};

