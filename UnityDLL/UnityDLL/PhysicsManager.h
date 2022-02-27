#pragma once
#include "Node.h"
#include <math.h>   
#include <vector>
#include "Fixer.h"
#include "Spring.h"
#include "Object.h"


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
	Integration IntegrationMethod = PhysicsManager::Integration::Symplectic;

	std::vector<Fixer*> Fixers;
	int m_numDoFs;

public:
	std::vector<Object*> SimObjects;


	PhysicsManager() {
		SimObjects = std::vector<Object*>();
	}
	~PhysicsManager();

	int AddObject(Vector3f position, Vector3f* vertices, int nVertices, int* triangles, int nTriangles);
	void AddFixer(Vector3f position, Vector3f scale);

	void Start();
	void Update(float time, float h);

	void StepSymplectic(float time, float h);

	void StepImplicit();

};

