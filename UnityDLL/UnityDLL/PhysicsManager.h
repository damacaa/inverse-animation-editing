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
	

	bool Paused = false;
	float TimeStep;
	Vector3f Gravity;
	Integration integrationMethod = Integration::Implicit;

	std::vector<Object*> SimObjects;
	std::vector<Fixer*> Fixers;
	int m_numDoFs;

	bool initialized = false;
	Eigen::VectorXd _v;
	float vDecrease = 1.0f;

	void PrintMat(SpMat mat, std::string name = "test");

public:
	bool Updated = false;
	

	PhysicsManager(Integration _IntegrationMethod) {
		integrationMethod = _IntegrationMethod;
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

