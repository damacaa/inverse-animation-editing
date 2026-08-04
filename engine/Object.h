#pragma once
#include "Types.h"
#include "Node.h"
#include "Spring.h"
#include "Face.h"
#include "Fixer.h"


class PhysicsManager;
class Collider;
class Object
{
private:

public:
	int id = -1;
	int index = -1;
	bool updated = false;
	float damping = 0.5f;
	float stiffness = 150.0f;
	double dragCoefficient = 1.0f;

	std::vector<Vector3f> vertexArray;
	std::vector<Vector3f> vertexArray2;
	int nVerts = 0;
	int nSprings = 0;
	int nFaces = 0;

	std::vector<Node> _nodes;
	std::vector<Spring> _springs;
	std::vector<Face> _faces;

	// Parameter values loaded from the scene, restored by ResetMass/ResetStiffness
	std::vector<double> initialVertexMass;
	std::vector<double> initialSpringStiffness;

	std::string optimizationSettings;

	Object(Vector3f* vertPos, bool* vertIsFixed, float* vertMass, int nVerts, int* springs, float* springStiffness, int nSprings,  int* triangles, int nTriangles, double dragCoefficient, float damping, std::string optimizationSettings, PhysicsManager* physicsManager);
	
	~Object();

	void Initialize(int* ind);

	int GetNumDoFs();

	void GetPosition(Eigen::VectorXd* position);

	void SetPosition(Eigen::VectorXd* position);

	void GetVelocity(Eigen::VectorXd* velocity);

	void SetVelocity(Eigen::VectorXd* velocity);

	void GetForce(Eigen::VectorXd* force, std::vector<Collider*> colliders);

	void GetdFdp(Eigen::VectorXd* dforce);

	void GetdFdp(std::vector<T>* dforce, int springOffset);

	void GetForceJacobian(std::vector<T>* derivPos, std::vector<T>* derivVel, std::vector<Collider*> colliders);

	void GetMass(std::vector<T>* mass);

	void GetMassInverse(std::vector<T>* massTripletVector);

	void Fixnodes(Fixer* f);

	void GetFixedIndices(std::vector<bool>* fixedIndices);

	void UpdateVertices();

	Vector3f* GetVertices();

	void SetMass(double param);

	void SetMass(Eigen::VectorXd params);

	void SetStiffness(double param);

	void SetStiffness(Eigen::VectorXd params);

	void ResetMass();

	void ResetStiffness();
};

