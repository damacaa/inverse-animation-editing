#pragma once
#include "Types.h"
#include "Node.h"
#include "Spring.h"
#include "Fixer.h"

class PhysicsManager;
class Object
{
private:
	//Eigen::Vector3d GetNormal(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c);

public:
	PhysicsManager* manager;

	int id = -1;//Id to return vertices
	int index = -1;//Id in matrix
	bool updated = false;
	float damping = 0.5f;
	float stiffness = 150.0f;
	float mass = 1.0f;

	//Eigen::Vector3d positon;

	Vector3f* vertexArray = 0;
	Vector3f* vertexArray2 = 0;
	int nVertices = 0;

	Node* nodeArray = 0;
	Spring* springArray = 0;
	int nSprings = 0;

	//int* triangles = 0;
	//int nTriangles = 0;

	Object(Vector3f pos, Vector3f* vertices, int nVerts, int* triangles, int nTriangles, float stiffness, float mass);
	~Object();

	void SetNodeMass(float parameter);

	void Update(float time, float h);

	void Initialize(int* ind);

	int GetNumDoFs();

	void GetPosition(Eigen::VectorXd* position);

	void SetPosition(Eigen::VectorXd* position);

	void GetVelocity(Eigen::VectorXd* velocity);

	void SetVelocity(Eigen::VectorXd* velocity);

	void GetForce(Eigen::VectorXd* force);

	void GetForceJacobian(std::vector<T>* derivPos, std::vector<T>* derivVel);

	void GetMass(Eigen::MatrixXd* mass);

	void GetMass(std::vector<T>* mass);

	void GetMassInverse(Eigen::MatrixXd* massInv);

	void GetMassInverse(std::vector<T>* massTripletVector);

	void FixVector(Eigen::VectorXd* v);

	void FixMatrix(Eigen::MatrixXd* M);

	void FixMatrix(SpMat* M);

	void FixnodeArray(Fixer* f);

	void GetFixedIndices(std::vector<bool>* fixedIndices);

	void UpdateVertices();

	Vector3f* GetVertices();
};

