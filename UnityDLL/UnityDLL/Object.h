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

	int id = -1;
	bool updated = false;
	int substeps = 5;
	float damping = 1.0f;
	float stiffness = 150.0f;

	Eigen::Vector3d positon;

	Vector3f* vertexArray = 0;
	Vector3f* vertexArray2 = 0;
	int nVertices = 0;

	Node* nodeArray = 0;
	Spring* springArray = 0;
	int nSprings = 0;

	//int* triangles = 0;
	//int nTriangles = 0;

	Object(Vector3f pos, Vector3f* vertices, int nVerts, int* triangles, int nTriangles);
	~Object();

	void Update(float time, float h);

	inline void Initialize(int ind) {};
	inline int GetNumDoFs() { return 0; };

	inline void GetPosition(Eigen::VectorXd* position) {};
	inline void SetPosition(Eigen::VectorXd* position) {};
	inline void GetVelocity(Eigen::VectorXd* velocity) {};
	inline void SetVelocity(Eigen::VectorXd* velocity) {};

	inline void GetForce(Eigen::VectorXd* force) {};
	inline void GetForceJacobian(Eigen::MatrixXd* dFdx, Eigen::MatrixXd* dFdv) {};

	inline void GetMass(Eigen::MatrixXd* mass) {};
	inline void GetMassInverse(Eigen::MatrixXd* massInv) {};

	inline void FixMatrix(Eigen::MatrixXd* M) {};
	inline void FixVector(Eigen::VectorXd* v) {};

	void FixnodeArray(Fixer* f);
	inline Vector3f* GetVertices() {
		for (int i = 0; i < nVertices; i++) {
			vertexArray[i] = Vector3f(
				(float)nodeArray[i].position.x(),
				(float)nodeArray[i].position.y(),
				(float)nodeArray[i].position.z());
		}

		memcpy(vertexArray2, vertexArray, sizeof(Vector3f) * nVertices); //Copy data only if vertexArray has been updated.

		updated = false;

		return vertexArray2;
	};
};

