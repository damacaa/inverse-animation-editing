#include "pch.h"
#include "Object.h"
#include <set>
#include <map>
#include "DebugHelper.h"

Object::Object(Vector3f* vertPos, bool* vertIsFixed, float* vertMass, int nVerts, int* springs, float* springStiffness, int nSprings, int* triangles, int nTriangles, double dragCoefficient, float damping, std::string optimizationSettings, PhysicsManager* physicsManager)
{
	this->stiffness = stiffness;
	this->damping = damping;

	this->nVerts = nVerts;
	this->nSprings = nSprings;
	this->nFaces = nTriangles / 3;

	vertexArray = new Vector3f[nVerts];
	vertexArray2 = new Vector3f[nVerts];

	memcpy(vertexArray, vertPos, sizeof(Vector3f) * nVerts);
	memcpy(vertexArray2, vertexArray, sizeof(Vector3f) * nVerts);

	//Create nodes
	_nodes = std::vector<Node>(nVerts);
	for (int i = 0; i < nVerts; i++) {

		_nodes[i].meshId = i;

		_nodes[i].position = Eigen::Vector3d(
			(double)vertPos[i].x,
			(double)vertPos[i].y,
			(double)vertPos[i].z);

		_nodes[i].SetMass(vertMass[i]);
		_nodes[i].SetDamping(damping);
		_nodes[i].isFixed = vertIsFixed[i];
	}

	_springs = std::vector<Spring>(nSprings);
	for (int i = 0; i < nSprings; i++)
	{
		int a = springs[2 * i];
		int b = springs[(2 * i) + 1];
		_springs[i] = Spring(&_nodes[a], &_nodes[b]);
		_springs[i].SetStiffness(springStiffness[i]);
		_springs[i].SetDamping(damping / 2.0);
	}

	_faces = std::vector<Face>(this->nFaces);
	for (size_t i = 0; i < this->nFaces; i++)
	{
		int id = 3 * i;
		_faces[i] = Face(&_nodes[triangles[id]], &_nodes[triangles[id + 1]], &_nodes[triangles[id + 2]], dragCoefficient);
	}

	this->dragCoefficient = dragCoefficient;

	this->optimizationSettings = optimizationSettings;

	this->physicsManager = physicsManager;

}

Object::~Object()
{
	delete[] vertexArray;
	delete[] vertexArray2;

	vertexArray = 0;
	vertexArray2 = 0;
}

void Object::Fixnodes(Fixer* f)
{
	for (size_t i = 0; i < nVerts; i++)
	{
		if (f->CheckNodeInside(&_nodes[i])) {
			_nodes[i].isFixed = true;
			_nodes[i].vel = Eigen::Vector3d(0, 0, 0);
		}
	}
}

void Object::Initialize(int* ind)
{
	DebugHelper d = DebugHelper();
	index = *ind;

	// Start scene nodes/edges
	for (int i = 0; i < nVerts; ++i) {
		_nodes[i].Initialize(index + 3 * i);
	}

	for (int i = 0; i < nSprings; ++i)
		_springs[i].Initialize(stiffness, damping);
}

int Object::GetNumDoFs()
{
	return 3 * nVerts;
}

void Object::GetPosition(Eigen::VectorXd* position)
{
	for (int i = 0; i < nVerts; ++i)
		_nodes[i].GetPosition(position);
}

void Object::SetPosition(Eigen::VectorXd* position)
{
	for (int i = 0; i < nVerts; ++i)
		_nodes[i].SetPosition(position);
	for (int i = 0; i < nSprings; ++i)
		_springs[i].UpdateState();
}

void Object::GetVelocity(Eigen::VectorXd* velocity)
{
	for (int i = 0; i < nVerts; ++i)
		_nodes[i].GetVelocity(velocity);
}

void Object::SetVelocity(Eigen::VectorXd* velocity)
{
	for (int i = 0; i < nVerts; ++i)
		_nodes[i].SetVelocity(velocity);
}

void Object::GetForce(Eigen::VectorXd* force, std::vector<Collider*> colliders)
{
	for (int i = 0; i < nVerts; ++i)
		_nodes[i].GetForce(force, colliders);

	for (int i = 0; i < nSprings; ++i)
		_springs[i].GetForce(force);

	for (int i = 0; i < nFaces; ++i)
		_faces[i].GetForce(force);
}

void Object::GetdFdp(Eigen::VectorXd* dforce)
{
	for (int i = 0; i < nSprings; ++i)
		_springs[i].GetdFdstiffness(dforce);
}

void Object::GetdFdp(std::vector<T>* dforce, int springOffset)
{
	for (int i = 0; i < nSprings; ++i)
		_springs[i].GetdFdstiffness(dforce, i + springOffset);
}

void Object::GetForceJacobian(std::vector<T>* derivPos, std::vector<T>* derivVel, std::vector<Collider*> colliders)
{
	for (int i = 0; i < nVerts; ++i)
		_nodes[i].GetForceJacobian(derivPos, derivVel, colliders);

	for (int i = 0; i < nSprings; ++i)
		_springs[i].GetForceJacobian(derivPos, derivVel);

	for (int i = 0; i < nFaces; ++i)
		_faces[i].GetForceJacobian(derivPos, derivVel);
}

void Object::GetFixedIndices(std::vector<bool>* fixedIndices)
{
	for (size_t i = 0; i < nVerts; i++)
	{
		(*fixedIndices)[_nodes[i].index] = _nodes[i].isFixed;
		(*fixedIndices)[_nodes[i].index + 1] = _nodes[i].isFixed;
		(*fixedIndices)[_nodes[i].index + 2] = _nodes[i].isFixed;
	}
}

void Object::UpdateVertices()
{
	for (int i = 0; i < nVerts; i++) {
		vertexArray[i] = Vector3f(
			(float)_nodes[i].position.x(),
			(float)_nodes[i].position.y(),
			(float)_nodes[i].position.z());
	}

	memcpy(vertexArray2, vertexArray, sizeof(Vector3f) * nVerts);

	updated = true;
}

void Object::GetMass(std::vector<T>* mass)
{
	for (int i = 0; i < nVerts; ++i)
		_nodes[i].GetMass(mass);
}

void Object::GetMassInverse(std::vector<T>* massInv)
{
	for (int i = 0; i < nVerts; ++i)
		_nodes[i].GetMassInv(massInv);
}

Vector3f* Object::GetVertices() {
	updated = false;

	return vertexArray2;
}

void Object::SetMass(double mass)
{
	for (size_t i = 0; i < nVerts; i++)
	{
		_nodes[i].SetMass(mass);
	}
}

void Object::SetStiffness(double param)
{
	for (size_t i = 0; i < nSprings; i++)
	{
		_springs[i].SetStiffness(param);
	}
}

void Object::SetMass(Eigen::VectorXd params)
{
	for (size_t i = 0; i < nVerts; i++)
	{
		_nodes[i].SetMass(params(i));
	}
}

void Object::SetStiffness(Eigen::VectorXd params)
{
	for (size_t i = 0; i < nSprings; i++)
	{
		_springs[i].SetStiffness(params(i));
	}
}
