#include "pch.h"
#include "Object.h"
#include <set>
#include <map>
#include "DebugHelper.h"

Object::Object(Vector3f pos, Vector3f* vertices, int nVerts, int* _triangles, int nTriangles, float stiffness, float mass)
{
	this->stiffness = stiffness;
	this->mass = mass;
	nVertices = nVerts;
	//positon = Eigen::Vector3d((double)pos.x, (double)pos.y, (double)pos.z);

	vertexArray = new Vector3f[nVertices];
	vertexArray2 = new Vector3f[nVertices];

	memcpy(vertexArray, vertices, sizeof(Vector3f) * nVertices);
	memcpy(vertexArray2, vertexArray, sizeof(Vector3f) * nVertices);

	//Create nodes
	nodeArray = new Node[nVertices];
	for (int i = 0; i < nVertices; i++) {
		vertexArray[i] = (Vector3f)vertices[i];
		nodeArray[i].position = Eigen::Vector3d(
			(double)vertexArray[i].x,
			(double)vertexArray[i].y,
			(double)vertexArray[i].z);

		nodeArray[i].meshId = i;
		nodeArray[i].damping = damping;
	}

	//Creating springs
	int* triangles = new int[nTriangles];
	memcpy(triangles, _triangles, sizeof(int) * nTriangles);

	std::map<std::string, Spring> springMap;

	Spring* newspringArray = new Spring[3];
	for (int i = 0; i < nTriangles; i += 3)
	{
		newspringArray[0] = Spring(&nodeArray[triangles[i]], &nodeArray[triangles[i + 1]], stiffness, damping);
		newspringArray[1] = Spring(&nodeArray[triangles[i]], &nodeArray[triangles[i + 2]], stiffness, damping);
		newspringArray[2] = Spring(&nodeArray[triangles[i + 1]], &nodeArray[triangles[i + 2]], stiffness, damping);

		newspringArray[0].oppositeNode = &nodeArray[triangles[i + 2]];
		newspringArray[1].oppositeNode = &nodeArray[triangles[i + 1]];
		newspringArray[2].oppositeNode = &nodeArray[triangles[i]];

		Eigen::Vector3d side1 = nodeArray[triangles[i + 1]].position - nodeArray[triangles[i]].position;
		Eigen::Vector3d side2 = nodeArray[triangles[i + 2]].position - nodeArray[triangles[i]].position;

		float area = 0.5f * (float)side1.cross(side2).norm();

		for (int x = 0; x < 3; x++)
		{//Add the proper mass to each node

			nodeArray[triangles[i + x]].volume += area / 3.0;

			std::map<std::string, Spring>::iterator it = springMap.find(newspringArray[x].id);

			if (it != springMap.end())
			{
				Node* a = newspringArray[x].oppositeNode;
				Node* b = springMap[newspringArray[x].id].oppositeNode;

				Spring extraSpring = Spring(a, b, stiffness / 2, damping);
				it = springMap.find(extraSpring.id);
				if (it != springMap.end())
				{
					springMap[newspringArray[x].id].volume += area;
				}
				else {
					springMap[extraSpring.id] = extraSpring;
				}
			}
			else
			{
				//La arista no está en el diccionario
				springMap[newspringArray[x].id] = newspringArray[x];
			}

			springMap[newspringArray[x].id].volume += area;
		}
	}

	nSprings = (int)springMap.size();
	springArray = new Spring[nSprings];

	std::map<std::string, Spring>::iterator it;
	int i = 0;
	for (it = springMap.begin(); it != springMap.end(); ++it) {
		springArray[i] = springMap[it->first];
		i++;
	}


	delete[] newspringArray;
	delete[] triangles;
}

Object::~Object()
{
	delete[] vertexArray;
	delete[] vertexArray2;
	delete[] nodeArray;

	vertexArray = 0;
	vertexArray2 = 0;
	nodeArray = 0;
}

void Object::Update(float time, float h)
{
	for (int i = 0; i < nSprings; i++)
		springArray[i].ComputeForces();

	for (int i = 0; i < nVertices; i++)
		nodeArray[i].UpdateOld(time, h);

	updated = true;
}

void Object::SetNodeMass(float newMass)
{
	for (size_t i = 0; i < nVertices; i++)
	{
		nodeArray[i].mass = newMass;
	}
}

void Object::FixnodeArray(Fixer* f)
{
	for (size_t i = 0; i < nVertices; i++)
	{
		if (f->CheckNodeInside(&nodeArray[i])) {
			nodeArray[i].isFixed = true;
			nodeArray[i].vel = Eigen::Vector3d(0,0,0);
		}
	}
}

void Object::Initialize(int* ind)
{
	index = *ind;

	// Start scene nodes/edges
	for (int i = 0; i < nVertices; ++i)
		nodeArray[i].Initialize(index + 3 * i); // Prepare

	for (int i = 0; i < nSprings; ++i)
		springArray[i].Initialize(stiffness, damping); // Prepare
}

int Object::GetNumDoFs()
{
	return 3 * nVertices;
}

void Object::GetPosition(Eigen::VectorXd* position)
{
	for (int i = 0; i < nVertices; ++i)
		nodeArray[i].GetPosition(position);
}

void Object::SetPosition(Eigen::VectorXd* position)
{
	for (int i = 0; i < nVertices; ++i)
		nodeArray[i].SetPosition(position);
	for (int i = 0; i < nSprings; ++i)
		springArray[i].UpdateState();
}

void Object::GetVelocity(Eigen::VectorXd* velocity)
{
	for (int i = 0; i < nVertices; ++i)
		nodeArray[i].GetVelocity(velocity);
}

void Object::SetVelocity(Eigen::VectorXd* velocity)
{
	for (int i = 0; i < nVertices; ++i)
		nodeArray[i].SetVelocity(velocity);
}

void Object::GetForce(Eigen::VectorXd* force)
{
	for (int i = 0; i < nVertices; ++i)
		nodeArray[i].GetForce(force);

	for (int i = 0; i < nSprings; ++i)
		springArray[i].GetForce(force);
}

#include <chrono>

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;
void Object::GetForceJacobian(std::vector<T>* derivPos, std::vector<T>* derivVel)
{
	//DebugHelper debug = DebugHelper();
	//std::string a = std::to_string(nVertices) + " vertices";
	//debug.RecordTime(a);
	for (int i = 0; i < nVertices; ++i)
		nodeArray[i].GetForceJacobian(derivPos, derivVel);

	//a = std::to_string(nSprings) + " springs";
	//debug.RecordTime(a);
	for (int i = 0; i < nSprings; ++i)
		springArray[i].GetForceJacobian(derivPos, derivVel);

	//debug.PrintTimes("forces");
}

void Object::GetMass(Eigen::MatrixXd* mass)
{
	for (int i = 0; i < nVertices; ++i)
		nodeArray[i].GetMass(mass);
}

void Object::GetFixedIndices(std::vector<bool>* fixedIndices)
{
	for (size_t i = 0; i < nVertices; i++)
	{
		(*fixedIndices)[nodeArray[i].index] = nodeArray[i].isFixed;
		(*fixedIndices)[nodeArray[i].index + 1] = nodeArray[i].isFixed;
		(*fixedIndices)[nodeArray[i].index + 2] = nodeArray[i].isFixed;
	}
}

void Object::UpdateVertices()
{
	for (int i = 0; i < nVertices; i++) {
		vertexArray[i] = Vector3f(
			(float)nodeArray[i].position.x(),
			(float)nodeArray[i].position.y(),
			(float)nodeArray[i].position.z());
	}

	memcpy(vertexArray2, vertexArray, sizeof(Vector3f) * nVertices);

	updated = true;
}

void Object::GetMass(std::vector<T>* mass)
{
	for (int i = 0; i < nVertices; ++i)
		nodeArray[i].GetMass(mass);
}

void Object::GetMassInverse(Eigen::MatrixXd* massInv)
{
	for (int i = 0; i < nVertices; ++i)
		nodeArray[i].GetMassInverse(massInv);
}

void Object::GetMassInverse(std::vector<T>* massInv)
{
	for (int i = 0; i < nVertices; ++i)
		nodeArray[i].GetMassInv(massInv);
}

void Object::FixVector(Eigen::VectorXd* v)
{
	for (int i = 0; i < nVertices; i++)
	{
		nodeArray[i].FixVector(v);
	}
}

void Object::FixMatrix(Eigen::MatrixXd* M)
{
	for (int i = 0; i < nVertices; i++)
	{
		nodeArray[i].FixMatrix(M);
	}
}

void Object::FixMatrix(SpMat* M)
{
	for (int i = 0; i < nVertices; i++)
		nodeArray[i].FixMatrix(M);
}

Vector3f* Object::GetVertices() {
	updated = false;

	return vertexArray2;
}
