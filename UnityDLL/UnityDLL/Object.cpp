#include "pch.h"
#include "Object.h"
#include <set>
#include <map>
#include "DebugHelper.h"

Object::Object(Vector3f* vertices, int nVerts, int* _triangles, int nTriangles, float stiffness, float density)
{
	this->stiffness = stiffness;
	this->density = density;
	this->nVerts = nVerts;
	//positon = Eigen::Vector3d((double)pos.x, (double)pos.y, (double)pos.z);

	vertexArray = new Vector3f[nVerts];
	vertexArray2 = new Vector3f[nVerts];

	memcpy(vertexArray, vertices, sizeof(Vector3f) * nVerts);
	memcpy(vertexArray2, vertexArray, sizeof(Vector3f) * nVerts);

	//Create nodes
	nodeArray = new Node[nVerts];
	for (int i = 0; i < nVerts; i++) {
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
		newspringArray[0] = Spring(&nodeArray[triangles[i]], &nodeArray[triangles[i + 1]]);
		newspringArray[1] = Spring(&nodeArray[triangles[i]], &nodeArray[triangles[i + 2]]);
		newspringArray[2] = Spring(&nodeArray[triangles[i + 1]], &nodeArray[triangles[i + 2]]);

		newspringArray[0].oppositeNode = &nodeArray[triangles[i + 2]];
		newspringArray[1].oppositeNode = &nodeArray[triangles[i + 1]];
		newspringArray[2].oppositeNode = &nodeArray[triangles[i]];

		newspringArray[0].stiffness = stiffness;
		newspringArray[1].stiffness = stiffness;
		newspringArray[2].stiffness = stiffness;

		Eigen::Vector3d side1 = nodeArray[triangles[i + 1]].position - nodeArray[triangles[i]].position;
		Eigen::Vector3d side2 = nodeArray[triangles[i + 2]].position - nodeArray[triangles[i]].position;

		double area = 0.5 * side1.cross(side2).norm();

		for (int x = 0; x < 3; x++)
		{//Add the proper mass to each node

			nodeArray[triangles[i + x]].volume += area / 3.0;

			std::map<std::string, Spring>::iterator it = springMap.find(newspringArray[x].id);

			if (it != springMap.end())
			{
				Node* a = newspringArray[x].oppositeNode;
				Node* b = springMap[newspringArray[x].id].oppositeNode;

				Spring extraSpring = Spring(a, b);//stiffness / 10.0/////////////////////////////////////////
				extraSpring.stiffness = stiffness / 2.0;
				extraSpring.volume += 2.0 * area;
				springMap[extraSpring.id] = extraSpring;
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

	//Setting simulation parameters
	for (int i = 0; i < nVerts; i++) {
		nodeArray[i].SetMass(density);
		nodeArray[i].SetDamping(damping);
	}

	for (size_t i = 0; i < nSprings; i++)
	{
		springArray[i].SetStiffness(springArray[i].stiffness);
		springArray[i].SetDamping(damping);
	}

	delete[] newspringArray;
	delete[] triangles;
}

Object::Object(Vector3f* vertPos, bool* vertIsFixed, float vertMass, int nVerts, int* springs, float* springStiffness, int nSprings, float damping)
{
	//this->density = density;
	this->stiffness = stiffness;
	this->damping = damping;

	this->nVerts = nVerts;
	this->nSprings = nSprings;

	vertexArray = new Vector3f[nVerts];
	vertexArray2 = new Vector3f[nVerts];

	memcpy(vertexArray, vertPos, sizeof(Vector3f) * nVerts);
	memcpy(vertexArray2, vertexArray, sizeof(Vector3f) * nVerts);

	//Create nodes
	nodeArray = new Node[nVerts];
	for (int i = 0; i < nVerts; i++) {

		nodeArray[i].meshId = i;

		nodeArray[i].position = Eigen::Vector3d(
			(double)vertPos[i].x,
			(double)vertPos[i].y,
			(double)vertPos[i].z);

		//nodeArray[i].volume = vertVolume[i];
		nodeArray[i].SetMass(vertMass);
		nodeArray[i].SetDamping(damping);
		nodeArray[i].isFixed = vertIsFixed[i];
	}

	springArray = new Spring[nSprings];
	for (int i = 0; i < nSprings; i++)
	{
		int a = springs[2 * i];
		int b = springs[(2 * i) + 1];
		springArray[i] = Spring(&nodeArray[a], &nodeArray[b]);
		//springArray[i].volume = springVolume[i];
		springArray[i].SetStiffness(springStiffness[i]);
		springArray[i].SetDamping(damping);
	}
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

void Object::FixnodeArray(Fixer* f)
{
	for (size_t i = 0; i < nVerts; i++)
	{
		if (f->CheckNodeInside(&nodeArray[i])) {
			nodeArray[i].isFixed = true;
			nodeArray[i].vel = Eigen::Vector3d(0, 0, 0);
		}
	}
}

void Object::Initialize(int* ind)
{
	DebugHelper d = DebugHelper();
	index = *ind;

	// Start scene nodes/edges
	for (int i = 0; i < nVerts; ++i) {
		d.PrintValue("", std::to_string(i));
		nodeArray[i].Initialize(index + 3 * i); // Prepare
	}


	for (int i = 0; i < nSprings; ++i)
		springArray[i].Initialize(stiffness, damping); // Prepare
}

int Object::GetNumDoFs()
{
	return 3 * nVerts;
}

void Object::GetPosition(Eigen::VectorXd* position)
{
	for (int i = 0; i < nVerts; ++i)
		nodeArray[i].GetPosition(position);
}

void Object::SetPosition(Eigen::VectorXd* position)
{
	for (int i = 0; i < nVerts; ++i)
		nodeArray[i].SetPosition(position);
	for (int i = 0; i < nSprings; ++i)
		springArray[i].UpdateState();
}

void Object::GetVelocity(Eigen::VectorXd* velocity)
{
	for (int i = 0; i < nVerts; ++i)
		nodeArray[i].GetVelocity(velocity);
}

void Object::SetVelocity(Eigen::VectorXd* velocity)
{
	for (int i = 0; i < nVerts; ++i)
		nodeArray[i].SetVelocity(velocity);
}

void Object::GetForce(Eigen::VectorXd* force)
{
	for (int i = 0; i < nVerts; ++i)
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
	for (int i = 0; i < nVerts; ++i)
		nodeArray[i].GetForceJacobian(derivPos, derivVel);

	//a = std::to_string(nSprings) + " springs";
	//debug.RecordTime(a);
	for (int i = 0; i < nSprings; ++i)
		springArray[i].GetForceJacobian(derivPos, derivVel);

	//debug.PrintTimes("forces");
}

void Object::GetFixedIndices(std::vector<bool>* fixedIndices)
{
	for (size_t i = 0; i < nVerts; i++)
	{
		(*fixedIndices)[nodeArray[i].index] = nodeArray[i].isFixed;
		(*fixedIndices)[nodeArray[i].index + 1] = nodeArray[i].isFixed;
		(*fixedIndices)[nodeArray[i].index + 2] = nodeArray[i].isFixed;
	}
}

void Object::UpdateVertices()
{
	for (int i = 0; i < nVerts; i++) {
		vertexArray[i] = Vector3f(
			(float)nodeArray[i].position.x(),
			(float)nodeArray[i].position.y(),
			(float)nodeArray[i].position.z());
	}

	memcpy(vertexArray2, vertexArray, sizeof(Vector3f) * nVerts);

	updated = true;
}

void Object::GetMass(std::vector<T>* mass)
{
	for (int i = 0; i < nVerts; ++i)
		nodeArray[i].GetMass(mass);
}

void Object::GetMassInverse(std::vector<T>* massInv)
{
	for (int i = 0; i < nVerts; ++i)
		nodeArray[i].GetMassInv(massInv);
}

Vector3f* Object::GetVertices() {
	updated = false;

	return vertexArray2;
}

void Object::SetMass(double mass)
{
	for (size_t i = 0; i < nVerts; i++)
	{
		nodeArray[i].SetMass(mass);
	}
}
