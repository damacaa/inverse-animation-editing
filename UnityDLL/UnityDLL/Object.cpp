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
	_nodes = std::vector<Node>(nVerts);
	//nodes = new Node[nVerts];
	for (int i = 0; i < nVerts; i++) {
		vertexArray[i] = (Vector3f)vertices[i];
		_nodes[i].position = Eigen::Vector3d(
			(double)vertexArray[i].x,
			(double)vertexArray[i].y,
			(double)vertexArray[i].z);

		_nodes[i].meshId = i;
		_nodes[i].damping = damping;
	}

	//Creating springs
	int* triangles = new int[nTriangles];
	memcpy(triangles, _triangles, sizeof(int) * nTriangles);

	std::map<std::string, Spring> springMap;

	Spring* newsprings = new Spring[3];
	for (int i = 0; i < nTriangles; i += 3)
	{
		newsprings[0] = Spring(&_nodes[triangles[i]], &_nodes[triangles[i + 1]]);
		newsprings[1] = Spring(&_nodes[triangles[i]], &_nodes[triangles[i + 2]]);
		newsprings[2] = Spring(&_nodes[triangles[i + 1]], &_nodes[triangles[i + 2]]);

		newsprings[0].oppositeNode = &_nodes[triangles[i + 2]];
		newsprings[1].oppositeNode = &_nodes[triangles[i + 1]];
		newsprings[2].oppositeNode = &_nodes[triangles[i]];

		newsprings[0].stiffness = stiffness;
		newsprings[1].stiffness = stiffness;
		newsprings[2].stiffness = stiffness;

		Eigen::Vector3d side1 = _nodes[triangles[i + 1]].position - _nodes[triangles[i]].position;
		Eigen::Vector3d side2 = _nodes[triangles[i + 2]].position - _nodes[triangles[i]].position;

		double area = 0.5 * side1.cross(side2).norm();

		for (int x = 0; x < 3; x++)
		{//Add the proper mass to each node

			_nodes[triangles[i + x]].volume += area / 3.0;

			std::map<std::string, Spring>::iterator it = springMap.find(newsprings[x].id);

			if (it != springMap.end())
			{
				Node* a = newsprings[x].oppositeNode;
				Node* b = springMap[newsprings[x].id].oppositeNode;

				Spring extraSpring = Spring(a, b);//stiffness / 10.0/////////////////////////////////////////
				extraSpring.stiffness = stiffness / 2.0;
				extraSpring.volume += 2.0 * area;
				springMap[extraSpring.id] = extraSpring;
			}
			else
			{
				//La arista no está en el diccionario
				springMap[newsprings[x].id] = newsprings[x];
			}

			springMap[newsprings[x].id].volume += area;
		}
	}


	nSprings = (int)springMap.size();
	//springs = new Spring[nSprings];
	_springs = std::vector<Spring>(nSprings);

	std::map<std::string, Spring>::iterator it;
	int i = 0;
	for (it = springMap.begin(); it != springMap.end(); ++it) {
		_springs[i] = springMap[it->first];
		i++;
	}

	//Setting simulation parameters
	for (int i = 0; i < nVerts; i++) {
		_nodes[i].SetMass(density);
		_nodes[i].SetDamping(damping);
	}

	for (size_t i = 0; i < nSprings; i++)
	{
		_springs[i].SetStiffness(_springs[i].stiffness);
		_springs[i].SetDamping(damping);
	}

	delete[] newsprings;
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
	_nodes =  std::vector<Node>(nVerts);
	for (int i = 0; i < nVerts; i++) {

		_nodes[i].meshId = i;

		_nodes[i].position = Eigen::Vector3d(
			(double)vertPos[i].x,
			(double)vertPos[i].y,
			(double)vertPos[i].z);

		//nodes[i].volume = vertVolume[i];
		_nodes[i].SetMass(vertMass);
		_nodes[i].SetDamping(damping);
		_nodes[i].isFixed = vertIsFixed[i];
	}

	_springs = std::vector<Spring>(nSprings);
	for (int i = 0; i < nSprings; i++)
	{
		int a = springs[2 * i];
		int b = springs[(2 * i) + 1];
		_springs[i] = Spring(&_nodes[a], &_nodes[b]);
		//springs[i].volume = springVolume[i];
		_springs[i].SetStiffness(springStiffness[i]);
		_springs[i].SetDamping(damping/2.0);
	}
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
		//d.PrintValue("", std::to_string(i));
		_nodes[i].Initialize(index + 3 * i); // Prepare
	}


	for (int i = 0; i < nSprings; ++i)
		_springs[i].Initialize(stiffness, damping); // Prepare
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

void Object::GetForce(Eigen::VectorXd* force)
{
	for (int i = 0; i < nVerts; ++i)
		_nodes[i].GetForce(force);

	for (int i = 0; i < nSprings; ++i)
		_springs[i].GetForce(force);
}

void Object::GetdFdp(Eigen::VectorXd* dforce)
{
	for (int i = 0; i < nSprings; ++i)
		_springs[i].GetdFdstiffness(dforce);
}

void Object::GetdFdp(std::vector<T>* dforce)
{
	for (int i = 0; i < nSprings; ++i)
		_springs[i].GetdFdstiffness(dforce);
}

void Object::GetForceJacobian(std::vector<T>* derivPos, std::vector<T>* derivVel)
{
	//DebugHelper debug = DebugHelper();
	//std::string a = std::to_string(nVertices) + " vertices";
	//debug.RecordTime(a);
	for (int i = 0; i < nVerts; ++i)
		_nodes[i].GetForceJacobian(derivPos, derivVel);

	//a = std::to_string(nSprings) + " springs";
	//debug.RecordTime(a);
	for (int i = 0; i < nSprings; ++i)
		_springs[i].GetForceJacobian(derivPos, derivVel);

	//debug.PrintTimes("forces");
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
