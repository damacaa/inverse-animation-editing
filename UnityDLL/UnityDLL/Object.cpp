#include "pch.h"
#include "Object.h"
#include <set>
#include <map>



Object::Object(Vector3f pos, Vector3f* vertices, int nVerts, int* _triangles, int nTriangles)
{
	nVertices = nVerts;
	positon = Eigen::Vector3d((double)pos.x, (double)pos.y, (double)pos.z);

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
		nodeArray[i].id = std::to_string(i);
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

		double area = 0.5f * side1.cross(side2).norm();

		//float area = 0.1f;

		for (int x = 0; x < 3; x++)
		{//Add the proper mass to each node

			nodeArray[triangles[i + x]].volume += area / 3;

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
	float newH = h / (float)substeps;

	for (size_t i = 0; i < substeps; i++)
	{

		for (int i = 0; i < nSprings; i++)
		{
			springArray[i].ComputeForces();
		}

		for (int i = 0; i < nVertices; i++) {
			nodeArray[i].Update(time, newH);
		}
	}

	updated = true;
}

/*Vector3f* Object::GetVertices()
{
	
}*/

void Object::FixnodeArray(Fixer* f)
{
	for (size_t i = 0; i < nVertices; i++)
	{
		if (f->CheckNodeInside(&nodeArray[i])) {
			nodeArray[i].isFixed = true;
		}
	}
}

/*
void Object::FixNodes(Fixer* f)
{
}



void Object::Initialize(int ind)
{
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

void Object::GetForceJacobian(Eigen::MatrixXd* dFdx, Eigen::MatrixXd* dFdv)
{
	for (int i = 0; i < nVertices; ++i)
		nodeArray[i].GetForceJacobian(dFdx, dFdv);
	for (int i = 0; i < nSprings; ++i)
		springArray[i].GetForceJacobian(dFdx, dFdv);
}

void Object::GetMass(Eigen::MatrixXd* mass)
{
	for (int i = 0; i < nVertices; ++i)
		nodeArray[i].GetMass(mass);
}

void Object::GetMassInverse(Eigen::MatrixXd* massInv)
{
	for (int i = 0; i < nVertices; ++i)
		nodeArray[i].GetMassInverse(massInv);
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
}*/
