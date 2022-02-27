#include "pch.h"
#include "Node.h"



/*void Node::Update(float time, float h)
{
	if (this->isFixed)
		return;

	UpdateSymplectic(h);
}

void Node::Initialize(int ind, float mass, float damping)
{
	// TO BE COMPLETED
}

// Get Force
void Node::GetForce(Eigen::VectorXd* force)
{
	// TO BE COMPLETED
}

// Get Force Jacobian
void Node::GetForceJacobian(Eigen::MatrixXd* dFdx, Eigen::MatrixXd* dFdv)
{
	// TO BE COMPLETED
}

void Node::GetPosition(Eigen::VectorXd* pos)
{
	Eigen::VectorXd vector = *pos;
	vector[index] = position[0];
	vector[index + 1] = position[1];
	vector[index + 2] = position[2];
}

void Node::SetPosition(Eigen::VectorXd* pos)
{
	Eigen::VectorXd vector = *pos;
	position = Eigen::Vector3d(vector[index], vector[index + 1], vector[index + 2]);
}

void Node::GetVelocity(Eigen::VectorXd* Vel)
{
	Eigen::RowVectorXd vector = *Vel;
	vector[index] = vel[0];
	vector[index + 1] = vel[1];
	vector[index + 2] = vel[2];
}

void Node::SetVelocity(Eigen::VectorXd* Vel)
{
	Eigen::RowVectorXd vector = *Vel;
	vel = Eigen::Vector3d(vector[index], vector[index + 1], vector[index + 2]);
}

void Node::GetMass(Eigen::MatrixXd* Mass)
{
	Eigen::MatrixXd matrix = *Mass;
	matrix(index, index) = mass;
	matrix(index + 1, index + 1) = mass;
	matrix(index + 2, index + 2) = mass;
}

void Node::GetMassInverse(Eigen::MatrixXd* massInv)
{
	Eigen::MatrixXd matrix = *massInv;
	matrix(index, index) = 1.0 / mass;
	matrix(index + 1, index + 1) = 1.0 / mass;
	matrix(index + 2, index + 2) = 1.0 / mass;
}

void Node::FixVector(Eigen::VectorXd* v)
{
	if (isFixed)
	{
		v[index] = 0.0;
		v[index + 1] = 0.0;
		v[index + 2] = 0.0;
	}
}

void Node::FixMatrix(Eigen::MatrixXd* M)
{
	if (isFixed)
	{
		for (int i = 0; i < M.rows(); i++)
		{
			M(index, i) = 0.0;
			M(index + 1, i) = 0.0;
			M(index + 2, i) = 0.0;
			M(i, index) = 0.0;
			M(i, index + 1) = 0.0;
			M(i, index + 2) = 0.0;
		}
		M(index, index) = 1.0;
		M(index + 1, index + 1) = 1.0;
		M(index + 2, index + 2) = 1.0;
	}
}*/
