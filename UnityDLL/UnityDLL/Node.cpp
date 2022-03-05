#include "pch.h"
#include "Node.h"

void Node::Initialize(int ind)
{
	index = ind;
}

void Node::GetPosition(Eigen::VectorXd* pos)
{
	pos->coeffRef(index) = position[0];
	pos->coeffRef(index + 1) = position[1];
	pos->coeffRef(index + 2) = position[2];
}

void Node::SetPosition(Eigen::VectorXd* pos)
{
	position = Eigen::Vector3d((*pos)[index], (*pos)[index + 1], (*pos)[index + 2]);
}

void Node::GetVelocity(Eigen::VectorXd* _vel)
{
	(*_vel)[index] = vel[0];
	(*_vel)[index + 1] = vel[1];
	(*_vel)[index + 2] = vel[2];
}

void Node::SetVelocity(Eigen::VectorXd* _vel)
{
	vel = Eigen::Vector3d((*_vel)[index], (*_vel)[index + 1], (*_vel)[index + 2]);
}

void Node::GetMass(Eigen::MatrixXd* Mass)
{
	(*Mass)(index, index) = mass;
	(*Mass)(index + 1, index + 1) = mass;
	(*Mass)(index + 2, index + 2) = mass;
}

void Node::GetMass(std::vector<T>* massTripletVector)
{
	(*massTripletVector).push_back(T(index, index, mass));
	(*massTripletVector).push_back(T(index + 1, index + 1, mass));
	(*massTripletVector).push_back(T(index + 2, index + 2, mass));
}

void Node::GetMassInverse(Eigen::MatrixXd* massInv)
{
	(*massInv)(index, index) = 1.0 / mass;
	(*massInv)(index + 1, index + 1) = 1.0 / mass;
	(*massInv)(index + 2, index + 2) = 1.0 / mass;
}

void Node::GetMassInv(std::vector<T>* massTripletVector)
{
	(*massTripletVector).push_back(T(index, index, 1.0 / mass));
	(*massTripletVector).push_back(T(index + 1, index + 1, 1.0 / mass));
	(*massTripletVector).push_back(T(index + 2, index + 2, 1.0 / mass));
}

void Node::GetForce(Eigen::VectorXd* force)
{
	(*force)[index] = 0;
	(*force)[index + 1] = mass * -9.81;
	(*force)[index + 2] = 0;
}

void Node::GetForceJacobian(std::vector<T>* derivPos, std::vector<T>* derivVel)
{
	(*derivPos).push_back(T(index + 1, index + 1, mass * -9.81));
}

void Node::FixVector(Eigen::VectorXd* v)
{
	if (isFixed)
	{
		(*v)[index] = 0.0;
		(*v)[index + 1] = 0.0;
		(*v)[index + 2] = 0.0;
	}
}

void Node::FixMatrix(Eigen::MatrixXd* M)
{
	if (isFixed)
	{
		for (int i = 0; i < (*M).rows(); i++)
		{
			(*M)(index, i) = 0.0;
			(*M)(index + 1, i) = 0.0;
			(*M)(index + 2, i) = 0.0;
			(*M)(i, index) = 0.0;
			(*M)(i, index + 1) = 0.0;
			(*M)(i, index + 2) = 0.0;
		}
		(*M)(index, index) = 1.0;
		(*M)(index + 1, index + 1) = 1.0;
		(*M)(index + 2, index + 2) = 1.0;
	}
}

void Node::FixMatrix(SpMat* M)
{
	
}

void Node::UpdateOld(float time, float h)
{
	if (this->isFixed)
		return;

	ComputeForces();
	acc = force / (volume * density);
	if (this->isFixed)
		return;

	vel += acc * h;
	position += vel * h;
	force = Eigen::Vector3d(0, 0, 0);
}
