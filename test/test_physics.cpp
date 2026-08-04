#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include "PhysicsManager.h"
#include "Node.h"
#include "Spring.h"

// --------------- Spring unit tests ---------------

TEST(SpringTest, force_at_rest_is_zero)
{
	Node a, b;
	a.position = Eigen::Vector3d(0, 0, 0);
	b.position = Eigen::Vector3d(0, 1, 0);
	Spring s(&a, &b);               // length == length0 == 1
	s.SetStiffness(100.0);
	s.Initialize(100.0, 0.0);

	Eigen::VectorXd force = Eigen::VectorXd::Zero(6);
	s.GetForce(&force);

	for (int i = 0; i < 6; ++i)
		EXPECT_DOUBLE_EQ(force(i), 0.0);
}

TEST(SpringTest, force_when_stretched)
{
	Node a, b;
	a.index = 0;
	b.index = 3;
	a.position = Eigen::Vector3d(0, 0, 0);
	b.position = Eigen::Vector3d(0, 2, 0);   // length == 2, rest = 2
	Spring s(&a, &b);
	s.SetStiffness(100.0);
	s.Initialize(100.0, 0.0);

	// manually stretch to length 3: a keeps (0,0,0), b goes to (0,3,0)
	a.position = Eigen::Vector3d(0, 0, 0);
	b.position = Eigen::Vector3d(0, 3, 0);
	s.UpdateState();

	// dir = (0,0,0) - (0,3,0) = (0, -1, 0)  normalized
	// f   = -100 * 1 * (0, -1, 0) = (0, 100, 0)
	//    on A: +(0,100,0)   on B: -(0,100,0)
	Eigen::VectorXd force = Eigen::VectorXd::Zero(6);
	s.GetForce(&force);

	EXPECT_NEAR(force(0),   0.0, 1e-10);
	EXPECT_NEAR(force(1), 100.0, 1e-10);
	EXPECT_NEAR(force(2),   0.0, 1e-10);
	EXPECT_NEAR(force(3),   0.0, 1e-10);
	EXPECT_NEAR(force(4), -100.0, 1e-10);
	EXPECT_NEAR(force(5),   0.0, 1e-10);
}

TEST(SpringTest, force_jacobian_position_no_damping)
{
	// With zero damping, analytical dF/dx is the pure spring derivative.
	// The damping contribution to dF/dx (through epsilon) is not implemented.
	Node a, b;
	a.position = Eigen::Vector3d(0.1, 0.0, 0.0);
	b.position = Eigen::Vector3d(0.0, 1.1, 0.0);
	a.index = 0;
	b.index = 3;
	Spring s(&a, &b);
	s.SetStiffness(50.0);
	s.SetDamping(0.0);
	s.Initialize(50.0, 0.0);

	std::vector<Eigen::Triplet<double>> dFdx, dFdv;
	s.GetForceJacobian(&dFdx, &dFdv);

	SpMat Jx(6, 6);
	Jx.setFromTriplets(dFdx.begin(), dFdx.end(),
		[](const double& a, const double& b) { return a + b; });

	const double eps = 1e-5;
	Eigen::VectorXd x0(6);
	x0 << a.position, b.position;

	for (int k = 0; k < 6; ++k) {
		Eigen::VectorXd xp = x0, xm = x0;
		xp(k) += eps;
		xm(k) -= eps;
		a.position = xp.head<3>();
		b.position = xp.tail<3>();
		s.UpdateState();
		Eigen::VectorXd f_plus = Eigen::VectorXd::Zero(6);
		s.GetForce(&f_plus);

		a.position = xm.head<3>();
		b.position = xm.tail<3>();
		s.UpdateState();
		Eigen::VectorXd f_minus = Eigen::VectorXd::Zero(6);
		s.GetForce(&f_minus);

		Eigen::VectorXd fd_col = (f_plus - f_minus) / (2.0 * eps);
		Eigen::VectorXd an_col = Jx.col(k);
		for (int r = 0; r < 6; ++r) {
			double atol = std::max(0.015, 0.015 * std::abs(an_col(r)));
			EXPECT_NEAR(an_col(r), fd_col(r), atol) << "  row " << r << " col " << k;
		}
	}
}

TEST(SpringTest, force_jacobian_velocity)
{
	Node a, b;
	a.position = Eigen::Vector3d(0.1, 0.0, 0.0);
	a.vel     = Eigen::Vector3d(0.0, 0.5, 0.0);
	b.position = Eigen::Vector3d(0.0, 1.1, 0.0);
	b.vel     = Eigen::Vector3d(0.3, 0.0, 0.0);
	a.index = 0;
	b.index = 3;
	Spring s(&a, &b);
	s.SetStiffness(50.0);
	s.SetDamping(0.1);
	s.Initialize(50.0, 0.1);

	std::vector<Eigen::Triplet<double>> dFdx, dFdv;
	s.GetForceJacobian(&dFdx, &dFdv);

	SpMat Jv(6, 6);
	Jv.setFromTriplets(dFdv.begin(), dFdv.end(),
		[](const double& a, const double& b) { return a + b; });

	const double eps = 1e-6;
	Eigen::VectorXd v0(6);
	v0 << a.vel, b.vel;

	for (int k = 0; k < 6; ++k) {
		Eigen::VectorXd vp = v0, vm = v0;
		vp(k) += eps;
		vm(k) -= eps;
		a.vel = vp.head<3>();
		b.vel = vp.tail<3>();
		Eigen::VectorXd fv_plus = Eigen::VectorXd::Zero(6);
		s.GetForce(&fv_plus);

		a.vel = vm.head<3>();
		b.vel = vm.tail<3>();
		Eigen::VectorXd fv_minus = Eigen::VectorXd::Zero(6);
		s.GetForce(&fv_minus);

		Eigen::VectorXd fdv_col = (fv_plus - fv_minus) / (2.0 * eps);
		Eigen::VectorXd anv_col = Jv.col(k);
		for (int r = 0; r < 6; ++r)
			EXPECT_NEAR(anv_col(r), fdv_col(r), 1e-3);
	}
}

// -------------- PhysicsManager integration tests ---------------

TEST(PhysicsManagerTest, free_fall)
{
	// Single unconstrained node under gravity, symplectic Euler.
	PhysicsManager pm(PhysicsManager::Integration::Symplectic, 1e-8);
	Vector3f verts[1] = { Vector3f(0, 10, 0) };
	bool fixed[1] = { false };
	float mass[1] = { 1.0f };
	int dummyTri = 0;

	pm.AddObject(verts, fixed, mass, 1,
		nullptr, nullptr, 0,
		&dummyTri, 0, 0.0, 0.0, "nn");
	pm.Start();

	float h = 0.01f;
	for (int i = 0; i < 100; ++i)
		pm.UpdatePhysics(i * h, h);

	int count = 0;
	Vector3f* v = pm.GetVertices(0, &count);
	ASSERT_EQ(count, 1);
	// y = y0 - 0.5 g t^2  (analytic: 10 - 4.905 = 5.095)
	// Symplectic Euler with h=0.01 gives ~5.046 (O(h) error is expected)
	EXPECT_NEAR(v[0].y, 5.046, 0.001);
	EXPECT_NEAR(v[0].x, 0.0, 1e-6);
}

TEST(PhysicsManagerTest, fixed_node_does_not_move)
{
	PhysicsManager pm(PhysicsManager::Integration::Implicit, 1e-6);
	Vector3f verts[1] = { Vector3f(5, -3, 7) };
	bool fixed[1] = { true };
	float mass[1] = { 1.0f };
	int dummyTri = 0;

	pm.AddObject(verts, fixed, mass, 1,
		nullptr, nullptr, 0,
		&dummyTri, 0, 0.0, 0.0, "nn");
	pm.Start();

	for (int i = 0; i < 500; ++i)
		pm.UpdatePhysics(i * 0.01f, 0.01f);

	int count = 0;
	Vector3f* v = pm.GetVertices(0, &count);
	EXPECT_FLOAT_EQ(v[0].x, 5.0f);
	EXPECT_FLOAT_EQ(v[0].y, -3.0f);
	EXPECT_FLOAT_EQ(v[0].z, 7.0f);
}

TEST(PhysicsManagerTest, implicit_spring_equilibrium)
{
	// 2 nodes, 1 spring, top fixed. Free node should settle near mg/k stretch.
	PhysicsManager pm(PhysicsManager::Integration::Implicit, 1e-4);
	Vector3f verts[2] = { Vector3f(0, 1, 0), Vector3f(0, 0, 0) };
	bool fixed[2] = { true, false };
	float mass[2] = { 1.0f, 1.0f };
	int springs[2] = { 0, 1 };
	float stiffness[1] = { 200.0f };
	int dummyTri = 0;

	pm.AddObject(verts, fixed, mass, 2,
		springs, stiffness, 1,
		&dummyTri, 0, 0.0, 0.5, "nn");
	pm.Start();

	float h = 0.005f;
	for (int i = 0; i < 400; ++i)
		pm.UpdatePhysics(i * h, h);

	int count;
	Vector3f* v = pm.GetVertices(0, &count);
	ASSERT_EQ(count, 2);
	// Equilibrium: stretch = mg/k = 9.81/200 = 0.04905
	// Free node y ≈ 0 - 0.04905 = -0.049
	EXPECT_NEAR(v[1].y, -0.04905, 0.001);
	EXPECT_EQ(v[0].y, 1.0f);
}
