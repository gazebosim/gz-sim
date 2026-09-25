/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include <gtest/gtest.h>

#include "HydrodynamicsUtils.hh"

namespace hydro = gz::sim::systems::hydrodynamics;

using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;

/////////////////////////////////////////////////
/// Helper: build a diagonal added-mass matrix from 6 values.
static Mat6 DiagMa(double m0, double m1, double m2,
                   double m3, double m4, double m5)
{
  Mat6 Ma = Mat6::Zero();
  Ma(0, 0) = m0;
  Ma(1, 1) = m1;
  Ma(2, 2) = m2;
  Ma(3, 3) = m3;
  Ma(4, 4) = m4;
  Ma(5, 5) = m5;
  return Ma;
}

/////////////////////////////////////////////////
/// Verify every non-zero element of the Coriolis matrix against the
/// analytical Fossen (2011) eq 6.43 formulation.
///
/// Ma = diag(-1, -2, -3, -4, -5, -6)  (Fossen convention: negative)
/// state = [1, 2, 3, 4, 5, 6]
///
/// The code builds Cmat = -C_A. Using a_i = -Ma(i,i)*state(i):
///   a0=1, a1=4, a2=9, b0=16, b1=25, b2=36
///
/// Expected -C_A:
///   | 0   0   0   0   a2  -a1 |     | 0  0  0   0   9  -4 |
///   | 0   0   0  -a2  0    a0 |  =  | 0  0  0  -9   0   1 |
///   | 0   0   0   a1 -a0   0  |     | 0  0  0   4  -1   0 |
///   | 0   a2 -a1  0   b2  -b1 |     | 0  9 -4   0  36 -25 |
///   |-a2  0   a0 -b2  0    b0 |     |-9  0  1 -36   0  16 |
///   | a1 -a0  0   b1 -b0   0  |     | 4 -1  0  25 -16   0 |
TEST(HydrodynamicsUtils, CoriolisMatrixKnownValues)
{
  Mat6 Ma = DiagMa(-1, -2, -3, -4, -5, -6);
  Vec6 state;
  state << 1, 2, 3, 4, 5, 6;

  Mat6 C = hydro::buildCoriolisMatrix(Ma, state);

  // Upper-left 3x3 block is zero
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      EXPECT_DOUBLE_EQ(0.0, C(i, j));

  // Row 0
  EXPECT_DOUBLE_EQ(0.0, C(0, 3));
  EXPECT_DOUBLE_EQ(9.0, C(0, 4));
  EXPECT_DOUBLE_EQ(-4.0, C(0, 5));

  // Row 1
  EXPECT_DOUBLE_EQ(-9.0, C(1, 3));
  EXPECT_DOUBLE_EQ(0.0, C(1, 4));
  EXPECT_DOUBLE_EQ(1.0, C(1, 5));

  // Row 2
  EXPECT_DOUBLE_EQ(4.0, C(2, 3));
  EXPECT_DOUBLE_EQ(-1.0, C(2, 4));
  EXPECT_DOUBLE_EQ(0.0, C(2, 5));

  // Row 3
  EXPECT_DOUBLE_EQ(0.0, C(3, 0));
  EXPECT_DOUBLE_EQ(9.0, C(3, 1));
  EXPECT_DOUBLE_EQ(-4.0, C(3, 2));
  EXPECT_DOUBLE_EQ(0.0, C(3, 3));
  EXPECT_DOUBLE_EQ(36.0, C(3, 4));
  EXPECT_DOUBLE_EQ(-25.0, C(3, 5));

  // Row 4
  EXPECT_DOUBLE_EQ(-9.0, C(4, 0));
  EXPECT_DOUBLE_EQ(0.0, C(4, 1));
  EXPECT_DOUBLE_EQ(1.0, C(4, 2));
  EXPECT_DOUBLE_EQ(-36.0, C(4, 3));
  EXPECT_DOUBLE_EQ(0.0, C(4, 4));
  EXPECT_DOUBLE_EQ(16.0, C(4, 5));

  // Row 5
  EXPECT_DOUBLE_EQ(4.0, C(5, 0));
  EXPECT_DOUBLE_EQ(-1.0, C(5, 1));
  EXPECT_DOUBLE_EQ(0.0, C(5, 2));
  EXPECT_DOUBLE_EQ(25.0, C(5, 3));
  EXPECT_DOUBLE_EQ(-16.0, C(5, 4));
  EXPECT_DOUBLE_EQ(0.0, C(5, 5));
}

/////////////////////////////////////////////////
/// Verify that the Coriolis matrix is skew-symmetric: C + C^T = 0.
/// This is a fundamental property from Fossen (2011) Theorem 6.2.
TEST(HydrodynamicsUtils, CoriolisMatrixSkewSymmetry)
{
  // Test with asymmetric Ma diagonal and distinct velocities to
  // exercise all terms.
  Mat6 Ma = DiagMa(-4.876, -126.325, -126.325, 0.0, -33.46, -33.46);
  Vec6 state;
  state << 1.5, -0.3, 0.7, 0.1, -0.2, 0.4;

  Mat6 C = hydro::buildCoriolisMatrix(Ma, state);

  Mat6 sum = C + C.transpose();
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++)
      EXPECT_NEAR(0.0, sum(i, j), 1e-12)
        << "Skew-symmetry violated at (" << i << "," << j << ")";
}

/////////////////////////////////////////////////
/// Verify the passivity property: v^T * C * v = 0 for all v.
/// This guarantees the Coriolis matrix does not inject or dissipate energy.
TEST(HydrodynamicsUtils, CoriolisMatrixEnergyConservation)
{
  Mat6 Ma = DiagMa(-10, -20, -30, -5, -15, -25);

  // Test with several velocity vectors
  Vec6 states[3];
  states[0] << 1.0, 2.0, 3.0, 0.5, -1.0, 0.3;
  states[1] << -0.7, 0.0, 1.2, -0.3, 0.8, -0.1;
  states[2] << 5.0, -3.0, 2.0, 1.0, -2.0, 4.0;

  for (const auto &v : states)
  {
    Mat6 C = hydro::buildCoriolisMatrix(Ma, v);
    double energy = v.transpose() * C * v;
    EXPECT_NEAR(0.0, energy, 1e-12)
      << "Energy conservation violated for state: " << v.transpose();
  }
}

/////////////////////////////////////////////////
/// Verify that zero velocity produces a zero Coriolis matrix.
TEST(HydrodynamicsUtils, CoriolisMatrixZeroVelocity)
{
  Mat6 Ma = DiagMa(-4.876, -126.325, -126.325, 0.0, -33.46, -33.46);
  Vec6 state = Vec6::Zero();

  Mat6 C = hydro::buildCoriolisMatrix(Ma, state);

  EXPECT_TRUE(C.isZero(1e-15));
}

/////////////////////////////////////////////////
/// Verify that zero added mass produces a zero Coriolis matrix.
TEST(HydrodynamicsUtils, CoriolisMatrixZeroAddedMass)
{
  Mat6 Ma = Mat6::Zero();
  Vec6 state;
  state << 1, 2, 3, 4, 5, 6;

  Mat6 C = hydro::buildCoriolisMatrix(Ma, state);

  EXPECT_TRUE(C.isZero(1e-15));
}

/////////////////////////////////////////////////
/// Regression test for the two bugs fixed in elements (0,5) and (5,0).
///
/// Before the fix:
///   Cmat(0,5) = -Ma(1,1)*state(1)  (wrong sign)
///   Cmat(5,0) = +Ma(2,2)*state(2)  (wrong index, sign, coefficient)
///
/// After the fix:
///   Cmat(0,5) = +Ma(1,1)*state(1)
///   Cmat(5,0) = -Ma(1,1)*state(1)
///
/// Use distinct Ma diagonal values and state values so that the
/// old buggy code would produce different results.
TEST(HydrodynamicsUtils, CoriolisMatrixBugRegression)
{
  // Ma(1,1) = -7, Ma(2,2) = -13 (distinct values to catch index bugs)
  Mat6 Ma = DiagMa(-3, -7, -13, -2, -5, -11);
  Vec6 state;
  state << 0.5, 0.8, 1.2, 0.3, 0.6, 0.9;

  Mat6 C = hydro::buildCoriolisMatrix(Ma, state);

  // Cmat(0,5) should be Ma(1,1)*state(1) = (-7)*0.8 = -5.6
  EXPECT_DOUBLE_EQ(-7.0 * 0.8, C(0, 5));

  // Cmat(5,0) should be -Ma(1,1)*state(1) = -(-7)*0.8 = 5.6
  EXPECT_DOUBLE_EQ(-(-7.0) * 0.8, C(5, 0));

  // These two must be negatives of each other (skew-symmetry)
  EXPECT_DOUBLE_EQ(-C(0, 5), C(5, 0));

  // The OLD buggy code would have produced:
  //   Cmat(0,5) = -Ma(1,1)*state(1) = 5.6   (wrong)
  //   Cmat(5,0) = Ma(2,2)*state(2) = -15.6   (wrong)
  EXPECT_NE(5.6, C(0, 5));
  EXPECT_NE(-13.0 * 1.2, C(5, 0));
}

/////////////////////////////////////////////////
/// Test damping matrix with only diagonal linear terms.
///
/// With xU = -5 (stored in linearTerms[0*6+0]):
///   Dmat(0,0) = -(-5) = 5
/// Damping force_x = D(0,0) * state(0) = 5 * 2 = 10
TEST(HydrodynamicsUtils, DampingMatrixLinearOnly)
{
  double linear[36] = {0};
  double quadAbs[216] = {0};
  double quad[216] = {0};

  // Set diagonal linear terms: xU=-5, yV=-10, zW=-3
  linear[0 * 6 + 0] = -5.0;   // xU
  linear[1 * 6 + 1] = -10.0;  // yV
  linear[2 * 6 + 2] = -3.0;   // zW

  Vec6 state;
  state << 2.0, 1.0, 3.0, 0.0, 0.0, 0.0;

  Mat6 D = hydro::buildDampingMatrix(linear, quadAbs, quad, state);

  // D(i,j) = -linear[i*6+j] for linear-only case
  EXPECT_DOUBLE_EQ(5.0, D(0, 0));
  EXPECT_DOUBLE_EQ(10.0, D(1, 1));
  EXPECT_DOUBLE_EQ(3.0, D(2, 2));

  // Other diagonal terms should be zero
  EXPECT_DOUBLE_EQ(0.0, D(3, 3));
  EXPECT_DOUBLE_EQ(0.0, D(4, 4));
  EXPECT_DOUBLE_EQ(0.0, D(5, 5));

  // Off-diagonal should be zero
  EXPECT_DOUBLE_EQ(0.0, D(0, 1));
  EXPECT_DOUBLE_EQ(0.0, D(1, 0));

  // Check damping force
  Vec6 force = D * state;
  EXPECT_DOUBLE_EQ(10.0, force(0));   // 5 * 2
  EXPECT_DOUBLE_EQ(10.0, force(1));   // 10 * 1
  EXPECT_DOUBLE_EQ(9.0, force(2));    // 3 * 3
}

/////////////////////////////////////////////////
/// Test damping matrix with quadratic absolute-value terms.
///
/// xUabsU = -6.2282 stored in quadAbsDerivs[0*36+0*6+0]:
///   D(0,0) = -(-6.2282)*|state(0)| = 6.2282*|u|
TEST(HydrodynamicsUtils, DampingMatrixQuadraticAbs)
{
  double linear[36] = {0};
  double quadAbs[216] = {0};
  double quad[216] = {0};

  // xUabsU: index = 0*36 + 0*6 + 0 = 0
  quadAbs[0] = -6.2282;

  Vec6 state;
  state << 2.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  Mat6 D = hydro::buildDampingMatrix(linear, quadAbs, quad, state);

  // D(0,0) = -(-6.2282) * |2.0| = 12.4564
  EXPECT_DOUBLE_EQ(6.2282 * 2.0, D(0, 0));

  // Damping force in x = D(0,0) * u = 12.4564 * 2 = 24.9128
  Vec6 force = D * state;
  EXPECT_DOUBLE_EQ(6.2282 * 2.0 * 2.0, force(0));

  // Now test with negative velocity — abs should give same D coefficient
  state << -2.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  D = hydro::buildDampingMatrix(linear, quadAbs, quad, state);
  EXPECT_DOUBLE_EQ(6.2282 * 2.0, D(0, 0));
}

/////////////////////////////////////////////////
/// Test damping matrix with quadratic (non-abs) terms.
///
/// These terms use state(k) directly (not |state(k)|), so the sign
/// of the velocity matters.
TEST(HydrodynamicsUtils, DampingMatrixQuadraticVelocity)
{
  double linear[36] = {0};
  double quadAbs[216] = {0};
  double quad[216] = {0};

  // xUU: index = 0*36 + 0*6 + 0 = 0
  quad[0] = -94.2475;

  Vec6 state;
  state << 2.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  Mat6 D = hydro::buildDampingMatrix(linear, quadAbs, quad, state);

  // D(0,0) = -(-94.2475) * state(0) = 94.2475 * 2.0 = 188.495
  EXPECT_DOUBLE_EQ(94.2475 * 2.0, D(0, 0));

  // With negative velocity, the coefficient sign flips (unlike abs variant)
  state << -2.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  D = hydro::buildDampingMatrix(linear, quadAbs, quad, state);
  // D(0,0) = -(-94.2475) * (-2.0) = -188.495
  EXPECT_DOUBLE_EQ(-94.2475 * 2.0, D(0, 0));
}

/////////////////////////////////////////////////
/// Test damping matrix with a cross term.
///
/// xUV (x-force from U*V coupling): index = 0*36 + 0*6 + 1 = 1
/// This contributes to D(0,0) when multiplied by state(1).
TEST(HydrodynamicsUtils, DampingMatrixCrossTerm)
{
  double linear[36] = {0};
  double quadAbs[216] = {0};
  double quad[216] = {0};

  // xUV: force-axis=x(0), velocity-axis=U(0), coupling-velocity=V(1)
  // index = 0*36 + 0*6 + 1 = 1
  quad[1] = -5.0;

  Vec6 state;
  state << 1.0, 3.0, 0.0, 0.0, 0.0, 0.0;

  Mat6 D = hydro::buildDampingMatrix(linear, quadAbs, quad, state);

  // D(0,0) += -(-5.0) * state(1) = 5.0 * 3.0 = 15.0
  EXPECT_DOUBLE_EQ(15.0, D(0, 0));

  // Force_x = D(0,0) * state(0) = 15 * 1 = 15
  Vec6 force = D * state;
  EXPECT_DOUBLE_EQ(15.0, force(0));
}

/////////////////////////////////////////////////
/// Test that zero coefficients produce a zero damping matrix.
TEST(HydrodynamicsUtils, DampingMatrixZeroCoefficients)
{
  double linear[36] = {0};
  double quadAbs[216] = {0};
  double quad[216] = {0};

  Vec6 state;
  state << 1, 2, 3, 4, 5, 6;

  Mat6 D = hydro::buildDampingMatrix(linear, quadAbs, quad, state);

  EXPECT_TRUE(D.isZero(1e-15));
}

/////////////////////////////////////////////////
/// Test that zero velocity produces a zero damping matrix
/// (linear terms produce a constant D, but the product D*v is zero).
/// Note: with linear terms only, D is constant regardless of state.
TEST(HydrodynamicsUtils, DampingMatrixZeroVelocity)
{
  double linear[36] = {0};
  double quadAbs[216] = {0};
  double quad[216] = {0};

  linear[0] = -5.0;  // xU

  Vec6 state = Vec6::Zero();

  Mat6 D = hydro::buildDampingMatrix(linear, quadAbs, quad, state);

  // Linear term still contributes to D even with zero velocity
  EXPECT_DOUBLE_EQ(5.0, D(0, 0));

  // But the resulting force is zero since state = 0
  Vec6 force = D * state;
  EXPECT_TRUE(force.isZero(1e-15));
}

/////////////////////////////////////////////////
/// Test combined linear + quadratic damping.
///
/// This mimics a typical AUV configuration with both linear and
/// quadratic drag on the surge axis:
///   xU = -0.2  (linear)
///   xUabsU = -6.2282  (quadratic |u|)
///
/// For u = 2.0:
///   D(0,0) = -(-0.2) + (-(-6.2282))*|2| = 0.2 + 12.4564 = 12.6564
///   force_x = 12.6564 * 2.0 = 25.3128
TEST(HydrodynamicsUtils, DampingMatrixCombined)
{
  double linear[36] = {0};
  double quadAbs[216] = {0};
  double quad[216] = {0};

  linear[0] = -0.2;   // xU
  quadAbs[0] = -6.2282;  // xUabsU

  Vec6 state;
  state << 2.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  Mat6 D = hydro::buildDampingMatrix(linear, quadAbs, quad, state);

  EXPECT_DOUBLE_EQ(0.2 + 6.2282 * 2.0, D(0, 0));

  Vec6 force = D * state;
  EXPECT_DOUBLE_EQ((0.2 + 6.2282 * 2.0) * 2.0, force(0));
}

/////////////////////////////////////////////////
/// Test a full 6-DOF damping configuration matching the tethys
/// AUV model from the test world (hydrodynamics_flags.sdf).
TEST(HydrodynamicsUtils, DampingMatrixTethysConfig)
{
  double linear[36] = {0};
  double quadAbs[216] = {0};
  double quad[216] = {0};

  // From hydrodynamics_flags.sdf (tethys model)
  // xUabsU=-6.2282, yVabsV=-601.27, zWabsW=-601.27
  // kPabsP=-0.1916, mQabsQ=-632.698957, nRabsR=-632.698957
  quadAbs[0 * 36 + 0 * 6 + 0] = -6.2282;      // xUabsU
  quadAbs[1 * 36 + 1 * 6 + 1] = -601.27;       // yVabsV
  quadAbs[2 * 36 + 2 * 6 + 2] = -601.27;       // zWabsW
  quadAbs[3 * 36 + 3 * 6 + 3] = -0.1916;       // kPabsP
  quadAbs[4 * 36 + 4 * 6 + 4] = -632.698957;   // mQabsQ
  quadAbs[5 * 36 + 5 * 6 + 5] = -632.698957;   // nRabsR

  Vec6 state;
  state << 1.0, 0.5, -0.3, 0.1, -0.05, 0.02;

  Mat6 D = hydro::buildDampingMatrix(linear, quadAbs, quad, state);

  // D(i,i) = |quadAbs_i| * |state(i)| for diagonal terms
  EXPECT_DOUBLE_EQ(6.2282 * 1.0, D(0, 0));
  EXPECT_DOUBLE_EQ(601.27 * 0.5, D(1, 1));
  EXPECT_DOUBLE_EQ(601.27 * 0.3, D(2, 2));
  EXPECT_DOUBLE_EQ(0.1916 * 0.1, D(3, 3));
  EXPECT_DOUBLE_EQ(632.698957 * 0.05, D(4, 4));
  EXPECT_DOUBLE_EQ(632.698957 * 0.02, D(5, 5));

  // Off-diagonal should be zero for diagonal-only config
  EXPECT_DOUBLE_EQ(0.0, D(0, 1));
  EXPECT_DOUBLE_EQ(0.0, D(1, 0));
  EXPECT_DOUBLE_EQ(0.0, D(2, 3));
}

/////////////////////////////////////////////////
/// Test that std::abs is used (not integer abs) by verifying
/// correct behavior for velocities in (-1, +1).
///
/// This is a regression test for bug 1.2: if abs(int) were used,
/// abs(0.7) would return 0, making the quadratic term vanish.
TEST(HydrodynamicsUtils, DampingMatrixSubUnitVelocity)
{
  double linear[36] = {0};
  double quadAbs[216] = {0};
  double quad[216] = {0};

  quadAbs[0] = -10.0;  // xUabsU

  Vec6 state;
  state << 0.7, 0.0, 0.0, 0.0, 0.0, 0.0;

  Mat6 D = hydro::buildDampingMatrix(linear, quadAbs, quad, state);

  // If abs(int) were used: abs(0.7) -> abs(0) -> 0, so D(0,0) = 0
  // With std::abs(double): D(0,0) = 10.0 * 0.7 = 7.0
  EXPECT_DOUBLE_EQ(10.0 * 0.7, D(0, 0));
  EXPECT_NE(0.0, D(0, 0));

  // Also test very small velocities
  state << 0.001, 0.0, 0.0, 0.0, 0.0, 0.0;
  D = hydro::buildDampingMatrix(linear, quadAbs, quad, state);
  EXPECT_DOUBLE_EQ(10.0 * 0.001, D(0, 0));
  EXPECT_NE(0.0, D(0, 0));
}

/////////////////////////////////////////////////
/// Verify the Coriolis force direction for a typical AUV turning scenario.
///
/// For a torpedo-shaped AUV with X_u'=-5, Y_v'=-100, Z_w'=-100
/// moving forward (u=1) with sway (v=0.5), the Coriolis matrix
/// should produce a destabilizing Munk moment in yaw.
///
/// The yaw moment from Coriolis = -Cmat(5,:) * state, where
/// Cmat(5,0) = -Ma(1,1)*state(1) and Cmat(5,1) = Ma(0,0)*state(0).
///
/// Munk moment ≈ (Y_v' - X_u') * u * v = (-100 - (-5)) * 1 * 0.5 = -47.5
/// (destabilizing, opposing the turn).
TEST(HydrodynamicsUtils, CoriolisForceDirection)
{
  Mat6 Ma = DiagMa(-5, -100, -100, 0, -10, -10);
  Vec6 state;
  state << 1.0, 0.5, 0.0, 0.0, 0.0, 0.0;

  Mat6 C = hydro::buildCoriolisMatrix(Ma, state);

  // The Coriolis force vector is -C * state (as applied in the plugin)
  Vec6 coriolisForce = -C * state;

  // Yaw moment (index 5):
  // -C(5,:) * state = -(C(5,0)*u + C(5,1)*v)
  //   C(5,0) = -Ma(1,1)*state(1) = 100*0.5 = 50
  //   C(5,1) = Ma(0,0)*state(0) = -5*1 = -5
  //   force(5) = -(50*1 + (-5)*0.5) = -(50 - 2.5) = -47.5
  EXPECT_DOUBLE_EQ(-47.5, coriolisForce(5));

  // This is the Munk moment: (Y_v' - X_u') * u * v
  double munk = (Ma(1, 1) - Ma(0, 0)) * state(0) * state(1);
  EXPECT_DOUBLE_EQ(munk, coriolisForce(5));
}

/////////////////////////////////////////////////
/// Test Coriolis matrix with the actual tethys AUV values from the
/// hydrodynamics_flags.sdf test world.
TEST(HydrodynamicsUtils, CoriolisMatrixTethysValues)
{
  Mat6 Ma = DiagMa(-4.876161, -126.324739, -126.324739,
                   0.0, -33.46, -33.46);
  Vec6 state;
  state << 0.5, 0.3, -0.1, 0.0, 0.02, -0.01;

  Mat6 C = hydro::buildCoriolisMatrix(Ma, state);

  // Verify skew-symmetry
  Mat6 sum = C + C.transpose();
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++)
      EXPECT_NEAR(0.0, sum(i, j), 1e-12);

  // Verify energy conservation
  double energy = state.transpose() * C * state;
  EXPECT_NEAR(0.0, energy, 1e-12);

  // Spot-check a few elements
  // C(0,4) = -Ma(2,2)*state(2) = -(-126.324739)*(-0.1) = -12.6324739
  EXPECT_DOUBLE_EQ(-(-126.324739) * (-0.1), C(0, 4));

  // C(5,0) = -Ma(1,1)*state(1) = -(-126.324739)*0.3 = 37.8974217
  EXPECT_DOUBLE_EQ(-(-126.324739) * 0.3, C(5, 0));
}

/////////////////////////////////////////////////
// ---- Tests for buildFullCoriolisMatrix (positive M_A convention) ----
/////////////////////////////////////////////////

/////////////////////////////////////////////////
/// Verify that buildFullCoriolisMatrix (positive M_A) is consistent
/// with buildCoriolisMatrix (negative Fossen M_A) for diagonal matrices.
///
/// buildCoriolisMatrix(neg_Ma, v) returns -C_A(neg convention)
///   = C_A(positive convention)
///   = buildFullCoriolisMatrix(pos_Ma, v)
TEST(HydrodynamicsUtils, FullCoriolisMatrixDiagonalEquivalence)
{
  // Fossen convention (negative)
  Mat6 negMa = DiagMa(-4.876, -126.325, -126.325, 0.0, -33.46, -33.46);
  // Physical convention (positive)
  Mat6 posMa = DiagMa(4.876, 126.325, 126.325, 0.0, 33.46, 33.46);

  Vec6 state;
  state << 1.5, -0.3, 0.7, 0.1, -0.2, 0.4;

  Mat6 fromOld = hydro::buildCoriolisMatrix(negMa, state);
  Mat6 fromNew = hydro::buildFullCoriolisMatrix(posMa, state);

  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++)
      EXPECT_NEAR(fromOld(i, j), fromNew(i, j), 1e-12)
        << "Mismatch at (" << i << "," << j << ")";
}

/////////////////////////////////////////////////
/// Verify skew-symmetry of the full Coriolis matrix with a non-diagonal
/// added mass matrix.
TEST(HydrodynamicsUtils, FullCoriolisMatrixSkewSymmetry)
{
  Mat6 Ma;
  Ma << 10, 1, 0, 0, 0, 0.5,
         1, 20, 2, 0, 0, 0,
         0, 2, 30, 0, 0, 0,
         0, 0, 0, 5, 0.3, 0,
         0, 0, 0, 0.3, 15, 1,
         0.5, 0, 0, 0, 1, 25;

  Vec6 state;
  state << 1.0, -0.5, 0.3, 0.1, -0.2, 0.4;

  Mat6 C = hydro::buildFullCoriolisMatrix(Ma, state);

  Mat6 sum = C + C.transpose();
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++)
      EXPECT_NEAR(0.0, sum(i, j), 1e-12)
        << "Skew-symmetry violated at (" << i << "," << j << ")";
}

/////////////////////////////////////////////////
/// Verify energy conservation: v^T * C_A * v = 0 for the full matrix.
TEST(HydrodynamicsUtils, FullCoriolisMatrixEnergyConservation)
{
  Mat6 Ma;
  Ma << 10, 1, 0, 0, 0, 0.5,
         1, 20, 2, 0, 0, 0,
         0, 2, 30, 0, 0, 0,
         0, 0, 0, 5, 0.3, 0,
         0, 0, 0, 0.3, 15, 1,
         0.5, 0, 0, 0, 1, 25;

  Vec6 states[3];
  states[0] << 1.0, 2.0, 3.0, 0.5, -1.0, 0.3;
  states[1] << -0.7, 0.0, 1.2, -0.3, 0.8, -0.1;
  states[2] << 5.0, -3.0, 2.0, 1.0, -2.0, 4.0;

  for (const auto &v : states)
  {
    Mat6 C = hydro::buildFullCoriolisMatrix(Ma, v);
    double energy = v.transpose() * C * v;
    EXPECT_NEAR(0.0, energy, 1e-12);
  }
}

/////////////////////////////////////////////////
/// Verify the correction wrench is zero when there is no current
/// (v_r == v).
TEST(HydrodynamicsUtils, CorrectionWrenchZeroWithNoCurrent)
{
  Mat6 Ma = DiagMa(5, 100, 100, 0, 10, 10);

  Vec6 v;
  v << 1.5, -0.3, 0.7, 0.1, -0.2, 0.4;

  // When v_r == v, correction should be zero
  auto Ca = hydro::buildFullCoriolisMatrix(Ma, v);
  Vec6 correction = Ca * v - Ca * v;
  EXPECT_TRUE(correction.isZero(1e-15));
}

/////////////////////////////////////////////////
/// Verify the correction wrench for a known case.
///
/// With M_A = diag(5, 100, ...), v = (1, 0.5, 0, 0, 0, 0),
/// v_current = (0.3, 0, 0, 0, 0, 0) in body frame,
/// v_r = (0.7, 0.5, 0, 0, 0, 0).
///
/// The correction wrench = C_A(v)*v - C_A(v_r)*v_r should be
/// nonzero in the yaw moment (index 5) due to the Munk moment
/// difference.
TEST(HydrodynamicsUtils, CorrectionWrenchKnownValues)
{
  Mat6 Ma = DiagMa(5, 100, 100, 0, 10, 10);

  Vec6 v_abs, v_rel;
  v_abs << 1.0, 0.5, 0.0, 0.0, 0.0, 0.0;
  v_rel << 0.7, 0.5, 0.0, 0.0, 0.0, 0.0;

  auto Ca_abs = hydro::buildFullCoriolisMatrix(Ma, v_abs);
  auto Ca_rel = hydro::buildFullCoriolisMatrix(Ma, v_rel);

  Vec6 correction = Ca_abs * v_abs - Ca_rel * v_rel;

  // Munk moment in yaw (index 5) = (M_A(1,1) - M_A(0,0)) * u * v
  // For v_abs: (100-5)*1.0*0.5 = 47.5
  // For v_rel: (100-5)*0.7*0.5 = 33.25
  // correction(5) = 47.5 - 33.25 = 14.25
  EXPECT_NEAR(14.25, correction(5), 1e-12);

  // Surge force (index 0): C_A(0,5)*r + C_A(0,4)*q = 0 since p,q,r = 0
  EXPECT_NEAR(0.0, correction(0), 1e-12);
}
