/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include "MeshInertiaCalculator.hh"

#include <algorithm>
#include <numeric>
#include <optional>
#include <vector>

#include <sdf/CustomInertiaCalcProperties.hh>
#include <sdf/Types.hh>

#include <gz/sim/Util.hh>

#include <gz/common/MeshManager.hh>
#include <gz/common/SubMesh.hh>

#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/MassMatrix3.hh>
#include <gz/math/Inertial.hh>
#include <gz/math/Quaternion.hh>

using namespace gz;
using namespace sim;

//////////////////////////////////////////////////
bool MeshInertiaCalculator::CorrectMassMatrix(
    math::MassMatrix3d &_massMatrix, double _tol)
{
  if (_massMatrix.IsValid())
    return true;

  if (!_massMatrix.IsPositive())
    return false;

  math::Quaterniond principalAxesOffset = _massMatrix.PrincipalAxesOffset();
  math::Vector3d principalMoments = _massMatrix.PrincipalMoments();

  // Track elements in principalMoments in ascending order using a sorted
  // indices array, i.e. sortedIndices[0] should point to the index in
  // principalMoments containing the smallest value, while sortedIndices[2]
  // should point to the index in principalMoments containing the largest value.
  std::vector<int> sortedIndices(3);
  std::iota(sortedIndices.begin(), sortedIndices.end(), 0);
  std::sort(sortedIndices.begin(), sortedIndices.end(), [&](int i, int j)
      { return principalMoments[i] < principalMoments[j]; } );

  // Check if principal moments are within tol of satisfying the
  // triangle inequality.
  const double ratio =
      (principalMoments[sortedIndices[0]] + principalMoments[sortedIndices[1]])
      / principalMoments[sortedIndices[2]];
  if ((1.0 - ratio) > _tol)
  {
    // The error is outside of tol so do not attempt to correct the mass matrix.
    return false;
  }
  // Scale the 2 smaller elements in principalMoments so that they
  // satisfy the triangle inequality
  const double scale = 1.0 / ratio;
  math::Vector3d correctedPrincipalMoments = principalMoments;
  correctedPrincipalMoments[sortedIndices[0]] *= scale;
  correctedPrincipalMoments[sortedIndices[1]] *= scale;

  // Recompute mass matrix with corrected principal moments.
  math::MassMatrix3d correctedPrincipalMassMatrix(_massMatrix.Mass(),
      correctedPrincipalMoments, math::Vector3d::Zero);
  math::Inertiald correctedPrincipalMassMatrixWithAxesOffset(
      correctedPrincipalMassMatrix,
      math::Pose3d(math::Vector3d::Zero, principalAxesOffset));
  _massMatrix.SetMoi(correctedPrincipalMassMatrixWithAxesOffset.Moi());

  return true;
}

//////////////////////////////////////////////////
void MeshInertiaCalculator::GetMeshTriangles(
  std::vector<Triangle> &_triangles,
  const gz::math::Vector3d &_meshScale,
  const gz::common::SubMesh *_subMesh)
{
  // Get the vertices & indices of the mesh
  double* vertArray = nullptr;
  int* indArray = nullptr;
  _subMesh->FillArrays(&vertArray, &indArray);

  // Add check to see if size of the ind array is divisible by 3
  for (unsigned int i = 0; i < _subMesh->IndexCount(); i += 3)
  {
    Triangle triangle;
    triangle.v0.X() = vertArray[static_cast<ptrdiff_t>(3 * indArray[i])];
    triangle.v0.Y() = vertArray[static_cast<ptrdiff_t>(3 * indArray[i] + 1)];
    triangle.v0.Z() = vertArray[static_cast<ptrdiff_t>(3 * indArray[i] + 2)];
    triangle.v1.X() = vertArray[static_cast<ptrdiff_t>(3 * indArray[i+1])];
    triangle.v1.Y() = vertArray[static_cast<ptrdiff_t>(3 * indArray[i+1] + 1)];
    triangle.v1.Z() = vertArray[static_cast<ptrdiff_t>(3 * indArray[i+1] + 2)];
    triangle.v2.X() = vertArray[static_cast<ptrdiff_t>(3 * indArray[i+2])];
    triangle.v2.Y() = vertArray[static_cast<ptrdiff_t>(3 * indArray[i+2] + 1)];
    triangle.v2.Z() = vertArray[static_cast<ptrdiff_t>(3 * indArray[i+2] + 2)];

    // Apply mesh scale to the triangle coordinates
    triangle.v0 = triangle.v0 * _meshScale;
    triangle.v1 = triangle.v1 * _meshScale;
    triangle.v2 = triangle.v2 * _meshScale;

    triangle.centroid = (triangle.v0 + triangle.v1 + triangle.v2) / 3;
    _triangles.push_back(triangle);
  }
}

//////////////////////////////////////////////////
void MeshInertiaCalculator::CalculateMassProperties(
  const std::vector<Triangle>& _triangles,
  double _density,
  gz::math::MassMatrix3d& _massMatrix,
  gz::math::Pose3d& _centreOfMass)
{
  // Some coefficients for the calculation of integral terms
  const double coefficients[10] = {1.0 / 6,   1.0 / 24, 1.0 / 24, 1.0 / 24,
                                   1.0 / 60,  1.0 / 60, 1.0 / 60, 1.0 / 120,
                                   1.0 / 120, 1.0 / 120};

  // Number of triangles of in the mesh
  std::size_t numTriangles = _triangles.size();

  // Vector to store cross products of 2 vectors of the triangles
  std::vector<gz::math::Vector3d> crosses(numTriangles);

  // Caculating cross products of 2 vectors emerging from a common vertex
  // This basically gives a vector normal to the plane of triangle
  for (std::size_t i = 0; i < numTriangles; ++i)
  {
    crosses[i] =
      (_triangles[i].v1 - _triangles[i].v0).Cross(
        _triangles[i].v2 - _triangles[i].v0);
  }

  // Calculate subexpressions of the integral
  std::vector<gz::math::Vector3d> f1(numTriangles), f2(numTriangles),
  f3(numTriangles), g0(numTriangles), g1(numTriangles), g2(numTriangles);

  for (std::size_t i = 0; i < numTriangles; ++i)
  {
      f1[i] = _triangles[i].v0 + _triangles[i].v1 + _triangles[i].v2;
      f2[i] = _triangles[i].v0 * _triangles[i].v0 +
              _triangles[i].v1 * _triangles[i].v1 +
              _triangles[i].v0 * _triangles[i].v1 +
              _triangles[i].v2 * f1[i];
      f3[i] = _triangles[i].v0 * _triangles[i].v0 * _triangles[i].v0 +
              _triangles[i].v0 * _triangles[i].v0 * _triangles[i].v1 +
              _triangles[i].v0 * _triangles[i].v1 * _triangles[i].v1 +
              _triangles[i].v1 * _triangles[i].v1 * _triangles[i].v1 +
              _triangles[i].v2 * f2[i];
      g0[i] = f2[i] + (_triangles[i].v0 + f1[i]) * (_triangles[i].v0);
      g1[i] = f2[i] + (_triangles[i].v1 + f1[i]) * (_triangles[i].v1);
      g2[i] = f2[i] + (_triangles[i].v2 + f1[i]) * (_triangles[i].v2);
  }

  // Calculate integral terms
  std::vector<double> integral(10);
  for (std::size_t i = 0; i < numTriangles; ++i)
  {
      double x0 = _triangles[i].v0.X();
      double y0 = _triangles[i].v0.Y();
      double z0 = _triangles[i].v0.Z();
      double x1 = _triangles[i].v1.X();
      double y1 = _triangles[i].v1.Y();
      double z1 = _triangles[i].v1.Z();
      double x2 = _triangles[i].v2.X();
      double y2 = _triangles[i].v2.Y();
      double z2 = _triangles[i].v2.Z();
      integral[0] += crosses[i].X() * f1[i].X();
      integral[1] += crosses[i].X() * f2[i].X();
      integral[2] += crosses[i].Y() * f2[i].Y();
      integral[3] += crosses[i].Z() * f2[i].Z();
      integral[4] += crosses[i].X() * f3[i].X();
      integral[5] += crosses[i].Y() * f3[i].Y();
      integral[6] += crosses[i].Z() * f3[i].Z();
      integral[7] += crosses[i].X() *
        (y0 * g0[i].X() + y1 * g1[i].X() + y2 * g2[i].X());
      integral[8] += crosses[i].Y() *
        (z0 * g0[i].Y() + z1 * g1[i].Y() + z2 * g2[i].Y());
      integral[9] += crosses[i].Z() *
        (x0 * g0[i].Z() + x1 * g1[i].Z() + x2 * g2[i].Z());
  }

  for (int i = 0; i < 10; ++i)
  {
      integral[i] *= coefficients[i];
  }

  // Accumulate the result and add it to MassMatrix object of gz::math
  double volume = integral[0];
  double mass = volume * _density;
  _centreOfMass.SetX(integral[1] / volume);
  _centreOfMass.SetY(integral[2] / volume);
  _centreOfMass.SetZ(integral[3] / volume);
  gz::math::Vector3d ixxyyzz = gz::math::Vector3d();
  gz::math::Vector3d ixyxzyz = gz::math::Vector3d();

  // Diagonal Elements of the Mass Matrix
  ixxyyzz.X() = (integral[5] + integral[6] - volume *
                (_centreOfMass.Y() * _centreOfMass.Y() +
                _centreOfMass.Z() * _centreOfMass.Z()));
  ixxyyzz.Y() = (integral[4] + integral[6] - volume *
                (_centreOfMass.Z() * _centreOfMass.Z() +
                _centreOfMass.X() * _centreOfMass.X()));
  ixxyyzz.Z() = integral[4] + integral[5] - volume *
                (_centreOfMass.X() * _centreOfMass.X() +
                _centreOfMass.Y() * _centreOfMass.Y());

  // Off Diagonal Elements of the Mass Matrix
  ixyxzyz.X() = -(integral[7] - volume * _centreOfMass.X() * _centreOfMass.Y());
  ixyxzyz.Y() = -(integral[9] - volume * _centreOfMass.X() * _centreOfMass.Z());
  ixyxzyz.Z() = -(integral[8] - volume * _centreOfMass.Y() * _centreOfMass.Z());

  // Set the values in the MassMatrix object
  _massMatrix.SetMass(mass);
  _massMatrix.SetDiagonalMoments(ixxyyzz * _density);
  _massMatrix.SetOffDiagonalMoments(ixyxzyz * _density);
}

//////////////////////////////////////////////////
std::optional<gz::math::Inertiald> MeshInertiaCalculator::operator()
  (sdf::Errors &_errors,
  const sdf::CustomInertiaCalcProperties& _calculatorParams)
{
  const double density = _calculatorParams.Density();

  auto sdfMesh = _calculatorParams.Mesh();

  if (sdfMesh == std::nullopt)
  {
    gzerr << "Could not calculate inertia for mesh "
              "as mesh SDF is std::nullopt" << std::endl;
    _errors.push_back({sdf::ErrorCode::FATAL_ERROR,
        "Could not calculate mesh inertia as mesh SDF is"
        "std::nullopt"});
    return std::nullopt;
  }

  const common::Mesh *mesh = loadMesh(*sdfMesh);
  if (!mesh)
  {
    gzerr << "Failed to load mesh: " << sdfMesh->Uri() << std::endl;
    _errors.push_back({sdf::ErrorCode::FATAL_ERROR,
        "Could not calculate mesh inertia as mesh is not loaded."});
    return std::nullopt;
  }

  // Compute inertia for each submesh then sum up to get the final inertia
  // values.
  gz::math::Inertiald meshInertial;
  for (unsigned int i = 0; i < mesh->SubMeshCount(); ++i)
  {
    std::vector<Triangle> meshTriangles;
    gz::math::MassMatrix3d meshMassMatrix;
    gz::math::Pose3d centreOfMass;

    // Create a list of Triangle objects from the mesh vertices and indices
    auto submesh = mesh->SubMeshByIndex(i).lock();
    this->GetMeshTriangles(meshTriangles, sdfMesh->Scale(), submesh.get());

    // Calculate mesh mass properties
    this->CalculateMassProperties(meshTriangles, density,
      meshMassMatrix, centreOfMass);

    if (!meshMassMatrix.IsValid())
    {
      gzwarn << "Auto-calculated mass matrix is invalid for mesh: "
             << mesh->Name() << ", submesh index: " << i << "."
             << std::endl;
      if (!this->CorrectMassMatrix(meshMassMatrix))
      {
        gzwarn << "  Unable to correct mass matrix. Skipping submesh."
               << std::endl;
        continue;
      }
      gzwarn << "  Successfully corrected mass matrix."
             << std::endl;
    }
    meshInertial += gz::math::Inertiald(meshMassMatrix, centreOfMass);
  }

  if (meshInertial.MassMatrix().Mass() <= 0.0 ||
      !meshInertial.MassMatrix().IsValid())
  {
    gzerr << "Failed to computed valid inertia in MeshInertiaCalculator. "
          << "Ensure that the mesh is water tight, or try optimizing the mesh "
          << "by setting the //mesh/@optimization attribute in SDF to "
          << "`convex_hull` or `convex_decomposition`."
          << std::endl;
    _errors.push_back({sdf::ErrorCode::WARNING,
        "Could not calculate valid mesh inertia"});
    return std::nullopt;
  }

  return meshInertial;
}
