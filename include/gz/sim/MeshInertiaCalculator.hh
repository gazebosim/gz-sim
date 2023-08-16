/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef GZ_SIM_MESHINERTIACALCULATOR_HH_
#define GZ_SIM_MESHINERTIACALCULATOR_HH_

#include <optional>

#include <sdf/sdf.hh>
#include <sdf/InterafaceMoiCalculator.hh>

#include "gz/sim/Util.hh"

#include <gz/common/graphics.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>

#include <gz/math/Vector3.hh>
#include <gz/math/MassMatrix3.hh>
#include <gz/math/Inertial.hh>


namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE
{
  struct Triangle
  {
    gz::math::Vector3d v0, v1, v2;
  };
  
  class GZ_SIM_VISIBLE MeshInertiaCalculator
  {
    public: void GetMeshTriangles(std::vector<Triangle>& triangles,
            const gz::common::Mesh* mesh);

    public: void CalculateMassProperties(std::vector<Triangle>& triangles, 
            double density, gz::math::MassMatrix3d& massMatrix,
            gz::math::Pose3d& centreOfMass);

    public: std::optional<gz::math::Inertiald> operator()(sdf::Errors& _errors,
            const sdf::InterfaceMoiCalculator& _calculatorParams);
    
    //public: sdf::CustomMOICalculator customCalculator = CalculateMeshInertia;
  };

  void MeshInertiaCalculator::GetMeshTriangles(std::vector<Triangle>& triangles,
          const gz::common::Mesh* mesh)
  {
    double* vertArray = NULL;
    int* indArray = NULL;

    mesh->FillArrays(&vertArray, &indArray);

    // Write code to check if size of the ind array is divisible by 3

    for (unsigned int i = 0; i < mesh->IndexCount(); i += 3)
    {
      Triangle triangle;
      triangle.v0.X() = vertArray[3 * indArray[i]];
      triangle.v0.Y() = vertArray[3 * indArray[i] + 1];
      triangle.v0.Z() = vertArray[3 * indArray[i] + 2];

      triangle.v1.X() = vertArray[3 * indArray[i+1]];
      triangle.v1.Y() = vertArray[3 * indArray[i+1] + 1];
      triangle.v1.Z() = vertArray[3 * indArray[i+1] + 2];

      triangle.v2.X() = vertArray[3 * indArray[i+2]];
      triangle.v2.Y() = vertArray[3 * indArray[i+2] + 1];
      triangle.v2.Z() = vertArray[3 * indArray[i+2] + 2];

      triangles.push_back(triangle);
    }
  }

  void MeshInertiaCalculator::CalculateMassProperties(std::vector<Triangle>& triangles,
            double density, gz::math::MassMatrix3d& massMatrix, gz::math::Pose3d& centreOfMass)
  {
    const double coefficients[10] = {1.0 / 6, 1.0 / 24, 1.0 / 24, 1.0 / 24, 1.0 / 60,
                                       1.0 / 60, 1.0 / 60, 1.0 / 120, 1.0 / 120, 1.0 / 120};

    // Number of triangles. Would probably add this as a data member of the MeshAdapterClass
    int numTriangles = triangles.size();

    // Calculate cross products of triangles 
    std::vector<gz::math::Vector3d> crosses(numTriangles);
    for (int i = 0; i < numTriangles; ++i) {
        crosses[i] = (triangles[i].v1 - triangles[i].v0).Cross(triangles[i].v2 - triangles[i].v0);
    }

    // Calculate subexpressions of the integral
    std::vector<gz::math::Vector3d> f1(numTriangles), f2(numTriangles), f3(numTriangles), g0(numTriangles), g1(numTriangles), g2(numTriangles);
    for (int i = 0; i < numTriangles; ++i) {
        f1[i] = triangles[i].v0 + triangles[i].v1 + triangles[i].v2;

        f2[i] = triangles[i].v0 * triangles[i].v0 +
                triangles[i].v1 * triangles[i].v1 +
                triangles[i].v0 * triangles[i].v1 +
                triangles[i].v2 * f1[i];

        f3[i] = triangles[i].v0 * triangles[i].v0 * triangles[i].v0 +
                triangles[i].v0 * triangles[i].v0 * triangles[i].v1 +
                triangles[i].v0 * triangles[i].v1 * triangles[i].v1 +
                triangles[i].v1 * triangles[i].v1 * triangles[i].v1 +
                triangles[i].v2 * f2[i];

        g0[i] = f2[i] + (triangles[i].v0 + f1[i]) * (triangles[i].v0);
        g1[i] = f2[i] + (triangles[i].v1 + f1[i]) * (triangles[i].v1);
        g2[i] = f2[i] + (triangles[i].v2 + f1[i]) * (triangles[i].v2);
    }

    // Calculate integral terms
    std::vector<double> integral(10);
    for (int i = 0; i < numTriangles; ++i) {
        double x0 = triangles[i].v0.X();
        double y0 = triangles[i].v0.Y();
        double z0 = triangles[i].v0.Z();

        double x1 = triangles[i].v1.X();
        double y1 = triangles[i].v1.Y();
        double z1 = triangles[i].v1.Z();

        double x2 = triangles[i].v2.X();
        double y2 = triangles[i].v2.Y();
        double z2 = triangles[i].v2.Z();

        integral[0] += crosses[i].X() * f1[i].X();
        integral[1] += crosses[i].X() * f2[i].X();
        integral[2] += crosses[i].Y() * f2[i].Y();
        integral[3] += crosses[i].Z() * f2[i].Z();
        integral[4] += crosses[i].X() * f3[i].X();
        integral[5] += crosses[i].Y() * f3[i].Y();
        integral[6] += crosses[i].Z() * f3[i].Z();
        integral[7] += crosses[i].X() * (y0 * g0[i].X() + y1 * g1[i].X() + y2 * g2[i].X());
        integral[8] += crosses[i].Y() * (z0 * g0[i].Y() + z1 * g1[i].Y() + z2 * g2[i].Y());
        integral[9] += crosses[i].Z() * (x0 * g0[i].Z() + x1 * g1[i].Z() + x2 * g2[i].Z());
    }

    for (int i = 0; i < 10; ++i) {
        integral[i] *= coefficients[i];
    }

    // Accumulate the result and add it to MassMatrix object of gz::math
    double volume = integral[0];
    double mass = volume * density;
    gz::math::Vector3d center_of_mass = gz::math::Vector3d();
    center_of_mass.X() = integral[0] / mass;
    center_of_mass.Y() = integral[1] / mass;
    center_of_mass.Z() = integral[2] / mass;

    gz::math::Vector3d ixxyyzz = gz::math::Vector3d();
    gz::math::Vector3d ixyxzyz = gz::math::Vector3d();

    // Diagonal Elements of the Mass Matrix
    ixxyyzz.X() = integral[5] + integral[6] - mass * (center_of_mass.Y() * center_of_mass.Y() + center_of_mass.Z() * center_of_mass.Z());
    ixxyyzz.Y() = -(integral[4] + integral[6] - mass * (center_of_mass.Z() * center_of_mass.Z() + center_of_mass.X() * center_of_mass.X()));
    ixxyyzz.Z() = -(integral[4] + integral[5] - mass * (center_of_mass.X() * center_of_mass.X() + center_of_mass.Y() * center_of_mass.Y()));

    // Off Diagonal Elements of the Mass Matrix
    ixyxzyz.X() = - (integral[7] - mass * center_of_mass.X() * center_of_mass.Y());
    ixyxzyz.Y() = - (integral[8] - mass * center_of_mass.Y() * center_of_mass.Z());
    ixyxzyz.Z() = - (integral[9] - mass * center_of_mass.X() * center_of_mass.Z());

    // Set the values in the MassMatrix object
    massMatrix.SetMass(mass);
    massMatrix.SetDiagonalMoments(ixxyyzz);
    massMatrix.SetOffDiagonalMoments(ixyxzyz);
    std::cout << "Mass of the mesh is: " << massMatrix.Mass() << std::endl;
    std::cout << "Diagonal Elements of the Inertia Matrix are: " << massMatrix.DiagonalMoments() << std::endl;
    std::cout << "Off Diagonal Elements of the Inertia Matrix are: " << massMatrix.OffDiagonalMoments() << std::endl;
  }

  std::optional<gz::math::Inertiald> MeshInertiaCalculator::operator()(sdf::Errors& _errors,
          const sdf::InterfaceMoiCalculator& _calculatorParams)
  {
    const gz::common::Mesh *mesh = nullptr;
    const double density = _calculatorParams.Density();
    const sdf::Mesh sdfMesh = _calculatorParams.Mesh();

    auto fullPath = asFullPath(sdfMesh.Uri(), sdfMesh.FilePath());
    if (fullPath.empty())
    {
      gzerr << "Mesh geometry missing uri" << std::endl;
      _errors.push_back({sdf::ErrorCode::URI_INVALID,
          "Attempting to load the mesh but the URI seems to be incorrect"});
      return std::nullopt;
    }

    // Load the Mesh
    gz::common::MeshManager *meshManager = gz::common::MeshManager::Instance();
    mesh = meshManager->Load(fullPath);

    std::vector<Triangle> meshTriangles;
    gz::math::MassMatrix3d meshMassMatrix;
    gz::math::Pose3d centreOfMass;

    // Create a list of Triangle objects from the mesh vertices and indices
    GetMeshTriangles(meshTriangles, mesh);

    CalculateMassProperties(meshTriangles, density, meshMassMatrix, centreOfMass);

    gz::math::Inertiald meshInertial;

    if (!meshInertial.SetMassMatrix(meshMassMatrix))
    {
      return std::nullopt;
    }
    else
    {
      meshInertial.SetPose(centreOfMass);
      return meshInertial;
    }
  }
}
}
}

#endif



