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

#ifndef GZ_SIM_MESHINERTIACALCULATOR_HH_
#define GZ_SIM_MESHINERTIACALCULATOR_HH_

#include <optional>
#include <vector>

#include <sdf/CustomInertiaCalcProperties.hh>
#include <sdf/Types.hh>

#include <gz/sim/Export.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/config.hh>

#include <gz/common/SubMesh.hh>

#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/MassMatrix3.hh>
#include <gz/math/Inertial.hh>


namespace gz
{
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE
    {
      /// \brief Relative error tolerance allowed when testing if principal
      /// moments of a mass matrix satify the triangle inequality.
      constexpr double kPrincipalMomentRelativeTol = 0.05;

      /// \struct Triangle gz/sim/MeshInertiaCalculator.hh
      /// \brief A struct to represent a triangle of the mesh
      /// An instance of the struct holds 3 Vector3D instances
      /// each of which represents a vertex of the triangle &
      /// one more Vector3D that represents the centroid of the
      /// triangle
      struct Triangle
      {
        gz::math::Vector3d v0, v1, v2;

        gz::math::Vector3d centroid;
      };

      /// \class MeshInertiaCalculator gz/sim/MeshInertiaCalculator.hh
      /// \brief Inertial Properties (Mass, Centre of Mass & Moments of
      /// Inertia) calculator for 3D meshes.
      ///
      /// This class overloads the () operator, therefore, an instance
      /// of this class registered with libsdformat as a Custom
      /// Inertia Calculator. This would be used to calculate the
      /// inertial properties of 3D mesh using SDFormat.
      ///
      /// The calculation method used in this class is described here:
      /// https://www.geometrictools.com/Documentation/PolyhedralMassProperties.pdf
      /// and it works on triangle water-tight meshes for simple polyhedron
      class GZ_SIM_VISIBLE MeshInertiaCalculator
      {
        /// \brief Constructor
        public: MeshInertiaCalculator() = default;

        /// \brief Function to correct an invalid mass matrix. The mass matrix
        /// to be corrected needs to be positive definite and within a small
        /// tolerance of satisfying the triangle inequality test. If the above
        /// conditions are not satisfied, the mass matrix will not be corrected.
        /// \param[in, out] _massMatrix Mass matrix to correct
        /// \param[in] _tol Relative error tolerance allowed when testing if
        /// principal moments of a mass matrix satify the triangle inequality.
        /// \return True if the mass matrix is already valid or successfully
        /// corrected, false otherwise.
        public: static bool CorrectMassMatrix(
            gz::math::MassMatrix3d &_massMatrix,
            double tol = kPrincipalMomentRelativeTol);

        /// \brief Function to get the vertices & indices of the given mesh
        /// & convert them into instances of the Triangle struct
        /// Each triangle represents a triangle in the mesh & is added
        /// to a vector
        /// \param[out] _triangles A vector to hold all the Triangle
        /// instances obtained
        /// from the vertices & indices of the mesh
        /// \param[in] _meshScale A vector with the scaling factor
        /// of all the 3 axes
        /// \param[in] _mesh Mesh object
        public: static void GetMeshTriangles(
          std::vector<Triangle> &_triangles,
          const gz::math::Vector3d &_meshScale,
          const gz::common::SubMesh* _mesh);

        /// \brief Function that calculates the mass, mass matrix & centre of
        /// mass of a mesh using a vector of Triangles of the mesh
        /// \param[in] _triangles A vector of all the Triangles of the mesh
        /// \param[in] _density Density of the mesh
        /// \param[out] _massMatrix MassMatrix object to hold mass &
        /// moment of inertia of the mesh
        /// \param[out] _inertiaOrigin Pose3d object to hold the origin about
        /// which the inertia tensor was calculated
        public: static void CalculateMassProperties(
          const std::vector<Triangle>& _triangles,
          double _density,
          gz::math::MassMatrix3d& _massMatrix,
          gz::math::Pose3d& _inertiaOrigin);

        /// \brief Overloaded () operator which allows an instance
        /// of this class to be registered as a Custom Inertia
        /// Calculator with libsdformat
        /// \param[out] _errors A vector of Errors object. Each object
        /// would contain an error code and an error message.
        /// \param _calculatorParams An instance of
        /// CustomInertiaCalcProperties. This instance can be used
        /// to access the data like density,
        /// properties of the mesh, etc.
        public: std::optional<gz::math::Inertiald> operator()(
          sdf::Errors& _errors,
          const sdf::CustomInertiaCalcProperties& _calculatorParams);
      };
    }
  }
}

#endif
