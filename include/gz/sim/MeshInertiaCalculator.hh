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

#include <sdf/sdf.hh>
#include <sdf/CustomInertiaCalcProperties.hh>

#include <gz/sim/Util.hh>
#include <gz/sim/Export.hh>

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
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE
    {
      /// \struct Triangle gz/sim/MeshInertiaCalculator.hh
      /// \brief A struct to represent a triangle of the mesh
      /// An instance of the struct holds 3 vector3d instances
      /// each of which represents a vertex of the triangle
      struct Triangle
      {
        gz::math::Vector3d v0, v1, v2;
      };

      /// \class MeshInertiaCalculator gz/sim/MeshInertiaCalculator.hh
      /// \brief Inertial Properties (Mass, Centre of Mass and Moments of Inertia)
      /// calculator for 3D meshes.
      ///
      /// This class overloads the () operator, therefore, an instance of this class
      /// registered with libsdformat as a Custom Inertia Calculator. This would used
      /// to calculate the inertial properties of 3D mesh using SDFormat.
      ///
      /// The calculation method used in this class is described here:
      /// https://www.geometrictools.com/Documentation/PolyhedralMassProperties.pdf
      /// and it works on triangle water-tight meshes for simple polyhedron
      class GZ_SIM_VISIBLE MeshInertiaCalculator
      {
        /// \brief Constructor
        public: MeshInertiaCalculator();

        /// \brief  Destructor
        public: ~MeshInertiaCalculator();

        /// \brief Function to get the vertices & indices of the given mesh
        /// & convert them into instances of the Triangle struct
        /// Each triangle represents a triangle in the mesh & is added to a vector
        /// \param[out] _triangles A vector to hold all the Triangle instances obtained 
        /// from the vertices & indices of the mesh
        /// \param[in] _mesh Mesh object
        public: void GetMeshTriangles(std::vector<Triangle>& _triangles,
                const gz::common::Mesh* _mesh);

        /// \brief Function that calculates the mass, mass matrix & centre of 
        /// mass of a mesh using a vector of Triangles of the mesh
        /// \param[in] _triangles A vector of all the Triangles of the mesh
        /// \param[in] _density Density of the mesh
        /// \param[out] _massMatrix MassMatrix object to hold mass &
        /// moment of inertia of the mesh
        /// \param[out] _centreOfMass Pose3d object to hold the centre of 
        /// mass of the object
        public: void CalculateMassProperties(
          std::vector<Triangle>& _triangles, 
          double _density,
          gz::math::MassMatrix3d& _massMatrix,
          gz::math::Pose3d& _centreOfMass);

        /// \brief Overloaded () operator which allows an instance of this class
        /// to be registered as a Custom Inertia Calculator with libsdformat
        /// \param[in] _errors A vector of Errors object. Each object
        /// would contain an error code and an error message.
        /// \param _calculatorParams An instance of CustomInertiaCalcProperties.
        /// This instance can be used to access the data like density, properties
        /// of the mesh, etc.
        public: std::optional<gz::math::Inertiald> operator()
          (sdf::Errors& _errors, 
          const sdf::CustomInertiaCalcProperties& _calculatorParams);  
      };
    }
  }
}

#endif
