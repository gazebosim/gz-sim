/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_COMPONENTS_GEOMETRY_HH_
#define IGNITION_GAZEBO_COMPONENTS_GEOMETRY_HH_

#include <memory>
#include <sdf/Geometry.hh>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>

namespace ignition
{
namespace gazebo
{
namespace components
{
  // Inline bracket to help doxygen filtering.
  inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
  // Forward declarations.
  class GeometryPrivate;

  /// \brief This component holds an entity's geometry.
  class IGNITION_GAZEBO_VISIBLE Geometry
  {
    /// \brief Constructor
    /// \param[in] _geometry The entity's geometry
    public: explicit Geometry(const sdf::Geometry &_geometry);

    /// \brief Destructor
    public: virtual ~Geometry();

    /// \brief Copy Constructor
    /// \param[in] _geometry Geometry component to copy.
    public: Geometry(const Geometry &_geometry);

    /// \brief Move Constructor
    /// \param[in] _geometry Geometry component to move.
    public: Geometry(Geometry &&_geometry) noexcept;

    /// \brief Move assignment operator.
    /// \param[in] _geometry Geometry component to move.
    /// \return Reference to this.
    public: Geometry &operator=(Geometry &&_geometry);

    /// \brief Copy assignment operator.
    /// \param[in] _geometry Geometry component to copy.
    /// \return Reference to this.
    public: Geometry &operator=(const Geometry &_geometry);

    /// \brief Get the geometry data.
    /// \return The actual geometry string.
    public: const sdf::Geometry &Data() const;

    /// \brief Private data pointer.
    private: std::unique_ptr<GeometryPrivate> dataPtr;
  };
  }
}
}
}
#endif
