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
#ifndef IGNITION_GAZEBO_COMPONENTS_MATERIAL_HH_
#define IGNITION_GAZEBO_COMPONENTS_MATERIAL_HH_

#include <memory>
#include <sdf/Material.hh>

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
  class MaterialPrivate;

  /// \brief This component holds an entity's material.
  class IGNITION_GAZEBO_VISIBLE Material
  {
    /// \brief Constructor
    /// \param[in] _material The entity's material
    public: explicit Material(const sdf::Material &_material);

    /// \brief Destructor
    public: virtual ~Material();

    /// \brief Copy Constructor
    /// \param[in] _material Material component to copy.
    public: Material(const Material &_material);

    /// \brief Move Constructor
    /// \param[in] _material Material component to move.
    public: Material(Material &&_material) noexcept;

    /// \brief Move assignment operator.
    /// \param[in] _material Material component to move.
    /// \return Reference to this.
    public: Material &operator=(Material &&_material);

    /// \brief Copy assignment operator.
    /// \param[in] _material Material component to copy.
    /// \return Reference to this.
    public: Material &operator=(const Material &_material);

    /// \brief Get the material data.
    /// \return The actual material string.
    public: const sdf::Material &Data() const;

    /// \brief Private data pointer.
    private: std::unique_ptr<MaterialPrivate> dataPtr;
  };
  }
}
}
}
#endif
