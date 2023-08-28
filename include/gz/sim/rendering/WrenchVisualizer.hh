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
#ifndef GZ_SIM_WRENCHVISUALIZER_HH_
#define GZ_SIM_WRENCHVISUALIZER_HH_

#include <memory>

#include <gz/sim/config.hh>
#include <gz/sim/rendering/Export.hh>
#include <gz/utils/ImplPtr.hh>

#include <gz/math/Vector3.hh>
#include <gz/rendering/RenderTypes.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE
{
namespace detail
{
  /// \brief Creates, deletes, and maintains force and torque visuals
  class GZ_SIM_RENDERING_VISIBLE WrenchVisualizer
  {
    /// \brief Constructor
    public: WrenchVisualizer();

    /// \brief Destructor
    public: ~WrenchVisualizer();

    /// \brief Initialize the Wrench visualizer
    /// \param[in] _scene The rendering scene where the visuals are created
    /// \return True if the scene is valid
    bool Init(rendering::ScenePtr _scene);

    /// \brief Create a new force visual
    /// \param[in] _material The material used for the visual
    /// \return Pointer to the created ArrowVisual
    public: rendering::ArrowVisualPtr CreateForceVisual(
      rendering::MaterialPtr _material);

    /// \brief Create a new torque visual
    /// \param[in] _material The material used for the visual
    /// \return Pointer to the created Visual
    public: rendering::VisualPtr CreateTorqueVisual(
      rendering::MaterialPtr _material);

    /// \brief Update the visual of a vector to match its direction and position
    /// \param[in] _visual Pointer to the vector visual to be updated
    /// \param[in] _direction Direction of the vector
    /// \param[in] _position Position of the arrow
    /// \param[in] _size Size of the arrow in meters
    /// \param[in] _tip True if _position specifies the tip of the vector,
    /// false if it specifies tha base of the vector
    public: void UpdateVectorVisual(rendering::VisualPtr _visual,
                                    const math::Vector3d &_direction,
                                    const math::Vector3d &_position,
                                    const double _size,
                                    const bool _tip = false);

    /// \internal
    /// \brief Private data pointer
    GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}
}
}
}
#endif
