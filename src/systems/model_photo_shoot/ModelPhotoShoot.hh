/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#ifndef GZ_SIM_SYSTEMS_MODELPHOTOSHOOT_HH_
#define GZ_SIM_SYSTEMS_MODELPHOTOSHOOT_HH_

#include <sdf/sdf.hh>

#include <memory>

#include "gz/sim/System.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declarations.
  class ModelPhotoShootPrivate;

  /// \brief This plugin is a port of the old ModelPropShop plugin from gazebo
  /// classic. It generates 5 pictures of a model: perspective, top, front,
  /// side and back. It can do it using the default position or moving the joint
  /// to random positions. It allows saving the camera and joint poses so it
  /// can be replicated in other systems.
  ///
  /// ## System Parameters
  /// - <translation_data_file> - Location to save the camera and joint poses.
  ///   [Optional]
  /// - <random_joints_pose> - Set to true to take pictures with the joints in
  ///   random poses instead of the default ones. This option only supports
  ///   single axis joints. [Optional]
  /// - A camera sensor must be set in the SDF file as it will be used by the
  ///   plugin to take the pictures. This allows the plugin user to set the
  ///   camera parameters as needed. [Required]
  ///
  /// ## Example
  /// An example configuration is installed with Gazebo. The example uses
  /// the Ogre2 rendering plugin to set the background color of the pictures.
  /// It also includes the camera sensor that will be used along with the
  /// corresponding parameters so they can be easily tunned.
  ///
  /// To run the example:
  /// ```
  /// gz sim model_photo_shoot.sdf -s -r --iterations 50
  /// ```
  /// This will start gazebo, load the model take the pictures and shutdown
  /// after 50 iterations. You will find the pictures in the same location you
  /// run the command.

  /// \brief System that takes snapshots of an sdf model
  class ModelPhotoShoot : public System,
                          public ISystemConfigure,
                          public ISystemPreUpdate
  {

    /// \brief Constructor
    public: ModelPhotoShoot();

    /// \brief Destructor
    public: ~ModelPhotoShoot() override = default;

    // Documentation inherited
    public: void Configure(const gz::sim::Entity &_id,
                    const std::shared_ptr<const sdf::Element> &_sdf,
                    gz::sim::EntityComponentManager &_ecm,
                    gz::sim::EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                    gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<ModelPhotoShootPrivate> dataPtr;
  };
}
}
}
}
#endif
