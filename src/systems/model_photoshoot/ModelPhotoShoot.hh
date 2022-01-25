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
#ifndef IGNITION_GAZEBO_SYSTEMS_MODELPHOTOSHOOT_HH_
#define IGNITION_GAZEBO_SYSTEMS_MODELPHOTOSHOOT_HH_

#include <sdf/sdf.hh>

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/rendering/Camera.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/transport/Node.hh>

namespace ignition {
namespace gazebo {
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems {
  /// \brief System that takes snapshots of an sdf model
  class ModelPhotoShoot : public ignition::gazebo::System,
                          public ignition::gazebo::ISystemConfigure {
  public:
    ModelPhotoShoot();

  public:
    ~ModelPhotoShoot();

  public:
    void Configure(const ignition::gazebo::Entity &_id,
                  const std::shared_ptr<const sdf::Element> &_sdf,
                  ignition::gazebo::EntityComponentManager &_ecm,
                  ignition::gazebo::EventManager &_eventMgr) final;

    void PerformPostRenderingOperations();

    void LoadModel(const ignition::gazebo::Entity &_entity,
                  ignition::gazebo::EntityComponentManager &_ecm);

    void SavePicture(const ignition::rendering::CameraPtr camera,
                    const ignition::math::Pose3d pose, const std::string name);

    /// \brief Name of the loaded model.
  private:
    std::string modelName;

    /// \brief Location of the modeal to load.
  private:
    std::string modelLocation;

    /// \brief Ignition publisher used to spawn the model.
  private:
    std::shared_ptr<ignition::transport::Node> factoryPub;

    /// \brief Connection to pre-render event callback.
  private:
    ignition::common::ConnectionPtr connection{nullptr};

    /// \brief Boolean to control we only take the pictures once.
  private:
    bool takePicture;

    /// \brief File to save translation data.
  private:
    std::ofstream savingFile;
  };
  } // namespace systems
} // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
} // namespace gazebo
} // namespace ignition
#endif
