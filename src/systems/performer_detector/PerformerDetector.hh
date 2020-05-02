/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef IGNITION_GAZEBO_SYSTEMS_PERFORMERDETECTOR_HH_
#define IGNITION_GAZEBO_SYSTEMS_PERFORMERDETECTOR_HH_

#include <memory>
#include <string>
#include <unordered_set>

#include <ignition/transport/Node.hh>

#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/System.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{

  class IGNITION_GAZEBO_VISIBLE PerformerDetector
      : public System,
        public ISystemConfigure,
        public ISystemPostUpdate
  {
    /// Documentation inherited
    public: PerformerDetector() = default;

    /// Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) final;

    /// Documentation inherited
    public: void PostUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm) final;

    /// \brief Check if the entity has already been detected
    /// \param [in] _entity The entity to test
    /// \returns True if the entity has already been detected
    private: bool IsAlreadyDetected(const Entity &_entity) const;

    /// \brief Add the entity to the list of detected entities
    /// \param [in] _entity The entity to add
    private: void AddToDetected(const Entity &_entity);

    /// \brief Remove the entity from the list of detected entities
    /// \param [in] _entity The entity to remove
    private: void RemoveFromDetected(const Entity &_entity);

    /// \brief Publish the event that the entity is detected or no longer
    /// detected.
    /// \param [in] _entity The entity to report
    /// \param [in] _name The name of the entity that triggered the event
    /// \param [in] _state The new state of the detector
    /// \param [in] _pose The pose of the entity that triggered the event
    /// \param [in] _stamp Time stamp of the event
    private: void Publish(const Entity &_entity, const std::string &_name,
                          bool _state, const math::Pose3d &_pose,
                          const std::chrono::steady_clock::duration &_stamp);

    /// \brief Keeps a set of detected entities
    private: std::unordered_set<Entity> detectedEntities;

    /// \brief The model associated with this system.
    private: Model model;

    /// \brief Name of the detector used as the frame_id in published messages.
    private: std::string detectorName;

    /// \brief Detector region. Only a box geometry is supported
    private: math::AxisAlignedBox detectorGeometry;

    /// \brief Ignition communication publisher.
    private: transport::Node::Publisher pub;

    /// \brief Whether the system has been initialized
    private: bool initialized{false};
  };

  }
}
}
}

#endif
