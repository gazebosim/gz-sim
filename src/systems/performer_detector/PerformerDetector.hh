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

#ifndef GZ_SIM_SYSTEMS_PERFORMERDETECTOR_HH_
#define GZ_SIM_SYSTEMS_PERFORMERDETECTOR_HH_

#include <map>
#include <memory>
#include <string>
#include <unordered_set>

#include <gz/math/AxisAlignedBox.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/Model.hh"
#include "gz/sim/System.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  /// \brief A system system that publishes on a topic when a performer enters
  /// or leaves a specified region.
  ///
  /// A performer is detected when a performer's volume, which is
  /// represented by a gz::math::AxisAlignedBox, intersects with the
  /// PerformerDetector's region, which is also represented by an
  /// gz::math::AxisAlignedBox. When a performer is detected, the system
  /// publishes a gz.msgs.Pose message with the pose of the detected
  /// performer with respect to the model containing the PerformerDetector.
  ///
  /// The name and id fields of the Pose message will be set to the name and the
  /// entity of the detected performer respectively. The header of the Pose
  /// message contains the time stamp of detection. The `data` field of the
  /// header will contain the key "frame_id" with a value set to the name of
  /// the model containing the PerformerDetector system and the key "state" with
  /// a value set to "1" if the performer is entering the detector's region and
  /// "0" if the performer is leaving the region. The `data` field of the
  /// header will also contain the key "count" with a value set to the
  /// number of performers currently in the region.
  ///
  /// The PerformerDetector has to be attached to a `<model>` and it's region
  /// is centered on the containing model's origin.
  ///
  /// The system does not assume that levels are enabled, but it does require
  /// performers to be specified.
  ///
  /// ## System Parameters
  ///
  /// - `<topic>`: Custom topic to be used for publishing when a performer is
  /// detected. If not set, the default topic with the following pattern would
  /// be used "/model/<model_name>/performer_detector/status". The topic type
  /// is gz.msgs.Pose
  ///
  /// - `<geometry>`: Detection region. Currently, only the `<box>` geometry is
  /// supported. The position of the geometry is derived from the pose of the
  /// containing model.
  ///
  /// - `<pose>`: Additional pose offset relative to the parent model's pose.
  /// This pose is added to the parent model pose when computing the
  /// detection region. Only the position component of the `<pose>` is used.
  ///
  /// - `<header_data>`: Zero or more key-value pairs that will be
  /// included in the header of the detection messages. A `<header_data>`
  /// element should have child `<key>` and `<value>` elements whose
  /// contents are interpreted as strings. Keys value pairs are stored in a
  /// map, which means the keys are unique.

  class PerformerDetector
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
                const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) final;

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

    /// \brief Gazebo communication publisher.
    private: transport::Node::Publisher pub;

    /// \brief Whether the system has been initialized
    private: bool initialized{false};

    /// \brief Additional pose offset for the plugin.
    private: math::Pose3d poseOffset;

    /// \brief Optional extra header data.
    private: std::map<std::string, std::string> extraHeaderData;
  };

  }
}
}
}

#endif
