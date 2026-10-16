/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
#ifndef GZ_SIM_SYSTEMS_PHYSICS_FRAME_DATA_WRITER_HH_
#define GZ_SIM_SYSTEMS_PHYSICS_FRAME_DATA_WRITER_HH_

#include <functional>
#include <optional>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/physics/FrameData.hh>

#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/config.hh"

namespace gz::sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems::physics_system
{
  /// \brief Compare two vectors with the tolerance used to decide whether a
  /// frame data component changed.
  /// \param[in] _a First vector.
  /// \param[in] _b Second vector.
  /// \return True if the vectors are equal within the tolerance.
  bool FrameDataVectorEqual(const math::Vector3d &_a, const math::Vector3d &_b);

  /// \brief Compare two poses with the tolerance used to decide whether a
  /// frame data component changed.
  /// \param[in] _a First pose.
  /// \param[in] _b Second pose.
  /// \return True if the poses are equal within the tolerance.
  bool FrameDataPoseEqual(const math::Pose3d &_a, const math::Pose3d &_b);

  /// \brief Compute the frame data of a point rigidly attached to a link.
  /// \param[in] _parentLink Entity of the link that the point is attached to.
  /// \param[in] _offset Pose of the point in the link frame.
  /// \return Frame data of the point relative to the world, or nullopt if
  /// the link is not simulated by the caller, in which case the point is
  /// skipped.
  using LinkFrameDataAtOffsetFn = std::function<
      std::optional<physics::FrameData3d>(
          const Entity &_parentLink, const math::Pose3d &_offset)>;

  /// \brief Write the frame data of a link to the ECM.
  ///
  /// The world pose, the world and body-fixed linear and angular velocities,
  /// and the world and body-fixed linear and angular accelerations are
  /// written. A component is only written if another system has already
  /// created it on the link, so links that nobody reads cost nothing. A
  /// written component is marked as changed only if its value moved by more
  /// than a small tolerance.
  ///
  /// This function does not write the link's Pose component, which depends on
  /// the pose of the link's model.
  ///
  /// \param[in,out] _ecm Mutable reference to the ECM.
  /// \param[in] _link Link entity.
  /// \param[in] _frameData Frame data of the link relative to the world.
  void WriteLinkFrameData(EntityComponentManager &_ecm, const Entity &_link,
      const physics::FrameData3d &_frameData);

  /// \brief Write the frame data of entities that are attached to a link,
  /// such as sensors and collisions.
  ///
  /// An entity is any entity with Pose and ParentEntity components. For each
  /// of the nine frame data components that other systems have created on it
  /// (see WriteLinkFrameData), the component is set to the frame data of the
  /// entity's pose in its parent link. The components are set directly. This
  /// matches the Physics system, which does not compare them with their
  /// previous values or mark them as changed.
  ///
  /// \param[in,out] _ecm Mutable reference to the ECM.
  /// \param[in] _frameDataAtOffset Returns the frame data of a point on a
  /// link. It is called once for each entity and component it visits.
  void WriteChildFrameData(EntityComponentManager &_ecm,
      const LinkFrameDataAtOffsetFn &_frameDataAtOffset);
}
}
}

#endif
