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
#include "FrameDataWriter.hh"

#include <gz/math/eigen3/Conversions.hh>

#include "gz/sim/components/AngularAcceleration.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/LinearAcceleration.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Pose.hh"

namespace gz::sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems::physics_system
{
namespace
{
/// \brief Set a component of a link if it exists, and mark it changed only if
/// its value changed.
/// \tparam ComponentT Component type.
/// \tparam EqualFn Equality function of the component's data.
/// \tparam ValueFn Function that returns the new data.
/// \param[in,out] _ecm Mutable reference to the ECM.
/// \param[in] _entity Entity that may have the component.
/// \param[in] _equal Equality function.
/// \param[in] _value Called only if the component exists.
template<typename ComponentT, typename EqualFn, typename ValueFn>
void SetIfPresent(EntityComponentManager &_ecm, const Entity &_entity,
    const EqualFn &_equal, const ValueFn &_value)
{
  auto comp = _ecm.Component<ComponentT>(_entity);
  if (!comp)
    return;

  auto state = comp->SetData(_value(), _equal) ?
      ComponentState::PeriodicChange : ComponentState::NoChange;
  _ecm.SetChanged(_entity, ComponentT::typeId, state);
}

/// \brief Set a component of every sensor, collision or other entity that is
/// attached to a link.
/// \tparam ComponentT Component type.
/// \tparam ValueFn Function that returns the new data from the frame data.
/// \param[in,out] _ecm Mutable reference to the ECM.
/// \param[in] _frameDataAtOffset Frame data of a point on a link.
/// \param[in] _value Computes the data from the frame data of the entity.
template<typename ComponentT, typename ValueFn>
void SetChildren(EntityComponentManager &_ecm,
    const LinkFrameDataAtOffsetFn &_frameDataAtOffset, const ValueFn &_value)
{
  _ecm.Each<components::Pose, ComponentT, components::ParentEntity>(
      [&](const Entity &,
          const components::Pose *_pose, ComponentT *_comp,
          const components::ParentEntity *_parent) -> bool
      {
        // The parent is a link if the entity is a sensor or a collision
        if (auto frameData = _frameDataAtOffset(_parent->Data(), _pose->Data()))
          *_comp = ComponentT(_value(*frameData));

        return true;
      });
}
}

//////////////////////////////////////////////////
bool FrameDataVectorEqual(const math::Vector3d &_a, const math::Vector3d &_b)
{
  return _a.Equal(_b, 1e-6);
}

//////////////////////////////////////////////////
bool FrameDataPoseEqual(const math::Pose3d &_a, const math::Pose3d &_b)
{
  return _a.Pos().Equal(_b.Pos(), 1e-6) && _a.Rot().Equal(_b.Rot(), 1e-6);
}

//////////////////////////////////////////////////
void WriteLinkFrameData(EntityComponentManager &_ecm, const Entity &_link,
    const physics::FrameData3d &_frameData)
{
  // World frame
  SetIfPresent<components::WorldPose>(_ecm, _link, FrameDataPoseEqual,
      [&]() { return math::eigen3::convert(_frameData.pose); });

  SetIfPresent<components::WorldLinearVelocity>(_ecm, _link,
      FrameDataVectorEqual,
      [&]() { return math::eigen3::convert(_frameData.linearVelocity); });

  SetIfPresent<components::WorldAngularVelocity>(_ecm, _link,
      FrameDataVectorEqual,
      [&]() { return math::eigen3::convert(_frameData.angularVelocity); });

  SetIfPresent<components::WorldLinearAcceleration>(_ecm, _link,
      FrameDataVectorEqual,
      [&]() { return math::eigen3::convert(_frameData.linearAcceleration); });

  SetIfPresent<components::WorldAngularAcceleration>(_ecm, _link,
      FrameDataVectorEqual,
      [&]() { return math::eigen3::convert(_frameData.angularAcceleration); });

  // Body-fixed frame
  const Eigen::Matrix3d R_bs = _frameData.pose.linear().transpose(); // NOLINT

  SetIfPresent<components::LinearVelocity>(_ecm, _link, FrameDataVectorEqual,
      [&]()
      {
        return math::eigen3::convert(
            Eigen::Vector3d(R_bs * _frameData.linearVelocity));
      });

  SetIfPresent<components::AngularVelocity>(_ecm, _link, FrameDataVectorEqual,
      [&]()
      {
        return math::eigen3::convert(
            Eigen::Vector3d(R_bs * _frameData.angularVelocity));
      });

  SetIfPresent<components::LinearAcceleration>(_ecm, _link,
      FrameDataVectorEqual,
      [&]()
      {
        return math::eigen3::convert(
            Eigen::Vector3d(R_bs * _frameData.linearAcceleration));
      });

  SetIfPresent<components::AngularAcceleration>(_ecm, _link,
      FrameDataVectorEqual,
      [&]()
      {
        return math::eigen3::convert(
            Eigen::Vector3d(R_bs * _frameData.angularAcceleration));
      });
}

//////////////////////////////////////////////////
void WriteChildFrameData(EntityComponentManager &_ecm,
    const LinkFrameDataAtOffsetFn &_frameDataAtOffset)
{
  // World frame
  SetChildren<components::WorldPose>(_ecm, _frameDataAtOffset,
      [](const physics::FrameData3d &_data)
      {
        return math::eigen3::convert(_data.pose);
      });

  SetChildren<components::WorldLinearVelocity>(_ecm, _frameDataAtOffset,
      [](const physics::FrameData3d &_data)
      {
        return math::eigen3::convert(_data.linearVelocity);
      });

  SetChildren<components::WorldAngularVelocity>(_ecm, _frameDataAtOffset,
      [](const physics::FrameData3d &_data)
      {
        return math::eigen3::convert(_data.angularVelocity);
      });

  SetChildren<components::WorldLinearAcceleration>(_ecm, _frameDataAtOffset,
      [](const physics::FrameData3d &_data)
      {
        return math::eigen3::convert(_data.linearAcceleration);
      });

  SetChildren<components::WorldAngularAcceleration>(_ecm, _frameDataAtOffset,
      [](const physics::FrameData3d &_data)
      {
        return math::eigen3::convert(_data.angularAcceleration);
      });

  // Body-fixed frame: rotate the world frame vectors by the inverse of the
  // entity's world orientation.
  SetChildren<components::LinearVelocity>(_ecm, _frameDataAtOffset,
      [](const physics::FrameData3d &_data)
      {
        return math::eigen3::convert(_data.pose).Rot().RotateVectorReverse(
            math::eigen3::convert(_data.linearVelocity));
      });

  SetChildren<components::AngularVelocity>(_ecm, _frameDataAtOffset,
      [](const physics::FrameData3d &_data)
      {
        return math::eigen3::convert(_data.pose).Rot().RotateVectorReverse(
            math::eigen3::convert(_data.angularVelocity));
      });

  SetChildren<components::LinearAcceleration>(_ecm, _frameDataAtOffset,
      [](const physics::FrameData3d &_data)
      {
        return math::eigen3::convert(_data.pose).Rot().RotateVectorReverse(
            math::eigen3::convert(_data.linearAcceleration));
      });

  SetChildren<components::AngularAcceleration>(_ecm, _frameDataAtOffset,
      [](const physics::FrameData3d &_data)
      {
        return math::eigen3::convert(_data.pose).Rot().RotateVectorReverse(
            math::eigen3::convert(_data.angularAcceleration));
      });
}
}
}
}
