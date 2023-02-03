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

/*
 * Development of this module has been funded by the Monterey Bay Aquarium
 * Research Institute (MBARI) and the David and Lucile Packard Foundation
 */

#ifndef GZ_SIM_TEST_HELPERS_MODEL_OBSERVER_HH
#define GZ_SIM_TEST_HELPERS_MODEL_OBSERVER_HH

#include <chrono>
#include <string>
#include <deque>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

using namespace gz;

/// Helper class to observe a model in a Gazebo simulation.
class ModelObserver
{
  /// Constructor.
  /// \param[in] _modelName Name of the model to observe.
  /// \param[in] _baseLinkName Name of the model's base link
  /// (to be used to measure linear and angular velocity).
  public: ModelObserver(
    const std::string &_modelName,
    const std::string &_baseLinkName = "base_link")
    : modelName(_modelName), baseLinkName(_baseLinkName)
  {
  }

  /// Limit buffers' window size.
  /// \param[in] _windowSize Maximum window size as a duration
  /// measured against the simulation clock.
  public: void LimitTo(std::chrono::steady_clock::duration _windowSize)
  {
    this->windowSize = _windowSize;
  }

  /// Update internal state from simulation state.
  /// \note To be called on world pre-update.
  /// \param[in] _info Info for the current iteration.
  /// \param[in] _ecm Mutable reference to Entity component manager
  public: void PreUpdate(
    const sim::UpdateInfo &,
    sim::EntityComponentManager &_ecm)
  {
    sim::World world(sim::worldEntity(_ecm));

    const sim::Entity modelEntity =
        world.ModelByName(_ecm, this->modelName);
    if (sim::kNullEntity != modelEntity)
    {
      sim::Model model(modelEntity);
      const sim::Entity linkEntity =
          model.LinkByName(_ecm, this->baseLinkName);
      sim::Link link(linkEntity);
      link.EnableVelocityChecks(_ecm, true);
    }
  }

  /// Update internal state from simulation state.
  /// \note To be called on world post-update.
  /// \param[in] _info Info for the current iteration.
  /// \param[in] _ecm Entity component manager to be queried.
  public: void Update(
    const sim::UpdateInfo &_info,
    const sim::EntityComponentManager &_ecm)
  {
    sim::World world(sim::worldEntity(_ecm));

    const sim::Entity modelEntity =
        world.ModelByName(_ecm, this->modelName);
    if (sim::kNullEntity != modelEntity)
    {
      this->times.push_back(_info.simTime);

      this->poses.push_back(
          sim::worldPose(modelEntity, _ecm));

      sim::Model model(modelEntity);
      const sim::Entity linkEntity =
          model.LinkByName(_ecm, this->baseLinkName);
      sim::Link link(linkEntity);

      auto linearVelocity = link.WorldLinearVelocity(_ecm);
      this->linearVelocities.push_back(
          linearVelocity.value_or(math::Vector3d::NaN));

      auto angularVelocity = link.WorldAngularVelocity(_ecm);
      this->angularVelocities.push_back(
          angularVelocity.value_or(math::Vector3d::NaN));

      if (this->windowSize != std::chrono::steady_clock::duration::zero())
      {
        while (this->times.back() - this->times.front() > this->windowSize)
        {
          this->times.pop_front();
          this->poses.pop_front();
          this->linearVelocities.pop_front();
          this->angularVelocities.pop_front();
        }
      }
    }
  }

  /// Returns simulation times at which the model was observed.
  public: const std::deque<std::chrono::steady_clock::duration> &Times() const
  {
    return this->times;
  }

  /// Returns model world poses seen.
  public: const std::deque<math::Pose3d> &Poses() const
  {
    return this->poses;
  }

  /// Returns model linear velocities seen.
  public: const std::deque<math::Vector3d> &LinearVelocities() const
  {
    return this->linearVelocities;
  }

  /// Returns model angular velocities seen.
  public: const std::deque<math::Vector3d> &AngularVelocities() const
  {
    return this->angularVelocities;
  }

  private: std::string modelName;

  private: std::string baseLinkName;

  private: std::chrono::steady_clock::duration windowSize{
      std::chrono::steady_clock::duration::zero()};

  private: std::deque<std::chrono::steady_clock::duration> times;

  private: std::deque<math::Pose3d> poses;

  private: std::deque<math::Vector3d> sphericalCoordinates;

  private: std::deque<math::Vector3d> linearVelocities;

  private: std::deque<math::Vector3d> angularVelocities;
};

#endif // GZ_SIM_TEST_HELPERS_MODEL_OBSERVER_HH
