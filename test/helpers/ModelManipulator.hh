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

#ifndef GZ_SIM_TEST_HELPERS_MODEL_MANIPULATOR_HH
#define GZ_SIM_TEST_HELPERS_MODEL_MANIPULATOR_HH

#include <optional>
#include <string>

#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>

using namespace gz;

/// \brief Helper class to manipulate a model in an Gazebo simulation.
class ModelManipulator
{
  /// \brief Constructor.
  /// \param[in] _modelName Name of the model to manipulate.
  public: ModelManipulator(const std::string &_modelName)
    : modelName(_modelName)
  {
  }

  /// \brief Update simulation state as necessary.
  /// \note To be called on world pre-update.
  /// \param[in] _ecm Entity component manager to update.
  public: void Update(sim::EntityComponentManager &_ecm)
  {
    if (this->linearVelocityRequest.has_value())
    {
      sim::World world(sim::worldEntity(_ecm));
      sim::Model model(world.ModelByName(_ecm, this->modelName));
      using LinearVelocityCmd = sim::components::LinearVelocityCmd;
      auto velocity = _ecm.Component<LinearVelocityCmd>(model.Entity());
      if (!velocity)
      {
        _ecm.CreateComponent(
          model.Entity(), LinearVelocityCmd(
            this->linearVelocityRequest.value()));
      }
      else
      {
        velocity->Data() = this->linearVelocityRequest.value();
      }
    }

    if (this->angularVelocityRequest.has_value())
    {
      sim::World world(sim::worldEntity(_ecm));
      sim::Model model(world.ModelByName(_ecm, this->modelName));
      using AngularVelocityCmd = sim::components::AngularVelocityCmd;
      auto velocity = _ecm.Component<AngularVelocityCmd>(model.Entity());
      if (!velocity)
      {
        _ecm.CreateComponent(
          model.Entity(), AngularVelocityCmd(
            this->angularVelocityRequest.value()));
      }
      else
      {
        velocity->Data() = this->angularVelocityRequest.value();
      }
    }
  }

  /// \brief Set model linear velocity in simulation.
  /// \param[in] _linearVelocity Model linear velocity in the world frame.
  public: void SetLinearVelocity(const math::Vector3d &_linearVelocity)
  {
    this->linearVelocityRequest = _linearVelocity;
  }

  /// \brief Set model angular velocity in simulation.
  /// \param[in] _angularVelocity Model angular velocity in the world frame.
  public: void SetAngularVelocity(const math::Vector3d &_angularVelocity)
  {
    this->angularVelocityRequest = _angularVelocity;
  }

  /// \brief Name of model
  private: std::string modelName;

  /// \brief Request to set linear velocity
  private: std::optional<math::Vector3d> linearVelocityRequest;

  /// \brief Request to set angular velocity
  private: std::optional<math::Vector3d> angularVelocityRequest;
};

#endif // GZ_SIM_TEST_HELPERS_MODEL_MANIPULATOR_HH
