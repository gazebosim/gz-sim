/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#include <gz/msgs/boolean.pb.h>
#include <memory>
#include <gz/common/Profiler.hh>
#include <gz/common/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Conversions.hh>
#include <sdf/sdf.hh>
#include "LockStep.hh"

using namespace gz;
using namespace sim;
using namespace systems;

//////////////////////////////////////////////////
LockStep::~LockStep()
{
}

//////////////////////////////////////////////////
void LockStep::Configure(const gz::sim::Entity &_entity,
                        const std::shared_ptr<const sdf::Element> &/*_element*/,
                        gz::sim::EntityComponentManager &_ecm,
                        gz::sim::EventManager &/*_eventManager*/)
{
  std::cerr << "LockStep::Configure() on entity: " << _entity << std::endl;

  // Populate the message with the full ECM state.
  this->stepMsg.Clear();
  _ecm.State(*this->stepMsg.mutable_state(), {}, {}, true);

  gz::msgs::Boolean rep;
  bool res;
  unsigned int timeout = 5000;

  // Request the PreUpdate plugin service.
  bool executed = node.Request("/Configure", this->stepMsg, timeout, rep, res);
  if (!executed)
  {
    std::cerr << "LockStep::Configure() timeout" << std::endl;
    return;
  }

  if (res)
  {
    std::cout << "LockStep::Configure() Correctly configured" << std::endl;
    this->configured = true;
  }
  else
    std::cout << "LockStep::Configure() failed" << std::endl;
}

//////////////////////////////////////////////////
void LockStep::PreUpdate(const gz::sim::UpdateInfo &_info,
                         gz::sim::EntityComponentManager &_ecm)
{
  if (!this->configured)
    return;

  // Populate the message with the full ECM state.
  this->stepMsg.Clear();
  gz::sim::set(this->stepMsg.mutable_stats(), _info);
  _ecm.State(*this->stepMsg.mutable_state(), {}, {}, true);

  gz::msgs::Boolean rep;
  bool res;
  unsigned int timeout = 5000;

  // Request the PreUpdate plugin service.
  bool executed = node.Request("/PreUpdate", this->stepMsg, timeout, rep, res);
  if (!executed)
  {
    std::cerr << "Preupdate service call timed out" << std::endl;
    return;
  }

  if (res)
    std::cout << "Preupdate response: [" << rep.data() << "]" << std::endl;
  else
    std::cout << "Preupdate service call failed" << std::endl;
}

//////////////////////////////////////////////////
void LockStep::Update(const gz::sim::UpdateInfo &_info,
                      gz::sim::EntityComponentManager &_ecm)
{
  if (!this->configured)
    return;

  // Populate the message with the full ECM state.
  this->stepMsg.Clear();
  gz::sim::set(this->stepMsg.mutable_stats(), _info);
  _ecm.State(*this->stepMsg.mutable_state(), {}, {}, true);

  gz::msgs::Boolean rep;
  bool res;
  unsigned int timeout = 5000;

  // Request the Update plugin service.
  bool executed = node.Request("/Update", this->stepMsg, timeout, rep, res);
  if (!executed)
  {
    std::cerr << "Update service call timed out" << std::endl;
    return;
  }

  if (res)
    std::cout << "Update response: [" << rep.data() << "]" << std::endl;
  else
    std::cout << "Update service call failed" << std::endl;
}

//////////////////////////////////////////////////
void LockStep::PostUpdate(const gz::sim::UpdateInfo &_info,
                          const gz::sim::EntityComponentManager &_ecm)
{
  if (!this->configured)
    return;

  // Populate the message with the full ECM state.
  this->stepMsg.Clear();
  gz::sim::set(this->stepMsg.mutable_stats(), _info);
  _ecm.State(*this->stepMsg.mutable_state(), {}, {}, true);

  gz::msgs::Boolean rep;
  bool res;
  unsigned int timeout = 5000;

  // Request the Update plugin service.
  bool executed = node.Request("/PostUpdate", this->stepMsg, timeout, rep, res);
  if (!executed)
  {
    std::cerr << "PostUpdate service call timed out" << std::endl;
    return;
  }

  if (res)
    std::cout << "PostUpdate response: [" << rep.data() << "]" << std::endl;
  else
    std::cout << "PostUpdate service call failed" << std::endl;
}

//////////////////////////////////////////////////
void LockStep::Reset(const gz::sim::UpdateInfo &/*_info*/,
                     gz::sim::EntityComponentManager &/*_ecm*/)
{
  gzdbg << "LockStep::Reset" << std::endl;
}

GZ_ADD_PLUGIN(LockStep,
              System,
              LockStep::ISystemConfigure,
              LockStep::ISystemPreUpdate,
              LockStep::ISystemUpdate,
              LockStep::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(LockStep,
                    "gz::sim::systems::LockStep")
