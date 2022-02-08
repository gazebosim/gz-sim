/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <ignition/math/Rand.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointPositionReset.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/plugin/Register.hh>

using namespace ignition;
using namespace gazebo;
namespace reset_plugin
{
class JointPositionRandomizer : public System,
                                public ISystemConfigure,
                                public ISystemReset
{
  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &,
                 EntityComponentManager &, EventManager &) override
  {
    this->targetEntity = _entity;
  }

  void Reset(const UpdateInfo &_info, EntityComponentManager &_ecm) override
  {
    std::cout << "Reset joints\n";
    auto joints = _ecm.EntitiesByComponents(
        components::ParentEntity(this->targetEntity), components::Joint());
    for (auto joint : joints)
    {
      double pos = math::Rand::DblUniform(0, IGN_PI);
      std::cout << "joint " << joint << " pos: " << pos << std::endl;
      _ecm.SetComponentData<components::JointPositionReset>(joint, {pos});
    }
  }

  private: Entity targetEntity;
};
}


IGNITION_ADD_PLUGIN(reset_plugin::JointPositionRandomizer,
                    ignition::gazebo::System,
                    reset_plugin::JointPositionRandomizer::ISystemConfigure,
                    reset_plugin::JointPositionRandomizer::ISystemReset)
