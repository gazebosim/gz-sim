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
#ifndef IGNITION_GAZEBO_TEST_TESTWORLDSYSTEM_HH_
#define IGNITION_GAZEBO_TEST_TESTWORLDSYSTEM_HH_

#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace gazebo
{
class TestWorldSystem :
  public System,
  public ISystemConfigure,
  public ISystemUpdate
{
  public: TestWorldSystem() = default;

  public: void Configure(const EntityId &_id,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &/*_eventManager*/) override
        {
          auto value = _sdf->Get<double>("world_key");
          _ecm.CreateComponent<double>(_id, value);
        }

  public: void Update(const gazebo::UpdateInfo &_info,
                      EntityComponentManager &) override
          {
            std::cout << "iteration " << _info.iterations << std::endl;
          }
};
}
}

#endif
