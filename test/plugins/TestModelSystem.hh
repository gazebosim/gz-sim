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
#ifndef IGNITION_GAZEBO_TEST_TESTMODELSYSTEM_HH_
#define IGNITION_GAZEBO_TEST_TESTMODELSYSTEM_HH_

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace gazebo
{
class TestModelSystem :
  public System,
  public ISystemConfigure
{
  public: TestModelSystem() = default;

  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &/*_eventManager*/) override
        {
          this->model = Model(_entity);

          auto link = this->model.LinkByName(_ecm, "link_1");

          // Fail to create component if link is not found
          if (link == kNullEntity)
          {
            ignerr << "Failed to find link" << std::endl;
            return;
          }

          auto value = _sdf->Get<int>("model_key");
          _ecm.CreateComponent<int>(_entity, value);
        }

  private: Model model;
};
}
}

#endif
