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
#ifndef IGNITION_GAZEBO_TEST_MOCKSYSTEM_HH_
#define IGNITION_GAZEBO_TEST_MOCKSYSTEM_HH_

#include <memory>

#include "ignition/gazebo/System.hh"

namespace ignition {
  namespace gazebo {
    class MockSystem :
      public gazebo::System,
      public gazebo::ISystemConfigure,
      public gazebo::ISystemPreUpdate,
      public gazebo::ISystemUpdate,
      public gazebo::ISystemPostUpdate
    {
      public: MockSystem() = default;
      public: ~MockSystem() = default;
      public: size_t configureCallCount {0};
      public: size_t preUpdateCallCount {0};
      public: size_t updateCallCount {0};
      public: size_t postUpdateCallCount {0};

      public: using CallbackType = std::function<void(
              const gazebo::UpdateInfo &, gazebo::EntityComponentManager &)>;

      public: using CallbackTypeConst =
              std::function<void(const gazebo::UpdateInfo &,
                                 const gazebo::EntityComponentManager &)>;

      public: std::function<void(const Entity &,
                    const std::shared_ptr<const sdf::Element> &,
                    EntityComponentManager &,
                    EventManager &)>
                configureCallback;
      public: CallbackType preUpdateCallback;
      public: CallbackType updateCallback;
      public: CallbackTypeConst postUpdateCallback;

      public: void Configure(const Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                EntityComponentManager &_ecm,
                EventManager &_eventMgr) override final
              {
                ++this->configureCallCount;
                if (this->configureCallback)
                  this->configureCallback(_entity, _sdf, _ecm, _eventMgr);
              }

      public: void PreUpdate(const gazebo::UpdateInfo &_info,
                    gazebo::EntityComponentManager &_ecm) override final
              {
                ++this->preUpdateCallCount;
                if (this->preUpdateCallback)
                  this->preUpdateCallback(_info, _ecm);
              }

      public: void Update(const gazebo::UpdateInfo &_info,
                    gazebo::EntityComponentManager &_ecm) override final
              {
                ++this->updateCallCount;
                if (this->updateCallback)
                  this->updateCallback(_info, _ecm);
              }

      public: void PostUpdate(const gazebo::UpdateInfo &_info,
                  const gazebo::EntityComponentManager &_ecm) override final
              {
                ++this->postUpdateCallCount;
                if (this->postUpdateCallback)
                  this->postUpdateCallback(_info, _ecm);
              }
    };
  }
}

#endif
