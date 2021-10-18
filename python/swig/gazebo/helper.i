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

%module(directors="1") helperfixture
%{
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/HelperFixture.hh>
#include <ignition/gazebo/ServerConfig.hh>
%}

%feature("director") Callback;
%feature("nodirector") HelperFixture;

%include "stdint.i"
%include <std_string.i>

namespace ignition
{
  namespace gazebo
  {
    class Callback
    {
    public:
        Callback();

        virtual ~Callback();
        virtual void call_update(
          const ignition::gazebo::UpdateInfo &,
          const ignition::gazebo::EntityComponentManager &);
        virtual void call_update_no_const(
          const ignition::gazebo::UpdateInfo &,
          ignition::gazebo::EntityComponentManager &);
        virtual void call_configure_callback(
          const ignition::gazebo::Entity &,
          ignition::gazebo::EntityComponentManager &);
    };

    class HelperFixture
    {
      %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";
      public: explicit HelperFixture(const std::string &_path);

      public: explicit HelperFixture(const ignition::gazebo::ServerConfig &_config);

      public: virtual ~HelperFixture();

      public: void OnPostUpdate(Callback &_callback);

      public: void OnPreUpdate(Callback &_callback);

      public: void OnUpdate(Callback &_callback);

      public: void OnConfigure(Callback &_callback);

      public: HelperFixture &Finalize();

      public: std::shared_ptr<ignition::gazebo::Server> Server() const;
    };
  }
}
