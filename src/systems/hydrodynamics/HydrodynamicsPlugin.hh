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

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/components.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs.hh>
#include <mutex>
#include <string>

namespace tethys_hydro 
{
  class HydrodynamicsPrivateData;

  /// This class provides hydrodynamic behaviour for underwater vehicles
  /// It is shamelessly based off Brian Bingham's plugin for VRX
  /// which in turn is based of fossen's equations.
  class HydrodynamicsPlugin:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate
  {
  public: HydrodynamicsPlugin();

  public: ~HydrodynamicsPlugin() override;  
  
  public: void Configure(
      const ignition::gazebo::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      ignition::gazebo::EntityComponentManager &_ecm,
      ignition::gazebo::EventManager &/*_eventMgr*/
      );  
  
  public: void PreUpdate(
      const ignition::gazebo::UpdateInfo &_info,
      ignition::gazebo::EntityComponentManager &_ecm);  
  
  private: std::unique_ptr<HydrodynamicsPrivateData> dataPtr;
  };
}
