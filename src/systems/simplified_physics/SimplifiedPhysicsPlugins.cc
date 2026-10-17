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

// A plugin library has a single registration hook, so the plugins of this
// library are registered together here.

#include <gz/plugin/Register.hh>

#include "PhysicsMode.hh"
#include "SimplifiedPhysics.hh"
#include "SimplifiedPhysicsModel.hh"

using namespace gz;
using namespace sim;
using namespace systems;

GZ_ADD_PLUGIN(SimplifiedPhysics,
              System,
              SimplifiedPhysics::ISystemConfigure,
              SimplifiedPhysics::ISystemConfigurePriority,
              SimplifiedPhysics::ISystemReset,
              SimplifiedPhysics::ISystemUpdate)

GZ_ADD_PLUGIN_ALIAS(SimplifiedPhysics,
                    simplified_physics::kProviderName)

GZ_ADD_PLUGIN(SimplifiedPhysicsModel,
              System,
              SimplifiedPhysicsModel::ISystemConfigure)

GZ_ADD_PLUGIN_ALIAS(SimplifiedPhysicsModel,
                    "gz::sim::systems::SimplifiedPhysicsModel")
