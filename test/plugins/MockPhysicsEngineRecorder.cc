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

// The record of the worlds that MockPhysicsEngine received. It is a library
// of its own, without a plugin, so a test can link it and share the record
// with the plugin that the Physics system loads. Linking the plugin library
// into the test itself makes the system plugins of gz-sim fail to load.

#include "MockPhysicsEngine.hh"

#include <vector>

namespace gz
{
namespace sim
{
namespace test
{
namespace
{
std::vector<MockPhysicsEngineWorld> &Worlds()
{
  static std::vector<MockPhysicsEngineWorld> worlds;
  return worlds;
}
}  // namespace

//////////////////////////////////////////////////
void RecordMockPhysicsEngineWorld(const MockPhysicsEngineWorld &_world)
{
  Worlds().push_back(_world);
}

//////////////////////////////////////////////////
const std::vector<MockPhysicsEngineWorld> &MockPhysicsEngineWorlds()
{
  return Worlds();
}
}  // namespace test
}  // namespace sim
}  // namespace gz
