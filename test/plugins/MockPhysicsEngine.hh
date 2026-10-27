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
#ifndef GZ_SIM_TEST_MOCKPHYSICSENGINE_HH_
#define GZ_SIM_TEST_MOCKPHYSICSENGINE_HH_

#include <string>
#include <vector>

#include <gz/math/Vector3.hh>

namespace gz
{
namespace sim
{
namespace test
{
/// \brief What the mock physics engine saw in an sdf::World passed to
/// ConstructWorld.
struct MockPhysicsEngineWorld
{
  /// \brief Name of the world.
  std::string name;

  /// \brief Gravity of the world.
  gz::math::Vector3d gravity;

  /// \brief Number of <physics> profiles in the world.
  std::size_t physicsCount{0};

  /// \brief Name of the first <physics> profile.
  std::string physicsName;

  /// \brief Value of <max_step_size> of the first <physics> profile.
  double maxStepSize{0.0};

  /// \brief Value of <real_time_factor> of the first <physics> profile.
  double realTimeFactor{0.0};

  /// \brief Whether the first <physics> profile has a <gz:mock_engine>
  /// custom element.
  bool hasCustomElement{false};

  /// \brief Value of the <gz:mock_engine> custom element.
  std::string customElementValue;
};

/// \brief Record a world that the mock physics engine plugin received.
/// \param[in] _world What the plugin saw in the world.
void RecordMockPhysicsEngineWorld(const MockPhysicsEngineWorld &_world);

/// \brief Worlds the mock physics engine plugin has received so far in this
/// process, in the order they were constructed.
const std::vector<MockPhysicsEngineWorld> &MockPhysicsEngineWorlds();
}
}
}

#endif
