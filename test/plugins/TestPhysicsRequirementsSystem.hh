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
#ifndef GZ_SIM_TEST_TESTPHYSICSREQUIREMENTSSYSTEM_HH_
#define GZ_SIM_TEST_TESTPHYSICSREQUIREMENTSSYSTEM_HH_

#include <string>
#include <vector>

#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
/// \brief A system that declares physics requirements through the
/// ISystemPhysicsRequirements interface. It does nothing else, and is loaded
/// as a plugin to test that the system manager finds the interface.
class TestPhysicsRequirementsSystem :
  public System,
  public ISystemPhysicsRequirements
{
  // Documentation inherited
  public: std::vector<std::string> PhysicsRequirements() override
  {
    return {"feature_a", "feature_b"};
  }
};
}
}

#endif
