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
#include "PhysicsMode.hh"

#include <gtest/gtest.h>

#include <string>

using namespace gz;
using namespace sim::systems::simplified_physics;

/////////////////////////////////////////////////
TEST(PhysicsMode, Parse)
{
  PhysicsMode mode{PhysicsMode::Full};

  EXPECT_TRUE(ParseMode("planar_kinematic", mode));
  EXPECT_EQ(PhysicsMode::PlanarKinematic, mode);

  EXPECT_TRUE(ParseMode("planar_dynamic", mode));
  EXPECT_EQ(PhysicsMode::PlanarDynamic, mode);

  EXPECT_TRUE(ParseMode("full", mode));
  EXPECT_EQ(PhysicsMode::Full, mode);

  // An unknown name leaves the output alone.
  mode = PhysicsMode::PlanarKinematic;
  for (const std::string name : {"", "Full", "planar", "planar kinematic",
      "tier1"})
  {
    EXPECT_FALSE(ParseMode(name, mode)) << name;
    EXPECT_EQ(PhysicsMode::PlanarKinematic, mode) << name;
  }
}

/////////////////////////////////////////////////
TEST(PhysicsMode, NameRoundTrip)
{
  for (const auto mode : {PhysicsMode::Full, PhysicsMode::PlanarKinematic,
      PhysicsMode::PlanarDynamic})
  {
    PhysicsMode parsed{PhysicsMode::Full};
    EXPECT_TRUE(ParseMode(ModeName(mode), parsed));
    EXPECT_EQ(mode, parsed);
  }
}

/////////////////////////////////////////////////
TEST(PhysicsMode, SupportedFeatures)
{
  // No planar mode serves a feature yet. The list must stay empty rather
  // than absent, because an absent list means that every feature is served.
  EXPECT_TRUE(SupportedFeatures(PhysicsMode::PlanarKinematic).empty());
  EXPECT_TRUE(SupportedFeatures(PhysicsMode::PlanarDynamic).empty());
}

/////////////////////////////////////////////////
TEST(PhysicsMode, ProviderName)
{
  EXPECT_EQ("gz::sim::systems::SimplifiedPhysics",
      std::string(kProviderName));
}
