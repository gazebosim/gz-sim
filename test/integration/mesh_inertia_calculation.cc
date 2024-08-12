/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include <gtest/gtest.h>
#include <optional>
#include <string>

#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>

#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>

#include <gz/common/Filesystem.hh>

#include <gz/math/Inertial.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Matrix3.hh>
#include <gz/math/Vector3.hh>

#include "test_config.hh"
#include "../helpers/EnvTestFixture.hh"
#include "../helpers/Relay.hh"

using namespace gz;
using namespace sim;

/// \brief Test Mesh Inertia Calculation
class MeshInertiaCalculationTest : public InternalFixture<::testing::Test>
{
};

TEST(MeshInertiaCalculationTest, CylinderColladaMeshInertiaCalculation)
{
  size_t kIter = 100u;

  // Start server and run.
  gz::sim::ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(
      PROJECT_SOURCE_PATH, "test", "worlds", "mesh_inertia_calculation.sdf"));

  common::setenv(
      "GZ_SIM_RESOURCE_PATH",
      common::joinPaths(PROJECT_SOURCE_PATH, "test", "worlds", "models"));

  gz::sim::Server server(serverConfig);

  // Create a system just to get the ECM
  EntityComponentManager *ecm;
  test::Relay testSystem;
  testSystem.OnPreUpdate(
    [&](const UpdateInfo &, EntityComponentManager &_ecm)
    {
      ecm = &_ecm;
    }
  );
  server.AddSystem(testSystem.systemPtr);

  ASSERT_FALSE(server.Running());
  ASSERT_FALSE(*server.Running(0));
  ASSERT_TRUE(server.Run(true, kIter, false));
  ASSERT_NE(nullptr, ecm);

  // Get link of collada cylinder
  gz::sim::Entity modelEntity = ecm->EntityByComponents(
    gz::sim::components::Name("cylinder_dae"),
    gz::sim::components::Model()
  );

  gz::sim::Model model = gz::sim::Model(modelEntity);
  ASSERT_TRUE(model.Valid(*ecm));

  gz::sim::Entity linkEntity = model.LinkByName(*ecm, "cylinder_dae");
  gz::sim::Link link = gz::sim::Link(linkEntity);
  ASSERT_TRUE(link.Valid(*ecm));

  // Enable checks for pose values
  link.EnableVelocityChecks(*ecm);

  ASSERT_NE(link.WorldInertiaMatrix(*ecm), std::nullopt);
  ASSERT_NE(link.WorldInertialPose(*ecm), std::nullopt);
  ASSERT_NE(link.WorldPose(*ecm), std::nullopt);

  // The cylinder has a radius of 1m, length of 2m, and density of 1240 kg/m³.
  // Volume: πr²h = 2π ≈ 6.283
  // Mass: ρV = (1240.0) * 2π ≈ 7791.1497
  // Ix = Iy : 1/12 * m(3r² + h²)  = m/12 * (3 + 4) ≈ 4544.83
  // Iz : ½mr² ≈ 3895.57
  gz::math::Inertiald meshInertial;
  meshInertial.SetMassMatrix(gz::math::MassMatrix3d(
      7791.1497,
      gz::math::Vector3d(4544.83, 4544.83, 3895.57),
      gz::math::Vector3d::Zero
    ));
    meshInertial.SetPose(gz::math::Pose3d::Zero);
  gz::math::Matrix3 inertiaMatrix = meshInertial.Moi();

  // Check the Inertia Matrix within a tolerance of 0.005 since we are
  // comparing a mesh cylinder with an ideal cylinder. For values more closer
  // to the ideal, a higher number of vertices would be required in mesh
  EXPECT_TRUE(
    link.WorldInertiaMatrix(*ecm).value().Equal(inertiaMatrix, 0.005));

  // Check the Inertial Pose and Link Pose
  EXPECT_EQ(link.WorldPose(*ecm).value(), gz::math::Pose3d::Zero);
  EXPECT_EQ(link.WorldInertialPose(*ecm).value(), gz::math::Pose3d::Zero);
}

TEST(MeshInertiaCalculationTest,
  CylinderColladaMeshWithNonCenterOriginInertiaCalculation)
{
  size_t kIter = 100u;

  // Start server and run.
  gz::sim::ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(
      PROJECT_SOURCE_PATH, "test", "worlds", "mesh_inertia_calculation.sdf"));

  common::setenv(
      "GZ_SIM_RESOURCE_PATH",
      common::joinPaths(PROJECT_SOURCE_PATH, "test", "worlds", "models"));

  gz::sim::Server server(serverConfig);

  // Create a system just to get the ECM
  EntityComponentManager *ecm;
  test::Relay testSystem;
  testSystem.OnPreUpdate(
    [&](const UpdateInfo &, EntityComponentManager &_ecm)
    {
      ecm = &_ecm;
    }
  );
  server.AddSystem(testSystem.systemPtr);

  ASSERT_FALSE(server.Running());
  ASSERT_FALSE(*server.Running(0));
  ASSERT_TRUE(server.Run(true, kIter, false));
  ASSERT_NE(nullptr, ecm);

  // Get link of collada cylinder
  gz::sim::Entity modelEntity = ecm->EntityByComponents(
    gz::sim::components::Name("cylinder_dae_bottom_origin"),
    gz::sim::components::Model()
  );

  gz::sim::Model model = gz::sim::Model(modelEntity);
  ASSERT_TRUE(model.Valid(*ecm));

  gz::sim::Entity linkEntity = model.LinkByName(*ecm,
    "cylinder_dae_bottom_origin");
  gz::sim::Link link = gz::sim::Link(linkEntity);
  ASSERT_TRUE(link.Valid(*ecm));

  // Enable checks for pose values
  link.EnableVelocityChecks(*ecm);

  ASSERT_NE(link.WorldInertiaMatrix(*ecm), std::nullopt);
  ASSERT_NE(link.WorldInertialPose(*ecm), std::nullopt);
  ASSERT_NE(link.WorldPose(*ecm), std::nullopt);

  // The cylinder has a radius of 1m, length of 2m, and density of 1240 kg/m³.
  // Volume: πr²h = 2π ≈ 6.283
  // Mass: ρV = (1240.0) * 2π ≈ 7791.1497
  // Ix = Iy : 1/12 * m(3r² + h²)  = m/12 * (3 + 4) ≈ 4544.83
  // Iz : ½mr² ≈ 3895.57
  gz::math::Inertiald meshInertial;
  meshInertial.SetMassMatrix(gz::math::MassMatrix3d(
      7791.1497,
      gz::math::Vector3d(4544.83, 4544.83, 3895.57),
      gz::math::Vector3d::Zero
    ));
    meshInertial.SetPose(gz::math::Pose3d::Zero);
  gz::math::Matrix3 inertiaMatrix = meshInertial.Moi();

  // Check the Inertia Matrix within a tolerance of 0.005 since we are
  // comparing a mesh cylinder with an ideal cylinder. For values more closer
  // to the ideal, a higher number of vertices would be required in mesh
  EXPECT_TRUE(
    link.WorldInertiaMatrix(*ecm).value().Equal(inertiaMatrix, 0.005));

  // Check the Inertial Pose and Link Pose
  EXPECT_EQ(link.WorldPose(*ecm).value(), gz::math::Pose3d::Zero);

  // Since the height of cylinder is 2m and origin is at center of bottom face
  // the center of mass (inertial pose) will be 1m above the ground
  EXPECT_EQ(link.WorldInertialPose(*ecm).value(),
    gz::math::Pose3d(0, 0, 1, 0, 0, 0));
}
