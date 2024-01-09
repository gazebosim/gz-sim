/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#define _USE_MATH_DEFINES
#include <cmath>

#include <gtest/gtest.h>

#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/System.hh>

#include <gz/utils/ExtraTestMacros.hh>

#include "../helpers/EnvTestFixture.hh"

// World file path.
const char *kWorldFilePath{"/test/worlds/added_mass_ellipsoids.sdf"};

// Names of the models.
const std::vector<std::string> kModelNames = {"body1", "body2", "body3"};

// Link name (all models use the same name).
const char* kLinkName = "link";

// Update rate of the server.
const double kRate = 1000;

// Force excitation amplitude and direction.
const double kForceVec[3] = {2000, 2000, 0};

// Force excitation angular velocity [rad / s].
const double kForceAngVel = 3 * GZ_PI;

// Torque excitation amplitude and direction.
const double kTorqueVec[3] = {200, 200, 0};

// Torque excitation angular velocity [rad / s].
const double kTorqueAngVel = 2 * GZ_PI;

// Total duration of the motion in iterations.
const uint64_t kIter = 1000;

// Tolerances.
/*  \TODO(mjcarroll) These tolerances are completely suitable for Jammy,
    but are too tight on focal.
const struct {double pos, angle, axis, lin_vel, ang_vel;} kTols = {
  1e-3,  // pos
  1e-5,  // angle
  1e-4,  // axis
  1e-5,  // lin_vel
  1e-4  // ang_vel
};
*/

const struct {double pos, angle, axis, lin_vel, ang_vel;} kTols = {
  1e-3,  // pos
  1e-5,  // angle
  1e-4,  // axis
  2e-5,  // lin_vel
  1e-4  // ang_vel
};

// Struct to store model/link state.
struct BodyState {
  gz::math::Vector3d pos;
  double angle;
  gz::math::Vector3d axis;
  gz::math::Vector3d lin_vel;
  gz::math::Vector3d ang_vel;
};

// Mapping from model name to expected state.
const std::map<std::string, BodyState> kExpectedStates =
{
  {
    "body1",
    {
      // pos
      gz::math::Vector3d(
        0.1615036362557803,
        0.10710173528687592,
        -0.0024940422102210013
      ),
      // angle
      0.31351100787675423,
      // axis
      gz::math::Vector3d(
        0.9487133476959254,
        0.2967465880246766,
        -0.10901580802481375
      ),
      // lin_vel
      gz::math::Vector3d(
        0.32087488227940963,
        0.212772641112832,
        -0.010283659814296216
      ),
      // ang_vel
      gz::math::Vector3d(
        0.0031500380024329826,
        0.0022451901112414984,
        -0.052564274312245286
      )
    }
  },
  {
    "body2",
    {
      // pos
      gz::math::Vector3d(
        0.19339397146633158,
        0.18238590690754014,
        -0.0006146904921505786
      ),
      // angle
      0.3185857744101227,
      // axis
      gz::math::Vector3d(
        0.9309057658841812,
        0.3615623345327693,
        -0.051837566405069514
      ),
      // lin_vel
      gz::math::Vector3d(
        0.386607475883442,
        0.36477533916100213,
        -0.002544369239292084
      ),
      // ang_vel
      gz::math::Vector3d(
        0.0005006537195657914,
        0.0007044864197475587,
        -0.012889587224151892
      )
    }
  },
  {
    "body3",
    {
      // pos
      gz::math::Vector3d(
        0.0616809222579082,
        0.020951194425796703,
        -0.0006836779301496875
      ),
      // angle
      0.2990323477528498,
      // axis
      gz::math::Vector3d(
        0.992866493007573,
        0.11317163977508175,
        -0.03752741682373132
      ),
      // lin_vel
      gz::math::Vector3d(
        0.12296544056644206,
        0.04157265120194949,
        -0.002832810385198371
      ),
      // ang_vel
      gz::math::Vector3d(
        0.0017159692398096078,
        0.00022402938638815302,
        -0.014274584996716126
      )
    }
  },
};

// Plugin that applies a sinusoidal wrench.
class SinusoidalWrenchPlugin:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
  public: void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr
  ) override;

  public: void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm
  ) override;

  public: gz::sim::EntityComponentManager *ecm{nullptr};

  public: std::map<std::string, gz::sim::Entity> link_entities;
};

// Store a reference to the ECM and to the link entities.
//
// The reference to the ECM is stored so it can be retrieved after the server
// has run.
void SinusoidalWrenchPlugin::Configure(
  const gz::sim::Entity &,
  const std::shared_ptr<const sdf::Element> &,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &
)
{
  this->ecm = &_ecm;

  for (std::string model_name : kModelNames) {
    gz::sim::Entity model_entity = _ecm.EntityByComponents(
      gz::sim::components::Name(model_name),
      gz::sim::components::Model()
    );
    ASSERT_NE(model_entity, gz::sim::kNullEntity);

    gz::sim::Model model = gz::sim::Model(model_entity);
    ASSERT_TRUE(model.Valid(_ecm));

    this->link_entities[model_name] = model.LinkByName(_ecm, kLinkName);
    ASSERT_NE(this->link_entities[model_name], gz::sim::kNullEntity);
    _ecm.CreateComponent(
      this->link_entities[model_name],
      gz::sim::components::WorldPose()
    );

    gz::sim::Link link = gz::sim::Link(this->link_entities[model_name]);
    ASSERT_TRUE(link.Valid(_ecm));
    link.EnableVelocityChecks(_ecm);
  }
}

// Apply sinusoidal wrench before integration.
void SinusoidalWrenchPlugin::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm
)
{
  for (std::string model_name : kModelNames) {
    gz::sim::Link link = gz::sim::Link(this->link_entities[model_name]);
    ASSERT_TRUE(link.Valid(_ecm));

    // Get time in seconds.
    double t = std::chrono::duration<double>(_info.simTime).count();

    gz::math::Vector3d force(
      kForceVec[0] * std::sin(kForceAngVel * t),
      kForceVec[1] * std::sin(kForceAngVel * t),
      kForceVec[2] * std::sin(kForceAngVel * t)
    );
    gz::math::Vector3d torque(
      kTorqueVec[0] * std::sin(kTorqueAngVel * t),
      kTorqueVec[1] * std::sin(kTorqueAngVel * t),
      kTorqueVec[2] * std::sin(kTorqueAngVel * t)
    );

    link.AddWorldWrench(_ecm, force, torque);
  }
}

class EmptyTestFixture: public InternalFixture<::testing::Test> {};

// Check that the link state at the end of the motion matches the expected
// state.
TEST_F(EmptyTestFixture,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(MotionTest))
{
  // Start server and run.
  gz::sim::ServerConfig serverConfig;
  serverConfig.SetSdfFile(
    common::joinPaths(PROJECT_SOURCE_PATH, kWorldFilePath)
  );
  serverConfig.SetUpdateRate(kRate);
  gz::sim::Server server = gz::sim::Server(serverConfig);
  auto system = std::make_shared<SinusoidalWrenchPlugin>();
  server.AddSystem(system);
  ASSERT_FALSE(server.Running());
  ASSERT_TRUE(server.Run(true, kIter, false));

  gz::sim::EntityComponentManager *ecm = system->ecm;

  for (std::string model_name : kModelNames) {
    gz::sim::Link link = gz::sim::Link(system->link_entities[model_name]);
    ASSERT_TRUE(link.Valid(*ecm));

    // Check pose.
    const std::optional<gz::math::Pose3d> maybe_pose = link.WorldPose(*ecm);
    EXPECT_TRUE(maybe_pose);
    if (maybe_pose) {
      const gz::math::Pose3d pose = maybe_pose.value();
      const std::optional<gz::math::Vector3d> maybe_pos = pose.Pos();
      EXPECT_TRUE(maybe_pos);
      if (maybe_pos) {
        gz::math::Vector3d pos = maybe_pos.value();
        EXPECT_LE(
          (pos - kExpectedStates.at(model_name).pos).Length(),
          kTols.pos
        );
      }

      const gz::math::Quaternion quat = pose.Rot();
      gz::math::Vector3d axis;
      double angle = 0;
      quat.AxisAngle(axis, angle);
      EXPECT_LE(
        (axis - kExpectedStates.at(model_name).axis).Length(),
        kTols.axis
      ) << kExpectedStates.at(model_name).axis;
      EXPECT_LE(
        std::abs(angle - kExpectedStates.at(model_name).angle),
        kTols.angle
      ) << kExpectedStates.at(model_name).angle;
    }

    // Check velocities.
    std::optional<gz::math::Vector3d> maybe_lin_vel =
      link.WorldLinearVelocity(*ecm);
    EXPECT_TRUE(maybe_lin_vel);
    if (maybe_lin_vel) {
      gz::math::Vector3d lin_vel = maybe_lin_vel.value();
      EXPECT_LE(
        (lin_vel - kExpectedStates.at(model_name).lin_vel).Length(),
        kTols.lin_vel
      ) << kExpectedStates.at(model_name).lin_vel;
    }

    std::optional<gz::math::Vector3d> maybe_ang_vel =
      link.WorldAngularVelocity(*ecm);
    EXPECT_TRUE(maybe_ang_vel);
    if (maybe_ang_vel) {
      gz::math::Vector3d ang_vel = maybe_ang_vel.value();
      EXPECT_LE(
        (ang_vel - kExpectedStates.at(model_name).ang_vel).Length(),
        kTols.ang_vel
      ) << kExpectedStates.at(model_name).ang_vel;
    }
  }
}
