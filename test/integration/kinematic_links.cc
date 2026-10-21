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

#include <gtest/gtest.h>

#include <gz/msgs/contacts.pb.h>

#include <map>
#include <string>

#include <gz/common/Console.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Server.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/ContactSensorData.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "test_config.hh"

#include "../helpers/EnvTestFixture.hh"
#include "../helpers/Relay.hh"

using namespace gz;
using namespace sim;

/// \brief Tests for the <kinematic> parameters of the Physics system, which
/// need a physics engine with kinematic links (bullet-featherstone).
class KinematicLinksTest : public InternalFixture<::testing::Test>
{
};

namespace
{
/// \brief SDF of a model with one 1 m box link.
/// \param[in] _name Model name.
/// \param[in] _x X position of the model.
/// \param[in] _z Height of the model.
/// \param[in] _kinematic Content of the <kinematic> element of the link, or
/// empty to leave the element out.
/// \param[in] _sensor True to add a contact sensor for the box collision.
/// \param[in] _collideWithoutContact True to set collide_without_contact on
/// the box collision.
std::string BoxSdf(const std::string &_name, double _x, double _z,
    const std::string &_kinematic, bool _sensor = false,
    bool _collideWithoutContact = false)
{
  std::string sdf = "<model name='" + _name + "'>"
      "<pose>" + std::to_string(_x) + " 0 " + std::to_string(_z) +
      " 0 0 0</pose>"
      "<link name='link'>";
  if (!_kinematic.empty())
    sdf += "<kinematic>" + _kinematic + "</kinematic>";
  sdf += "<collision name='collision'>"
      "<geometry><box><size>1 1 1</size></box></geometry>";
  if (_collideWithoutContact)
  {
    sdf += "<surface><contact>"
        "<collide_without_contact>true</collide_without_contact>"
        "</contact></surface>";
  }
  sdf += "</collision>";
  if (_sensor)
  {
    sdf += "<sensor name='contact_sensor' type='contact'>"
        "<contact><collision>collision</collision></contact>"
        "<update_rate>1000</update_rate>"
        "</sensor>";
  }
  sdf += "</link></model>";
  return sdf;
}

/// \brief World with a static ground plane and, 3 m apart from each other,
/// - unset_high, kinematic_high, dynamic_high: boxes 2 m above the ground
///   without <kinematic>, with <kinematic>true</kinematic> and with
///   <kinematic>false</kinematic>;
/// - unset_sunk: a box without <kinematic>, sunk 0.2 m into the ground,
///   with a contact sensor, and a sphere with <kinematic>false</kinematic>
///   dropped on top of it;
/// - cwc_sunk: the same box with collide_without_contact and without the
///   sphere.
/// \param[in] _kinematicEnabled Value of <kinematic><enabled>.
/// \param[in] _contactsEnabled Value of <kinematic><contacts><enabled>.
std::string WorldSdf(bool _kinematicEnabled, bool _contactsEnabled)
{
  auto boolStr = [](bool _b) { return std::string(_b ? "true" : "false"); };
  return "<?xml version='1.0'?><sdf version='1.9'>"
      "<world name='kinematic_links'>"
      "<physics name='1ms' type='ignored'>"
      "<max_step_size>0.001</max_step_size>"
      "<real_time_factor>0</real_time_factor>"
      "</physics>"
      "<plugin filename='gz-sim-physics-system' "
      "name='gz::sim::systems::Physics'>"
      "<engine><filename>gz-physics-bullet-featherstone-plugin</filename>"
      "</engine>"
      "<kinematic>"
      "<enabled>" + boolStr(_kinematicEnabled) + "</enabled>"
      "<contacts><enabled>" + boolStr(_contactsEnabled) + "</enabled>"
      "</contacts>"
      "</kinematic>"
      "</plugin>"
      "<plugin filename='gz-sim-contact-system' "
      "name='gz::sim::systems::Contact'/>"
      "<model name='ground_plane'><static>true</static>"
      "<link name='link'><collision name='collision'><geometry>"
      "<plane><normal>0 0 1</normal><size>100 100</size></plane>"
      "</geometry></collision></link></model>"
      + BoxSdf("unset_high", 0.0, 2.0, "")
      + BoxSdf("kinematic_high", 3.0, 2.0, "true")
      + BoxSdf("dynamic_high", 6.0, 2.0, "false")
      + BoxSdf("unset_sunk", 9.0, 0.3, "", true)
      + BoxSdf("cwc_sunk", 12.0, 0.3, "", true, true) +
      "<model name='sphere'><pose>9 0 1.3 0 0 0</pose><link name='link'>"
      "<kinematic>false</kinematic>"
      "<collision name='collision'><geometry><sphere><radius>0.25</radius>"
      "</sphere></geometry></collision></link></model>"
      "</world></sdf>";
}

/// \brief Height of the models of the world, by name, and the number of
/// contact points of the contact sensors of the sunk boxes with the ground
/// plane and with the sphere, recorded from the ECM after each update.
struct State
{
  std::map<std::string, double> heights;
  std::map<std::string, int> groundContacts;
  std::map<std::string, int> sphereContacts;

  void Record(const EntityComponentManager &_ecm)
  {
    _ecm.Each<components::Model, components::Name>(
        [&](const Entity &_entity, const components::Model *,
            const components::Name *_name) -> bool
        {
          this->heights[_name->Data()] = worldPose(_entity, _ecm).Pos().Z();
          return true;
        });

    for (const std::string &model : {"unset_sunk", "cwc_sunk"})
    {
      this->groundContacts[model] = 0;
      this->sphereContacts[model] = 0;
      const Entity modelEntity = _ecm.EntityByComponents(
          components::Model(), components::Name(model));
      const Entity collision = _ecm.EntityByComponents(
          components::Collision(), components::Name("collision"),
          components::ParentEntity(_ecm.EntityByComponents(
              components::Name("link"),
              components::ParentEntity(modelEntity))));
      const auto *data =
          _ecm.Component<components::ContactSensorData>(collision);
      if (nullptr == data)
        continue;
      for (const auto &contact : data->Data().contact())
      {
        const std::string names =
            contact.collision1().name() + " " + contact.collision2().name();
        if (names.find("ground_plane") != std::string::npos)
          this->groundContacts[model] += contact.position_size();
        if (names.find("sphere") != std::string::npos)
          this->sphereContacts[model] += contact.position_size();
      }
    }
  }
};

/// \brief Run a world for a number of iterations and return its final state.
State RunWorld(const std::string &_sdf, std::size_t _iterations)
{
  ServerConfig serverConfig;
  serverConfig.SetSdfString(_sdf);
  Server server(serverConfig);

  State state;
  test::Relay recorder;
  recorder.OnPostUpdate(
      [&state](const UpdateInfo &, const EntityComponentManager &_ecm)
      {
        state.Record(_ecm);
      });
  server.AddSystem(recorder.systemPtr);
  server.Run(true, _iterations, false);
  return state;
}
}  // namespace

/////////////////////////////////////////////////
// <kinematic><enabled> is the default for links that do not set <kinematic>
// in their SDF; the element of a link overrides it in both directions.
TEST_F(KinematicLinksTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(KinematicEnabled))
{
  // Kinematic by default: only the link with <kinematic>false</kinematic>
  // falls. The sphere lands on the kinematic box it was dropped on.
  State state = RunWorld(WorldSdf(true, true), 500);
  EXPECT_NEAR(2.0, state.heights["unset_high"], 1e-3);
  EXPECT_NEAR(2.0, state.heights["kinematic_high"], 1e-3);
  EXPECT_GT(1.5, state.heights["dynamic_high"]);
  EXPECT_NEAR(0.3, state.heights["unset_sunk"], 1e-3);
  EXPECT_NEAR(1.05, state.heights["sphere"], 2e-2);

  // Dynamic by default: only the link with <kinematic>true</kinematic> stays.
  state = RunWorld(WorldSdf(false, true), 500);
  EXPECT_GT(1.5, state.heights["unset_high"]);
  EXPECT_NEAR(2.0, state.heights["kinematic_high"], 1e-3);
  EXPECT_GT(1.5, state.heights["dynamic_high"]);
  // The box sunk into the ground is dynamic and pushed out of it
  EXPECT_LT(0.35, state.heights["unset_sunk"]);
}

/////////////////////////////////////////////////
// <kinematic><contacts><enabled> controls whether the contacts of kinematic
// links with static (and kinematic) links are detected; contacts with
// dynamic links always are, and collide_without_contact keeps the contacts
// of a collision detected.
TEST_F(KinematicLinksTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(KinematicContacts))
{
  // Contacts enabled (default): the sunk kinematic box reports contacts with
  // the ground and with the sphere resting on it.
  State state = RunWorld(WorldSdf(true, true), 500);
  EXPECT_NEAR(1.05, state.heights["sphere"], 2e-2);
  EXPECT_LT(0, state.groundContacts["unset_sunk"]);
  EXPECT_LT(0, state.sphereContacts["unset_sunk"]);
  EXPECT_LT(0, state.groundContacts["cwc_sunk"]);

  // Contacts disabled: no contacts with the ground, except for the box with
  // collide_without_contact; the sphere still rests on the box and its
  // contacts are reported.
  state = RunWorld(WorldSdf(true, false), 500);
  EXPECT_NEAR(1.05, state.heights["sphere"], 2e-2);
  EXPECT_EQ(0, state.groundContacts["unset_sunk"]);
  EXPECT_LT(0, state.sphereContacts["unset_sunk"]);
  EXPECT_LT(0, state.groundContacts["cwc_sunk"]);
}
