/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#include <chrono>

#include <sdf/Box.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Gui.hh>
#include <sdf/Light.hh>
#include <sdf/Mesh.hh>
#include <sdf/Plane.hh>
#include <sdf/Root.hh>
#include <sdf/Sphere.hh>
#include <sdf/World.hh>

#include <ignition/msgs/Utility.hh>

#include "ignition/gazebo/Conversions.hh"

using namespace ignition;
using namespace gazebo;
using namespace std::chrono_literals;

/////////////////////////////////////////////////
TEST(Conversions, Light)
{
  sdf::Light light;
  light.SetName("test_convert_light");
  light.SetType(sdf::LightType::DIRECTIONAL);
  light.SetPose({3, 2, 1, 0, IGN_PI, 0});
  light.SetPoseFrame("world");
  light.SetCastShadows(true);
  light.SetDiffuse(ignition::math::Color(0.4f, 0.5f, 0.6f, 1.0));
  light.SetSpecular(ignition::math::Color(0.8f, 0.9f, 0.1f, 1.0));
  light.SetAttenuationRange(3.2);
  light.SetConstantAttenuationFactor(0.5);
  light.SetLinearAttenuationFactor(0.1);
  light.SetQuadraticAttenuationFactor(0.01);
  light.SetDirection({0.1, 0.2, 1});
  light.SetSpotInnerAngle(1.9);
  light.SetSpotOuterAngle(3.3);
  light.SetSpotFalloff(0.9);

  msgs::Light lightMsg;
  lightMsg = convert<msgs::Light>(light);
  EXPECT_EQ("test_convert_light", lightMsg.name());
  EXPECT_EQ(msgs::Light_LightType_DIRECTIONAL, lightMsg.type());
  EXPECT_EQ(math::Pose3d(3, 2, 1, 0, IGN_PI, 0),
      msgs::Convert(lightMsg.pose()));
  /// \todo(anyone) add pose frame fields in ign-msgs?
  // EXPECT_EQ("world", lightMsg.pose_frame());
  EXPECT_TRUE(lightMsg.cast_shadows());
  EXPECT_EQ(math::Color(0.4f, 0.5f, 0.6f, 1),
      msgs::Convert(lightMsg.diffuse()));
  EXPECT_EQ(math::Color(0.8f, 0.9f, 0.1f, 1),
      msgs::Convert(lightMsg.specular()));
  EXPECT_FLOAT_EQ(3.2, lightMsg.range());
  EXPECT_FLOAT_EQ(0.5, lightMsg.attenuation_constant());
  EXPECT_FLOAT_EQ(0.1, lightMsg.attenuation_linear());
  EXPECT_FLOAT_EQ(0.01, lightMsg.attenuation_quadratic());
  EXPECT_EQ(math::Vector3d(0.1, 0.2, 1), msgs::Convert(lightMsg.direction()));
  EXPECT_EQ(math::Angle(1.9), lightMsg.spot_inner_angle());
  EXPECT_EQ(math::Angle(3.3), lightMsg.spot_outer_angle());
  EXPECT_FLOAT_EQ(0.9, lightMsg.spot_falloff());
}

/////////////////////////////////////////////////
TEST(Conversions, Gui)
{
  sdf::Root root;
  root.LoadSdfString("<?xml version='1.0'?><sdf version='1.6'>"
      "<world name='default'>"
      "  <gui fullscreen='true'>"
      "    <plugin filename='plugin-file-1' name='plugin-1'>"
      "      <banana>3</banana>"
      "    </plugin>"
      "    <plugin filename='plugin-file-2' name='plugin-2'>"
      "      <watermelon>0.5</watermelon>"
      "    </plugin>"
      "  </gui>"
      "</world></sdf>");

  auto world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);

  auto gui = world->Gui();
  ASSERT_NE(nullptr, gui);

  auto guiMsg = convert<msgs::GUI>(*gui);
  EXPECT_TRUE(guiMsg.fullscreen());
  ASSERT_EQ(2, guiMsg.plugin_size());

  const auto &plugin1 = guiMsg.plugin(0);
  EXPECT_EQ("plugin-file-1", plugin1.filename());
  EXPECT_EQ("plugin-1", plugin1.name());

  EXPECT_NE(plugin1.innerxml().find(
      "<banana>3</banana>"),
      std::string::npos);

  const auto &plugin2 = guiMsg.plugin(1);
  EXPECT_EQ("plugin-file-2", plugin2.filename());
  EXPECT_EQ("plugin-2", plugin2.name());
  EXPECT_NE(plugin2.innerxml().find(
      "<watermelon>0.5</watermelon>"),
      std::string::npos);
}

/////////////////////////////////////////////////
TEST(Conversions, Time)
{
  std::chrono::steady_clock::duration duration{2ms};

  auto msg = convert<msgs::Time>(duration);
  EXPECT_EQ(0, msg.sec());
  EXPECT_EQ(2000000, msg.nsec());

  auto duration2 = convert<std::chrono::steady_clock::duration>(msg);
  EXPECT_EQ(duration, duration2);
  EXPECT_EQ(2000000, duration2.count());
}

/////////////////////////////////////////////////
TEST(Conversions, BoxGeometry)
{
  msgs::Geometry msgsGeom;
  msgsGeom.set_type(msgs::Geometry::BOX);
  msgs::Set(msgsGeom.mutable_box()->mutable_size(),
      ignition::math::Vector3d(1, 2, 3));

  sdf::Geometry sdfGeom = convert<sdf::Geometry>(msgsGeom);
  ASSERT_NE(nullptr, sdfGeom.BoxShape());
  EXPECT_EQ(ignition::math::Vector3d(1, 2, 3),
      sdfGeom.BoxShape()->Size());

  msgs::Geometry convertedMsgGeom = convert<msgs::Geometry>(sdfGeom);
  EXPECT_EQ(msgs::Geometry::BOX, convertedMsgGeom.type());
  EXPECT_EQ(ignition::math::Vector3d(1, 2, 3),
      msgs::Convert(convertedMsgGeom.box().size()));
}

/////////////////////////////////////////////////
TEST(Conversions, SphereGeometry)
{
  msgs::Geometry msgsGeom;
  msgsGeom.set_type(msgs::Geometry::SPHERE);
  msgsGeom.mutable_sphere()->set_radius(1.2);

  sdf::Geometry sdfGeom = convert<sdf::Geometry>(msgsGeom);
  ASSERT_NE(nullptr, sdfGeom.SphereShape());
  EXPECT_DOUBLE_EQ(1.2, sdfGeom.SphereShape()->Radius());

  msgs::Geometry convertedMsgGeom = convert<msgs::Geometry>(sdfGeom);
  EXPECT_EQ(msgs::Geometry::SPHERE, convertedMsgGeom.type());
  EXPECT_DOUBLE_EQ(1.2, convertedMsgGeom.sphere().radius());
}

/////////////////////////////////////////////////
TEST(Conversions, CylinderGeometry)
{
  msgs::Geometry msgsGeom;
  msgsGeom.set_type(msgs::Geometry::CYLINDER);
  msgsGeom.mutable_cylinder()->set_radius(2.3);
  msgsGeom.mutable_cylinder()->set_length(3.4);

  sdf::Geometry sdfGeom = convert<sdf::Geometry>(msgsGeom);
  ASSERT_NE(nullptr, sdfGeom.CylinderShape());
  EXPECT_DOUBLE_EQ(2.3, sdfGeom.CylinderShape()->Radius());
  EXPECT_DOUBLE_EQ(3.4, sdfGeom.CylinderShape()->Length());

  msgs::Geometry convertedMsgGeom = convert<msgs::Geometry>(sdfGeom);
  EXPECT_EQ(msgs::Geometry::CYLINDER, convertedMsgGeom.type());
  EXPECT_DOUBLE_EQ(2.3, convertedMsgGeom.cylinder().radius());
  EXPECT_DOUBLE_EQ(3.4, convertedMsgGeom.cylinder().length());
}

/////////////////////////////////////////////////
TEST(Conversions, PlaneGeometry)
{
  msgs::Geometry msgsGeom;
  msgsGeom.set_type(msgs::Geometry::PLANE);
  msgs::Set(msgsGeom.mutable_plane()->mutable_normal(),
      ignition::math::Vector3d(0, 0, 1));
  msgs::Set(msgsGeom.mutable_plane()->mutable_size(),
      ignition::math::Vector2d(1, 2));

  sdf::Geometry sdfGeom = convert<sdf::Geometry>(msgsGeom);
  ASSERT_NE(nullptr, sdfGeom.PlaneShape());
  EXPECT_EQ(ignition::math::Vector3d(0, 0, 1),
      sdfGeom.PlaneShape()->Normal());
  EXPECT_EQ(ignition::math::Vector2d(1, 2),
      sdfGeom.PlaneShape()->Size());

  msgs::Geometry convertedMsgGeom = convert<msgs::Geometry>(sdfGeom);
  EXPECT_EQ(msgs::Geometry::PLANE, convertedMsgGeom.type());
  EXPECT_EQ(ignition::math::Vector3d(0, 0, 1),
      msgs::Convert(convertedMsgGeom.plane().normal()));
  EXPECT_EQ(ignition::math::Vector2d(1, 2),
      msgs::Convert(convertedMsgGeom.plane().size()));
}

/////////////////////////////////////////////////
TEST(Conversions, MeshGeometry)
{
  msgs::Geometry msgsGeom;
  msgsGeom.set_type(msgs::Geometry::MESH);
  msgsGeom.mutable_mesh()->set_filename("myfile");
  msgsGeom.mutable_mesh()->set_submesh("mysub");
  msgsGeom.mutable_mesh()->set_center_submesh(true);
  msgs::Set(msgsGeom.mutable_mesh()->mutable_scale(),
      ignition::math::Vector3d(2, 4, 6));

  sdf::Geometry sdfGeom = convert<sdf::Geometry>(msgsGeom);
  ASSERT_NE(nullptr, sdfGeom.MeshShape());
  EXPECT_EQ("myfile", sdfGeom.MeshShape()->Uri());
  EXPECT_EQ("mysub", sdfGeom.MeshShape()->Submesh());
  EXPECT_TRUE(sdfGeom.MeshShape()->CenterSubmesh());
  EXPECT_EQ(ignition::math::Vector3d(2, 4, 6),
      sdfGeom.MeshShape()->Scale());

  msgs::Geometry convertedMsgGeom = convert<msgs::Geometry>(sdfGeom);
  EXPECT_EQ(msgs::Geometry::MESH, convertedMsgGeom.type());
  EXPECT_EQ("myfile", convertedMsgGeom.mesh().filename());
  EXPECT_EQ("mysub", convertedMsgGeom.mesh().submesh());
  EXPECT_TRUE(convertedMsgGeom.mesh().center_submesh());
  EXPECT_EQ(ignition::math::Vector3d(2, 4, 6),
      msgs::Convert(convertedMsgGeom.mesh().scale()));
}
