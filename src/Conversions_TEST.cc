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
TEST(Conversions, Material)
{
  sdf::Material material;
  material.SetDiffuse(ignition::math::Color(0.1f, 0.2f, 0.3f, 0.4f));
  material.SetSpecular(ignition::math::Color(0.5f, 0.6f, 0.7f, 0.8f));
  material.SetAmbient(ignition::math::Color(0.9f, 1.0f, 1.1f, 1.2f));
  material.SetEmissive(ignition::math::Color(1.3f, 1.4f, 1.5f, 1.6f));

  auto materialMsg = convert<msgs::Material>(material);
  EXPECT_EQ(math::Color(0.1f, 0.2f, 0.3f, 0.4f),
      msgs::Convert(materialMsg.diffuse()));
  EXPECT_EQ(math::Color(0.5f, 0.6f, 0.7f, 0.8f),
      msgs::Convert(materialMsg.specular()));
  EXPECT_EQ(math::Color(0.9f, 1.0f, 1.1f, 1.2f),
      msgs::Convert(materialMsg.ambient()));
  EXPECT_EQ(math::Color(1.3f, 1.4f, 1.5f, 1.6f),
      msgs::Convert(materialMsg.emissive()));

  auto newMaterial = convert<sdf::Material>(materialMsg);
  EXPECT_EQ(math::Color(0.1f, 0.2f, 0.3f, 0.4f), newMaterial.Diffuse());
  EXPECT_EQ(math::Color(0.5f, 0.6f, 0.7f, 0.8f), newMaterial.Specular());
  EXPECT_EQ(math::Color(0.9f, 1.0f, 1.1f, 1.2f), newMaterial.Ambient());
  EXPECT_EQ(math::Color(1.3f, 1.4f, 1.5f, 1.6f), newMaterial.Emissive());
}

/////////////////////////////////////////////////
TEST(Conversions, GeometryBox)
{
  sdf::Geometry geometry;
  geometry.SetType(sdf::GeometryType::BOX);

  sdf::Box boxShape;
  boxShape.SetSize(ignition::math::Vector3d(1, 2, 3));
  geometry.SetBoxShape(boxShape);

  auto geometryMsg = convert<msgs::Geometry>(geometry);
  EXPECT_EQ(msgs::Geometry::BOX, geometryMsg.type());
  EXPECT_TRUE(geometryMsg.has_box());
  EXPECT_EQ(math::Vector3d(1, 2, 3),
      msgs::Convert(geometryMsg.box().size()));

  auto newGeometry = convert<sdf::Geometry>(geometryMsg);
  EXPECT_EQ(sdf::GeometryType::BOX, newGeometry.Type());
  ASSERT_NE(nullptr, newGeometry.BoxShape());
  EXPECT_EQ(math::Vector3d(1, 2, 3), newGeometry.BoxShape()->Size());
}

/////////////////////////////////////////////////
TEST(Conversions, GeometrySphere)
{
  sdf::Geometry geometry;
  geometry.SetType(sdf::GeometryType::SPHERE);

  sdf::Sphere sphereShape;
  sphereShape.SetRadius(1.23);
  geometry.SetSphereShape(sphereShape);

  auto geometryMsg = convert<msgs::Geometry>(geometry);
  EXPECT_EQ(msgs::Geometry::SPHERE, geometryMsg.type());
  EXPECT_TRUE(geometryMsg.has_sphere());
  EXPECT_DOUBLE_EQ(1.23, geometryMsg.sphere().radius());

  auto newGeometry = convert<sdf::Geometry>(geometryMsg);
  EXPECT_EQ(sdf::GeometryType::SPHERE, newGeometry.Type());
  ASSERT_NE(nullptr, newGeometry.SphereShape());
  EXPECT_DOUBLE_EQ(1.23, newGeometry.SphereShape()->Radius());
}

/////////////////////////////////////////////////
TEST(Conversions, GeometryCylinder)
{
  sdf::Geometry geometry;
  geometry.SetType(sdf::GeometryType::CYLINDER);

  sdf::Cylinder cylinderShape;
  cylinderShape.SetRadius(1.23);
  cylinderShape.SetLength(4.56);
  geometry.SetCylinderShape(cylinderShape);

  auto geometryMsg = convert<msgs::Geometry>(geometry);
  EXPECT_EQ(msgs::Geometry::CYLINDER, geometryMsg.type());
  EXPECT_TRUE(geometryMsg.has_cylinder());
  EXPECT_DOUBLE_EQ(1.23, geometryMsg.cylinder().radius());
  EXPECT_DOUBLE_EQ(4.56, geometryMsg.cylinder().length());

  auto newGeometry = convert<sdf::Geometry>(geometryMsg);
  EXPECT_EQ(sdf::GeometryType::CYLINDER, newGeometry.Type());
  ASSERT_NE(nullptr, newGeometry.CylinderShape());
  EXPECT_DOUBLE_EQ(1.23, newGeometry.CylinderShape()->Radius());
  EXPECT_DOUBLE_EQ(4.56, newGeometry.CylinderShape()->Length());
}

/////////////////////////////////////////////////
TEST(Conversions, GeometryMesh)
{
  sdf::Geometry geometry;
  geometry.SetType(sdf::GeometryType::MESH);

  sdf::Mesh meshShape;
  meshShape.SetScale(ignition::math::Vector3d(1, 2, 3));
  meshShape.SetUri("file://watermelon");
  meshShape.SetSubmesh("grape");
  meshShape.SetCenterSubmesh(true);
  geometry.SetMeshShape(meshShape);

  auto geometryMsg = convert<msgs::Geometry>(geometry);
  EXPECT_EQ(msgs::Geometry::MESH, geometryMsg.type());
  EXPECT_TRUE(geometryMsg.has_mesh());
  EXPECT_EQ(math::Vector3d(1, 2, 3),
      msgs::Convert(geometryMsg.mesh().scale()));
  EXPECT_EQ("file://watermelon", geometryMsg.mesh().filename());
  EXPECT_EQ("grape", geometryMsg.mesh().submesh());
  EXPECT_TRUE(geometryMsg.mesh().center_submesh());

  auto newGeometry = convert<sdf::Geometry>(geometryMsg);
  EXPECT_EQ(sdf::GeometryType::MESH, newGeometry.Type());
  ASSERT_NE(nullptr, newGeometry.MeshShape());
  EXPECT_EQ(math::Vector3d(1, 2, 3), newGeometry.MeshShape()->Scale());
  EXPECT_EQ("file://watermelon", newGeometry.MeshShape()->Uri());
  EXPECT_EQ("grape", newGeometry.MeshShape()->Submesh());
  EXPECT_TRUE(newGeometry.MeshShape()->CenterSubmesh());
}

/////////////////////////////////////////////////
TEST(Conversions, GeometryPlane)
{
  sdf::Geometry geometry;
  geometry.SetType(sdf::GeometryType::PLANE);

  sdf::Plane planeShape;
  planeShape.SetSize(ignition::math::Vector2d(1, 2));
  planeShape.SetNormal(ignition::math::Vector3d::UnitY);
  geometry.SetPlaneShape(planeShape);

  auto geometryMsg = convert<msgs::Geometry>(geometry);
  EXPECT_EQ(msgs::Geometry::PLANE, geometryMsg.type());
  EXPECT_TRUE(geometryMsg.has_plane());
  EXPECT_EQ(math::Vector2d(1, 2),
      msgs::Convert(geometryMsg.plane().size()));
  EXPECT_EQ(math::Vector3d::UnitY,
      msgs::Convert(geometryMsg.plane().normal()));

  auto newGeometry = convert<sdf::Geometry>(geometryMsg);
  EXPECT_EQ(sdf::GeometryType::PLANE, newGeometry.Type());
  ASSERT_NE(nullptr, newGeometry.PlaneShape());
  EXPECT_EQ(math::Vector2d(1, 2), newGeometry.PlaneShape()->Size());
  EXPECT_EQ(math::Vector3d::UnitY, newGeometry.PlaneShape()->Normal());
}
