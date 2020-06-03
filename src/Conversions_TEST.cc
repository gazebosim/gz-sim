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

#include <sdf/Actor.hh>
#include <sdf/Altimeter.hh>
#include <sdf/Atmosphere.hh>
#include <sdf/Box.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Gui.hh>
#include <sdf/Light.hh>
#include <sdf/Magnetometer.hh>
#include <sdf/Mesh.hh>
#include <sdf/Pbr.hh>
#include <sdf/Plane.hh>
#include <sdf/Root.hh>
#include <sdf/Scene.hh>
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
  light.SetRawPose({3, 2, 1, 0, IGN_PI, 0});
  light.SetPoseRelativeTo("world");
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

  auto newLight = convert<sdf::Light>(lightMsg);
  EXPECT_EQ("test_convert_light", newLight.Name());
  EXPECT_EQ(sdf::LightType::DIRECTIONAL, newLight.Type());
  EXPECT_EQ(math::Pose3d(3, 2, 1, 0, IGN_PI, 0), newLight.RawPose());
  /// \todo(anyone) add pose frame fields in ign-msgs?
  // EXPECT_EQ("world", newLight.PoseRelativeTo());
  EXPECT_TRUE(newLight.CastShadows());
  EXPECT_EQ(math::Color(0.4f, 0.5f, 0.6f, 1.0f), newLight.Diffuse());
  EXPECT_EQ(math::Color(0.8f, 0.9f, 0.1f, 1.0f), newLight.Specular());
  EXPECT_FLOAT_EQ(3.2, newLight.AttenuationRange());
  EXPECT_FLOAT_EQ(0.5, newLight.ConstantAttenuationFactor());
  EXPECT_FLOAT_EQ(0.1, newLight.LinearAttenuationFactor());
  EXPECT_FLOAT_EQ(0.01, newLight.QuadraticAttenuationFactor());
  EXPECT_EQ(math::Vector3d(0.1, 0.2, 1), newLight.Direction());
  EXPECT_EQ(math::Angle(1.9), newLight.SpotInnerAngle());
  EXPECT_EQ(math::Angle(3.3), newLight.SpotOuterAngle());
  EXPECT_FLOAT_EQ(0.9, newLight.SpotFalloff());
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
TEST(Conversions, Entity)
{
  std::string model = "model";
  auto entityType = convert<msgs::Entity_Type>(model);
  EXPECT_EQ(msgs::Entity_Type_MODEL, entityType);

  std::string empty = "";
  auto entityType2 = convert<msgs::Entity_Type>(empty);
  EXPECT_EQ(msgs::Entity_Type_NONE, entityType2);
}

/////////////////////////////////////////////////
TEST(Conversions, Pose)
{
  msgs::Pose msg;
  msg.mutable_position()->set_x(1);
  msg.mutable_position()->set_y(2);
  msg.mutable_position()->set_z(3);
  msg.mutable_orientation()->set_w(0.1);
  msg.mutable_orientation()->set_x(0.2);
  msg.mutable_orientation()->set_y(0.3);
  msg.mutable_orientation()->set_z(0.4);

  auto pose = convert<math::Pose3d>(msg);
  EXPECT_EQ(math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3, 0.4), pose);
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
  material.SetLighting(true);

  sdf::Pbr pbr;
  sdf::PbrWorkflow workflow;
  workflow.SetType(sdf::PbrWorkflowType::METAL);
  workflow.SetAlbedoMap("albedo_map.png");
  workflow.SetNormalMap("normal_map.png");
  workflow.SetEnvironmentMap("environment_map.png");
  workflow.SetAmbientOcclusionMap("ambient_occlusion_map.png");
  workflow.SetMetalnessMap("metalness_map.png");
  workflow.SetRoughnessMap("roughness_map.png");
  workflow.SetEmissiveMap("emissive_map.png");
  workflow.SetGlossinessMap("dummy_glossiness_map.png");
  workflow.SetSpecularMap("dummy_specular_map.png");
  workflow.SetMetalness(0.3);
  workflow.SetRoughness(0.9);
  workflow.SetGlossiness(0.1);
  pbr.SetWorkflow(workflow.Type(), workflow);
  material.SetPbrMaterial(pbr);

  auto materialMsg = convert<msgs::Material>(material);
  EXPECT_EQ(math::Color(0.1f, 0.2f, 0.3f, 0.4f),
      msgs::Convert(materialMsg.diffuse()));
  EXPECT_EQ(math::Color(0.5f, 0.6f, 0.7f, 0.8f),
      msgs::Convert(materialMsg.specular()));
  EXPECT_EQ(math::Color(0.9f, 1.0f, 1.1f, 1.2f),
      msgs::Convert(materialMsg.ambient()));
  EXPECT_EQ(math::Color(1.3f, 1.4f, 1.5f, 1.6f),
      msgs::Convert(materialMsg.emissive()));
  EXPECT_TRUE(materialMsg.lighting());

  EXPECT_TRUE(materialMsg.has_pbr());
  const auto &pbrMsg = materialMsg.pbr();
  EXPECT_EQ(msgs::Material_PBR_WorkflowType_METAL, pbrMsg.type());
  EXPECT_EQ("albedo_map.png", pbrMsg.albedo_map());
  EXPECT_EQ("normal_map.png", pbrMsg.normal_map());
  EXPECT_EQ("roughness_map.png", pbrMsg.roughness_map());
  EXPECT_EQ("metalness_map.png", pbrMsg.metalness_map());
  EXPECT_EQ("environment_map.png", pbrMsg.environment_map());
  EXPECT_EQ("emissive_map.png", pbrMsg.emissive_map());
  EXPECT_EQ("ambient_occlusion_map.png", pbrMsg.ambient_occlusion_map());
  EXPECT_EQ("dummy_glossiness_map.png", pbrMsg.glossiness_map());
  EXPECT_EQ("dummy_specular_map.png", pbrMsg.specular_map());
  EXPECT_DOUBLE_EQ(0.3, pbrMsg.metalness());
  EXPECT_DOUBLE_EQ(0.9, pbrMsg.roughness());
  EXPECT_DOUBLE_EQ(0.1, pbrMsg.glossiness());

  auto newMaterial = convert<sdf::Material>(materialMsg);
  EXPECT_EQ(math::Color(0.1f, 0.2f, 0.3f, 0.4f), newMaterial.Diffuse());
  EXPECT_EQ(math::Color(0.5f, 0.6f, 0.7f, 0.8f), newMaterial.Specular());
  EXPECT_EQ(math::Color(0.9f, 1.0f, 1.1f, 1.2f), newMaterial.Ambient());
  EXPECT_EQ(math::Color(1.3f, 1.4f, 1.5f, 1.6f), newMaterial.Emissive());
  EXPECT_TRUE(newMaterial.Lighting());

  sdf::Pbr *newPbrMaterial = newMaterial.PbrMaterial();
  ASSERT_NE(nullptr, newPbrMaterial);
  sdf::PbrWorkflow *newWorkflow =
      newPbrMaterial->Workflow(sdf::PbrWorkflowType::METAL);
  ASSERT_NE(nullptr, newWorkflow);
  EXPECT_EQ("albedo_map.png", newWorkflow->AlbedoMap());
  EXPECT_EQ("normal_map.png", newWorkflow->NormalMap());
  EXPECT_EQ("roughness_map.png", newWorkflow->RoughnessMap());
  EXPECT_EQ("metalness_map.png", newWorkflow->MetalnessMap());
  EXPECT_EQ("environment_map.png", newWorkflow->EnvironmentMap());
  EXPECT_EQ("emissive_map.png", newWorkflow->EmissiveMap());
  EXPECT_EQ("ambient_occlusion_map.png", newWorkflow->AmbientOcclusionMap());
  EXPECT_EQ("dummy_glossiness_map.png", newWorkflow->GlossinessMap());
  EXPECT_EQ("dummy_specular_map.png", newWorkflow->SpecularMap());
  EXPECT_DOUBLE_EQ(0.3, newWorkflow->Metalness());
  EXPECT_DOUBLE_EQ(0.9, newWorkflow->Roughness());
  EXPECT_DOUBLE_EQ(0.1, newWorkflow->Glossiness());
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

/////////////////////////////////////////////////
TEST(Conversions, Inertial)
{
  math::MassMatrix3d massMatrix;
  massMatrix.SetMass(1.1);
  massMatrix.SetIxx(2.2);
  massMatrix.SetIyy(3.3);
  massMatrix.SetIzz(4.4);
  massMatrix.SetIxy(5.5);
  massMatrix.SetIxz(6.6);
  massMatrix.SetIyz(7.7);

  math::Inertiald inertial;
  inertial.SetPose(math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));
  inertial.SetMassMatrix(massMatrix);

  auto inertialMsg = convert<msgs::Inertial>(inertial);
  EXPECT_EQ(math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3),
      msgs::Convert(inertialMsg.pose()));
  EXPECT_DOUBLE_EQ(1.1, inertialMsg.mass());
  EXPECT_DOUBLE_EQ(2.2, inertialMsg.ixx());
  EXPECT_DOUBLE_EQ(3.3, inertialMsg.iyy());
  EXPECT_DOUBLE_EQ(4.4, inertialMsg.izz());
  EXPECT_DOUBLE_EQ(5.5, inertialMsg.ixy());
  EXPECT_DOUBLE_EQ(6.6, inertialMsg.ixz());
  EXPECT_DOUBLE_EQ(7.7, inertialMsg.iyz());

  auto newInertial = convert<math::Inertiald>(inertialMsg);
  EXPECT_EQ(math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3), newInertial.Pose());
  EXPECT_DOUBLE_EQ(1.1, newInertial.MassMatrix().Mass());
  EXPECT_DOUBLE_EQ(2.2, newInertial.MassMatrix().Ixx());
  EXPECT_DOUBLE_EQ(3.3, newInertial.MassMatrix().Iyy());
  EXPECT_DOUBLE_EQ(4.4, newInertial.MassMatrix().Izz());
  EXPECT_DOUBLE_EQ(5.5, newInertial.MassMatrix().Ixy());
  EXPECT_DOUBLE_EQ(6.6, newInertial.MassMatrix().Ixz());
  EXPECT_DOUBLE_EQ(7.7, newInertial.MassMatrix().Iyz());
}

/////////////////////////////////////////////////
TEST(Conversions, JointAxis)
{
  sdf::JointAxis jointAxis;
  jointAxis.SetXyz(math::Vector3d(1, 2, 3));
  jointAxis.SetXyzExpressedIn("__model__");
  jointAxis.SetDamping(0.1);
  jointAxis.SetFriction(0.2);
  jointAxis.SetLower(0.3);
  jointAxis.SetUpper(0.4);
  jointAxis.SetEffort(0.5);
  jointAxis.SetMaxVelocity(0.6);

  auto axisMsg = convert<msgs::Axis>(jointAxis);
  EXPECT_EQ(math::Vector3d(1, 2, 3), msgs::Convert(axisMsg.xyz()));
  EXPECT_DOUBLE_EQ(0.1, axisMsg.damping());
  EXPECT_DOUBLE_EQ(0.2, axisMsg.friction());
  EXPECT_DOUBLE_EQ(0.3, axisMsg.limit_lower());
  EXPECT_DOUBLE_EQ(0.4, axisMsg.limit_upper());
  EXPECT_DOUBLE_EQ(0.5, axisMsg.limit_effort());
  EXPECT_DOUBLE_EQ(0.6, axisMsg.limit_velocity());
  EXPECT_EQ(axisMsg.xyz_expressed_in(), "__model__");

  auto newJointAxis = convert<sdf::JointAxis>(axisMsg);
  EXPECT_EQ(math::Vector3d(1, 2, 3), newJointAxis.Xyz());
  EXPECT_DOUBLE_EQ(0.1, newJointAxis.Damping());
  EXPECT_DOUBLE_EQ(0.2, newJointAxis.Friction());
  EXPECT_DOUBLE_EQ(0.3, newJointAxis.Lower());
  EXPECT_DOUBLE_EQ(0.4, newJointAxis.Upper());
  EXPECT_DOUBLE_EQ(0.5, newJointAxis.Effort());
  EXPECT_DOUBLE_EQ(0.6, newJointAxis.MaxVelocity());
  EXPECT_EQ(newJointAxis.XyzExpressedIn(), "__model__");
}

/////////////////////////////////////////////////
TEST(Conversions, Scene)
{
  sdf::Scene scene;
  scene.SetAmbient(ignition::math::Color(0.1f, 0.2f, 0.3f, 0.4f));
  scene.SetBackground(ignition::math::Color(0.5f, 0.6f, 0.7f, 0.8f));
  scene.SetShadows(true);
  scene.SetGrid(true);
  scene.SetOriginVisual(true);

  auto sceneMsg = convert<msgs::Scene>(scene);
  EXPECT_EQ(math::Color(0.1f, 0.2f, 0.3f, 0.4f),
      msgs::Convert(sceneMsg.ambient()));
  EXPECT_EQ(math::Color(0.5f, 0.6f, 0.7f, 0.8f),
      msgs::Convert(sceneMsg.background()));
  EXPECT_TRUE(sceneMsg.shadows());
  EXPECT_TRUE(sceneMsg.grid());
  EXPECT_TRUE(sceneMsg.origin_visual());

  auto newScene = convert<sdf::Scene>(sceneMsg);
  EXPECT_EQ(math::Color(0.1f, 0.2f, 0.3f, 0.4f), newScene.Ambient());
  EXPECT_EQ(math::Color(0.5f, 0.6f, 0.7f, 0.8f), newScene.Background());
  EXPECT_TRUE(newScene.Shadows());
  EXPECT_TRUE(newScene.Grid());
  EXPECT_TRUE(newScene.OriginVisual());
}

/////////////////////////////////////////////////
TEST(Conversions, Atmosphere)
{
  sdf::Atmosphere atmosphere;
  atmosphere.SetType(sdf::AtmosphereType::ADIABATIC);
  atmosphere.SetTemperature(math::Temperature(234.5));
  atmosphere.SetPressure(88.6);

  auto atmosphereMsg = convert<msgs::Atmosphere>(atmosphere);
  EXPECT_EQ(msgs::Atmosphere::ADIABATIC, atmosphereMsg.type());
  EXPECT_DOUBLE_EQ(234.5, atmosphereMsg.temperature());
  EXPECT_DOUBLE_EQ(88.6, atmosphereMsg.pressure());

  auto newAtmosphere = convert<sdf::Atmosphere>(atmosphereMsg);
  EXPECT_EQ(sdf::AtmosphereType::ADIABATIC, newAtmosphere.Type());
  EXPECT_EQ(math::Temperature(234.5), newAtmosphere.Temperature());
  EXPECT_DOUBLE_EQ(88.6, newAtmosphere.Pressure());
}

/////////////////////////////////////////////////
TEST(CONVERSIONS, MagnetometerSensor)
{
  sdf::Sensor sensor;
  sensor.SetName("my_sensor");
  sensor.SetType(sdf::SensorType::MAGNETOMETER);
  sensor.SetUpdateRate(12.4);
  sensor.SetTopic("my_topic");
  sensor.SetRawPose(ignition::math::Pose3d(1, 2, 3, 0, 0, 0));

  sdf::Noise noise;
  noise.SetType(sdf::NoiseType::GAUSSIAN);
  noise.SetMean(1.2);
  noise.SetStdDev(2.6);
  noise.SetBiasMean(0.2);
  noise.SetBiasStdDev(12.16);
  noise.SetPrecision(0.01);

  sdf::Magnetometer mag;
  mag.SetXNoise(noise);
  sensor.SetMagnetometerSensor(mag);

  msgs::Sensor msg = convert<msgs::Sensor>(sensor);
  EXPECT_EQ(sensor.Name(), msg.name());
  EXPECT_EQ(sensor.TypeStr(), msg.type());
  EXPECT_DOUBLE_EQ(sensor.UpdateRate(), msg.update_rate());
  EXPECT_EQ(sensor.Topic(), msg.topic());
  EXPECT_EQ(sensor.RawPose(), msgs::Convert(msg.pose()));

  ASSERT_TRUE(msg.has_magnetometer());

  sdf::Noise defaultNoise;
  sdf::Noise convertedNoise;
  convertedNoise = convert<sdf::Noise>(msg.magnetometer().x_noise());
  EXPECT_EQ(noise, convertedNoise);

  EXPECT_FALSE(msg.magnetometer().has_y_noise());
  EXPECT_FALSE(msg.magnetometer().has_z_noise());
}

/////////////////////////////////////////////////
TEST(CONVERSIONS, AltimeterSensor)
{
  sdf::Sensor sensor;
  sensor.SetName("my_sensor");
  sensor.SetType(sdf::SensorType::ALTIMETER);
  sensor.SetUpdateRate(12.4);
  sensor.SetTopic("my_topic");
  sensor.SetRawPose(ignition::math::Pose3d(1, 2, 3, 0, 0, 0));

  sdf::Noise noise;
  noise.SetType(sdf::NoiseType::GAUSSIAN);
  noise.SetMean(1.2);
  noise.SetStdDev(2.6);
  noise.SetBiasMean(0.2);
  noise.SetBiasStdDev(12.16);
  noise.SetPrecision(0.01);

  sdf::Altimeter alt;
  alt.SetVerticalPositionNoise(noise);
  sensor.SetAltimeterSensor(alt);

  msgs::Sensor msg = convert<msgs::Sensor>(sensor);
  EXPECT_EQ(sensor.Name(), msg.name());
  EXPECT_EQ(sensor.TypeStr(), msg.type());
  EXPECT_DOUBLE_EQ(sensor.UpdateRate(), msg.update_rate());
  EXPECT_EQ(sensor.Topic(), msg.topic());
  EXPECT_EQ(sensor.RawPose(), msgs::Convert(msg.pose()));

  ASSERT_TRUE(msg.has_altimeter());

  sdf::Noise defaultNoise;
  sdf::Noise convertedNoise;
  convertedNoise = convert<sdf::Noise>(
      msg.altimeter().vertical_position_noise());
  EXPECT_EQ(noise, convertedNoise);

  EXPECT_FALSE(msg.altimeter().has_vertical_velocity_noise());
}

/////////////////////////////////////////////////
TEST(Conversions, UpdateInfo)
{
  UpdateInfo info;
  info.simTime = 1234ms;
  info.realTime = 2345ms;
  info.dt = 3456ms;
  info.iterations = 1234;
  info.paused = true;

  auto statsMsg = convert<msgs::WorldStatistics>(info);
  EXPECT_EQ(1, statsMsg.sim_time().sec());
  EXPECT_EQ(234000000, statsMsg.sim_time().nsec());
  EXPECT_EQ(2, statsMsg.real_time().sec());
  EXPECT_EQ(345000000, statsMsg.real_time().nsec());
  EXPECT_EQ(3, statsMsg.step_size().sec());
  EXPECT_EQ(456000000, statsMsg.step_size().nsec());
  EXPECT_EQ(1234u, statsMsg.iterations());
  EXPECT_TRUE(statsMsg.paused());

  msgs::WorldStatistics statsMsg2;
  set(&statsMsg2, info);
  EXPECT_EQ(1, statsMsg2.sim_time().sec());
  EXPECT_EQ(234000000, statsMsg2.sim_time().nsec());
  EXPECT_EQ(2, statsMsg2.real_time().sec());
  EXPECT_EQ(345000000, statsMsg2.real_time().nsec());
  EXPECT_EQ(3, statsMsg2.step_size().sec());
  EXPECT_EQ(456000000, statsMsg2.step_size().nsec());
  EXPECT_EQ(1234u, statsMsg2.iterations());
  EXPECT_TRUE(statsMsg2.paused());

  auto newInfo = convert<UpdateInfo>(statsMsg);
  EXPECT_EQ(1234000000, newInfo.simTime.count());
  EXPECT_TRUE(newInfo.paused);
}

/////////////////////////////////////////////////
TEST(Conversions, AxisAlignedbox)
{
  math::AxisAlignedBox aabb;
  aabb.Min() = math::Vector3d(-1, -2, -3);
  aabb.Max() = math::Vector3d(1, 2, 3);

  auto aabbMsg = convert<msgs::AxisAlignedBox>(aabb);
  auto min = msgs::Convert(aabbMsg.min_corner());
  auto max = msgs::Convert(aabbMsg.max_corner());
  EXPECT_EQ(math::Vector3d(-1, -2, -3), min);
  EXPECT_EQ(math::Vector3d(1, 2, 3), max);

  msgs::AxisAlignedBox aabbMsg2;
  msgs::Set(aabbMsg2.mutable_min_corner(), math::Vector3d(2, 3, 4));
  msgs::Set(aabbMsg2.mutable_max_corner(), math::Vector3d(20, 30, 40));

  auto aabb2 = convert<math::AxisAlignedBox>(aabbMsg2);
  EXPECT_EQ(math::Vector3d(2, 3, 4), aabb2.Min());
  EXPECT_EQ(math::Vector3d(20, 30, 40), aabb2.Max());
}

/////////////////////////////////////////////////
TEST(Conversions, Actor)
{
  sdf::Animation anim;
  anim.SetName("animation");
  anim.SetFilename("animation_filename");
  anim.SetScale(1.23);
  anim.SetInterpolateX(true);

  sdf::Waypoint way;
  way.SetTime(0.123);
  way.SetPose({6, 5, 4, 0, 0, 0});

  sdf::Trajectory traj;
  traj.SetId(456);
  traj.SetType("traj_type");
  traj.SetTension(7.89);
  traj.AddWaypoint(way);

  sdf::Actor actor;
  actor.SetName("test_convert_actor");
  actor.SetRawPose({3, 2, 1, 0, 0, 0});
  actor.SetSkinFilename("walk.dae");
  actor.SetSkinScale(2.0);
  actor.SetScriptLoop(true);
  actor.SetScriptDelayStart(2.8);
  actor.SetScriptAutoStart(true);
  actor.AddAnimation(anim);
  actor.AddTrajectory(traj);

  msgs::Actor actorMsg = convert<msgs::Actor>(actor);

  EXPECT_TRUE(actorMsg.has_entity());
  EXPECT_EQ("test_convert_actor", actorMsg.entity().name());
  EXPECT_EQ(math::Pose3d(3, 2, 1, 0, 0, 0),
      msgs::Convert(actorMsg.pose()));
  EXPECT_EQ("walk.dae", actorMsg.skin_filename());
  EXPECT_FLOAT_EQ(2.0, actorMsg.skin_scale());
  EXPECT_TRUE(actorMsg.script_loop());
  EXPECT_FLOAT_EQ(2.8, actorMsg.script_delay_start());
  EXPECT_TRUE(actorMsg.script_auto_start());

  ASSERT_EQ(1, actorMsg.animations_size());
  EXPECT_EQ("animation", actorMsg.animations(0).name());
  EXPECT_EQ("animation_filename", actorMsg.animations(0).filename());
  EXPECT_FLOAT_EQ(1.23, actorMsg.animations(0).scale());
  EXPECT_TRUE(actorMsg.animations(0).interpolate_x());

  ASSERT_EQ(1, actorMsg.trajectories_size());
  EXPECT_EQ(456u, actorMsg.trajectories(0).id());
  EXPECT_EQ("traj_type", actorMsg.trajectories(0).type());
  EXPECT_FLOAT_EQ(7.89, actorMsg.trajectories(0).tension());

  ASSERT_EQ(1, actorMsg.trajectories(0).waypoints_size());
  EXPECT_FLOAT_EQ(0.123, actorMsg.trajectories(0).waypoints(0).time());
  EXPECT_EQ(math::Pose3d(6, 5, 4, 0, 0, 0),
      msgs::Convert(actorMsg.trajectories(0).waypoints(0).pose()));

  auto newActor = convert<sdf::Actor>(actorMsg);
  EXPECT_EQ("test_convert_actor", newActor.Name());
  EXPECT_EQ(math::Pose3d(3, 2, 1, 0, 0, 0), newActor.RawPose());
  EXPECT_EQ("walk.dae", newActor.SkinFilename());
  EXPECT_FLOAT_EQ(2.0, newActor.SkinScale());
  EXPECT_TRUE(newActor.ScriptLoop());
  EXPECT_FLOAT_EQ(2.8, newActor.ScriptDelayStart());
  EXPECT_TRUE(newActor.ScriptAutoStart());

  ASSERT_EQ(1u, newActor.AnimationCount());
  EXPECT_EQ("animation", newActor.AnimationByIndex(0)->Name());
  EXPECT_EQ("animation_filename", newActor.AnimationByIndex(0)->Filename());
  EXPECT_FLOAT_EQ(1.23, newActor.AnimationByIndex(0)->Scale());
  EXPECT_TRUE(newActor.AnimationByIndex(0)->InterpolateX());

  ASSERT_EQ(1u, newActor.TrajectoryCount());
  EXPECT_EQ(456u, newActor.TrajectoryByIndex(0)->Id());
  EXPECT_EQ("traj_type", newActor.TrajectoryByIndex(0)->Type());
  EXPECT_FLOAT_EQ(7.89, newActor.TrajectoryByIndex(0)->Tension());

  ASSERT_EQ(1u, newActor.TrajectoryByIndex(0)->WaypointCount());
  EXPECT_FLOAT_EQ(0.123,
          newActor.TrajectoryByIndex(0)->WaypointByIndex(0)->Time());
  EXPECT_EQ(math::Pose3d(6, 5, 4, 0, 0, 0),
      newActor.TrajectoryByIndex(0)->WaypointByIndex(0)->Pose());
}
