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
#include <sdf/Capsule.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Element.hh>
#include <sdf/Ellipsoid.hh>
#include <sdf/Gui.hh>
#include <sdf/Heightmap.hh>
#include <sdf/Light.hh>
#include <sdf/Magnetometer.hh>
#include <sdf/Mesh.hh>
#include <sdf/Pbr.hh>
#include <sdf/Physics.hh>
#include <sdf/Plane.hh>
#include <sdf/Polyline.hh>
#include <sdf/Projector.hh>
#include <sdf/Root.hh>
#include <sdf/Scene.hh>
#include <sdf/Sphere.hh>
#include <sdf/World.hh>

#include <gz/msgs/Utility.hh>

#include "gz/sim/Conversions.hh"

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

/////////////////////////////////////////////////
TEST(Conversions, Light)
{
  sdf::Light light;
  light.SetName("test_convert_light");
  light.SetType(sdf::LightType::DIRECTIONAL);
  light.SetRawPose({3, 2, 1, 0, GZ_PI, 0});
  light.SetPoseRelativeTo("world");
  light.SetCastShadows(true);
  light.SetVisualize(true);
  light.SetDiffuse(math::Color(0.4f, 0.5f, 0.6f, 1.0));
  light.SetSpecular(math::Color(0.8f, 0.9f, 0.1f, 1.0));
  light.SetAttenuationRange(3.2);
  light.SetConstantAttenuationFactor(0.5);
  light.SetLinearAttenuationFactor(0.1);
  light.SetQuadraticAttenuationFactor(0.01);
  light.SetDirection({0.1, 0.2, 1});
  light.SetSpotInnerAngle(1.9);
  light.SetSpotOuterAngle(3.3);
  light.SetSpotFalloff(0.9);
  light.SetIntensity(1.7);
  light.SetLightOn(true);

  msgs::Light lightMsg;
  lightMsg = convert<msgs::Light>(light);
  EXPECT_EQ("test_convert_light", lightMsg.name());
  EXPECT_EQ(msgs::Light_LightType_DIRECTIONAL, lightMsg.type());
  EXPECT_EQ(math::Pose3d(3, 2, 1, 0, GZ_PI, 0),
      msgs::Convert(lightMsg.pose()));
  /// \todo(anyone) add pose frame fields in gz-msgs?
  // EXPECT_EQ("world", lightMsg.pose_frame());
  EXPECT_TRUE(lightMsg.cast_shadows());
  EXPECT_FALSE(lightMsg.is_light_off());
  EXPECT_TRUE(lightMsg.visualize_visual());
  EXPECT_EQ(math::Color(0.4f, 0.5f, 0.6f, 1),
      msgs::Convert(lightMsg.diffuse()));
  EXPECT_EQ(math::Color(0.8f, 0.9f, 0.1f, 1),
      msgs::Convert(lightMsg.specular()));
  EXPECT_FLOAT_EQ(3.2f, lightMsg.range());
  EXPECT_FLOAT_EQ(0.5f, lightMsg.attenuation_constant());
  EXPECT_FLOAT_EQ(0.1f, lightMsg.attenuation_linear());
  EXPECT_FLOAT_EQ(0.01f, lightMsg.attenuation_quadratic());
  EXPECT_EQ(math::Vector3d(0.1, 0.2, 1), msgs::Convert(lightMsg.direction()));
  EXPECT_EQ(math::Angle(1.9), lightMsg.spot_inner_angle());
  EXPECT_EQ(math::Angle(3.3), lightMsg.spot_outer_angle());
  EXPECT_FLOAT_EQ(0.9f, lightMsg.spot_falloff());
  EXPECT_FLOAT_EQ(1.7f, lightMsg.intensity());

  auto newLight = convert<sdf::Light>(lightMsg);
  EXPECT_EQ("test_convert_light", newLight.Name());
  EXPECT_EQ(sdf::LightType::DIRECTIONAL, newLight.Type());
  EXPECT_EQ(math::Pose3d(3, 2, 1, 0, GZ_PI, 0), newLight.RawPose());
  /// \todo(anyone) add pose frame fields in gz-msgs?
  // EXPECT_EQ("world", newLight.PoseRelativeTo());
  EXPECT_TRUE(newLight.CastShadows());
  EXPECT_TRUE(newLight.Visualize());
  EXPECT_EQ(math::Color(0.4f, 0.5f, 0.6f, 1.0f), newLight.Diffuse());
  EXPECT_EQ(math::Color(0.8f, 0.9f, 0.1f, 1.0f), newLight.Specular());
  EXPECT_FLOAT_EQ(3.2f, newLight.AttenuationRange());
  EXPECT_FLOAT_EQ(0.5f, newLight.ConstantAttenuationFactor());
  EXPECT_FLOAT_EQ(0.1f, newLight.LinearAttenuationFactor());
  EXPECT_FLOAT_EQ(0.01f, newLight.QuadraticAttenuationFactor());
  EXPECT_EQ(math::Vector3d(0.1, 0.2, 1), newLight.Direction());
  EXPECT_EQ(math::Angle(1.9), newLight.SpotInnerAngle());
  EXPECT_EQ(math::Angle(3.3), newLight.SpotOuterAngle());
  EXPECT_FLOAT_EQ(0.9f, newLight.SpotFalloff());
  EXPECT_FLOAT_EQ(1.7f, newLight.Intensity());

  EXPECT_EQ("", convert(sdf::LightType::INVALID));
  EXPECT_EQ("point", convert(sdf::LightType::POINT));
  EXPECT_EQ("directional", convert(sdf::LightType::DIRECTIONAL));
  EXPECT_EQ("spot", convert(sdf::LightType::SPOT));

  EXPECT_EQ(sdf::LightType::POINT, convert("POINT"));
  EXPECT_EQ(sdf::LightType::DIRECTIONAL, convert("DireCtiOnal"));
  EXPECT_EQ(sdf::LightType::SPOT, convert("spot"));
  EXPECT_EQ(sdf::LightType::INVALID, convert("gazebo"));
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
TEST(Conversions, Physics)
{
  // Test conversion from msg to sdf
  msgs::Physics msg;
  msg.set_real_time_factor(1.23);
  msg.set_max_step_size(0.12);

  auto physics = convert<sdf::Physics>(msg);
  EXPECT_DOUBLE_EQ(1.23, physics.RealTimeFactor());
  EXPECT_DOUBLE_EQ(0.12, physics.MaxStepSize());

  // Test conversion from sdf to msg
  sdf::Physics physSdf;
  physSdf.SetMaxStepSize(0.34);
  physSdf.SetRealTimeFactor(2.34);

  auto physMsg = convert<msgs::Physics>(physSdf);
  EXPECT_DOUBLE_EQ(2.34, physMsg.real_time_factor());
  EXPECT_DOUBLE_EQ(0.34, physMsg.max_step_size());
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

  // Test empty orientation.
  msgs::Pose msg2;
  msg2.mutable_position()->set_x(1);
  msg2.mutable_position()->set_y(2);
  msg2.mutable_position()->set_z(3);

  pose = convert<math::Pose3d>(msg2);
  EXPECT_EQ(math::Pose3d(1, 2, 3, 1.0, 0, 0, 0), pose);
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
  material.SetDiffuse(math::Color(0.1f, 0.2f, 0.3f, 0.4f));
  material.SetSpecular(math::Color(0.5f, 0.6f, 0.7f, 0.8f));
  material.SetAmbient(math::Color(0.9f, 1.0f, 1.1f, 1.2f));
  material.SetShininess(0.5);
  material.SetEmissive(math::Color(1.3f, 1.4f, 1.5f, 1.6f));
  material.SetLighting(true);
  material.SetRenderOrder(2.5);
  material.SetDoubleSided(true);

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
  workflow.SetLightMap("light_map.png", 1u);
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
  EXPECT_DOUBLE_EQ(0.5, materialMsg.shininess());
  EXPECT_EQ(math::Color(1.3f, 1.4f, 1.5f, 1.6f),
      msgs::Convert(materialMsg.emissive()));
  EXPECT_TRUE(materialMsg.lighting());
  EXPECT_DOUBLE_EQ(2.5, materialMsg.render_order());
  EXPECT_TRUE(materialMsg.double_sided());

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
  EXPECT_EQ("light_map.png", pbrMsg.light_map());
  EXPECT_EQ(1u, pbrMsg.light_map_texcoord_set());

  EXPECT_DOUBLE_EQ(0.3, pbrMsg.metalness());
  EXPECT_DOUBLE_EQ(0.9, pbrMsg.roughness());
  EXPECT_DOUBLE_EQ(0.1, pbrMsg.glossiness());

  auto newMaterial = convert<sdf::Material>(materialMsg);
  EXPECT_EQ(math::Color(0.1f, 0.2f, 0.3f, 0.4f), newMaterial.Diffuse());
  EXPECT_EQ(math::Color(0.5f, 0.6f, 0.7f, 0.8f), newMaterial.Specular());
  EXPECT_EQ(math::Color(0.9f, 1.0f, 1.1f, 1.2f), newMaterial.Ambient());
  EXPECT_DOUBLE_EQ(0.5, newMaterial.Shininess());
  EXPECT_EQ(math::Color(1.3f, 1.4f, 1.5f, 1.6f), newMaterial.Emissive());
  EXPECT_TRUE(newMaterial.Lighting());
  EXPECT_TRUE(newMaterial.DoubleSided());
  EXPECT_DOUBLE_EQ(2.5, newMaterial.RenderOrder());

  auto newPbrMaterial = newMaterial.PbrMaterial();
  ASSERT_NE(nullptr, newPbrMaterial);
  auto newWorkflow = newPbrMaterial->Workflow(sdf::PbrWorkflowType::METAL);
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
  EXPECT_EQ("light_map.png", newWorkflow->LightMap());
  EXPECT_EQ(1u, newWorkflow->LightMapTexCoordSet());
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
  boxShape.SetSize(math::Vector3d(1, 2, 3));
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
TEST(Conversions, GeometryCapsule)
{
  sdf::Geometry geometry;
  geometry.SetType(sdf::GeometryType::CAPSULE);

  sdf::Capsule capsuleShape;
  capsuleShape.SetRadius(1.23);
  capsuleShape.SetLength(4.56);
  geometry.SetCapsuleShape(capsuleShape);

  auto geometryMsg = convert<msgs::Geometry>(geometry);
  EXPECT_EQ(msgs::Geometry::CAPSULE, geometryMsg.type());
  EXPECT_TRUE(geometryMsg.has_capsule());
  EXPECT_DOUBLE_EQ(1.23, geometryMsg.capsule().radius());
  EXPECT_DOUBLE_EQ(4.56, geometryMsg.capsule().length());

  auto newGeometry = convert<sdf::Geometry>(geometryMsg);
  EXPECT_EQ(sdf::GeometryType::CAPSULE, newGeometry.Type());
  ASSERT_NE(nullptr, newGeometry.CapsuleShape());
  EXPECT_DOUBLE_EQ(1.23, newGeometry.CapsuleShape()->Radius());
  EXPECT_DOUBLE_EQ(4.56, newGeometry.CapsuleShape()->Length());
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
TEST(Conversions, GeometryEllipsoid)
{
  sdf::Geometry geometry;
  geometry.SetType(sdf::GeometryType::ELLIPSOID);

  sdf::Ellipsoid ellipsoidShape;
  ellipsoidShape.SetRadii(gz::math::Vector3d(1.2, 3.2, 2.4));
  geometry.SetEllipsoidShape(ellipsoidShape);

  auto geometryMsg = convert<msgs::Geometry>(geometry);
  EXPECT_EQ(msgs::Geometry::ELLIPSOID, geometryMsg.type());
  EXPECT_TRUE(geometryMsg.has_ellipsoid());
  EXPECT_EQ(gz::math::Vector3d(1.2, 3.2, 2.4),
    msgs::Convert(geometryMsg.ellipsoid().radii()));

  auto newGeometry = convert<sdf::Geometry>(geometryMsg);
  EXPECT_EQ(sdf::GeometryType::ELLIPSOID, newGeometry.Type());
  ASSERT_NE(nullptr, newGeometry.EllipsoidShape());
  EXPECT_EQ(gz::math::Vector3d(1.2, 3.2, 2.4),
    newGeometry.EllipsoidShape()->Radii());
}

/////////////////////////////////////////////////
TEST(Conversions, GeometryMesh)
{
  sdf::Geometry geometry;
  geometry.SetType(sdf::GeometryType::MESH);

  sdf::Mesh meshShape;
  meshShape.SetScale(math::Vector3d(1, 2, 3));
  meshShape.SetUri("file://watermelon");
  meshShape.SetSubmesh("grape");
  meshShape.SetCenterSubmesh(true);
  meshShape.SetOptimization("convex_decomposition");
  sdf::ConvexDecomposition convexDecomp;
  convexDecomp.SetMaxConvexHulls(4);
  convexDecomp.SetVoxelResolution(10000);
  meshShape.SetConvexDecomposition(convexDecomp);
  geometry.SetMeshShape(meshShape);

  auto geometryMsg = convert<msgs::Geometry>(geometry);
  EXPECT_EQ(msgs::Geometry::MESH, geometryMsg.type());
  EXPECT_TRUE(geometryMsg.has_mesh());
  EXPECT_EQ(math::Vector3d(1, 2, 3),
      msgs::Convert(geometryMsg.mesh().scale()));
  EXPECT_EQ("file://watermelon", geometryMsg.mesh().filename());
  EXPECT_EQ("grape", geometryMsg.mesh().submesh());
  EXPECT_TRUE(geometryMsg.mesh().center_submesh());
  auto header = geometryMsg.header().data(0);
  EXPECT_EQ("optimization", header.key());
  EXPECT_EQ("convex_decomposition", header.value(0));
  header = geometryMsg.header().data(1);
  EXPECT_EQ("max_convex_hulls", header.key());
  EXPECT_EQ("4", header.value(0));
  header = geometryMsg.header().data(2);
  EXPECT_EQ("voxel_resolution", header.key());
  EXPECT_EQ("10000", header.value(0));

  auto newGeometry = convert<sdf::Geometry>(geometryMsg);
  EXPECT_EQ(sdf::GeometryType::MESH, newGeometry.Type());
  ASSERT_NE(nullptr, newGeometry.MeshShape());
  EXPECT_EQ(math::Vector3d(1, 2, 3), newGeometry.MeshShape()->Scale());
  EXPECT_EQ("file://watermelon", newGeometry.MeshShape()->Uri());
  EXPECT_EQ("grape", newGeometry.MeshShape()->Submesh());
  EXPECT_TRUE(newGeometry.MeshShape()->CenterSubmesh());
  EXPECT_EQ("convex_decomposition", newGeometry.MeshShape()->OptimizationStr());
  auto newConvexDecomp = newGeometry.MeshShape()->ConvexDecomposition();
  ASSERT_NE(nullptr, newConvexDecomp);
  EXPECT_EQ(4, newConvexDecomp->MaxConvexHulls());
  EXPECT_EQ(10000, newConvexDecomp->VoxelResolution());
}

/////////////////////////////////////////////////
TEST(Conversions, GeometryPlane)
{
  sdf::Geometry geometry;
  geometry.SetType(sdf::GeometryType::PLANE);

  sdf::Plane planeShape;
  planeShape.SetSize(math::Vector2d(1, 2));
  planeShape.SetNormal(math::Vector3d::UnitY);
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
TEST(Conversions, GeometryHeightmap)
{
  sdf::Geometry geometry;
  geometry.SetType(sdf::GeometryType::HEIGHTMAP);

  sdf::Heightmap heightmap;
  heightmap.SetUri("file://heights.png");
  heightmap.SetSize(gz::math::Vector3d(1, 2, 3));
  heightmap.SetPosition(gz::math::Vector3d(4, 5, 6));
  heightmap.SetUseTerrainPaging(true);
  heightmap.SetSampling(16u);

  sdf::HeightmapTexture texture;
  texture.SetDiffuse("file://diffuse.png");
  texture.SetNormal("file://normal.png");
  texture.SetSize(1.23);
  heightmap.AddTexture(texture);

  sdf::HeightmapBlend blend;
  blend.SetMinHeight(123.456);
  blend.SetFadeDistance(456.789);
  heightmap.AddBlend(blend);

  geometry.SetHeightmapShape(heightmap);

  auto geometryMsg = convert<msgs::Geometry>(geometry);

  EXPECT_EQ(msgs::Geometry::HEIGHTMAP, geometryMsg.type());
  EXPECT_TRUE(geometryMsg.has_heightmap());
  EXPECT_EQ(math::Vector3d(1, 2, 3),
      msgs::Convert(geometryMsg.heightmap().size()));
  EXPECT_EQ("file://heights.png", geometryMsg.heightmap().filename());
  EXPECT_TRUE(geometryMsg.heightmap().use_terrain_paging());
  EXPECT_EQ(16u, geometryMsg.heightmap().sampling());

  ASSERT_EQ(1, geometryMsg.heightmap().texture().size());
  EXPECT_DOUBLE_EQ(1.23, geometryMsg.heightmap().texture(0).size());
  EXPECT_EQ("file://diffuse.png", geometryMsg.heightmap().texture(0).diffuse());
  EXPECT_EQ("file://normal.png", geometryMsg.heightmap().texture(0).normal());

  ASSERT_EQ(1, geometryMsg.heightmap().blend().size());
  EXPECT_DOUBLE_EQ(123.456, geometryMsg.heightmap().blend(0).min_height());
  EXPECT_DOUBLE_EQ(456.789, geometryMsg.heightmap().blend(0).fade_dist());

  auto newGeometry = convert<sdf::Geometry>(geometryMsg);

  EXPECT_EQ(sdf::GeometryType::HEIGHTMAP, newGeometry.Type());

  auto newHeightmap = newGeometry.HeightmapShape();
  ASSERT_NE(nullptr, newHeightmap);
  EXPECT_EQ(math::Vector3d(1, 2, 3), newHeightmap->Size());
  EXPECT_EQ("file://heights.png", newHeightmap->Uri());
  EXPECT_TRUE(newHeightmap->UseTerrainPaging());
  EXPECT_EQ(16u, newHeightmap->Sampling());

  ASSERT_EQ(1u, newHeightmap->TextureCount());
  EXPECT_DOUBLE_EQ(1.23, newHeightmap->TextureByIndex(0)->Size());
  EXPECT_EQ("file://diffuse.png", newHeightmap->TextureByIndex(0)->Diffuse());
  EXPECT_EQ("file://normal.png", newHeightmap->TextureByIndex(0)->Normal());

  ASSERT_EQ(1u, newHeightmap->BlendCount());
  EXPECT_DOUBLE_EQ(123.456, newHeightmap->BlendByIndex(0)->MinHeight());
  EXPECT_DOUBLE_EQ(456.789, newHeightmap->BlendByIndex(0)->FadeDistance());
}

/////////////////////////////////////////////////
TEST(Conversions, GeometryPolyline)
{
  sdf::Geometry geometry;
  geometry.SetType(sdf::GeometryType::POLYLINE);

  sdf::Polyline polylineShape;
  polylineShape.SetHeight(1.23);
  polylineShape.AddPoint({4.5, 6.7});
  polylineShape.AddPoint({8.9, 10.11});
  geometry.SetPolylineShape({polylineShape});

  auto geometryMsg = convert<msgs::Geometry>(geometry);
  EXPECT_EQ(msgs::Geometry::POLYLINE, geometryMsg.type());
  ASSERT_EQ(1, geometryMsg.polyline_size());
  EXPECT_DOUBLE_EQ(1.23, geometryMsg.polyline(0).height());
  ASSERT_EQ(2, geometryMsg.polyline(0).point_size());
  EXPECT_DOUBLE_EQ(4.5, geometryMsg.polyline(0).point(0).x());
  EXPECT_DOUBLE_EQ(6.7, geometryMsg.polyline(0).point(0).y());
  EXPECT_DOUBLE_EQ(8.9, geometryMsg.polyline(0).point(1).x());
  EXPECT_DOUBLE_EQ(10.11, geometryMsg.polyline(0).point(1).y());

  auto newGeometry = convert<sdf::Geometry>(geometryMsg);
  EXPECT_EQ(sdf::GeometryType::POLYLINE, newGeometry.Type());
  ASSERT_FALSE(newGeometry.PolylineShape().empty());
  ASSERT_EQ(1u, newGeometry.PolylineShape().size());
  EXPECT_DOUBLE_EQ(1.23, newGeometry.PolylineShape()[0].Height());
  ASSERT_EQ(2u, newGeometry.PolylineShape()[0].PointCount());
  EXPECT_DOUBLE_EQ(4.5, newGeometry.PolylineShape()[0].PointByIndex(0)->X());
  EXPECT_DOUBLE_EQ(6.7, newGeometry.PolylineShape()[0].PointByIndex(0)->Y());
  EXPECT_DOUBLE_EQ(8.9, newGeometry.PolylineShape()[0].PointByIndex(1)->X());
  EXPECT_DOUBLE_EQ(10.11, newGeometry.PolylineShape()[0].PointByIndex(1)->Y());
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
  EXPECT_TRUE(jointAxis.SetXyz(math::Vector3d(1, 2, 3)).empty());
  jointAxis.SetXyzExpressedIn("__model__");
  jointAxis.SetDamping(0.1);
  jointAxis.SetFriction(0.2);
  jointAxis.SetLower(0.3);
  jointAxis.SetUpper(0.4);
  jointAxis.SetEffort(0.5);
  jointAxis.SetMaxVelocity(0.6);

  auto axisMsg = convert<msgs::Axis>(jointAxis);
  EXPECT_EQ(math::Vector3d(1, 2, 3).Normalized(), msgs::Convert(axisMsg.xyz()));
  EXPECT_DOUBLE_EQ(0.1, axisMsg.damping());
  EXPECT_DOUBLE_EQ(0.2, axisMsg.friction());
  EXPECT_DOUBLE_EQ(0.3, axisMsg.limit_lower());
  EXPECT_DOUBLE_EQ(0.4, axisMsg.limit_upper());
  EXPECT_DOUBLE_EQ(0.5, axisMsg.limit_effort());
  EXPECT_DOUBLE_EQ(0.6, axisMsg.limit_velocity());
  EXPECT_EQ(axisMsg.xyz_expressed_in(), "__model__");

  auto newJointAxis = convert<sdf::JointAxis>(axisMsg);
  EXPECT_EQ(math::Vector3d(1, 2, 3).Normalized(), newJointAxis.Xyz());
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
  scene.SetAmbient(math::Color(0.1f, 0.2f, 0.3f, 0.4f));
  scene.SetBackground(math::Color(0.5f, 0.6f, 0.7f, 0.8f));
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
  EXPECT_FALSE(sceneMsg.has_sky());

  auto newScene = convert<sdf::Scene>(sceneMsg);
  EXPECT_EQ(math::Color(0.1f, 0.2f, 0.3f, 0.4f), newScene.Ambient());
  EXPECT_EQ(math::Color(0.5f, 0.6f, 0.7f, 0.8f), newScene.Background());
  EXPECT_TRUE(newScene.Shadows());
  EXPECT_TRUE(newScene.Grid());
  EXPECT_TRUE(newScene.OriginVisual());
  EXPECT_EQ(nullptr, newScene.Sky());

  // sky
  sdf::Sky sky;
  sky.SetTime(10);
  sky.SetSunrise(4.0);
  sky.SetSunset(15.0);
  sky.SetCloudSpeed(5.0);
  sky.SetCloudDirection(math::Angle(3.14));
  sky.SetCloudHumidity(0.11);
  sky.SetCloudMeanSize(0.88);
  sky.SetCloudAmbient(math::Color::Red);
  sky.SetCubemapUri("test.dds");
  scene.SetSky(sky);

  auto sceneSkyMsg = convert<msgs::Scene>(scene);
  EXPECT_TRUE(sceneSkyMsg.has_sky());
  EXPECT_DOUBLE_EQ(10.0, sceneSkyMsg.sky().time());
  EXPECT_DOUBLE_EQ(4.0, sceneSkyMsg.sky().sunrise());
  EXPECT_DOUBLE_EQ(15.0, sceneSkyMsg.sky().sunset());
  EXPECT_DOUBLE_EQ(5.0, sceneSkyMsg.sky().wind_speed());
  EXPECT_DOUBLE_EQ(3.14, sceneSkyMsg.sky().wind_direction());
  EXPECT_DOUBLE_EQ(0.11, sceneSkyMsg.sky().humidity());
  EXPECT_DOUBLE_EQ(0.88, sceneSkyMsg.sky().mean_cloud_size());
  EXPECT_EQ(math::Color::Red,
      msgs::Convert(sceneSkyMsg.sky().cloud_ambient()));
  EXPECT_EQ("test.dds", sceneSkyMsg.sky().cubemap_uri());

  auto newSceneSky = convert<sdf::Scene>(sceneSkyMsg);
  ASSERT_NE(nullptr, newSceneSky.Sky());
  EXPECT_DOUBLE_EQ(10.0, newSceneSky.Sky()->Time());
  EXPECT_DOUBLE_EQ(4.0, newSceneSky.Sky()->Sunrise());
  EXPECT_DOUBLE_EQ(15.0, newSceneSky.Sky()->Sunset());
  EXPECT_DOUBLE_EQ(5.0, newSceneSky.Sky()->CloudSpeed());
  EXPECT_EQ(math::Angle(3.14), newSceneSky.Sky()->CloudDirection());
  EXPECT_DOUBLE_EQ(0.11, newSceneSky.Sky()->CloudHumidity());
  EXPECT_DOUBLE_EQ(0.88, newSceneSky.Sky()->CloudMeanSize());
  EXPECT_EQ(math::Color::Red, newSceneSky.Sky()->CloudAmbient());
  EXPECT_EQ("test.dds", newSceneSky.Sky()->CubemapUri());
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
TEST(Conversions, MagnetometerSensor)
{
  sdf::Sensor sensor;
  sensor.SetName("my_sensor");
  sensor.SetType(sdf::SensorType::MAGNETOMETER);
  sensor.SetUpdateRate(12.4);
  sensor.SetTopic("my_topic");
  sensor.SetRawPose(math::Pose3d(1, 2, 3, 0, 0, 0));

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
TEST(Conversions, AltimeterSensor)
{
  sdf::Sensor sensor;
  sensor.SetName("my_sensor");
  sensor.SetType(sdf::SensorType::ALTIMETER);
  sensor.SetUpdateRate(12.4);
  sensor.SetTopic("my_topic");
  sensor.SetRawPose(math::Pose3d(1, 2, 3, 0, 0, 0));

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
  EXPECT_FLOAT_EQ(2.0f, actorMsg.skin_scale());
  EXPECT_TRUE(actorMsg.script_loop());
  EXPECT_FLOAT_EQ(2.8f, actorMsg.script_delay_start());
  EXPECT_TRUE(actorMsg.script_auto_start());

  ASSERT_EQ(1, actorMsg.animations_size());
  EXPECT_EQ("animation", actorMsg.animations(0).name());
  EXPECT_EQ("animation_filename", actorMsg.animations(0).filename());
  EXPECT_FLOAT_EQ(1.23f, actorMsg.animations(0).scale());
  EXPECT_TRUE(actorMsg.animations(0).interpolate_x());

  ASSERT_EQ(1, actorMsg.trajectories_size());
  EXPECT_EQ(456u, actorMsg.trajectories(0).id());
  EXPECT_EQ("traj_type", actorMsg.trajectories(0).type());
  EXPECT_FLOAT_EQ(7.89f, actorMsg.trajectories(0).tension());

  ASSERT_EQ(1, actorMsg.trajectories(0).waypoints_size());
  EXPECT_FLOAT_EQ(0.123f, actorMsg.trajectories(0).waypoints(0).time());
  EXPECT_EQ(math::Pose3d(6, 5, 4, 0, 0, 0),
      msgs::Convert(actorMsg.trajectories(0).waypoints(0).pose()));

  auto newActor = convert<sdf::Actor>(actorMsg);
  EXPECT_EQ("test_convert_actor", newActor.Name());
  EXPECT_EQ(math::Pose3d(3, 2, 1, 0, 0, 0), newActor.RawPose());
  EXPECT_EQ("walk.dae", newActor.SkinFilename());
  EXPECT_FLOAT_EQ(2.0f, newActor.SkinScale());
  EXPECT_TRUE(newActor.ScriptLoop());
  EXPECT_FLOAT_EQ(2.8f, newActor.ScriptDelayStart());
  EXPECT_TRUE(newActor.ScriptAutoStart());

  ASSERT_EQ(1u, newActor.AnimationCount());
  EXPECT_EQ("animation", newActor.AnimationByIndex(0)->Name());
  EXPECT_EQ("animation_filename", newActor.AnimationByIndex(0)->Filename());
  EXPECT_FLOAT_EQ(1.23f, newActor.AnimationByIndex(0)->Scale());
  EXPECT_TRUE(newActor.AnimationByIndex(0)->InterpolateX());

  ASSERT_EQ(1u, newActor.TrajectoryCount());
  EXPECT_EQ(456u, newActor.TrajectoryByIndex(0)->Id());
  EXPECT_EQ("traj_type", newActor.TrajectoryByIndex(0)->Type());
  EXPECT_FLOAT_EQ(7.89f, newActor.TrajectoryByIndex(0)->Tension());

  ASSERT_EQ(1u, newActor.TrajectoryByIndex(0)->WaypointCount());
  EXPECT_FLOAT_EQ(0.123f,
          newActor.TrajectoryByIndex(0)->WaypointByIndex(0)->Time());
  EXPECT_EQ(math::Pose3d(6, 5, 4, 0, 0, 0),
      newActor.TrajectoryByIndex(0)->WaypointByIndex(0)->Pose());
}

/////////////////////////////////////////////////
TEST(Conversions, ParticleEmitter)
{
  sdf::ParticleEmitter emitter;
  emitter.SetName("my_emitter");
  emitter.SetType(sdf::ParticleEmitterType::BOX);
  emitter.SetEmitting(false);
  emitter.SetDuration(12);
  emitter.SetLifetime(56);
  emitter.SetRate(0.5);
  emitter.SetScaleRate(1.2);
  emitter.SetMinVelocity(0.1);
  emitter.SetMaxVelocity(0.2);
  emitter.SetSize(math::Vector3d(1, 2, 3));
  emitter.SetParticleSize(math::Vector3d(4, 5, 6));
  emitter.SetColorStart(math::Color(0.1f, 0.2f, 0.3f));
  emitter.SetColorEnd(math::Color(0.4f, 0.5f, 0.6f));
  emitter.SetColorRangeImage("range_image");
  emitter.SetTopic("my_topic");
  emitter.SetRawPose(math::Pose3d(1, 2, 3, 0, 0, 0));
  emitter.SetScatterRatio(0.9f);

  sdf::Material material;
  sdf::Pbr pbr;
  sdf::PbrWorkflow workflow;
  workflow.SetType(sdf::PbrWorkflowType::METAL);
  workflow.SetAlbedoMap("albedo_map.png");
  pbr.SetWorkflow(workflow.Type(), workflow);
  material.SetPbrMaterial(pbr);

  emitter.SetMaterial(material);

  // Convert SDF to a message.
  msgs::ParticleEmitter emitterMsg = convert<msgs::ParticleEmitter>(emitter);

  EXPECT_EQ("my_emitter", emitterMsg.name());
  EXPECT_EQ(msgs::ParticleEmitter::BOX, emitterMsg.type());
  EXPECT_FALSE(emitterMsg.emitting().data());
  EXPECT_NEAR(12, emitterMsg.duration().data(), 1e-3);
  EXPECT_NEAR(56, emitterMsg.lifetime().data(), 1e-3);
  EXPECT_NEAR(0.5, emitterMsg.rate().data(), 1e-3);
  EXPECT_NEAR(1.2, emitterMsg.scale_rate().data(), 1e-3);
  EXPECT_NEAR(0.1, emitterMsg.min_velocity().data(), 1e-3);
  EXPECT_NEAR(0.2, emitterMsg.max_velocity().data(), 1e-3);
  EXPECT_EQ(math::Vector3d(1, 2, 3), msgs::Convert(emitterMsg.size()));
  EXPECT_EQ(math::Vector3d(4, 5, 6), msgs::Convert(emitterMsg.particle_size()));
  EXPECT_EQ(math::Color(0.1f, 0.2f, 0.3f),
      msgs::Convert(emitterMsg.color_start()));
  EXPECT_EQ(math::Color(0.4f, 0.5f, 0.6f),
      msgs::Convert(emitterMsg.color_end()));
  EXPECT_EQ("range_image", emitterMsg.color_range_image().data());
  EXPECT_EQ("my_topic", emitterMsg.topic().data());

  EXPECT_FLOAT_EQ(0.9f, emitterMsg.particle_scatter_ratio().data());

  EXPECT_EQ(math::Pose3d(1, 2, 3, 0, 0, 0), msgs::Convert(emitterMsg.pose()));

  auto pbrMsg = emitterMsg.material().pbr();
  EXPECT_EQ(msgs::Material::PBR::METAL, pbrMsg.type());
  EXPECT_EQ("albedo_map.png", pbrMsg.albedo_map());

  // Convert the message back to SDF.
  sdf::ParticleEmitter emitter2 = convert<sdf::ParticleEmitter>(emitterMsg);
  EXPECT_EQ(emitter2.Name(), emitter.Name());
  EXPECT_EQ(emitter2.Type(), emitter.Type());
  EXPECT_EQ(emitter2.Emitting(), emitter.Emitting());
  EXPECT_NEAR(emitter2.Duration(), emitter.Duration(), 1e-3);
  EXPECT_NEAR(emitter2.Lifetime(), emitter.Lifetime(), 1e-3);
  EXPECT_NEAR(emitter2.Rate(), emitter.Rate(), 1e-3);
  EXPECT_NEAR(emitter2.ScaleRate(), emitter.ScaleRate(), 1e-3);
  EXPECT_NEAR(emitter2.MinVelocity(), emitter.MinVelocity(), 1e-3);
  EXPECT_NEAR(emitter2.MaxVelocity(), emitter.MaxVelocity(), 1e-3);
  EXPECT_EQ(emitter2.Size(), emitter.Size());
  EXPECT_EQ(emitter2.ParticleSize(), emitter.ParticleSize());
  EXPECT_EQ(emitter2.ColorStart(), emitter.ColorStart());
  EXPECT_EQ(emitter2.ColorEnd(), emitter.ColorEnd());
  EXPECT_EQ(emitter2.ColorRangeImage(), emitter.ColorRangeImage());
  EXPECT_EQ(emitter2.Topic(), emitter.Topic());
  EXPECT_EQ(emitter2.RawPose(), emitter.RawPose());
  EXPECT_FLOAT_EQ(emitter2.ScatterRatio(), emitter.ScatterRatio());
}

/////////////////////////////////////////////////
TEST(Conversions, Projector)
{
  sdf::Projector projector;
  projector.SetName("my_projector");
  projector.SetNearClip(0.03);
  projector.SetFarClip(30);
  projector.SetHorizontalFov(math::Angle(0.4));
  projector.SetTexture("projector.png");
  projector.SetVisibilityFlags(0xFF);

  // Convert SDF to a message.
  msgs::Projector projectorMsg = convert<msgs::Projector>(projector);

  EXPECT_EQ("my_projector", projectorMsg.name());
  EXPECT_NEAR(0.03, projectorMsg.near_clip(), 1e-3);
  EXPECT_NEAR(30, projectorMsg.far_clip(), 1e-3);
  EXPECT_NEAR(0.4, projectorMsg.fov(), 1e-3);
  EXPECT_EQ("projector.png", projectorMsg.texture());
  EXPECT_EQ(0xFF, projectorMsg.visibility_flags());

  // Convert the message back to SDF.
  sdf::Projector projector2 = convert<sdf::Projector>(projectorMsg);
  EXPECT_EQ(projector2.Name(), projector.Name());
  EXPECT_NEAR(projector2.NearClip(), projector.NearClip(), 1e-3);
  EXPECT_NEAR(projector2.FarClip(), projector.FarClip(), 1e-3);
  EXPECT_EQ(projector2.HorizontalFov(), projector.HorizontalFov());
  EXPECT_EQ(projector2.Texture(), projector.Texture());
  EXPECT_EQ(projector2.VisibilityFlags(), projector.VisibilityFlags());
}

/////////////////////////////////////////////////
TEST(Conversions, PluginElement)
{
  sdf::Root root;
  root.LoadSdfString("<?xml version='1.0'?><sdf version='1.6'>"
      "<world name='default'>"
      "  <plugin filename='plum' name='peach'>"
      "    <avocado>0.5</avocado>"
      "  </plugin>"
      "</world></sdf>");

  auto world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);

  auto worldElem = world->Element();
  ASSERT_NE(nullptr, worldElem);

  auto pluginElem = worldElem->GetElement("plugin");
  ASSERT_NE(nullptr, pluginElem);

  auto pluginMsg = convert<msgs::Plugin>(*(pluginElem.get()));
  EXPECT_EQ("plum", pluginMsg.filename());
  EXPECT_EQ("peach", pluginMsg.name());

  EXPECT_NE(pluginMsg.innerxml().find("<avocado>0.5</avocado>"),
      std::string::npos);
}

/////////////////////////////////////////////////
TEST(Conversions, Plugin)
{
  sdf::Plugin pluginSdf;
  pluginSdf.SetName("peach");
  pluginSdf.SetFilename("plum");

  auto content = std::make_shared<sdf::Element>();
  content->SetName("avocado");
  content->AddValue("double", "0.5", false, "");
  pluginSdf.InsertContent(content);

  auto pluginMsg = convert<msgs::Plugin>(pluginSdf);
  EXPECT_EQ("plum", pluginMsg.filename());
  EXPECT_EQ("peach", pluginMsg.name());

  EXPECT_NE(pluginMsg.innerxml().find("<avocado>0.5</avocado>"),
      std::string::npos) << pluginMsg.innerxml();
}

/////////////////////////////////////////////////
TEST(Conversions, MsgsPluginToSdf)
{
  std::string innerXml ="<test>another_test</test>\n";
  std::string innerXml2 ="<peanut>butter</peanut>\n";

  msgs::Plugin msgPlugin;
  msgPlugin.set_name("foo");
  msgPlugin.set_filename("bar");
  msgPlugin.set_innerxml(innerXml);

  // Test conversion of a single msgs::Plugin to sdf::Plugin
  sdf::Plugin sdfPlugin = convert<sdf::Plugin>(msgPlugin);
  EXPECT_EQ("foo", sdfPlugin.Name());
  EXPECT_EQ("bar", sdfPlugin.Filename());
  ASSERT_EQ(1u, sdfPlugin.Contents().size());
  EXPECT_EQ(innerXml, sdfPlugin.Contents()[0]->ToString(""));

  // Test conversion of a msgs::Plugin_V with 1 plugin to sdf::Plugins
  msgs::Plugin_V msgsPlugin;
  msgsPlugin.add_plugins()->CopyFrom(msgPlugin);
  sdf::Plugins sdfPlugins = convert<sdf::Plugins>(msgsPlugin);
  ASSERT_EQ(1u, sdfPlugins.size());
  EXPECT_EQ("foo", sdfPlugins[0].Name());
  EXPECT_EQ("bar", sdfPlugins[0].Filename());
  ASSERT_EQ(1u, sdfPlugins[0].Contents().size());
  EXPECT_EQ(innerXml, sdfPlugins[0].Contents()[0]->ToString(""));

  // Add another plugin the msgs::Plugin_V
  msgs::Plugin anotherPlugin;
  anotherPlugin.set_name("sandwich");
  anotherPlugin.set_filename("time");
  anotherPlugin.set_innerxml(innerXml + innerXml2);
  msgsPlugin.add_plugins()->CopyFrom(anotherPlugin);
  sdfPlugins = convert<sdf::Plugins>(msgsPlugin);
  ASSERT_EQ(2u, sdfPlugins.size());
  EXPECT_EQ("foo", sdfPlugins[0].Name());
  EXPECT_EQ("bar", sdfPlugins[0].Filename());
  ASSERT_EQ(1u, sdfPlugins[0].Contents().size());
  EXPECT_EQ(innerXml, sdfPlugins[0].Contents()[0]->ToString(""));
  EXPECT_EQ("sandwich", sdfPlugins[1].Name());
  EXPECT_EQ("time", sdfPlugins[1].Filename());
  ASSERT_EQ(2u, sdfPlugins[1].Contents().size());
  EXPECT_EQ(innerXml, sdfPlugins[1].Contents()[0]->ToString(""));
  EXPECT_EQ(innerXml2, sdfPlugins[1].Contents()[1]->ToString(""));
}

/////////////////////////////////////////////////
TEST(Conversions, GeometryEmpty)
{
  sdf::Geometry geometry;
  geometry.SetType(sdf::GeometryType::EMPTY);

  auto geometryMsg = convert<msgs::Geometry>(geometry);
  EXPECT_EQ(msgs::Geometry::EMPTY, geometryMsg.type());
}
