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

#include <gz/msgs/light.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Angle.hh>
#include <gz/math/Color.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <sdf/Light.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Light.hh>
#include <gz/sim/components/Light.hh>
#include <gz/sim/components/LightCmd.hh>
#include <gz/sim/components/LightType.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>

#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

class LightIntegrationTest : public InternalFixture<::testing::Test>
{
};

//////////////////////////////////////////////////
TEST_F(LightIntegrationTest, Valid)
{
  EntityComponentManager ecm;

  // No ID
  {
    Light light;
    EXPECT_FALSE(light.Valid(ecm));
  }

  // Missing light component
  {
    auto id = ecm.CreateEntity();
    Light light(id);
    EXPECT_FALSE(light.Valid(ecm));
  }

  // Valid
  {
    auto id = ecm.CreateEntity();
    ecm.CreateComponent<components::Light>(id, components::Light());

    Light light(id);
    EXPECT_TRUE(light.Valid(ecm));
  }
}

//////////////////////////////////////////////////
TEST_F(LightIntegrationTest, Name)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Light>(id, components::Light());

  Light light(id);

  // No name
  EXPECT_EQ(std::nullopt, light.Name(ecm));

  // Add name
  ecm.CreateComponent<components::Name>(id, components::Name("light_name"));
  EXPECT_EQ("light_name", light.Name(ecm));
}

//////////////////////////////////////////////////
TEST_F(LightIntegrationTest, Pose)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Light>(id, components::Light());

  Light light(id);

  // No pose
  EXPECT_EQ(std::nullopt, light.Pose(ecm));

  // Add pose
  math::Pose3d pose(1, 2, 3, 0.1, 0.2, 0.3);
  ecm.CreateComponent<components::Pose>(id,
      components::Pose(pose));
  EXPECT_EQ(pose, light.Pose(ecm));
}

//////////////////////////////////////////////////
TEST_F(LightIntegrationTest, Type)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Light>(id, components::Light());

  Light light(id);

  // No light type
  EXPECT_EQ(std::nullopt, light.Type(ecm));

  // Add type
  std::string lightType = "point";
  ecm.CreateComponent<components::LightType>(id,
      components::LightType(lightType));
  EXPECT_EQ(lightType, light.Type(ecm));
}

//////////////////////////////////////////////////
TEST_F(LightIntegrationTest, CastShadows)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Light>(id, components::Light());

  Light light(id);

  sdf::Light lightSdf;
  lightSdf.SetCastShadows(true);
  ecm.CreateComponent<components::Light>(id,
      components::Light(lightSdf));
  EXPECT_TRUE(*light.CastShadows(ecm));
}

//////////////////////////////////////////////////
TEST_F(LightIntegrationTest, Intensity)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Light>(id, components::Light());

  Light light(id);

  sdf::Light lightSdf;
  double intensity = 0.2;
  lightSdf.SetIntensity(intensity);
  ecm.CreateComponent<components::Light>(id,
      components::Light(lightSdf));
  EXPECT_DOUBLE_EQ(intensity, *light.Intensity(ecm));
}

//////////////////////////////////////////////////
TEST_F(LightIntegrationTest, Direction)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Light>(id, components::Light());

  Light light(id);

  sdf::Light lightSdf;
  math::Vector3d dir(1.0, 0.0, 0.0);
  lightSdf.SetDirection(dir);
  ecm.CreateComponent<components::Light>(id,
      components::Light(lightSdf));
  EXPECT_EQ(dir, *light.Direction(ecm));
}

//////////////////////////////////////////////////
TEST_F(LightIntegrationTest, DiffuseColor)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Light>(id, components::Light());

  Light light(id);

  sdf::Light lightSdf;
  math::Color color(1.0, 1.0, 0.0);
  lightSdf.SetDiffuse(color);
  ecm.CreateComponent<components::Light>(id,
      components::Light(lightSdf));
  EXPECT_EQ(color, *light.DiffuseColor(ecm));
}

//////////////////////////////////////////////////
TEST_F(LightIntegrationTest, SpecularColor)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Light>(id, components::Light());

  Light light(id);

  sdf::Light lightSdf;
  math::Color color(1.0, 1.0, 0.0);
  lightSdf.SetSpecular(color);
  ecm.CreateComponent<components::Light>(id,
      components::Light(lightSdf));
  EXPECT_EQ(color, *light.SpecularColor(ecm));
}

//////////////////////////////////////////////////
TEST_F(LightIntegrationTest, AttenuationRange)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Light>(id, components::Light());

  Light light(id);

  sdf::Light lightSdf;
  double range = 2.4;
  lightSdf.SetAttenuationRange(range);
  ecm.CreateComponent<components::Light>(id,
      components::Light(lightSdf));
  EXPECT_DOUBLE_EQ(range, *light.AttenuationRange(ecm));
}

//////////////////////////////////////////////////
TEST_F(LightIntegrationTest, AttenuationConstant)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Light>(id, components::Light());

  Light light(id);

  sdf::Light lightSdf;
  double value = 0.2;
  lightSdf.SetConstantAttenuationFactor(value);
  ecm.CreateComponent<components::Light>(id,
      components::Light(lightSdf));
  EXPECT_DOUBLE_EQ(value, *light.AttenuationConstant(ecm));
}

//////////////////////////////////////////////////
TEST_F(LightIntegrationTest, AttenuationLinear)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Light>(id, components::Light());

  Light light(id);

  sdf::Light lightSdf;
  double value = 0.1;
  lightSdf.SetLinearAttenuationFactor(value);
  ecm.CreateComponent<components::Light>(id,
      components::Light(lightSdf));
  EXPECT_DOUBLE_EQ(value, *light.AttenuationLinear(ecm));
}

//////////////////////////////////////////////////
TEST_F(LightIntegrationTest, AttenuationQuadratic)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Light>(id, components::Light());

  Light light(id);

  sdf::Light lightSdf;
  double value = 0.01;
  lightSdf.SetQuadraticAttenuationFactor(value);
  ecm.CreateComponent<components::Light>(id,
      components::Light(lightSdf));
  EXPECT_DOUBLE_EQ(value, *light.AttenuationQuadratic(ecm));
}

//////////////////////////////////////////////////
TEST_F(LightIntegrationTest, SpotInnerAngle)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Light>(id, components::Light());

  Light light(id);

  sdf::Light lightSdf;
  math::Angle angle(1.57);
  lightSdf.SetSpotInnerAngle(angle);
  ecm.CreateComponent<components::Light>(id,
      components::Light(lightSdf));
  EXPECT_EQ(angle, *light.SpotInnerAngle(ecm));
}


//////////////////////////////////////////////////
TEST_F(LightIntegrationTest, SpotOuterAngle)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Light>(id, components::Light());

  Light light(id);

  sdf::Light lightSdf;
  math::Angle angle(2.3);
  lightSdf.SetSpotOuterAngle(angle);
  ecm.CreateComponent<components::Light>(id,
      components::Light(lightSdf));
  EXPECT_EQ(angle, *light.SpotOuterAngle(ecm));
}

//////////////////////////////////////////////////
TEST_F(LightIntegrationTest, SpotFalloff)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Light>(id, components::Light());

  Light light(id);

  sdf::Light lightSdf;
  double falloff(0.3);
  lightSdf.SetSpotFalloff(falloff);
  ecm.CreateComponent<components::Light>(id,
      components::Light(lightSdf));
  EXPECT_DOUBLE_EQ(falloff, *light.SpotFalloff(ecm));
}

/////////////////////////////////////////////////
TEST_F(LightIntegrationTest, SetPose)
{
  EntityComponentManager ecm;

  auto eLight = ecm.CreateEntity();
  ecm.CreateComponent(eLight, components::Light());

  Light light(eLight);
  EXPECT_EQ(eLight, light.Entity());

  ASSERT_TRUE(light.Valid(ecm));

  // No LightCmd should exist by default
  EXPECT_EQ(nullptr, ecm.Component<components::LightCmd>(eLight));

  math::Pose3d poseCmd(0.1, -2, 30, 0.2, 0.3, 0.8);
  light.SetPose(ecm, poseCmd);

  // light cmd should exist
  EXPECT_NE(nullptr, ecm.Component<components::LightCmd>(eLight));
  EXPECT_EQ(poseCmd, msgs::Convert(
      ecm.Component<components::LightCmd>(eLight)->Data().pose()));

  // Make sure the light cmd is updated
  math::Pose3d poseCmd2(9.3, -8, -1, 0.0, -0.3, 3.8);
  light.SetPose(ecm, poseCmd2);
  EXPECT_EQ(poseCmd2, msgs::Convert(
      ecm.Component<components::LightCmd>(eLight)->Data().pose()));
}

/////////////////////////////////////////////////
TEST_F(LightIntegrationTest, SetCastShadows)
{
  EntityComponentManager ecm;

  auto eLight = ecm.CreateEntity();
  ecm.CreateComponent(eLight, components::Light());

  Light light(eLight);
  EXPECT_EQ(eLight, light.Entity());

  ASSERT_TRUE(light.Valid(ecm));

  // No LightCmd should exist by default
  EXPECT_EQ(nullptr, ecm.Component<components::LightCmd>(eLight));

  bool castShadows = true;
  light.SetCastShadows(ecm, castShadows);

  // light cmd should exist
  EXPECT_NE(nullptr, ecm.Component<components::LightCmd>(eLight));
  EXPECT_EQ(castShadows,
    ecm.Component<components::LightCmd>(eLight)->Data().cast_shadows());

  // Make sure the light cmd is updated
  bool castShadows2 = false;
  light.SetCastShadows(ecm, castShadows2);
  EXPECT_EQ(castShadows2,
    ecm.Component<components::LightCmd>(eLight)->Data().cast_shadows());
}

/////////////////////////////////////////////////
TEST_F(LightIntegrationTest, SetIntensity)
{
  EntityComponentManager ecm;

  auto eLight = ecm.CreateEntity();
  ecm.CreateComponent(eLight, components::Light());

  Light light(eLight);
  EXPECT_EQ(eLight, light.Entity());

  ASSERT_TRUE(light.Valid(ecm));

  // No LightCmd should exist by default
  EXPECT_EQ(nullptr, ecm.Component<components::LightCmd>(eLight));

  double intensity = 0.3;
  light.SetIntensity(ecm, intensity);

  // light cmd should exist
  EXPECT_NE(nullptr, ecm.Component<components::LightCmd>(eLight));
  EXPECT_FLOAT_EQ(intensity,
    ecm.Component<components::LightCmd>(eLight)->Data().intensity());

  // Make sure the light cmd is updated
  double intensity2 = 0.001;
  light.SetIntensity(ecm, intensity2);
  EXPECT_FLOAT_EQ(intensity2,
    ecm.Component<components::LightCmd>(eLight)->Data().intensity());
}

/////////////////////////////////////////////////
TEST_F(LightIntegrationTest, SetDirection)
{
  EntityComponentManager ecm;

  auto eLight = ecm.CreateEntity();
  ecm.CreateComponent(eLight, components::Light());

  Light light(eLight);
  EXPECT_EQ(eLight, light.Entity());

  ASSERT_TRUE(light.Valid(ecm));

  // No LightCmd should exist by default
  EXPECT_EQ(nullptr, ecm.Component<components::LightCmd>(eLight));

  math::Vector3d dir(0.3, 0.4, 0.6);
  light.SetDirection(ecm, dir);

  // light cmd should exist
  EXPECT_NE(nullptr, ecm.Component<components::LightCmd>(eLight));
  EXPECT_EQ(dir, msgs::Convert(
      ecm.Component<components::LightCmd>(eLight)->Data().direction()));

  // Make sure the light cmd is updated
  math::Vector3d dir2(1.0, 0.0, 0.0);
  light.SetDirection(ecm, dir2);
  EXPECT_EQ(dir2, msgs::Convert(
      ecm.Component<components::LightCmd>(eLight)->Data().direction()));
}

/////////////////////////////////////////////////
TEST_F(LightIntegrationTest, SetDiffuseColor)
{
  EntityComponentManager ecm;

  auto eLight = ecm.CreateEntity();
  ecm.CreateComponent(eLight, components::Light());

  Light light(eLight);
  EXPECT_EQ(eLight, light.Entity());

  ASSERT_TRUE(light.Valid(ecm));

  // No LightCmd should exist by default
  EXPECT_EQ(nullptr, ecm.Component<components::LightCmd>(eLight));

  math::Color color(1.0, 0.0, 1.0);
  light.SetDiffuseColor(ecm, color);

  // light cmd should exist
  EXPECT_NE(nullptr, ecm.Component<components::LightCmd>(eLight));
  EXPECT_EQ(color, msgs::Convert(
    ecm.Component<components::LightCmd>(eLight)->Data().diffuse()));

  // Make sure the light cmd is updated
  math::Color color2(1.0, 0.5, 0.5);
  light.SetDiffuseColor(ecm, color2);
  EXPECT_EQ(color2, msgs::Convert(
    ecm.Component<components::LightCmd>(eLight)->Data().diffuse()));
}

/////////////////////////////////////////////////
TEST_F(LightIntegrationTest, SetSpecularColor)
{
  EntityComponentManager ecm;

  auto eLight = ecm.CreateEntity();
  ecm.CreateComponent(eLight, components::Light());

  Light light(eLight);
  EXPECT_EQ(eLight, light.Entity());

  ASSERT_TRUE(light.Valid(ecm));

  // No LightCmd should exist by default
  EXPECT_EQ(nullptr, ecm.Component<components::LightCmd>(eLight));

  math::Color color(1.0, 1.0, 0.0);
  light.SetSpecularColor(ecm, color);

  // light cmd should exist
  EXPECT_NE(nullptr, ecm.Component<components::LightCmd>(eLight));
  EXPECT_EQ(color, msgs::Convert(
    ecm.Component<components::LightCmd>(eLight)->Data().specular()));

  // Make sure the light cmd is updated
  math::Color color2(0.0, 1.0, 0.0);
  light.SetSpecularColor(ecm, color2);
  EXPECT_EQ(color2, msgs::Convert(
    ecm.Component<components::LightCmd>(eLight)->Data().specular()));
}

//////////////////////////////////////////////////
TEST_F(LightIntegrationTest, SetAttenuationRange)
{
  EntityComponentManager ecm;

  auto eLight = ecm.CreateEntity();
  ecm.CreateComponent(eLight, components::Light());

  Light light(eLight);
  EXPECT_EQ(eLight, light.Entity());

  ASSERT_TRUE(light.Valid(ecm));

  // No LightCmd should exist by default
  EXPECT_EQ(nullptr, ecm.Component<components::LightCmd>(eLight));

  double range = 56.2;
  light.SetAttenuationRange(ecm, range);

  // light cmd should exist
  EXPECT_NE(nullptr, ecm.Component<components::LightCmd>(eLight));
  EXPECT_FLOAT_EQ(range,
    ecm.Component<components::LightCmd>(eLight)->Data().range());

  // Make sure the light cmd is updated
  double range2 = 2777.9;
  light.SetAttenuationRange(ecm, range2);
  EXPECT_FLOAT_EQ(range2,
    ecm.Component<components::LightCmd>(eLight)->Data().range());
}

//////////////////////////////////////////////////
TEST_F(LightIntegrationTest, SetAttenuationConstant)
{
  EntityComponentManager ecm;

  auto eLight = ecm.CreateEntity();
  ecm.CreateComponent(eLight, components::Light());

  Light light(eLight);
  EXPECT_EQ(eLight, light.Entity());

  ASSERT_TRUE(light.Valid(ecm));

  // No LightCmd should exist by default
  EXPECT_EQ(nullptr, ecm.Component<components::LightCmd>(eLight));

  double value = 3.0;
  light.SetAttenuationConstant(ecm, value);

  // light cmd should exist
  EXPECT_NE(nullptr, ecm.Component<components::LightCmd>(eLight));
  EXPECT_FLOAT_EQ(value,
    ecm.Component<components::LightCmd>(eLight)->Data().attenuation_constant());

  // Make sure the light cmd is updated
  double value2 = 5.0;
  light.SetAttenuationConstant(ecm, value2);
  EXPECT_FLOAT_EQ(value2,
    ecm.Component<components::LightCmd>(eLight)->Data().attenuation_constant());
}

//////////////////////////////////////////////////
TEST_F(LightIntegrationTest, SetAttenuationLinear)
{
  EntityComponentManager ecm;

  auto eLight = ecm.CreateEntity();
  ecm.CreateComponent(eLight, components::Light());

  Light light(eLight);
  EXPECT_EQ(eLight, light.Entity());

  ASSERT_TRUE(light.Valid(ecm));

  // No LightCmd should exist by default
  EXPECT_EQ(nullptr, ecm.Component<components::LightCmd>(eLight));

  double value = 0.1;
  light.SetAttenuationLinear(ecm, value);

  // light cmd should exist
  EXPECT_NE(nullptr, ecm.Component<components::LightCmd>(eLight));
  EXPECT_FLOAT_EQ(value,
    ecm.Component<components::LightCmd>(eLight)->Data().attenuation_linear());

  // Make sure the light cmd is updated
  double value2 = 0.2;
  light.SetAttenuationLinear(ecm, value2);
  EXPECT_FLOAT_EQ(value2,
    ecm.Component<components::LightCmd>(eLight)->Data().attenuation_linear());
}

//////////////////////////////////////////////////
TEST_F(LightIntegrationTest, SetAttenuationQuadratic)
{
  EntityComponentManager ecm;

  auto eLight = ecm.CreateEntity();
  ecm.CreateComponent(eLight, components::Light());

  Light light(eLight);
  EXPECT_EQ(eLight, light.Entity());

  ASSERT_TRUE(light.Valid(ecm));

  // No LightCmd should exist by default
  EXPECT_EQ(nullptr, ecm.Component<components::LightCmd>(eLight));

  double value = 0.3;
  light.SetAttenuationQuadratic(ecm, value);

  // light cmd should exist
  EXPECT_NE(nullptr, ecm.Component<components::LightCmd>(eLight));
  EXPECT_FLOAT_EQ(value,
      ecm.Component<components::LightCmd>(
      eLight)->Data().attenuation_quadratic());

  // Make sure the light cmd is updated
  double value2 = 0.7;
  light.SetAttenuationQuadratic(ecm, value2);
  EXPECT_FLOAT_EQ(value2,
      ecm.Component<components::LightCmd>(
      eLight)->Data().attenuation_quadratic());
}

//////////////////////////////////////////////////
TEST_F(LightIntegrationTest, SetSpotInnerAngle)
{
  EntityComponentManager ecm;

  auto eLight = ecm.CreateEntity();
  ecm.CreateComponent(eLight, components::Light());

  Light light(eLight);
  EXPECT_EQ(eLight, light.Entity());

  ASSERT_TRUE(light.Valid(ecm));

  // No LightCmd should exist by default
  EXPECT_EQ(nullptr, ecm.Component<components::LightCmd>(eLight));

  math::Angle angle(2.9);
  light.SetSpotInnerAngle(ecm, angle.Radian());

  // light cmd should exist
  EXPECT_NE(nullptr, ecm.Component<components::LightCmd>(eLight));
  EXPECT_FLOAT_EQ(angle.Radian(),
    ecm.Component<components::LightCmd>(eLight)->Data().spot_inner_angle());

  // Make sure the light cmd is updated
  math::Angle angle2(0.9);
  light.SetSpotInnerAngle(ecm, angle2.Radian());
  EXPECT_FLOAT_EQ(angle2.Radian(),
    ecm.Component<components::LightCmd>(eLight)->Data().spot_inner_angle());
}

//////////////////////////////////////////////////
TEST_F(LightIntegrationTest, SetSpotOuterAngle)
{
  EntityComponentManager ecm;

  auto eLight = ecm.CreateEntity();
  ecm.CreateComponent(eLight, components::Light());

  Light light(eLight);
  EXPECT_EQ(eLight, light.Entity());

  ASSERT_TRUE(light.Valid(ecm));

  // No LightCmd should exist by default
  EXPECT_EQ(nullptr, ecm.Component<components::LightCmd>(eLight));

  math::Angle angle(3.1);
  light.SetSpotOuterAngle(ecm, angle.Radian());

  // light cmd should exist
  EXPECT_NE(nullptr, ecm.Component<components::LightCmd>(eLight));
  EXPECT_FLOAT_EQ(angle.Radian(),
    ecm.Component<components::LightCmd>(eLight)->Data().spot_outer_angle());

  // Make sure the light cmd is updated
  math::Angle angle2(0.8);
  light.SetSpotOuterAngle(ecm, angle2.Radian());
  EXPECT_FLOAT_EQ(angle2.Radian(),
    ecm.Component<components::LightCmd>(eLight)->Data().spot_outer_angle());
}

//////////////////////////////////////////////////
TEST_F(LightIntegrationTest, SetSpotFalloff)
{
  EntityComponentManager ecm;

  auto eLight = ecm.CreateEntity();
  ecm.CreateComponent(eLight, components::Light());

  Light light(eLight);
  EXPECT_EQ(eLight, light.Entity());

  ASSERT_TRUE(light.Valid(ecm));

  // No LightCmd should exist by default
  EXPECT_EQ(nullptr, ecm.Component<components::LightCmd>(eLight));

  double falloff = 0.3;
  light.SetSpotFalloff(ecm, falloff);

  // light cmd should exist
  EXPECT_NE(nullptr, ecm.Component<components::LightCmd>(eLight));
  EXPECT_FLOAT_EQ(falloff,
    ecm.Component<components::LightCmd>(eLight)->Data().spot_falloff());

  // Make sure the light cmd is updated
  double falloff2 = 1.0;
  light.SetSpotFalloff(ecm, falloff2);
  EXPECT_FLOAT_EQ(falloff2,
    ecm.Component<components::LightCmd>(eLight)->Data().spot_falloff());
}

//////////////////////////////////////////////////
TEST_F(LightIntegrationTest, Parent)
{
  EntityComponentManager ecm;

  {
    // Link as parent
    auto eLink = ecm.CreateEntity();
    ecm.CreateComponent(eLink, components::Link());
    auto eLight = ecm.CreateEntity();
    ecm.CreateComponent(eLight, components::Light());

    Light light(eLight);
    EXPECT_EQ(eLight, light.Entity());
    EXPECT_FALSE(light.Parent(ecm).has_value());

    ecm.CreateComponent<components::ParentEntity>(eLight,
        components::ParentEntity(eLink));
    ASSERT_TRUE(light.Valid(ecm));

    // Check parent link
    EXPECT_EQ(eLink, ecm.ParentEntity(eLight));
    auto parentLink = light.Parent(ecm);
    EXPECT_EQ(eLink, parentLink);
  }

  {
    // World as parent
    auto eWorld = ecm.CreateEntity();
    ecm.CreateComponent(eWorld, components::World());
    auto eLight = ecm.CreateEntity();
    ecm.CreateComponent(eLight, components::Light());

    Light light(eLight);
    EXPECT_EQ(eLight, light.Entity());
    EXPECT_FALSE(light.Parent(ecm).has_value());

    ecm.CreateComponent<components::ParentEntity>(eLight,
        components::ParentEntity(eWorld));
    ASSERT_TRUE(light.Valid(ecm));

    // Check parent world
    EXPECT_EQ(eWorld, ecm.ParentEntity(eLight));
    auto parentWorld = light.Parent(ecm);
    EXPECT_EQ(eWorld, parentWorld);
  }
}
