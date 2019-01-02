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

#include <sdf/Light.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <ignition/msgs/Utility.hh>

#include "ignition/gazebo/Conversions.hh"
#include "ignition/gazebo/test_config.hh"

using namespace ignition;
using namespace gazebo;

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
  lightMsg = Convert<msgs::Light>(light);
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
