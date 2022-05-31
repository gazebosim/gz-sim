/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include "LogicalAudio.hh"

namespace logical_audio = gz::sim::logical_audio;

using AttenuationFunction = logical_audio::AttenuationFunction;
using AttenuationShape = logical_audio::AttenuationShape;

//////////////////////////////////////////////////
TEST(LogicalAudioTest, Detect)
{
  // expect a detection if the volume level is >= the detection threshold
  // (as long as the volume level is > 0.0)
  EXPECT_TRUE(logical_audio::detect(1.0, 0.0));
  EXPECT_TRUE(logical_audio::detect(0.1, 0.0));
  EXPECT_TRUE(logical_audio::detect(0.7, 0.5));
  EXPECT_TRUE(logical_audio::detect(1.0, 1.0));

  // expect no detection if the volume level is < the detection threshold
  EXPECT_FALSE(logical_audio::detect(0.2, 0.5));
  EXPECT_FALSE(logical_audio::detect(0.7, 0.9));
  EXPECT_FALSE(logical_audio::detect(0.9, 1.0));

  // make sure no detections occur if the volume level is 0
  EXPECT_FALSE(logical_audio::detect(0.0, 0.0));
  EXPECT_FALSE(logical_audio::detect(0.0, 0.5));
  EXPECT_FALSE(logical_audio::detect(0.0, 1.0));
}

//////////////////////////////////////////////////
TEST(LogicalAudioTest, ComputeVolume)
{
  // make sure computed volume is the source's emission volume inside of the
  // inner radius
  EXPECT_DOUBLE_EQ(1.0,
      logical_audio::computeVolume(true, AttenuationFunction::LINEAR,
        AttenuationShape::SPHERE, 1.0, 1.0, 5.0,
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  EXPECT_DOUBLE_EQ(1.0,
      logical_audio::computeVolume(true, AttenuationFunction::LINEAR,
        AttenuationShape::SPHERE, 1.0, 1.0, 5.0,
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.5, 0.5, 0.5, 0.0, 0.0, 0.0}));
  EXPECT_DOUBLE_EQ(0.3,
      logical_audio::computeVolume(true, AttenuationFunction::LINEAR,
        AttenuationShape::SPHERE, 0.3, 4.0, 5.0,
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 2.0, 0.5, 1.0, 6.0, 9.0}));

  // check volume at the inner radius
  EXPECT_DOUBLE_EQ(1.0,
      logical_audio::computeVolume(true, AttenuationFunction::LINEAR,
        AttenuationShape::SPHERE, 1.0, 1.0, 5.0,
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  EXPECT_DOUBLE_EQ(0.6,
      logical_audio::computeVolume(true, AttenuationFunction::LINEAR,
        AttenuationShape::SPHERE, 0.6, 10.0, 25.0,
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 10.0, 0.5, 2.0, -0.1}));

  // check volume between the inner radius and falloff distance
  EXPECT_DOUBLE_EQ(0.5,
      logical_audio::computeVolume(true, AttenuationFunction::LINEAR,
        AttenuationShape::SPHERE, 1.0, 1.0, 11.0,
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {6.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  EXPECT_DOUBLE_EQ(0.1,
      logical_audio::computeVolume(true, AttenuationFunction::LINEAR,
        AttenuationShape::SPHERE, 1.0, 1.0, 11.0,
        {0.0, 0.0, 0.0, -0.1, 0.6, 2.0},
        {10.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  EXPECT_DOUBLE_EQ(0.5,
      logical_audio::computeVolume(true, AttenuationFunction::LINEAR,
        AttenuationShape::SPHERE, 1.0, 0.0, 10.0,
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {5.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  EXPECT_DOUBLE_EQ(0.75,
      logical_audio::computeVolume(true, AttenuationFunction::LINEAR,
        AttenuationShape::SPHERE, 1.0, 0.0, 10.0,
        {0.0, 0.0, 0.0, 0.8, 0.2, 5.0},
        {0.0, 2.5, 0.0, 1.0, 0.3, 9.0}));
  EXPECT_DOUBLE_EQ(0.125,
      logical_audio::computeVolume(true, AttenuationFunction::LINEAR,
        AttenuationShape::SPHERE, 0.5, 0.0, 10.0,
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 7.5, -1.0, -0.3, 7.2}));

  // check volume at the falloff distance (make sure it is 0)
  EXPECT_DOUBLE_EQ(0.0,
      logical_audio::computeVolume(true, AttenuationFunction::LINEAR,
        AttenuationShape::SPHERE, 1.0, 0.0, 10.0,
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {10.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  EXPECT_DOUBLE_EQ(0.0,
      logical_audio::computeVolume(true, AttenuationFunction::LINEAR,
        AttenuationShape::SPHERE, 0.3, 2.0, 5.0,
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {3.0, 4.0, 0.0, 1.0, -0.5, 0.8}));

  // check volume past the falloff distance (make sure it is 0)
  EXPECT_DOUBLE_EQ(0.0,
      logical_audio::computeVolume(true, AttenuationFunction::LINEAR,
        AttenuationShape::SPHERE, 1.0, 5.0, 10.0,
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {10.0, 10.0, 10.0, 0.0, 0.0, 0.0}));
  EXPECT_DOUBLE_EQ(0.0,
      logical_audio::computeVolume(true, AttenuationFunction::LINEAR,
        AttenuationShape::SPHERE, 0.4, 5.0, 8.0,
        {0.0, 0.0, 0.0, 1.0, 2.0, 3.0},
        {210.0, 15.0, 0.0, 1.0, 5.0, -0.7}));

  // make sure computed volume is 0 if source isn't playing
  EXPECT_DOUBLE_EQ(0.0,
      logical_audio::computeVolume(false, AttenuationFunction::LINEAR,
        AttenuationShape::SPHERE, 1.0, 10.0, 100.0,
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 5.0, 0.0, 0.0, 0.0}));
  EXPECT_DOUBLE_EQ(0.0,
      logical_audio::computeVolume(false, AttenuationFunction::LINEAR,
        AttenuationShape::SPHERE, 0.5, 10.0, 100.0,
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  EXPECT_DOUBLE_EQ(0.0,
      logical_audio::computeVolume(false, AttenuationFunction::LINEAR,
        AttenuationShape::SPHERE, 0.0, 10.0, 100.0,
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  EXPECT_DOUBLE_EQ(0.0,
      logical_audio::computeVolume(false, AttenuationFunction::LINEAR,
        AttenuationShape::SPHERE, 0.8, 0.0, 100.0,
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {50.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  EXPECT_DOUBLE_EQ(0.0,
      logical_audio::computeVolume(false, AttenuationFunction::LINEAR,
        AttenuationShape::SPHERE, 0.8, 0.0, 100.0,
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {500.0, 0.0, 0.0, 0.0, 0.0, 0.0}));

  // make sure undefined attenuation functions and shapes are handled
  EXPECT_DOUBLE_EQ(-1.0,
      logical_audio::computeVolume(true, AttenuationFunction::UNDEFINED,
        AttenuationShape::SPHERE, 1.0, 10.0, 100.0,
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 5.0, 0.0, 0.0, 0.0}));
  EXPECT_DOUBLE_EQ(-1.0,
      logical_audio::computeVolume(true, AttenuationFunction::LINEAR,
        AttenuationShape::UNDEFINED, 1.0, 10.0, 100.0,
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 5.0, 0.0, 0.0, 0.0}));

  // make sure computed volume is always 0 if source emission volume is 0
  EXPECT_DOUBLE_EQ(0.0,
      logical_audio::computeVolume(true, AttenuationFunction::LINEAR,
        AttenuationShape::SPHERE, 0.0, 10.0, 100.0,
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  EXPECT_DOUBLE_EQ(0.0,
      logical_audio::computeVolume(true, AttenuationFunction::LINEAR,
        AttenuationShape::SPHERE, 0.0, 10.0, 100.0,
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {20.0, 20.0, 20.0, 0.0, 0.0, 0.0}));
}

//////////////////////////////////////////////////
TEST(LogicalAudioTest, AttenuationSetters)
{
  // test valid attenuation function strings
  AttenuationFunction func = AttenuationFunction::UNDEFINED;
  EXPECT_EQ(AttenuationFunction::UNDEFINED, func);
  logical_audio::setAttenuationFunction(func, "linear");
  EXPECT_EQ(AttenuationFunction::LINEAR, func);
  func = AttenuationFunction::UNDEFINED;
  EXPECT_EQ(AttenuationFunction::UNDEFINED, func);
  logical_audio::setAttenuationFunction(func, "LINEAR");
  EXPECT_EQ(AttenuationFunction::LINEAR, func);
  func = AttenuationFunction::UNDEFINED;
  EXPECT_EQ(AttenuationFunction::UNDEFINED, func);
  logical_audio::setAttenuationFunction(func, "LiNeaR");
  EXPECT_EQ(AttenuationFunction::LINEAR, func);

  // test valid attenuation shape strings
  AttenuationShape shape = AttenuationShape::UNDEFINED;
  EXPECT_EQ(AttenuationShape::UNDEFINED, shape);
  logical_audio::setAttenuationShape(shape, "sphere");
  EXPECT_EQ(AttenuationShape::SPHERE, shape);
  shape = AttenuationShape::UNDEFINED;
  EXPECT_EQ(AttenuationShape::UNDEFINED, shape);
  logical_audio::setAttenuationShape(shape, "SPHERE");
  EXPECT_EQ(AttenuationShape::SPHERE, shape);
  shape = AttenuationShape::UNDEFINED;
  EXPECT_EQ(AttenuationShape::UNDEFINED, shape);
  logical_audio::setAttenuationShape(shape, "sPHerE");
  EXPECT_EQ(AttenuationShape::SPHERE, shape);

  // test invalid attenuation function strings
  func = AttenuationFunction::LINEAR;
  EXPECT_EQ(AttenuationFunction::LINEAR, func);
  logical_audio::setAttenuationFunction(func, "linear ");
  EXPECT_EQ(AttenuationFunction::UNDEFINED, func);
  func = AttenuationFunction::LINEAR;
  EXPECT_EQ(AttenuationFunction::LINEAR, func);
  logical_audio::setAttenuationFunction(func, " LINEAR");
  EXPECT_EQ(AttenuationFunction::UNDEFINED, func);
  func = AttenuationFunction::LINEAR;
  EXPECT_EQ(AttenuationFunction::LINEAR, func);
  logical_audio::setAttenuationFunction(func, "some random string");
  EXPECT_EQ(AttenuationFunction::UNDEFINED, func);

  // test invalid attenuation shape strings
  shape = AttenuationShape::SPHERE;
  EXPECT_EQ(AttenuationShape::SPHERE, shape);
  logical_audio::setAttenuationShape(shape, " sphere ");
  EXPECT_EQ(AttenuationShape::UNDEFINED, shape);
  shape = AttenuationShape::SPHERE;
  EXPECT_EQ(AttenuationShape::SPHERE, shape);
  logical_audio::setAttenuationShape(shape, "SPHERE ");
  EXPECT_EQ(AttenuationShape::UNDEFINED, shape);
  shape = AttenuationShape::SPHERE;
  EXPECT_EQ(AttenuationShape::SPHERE, shape);
  logical_audio::setAttenuationShape(shape, "hello, world!");
  EXPECT_EQ(AttenuationShape::UNDEFINED, shape);
}

//////////////////////////////////////////////////
TEST(LogicalAudioTest, SourceComponentValidators)
{
  // make sure already valid inner radius and falloff distance aren't changed
  double innerRadius = 1.0;
  double falloffDistance = 5.0;
  logical_audio::validateInnerRadiusAndFalloffDistance(innerRadius,
      falloffDistance);
  EXPECT_DOUBLE_EQ(1.0, innerRadius);
  EXPECT_DOUBLE_EQ(5.0, falloffDistance);
  innerRadius = 0.0;
  falloffDistance = 0.5;
  logical_audio::validateInnerRadiusAndFalloffDistance(innerRadius,
      falloffDistance);
  EXPECT_DOUBLE_EQ(0.0, innerRadius);
  EXPECT_DOUBLE_EQ(0.5, falloffDistance);

  // make sure invalid inner radius and falloff distance are changed
  innerRadius = 1.0;
  falloffDistance = 0.5;
  logical_audio::validateInnerRadiusAndFalloffDistance(innerRadius,
      falloffDistance);
  EXPECT_DOUBLE_EQ(1.0, innerRadius);
  EXPECT_DOUBLE_EQ(2.0, falloffDistance);
  innerRadius = -1.0;
  falloffDistance = 0.5;
  logical_audio::validateInnerRadiusAndFalloffDistance(innerRadius,
      falloffDistance);
  EXPECT_DOUBLE_EQ(0.0, innerRadius);
  EXPECT_DOUBLE_EQ(0.5, falloffDistance);
  innerRadius = -1.0;
  falloffDistance = -0.5;
  logical_audio::validateInnerRadiusAndFalloffDistance(innerRadius,
      falloffDistance);
  EXPECT_DOUBLE_EQ(0.0, innerRadius);
  EXPECT_DOUBLE_EQ(1.0, falloffDistance);

  // make sure valid volume level isn't clipped
  double vol = 0.5;
  logical_audio::validateVolumeLevel(vol);
  EXPECT_DOUBLE_EQ(0.5, vol);
  vol = 0.0;
  logical_audio::validateVolumeLevel(vol);
  EXPECT_DOUBLE_EQ(0.0, vol);
  vol = 1.0;
  logical_audio::validateVolumeLevel(vol);
  EXPECT_DOUBLE_EQ(1.0, vol);

  // make sure invalid volume level is clipped
  vol = 2.0;
  logical_audio::validateVolumeLevel(vol);
  EXPECT_DOUBLE_EQ(1.0, vol);
  vol = -5.0;
  logical_audio::validateVolumeLevel(vol);
  EXPECT_DOUBLE_EQ(0.0, vol);
}
