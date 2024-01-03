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

#include <gz/msgs/particle_emitter.pb.h>
#include <gz/msgs/wrench.pb.h>
#include <gz/msgs/Utility.hh>

#include <chrono>

#include <sdf/Cylinder.hh>
#include <sdf/Element.hh>
#include <sdf/AirPressure.hh>
#include <sdf/Altimeter.hh>
#include <sdf/Imu.hh>
#include <sdf/Magnetometer.hh>
#include <sdf/Material.hh>
#include <sdf/Noise.hh>
#include <sdf/Pbr.hh>
#include <sdf/sdf.hh>
#include <sdf/Sensor.hh>

#include "gz/sim/components/Actor.hh"
#include "gz/sim/components/AirPressureSensor.hh"
#include "gz/sim/components/Altimeter.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/Camera.hh"
#include "gz/sim/components/CanonicalLink.hh"
#include "gz/sim/components/ChildLinkName.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/DetachableJoint.hh"
#include "gz/sim/components/Geometry.hh"
#include "gz/sim/components/Gravity.hh"
#include "gz/sim/components/Imu.hh"
#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointAxis.hh"
#include "gz/sim/components/JointEffortLimitsCmd.hh"
#include "gz/sim/components/JointPositionLimitsCmd.hh"
#include "gz/sim/components/JointTransmittedWrench.hh"
#include "gz/sim/components/JointVelocityLimitsCmd.hh"
#include "gz/sim/components/JointType.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/components/Level.hh"
#include "gz/sim/components/LevelBuffer.hh"
#include "gz/sim/components/LevelEntityNames.hh"
#include "gz/sim/components/Light.hh"
#include "gz/sim/components/LinearAcceleration.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/LogicalAudio.hh"
#include "gz/sim/components/Magnetometer.hh"
#include "gz/sim/components/Material.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/ParentLinkName.hh"
#include "gz/sim/components/ParticleEmitter.hh"
#include "gz/sim/components/Performer.hh"
#include "gz/sim/components/PerformerAffinity.hh"
#include "gz/sim/components/PerformerLevels.hh"
#include "gz/sim/components/PhysicsEnginePlugin.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Projector.hh"
#include "gz/sim/components/Scene.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/SourceFilePath.hh"
#include "gz/sim/components/Static.hh"
#include "gz/sim/components/TemperatureRange.hh"
#include "gz/sim/components/ThreadPitch.hh"
#include "gz/sim/components/Visual.hh"
#include "gz/sim/components/World.hh"
#include "test_config.hh"  // NOLINT(build/include)
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

class ComponentsTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Actor)
{
  sdf::Actor data1;
  data1.SetName("abc");
  data1.SetRawPose({3, 2, 1, 0, 0, 0});
  data1.SetSkinFilename("def");

  sdf::Actor data2;

  // Create components
  auto comp11 = components::Actor(data1);
  auto comp12 = components::Actor(data1);
  auto comp2 = components::Actor(data2);

  // TODO(anyone) Equality operators

  // Stream operators
  std::ostringstream ostr;
  comp11.Serialize(ostr);
  std::istringstream istr(ostr.str());
  components::Actor comp3;
  comp3.Deserialize(istr);
  EXPECT_EQ("abc", comp3.Data().Name());
  EXPECT_EQ("def", comp3.Data().SkinFilename());
  EXPECT_EQ(math::Pose3d(3, 2, 1, 0, 0, 0), comp3.Data().RawPose());
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, AnimationName)
{
  // Create components
  auto comp11 = components::AnimationName("comp1");
  auto comp12 = components::AnimationName("comp1");
  auto comp2 = components::AnimationName("comp2");

  // Equality operators
  EXPECT_EQ(comp11, comp12);
  EXPECT_NE(comp11, comp2);
  EXPECT_TRUE(comp11 == comp12);
  EXPECT_TRUE(comp11 != comp2);
  EXPECT_FALSE(comp11 == comp2);
  EXPECT_FALSE(comp11 != comp12);

  // Stream operators
  std::ostringstream ostr;
  comp11.Serialize(ostr);
  EXPECT_EQ("comp1", ostr.str());

  std::istringstream istr("comp3");
  components::AnimationName comp3;
  comp3.Deserialize(istr);
  EXPECT_EQ("comp3", comp3.Data());
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, AnimationTime)
{
  auto start = std::chrono::steady_clock::now();
  auto end1 = start + std::chrono::seconds(5);
  auto end2 = start + std::chrono::seconds(10);

  // Create components
  auto comp1 = components::AnimationTime(end1 - start);
  auto comp2 = components::AnimationTime(end2 - start);

  // Equality operators
  EXPECT_NE(comp1, comp2);
  EXPECT_FALSE(comp1 == comp2);
  EXPECT_TRUE(comp1 != comp2);

  // Stream operators
  std::ostringstream ostr;
  comp1.Serialize(ostr);
  EXPECT_EQ("5000000000", ostr.str());

  std::istringstream istr(ostr.str());
  components::AnimationTime comp3;
  comp3.Deserialize(istr);
  EXPECT_EQ(comp1, comp3);
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, AirPressureSensor)
{
  sdf::Sensor data1;
  data1.SetName("abc");
  data1.SetType(sdf::SensorType::AIR_PRESSURE);
  data1.SetRawPose(math::Pose3d(1, 2, 3, 0, 0, 0));

  sdf::AirPressure airPressure1;
  data1.SetAirPressureSensor(airPressure1);

  sdf::Sensor data2;

  // Create components
  auto comp11 = components::AirPressureSensor(data1);
  auto comp12 = components::AirPressureSensor(data1);
  auto comp2 = components::AirPressureSensor(data2);

  // Equality operators
  EXPECT_EQ(comp11, comp12);
  EXPECT_NE(comp11, comp2);
  EXPECT_TRUE(comp11 == comp12);
  EXPECT_TRUE(comp11 != comp2);
  EXPECT_FALSE(comp11 == comp2);
  EXPECT_FALSE(comp11 != comp12);

  // Stream operators
  std::ostringstream ostr;
  comp11.Serialize(ostr);
  std::istringstream istr(ostr.str());
  components::AirPressureSensor comp3;
  comp3.Deserialize(istr);
  EXPECT_EQ("abc", comp3.Data().Name());
  EXPECT_EQ(sdf::SensorType::AIR_PRESSURE, comp3.Data().Type());
  EXPECT_EQ(math::Pose3d(1, 2, 3, 0, 0, 0), comp3.Data().RawPose());
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Altimeter)
{
  sdf::Sensor data1;
  sdf::Altimeter altimeter;
  sdf::Noise noise;
  noise.SetType(sdf::NoiseType::GAUSSIAN);
  noise.SetMean(0.3);
  altimeter.SetVerticalVelocityNoise(noise);
  data1.SetAltimeterSensor(altimeter);
  data1.SetType(sdf::SensorType::ALTIMETER);

  sdf::Sensor data2;
  sdf::Altimeter altimeter2;
  sdf::Noise noise2;
  noise2.SetType(sdf::NoiseType::GAUSSIAN);
  noise2.SetMean(0.2);
  altimeter2.SetVerticalVelocityNoise(noise2);
  data2.SetAltimeterSensor(altimeter2);
  data1.SetType(sdf::SensorType::ALTIMETER);

  // Create components
  auto comp11 = components::Altimeter(data1);
  auto comp12 = components::Altimeter(data1);
  auto comp2 = components::Altimeter(data2);

  // Equality operators
  EXPECT_EQ(comp11, comp12);
  EXPECT_NE(comp11, comp2);
  EXPECT_TRUE(comp11 == comp12);
  EXPECT_TRUE(comp11 != comp2);
  EXPECT_FALSE(comp11 == comp2);
  EXPECT_FALSE(comp11 != comp12);

  // Stream operator
  std::ostringstream ostr;
  comp11.Serialize(ostr);
  std::istringstream istr(ostr.str());
  components::Altimeter comp3;
  comp3.Deserialize(istr);
  EXPECT_EQ(sdf::SensorType::ALTIMETER, comp3.Data().Type());
  EXPECT_EQ(sdf::NoiseType::GAUSSIAN,
      comp3.Data().AltimeterSensor()->VerticalVelocityNoise().Type());
  EXPECT_DOUBLE_EQ(0.3,
      comp3.Data().AltimeterSensor()->VerticalVelocityNoise().Mean());
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, AngularVelocity)
{
  // Create components
  auto comp11 = components::AngularVelocity(math::Vector3d(1, 2, 3));
  auto comp12 = components::AngularVelocity(math::Vector3d(1, 2, 3));
  auto comp2 = components::AngularVelocity(math::Vector3d(2, 0, 0));

  // Equality operators
  EXPECT_EQ(comp11, comp12);
  EXPECT_NE(comp11, comp2);
  EXPECT_TRUE(comp11 == comp12);
  EXPECT_TRUE(comp11 != comp2);
  EXPECT_FALSE(comp11 == comp2);
  EXPECT_FALSE(comp11 != comp12);

  // Stream operators
  std::ostringstream ostr;
  comp11.Serialize(ostr);
  EXPECT_EQ("1 2 3", ostr.str());

  std::istringstream istr("3 2 1");
  components::AngularVelocity comp3(math::Vector3d::Zero);
  comp3.Deserialize(istr);
  EXPECT_EQ(math::Vector3d(3, 2, 1), comp3.Data());
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Camera)
{
  sdf::Sensor data1;
  sdf::Sensor data2;
  data2.SetName("other_name");

  // Create components
  auto comp11 = components::Camera(data1);
  auto comp12 = components::Camera(data1);
  auto comp2 = components::Camera(data2);

  // Equality operators
  EXPECT_EQ(comp11, comp12);
  EXPECT_NE(comp11, comp2);
  EXPECT_TRUE(comp11 == comp12);
  EXPECT_TRUE(comp11 != comp2);
  EXPECT_FALSE(comp11 == comp2);
  EXPECT_FALSE(comp11 != comp12);

  // TODO(anyone) Stream operator
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, CanonicalLink)
{
  // Create components
  auto comp1 = components::CanonicalLink();
  auto comp2 = components::CanonicalLink();

  // Equality operators
  EXPECT_EQ(comp1, comp2);
  EXPECT_TRUE(comp1 == comp2);
  EXPECT_FALSE(comp1 != comp2);

  // Stream operators
  std::ostringstream ostr;
  comp1.Serialize(ostr);
  EXPECT_EQ("-", ostr.str());

  std::istringstream istr("ignored");
  components::CanonicalLink comp3;
  comp3.Deserialize(istr);
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, ChildLinkName)
{
  // Create components
  auto comp11 = components::ChildLinkName("comp1");
  auto comp12 = components::ChildLinkName("comp1");
  auto comp2 = components::ChildLinkName("comp2");

  // Equality operators
  EXPECT_EQ(comp11, comp12);
  EXPECT_NE(comp11, comp2);
  EXPECT_TRUE(comp11 == comp12);
  EXPECT_TRUE(comp11 != comp2);
  EXPECT_FALSE(comp11 == comp2);
  EXPECT_FALSE(comp11 != comp12);

  // Stream operators
  std::ostringstream ostr;
  comp11.Serialize(ostr);
  EXPECT_EQ("comp1", ostr.str());

  std::istringstream istr("comp3");
  components::ChildLinkName comp3;
  comp3.Deserialize(istr);
  EXPECT_EQ("comp3", comp3.Data());
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Collision)
{
  // Create components
  auto comp1 = components::Collision();
  auto comp2 = components::Collision();

  // Equality operators
  EXPECT_EQ(comp1, comp2);
  EXPECT_TRUE(comp1 == comp2);
  EXPECT_FALSE(comp1 != comp2);

  // Stream operators
  std::ostringstream ostr;
  comp1.Serialize(ostr);
  EXPECT_EQ("-", ostr.str());

  std::istringstream istr("ignored");
  components::Collision comp3;
  comp3.Deserialize(istr);
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, DetachableJoint)
{
  components::DetachableJointInfo info1;
  info1.parentLink = 1;
  info1.childLink = 2;
  // Create components
  auto comp1 = components::DetachableJoint(info1);
  auto comp2 = components::DetachableJoint(info1);

  // Equality operators
  EXPECT_EQ(comp1, comp2);
  EXPECT_TRUE(comp1 == comp2);
  EXPECT_FALSE(comp1 != comp2);

  // Stream operators
  std::ostringstream ostr;
  comp1.Serialize(ostr);
  EXPECT_EQ("1 2 fixed", ostr.str());

  std::istringstream istr(ostr.str());
  components::DetachableJoint comp3;
  comp3.Deserialize(istr);
  EXPECT_EQ(comp1, comp3);
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Geometry)
{
  auto data1 = sdf::Geometry();
  data1.SetType(sdf::GeometryType::CYLINDER);
  sdf::Cylinder cylinderShape;
  cylinderShape.SetRadius(1.23);
  cylinderShape.SetLength(4.56);
  data1.SetCylinderShape(cylinderShape);

  auto data2 = sdf::Geometry();

  // Create components
  auto comp11 = components::Geometry(data1);
  auto comp12 = components::Geometry(data1);
  auto comp2 = components::Geometry(data2);

  // TODO(anyone) Equality operators

  // Stream operators
  std::ostringstream ostr;
  comp11.Serialize(ostr);
  std::istringstream istr(ostr.str());
  components::Geometry comp3;
  comp3.Deserialize(istr);
  EXPECT_EQ(sdf::GeometryType::CYLINDER, comp3.Data().Type());
  ASSERT_NE(nullptr, comp3.Data().CylinderShape());
  EXPECT_DOUBLE_EQ(1.23, comp3.Data().CylinderShape()->Radius());
  EXPECT_DOUBLE_EQ(4.56, comp3.Data().CylinderShape()->Length());
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Gravity)
{
  // Create components
  auto comp11 = components::Gravity(math::Vector3d(1, 2, 3));
  auto comp12 = components::Gravity(math::Vector3d(1, 2, 3));
  auto comp2 = components::Gravity(math::Vector3d(2, 0, 0));

  // Equality operators
  EXPECT_EQ(comp11, comp12);
  EXPECT_NE(comp11, comp2);
  EXPECT_TRUE(comp11 == comp12);
  EXPECT_TRUE(comp11 != comp2);
  EXPECT_FALSE(comp11 == comp2);
  EXPECT_FALSE(comp11 != comp12);

  // Stream operators
  std::ostringstream ostr;
  comp11.Serialize(ostr);
  EXPECT_EQ("1 2 3", ostr.str());

  std::istringstream istr("3 2 1");
  components::Gravity comp3(math::Vector3d::Zero);
  comp3.Deserialize(istr);
  EXPECT_EQ(math::Vector3d(3, 2, 1), comp3.Data());
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Imu)
{
  sdf::Sensor data1;
  data1.SetName("imu_sensor");
  data1.SetType(sdf::SensorType::IMU);
  data1.SetUpdateRate(100);
  data1.SetTopic("imu_data");
  data1.SetRawPose(math::Pose3d(1, 2, 3, 0, 0, 0));

  sdf::Imu imu1;
  data1.SetImuSensor(imu1);

  sdf::Sensor data2;
  data2.SetName("other_name");

  // Create components
  auto comp11 = components::Imu(data1);
  auto comp12 = components::Imu(data1);
  auto comp2 = components::Imu(data2);

  // Equality operators
  EXPECT_EQ(comp11, comp12);
  EXPECT_NE(comp11, comp2);
  EXPECT_TRUE(comp11 == comp12);
  EXPECT_TRUE(comp11 != comp2);
  EXPECT_FALSE(comp11 == comp2);
  EXPECT_FALSE(comp11 != comp12);

  // Stream operators
  std::ostringstream ostr;
  comp11.Serialize(ostr);
  std::istringstream istr(ostr.str());
  components::Imu comp3;
  comp3.Deserialize(istr);
  EXPECT_EQ("imu_sensor", comp3.Data().Name());
  EXPECT_EQ(sdf::SensorType::IMU, comp3.Data().Type());
  EXPECT_EQ("imu_data", comp3.Data().Topic());
  EXPECT_DOUBLE_EQ(100, comp3.Data().UpdateRate());
  EXPECT_EQ(math::Pose3d(1, 2, 3, 0, 0, 0), comp3.Data().RawPose());
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Inertial)
{
  // Create components
  auto comp11 = components::Inertial(math::Inertiald(math::MassMatrix3d(1,
      math::Vector3d::Zero, math::Vector3d::Zero), math::Pose3d::Zero));
  auto comp12 = components::Inertial(math::Inertiald(math::MassMatrix3d(1,
      math::Vector3d::Zero, math::Vector3d::Zero), math::Pose3d::Zero));
  auto comp2 = components::Inertial(math::Inertiald(math::MassMatrix3d(2,
      math::Vector3d::Zero, math::Vector3d::Zero), math::Pose3d::Zero));

  // Equality operators
  EXPECT_EQ(comp11, comp12);
  EXPECT_NE(comp11, comp2);
  EXPECT_TRUE(comp11 == comp12);
  EXPECT_TRUE(comp11 != comp2);
  EXPECT_FALSE(comp11 == comp2);
  EXPECT_FALSE(comp11 != comp12);

  // Stream operators
  std::ostringstream ostr;
  comp11.Serialize(ostr);
  std::istringstream istr(ostr.str());
  components::Inertial comp3;
  comp3.Deserialize(istr);
  EXPECT_DOUBLE_EQ(1.0, comp3.Data().MassMatrix().Mass());
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Joint)
{
  // Create components
  auto comp1 = components::Joint();
  auto comp2 = components::Joint();

  // Equality operators
  EXPECT_EQ(comp1, comp2);
  EXPECT_TRUE(comp1 == comp2);
  EXPECT_FALSE(comp1 != comp2);

  // Stream operators
  std::ostringstream ostr;
  comp1.Serialize(ostr);
  EXPECT_EQ("-", ostr.str());

  std::istringstream istr("ignored");
  components::Joint comp3;
  comp3.Deserialize(istr);
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, JointAxis)
{
  auto data1 = sdf::JointAxis();
  EXPECT_TRUE(data1.SetXyz(math::Vector3d(1, 2, 3)).empty());
  data1.SetXyzExpressedIn("__model__");
  data1.SetDamping(0.1);
  data1.SetFriction(0.2);
  data1.SetLower(0.3);
  data1.SetUpper(0.4);
  data1.SetEffort(0.5);
  data1.SetMaxVelocity(0.6);

  auto data2 = sdf::JointAxis();

  // Create components
  auto comp11 = components::JointAxis(data1);
  auto comp12 = components::JointAxis(data1);
  auto comp2 = components::JointAxis(data2);

  // TODO(anyone) Equality operators

  // Stream operators
  std::ostringstream ostr;
  comp11.Serialize(ostr);
  std::istringstream istr(ostr.str());

  components::JointAxis comp3;
  comp3.Deserialize(istr);

  EXPECT_EQ(math::Vector3d(1, 2, 3).Normalized(), comp3.Data().Xyz());
  EXPECT_DOUBLE_EQ(0.1, comp3.Data().Damping());
  EXPECT_DOUBLE_EQ(0.2, comp3.Data().Friction());
  EXPECT_DOUBLE_EQ(0.3, comp3.Data().Lower());
  EXPECT_DOUBLE_EQ(0.4, comp3.Data().Upper());
  EXPECT_DOUBLE_EQ(0.5, comp3.Data().Effort());
  EXPECT_DOUBLE_EQ(0.6, comp3.Data().MaxVelocity());
  EXPECT_EQ(comp3.Data().XyzExpressedIn(), "__model__");
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, JointEffortLimitsCmd)
{
  // Create components
  auto comp1 = components::JointEffortLimitsCmd();
  auto comp2 = components::JointEffortLimitsCmd();

  // Equality operators
  EXPECT_EQ(comp1, comp2);
  EXPECT_TRUE(comp1 == comp2);
  EXPECT_FALSE(comp1 != comp2);

  // Stream operators
  std::ostringstream ostr;
  comp1.Serialize(ostr);
  EXPECT_EQ("0", ostr.str());

  comp2.Data().push_back(math::Vector2d(-1.0, 1.0));

  std::ostringstream ostr2;
  comp2.Serialize(ostr2);
  EXPECT_EQ("1 -1 1", ostr2.str());

  comp2.Data().push_back(math::Vector2d(-2.5, 2.5));

  std::ostringstream ostr3;
  comp2.Serialize(ostr3);
  EXPECT_EQ("2 -1 1 -2.5 2.5", ostr3.str());

  std::istringstream istr("ignored");
  components::JointEffortLimitsCmd comp3;
  comp3.Deserialize(istr);
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, JointPositionLimitsCmd)
{
  // Create components
  auto comp1 = components::JointPositionLimitsCmd();
  auto comp2 = components::JointPositionLimitsCmd();
  components::JointPositionLimitsCmd comp3;

  // Equality operators
  EXPECT_EQ(comp1, comp2);
  EXPECT_TRUE(comp1 == comp2);
  EXPECT_FALSE(comp1 != comp2);

  // Stream operators
  std::ostringstream ostr;
  comp1.Serialize(ostr);
  EXPECT_EQ("0", ostr.str());

  auto istr = std::istringstream(ostr.str());
  comp3.Deserialize(istr);
  EXPECT_EQ(comp1, comp3);

  comp2.Data().push_back(math::Vector2d(-1.0, 1.0));

  std::ostringstream ostr2;
  comp2.Serialize(ostr2);
  EXPECT_EQ("1 -1 1", ostr2.str());

  istr = std::istringstream(ostr2.str());
  comp3.Deserialize(istr);
  EXPECT_EQ(comp2, comp3);

  comp2.Data().push_back(math::Vector2d(-2.5, 2.5));

  std::ostringstream ostr3;
  comp2.Serialize(ostr3);
  EXPECT_EQ("2 -1 1 -2.5 2.5", ostr3.str());

  istr = std::istringstream(ostr3.str());
  comp3.Deserialize(istr);
  EXPECT_EQ(comp2, comp3);

  istr = std::istringstream("ignored");
  comp3.Deserialize(istr);
  EXPECT_EQ(comp1, comp3);
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, JointVelocityLimitsCmd)
{
  // Create components
  auto comp1 = components::JointVelocityLimitsCmd();
  auto comp2 = components::JointVelocityLimitsCmd();

  // Equality operators
  EXPECT_EQ(comp1, comp2);
  EXPECT_TRUE(comp1 == comp2);
  EXPECT_FALSE(comp1 != comp2);

  // Stream operators
  std::ostringstream ostr;
  comp1.Serialize(ostr);
  EXPECT_EQ("0", ostr.str());

  comp2.Data().push_back(math::Vector2d(-1.0, 1.0));

  std::ostringstream ostr2;
  comp2.Serialize(ostr2);
  EXPECT_EQ("1 -1 1", ostr2.str());

  comp2.Data().push_back(math::Vector2d(-2.5, 2.5));

  std::ostringstream ostr3;
  comp2.Serialize(ostr3);
  EXPECT_EQ("2 -1 1 -2.5 2.5", ostr3.str());

  std::istringstream istr("ignored");
  components::JointVelocityLimitsCmd comp3;
  comp3.Deserialize(istr);
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, JointType)
{
  auto data1 = sdf::JointType::FIXED;
  auto data2 = sdf::JointType::REVOLUTE;

  // Create components
  auto comp11 = components::JointType(data1);
  auto comp12 = components::JointType(data1);
  auto comp2 = components::JointType(data2);

  // Equality operators
  EXPECT_EQ(comp11, comp12);
  EXPECT_NE(comp11, comp2);
  EXPECT_TRUE(comp11 == comp12);
  EXPECT_TRUE(comp11 != comp2);
  EXPECT_FALSE(comp11 == comp2);
  EXPECT_FALSE(comp11 != comp12);

  // Stream operators
  std::ostringstream ostr;
  comp11.Serialize(ostr);
  EXPECT_EQ(std::to_string(static_cast<int>(sdf::JointType::FIXED)),
      ostr.str());

  std::istringstream istr(std::to_string(static_cast<int>(
      sdf::JointType::SCREW)));
  components::JointType comp3;
  comp3.Deserialize(istr);
  EXPECT_EQ(sdf::JointType::SCREW, comp3.Data());
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, JointVelocity)
{
  // Create components
  auto comp11 = components::JointVelocity({1.2, 2.3, 3.4});

  // Stream operators
  std::ostringstream ostr;
  comp11.Serialize(ostr);

  std::istringstream istr(ostr.str());
  components::JointVelocity comp3;
  comp3.Deserialize(istr);
  ASSERT_EQ(3u, comp3.Data().size());
  EXPECT_DOUBLE_EQ(1.2, comp3.Data()[0]);
  EXPECT_DOUBLE_EQ(2.3, comp3.Data()[1]);
  EXPECT_DOUBLE_EQ(3.4, comp3.Data()[2]);
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, JointVelocityCmd)
{
  // Create components
  auto comp11 = components::JointVelocityCmd({1.2, 2.3, 3.4});

  // Stream operators
  std::ostringstream ostr;
  comp11.Serialize(ostr);

  std::istringstream istr(ostr.str());
  components::JointVelocityCmd comp3;
  comp3.Deserialize(istr);
  ASSERT_EQ(3u, comp3.Data().size());
  EXPECT_DOUBLE_EQ(1.2, comp3.Data()[0]);
  EXPECT_DOUBLE_EQ(2.3, comp3.Data()[1]);
  EXPECT_DOUBLE_EQ(3.4, comp3.Data()[2]);
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Level)
{
  // Create components
  auto comp1 = components::Level();
  auto comp2 = components::Level();

  // Equality operators
  EXPECT_EQ(comp1, comp2);
  EXPECT_TRUE(comp1 == comp2);
  EXPECT_FALSE(comp1 != comp2);

  // Stream operators
  std::ostringstream ostr;
  comp1.Serialize(ostr);
  EXPECT_EQ("-", ostr.str());

  std::istringstream istr("ignored");
  components::Level comp3;
  comp3.Deserialize(istr);
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, LevelBuffer)
{
  // Create components
  auto comp11 = components::LevelBuffer(1.5);

  // No double comparisons

  // Stream operators
  std::ostringstream ostr;
  comp11.Serialize(ostr);
  EXPECT_EQ("1.5", ostr.str());

  std::istringstream istr("3.3");
  components::LevelBuffer comp3;
  comp3.Deserialize(istr);
  EXPECT_DOUBLE_EQ(3.3, comp3.Data());
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, LevelEntityNames)
{
  std::set<std::string> data1({"level1", "level2"});
  std::set<std::string> data2({"level1"});

  // Create components
  auto comp11 = components::LevelEntityNames(data1);
  auto comp12 = components::LevelEntityNames(data1);
  auto comp2 = components::LevelEntityNames(data2);

  // Equality operators
  EXPECT_EQ(comp11, comp12);
  EXPECT_NE(comp11, comp2);
  EXPECT_TRUE(comp11 == comp12);
  EXPECT_TRUE(comp11 != comp2);
  EXPECT_FALSE(comp11 == comp2);
  EXPECT_FALSE(comp11 != comp12);

  // Stream operators
  std::ostringstream ostr;
  comp11.Serialize(ostr);
  EXPECT_EQ("level1\x1Flevel2\x1F", ostr.str());

  std::istringstream istr("level3\x1Flevel4");
  components::LevelEntityNames comp3;
  comp3.Deserialize(istr);

  std::set<std::string> data3({"level3", "level4"});
  EXPECT_EQ(data3, comp3.Data());
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Light)
{
  auto data1 = sdf::Light();
  data1.SetType(sdf::LightType::POINT);
  data1.SetName("light_test");
  data1.SetRawPose(math::Pose3d(1, 2, 4, 0, 0, GZ_PI));
  data1.SetDiffuse(math::Color(1, 0, 0, 1));
  data1.SetSpecular(math::Color(0, 1, 0, 1));
  data1.SetCastShadows(true);
  data1.SetAttenuationRange(1.3);
  data1.SetLinearAttenuationFactor(0.3);
  data1.SetQuadraticAttenuationFactor(0.1);
  data1.SetConstantAttenuationFactor(0.05);
  data1.SetDirection(math::Vector3d(2, 3, 4));
  data1.SetSpotInnerAngle(math::Angle(0.3));
  data1.SetSpotOuterAngle(math::Angle(2.3));
  data1.SetSpotFalloff(5.15);
  data1.SetIntensity(1.55);

  auto data2 = sdf::Light();

  // Create components
  auto comp11 = components::Light(data1);
  auto comp12 = components::Light(data1);
  auto comp2 = components::Light(data2);

  // TODO(anyone) Equality operators

  // Stream operators
  std::ostringstream ostr;
  comp11.Serialize(ostr);
  std::istringstream istr(ostr.str());
  components::Light comp3;
  comp3.Deserialize(istr);
  EXPECT_EQ(sdf::LightType::POINT, comp3.Data().Type());
  EXPECT_EQ("light_test", comp3.Data().Name());
  EXPECT_EQ(math::Pose3d(1, 2, 4, 0, 0, GZ_PI), comp3.Data().RawPose());
  EXPECT_EQ(math::Color(1, 0, 0, 1), comp3.Data().Diffuse());
  EXPECT_EQ(math::Color(0, 1, 0, 1), comp3.Data().Specular());
  EXPECT_TRUE(comp3.Data().CastShadows());
  EXPECT_FLOAT_EQ(1.3f, comp3.Data().AttenuationRange());
  EXPECT_FLOAT_EQ(0.3f, comp3.Data().LinearAttenuationFactor());
  EXPECT_FLOAT_EQ(0.1f, comp3.Data().QuadraticAttenuationFactor());
  EXPECT_FLOAT_EQ(0.05f, comp3.Data().ConstantAttenuationFactor());
  EXPECT_EQ(math::Angle(0.3), comp3.Data().SpotInnerAngle());
  EXPECT_EQ(math::Angle(2.3), comp3.Data().SpotOuterAngle());
  EXPECT_FLOAT_EQ(5.15f, comp3.Data().SpotFalloff());
  EXPECT_EQ(math::Vector3d(2, 3, 4), comp3.Data().Direction());
  EXPECT_FLOAT_EQ(1.55f, comp3.Data().Intensity());
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, LinearAcceleration)
{
  // Create components
  auto comp11 = components::LinearAcceleration(math::Vector3d(1, 2, 3));
  auto comp12 = components::LinearAcceleration(math::Vector3d(1, 2, 3));
  auto comp2 = components::LinearAcceleration(math::Vector3d(2, 0, 0));

  // Equality operators
  EXPECT_EQ(comp11, comp12);
  EXPECT_NE(comp11, comp2);
  EXPECT_TRUE(comp11 == comp12);
  EXPECT_TRUE(comp11 != comp2);
  EXPECT_FALSE(comp11 == comp2);
  EXPECT_FALSE(comp11 != comp12);

  // Stream operators
  std::ostringstream ostr;
  comp11.Serialize(ostr);
  EXPECT_EQ("1 2 3", ostr.str());

  std::istringstream istr("3 2 1");
  components::LinearAcceleration comp3(math::Vector3d::Zero);
  comp3.Deserialize(istr);
  EXPECT_EQ(math::Vector3d(3, 2, 1), comp3.Data());
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, LinearVelocity)
{
  // Create components
  auto comp11 = components::LinearVelocity(math::Vector3d(1, 2, 3));
  auto comp12 = components::LinearVelocity(math::Vector3d(1, 2, 3));
  auto comp2 = components::LinearVelocity(math::Vector3d(2, 0, 0));

  // Equality operators
  EXPECT_EQ(comp11, comp12);
  EXPECT_NE(comp11, comp2);
  EXPECT_TRUE(comp11 == comp12);
  EXPECT_TRUE(comp11 != comp2);
  EXPECT_FALSE(comp11 == comp2);
  EXPECT_FALSE(comp11 != comp12);

  // Stream operators
  std::ostringstream ostr;
  comp11.Serialize(ostr);
  EXPECT_EQ("1 2 3", ostr.str());

  std::istringstream istr("3 2 1");
  components::LinearVelocity comp3(math::Vector3d::Zero);
  comp3.Deserialize(istr);
  EXPECT_EQ(math::Vector3d(3, 2, 1), comp3.Data());
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Link)
{
  // Create components
  auto comp1 = components::Link();
  auto comp2 = components::Link();

  // Equality operators
  EXPECT_EQ(comp1, comp2);
  EXPECT_TRUE(comp1 == comp2);
  EXPECT_FALSE(comp1 != comp2);

  // Stream operators
  std::ostringstream ostr;
  comp1.Serialize(ostr);
  EXPECT_EQ("-", ostr.str());

  std::istringstream istr("ignored");
  components::Link comp3;
  comp3.Deserialize(istr);
}

//////////////////////////////////////////////////
TEST_F(ComponentsTest, LogicalAudioSource)
{
  logical_audio::Source source1;
  source1.id = 0;
  source1.attFunc = logical_audio::AttenuationFunction::LINEAR;
  source1.attShape = logical_audio::AttenuationShape::UNDEFINED;
  source1.innerRadius = 0.25;
  source1.falloffDistance = 5.0;
  source1.emissionVolume = 1.0;

  logical_audio::Source source2;
  source2.id = 1;
  source2.attFunc = source1.attFunc;
  source2.attShape = source1.attShape;
  source2.innerRadius = source1.innerRadius;
  source2.falloffDistance = source1.falloffDistance;
  source2.emissionVolume = source1.emissionVolume;

  // create components
  auto comp1 = components::LogicalAudioSource(source1);
  auto comp2 = components::LogicalAudioSource(source2);

  // equality operators
  EXPECT_NE(comp1, comp2);
  EXPECT_FALSE(comp1 == comp2);
  EXPECT_TRUE(comp1 != comp2);

  // stream operators
  std::ostringstream ostr;
  comp1.Serialize(ostr);
  EXPECT_EQ("0 0 1 0.25 5 1", ostr.str());

  std::istringstream istr(ostr.str());
  components::LogicalAudioSource comp3;
  comp3.Deserialize(istr);
  EXPECT_EQ(comp1, comp3);
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, LogicalAudioSourcePlayInfo)
{
  auto start = std::chrono::steady_clock::now();
  auto end = start + std::chrono::seconds(1);

  logical_audio::SourcePlayInfo playInfo1;
  playInfo1.playing = true;
  playInfo1.playDuration = std::chrono::seconds(1);
  playInfo1.startTime = end - start;

  logical_audio::SourcePlayInfo playInfo2;
  playInfo2.playing = false;
  playInfo2.playDuration = std::chrono::seconds(5);
  playInfo2.startTime = end - start;

  // create components
  auto comp1 = components::LogicalAudioSourcePlayInfo(playInfo1);
  auto comp2 = components::LogicalAudioSourcePlayInfo(playInfo2);

  // equality operators
  EXPECT_NE(comp1, comp2);
  EXPECT_FALSE(comp1 == comp2);
  EXPECT_TRUE(comp1 != comp2);

  // stream operators
  std::ostringstream ostr;
  comp1.Serialize(ostr);
  EXPECT_EQ("1 1 1000000000", ostr.str());

  std::istringstream istr(ostr.str());
  components::LogicalAudioSourcePlayInfo comp3;
  comp3.Deserialize(istr);
  EXPECT_EQ(comp1, comp3);
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, LogicalMicrophone)
{
  logical_audio::Microphone mic1;
  mic1.id = 4;
  mic1.volumeDetectionThreshold = 0.5;

  logical_audio::Microphone mic2;
  mic2.id = 8;
  mic2.volumeDetectionThreshold = 0.8;

  // create components
  auto comp1 = components::LogicalMicrophone(mic1);
  auto comp2 = components::LogicalMicrophone(mic2);

  // equality operators
  EXPECT_NE(comp1, comp2);
  EXPECT_FALSE(comp1 == comp2);
  EXPECT_TRUE(comp1 != comp2);

  // stream operators
  std::ostringstream ostr;
  comp1.Serialize(ostr);
  EXPECT_EQ("4 0.5", ostr.str());

  std::istringstream istr(ostr.str());
  components::LogicalMicrophone comp3;
  comp3.Deserialize(istr);
  EXPECT_EQ(comp1, comp3);
  EXPECT_DOUBLE_EQ(comp1.Data().volumeDetectionThreshold,
      comp3.Data().volumeDetectionThreshold);
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Magnetometer)
{
  sdf::Sensor data1;
  data1.SetName("banana");
  data1.SetType(sdf::SensorType::MAGNETOMETER);
  data1.SetUpdateRate(12.4);
  data1.SetTopic("grape");
  data1.SetRawPose(math::Pose3d(1, 2, 3, 0, 0, 0));

  sdf::Magnetometer mag1;
  data1.SetMagnetometerSensor(mag1);

  sdf::Sensor data2;

  // Create components
  auto comp11 = components::Magnetometer(data1);
  auto comp12 = components::Magnetometer(data1);
  auto comp2 = components::Magnetometer(data2);

  // Equality operators
  EXPECT_EQ(comp11, comp12);
  EXPECT_NE(comp11, comp2);
  EXPECT_TRUE(comp11 == comp12);
  EXPECT_TRUE(comp11 != comp2);
  EXPECT_FALSE(comp11 == comp2);
  EXPECT_FALSE(comp11 != comp12);

  // Stream operators
  std::ostringstream ostr;
  comp11.Serialize(ostr);
  std::istringstream istr(ostr.str());
  components::Magnetometer comp3;
  comp3.Deserialize(istr);
  EXPECT_EQ("banana", comp3.Data().Name());
  EXPECT_EQ(sdf::SensorType::MAGNETOMETER, comp3.Data().Type());
  EXPECT_EQ("grape", comp3.Data().Topic());
  EXPECT_DOUBLE_EQ(12.4, comp3.Data().UpdateRate());
  EXPECT_EQ(math::Pose3d(1, 2, 3, 0, 0, 0), comp3.Data().RawPose());
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Material)
{
  auto data1 = sdf::Material();
  data1.SetAmbient(math::Color(1, 0, 0, 1));
  data1.SetDiffuse(math::Color(1, 0, 1, 1));
  data1.SetSpecular(math::Color(1, 1, 0, 1));
  data1.SetEmissive(math::Color(1, 1, 1, 1));
  data1.SetLighting(false);

  sdf::Pbr pbr;
  sdf::PbrWorkflow workflow;
  workflow.SetType(sdf::PbrWorkflowType::METAL);
  workflow.SetAlbedoMap("albedo_map.png");
  workflow.SetNormalMap("normal_map.png");
  workflow.SetEnvironmentMap("environment_map.png");
  workflow.SetAmbientOcclusionMap("ambient_occlusion_map.png");
  workflow.SetMetalnessMap("metalness_map.png");
  workflow.SetRoughnessMap("roughness_map.png");
  workflow.SetGlossinessMap("dummy_glossiness_map.png");
  workflow.SetSpecularMap("dummy_specular_map.png");
  workflow.SetMetalness(0.3);
  workflow.SetRoughness(0.9);
  workflow.SetGlossiness(0.1);
  pbr.SetWorkflow(workflow.Type(), workflow);
  data1.SetPbrMaterial(pbr);

  auto data2 = sdf::Material();

  // Create components
  auto comp11 = components::Material(data1);
  auto comp2 = components::Material(data2);

  // TODO(anyone) Equality operators

  // Stream operators
  std::ostringstream ostr;
  comp11.Serialize(ostr);
  std::istringstream istr(ostr.str());
  components::Material comp3;
  comp3.Deserialize(istr);
  EXPECT_EQ(math::Color(1, 0, 0, 1), comp3.Data().Ambient());
  EXPECT_EQ(math::Color(1, 0, 1, 1), comp3.Data().Diffuse());
  EXPECT_EQ(math::Color(1, 1, 0, 1), comp3.Data().Specular());
  EXPECT_EQ(math::Color(1, 1, 1, 1), comp3.Data().Emissive());
  EXPECT_FALSE(comp3.Data().Lighting());

  auto newPbrMaterial = comp3.Data().PbrMaterial();
  ASSERT_NE(nullptr, newPbrMaterial);
  auto newWorkflow = newPbrMaterial->Workflow(sdf::PbrWorkflowType::METAL);
  ASSERT_NE(nullptr, newWorkflow);
  EXPECT_EQ("albedo_map.png", newWorkflow->AlbedoMap());
  EXPECT_EQ("normal_map.png", newWorkflow->NormalMap());
  EXPECT_EQ("roughness_map.png", newWorkflow->RoughnessMap());
  EXPECT_EQ("metalness_map.png", newWorkflow->MetalnessMap());
  EXPECT_EQ("environment_map.png", newWorkflow->EnvironmentMap());
  EXPECT_EQ("ambient_occlusion_map.png", newWorkflow->AmbientOcclusionMap());
  EXPECT_EQ("dummy_glossiness_map.png", newWorkflow->GlossinessMap());
  EXPECT_EQ("dummy_specular_map.png", newWorkflow->SpecularMap());
  EXPECT_DOUBLE_EQ(0.3, newWorkflow->Metalness());
  EXPECT_DOUBLE_EQ(0.9, newWorkflow->Roughness());
  EXPECT_DOUBLE_EQ(0.1, newWorkflow->Glossiness());
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Model)
{
  // Create components
  auto comp1 = components::Model();
  auto comp2 = components::Model();

  // Equality operators
  EXPECT_EQ(comp1, comp2);
  EXPECT_TRUE(comp1 == comp2);
  EXPECT_FALSE(comp1 != comp2);

  // Stream operators
  std::ostringstream ostr;
  comp1.Serialize(ostr);
  EXPECT_EQ("-", ostr.str());

  std::istringstream istr("ignored");
  components::Model comp3;
  comp3.Deserialize(istr);
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, ModelSdf)
{
  std::ostringstream stream;
  std::string version = SDF_VERSION;
  stream
    << "<?xml version=\"1.0\" ?>"
    << "<sdf version='" << version << "'>"
    << "  <world name=\"modelSDF\">"
    << "    <physics name=\"1ms\" type=\"ode\">"
    << "      <max_step_size>0.001</max_step_size>"
    << "      <real_time_factor>1.0</real_time_factor>"
    << "    </physics>"
    << "    <plugin"
    << "      filename=\"gz-sim-physics-system\""
    << "      name=\"gz::sim::systems::Physics\">"
    << "    </plugin>"
    << "    <model name='my_model'>"
    << "      <link name='link'>"
    << "        <light type= 'point' name='my_light'>"
    << "          <pose>0.1 0 0 0 0 0</pose>"
    << "          <diffuse>0.2 0.3 0.4 1</diffuse>"
    << "          <specular>0.3 0.4 0.5 1</specular>"
    << "        </light>"
    << "      </link>"
    << "    </model>"
    << "  </world>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  ASSERT_TRUE(sdf::readString(stream.str(), sdfParsed));

  // model
  EXPECT_TRUE(sdfParsed->Root()->HasElement("world"));
  sdf::ElementPtr worldElem = sdfParsed->Root()->GetElement("world");
  EXPECT_TRUE(worldElem->HasElement("model"));
  sdf::ElementPtr modelElem = worldElem->GetElement("model");
  EXPECT_TRUE(modelElem->HasAttribute("name"));
  EXPECT_EQ(modelElem->Get<std::string>("name"), "my_model");

  sdf::Model model;
  model.Load(modelElem);
  EXPECT_EQ("my_model", model.Name());

  // Create components
  auto comp1 = components::ModelSdf(model);
  components::ModelSdf comp2;

  // Stream operators
  std::ostringstream ostr;
  comp1.Serialize(ostr);

  std::istringstream istr(ostr.str());
  comp2.Deserialize(istr);

  EXPECT_EQ("my_model", comp2.Data().Name());
  EXPECT_EQ(1u, comp2.Data().LinkCount());
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Name)
{
  // Create components
  auto comp11 = components::Name("comp1");
  auto comp12 = components::Name("comp1");
  auto comp2 = components::Name("comp2");

  // Equality operators
  EXPECT_EQ(comp11, comp12);
  EXPECT_NE(comp11, comp2);
  EXPECT_TRUE(comp11 == comp12);
  EXPECT_TRUE(comp11 != comp2);
  EXPECT_FALSE(comp11 == comp2);
  EXPECT_FALSE(comp11 != comp12);

  // Stream operators
  for (auto str : {
      "boring",
      "snake_case",
      "camelCase",
      "with s p a c e s 123",
      "with/slash",
      "tópico",
      "トピック"
    })
  {
    auto compA = components::Name(str);
    std::ostringstream ostr;
    compA.Serialize(ostr);
    EXPECT_EQ(str, ostr.str());

    std::istringstream istr(str);
    components::Name compB;
    compB.Deserialize(istr);
    EXPECT_EQ(str, compB.Data());
  }
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, ParentEntity)
{
  // Create components
  auto comp11 = components::ParentEntity(1);
  auto comp12 = components::ParentEntity(1);
  auto comp2 = components::ParentEntity(2);

  // Equality operators
  EXPECT_EQ(comp11, comp12);
  EXPECT_NE(comp11, comp2);
  EXPECT_TRUE(comp11 == comp12);
  EXPECT_TRUE(comp11 != comp2);
  EXPECT_FALSE(comp11 == comp2);
  EXPECT_FALSE(comp11 != comp12);

  // Stream operators
  std::ostringstream ostr;
  comp11.Serialize(ostr);
  EXPECT_EQ("1", ostr.str());

  std::istringstream istr("3");
  components::ParentEntity comp3(kNullEntity);
  comp3.Deserialize(istr);
  EXPECT_EQ(3u, comp3.Data());
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, ParentLinkName)
{
  // Create components
  auto comp11 = components::ParentLinkName("comp1");
  auto comp12 = components::ParentLinkName("comp1");
  auto comp2 = components::ParentLinkName("comp2");

  // Equality operators
  EXPECT_EQ(comp11, comp12);
  EXPECT_NE(comp11, comp2);
  EXPECT_TRUE(comp11 == comp12);
  EXPECT_TRUE(comp11 != comp2);
  EXPECT_FALSE(comp11 == comp2);
  EXPECT_FALSE(comp11 != comp12);

  // Stream operators
  std::ostringstream ostr;
  comp11.Serialize(ostr);
  EXPECT_EQ("comp1", ostr.str());

  std::istringstream istr("comp3");
  components::ParentLinkName comp3(std::string(""));
  comp3.Deserialize(istr);
  EXPECT_EQ("comp3", comp3.Data());
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Performer)
{
  // Create components
  auto comp1 = components::Performer();
  auto comp2 = components::Performer();

  // Equality operators
  EXPECT_EQ(comp1, comp2);
  EXPECT_TRUE(comp1 == comp2);
  EXPECT_FALSE(comp1 != comp2);

  // Stream operators
  std::ostringstream ostr;
  comp1.Serialize(ostr);
  EXPECT_EQ("-", ostr.str());

  std::istringstream istr("ignored");
  components::Performer comp3;
  comp3.Deserialize(istr);
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, PerformerAffinity)
{
  // Create components
  auto comp11 = components::PerformerAffinity("comp1");
  auto comp12 = components::PerformerAffinity("comp1");
  auto comp2 = components::PerformerAffinity("comp2");

  // Equality operators
  EXPECT_EQ(comp11, comp12);
  EXPECT_NE(comp11, comp2);
  EXPECT_TRUE(comp11 == comp12);
  EXPECT_TRUE(comp11 != comp2);
  EXPECT_FALSE(comp11 == comp2);
  EXPECT_FALSE(comp11 != comp12);

  // Stream operators
  std::ostringstream ostr;
  comp11.Serialize(ostr);
  EXPECT_EQ("comp1", ostr.str());

  std::istringstream istr("comp3");
  components::PerformerAffinity comp3;
  comp3.Deserialize(istr);
  EXPECT_EQ("comp3", comp3.Data());
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, PerformerLevels)
{
  // Create components
  auto comp11 = components::PerformerLevels({1, 2, 3});
  auto comp12 = components::PerformerLevels({1, 2, 3});
  auto comp2 = components::PerformerLevels({4, 5});

  // TODO(anyone) Equality operators

  // Stream operators
  std::ostringstream ostr;
  comp11.Serialize(ostr);
  EXPECT_EQ("1 2 3 ", ostr.str());

  std::istringstream istr("7 8 9 10");
  components::PerformerLevels comp3;
  comp3.Deserialize(istr);
  EXPECT_EQ(4u, comp3.Data().size());
  EXPECT_EQ(1u, comp3.Data().count(7));
  EXPECT_EQ(1u, comp3.Data().count(8));
  EXPECT_EQ(1u, comp3.Data().count(9));
  EXPECT_EQ(1u, comp3.Data().count(10));
  EXPECT_EQ(0u, comp3.Data().count(1));
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, PhysicsEnginePlugin)
{
  // Create components
  auto comp11 = components::PhysicsEnginePlugin("engine-plugin");
  auto comp12 = components::PhysicsEnginePlugin("engine-plugin");
  auto comp2 = components::PhysicsEnginePlugin("another-engine-plugin");

  // TODO(anyone) Equality operators

  // Stream operators
  std::ostringstream ostr;
  comp11.Serialize(ostr);
  EXPECT_EQ("engine-plugin", ostr.str());

  std::istringstream istr("libengine-plugin.so");
  components::PhysicsEnginePlugin comp3;
  comp3.Deserialize(istr);
  EXPECT_EQ("libengine-plugin.so", comp3.Data());
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Pose)
{
  // Create components
  auto comp11 = components::Pose(math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));
  auto comp12 = components::Pose(math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));
  auto comp2 = components::Pose(math::Pose3d(2, 0, 0, 0, 0, 0));

  // Equality operators
  EXPECT_EQ(comp11, comp12);
  EXPECT_NE(comp11, comp2);
  EXPECT_TRUE(comp11 == comp12);
  EXPECT_TRUE(comp11 != comp2);
  EXPECT_FALSE(comp11 == comp2);
  EXPECT_FALSE(comp11 != comp12);

  // Stream operators
  std::ostringstream ostr;
  comp11.Serialize(ostr);
  EXPECT_EQ("1 2 3 0.1 0.2 0.3", ostr.str());

  std::istringstream istr("3 2 1 0.3 0.2 0.1");
  components::Pose comp3(math::Pose3d::Zero);
  comp3.Deserialize(istr);
  EXPECT_EQ(math::Pose3d(3, 2, 1, 0.3, 0.2, 0.1), comp3.Data());
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Sensor)
{
  // Create components
  auto comp1 = components::Sensor();
  auto comp2 = components::Sensor();

  // Equality operators
  EXPECT_EQ(comp1, comp2);
  EXPECT_TRUE(comp1 == comp2);
  EXPECT_FALSE(comp1 != comp2);

  // Stream operators
  std::ostringstream ostr;
  comp1.Serialize(ostr);
  EXPECT_EQ("-", ostr.str());

  std::istringstream istr("ignored");
  components::Sensor comp3;
  comp3.Deserialize(istr);
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, SourceFilePath)
{
  // Create components
  auto comp11 = components::SourceFilePath("comp1");
  auto comp12 = components::SourceFilePath("comp1");
  auto comp2 = components::SourceFilePath("comp2");

  // Equality operators
  EXPECT_EQ(comp11, comp12);
  EXPECT_NE(comp11, comp2);
  EXPECT_TRUE(comp11 == comp12);
  EXPECT_TRUE(comp11 != comp2);
  EXPECT_FALSE(comp11 == comp2);
  EXPECT_FALSE(comp11 != comp12);

  // Stream operators
  std::ostringstream ostr;
  comp11.Serialize(ostr);
  EXPECT_EQ("comp1", ostr.str());

  std::istringstream istr("comp3");
  components::SourceFilePath comp3;
  comp3.Deserialize(istr);
  EXPECT_EQ("comp3", comp3.Data());
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Static)
{
  // Create components
  auto comp11 = components::Static(true);
  auto comp12 = components::Static(true);
  auto comp2 = components::Static(false);

  // Equality operators
  EXPECT_EQ(comp11, comp12);
  EXPECT_NE(comp11, comp2);
  EXPECT_TRUE(comp11 == comp12);
  EXPECT_TRUE(comp11 != comp2);
  EXPECT_FALSE(comp11 == comp2);
  EXPECT_FALSE(comp11 != comp12);
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, TemperatureRange)
{
  // TODO(adlarkin) make sure min can't be >= max?
  components::TemperatureRangeInfo range1;
  range1.min = math::Temperature(125.0);
  range1.max = math::Temperature(300.0);

  components::TemperatureRangeInfo range2;
  range2.min = math::Temperature(140.0);
  range2.max = math::Temperature(200.0);

  components::TemperatureRangeInfo range3;
  range3.min = math::Temperature(125.0);
  range3.max = math::Temperature(300.0);

  // Create components
  auto comp1 = components::TemperatureRange(range1);
  auto comp2 = components::TemperatureRange(range2);
  auto comp3 = components::TemperatureRange(range3);

  // Equality operators
  EXPECT_EQ(comp1, comp3);
  EXPECT_NE(comp1, comp2);
  EXPECT_NE(comp2, comp3);
  EXPECT_FALSE(comp1 == comp2);
  EXPECT_TRUE(comp1 != comp2);

  // Stream operators
  std::ostringstream ostr;
  comp1.Serialize(ostr);
  EXPECT_EQ("125 300", ostr.str());

  std::istringstream istr(ostr.str());
  components::TemperatureRange comp4;
  comp4.Deserialize(istr);
  EXPECT_EQ(comp1, comp4);
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, ThreadPitch)
{
  // Create components
  auto comp11 = components::ThreadPitch(1.2);

  // No double comparisons

  // Stream operators
  std::ostringstream ostr;
  comp11.Serialize(ostr);
  EXPECT_EQ("1.2", ostr.str());

  std::istringstream istr("3.4");
  components::ThreadPitch comp3;
  comp3.Deserialize(istr);
  EXPECT_DOUBLE_EQ(3.4, comp3.Data());
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Visual)
{
  // Create components
  auto comp1 = components::Visual();
  auto comp2 = components::Visual();

  // Equality operators
  EXPECT_EQ(comp1, comp2);
  EXPECT_TRUE(comp1 == comp2);
  EXPECT_FALSE(comp1 != comp2);

  // Stream operators
  std::ostringstream ostr;
  comp1.Serialize(ostr);
  EXPECT_EQ("-", ostr.str());

  std::istringstream istr("ignored");
  components::Visual comp3;
  comp3.Deserialize(istr);
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, World)
{
  // Create components
  auto comp1 = components::World();
  auto comp2 = components::World();

  // Equality operators
  EXPECT_EQ(comp1, comp2);
  EXPECT_TRUE(comp1 == comp2);
  EXPECT_FALSE(comp1 != comp2);

  // Stream operators
  std::ostringstream ostr;
  comp1.Serialize(ostr);
  EXPECT_EQ("-", ostr.str());

  std::istringstream istr("ignored");
  components::World comp3;
  comp3.Deserialize(istr);
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Scene)
{
  auto data1 = sdf::Scene();
  data1.SetAmbient(math::Color(1, 0, 1, 1));
  data1.SetBackground(math::Color(1, 1, 0, 1));
  data1.SetShadows(true);
  data1.SetGrid(false);
  data1.SetOriginVisual(true);

  // Create components
  auto comp11 = components::Scene(data1);

  // TODO(anyone) Equality operators

  // Stream operators
  std::ostringstream ostr;
  comp11.Serialize(ostr);
  std::istringstream istr(ostr.str());
  components::Scene comp3;
  comp3.Deserialize(istr);
  EXPECT_EQ(math::Color(1, 0, 1, 1), comp3.Data().Ambient());
  EXPECT_EQ(math::Color(1, 1, 0, 1), comp3.Data().Background());
  EXPECT_TRUE(comp3.Data().Shadows());
  EXPECT_FALSE(comp3.Data().Grid());
  EXPECT_TRUE(comp3.Data().OriginVisual());
}

//////////////////////////////////////////////////
TEST_F(ComponentsTest, ParticleEmitter)
{
  msgs::ParticleEmitter emitter1;
  emitter1.set_name("emitter1");
  emitter1.set_id(0);
  emitter1.set_type(gz::msgs::ParticleEmitter_EmitterType_BOX);
  emitter1.mutable_size()->set_x(1);
  emitter1.mutable_size()->set_y(2);
  emitter1.mutable_size()->set_z(3);
  emitter1.mutable_rate()->set_data(4.0);
  emitter1.mutable_duration()->set_data(5.0);
  emitter1.mutable_emitting()->set_data(false);
  emitter1.mutable_particle_size()->set_x(0.1);
  emitter1.mutable_particle_size()->set_y(0.2);
  emitter1.mutable_particle_size()->set_z(0.3);
  emitter1.mutable_lifetime()->set_data(6.0);
  emitter1.mutable_min_velocity()->set_data(7.0);
  emitter1.mutable_max_velocity()->set_data(8.0);
  emitter1.mutable_color_start()->set_r(1.0);
  emitter1.mutable_color_start()->set_g(0);
  emitter1.mutable_color_start()->set_b(0);
  emitter1.mutable_color_start()->set_a(0);
  emitter1.mutable_color_end()->set_r(1.0);
  emitter1.mutable_color_end()->set_g(1.0);
  emitter1.mutable_color_end()->set_b(1.0);
  emitter1.mutable_color_end()->set_a(0);
  emitter1.mutable_scale_rate()->set_data(9.0);
  emitter1.mutable_color_range_image()->set_data("path_to_texture");

  msgs::ParticleEmitter emitter2;
  emitter2.set_name("emitter2");
  emitter2.set_id(1);
  emitter2.set_type(gz::msgs::ParticleEmitter_EmitterType_BOX);
  emitter2.mutable_size()->set_x(1);
  emitter2.mutable_size()->set_y(2);
  emitter2.mutable_size()->set_z(3);
  emitter2.mutable_rate()->set_data(4.0);
  emitter2.mutable_duration()->set_data(5.0);
  emitter2.mutable_emitting()->set_data(false);
  emitter2.mutable_particle_size()->set_x(0.1);
  emitter2.mutable_particle_size()->set_y(0.2);
  emitter2.mutable_particle_size()->set_z(0.3);
  emitter2.mutable_lifetime()->set_data(6.0);
  emitter2.mutable_min_velocity()->set_data(7.0);
  emitter2.mutable_max_velocity()->set_data(8.0);
  emitter2.mutable_color_start()->set_r(1.0);
  emitter2.mutable_color_start()->set_g(0);
  emitter2.mutable_color_start()->set_b(0);
  emitter2.mutable_color_start()->set_a(0);
  emitter2.mutable_color_end()->set_r(1.0);
  emitter2.mutable_color_end()->set_g(1.0);
  emitter2.mutable_color_end()->set_b(1.0);
  emitter2.mutable_color_end()->set_a(0);
  emitter2.mutable_scale_rate()->set_data(9.0);
  emitter2.mutable_color_range_image()->set_data("path_to_texture");

  // Create components.
  auto comp1 = components::ParticleEmitter(emitter1);
  auto comp2 = components::ParticleEmitter(emitter2);

  // Stream operators.
  std::ostringstream ostr;
  comp1.Serialize(ostr);

  std::istringstream istr(ostr.str());
  components::ParticleEmitter comp3;
  comp3.Deserialize(istr);
  EXPECT_EQ(comp1.Data().id(), comp3.Data().id());
}

//////////////////////////////////////////////////
TEST_F(ComponentsTest, ParticleEmitterCmd)
{
  msgs::ParticleEmitter emitter1;
  emitter1.set_name("emitter1");
  emitter1.mutable_emitting()->set_data(true);

  // Create components.
  auto comp1 = components::ParticleEmitterCmd(emitter1);

  // Stream operators.
  std::ostringstream ostr;
  comp1.Serialize(ostr);

  std::istringstream istr(ostr.str());
  components::ParticleEmitter comp3;
  comp3.Deserialize(istr);
  EXPECT_EQ(comp1.Data().emitting().data(), comp3.Data().emitting().data());
  EXPECT_EQ(comp1.Data().name(), comp3.Data().name());
}

//////////////////////////////////////////////////
TEST_F(ComponentsTest, Projector)
{
  // Create components
  sdf::Projector projector1;
  projector1.SetName("projector1");
  projector1.SetRawPose(math::Pose3d(0, 3, 4, GZ_PI, 0, 0));
  projector1.SetNearClip(1.5);
  projector1.SetFarClip(10.3);
  projector1.SetHorizontalFov(math::Angle(3.0));
  projector1.SetVisibilityFlags(0xFE);
  projector1.SetTexture("path_to_texture");
  auto comp1 = components::Projector(projector1);

  // stream operators
  std::ostringstream ostr;
  comp1.Serialize(ostr);
  std::istringstream istr(ostr.str());
  components::Projector comp3;
  comp3.Deserialize(istr);
  EXPECT_EQ("projector1", comp3.Data().Name());
  EXPECT_EQ(math::Pose3d(0, 3, 4, GZ_PI, 0, 0), comp3.Data().RawPose());
  EXPECT_DOUBLE_EQ(1.5, comp3.Data().NearClip());
  EXPECT_DOUBLE_EQ(10.3, comp3.Data().FarClip());
  EXPECT_EQ(math::Angle(3.0), comp3.Data().HorizontalFov());
  EXPECT_EQ(0xFE, comp3.Data().VisibilityFlags());
  EXPECT_EQ("path_to_texture", comp3.Data().Texture());
}

//////////////////////////////////////////////////
TEST_F(ComponentsTest, JointTransmittedWrench)
{
  msgs::Wrench wrench;
  msgs::Set(wrench.mutable_torque(), {10, 20, 30});
  msgs::Set(wrench.mutable_force(), {1, 2, 3});

  // // Create components.
  auto comp1 = components::JointTransmittedWrench(wrench);

  // Stream operators.
  std::ostringstream ostr;
  comp1.Serialize(ostr);

  std::istringstream istr(ostr.str());
  components::JointTransmittedWrench comp2;
  comp2.Deserialize(istr);
  EXPECT_EQ(msgs::Convert(comp2.Data().force()), msgs::Convert(wrench.force()));
  EXPECT_EQ(msgs::Convert(comp2.Data().torque()),
            msgs::Convert(wrench.torque()));
}
