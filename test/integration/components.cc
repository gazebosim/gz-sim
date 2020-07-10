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

#include <sdf/Cylinder.hh>
#include <sdf/Element.hh>
#include <sdf/AirPressure.hh>
#include <sdf/Altimeter.hh>
#include <sdf/Imu.hh>
#include <sdf/Magnetometer.hh>
#include <sdf/Material.hh>
#include <sdf/Noise.hh>
#include <sdf/Pbr.hh>
#include <sdf/Sensor.hh>

#include "ignition/gazebo/components/Actor.hh"
#include "ignition/gazebo/components/AirPressureSensor.hh"
#include "ignition/gazebo/components/Altimeter.hh"
#include "ignition/gazebo/components/AngularVelocity.hh"
#include "ignition/gazebo/components/Camera.hh"
#include "ignition/gazebo/components/CanonicalLink.hh"
#include "ignition/gazebo/components/ChildLinkName.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/DetachableJoint.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Gravity.hh"
#include "ignition/gazebo/components/Imu.hh"
#include "ignition/gazebo/components/Inertial.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/JointAxis.hh"
#include "ignition/gazebo/components/JointType.hh"
#include "ignition/gazebo/components/JointVelocity.hh"
#include "ignition/gazebo/components/JointVelocityCmd.hh"
#include "ignition/gazebo/components/Level.hh"
#include "ignition/gazebo/components/LevelBuffer.hh"
#include "ignition/gazebo/components/LevelEntityNames.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/LinearAcceleration.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Magnetometer.hh"
#include "ignition/gazebo/components/Material.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/ParentLinkName.hh"
#include "ignition/gazebo/components/Performer.hh"
#include "ignition/gazebo/components/PerformerLevels.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Scene.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/Static.hh"
#include "ignition/gazebo/components/ThreadPitch.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/test_config.hh"  // NOLINT(build/include)


using namespace ignition;
using namespace gazebo;

class ComponentsTest : public ::testing::Test
{
  protected: void SetUp() override
  {
    common::Console::SetVerbosity(4);
  }
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
  EXPECT_EQ(ignition::math::Pose3d(3, 2, 1, 0, 0, 0), comp3.Data().RawPose());
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, AirPressureSensor)
{
  sdf::Sensor data1;
  data1.SetName("abc");
  data1.SetType(sdf::SensorType::AIR_PRESSURE);
  data1.SetRawPose(ignition::math::Pose3d(1, 2, 3, 0, 0, 0));

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
  EXPECT_EQ(ignition::math::Pose3d(1, 2, 3, 0, 0, 0), comp3.Data().RawPose());
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
  data1.SetRawPose(ignition::math::Pose3d(1, 2, 3, 0, 0, 0));

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
  EXPECT_EQ(ignition::math::Pose3d(1, 2, 3, 0, 0, 0), comp3.Data().RawPose());
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
  data1.SetXyz(math::Vector3d(1, 2, 3));
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

  EXPECT_EQ(math::Vector3d(1, 2, 3), comp3.Data().Xyz());
  EXPECT_DOUBLE_EQ(0.1, comp3.Data().Damping());
  EXPECT_DOUBLE_EQ(0.2, comp3.Data().Friction());
  EXPECT_DOUBLE_EQ(0.3, comp3.Data().Lower());
  EXPECT_DOUBLE_EQ(0.4, comp3.Data().Upper());
  EXPECT_DOUBLE_EQ(0.5, comp3.Data().Effort());
  EXPECT_DOUBLE_EQ(0.6, comp3.Data().MaxVelocity());
  EXPECT_EQ(comp3.Data().XyzExpressedIn(), "__model__");
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
  EXPECT_EQ("level1 level2 ", ostr.str());

  std::istringstream istr("level3 level4");
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
  data1.SetRawPose(math::Pose3d(1, 2, 4, 0, 0, IGN_PI));
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
  EXPECT_EQ(math::Pose3d(1, 2, 4, 0, 0, IGN_PI), comp3.Data().RawPose());
  EXPECT_EQ(math::Color(1, 0, 0, 1), comp3.Data().Diffuse());
  EXPECT_EQ(math::Color(0, 1, 0, 1), comp3.Data().Specular());
  EXPECT_TRUE(comp3.Data().CastShadows());
  EXPECT_FLOAT_EQ(1.3, comp3.Data().AttenuationRange());
  EXPECT_FLOAT_EQ(0.3, comp3.Data().LinearAttenuationFactor());
  EXPECT_FLOAT_EQ(0.1, comp3.Data().QuadraticAttenuationFactor());
  EXPECT_FLOAT_EQ(0.05, comp3.Data().ConstantAttenuationFactor());
  EXPECT_EQ(math::Angle(0.3), comp3.Data().SpotInnerAngle());
  EXPECT_EQ(math::Angle(2.3), comp3.Data().SpotOuterAngle());
  EXPECT_FLOAT_EQ(5.15, comp3.Data().SpotFalloff());
  EXPECT_EQ(math::Vector3d(2, 3, 4), comp3.Data().Direction());
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

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Magnetometer)
{
  sdf::Sensor data1;
  data1.SetName("banana");
  data1.SetType(sdf::SensorType::MAGNETOMETER);
  data1.SetUpdateRate(12.4);
  data1.SetTopic("grape");
  data1.SetRawPose(ignition::math::Pose3d(1, 2, 3, 0, 0, 0));

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
  EXPECT_EQ(ignition::math::Pose3d(1, 2, 3, 0, 0, 0), comp3.Data().RawPose());
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

  sdf::Pbr *newPbrMaterial = comp3.Data().PbrMaterial();
  ASSERT_NE(nullptr, newPbrMaterial);
  sdf::PbrWorkflow *newWorkflow =
      newPbrMaterial->Workflow(sdf::PbrWorkflowType::METAL);
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
  std::ostringstream ostr;
  comp11.Serialize(ostr);
  EXPECT_EQ("comp1", ostr.str());

  std::istringstream istr("comp3");
  components::Name comp3;
  comp3.Deserialize(istr);
  EXPECT_EQ("comp3", comp3.Data());
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
