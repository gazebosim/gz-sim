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

#include "ignition/gazebo/components/Altimeter.hh"
#include "ignition/gazebo/components/AngularVelocity.hh"
#include "ignition/gazebo/components/Camera.hh"
#include "ignition/gazebo/components/CanonicalLink.hh"
#include "ignition/gazebo/components/ChildLinkName.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Gravity.hh"
#include "ignition/gazebo/components/Imu.hh"
#include "ignition/gazebo/components/Inertial.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/JointAxis.hh"
#include "ignition/gazebo/components/JointType.hh"
#include "ignition/gazebo/components/JointVelocity.hh"
#include "ignition/gazebo/components/Level.hh"
#include "ignition/gazebo/components/LevelBuffer.hh"
#include "ignition/gazebo/components/LevelEntityNames.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/LinearAcceleration.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Material.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/ParentLinkName.hh"
#include "ignition/gazebo/components/Performer.hh"
#include "ignition/gazebo/components/Pose.hh"
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
TEST_F(ComponentsTest, Altimeter)
{
  auto data1 = std::make_shared<sdf::Element>();
  auto data2 = std::make_shared<sdf::Element>();

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

  // TODO(anyone) Stream operator
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
  ostr << comp11;
  EXPECT_EQ("1 2 3", ostr.str());

  std::istringstream istr("3 2 1");
  components::AngularVelocity comp3(math::Vector3d::Zero);
  istr >> comp3;
  EXPECT_EQ(math::Vector3d(3, 2, 1), comp3.Data());
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Camera)
{
  auto data1 = std::make_shared<sdf::Element>();
  auto data2 = std::make_shared<sdf::Element>();

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
  ostr << comp1;
  EXPECT_TRUE(ostr.str().empty());

  std::istringstream istr("ignored");
  components::CanonicalLink comp3;
  istr >> comp3;
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
  ostr << comp11;
  EXPECT_EQ("comp1", ostr.str());

  std::istringstream istr("comp3");
  components::ChildLinkName comp3;
  istr >> comp3;
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
  ostr << comp1;
  EXPECT_TRUE(ostr.str().empty());

  std::istringstream istr("ignored");
  components::Collision comp3;
  istr >> comp3;
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
  ostr << comp11;
  std::istringstream istr(ostr.str());
  components::Geometry comp3;
  istr >> comp3;
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
  ostr << comp11;
  EXPECT_EQ("1 2 3", ostr.str());

  std::istringstream istr("3 2 1");
  components::Gravity comp3(math::Vector3d::Zero);
  istr >> comp3;
  EXPECT_EQ(math::Vector3d(3, 2, 1), comp3.Data());
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Imu)
{
  auto data1 = std::make_shared<sdf::Element>();
  auto data2 = std::make_shared<sdf::Element>();

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

  // TODO(anyone) Stream operator
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
  ostr << comp11;
  std::istringstream istr(ostr.str());
  components::Inertial comp3;
  istr >> comp3;
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
  ostr << comp1;
  EXPECT_TRUE(ostr.str().empty());

  std::istringstream istr("ignored");
  components::CanonicalLink comp3;
  istr >> comp3;
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, JointAxis)
{
  auto data1 = sdf::JointAxis();
  auto data2 = sdf::JointAxis();

  // Create components
  auto comp11 = components::JointAxis(data1);
  auto comp12 = components::JointAxis(data1);
  auto comp2 = components::JointAxis(data2);

  // TODO(anyone) Stream operator
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, JointType)
{
  auto data1 = sdf::JointType();
  auto data2 = sdf::JointType();

  // Create components
  auto comp11 = components::JointType(data1);
  auto comp12 = components::JointType(data1);
  auto comp2 = components::JointType(data2);

  // TODO(anyone) Stream operator
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, JointVelocity)
{
  // Create components
  auto comp11 = components::JointVelocity(1.2);

  // No double comparisons

  // Stream operators
  std::ostringstream ostr;
  ostr << comp11;
  EXPECT_EQ("1.2", ostr.str());

  std::istringstream istr("3.4");
  components::JointVelocity comp3;
  istr >> comp3;
  EXPECT_DOUBLE_EQ(3.4, comp3.Data());
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
  ostr << comp1;
  EXPECT_TRUE(ostr.str().empty());

  std::istringstream istr("ignored");
  components::Level comp3;
  istr >> comp3;
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, LevelBuffer)
{
  // Create components
  auto comp11 = components::LevelBuffer(1.5);

  // No double comparisons

  // Stream operators
  std::ostringstream ostr;
  ostr << comp11;
  EXPECT_EQ("1.5", ostr.str());

  std::istringstream istr("3.3");
  components::LevelBuffer comp3;
  istr >> comp3;
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
  ostr << comp11;
  EXPECT_EQ("level1 level2 ", ostr.str());

  std::istringstream istr("level3 level4");
  components::LevelEntityNames comp3;
  istr >> comp3;

  std::set<std::string> data3({"level3", "level4"});
  EXPECT_EQ(data3, comp3.Data());
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Light)
{
  auto data1 = sdf::Light();
  auto data2 = sdf::Light();

  // Create components
  auto comp11 = components::Light(data1);
  auto comp12 = components::Light(data1);
  auto comp2 = components::Light(data2);

  // TODO(anyone) Stream operator
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
  ostr << comp11;
  EXPECT_EQ("1 2 3", ostr.str());

  std::istringstream istr("3 2 1");
  components::LinearAcceleration comp3(math::Vector3d::Zero);
  istr >> comp3;
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
  ostr << comp11;
  EXPECT_EQ("1 2 3", ostr.str());

  std::istringstream istr("3 2 1");
  components::LinearVelocity comp3(math::Vector3d::Zero);
  istr >> comp3;
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
  ostr << comp1;
  EXPECT_TRUE(ostr.str().empty());

  std::istringstream istr("ignored");
  components::Link comp3;
  istr >> comp3;
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Material)
{
  auto data1 = sdf::Material();
  data1.SetAmbient(math::Color(1, 0, 0, 1));
  auto data2 = sdf::Material();

  // Create components
  auto comp11 = components::Material(data1);
  auto comp12 = components::Material(data1);
  auto comp2 = components::Material(data2);

  // Stream operators
  std::ostringstream ostr;
  ostr << comp11;
  std::istringstream istr(ostr.str());
  components::Material comp3;
  istr >> comp3;
  EXPECT_EQ(math::Color(1, 0, 0, 1), comp3.Data().Ambient());
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
  ostr << comp1;
  EXPECT_TRUE(ostr.str().empty());

  std::istringstream istr("ignored");
  components::Model comp3;
  istr >> comp3;
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
  ostr << comp11;
  EXPECT_EQ("comp1", ostr.str());

  std::istringstream istr("comp3");
  components::Name comp3;
  istr >> comp3;
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
  ostr << comp11;
  EXPECT_EQ("1", ostr.str());

  std::istringstream istr("3");
  components::ParentEntity comp3(kNullEntity);
  istr >> comp3;
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
  ostr << comp11;
  EXPECT_EQ("comp1", ostr.str());

  std::istringstream istr("comp3");
  components::ParentLinkName comp3(std::string(""));
  istr >> comp3;
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
  ostr << comp1;
  EXPECT_TRUE(ostr.str().empty());

  std::istringstream istr("ignored");
  components::Performer comp3;
  istr >> comp3;
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
  ostr << comp11;
  EXPECT_EQ("1 2 3 0.1 0.2 0.3", ostr.str());

  std::istringstream istr("3 2 1 0.3 0.2 0.1");
  components::Pose comp3(math::Pose3d::Zero);
  istr >> comp3;
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
  ostr << comp1;
  EXPECT_TRUE(ostr.str().empty());

  std::istringstream istr("ignored");
  components::Sensor comp3;
  istr >> comp3;
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
  ostr << comp11;
  EXPECT_EQ("1.2", ostr.str());

  std::istringstream istr("3.4");
  components::ThreadPitch comp3;
  istr >> comp3;
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
  ostr << comp1;
  EXPECT_TRUE(ostr.str().empty());

  std::istringstream istr("ignored");
  components::Visual comp3;
  istr >> comp3;
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
  ostr << comp1;
  EXPECT_TRUE(ostr.str().empty());

  std::istringstream istr("ignored");
  components::World comp3;
  istr >> comp3;
}

