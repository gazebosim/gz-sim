/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#include "gazebo/math/Vector3Stats.hh"
#include "test/util.hh"

using namespace gazebo;

class Vector3StatsTest : public gazebo::testing::AutoLogFixture
{
  /// \brief Get X value of statistic _name.
  public: double X(const std::string &_name) const;

  /// \brief Get Y value of statistic _name.
  public: double Y(const std::string &_name) const;

  /// \brief Get Z value of statistic _name.
  public: double Z(const std::string &_name) const;

  /// \brief Get Mag value of statistic _name.
  public: double Mag(const std::string &_name) const;

  /// \brief Stats instance.
  public: math::Vector3Stats stats;
};

//////////////////////////////////////////////////
double Vector3StatsTest::X(const std::string &_name) const
{
  return this->stats.X().Map()[_name];
}

//////////////////////////////////////////////////
double Vector3StatsTest::Y(const std::string &_name) const
{
  return this->stats.Y().Map()[_name];
}

//////////////////////////////////////////////////
double Vector3StatsTest::Z(const std::string &_name) const
{
  return this->stats.Z().Map()[_name];
}

//////////////////////////////////////////////////
double Vector3StatsTest::Mag(const std::string &_name) const
{
  return this->stats.Mag().Map()[_name];
}

//////////////////////////////////////////////////
TEST_F(Vector3StatsTest, Vector3Stats)
{
  {
    // Constructor
    math::Vector3Stats v3stats;
    EXPECT_TRUE(v3stats.X().Map().empty());
    EXPECT_TRUE(v3stats.Y().Map().empty());
    EXPECT_TRUE(v3stats.Z().Map().empty());
    EXPECT_TRUE(v3stats.Mag().Map().empty());
    EXPECT_EQ(v3stats.X().Count(), 0u);
    EXPECT_EQ(v3stats.Y().Count(), 0u);
    EXPECT_EQ(v3stats.Z().Count(), 0u);
    EXPECT_EQ(v3stats.Mag().Count(), 0u);

    // Reset
    v3stats.Reset();
    EXPECT_TRUE(v3stats.X().Map().empty());
    EXPECT_TRUE(v3stats.Y().Map().empty());
    EXPECT_TRUE(v3stats.Z().Map().empty());
    EXPECT_TRUE(v3stats.Mag().Map().empty());
    EXPECT_EQ(v3stats.X().Count(), 0u);
    EXPECT_EQ(v3stats.Y().Count(), 0u);
    EXPECT_EQ(v3stats.Z().Count(), 0u);
    EXPECT_EQ(v3stats.Mag().Count(), 0u);
  }

  {
    // InsertStatistics
    math::Vector3Stats v3stats;
    EXPECT_TRUE(v3stats.X().Map().empty());
    EXPECT_TRUE(v3stats.Y().Map().empty());
    EXPECT_TRUE(v3stats.Z().Map().empty());
    EXPECT_TRUE(v3stats.Mag().Map().empty());

    EXPECT_TRUE(v3stats.InsertStatistics("maxAbs"));
    EXPECT_FALSE(v3stats.InsertStatistics("maxAbs"));
    EXPECT_FALSE(v3stats.InsertStatistic("maxAbs"));
    EXPECT_FALSE(v3stats.X().Map().empty());
    EXPECT_FALSE(v3stats.Y().Map().empty());
    EXPECT_FALSE(v3stats.Z().Map().empty());
    EXPECT_FALSE(v3stats.Mag().Map().empty());

    // Map with no data
    {
      std::map<std::string, double> map = v3stats.X().Map();
      EXPECT_EQ(map.size(), 1u);
      EXPECT_EQ(map.count("maxAbs"), 1u);
    }
    {
      std::map<std::string, double> map = v3stats.Y().Map();
      EXPECT_EQ(map.size(), 1u);
      EXPECT_EQ(map.count("maxAbs"), 1u);
    }
    {
      std::map<std::string, double> map = v3stats.Z().Map();
      EXPECT_EQ(map.size(), 1u);
      EXPECT_EQ(map.count("maxAbs"), 1u);
    }
    {
      std::map<std::string, double> map = v3stats.Mag().Map();
      EXPECT_EQ(map.size(), 1u);
      EXPECT_EQ(map.count("maxAbs"), 1u);
    }

    // Insert some data
    EXPECT_EQ(v3stats.X().Count(), 0u);
    EXPECT_EQ(v3stats.Y().Count(), 0u);
    EXPECT_EQ(v3stats.Z().Count(), 0u);
    EXPECT_EQ(v3stats.Mag().Count(), 0u);

    v3stats.InsertData(math::Vector3::UnitX);
    v3stats.InsertData(math::Vector3::UnitX);
    v3stats.InsertData(math::Vector3::UnitY);

    EXPECT_EQ(v3stats.X().Count(), 3u);
    EXPECT_EQ(v3stats.Y().Count(), 3u);
    EXPECT_EQ(v3stats.Z().Count(), 3u);
    EXPECT_EQ(v3stats.Mag().Count(), 3u);

    EXPECT_NEAR(v3stats.X().Map()["maxAbs"], 1.0, 1e-10);
    EXPECT_NEAR(v3stats.Y().Map()["maxAbs"], 1.0, 1e-10);
    EXPECT_DOUBLE_EQ(v3stats.Z().Map()["maxAbs"], 0.0);
    EXPECT_NEAR(v3stats.Mag().Map()["maxAbs"], 1.0, 1e-10);
  }

  // Const accessors
  {
    EXPECT_TRUE(this->stats.X().Map().empty());
    EXPECT_TRUE(this->stats.Y().Map().empty());
    EXPECT_TRUE(this->stats.Z().Map().empty());
    EXPECT_TRUE(this->stats.Mag().Map().empty());

    const std::string name("maxAbs");
    EXPECT_TRUE(this->stats.InsertStatistics(name));

    this->stats.InsertData(math::Vector3::UnitX);
    this->stats.InsertData(math::Vector3::UnitX);
    this->stats.InsertData(math::Vector3::UnitY);

    EXPECT_EQ(this->stats.X().Count(), 3u);
    EXPECT_EQ(this->stats.Y().Count(), 3u);
    EXPECT_EQ(this->stats.Z().Count(), 3u);
    EXPECT_EQ(this->stats.Mag().Count(), 3u);

    EXPECT_NEAR(this->stats.X().Map()[name], 1.0, 1e-10);
    EXPECT_NEAR(this->stats.Y().Map()[name], 1.0, 1e-10);
    EXPECT_DOUBLE_EQ(this->stats.Z().Map()[name], 0.0);
    EXPECT_NEAR(this->stats.Mag().Map()[name], 1.0, 1e-10);

    EXPECT_NEAR(this->X(name), 1.0, 1e-10);
    EXPECT_NEAR(this->Y(name), 1.0, 1e-10);
    EXPECT_DOUBLE_EQ(this->Z(name), 0.0);
    EXPECT_NEAR(this->Mag(name), 1.0, 1e-10);
  }
}

