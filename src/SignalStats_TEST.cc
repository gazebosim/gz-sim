/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include <ignition/math/Rand.hh>
#include <ignition/math/SignalStats.hh>

using namespace ignition;

//////////////////////////////////////////////////
TEST(SignalStatsTest, SignalMaximumConstructor)
{
  // Constructor
  math::SignalMaximum max;
  EXPECT_DOUBLE_EQ(max.Value(), 0.0);
  EXPECT_EQ(max.Count(), 0u);
  EXPECT_EQ(max.ShortName(), std::string("max"));

  // Reset
  max.Reset();
  EXPECT_DOUBLE_EQ(max.Value(), 0.0);
  EXPECT_EQ(max.Count(), 0u);
}

//////////////////////////////////////////////////
TEST(SignalStatsTest, SignalMaximumConstantValues)
{
  // Constant values, max should match
  math::SignalMaximum max;
  EXPECT_DOUBLE_EQ(max.Value(), 0.0);
  EXPECT_EQ(max.Count(), 0u);

  const double value = 3.14159;

  // Loop two times to verify Reset
  for (int j = 0; j < 2; ++j)
  {
    for (unsigned int i = 1; i <= 10; ++i)
    {
      max.InsertData(value);
      EXPECT_DOUBLE_EQ(max.Value(), value);
      EXPECT_EQ(max.Count(), i);
    }

    // Reset
    max.Reset();
    EXPECT_DOUBLE_EQ(max.Value(), 0.0);
    EXPECT_EQ(max.Count(), 0u);
  }
}

//////////////////////////////////////////////////
TEST(SignalStatsTest, SignalMaximumAlternatingValues)
{
  // Values with alternating sign, increasing magnitude
  // Should always match positive value
  math::SignalMaximum max;
  EXPECT_DOUBLE_EQ(max.Value(), 0.0);
  EXPECT_EQ(max.Count(), 0u);

  const double value = 3.14159;

  // Loop two times to verify Reset
  for (int j = 0; j < 2; ++j)
  {
    for (unsigned int i = 1; i <= 10; ++i)
    {
      max.InsertData(value * i);
      EXPECT_DOUBLE_EQ(max.Value(), value * i);
      max.InsertData(-value * i);
      EXPECT_DOUBLE_EQ(max.Value(), value * i);
      EXPECT_EQ(max.Count(), i*2);
    }

    // Reset
    max.Reset();
    EXPECT_DOUBLE_EQ(max.Value(), 0.0);
    EXPECT_EQ(max.Count(), 0u);
  }
}

//////////////////////////////////////////////////
TEST(SignalStatsTest, SignalMean)
{
  {
    // Constructor
    math::SignalMean mean;
    EXPECT_DOUBLE_EQ(mean.Value(), 0.0);
    EXPECT_EQ(mean.Count(), 0u);
    EXPECT_EQ(mean.ShortName(), std::string("mean"));

    // Reset
    mean.Reset();
    EXPECT_DOUBLE_EQ(mean.Value(), 0.0);
    EXPECT_EQ(mean.Count(), 0u);
  }

  {
    // Constant values, mean should match
    math::SignalMean mean;
    EXPECT_DOUBLE_EQ(mean.Value(), 0.0);
    EXPECT_EQ(mean.Count(), 0u);

    const double value = 3.14159;

    // Loop two times to verify Reset
    for (int j = 0; j < 2; ++j)
    {
      for (unsigned int i = 1; i <= 10; ++i)
      {
        mean.InsertData(value);
        EXPECT_DOUBLE_EQ(mean.Value(), value);
        EXPECT_EQ(mean.Count(), i);
      }

      // Reset
      mean.Reset();
      EXPECT_DOUBLE_EQ(mean.Value(), 0.0);
      EXPECT_EQ(mean.Count(), 0u);
    }
  }

  {
    // Values with alternating sign, increasing magnitude
    // Should be zero every other time
    math::SignalMean mean;
    EXPECT_DOUBLE_EQ(mean.Value(), 0.0);
    EXPECT_EQ(mean.Count(), 0u);

    const double value = 3.14159;

    // Loop two times to verify Reset
    for (int j = 0; j < 2; ++j)
    {
      for (unsigned int i = 1; i <= 10; ++i)
      {
        mean.InsertData(value * i);
        mean.InsertData(-value * i);
        EXPECT_DOUBLE_EQ(mean.Value(), 0.0);
        EXPECT_EQ(mean.Count(), i*2);
      }

      // Reset
      mean.Reset();
      EXPECT_DOUBLE_EQ(mean.Value(), 0.0);
      EXPECT_EQ(mean.Count(), 0u);
    }
  }
}

//////////////////////////////////////////////////
TEST(SignalStatsTest, SignalMinimumConstructor)
{
  // Constructor
  math::SignalMinimum min;
  EXPECT_DOUBLE_EQ(min.Value(), 0.0);
  EXPECT_EQ(min.Count(), 0u);
  EXPECT_EQ(min.ShortName(), std::string("min"));

  // Reset
  min.Reset();
  EXPECT_DOUBLE_EQ(min.Value(), 0.0);
  EXPECT_EQ(min.Count(), 0u);
}

//////////////////////////////////////////////////
TEST(SignalStatsTest, SignalMinimumConstantValues)
{
  // Constant values, min should match
  math::SignalMinimum min;
  EXPECT_DOUBLE_EQ(min.Value(), 0.0);
  EXPECT_EQ(min.Count(), 0u);

  const double value = 3.14159;

  // Loop two times to verify Reset
  for (int j = 0; j < 2; ++j)
  {
    for (unsigned int i = 1; i <= 10; ++i)
    {
      min.InsertData(value);
      EXPECT_DOUBLE_EQ(min.Value(), value);
      EXPECT_EQ(min.Count(), i);
    }

    // Reset
    min.Reset();
    EXPECT_DOUBLE_EQ(min.Value(), 0.0);
    EXPECT_EQ(min.Count(), 0u);
  }
}

//////////////////////////////////////////////////
TEST(SignalStatsTest, SignalMinimumAlternatingValues)
{
  // Values with alternating sign, increasing magnitude
  // Should always match negative value
  math::SignalMinimum min;
  EXPECT_DOUBLE_EQ(min.Value(), 0.0);
  EXPECT_EQ(min.Count(), 0u);

  const double value = 3.14159;

  // Loop two times to verify Reset
  for (int j = 0; j < 2; ++j)
  {
    for (unsigned int i = 1; i <= 10; ++i)
    {
      min.InsertData(value * i);
      min.InsertData(-value * i);
      EXPECT_DOUBLE_EQ(min.Value(), -value * i);
      EXPECT_EQ(min.Count(), i*2);
    }

    // Reset
    min.Reset();
    EXPECT_DOUBLE_EQ(min.Value(), 0.0);
    EXPECT_EQ(min.Count(), 0u);
  }
}

//////////////////////////////////////////////////
TEST(SignalStatsTest, SignalRootMeanSquare)
{
  {
    // Constructor
    math::SignalRootMeanSquare rms;
    EXPECT_DOUBLE_EQ(rms.Value(), 0.0);
    EXPECT_EQ(rms.Count(), 0u);
    EXPECT_EQ(rms.ShortName(), std::string("rms"));

    // Reset
    rms.Reset();
    EXPECT_DOUBLE_EQ(rms.Value(), 0.0);
    EXPECT_EQ(rms.Count(), 0u);
  }

  {
    // Constant values, rms should match
    math::SignalRootMeanSquare rms;
    EXPECT_DOUBLE_EQ(rms.Value(), 0.0);
    EXPECT_EQ(rms.Count(), 0u);

    const double value = 3.14159;

    // Loop two times to verify Reset
    for (int j = 0; j < 2; ++j)
    {
      for (unsigned int i = 1; i <= 10; ++i)
      {
        rms.InsertData(value);
        EXPECT_DOUBLE_EQ(rms.Value(), value);
        EXPECT_EQ(rms.Count(), i);
      }

      // Reset
      rms.Reset();
      EXPECT_DOUBLE_EQ(rms.Value(), 0.0);
      EXPECT_EQ(rms.Count(), 0u);
    }
  }

  {
    // Values with alternating sign, same magnitude
    // rms should match absolute value every time
    math::SignalRootMeanSquare rms;
    EXPECT_DOUBLE_EQ(rms.Value(), 0.0);
    EXPECT_EQ(rms.Count(), 0u);

    const double value = 3.14159;

    // Loop two times to verify Reset
    for (int j = 0; j < 2; ++j)
    {
      for (unsigned int i = 1; i <= 10; ++i)
      {
        rms.InsertData(value);
        EXPECT_DOUBLE_EQ(rms.Value(), value);
        EXPECT_EQ(rms.Count(), i*2-1);

        rms.InsertData(-value);
        EXPECT_DOUBLE_EQ(rms.Value(), value);
        EXPECT_EQ(rms.Count(), i*2);
      }

      // Reset
      rms.Reset();
      EXPECT_DOUBLE_EQ(rms.Value(), 0.0);
      EXPECT_EQ(rms.Count(), 0u);
    }
  }
}

//////////////////////////////////////////////////
TEST(SignalStatsTest, SignalMaxAbsoluteValue)
{
  {
    // Constructor
    math::SignalMaxAbsoluteValue max;
    EXPECT_DOUBLE_EQ(max.Value(), 0.0);
    EXPECT_EQ(max.Count(), 0u);
    EXPECT_EQ(max.ShortName(), std::string("maxAbs"));

    // Reset
    max.Reset();
    EXPECT_DOUBLE_EQ(max.Value(), 0.0);
    EXPECT_EQ(max.Count(), 0u);
  }

  {
    // Constant values, max should match
    math::SignalMaxAbsoluteValue max;
    EXPECT_DOUBLE_EQ(max.Value(), 0.0);
    EXPECT_EQ(max.Count(), 0u);

    const double value = 3.14159;

    // Loop two times to verify Reset
    for (int j = 0; j < 2; ++j)
    {
      for (unsigned int i = 1; i <= 10; ++i)
      {
        max.InsertData(value);
        EXPECT_DOUBLE_EQ(max.Value(), value);
        EXPECT_EQ(max.Count(), i);
      }

      // Reset
      max.Reset();
      EXPECT_DOUBLE_EQ(max.Value(), 0.0);
      EXPECT_EQ(max.Count(), 0u);
    }
  }

  {
    // Values with alternating sign, increasing magnitude
    // max should match absolute value every time
    math::SignalMaxAbsoluteValue max;
    EXPECT_DOUBLE_EQ(max.Value(), 0.0);
    EXPECT_EQ(max.Count(), 0u);

    const double value = 3.14159;

    // Loop two times to verify Reset
    for (int j = 0; j < 2; ++j)
    {
      for (unsigned int i = 1; i <= 10; ++i)
      {
        max.InsertData(value * i);
        EXPECT_DOUBLE_EQ(max.Value(), value * i);
        EXPECT_EQ(max.Count(), i*2-1);

        max.InsertData(-value * i);
        EXPECT_DOUBLE_EQ(max.Value(), value * i);
        EXPECT_EQ(max.Count(), i*2);
      }

      // Reset
      max.Reset();
      EXPECT_DOUBLE_EQ(max.Value(), 0.0);
      EXPECT_EQ(max.Count(), 0u);
    }
  }
}

//////////////////////////////////////////////////
TEST(SignalStatsTest, SignalVarianceConstructor)
{
  // Constructor
  math::SignalVariance var;
  EXPECT_DOUBLE_EQ(var.Value(), 0.0);
  EXPECT_EQ(var.Count(), 0u);
  EXPECT_EQ(var.ShortName(), std::string("var"));

  // Reset
  var.Reset();
  EXPECT_DOUBLE_EQ(var.Value(), 0.0);
  EXPECT_EQ(var.Count(), 0u);
}

//////////////////////////////////////////////////
TEST(SignalStatsTest, SignalVarianceOneValue)
{
  // Add one value, expect 0.0 variance
  std::vector<double> values = {0, 1.0, 10.0, -100.0};
  for (auto value : values)
  {
    math::SignalVariance var;
    var.InsertData(value);
    EXPECT_EQ(var.Count(), 1u);
    EXPECT_DOUBLE_EQ(0.0, var.Value());

    // Reset
    var.Reset();
    EXPECT_DOUBLE_EQ(0.0, var.Value());
    EXPECT_EQ(var.Count(), 0u);
  }
}

//////////////////////////////////////////////////
TEST(SignalStatsTest, SignalVarianceConstantValues)
{
  // Constant values, expect 0.0 variance
  math::SignalVariance var;
  const double value = 3.14159;

  // Loop two times to verify Reset
  for (int j = 0; j < 2; ++j)
  {
    for (unsigned int i = 1; i <= 10; ++i)
    {
      var.InsertData(value);
      EXPECT_DOUBLE_EQ(0.0, var.Value());
      EXPECT_EQ(var.Count(), i);
    }

    // Reset
    var.Reset();
    EXPECT_DOUBLE_EQ(var.Value(), 0.0);
    EXPECT_EQ(var.Count(), 0u);
  }
}

//////////////////////////////////////////////////
TEST(SignalStatsTest, SignalVarianceRandomValues)
{
  // Random normally distributed values
  // The sample variance has the following variance:
  // 2 variance^2 / (count - 1)
  // en.wikipedia.org/wiki/Variance#Distribution_of_the_sample_variance
  // We will use 5 sigma (4e-5 chance of failure)
  math::SignalVariance var;
  const double stdDev = 3.14159;
  const int count = 10000;
  const double sigma = 5.0;
  for (int i = 0; i < count; ++i)
  {
    var.InsertData(math::Rand::DblNormal(0.0, stdDev));
  }
  const double variance = stdDev*stdDev;
  double sampleVariance2 = 2 * variance*variance / (count - 1);
  EXPECT_NEAR(var.Value(), variance, sigma*sqrt(sampleVariance2));
  std::cout << "True variance " << variance
            << ", measured variance " << var.Value()
            << ", sigma " << sqrt(sampleVariance2)
            << std::endl;

  // Reset
  var.Reset();
  EXPECT_DOUBLE_EQ(var.Value(), 0.0);
  EXPECT_EQ(var.Count(), 0u);
}

//////////////////////////////////////////////////
TEST(SignalStatsTest, SignalStats)
{
  {
    // Constructor
    math::SignalStats stats;
    EXPECT_TRUE(stats.Map().empty());
    EXPECT_EQ(stats.Count(), 0u);

    math::SignalStats stats2 = stats;
    EXPECT_EQ(stats.Count(), stats2.Count());

    // Reset
    stats.Reset();
    EXPECT_TRUE(stats.Map().empty());
    EXPECT_EQ(stats.Count(), 0u);
  }

  {
    // InsertStatistic
    math::SignalStats stats;
    EXPECT_TRUE(stats.Map().empty());

    EXPECT_TRUE(stats.InsertStatistic("max"));
    EXPECT_FALSE(stats.InsertStatistic("max"));
    EXPECT_FALSE(stats.Map().empty());

    EXPECT_TRUE(stats.InsertStatistic("maxAbs"));
    EXPECT_FALSE(stats.InsertStatistic("maxAbs"));
    EXPECT_FALSE(stats.Map().empty());

    EXPECT_TRUE(stats.InsertStatistic("mean"));
    EXPECT_FALSE(stats.InsertStatistic("mean"));
    EXPECT_FALSE(stats.Map().empty());

    EXPECT_TRUE(stats.InsertStatistic("min"));
    EXPECT_FALSE(stats.InsertStatistic("min"));
    EXPECT_FALSE(stats.Map().empty());

    EXPECT_TRUE(stats.InsertStatistic("rms"));
    EXPECT_FALSE(stats.InsertStatistic("rms"));
    EXPECT_FALSE(stats.Map().empty());

    EXPECT_TRUE(stats.InsertStatistic("var"));
    EXPECT_FALSE(stats.InsertStatistic("var"));
    EXPECT_FALSE(stats.Map().empty());

    EXPECT_FALSE(stats.InsertStatistic("FakeStatistic"));

    // Map with no data
    std::map<std::string, double> map = stats.Map();
    EXPECT_FALSE(map.empty());
    EXPECT_EQ(map.size(), 6u);
    EXPECT_EQ(map.count("max"), 1u);
    EXPECT_EQ(map.count("maxAbs"), 1u);
    EXPECT_EQ(map.count("mean"), 1u);
    EXPECT_EQ(map.count("min"), 1u);
    EXPECT_EQ(map.count("rms"), 1u);
    EXPECT_EQ(map.count("var"), 1u);
    EXPECT_EQ(map.count("FakeStatistic"), 0u);

    math::SignalStats stats2 = stats;
    std::map<std::string, double> map2 = stats2.Map();
    EXPECT_FALSE(map2.empty());
    EXPECT_EQ(map.size(), map2.size());
    EXPECT_EQ(map.count("max"), map2.count("max"));
    EXPECT_EQ(map.count("maxAbs"), map2.count("maxAbs"));
    EXPECT_EQ(map.count("mean"), map2.count("mean"));
    EXPECT_EQ(map.count("min"), map2.count("min"));
    EXPECT_EQ(map.count("rms"), map2.count("rms"));
    EXPECT_EQ(map.count("var"), map2.count("var"));
    EXPECT_EQ(map.count("FakeStatistic"), map2.count("FakeStatistic"));
  }

  {
    // InsertStatistics
    math::SignalStats stats;
    EXPECT_FALSE(stats.InsertStatistics(""));
    EXPECT_TRUE(stats.Map().empty());

    EXPECT_TRUE(stats.InsertStatistics("maxAbs,rms"));
    EXPECT_EQ(stats.Map().size(), 2u);
    EXPECT_FALSE(stats.InsertStatistics("maxAbs,rms"));
    EXPECT_FALSE(stats.InsertStatistics("maxAbs"));
    EXPECT_FALSE(stats.InsertStatistics("rms"));
    EXPECT_EQ(stats.Map().size(), 2u);

    EXPECT_FALSE(stats.InsertStatistics("mean,FakeStatistic"));
    EXPECT_EQ(stats.Map().size(), 3u);

    EXPECT_FALSE(stats.InsertStatistics("var,FakeStatistic"));
    EXPECT_EQ(stats.Map().size(), 4u);

    EXPECT_FALSE(stats.InsertStatistics("max,FakeStatistic"));
    EXPECT_EQ(stats.Map().size(), 5u);

    EXPECT_FALSE(stats.InsertStatistics("min,FakeStatistic"));
    EXPECT_EQ(stats.Map().size(), 6u);

    EXPECT_FALSE(stats.InsertStatistics("FakeStatistic"));
    EXPECT_EQ(stats.Map().size(), 6u);

    // Map with no data
    std::map<std::string, double> map = stats.Map();
    EXPECT_FALSE(map.empty());
    EXPECT_EQ(map.size(), 6u);
    EXPECT_EQ(map.count("max"), 1u);
    EXPECT_EQ(map.count("maxAbs"), 1u);
    EXPECT_EQ(map.count("mean"), 1u);
    EXPECT_EQ(map.count("min"), 1u);
    EXPECT_EQ(map.count("rms"), 1u);
    EXPECT_EQ(map.count("var"), 1u);
    EXPECT_EQ(map.count("FakeStatistic"), 0u);
  }

  {
    // Add some statistics
    math::SignalStats stats;
    EXPECT_TRUE(stats.InsertStatistics("max,maxAbs,mean,min,rms"));
    EXPECT_EQ(stats.Map().size(), 5u);

    // No data yet
    EXPECT_EQ(stats.Count(), 0u);

    // Insert data with alternating signs
    const double value = 3.14159;
    stats.InsertData(value);
    stats.InsertData(-value);
    EXPECT_EQ(stats.Count(), 2u);

    {
      std::map<std::string, double> map = stats.Map();
      EXPECT_DOUBLE_EQ(map["max"], value);
      EXPECT_DOUBLE_EQ(map["maxAbs"], value);
      EXPECT_DOUBLE_EQ(map["min"], -value);
      EXPECT_DOUBLE_EQ(map["rms"], value);
      EXPECT_DOUBLE_EQ(map["mean"], 0.0);
    }

    stats.Reset();
    EXPECT_EQ(stats.Map().size(), 5u);
    EXPECT_EQ(stats.Count(), 0u);
    {
      std::map<std::string, double> map = stats.Map();
      EXPECT_DOUBLE_EQ(map["max"], 0.0);
      EXPECT_DOUBLE_EQ(map["maxAbs"], 0.0);
      EXPECT_DOUBLE_EQ(map["min"], 0.0);
      EXPECT_DOUBLE_EQ(map["rms"], 0.0);
      EXPECT_DOUBLE_EQ(map["mean"], 0.0);
    }
  }
}

