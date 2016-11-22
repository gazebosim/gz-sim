# Copyright (C) 2012-2014 Open Source Robotics Foundation
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#!/usr/bin/env ruby

require 'test/unit/ui/console/testrunner'
require 'test/unit'
require 'math'

class Angle_TEST < Test::Unit::TestCase
  def test_construction
    angle1 = Ignition::Math::Angle.new;

    
    assert(angle1.Radian() == 0.0,
          "Angle::Radian() should equal zero")
  
    # angle1.Degree(180.0);
    # EXPECT_TRUE(angle1 == IGN_PI);
  
    # EXPECT_FALSE(angle1 == IGN_PI + 0.1);
    # EXPECT_TRUE(angle1 == IGN_PI + 0.0001);
    # EXPECT_TRUE(angle1 == IGN_PI - 0.0001);
    # EXPECT_TRUE(math::Angle(0) == math::Angle(0));
    # EXPECT_TRUE(math::Angle(0) == math::Angle(0.001));
  
    # angle1 = math::Angle(0.1) - math::Angle(0.3);
    # EXPECT_TRUE(angle1 == -0.2);
  
    # math::Angle angle(0.5);
    # EXPECT_TRUE(math::equal(0.5, angle.Radian()));
  
    # angle.Radian(IGN_PI);
    # EXPECT_TRUE(math::equal(IGN_RTOD(IGN_PI), angle.Degree()));
  
    # angle.Normalize();
    # EXPECT_TRUE(math::equal(IGN_RTOD(IGN_PI), angle.Degree()));
  
    # angle = math::Angle(0.1) + math::Angle(0.2);
    # EXPECT_TRUE(math::equal(0.3, angle.Radian()));
  
    # angle = math::Angle(0.1) * math::Angle(0.2);
    # EXPECT_TRUE(math::equal(0.02, angle.Radian()));
  
    # angle = math::Angle(0.1) / math::Angle(0.2);
    # EXPECT_TRUE(math::equal(0.5, angle.Radian()));
  
    # angle -= math::Angle(0.1);
    # EXPECT_TRUE(math::equal(0.4, angle.Radian()));
  
    # angle += math::Angle(0.2);
    # EXPECT_TRUE(math::equal(0.6, angle.Radian()));
  
    # angle *= math::Angle(0.5);
    # EXPECT_TRUE(math::equal(0.3, angle.Radian()));
  
    # angle /= math::Angle(0.1);
    # EXPECT_TRUE(math::equal(3.0, angle.Radian()));
    # EXPECT_TRUE(angle == math::Angle(3));
    # EXPECT_TRUE(angle != math::Angle(2));
    # EXPECT_TRUE(angle < math::Angle(4));
    # EXPECT_TRUE(angle > math::Angle(2));
    # EXPECT_TRUE(angle >= math::Angle(3));
    # EXPECT_TRUE(angle <= math::Angle(3));
  
    # angle = 1.2;
    # EXPECT_TRUE(angle <= 1.21);
    # EXPECT_FALSE(angle <= 1.19);
    # EXPECT_TRUE(angle <= 1.2);
    # EXPECT_FALSE(angle <= -1.19);
  
    # EXPECT_TRUE(math::Angle(1.2) <= math::Angle(1.2000000001));
    # EXPECT_TRUE(math::Angle(1.2000000001) <= math::Angle(1.2));
  
    # angle = 1.2;
    # EXPECT_FALSE(angle >= 1.21);
    # EXPECT_TRUE(angle >= 1.19);
    # EXPECT_TRUE(angle >= 1.2);
    # EXPECT_TRUE(angle >= -1.19);
  
    # EXPECT_TRUE(math::Angle(1.2) >= math::Angle(1.2000000001));
    # EXPECT_TRUE(math::Angle(1.2000000001) >= math::Angle(1.2));
  end
end
