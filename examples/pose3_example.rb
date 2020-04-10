# Copyright (C) 2019 Open Source Robotics Foundation
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

# This example will only work if the Ruby interface library was compiled and
# installed.
#
# Modify the RUBYLIB environment variable to include the ignition math
# library install path. For example, if you install to /user:
#
# $ export RUBYLIB=/usr/lib/ruby:$RUBYLIB
#
require 'ignition/math'

# Construct a default Pose3d.
p = Ignition::Math::Pose3d.new
printf("A default Pose3d has the following values\n" +
       "%f %f %f %f %f %f\n", p.Pos().X(), p.Pos().Y(), p.Pos().Z(),
       p.Rot().Euler().X(), p.Rot().Euler().Y(), p.Rot().Euler().Z())

# Construct a pose at position 1, 2, 3 with a yaw of PI radians.
p1 = Ignition::Math::Pose3d.new(1, 2, 3, 0, 0, Math::PI)
printf("A pose3d(1, 2, 3, 0, 0, IGN_PI) has the following values\n" +
       "%f %f %f %f %f %f\n", p1.Pos().X(), p1.Pos().Y(), p1.Pos().Z(),
       p1.Rot().Euler().X(), p1.Rot().Euler().Y(), p1.Rot().Euler().Z())

# Set the position of a pose to 10, 20, 30
p.Pos().Set(10, 20, 30)

p3 = p * p1
printf("Result of combining two poses is\n"+
        "%f %f %f %f %f %f\n", p3.Pos().X(), p3.Pos().Y(), p3.Pos().Z(),
       p3.Rot().Euler().X(), p3.Rot().Euler().Y(), p3.Rot().Euler().Z())
