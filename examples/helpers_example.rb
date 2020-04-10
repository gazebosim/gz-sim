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

printf("The volume of a sphere with r=2 is %f.\n", IGN_SPHERE_VOLUME(2))

printf("The volume of a cylinder with r=4 and l=5 is %f.\n",
       IGN_CYLINDER_VOLUME(4, 5))

printf("The volume of a box with x=1, y=2, and z=3 is %f.\n",
   IGN_BOX_VOLUME(1, 2, 3))

printf("The result of clamping 2.4 to the range [1,2] is %f.\n",
  Ignition::Math::Clamp(2.4, 1, 2))

std::vector v{1, 2, 3, 4, 5};
printf("The mean of a vector containing {1, 2, 3, 4, 5} is %f.\n",
 Ignition::Math::mean(v))

printf("The variance of a vector containing {1, 2, 3, 4, 5} is %f.\n",
  Ignition::Math::variance(v))

printf("The result of rounding up 3 to the next power of two is %f.\n",
    << ignition::math::roundUpPowerOfTwo(3) << std::endl;


