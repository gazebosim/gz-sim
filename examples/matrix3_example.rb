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

# Construct a default matrix3.
m = Ignition::Math::Matrix3d.new
printf("The default constructed matrix m has the following values.\n\t" +
       "%2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f\n",
       m.(0, 0), m.(0, 1), m.(0, 2),
       m.(1, 0), m.(1, 1), m.(1, 2),
       m.(2, 0), m.(2, 1), m.(2, 2))

# Set the first column of the matrix.
m.SetCol(0, Ignition::Math::Vector3d.new(3, 4, 5))
printf("Setting the first column of the matrix m to 3, 4, 5.\n\t" +
       "%2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f\n",
       m.(0, 0), m.(0, 1), m.(0, 2),
       m.(1, 0), m.(1, 1), m.(1, 2),
       m.(2, 0), m.(2, 1), m.(2, 2))

# Transpose the matrix.
t = m.Transposed()
printf("The transposed matrix t has the values.\n\t"+
       "%2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f\n",
       t.(0, 0), t.(0, 1), t.(0, 2),
       t.(1, 0), t.(1, 1), t.(1, 2),
       t.(2, 0), t.(2, 1), t.(2, 2))

# Multiply the two matrices.
m = m * t
printf("m * t = " +
       "%2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f %2.1f\n",
       m.(0, 0), m.(0, 1), m.(0, 2),
       m.(1, 0), m.(1, 1), m.(1, 2),
       m.(2, 0), m.(2, 1), m.(2, 2))
