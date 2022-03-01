# Copyright (C) 2021 Open Source Robotics Foundation
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

# This example will only work if the Python interface library was compiled and
# installed.
#
# Modify the PYTHONPATH environment variable to include the ignition math
# library install path. For example, if you install to /usr:
#
# $ export PYTHONPATH=/usr/lib/python:$PYTHONPATH
#

import ignition.math

print("PI in degrees = {}\n".format(ignition.math.Angle.PI.degree()))

a1 = ignition.math.Angle(1.5707)
a2 = ignition.math.Angle(0.7854)
print("a1 = {} radians, {} degrees\n".format(a1.radian(), a1.degree()))
print("a2 = {} radians, {} degrees\n".format(a2.radian(), a2.degree()))
print("a1 * a2 = {} radians, {} degrees\n".format((a1 * a2).radian(),
      (a1 * a2).degree()))
print("a1 + a2 = {} radians, {} degrees\n".format((a1 + a2).radian(),
      (a1 + a2).degree()))
print("a1 - a2 = {} radians, {} degrees\n".format((a1 - a2).radian(),
      (a1 - a2).degree()))

a3 = ignition.math.Angle(15.707)
print("a3 = {} radians, {} degrees\n".format(a3.radian(), a3.degree()))
a3.normalize()
print("a3.Normalize = {} radians, {} degrees\n".format(a3.radian(),
                                                       a3.degree()))
