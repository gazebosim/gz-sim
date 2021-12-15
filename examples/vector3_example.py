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
# library install path. For example, if you install to /user:
#
# $ export PYTHONPATH=/usr/lib/python:$PYTHONPATH
#
import ignition.math

v1 = ignition.math.Vector3d(0, 0, 3)
print("v =: {} {} {}\n".format(v1.x(), v1.y(), v1.z()))

v2 = ignition.math.Vector3d(4, 0, 0)
print("v2 = {} {} {}\n".format(v2.x(), v2.y(), v2.z()))

v3 = v1 + v2
print("v1 + v2 = {} {} {}\n".format(v3.x(), v3.y(), v3.z()))

print("v1.Distance(v2) = {}\n".format(v1.distance(v2)))
