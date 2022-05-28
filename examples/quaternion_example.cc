/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
//! [complete]
#include <stdio.h>
#include <gz/math/Quaternion.hh>

int main(int argc, char **argv)
{
  // Construct a default quaternion.
  gz::math::Quaterniond q;
  printf("A default quaternion has the following values\n" \
       "\tW=%f X=%f Y=%f Z=%f\n", q.W(), q.X(), q.Y(), q.Z());

  // Set the quaternion to [1, 0, 0, 0], the identity.
  q = gz::math::Quaterniond::Identity;
  printf("The identity quaternion has the following values\n" \
       "\tW=%f X=%f Y=%f Z=%f\n", q.W(), q.X(), q.Y(), q.Z());

  // Create a new quaternion using Euler angles.
  gz::math::Quaterniond q2(0, 0, 3.14);
  printf("A quaternion initialized from roll=0, pitch=0, and yaw=3.14 has"\
         "the following values\n" \
         "\tW=%f X=%f Y=%f Z=%f\n", q2.W(), q2.X(), q2.Y(), q2.Z());

  // Get the Euler angles back from the quaternion.
  gz::math::Vector3d euler = q2.Euler();
  printf("Getting back the euler angles from the quaternion\n"\
         "\troll=%f pitch=%f yaw=%f\n", euler.X(), euler.Y(), euler.Z());
}
//! [complete]
