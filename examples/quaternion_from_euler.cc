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

#include <iostream>
#include <sstream>
#include <ignition/math/Matrix3.hh>
#include <ignition/math/Quaternion.hh>

// Copied from urdfdom
static inline double strToDouble(const char *in)
{
  std::stringstream ss;
  ss.imbue(std::locale::classic());

  ss << in;

  double out;
  ss >> out;

  if (ss.fail() || !ss.eof()) {
    std::cerr << "Trying to convert " << in << std::endl;
    throw std::runtime_error("Failed converting string to double");
  }

  return out;
}

int main(int argc, char **argv)
{
  if (argc != 4)
  {
    std::cerr << "Invalid usage\n\n"
              << "Usage (angles specified in degrees):\n"
              << "  quaternion_from_euler "
              << "<float_roll> <float_pitch> <float_yaw>\n\n"
              << "Example\n"
              << "  quaternion_from_euler 0 0 1.57"
              << std::endl;
    return -1;
  }

  double roll = IGN_DTOR(strToDouble(argv[1]));
  double pitch = IGN_DTOR(strToDouble(argv[2]));
  double yaw = IGN_DTOR(strToDouble(argv[3]));

  std::cout << "Converting Euler angles:\n";
  printf(" roll  % .6f radians\n"
         " pitch % .6f radians\n"
         " yaw   % .6f radians\n\n",
          roll, pitch, yaw);
  printf(" roll  % 12.6f degrees\n"
         " pitch % 12.6f degrees\n"
         " yaw   % 12.6f degrees\n",
          IGN_RTOD(roll),
          IGN_RTOD(pitch),
          IGN_RTOD(yaw));

//![constructor]
  ignition::math::Quaterniond q(roll, pitch, yaw);
  ignition::math::Matrix3d m(q);
//![constructor]

  std::cout << "\nto Quaternion\n";
//! [access quaterion]
  printf(" W % .6f\n X % .6f\n Y % .6f\n Z % .6f\n",
        q.W(), q.X(), q.Y(), q.Z());
//! [access quaterion]

//! [rotation matrix]
  std::cout << "\nto Rotation matrix\n";
  printf("   % .6f  % .6f  % .6f\n"
         "   % .6f  % .6f  % .6f\n"
         "   % .6f  % .6f  % .6f\n",
          m(0, 0), m(0, 1), m(0, 2),
          m(1, 0), m(1, 1), m(1, 2),
          m(2, 0), m(2, 1), m(2, 2));
//! [rotation matrix]
}
