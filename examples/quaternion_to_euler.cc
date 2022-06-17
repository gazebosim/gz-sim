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
#include <gz/math/Angle.hh>
#include <gz/math/Matrix3.hh>
#include <gz/math/Quaternion.hh>

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
  if (argc != 5)
  {
    std::cerr << "Invalid usage\n\n"
              << "Usage:\n"
              << "  quaternion_to_euler "
              << "<float_w> <float_x> <float_y> <float_z>\n\n"
              << "Example\n"
              << "  quaternion_to_euler 0.5 0.5 0.5 0.5"
              << std::endl;
    return -1;
  }

  double w = strToDouble(argv[1]);
  double x = strToDouble(argv[2]);
  double y = strToDouble(argv[3]);
  double z = strToDouble(argv[4]);

  std::cout << "Normalizing Quaternion components:"
            << "\n  W " << w
            << "\n  X " << x
            << "\n  Y " << y
            << "\n  Z " << z
            << std::endl;
  gz::math::Quaterniond q(w, x, y, z);
  q.Normalize();
  std::cout << "to"
            << "\n  W " << q.W()
            << "\n  X " << q.X()
            << "\n  Y " << q.Y()
            << "\n  Z " << q.Z()
            << std::endl;

  gz::math::Matrix3d m(q);
//![constructor]
  gz::math::Vector3d euler(q.Euler());
//![constructor]

  std::cout << "\nConverting to Euler angles\n";
  printf(" roll  % .6f radians\n"
         " pitch % .6f radians\n"
         " yaw   % .6f radians\n\n",
          euler.X(), euler.Y(), euler.Z());
  printf(" roll  % .6f degrees\n"
         " pitch % .6f degrees\n"
         " yaw   % .6f degrees\n",
          GZ_RTOD(euler.X()),
          GZ_RTOD(euler.Y()),
          GZ_RTOD(euler.Z()));

  std::cout << "\nto Rotation matrix\n";
  printf("   % .6f  % .6f  % .6f\n"
         "   % .6f  % .6f  % .6f\n"
         "   % .6f  % .6f  % .6f\n",
          m(0, 0), m(0, 1), m(0, 2),
          m(1, 0), m(1, 1), m(1, 2),
          m(2, 0), m(2, 1), m(2, 2));
}
