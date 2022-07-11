/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

//This is a sample code showing how an Gazebo simulator executable works
#include <iostream>
#include <ignition/msgs.hh>
int main()
{
  ignition::msgs::Vector3d point1;
  point1.set_x(std::rand());
  point1.set_y(std::rand());
  point1.set_z(std::rand());
  ignition::msgs::Vector3d point2;
  point2.set_x(std::rand());
  point2.set_y(std::rand());
  point2.set_z(std::rand());
  std::cout << "Random_point1:\n" << point1.DebugString() << std::endl;
  std::cout << "Random_point2:\n" << point2.DebugString() << std::endl;
  return 0;
}