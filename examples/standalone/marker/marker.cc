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

#include <ignition/transport.hh>
#include <ignition/math.hh>
#include <ignition/msgs.hh>

#include <iostream>

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  ignition::transport::Node node;

  // Create the marker message
  ignition::msgs::Marker markerMsg;
  ignition::msgs::Material matMsg;
  markerMsg.set_ns("default");
  markerMsg.set_id(0);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::SPHERE);
  markerMsg.set_visibility(ignition::msgs::Marker::GUI);

  // Set color to Blue
  markerMsg.mutable_material()->mutable_ambient()->set_r(0);
  markerMsg.mutable_material()->mutable_ambient()->set_g(0);
  markerMsg.mutable_material()->mutable_ambient()->set_b(1);
  markerMsg.mutable_material()->mutable_ambient()->set_a(1);
  markerMsg.mutable_material()->mutable_diffuse()->set_r(0);
  markerMsg.mutable_material()->mutable_diffuse()->set_g(0);
  markerMsg.mutable_material()->mutable_diffuse()->set_b(1);
  markerMsg.mutable_material()->mutable_diffuse()->set_a(1);
  markerMsg.mutable_lifetime()->set_sec(2);
  markerMsg.mutable_lifetime()->set_nsec(0);
  ignition::msgs::Set(markerMsg.mutable_scale(),
                    ignition::math::Vector3d(1.0, 1.0, 1.0));

  // The rest of this function adds different shapes and/or modifies shapes.
  // Read the terminal statements to figure out what each node.Request
  // call accomplishes.
  std::cout << "Spawning a blue sphere with lifetime 2s\n";
  std::this_thread::sleep_for(std::chrono::seconds(4));
  ignition::msgs::Set(markerMsg.mutable_pose(),
                      ignition::math::Pose3d(2, 2, 0, 0, 0, 0));
  node.Request("/marker", markerMsg);
  std::cout << "Sleeping for 2 seconds\n";
  std::this_thread::sleep_for(std::chrono::seconds(2));

  std::cout << "Spawning a black sphere at the origin with no lifetime\n";
  std::this_thread::sleep_for(std::chrono::seconds(4));
  markerMsg.set_id(1);
  ignition::msgs::Set(markerMsg.mutable_pose(),
                      ignition::math::Pose3d::Zero);
  markerMsg.mutable_material()->mutable_ambient()->set_b(0);
  markerMsg.mutable_material()->mutable_diffuse()->set_b(0);
  markerMsg.mutable_lifetime()->set_sec(0);
  node.Request("/marker", markerMsg);

  std::cout << "Moving the sphere to x=0, y=1, z=1\n";
  std::this_thread::sleep_for(std::chrono::seconds(4));
  ignition::msgs::Set(markerMsg.mutable_pose(),
                      ignition::math::Pose3d(0, 1, 1, 0, 0, 0));
  node.Request("/marker", markerMsg);

  std::cout << "Shrinking the sphere\n";
  std::this_thread::sleep_for(std::chrono::seconds(4));
  ignition::msgs::Set(markerMsg.mutable_scale(),
                    ignition::math::Vector3d(0.2, 0.2, 0.2));
  node.Request("/marker", markerMsg);

  std::cout << "Changing the sphere to red\n";
  markerMsg.mutable_material()->mutable_ambient()->set_r(1);
  markerMsg.mutable_material()->mutable_ambient()->set_g(0);
  markerMsg.mutable_material()->mutable_ambient()->set_b(0);
  markerMsg.mutable_material()->mutable_diffuse()->set_r(1);
  markerMsg.mutable_material()->mutable_diffuse()->set_g(0);
  markerMsg.mutable_material()->mutable_diffuse()->set_b(0);
  std::this_thread::sleep_for(std::chrono::seconds(4));
  node.Request("/marker", markerMsg);

  std::cout << "Adding a green box\n";
  markerMsg.mutable_material()->mutable_ambient()->set_r(0);
  markerMsg.mutable_material()->mutable_ambient()->set_g(1);
  markerMsg.mutable_material()->mutable_ambient()->set_b(0);
  markerMsg.mutable_material()->mutable_diffuse()->set_r(0);
  markerMsg.mutable_material()->mutable_diffuse()->set_g(1);
  markerMsg.mutable_material()->mutable_diffuse()->set_b(0);
  std::this_thread::sleep_for(std::chrono::seconds(4));
  markerMsg.set_id(2);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::BOX);
  ignition::msgs::Set(markerMsg.mutable_scale(),
                    ignition::math::Vector3d(1.0, 1.0, 1.0));
  ignition::msgs::Set(markerMsg.mutable_pose(),
                    ignition::math::Pose3d(2, 0, .5, 0, 0, 0));
  node.Request("/marker", markerMsg);

  std::cout << "Changing the green box to a cylinder\n";
  std::this_thread::sleep_for(std::chrono::seconds(4));
  markerMsg.set_type(ignition::msgs::Marker::CYLINDER);
  node.Request("/marker", markerMsg);

  std::cout << "Connecting the sphere and cylinder with a line\n";
  std::this_thread::sleep_for(std::chrono::seconds(4));
  markerMsg.set_id(3);
  ignition::msgs::Set(markerMsg.mutable_pose(),
                    ignition::math::Pose3d(0, 0, 0, 0, 0, 0));
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::LINE_LIST);
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(0.0, 1.0, 1.0));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(2, 0, 0.5));
  node.Request("/marker", markerMsg);

  std::cout << "Adding a square around the origin\n";
  std::this_thread::sleep_for(std::chrono::seconds(4));
  markerMsg.set_id(4);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::LINE_STRIP);
  ignition::msgs::Set(markerMsg.mutable_point(0),
      ignition::math::Vector3d(0.5, 0.5, 0.05));
  ignition::msgs::Set(markerMsg.mutable_point(1),
      ignition::math::Vector3d(0.5, -0.5, 0.05));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(-0.5, -0.5, 0.05));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(-0.5, 0.5, 0.05));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(0.5, 0.5, 0.05));
  node.Request("/marker", markerMsg);

  std::cout << "Adding 100 points inside the square\n";
  std::this_thread::sleep_for(std::chrono::seconds(4));
  markerMsg.set_id(5);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::POINTS);
  markerMsg.clear_point();
  for (int i = 0; i < 100; ++i)
  {
    ignition::msgs::Set(markerMsg.add_point(),
        ignition::math::Vector3d(
          ignition::math::Rand::DblUniform(-0.49, 0.49),
          ignition::math::Rand::DblUniform(-0.49, 0.49),
          0.05));
  }
  node.Request("/marker", markerMsg);

  std::cout << "Adding a semi-circular triangle fan\n";
  std::this_thread::sleep_for(std::chrono::seconds(4));
  markerMsg.set_id(6);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_FAN);
  markerMsg.clear_point();
  ignition::msgs::Set(markerMsg.mutable_pose(),
                    ignition::math::Pose3d(0, 1.5, 0, 0, 0, 0));
  ignition::msgs::Set(markerMsg.add_point(),
        ignition::math::Vector3d(0, 0, 0.05));
  double radius = 2;
  for (double t = 0; t <= M_PI; t+= 0.01)
  {
    ignition::msgs::Set(markerMsg.add_point(),
        ignition::math::Vector3d(radius * cos(t), radius * sin(t), 0.05));
  }
  node.Request("/marker", markerMsg);

  std::cout << "Adding two triangles using a triangle list\n";
  std::this_thread::sleep_for(std::chrono::seconds(4));
  markerMsg.set_id(7);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_LIST);
  markerMsg.clear_point();
  ignition::msgs::Set(markerMsg.mutable_pose(),
                    ignition::math::Pose3d(0, -1.5, 0, 0, 0, 0));
  ignition::msgs::Set(markerMsg.add_point(),
        ignition::math::Vector3d(0, 0, 0.05));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(1, 0, 0.05));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(1, 1, 0.05));

  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(1, 1, 0.05));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(2, 1, 0.05));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(2, 2, 0.05));

  node.Request("/marker", markerMsg);

  std::cout << "Adding a rectangular triangle strip\n";
  std::this_thread::sleep_for(std::chrono::seconds(4));
  markerMsg.set_id(8);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_STRIP);
  markerMsg.clear_point();
  ignition::msgs::Set(markerMsg.mutable_pose(),
                    ignition::math::Pose3d(-2, -2, 0, 0, 0, 0));
  ignition::msgs::Set(markerMsg.add_point(),
        ignition::math::Vector3d(0, 0, 0.05));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(1, 0, 0.05));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(0, 1, 0.05));

  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(1, 1, 0.05));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(0, 2, 0.05));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(1, 2, 0.05));

  node.Request("/marker", markerMsg);

  std::cout << "Deleting all the markers\n";
  std::this_thread::sleep_for(std::chrono::seconds(4));
  markerMsg.set_action(ignition::msgs::Marker::DELETE_ALL);
  node.Request("/marker", markerMsg);
}
