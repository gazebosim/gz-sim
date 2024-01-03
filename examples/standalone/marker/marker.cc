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

#include <gz/transport.hh>
#include <gz/math.hh>
#include <gz/msgs.hh>
#include <gz/common/Util.hh>

#include <iostream>

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  gz::transport::Node node;

  // Create the marker message
  gz::msgs::Marker markerMsg;
  gz::msgs::Material matMsg;
  markerMsg.set_ns("default");
  markerMsg.set_id(0);
  markerMsg.set_action(gz::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(gz::msgs::Marker::SPHERE);
  markerMsg.set_visibility(gz::msgs::Marker::GUI);

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
  gz::msgs::Set(markerMsg.mutable_scale(),
                    gz::math::Vector3d(1.0, 1.0, 1.0));

  // The rest of this function adds different shapes and/or modifies shapes.
  // Read the terminal statements to figure out what each node.Request
  // call accomplishes.
  std::cout << "Spawning a blue sphere with lifetime 2s\n";
  GZ_SLEEP_S(4);
  gz::msgs::Set(markerMsg.mutable_pose(),
                      gz::math::Pose3d(2, 2, 0, 0, 0, 0));
  node.Request("/marker", markerMsg);
  std::cout << "Sleeping for 2 seconds\n";
  GZ_SLEEP_S(2);

  std::cout << "Spawning a black sphere at the origin with no lifetime\n";
  GZ_SLEEP_S(4);
  markerMsg.set_id(1);
  gz::msgs::Set(markerMsg.mutable_pose(),
                      gz::math::Pose3d::Zero);
  markerMsg.mutable_material()->mutable_ambient()->set_b(0);
  markerMsg.mutable_material()->mutable_diffuse()->set_b(0);
  markerMsg.mutable_lifetime()->set_sec(0);
  node.Request("/marker", markerMsg);

  std::cout << "Moving the sphere to x=0, y=1, z=1\n";
  GZ_SLEEP_S(4);
  gz::msgs::Set(markerMsg.mutable_pose(),
                      gz::math::Pose3d(0, 1, 1, 0, 0, 0));
  node.Request("/marker", markerMsg);

  std::cout << "Shrinking the sphere\n";
  GZ_SLEEP_S(4);
  gz::msgs::Set(markerMsg.mutable_scale(),
                    gz::math::Vector3d(0.2, 0.2, 0.2));
  node.Request("/marker", markerMsg);

  std::cout << "Changing the sphere to red\n";
  markerMsg.mutable_material()->mutable_ambient()->set_r(1);
  markerMsg.mutable_material()->mutable_ambient()->set_g(0);
  markerMsg.mutable_material()->mutable_ambient()->set_b(0);
  markerMsg.mutable_material()->mutable_diffuse()->set_r(1);
  markerMsg.mutable_material()->mutable_diffuse()->set_g(0);
  markerMsg.mutable_material()->mutable_diffuse()->set_b(0);
  GZ_SLEEP_S(4);
  node.Request("/marker", markerMsg);

  std::cout << "Adding a green box\n";
  markerMsg.mutable_material()->mutable_ambient()->set_r(0);
  markerMsg.mutable_material()->mutable_ambient()->set_g(1);
  markerMsg.mutable_material()->mutable_ambient()->set_b(0);
  markerMsg.mutable_material()->mutable_diffuse()->set_r(0);
  markerMsg.mutable_material()->mutable_diffuse()->set_g(1);
  markerMsg.mutable_material()->mutable_diffuse()->set_b(0);
  GZ_SLEEP_S(4);
  markerMsg.set_id(2);
  markerMsg.set_action(gz::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(gz::msgs::Marker::BOX);
  gz::msgs::Set(markerMsg.mutable_scale(),
                    gz::math::Vector3d(1.0, 1.0, 1.0));
  gz::msgs::Set(markerMsg.mutable_pose(),
                    gz::math::Pose3d(2, 0, .5, 0, 0, 0));
  node.Request("/marker", markerMsg);

  std::cout << "Changing the green box to a cylinder\n";
  GZ_SLEEP_S(4);
  markerMsg.set_type(gz::msgs::Marker::CYLINDER);
  node.Request("/marker", markerMsg);

  std::cout << "Connecting the sphere and cylinder with a line\n";
  GZ_SLEEP_S(4);
  markerMsg.set_id(3);
  gz::msgs::Set(markerMsg.mutable_pose(),
                    gz::math::Pose3d(0, 0, 0, 0, 0, 0));
  markerMsg.set_action(gz::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(gz::msgs::Marker::LINE_LIST);
  gz::msgs::Set(markerMsg.add_point(),
      gz::math::Vector3d(0.0, 1.0, 1.0));
  gz::msgs::Set(markerMsg.add_point(),
      gz::math::Vector3d(2, 0, 0.5));
  node.Request("/marker", markerMsg);

  std::cout << "Adding a square around the origin\n";
  GZ_SLEEP_S(4);
  markerMsg.set_id(4);
  markerMsg.set_action(gz::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(gz::msgs::Marker::LINE_STRIP);
  gz::msgs::Set(markerMsg.mutable_point(0),
      gz::math::Vector3d(0.5, 0.5, 0.05));
  gz::msgs::Set(markerMsg.mutable_point(1),
      gz::math::Vector3d(0.5, -0.5, 0.05));
  gz::msgs::Set(markerMsg.add_point(),
      gz::math::Vector3d(-0.5, -0.5, 0.05));
  gz::msgs::Set(markerMsg.add_point(),
      gz::math::Vector3d(-0.5, 0.5, 0.05));
  gz::msgs::Set(markerMsg.add_point(),
      gz::math::Vector3d(0.5, 0.5, 0.05));
  node.Request("/marker", markerMsg);

  std::cout << "Adding 100 points inside the square\n";
  GZ_SLEEP_S(4);
  markerMsg.set_id(5);
  markerMsg.set_action(gz::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(gz::msgs::Marker::POINTS);
  markerMsg.clear_point();
  for (int i = 0; i < 100; ++i)
  {
    gz::msgs::Set(markerMsg.add_point(),
        gz::math::Vector3d(
          gz::math::Rand::DblUniform(-0.49, 0.49),
          gz::math::Rand::DblUniform(-0.49, 0.49),
          0.05));
  }
  node.Request("/marker", markerMsg);

  std::cout << "Adding a semi-circular triangle fan\n";
  GZ_SLEEP_S(4);
  markerMsg.set_id(6);
  markerMsg.set_action(gz::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(gz::msgs::Marker::TRIANGLE_FAN);
  markerMsg.clear_point();
  gz::msgs::Set(markerMsg.mutable_pose(),
                    gz::math::Pose3d(0, 1.5, 0, 0, 0, 0));
  gz::msgs::Set(markerMsg.add_point(),
        gz::math::Vector3d(0, 0, 0.05));
  double radius = 2;
  for (double t = 0; t <= GZ_PI; t+= 0.01)
  {
    gz::msgs::Set(markerMsg.add_point(),
        gz::math::Vector3d(radius * cos(t), radius * sin(t), 0.05));
  }
  node.Request("/marker", markerMsg);

  std::cout << "Adding two triangles using a triangle list\n";
  GZ_SLEEP_S(4);
  markerMsg.set_id(7);
  markerMsg.set_action(gz::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(gz::msgs::Marker::TRIANGLE_LIST);
  markerMsg.clear_point();
  gz::msgs::Set(markerMsg.mutable_pose(),
                    gz::math::Pose3d(0, -1.5, 0, 0, 0, 0));
  gz::msgs::Set(markerMsg.add_point(),
        gz::math::Vector3d(0, 0, 0.05));
  gz::msgs::Set(markerMsg.add_point(),
      gz::math::Vector3d(1, 0, 0.05));
  gz::msgs::Set(markerMsg.add_point(),
      gz::math::Vector3d(1, 1, 0.05));

  gz::msgs::Set(markerMsg.add_point(),
      gz::math::Vector3d(1, 1, 0.05));
  gz::msgs::Set(markerMsg.add_point(),
      gz::math::Vector3d(2, 1, 0.05));
  gz::msgs::Set(markerMsg.add_point(),
      gz::math::Vector3d(2, 2, 0.05));

  node.Request("/marker", markerMsg);

  std::cout << "Adding a rectangular triangle strip\n";
  GZ_SLEEP_S(4);
  markerMsg.set_id(8);
  markerMsg.set_action(gz::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(gz::msgs::Marker::TRIANGLE_STRIP);
  markerMsg.clear_point();
  gz::msgs::Set(markerMsg.mutable_pose(),
                    gz::math::Pose3d(-2, -2, 0, 0, 0, 0));
  gz::msgs::Set(markerMsg.add_point(),
        gz::math::Vector3d(0, 0, 0.05));
  gz::msgs::Set(markerMsg.add_point(),
      gz::math::Vector3d(1, 0, 0.05));
  gz::msgs::Set(markerMsg.add_point(),
      gz::math::Vector3d(0, 1, 0.05));

  gz::msgs::Set(markerMsg.add_point(),
      gz::math::Vector3d(1, 1, 0.05));
  gz::msgs::Set(markerMsg.add_point(),
      gz::math::Vector3d(0, 2, 0.05));
  gz::msgs::Set(markerMsg.add_point(),
      gz::math::Vector3d(1, 2, 0.05));

  node.Request("/marker", markerMsg);
  std::cout << "Adding multiple markers via /marker_array\n";
  GZ_SLEEP_S(4);

  gz::msgs::Marker_V markerMsgs;
  gz::msgs::Boolean res;
  bool result;
  unsigned int timeout = 5000;

  // Create first blue sphere marker
  auto markerMsg1 = markerMsgs.add_marker();
  markerMsg1->set_ns("default");
  markerMsg1->set_id(0);
  markerMsg1->set_action(gz::msgs::Marker::ADD_MODIFY);
  markerMsg1->set_type(gz::msgs::Marker::SPHERE);
  markerMsg1->set_visibility(gz::msgs::Marker::GUI);

  // Set color to Blue
  markerMsg1->mutable_material()->mutable_ambient()->set_r(0);
  markerMsg1->mutable_material()->mutable_ambient()->set_g(0);
  markerMsg1->mutable_material()->mutable_ambient()->set_b(1);
  markerMsg1->mutable_material()->mutable_ambient()->set_a(1);
  markerMsg1->mutable_material()->mutable_diffuse()->set_r(0);
  markerMsg1->mutable_material()->mutable_diffuse()->set_g(0);
  markerMsg1->mutable_material()->mutable_diffuse()->set_b(1);
  markerMsg1->mutable_material()->mutable_diffuse()->set_a(1);
  gz::msgs::Set(markerMsg1->mutable_scale(),
                    gz::math::Vector3d(1.0, 1.0, 1.0));
  gz::msgs::Set(markerMsg1->mutable_pose(),
                      gz::math::Pose3d(3, 3, 0, 0, 0, 0));

  // Create second red box marker
  auto markerMsg2 = markerMsgs.add_marker();
  markerMsg2->set_ns("default");
  markerMsg2->set_id(0);
  markerMsg2->set_action(gz::msgs::Marker::ADD_MODIFY);
  markerMsg2->set_type(gz::msgs::Marker::BOX);
  markerMsg2->set_visibility(gz::msgs::Marker::GUI);

  // Set color to Red
  markerMsg2->mutable_material()->mutable_ambient()->set_r(1);
  markerMsg2->mutable_material()->mutable_ambient()->set_g(0);
  markerMsg2->mutable_material()->mutable_ambient()->set_b(0);
  markerMsg2->mutable_material()->mutable_ambient()->set_a(1);
  markerMsg2->mutable_material()->mutable_diffuse()->set_r(1);
  markerMsg2->mutable_material()->mutable_diffuse()->set_g(0);
  markerMsg2->mutable_material()->mutable_diffuse()->set_b(0);
  markerMsg2->mutable_material()->mutable_diffuse()->set_a(1);
  markerMsg2->mutable_lifetime()->set_sec(2);
  markerMsg2->mutable_lifetime()->set_nsec(0);
  gz::msgs::Set(markerMsg2->mutable_scale(),
                    gz::math::Vector3d(1.0, 1.0, 1.0));
  gz::msgs::Set(markerMsg2->mutable_pose(),
                      gz::math::Pose3d(3, 3, 2, 0, 0, 0));

  // Create third green cylinder marker
  auto markerMsg3 = markerMsgs.add_marker();
  markerMsg3->set_ns("default");
  markerMsg3->set_id(0);
  markerMsg3->set_action(gz::msgs::Marker::ADD_MODIFY);
  markerMsg3->set_type(gz::msgs::Marker::CYLINDER);
  markerMsg3->set_visibility(gz::msgs::Marker::GUI);

  // Set color to Green
  markerMsg3->mutable_material()->mutable_ambient()->set_r(0);
  markerMsg3->mutable_material()->mutable_ambient()->set_g(1);
  markerMsg3->mutable_material()->mutable_ambient()->set_b(0);
  markerMsg3->mutable_material()->mutable_ambient()->set_a(1);
  markerMsg3->mutable_material()->mutable_diffuse()->set_r(0);
  markerMsg3->mutable_material()->mutable_diffuse()->set_g(1);
  markerMsg3->mutable_material()->mutable_diffuse()->set_b(0);
  markerMsg3->mutable_material()->mutable_diffuse()->set_a(1);
  markerMsg3->mutable_lifetime()->set_sec(2);
  markerMsg3->mutable_lifetime()->set_nsec(0);
  gz::msgs::Set(markerMsg3->mutable_scale(),
                    gz::math::Vector3d(1.0, 1.0, 1.0));
  gz::msgs::Set(markerMsg3->mutable_pose(),
                      gz::math::Pose3d(3, 3, 4, 0, 0, 0));

  // Publish the three created markers above simultaneously
  node.Request("/marker_array", markerMsgs, timeout, res, result);

  std::cout << "Deleting all the markers\n";
  GZ_SLEEP_S(4);
  markerMsg.set_action(gz::msgs::Marker::DELETE_ALL);
  node.Request("/marker", markerMsg);
}
