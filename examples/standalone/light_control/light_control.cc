/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <chrono>
#include <iostream>

#include <ignition/gazebo/components/Light.hh>
#include <ignition/gazebo/components/LightCmd.hh>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

using namespace std::chrono_literals;

// Create a transport node.
ignition::transport::Node node;

bool result;
// timeout used for services
constexpr unsigned int timeout = 5000;

std::mt19937 twister(std::time(0));
std::uniform_real_distribution<double> distr(0, 1000);
constexpr double epsilon = 0.1;

float directionX = 0.5;
float directionY = 0.2;
float directionZ = -0.9;

void createLight()
{
  ignition::msgs::Boolean rep;
//! [create light]
  ignition::msgs::EntityFactory entityFactoryRequest;

  entityFactoryRequest.mutable_light()->set_name("point");
  entityFactoryRequest.mutable_light()->set_range(4);
  entityFactoryRequest.mutable_light()->set_attenuation_linear(0.5);
  entityFactoryRequest.mutable_light()->set_attenuation_constant(0.2);
  entityFactoryRequest.mutable_light()->set_attenuation_quadratic(0.01);
  entityFactoryRequest.mutable_light()->set_cast_shadows(false);
  entityFactoryRequest.mutable_light()->set_type(ignition::msgs::Light::POINT);
  ignition::msgs::Set(
    entityFactoryRequest.mutable_light()->mutable_direction(),
    ignition::math::Vector3d(directionX, directionY, directionZ));
  ignition::msgs::Set(entityFactoryRequest.mutable_light()->mutable_pose(),
    ignition::math::Pose3d(0.0, 0, 3.0, 0.0, 0.0, 0.0));
//! [create light]

//! [call service create]
  bool executedEntityFactory = node.Request("/world/empty/create",
        entityFactoryRequest, timeout, rep, result);
  if (executedEntityFactory)
  {
    if (result)
      std::cout << "Light was created : [" << rep.data() << "]" << std::endl;
    else
    {
      std::cout << "Service call failed" << std::endl;
      return;
    }
  }
  else
    std::cerr << "Service call timed out" << std::endl;
//! [call service create]
}

void createSphere()
{
//! [create sphere]
  auto modelStr = R"(
  <?xml version="1.0" ?>
  <sdf version='1.7'>
    <model name='sphere'>
      <link name='link'>
        <pose>0 0 0.5 0 0 0</pose>
        <visual name='visual'>
          <geometry><sphere><radius>1</radius></sphere></geometry>
        </visual>
        <collision name='collision'>
          <geometry><sphere><radius>1</radius></sphere></geometry>
        </collision>
      </link>
    </model>
  </sdf>)";

  ignition::msgs::EntityFactory req;
  ignition::msgs::Boolean res;
  req.set_sdf(modelStr);
//! [create sphere]

//! [call service create sphere]
  bool executed = node.Request("/world/empty/create",
            req, timeout, res, result);
  if (executed)
  {
    if (result)
      std::cout << "Sphere was created : [" << res.data() << "]" << std::endl;
    else
    {
      std::cout << "Service call failed" << std::endl;
      return;
    }
  }
  else
    std::cerr << "Service call timed out" << std::endl;
//! [call service create sphere]
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ignition::msgs::Boolean rep;
  ignition::msgs::Light lightRequest;
  auto lightConfigService = "/world/empty/light_config";

  createSphere();
  createLight();

  while (1){
    float x, y, z;
    double m = 0;
//! [random numbers]
    while (m < epsilon) {
      x = distr(twister);
      y = distr(twister);
      z = distr(twister);
      m = std::sqrt(x*x + y*y + z*z);
    }
    x /= m;
    y /= m;
    z /= m;
//! [random numbers]

//! [modify light]
    lightRequest.set_name("point");
    lightRequest.set_range(4);
    lightRequest.set_attenuation_linear(0.5);
    lightRequest.set_attenuation_constant(0.2);
    lightRequest.set_attenuation_quadratic(0.01);
    lightRequest.set_cast_shadows(false);
    lightRequest.set_type(ignition::msgs::Light::POINT);
    ignition::msgs::Set(lightRequest.mutable_direction(),
      ignition::math::Vector3d(directionX, directionY, directionZ));
    ignition::msgs::Set(lightRequest.mutable_pose(),
      ignition::math::Pose3d(0.0, -1.5, 3.0, 0.0, 0.0, 0.0));
    ignition::msgs::Set(lightRequest.mutable_diffuse(),
      ignition::math::Color(x, y, z, 1));
    ignition::msgs::Set(lightRequest.mutable_specular(),
      ignition::math::Color(x, y, z, 1));
//! [modify light]
    bool executed = node.Request(lightConfigService, lightRequest, timeout,
        rep, result);
    std::cout << "Service called: [" << x << ", " << y << ", " << z << "]"
      << std::endl;

    if (executed)
    {
      if (result)
        std::cout << "Response: [" << rep.data() << "]" << std::endl;
      else
        std::cout << "Service call failed" << std::endl;
    }
    else
      std::cerr << "Service call timed out" << std::endl;

    std::this_thread::sleep_for(1s);
  }
}
