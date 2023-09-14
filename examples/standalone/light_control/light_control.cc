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
#include <random>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/light.pb.h>

#include <gz/sim/components/Light.hh>
#include <gz/sim/components/LightCmd.hh>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>

using namespace std::chrono_literals;

// Create a transport node.
gz::transport::Node node;

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
  gz::msgs::Boolean rep;
//! [create light]
  gz::msgs::EntityFactory entityFactoryRequest;

  entityFactoryRequest.mutable_light()->set_name("point");
  entityFactoryRequest.mutable_light()->set_range(4);
  entityFactoryRequest.mutable_light()->set_intensity(1);
  entityFactoryRequest.mutable_light()->set_attenuation_linear(0.5);
  entityFactoryRequest.mutable_light()->set_attenuation_constant(0.2);
  entityFactoryRequest.mutable_light()->set_attenuation_quadratic(0.01);
  entityFactoryRequest.mutable_light()->set_cast_shadows(false);
  entityFactoryRequest.mutable_light()->set_type(gz::msgs::Light::POINT);
  gz::msgs::Set(
    entityFactoryRequest.mutable_light()->mutable_direction(),
    gz::math::Vector3d(directionX, directionY, directionZ));
  gz::msgs::Set(entityFactoryRequest.mutable_light()->mutable_pose(),
    gz::math::Pose3d(0.0, 0, 3.0, 0.0, 0.0, 0.0));
//! [create light]

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
}

void createSphere()
{
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

  gz::msgs::EntityFactory req;
  gz::msgs::Boolean res;
  req.set_sdf(modelStr);

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
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  gz::msgs::Boolean rep;
  gz::msgs::Light lightRequest;
  auto lightConfigService = "/world/empty/light_config";

  createSphere();
  createLight();

  while (1)
  {
    float r, g, b;
    double m = 0;
//! [random numbers]
    while (m < epsilon)
    {
      r = distr(twister);
      g = distr(twister);
      b = distr(twister);
      m = std::sqrt(r*r + b*b + g*g);
    }
    r /= m;
    g /= m;
    b /= m;
//! [random numbers]

//! [modify light]
    lightRequest.set_name("point");
    lightRequest.set_range(4);
    lightRequest.set_intensity(1);
    lightRequest.set_attenuation_linear(0.5);
    lightRequest.set_attenuation_constant(0.2);
    lightRequest.set_attenuation_quadratic(0.01);
    lightRequest.set_cast_shadows(false);
    lightRequest.set_type(gz::msgs::Light::POINT);
    // direction field only affects spot / directional lights
    gz::msgs::Set(lightRequest.mutable_direction(),
      gz::math::Vector3d(directionX, directionY, directionZ));
    gz::msgs::Set(lightRequest.mutable_pose(),
      gz::math::Pose3d(0.0, -1.5, 3.0, 0.0, 0.0, 0.0));
    gz::msgs::Set(lightRequest.mutable_diffuse(),
      gz::math::Color(r, g, b, 1));
    gz::msgs::Set(lightRequest.mutable_specular(),
      gz::math::Color(r, g, b, 1));
//! [modify light]
    bool executed = node.Request(lightConfigService, lightRequest, timeout,
        rep, result);
    std::cout << "Service called: [" << r << ", " << g << ", " << b << "]"
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
