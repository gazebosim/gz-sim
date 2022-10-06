/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <gz/msgs/entity_factory.pb.h>

#include <iostream>

#include <gz/math/Color.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>

// Create a transport node.
gz::transport::Node node;

// timeout used for services
constexpr unsigned int timeout = 5000;

void createLight()
{
  bool result;
  gz::msgs::Boolean rep;
//! [create light]
  gz::msgs::EntityFactory entityFactoryRequest;

  entityFactoryRequest.mutable_light()->set_name("point");
  entityFactoryRequest.mutable_light()->set_range(4);
  entityFactoryRequest.mutable_light()->set_attenuation_linear(0.5);
  entityFactoryRequest.mutable_light()->set_attenuation_constant(0.2);
  entityFactoryRequest.mutable_light()->set_attenuation_quadratic(0.01);
  entityFactoryRequest.mutable_light()->set_cast_shadows(false);
  entityFactoryRequest.mutable_light()->set_type(gz::msgs::Light::POINT);
  gz::msgs::Set(
    entityFactoryRequest.mutable_light()->mutable_direction(),
    gz::math::Vector3d(0.5, 0.2, -0.9));
  gz::msgs::Set(entityFactoryRequest.mutable_light()->mutable_pose(),
    gz::math::Pose3d(0.0, 0, 3.0, 0.0, 0.0, 0.0));
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

void createEntityFromStr(const std::string modelStr)
{
//! [call service create sphere]
  bool result;
  gz::msgs::EntityFactory req;
  gz::msgs::Boolean res;
  req.set_sdf(modelStr);

  bool executed = node.Request("/world/empty/create",
            req, timeout, res, result);
  if (executed)
  {
    if (result)
      std::cout << "Entity was created : [" << res.data() << "]" << std::endl;
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
std::string generateLightStr(
  const std::string light_type, const std::string name,
  const bool cast_shadows, const gz::math::Pose3d pose,
  const gz::math::Color diffuse,
  const gz::math::Color specular,
  const double attRange, const double attConstant,
  const double attLinear, const double attQuadratic,
  const gz::math::Vector3d direction,
  const double spot_inner_angle,
  const double spot_outer_angle,
  const double spot_falloff
)
{
//! [create light str]
  std::string lightStr = std::string("<sdf version='1.7'>") +
    "<light type='" + light_type + "' name='" + name + "'> " +
      "<cast_shadows>" + std::to_string(cast_shadows) + "</cast_shadows>" +
      "<pose>" +
      std::to_string(pose.Pos().X()) + " " +
      std::to_string(pose.Pos().Y()) + " " +
      std::to_string(pose.Pos().Z()) + " " +
      std::to_string(pose.Rot().Roll()) + " " +
      std::to_string(pose.Rot().Pitch()) + " " +
      std::to_string(pose.Rot().Yaw()) +
      "</pose>" +
      "<diffuse>" +
      std::to_string(diffuse.R()) + " " +
      std::to_string(diffuse.G()) + " " +
      std::to_string(diffuse.B()) + " " +
      std::to_string(diffuse.A()) +
      "</diffuse>" +
      "<specular>" +
      std::to_string(specular.R()) + " " +
      std::to_string(specular.G()) + " " +
      std::to_string(specular.B()) + " " +
      std::to_string(specular.A()) +
      "</specular>" +
      "<attenuation>" +
        "<range>" + std::to_string(attRange) + "</range>" +
        "<constant>" + std::to_string(attConstant) + "</constant>" +
        "<linear>" + std::to_string(attLinear) +   "</linear>" +
        "<quadratic>" + std::to_string(attQuadratic) + "</quadratic>" +
      "</attenuation>" +
      "<direction>" +
      std::to_string(direction.X()) + " " +
      std::to_string(direction.Y()) + " " +
      std::to_string(direction.Z()) +
      "</direction>" +
      "<spot>" +
        "<inner_angle>" + std::to_string(spot_inner_angle) + "</inner_angle>" +
        "<outer_angle>" + std::to_string(spot_outer_angle) + "</outer_angle>" +
        "<falloff>" + std::to_string(spot_falloff) + "</falloff>" +
      "</spot>" +
    "</light></sdf>";
//! [create light str]
  return lightStr;
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
//! [create sphere]
  auto sphereStr = R"(
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
//! [create sphere]

  createEntityFromStr(sphereStr);

  createEntityFromStr(
    generateLightStr("spot", "spot_light", false,
      gz::math::Pose3d(0, 0, 4, 0, 0, 0),
      gz::math::Color(0, 0, 1.0, 1.0),
      gz::math::Color(0, 0, 1.0, 1.0),
      1.0, 0.2, 0.2, 0.001,
      gz::math::Vector3d(0.5, 0.2, -0.9),
      0.15, 0.45, 1.0));

  createLight();
}
