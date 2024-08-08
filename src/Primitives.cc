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

#include <gz/common/Util.hh>
#include <gz/common/Console.hh>
#include "gz/sim/Primitives.hh"

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
constexpr const char * kBoxSdf = R"(<?xml version="1.0"?>
<sdf version="1.9">
  <model name="box">
    <pose>0 0 0.5 0 0 0</pose>
    <link name="box_link">
      <inertial>
        <inertia>
          <ixx>0.16666</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.16666</iyy>
          <iyz>0</iyz>
          <izz>0.16666</izz>
        </inertia>
        <mass>1.0</mass>
      </inertial>
      <collision name="box_collision">
        <geometry>
          <box>
            <size>1 1 1</size>
          </box>
        </geometry>
      </collision>
      <visual name="box_visual">
        <geometry>
          <box>
            <size>1 1 1</size>
          </box>
        </geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
          <specular>1 1 1 1</specular>
        </material>
     </visual>
    </link>
  </model>
</sdf>
)";

/////////////////////////////////////////////////
constexpr const char * kSphereSdf = R"(<?xml version="1.0"?>
<sdf version="1.9">
  <model name="sphere">
    <pose>0 0 0.5 0 0 0</pose>
    <link name="sphere_link">
      <inertial>
        <inertia>
          <ixx>0.1</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.1</iyy>
          <iyz>0</iyz>
          <izz>0.1</izz>
        </inertia>
        <mass>1.0</mass>
      </inertial>
      <collision name="sphere_collision">
        <geometry>
          <sphere>
            <radius>0.5</radius>
          </sphere>
        </geometry>
      </collision>
      <visual name="sphere_visual">
        <geometry>
          <sphere>
            <radius>0.5</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
          <specular>1 1 1 1</specular>
        </material>
     </visual>
    </link>
  </model>
</sdf>
)";

/////////////////////////////////////////////////
constexpr const char * kConeSdf = R"(<?xml version="1.0"?>
<sdf version="1.9">
  <model name="cone">
    <pose>0 0 0.5 0 0 0</pose>
    <link name="cone_link">
      <inertial>
        <inertia>
          <ixx>0.075</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.075</iyy>
          <iyz>0</iyz>
          <izz>0.075</izz>
        </inertia>
        <mass>1.0</mass>
      </inertial>
      <collision name="cone_collision">
        <geometry>
          <cone>
            <radius>0.5</radius>
            <length>1.0</length>
          </cone>
        </geometry>
      </collision>
      <visual name="cone_visual">
        <geometry>
          <cone>
            <radius>0.5</radius>
            <length>1.0</length>
          </cone>
        </geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
          <specular>1 1 1 1</specular>
        </material>
     </visual>
    </link>
  </model>
</sdf>
)";

/////////////////////////////////////////////////
constexpr const char * kCylinderSdf = R"(<?xml version="1.0"?>
<sdf version="1.9">
  <model name="cylinder">
    <pose>0 0 0.5 0 0 0</pose>
    <link name="cylinder_link">
      <inertial>
        <inertia>
          <ixx>0.1458</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.1458</iyy>
          <iyz>0</iyz>
          <izz>0.125</izz>
        </inertia>
        <mass>1.0</mass>
      </inertial>
      <collision name="cylinder_collision">
        <geometry>
          <cylinder>
            <radius>0.5</radius>
            <length>1.0</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="cylinder_visual">
        <geometry>
          <cylinder>
            <radius>0.5</radius>
            <length>1.0</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
          <specular>1 1 1 1</specular>
        </material>
     </visual>
    </link>
  </model>
</sdf>
)";

/////////////////////////////////////////////////
constexpr const char * kCapsuleSdf = R"(<?xml version="1.0"?>
<sdf version="1.9">
  <model name="capsule">
    <pose>0 0 0.5 0 0 0</pose>
    <link name="capsule_link">
      <inertial>
        <inertia>
          <ixx>0.074154</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.074154</iyy>
          <iyz>0</iyz>
          <izz>0.018769</izz>
        </inertia>
        <mass>1.0</mass>
      </inertial>
      <collision name="capsule_collision">
        <geometry>
          <capsule>
            <radius>0.2</radius>
            <length>0.6</length>
          </capsule>
        </geometry>
      </collision>
      <visual name="capsule_visual">
        <geometry>
          <capsule>
            <radius>0.2</radius>
            <length>0.6</length>
          </capsule>
        </geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
          <specular>1 1 1 1</specular>
        </material>
     </visual>
    </link>
  </model>
</sdf>
)";

/////////////////////////////////////////////////
constexpr const char *kEllipsoidSdf = R"(<?xml version="1.0"?>
<sdf version="1.9">
  <model name="ellipsoid">
    <pose>0 0 0.5 0 0 0</pose>
    <link name="ellipsoid_link">
      <inertial>
        <inertia>
          <ixx>0.068</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.058</iyy>
          <iyz>0</iyz>
          <izz>0.026</izz>
        </inertia>
        <mass>1.0</mass>
      </inertial>
      <collision name="ellipsoid_collision">
        <geometry>
          <ellipsoid>
            <radii>0.2 0.3 0.5</radii>
          </ellipsoid>
        </geometry>
      </collision>
      <visual name="ellipsoid_visual">
        <geometry>
          <ellipsoid>
            <radii>0.2 0.3 0.5</radii>
          </ellipsoid>
        </geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
          <specular>1 1 1 1</specular>
        </material>
     </visual>
    </link>
  </model>
</sdf>
)";

/////////////////////////////////////////////////
constexpr const char *kDirectionalSdf = R"(<?xml version="1.0"?>
<sdf version="1.9">
  <light type='directional' name='directionallight'>
    <pose>0 0 2 0 0 0</pose>
    <cast_shadows>true</cast_shadows>
    <diffuse>0.8 0.8 0.8 1</diffuse>
    <specular>0.2 0.2 0.2 1</specular>
    <attenuation>
      <range>1000</range>
      <constant>0.9</constant>
      <linear>0.01</linear>
      <quadratic>0.001</quadratic>
    </attenuation>
    <direction>0 0 -1</direction>
  </light>
</sdf>
)";

/////////////////////////////////////////////////
constexpr const char *kPointSdf = R"(<?xml version="1.0"?>
<sdf version="1.9">
  <light type='point' name='pointlight'>
    <pose>0 0 2 0 0 0</pose>
    <cast_shadows>false</cast_shadows>
    <diffuse>0.5 0.5 0.5 1</diffuse>
    <specular>0.5 0.5 0.5 1</specular>
    <attenuation>
      <range>4</range>
      <constant>0.2</constant>
      <linear>0.5</linear>
      <quadratic>0.01</quadratic>
    </attenuation>
  </light>
</sdf>
)";

/////////////////////////////////////////////////
constexpr const char *kSpotSdf = R"(<?xml version="1.0"?>
<sdf version="1.9">
  <light type='spot' name='spotlight'>
    <pose>0 0 2 0 0 0</pose>
    <cast_shadows>true</cast_shadows>
    <diffuse>0.5 0.5 0.5 1</diffuse>
    <specular>0.5 0.5 0.5 1</specular>
    <attenuation>
      <range>4</range>
      <constant>0.2</constant>
      <linear>0.5</linear>
      <quadratic>0.01</quadratic>
    </attenuation>
    <direction>0 0 -1</direction>
    <spot>
      <inner_angle>0.1</inner_angle>
      <outer_angle>0.5</outer_angle>
      <falloff>0.8</falloff>
    </spot>
  </light>
</sdf>
)";

/////////////////////////////////////////////////
std::string gz::sim::getPrimitiveShape(const PrimitiveShape &_type)
{
  switch(_type)
  {
    case PrimitiveShape::kBox:
      return kBoxSdf;
    case PrimitiveShape::kSphere:
      return kSphereSdf;
    case PrimitiveShape::kCone:
      return kConeSdf;
    case PrimitiveShape::kCylinder:
      return kCylinderSdf;
    case PrimitiveShape::kCapsule:
      return kCapsuleSdf;
    case PrimitiveShape::kEllipsoid:
      return kEllipsoidSdf;
    default:
      return "";
  }
}

/////////////////////////////////////////////////
std::string gz::sim::getPrimitiveLight(const PrimitiveLight &_type)
{
  switch(_type)
  {
    case PrimitiveLight::kDirectional:
      return kDirectionalSdf;
    case PrimitiveLight::kPoint:
      return kPointSdf;
    case PrimitiveLight::kSpot:
      return kSpotSdf;
    default:
      return "";
  }
}

/////////////////////////////////////////////////
std::string gz::sim::getPrimitive(const std::string &_typeName)
{
  std::string type = common::lowercase(_typeName);

  if (type == "box")
    return getPrimitiveShape(PrimitiveShape::kBox);
  else if (type == "sphere")
    return getPrimitiveShape(PrimitiveShape::kSphere);
  else if (type == "cylinder")
    return getPrimitiveShape(PrimitiveShape::kCylinder);
  else if (type == "cone")
    return getPrimitiveShape(PrimitiveShape::kCone);
  else if (type == "capsule")
    return getPrimitiveShape(PrimitiveShape::kCapsule);
  else if (type == "ellipsoid")
    return getPrimitiveShape(PrimitiveShape::kEllipsoid);
  else if (type == "point")
    return getPrimitiveLight(PrimitiveLight::kPoint);
  else if (type == "directional")
    return getPrimitiveLight(PrimitiveLight::kDirectional);
  else if (type == "spot")
    return getPrimitiveLight(PrimitiveLight::kSpot);

  gzwarn << "Invalid model string " << type << "\n";
  gzwarn << "The valid options are:\n";
  gzwarn << " - box\n";
  gzwarn << " - sphere\n";
  gzwarn << " - cone\n";
  gzwarn << " - cylinder\n";
  gzwarn << " - capsule\n";
  gzwarn << " - ellipsoid\n";
  gzwarn << " - point\n";
  gzwarn << " - directional\n";
  gzwarn << " - spot\n";
  return "";
}
