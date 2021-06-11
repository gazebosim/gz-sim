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

#include "ModelCommandAPI.hh"

#include <string>
#include <vector>
#include <map>

#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/gazebo/components/AxisAlignedBox.hh>
#include <ignition/gazebo/components/ChildLinkName.hh>
#include <ignition/gazebo/components/Inertial.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointType.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/ParentLinkName.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/msgs.hh>
#include <ignition/msgs/serialized.pb.h>
#include <ignition/transport/Node.hh>

//////////////////////////////////////////////////
std::string getWorldName()
{
  std::string world_name;

  // Create a transport node.
  ignition::transport::Node node;

  bool executed{false};
  bool result{false};
  unsigned int timeout{5000};
  std::string service{"/gazebo/worlds"};

  // Request and block
  ignition::msgs::StringMsg_V res;
  executed = node.Request(service, timeout, res, result);

  if (!executed)
  {
    std::cerr << std::endl << "Service call to [" << service << "] timed out"
              << std::endl;
    return "";
  }

  if (!result)
  {
    std::cerr << std::endl << "Service call to [" << service << "] failed"
              << std::endl;
    return "";
  }

  world_name += res.data().Get(0);

  return world_name;
}

//////////////////////////////////////////////////
extern "C" IGNITION_GAZEBO_VISIBLE void cmdModelList()
{
  std::string world = getWorldName();
  if (world.empty())
    return;

  // Create a transport node.
  ignition::transport::Node node;

  bool executed{false};
  bool result{false};
  unsigned int timeout{5000};
  std::string service{"/world/" + world + "/state"};

  // Request and block
  ignition::msgs::SerializedStepMap res;
  executed = node.Request(service, timeout, res, result);

  if (!executed)
  {
    std::cerr << std::endl << "Service call to [" << service << "] timed out"
              << std::endl;
    return;
  }

  if (!result)
  {
    std::cerr << std::endl << "Service call to [" << service << "] failed"
              << std::endl;
    return;
  }

  // Instantiate an ECM and populate with data from message
  ignition::gazebo::EntityComponentManager ecm;
  ecm.SetState(res.state());

  auto models = ecm.EntitiesByComponents(
    ignition::gazebo::components::ParentEntity(1),
    ignition::gazebo::components::Model());

  std::cout << "Available models:" << std::endl;

  for(auto &m : models)
  {
    auto nameComp =
        ecm.Component<ignition::gazebo::components::Name>(m);
    std::cout << "    - " << nameComp->Data() << std::endl;
  }
}

//////////////////////////////////////////////////
void printPose(uint64_t entity,
                ignition::gazebo::EntityComponentManager &ecm){
  auto modelPose = ecm.EntitiesByComponents(
    ignition::gazebo::components::ParentEntity(entity),
    ignition::gazebo::components::Pose());
  {
    std::string poseInfo;
    auto poseComp =
        ecm.Component<ignition::gazebo::components::Pose>(entity);
    auto nameComp =
        ecm.Component<ignition::gazebo::components::Name>(entity);
    if (poseComp)
    {
      poseInfo += "\n      [" + std::to_string(poseComp->Data().X()) + " | "
                              + std::to_string(poseComp->Data().Y()) + " | "
                              + std::to_string(poseComp->Data().Z()) + "]\n"
                    "      [" + std::to_string(poseComp->Data().Roll()) + " | "
                              + std::to_string(poseComp->Data().Pitch()) + " | "
                              + std::to_string(poseComp->Data().Yaw()) + "]";

      std::cout << "Name: " << nameComp->Data() << std::endl
                << "  - Pose: " << poseInfo << std::endl << std::endl;
    }
  }

}

//////////////////////////////////////////////////
void printLinks(uint64_t entity,
                ignition::gazebo::EntityComponentManager &ecm,
                std::string link_name)
{
  auto links = ecm.EntitiesByComponents(
  ignition::gazebo::components::ParentEntity(entity),
  ignition::gazebo::components::Link());
  for(auto &_entity : links)
  {
    {
      auto parentComp =
          ecm.Component<ignition::gazebo::components::ParentEntity>(_entity);

      auto nameComp =
          ecm.Component<ignition::gazebo::components::Name>(_entity);

      if(link_name.length() && link_name != nameComp->Data())
          continue;

      std::string parentInfo;
      if (parentComp)
      {
        auto parentNameComp =
            ecm.Component<ignition::gazebo::components::Name>(
            parentComp->Data());

        if (parentNameComp)
        {
          parentInfo += parentNameComp->Data() + " ";
        }
        parentInfo += "[" + std::to_string(parentComp->Data()) + "]";
      }

      std::cout << "  - Link [" << _entity << "]" << std::endl
                << "    - Name: " << nameComp->Data() << std::endl
                << "    - Parent: " << parentInfo << std::endl;
    }
    {
      auto inertialComp =
          ecm.Component<ignition::gazebo::components::Inertial>(_entity);

      std::string massInfo;
      std::string inertialInfo;
      if (inertialComp)
      {
        auto inertialMatrix =  inertialComp->Data().MassMatrix();
        auto massComp = inertialComp->Data().MassMatrix().Mass();

        massInfo += "[" + std::to_string(massComp) + "]";
        inertialInfo +=
              "\n        [" + std::to_string(inertialMatrix.Ixx()) + " | "
                            + std::to_string(inertialMatrix.Ixy()) + " | "
                            + std::to_string(inertialMatrix.Ixz()) + "]\n"
                "        [" + std::to_string(inertialMatrix.Ixy()) + " | "
                            + std::to_string(inertialMatrix.Iyy()) + " | "
                            + std::to_string(inertialMatrix.Iyz()) + "]\n"
                "        [" + std::to_string(inertialMatrix.Ixz()) + " | "
                            + std::to_string(inertialMatrix.Iyz()) + " | "
                            + std::to_string(inertialMatrix.Izz()) + "]";
        std::cout << "    - Mass: " << massInfo << std::endl
                  << "    - Inertial Matrix: " << inertialInfo << std::endl;
      }
    }
    {
      auto poseComp =
          ecm.Component<ignition::gazebo::components::Pose>(_entity);

      std::string poseInfo;
      if (poseComp)
      {
        poseInfo +=
              "\n        [" + std::to_string(poseComp->Data().X()) + " | "
                            + std::to_string(poseComp->Data().Y()) + " | "
                            + std::to_string(poseComp->Data().Z()) + "]\n"
                "        [" + std::to_string(poseComp->Data().Roll()) + " | "
                            + std::to_string(poseComp->Data().Pitch()) + " | "
                            + std::to_string(poseComp->Data().Yaw()) + "]";

        std::cout << "    - Pose: " << poseInfo << std::endl;
      }
    }
  }
}

//////////////////////////////////////////////////
void printJoints(uint64_t entity,
                ignition::gazebo::EntityComponentManager &ecm,
                std::string joint_name)
{
  auto joints = ecm.EntitiesByComponents(
  ignition::gazebo::components::ParentEntity(entity),
  ignition::gazebo::components::Joint());

  for(auto &_entity : joints)
  {
    {
      auto parentComp =
          ecm.Component<ignition::gazebo::components::ParentEntity>(_entity);

      auto nameComp =
          ecm.Component<ignition::gazebo::components::Name>(_entity);

      if(joint_name.length() && joint_name != nameComp->Data())
          continue;

      std::string parentInfo;
      if (parentComp)
      {
        auto parentNameComp =
            ecm.Component<ignition::gazebo::components::Name>(
            parentComp->Data());

        if (parentNameComp)
        {
          parentInfo += parentNameComp->Data() + " ";
        }
        parentInfo += "[" + std::to_string(parentComp->Data()) + "]";
      }

      std::cout << "  - Joint [" << _entity << "]" << std::endl
                << "    - Name: " << nameComp->Data() << std::endl
                << "    - Parent: " << parentInfo << std::endl;
    }
    {

      std::map<sdf::JointType, std::string> jointTypes = {
        {sdf::JointType::REVOLUTE, "revolute"},
        {sdf::JointType::BALL, "ball"},
        {sdf::JointType::CONTINUOUS, "continuous"},
        {sdf::JointType::FIXED, "fixed"},
        {sdf::JointType::GEARBOX, "gearbox"},
        {sdf::JointType::PRISMATIC,  "prismatic"},
        {sdf::JointType::REVOLUTE2, "revolute2"},
        {sdf::JointType::SCREW, "screw"},
        {sdf::JointType::UNIVERSAL, "universal"}
      };

      auto jointComp =
          ecm.Component<ignition::gazebo::components::JointType>(_entity);
      auto childLinkComp =
          ecm.Component<ignition::gazebo::components::ChildLinkName>(_entity);
      auto parentLinkComp =
          ecm.Component<ignition::gazebo::components::ParentLinkName>(_entity);

      if (childLinkComp && parentLinkComp)
      {
        std::cout << "    - Joint type:  " << jointTypes[jointComp->Data()]
        << "\n"   << "    - Parent Link: [" << childLinkComp->Data() << "]\n"
                  << "    - Child Link:  [" << parentLinkComp->Data() << "]\n";
      }
    }
  }
}

//////////////////////////////////////////////////
void printBoundingBox(uint64_t entity,
                ignition::gazebo::EntityComponentManager &ecm)
{
  auto bounding_boxes = ecm.EntitiesByComponents(
  ignition::gazebo::components::ParentEntity(entity),
  ignition::gazebo::components::AxisAlignedBox());

  for(auto &_entity : bounding_boxes)
  {
    {
      auto parentComp =
          ecm.Component<ignition::gazebo::components::ParentEntity>(_entity);

      auto nameComp =
          ecm.Component<ignition::gazebo::components::Name>(_entity);

      std::string parentInfo;
      if (parentComp)
      {
        auto parentNameComp =
            ecm.Component<ignition::gazebo::components::Name>(
            parentComp->Data());

        if (parentNameComp)
        {
          parentInfo += parentNameComp->Data() + " ";
        }
        parentInfo += "[" + std::to_string(parentComp->Data()) + "]";
      }

      std::cout << "  - Bounding Box [" << _entity << "]" << std::endl
                << "    - Name: " << nameComp->Data() << std::endl
                << "    - Parent: " << parentInfo << std::endl;
    }
    {
      auto boxComp =
          ecm.Component<ignition::gazebo::components::AxisAlignedBox>(_entity);

      if (boxComp)
      {

        std::cout << "     Size:   [" << boxComp->Data().Size() << "]\n"
                  << "     Center: [" << boxComp->Data().Center() << "]\n"
                  << "     Min:    [" << boxComp->Data().Min() << "]\n"
                  << "     Max:    [" << boxComp->Data().Max() << "]\n";
      }
    }
  }
}

//////////////////////////////////////////////////
extern "C" IGNITION_GAZEBO_VISIBLE void cmdModelInfo(
  const char *_model, int _pose, int _link, const char *_link_name,
  int _joint, const char *_joint_name)
{
  std::string link_name{""};
  if(_link_name)
    link_name = _link_name;
  std::string joint_name{""};
  if(_joint_name)
    joint_name = _joint_name;

  bool printAll{false};
  if(!_pose && !_link && !_joint)
    printAll = true;

  std::string model{_model};
  // Get arguments
  std::string world = getWorldName();
  if (world.empty())
    return;
  // Create a transport node.
  ignition::transport::Node node;

  bool executed{false};
  bool result{false};
  unsigned int timeout{5000};
  std::string service{"/world/" + world + "/state"};

  std::cout << std::endl << "Requesting state for world [" << world
            << "] on service [" << service << "]..." << std::endl << std::endl;

  // Request and block
  ignition::msgs::SerializedStepMap res;
  executed = node.Request(service, timeout, res, result);

  if (!executed)
  {
    std::cerr << std::endl << "Service call to [" << service << "] timed out"
              << std::endl;
    return;
  }

  if (!result)
  {
    std::cerr << std::endl << "Service call to [" << service << "] failed"
              << std::endl;
    return;
  }

  // Instantiate an ECM and populate with data from message
  ignition::gazebo::EntityComponentManager ecm;
  ecm.SetState(res.state());

  // Get the desired model entity.
  auto entity = ecm.EntityByComponents(
    ignition::gazebo::components::Name(model),
    ignition::gazebo::components::Model());

  // Get the pose of the model
  if(printAll | _pose)
    printPose(entity, ecm);

  // Get the links information
  if(printAll | _link)
    printLinks(entity, ecm, link_name);

  // Get the links information
  if(printAll | _joint)
    printJoints(entity, ecm, joint_name);

  // Get the bounding_boxes information
  if(printAll)
    printBoundingBox(entity, ecm);
}
