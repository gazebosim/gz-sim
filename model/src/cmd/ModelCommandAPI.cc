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

#include "ModelCommandAPI.hh"

#include <string>
#include <vector>
#include <map>

#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>
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

namespace {

//////////////////////////////////////////////////
// \brief Get the name of the world being used by calling
// `/gazebo/worlds` service.
// \return The name of the world if service is available,
// an empty string otherwise.
std::string getWorldName()
{
  // Create a transport node.
  ignition::transport::Node node;

  bool result{false};
  const unsigned int timeout{5000};
  const std::string service{"/gazebo/worlds"};

  // Request and block
  ignition::msgs::StringMsg_V res;

  if (!node.Request(service, timeout, res, result))
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

  return res.data().Get(0);
}

//////////////////////////////////////////////////
// \brief Set the state of a ECM instance with a world snapshot.
// \param _ecm ECM instance to be populated.
// \return boolean indicating if it was able to populate the ECM.
bool PopulateECM(ignition::gazebo::EntityComponentManager &_ecm)
{
  const std::string world = getWorldName();
  if (world.empty())
  {
    std::cerr << "Command failed when trying to get the world name of "
              << "the running simulation." << std::endl;
    return false;
  }
  // Create a transport node.
  ignition::transport::Node node;
  bool result{false};
  const unsigned int timeout{5000};
  const std::string service{"/world/" + world + "/state"};

  std::cout << std::endl << "Requesting state for world [" << world
            << "]..." << std::endl << std::endl;

  // Request and block
  ignition::msgs::SerializedStepMap res;

  if (!node.Request(service, timeout, res, result))
  {
    std::cerr << std::endl << "Service call to [" << service << "] timed out"
              << std::endl;
    return false;
  }

  if (!result)
  {
    std::cerr << std::endl << "Service call to [" << service << "] failed"
              << std::endl;
    return false;
  }

  // Instantiate an ECM and populate with data from message
  _ecm.SetState(res.state());
  return true;
}


//////////////////////////////////////////////////
// \brief Print the model pose information.
// \param[in] _entity Entity of the model requested.
// \param[in] _ecm ECM ready for requests.
void printPose(const uint64_t _entity,
               const ignition::gazebo::EntityComponentManager &_ecm){
  const auto modelPose = _ecm.EntitiesByComponents(
    ignition::gazebo::components::ParentEntity(_entity),
    ignition::gazebo::components::Pose());

  const auto poseComp =
      _ecm.Component<ignition::gazebo::components::Pose>(_entity);
  const auto nameComp =
      _ecm.Component<ignition::gazebo::components::Name>(_entity);
  if (poseComp)
  {
    std::string poseInfo;
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

//////////////////////////////////////////////////
// \brief Print the model links information.
// \param[in] _entity Entity of the model requested.
// \param[in] _ecm ECM ready for requests.
// \param[in] _linkName Link to be printed, if nullptr, print all links.
void printLinks(const uint64_t _entity,
                const ignition::gazebo::EntityComponentManager &_ecm,
                const std::string &_linkName)
{
  const auto links = _ecm.EntitiesByComponents(
  ignition::gazebo::components::ParentEntity(_entity),
  ignition::gazebo::components::Link());
  for(const auto &entity : links)
  {
    const auto parentComp =
        _ecm.Component<ignition::gazebo::components::ParentEntity>(entity);

    const auto nameComp =
        _ecm.Component<ignition::gazebo::components::Name>(entity);

    if(_linkName.length() && _linkName != nameComp->Data())
        continue;

    if (parentComp)
    {
      std::string parentInfo;
      const auto parentNameComp =
          _ecm.Component<ignition::gazebo::components::Name>(
          parentComp->Data());

      if (parentNameComp)
      {
        parentInfo += parentNameComp->Data() + " ";
      }
      parentInfo += "[" + std::to_string(parentComp->Data()) + "]";
      std::cout << "  - Link [" << entity << "]" << std::endl
                << "    - Name: " << nameComp->Data() << std::endl
                << "    - Parent: " << parentInfo << std::endl;
    }

    const auto inertialComp =
        _ecm.Component<ignition::gazebo::components::Inertial>(entity);

    if (inertialComp)
    {
      const auto inertialMatrix =  inertialComp->Data().MassMatrix();
      const auto massComp = inertialComp->Data().MassMatrix().Mass();

      const std::string massInfo = "[" + std::to_string(massComp) + "]";
      const std::string inertialInfo =
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

    const auto poseComp =
        _ecm.Component<ignition::gazebo::components::Pose>(entity);

    if (poseComp)
    {
      const std::string poseInfo =
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

//////////////////////////////////////////////////
// \brief Print the model joints information.
// \param[in] _entity Entity of the model requested.
// \param[in] _ecm ECM ready for requests.
// \param[in] _jointName Joint to be printed, if nullptr, print all joints.
void printJoints(const uint64_t entity,
                const ignition::gazebo::EntityComponentManager &_ecm,
                const std::string &_jointName)
{
  static const std::map<sdf::JointType, std::string> jointTypes = {
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

  const auto joints = _ecm.EntitiesByComponents(
  ignition::gazebo::components::ParentEntity(entity),
  ignition::gazebo::components::Joint());

  for(const auto &_entity : joints)
  {
    const auto parentComp =
        _ecm.Component<ignition::gazebo::components::ParentEntity>(_entity);

    const auto nameComp =
        _ecm.Component<ignition::gazebo::components::Name>(_entity);

    if(_jointName.length() && _jointName != nameComp->Data())
        continue;

    if (parentComp)
    {
      std::string parentInfo;
      auto parentNameComp =
          _ecm.Component<ignition::gazebo::components::Name>(
          parentComp->Data());

      if (parentNameComp)
      {
        parentInfo += parentNameComp->Data() + " ";
      }
      parentInfo += "[" + std::to_string(parentComp->Data()) + "]";

      std::cout << "  - Joint [" << _entity << "]" << std::endl
                << "    - Name: " << nameComp->Data() << std::endl
                << "    - Parent: " << parentInfo << std::endl;
    }

    const auto jointComp =
        _ecm.Component<ignition::gazebo::components::JointType>(_entity);
    const auto childLinkComp =
        _ecm.Component<ignition::gazebo::components::ChildLinkName>(_entity);
    const auto parentLinkComp =
        _ecm.Component<ignition::gazebo::components::ParentLinkName>(_entity);

    if (childLinkComp && parentLinkComp)
    {
      std::cout << "    - Joint type:  " << jointTypes.at(jointComp->Data())
      << "\n"   << "    - Parent Link: [" << childLinkComp->Data() << "]\n"
                << "    - Child Link:  [" << parentLinkComp->Data() << "]\n";
    }
  }
}
}

//////////////////////////////////////////////////
extern "C" IGNITION_GAZEBO_VISIBLE void cmdModelList()
{
  ignition::gazebo::EntityComponentManager ecm{};
  if(!PopulateECM(ecm))
  {
    return;
  }
  const auto models = ecm.EntitiesByComponents(
    ignition::gazebo::components::ParentEntity(1),
    ignition::gazebo::components::Model());

  std::cout << "Available models:" << std::endl;

  for(const auto &m : models)
  {
    const auto nameComp =
        ecm.Component<ignition::gazebo::components::Name>(m);
    std::cout << "    - " << nameComp->Data() << std::endl;
  }
}

//////////////////////////////////////////////////
extern "C" IGNITION_GAZEBO_VISIBLE void cmdModelInfo(
  const char *_model, int _pose, int _link, const char *_linkName,
  int _joint, const char *_joint_name)
{
  std::string linkName{""};
  if(_linkName)
    linkName = _linkName;
  std::string jointName{""};
  if(_joint_name)
    jointName = _joint_name;

  bool printAll{false};
  if(!_pose && !_link && !_joint)
    printAll = true;

  if(!_model)
  {
    std::cerr << std::endl << "Model name not found" << std::endl;
    return;
  }
  const std::string model{_model};
  // Get arguments

  ignition::gazebo::EntityComponentManager ecm{};
  if(!PopulateECM(ecm))
    return;

  // Get the desired model entity.
  auto entity = ecm.EntityByComponents(
    ignition::gazebo::components::Name(model),
    ignition::gazebo::components::Model());

  // Get the pose of the model
  if(printAll | _pose)
    printPose(entity, ecm);

  // Get the links information
  if(printAll | _link)
    printLinks(entity, ecm, linkName);

  // Get the links information
  if(printAll | _joint)
    printJoints(entity, ecm, jointName);
}
