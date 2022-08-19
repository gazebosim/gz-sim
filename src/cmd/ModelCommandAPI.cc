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

#include <ignition/msgs/serialized.pb.h>
#include <ignition/msgs/stringmsg.pb.h>

#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/components/ChildLinkName.hh>
#include <ignition/gazebo/components/Inertial.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointAxis.hh>
#include <ignition/gazebo/components/JointType.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/ParentLinkName.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/transport/Node.hh>

using namespace gz;
using namespace sim;

//////////////////////////////////////////////////
/// \brief Get the name of the world being used by calling
/// `/gazebo/worlds` service.
/// \return The name of the world if service is available,
/// an empty string otherwise.
std::string getWorldName()
{
  // Create a transport node.
  transport::Node node;

  bool result{false};
  const unsigned int timeout{5000};
  const std::string service{"/gazebo/worlds"};

  // Request and block
  msgs::StringMsg_V res;

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
/// \brief Get entity info: name and entity ID
/// \param[in] _entity Entity to get info
/// \param[in] _ecm Entity component manager
/// \return "<entity name> [<entity ID>]"
std::string entityInfo(Entity _entity, const EntityComponentManager &_ecm)
{
  std::string info;

  const auto nameComp = _ecm.Component<components::Name>( _entity);
  if (nameComp)
  {
    info += nameComp->Data() + " ";
  }
  info += "[" + std::to_string(_entity) + "]";
  return info;
}

//////////////////////////////////////////////////
/// \brief Get entity info: name and entity ID
/// \param[in] _entity Name of entity to get info
/// \param[in] _ecm Entity component manager
/// \return "<entity name> [<entity ID>]"
std::string entityInfo(const std::string &_name,
    const EntityComponentManager &_ecm)
{
  std::string info{_name};

  auto entity = _ecm.EntityByComponents(components::Name(_name));
  if (kNullEntity != entity)
  {
    info += " [" + std::to_string(entity) + "]";
  }
  return info;
}

//////////////////////////////////////////////////
/// \brief Get pose info in a standard way
/// \param[in] _pose Pose to print
/// \param[in] _prefix Indentation prefix for every line
/// \return Pose formatted in a standard way
std::string poseInfo(math::Pose3d _pose, const std::string &_prefix)
{
  return
    _prefix + "[" + std::to_string(_pose.X()) + " | "
                  + std::to_string(_pose.Y()) + " | "
                  + std::to_string(_pose.Z()) + "]\n" +
    _prefix + "[" + std::to_string(_pose.Roll()) + " | "
                  + std::to_string(_pose.Pitch()) + " | "
                  + std::to_string(_pose.Yaw()) + "]";
}

//////////////////////////////////////////////////
// \brief Set the state of a ECM instance with a world snapshot.
// \param _ecm ECM instance to be populated.
// \return boolean indicating if it was able to populate the ECM.
bool populateECM(EntityComponentManager &_ecm)
{
  const std::string world = getWorldName();
  if (world.empty())
  {
    std::cerr << "Command failed when trying to get the world name of "
              << "the running simulation." << std::endl;
    return false;
  }
  // Create a transport node.
  transport::Node node;
  bool result{false};
  const unsigned int timeout{5000};
  const std::string service{"/world/" + world + "/state"};

  std::cout << std::endl << "Requesting state for world [" << world
            << "]..." << std::endl << std::endl;

  // Request and block
  msgs::SerializedStepMap res;

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
// \brief Print the model information.
// \param[in] _entity Entity of the model requested.
// \param[in] _ecm ECM ready for requests.
void printModelInfo(const uint64_t _entity,
               const EntityComponentManager &_ecm)
{
  const auto poseComp =
      _ecm.Component<components::Pose>(_entity);
  const auto nameComp =
      _ecm.Component<components::Name>(_entity);
  if (poseComp && nameComp)
  {
    std::cout << "Model: [" << _entity << "]" << std::endl
              << "  - Name: " << nameComp->Data() << std::endl
              << "  - Pose [ XYZ (m) ] [ RPY (rad) ]:" << std::endl
              << poseInfo(poseComp->Data(), "      ") << std::endl;
  }
}

//////////////////////////////////////////////////
// \brief Print the model links information.
// \param[in] _modelEntity Entity of the model requested.
// \param[in] _ecm ECM ready for requests.
// \param[in] _linkName Link to be printed, if empty, print all links.
void printLinks(const uint64_t _modelEntity,
                const EntityComponentManager &_ecm,
                const std::string &_linkName)
{
  const auto links = _ecm.EntitiesByComponents(
      components::ParentEntity(_modelEntity), components::Link());
  for (const auto &entity : links)
  {
    const auto nameComp = _ecm.Component<components::Name>(entity);

    if (_linkName.length() && _linkName != nameComp->Data())
        continue;

    std::cout << "  - Link [" << entity << "]" << std::endl
              << "    - Name: " << nameComp->Data() << std::endl
              << "    - Parent: " << entityInfo(_modelEntity, _ecm)
              << std::endl;

    const auto inertialComp = _ecm.Component<components::Inertial>(entity);

    if (inertialComp)
    {
      const auto inertialMatrix = inertialComp->Data().MassMatrix();
      const auto mass = inertialComp->Data().MassMatrix().Mass();

      const std::string massInfo = "[" + std::to_string(mass) + "]";
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
      std::cout << "    - Mass (kg): " << massInfo << std::endl
                << "    - Inertial Pose [ XYZ (m) ] [ RPY (rad) ]:"
                << std::endl
                << poseInfo(inertialComp->Data().Pose(), "        ")
                << std::endl
                << "    - Inertial Matrix (kg.m^2):"
                << inertialInfo << std::endl;
    }

    const auto poseComp = _ecm.Component<components::Pose>(entity);
    if (poseComp)
    {
      std::cout << "    - Pose [ XYZ (m) ] [ RPY (rad) ]:" << std::endl
                << poseInfo(poseComp->Data(), "        ") << std::endl;
    }
  }
}

//////////////////////////////////////////////////
// \brief Print the model joints information.
// \param[in] _modelEntity Entity of the model requested.
// \param[in] _ecm ECM ready for requests.
// \param[in] _jointName Joint to be printed, if nullptr, print all joints.
void printJoints(const uint64_t _modelEntity,
                const EntityComponentManager &_ecm,
                const std::string &_jointName)
{
  static const std::map<sdf::JointType, std::string> jointTypes =
  {
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
      components::ParentEntity(_modelEntity), components::Joint());

  for (const auto &entity : joints)
  {
    const auto nameComp = _ecm.Component<components::Name>(entity);

    if (_jointName.length() && _jointName != nameComp->Data())
      continue;

    std::cout << "  - Joint [" << entity << "]" << std::endl
              << "    - Name: " << nameComp->Data() << std::endl
              << "    - Parent: " << entityInfo(_modelEntity, _ecm)
              << std::endl;

    const auto jointTypeComp = _ecm.Component<components::JointType>(entity);
    if (jointTypeComp)
    {
      std::cout << "    - Type: " << jointTypes.at(jointTypeComp->Data())
                << std::endl;
    }

    const auto childLinkComp =
        _ecm.Component<components::ChildLinkName>(entity);
    const auto parentLinkComp =
        _ecm.Component<components::ParentLinkName>(entity);

    if (childLinkComp && parentLinkComp)
    {
      std::cout << "    - Parent Link: "
                << entityInfo(parentLinkComp->Data(), _ecm) << "\n"
                << "    - Child Link: "
                << entityInfo(childLinkComp->Data(), _ecm) << "\n";
    }

    const auto poseComp = _ecm.Component<components::Pose>(entity);
    if (poseComp)
    {
      std::cout << "    - Pose [ XYZ (m) ] [ RPY (rad) ]:" << std::endl
                << poseInfo(poseComp->Data(), "        ") << std::endl;
    }

    const auto axisComp = _ecm.Component<components::JointAxis>(entity);
    if (axisComp)
    {
      std::cout << "    - Axis unit vector [ XYZ ]:\n"
                   "        [" << axisComp->Data().Xyz().X() << " | "
                               << axisComp->Data().Xyz().Y() << " | "
                               << axisComp->Data().Xyz().Z() << "]\n";
    }
  }
}

//////////////////////////////////////////////////
extern "C" IGNITION_GAZEBO_VISIBLE void cmdModelList()
{
  EntityComponentManager ecm{};
  if (!populateECM(ecm))
  {
    return;
  }

  auto world = ecm.EntityByComponents(components::World());
  if (kNullEntity == world)
  {
    std::cout << "No world found." << std::endl;
    return;
  }

  const auto models = ecm.EntitiesByComponents(
    components::ParentEntity(world), components::Model());

  if (models.size() == 0)
  {
    std::cout << "No models in world [" << world << "]" << std::endl;
    return;
  }

  std::cout << "Available models:" << std::endl;

  for (const auto &m : models)
  {
    const auto nameComp = ecm.Component<components::Name>(m);
    std::cout << "    - " << nameComp->Data() << std::endl;
  }
}

//////////////////////////////////////////////////
extern "C" IGNITION_GAZEBO_VISIBLE void cmdModelInfo(
    const char *_modelName, int _pose, const char *_linkName,
    const char *_jointName)
{
  std::string linkName{""};
  if (_linkName)
    linkName = _linkName;
  std::string jointName{""};
  if (_jointName)
    jointName = _jointName;
  bool printAll{false};
  if (!_pose && !_linkName && !_jointName)
    printAll = true;

  if (!_modelName)
  {
    std::cerr << std::endl << "Model name not found" << std::endl;
    return;
  }

  EntityComponentManager ecm{};
  if (!populateECM(ecm))
    return;

  // Get the desired model entity.
  auto entity = ecm.EntityByComponents(components::Name(_modelName),
      components::Model());

  if (entity == kNullEntity)
    std::cout << "No model named <" << _modelName << "> was found" << std::endl;

  // Get the pose of the model
  if (printAll | _pose)
    printModelInfo(entity, ecm);

  // Get the links information
  if (printAll | (_linkName != nullptr))
    printLinks(entity, ecm, linkName);

  // Get the joints information
  if (printAll | (_jointName != nullptr))
    printJoints(entity, ecm, jointName);
}
