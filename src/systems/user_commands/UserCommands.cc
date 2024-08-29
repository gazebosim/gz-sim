/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 * Copyright (C) 2024 CogniPilot Foundation
 * Copyright (C) 2024 Rudis Laboratories LLC
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

#include "UserCommands.hh"

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4251)
#endif

#include <google/protobuf/message.h>

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/entity_factory_v.pb.h>
#include <gz/msgs/light.pb.h>
#include <gz/msgs/material_color.pb.h>
#include <gz/msgs/physics.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/spherical_coordinates.pb.h>
#include <gz/msgs/visual.pb.h>
#include <gz/msgs/wheel_slip_parameters_cmd.pb.h>

#include <string>
#include <utility>
#include <unordered_set>
#include <vector>

#include <gz/math/Color.hh>
#include <gz/math/SphericalCoordinates.hh>
#include <gz/msgs/Utility.hh>

#include <sdf/Physics.hh>
#include <sdf/Root.hh>
#include <sdf/Error.hh>
#include <sdf/Light.hh>

#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/common/Profiler.hh"

#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/Light.hh"
#include "gz/sim/components/LightCmd.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/PoseCmd.hh"
#include "gz/sim/components/PhysicsCmd.hh"
#include "gz/sim/components/SphericalCoordinates.hh"
#include "gz/sim/components/Visual.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/Conversions.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/SdfEntityCreator.hh"
#include "gz/sim/System.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/World.hh"
#include "gz/sim/components/ContactSensorData.hh"
#include "gz/sim/components/ContactSensor.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/VisualCmd.hh"
#include "gz/sim/components/WheelSlipCmd.hh"

using namespace gz;
using namespace sim;
using namespace systems;

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
/// \brief This class is passed to every command and contains interfaces that
/// can be shared among all commands. For example, all create and remove
/// commands can use the `creator` object.
class UserCommandsInterface
{
  /// \brief Pointer to entity component manager. We don't assume ownership.
  public: EntityComponentManager *ecm{nullptr};

  /// \brief Creator interface, shared among all commands that need it.
  public: std::unique_ptr<SdfEntityCreator> creator{nullptr};

  /// \brief World entity.
  public: Entity worldEntity{kNullEntity};

  /// \brief Check if there's a contact sensor connected to a collision
  /// component
  /// \param[in] _collision Collision entity to be checked
  /// \return True if a contact sensor is connected to the collision entity,
  /// false otherwise
  public: bool HasContactSensor(const Entity _collision);

  /// \brief Bool to set all matching light entities.
  public: bool setAllLightEntities = false;
};

/// \brief All user commands should inherit from this class so they can be
/// undone / redone.
class UserCommandBase
{
  /// \brief Constructor
  /// \param[in] _msg Message containing user command.
  /// \param[in] _iface Pointer to interfaces shared by all commands.
  public: UserCommandBase(google::protobuf::Message *_msg,
      std::shared_ptr<UserCommandsInterface> &_iface);

  /// \brief Destructor.
  public: virtual ~UserCommandBase();

  /// \brief Execute the command. All subclasses must implement this
  /// function and update entities and components so the command takes effect.
  /// \return True if command was properly executed.
  public: virtual bool Execute() = 0;

  /// \brief Message containing command.
  protected: google::protobuf::Message *msg{nullptr};

  /// \brief Keep pointer to interfaces shared among commands.
  protected: const std::shared_ptr<UserCommandsInterface> iface{nullptr};
};

/// \brief Command to spawn an entity into simulation.
class CreateCommand : public UserCommandBase
{
  /// \brief Constructor
  /// \param[in] _msg Factory message.
  /// \param[in] _iface Pointer to user commands interface.
  public: CreateCommand(msgs::EntityFactory *_msg,
      std::shared_ptr<UserCommandsInterface> &_iface);

  // Documentation inherited
  public: bool Execute() final;
};

/// \brief Command to remove an entity from simulation.
class RemoveCommand : public UserCommandBase
{
  /// \brief Constructor
  /// \param[in] _msg Message identifying the entity to be removed.
  /// \param[in] _iface Pointer to user commands interface.
  public: RemoveCommand(msgs::Entity *_msg,
      std::shared_ptr<UserCommandsInterface> &_iface);

  // Documentation inherited
  public: bool Execute() final;
};

/// \brief Command to modify a light entity from simulation.
class LightCommand : public UserCommandBase
{
  /// \brief Constructor
  /// \param[in] _msg Message identifying the entity to be edited.
  /// \param[in] _iface Pointer to user commands interface.
  public: LightCommand(msgs::Light *_msg,
      std::shared_ptr<UserCommandsInterface> &_iface);

  // Documentation inherited
  public: bool Execute() final;

  /// \brief Light equality comparison function.
  public: std::function<bool(const msgs::Light &, const msgs::Light &)>
          lightEql { [](const msgs::Light &_a, const msgs::Light &_b)
            {
             return
                _a.is_light_off() == _b.is_light_off() &&
                _a.visualize_visual() == _b.visualize_visual() &&
                _a.type() == _b.type() &&
                _a.name() == _b.name() &&
                math::equal(
                   _a.diffuse().a(), _b.diffuse().a(), 1e-6f) &&
                math::equal(
                  _a.diffuse().r(), _b.diffuse().r(), 1e-6f) &&
                math::equal(
                  _a.diffuse().g(), _b.diffuse().g(), 1e-6f) &&
                math::equal(
                  _a.diffuse().b(), _b.diffuse().b(), 1e-6f) &&
                math::equal(
                  _a.specular().a(), _b.specular().a(), 1e-6f) &&
                math::equal(
                  _a.specular().r(), _b.specular().r(), 1e-6f) &&
                math::equal(
                  _a.specular().g(), _b.specular().g(), 1e-6f) &&
                math::equal(
                  _a.specular().b(), _b.specular().b(), 1e-6f) &&
                math::equal(
                  _a.range(), _b.range(), 1e-6f) &&
               math::equal(
                 _a.attenuation_linear(),
                 _b.attenuation_linear(),
                 1e-6f) &&
               math::equal(
                 _a.attenuation_constant(),
                 _b.attenuation_constant(),
                 1e-6f) &&
               math::equal(
                 _a.attenuation_quadratic(),
                 _b.attenuation_quadratic(),
                 1e-6f) &&
               _a.cast_shadows() == _b.cast_shadows() &&
               math::equal(
                 _a.intensity(),
                 _b.intensity(),
                 1e-6f) &&
               math::equal(
                 _a.direction().x(), _b.direction().x(), 1e-6) &&
               math::equal(
                 _a.direction().y(), _b.direction().y(), 1e-6) &&
               math::equal(
                 _a.direction().z(), _b.direction().z(), 1e-6) &&
               math::equal(
                 _a.spot_inner_angle(), _b.spot_inner_angle(), 1e-6f) &&
               math::equal(
                 _a.spot_outer_angle(), _b.spot_outer_angle(), 1e-6f) &&
               math::equal(_a.spot_falloff(), _b.spot_falloff(), 1e-6f);
            }};
};

/// \brief Command to update an entity's pose transform.
class PoseCommand : public UserCommandBase
{
  /// \brief Constructor
  /// \param[in] _msg pose message.
  /// \param[in] _iface Pointer to user commands interface.
  public: PoseCommand(msgs::Pose *_msg,
      std::shared_ptr<UserCommandsInterface> &_iface);

  // Documentation inherited
  public: bool Execute() final;
};

/// \brief Command to update an entity's pose transform.
class PoseVectorCommand : public UserCommandBase
{
  /// \brief Constructor
  /// \param[in] _msg pose_v message.
  /// \param[in] _iface Pointer to user commands interface.
  public: PoseVectorCommand(msgs::Pose_V *_msg,
      std::shared_ptr<UserCommandsInterface> &_iface);

  // Documentation inherited
  public: bool Execute() final;
};

/// \brief Command to modify the physics parameters of a simulation.
class PhysicsCommand : public UserCommandBase
{
  /// \brief Constructor
  /// \param[in] _msg Message containing the new physics parameters.
  /// \param[in] _iface Pointer to user commands interface.
  public: PhysicsCommand(msgs::Physics *_msg,
      std::shared_ptr<UserCommandsInterface> &_iface);

  // Documentation inherited
  public: bool Execute() final;
};

/// \brief Command to modify the spherical coordinates of a simulation.
class SphericalCoordinatesCommand : public UserCommandBase
{
  /// \brief Constructor
  /// \param[in] _msg Message containing the new coordinates.
  /// \param[in] _iface Pointer to user commands interface.
  public: SphericalCoordinatesCommand(msgs::SphericalCoordinates *_msg,
      std::shared_ptr<UserCommandsInterface> &_iface);

  // Documentation inherited
  public: bool Execute() final;
};

/// \brief Command to enable a collision component.
class EnableCollisionCommand : public UserCommandBase
{
  /// \brief Constructor
  /// \param[in] _msg Message identifying the collision to be enabled.
  /// \param[in] _iface Pointer to user commands interface.
  public: EnableCollisionCommand(msgs::Entity *_msg,
      std::shared_ptr<UserCommandsInterface> &_iface);

  // Documentation inherited
  public: bool Execute() final;
};

/// \brief Command to disable a collision component.
class DisableCollisionCommand : public UserCommandBase
{
  /// \brief Constructor
  /// \param[in] _msg Message identifying the collision to be disabled.
  /// \param[in] _iface Pointer to user commands interface.
  public: DisableCollisionCommand(msgs::Entity *_msg,
      std::shared_ptr<UserCommandsInterface> &_iface);

  // Documentation inherited
  public: bool Execute() final;
};


/// \brief Command to modify a visual entity from simulation.
class VisualCommand : public UserCommandBase
{
  /// \brief Constructor
  /// \param[in] _msg Message containing the visual parameters.
  /// \param[in] _iface Pointer to user commands interface.
  public: VisualCommand(msgs::Visual *_msg,
      std::shared_ptr<UserCommandsInterface> &_iface);

  /// \brief Constructor
  /// \param[in] _msg Message containing the material color parameters.
  /// \param[in] _iface Pointer to user commands interface.
  public: VisualCommand(msgs::MaterialColor *_msg,
      std::shared_ptr<UserCommandsInterface> &_iface);

  // Documentation inherited
  public: bool Execute() final;

  /// \brief Visual equality comparision function
  /// TODO(anyone) Currently only checks for material colors equality,
  /// need to extend to others
  public: std::function<bool(const msgs::Visual &, const msgs::Visual &)>
          visualEql { [](const msgs::Visual &_a, const msgs::Visual &_b)
            {
              auto aMaterial = _a.material(), bMaterial = _b.material();
              return
                _a.name() == _b.name() &&
                _a.id() == _b.id() &&
                math::equal(
                  aMaterial.ambient().r(), bMaterial.ambient().r(), 1e-6f) &&
                math::equal(
                  aMaterial.ambient().g(), bMaterial.ambient().g(), 1e-6f) &&
                math::equal(
                  aMaterial.ambient().b(), bMaterial.ambient().b(), 1e-6f) &&
                math::equal(
                  aMaterial.ambient().a(), bMaterial.ambient().a(), 1e-6f) &&
                math::equal(
                  aMaterial.diffuse().r(), bMaterial.diffuse().r(), 1e-6f) &&
                math::equal(
                  aMaterial.diffuse().g(), bMaterial.diffuse().g(), 1e-6f) &&
                math::equal(
                  aMaterial.diffuse().b(), bMaterial.diffuse().b(), 1e-6f) &&
                math::equal(
                  aMaterial.diffuse().a(), bMaterial.diffuse().a(), 1e-6f) &&
                math::equal(
                  aMaterial.specular().r(), bMaterial.specular().r(), 1e-6f) &&
                math::equal(
                  aMaterial.specular().g(), bMaterial.specular().g(), 1e-6f) &&
                math::equal(
                  aMaterial.specular().b(), bMaterial.specular().b(), 1e-6f) &&
                math::equal(
                  aMaterial.specular().a(), bMaterial.specular().a(), 1e-6f) &&
                math::equal(
                  aMaterial.emissive().r(), bMaterial.emissive().r(), 1e-6f) &&
                math::equal(
                  aMaterial.emissive().g(), bMaterial.emissive().g(), 1e-6f) &&
                math::equal(
                  aMaterial.emissive().b(), bMaterial.emissive().b(), 1e-6f) &&
                math::equal(
                  aMaterial.emissive().a(), bMaterial.emissive().a(), 1e-6f);
            }};
};

/// \brief Command to modify a wheel entity from simulation.
class WheelSlipCommand : public UserCommandBase
{
  /// \brief Constructor
  /// \param[in] _msg Message containing the wheel slip parameters.
  /// \param[in] _iface Pointer to user commands interface.
  public: WheelSlipCommand(msgs::WheelSlipParametersCmd *_msg,
      std::shared_ptr<UserCommandsInterface> &_iface);

  // Documentation inherited
  public: bool Execute() final;

  /// \brief WheelSlip equality comparision function
  public: std::function<bool(
    const msgs::WheelSlipParametersCmd &, const msgs::WheelSlipParametersCmd &)>
          wheelSlipEql {
            [](
              const msgs::WheelSlipParametersCmd &_a,
              const msgs::WheelSlipParametersCmd &_b)
            {
              return
                (
                  (
                    _a.entity().id() != kNullEntity &&
                    _a.entity().id() == _b.entity().id()
                  ) ||
                  (
                    _a.entity().name() == _b.entity().name() &&
                    _a.entity().type() == _b.entity().type()
                  )
                ) &&
                math::equal(
                  _a.slip_compliance_lateral(),
                  _b.slip_compliance_lateral(),
                  1e-6) &&
                math::equal(
                  _a.slip_compliance_longitudinal(),
                  _b.slip_compliance_longitudinal(),
                  1e-6);
            }};
};
}
}
}
}

/// \brief Private UserCommands data class.
class gz::sim::systems::UserCommandsPrivate
{
  /// \brief Callback for create service
  /// \param[in] _req Request containing entity description.
  /// \param[out] _res True if message successfully received and queued.
  /// It does not mean that the entity will be successfully spawned.
  /// \return True if successful.
  public: bool CreateService(const msgs::EntityFactory &_req,
      msgs::Boolean &_res);

  /// \brief Callback for multiple create service
  /// \param[in] _req Request containing one or more entity descriptions.
  /// \param[out] _res True if message successfully received and queued.
  /// It does not mean that the entities will be successfully spawned.
  /// \return True if successful.
  public: bool CreateServiceMultiple(
              const msgs::EntityFactory_V &_req, msgs::Boolean &_res);

  /// \brief Callback for remove service
  /// \param[in] _req Request containing identification of entity to be removed.
  /// \param[out] _res True if message successfully received and queued.
  /// It does not mean that the entity will be successfully removed.
  /// \return True if successful.
  public: bool RemoveService(const msgs::Entity &_req,
      msgs::Boolean &_res);

  /// \brief Callback for light service
  /// \param[in] _req Request containing light update of an entity.
  /// \param[out] _res True if message successfully received and queued.
  /// It does not mean that the light will be successfully updated.
  /// \return True if successful.
  public: bool LightService(const msgs::Light &_req, msgs::Boolean &_res);

  /// \brief Callback for light subscription
  /// \param[in] _msg Light message
  public: void OnCmdLight(const msgs::Light &_msg);

  /// \brief Callback for MaterialColor subscription
  /// \param[in] _msg MaterialColor message
  public: void OnCmdMaterialColor(const msgs::MaterialColor &_msg);

  /// \brief Callback for pose service
  /// \param[in] _req Request containing pose update of an entity.
  /// \param[out] _res True if message successfully received and queued.
  /// It does not mean that the entity will be successfully moved.
  /// \return True if successful.
  public: bool PoseService(const msgs::Pose &_req, msgs::Boolean &_res);

  /// \brief Callback for pose_v service
  /// \param[in] _req Request containing pose update of several entities.
  /// \param[out] _res True if message successfully received and queued.
  /// It does not mean that the entity will be successfully moved.
  /// \return True if successful.
  public: bool PoseVectorService(const msgs::Pose_V &_req, msgs::Boolean &_res);

  /// \brief Callback for physics service
  /// \param[in] _req Request containing updates to the physics parameters.
  /// \param[out] _res True if message successfully received and queued.
  /// It does not mean that the physics parameters will be successfully updated.
  /// \return True if successful.
  public: bool PhysicsService(const msgs::Physics &_req, msgs::Boolean &_res);

  /// \brief Callback for spherical coordinates service
  /// \param[in] _req Request containing updates to the spherical coordinates.
  /// \param[in] _res True if message successfully received and queued.
  /// It does not mean that the physics parameters will be successfully updated.
  /// \return True if successful.
  public: bool SphericalCoordinatesService(
      const msgs::SphericalCoordinates &_req, msgs::Boolean &_res);

  /// \brief Callback for enable collision service
  /// \param[in] _req Request containing collision entity.
  /// \param[out] _res True if message successfully received and queued.
  /// It does not mean that the collision will be successfully enabled.
  /// \return True if successful.
  public: bool EnableCollisionService(
      const msgs::Entity &_req, msgs::Boolean &_res);

  /// \brief Callback for disable collision service
  /// \param[in] _req Request containing collision entity.
  /// \param[out] _res True if message successfully received and queued.
  /// It does not mean that the collision will be successfully disabled.
  /// \return True if successful.
  public: bool DisableCollisionService(
      const msgs::Entity &_req, msgs::Boolean &_res);

  /// \brief Callback for visual service
  /// \param[in] _req Request containing visual updates of an entity
  /// \param[out] _res True if message sucessfully received and queued.
  /// It does not mean that the viusal will be successfully updated
  /// \return True if successful.
  public: bool VisualService(const msgs::Visual &_req, msgs::Boolean &_res);

  /// \brief Callback for wheel slip service
  /// \param[in] _req Request containing wheel slip parameter updates of an
  ///  entity.
  /// \param[out] _res True if message sucessfully received and queued.
  /// It does not mean that the wheel slip parameters will be successfully
  /// updated.
  /// \return True if successful.
  public: bool WheelSlipService(
    const msgs::WheelSlipParametersCmd &_req, msgs::Boolean &_res);

  /// \brief Queue of commands pending execution.
  public: std::vector<std::unique_ptr<UserCommandBase>> pendingCmds;

  /// \brief Gazebo communication node.
  public: transport::Node node;

  /// \brief Object holding several interfaces that can be used by any command.
  public: std::shared_ptr<UserCommandsInterface> iface{nullptr};

  /// \brief Mutex to protect pending queue.
  public: std::mutex pendingMutex;
};

/// \brief Pose3d equality comparison function.
/// \param[in] _a A pose to compare
/// \param[in] _b Another pose to compare
bool pose3Eql(const math::Pose3d &_a, const math::Pose3d &_b)
{
  return _a.Pos().Equal(_b.Pos(), 1e-6) &&
    math::equal(_a.Rot().X(), _b.Rot().X(), 1e-6) &&
    math::equal(_a.Rot().Y(), _b.Rot().Y(), 1e-6) &&
    math::equal(_a.Rot().Z(), _b.Rot().Z(), 1e-6) &&
    math::equal(_a.Rot().W(), _b.Rot().W(), 1e-6);
}

/// \brief Update pose for a specific pose message
/// \param[in] _req Message containing new pose
/// \param[in] _iface Pointer to user commands interface.
/// \return True if successful.
bool updatePose(
  const msgs::Pose &_req,
  std::shared_ptr<UserCommandsInterface> _iface);

//////////////////////////////////////////////////
UserCommands::UserCommands() : System(),
    dataPtr(std::make_unique<UserCommandsPrivate>())
{
}

//////////////////////////////////////////////////
UserCommands::~UserCommands() = default;

//////////////////////////////////////////////////
bool UserCommandsInterface::HasContactSensor(const Entity _collision)
{
  auto *linkEntity = ecm->Component<components::ParentEntity>(_collision);

  if (linkEntity == nullptr)
  {
    return false;
  }

  auto allLinkSensors =
    ecm->EntitiesByComponents(components::Sensor(),
      components::ParentEntity(*linkEntity));

  for (auto const &sensor : allLinkSensors)
  {
    // Check if it is a contact sensor
    auto isContactSensor =
      ecm->EntityHasComponentType(sensor, components::ContactSensor::typeId);
    if (!isContactSensor)
      continue;

    // Check if sensor is connected to _collision
    auto componentName = ecm->Component<components::Name>(_collision);
    std::string collisionName = componentName->Data();
    auto sensorSDF = ecm->Component<components::ContactSensor>(sensor)->Data();
    auto sensorCollisionName =
      sensorSDF->GetElement("contact")->Get<std::string>("collision");

    if (collisionName == sensorCollisionName)
    {
      return true;
    }
  }

  return false;
}

//////////////////////////////////////////////////
void UserCommands::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventManager)
{
  // Create interfaces shared among commands
  this->dataPtr->iface = std::make_shared<UserCommandsInterface>();
  this->dataPtr->iface->worldEntity = _entity;
  this->dataPtr->iface->ecm = &_ecm;
  this->dataPtr->iface->creator =
      std::make_unique<SdfEntityCreator>(_ecm, _eventManager);

  const components::Name *constCmp = _ecm.Component<components::Name>(_entity);
  const std::string &worldName = constCmp->Data();

  auto validWorldName = transport::TopicUtils::AsValidTopic(worldName);
  if (validWorldName.empty())
  {
    gzerr << "World name [" << worldName
           << "] doesn't work well with transport, services not advertised."
           << std::endl;
    return;
  }

  // Create service
  std::string createService{"/world/" + validWorldName + "/create"};
  this->dataPtr->node.Advertise(createService,
      &UserCommandsPrivate::CreateService, this->dataPtr.get());

  // Create service for EntityFactory_V
  std::string createServiceMultiple{"/world/" + validWorldName +
      "/create_multiple"};
  this->dataPtr->node.Advertise(createServiceMultiple,
      &UserCommandsPrivate::CreateServiceMultiple, this->dataPtr.get());

  gzmsg << "Create service on [" << createService << "]" << std::endl;

  // Remove service
  std::string removeService{"/world/" + validWorldName + "/remove"};
  this->dataPtr->node.Advertise(removeService,
      &UserCommandsPrivate::RemoveService, this->dataPtr.get());

  gzmsg << "Remove service on [" << removeService << "]" << std::endl;

  // Pose service
  std::string poseService{"/world/" + validWorldName + "/set_pose"};
  this->dataPtr->node.Advertise(poseService,
      &UserCommandsPrivate::PoseService, this->dataPtr.get());

  gzmsg << "Pose service on [" << poseService << "]" << std::endl;

  // Pose vector service
  std::string poseVectorService{
    "/world/" + validWorldName + "/set_pose_vector"};
  this->dataPtr->node.Advertise(poseVectorService,
      &UserCommandsPrivate::PoseVectorService, this->dataPtr.get());

  gzmsg << "Pose service on [" << poseVectorService << "]" << std::endl;

  // Light service
  std::string lightService{"/world/" + validWorldName + "/light_config"};
  this->dataPtr->node.Advertise(lightService,
      &UserCommandsPrivate::LightService, this->dataPtr.get());

  gzmsg << "Light configuration service on [" << lightService << "]"
    << std::endl;

  std::string lightTopic{"/world/" + validWorldName + "/light_config"};
  this->dataPtr->node.Subscribe(lightTopic, &UserCommandsPrivate::OnCmdLight,
                                this->dataPtr.get());

  if (_sdf->HasElement("set_all_light_entities"))
  {
    this->dataPtr->iface->setAllLightEntities =
      _sdf->Get<bool>("set_all_light_entities");
    gzdbg << "Set all light entities: "
          << this->dataPtr->iface->setAllLightEntities
          << std::endl;
  }

  std::string materialColorTopic{
    "/world/" + validWorldName + "/material_color"};
  this->dataPtr->node.Subscribe(materialColorTopic,
      &UserCommandsPrivate::OnCmdMaterialColor, this->dataPtr.get());

  // Physics service
  std::string physicsService{"/world/" + validWorldName + "/set_physics"};
  this->dataPtr->node.Advertise(physicsService,
      &UserCommandsPrivate::PhysicsService, this->dataPtr.get());

  gzmsg << "Physics service on [" << physicsService << "]" << std::endl;

  // Spherical coordinates service
  std::string sphericalCoordinatesService{"/world/" + validWorldName +
      "/set_spherical_coordinates"};
  this->dataPtr->node.Advertise(sphericalCoordinatesService,
      &UserCommandsPrivate::SphericalCoordinatesService, this->dataPtr.get());

  gzmsg << "SphericalCoordinates service on [" << sphericalCoordinatesService
         << "]" << std::endl;

  // Enable collision service
  std::string enableCollisionService{
    "/world/" + validWorldName + "/enable_collision"};
  this->dataPtr->node.Advertise(enableCollisionService,
      &UserCommandsPrivate::EnableCollisionService, this->dataPtr.get());

  gzmsg << "Enable collision service on [" << enableCollisionService << "]"
    << std::endl;

  // Disable collision service
  std::string disableCollisionService{
    "/world/" + validWorldName + "/disable_collision"};
  this->dataPtr->node.Advertise(disableCollisionService,
      &UserCommandsPrivate::DisableCollisionService, this->dataPtr.get());

  gzmsg << "Disable collision service on [" << disableCollisionService << "]"
    << std::endl;

  // Visual service
  std::string visualService
      {"/world/" + validWorldName + "/visual_config"};
  this->dataPtr->node.Advertise(visualService,
      &UserCommandsPrivate::VisualService, this->dataPtr.get());

  gzmsg << "Material service on [" << visualService << "]" << std::endl;

  // Wheel slip service
  std::string wheelSlipService
      {"/world/" + validWorldName + "/wheel_slip"};
  this->dataPtr->node.Advertise(wheelSlipService,
      &UserCommandsPrivate::WheelSlipService, this->dataPtr.get());

  gzmsg << "Material service on [" << wheelSlipService << "]" << std::endl;
}

//////////////////////////////////////////////////
void UserCommands::PreUpdate(const UpdateInfo &/*_info*/,
    EntityComponentManager &)
{
  GZ_PROFILE("UserCommands::PreUpdate");
  // make a copy the cmds so execution does not block receiving other
  // incoming cmds
  std::vector<std::unique_ptr<UserCommandBase>> cmds;
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->pendingMutex);
    if (this->dataPtr->pendingCmds.empty())
      return;
    cmds = std::move(this->dataPtr->pendingCmds);
    this->dataPtr->pendingCmds.clear();
  }

  // TODO(louise) Record current world state for undo

  // Execute pending commands
  for (auto &cmd : cmds)
  {
    // Execute
    if (!cmd->Execute())
      continue;

    // TODO(louise) Update command with current world state

    // TODO(louise) Move to undo list
  }

  // TODO(louise) Clear redo list
}

//////////////////////////////////////////////////
bool UserCommandsPrivate::CreateServiceMultiple(
    const msgs::EntityFactory_V &_req, msgs::Boolean &_res)
{
  std::lock_guard<std::mutex> lock(this->pendingMutex);
  for (int i = 0; i < _req.data_size(); ++i)
  {
    const msgs::EntityFactory &msg = _req.data(i);
    // Create command and push it to queue
    auto msgCopy = msg.New();
    msgCopy->CopyFrom(msg);
    auto cmd = std::make_unique<CreateCommand>(msgCopy, this->iface);
    this->pendingCmds.push_back(std::move(cmd));
  }

  _res.set_data(true);
  return true;
}

//////////////////////////////////////////////////
bool UserCommandsPrivate::CreateService(const msgs::EntityFactory &_req,
    msgs::Boolean &_res)
{
  // Create command and push it to queue
  auto msg = _req.New();
  msg->CopyFrom(_req);
  auto cmd = std::make_unique<CreateCommand>(msg, this->iface);

  // Push to pending
  {
    std::lock_guard<std::mutex> lock(this->pendingMutex);
    this->pendingCmds.push_back(std::move(cmd));
  }

  _res.set_data(true);
  return true;
}

//////////////////////////////////////////////////
bool UserCommandsPrivate::RemoveService(const msgs::Entity &_req,
    msgs::Boolean &_res)
{
  // Create command and push it to queue
  auto msg = _req.New();
  msg->CopyFrom(_req);
  auto cmd = std::make_unique<RemoveCommand>(msg, this->iface);

  // Push to pending
  {
    std::lock_guard<std::mutex> lock(this->pendingMutex);
    this->pendingCmds.push_back(std::move(cmd));
  }

  _res.set_data(true);
  return true;
}

//////////////////////////////////////////////////
bool UserCommandsPrivate::LightService(const msgs::Light &_req,
    msgs::Boolean &_res)
{
  // Create command and push it to queue
  auto msg = _req.New();
  msg->CopyFrom(_req);
  auto cmd = std::make_unique<LightCommand>(msg, this->iface);

  // Push to pending
  {
    std::lock_guard<std::mutex> lock(this->pendingMutex);
    this->pendingCmds.push_back(std::move(cmd));
  }

  _res.set_data(true);
  return true;
}

//////////////////////////////////////////////////
void UserCommandsPrivate::OnCmdLight(const msgs::Light &_msg)
{
  auto msg = _msg.New();
  msg->CopyFrom(_msg);
  auto cmd = std::make_unique<LightCommand>(msg, this->iface);

  // Push to pending
  {
    std::lock_guard<std::mutex> lock(this->pendingMutex);
    this->pendingCmds.push_back(std::move(cmd));
  }
}

//////////////////////////////////////////////////
bool UserCommandsPrivate::PoseService(const msgs::Pose &_req,
    msgs::Boolean &_res)
{
  // Create command and push it to queue
  auto msg = _req.New();
  msg->CopyFrom(_req);
  auto cmd = std::make_unique<PoseCommand>(msg, this->iface);

  // Push to pending
  {
    std::lock_guard<std::mutex> lock(this->pendingMutex);
    this->pendingCmds.push_back(std::move(cmd));
  }

  _res.set_data(true);
  return true;
}

//////////////////////////////////////////////////
bool UserCommandsPrivate::PoseVectorService(const msgs::Pose_V &_req,
    msgs::Boolean &_res)
{
  // Create command and push it to queue
  auto msg = _req.New();
  msg->CopyFrom(_req);
  auto cmd = std::make_unique<PoseVectorCommand>(msg, this->iface);

  // Push to pending
  {
    std::lock_guard<std::mutex> lock(this->pendingMutex);
    this->pendingCmds.push_back(std::move(cmd));
  }

  _res.set_data(true);
  return true;
}

//////////////////////////////////////////////////
bool UserCommandsPrivate::EnableCollisionService(const msgs::Entity &_req,
    msgs::Boolean &_res)
{
  // Create command and push it to queue
  auto msg = _req.New();
  msg->CopyFrom(_req);
  auto cmd = std::make_unique<EnableCollisionCommand>(msg, this->iface);

  // Push to pending
  {
    std::lock_guard<std::mutex> lock(this->pendingMutex);
    this->pendingCmds.push_back(std::move(cmd));
  }

  _res.set_data(true);
  return true;
}

//////////////////////////////////////////////////
bool UserCommandsPrivate::DisableCollisionService(const msgs::Entity &_req,
    msgs::Boolean &_res)
{
  // Create command and push it to queue
  auto msg = _req.New();
  msg->CopyFrom(_req);
  auto cmd = std::make_unique<DisableCollisionCommand>(msg, this->iface);

  // Push to pending
  {
    std::lock_guard<std::mutex> lock(this->pendingMutex);
    this->pendingCmds.push_back(std::move(cmd));
  }

  _res.set_data(true);
  return true;
}

//////////////////////////////////////////////////
bool UserCommandsPrivate::PhysicsService(const msgs::Physics &_req,
    msgs::Boolean &_res)
{
  // Create command and push it to queue
  auto msg = _req.New();
  msg->CopyFrom(_req);
  auto cmd = std::make_unique<PhysicsCommand>(msg, this->iface);
  // Push to pending
  {
    std::lock_guard<std::mutex> lock(this->pendingMutex);
    this->pendingCmds.push_back(std::move(cmd));
  }

  _res.set_data(true);
  return true;
}

//////////////////////////////////////////////////
bool UserCommandsPrivate::VisualService(const msgs::Visual &_req,
    msgs::Boolean &_res)
{
  // Create command and push it to queue
  auto msg = _req.New();
  msg->CopyFrom(_req);
  auto cmd = std::make_unique<VisualCommand>(msg, this->iface);
  // Push to pending
  {
    std::lock_guard<std::mutex> lock(this->pendingMutex);
    this->pendingCmds.push_back(std::move(cmd));
  }

  _res.set_data(true);
  return true;
}

//////////////////////////////////////////////////
void UserCommandsPrivate::OnCmdMaterialColor(const msgs::MaterialColor &_msg)
{
  auto msg = _msg.New();
  msg->CopyFrom(_msg);
  auto cmd = std::make_unique<VisualCommand>(msg, this->iface);
  // Push to pending
  {
    std::lock_guard<std::mutex> lock(this->pendingMutex);
    this->pendingCmds.push_back(std::move(cmd));
  }

  return;
}

//////////////////////////////////////////////////
bool UserCommandsPrivate::WheelSlipService(
    const msgs::WheelSlipParametersCmd &_req,
    msgs::Boolean &_res)
{
  // Create command and push it to queue
  auto msg = _req.New();
  msg->CopyFrom(_req);
  auto cmd = std::make_unique<WheelSlipCommand>(msg, this->iface);
  // Push to pending
  {
    std::lock_guard<std::mutex> lock(this->pendingMutex);
    this->pendingCmds.push_back(std::move(cmd));
  }

  _res.set_data(true);
  return true;
}

//////////////////////////////////////////////////
bool UserCommandsPrivate::SphericalCoordinatesService(
    const msgs::SphericalCoordinates &_req, msgs::Boolean &_res)
{
  // Create command and push it to queue
  auto msg = _req.New();
  msg->CopyFrom(_req);
  auto cmd = std::make_unique<SphericalCoordinatesCommand>(msg, this->iface);
  // Push to pending
  {
    std::lock_guard<std::mutex> lock(this->pendingMutex);
    this->pendingCmds.push_back(std::move(cmd));
  }

  _res.set_data(true);
  return true;
}

//////////////////////////////////////////////////
UserCommandBase::UserCommandBase(google::protobuf::Message *_msg,
    std::shared_ptr<UserCommandsInterface> &_iface)
    : msg(_msg), iface(_iface)
{
}

//////////////////////////////////////////////////
UserCommandBase::~UserCommandBase()
{
  if (this->msg != nullptr)
    delete this->msg;
  this->msg = nullptr;
}

//////////////////////////////////////////////////
CreateCommand::CreateCommand(msgs::EntityFactory *_msg,
    std::shared_ptr<UserCommandsInterface> &_iface)
    : UserCommandBase(_msg, _iface)
{
}

//////////////////////////////////////////////////
bool CreateCommand::Execute()
{
  auto createMsg = dynamic_cast<const msgs::EntityFactory *>(this->msg);
  if (nullptr == createMsg)
  {
    gzerr << "Internal error, null create message" << std::endl;
    return false;
  }

  // Load SDF
  sdf::Root root;
  sdf::Light lightSdf;
  sdf::Errors errors;
  switch (createMsg->from_case())
  {
    case msgs::EntityFactory::kSdf:
    {
      errors = root.LoadSdfString(createMsg->sdf());
      break;
    }
    case msgs::EntityFactory::kSdfFilename:
    {
      errors = root.Load(createMsg->sdf_filename());
      break;
    }
    case msgs::EntityFactory::kModel:
    {
      // TODO(louise) Support model msg
      gzerr << "model field not yet supported." << std::endl;
      return false;
    }
    case msgs::EntityFactory::kLight:
    {
      lightSdf = convert<sdf::Light>(createMsg->light());
      break;
    }
    case msgs::EntityFactory::kCloneName:
    {
      auto validClone = false;
      auto clonedEntity = kNullEntity;
      auto entityToClone = this->iface->ecm->EntityByComponents(
          components::Name(createMsg->clone_name()));
      if (kNullEntity != entityToClone)
      {
        auto parentComp =
          this->iface->ecm->Component<components::ParentEntity>(entityToClone);

        // TODO(anyone) add better support for creating non-top level entities.
        // For now, we will only clone top level entities
        if (parentComp && parentComp->Data() == this->iface->worldEntity)
        {
          auto parentEntity = parentComp->Data();
          clonedEntity = this->iface->ecm->Clone(entityToClone,
              parentEntity, createMsg->name(), createMsg->allow_renaming());
          validClone = kNullEntity != clonedEntity;
        }
      }

      if (!validClone)
      {
        gzerr << "Request to clone an entity named ["
          << createMsg->clone_name() << "] failed." << std::endl;
        return false;
      }

      if (createMsg->has_pose())
      {
        // TODO(anyone) handle if relative_to is filled
        auto pose = sim::convert<math::Pose3d>(createMsg->pose());
        this->iface->ecm->SetComponentData<components::Pose>(clonedEntity,
            pose);
      }
      return true;
    }
    default:
    {
      gzerr << "Missing [from] field in create message." << std::endl;
      return false;
    }
  }

  if (!errors.empty())
  {
    for (auto &err : errors)
      gzerr << err << std::endl;
    return false;
  }

  bool isModel{false};
  bool isLight{false};
  bool isActor{false};
  bool isRoot{false};
  if (nullptr != root.Model())
  {
    isRoot = true;
    isModel = true;
  }
  else if (nullptr != root.Light())
  {
    isRoot = true;
    isLight = true;
  }
  else if (nullptr != root.Actor())
  {
    isRoot = true;
    isActor = true;
  }
  else if (!lightSdf.Name().empty())
  {
    isLight = true;
  }
  else
  {
    gzerr << "Expected exactly one top-level <model>, <light> or <actor> on"
           << " SDF." << std::endl;
    return false;
  }

  if ((isModel && isLight) || (isModel && isActor) || (isLight && isActor))
  {
    gzwarn << "Expected exactly one top-level <model>, <light> or <actor>, "
            << "but found more. Only the 1st will be spawned." << std::endl;
  }

  // Check the name of the entity being spawned
  std::string desiredName;
  if (!createMsg->name().empty())
  {
    desiredName = createMsg->name();
  }
  else if (isModel)
  {
    desiredName = root.Model()->Name();
  }
  else if (isLight && isRoot)
  {
    desiredName = root.Light()->Name();
  }
  else if (isLight)
  {
    desiredName = lightSdf.Name();
  }
  else if (isActor)
  {
    desiredName = root.Actor()->Name();
  }

  // Check if there's already a top-level entity with the given name
  if (kNullEntity != this->iface->ecm->EntityByComponents(
      components::Name(desiredName),
      components::ParentEntity(this->iface->worldEntity)))
  {
    if (!createMsg->allow_renaming())
    {
      gzwarn << "Entity named [" << desiredName << "] already exists and "
              << "[allow_renaming] is false. Entity not spawned."
              << std::endl;
      return false;
    }

    // Generate unique name
    std::string newName = desiredName;
    int i = 0;
    while (kNullEntity != this->iface->ecm->EntityByComponents(
      components::Name(newName),
      components::ParentEntity(this->iface->worldEntity)))
    {
      newName = desiredName + "_" + std::to_string(i++);
    }
    desiredName = newName;
  }

  // Create entities
  Entity entity{kNullEntity};
  if (isModel)
  {
    auto model = *root.Model();
    model.SetName(desiredName);
    entity = this->iface->creator->CreateEntities(&model);
  }
  else if (isLight && isRoot)
  {
    auto light = *root.Light();
    light.SetName(desiredName);
    entity = this->iface->creator->CreateEntities(&light);
  }
  else if (isLight)
  {
    lightSdf.SetName(desiredName);
    entity = this->iface->creator->CreateEntities(&lightSdf);
  }
  else if (isActor)
  {
    auto actor = *root.Actor();
    actor.SetName(desiredName);
    entity = this->iface->creator->CreateEntities(&actor);
  }

  this->iface->creator->SetParent(entity, this->iface->worldEntity);

  // Pose
  std::optional<math::Pose3d> createPose;
  if (createMsg->has_pose())
  {
    createPose = msgs::Convert(createMsg->pose());
  }

  // Spherical coordinates
  if (createMsg->has_spherical_coordinates())
  {
    gzerr << "HasSphericalCoordinates" << std::endl;
    auto scComp = this->iface->ecm->Component<components::SphericalCoordinates>(
        this->iface->worldEntity);
    if (nullptr == scComp)
    {
      gzwarn << "Trying to create entity [" << desiredName
              << "] with spherical coordinates, but world's spherical "
              << "coordinates aren't set. Entity will be created at the world "
              << "origin." << std::endl;
    }
    else
    {
      // deg to rad
      math::Vector3d latLonEle{
          GZ_DTOR(createMsg->spherical_coordinates().latitude_deg()),
          GZ_DTOR(createMsg->spherical_coordinates().longitude_deg()),
          createMsg->spherical_coordinates().elevation()};

      auto pos = scComp->Data().PositionTransform(latLonEle,
          math::SphericalCoordinates::SPHERICAL,
          math::SphericalCoordinates::LOCAL2);

      // Override pos and add to yaw
      if (!createPose.has_value())
        createPose = math::Pose3d::Zero;
      createPose.value().SetX(pos.X());
      createPose.value().SetY(pos.Y());
      createPose.value().SetZ(pos.Z());
      createPose.value().Rot() = math::Quaterniond(0, 0,
          GZ_DTOR(createMsg->spherical_coordinates().heading_deg())) *
          createPose.value().Rot();
    }
  }

  if (createPose.has_value())
  {
    auto poseComp = this->iface->ecm->Component<components::Pose>(entity);
    *poseComp = components::Pose(createPose.value());
  }

  gzdbg << "Created entity [" << entity << "] named [" << desiredName << "]"
         << std::endl;

  return true;
}

//////////////////////////////////////////////////
RemoveCommand::RemoveCommand(msgs::Entity *_msg,
    std::shared_ptr<UserCommandsInterface> &_iface)
    : UserCommandBase(_msg, _iface)
{
}

//////////////////////////////////////////////////
bool RemoveCommand::Execute()
{
  auto removeMsg = dynamic_cast<const msgs::Entity *>(this->msg);
  if (nullptr == removeMsg)
  {
    gzerr << "Internal error, null remove message" << std::endl;
    return false;
  }

  auto entity = entityFromMsg(*this->iface->ecm, *removeMsg);
  if (entity == kNullEntity)
  {
    gzerr << "Entity named [" << removeMsg->name() << "] of type ["
           << removeMsg->type() << "] not found, so not removed." << std::endl;
    return false;
  }

  // Check that we support removing this entity
  auto parent = this->iface->ecm->ParentEntity(entity);
  if (nullptr == this->iface->ecm->Component<components::World>(parent))
  {
    gzerr << "Entity [" << entity
           << "] is not a direct child of the world, so it can't be removed."
           << std::endl;
    return false;
  }

  if (nullptr == this->iface->ecm->Component<components::Model>(entity) &&
      nullptr == this->iface->ecm->Component<components::Light>(entity))
  {
    gzerr << "Entity [" << entity
           << "] is not a model or a light, so it can't be removed."
           << std::endl;
    return false;
  }

  gzdbg << "Requesting removal of entity [" << entity << "]" << std::endl;
  this->iface->creator->RequestRemoveEntity(entity);
  return true;
}

//////////////////////////////////////////////////
LightCommand::LightCommand(msgs::Light *_msg,
    std::shared_ptr<UserCommandsInterface> &_iface)
    : UserCommandBase(_msg, _iface)
{
}

//////////////////////////////////////////////////
bool LightCommand::Execute()
{
  auto lightMsg = dynamic_cast<msgs::Light *>(this->msg);
  if (nullptr == lightMsg)
  {
    gzerr << "Internal error, null light message" << std::endl;
    return false;
  }
  Entity lightEntity = kNullEntity;
  if (this->iface->setAllLightEntities)
  {
    auto entities = entitiesFromScopedName(lightMsg->name(),
        *this->iface->ecm);
    if (entities.empty())
    {
      gzwarn << "Entity name: " << lightMsg->name() << ", is not found."
             << std::endl;
      return false;
    }
    for (const Entity &id : entities)
    {
      lightEntity = kNullEntity;
      lightMsg->set_id(id);
      if (lightMsg->id() != kNullEntity)
      {
        lightEntity = lightMsg->id();
      }
      if (!lightEntity)
      {
        gzmsg << "Failed to find light entity named [" << lightMsg->name()
          << "]." << std::endl;
        return false;
      }

      auto lightPose =
        this->iface->ecm->Component<components::Pose>(lightEntity);
      if (nullptr == lightPose)
      {
        lightEntity = kNullEntity;
      }

      if (!lightEntity)
      {
        gzmsg << "Pose component not available" << std::endl;
        return false;
      }

      if (lightMsg->has_pose())
      {
        lightPose->Data().Pos() = msgs::Convert(lightMsg->pose()).Pos();
      }

      auto lightCmdComp =
        this->iface->ecm->Component<components::LightCmd>(lightEntity);
      if (!lightCmdComp)
      {
        this->iface->ecm->CreateComponent(
            lightEntity, components::LightCmd(*lightMsg));
      }
      else
      {
        auto state = lightCmdComp->SetData(*lightMsg, this->lightEql) ?
            ComponentState::OneTimeChange :
            ComponentState::NoChange;
        this->iface->ecm->SetChanged(lightEntity, components::LightCmd::typeId,
          state);
      }
    }
  }
  else
  {
    lightEntity = kNullEntity;
    if (lightMsg->id() != kNullEntity)
    {
      lightEntity = lightMsg->id();
    }
    else if (!lightMsg->name().empty())
    {
      if (lightMsg->parent_id() != kNullEntity)
      {
        lightEntity = this->iface->ecm->EntityByComponents(
          components::Name(lightMsg->name()),
          components::ParentEntity(lightMsg->parent_id()));
      }
      else
      {
        lightEntity = this->iface->ecm->EntityByComponents(
          components::Name(lightMsg->name()));
      }
    }
    if (kNullEntity == lightEntity)
    {
      gzerr << "Failed to find light with name [" << lightMsg->name()
             << "], ID [" << lightMsg->id() << "] and parent ID ["
             << lightMsg->parent_id() << "]." << std::endl;
      return false;
    }

    if (!lightEntity)
    {
      gzmsg << "Failed to find light entity named [" << lightMsg->name()
        << "]." << std::endl;
      return false;
    }

    auto lightPose = this->iface->ecm->Component<components::Pose>(lightEntity);
    if (nullptr == lightPose)
    {
      lightEntity = kNullEntity;
    }

    if (!lightEntity)
    {
      gzmsg << "Pose component not available" << std::endl;
      return false;
    }

    if (lightMsg->has_pose())
    {
      lightPose->Data().Pos() = msgs::Convert(lightMsg->pose()).Pos();
    }

    auto lightCmdComp =
      this->iface->ecm->Component<components::LightCmd>(lightEntity);
    if (!lightCmdComp)
    {
      this->iface->ecm->CreateComponent(
          lightEntity, components::LightCmd(*lightMsg));
    }
    else
    {
      auto state = lightCmdComp->SetData(*lightMsg, this->lightEql) ?
          ComponentState::OneTimeChange :
          ComponentState::NoChange;
      this->iface->ecm->SetChanged(lightEntity, components::LightCmd::typeId,
        state);
    }
  }
  return true;
}

//////////////////////////////////////////////////
bool updatePose(
  const msgs::Pose &_poseMsg,
  std::shared_ptr<UserCommandsInterface> _iface)
{
  // Check the name of the entity being spawned
  std::string entityName = _poseMsg.name();
  Entity entity = kNullEntity;
  // TODO(anyone) Update pose message to use Entity, with default ID null
  if (_poseMsg.id() != kNullEntity && _poseMsg.id() != 0)
  {
    entity = _poseMsg.id();
  }
  else if (!entityName.empty())
  {
    entity = _iface->ecm->EntityByComponents(components::Name(entityName),
      components::ParentEntity(_iface->worldEntity));
  }

  if (!_iface->ecm->HasEntity(entity))
  {
    gzerr << "Unable to update the pose for entity id:[" << _poseMsg.id()
           << "], name[" << entityName << "]" << std::endl;
    return false;
  }

  auto poseCmdComp =
    _iface->ecm->Component<components::WorldPoseCmd>(entity);
  if (!poseCmdComp)
  {
    _iface->ecm->CreateComponent(
        entity, components::WorldPoseCmd(msgs::Convert(_poseMsg)));
  }
  else
  {
    /// \todo(anyone) Moving an object is not captured in a log file.
    auto state = poseCmdComp->SetData(msgs::Convert(_poseMsg), pose3Eql) ?
        ComponentState::OneTimeChange :
        ComponentState::NoChange;
    _iface->ecm->SetChanged(entity, components::WorldPoseCmd::typeId,
        state);
  }
  return true;
}

//////////////////////////////////////////////////
PoseCommand::PoseCommand(msgs::Pose *_msg,
    std::shared_ptr<UserCommandsInterface> &_iface)
    : UserCommandBase(_msg, _iface)
{
}

//////////////////////////////////////////////////
bool PoseCommand::Execute()
{
  auto poseMsg = dynamic_cast<const msgs::Pose *>(this->msg);
  if (nullptr == poseMsg)
  {
    gzerr << "Internal error, null create message" << std::endl;
    return false;
  }

  return updatePose(*poseMsg, this->iface);
}

//////////////////////////////////////////////////
PoseVectorCommand::PoseVectorCommand(msgs::Pose_V *_msg,
    std::shared_ptr<UserCommandsInterface> &_iface)
    : UserCommandBase(_msg, _iface)
{
}

//////////////////////////////////////////////////
bool PoseVectorCommand::Execute()
{
  auto poseVectorMsg = dynamic_cast<const msgs::Pose_V *>(this->msg);
  if (nullptr == poseVectorMsg)
  {
    gzerr << "Internal error, null create message" << std::endl;
    return false;
  }

  for (int i = 0; i < poseVectorMsg->pose_size(); i++)
  {
    if (!updatePose(poseVectorMsg->pose(i), this->iface))
    {
      return false;
    }
  }

  return true;
}

//////////////////////////////////////////////////
PhysicsCommand::PhysicsCommand(msgs::Physics *_msg,
    std::shared_ptr<UserCommandsInterface> &_iface)
    : UserCommandBase(_msg, _iface)
{
}

//////////////////////////////////////////////////
bool PhysicsCommand::Execute()
{
  auto physicsMsg = dynamic_cast<const msgs::Physics *>(this->msg);
  if (nullptr == physicsMsg)
  {
    gzerr << "Internal error, null physics message" << std::endl;
    return false;
  }

  auto worldEntity = this->iface->ecm->EntityByComponents(components::World());
  if (worldEntity == kNullEntity)
  {
    gzmsg << "Failed to find world entity" << std::endl;
    return false;
  }

  if (!this->iface->ecm->EntityHasComponentType(worldEntity,
    components::PhysicsCmd().TypeId()))
  {
    this->iface->ecm->CreateComponent(worldEntity,
        components::PhysicsCmd(*physicsMsg));
  }

  return true;
}

//////////////////////////////////////////////////
SphericalCoordinatesCommand::SphericalCoordinatesCommand(
    msgs::SphericalCoordinates *_msg,
    std::shared_ptr<UserCommandsInterface> &_iface)
    : UserCommandBase(_msg, _iface)
{
}

//////////////////////////////////////////////////
bool SphericalCoordinatesCommand::Execute()
{
  auto sphericalCoordinatesMsg =
      dynamic_cast<const msgs::SphericalCoordinates *>(this->msg);
  if (nullptr == sphericalCoordinatesMsg)
  {
    gzerr << "Internal error, null SphericalCoordinates message" << std::endl;
    return false;
  }

  // World
  if (!sphericalCoordinatesMsg->has_entity())
  {
    World world(this->iface->worldEntity);
    world.SetSphericalCoordinates(*this->iface->ecm,
        msgs::Convert(*sphericalCoordinatesMsg));
    return true;
  }

  // Entity
  auto entity = entityFromMsg(*this->iface->ecm,
      sphericalCoordinatesMsg->entity());

  if (!this->iface->ecm->HasEntity(entity))
  {
    gzerr << "Unable to update the pose for entity [" << entity
           << "]: entity doesn't exist." << std::endl;
    return false;
  }

  auto scComp = this->iface->ecm->Component<components::SphericalCoordinates>(
      this->iface->worldEntity);
  if (nullptr == scComp)
  {
    gzerr << "Trying to move entity [" << entity
           << "] using spherical coordinates, but world's spherical "
           << "coordinates aren't set." << std::endl;
    return false;
  }

  // deg to rad
  math::Vector3d latLonEle{
      GZ_DTOR(sphericalCoordinatesMsg->latitude_deg()),
      GZ_DTOR(sphericalCoordinatesMsg->longitude_deg()),
      sphericalCoordinatesMsg->elevation()};

  auto pos = scComp->Data().PositionTransform(latLonEle,
      math::SphericalCoordinates::SPHERICAL,
      math::SphericalCoordinates::LOCAL2);

  math::Pose3d pose{pos.X(), pos.Y(), pos.Z(), 0, 0,
          GZ_DTOR(sphericalCoordinatesMsg->heading_deg())};

  auto poseCmdComp =
    this->iface->ecm->Component<components::WorldPoseCmd>(entity);
  if (!poseCmdComp)
  {
    this->iface->ecm->CreateComponent(entity, components::WorldPoseCmd(pose));
  }
  else
  {
    auto state = poseCmdComp->SetData(pose, pose3Eql) ?
        ComponentState::OneTimeChange :
        ComponentState::NoChange;
    this->iface->ecm->SetChanged(entity, components::WorldPoseCmd::typeId,
        state);
  }

  return true;
}

//////////////////////////////////////////////////
EnableCollisionCommand::EnableCollisionCommand(msgs::Entity *_msg,
    std::shared_ptr<UserCommandsInterface> &_iface)
    : UserCommandBase(_msg, _iface)
{
}

//////////////////////////////////////////////////
bool EnableCollisionCommand::Execute()
{
  auto entityMsg = dynamic_cast<const msgs::Entity *>(this->msg);
  if (nullptr == entityMsg)
  {
    gzerr << "Internal error, null create message" << std::endl;
    return false;
  }

  // Check Entity type
  if (entityMsg->type() != msgs::Entity::COLLISION)
  {
    gzwarn << "Expected msgs::Entity::Type::COLLISION, exiting service..."
      << std::endl;
    return false;
  }

  // Check if collision is connected to a contact sensor
  if (this->iface->HasContactSensor(entityMsg->id()))
  {
    gzwarn << "Requested collision is connected to a contact sensor, "
      << "exiting service..." << std::endl;
    return false;
  }

  // Create ContactSensorData component
  auto contactDataComp =
    this->iface->ecm->Component<
      components::ContactSensorData>(entityMsg->id());
  if (contactDataComp)
  {
    gzwarn << "Can't create component that already exists" << std::endl;
    return false;
  }

  this->iface->ecm->
    CreateComponent(entityMsg->id(), components::ContactSensorData());
  gzdbg << "Enabled collision [" << entityMsg->id() << "]" << std::endl;

  return true;
}

//////////////////////////////////////////////////
DisableCollisionCommand::DisableCollisionCommand(msgs::Entity *_msg,
    std::shared_ptr<UserCommandsInterface> &_iface)
    : UserCommandBase(_msg, _iface)
{
}

//////////////////////////////////////////////////
bool DisableCollisionCommand::Execute()
{
  auto entityMsg = dynamic_cast<const msgs::Entity *>(this->msg);
  if (nullptr == entityMsg)
  {
    gzerr << "Internal error, null create message" << std::endl;
    return false;
  }

  // Check Entity type
  if (entityMsg->type() != msgs::Entity::COLLISION)
  {
    gzwarn << "Expected msgs::Entity::Type::COLLISION, exiting service..."
      << std::endl;
    return false;
  }

  // Check if collision is connected to a contact sensor
  if (this->iface->HasContactSensor(entityMsg->id()))
  {
    gzwarn << "Requested collision is connected to a contact sensor, "
      << "exiting service..." << std::endl;
    return false;
  }

  // Remove ContactSensorData component
  auto *contactDataComp =
    this->iface->ecm->Component<
      components::ContactSensorData>(entityMsg->id());
  if (!contactDataComp)
  {
    gzwarn << "No ContactSensorData detected inside entity " << entityMsg->id()
      << std::endl;
    return false;
  }

  this->iface->ecm->
    RemoveComponent(entityMsg->id(), components::ContactSensorData::typeId);

  gzdbg << "Disabled collision [" << entityMsg->id() << "]" << std::endl;

  return true;
}

//////////////////////////////////////////////////
VisualCommand::VisualCommand(msgs::Visual *_msg,
    std::shared_ptr<UserCommandsInterface> &_iface)
    : UserCommandBase(_msg, _iface)
{
}

//////////////////////////////////////////////////
VisualCommand::VisualCommand(msgs::MaterialColor *_msg,
    std::shared_ptr<UserCommandsInterface> &_iface)
    : UserCommandBase(_msg, _iface)
{
}

//////////////////////////////////////////////////
bool VisualCommand::Execute()
{
  auto visualMsg = dynamic_cast<const msgs::Visual *>(this->msg);
  auto materialColorMsg = dynamic_cast<const msgs::MaterialColor *>(this->msg);
  if (visualMsg != nullptr)
  {
    Entity visualEntity = kNullEntity;
    if (visualMsg->id() != kNullEntity)
    {
      visualEntity = visualMsg->id();
    }
    else if (!visualMsg->name().empty() && !visualMsg->parent_name().empty())
    {
      Entity parentEntity =
        this->iface->ecm->EntityByComponents(
          components::Name(visualMsg->parent_name()));

      auto entities =
        this->iface->ecm->ChildrenByComponents(parentEntity,
          components::Name(visualMsg->name()));

      // When size > 1, we don't know which entity to modify
      if (entities.size() == 1)
      {
        visualEntity = entities[0];
      }
    }

    if (visualEntity == kNullEntity)
    {
      gzerr << "Failed to find visual entity" << std::endl;
      return false;
    }

    auto visualCmdComp =
        this->iface->ecm->Component<components::VisualCmd>(visualEntity);
    if (!visualCmdComp)
    {
      this->iface->ecm->CreateComponent(
          visualEntity, components::VisualCmd(*visualMsg));
    }
    else
    {
      auto state = visualCmdComp->SetData(*visualMsg, this->visualEql) ?
          ComponentState::OneTimeChange : ComponentState::NoChange;
      this->iface->ecm->SetChanged(
          visualEntity, components::VisualCmd::typeId, state);
    }
  }
  else if (materialColorMsg != nullptr)
  {
    Entity visualEntity = kNullEntity;
    int numberOfEntities = 0;
    auto entities = entitiesFromScopedName(materialColorMsg->entity().name(),
      *this->iface->ecm);
    if (entities.empty())
    {
      gzwarn << "Entity name: " << materialColorMsg->entity().name()
             << ", is not found."
             << std::endl;
      return false;
    }
    for (const Entity &id : entities)
    {
      if ((numberOfEntities > 0) && (materialColorMsg->entity_match() !=
        gz::msgs::MaterialColor::EntityMatch::MaterialColor_EntityMatch_ALL))
      {
        return true;
      }
      numberOfEntities++;
      msgs::Visual visualMCMsg;
      visualMCMsg.set_id(id);
      visualMCMsg.mutable_material()->mutable_ambient()->CopyFrom(
        materialColorMsg->ambient());
      visualMCMsg.mutable_material()->mutable_diffuse()->CopyFrom(
        materialColorMsg->diffuse());
      visualMCMsg.mutable_material()->mutable_specular()->CopyFrom(
        materialColorMsg->specular());
      visualMCMsg.mutable_material()->mutable_emissive()->CopyFrom(
        materialColorMsg->emissive());
      // TODO(anyone) Enable setting shininess
      visualEntity = kNullEntity;
      if (visualMCMsg.id() != kNullEntity)
      {
        visualEntity = visualMCMsg.id();
      }
      if (visualEntity == kNullEntity)
      {
        gzerr << "Failed to find visual entity" << std::endl;
        return false;
      }

      auto visualCmdComp =
          this->iface->ecm->Component<components::VisualCmd>(visualEntity);
      if (!visualCmdComp)
      {
        this->iface->ecm->CreateComponent(
            visualEntity, components::VisualCmd(visualMCMsg));
      }
      else
      {
        auto state = visualCmdComp->SetData(visualMCMsg, this->visualEql) ?
            ComponentState::OneTimeChange : ComponentState::NoChange;
        this->iface->ecm->SetChanged(
            visualEntity, components::VisualCmd::typeId, state);
      }
    }
  }
  else
  {
    gzerr <<
      "VisualCommand _msg does not match MaterialColor or Visual msg type."
      << std::endl;
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
WheelSlipCommand::WheelSlipCommand(msgs::WheelSlipParametersCmd *_msg,
    std::shared_ptr<UserCommandsInterface> &_iface)
    : UserCommandBase(_msg, _iface)
{
}

//////////////////////////////////////////////////
bool WheelSlipCommand::Execute()
{
  auto wheelSlipMsg = dynamic_cast<const msgs::WheelSlipParametersCmd *>(
      this->msg);
  if (nullptr == wheelSlipMsg)
  {
    gzerr << "Internal error, null wheel slip message" << std::endl;
    return false;
  }
  const auto & ecm = *this->iface->ecm;
  Entity entity = entityFromMsg(ecm, wheelSlipMsg->entity());
  if (kNullEntity == entity)
  {
    return false;
  }

  auto doForEachLink = [this, wheelSlipMsg](Entity linkEntity) {
    auto wheelSlipCmdComp =
      this->iface->ecm->Component<components::WheelSlipCmd>(linkEntity);
    if (!wheelSlipCmdComp)
    {
      this->iface->ecm->CreateComponent(
          linkEntity, components::WheelSlipCmd(*wheelSlipMsg));
    }
    else
    {
      auto state = wheelSlipCmdComp->SetData(
        *wheelSlipMsg, this->wheelSlipEql) ? ComponentState::OneTimeChange
        : ComponentState::NoChange;
      this->iface->ecm->SetChanged(
          linkEntity, components::WheelSlipCmd::typeId, state);
    }
  };
  const components::BaseComponent * component =
    ecm.Component<components::Link>(entity);

  if (nullptr != component) {
    doForEachLink(entity);
    return true;
  }
  component = ecm.Component<components::Model>(entity);
  if (nullptr != component) {
    Model model{entity};
    for (const auto & linkEntity : model.Links(*this->iface->ecm)) {
      doForEachLink(linkEntity);
    }
    return true;
  }
  gzerr << "Found entity with scoped name [" << wheelSlipMsg->entity().name()
          << "], is neither a model or a link." << std::endl;
  return false;
}

GZ_ADD_PLUGIN(UserCommands, System,
  UserCommands::ISystemConfigure,
  UserCommands::ISystemPreUpdate
)

GZ_ADD_PLUGIN_ALIAS(UserCommands,
                          "gz::sim::systems::UserCommands")
