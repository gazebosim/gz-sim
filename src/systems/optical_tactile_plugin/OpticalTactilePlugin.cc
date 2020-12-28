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

#include <algorithm>
#include <optional>
#include <string>
#include <vector>

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/common/Time.hh>

#include <sdf/Element.hh>

#include "ignition/gazebo/components/ContactSensor.hh"
#include "ignition/gazebo/components/ContactSensorData.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/DepthCamera.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "sdf/Box.hh"

#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"

#include "OpticalTactilePlugin.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;
using namespace optical_tactile_sensor;

class ignition::gazebo::systems::OpticalTactilePluginPrivate
{
  /// \brief Load the Contact sensor from an sdf element
  /// \param[in] _ecm Immutable reference to the EntityComponentManager
  public: void Load(const EntityComponentManager &_ecm);

  /// \brief Actual function that enables the plugin.
  /// \param[in] _value True to enable plugin.
  public: void Enable(const bool _value);

  /// \brief Callback for the depth camera
  /// \param[in] _msg Message from the subscribed topic
  public: void DepthCameraCallback(
    const ignition::msgs::PointCloudPacked &_msg);

  /// \brief Map the (i,j) coordinates defined in the top-left corner of the
  /// camera into its corresponding (X,Y,Z) measurement with respect to the
  /// camera's origin in the world frame
  /// \param[in] _i Horizontal camera coordinate defined in the top-left corner
  /// of the image, pointing rightwards
  /// \param[in] _j Vertical camera coordinate defined in the top-left corner
  /// of the image, pointing downwards
  /// \param[in] _msgBuffer Buffer with the point cloud data
  /// \returns The corresponding (X,Y,Z) point
  public: ignition::math::Vector3f MapPointCloudData(const uint64_t &_i,
    const uint64_t &_j, const char *_msgBuffer);

  /// \brief Check if a specific point from the depth camera is inside
  /// the contact surface.
  /// \param[in] _point Point from the depth camera
  public: bool PointInsideSensor(ignition::math::Vector3f _point);

  /// \brief Computes the normal forces of the Optical Tactile sensor
  /// \param[in] _msg Message from the depth camera
  /// \param[in] _visualizeForces Whether to visualize the forces or not
  ///
  /// Implementation inspired by
  /// https://stackoverflow.com/questions/
  /// 34644101/calculate-surface-normals-from-depth-image-
  /// using-neighboring-pixels-cross-produc
  public: void ComputeNormalForces(
    const ignition::msgs::PointCloudPacked &_msg,
    const bool _visualizeForces);

  /// \brief Resolution of the visualization in pixels to skip.
  public: int visualizationResolution{30};

  /// \brief Whether to visualize the normal forces.
  public: bool visualizeForces{false};

  /// \brief Model interface.
  public: Model model{kNullEntity};

  /// \brief Transport node
  public: transport::Node node;

  /// \brief Entity representing the contact sensor of the plugin
  public: ignition::gazebo::Entity sensorCollisionEntity;

  /// \brief Entity representing the collision of the object in contact with
  /// the optical tactile sensor.
  public: ignition::gazebo::Entity objectCollisionEntity;

  /// \brief Whether the plugin is enabled.
  public: bool enabled{true};

  /// \brief Initialization flag.
  public: bool initialized{false};

  /// \brief Name of the model to which the sensor is attached.
  public: std::string modelName;

  /// \brief Length of the visualized forces
  public: double forceLength{0.01};

  /// \brief Pose of the sensor model
  public: ignition::math::Pose3f tactileSensorWorldPose;

  /// \brief Pose of the sensor model in the previous iteration
  public: ignition::math::Pose3f prevTactileSensorWorldPose;

  /// \brief Offset between depth camera pose and model pose
  public: ignition::math::Pose3f depthCameraOffset;

  /// \brief Whether a new message has been returned by the depth camera
  public: bool newCameraMsg{false};

  /// \brief Depth camera update rate
  public: float cameraUpdateRate{1};

  /// \brief Message returned by the depth camera
  public: ignition::msgs::PointCloudPacked cameraMsg;

  /// \brief Mutex for variables mutated by the camera callback.
  /// The variables are: newCameraMsg, cameraMsg.
  public: std::mutex serviceMutex;

  /// \brief If true, the plugin will draw a marker in the place of the
  /// contact sensor so it's easy to know its position
  public: bool visualizeSensor{false};

  /// \brief Size of the contact sensor
  public: ignition::math::Vector3d sensorSize;

  /// \brief Extended sensing distance. The sensor will output data coming from
  /// its volume plus this distance.
  public: double extendedSensing{0.001};

  /// \brief Pointer to visualization object
  public: std::unique_ptr<OpticalTactilePluginVisualization> visualizePtr;

  /// \brief Flag for checking the data type returned by the DepthCamera
  public: bool checkDepthCameraData{true};

  /// \brief Flag for allowing the plugin to output error/warning only once
  public: bool initErrorPrinted{false};
};

//////////////////////////////////////////////////
OpticalTactilePlugin::OpticalTactilePlugin()
  : System(), dataPtr(std::make_unique<OpticalTactilePluginPrivate>())
{
}

//////////////////////////////////////////////////
void OpticalTactilePlugin::Configure(const Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  EntityComponentManager &_ecm,
  EventManager &)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "Optical tactile plugin should be attached to a model entity. "
      << "Failed to initialize." << std::endl;
    return;
  }

  // Get the sdf parameters

  if (!_sdf->HasElement("enabled"))
  {
    igndbg << "Missing parameter <enabled>, "
      << "setting to " << this->dataPtr->enabled << std::endl;
  }
  else
  {
    this->dataPtr->enabled = _sdf->Get<bool>("enabled");
  }

  if (!_sdf->HasElement("visualization_resolution"))
  {
    igndbg << "Missing parameter <visualization_resolution>, "
      << "setting to " << this->dataPtr->visualizationResolution << std::endl;
  }
  else
  {
    if (_sdf->Get<int>("visualization_resolution") < 0)
    {
      ignwarn << "Parameter <visualization_resolution> must be positive, "
        << "setting to " << this->dataPtr->visualizationResolution << std::endl;
    }
    else
    {
      this->dataPtr->visualizationResolution =
        _sdf->Get<int>("visualization_resolution");
    }
  }

  if (!_sdf->HasElement("visualize_forces"))
  {
    igndbg << "Missing parameter <visualize_forces>, "
      << "setting to " << this->dataPtr->visualizeForces << std::endl;
  }
  else
  {
    this->dataPtr->visualizeForces = _sdf->Get<bool>("visualize_forces");
  }

  if (!_sdf->HasElement("extended_sensing"))
  {
    igndbg << "Missing parameter <extended_sensing>, "
      << "setting to " << this->dataPtr->extendedSensing << std::endl;
  }
  else
  {
    if (_sdf->Get<double>("extended_sensing") < 0)
    {
      ignwarn << "Parameter <extended_sensing> must be positive, "
        << "setting to " << this->dataPtr->extendedSensing << std::endl;
    }
    else
    {
      this->dataPtr->extendedSensing = _sdf->Get<double>("extended_sensing");
    }
  }

  if (!_sdf->HasElement("visualize_sensor"))
  {
    igndbg << "Missing parameter <visualize_sensor>, "
      << "setting to " << this->dataPtr->visualizeSensor << std::endl;
  }
  else
  {
    this->dataPtr->visualizeSensor = _sdf->Get<bool>("visualize_sensor");
  }

  if (!_sdf->HasElement("force_length"))
  {
    igndbg << "Missing parameter <force_length>, "
      << "setting to " << this->dataPtr->forceLength << std::endl;
  }
  else
  {
    if (_sdf->Get<double>("force_length") < 0)
    {
      ignwarn << "Parameter <force_length> must be positive, "
        << "setting to " << this->dataPtr->forceLength << std::endl;
    }
    else
    {
      this->dataPtr->forceLength = _sdf->Get<double>("force_length");
    }
  }
}

//////////////////////////////////////////////////
void OpticalTactilePlugin::PreUpdate(const UpdateInfo &_info,
  EntityComponentManager &_ecm)
{
  IGN_PROFILE("OpticalTactilePlugin::PreUpdate");

  // Nothing left to do if paused
  if (_info.paused)
    return;

  if (!this->dataPtr->initialized)
  {
    // We call Load here instead of Configure because we can't be guaranteed
    // that all entities have been created when Configure is called
    this->dataPtr->Load(_ecm);
  }

  // This is not an "else" because "initialized" can be set
  // in the if block above
  if (this->dataPtr->initialized)
  {
    // Get the first object being touched by the sensor
    // We assume there's only one object being touched
    auto csd = _ecm.Component<components::ContactSensorData>(
      this->dataPtr->sensorCollisionEntity);
    for (const auto &contact : csd->Data().contact())
    {
      this->dataPtr->objectCollisionEntity =
        contact.collision2().id();
      break;
    }

    // Get the tactile sensor pose, i.e. the model pose
    ignition::math::Pose3d tactileSensorPose =
      _ecm.Component<components::Pose>(
        this->dataPtr->model.Entity())->Data();

    // Depth camera data is float, so convert Pose3d to Pose3f
    this->dataPtr->tactileSensorWorldPose = ignition::math::Pose3f(
      tactileSensorPose.Pos().X(),
      tactileSensorPose.Pos().Y(),
      tactileSensorPose.Pos().Z(),
      tactileSensorPose.Rot().W(),
      tactileSensorPose.Rot().X(),
      tactileSensorPose.Rot().Y(),
      tactileSensorPose.Rot().Z());
  }
}

//////////////////////////////////////////////////
void OpticalTactilePlugin::PostUpdate(
  const ignition::gazebo::UpdateInfo &_info,
  const ignition::gazebo::EntityComponentManager &/*_ecm*/)
{
  IGN_PROFILE("OpticalTactilePlugin::PostUpdate");

  // Nothing left to do if paused or failed to initialize.
  if (_info.paused || !this->dataPtr->initialized)
    return;

  // TODO(anyone) Get ContactSensor data and merge it with DepthCamera data

  // Process camera message if it's new
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
    if (this->dataPtr->newCameraMsg)
    {
      this->dataPtr->ComputeNormalForces(this->dataPtr->cameraMsg,
        this->dataPtr->visualizeForces);

      this->dataPtr->newCameraMsg = false;
    }
  }

  // Publish sensor marker if required and sensor pose has changed
  if (this->dataPtr->visualizeSensor &&
    (this->dataPtr->tactileSensorWorldPose !=
    this->dataPtr->prevTactileSensorWorldPose))
  {
    this->dataPtr->visualizePtr->RequestSensorMarkerMsg(
      this->dataPtr->tactileSensorWorldPose);
  }

  // Store the pose of the sensor in the current iteration
  this->dataPtr->prevTactileSensorWorldPose =
    this->dataPtr->tactileSensorWorldPose;
}

//////////////////////////////////////////////////
void OpticalTactilePluginPrivate::Load(const EntityComponentManager &_ecm)
{
  // Check plugin structure
  auto allLinks =
    _ecm.ChildrenByComponents(this->model.Entity(), components::Link());
  // Model should only have 1 link
  // todo(anyone) The plugin should be able to handle more than one link. In
  // that case, one of the links would be the actual tactile sensor.
  if (allLinks.size() != 1)
  {
    if (!this->initErrorPrinted)
    {
      ignerr << "Plugin must have exactly 1 link (only printed once)"
        << std::endl;
      this->initErrorPrinted = true;
    }
    return;
  }

  // Link should have a collision component linked to a contact sensor
  auto linkCollisions =
    _ecm.ChildrenByComponents(allLinks[0], components::Collision());

  if (linkCollisions.size() == 0)
  {
    if (!this->initErrorPrinted)
    {
      ignerr << "Link must have at least 1 collision (only printed once)"
        << std::endl;
      this->initErrorPrinted = true;
    }
    return;
  }

  for (const Entity &colEntity : linkCollisions)
  {
    if (_ecm.EntityHasComponentType(colEntity,
        components::ContactSensorData::typeId))
    {
      this->sensorCollisionEntity = colEntity;

      this->modelName = this->model.Name(_ecm);

      // Get the size of the collision element
      const auto geometry =
        _ecm.Component<components::Geometry>(colEntity)->Data();
      const auto *box = geometry.BoxShape();
      if (box == nullptr)
      {
        if (!this->initErrorPrinted)
        {
          ignerr << "Contact sensor geometry must be a box"
            << " (only printed once)" << std::endl;
          this->initErrorPrinted = true;
        }
        return;
      }
      this->sensorSize = geometry.BoxShape()->Size();
    }
  }

  // Link should have exactly 1 ContactSensor and 1 DepthCamera sensor
  auto sensorsInsideLink =
    _ecm.ChildrenByComponents(allLinks[0], components::Sensor());

  int depthCameraCounter = 0;
  int contactSensorCounter = 0;
  sdf::Sensor depthCameraSdf;
  components::Pose depthCameraPose = components::Pose();
  for (const Entity &sensor : sensorsInsideLink)
  {
    if (_ecm.EntityHasComponentType(sensor, components::DepthCamera::typeId))
    {
      depthCameraCounter += 1;
      depthCameraSdf =
        _ecm.Component<components::DepthCamera>(sensor)->Data();
      depthCameraPose = *(_ecm.Component<components::Pose>(sensor));
    }

    if (_ecm.EntityHasComponentType(sensor, components::ContactSensor::typeId))
    {
      contactSensorCounter += 1;
    }
  }

  if (((depthCameraCounter != 1) || (contactSensorCounter != 1)))
  {
    if (!this->initErrorPrinted)
    {
      ignerr << "Link must have exactly 1 depth camera sensor and "
        << "1 contact sensor (only printed once)" << std::endl;
      this->initErrorPrinted = true;
    }
    return;
  }

  // Store depth camera update rate
  if (!depthCameraSdf.Element()->HasElement("update_rate"))
  {
    if (!this->initErrorPrinted)
    {
      ignerr << "Depth camera should have an <update_rate> value "
        << "(only printed once)" << std::endl;
      this->initErrorPrinted = true;
    }
    return;
  }
  this->cameraUpdateRate = depthCameraSdf.UpdateRate();

  // Depth camera data is float, so convert Pose3d to Pose3f
  this->depthCameraOffset = ignition::math::Pose3f(
    depthCameraPose.Data().Pos().X(),
    depthCameraPose.Data().Pos().Y(),
    depthCameraPose.Data().Pos().Z(),
    depthCameraPose.Data().Rot().W(),
    depthCameraPose.Data().Rot().X(),
    depthCameraPose.Data().Rot().Y(),
    depthCameraPose.Data().Rot().Z());

  // Configure subscriber for depth camera images
  if (!depthCameraSdf.Element()->HasElement("topic"))
  {
    ignwarn << "Depth camera publishing to __default__ topic. "
      << "It's possible that two depth cameras are publishing into the same "
      << "topic" << std::endl;
  }
  else
  {
    igndbg << "Depth camera publishing to "
      << depthCameraSdf.Topic() << " topic" << std::endl;
  }

  std::string topic =
    "/" + depthCameraSdf.Topic() + "/points";
  if (!this->node.Subscribe(topic,
      &OpticalTactilePluginPrivate::DepthCameraCallback, this))
  {
    if (!this->initErrorPrinted)
    {
      ignerr << "Error subscribing to topic " << "[" << topic << "]. "
        << "<topic> must not contain '/' (only printing once)" << std::endl;
      this->initErrorPrinted = true;
    }
    return;
  }

  // Instantiate the visualization class
  this->visualizePtr = std::make_unique<
    OpticalTactilePluginVisualization>(
      this->modelName,
      this->sensorSize,
      this->forceLength,
      this->cameraUpdateRate,
      this->depthCameraOffset,
      this->visualizationResolution);

  this->initialized = true;
}

//////////////////////////////////////////////////
void OpticalTactilePluginPrivate::Enable(const bool /*_value*/)
{
  // todo(mcres) Implement method
}

//////////////////////////////////////////////////
void OpticalTactilePluginPrivate::DepthCameraCallback(
  const ignition::msgs::PointCloudPacked &_msg)
{
  // This check avoids running the callback at t=0 and getting
  // unexpected markers in the scene
  if (!this->initialized)
    return;

  // Check whether DepthCamera returns FLOAT32 data
  if (this->checkDepthCameraData)
  {
    int float32Type = 6;
    for (auto const &field : _msg.field())
    {
      if (field.datatype() != float32Type)
      {
        ignerr << "FLOAT32 is expected for a casting to float *" << std::endl;
        return;
      }
    }
    this->checkDepthCameraData = false;
  }

  {
    std::lock_guard<std::mutex> lock(this->serviceMutex);
    this->cameraMsg = _msg;
    this->newCameraMsg = true;
  }
}

//////////////////////////////////////////////////
ignition::math::Vector3f OpticalTactilePluginPrivate::MapPointCloudData(
  const uint64_t &_i, const uint64_t &_j, const char *_msgBuffer)
{
  IGN_PROFILE("OpticalTactilePlugin::MapPointCloudData");

  // Initialize return variable
  ignition::math::Vector3f measuredPoint(0, 0, 0);

  // Nothing left to do if failed to initialize.
  if (!this->initialized)
    return measuredPoint;

  // Save original position of the pointer
  const char *temporaryMsgBuffer = _msgBuffer;

  // Number of bytes from the beginning of the pointer (image coordinates at
  // 0,0) to the desired (i,j) position
  uint32_t msgBufferIndex =
    _j*this->cameraMsg.row_step() + _i*this->cameraMsg.point_step();

  temporaryMsgBuffer += msgBufferIndex;
  int fieldIndex = 0;

  // X coordinate
  measuredPoint.X() = *reinterpret_cast<const float *>(
    temporaryMsgBuffer + this->cameraMsg.field(fieldIndex++).offset());
  // Y coordinate
  measuredPoint.Y() = *reinterpret_cast<const float *>(
    temporaryMsgBuffer + this->cameraMsg.field(fieldIndex++).offset());
  // Z coordinate
  measuredPoint.Z() = *reinterpret_cast<const float *>(
    temporaryMsgBuffer + this->cameraMsg.field(fieldIndex).offset());

  // Check if point is inside the sensor
  bool pointInside = this->PointInsideSensor(measuredPoint);
  if (!pointInside)
  {
    measuredPoint.X() = ignition::math::INF_F;
    measuredPoint.Y() = ignition::math::INF_F;
    measuredPoint.Z() = ignition::math::INF_F;
  }

  return measuredPoint;
}

//////////////////////////////////////////////////
bool OpticalTactilePluginPrivate::PointInsideSensor(
  ignition::math::Vector3f _point)
{
  IGN_PROFILE("OpticalTactilePlugin::PointInsideSensor");

  // Nothing left to do if failed to initialize.
  if (!this->initialized)
    return false;

  // We assume that the depth camera is placed behind the contact surface, i.e.
  // displaced in the -X direction with respect to the model's origin
  bool insideX =
    (_point.X() >= std::abs(this->depthCameraOffset.X()) -
      this->extendedSensing) &&
    (_point.X() <= (std::abs(this->depthCameraOffset.X()) +
      this->sensorSize.X() + this->extendedSensing));

  bool insideY =
    (_point.Y() <= this->sensorSize.Y() / 2 + this->extendedSensing) &&
    (_point.Y() >= - this->sensorSize.Y() / 2 - this->extendedSensing);

  bool insideZ =
    (_point.Z() <= this->sensorSize.Z() / 2 + this->extendedSensing) &&
    (_point.Z() >= - this->sensorSize.Z() / 2 - this->extendedSensing);

  return (insideX && insideY && insideZ);
}

//////////////////////////////////////////////////
void OpticalTactilePluginPrivate::ComputeNormalForces(
  const ignition::msgs::PointCloudPacked &_msg,
  const bool _visualizeForces)
{
  IGN_PROFILE("OpticalTactilePlugin::ComputeNormalForces");

  // Nothing left to do if failed to initialize.
  if (!this->initialized)
    return;

  // Get data from the message
  const char *msgBuffer = _msg.data().data();

  // Declare variables for storing the XYZ points
  ignition::math::Vector3f p1, p2, p3, p4, markerPosition;

  // Marker messages representing the normal forces
  ignition::msgs::Marker positionMarkerMsg;
  ignition::msgs::Marker forceMarkerMsg;

  // We don't get the image's edges because there are no adjacent points to
  // compute the forces
  for (uint64_t j = 1; j < (_msg.height() - 1);
      j += this->visualizationResolution)
  {
    for (uint64_t i = 1; i < (_msg.width() - 1);
        i += this->visualizationResolution)
    {
      // Get points for computing normal forces
      p1 = this->MapPointCloudData(i + 1, j, msgBuffer);
      p2 = this->MapPointCloudData(i - 1, j, msgBuffer);
      p3 = this->MapPointCloudData(i, j + 1, msgBuffer);
      p4 = this->MapPointCloudData(i, j - 1, msgBuffer);

      float dxdi = (p1.X() - p2.X()) / std::abs(p1.Y() - p2.Y());
      float dxdj =  (p3.X() - p4.X()) / std::abs(p3.Z() - p4.Z());

      ignition::math::Vector3f direction(-1, -dxdi, -dxdj);

      // todo(anyone) multiply vector by contact forces info

      // todo(anyone) Replace with MatrixX and use vector multiplication instead
      // of for-loops once the following issue is completed:
      // https://github.com/ignitionrobotics/ign-math/issues/144
      ignition::math::Vector3f normalForce = direction.Normalized();

      // todo(mcres) Normal forces are computed even if visualization
      // is turned off. These forces should be published in the future.
      if (!_visualizeForces)
        continue;

      markerPosition = this->MapPointCloudData(i, j, msgBuffer);
      this->visualizePtr->AddNormalForceToMarkerMsgs(positionMarkerMsg,
        forceMarkerMsg, markerPosition, normalForce,
        this->tactileSensorWorldPose);
    }
  }

  if (_visualizeForces)
  {
    this->visualizePtr->RequestNormalForcesMarkerMsgs(positionMarkerMsg,
      forceMarkerMsg);
  }
}

IGNITION_ADD_PLUGIN(OpticalTactilePlugin,
  ignition::gazebo::System,
  OpticalTactilePlugin::ISystemConfigure,
  OpticalTactilePlugin::ISystemPreUpdate,
  OpticalTactilePlugin::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(OpticalTactilePlugin,
  "ignition::gazebo::systems::OpticalTactilePlugin")
