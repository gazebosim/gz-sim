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

#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"

#include "OpticalTactilePlugin.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::OpticalTactilePluginPrivate
{
  /// \brief Load the Contact sensor from an sdf element
  /// \param[in] _sdf SDF element describing the Contact sensor
  /// \param[in] _ecm Immutable reference to the EntityComponentManager
  public: void Load(const EntityComponentManager &_ecm,
                    const std::shared_ptr<const sdf::Element> &_sdf);

  /// \brief Actual function that enables the plugin.
  /// \param[in] _value True to enable plugin.
  public: void Enable(const bool _value);

  /// \brief Filters out new collisions fetched not related to the sensors
  /// \param[in] _ecm Immutable reference to the EntityComponentManager
  /// \param[in] _entities Collision entities to filter
  public: void FilterOutCollisions(const EntityComponentManager &_ecm,
                                    const std::vector<Entity> &_entities);

  /// \brief Interpolates contact data
  /// \param[in] _contactMsg Message containing the data to interpolate
  public: void InterpolateData(const ignition::msgs::Contact &_contactMsg);

  /// \brief Callback for the Depth Camera
  /// \param[in] _msg Message from the subscribed topic
  public: void DepthCameraCallback(
                const ignition::msgs::PointCloudPacked &_msg);

  /// \brief Unpacks the point cloud retrieved by the Depth Camera
  /// into XYZ data
  /// \param[in] _msg Message containing the data to unpack
  public: void UnpackPointCloudMsg(
                const ignition::msgs::PointCloudPacked &_msg);

  /// \brief Computes the normal forces of the Optical Tactile sensor
  /// \param[in] _msg Message returned by the Depth Camera
  /// \param[in] _visualizeForces Whether to visualize the forces or not
  ///
  /// Implementation inspired by
  /// https://stackoverflow.com/questions/
  /// 34644101/calculate-surface-normals-from-depth-image-
  /// using-neighboring-pixels-cross-produc
  public: void ComputeNormalForces(
                const ignition::msgs::PointCloudPacked &_msg,
                const bool _visualizeForces);

  /// \brief Resolution of the sensor in pixels to skip.
  public: int resolution{100};

  /// \brief Whether to visualize the normal forces.
  public: bool visualizeForces{false};

  /// \brief Model interface.
  public: Model model{kNullEntity};

  /// \brief Transport node to visualize data on Gazebo.
  public: transport::Node node;

  /// \brief Collision entities that have been designated as contact sensors.
  public: ignition::gazebo::Entity contactSensorEntity;

  /// \brief Collision entities of the simulation.
  public: std::vector<Entity> collisionEntities;

  /// \brief Filtered collisions from the simulation that belong to
  /// one or more sensors.
  public: std::vector<Entity> filteredCollisionEntities;

  /// \brief Whether the plugin is enabled.
  public: bool enabled{true};

  /// \brief Initialization flag.
  public: bool initialized{false};

  /// \brief Copy of the sdf configuration used for this plugin.
  public: std::shared_ptr<const sdf::Element> sdfConfig{nullptr};

  /// \brief Name of the model to which the sensor is attached.
  public: std::string modelName;

  /// \brief Message for visualizing contact positions
  public: ignition::msgs::Marker positionMarkerMsg;

  /// \brief Radius of the visualized contact sphere
  public: double contactRadius{0.003};

  /// \brief Message for visualizing contact forces
  public: ignition::msgs::Marker forceMarkerMsg;

  /// \brief Length of the visualized force cylinder
  public: double forceLength{0.01};

  /// \brief Radius of the visualized force cylinder
  public: double forceRadius{0.001};

  /// \brief Position interpolated from the Contact messages
  public: std::vector<ignition::math::Vector3d> interpolatedPosition;

  /// \brief Pose of the sensor model
  public: ignition::math::Pose3f sensorWorldPose;

  /// \brief Pose of the sensor model in the previous iteration
  public: ignition::math::Pose3f prevSensorWorldPose;

  /// \brief Offset between Depth Camera pose and model pose
  public: ignition::math::Pose3f depthCameraOffset;

  /// \brief Whether a new message has been returned by the Depth Camera
  public: bool newCameraMsg{false};

  /// \brief Depth Camera update rate
  public: float cameraUpdateRate{1};

  /// \brief Message returned by the Depth Camera
  public: ignition::msgs::PointCloudPacked cameraMsg;

  /// \brief 3D vector to store unpacked XYZ points
  std::vector<std::vector<std::vector<float>>> imageXYZ;

  /// \brief Mutex for variables mutated by the camera callback.
  /// The variables are: newCameraMsg, cameraMsg.
  public: std::mutex serviceMutex;

  /// \brief If true, the plugin will draw a marker in the place of the
  /// contact sensor so it's easy to know its position
  public: bool visualizeSensor{false};

  /// \brief Message for visualizing the sensor
  public: ignition::msgs::Marker sensorMarkerMsg;

  /// \brief Size of the contact sensor
  public: ignition::math::Vector3d sensorSize;

  /// \brief Extended sensing distance. The sensor will output data coming from
  /// its volume plus this distance.
  public: double extendedSensing{0.001};
};

//////////////////////////////////////////////////
void OpticalTactilePluginPrivate::Load(const EntityComponentManager &_ecm,
                              const std::shared_ptr<const sdf::Element> &_sdf)
{
    igndbg << "Loading plugin [OpticalTactilePlugin]" << std::endl;

    // Get sdf parameters
    if (!_sdf->HasElement("resolution"))
    {
        ignlog << "Missing required parameter <resolution>, "
            << "using default value" << std::endl;
    }
    else
    {
        if (_sdf->Get<int>("resolution") < 0)
        {
          ignwarn << "Parameter <resolution> must be positive, "
              << "using default value" << std::endl;
        }
        else
        {
          this->resolution = _sdf->Get<int>("resolution");
        }
    }

    if (!_sdf->HasElement("visualize_forces"))
    {
        ignlog << "Missing required parameter <visualize_forces>, "
            << "using default value" << std::endl;
    }
    else
    {
        this->visualizeForces = _sdf->Get<bool>("visualize_forces");
    }

    if (!_sdf->HasElement("extended_sensing"))
    {
        ignlog << "Missing required parameter <extended_sensing>, "
            << "using default value" << std::endl;
    }
    else
    {
        this->extendedSensing = _sdf->Get<double>("extended_sensing");
    }

    if (!_sdf->HasElement("visualize_sensor"))
    {
        ignlog << "Missing required parameter <visualize_sensor>, "
            << "using default value" << std::endl;
    }
    else
    {
        this->visualizeSensor = _sdf->Get<bool>("visualize_sensor");
    }

    // todo(anyone) The plugin should be able to handle more than one link. In
    // that case, one of the links would be the actual tactile sensor.

    // Check SDF structure
    // Model should only have 1 link
    auto allLinks =
        _ecm.ChildrenByComponents(this->model.Entity(), components::Link());
    if (allLinks.size() != 1)
    {
      ignerr << "Plugin must have 1 link" << std::endl;
      return;
    }

    // Link should have a collision component linked to a contact sensor
    auto linkCollisions =
        _ecm.ChildrenByComponents(allLinks[0], components::Collision());
    for (const Entity &colEntity : linkCollisions)
    {
      if (_ecm.EntityHasComponentType(colEntity,
          components::ContactSensorData::typeId))
      {
        this->contactSensorEntity = colEntity;

        this->modelName = this->model.Name(_ecm);

        igndbg << "Contactensor detected within model "
            << this->modelName << std::endl;
      }
    }

    // Link should have 1 Contact sensor and 1 Depth Camera sensor
    auto sensorsInsideLink =
        _ecm.ChildrenByComponents(allLinks[0], components::Sensor());

    bool depthCameraInLink = false;
    bool contactSensorInLink = false;
    sdf::ElementPtr depthCameraSdf = nullptr;
    for (const Entity &sensor : sensorsInsideLink)
    {
      if (_ecm.EntityHasComponentType(sensor,
                                      components::DepthCamera::typeId))
      {
        depthCameraInLink = true;
        depthCameraSdf =
          _ecm.Component<components::DepthCamera>(sensor)->Data().Element();
      }

      if (_ecm.EntityHasComponentType(sensor,
                                      components::ContactSensor::typeId))
      {
        contactSensorInLink = true;
      }
    }

    if (!depthCameraInLink || !contactSensorInLink ||
      sensorsInsideLink.size() != 2)
    {
      ignerr << "Link must have 1 Depth Camera sensor and "
        << "1 Contact sensor" << std::endl;
      return;
    }

    // Store depth camera update rate and offset from model
    auto offset = depthCameraSdf->Get<ignition::math::Pose3d>("pose");
    this->cameraUpdateRate = 1;
    if (depthCameraSdf->HasElement("update_rate"))
    {
      this->cameraUpdateRate = depthCameraSdf->Get<float>("update_rate");
    }

    // Depth Camera data is float, so convert Pose3d to Pose3f
    this->depthCameraOffset = ignition::math::Pose3f(
        offset.Pos().X(), offset.Pos().Y(), offset.Pos().Z(),
        offset.Rot().W(), offset.Rot().X(), offset.Rot().Y(),
        offset.Rot().Z());

    // Configure subscriber for Depth Camera images
    std::string topic = "/depth_camera/points";
    if (depthCameraSdf->HasElement("topic"))
      topic = "/" + depthCameraSdf->Get<std::string>("topic") + "/points";

    if (!this->node.Subscribe(topic,
        &OpticalTactilePluginPrivate::DepthCameraCallback,
        this))
    {
      ignerr << "Error subscribing to topic " << "[" << topic << "]. "
        << "<topic> must not contain '/'" << std::endl;
      return;
    }
}

//////////////////////////////////////////////////
void OpticalTactilePluginPrivate::Enable(const bool _value)
{
    // Just a placeholder method for the moment
    _value;
}

//////////////////////////////////////////////////
void OpticalTactilePluginPrivate::FilterOutCollisions(
                            const EntityComponentManager &_ecm,
                            const std::vector<Entity> &_entities)
{
    if (_entities.empty())
        return;

    // Clear vector that contains old data
    this->filteredCollisionEntities.clear();

    // Get collisions from the sensor
    for (const Entity &entity : _entities)
    {
        std::string name = scopedName(entity, _ecm);

        if (name.find(this->modelName) != std::string::npos)
        {
            this->filteredCollisionEntities.push_back(entity);
        }
    }
}

//////////////////////////////////////////////////
void OpticalTactilePluginPrivate::InterpolateData(
                    const ignition::msgs::Contact &contact)
{
    // The Optical Tactile Sensor is supposed to be box-shaped.
    // Contacts returned by the Contact Sensor in this type of shapes should be
    // 4 points in the corners of some of its faces, in this case the one
    // corresponding to the 'sensing' face.
    // Given that the number of contacts may change even when the objects are
    // at rest, data is only interpolated when the number of contacts
    // returned is 4.

    if (contact.position_size() == 4)
    {
        ignition::math::Vector3d contact1 =
            ignition::msgs::Convert(contact.position(0));
        ignition::math::Vector3d contact2 =
            ignition::msgs::Convert(contact.position(1));
        ignition::math::Vector3d contact3 =
            ignition::msgs::Convert(contact.position(2));
        ignition::math::Vector3d contact4 =
            ignition::msgs::Convert(contact.position(3));

        ignition::math::Vector3d direction1 =
            (contact2 - contact1).Normalized() * this->resolution;
        ignition::math::Vector3d direction2 =
            (contact4 - contact1).Normalized() * this->resolution;

        ignition::math::Vector3d interpolatedVector = contact1;
        this->interpolatedPosition.push_back(contact1);

        // auxiliary Vector3d to iterate through contacts
        ignition::math::Vector3d tempVector(0.0, 0.0, 0.0);

        uint64_t steps1 =
            contact1.Distance(contact2) / this->resolution;
        uint64_t steps2 =
            contact1.Distance(contact4) / this->resolution;

        for (uint64_t index1 = 0; index1 <= steps1; ++index1)
        {
            tempVector = interpolatedVector;
            for (uint64_t index2 = 0; index2 < steps2; ++index2)
            {
                interpolatedVector += direction2;
                this->interpolatedPosition.push_back(interpolatedVector);
            }
            if (index1 != steps1)
            {
                interpolatedVector = tempVector;
                interpolatedVector += direction1;
                this->interpolatedPosition.push_back(interpolatedVector);
            }
        }
    }
}

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
    this->dataPtr->sdfConfig = _sdf;
    this->dataPtr->model = Model(_entity);

    if (!this->dataPtr->model.Valid(_ecm))
    {
      ignerr << "Touch plugin should be attached to a model entity. "
        << "Failed to initialize." << std::endl;
      return;
    }

    // Configure Marker messages for position and force of the contacts
    // Blue spheres for positions
    // Green cylinders for forces

    // Get markers parameters

    if (!_sdf->HasElement("contact_radius"))
    {
        ignlog << "Missing required parameter <contact_radius>, "
            << "using default value" << std::endl;
    }
    else
    {
        this->dataPtr->contactRadius = _sdf->Get<double>("contact_radius");
    }

    if (!_sdf->HasElement("force_radius"))
    {
        ignlog << "Missing required parameter <force_radius>, "
            << "using default value" << std::endl;
    }
    else
    {
        this->dataPtr->forceRadius = _sdf->Get<double>("force_radius");
    }

    if (!_sdf->HasElement("force_length"))
    {
        ignlog << "Missing required parameter <force_length>, "
            << "using default value" << std::endl;
    }
    else
    {
        this->dataPtr->forceLength = _sdf->Get<double>("force_length");
    }

    // Create the marker message
    this->dataPtr->positionMarkerMsg.set_ns("positions");
    this->dataPtr->positionMarkerMsg.set_action(
        ignition::msgs::Marker::ADD_MODIFY);
    this->dataPtr->positionMarkerMsg.set_type(
        ignition::msgs::Marker::SPHERE);
    this->dataPtr->positionMarkerMsg.set_visibility(
        ignition::msgs::Marker::GUI);

    this->dataPtr->forceMarkerMsg.set_ns("forces");
    this->dataPtr->forceMarkerMsg.set_action(
        ignition::msgs::Marker::ADD_MODIFY);
    this->dataPtr->forceMarkerMsg.set_type(
        ignition::msgs::Marker::CYLINDER);
    this->dataPtr->forceMarkerMsg.set_visibility(
        ignition::msgs::Marker::GUI);

    this->dataPtr->sensorMarkerMsg.set_ns("sensor");
    this->dataPtr->sensorMarkerMsg.set_id(1);
    this->dataPtr->sensorMarkerMsg.set_action(
        ignition::msgs::Marker::ADD_MODIFY);
    this->dataPtr->sensorMarkerMsg.set_type(
        ignition::msgs::Marker::BOX);
    this->dataPtr->sensorMarkerMsg.set_visibility(
        ignition::msgs::Marker::GUI);

    // Set material properties
    this->dataPtr->
        positionMarkerMsg.mutable_material()->mutable_ambient()->set_r(0);
    this->dataPtr->
        positionMarkerMsg.mutable_material()->mutable_ambient()->set_g(0);
    this->dataPtr->
        positionMarkerMsg.mutable_material()->mutable_ambient()->set_b(1);
    this->dataPtr->
        positionMarkerMsg.mutable_material()->mutable_ambient()->set_a(1);
    this->dataPtr->
        positionMarkerMsg.mutable_material()->mutable_diffuse()->set_r(0);
    this->dataPtr->
        positionMarkerMsg.mutable_material()->mutable_diffuse()->set_g(0);
    this->dataPtr->
        positionMarkerMsg.mutable_material()->mutable_diffuse()->set_b(1);
    this->dataPtr->
        positionMarkerMsg.mutable_material()->mutable_diffuse()->set_a(1);
    this->dataPtr->
        positionMarkerMsg.mutable_lifetime()->
          set_sec(this->dataPtr->cameraUpdateRate);

    this->dataPtr->
        forceMarkerMsg.mutable_material()->mutable_ambient()->set_r(0);
    this->dataPtr->
        forceMarkerMsg.mutable_material()->mutable_ambient()->set_g(1);
    this->dataPtr->
        forceMarkerMsg.mutable_material()->mutable_ambient()->set_b(0);
    this->dataPtr->
        forceMarkerMsg.mutable_material()->mutable_ambient()->set_a(1);
    this->dataPtr->
        forceMarkerMsg.mutable_material()->mutable_diffuse()->set_r(0);
    this->dataPtr->
        forceMarkerMsg.mutable_material()->mutable_diffuse()->set_g(1);
    this->dataPtr->
        forceMarkerMsg.mutable_material()->mutable_diffuse()->set_b(0);
    this->dataPtr->
        forceMarkerMsg.mutable_material()->mutable_diffuse()->set_a(1);
    this->dataPtr->
        forceMarkerMsg.mutable_lifetime()->
          set_sec(this->dataPtr->cameraUpdateRate);

    this->dataPtr->
        sensorMarkerMsg.mutable_material()->mutable_ambient()->set_r(0.5);
    this->dataPtr->
        sensorMarkerMsg.mutable_material()->mutable_ambient()->set_g(0.5);
    this->dataPtr->
        sensorMarkerMsg.mutable_material()->mutable_ambient()->set_b(0.5);
    this->dataPtr->
        sensorMarkerMsg.mutable_material()->mutable_ambient()->set_a(0.75);
    this->dataPtr->
        sensorMarkerMsg.mutable_material()->mutable_diffuse()->set_r(0.5);
    this->dataPtr->
        sensorMarkerMsg.mutable_material()->mutable_diffuse()->set_g(0.5);
    this->dataPtr->
        sensorMarkerMsg.mutable_material()->mutable_diffuse()->set_b(0.5);
    this->dataPtr->
        sensorMarkerMsg.mutable_material()->mutable_diffuse()->set_a(0.75);
    this->dataPtr->
        sensorMarkerMsg.mutable_lifetime()->set_sec(0);
    this->dataPtr->
        sensorMarkerMsg.mutable_lifetime()->set_nsec(0);

    // Set scales
    ignition::msgs::Set(this->dataPtr->positionMarkerMsg.mutable_scale(),
        ignition::math::Vector3d(this->dataPtr->contactRadius,
        this->dataPtr->contactRadius,
        this->dataPtr->contactRadius));

    ignition::msgs::Set(this->dataPtr->forceMarkerMsg.mutable_scale(),
        ignition::math::Vector3d(this->dataPtr->forceRadius,
        this->dataPtr->forceRadius, this->dataPtr->forceLength));

    // Get the size of the sensor from the SDF

    // If there's no <collision> specified inside <link>, a default one
    // is set
    this->dataPtr->sensorSize =
      _sdf->GetParent()->GetElement("link")->GetElement("collision")->
      GetElement("geometry")->GetElement("box")->
      Get<ignition::math::Vector3d>("size");

    ignition::msgs::Set(this->dataPtr->sensorMarkerMsg.mutable_scale(),
      this->dataPtr->sensorSize);
}

//////////////////////////////////////////////////
void OpticalTactilePlugin::PreUpdate(const UpdateInfo &_info,
                            EntityComponentManager &_ecm)
{
    IGN_PROFILE("TouchPluginPrivate::PreUpdate");

    // Nothing left to do if paused
    if (_info.paused)
      return;

    if (!this->dataPtr->initialized)
    {
        // We call Load here instead of Configure because we can't be guaranteed
        // that all entities have been created when Configure is called
        this->dataPtr->Load(_ecm, this->dataPtr->sdfConfig);
        this->dataPtr->initialized = true;
    }

    // This is not an "else" because "initialized" can be set
    // in the if block above
    if (this->dataPtr->initialized)
    {
      // Fetch new collisions from simulation
      std::vector<Entity> newCollisions;
      _ecm.Each<components::Collision>(
          [&](const Entity &_entity, const components::Collision *) -> bool
          {
            newCollisions.push_back(_entity);
            return true;
          });
      this->dataPtr->FilterOutCollisions(_ecm, newCollisions);

      // Get the Depth Camera pose
      ignition::math::Pose3d sensorPose =
          _ecm.Component<components::Pose>(
            this->dataPtr->model.Entity())->Data();

      // Depth Camera data is float, so convert Pose3d to Pose3f
      this->dataPtr->sensorWorldPose = ignition::math::Pose3f(
          sensorPose.Pos().X(), sensorPose.Pos().Y(), sensorPose.Pos().Z(),
          sensorPose.Rot().W(), sensorPose.Rot().X(), sensorPose.Rot().Y(),
          sensorPose.Rot().Z());
    }
}

//////////////////////////////////////////////////
void OpticalTactilePlugin::PostUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
    IGN_PROFILE("TouchPlugin::PostUpdate");

    // Nothing left to do if paused.
    if (_info.paused)
      return;

    // Delete old interpolated positions
    this->dataPtr->interpolatedPosition.clear();

    for (const Entity &colEntity : this->dataPtr->filteredCollisionEntities)
    {
        auto* contacts =
            _ecm.Component<components::ContactSensorData>(colEntity);
        if (contacts)
        {
            for (const auto &contact : contacts->Data().contact())
            {
                // Interpolate data returned by the Contact message
                this->dataPtr->InterpolateData(contact);
            }
        }
    }

    // Process camera message if it's new
    {
      std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
      if (this->dataPtr->newCameraMsg)
      {
        this->dataPtr->UnpackPointCloudMsg(this->dataPtr->cameraMsg);
        this->dataPtr->ComputeNormalForces(this->dataPtr->cameraMsg,
            this->dataPtr->visualizeForces);

        this->dataPtr->newCameraMsg = false;
      }
    }

    // Publish sensor marker if required and sensor pose has changed
    if (this->dataPtr->visualizeSensor &&
      (this->dataPtr->sensorWorldPose != this->dataPtr->prevSensorWorldPose))
    {
      this->dataPtr->sensorMarkerMsg.set_action(
          ignition::msgs::Marker::ADD_MODIFY);
      this->dataPtr->sensorMarkerMsg.set_id(1);

      this->dataPtr->sensorMarkerMsg.mutable_pose()
        ->mutable_position()->set_x(this->dataPtr->sensorWorldPose.Pos().X());
      this->dataPtr->sensorMarkerMsg.mutable_pose()
        ->mutable_position()->set_y(this->dataPtr->sensorWorldPose.Pos().Y());
      this->dataPtr->sensorMarkerMsg.mutable_pose()
        ->mutable_position()->set_z(this->dataPtr->sensorWorldPose.Pos().Z());
      this->dataPtr->sensorMarkerMsg.mutable_pose()->
        mutable_orientation()->set_x(
          this->dataPtr->sensorWorldPose.Rot().X());
      this->dataPtr->sensorMarkerMsg.mutable_pose()->
        mutable_orientation()->set_y(
          this->dataPtr->sensorWorldPose.Rot().Y());
      this->dataPtr->sensorMarkerMsg.mutable_pose()->
        mutable_orientation()->set_z(
          this->dataPtr->sensorWorldPose.Rot().Z());
      this->dataPtr->sensorMarkerMsg.mutable_pose()->
        mutable_orientation()->set_w(
          this->dataPtr->sensorWorldPose.Rot().W());

      this->dataPtr->node.Request("/marker", this->dataPtr->sensorMarkerMsg);
    }

    // Store the pose of the sensor in the current iteration
    this->dataPtr->prevSensorWorldPose = this->dataPtr->sensorWorldPose;
}

//////////////////////////////////////////////////
void OpticalTactilePluginPrivate::DepthCameraCallback(
        const ignition::msgs::PointCloudPacked &_msg)
{
  // This check avoids running the callback at t=0 and getting
  // unexpected markers in the scene
  if (!this->initialized)
      return;

  {
    std::lock_guard<std::mutex> lock(this->serviceMutex);
    this->cameraMsg = _msg;
    this->newCameraMsg = true;
  }
}

//////////////////////////////////////////////////
void OpticalTactilePluginPrivate::UnpackPointCloudMsg(
                        const ignition::msgs::PointCloudPacked &_msg)
{
  // 3D vector to store XYZ points
  this->imageXYZ = std::vector<std::vector<std::vector<float>>>
      (_msg.width(),
      std::vector<std::vector<float>>(_msg.height(),
      std::vector<float>(3)));

  // Unpack point cloud message
  std::string msgBuffer = _msg.data();
  char *msgBufferIndex = msgBuffer.data();

  for (uint64_t j = 0; j < _msg.height(); ++j)
  {
    for (uint64_t i = 0; i < _msg.width(); ++i)
    {
      int fieldIndex = 0;

      // x coordinate in pixel (i,j)
      this->imageXYZ[i][j][0] =  *reinterpret_cast<float *>(
        msgBufferIndex + _msg.field(fieldIndex++).offset());
      // y coordinate in pixel (i,j)
      this->imageXYZ[i][j][1] = *reinterpret_cast<float *>(
        msgBufferIndex + _msg.field(fieldIndex++).offset());
      // z coordinate in pixel (i,j)
      this->imageXYZ[i][j][2] = *reinterpret_cast<float *>(
        msgBufferIndex + _msg.field(fieldIndex++).offset());

      // Filter out points outside of the contact sensor

      // We assume that the Depth Camera is only displaced in the -X direction
      bool insideX = (this->imageXYZ[i][j][0] >= std::abs(
        this->depthCameraOffset.X()) - this->extendedSensing)
        && (this->imageXYZ[i][j][0] <= (std::abs(this->depthCameraOffset.X()) +
        this->sensorSize.X() + this->extendedSensing));

      bool insideY = (this->imageXYZ[i][j][1] <= this->sensorSize.Y() / 2 +
        this->extendedSensing)
        && (this->imageXYZ[i][j][1] >= - this->sensorSize.Y() / 2 -
        this->extendedSensing);

      bool insideZ = (this->imageXYZ[i][j][2] <= this->sensorSize.Z() / 2 +
        this->extendedSensing)
        && (this->imageXYZ[i][j][2] >= - this->sensorSize.Z() / 2 -
        this->extendedSensing);

      if (!insideX || !insideY || !insideZ)
      {
        this->imageXYZ[i][j][0] = ignition::math::INF_D;
        this->imageXYZ[i][j][1] = ignition::math::INF_D;
        this->imageXYZ[i][j][2] = ignition::math::INF_D;
      }

      msgBufferIndex += _msg.point_step();
    }
  }
}

//////////////////////////////////////////////////
void OpticalTactilePluginPrivate::ComputeNormalForces(
                    const ignition::msgs::PointCloudPacked &_msg,
                    const bool _visualizeForces)
{
  // 2D vector to store normal force vectors
  std::vector<std::vector<ignition::math::Vector3f>> normalForces(_msg.width(),
      std::vector<ignition::math::Vector3f>(_msg.height()));

  // Visualization is downsampled according to plugin parameter <resolution>
  // if <visualize_forces> is set to true
  bool visualizeRow = true;
  int rowCounter = 0;

  bool visualizeColumn = true;
  int columnCounter = 0;

  // Variable for setting the markers id through the iteration
  int marker_id = 1;

  for (uint64_t j = 1; j < (_msg.height() - 1); ++j)
  {
    for (uint64_t i = 1; i < (_msg.width() - 1); ++i)
    {
      // Compute normal forces
      double dxdi =
          (this->imageXYZ[i+1][j][0] -
          this->imageXYZ[i-1][j][0])/
          std::abs(this->imageXYZ[i+1][j][1] -
          this->imageXYZ[i-1][j][1]);

      double dxdj =
          (this->imageXYZ[i][j+1][0] -
          this->imageXYZ[i][j-1][0])/
          std::abs(this->imageXYZ[i][j+1][2] -
          this->imageXYZ[i][j-1][2]);

      ignition::math::Vector3f direction(-1, -dxdi, -dxdj);

      // todo(anyone) multiply vector by contact forces info
      normalForces[i][j] = direction.Normalized();

      // todo(mcres) normal forces will be published even if visualization
      // is turned off
      if (!_visualizeForces)
          continue;

      // Downsampling for visualization
      if (visualizeRow && visualizeColumn)
      {
        // Add new markers with specific lifetime
        this->positionMarkerMsg.set_id(marker_id);
        this->positionMarkerMsg.set_action(
            ignition::msgs::Marker::ADD_MODIFY);
        this->positionMarkerMsg.mutable_lifetime()->set_sec(
            this->cameraUpdateRate);
        this->positionMarkerMsg.mutable_lifetime()->set_nsec(0);

        this->forceMarkerMsg.set_id(marker_id++);
        this->forceMarkerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
        this->forceMarkerMsg.mutable_lifetime()->set_sec(
            this->cameraUpdateRate);
        this->forceMarkerMsg.mutable_lifetime()->set_nsec(0);

        // The pose of the  markers must be published with reference
        // to the simulation origin
        ignition::math::Vector3f forceMarkerSensorPosition
            (this->imageXYZ[i][j][0], this->imageXYZ[i][j][1],
            this->imageXYZ[i][j][2]);

        ignition::math::Quaternionf forceMarkerSensorQuaternion;
        forceMarkerSensorQuaternion.From2Axes(
          ignition::math::Vector3f(0, 0, 1), normalForces[i][j]);

        // The position of the force marker is modified in order to place the
        // end of the cylinder in the surface, not its middle point
        ignition::math::Pose3f forceMarkerSensorPose(
            forceMarkerSensorPosition +
            normalForces[i][j] * (this->forceLength/2),
            forceMarkerSensorQuaternion);

        ignition::math::Pose3f forceMarkerWorldPose =
            (forceMarkerSensorPose + this->depthCameraOffset) +
            this->sensorWorldPose;
        forceMarkerWorldPose.Correct();

        ignition::math::Pose3f positionMarkerSensorPose(
            forceMarkerSensorPosition, forceMarkerSensorQuaternion);
        ignition::math::Pose3f positionMarkerWorldPose =
            (positionMarkerSensorPose + this->depthCameraOffset) +
            this->sensorWorldPose;
        positionMarkerWorldPose.Correct();

        // Set markers pose messages from previously computed poses
        this->forceMarkerMsg.mutable_pose()
          ->mutable_position()->set_x(forceMarkerWorldPose.Pos().X());
        this->forceMarkerMsg.mutable_pose()
          ->mutable_position()->set_y(forceMarkerWorldPose.Pos().Y());
        this->forceMarkerMsg.mutable_pose()
          ->mutable_position()->set_z(forceMarkerWorldPose.Pos().Z());
        this->forceMarkerMsg.mutable_pose()
          ->mutable_orientation()->set_x(forceMarkerWorldPose.Rot().X());
        this->forceMarkerMsg.mutable_pose()
          ->mutable_orientation()->set_y(forceMarkerWorldPose.Rot().Y());
        this->forceMarkerMsg.mutable_pose()
          ->mutable_orientation()->set_z(forceMarkerWorldPose.Rot().Z());
        this->forceMarkerMsg.mutable_pose()
          ->mutable_orientation()->set_w(forceMarkerWorldPose.Rot().W());

        this->positionMarkerMsg.mutable_pose()
          ->mutable_position()->set_x(positionMarkerWorldPose.Pos().X());
        this->positionMarkerMsg.mutable_pose()
          ->mutable_position()->set_y(positionMarkerWorldPose.Pos().Y());
        this->positionMarkerMsg.mutable_pose()
          ->mutable_position()->set_z(positionMarkerWorldPose.Pos().Z());
        this->positionMarkerMsg.mutable_pose()
          ->mutable_orientation()->set_x(positionMarkerWorldPose.Rot().X());
        this->positionMarkerMsg.mutable_pose()
          ->mutable_orientation()->set_y(positionMarkerWorldPose.Rot().Y());
        this->positionMarkerMsg.mutable_pose()
          ->mutable_orientation()->set_z(positionMarkerWorldPose.Rot().Z());
        this->positionMarkerMsg.mutable_pose()
          ->mutable_orientation()->set_w(positionMarkerWorldPose.Rot().W());

        this->node.Request("/marker", this->forceMarkerMsg);
        this->node.Request("/marker", this->positionMarkerMsg);
      }
      visualizeColumn = (this->resolution == ++columnCounter);
      if (visualizeColumn)
          columnCounter = 0;
    }
    visualizeColumn = true;
    visualizeRow = (this->resolution == ++rowCounter);
    if (visualizeRow)
        rowCounter = 0;
  }
}

IGNITION_ADD_PLUGIN(OpticalTactilePlugin,
                    ignition::gazebo::System,
                    OpticalTactilePlugin::ISystemConfigure,
                    OpticalTactilePlugin::ISystemPreUpdate,
                    OpticalTactilePlugin::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(OpticalTactilePlugin,
    "ignition::gazebo::systems::OpticalTactilePlugin")
