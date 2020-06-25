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
                    const sdf::ElementPtr &_sdf);

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
  public: int visualize_forces{false};

  /// \brief Model interface.
  public: Model model{kNullEntity};

  /// \brief Transport node to visualize data on Gazebo.
  public: transport::Node node;

  /// \brief Collision entities that have been designated as contact sensors.
  public: std::vector<Entity> sensorEntities;

  /// \brief Collision entities of the simulation.
  public: std::vector<Entity> collisionEntities;

  /// \brief Filtered collisions from the simulation that belong to
  /// one or more sensors.
  public: std::vector<Entity> filteredCollisionEntities;

  /// \brief Wheter the plugin is enabled.
  public: bool enabled{true};

  /// \brief Initialization flag.
  public: bool initialized{false};

  /// \brief Copy of the sdf configuration used for this plugin.
  public: sdf::ElementPtr sdfConfig;

  /// \brief Name of the model to which the sensor is attached.
  public: std::string modelName;

  /// \brief Message for visualizing contact positions
  public: ignition::msgs::Marker positionMarkerMsg;

  /// \brief Radius of the visualized contact sphere
  public: double contactRadius{0.10};

  /// \brief Message for visualizing contact forces
  public: ignition::msgs::Marker forceMarkerMsg;

  /// \brief Length of the visualized force cylinder
  public: double forceLenght{0.20};

  /// \brief Position interpolated from the Contact messages
  public: std::vector<ignition::math::Vector3d> interpolatedPosition;

  /// \brief Pose of the sensor model
  public: ignition::math::Pose3f sensorWorldPose;

  /// \brief Whether a new message has been returned by the Depth Camera
  public: bool newCameraMsg{false};

  /// \brief Depth Camera update rate
  public: float cameraUpdateRate{1};

  /// \brief Message returned by the Depth Camera
  public: ignition::msgs::PointCloudPacked cameraMsg;

  /// \brief 3D vector to store unpacked XYZ points
  std::vector<std::vector<std::vector<float>>> imageXYZ;
};

//////////////////////////////////////////////////
void OpticalTactilePluginPrivate::Load(const EntityComponentManager &_ecm,
                                       const sdf::ElementPtr &_sdf)
{
    igndbg << "Loading plugin [OpticalTactilePlugin]" << std::endl;

    // Get sdf parameters
    if (!_sdf->HasElement("resolution"))
    {
        ignerr << "Missing required parameter <resolution>." << std::endl;
        return;
    }
    else
    {
        this->resolution = _sdf->Get<int>("resolution");
    }

    if (!_sdf->HasElement("visualize_forces"))
    {
        ignerr << "Missing required parameter <visualize_forces>." << std::endl;
        return;
    }
    else
    {
        this->visualize_forces = _sdf->Get<bool>("visualize_forces");
    }

    // Get all the sensors
    auto allLinks =
        _ecm.ChildrenByComponents(this->model.Entity(), components::Link());

    for (const Entity linkEntity : allLinks)
    {
        auto linkCollisions =
            _ecm.ChildrenByComponents(linkEntity, components::Collision());
        for (const Entity colEntity : linkCollisions)
        {
            if (_ecm.EntityHasComponentType(colEntity,
                components::ContactSensorData::typeId))
            {
                this->sensorEntities.push_back(colEntity);

                this->modelName = this->model.Name(_ecm);

                igndbg << "Sensor detected within model "
                    << this->modelName << std::endl;
            }
        }
    }
}

//////////////////////////////////////////////////
void OpticalTactilePluginPrivate::Enable(const bool _value)
{
    // Just a placeholder method for the moment
    _value;
}

//////////////////////////////////////////////////
void OpticalTactilePlugin::Update(const UpdateInfo &_info,
                                  EntityComponentManager &_ecm)
{
    // Nothing left to do if paused or not allowed to update
    if (_info.paused)
      return;

    IGN_PROFILE("TouchPluginPrivate::Update");

    // Delete old interpolated positions
    this->dataPtr->interpolatedPosition.clear();

    for (const Entity colEntity : this->dataPtr->filteredCollisionEntities)
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
    if (this->dataPtr->newCameraMsg)
    {
      this->dataPtr->UnpackPointCloudMsg(this->dataPtr->cameraMsg);
      this->dataPtr->ComputeNormalForces(this->dataPtr->cameraMsg,
          this->dataPtr->visualize_forces);

      this->dataPtr->newCameraMsg = false;
    }
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
    for (Entity entity : _entities)
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
    this->dataPtr->sdfConfig = _sdf->Clone();
    this->dataPtr->model = Model(_entity);

    if (!this->dataPtr->model.Valid(_ecm))
    {
      ignerr << "Touch plugin should be attached to a model entity. "
            << "Failed to initialize." << std::endl;
      return;
    }

    // Configure subscriber for Depth Camera images
    sdf::ElementPtr depthCameraSdf =
      _sdf->GetParent()->GetElement("link")->GetElement("sensor");
    std::string topic = "/depth_camera/points";

    if (depthCameraSdf->HasElement("topic"))
      topic = "/" + depthCameraSdf->Get<std::string>("topic") + "/points";

    if (!this->dataPtr->node.Subscribe(topic,
      &OpticalTactilePluginPrivate::DepthCameraCallback,
      this->dataPtr.get()))
    {
      ignerr << "Error subscribing to topic " << "[" << topic << "]. "
      "<topic> must not contain '/'" << std::endl;
      return;
    }

    // Configure Marker messages for position and force of the contacts
    // Blue spheres for positions
    // Green cylinders for forces

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

    // Set scales
    ignition::msgs::Set(this->dataPtr->positionMarkerMsg.mutable_scale(),
        ignition::math::Vector3d(this->dataPtr->contactRadius,
        this->dataPtr->contactRadius,
        this->dataPtr->contactRadius));

    ignition::msgs::Set(this->dataPtr->forceMarkerMsg.mutable_scale(),
        ignition::math::Vector3d(0.02, 0.02, this->dataPtr->forceLenght));
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
    const ignition::gazebo::EntityComponentManager &)
{
    IGN_PROFILE("TouchPlugin::PostUpdate");

    // Nothing left to do if paused.
    if (_info.paused)
      return;
}

//////////////////////////////////////////////////
void OpticalTactilePluginPrivate::DepthCameraCallback(
        const ignition::msgs::PointCloudPacked &_msg)
{
  // This check avoids running the callback at t=0 and getting
  // unexpected markers in the scene
  if (!this->initialized)
      return;

  this->cameraMsg = _msg;
  this->newCameraMsg = true;
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

  for (uint32_t j = 0; j < _msg.height(); ++j)
  {
    for (uint32_t i = 0; i < _msg.width(); ++i)
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

  int index = 1;


  for (uint32_t j = 1; j < (_msg.height() - 1); ++j)
  {
    for (uint32_t i = 1; i < (_msg.width() - 1); ++i)
    {
      // Compute normal forces
      double dxdi =
          (this->imageXYZ[i+1][j][0] -
          this->imageXYZ[i-1][j][0])/
          std::abs(this->imageXYZ[i+1][j][1] -
          this->imageXYZ[i-1][j][1]);

      double dxdj =
          (this->imageXYZ[i][j+1][0]
          - this->imageXYZ[i][j-1][0])/
          std::abs(this->imageXYZ[i][j+1][2]
          - this->imageXYZ[i][j-1][2]);

      ignition::math::Vector3f direction(-1, -dxdi, -dxdj);

      // todo(anyone) multiply vector by contact forces info
      normalForces[i][j] = direction.Normalized();

      if (!_visualizeForces)
          continue;

      // Downsampling for visualization
      if (visualizeRow && visualizeColumn)
        {
          // Add new markers with specific lifetime
          this->positionMarkerMsg.set_id(index);
          this->positionMarkerMsg.set_action(
              ignition::msgs::Marker::ADD_MODIFY);
          this->positionMarkerMsg.mutable_lifetime()->set_sec(
              this->cameraUpdateRate);
          this->positionMarkerMsg.mutable_lifetime()->set_nsec(0);

          this->forceMarkerMsg.set_id(index++);
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
              normalForces[i][j] * (this->forceLenght/2),
              forceMarkerSensorQuaternion);

          ignition::math::Pose3f forceMarkerWorldPose =
              forceMarkerSensorPose + this->sensorWorldPose;
          forceMarkerWorldPose.Correct();

          ignition::math::Pose3f positionMarkerSensorPose(
              forceMarkerSensorPosition, forceMarkerSensorQuaternion);
          ignition::math::Pose3f positionMarkerWorldPose =
              positionMarkerSensorPose + this->sensorWorldPose;
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
                    OpticalTactilePlugin::ISystemUpdate,
                    OpticalTactilePlugin::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(OpticalTactilePlugin,
    "ignition::gazebo::systems::OpticalTactilePlugin")
