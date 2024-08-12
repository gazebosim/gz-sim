/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#include "EnvironmentalSensorSystem.hh"
#include "TransformTypes.hh"

#include <gz/plugin/Register.hh>

#include <gz/common/StringUtils.hh>

#include <sdf/Sensor.hh>
#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/Environment.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/SphericalCoordinates.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/Util.hh>

#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/SensorTypes.hh>
#include <gz/sensors/Util.hh>

#include <gz/math/Pose3.hh>

#include <chrono>
#include <unordered_map>
#include <utility>
#include <unordered_set>
#include <memory>
#include <string>

using namespace gz;
using namespace gz::sim;

/// Sensor prefix to be used. All envionment_sensors are to be prefixed by
/// "environment_sensor/" in their gz:type field.
constexpr char SENSOR_TYPE_PREFIX[] =  "environmental_sensor/";

////////////////////////////////////////////////////////////////
/// \brief Envirtonment Sensor used for looking up environment values in our
/// CSV file.
class EnvironmentalSensor : public gz::sensors::Sensor
{
  /// \brief Node for communication
  private: gz::transport::Node node;

  /// \brief Publishes sensor data
  private: gz::transport::Node::Publisher pub;

  ////////////////////////////////////////////////////////////////
  /// Documentation inherited
  public: virtual bool Load(const sdf::Sensor &_sdf) override
  {
    auto type = gz::sensors::customType(_sdf);
    if (!common::StartsWith(type, SENSOR_TYPE_PREFIX))
    {
      gzerr << "Trying to load [" << SENSOR_TYPE_PREFIX
        << "] sensor, but got type ["
        << type << "] instead." <<  std::endl;
      return false;
    }

    // Load common sensor params
    gz::sensors::Sensor::Load(_sdf);

    // Handle the field type
    if (_sdf.Element() != nullptr &&
      _sdf.Element()->HasElement("output_format"))
    {
      std::string format = _sdf.Element()->Get<std::string>("output_format");
      if (format == "scalar")
      {
        this->numberOfFields = 1;
      }
      else if (format == "vector3")
      {
        this->numberOfFields = 3;
      }
      else
      {
        gzerr << "Invalid output format " << format << "given."
          << "valid options are scalar or vector3. "
          << "Defaulting to 1." << std::endl;
      }
    }

    // Handle the transform type
    if (_sdf.Element() != nullptr &&
      _sdf.Element()->HasElement("transform_type"))
    {
      auto res =
        getTransformType(_sdf.Element()->Get<std::string>("transform_type"));

      if (!res.has_value())
      {
        gzerr << "Invalid transform type of "
          << _sdf.Element()->Get<std::string>("transform_type")
          << " given. Valid types are ADD_VELOCITY_LOCAL, ADD_VELOCITY_GLOBAL,"
          << " LOCAL, and GLOBAL. Defaulting to GLOBAL." << std::endl;
      }
      else
      {
        this->transformType = res.value();
      }
    }

    switch(this->numberOfFields) {
      case 1:
        this->pub = this->node.Advertise<gz::msgs::Double>(this->Topic());
        break;
      case 3:
        this->pub = this->node.Advertise<gz::msgs::Vector3d>(this->Topic());
        break;
      default:
        gzerr << "Unable to create publisher too many fields" << "\n";
    }

    // If "environment_variable" is defined then remap
    // sensor from existing data.
    if (_sdf.Element() != nullptr &&
      this->numberOfFields == 1 &&
      _sdf.Element()->HasElement("environment_variable"))
    {
      this->fieldName[0] =
        _sdf.Element()->Get<std::string>("environment_variable");
    }
    else if (_sdf.Element() != nullptr &&
      this->numberOfFields == 3)
    {
      if (_sdf.Element()->HasElement("environment_variable_x"))
      {
        this->fieldName[0] =
          _sdf.Element()->Get<std::string>("environment_variable_x");
      }
      if (_sdf.Element()->HasElement("environment_variable_y"))
      {
        this->fieldName[1] =
          _sdf.Element()->Get<std::string>("environment_variable_y");
      }
      if (_sdf.Element()->HasElement("environment_variable_z"))
      {
        this->fieldName[2] =
          _sdf.Element()->Get<std::string>("environment_variable_z");
      }
    }
    else
    {
      this->fieldName[0] = type.substr(strlen(SENSOR_TYPE_PREFIX));
    }

    // Allow setting custom frame_ids
    if (_sdf.Element() != nullptr &&
      _sdf.Element()->HasElement("frame_id"))
    {
      this->frameId = _sdf.Element()->Get<std::string>("frame_id");
    }

    if (this->numberOfFields == 1)
    {
      gzdbg << "Loaded environmental sensor for " << this->fieldName[0]
        << " publishing on " << this->Topic() << std::endl;
    }
    else
    {
      gzdbg << "Loaded environmental sensor for [" << this->fieldName[0] << ", "
        << this->fieldName[1] << ", " << this->fieldName[2]
        << "] publishing on " << this->Topic() << std::endl;
    }


    return true;
  }

  ////////////////////////////////////////////////////////////////
  /// \brief Update the sensor and generate data
  /// \param[in] _now The current time
  /// \return True if the update was successful
  public: virtual bool Update(
    const std::chrono::steady_clock::duration &_now) override
  {
    if (!this->ready) return false;


    std::optional<double> dataPoints[3];
    for (std::size_t i = 0; i < this->numberOfFields; ++i)
    {
      if (this->fieldName[i] == "")
      {
        // Empty field name means the column should default to zero.
        dataPoints[i] = 0;
        continue;
      }

      if (!this->session[i].has_value()) return false;

      // Step time if its not static
      if (!this->gridField->staticTime)
        this->session[i] = this->gridField->frame[this->fieldName[i]].StepTo(
          this->session[i].value(),
          std::chrono::duration<double>(_now).count());

      if (!this->session[i].has_value()) return false;

      dataPoints[i] = this->gridField->frame[this->fieldName[i]].LookUp(
        this->session[i].value(), this->position);
    }

    if (this->numberOfFields == 1) {
      gz::msgs::Double msg;
      *msg.mutable_header()->mutable_stamp() = gz::msgs::Convert(_now);
      auto frame = msg.mutable_header()->add_data();
      frame->set_key("frame_id");
      frame->add_value((this->frameId == "") ? this->Name() : this->frameId);
      auto data = dataPoints[0];
      if (!data.has_value())
      {
        gzwarn << "Failed to acquire value perhaps out of field?\n";
        return false;
      }
      msg.set_data(data.value());
      // TODO(anyone) Add sensor noise.
      this->pub.Publish(msg);
    }
    else if (this->numberOfFields == 3)
    {
      gz::msgs::Vector3d msg;
      *msg.mutable_header()->mutable_stamp() = gz::msgs::Convert(_now);
      auto frame = msg.mutable_header()->add_data();
      frame->set_key("frame_id");
      frame->add_value((this->frameId == "") ? this->Name() : this->frameId);

      if (!dataPoints[0].has_value() || !dataPoints[1].has_value()
        || !dataPoints[2].has_value())
      {
        gzwarn << "Failed to acquire value perhaps out of field?\n";
        return false;
      }

      math::Vector3d vector(
        dataPoints[0].value(), dataPoints[1].value(), dataPoints[2].value());
      auto transformed = transformFrame(this->transformType, this->objectPose,
        this->velocity, vector);

      msg.set_x(transformed.X());
      msg.set_y(transformed.Y());
      msg.set_z(transformed.Z());
      this->pub.Publish(msg);
    }
    return true;
  }

  ////////////////////////////////////////////////////////////////
  /// \brief Attempts to set a data table.
  /// \param[in] _data - The data table
  /// \param[in] _curr_time - The current time.
  public: void SetDataTable(
    const components::Environment* _data,
    const std::chrono::steady_clock::duration &_curr_time)
  {
    gzdbg << "Setting new data table\n";
    auto data = _data->Data();
    for (std::size_t i = 0; i < this->numberOfFields; ++i)
    {
      // If field name is empty it was intentionally left out so the column
      // should default to zero. Otherwise if wrong name throw an error and
      // disable output.
      if (this->fieldName[i] != "" && !data->frame.Has(this->fieldName[i]))
      {
        gzwarn << "Environmental sensor could not find field "
          << this->fieldName[i] << "\n";
        this->ready = false;
        return;
      }
    }

    this->gridField = data;
    for (std::size_t i = 0; i < this->numberOfFields; ++i)
    {
      if (this->fieldName[i] == "")
        continue;

      this->session[i] =
        this->gridField->frame[this->fieldName[i]].CreateSession();
      if (!this->gridField->staticTime)
      {
        this->session[i] = this->gridField->frame[this->fieldName[i]].StepTo(
          *this->session[i],
          std::chrono::duration<double>(_curr_time).count());
      }

      if(!this->session[i].has_value())
      {
        gzerr << "Exceeded time stamp." << std::endl;
      }
    }

    this->ready = true;
  }

  ////////////////////////////////////////////////////////////////
  /// \brief Updates the position of the sensor at each time step
  /// to track a given entity,
  /// \param[in] - The sensor entity.
  /// \param[in] - The ECM to use for tracking.
  /// \returns True if successful, false if not.
  public: bool UpdatePosition(
    const Entity _entity,
    const EntityComponentManager& _ecm)
  {
    if (!this->ready) return false;

    this->objectPose = worldPose(_entity, _ecm);
    const auto worldPosition = this->objectPose.Pos();
    auto lookupCoords =
      getGridFieldCoordinates(_ecm, worldPosition, this->gridField);

    if (!lookupCoords.has_value())
    {
      return false;
    }

    auto vel = _ecm.Component<components::WorldLinearVelocity>(_entity);
    if (vel != nullptr)
    {
      this->velocity = vel->Data();
    }
    this->position = lookupCoords.value();
    return true;
  }

  ////////////////////////////////////////////////////////////////
  public: std::string Field() const
  {
    return fieldName[0];
  }

  private: bool ready {false};
  private: math::Vector3d position, velocity;
  private: math::Pose3d objectPose;
  private: std::size_t numberOfFields{1};
  private: std::string fieldName[3];
  private: std::string frameId;
  private: std::optional<gz::math::InMemorySession<double, double>> session[3];
  private: std::shared_ptr<gz::sim::components::EnvironmentalData>
    gridField;
  private: TransformType transformType{TransformType::GLOBAL};
};

class gz::sim::EnvironmentalSensorSystemPrivate {
  public: std::unordered_map<Entity, std::shared_ptr<EnvironmentalSensor>>
    entitySensorMap;

  public: std::unordered_set<std::string> fields;

  public: void RemoveSensorEntities(
    const gz::sim::EntityComponentManager &_ecm)
  {
    _ecm.EachRemoved<components::CustomSensor>(
      [&](const Entity &_entity,
          const components::CustomSensor *)->bool
        {
          if (this->entitySensorMap.erase(_entity) == 0)
          {
            gzerr << "Internal error, missing environment sensor for entity ["
                          << _entity << "]" << std::endl;
          }
          return true;
        });
  }

  public: Entity worldEntity;

  public: bool useSphericalCoords{false};
};


////////////////////////////////////////////////////////////////
EnvironmentalSensorSystem::EnvironmentalSensorSystem () :
  dataPtr(std::make_unique<EnvironmentalSensorSystemPrivate>())
{

}

////////////////////////////////////////////////////////////////
void EnvironmentalSensorSystem::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &/*_sdf*/,
  gz::sim::EntityComponentManager &/*_ecm*/,
  gz::sim::EventManager &/*_eventMgr*/)
{
  dataPtr->worldEntity = _entity;
}

////////////////////////////////////////////////////////////////
void EnvironmentalSensorSystem::PreUpdate(const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  _ecm.EachNew<components::CustomSensor, components::ParentEntity>(
    [&](const Entity &_entity,
        const components::CustomSensor *_custom,
        const components::ParentEntity *_parent)->bool
      {
        // Get sensor's scoped name without the world
        auto sensorScopedName =
          removeParentScope(scopedName(_entity, _ecm, "::", false), "::");
        sdf::Sensor data = _custom->Data();
        data.SetName(sensorScopedName);

        auto type = gz::sensors::customType(data);
        if (!common::StartsWith(type, SENSOR_TYPE_PREFIX))
        {
          return true;
        }

        // Default to scoped name as topic
        if (data.Topic().empty())
        {
          std::string topic = scopedName(_entity, _ecm) + "/" + type;
          data.SetTopic(topic);
        }

        gz::sensors::SensorFactory sensorFactory;
        auto sensor = sensorFactory.CreateSensor<EnvironmentalSensor>(data);

        // Set sensor parent
        auto parentName = _ecm.Component<components::Name>(
            _parent->Data())->Data();
        sensor->SetParent(parentName);

        // Set topic on Gazebo
        _ecm.CreateComponent(_entity,
            components::SensorTopic(sensor->Topic()));

        // Get current EnvironmentalData component
        auto environData =
          _ecm.Component<components::Environment>(
            worldEntity(_ecm));

        if (environData != nullptr)
        {
          sensor->SetDataTable(environData, _info.simTime);
        }
        else
        {
          gzerr << "No sensor data loaded\n";
        }

        enableComponent<components::WorldLinearVelocity>(_ecm, _entity);

        // Keep track of this sensor
        this->dataPtr->entitySensorMap.insert(std::make_pair(_entity,
            std::move(sensor)));
        return true;
      });
}

////////////////////////////////////////////////////////////////
void EnvironmentalSensorSystem::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  _ecm.EachNew<components::Environment>([&](const Entity &/*_entity*/,
    const components::Environment *_environmental_data)->bool
    {
      for (auto &[entity, sensor] : this->dataPtr->entitySensorMap)
      {
        sensor->SetDataTable(_environmental_data, _info.simTime);
      }
      return true;
    });
  // Only update and publish if not paused.
  if (!_info.paused)
  {

    for (auto &[entity, sensor] : this->dataPtr->entitySensorMap)
    {
      sensor->UpdatePosition(entity, _ecm);
      sensor->Update(_info.simTime);
    }
  }

  this->dataPtr->RemoveSensorEntities(_ecm);
}

GZ_ADD_PLUGIN(
  EnvironmentalSensorSystem, System,
  EnvironmentalSensorSystem::ISystemConfigure,
  EnvironmentalSensorSystem::ISystemPreUpdate,
  EnvironmentalSensorSystem::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(
  EnvironmentalSensorSystem,
  "gz::sim::systems::EnvironmentalSystem")
