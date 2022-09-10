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

#include <gz/plugin/Register.hh>

#include <gz/common/StringUtils.hh>

#include <sdf/Sensor.hh>
#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Environment.hh>
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

    this->pub = this->node.Advertise<gz::msgs::Double>(this->Topic());

    this->field = type.substr(strlen(SENSOR_TYPE_PREFIX));

    gzdbg << "Loaded environmental sensor for " << this->field
      << " publishing on " << this->Topic() << std::endl;

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

    if (!this->session.has_value()) return false;

    this->session = this->gridField->frame[this->field].StepTo(
      this->session.value(), std::chrono::duration<double>(_now).count());

    if (!this->session.has_value()) return false;

    gz::msgs::Double msg;
    *msg.mutable_header()->mutable_stamp() = gz::msgs::Convert(_now);
    auto frame = msg.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value(this->Name());
    auto data = this->gridField->frame[this->field].LookUp(
      this->session.value(), this->position);
    if (!data.has_value())
      return false;
    msg.set_data(data.value());
    // TODO(anyone) Add sensor noise.
    this->pub.Publish(msg);
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
    if(!data->frame.Has(this->field))
    {
      gzwarn << "Environmental sensor could not find field "
        << this->field << "\n";
      this->ready = false;
      return;
    }

    this->gridField = data;
    this->session = this->gridField->frame[this->field].CreateSession();
    this->session = this->gridField->frame[this->field].StepTo(
      *this->session,
      std::chrono::duration<double>(_curr_time).count());
    this->ready = true;

    if(!this->session.has_value())
    {
      gzerr << "Exceeded time stamp." << std::endl;
    }
  }

  ////////////////////////////////////////////////////////////////
  /// \brief Updates the position of the sensor at each time step
  /// to track a given entity,
  /// \param[in] - The sensor entity.
  /// \param[in] - The ECM to use for tracking.
  public: bool UpdatePosition(
    const Entity _entity,
    const EntityComponentManager& _ecm)
  {
    const auto position = worldPose(_entity, _ecm);
    if (this->gridField->reference != math::SphericalCoordinates::LOCAL2)
    {
        auto origin = _ecm.Component<components::SphericalCoordinates>(worldEntity(_ecm));
        if (!origin) 
        {
            return false;
        }
        this->position = origin->Data().PositionTransform(
            position, math::SphericalCoordinates::LOCAL2, this->gridField->reference);
    }
    else
    {
        this->position = position;
    }
    return true;
  }

  ////////////////////////////////////////////////////////////////
  public: std::string Field() const
  {
    return field;
  }

  private: bool ready {false};
  private: math::Vector3d position;
  private: std::string field;
  private: std::optional<gz::math::InMemorySession<double, double>> session;
  private: std::shared_ptr<gz::sim::v7::components::EnvironmentalData>
    gridField;
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
  const std::shared_ptr<const sdf::Element> &_sdf,
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
        else
        {
          gzerr << data.Topic() << std::endl;
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
