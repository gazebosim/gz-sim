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

#include <sdf/Sensor.hh>
#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/EnvironmentalData.hh>
#include <gz/sim/Util.hh>

#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/SensorTypes.hh>
#include <gz/sensors/Util.hh>

using namespace gz::sim;

class EnvironmentalSensor : public gz::sensors::Sensor
{
   /// \brief Node for communication
  private: gz::transport::Node node;

  /// \brief Publishes sensor data
  private: gz::transport::Node::Publisher pub;

  private: const std::string SENSOR_TYPE_PREFIX =  "environmental_sensor/";
  public: virtual bool Load(const sdf::Sensor &_sdf) override
  {
    auto type = gz::sensors::customType(_sdf);
    if (type.substr(0, SENSOR_TYPE_PREFIX.size()) == SENSOR_TYPE_PREFIX ||
      type.size() <= SENSOR_TYPE_PREFIX.size())
    {
      gzerr << "Trying to load [" << SENSOR_TYPE_PREFIX << "] sensor, but got type ["
            << type << "] instead." << std::endl;
      return false;
    }

    this->pub = this->node.Advertise<gz::msgs::Double>(this->Topic());

    this->field = type.substr(SENSOR_TYPE_PREFIX.size());

    gzdbg << "Loaded environmental sensor for " << this->field<< std::endl;

    // Load common sensor params
    gz::sensors::Sensor::Load(_sdf);
    return true;
  }

  /// \brief Update the sensor and generate data
  /// \param[in] _now The current time
  /// \return True if the update was successful
  public: virtual bool Update(
    const std::chrono::steady_clock::duration &_now) override
  {
    if (!data_set) return false;

    gz::msgs::Double msg;
    *msg.mutable_header()->mutable_stamp() = gz::msgs::Convert(_now);
    auto frame = msg.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value(this->Name());
    msg.set_data();
    //TODO(anyone) Add sensor noise.
    this->pub.Publish(msg);
    return true;
  }

  public: void TrackComponent(const components::EnvironmentalData* _data)
  {
    auto data = _data->Data();
   
  }

  public: void SetPose(const double& _data)
  {
  }

  public: std::string Field() const
  {
  }

  private: std::string field;
  private: std::optional<gz::math::InMemorySession<double, double>> session;
  private: gz::math::InMemoryTimeVaryingVolumetricGrid<double, double, double>
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
            gzerr << "Internal error, missing odometer for entity ["
                          << _entity << "]" << std::endl;
          }
          return true;
        });
  }

  public: Entity worldEntity;

  public: bool useSphericalCoords{false};
};

EnvironmentalSensorSystem::EnvironmentalSensorSystem () :
  dataPtr(std::make_unique<EnvironmentalSensorSystemPrivate>())
{

}

void EnvironmentalSensorSystem::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &/*_eventMgr*/)
{
  dataPtr->worldEntity = _entity;

  if (_sdf->HasElement("use_spherical_coordinates"))
  {
    this->dataPtr->useSphericalCoords =
      _sdf->Get<bool>("use_spherical_coordinates");
  }
}

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

        // Default to scoped name as topic
        if (data.Topic().empty())
        {
          std::string topic = scopedName(_entity, _ecm) + type;
          data.SetTopic(topic);
        }

        gz::sensors::SensorFactory sensorFactory;
        auto sensor = sensorFactory.CreateSensor<EnvironmentalSensor>(data);
        if (nullptr == sensor)
        {
          gzerr <<
            "Failed to create environmental sensor [" << sensorScopedName << "]"
            << std::endl;
          return false;
        }

        // Set sensor parent
        auto parentName = _ecm.Component<components::Name>(
            _parent->Data())->Data();
        sensor->SetParent(parentName);

        // Set topic on Gazebo
        _ecm.CreateComponent(_entity,
            components::SensorTopic(sensor->Topic()));

        // Keep track of this sensor
        this->dataPtr->entitySensorMap.insert(std::make_pair(_entity,
            std::move(sensor)));

        return true;
      });
}

void EnvironmentalSensorSystem::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{

  // Only update and publish if not paused.
  if (_info.paused)
  {

    for (auto &[entity, sensor] : this->dataPtr->entitySensorMap)
    {
      auto position = worldPose(entity, _ecm).Pos();
      //sensor->SetData();
      sensor->Update(_info.simTime);
    }
  }

  this->dataPtr->RemoveSensorEntities(_ecm);
}

GZ_ADD_PLUGIN(
  EnvironmentalSensorSystem, System,
  EnvironmentalSensorSystem::ISystemConfigure,
  EnvironmentalSensorSystem::ISystemPreUpdate
)

GZ_ADD_PLUGIN_ALIAS(
  EnvironmentalSensorSystem,
  "gz::sim::systems::EnvironmentalSystem")
