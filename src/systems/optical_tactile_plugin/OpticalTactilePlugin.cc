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

#include <sdf/Element.hh>

#include "ignition/gazebo/components/ContactSensor.hh"
#include "ignition/gazebo/components/ContactSensorData.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Sensor.hh"

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

    /// \brief Process contact sensor data
    /// \param[in] _info Simulation update info
    /// \param[in] _ecm Immutable reference to the EntityComponentManager
    public: void Update(const UpdateInfo &_info,
                        const EntityComponentManager &_ecm);

    /// \brief Filters out new collisions fetched not related to the sensors
    public: void FilterOutCollisions(const EntityComponentManager &_ecm,
                                  const std::vector<Entity> &_entities);

    /// \brief Resolution of the sensor in mm.
    public: double resolution;

    /// \brief Model interface.
    public: Model model{kNullEntity};

    /// \brief Transport node to keep services alive.
    public: transport::Node node;

    /// \brief Publisher that publishes the sensors' contacts.
    public: std::optional<transport::Node::Publisher>sensorContactsPub;

    /// \brief Collision entities that have been designated as contact sensors.
    public: std::vector<Entity> sensorEntities;

    /// \brief Collision entities of the simulation.
    public: std::vector<Entity> collisionEntities;

    /// \brief Filtered collisions from the simulation that belong to one or more sensors.
    public: std::vector<Entity> filteredCollisionEntities;

    /// \brief Wheter the plugin is enabled.
    public: bool enabled{true};

    /// \brief Initialization flag.
    public: bool initialized{false};

    /// \brief Copy of the sdf configuration used for this plugin.
    public: sdf::ElementPtr sdfConfig;

    /// \brief Name of the model in which the sensor(s) is attached.
    public: std::string modelName;
    

};

//////////////////////////////////////////////////
void OpticalTactilePluginPrivate::Load(const EntityComponentManager &_ecm,
                         const sdf::ElementPtr &_sdf)
{

    igndbg << "Loading plugin [OpticalTactilePlugin]" << std::endl;

    // Get sdf parameters
    if (!_sdf->HasElement("resolution")) {
        ignerr << "Missing required parameter <resolution>." << std::endl;
        return;
    } 
    else 
    {
        this->resolution = _sdf->Get<double>("resolution");
        igndbg << "Sensor resolution: " << this->resolution << " mm" << std::endl;
    }

    // Get all the sensors
    auto allLinks =
        _ecm.ChildrenByComponents(this->model.Entity(), components::Link());

    for (const Entity linkEntity : allLinks)
    {
        auto linkCollisions=
            _ecm.ChildrenByComponents(linkEntity, components::Collision());
        for (const Entity colEntity : linkCollisions)
        {
            if (_ecm.EntityHasComponentType(colEntity,
                                            components::ContactSensorData::typeId))
            {
                this->sensorEntities.push_back(colEntity);

                this->modelName = this->model.Name(_ecm);

                igndbg << "Sensor detected within model " << this->modelName << std::endl;
            }
        }
    }


}

//////////////////////////////////////////////////
void OpticalTactilePluginPrivate::Enable(const bool _value)
{
    igndbg << "OpticalTactilePluginPrivate::Enable" << std::endl;
}

//////////////////////////////////////////////////
void OpticalTactilePluginPrivate::Update(const UpdateInfo &_info,
                        const EntityComponentManager &_ecm)
{
    IGN_PROFILE("TouchPluginPrivate::Update");
    
    // Print collision messages which have the sensor as one collision
    for (const Entity colEntity: this->filteredCollisionEntities)
    {
        auto* contacts = _ecm.Component<components::ContactSensorData>(colEntity);
        if (contacts)
        {
            for (const auto &contact : contacts->Data().contact())
            {
                ignmsg << "Collision between Entity " << contact.collision1().id()
                << " and " << contact.collision2().id() << std::endl;
            }
        }
    }
}

//////////////////////////////////////////////////
void OpticalTactilePluginPrivate::FilterOutCollisions(const EntityComponentManager &_ecm,
                                                      const std::vector<Entity> &_entities)
{
    if (_entities.empty())
        return;

    // Clear vector that contains old data
    this->filteredCollisionEntities.clear();

    // Get collisions from the sensor 
    for (Entity entity : _entities)
    {
        std::string name = scopedName(entity,_ecm);
        igndbg << "scopedName: " << name << std::endl;
        if (name.find(this->modelName) != std::string::npos)
        {
            igndbg << "Filtered collision that belongs to a sensor" << std::endl;
            this->filteredCollisionEntities.push_back(entity);
        }
    }
    
}

//////////////////////////////////////////////////
OpticalTactilePlugin::OpticalTactilePlugin()
            : System(), dataPtr(std::make_unique<OpticalTactilePluginPrivate>())
{
    igndbg << "OpticalTactilePlugin Constructor" << std::endl;
}

//////////////////////////////////////////////////
void OpticalTactilePlugin::Configure(const Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            EntityComponentManager &_ecm,
                            EventManager &_eventMgr)
{
    igndbg << "OpticalTactilePlugin::Configure" << std::endl;
    this->dataPtr->sdfConfig = _sdf->Clone();
    this->dataPtr->model = Model(_entity);
}

//////////////////////////////////////////////////
void OpticalTactilePlugin::PreUpdate(const UpdateInfo &_info,
                            EntityComponentManager &_ecm)
{
    IGN_PROFILE("TouchPluginPrivate::PreUpdate");

    if (!this->dataPtr->initialized)
    {
        this->dataPtr->Load(_ecm, this->dataPtr->sdfConfig);
        this->dataPtr->initialized = true;
    }

    // Update new collision messages

    // This is not an "else" because "initialized" can be set in the if block
    // above
    if (this->dataPtr->initialized)
    {
        // Fetch new collisions from simulation
        std::vector<Entity> newCollisions;
        _ecm.EachNew<components::Collision>(
            [&](const Entity &_entity, const components::Collision *) -> bool
            {
            igndbg << "Fetched new collision" << std::endl;
            newCollisions.push_back(_entity);
            return true;
            });

        this->dataPtr->FilterOutCollisions(_ecm, newCollisions);
    }
}

//////////////////////////////////////////////////
void OpticalTactilePlugin::PostUpdate(
                            const ignition::gazebo::UpdateInfo &_info,
                            const ignition::gazebo::EntityComponentManager &_ecm)
{
    IGN_PROFILE("TouchPluginPrivate::PostUpdate");

    this->dataPtr->Update(_info, _ecm);

}

IGNITION_ADD_PLUGIN(OpticalTactilePlugin,
                    ignition::gazebo::System,
                    OpticalTactilePlugin::ISystemConfigure,
                    OpticalTactilePlugin::ISystemPreUpdate,
                    OpticalTactilePlugin::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(OpticalTactilePlugin, "ignition::gazebo::systems::OpticalTactilePlugin")