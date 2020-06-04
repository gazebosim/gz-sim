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
    /// \param[in] _topic string with topic name
    /// \param[in] _collisionEntities A list of entities that act as contact
    /// sensors
    public: void Load(const sdf::ElementPtr &_sdf, const std::string &_topic,
                        const std::vector<Entity> &_collisionEntities);

    /// \brief Actual function that enables the plugin.
    /// \param[in] _value True to enable plugin.
    public: void Enable(const bool _value);

    /// \brief Process contact sensor data and determine if a touch event occurs
    /// \param[in] _info Simulation update info
    /// \param[in] _ecm Immutable reference to the EntityComponentManager
    public: void Update(const UpdateInfo &_info,
                        const EntityComponentManager &_ecm);

};

//////////////////////////////////////////////////
void OpticalTactilePluginPrivate::Load(const sdf::ElementPtr &_sdf, const std::string &_topic,
                                    const std::vector<Entity> &_collisionEntities)
{

}

//////////////////////////////////////////////////
void OpticalTactilePluginPrivate::Enable(const bool _value)
{

}

//////////////////////////////////////////////////
void OpticalTactilePluginPrivate::Update(const UpdateInfo &_info,
                        const EntityComponentManager &_ecm)
{

}