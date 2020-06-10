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
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Sensor.hh"

#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"

#include <ignition/transport.hh>
#include <ignition/math.hh>
#include <ignition/msgs.hh>

#include <google/protobuf/message.h>

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

    /// \brief Visualize the sensor's data using the Marker message
    public: void VisualizeSensorData(std::vector<ignition::math::Vector3d> &positions);

    /// \brief Interpolates contact data
    public: void InterpolateData(const ignition::msgs::Contact &contactMsg);

    /// \brief Resolution of the sensor in mm.
    public: double resolution;

    /// \brief Model interface.
    public: Model model{kNullEntity};

    /// \brief Transport node to visualize data on Gazebo.
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
    
    /// \brief Message for visualizing contact positions
    public: ignition::msgs::Marker positionMarkerMsg;

    /// \brief Radius of the visualized contact sphere
    public: double contactRadius{0.20};

    /// \brief Message for visualizing contact forces
    public: ignition::msgs::Marker forceMarkerMsg;

    /// \brief Length of the visualized force cylinder
    public: double forceLenght{0.20};

    /// \brief Position interpolated from the Contact messages
    public: std::vector<ignition::math::Vector3d> interpolatedPosition;

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
        // resolution is specified in mm
        this->resolution = _sdf->Get<double>("resolution")/1000;
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
                // Interpolate data returned by the Contact message
                this->InterpolateData(contact);
          
                // Visualize interpolated data
                this->VisualizeSensorData(this->interpolatedPosition);

            }

            //ignition::common::Time::Sleep(ignition::common::Time(0.1));
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
void OpticalTactilePluginPrivate::InterpolateData(const ignition::msgs::Contact &contact)
{
    // Delete old positions
    this->interpolatedPosition.clear();

    // Interpolate new ones
    if (contact.position_size() == 4)
    {
        igndbg << "Interpolate 4 position" << std::endl;
        ignition::math::Vector3d contact1 = ignition::msgs::Convert(contact.position(0));
        ignition::math::Vector3d contact2 = ignition::msgs::Convert(contact.position(1));
        ignition::math::Vector3d contact3 = ignition::msgs::Convert(contact.position(2));
        ignition::math::Vector3d contact4 = ignition::msgs::Convert(contact.position(3));

        ignition::math::Vector3d direction1 = (contact2 - contact1).Normalized() * this->resolution;
        ignition::math::Vector3d direction2 = (contact4 - contact1).Normalized() * this->resolution;

        ignition::math::Vector3d interpolatedVector = contact1;
        this->interpolatedPosition.push_back(contact1);

        // auxiliary Vector3d to iterate through contacts
        ignition::math::Vector3d tempVector (0.0, 0.0, 0.0);

        long unsigned int steps1 = contact1.Distance(contact2) / this->resolution;
        long unsigned int steps2 = contact1.Distance(contact4) / this->resolution;
        
        igndbg << "\n contact1: " << contact1 << "\n"
        << "contact2: " << contact2 << "\n" 
        << "contact3: " << contact3 << "\n"
        << "contact4: " << contact4 << "\n"
        << "direction1: " << direction1 << "\n"
        << "direction2: " << direction2 << "\n";
        
        igndbg << "steps1 = " << steps1 << std::endl;
        igndbg << "contact1.Distance(contact2) = " << contact1.Distance(contact2) << std::endl;
        igndbg << "this->resolution = " << this->resolution << std::endl;

        igndbg << "steps2 = " << steps2 << std::endl;
        igndbg << "contact1.Distance(contact4) = " << contact1.Distance(contact4) << std::endl;
        igndbg << "this->resolution = " << this->resolution << std::endl;
        igndbg << "interpolatedVector: " << interpolatedVector << std::endl;
        for (long unsigned int index1 = 0; index1 <= steps1; ++index1)
        {
            tempVector = interpolatedVector;
            for (long unsigned int index2 = 0; index2 < steps2; ++index2)
            {
                interpolatedVector += direction2;
                igndbg << "interpolatedVector: " << interpolatedVector << std::endl;
                this->interpolatedPosition.push_back(interpolatedVector);
            }
            if (index1 != steps1)
            {
                interpolatedVector = tempVector;
                interpolatedVector += direction1;
                igndbg << "interpolatedVector: " << interpolatedVector << std::endl;
                this->interpolatedPosition.push_back(interpolatedVector);  
            }         
        }
    }   
}

//////////////////////////////////////////////////
void OpticalTactilePluginPrivate::VisualizeSensorData(std::vector<ignition::math::Vector3d> &positions)
{
    // Delete previous shapes already in simulation
    this->positionMarkerMsg.set_action(ignition::msgs::Marker::DELETE_ALL);
    this->forceMarkerMsg.set_action(ignition::msgs::Marker::DELETE_ALL);
    this->node.Request("/marker",this->positionMarkerMsg);
    this->node.Request("/marker",this->forceMarkerMsg);

    // Add the new ones
    this->positionMarkerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
    this->forceMarkerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);

    for (int index = 0; index < positions.size(); ++index)
    {
        this->positionMarkerMsg.set_id(index);
        this->forceMarkerMsg.set_id(index);

        ignition::msgs::Set(this->positionMarkerMsg.mutable_pose(),
                        ignition::math::Pose3d(positions[index].X(), positions[index].Y(), 
                        positions[index].Z(), 0, 0, 0));
        ignition::msgs::Set(this->forceMarkerMsg.mutable_pose(),
                        ignition::math::Pose3d(positions[index].X(), positions[index].Y(), 
                        positions[index].Z() + this->forceLenght, 0, 0, 0));

        this->node.Request("/marker",this->positionMarkerMsg);
        this->node.Request("/marker",this->forceMarkerMsg);
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

    // Configure Marker messages for position and force of the contacts
    // Blue spheres for positions
    // Green cylinders for forces

    // Create the marker message
    this->dataPtr->positionMarkerMsg.set_ns("positions");
    this->dataPtr->positionMarkerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
    this->dataPtr->positionMarkerMsg.set_type(ignition::msgs::Marker::SPHERE);
    this->dataPtr->positionMarkerMsg.set_visibility(ignition::msgs::Marker::GUI);

    this->dataPtr->forceMarkerMsg.set_ns("forces");
    this->dataPtr->forceMarkerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
    this->dataPtr->forceMarkerMsg.set_type(ignition::msgs::Marker::CYLINDER);
    this->dataPtr->forceMarkerMsg.set_visibility(ignition::msgs::Marker::GUI);

    // Set material properties 
    this->dataPtr->positionMarkerMsg.mutable_material()->mutable_ambient()->set_r(0);
    this->dataPtr->positionMarkerMsg.mutable_material()->mutable_ambient()->set_g(0);
    this->dataPtr->positionMarkerMsg.mutable_material()->mutable_ambient()->set_b(1);
    this->dataPtr->positionMarkerMsg.mutable_material()->mutable_ambient()->set_a(1);
    this->dataPtr->positionMarkerMsg.mutable_material()->mutable_diffuse()->set_r(0);
    this->dataPtr->positionMarkerMsg.mutable_material()->mutable_diffuse()->set_g(0);
    this->dataPtr->positionMarkerMsg.mutable_material()->mutable_diffuse()->set_b(1);
    this->dataPtr->positionMarkerMsg.mutable_material()->mutable_diffuse()->set_a(1);
    this->dataPtr->positionMarkerMsg.mutable_lifetime()->set_sec(2);
    this->dataPtr->positionMarkerMsg.mutable_lifetime()->set_nsec(0);

    this->dataPtr->forceMarkerMsg.mutable_material()->mutable_ambient()->set_r(0);
    this->dataPtr->forceMarkerMsg.mutable_material()->mutable_ambient()->set_g(1);
    this->dataPtr->forceMarkerMsg.mutable_material()->mutable_ambient()->set_b(0);
    this->dataPtr->forceMarkerMsg.mutable_material()->mutable_ambient()->set_a(1);
    this->dataPtr->forceMarkerMsg.mutable_material()->mutable_diffuse()->set_r(0);
    this->dataPtr->forceMarkerMsg.mutable_material()->mutable_diffuse()->set_g(1);
    this->dataPtr->forceMarkerMsg.mutable_material()->mutable_diffuse()->set_b(0);
    this->dataPtr->forceMarkerMsg.mutable_material()->mutable_diffuse()->set_a(1);
    this->dataPtr->forceMarkerMsg.mutable_lifetime()->set_sec(2);
    this->dataPtr->forceMarkerMsg.mutable_lifetime()->set_nsec(0);

    // Set scales
    ignition::msgs::Set(this->dataPtr->positionMarkerMsg.mutable_scale(),
                        ignition::math::Vector3d(this->dataPtr->contactRadius, this->dataPtr->contactRadius, 
                        this->dataPtr->contactRadius));

    ignition::msgs::Set(this->dataPtr->forceMarkerMsg.mutable_scale(),
                        ignition::math::Vector3d(0.05, 0.05, this->dataPtr->forceLenght));
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