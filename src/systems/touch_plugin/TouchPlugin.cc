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

#include "TouchPlugin.hh"

#include <algorithm>
#include <optional>
#include <string>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <sdf/Element.hh>

#include "gz/sim/components/ContactSensor.hh"
#include "gz/sim/components/ContactSensorData.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Sensor.hh"

#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::TouchPluginPrivate
{
  // Initialize the plugin
  public: void Load(const EntityComponentManager &_ecm,
                    const sdf::ElementPtr &_sdf);

  /// \brief Actual function that enables the plugin.
  /// \param[in] _value True to enable plugin.
  public: void Enable(const bool _value);

  /// \brief Process contact sensor data and determine if a touch event occurs
  /// \param[in] _info Simulation update info
  /// \param[in] _ecm Immutable reference to the EntityComponentManager
  public: void Update(const UpdateInfo &_info,
                      const EntityComponentManager &_ecm);

  /// \brief Add target entities. Called when new collisions are found
  /// \param[in] _ecm Immutable reference to the EntityComponentManager
  /// \param[in] _entities List of potential entities to add to targetEntities.
  public: void AddTargetEntities(const EntityComponentManager &_ecm,
                                 const std::vector<Entity> &_entities);

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Transport node to keep services alive
  transport::Node node;

  /// \brief Collision entities that have been designated as contact sensors.
  /// These will be checked against the targetEntities to establish whether this
  /// model is touching the targets
  public: std::vector<Entity> collisionEntities;

  /// \brief Name of target. Kept for debug printing
  public: std::string targetName;

  /// \brief Target collisions which this model should be touching.
  public: std::vector<Entity> targetEntities;

  /// \brief std::chrono::duration type used throught this plugin
  public: using DurationType = std::chrono::duration<double>;

  /// \brief Target time to continuously touch.
  public: DurationType targetTime{0};

  /// \brief Time when started touching.
  public: DurationType touchStart{0};

  /// \brief Namespace for transport topics.
  public: std::string ns;

  /// \brief Publisher which publishes a message after touched for enough time
  public: std::optional<transport::Node::Publisher> touchedPub;

  /// \brief Copy of the sdf configuration used for this plugin
  public: sdf::ElementPtr sdfConfig;

  /// \brief Initialization flag
  public: bool initialized{false};

  /// \brief Set during Load to true if the configuration for the plugin is
  /// valid and the pre and post update can run
  public: bool validConfig{false};

  /// \brief Whether the plugin is enabled.
  public: bool enabled{false};

  /// Value used to reset the world with the initial value
  public: bool enableInitialValue{false};

  /// \brief Mutex for variables mutated by the service callback.
  /// The variables are: touchPub, touchStart, enabled
  public: std::mutex serviceMutex;
};

//////////////////////////////////////////////////
void TouchPlugin::Reset(const gz::sim::UpdateInfo &/*_info*/,
  gz::sim::EntityComponentManager &/*_ecm*/)
{
  this->dataPtr->Enable(this->dataPtr->enableInitialValue);
}

//////////////////////////////////////////////////
void TouchPluginPrivate::Load(const EntityComponentManager &_ecm,
                         const sdf::ElementPtr &_sdf)
{
  // Get target substring
  if (!_sdf->HasElement("target"))
  {
    gzerr << "Missing required parameter <target>." << std::endl;
    return;
  }

  this->targetName = _sdf->GetElement("target")->Get<std::string>();

  std::vector<Entity> potentialEntities;
  _ecm.Each<components::Collision>(
      [&](const Entity &_entity, const components::Collision *) -> bool
      {
        potentialEntities.push_back(_entity);
        return true;
      });

  this->AddTargetEntities(_ecm, potentialEntities);

  // Create a list of collision entities that have been marked as contact
  // sensors in this model. These are collisions that have a ContactSensorData
  // component
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
        this->collisionEntities.push_back(colEntity);
      }
    }
  }

  // Namespace
  if (!_sdf->HasElement("namespace"))
  {
    gzerr << "Missing required parameter <namespace>" << std::endl;
    return;
  }
  this->ns = transport::TopicUtils::AsValidTopic(_sdf->Get<std::string>(
      "namespace"));
  if (this->ns.empty())
  {
    gzerr << "<namespace> [" << _sdf->Get<std::string>("namespace")
           << "] is invalid." << std::endl;
    return;
  }

  // Target time
  if (!_sdf->HasElement("time"))
  {
    gzerr << "Missing required parameter <time>" << std::endl;
    return;
  }

  this->targetTime = DurationType(_sdf->Get<double>("time"));

  // Start/stop "service"
  std::string enableService{"/" + this->ns + "/enable"};
  std::function<void(const msgs::Boolean &)> enableCb =
      [this](const msgs::Boolean &_req)
      {
        this->Enable(_req.data());
      };
  this->node.Advertise(enableService, enableCb);

  this->validConfig = true;

  // Start enabled or not
  if (_sdf->Get<bool>("enabled", false).first)
  {
    this->enableInitialValue = true;
    this->Enable(true);
  }
}

//////////////////////////////////////////////////
void TouchPluginPrivate::Enable(const bool _value)
{
  std::lock_guard<std::mutex> lock(this->serviceMutex);

  if (_value)
  {
    if (!this->touchedPub.has_value()){
      this->touchedPub = this->node.Advertise<msgs::Boolean>(
          "/" + this->ns + "/touched");
    }

    this->touchStart = DurationType::zero();
    this->enabled = true;

    gzdbg << "Started touch plugin [" << this->ns << "]" << std::endl;
  }
  else
  {
    this->enabled = false;

    gzdbg << "Stopped touch plugin [" << this->ns << "]" << std::endl;
  }
}

//////////////////////////////////////////////////
TouchPlugin::TouchPlugin()
    : System(), dataPtr(std::make_unique<TouchPluginPrivate>())
{
}

//////////////////////////////////////////////////
void TouchPluginPrivate::Update(const UpdateInfo &_info,
                                const EntityComponentManager &_ecm)
{
  GZ_PROFILE("TouchPluginPrivate::Update");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  {
    std::lock_guard<std::mutex> lock(this->serviceMutex);
    if (!this->enabled)
      return;
  }

  if (_info.paused)
    return;

  bool touching{false};
  // Iterate through all the target entities and check if there is a contact
  // between the target entity and this model
  for (const Entity colEntity : this->collisionEntities)
  {
    auto *contacts = _ecm.Component<components::ContactSensorData>(colEntity);
    if (contacts)
    {
      // Check if the contacts include one of the target entities.
      for (const auto &contact : contacts->Data().contact())
      {
        bool col1Target = std::binary_search(this->targetEntities.begin(),
            this->targetEntities.end(),
            contact.collision1().id());
        bool col2Target = std::binary_search(this->targetEntities.begin(),
            this->targetEntities.end(),
            contact.collision2().id());
        if (col1Target || col2Target)
        {
          touching = true;
        }
      }
    }
  }

  if (!touching)
  {
    std::lock_guard<std::mutex> lock(this->serviceMutex);
    if (this->touchStart != DurationType::zero())
    {
      gzdbg << "Model [" << this->model.Name(_ecm)
             << "] not touching anything at [" << _info.simTime.count()
             << "]" << std::endl;
    }
    this->touchStart = DurationType::zero();
    return;
  }

  // Start touch timer
  {
    std::lock_guard<std::mutex> lock(this->serviceMutex);
    if (this->touchStart == DurationType::zero())
    {
      this->touchStart =
        std::chrono::duration_cast<DurationType>(_info.simTime);

      gzdbg << "Model [" << this->model.Name(_ecm) << "] started touching ["
        << this->targetName << "] at " << this->touchStart.count() << " s"
        << std::endl;
    }
  }

  // Check if it has been touched for long enough
  auto completed = (std::chrono::duration_cast<DurationType>(_info.simTime) -
      this->touchStart) > this->targetTime;

  // This is a single-use plugin. After touched, publish a message
  // and stop updating
  if (completed)
  {
    gzdbg << "Model [" << this->model.Name(_ecm) << "] touched ["
      << this->targetName << "] exclusively for "
      << this->targetTime.count() << " s" << std::endl;

    {
      std::lock_guard<std::mutex> lock(this->serviceMutex);
      if (this->enabled)
      {
        msgs::Boolean msg;
        msg.set_data(true);
        this->touchedPub->Publish(msg);
      }
    }
    // Disable
    this->Enable(false);
  }
}

//////////////////////////////////////////////////
void TouchPluginPrivate::AddTargetEntities(const EntityComponentManager &_ecm,
                                           const std::vector<Entity> &_entities)
{
  if (_entities.empty())
    return;

  for (Entity entity : _entities)
  {
    // The target name can be a substring of the desired collision name so we
    // have to iterate through all collisions and check if their scoped name has
    // this substring
    std::string name = scopedName(entity, _ecm);
    if (name.find(this->targetName) != std::string::npos)
    {
      this->targetEntities.push_back(entity);
    }
  }

  // Sort so that we can do binary search later on.
  std::sort(this->targetEntities.begin(), this->targetEntities.end());
}

//////////////////////////////////////////////////
void TouchPlugin::Configure(const Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            EntityComponentManager &_ecm, EventManager &)
{
  this->dataPtr->model = Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "Touch plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }
  this->dataPtr->sdfConfig = _sdf->Clone();
}

//////////////////////////////////////////////////
void TouchPlugin::PreUpdate(const UpdateInfo &, EntityComponentManager &_ecm)
{
  GZ_PROFILE("TouchPlugin::PreUpdate");
  if ((!this->dataPtr->initialized) && this->dataPtr->sdfConfig)
  {
    // We call Load here instead of Configure because we can't be guaranteed
    // that all entities have been created when Configure is called
    this->dataPtr->Load(_ecm, this->dataPtr->sdfConfig);
    this->dataPtr->initialized = true;
  }

  // If Load() was successful, validConfig is set to true
  if (this->dataPtr->validConfig)
  {
    // Update target entities when new collisions are added
    std::vector<Entity> potentialEntities;
    _ecm.EachNew<components::Collision>(
        [&](const Entity &_entity, const components::Collision *) -> bool
        {
          potentialEntities.push_back(_entity);
          return true;
        });
    this->dataPtr->AddTargetEntities(_ecm, potentialEntities);
  }
}

//////////////////////////////////////////////////
void TouchPlugin::PostUpdate(const UpdateInfo &_info,
                             const EntityComponentManager &_ecm)
{
  GZ_PROFILE("TouchPlugin::PostUpdate");
  if (this->dataPtr->validConfig)
  {
    this->dataPtr->Update(_info, _ecm);
  }
}

GZ_ADD_PLUGIN(TouchPlugin,
              System,
              TouchPlugin::ISystemConfigure,
              TouchPlugin::ISystemPreUpdate,
              TouchPlugin::ISystemPostUpdate,
              TouchPlugin::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(TouchPlugin, "gz::sim::systems::TouchPlugin")
