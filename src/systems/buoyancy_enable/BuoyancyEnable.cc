/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <gz/common/Profiler.hh>

#include <gz/plugin/Register.hh>

#include <gz/transport/Node.hh>
#include <gz/transport/TopicUtils.hh>

#include <sdf/sdf.hh>

#include "gz/sim/components/Name.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

#include "BuoyancyEnable.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::BuoyancyEnablePrivate
{
  /// \brief Resolve the model's scoped name and the world's name, and from
  /// them the names to register and the service to send them to. Deliberately
  /// not done in Configure: a model spawned into a running world is configured
  /// before its parent chain is in the ECM, so the world cannot be found yet
  /// and the scoped name is not trustworthy either. By the first PreUpdate it
  /// is all there.
  /// \param[in] _ecm Entity component manager.
  /// \return True once resolved; false to try again next iteration.
  public: bool Resolve(const EntityComponentManager &_ecm);

  /// \brief The model this plugin is attached to.
  public: Entity model{kNullEntity};

  /// \brief Link names from SDF. Empty means the model as a whole.
  public: std::vector<std::string> linkNames;

  /// \brief Scoped names still waiting to be accepted by the world's Buoyancy
  /// system, in the form its <enable> list takes. Emptied as replies arrive.
  public: std::vector<std::string> pending;

  /// \brief Service to send them to. Empty until Resolve succeeds.
  public: std::string enableService;

  /// \brief Whether Resolve has succeeded.
  public: bool resolved{false};

  /// \brief Whether this plugin has given up: attached to something that is
  /// not a model, which no amount of retrying fixes.
  public: bool disabled{false};

  /// \brief Transport node.
  public: transport::Node node;

  /// \brief Whether the first failed attempt has been reported, so that a
  /// server that takes a few iterations to advertise does not fill the log.
  public: bool warned{false};
};

//////////////////////////////////////////////////
bool BuoyancyEnablePrivate::Resolve(const EntityComponentManager &_ecm)
{
  // The name the world's Buoyancy system compares against is the scoped name
  // minus the world, which is the name the model was spawned under rather than
  // the one its file carries.
  const std::string modelName = removeParentScope(
      scopedName(this->model, _ecm, "::", false), "::");
  if (modelName.empty())
    return false;

  // By world entity rather than by walking up from the model: the walk needs a
  // parent chain that a just-spawned model does not have yet.
  const Entity world = worldEntity(_ecm);
  const auto *worldNameComp = _ecm.Component<components::Name>(world);
  if (!worldNameComp)
    return false;

  const std::string service = transport::TopicUtils::AsValidTopic(
      "/world/" + worldNameComp->Data() + "/buoyancy/enable");
  if (service.empty())
  {
    gzerr << "Cannot build a valid service name from world name ["
      << worldNameComp->Data() << "]; [" << modelName << "] cannot register "
      << "for buoyancy and will not float." << std::endl;
    this->disabled = true;
    return false;
  }

  for (const auto &linkName : this->linkNames)
    this->pending.push_back(modelName + "::" + linkName);

  // No <link> at all means the model as a whole, which the world's parent walk
  // resolves to every link under it.
  if (this->pending.empty())
    this->pending.push_back(modelName);

  this->enableService = service;
  this->resolved = true;
  return true;
}

//////////////////////////////////////////////////
BuoyancyEnable::BuoyancyEnable()
  : dataPtr(std::make_unique<BuoyancyEnablePrivate>())
{
}

//////////////////////////////////////////////////
BuoyancyEnable::~BuoyancyEnable() = default;

//////////////////////////////////////////////////
void BuoyancyEnable::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  Model model(_entity);
  if (!model.Valid(_ecm))
  {
    gzerr << "BuoyancyEnable should be attached to a <model>, not a <world>."
      << std::endl;
    this->dataPtr->disabled = true;
    return;
  }

  this->dataPtr->model = _entity;

  // Only the SDF is read here. Everything that needs the ECM waits for
  // PreUpdate; see BuoyancyEnablePrivate::Resolve.
  if (_sdf->HasElement("link"))
  {
    for (auto linkElem = _sdf->FindElement("link");
        linkElem != nullptr;
        linkElem = linkElem->GetNextElement("link"))
    {
      const auto linkName = linkElem->Get<std::string>();
      if (linkName.empty())
      {
        gzwarn << "Ignoring empty <link> in BuoyancyEnable on ["
          << model.Name(_ecm) << "]." << std::endl;
        continue;
      }
      this->dataPtr->linkNames.push_back(linkName);
    }
  }
}

//////////////////////////////////////////////////
void BuoyancyEnable::PreUpdate(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("BuoyancyEnable::PreUpdate");

  if (this->dataPtr->disabled)
    return;

  if (!this->dataPtr->resolved && !this->dataPtr->Resolve(_ecm))
    return;

  // Nothing left to register: the common case, from the second or third
  // iteration onwards.
  if (this->dataPtr->pending.empty())
    return;

  // Retried rather than done once in Configure because the world's Buoyancy
  // system may not have advertised yet when this model is spawned into a
  // running server. A short timeout keeps a missing service from stalling the
  // loop; the request is idempotent, so a reply lost to the timeout costs
  // nothing but another attempt.
  static constexpr auto kTimeout = std::chrono::milliseconds(50);

  std::vector<std::string> stillPending;
  for (const auto &name : this->dataPtr->pending)
  {
    msgs::StringMsg req;
    req.set_data(name);

    msgs::Boolean rep;
    bool result{false};
    const bool executed = this->dataPtr->node.Request(
        this->dataPtr->enableService, req,
        static_cast<unsigned int>(kTimeout.count()), rep, result);

    if (executed && result && rep.data())
    {
      gzmsg << "Registered [" << name << "] for buoyancy." << std::endl;
      continue;
    }

    stillPending.push_back(name);
  }

  if (!stillPending.empty() && !this->dataPtr->warned)
  {
    this->dataPtr->warned = true;
    gzwarn << "Buoyancy enable service [" << this->dataPtr->enableService
      << "] did not answer yet; retrying every iteration. If this never "
      << "succeeds, the world is not running the Buoyancy system."
      << std::endl;
  }

  this->dataPtr->pending = std::move(stillPending);
}

GZ_ADD_PLUGIN(BuoyancyEnable,
                    System,
                    BuoyancyEnable::ISystemConfigure,
                    BuoyancyEnable::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(BuoyancyEnable,
                          "gz::sim::systems::BuoyancyEnable")
