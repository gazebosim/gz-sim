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

#include "Breadcrumbs.hh"

#include <gz/msgs/empty.pb.h>

#include <algorithm>
#include <iterator>
#include <string>
#include <utility>

#include <gz/common/Profiler.hh>

#include <gz/math/Quaternion.hh>
#include <gz/plugin/Register.hh>

#include <sdf/parser.hh>
#include <sdf/Geometry.hh>

#include "gz/sim/components/CanonicalLink.hh"
#include "gz/sim/components/DetachableJoint.hh"
#include "gz/sim/components/Geometry.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Performer.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

//////////////////////////////////////////////////
void Breadcrumbs::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
{
  this->maxDeployments =
      _sdf->Get<int>("max_deployments", this->maxDeployments).first;

  // Exit early if breadcrumbs deployments are set to zero.
  if (this->maxDeployments == 0)
  {
    gzmsg << "Breadcrumbs max deployment is == 0. Breadcrumbs are disabled."
      << std::endl;
    return;
  }

  double period =
      _sdf->Get<double>("disable_physics_time", 0.0).first;
  this->disablePhysicsTime =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(period));

  this->allowRenaming =
      _sdf->Get<bool>("allow_renaming", this->allowRenaming).first;

  this->model = Model(_entity);
  if (!this->model.Valid(_ecm))
  {
    gzerr << "The Breadcrumbs system should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  if (!_sdf->HasElement("breadcrumb"))
  {
    gzerr << "<breadcrumb> not set" << std::endl;
    return;
  }

  auto breadcrumb = _sdf->GetElementImpl("breadcrumb");

  if (!breadcrumb->HasElement("sdf"))
  {
    gzerr << "<sdf> not found in <breadcrumb>" << std::endl;
    return;
  }

  auto sdfElem = breadcrumb->GetElementImpl("sdf");
  // We can't load the model directly because it won't go through the SDF
  // validation process, so we first convert to text and call
  // Root::LoadSdfString
  sdf::Errors errors = this->modelRoot.LoadSdfString(sdfElem->ToString(""));
  if (!errors.empty())
  {
    for (const auto &e : errors)
    {
      gzerr << e.Message() << std::endl;
    }
    return;
  }
  if (nullptr == this->modelRoot.Model())
  {
    gzerr << "Model not found in <breadcrumb>" << std::endl;
    return;
  }

  if (_sdf->HasElement("performer_volume"))
  {
    auto vol = _sdf->GetElementImpl("performer_volume");
    sdf::Geometry geom;
    geom.Load(vol->GetElementImpl("geometry"));
    if (nullptr != geom.BoxShape())
    {
      this->performerGeometry = std::move(geom);
      this->isPerformer = true;
    }
    else
    {
      gzerr << "Geometry specified in <performer_volume> is invalid\n";
      return;
    }
  }

  // Subscribe to commands
  std::vector<std::string> topics;
  if (_sdf->HasElement("topic"))
  {
    topics.push_back(_sdf->Get<std::string>("topic"));
  }
  topics.push_back("/model/" +
      this->model.Name(_ecm) + "/breadcrumbs/" +
      this->modelRoot.Model()->Name() + "/deploy");
  this->topic = validTopic(topics);

  this->topicStatistics = _sdf->Get<bool>("topic_statistics",
      this->topicStatistics).first;

  // Enable topic statistics when requested.
  if (this->topicStatistics)
  {
    if (!node.EnableStats(this->topic, true))
    {
      gzerr << "Unable to enable topic statistics on topic["
        << this->topic << "]." << std::endl;
      this->topicStatistics = false;
    }
  }

  this->node.Subscribe(this->topic, &Breadcrumbs::OnDeploy, this);
  this->remainingPub = this->node.Advertise<msgs::Int32>(
      this->topic + "/remaining");

  gzmsg << "Breadcrumbs subscribing to deploy messages on ["
    << this->topic << "]" << std::endl;

  this->creator = std::make_unique<SdfEntityCreator>(_ecm, _eventMgr);

  this->worldEntity = _ecm.EntityByComponents(components::World());

  this->initialized = true;
}

//////////////////////////////////////////////////
void Breadcrumbs::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("Breadcrumbs::PreUpdate");

  if (this->initialized)
  {
    std::vector<bool> cmds;
    {
      std::lock_guard<std::mutex> lock(this->pendingCmdsMutex);
      std::copy(this->pendingCmds.begin(), this->pendingCmds.end(),
                std::back_inserter(cmds));
      this->pendingCmds.clear();
    }
    // Check that the model is valid before continuing. This check is needed
    // because the model associated with the Breadcrumbs might have been
    // unloaded by the level manager. Ideally, this system would have been
    // unloaded along with the model, but that is not currently the case. See
    // issue #113
    if (!this->model.Valid(_ecm))
    {
      return;
    }

    auto poseComp = _ecm.Component<components::Pose>(this->model.Entity());

    for (std::size_t i = 0; i < cmds.size(); ++i)
    {
      if (this->maxDeployments < 0 ||
          this->numDeployments < this->maxDeployments)
      {
        sdf::Model modelToSpawn = *this->modelRoot.Model();
        std::string desiredName =
            modelToSpawn.Name() + "_" + std::to_string(this->numDeployments);

        std::vector<std::string> modelNames;
        _ecm.Each<components::Name, components::Model>(
            [&modelNames](const Entity &, const components::Name *_name,
                          const components::Model *)
            {
              modelNames.push_back(_name->Data());
              return true;
            });

        // Check if there's a model with the same name.
        auto it = std::find(modelNames.begin(), modelNames.end(), desiredName);
        if (it != modelNames.end())
        {
          if (!this->allowRenaming)
          {
            gzwarn << "Entity named [" << desiredName
                    << "] already exists and "
                    << "[allow_renaming] is false. Entity not spawned."
                    << std::endl;
            return;
          }

          std::string newName = desiredName;
          int counter = 0;
          while (std::find(modelNames.begin(), modelNames.end(), newName) !=
                 modelNames.end())
          {
            newName = desiredName + "_" + std::to_string(++counter);
          }
          desiredName = newName;
        }

        modelToSpawn.SetName(desiredName);
        modelToSpawn.SetRawPose(poseComp->Data() * modelToSpawn.RawPose());
        gzmsg << "Deploying " << modelToSpawn.Name() << " at "
               << modelToSpawn.RawPose() << std::endl;
        Entity entity = this->creator->CreateEntities(&modelToSpawn);
        this->creator->SetParent(entity, this->worldEntity);

        // keep track of entities that are set to auto disable
        if (!modelToSpawn.Static() &&
            this->disablePhysicsTime >
            std::chrono::steady_clock::duration::zero())
        {
          this->autoStaticEntities[entity] = _info.simTime;
        }

        if (this->isPerformer)
        {
          auto worldName =
              _ecm.Component<components::Name>(this->worldEntity)->Data();
          msgs::StringMsg req;
          req.set_data(modelToSpawn.Name());
          this->node.Request<msgs::StringMsg, msgs::Boolean>(
              "/world/" + worldName + "/level/set_performer", req,
              [](const msgs::Boolean &, const bool)
              {
              });

          // When using the set_performer service, the performer gets a default
          // geometry for its bounding volume. To update the geometry, we make a
          // list of performer breadcrumbs and process them as we detect that
          // they have become performers
          this->pendingGeometryUpdate.insert(entity);
        }

        ++this->numDeployments;
      }
      else
      {
        gzmsg << "Asked to deploy " << this->modelRoot.Model()->Name()
               << " but the maximum number of deployments has reached the "
               << "limit of " << this->maxDeployments << std::endl;
      }

      // Publish remaining deployments
      msgs::Int32 remainingMsg;
      remainingMsg.set_data(this->maxDeployments - this->numDeployments);
      this->remainingPub.Publish(remainingMsg);
    }

    std::set<Entity> processedEntities;
    for (const auto &e : this->pendingGeometryUpdate)
    {
      Entity perf = _ecm.EntityByComponents(components::Performer(),
                                            components::ParentEntity(e));
      if (perf == kNullEntity)
      {
        continue;
      }

      auto geomComp = _ecm.Component<components::Geometry>(perf);
      if (geomComp)
      {
        geomComp->SetData(
            *this->performerGeometry,
            [](const sdf::Geometry &, const sdf::Geometry &) -> bool
            {
              // We'll assume that the data is changed.
              return true;
            });
        _ecm.SetChanged(e, geomComp->TypeId());

        processedEntities.insert(e);
      }
    }

    // Remove processed entities from the pending list
    for (const auto &e : processedEntities)
    {
      this->pendingGeometryUpdate.erase(e);
    }

    // make entities static when auto disable period is reached.
    for (auto it = this->autoStaticEntities.begin();
        it != this->autoStaticEntities.end();)
    {
      auto td = _info.simTime - it->second;
      if (td > this->disablePhysicsTime)
      {
        auto name = _ecm.Component<components::Name>(it->first)->Data();
        if (!this->MakeStatic(it->first, _ecm))
        {
          gzerr << "Failed to make breadcrumb '" << name
                 << "' static." << std::endl;
        }
        else
        {
          gzmsg << "Breadcrumb '" << name << "' is now static." << std::endl;
        }
        this->autoStaticEntities.erase(it++);
      }
      else
      {
        ++it;
      }
    }
  }
}

//////////////////////////////////////////////////
bool Breadcrumbs::MakeStatic(Entity _entity, EntityComponentManager &_ecm)
{
  // make breadcrumb static by spawning a static model and attaching the
  // breadcrumb to the static model
  // todo(anyone) Add a feature in gz-physics to support making a model
  // static
  if (this->staticModelToSpawn.LinkCount() == 0u)
  {
    sdf::ElementPtr staticModelSDF(new sdf::Element);
    sdf::initFile("model.sdf", staticModelSDF);
    staticModelSDF->GetAttribute("name")->Set("static_model");
    staticModelSDF->GetElement("static")->Set(true);
    sdf::ElementPtr linkElem = staticModelSDF->AddElement("link");
    linkElem->GetAttribute("name")->Set("static_link");
    this->staticModelToSpawn.Load(staticModelSDF);
  }

  auto bcPoseComp = _ecm.Component<components::Pose>(_entity);
  if (!bcPoseComp)
    return false;
  math::Pose3d p = bcPoseComp->Data();
  this->staticModelToSpawn.SetRawPose(p);

  auto nameComp = _ecm.Component<components::Name>(_entity);
  this->staticModelToSpawn.SetName(nameComp->Data() + "__static__");

  Entity staticEntity = this->creator->CreateEntities(&staticModelToSpawn);
  this->creator->SetParent(staticEntity, this->worldEntity);

  Entity parentLinkEntity = _ecm.EntityByComponents(
      components::Link(), components::ParentEntity(staticEntity),
      components::Name("static_link"));

  if (parentLinkEntity == kNullEntity)
    return false;

  Entity childLinkEntity = _ecm.EntityByComponents(
      components::CanonicalLink(), components::ParentEntity(_entity));

  if (childLinkEntity == kNullEntity)
    return false;

  Entity detachableJointEntity = _ecm.CreateEntity();
  _ecm.CreateComponent(detachableJointEntity,
      components::DetachableJoint(
      {parentLinkEntity, childLinkEntity, "fixed"}));

  return true;
}


//////////////////////////////////////////////////
void Breadcrumbs::OnDeploy(const msgs::Empty &)
{
  GZ_PROFILE("Breadcrumbs::PreUpdate");
  {
    std::lock_guard<std::mutex> lock(this->pendingCmdsMutex);

    this->pendingCmds.push_back(true);
  }

  // Check topic statistics for dropped messages
  if (this->topicStatistics)
  {
    gzmsg << "Received breadcrumb deployment for " <<
      this->modelRoot.Model()->Name() << std::endl;
    std::optional<transport::TopicStatistics> stats =
      this->node.TopicStats(this->topic);
    if (stats)
    {
      if (stats->DroppedMsgCount() > 0)
      {
        gzwarn << "Dropped message count of " << stats->DroppedMsgCount()
          << " for breadcrumbs on model "
          << this->modelRoot.Model()->Name() << std::endl;
      }
    }
    else
    {
      gzerr << "Unable to get topic statistics for topic["
        << this->topic << "]." << std::endl;
    }
  }
}

GZ_ADD_PLUGIN(Breadcrumbs,
                    System,
                    Breadcrumbs::ISystemConfigure,
                    Breadcrumbs::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(Breadcrumbs, "gz::sim::systems::Breadcrumbs")
