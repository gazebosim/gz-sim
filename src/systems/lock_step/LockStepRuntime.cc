/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#include <string>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/sim/Conversions.hh>
#include <gz/sim/SystemLoader.hh>
#include <gz/sim/Util.hh>
#include <sdf/sdf.hh>

#include "LockStepRuntime.hh"
#include "RuntimeConfig.hh"

using namespace gz;
using namespace sim;

//////////////////////////////////////////////////
std::unique_ptr<LockStepRuntime> LockStepRuntime::Create(
  RuntimeConfig _runtime_config)
{
  SystemLoader loader;
  std::optional<SystemPluginPtr> systemPlugin = loader.LoadPlugin(
                  _runtime_config.plugin);
  if (!systemPlugin.has_value() || !*systemPlugin)
  {
    gzerr << "Failed to instantiate plugin.\n";
    return nullptr;
  }
  gzmsg << "Loaded plugin [" << _runtime_config.plugin.Filename() << "]\n";

  auto *runtimePtr = new LockStepRuntime(std::move(*systemPlugin),
                                      std::move(_runtime_config));
  auto runtime = std::unique_ptr<LockStepRuntime>(runtimePtr);
  if (!runtime->InitServices())
  {
    return nullptr;
  }
  return runtime;
}

//////////////////////////////////////////////////
LockStepRuntime::LockStepRuntime(SystemPluginPtr _systemPlugin,
                                 RuntimeConfig _config)
  : systemPlugin(std::move(_systemPlugin)),
    systemConfigure(systemPlugin->QueryInterface<ISystemConfigure>()),
    systemPreupdate(systemPlugin->QueryInterface<ISystemPreUpdate>()),
    systemUpdate(systemPlugin->QueryInterface<ISystemUpdate>()),
    systemPostupdate(systemPlugin->QueryInterface<ISystemPostUpdate>()),
    config(_config)
{}

//////////////////////////////////////////////////
LockStepRuntime::~LockStepRuntime()
{
  this->node.UnadvertiseSrv(this->config.configureService);
  this->node.UnadvertiseSrv(this->config.preupdateService);
  this->node.UnadvertiseSrv(this->config.updateService);
  this->node.UnadvertiseSrv(this->config.postupdateService);
}

//////////////////////////////////////////////////
bool LockStepRuntime::PublishStats() const
{
  // TODO: publish stats
  return true;
}

//////////////////////////////////////////////////
bool LockStepRuntime::InitServices()
{
  if (this->systemConfigure != nullptr)
  {
    bool success = this->node.Advertise(this->config.configureService,
        &LockStepRuntime::OnConfigure, this);
    if (!success)
    {
      gzerr << "Failed to advertise Configure service on ["
            << this->config.configureService << "]\n";
      return false;
    }
    gzmsg << "Advertised Configure service on ["
            << this->config.configureService << "]\n";
  }

  if (this->systemPreupdate != nullptr)
  {
    bool success = this->node.Advertise(this->config.preupdateService,
        &LockStepRuntime::OnPreupdate, this);
    if (!success)
    {
      gzerr << "Failed to advertise Preupdate service on ["
            << this->config.preupdateService << "]\n";
      return false;
    }
    gzmsg << "Advertised Configure service on ["
            << this->config.preupdateService << "]\n";
  }

  if (this->systemUpdate != nullptr)
  {
    bool success = this->node.Advertise(this->config.updateService,
        &LockStepRuntime::OnUpdate, this);
    if (!success)
    {
      gzerr << "Failed to advertise Update service on ["
            << this->config.updateService << "]\n";
      return false;
    }
    gzmsg << "Advertised Update service on ["
            << this->config.updateService << "]\n";
  }

  if (this->systemPostupdate != nullptr)
  {
    bool success = this->node.Advertise(this->config.postupdateService,
        &LockStepRuntime::OnPostUpdate, this);
    if (!success)
    {
      gzerr << "Failed to advertise PostUpdate service on ["
            << this->config.postupdateService << "]\n";
      return false;
    }
    gzmsg << "Advertised PostUpdate service on ["
            << this->config.postupdateService << "]\n";
  }

  return true;
}

//////////////////////////////////////////////////
bool LockStepRuntime::OnConfigure(const gz::msgs::SerializedStepMap& _req,
                  gz::msgs::Boolean& _rep)
{
  // TODO check/set state to enforce Configure call only once.
  this->ecm.SetState(_req.state());
  GZ_ASSERT(this->systemConfigure != nullptr,
            "OnConfigure should only be called for systems with Configure.");
  // TODO: parent entity should be populated in request.
  Entity worldEnt = sim::worldEntity(this->ecm);
  this->systemConfigure->Configure(
                  worldEnt,
                  this->config.plugin.ToElement(),
                  this->ecm,
                  this->eventManager);
  // TODO: reply should be SerializedStateMap to relay updates from the system
  // to the server.
  _rep.set_data(true);
  return true;
}

//////////////////////////////////////////////////
bool LockStepRuntime::OnPreupdate(const gz::msgs::SerializedStepMap& _req,
                  gz::msgs::Boolean& _rep)
{
  // TODO: implement
  _rep.set_data(true);
  return true;
}

//////////////////////////////////////////////////
bool LockStepRuntime::OnUpdate(const gz::msgs::SerializedStepMap& _req,
                  gz::msgs::Boolean& _rep)
{
  this->ecm.SetState(_req.state());
  auto info = sim::convert<UpdateInfo>(_req.stats());
  this->systemUpdate->Update(info, this->ecm);
  _rep.set_data(true);
  return true;
}

//////////////////////////////////////////////////
bool LockStepRuntime::OnPostUpdate(const gz::msgs::SerializedStepMap& _req,
                  gz::msgs::Boolean& _rep)
{
  this->ecm.SetState(_req.state());
  auto info = sim::convert<UpdateInfo>(_req.stats());
  this->systemPostupdate->PostUpdate(info, this->ecm);
  _rep.set_data(true);

  // Perform end-of step ECM operations
  // TODO: move to a state machine to force a sequence to the calls.
  this->ecm.ClearNewlyCreatedEntities();
  this->ecm.ProcessRemoveEntityRequests();
  this->ecm.ClearRemovedComponents();
  this->ecm.SetAllComponentsUnchanged();

  return true;
}
