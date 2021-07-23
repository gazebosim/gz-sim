/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/ServerConfig.hh"

#include "ignition/gazebo/TestFixture.hh"

class HelperSystem;

class ignition::gazebo::TestFixture::Implementation
{
  /// \brief Initialize fixture
  /// \param[in] _config Server config
  public: void Init(const ServerConfig &_config);

  /// \brief Pointer to underlying server
  public: std::shared_ptr<gazebo::Server> server{nullptr};

  /// \brief Pointer to underlying Helper interface
  public: HelperSystem *helperSystem{nullptr};
};

using namespace ignition;
using namespace gazebo;

/// \brief
class HelperSystem :
  public System,
  public ISystemConfigure,
  public ISystemPreUpdate,
  public ISystemUpdate,
  public ISystemPostUpdate
{
  // Documentation inherited
  public: void Configure(
                const Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                EntityComponentManager &_ecm,
                EventManager &_eventMgr) override;

  // Documentation inherited
  public: void PreUpdate(const UpdateInfo &_info,
                EntityComponentManager &_ecm) override;

  // Documentation inherited
  public: void Update(const UpdateInfo &_info,
                EntityComponentManager &_ecm) override;

  // Documentation inherited
  public: void PostUpdate(const UpdateInfo &_info,
                const EntityComponentManager &_ecm) override;

  /// \brief Function to call every time  we configure a world
  public: std::function<void(const Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                EntityComponentManager &_ecm,
                EventManager &_eventMgr)>
      configureCallback;

  /// \brief Function to call every pre-update
  public: std::function<void(const UpdateInfo &, EntityComponentManager &)>
      preUpdateCallback;

  /// \brief Function to call every update
  public: std::function<void(const UpdateInfo &, EntityComponentManager &)>
      updateCallback;

  /// \brief Function to call every post-update
  public: std::function<void(const UpdateInfo &,
      const EntityComponentManager &)> postUpdateCallback;
};

/////////////////////////////////////////////////
void HelperSystem::Configure(
                const Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                EntityComponentManager &_ecm,
                EventManager &_eventMgr)
{
  if (this->configureCallback)
    this->configureCallback(_entity, _sdf, _ecm, _eventMgr);
}

/////////////////////////////////////////////////
void HelperSystem::PreUpdate(const UpdateInfo &_info,
      EntityComponentManager &_ecm)
{
  if (this->preUpdateCallback)
    this->preUpdateCallback(_info, _ecm);
}

/////////////////////////////////////////////////
void HelperSystem::Update(const UpdateInfo &_info,
      EntityComponentManager &_ecm)
{
  if (this->updateCallback)
    this->updateCallback(_info, _ecm);
}

/////////////////////////////////////////////////
void HelperSystem::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  if (this->postUpdateCallback)
    this->postUpdateCallback(_info, _ecm);
}

//////////////////////////////////////////////////
TestFixture::TestFixture(const std::string &_sdf)
  : dataPtr(utils::MakeUniqueImpl<Implementation>())
{
  ServerConfig config;
  config.SetSdfFile(_sdf);
  this->dataPtr->Init(config);
}

//////////////////////////////////////////////////
TestFixture::TestFixture(const ServerConfig &_config)
  : dataPtr(utils::MakeUniqueImpl<Implementation>())
{
  this->dataPtr->Init(_config);
}

//////////////////////////////////////////////////
void TestFixture::Implementation::Init(const ServerConfig &_config)
{
  this->helperSystem = new HelperSystem();
  this->server = std::make_shared<gazebo::Server>(_config);
}

//////////////////////////////////////////////////
TestFixture &TestFixture::Finalize()
{
  auto systemPtr = dynamic_cast<System *>(this->dataPtr->helperSystem);
  this->dataPtr->server->AddSystem(systemPtr);
  return *this;
}

//////////////////////////////////////////////////
TestFixture &TestFixture::OnConfigure(std::function<void(
          const Entity &_entity,
          const std::shared_ptr<const sdf::Element> &_sdf,
          EntityComponentManager &_ecm,
          EventManager &_eventMgr)> _cb)
{
  if (nullptr != this->dataPtr->helperSystem)
    this->dataPtr->helperSystem->configureCallback = std::move(_cb);
  return *this;
}

//////////////////////////////////////////////////
TestFixture &TestFixture::OnPreUpdate(std::function<void(
          const UpdateInfo &, EntityComponentManager &)> _cb)
{
  if (nullptr != this->dataPtr->helperSystem)
    this->dataPtr->helperSystem->preUpdateCallback = std::move(_cb);
  return *this;
}

//////////////////////////////////////////////////
TestFixture &TestFixture::OnUpdate(std::function<void(
          const UpdateInfo &, EntityComponentManager &)> _cb)
{
  if (nullptr != this->dataPtr->helperSystem)
    this->dataPtr->helperSystem->updateCallback = std::move(_cb);
  return *this;
}

//////////////////////////////////////////////////
TestFixture &TestFixture::OnPostUpdate(std::function<void(
          const UpdateInfo &, const EntityComponentManager &)> _cb)
{
  if (nullptr != this->dataPtr->helperSystem)
    this->dataPtr->helperSystem->postUpdateCallback = std::move(_cb);
  return *this;
}

//////////////////////////////////////////////////
std::shared_ptr<gazebo::Server> TestFixture::Server() const
{
  return this->dataPtr->server;
}

