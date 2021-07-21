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

  /// \brief Pointer to underlying system
  public: SystemPluginPtr systemPluginPtr;

  /// \brief Pointer to underlying system
  public: System *systemPtr{nullptr};

  /// \brief Pointer to underlying Helper interface
  //public: std::unique_ptr<HelperSystem> helperSystem{nullptr};
  public: HelperSystem *helperSystem{nullptr};
};

using namespace ignition;
using namespace gazebo;

/// \brief
class HelperSystem :
  public System,
  public ISystemPreUpdate,
  public ISystemUpdate,
  public ISystemPostUpdate
{
  // Documentation inherited
  public: void PreUpdate(const UpdateInfo &_info,
                EntityComponentManager &_ecm) override;

  // Documentation inherited
  public: void Update(const UpdateInfo &_info,
                EntityComponentManager &_ecm) override;

  // Documentation inherited
  public: void PostUpdate(const UpdateInfo &_info,
              const EntityComponentManager &_ecm) override;

  public: size_t preUpdateCallCount {0};
  public: size_t updateCallCount {0};
  public: size_t postUpdateCallCount {0};

  public: std::function<void(const UpdateInfo &, EntityComponentManager &)>
      preUpdateCallback;
  public: std::function<void(const UpdateInfo &, EntityComponentManager &)>
      updateCallback;
  public: std::function<void(const UpdateInfo &, const EntityComponentManager &)>
      postUpdateCallback;
};

/////////////////////////////////////////////////
void HelperSystem::PreUpdate(const UpdateInfo &_info,
      EntityComponentManager &_ecm)
{
  ++this->preUpdateCallCount;
  if (this->preUpdateCallback)
    this->preUpdateCallback(_info, _ecm);
}

/////////////////////////////////////////////////
void HelperSystem::Update(const UpdateInfo &_info,
      EntityComponentManager &_ecm)
{
  ++this->updateCallCount;
  if (this->updateCallback)
    this->updateCallback(_info, _ecm);
}

/////////////////////////////////////////////////
void HelperSystem::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  ++this->postUpdateCallCount;
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
  this->systemPtr = dynamic_cast<System *>(this->helperSystem);

  this->server = std::make_shared<gazebo::Server>(_config);
  this->server->AddSystem(this->systemPtr);
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

