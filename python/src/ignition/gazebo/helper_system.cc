/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#include <pybind11/pybind11.h>
#include <pybind11/functional.h>

#include "helper_system.hh"

namespace ignition
{
namespace gazebo
{
namespace python
{
/////////////////////////////////////////////////
void HelperSystem::Configure(
                const Entity &_entity,
                const std::shared_ptr<const sdf::Element> &/*_sdf*/,
                ignition::gazebo::EntityComponentManager &_ecm,
                EventManager &/*_eventMgr*/)
{
  if (this->configureCallback_internal)
  {
    auto ecm = ignition::gazebo::python::EntityComponentManager(_ecm);
    this->configureCallback_internal(_entity, ecm);
  }
}

/////////////////////////////////////////////////
void HelperSystem::PreUpdate(const UpdateInfo &_info,
      ignition::gazebo::EntityComponentManager &_ecm)
{
  if (this->preUpdateCallback_internal)
  {
    auto ecm = ignition::gazebo::python::EntityComponentManager(_ecm);
    this->preUpdateCallback_internal(_info, ecm);
  }
}

/////////////////////////////////////////////////
void HelperSystem::Update(const UpdateInfo &_info,
      ignition::gazebo::EntityComponentManager &_ecm)
{
  if (this->updateCallback_internal)
  {
    auto ecm = ignition::gazebo::python::EntityComponentManager(_ecm);
    this->updateCallback_internal(_info, ecm);
  }
}

/////////////////////////////////////////////////
void HelperSystem::PostUpdate(const UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  if (this->postUpdateCallback_internal)
    this->postUpdateCallback_internal(_info,
      ignition::gazebo::python::EntityComponentManager(_ecm));
}

//////////////////////////////////////////////////
class HelperFixturePrivate
{
  /// \brief Initialize fixture
  /// \param[in] _config Server config
  public: void Init(const ServerConfig &_config);

  /// \brief Pointer to underlying server
  public: std::shared_ptr<gazebo::Server> server{nullptr};

  /// \brief Pointer to underlying Helper interface
  public: std::shared_ptr<HelperSystem> helperSystem{nullptr};

  /// \brief Flag to make sure Finalize is only called once
  public: bool finalized{false};
};

//////////////////////////////////////////////////
HelperFixture::HelperFixture(const std::string &_path)
  : dataPtr(std::make_shared<HelperFixturePrivate>())
{
  ServerConfig config;
  config.SetSdfFile(_path);
  this->dataPtr->Init(config);
}

//////////////////////////////////////////////////
HelperFixture::HelperFixture(const ServerConfig &_config)
  : dataPtr(new HelperFixturePrivate())
{
  this->dataPtr->Init(_config);
}

//////////////////////////////////////////////////
HelperFixture::~HelperFixture()
{
  dataPtr = nullptr;
}

void
HelperFixture::Destroy()
{
  dataPtr.reset();
}

//////////////////////////////////////////////////
void HelperFixturePrivate::Init(const ServerConfig &_config)
{
  this->helperSystem = std::make_shared<HelperSystem>();
  this->server = std::make_shared<gazebo::Server>(_config);
}

//////////////////////////////////////////////////
HelperFixture &HelperFixture::Finalize()
{
  if (this->dataPtr->finalized)
  {
    ignwarn << "Fixture has already been finalized, this only needs to be done"
            << " once." << std::endl;
    return *this;
  }

  this->dataPtr->server->AddSystem(this->dataPtr->helperSystem);

  this->dataPtr->finalized = true;
  return *this;
}

//////////////////////////////////////////////////
HelperFixture &HelperFixture::OnConfigure(std::function<void(
          const Entity &_entity,
          ignition::gazebo::python::EntityComponentManager &_ecm)> _cb)
{
  if (nullptr != this->dataPtr->helperSystem)
    this->dataPtr->helperSystem->configureCallback_internal = std::move(_cb);
  return *this;
}

//////////////////////////////////////////////////
HelperFixture &HelperFixture::OnPreUpdate(std::function<void(
  const UpdateInfo &, ignition::gazebo::python::EntityComponentManager &)> _cb)
{
  if (nullptr != this->dataPtr->helperSystem)
    this->dataPtr->helperSystem->preUpdateCallback_internal = std::move(_cb);
  return *this;
}

//////////////////////////////////////////////////
HelperFixture &HelperFixture::OnUpdate(std::function<void(
  const UpdateInfo &, ignition::gazebo::python::EntityComponentManager &)> _cb)
{
  if (nullptr != this->dataPtr->helperSystem)
    this->dataPtr->helperSystem->updateCallback_internal = std::move(_cb);
  return *this;
}

//////////////////////////////////////////////////
HelperFixture &HelperFixture::OnPostUpdate(std::function<void(
  const UpdateInfo &,
  const ignition::gazebo::python::EntityComponentManager &)> _cb)
{
  if (nullptr != this->dataPtr->helperSystem)
    this->dataPtr->helperSystem->postUpdateCallback_internal = std::move(_cb);
  return *this;
}

//////////////////////////////////////////////////
std::shared_ptr<gazebo::Server> HelperFixture::Server() const
{
  return this->dataPtr->server;
}

void
define_gazebo_helper_fixture(pybind11::object module)
{
  pybind11::class_<HelperFixture,
             ignition::gazebo::python::Destroyable,
             std::shared_ptr<HelperFixture>>(module, "HelperFixture")
  .def(pybind11::init<const std::string &>())
  .def(
    "server", &HelperFixture::Server,
    "Get pointer to underlying server."
  )
  .def(
    "finalize", &HelperFixture::Finalize,
    "Finalize all the functions and add fixture to server."
  )
  .def(
    "on_pre_update", &HelperFixture::OnPreUpdate,
    "Wrapper around a system's pre-update callback"
  )
  .def(
    "on_update", &HelperFixture::OnUpdate,
    "Wrapper around a system's update callback"
  )
  .def(
    "on_post_update", &HelperFixture::OnPostUpdate,
    "Wrapper around a system's post-update callback"
  )
  .def(
    "on_configure", &HelperFixture::OnConfigure,
    "Wrapper around a system's pre-update callback"
  );
}
}
}
}
