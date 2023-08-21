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
 */

#include <pybind11/pybind11.h>

#include "EntityComponentManager.hh"
#include "EventManager.hh"
#include "Server.hh"
#include "ServerConfig.hh"
#include "TestFixture.hh"
#include "UpdateInfo.hh"
#include "Util.hh"
#include "World.hh"

PYBIND11_MODULE(BINDINGS_MODULE_NAME, m) {
  m.doc() = "Gazebo Sim Python Library.";

  gz::sim::python::defineSimEntityComponentManager(m);
  gz::sim::python::defineSimEventManager(m);
  gz::sim::python::defineSimServer(m);
  gz::sim::python::defineSimServerConfig(m);
  gz::sim::python::defineSimTestFixture(m);
  gz::sim::python::defineSimUpdateInfo(m);
  gz::sim::python::defineSimWorld(m);
  gz::sim::python::defineSimUtil(m);
}
