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

PYBIND11_MODULE(gazebo, m) {
  m.doc() = "Ignition Gazebo Python Library.";

  ignition::gazebo::python::defineGazeboEntityComponentManager(m);
  ignition::gazebo::python::defineGazeboEventManager(m);
  ignition::gazebo::python::defineGazeboServer(m);
  ignition::gazebo::python::defineGazeboServerConfig(m);
  ignition::gazebo::python::defineGazeboTestFixture(m);
  ignition::gazebo::python::defineGazeboUpdateInfo(m);
  ignition::gazebo::python::defineGazeboWorld(m);
  ignition::gazebo::python::defineGazeboUtil(m);
}
