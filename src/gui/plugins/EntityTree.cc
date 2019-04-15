/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include <iostream>
#include <ignition/common/Console.hh>
#include <ignition/plugin/Register.hh>

#include "ignition/gazebo/gui/plugins/EntityTree.hh"

namespace ignition
{
namespace gazebo::gui
{
  class EntityTreePrivate
  {
  };
}
}

using namespace ignition::gazebo::gui;
using namespace ignition::gui;

/////////////////////////////////////////////////
EntityTree::EntityTree()
  : Plugin(), dataPtr(new EntityTreePrivate)
{
}

/////////////////////////////////////////////////
EntityTree::~EntityTree()
{
}

/////////////////////////////////////////////////
void EntityTree::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "EntityTree";
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::gui::EntityTree,
                    ignition::gui::Plugin)
