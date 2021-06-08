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
#include <limits>
#include <string>

#include <ignition/plugin/Register.hh>
#include "ignition/gazebo/components/Label.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "Label.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

//////////////////////////////////////////////////
Label::Label() : System()
{
}

//////////////////////////////////////////////////
Label::~Label() = default;

//////////////////////////////////////////////////
void Label::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gazebo::EntityComponentManager &_ecm,
    gazebo::EventManager & /*_eventMgr*/)
{
  const std::string labelTag = "label";

  std::string parentName = _sdf->GetParent()->GetName();

  if (!_sdf->HasElement(labelTag))
  {
    ignerr << "Failed to load Label system, label tag not found";
    return;
  }

  int label = _sdf->Get<int>(labelTag);

  if (label < 0 || label > 255)
  {
    ignerr << "Failed to Configure Label system, value " << label
      << " is not in [0-255] range";
    return;
  }

  /// Set the component to the visual to set its user data, but
  /// if the plugin is inside the <model> tag, get its visual child
  if (parentName == "visual")
  {
    _ecm.CreateComponent(_entity, components::Label(label));
  }
  else if (parentName == "model")
  {
    // get link child of parent model
    auto links = _ecm.ChildrenByComponents<components::Link>(
      _entity, components::Link());
    if (links.size() > 0)
    {
      Entity linkEntity = links[0];

      // get visual child of parent link
      auto visuals = _ecm.ChildrenByComponents<components::Visual>(
        linkEntity, components::Visual());
      if (visuals.size() > 0)
      {
        Entity visualEntity = visuals[0];
        _ecm.CreateComponent(visualEntity, components::Label(label));
      }
    }
  }
}

IGNITION_ADD_PLUGIN(Label, System, Label::ISystemConfigure)
IGNITION_ADD_PLUGIN_ALIAS(Label, "ignition::gazebo::systems::Label")
