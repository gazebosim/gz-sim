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
#include "Label.hh"
#include <string>

#include <ignition/plugin/Register.hh>
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/components/SemanticLabel.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Visual.hh"

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

  if (!_sdf->HasElement(labelTag))
  {
    ignerr << "Failed to load Label system; label tag not found.\n";
    return;
  }

  auto label = _sdf->Get<int>(labelTag);

  if (label < 0 || label > 255)
  {
    ignerr << "Failed to configure Label system; value " << label
      << " is not in [0-255] range.\n";
    return;
  }

  const std::string parentName = _sdf->GetParent()->GetName();

  // Set the component to the visual to set its user data, but
  // if the plugin is inside the <model> tag, get its visual child
  if (parentName == "visual")
  {
    _ecm.CreateComponent(_entity, components::SemanticLabel(label));
  }
  else if (parentName == "model")
  {
    // Get link childern of parent model
    auto links = _ecm.ChildrenByComponents<components::Link>(
      _entity, components::Link());

    for (auto linkEntity : links)
    {
      // get visual child of parent link
      auto visuals = _ecm.ChildrenByComponents<components::Visual>(
        linkEntity, components::Visual());

      // Create label component to all visual childern
      for (auto visualEntity : visuals)
      {
        _ecm.CreateComponent(visualEntity,
          components::SemanticLabel(label));
      }
    }
  }
  else
  {
    ignerr << "Label plugin is not a child of model or visual " <<
      "label will be ignored. \n";
    return;
  }
}

IGNITION_ADD_PLUGIN(Label, System, Label::ISystemConfigure)
IGNITION_ADD_PLUGIN_ALIAS(Label, "ignition::gazebo::systems::Label")
