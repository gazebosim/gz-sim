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

#include <ignition/common/Console.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/math/Color.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/rendering.hh>

#include "ignition/gazebo/gui/GuiEvents.hh"
#include "OriginAxesConfig.hh"

namespace ignition::gazebo
{
  class OriginAxesConfigPrivate
  {
    /// \brief origin axes ptr in a scene (defined by the name worldOriginAxes)
    public: rendering::VisualPtr originAxes = nullptr;

    /// \brief Flag that indicates whether there are new updates to be rendered.
    public: bool dirty{false};

    /// \brief Default visible state
    bool visible{true};
  };
}

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
OriginAxesConfig::OriginAxesConfig()
  : ignition::gui::Plugin(), dataPtr(std::make_unique<OriginAxesConfigPrivate>())
{
}

/////////////////////////////////////////////////
OriginAxesConfig::~OriginAxesConfig() = default;

/////////////////////////////////////////////////
void OriginAxesConfig::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Origin Axis";

  ignition::gui::App()->findChild<
      ignition::gui::MainWindow *>()->installEventFilter(this);
}

/////////////////////////////////////////////////
bool OriginAxesConfig::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == ignition::gazebo::gui::events::Render::Type)
  {
    // This event is called in Scene3d's RenderThread, so it's safe to make
    // rendering calls here
    this->UpdateOriginArrows();
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

/////////////////////////////////////////////////
void OriginAxesConfig::UpdateOriginArrows()
{
  // Load axes if they don't already exist
  if (!this->dataPtr->originAxes)
    this->LoadOriginAxes();

  // If axes were not loaded successfully, don't update
  if (!this->dataPtr->originAxes)
    return;

  if (!this->dataPtr->dirty)
    return;

  this->dataPtr->dirty = false;

  this->dataPtr->originAxes->SetVisible(this->dataPtr->visible);
}

/////////////////////////////////////////////////
void OriginAxesConfig::LoadOriginAxes()
{
  auto loadedEngNames = rendering::loadedEngines();
  if (loadedEngNames.empty())
    return;

  // assume there is only one engine loaded
  auto engineName = loadedEngNames[0];
  if (loadedEngNames.size() > 1)
  {
    igndbg << "More than one engine is available. "
      << "Origin axes config plugin will use engine ["
        << engineName << "]" << std::endl;
  }
  auto engine = rendering::engine(engineName);
  if (!engine)
  {
    ignerr << "Internal error: failed to load engine [" << engineName
      << "]. Origin axes plugin won't work." << std::endl;
    return;
  }

  if (engine->SceneCount() == 0)
    return;

  // assume there is only one scene
  // load scene
  auto scene = engine->SceneByIndex(0);
  if (!scene)
  {
    ignerr << "Internal error: scene is null." << std::endl;
    return;
  }

  if (!scene->IsInitialized() || scene->VisualCount() == 0)
  {
    return;
  }

  this->dataPtr->originAxes = scene->VisualByName("worldOriginAxes");

  if (this->dataPtr->originAxes)
    return;

  auto root = scene->RootVisual();
  this->dataPtr->originAxes = scene->CreateAxisVisual("worldOriginAxes");
  if (!this->dataPtr->originAxes)
  {
    ignerr << "Failed to create origin axes, origin axes config plugin won't work."
            << std::endl;
    // If we get here, most likely the render engine and scene are fully loaded,
    // but they don't support arrow visuals. So stop trying.
    ignition::gui::App()->findChild<
        ignition::gui::MainWindow *>()->removeEventFilter(this);
    return;
  }
}

/////////////////////////////////////////////////
void OriginAxesConfig::UpdateLength(double _length)
{
  this->dataPtr->originAxes->SetLocalScale(1, 1, _length);
  this->dataPtr->dirty = true;
}

/////////////////////////////////////////////////
void OriginAxesConfig::OnShow(bool _checked)
{
  this->dataPtr->visible = _checked;
  this->dataPtr->dirty = true;
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::OriginAxesConfig,
                    ignition::gui::Plugin)
