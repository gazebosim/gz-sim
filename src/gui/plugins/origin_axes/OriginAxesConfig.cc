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
    /// \brief Assume only one gridptr in a scene
    public: rendering::VisualPtr x_axis = nullptr;
    public: rendering::VisualPtr y_axis = nullptr;
    public: rendering::VisualPtr z_axis = nullptr;

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
  // Load grid if it doesn't already exist
  if (!this->dataPtr->x_axis || !this->dataPtr->y_axis || !this->dataPtr->z_axis)
    this->LoadOriginAxes();

  // If grid was not loaded successfully, don't update
  if (!this->dataPtr->x_axis || !this->dataPtr->y_axis || !this->dataPtr->z_axis)
    return;

  if (!this->dataPtr->dirty)
    return;

  this->dataPtr->dirty = false;

  auto visual_x = this->dataPtr->x_axis;
  if (visual_x)
  {
    visual_x->SetVisible(this->dataPtr->visible);
  }

  auto visual_y = this->dataPtr->y_axis;
  if (visual_y)
  {
    visual_y->SetVisible(this->dataPtr->visible);
  }

  auto visual_z = this->dataPtr->z_axis;
  if (visual_z)
  {
    visual_z->SetVisible(this->dataPtr->visible);
  }
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
      << "Grid config plugin will use engine ["
        << engineName << "]" << std::endl;
  }
  auto engine = rendering::engine(engineName);
  if (!engine)
  {
    ignerr << "Internal error: failed to load engine [" << engineName
      << "]. Grid plugin won't work." << std::endl;
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

  if (this->dataPtr->x_axis || this->dataPtr->y_axis || this->dataPtr->z_axis)
    return;

  // Create grid
  ignwarn << "Creating origin axes" << std::endl;

  auto root = scene->RootVisual();
  this->dataPtr->x_axis = scene->CreateArrowVisual();
  this->dataPtr->y_axis = scene->CreateArrowVisual();
  this->dataPtr->z_axis = scene->CreateArrowVisual();
  if (!this->dataPtr->x_axis || !this->dataPtr->y_axis || !this->dataPtr->z_axis)
  {
    ignerr << "Failed to create origin axes, origin axes config plugin won't work."
            << std::endl;

    // If we get here, most likely the render engine and scene are fully loaded,
    // but they don't support grids. So stop trying.
    ignition::gui::App()->findChild<
        ignition::gui::MainWindow *>()->removeEventFilter(this);
    return;
  }

  this->dataPtr->x_axis->SetLocalPosition(0, 0, 0);
  this->dataPtr->x_axis->SetLocalRotation(0, IGN_PI / 2, 0);
  this->dataPtr->x_axis->SetMaterial("Default/TransRed");
  root->AddChild(this->dataPtr->x_axis);

  this->dataPtr->y_axis->SetLocalPosition(0, 0, 0);
  this->dataPtr->y_axis->SetLocalRotation(-IGN_PI / 2, 0, 0);
  this->dataPtr->y_axis->SetMaterial("Default/TransGreen");
  root->AddChild(this->dataPtr->y_axis);

  this->dataPtr->z_axis->SetLocalPosition(0, 0, 0);
  this->dataPtr->z_axis->SetLocalRotation(0, 0, 0);
  this->dataPtr->z_axis->SetMaterial("Default/TransBlue");
  root->AddChild(this->dataPtr->z_axis);
}

/////////////////////////////////////////////////
void OriginAxesConfig::UpdateSize(int _cellCount)
{
  this->dataPtr->x_axis->SetLocalScale(1, 1, _cellCount);
  this->dataPtr->y_axis->SetLocalScale(1, 1, _cellCount);
  this->dataPtr->z_axis->SetLocalScale(1, 1, _cellCount);
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
