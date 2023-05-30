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

#include <gz/gui/Application.hh>
//! [includeGuiEvents]
#include <gz/gui/GuiEvents.hh>
//! [includeGuiEvents]
#include <gz/gui/MainWindow.hh>
#include <gz/math/Rand.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>

#include "RenderingGuiPlugin.hh"

/////////////////////////////////////////////////
void RenderingGuiPlugin::LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/)
{
  // This is necessary to receive the Render event on eventFilter
//! [connectToGuiEvent]
  gz::gui::App()->findChild<
      gz::gui::MainWindow *>()->installEventFilter(this);
//! [connectToGuiEvent]
}

/////////////////////////////////////////////////
void RenderingGuiPlugin::RandomColor()
{
  this->dirty = true;
}

/////////////////////////////////////////////////
//! [eventFilter]
bool RenderingGuiPlugin::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gz::gui::events::Render::kType)
  {
    // This event is called in the render thread, so it's safe to make
    // rendering calls here
    this->PerformRenderingOperations();
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}
//! [eventFilter]

/////////////////////////////////////////////////
//! [performRenderingOperations]
void RenderingGuiPlugin::PerformRenderingOperations()
{
  if (!this->dirty)
  {
    return;
  }

  if (nullptr == this->scene)
  {
    this->FindScene();
  }

  if (nullptr == this->scene)
    return;

  this->scene->SetAmbientLight({
      static_cast<float>(gz::math::Rand::DblUniform(0.0, 1.0)),
      static_cast<float>(gz::math::Rand::DblUniform(0.0, 1.0)),
      static_cast<float>(gz::math::Rand::DblUniform(0.0, 1.0)),
      1.0});

  this->dirty = false;
}
//! [performRenderingOperations]

/////////////////////////////////////////////////
void RenderingGuiPlugin::FindScene()
{
  auto loadedEngNames = gz::rendering::loadedEngines();
  if (loadedEngNames.empty())
  {
    gzdbg << "No rendering engine is loaded yet" << std::endl;
    return;
  }

  // assume there is only one engine loaded
  auto engineName = loadedEngNames[0];
  if (loadedEngNames.size() > 1)
  {
    gzdbg << "More than one engine is available. "
      << "Using engine [" << engineName << "]" << std::endl;
  }
  auto engine = gz::rendering::engine(engineName);
  if (!engine)
  {
    gzerr << "Internal error: failed to load engine [" << engineName
      << "]. Grid plugin won't work." << std::endl;
    return;
  }

  if (engine->SceneCount() == 0)
  {
    gzdbg << "No scene has been created yet" << std::endl;
    return;
  }

  // Get first scene
  auto scenePtr = engine->SceneByIndex(0);
  if (nullptr == scenePtr)
  {
    gzerr << "Internal error: scene is null." << std::endl;
    return;
  }

  if (engine->SceneCount() > 1)
  {
    gzdbg << "More than one scene is available. "
      << "Using scene [" << scene->Name() << "]" << std::endl;
  }

  if (!scenePtr->IsInitialized() || nullptr == scenePtr->RootVisual())
  {
    return;
  }

  this->scene = scenePtr;
}

// Register this plugin
GZ_ADD_PLUGIN(RenderingGuiPlugin,
                    gz::gui::Plugin)
