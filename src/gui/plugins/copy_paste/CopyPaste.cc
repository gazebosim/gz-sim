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

#include "CopyPaste.hh"

#include <memory>
#include <mutex>
#include <string>

#include <ignition/common/Console.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/msgs.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport.hh>

namespace ignition::gazebo
{
  class CopyPastePrivate
  {
    public: std::string copiedData = "";

    public: transport::Node node;

    public: const std::string copyService = "/gui/copy";

    public: const std::string pasteService = "/gui/paste";

    public: bool CopyServiceCB(const ignition::msgs::StringMsg &_req,
                ignition::msgs::Boolean &_resp);

    public: bool PasteServiceCB(const ignition::msgs::Empty &_req,
                ignition::msgs::Boolean &_resp);

    /// \brief A mutex to ensure that there are no race conditions between
    /// copy/paste
    public: std::mutex mutex;
  };
}

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
CopyPaste::CopyPaste()
  : ignition::gui::Plugin(), dataPtr(std::make_unique<CopyPastePrivate>())
{
  if (!this->dataPtr->node.Advertise(this->dataPtr->copyService,
        &CopyPastePrivate::CopyServiceCB, this->dataPtr.get()))
  {
    ignerr << "Error advertising service [" << this->dataPtr->copyService
      << "]" << std::endl;
  }

  if (!this->dataPtr->node.Advertise(this->dataPtr->pasteService,
        &CopyPastePrivate::PasteServiceCB, this->dataPtr.get()))
  {
    ignerr << "Error advertising service [" << this->dataPtr->pasteService
      << "]" << std::endl;
  }
}

/////////////////////////////////////////////////
CopyPaste::~CopyPaste() = default;

/////////////////////////////////////////////////
void CopyPaste::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Copy/Paste";
}

/////////////////////////////////////////////////
bool CopyPastePrivate::CopyServiceCB(const ignition::msgs::StringMsg &_req,
    ignition::msgs::Boolean &_resp)
{
  {
    std::lock_guard<std::mutex> guard(this->mutex);
    this->copiedData = _req.data();
  }
  _resp.set_data(true);
  return true;
}

/////////////////////////////////////////////////
bool CopyPastePrivate::PasteServiceCB(const ignition::msgs::Empty &/*_req*/,
    ignition::msgs::Boolean &_resp)
{
  {
    std::lock_guard<std::mutex> guard(this->mutex);

    // we should only paste if something has been copied
    if (!this->copiedData.empty())
    {
      ignition::gui::events::SpawnCloneFromName event(this->copiedData);
      ignition::gui::App()->sendEvent(
        ignition::gui::App()->findChild<ignition::gui::MainWindow *>(),
        &event);
    }
  }
  _resp.set_data(true);
  return true;
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::CopyPaste,
                    ignition::gui::Plugin)
