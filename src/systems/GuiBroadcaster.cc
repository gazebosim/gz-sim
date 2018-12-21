/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <ignition/msgs/gui.pb.h>

#include <sdf/Gui.hh>
#include <ignition/plugin/RegisterMore.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/Conversions.hh"
#include "ignition/gazebo/components/Gui.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/systems/GuiBroadcaster.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

// Private data class.
class ignition::gazebo::systems::GuiBroadcasterPrivate
{
  /// \brief Callback for scene info service.
  /// \param[out] _res Response containing the latest scene message.
  /// \return True if successful.
  public: bool GuiInfoService(ignition::msgs::GUI &_res);

  /// \brief Transport node.
  public: std::unique_ptr<transport::Node> node{nullptr};

  /// \brief Keep the latest GUI message.
  public: msgs::GUI msg;
};

//////////////////////////////////////////////////
GuiBroadcaster::GuiBroadcaster()
  : System(), dataPtr(std::make_unique<GuiBroadcasterPrivate>())
{
}

//////////////////////////////////////////////////
GuiBroadcaster::~GuiBroadcaster()
{
}

//////////////////////////////////////////////////
void GuiBroadcaster::Configure(const EntityId &_id,
    const std::shared_ptr<const sdf::Element> &/*_sdf*/,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  // Get world name
  auto worldName = _ecm.Component<components::Name>(_id)->Data();

  // Get <gui> element
  auto guiComp = _ecm.Component<components::Gui>(_id);
  if (nullptr != guiComp)
  {
    // Publish empty messages for worlds that have no GUI in the beginning.
    // In the future, support adding GUI from the server at runtime.
    auto gui = guiComp->Data();
    this->dataPtr->msg = Convert<msgs::GUI>(gui);
  }

  // Gui info service
  transport::NodeOptions opts;
  opts.SetNameSpace("/world/" + worldName);
  this->dataPtr->node = std::make_unique<transport::Node>(opts);

  std::string infoService{"gui/info"};

  this->dataPtr->node->Advertise(infoService,
      &GuiBroadcasterPrivate::GuiInfoService, this->dataPtr.get());

  ignmsg << "Serving GUI information on [" << opts.NameSpace() << "/"
         << infoService << "]" << std::endl;
}

//////////////////////////////////////////////////
bool GuiBroadcasterPrivate::GuiInfoService(ignition::msgs::GUI &_res)
{
  _res.Clear();

  _res.CopyFrom(this->msg);

  return true;
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::GuiBroadcaster,
                    ignition::gazebo::System,
                    GuiBroadcaster::ISystemConfigure)
