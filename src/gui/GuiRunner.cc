/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/fuel_tools/Interface.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>

// Include all components so they have first-class support
#include "gz/sim/components/components.hh"
#include "gz/sim/Conversions.hh"
#include "gz/sim/gui/GuiRunner.hh"
#include "gz/sim/gui/GuiSystem.hh"

using namespace gz;
using namespace gz::sim;

// Register SerializedStepMap to the Qt meta type system so we can pass objects
// of this type in QMetaObject::invokeMethod
Q_DECLARE_METATYPE(msgs::SerializedStepMap)

/////////////////////////////////////////////////
GuiRunner::GuiRunner(const std::string &_worldName)
{
  qRegisterMetaType<msgs::SerializedStepMap>();

  this->setProperty("worldName", QString::fromStdString(_worldName));

  auto win = gui::App()->findChild<gz::gui::MainWindow *>();
  auto winWorldNames = win->property("worldNames").toStringList();
  winWorldNames.append(QString::fromStdString(_worldName));
  win->setProperty("worldNames", winWorldNames);

  this->stateTopic = "/world/" + _worldName + "/state";

  common::addFindFileURICallback([] (common::URI _uri)
  {
    return fuel_tools::fetchResource(_uri.Str());
  });

  igndbg << "Requesting initial state from [" << this->stateTopic << "]..."
         << std::endl;

  this->RequestState();

  // Periodically update the plugins
  QPointer<QTimer> timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &GuiRunner::UpdatePlugins);
  timer->start(33);
}

/////////////////////////////////////////////////
GuiRunner::~GuiRunner() = default;

/////////////////////////////////////////////////
void GuiRunner::RequestState()
{
  // set up service for async state response callback
  std::string id = std::to_string(gui::App()->applicationPid());
  std::string reqSrv =
      this->node.Options().NameSpace() + "/" + id + "/state_async";
  this->node.Advertise(reqSrv, &GuiRunner::OnStateAsyncService, this);
  msgs::StringMsg req;
  req.set_data(reqSrv);

  // send async state request
  this->node.Request(this->stateTopic + "_async", req);
}

/////////////////////////////////////////////////
void GuiRunner::OnPluginAdded(const QString &)
{
  // This function used to call Update on the plugin, but that's no longer
  // necessary. The function is left here for ABI compatibility.
}

/////////////////////////////////////////////////
void GuiRunner::OnStateAsyncService(const msgs::SerializedStepMap &_res)
{
  this->OnState(_res);

  // todo(anyone) store reqSrv string in a member variable and use it here
  // and in RequestState()
  std::string id = std::to_string(gui::App()->applicationPid());
  std::string reqSrv =
      this->node.Options().NameSpace() + "/" + id + "/state_async";
  this->node.UnadvertiseSrv(reqSrv);

  // Only subscribe to periodic updates after receiving initial state
  if (this->node.SubscribedTopics().empty())
    this->node.Subscribe(this->stateTopic, &GuiRunner::OnState, this);
}

/////////////////////////////////////////////////
void GuiRunner::OnState(const msgs::SerializedStepMap &_msg)
{
  IGN_PROFILE_THREAD_NAME("GuiRunner::OnState");
  IGN_PROFILE("GuiRunner::Update");
  // Since this function may be called from a transport thread, we push the
  // OnStateQt function to the queue so that its called from the Qt thread. This
  // ensures that only one thread has access to the ecm and updateInfo
  // variables.
  QMetaObject::invokeMethod(this, "OnStateQt", Qt::QueuedConnection,
                            Q_ARG(msgs::SerializedStepMap, _msg));
}

/////////////////////////////////////////////////
void GuiRunner::OnStateQt(const msgs::SerializedStepMap &_msg)
{
  IGN_PROFILE_THREAD_NAME("Qt thread");
  IGN_PROFILE("GuiRunner::Update");
  this->ecm.SetState(_msg.state());

  // Update all plugins
  this->updateInfo = convert<UpdateInfo>(_msg.stats());
  this->UpdatePlugins();
  this->ecm.ClearNewlyCreatedEntities();
  this->ecm.ProcessRemoveEntityRequests();
}

/////////////////////////////////////////////////
void GuiRunner::UpdatePlugins()
{
  auto plugins = gui::App()->findChildren<GuiSystem *>();
  for (auto plugin : plugins)
  {
    plugin->Update(this->updateInfo, this->ecm);
  }
}
