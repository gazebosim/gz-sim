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
#ifndef GZ_GAZEBO_GUI_GUIRUNNER_HH_
#define GZ_GAZEBO_GUI_GUIRUNNER_HH_

#include <ignition/msgs/serialized.pb.h>

#include <QtCore>
#include <memory>
#include <mutex>
#include <string>

#include <gz/transport/Node.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/gui/Export.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
/// \brief Responsible for running GUI systems as new states are received from
/// the backend.
class IGNITION_GAZEBO_GUI_VISIBLE GuiRunner : public QObject
{
  Q_OBJECT

  /// \brief Constructor
  /// \param[in] _worldName World name.
  public: explicit GuiRunner(const std::string &_worldName);

  /// \brief Destructor
  public: ~GuiRunner() override;

  /// \brief Callback when a plugin has been added.
  /// This function has no effect and is left here for ABI compatibility.
  /// \param[in] _objectName Plugin's object name.
  public slots: void OnPluginAdded(const QString &_objectName);

  /// \brief Make a new state request to the server.
  public slots: void RequestState();

  /// \brief Callback for the async state service.
  /// \param[in] _res Response containing new state.
  private: void OnStateAsyncService(const msgs::SerializedStepMap &_res);

  /// \brief Callback when a new state is received from the server. Actual
  /// updating of the ECM is delegated to OnStateQt
  /// \param[in] _msg New state message.
  private: void OnState(const msgs::SerializedStepMap &_msg);

  /// \brief Called by the Qt thread to update the ECM with new state
  /// \param[in] _msg New state message.
  private: Q_INVOKABLE void OnStateQt(const msgs::SerializedStepMap &_msg);

  /// \brief Update the plugins.
  /// \todo(anyone) Move to GuiRunner::Implementation when porting to v5
  private: Q_INVOKABLE void UpdatePlugins();

  /// \brief Entity-component manager.
  private: ignition::gazebo::EntityComponentManager ecm;

  /// \brief Transport node.
  private: transport::Node node;

  /// \brief Topic to request state
  private: std::string stateTopic;

  /// \brief Latest update info
  private: UpdateInfo updateInfo;
};
}
}
}
#endif
