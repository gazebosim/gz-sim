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
#ifndef GZ_SIM_GUI_GUIRUNNER_HH_
#define GZ_SIM_GUI_GUIRUNNER_HH_

#include <gz/msgs/serialized_map.pb.h>

#include <QtCore>
#include <string>

#include <gz/utils/ImplPtr.hh>

#include "gz/sim/config.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/gui/Export.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
/// \brief Responsible for running GUI systems as new states are received from
/// the backend.
class GZ_SIM_GUI_VISIBLE GuiRunner : public QObject
{
  Q_OBJECT

  /// \brief Constructor
  /// \param[in] _worldName World name.
  public: explicit GuiRunner(const std::string &_worldName);

  /// \brief Destructor
  public: ~GuiRunner() override;

  /// \brief Get the event manager for the gui
  public: EventManager &GuiEventManager() const;

  // Documentation inherited
  protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

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
  private: Q_INVOKABLE void UpdatePlugins();

  /// \brief Load systems
  private: void LoadSystems();

  /// \brief Update systems
  private: void UpdateSystems();

  /// \brief Pointer to private data.
  GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
};
}
}
}
#endif
