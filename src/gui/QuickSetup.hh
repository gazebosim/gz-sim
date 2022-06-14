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
#ifndef IGNITION_GAZEBO_GUI_QUICKSETUPHANDLER_HH_
#define IGNITION_GAZEBO_GUI_QUICKSETUPHANDLER_HH_

#include <QtCore>
#include <QDesktopServices>
#include <string>

#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Export.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace gui
{
/// \brief Class for handling about dialog
class QuickSetupHandler : public QObject
{
  Q_OBJECT

  /// \brief Constructor
  public: QuickSetupHandler();

  /// \brief Get worlds path
  /// \return worlds directory path
  Q_INVOKABLE QString getWorldsPath();

  /// \brief Function called from QML when user clicks on a link
  /// \param[in] _url Url to web page.
  Q_INVOKABLE void SetStartingWorld(QString _url);

  std::string GetStartingWorld();
  
  /// \brief Callback in Qt thread when skip button is clicked.
  Q_INVOKABLE void OnSkip();

  /// \brief Callback when checkbox is clicked.
  /// \param[in] _checked indicates show quick setup dialog again or not
  Q_INVOKABLE void OnShowAgain(bool _checked);

  private: std::string worldsPath;

  private: bool showAgain;

  private: std::string startingWorld;
};
}
}
}
}
#endif
