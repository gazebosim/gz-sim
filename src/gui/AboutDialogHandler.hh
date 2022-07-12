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
#ifndef GZ_SIM_GUI_ABOUTDIALOGHANDLER_HH_
#define GZ_SIM_GUI_ABOUTDIALOGHANDLER_HH_

#include <QtCore>
#include <QDesktopServices>
#include <string>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Export.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace gui
{
/// \brief Class for handling about dialog
class AboutDialogHandler : public QObject
{
  Q_OBJECT

  /// \brief Constructor
  public: AboutDialogHandler();

  /// \brief Get version information
  /// \return Version information in rich text format
  Q_INVOKABLE QString getVersionInformation();

  /// \brief Function called from QML when user clicks on a link
  /// \param[in] _url Url to web page.
  Q_INVOKABLE void openURL(QString _url);

  /// \brief Version information and links to online resources
  private: std::string aboutText;
};
}
}
}
}
#endif
