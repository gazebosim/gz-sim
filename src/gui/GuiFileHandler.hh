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
#ifndef GZ_SIM_GUI_GUIFILEHANDLER_HH_
#define GZ_SIM_GUI_GUIFILEHANDLER_HH_

#include <gz/msgs/sdf_generator_config.pb.h>

#include <QtCore>
#include <string>

#include <gz/transport/Node.hh>

#include "gz/sim/Export.hh"
#include "gz/sim/config.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace gui
{
/// \brief Class for handling saving and loading of SDFormat files
class GuiFileHandler : public QObject
{
  Q_OBJECT

  /// \brief Function called from QML when user asks to save a world file
  /// \param[in] _fileUrl Url to world file.
  /// \param[in] _config Object that contains configuration options for SDFormat
  /// file generation
  public: Q_INVOKABLE void SaveWorldAs(const QString &_fileUrl,
                                       QObject *_config);

  /// \brief Signal for displaying status messages to users
  /// \param[in] _status New status value. False if saving world failed.
  /// \param[in] _msg New status message.
  signals: void newSaveWorldStatus(bool _status, const QString &_msg);

  /// \brief Transport node.
  private: transport::Node node;
};
}
}
}
}
#endif
