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
#ifndef IGNITION_GAZEBO_GUI_PATHMANAGER_HH_
#define IGNITION_GAZEBO_GUI_PATHMANAGER_HH_

#include <QtCore>

#include <ignition/transport/Node.hh>

#include "ignition/gazebo/Export.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace gui
{
/// \brief Class for handling paths and their environment variables.
/// It queries the server for paths at startup, and keeps paths updated
/// whenever they change in the server.
class IGNITION_GAZEBO_VISIBLE PathManager : public QObject
{
  Q_OBJECT

  /// \brief Constructor
  public: PathManager();

  /// \brief Transport node.
  private: transport::Node node;
};
}
}
}
}
#endif
