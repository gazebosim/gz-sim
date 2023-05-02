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

#include <gz/common/Console.hh>

#include "gz/sim/gui/TmpIface.hh"

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
TmpIface::TmpIface()
{
}


/////////////////////////////////////////////////
bool TmpIface::OnServerControl(const msgs::ServerControl &/*_req*/,
                                     msgs::Boolean &/*_res*/)
{
  ignerr << "Called wrong /server_control callback. Call the one in "
         << "ServerPrivate instead." << std::endl;
  return false;
}

/////////////////////////////////////////////////
void TmpIface::OnNewWorld()
{
  std::function<void(const msgs::Boolean &, const bool)> cb =
      [](const msgs::Boolean &_res, const bool _result)
  {
    igndbg << "NewWorld callback: result: " << _result << std::endl;
    igndbg << "NewWorld callback: response: " << _res.DebugString()
           << std::endl;
  };

  msgs::ServerControl req;
  req.set_new_world(true);
  this->node.Request("/server_control", req, cb);
}

/////////////////////////////////////////////////
void TmpIface::OnLoadWorld(const QString &_path)
{
  auto localPath = QUrl(_path).toLocalFile();
  if (localPath.isEmpty())
    localPath = _path;

  std::function<void(const msgs::Boolean &, const bool)> cb =
      [](const msgs::Boolean &_res, const bool _result)
  {
    igndbg << "LoadWorld callback: result: " << _result << std::endl;
    igndbg << "LoadWorld callback: response: " << _res.DebugString()
           << std::endl;
  };

  msgs::ServerControl req;
  req.set_open_filename(localPath.toStdString());
  this->node.Request("/server_control", req, cb);
}

/////////////////////////////////////////////////
void TmpIface::OnSaveWorldAs(const QString &_path)
{
  auto localPath = QUrl(_path).toLocalFile();
  if (localPath.isEmpty())
    localPath = _path;

  std::function<void(const msgs::Boolean &, const bool)> cb =
      [](const msgs::Boolean &_res, const bool _result)
  {
    igndbg << "SaveWorldAs callback: result: " << _result << std::endl;
    igndbg << "SaveWorldAs callback: response: " << _res.DebugString()
           << std::endl;
  };

  msgs::ServerControl req;
  req.set_save_filename(localPath.toStdString());
  this->node.Request("/server_control", req, cb);
}
