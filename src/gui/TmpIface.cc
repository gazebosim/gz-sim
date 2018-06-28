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

#include <ignition/common/Console.hh>

#include "ignition/gazebo/gui/TmpIface.hh"

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
TmpIface::TmpIface()// : dataPtr(new TmpIfacePrivate)
{
  // World control
  this->node.Advertise("/world_control",
        &TmpIface::OnWorldControl, this);

  // World statistics
  this->worldStatsPub =
      this->node.Advertise<msgs::WorldStatistics>("/world_stats");

  std::thread([this]()
  {
    while (true)
    {
      static int sec{0};

      msgs::WorldStatistics msg;
      auto time = msg.mutable_sim_time();
      time->set_sec(sec++);

      this->worldStatsPub.Publish(msg);

      std::this_thread::sleep_for(
          std::chrono::milliseconds(1000));
    }
  }).detach();

  // Server control
  this->node.Advertise("/server_control",
      &TmpIface::OnServerControl, this);
}

/////////////////////////////////////////////////
bool TmpIface::OnWorldControl(const msgs::WorldControl &_req,
                                    msgs::Boolean &_res)
{
  igndbg << "OnWorldControl: request" << std::endl;
  igndbg << _req.DebugString() << std::endl;

  igndbg << "OnWorldControl: response" << std::endl;
  igndbg << _res.DebugString() << std::endl;

  return true;
}

/////////////////////////////////////////////////
bool TmpIface::OnServerControl(const msgs::ServerControl &_req,
                                     msgs::Boolean &_res)
{
  igndbg << "OnServerControl: request" << std::endl;
  igndbg << _req.DebugString() << std::endl;

  _res.set_data(true);

  igndbg << "OnServerControl: response" << std::endl;
  igndbg << _res.DebugString() << std::endl;

  return true;
}

/////////////////////////////////////////////////
void TmpIface::OnLoadWorld(const QString &_path)
{
  auto localPath = QUrl(_path).toLocalFile();
  if (localPath.isEmpty())
    localPath = _path;

  std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
      [](const ignition::msgs::Boolean &_res, const bool _result)
  {
    igndbg << "LoadWorld callback: result: " << _result << std::endl;
    igndbg << "LoadWorld callback: response: " << _res.DebugString()
           << std::endl;
  };

  msgs::ServerControl req;
  req.set_open_filename(localPath.toStdString());
  this->node.Request("/server_control", req, cb);
}

