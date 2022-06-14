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

#include "QuickSetup.hh"

#include <ignition/common/SystemPaths.hh>
#include <ignition/common/Console.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/gui/Application.hh>

#include "ignition/gazebo/Util.hh"

using namespace ignition;
using namespace gazebo;
using namespace gazebo::gui;

/////////////////////////////////////////////////
QuickSetupHandler::QuickSetupHandler()
{
  common::SystemPaths systemPaths;

  // Worlds from environment variable
  systemPaths.SetFilePathEnv(kResourcePathEnv);

  // Worlds installed with ign-gazebo
  systemPaths.AddFilePaths(IGN_GAZEBO_WORLD_INSTALL_DIR);

  // TODO (mh) use Qt.resolvedUrl(filePath) instead of appending
  // worldsPath = "file://";
  for (const std::string &filePath : systemPaths.FilePaths())
  {
    if(filePath.find("world") !=std::string::npos){
      worldsPath += filePath;
    }
  }
}

/////////////////////////////////////////////////
QString QuickSetupHandler::getWorldsPath()
{
  ignerr<< this->worldsPath;
  return QString::fromUtf8(this->worldsPath.c_str());
}

std::string QuickSetupHandler::GetStartingWorld()
{
  return this->startingWorld;
}

/////////////////////////////////////////////////
void QuickSetupHandler::SetStartingWorld(QString _url)
{
  startingWorld = _url.toStdString();
}

/////////////////////////////////////////////////
void QuickSetupHandler::OnSkip()
{
  ignerr << "Implment window closing here " << std::endl; 
}

/////////////////////////////////////////////////
void QuickSetupHandler::OnShowAgain(bool _checked)
{
  showAgain = _checked;
}
