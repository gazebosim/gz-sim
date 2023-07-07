/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include "QuickStartHandler.hh"

#include "gz/sim/InstallationDirectories.hh"

using namespace gz;
using namespace sim::gui;

/////////////////////////////////////////////////
QString QuickStartHandler::WorldsPath() const
{
  std::string worldsPathLocal = gz::sim::getWorldInstallDir();
  return QString::fromUtf8(worldsPathLocal.c_str());
}

/////////////////////////////////////////////////
std::string QuickStartHandler::StartingWorld() const
{
  return this->startingWorld;
}

/////////////////////////////////////////////////
QString QuickStartHandler::Distribution() const
{
  return QString::fromUtf8(GZ_DISTRIBUTION);
}

/////////////////////////////////////////////////
QString QuickStartHandler::SimVersion() const
{
  return QString::fromUtf8(GZ_SIM_VERSION_FULL);
}

/////////////////////////////////////////////////
void QuickStartHandler::SetStartingWorld(const QString &_url)
{
  this->startingWorld = _url.toStdString();
}

/////////////////////////////////////////////////
void QuickStartHandler::SetShowAgain(const bool _showAgain)
{
  this->showAgain = !_showAgain;
}

/////////////////////////////////////////////////
bool QuickStartHandler::ShowAgain() const
{
  return this->showAgain;
}
