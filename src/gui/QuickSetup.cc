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

#include <ignition/common/Console.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/gui/Application.hh>

using namespace ignition;
using namespace gazebo;
using namespace gazebo::gui;

/////////////////////////////////////////////////
QuickSetupHandler::QuickSetupHandler()
{
  aboutText += std::string(IGNITION_GAZEBO_VERSION_HEADER);
  aboutText += "<table class='nostyle'>"
                 "<tr>"
                 "</tr>"
               "</table>";
}

/////////////////////////////////////////////////
QString QuickSetupHandler::getVersionInformation()
{
  return QString::fromStdString(this->aboutText);
}

/////////////////////////////////////////////////
void QuickSetupHandler::openURL(QString _url)
{
  QDesktopServices::openUrl(QUrl(_url));
}

/////////////////////////////////////////////////
void QuickSetupHandler::OnSkip()
{
  // auto mainWin = ignition::gui::App()->findChild<ignition::gui::MainWindow *>("quickSetup");
  // mainWin->QuickWindow()->close(); 
  // auto mainWin = ignition::gui::App()->findChild<ignition::gui::MainWindow *>();
  for (size_t i = 0; i < ignition::gui::App()->children().size(); i++)
  {
      ignerr << "count " << std::endl; 
  }
  
  // auto win = mainWin->QuickWindow();
  // mainWin->findChild<QQuickItem *>("quickSetup")->close();

  // ignition::gui::App()->findChild<MainWindow *>()->QuickWindow()->close();
}