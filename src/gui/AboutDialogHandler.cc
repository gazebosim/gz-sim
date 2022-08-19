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

#include "AboutDialogHandler.hh"

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/gui/Application.hh>

using namespace gz;
using namespace sim;
using namespace sim::gui;

/////////////////////////////////////////////////
AboutDialogHandler::AboutDialogHandler()
{
  aboutText += std::string(IGNITION_GAZEBO_VERSION_HEADER);
  aboutText += "<table class='nostyle'>"
                 "<tr>"
                   "<td style='padding-right: 10px;'>Documentation:"
                   "</td>"
                   "<td>"
                     "<a href='https://ignitionrobotics.org/libs/gazebo' "
                     "style='text-decoration: none; color: #f58113'>"
                       "https://ignitionrobotics.org/libs/gazebo"
                     "</a>"
                   "</td>"
                 "</tr>"
                 "<tr>"
                   "<td style='padding-right: 10px;'>"
                     "Tutorials:"
                   "</td>"
                   "<td>"
                     "<a href='https://ignitionrobotics.org/docs/' "
                     "style='text-decoration: none; color: #f58113'>"
                       "https://ignitionrobotics.org/docs/"
                     "</a>"
                   "</td>"
                 "</tr>"
               "</table>";
}

/////////////////////////////////////////////////
QString AboutDialogHandler::getVersionInformation()
{
  return QString::fromStdString(this->aboutText);
}

/////////////////////////////////////////////////
void AboutDialogHandler::openURL(QString _url)
{
  QDesktopServices::openUrl(QUrl(_url));
}
