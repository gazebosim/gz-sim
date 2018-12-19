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
#include <signal.h>
#include <tinyxml2.h>

#include <ignition/common/Console.hh>

#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>

#include <iostream>

#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/gui/TmpIface.hh"

//////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // TODO: cmd line args

  // Temporary transport interface
  auto tmp = std::make_unique<ignition::gazebo::TmpIface>();

  // Initialize Qt app
  ignition::gui::Application app(_argc, _argv);

  // Load configuration file
  auto configPath = ignition::common::joinPaths(
      IGNITION_GAZEBO_GUI_CONFIG_PATH, "gui.config");

  if (!app.LoadConfig(configPath))
  {
    return -1;
  }

  // Customize window
  auto win = app.findChild<ignition::gui::MainWindow *>()->QuickWindow();
  win->setProperty("title", "Gazebo");

  // Let QML files use TmpIface' functions and properties
  auto context = new QQmlContext(app.Engine()->rootContext());
  context->setContextProperty("TmpIface", tmp.get());

  // Instantiate GazeboDrawer.qml file into a component
  QQmlComponent component(app.Engine(), ":/Gazebo/GazeboDrawer.qml");
  auto gzDrawerItem = qobject_cast<QQuickItem *>(component.create(context));
  if (gzDrawerItem)
  {
    // C++ ownership
    QQmlEngine::setObjectOwnership(gzDrawerItem, QQmlEngine::CppOwnership);

    // Add to main window
    auto parentDrawerItem = win->findChild<QQuickItem *>("sideDrawer");
    gzDrawerItem->setParentItem(parentDrawerItem);
    gzDrawerItem->setParent(app.Engine());
  }
  else
  {
    ignerr << "Failed to instantiate custom drawer, drawer will be empty"
           << std::endl;
  }

  // Request GUI info
  // TODO
  std::string service{"/world/shapes/gui/info"};
  ignition::transport::Node node;

  bool executed{false};
  bool result{false};
  unsigned int timeout{5000};

  igndbg << std::endl << "Requesting GUI from [" << service
            << "]..." << std::endl << std::endl;

  // Request and block
  ignition::msgs::GUI res;
  executed = node.Request(service, timeout, res, result);

  if (!executed)
    ignerr << std::endl << "Service call timed out" << std::endl;
  else if (!result)
    ignerr << std::endl<< "Service call failed" << std::endl;

  for (int p = 0; p < res.plugin_size(); ++p)
  {
    auto plugin = res.plugin(p);
    auto fileName = plugin.filename();
    std::string pluginStr = "<plugin filename='" + fileName + "'>" +
        plugin.innerxml() + "</plugin>";

    tinyxml2::XMLDocument pluginDoc;
    pluginDoc.Parse(pluginStr.c_str());

    ignition::gui::App()->LoadPlugin(fileName,
        pluginDoc.FirstChildElement("plugin"));
  }

  // Run main window.
  // This blocks until the window is closed or we receive a SIGINT
  app.exec();

  igndbg << "Shutting down" << std::endl;
  return 0;
}
