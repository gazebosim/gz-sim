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
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/stringmsg.pb.h>

#include <iostream>
#include <ignition/common/Console.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/transport/Publisher.hh>

#include "ignition/gazebo/test_config.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/gui/GuiEvents.hh"

#include "Shapes.hh"

namespace ignition::gazebo
{
  class ShapesPrivate
  {
    /// \brief Ignition communication node.
    public: transport::Node node;

    /// \brief Mutex to protect mode
    public: std::mutex mutex;

    /// \brief Transform control service name
    public: std::string service;
  };
}

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
Shapes::Shapes()
  : ignition::gui::Plugin(),
  dataPtr(std::make_unique<ShapesPrivate>())
{
}

/////////////////////////////////////////////////
Shapes::~Shapes() = default;

/////////////////////////////////////////////////
void Shapes::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Shapes";

  // For transform requests
  this->dataPtr->service = "/gui/shapes";
}

/////////////////////////////////////////////////
void Shapes::OnMode(const QString &_mode)
{
  std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
      [](const ignition::msgs::Boolean &/*_rep*/, const bool _result)
  {
    if (!_result)
      ignerr << "Error setting shape" << std::endl;
  };

  ignition::msgs::StringMsg req;

  std::string sdfString;
  if (_mode == "box")
  {
    sdfString = std::string("<?xml version=\"1.0\"?>"
                            "<sdf version=\"1.6\">"
                              "<model name=\"box\">"
                                "<pose>0 0 0.5 0 0 0</pose>"
                                "<link name=\"box_link\">"
                                  "<inertial>"
                                    "<inertia>"
                                      "<ixx>1</ixx>"
                                      "<ixy>0</ixy>"
                                      "<ixz>0</ixz>"
                                      "<iyy>1</iyy>"
                                      "<iyz>0</iyz>"
                                      "<izz>1</izz>"
                                    "</inertia>"
                                    "<mass>1.0</mass>"
                                  "</inertial>"
                                  "<collision name=\"box_collision\">"
                                    "<geometry>"
                                      "<box>"
                                        "<size>1 1 1</size>"
                                      "</box>"
                                    "</geometry>"
                                  "</collision>"
                                  "<visual name=\"box_visual\">"
                                    "<geometry>"
                                      "<box>"
                                        "<size>1 1 1</size>"
                                      "</box>"
                                    "</geometry>"
                                  "</visual>"
                                "</link>"
                              "</model>"
                            "</sdf>");
  }
  else if (_mode == "sphere")
  {
    sdfString = std::string("<?xml version=\"1.0\"?>"
                            "<sdf version=\"1.6\">"
                              "<model name=\"sphere\">"
                                "<pose>0 0 0.5 0 0 0</pose>"
                                "<link name=\"sphere_link\">"
                                  "<inertial>"
                                    "<inertia>"
                                      "<ixx>1</ixx>"
                                      "<ixy>0</ixy>"
                                      "<ixz>0</ixz>"
                                      "<iyy>1</iyy>"
                                      "<iyz>0</iyz>"
                                      "<izz>1</izz>"
                                    "</inertia>"
                                    "<mass>1.0</mass>"
                                  "</inertial>"
                                  "<collision name=\"sphere_collision\">"
                                    "<geometry>"
                                      "<sphere>"
                                        "<radius>0.5</radius>"
                                      "</sphere>"
                                    "</geometry>"
                                  "</collision>"
                                  "<visual name=\"sphere_visual\">"
                                    "<geometry>"
                                      "<sphere>"
                                        "<radius>0.5</radius>"
                                      "</sphere>"
                                    "</geometry>"
                                  "</visual>"
                                "</link>"
                              "</model>"
                            "</sdf>");
  }
  else if (_mode == "cylinder")
  {
    sdfString = std::string("<?xml version=\"1.0\"?>"
                            "<sdf version=\"1.6\">"
                              "<model name=\"cylinder\">"
                                "<pose>0 0 0.5 0 0 0</pose>"
                                "<link name=\"cylinder_link\">"
                                  "<inertial>"
                                    "<inertia>"
                                      "<ixx>1</ixx>"
                                      "<ixy>0</ixy>"
                                      "<ixz>0</ixz>"
                                      "<iyy>1</iyy>"
                                      "<iyz>0</iyz>"
                                      "<izz>1</izz>"
                                    "</inertia>"
                                    "<mass>1.0</mass>"
                                  "</inertial>"
                                  "<collision name=\"cylinder_collision\">"
                                    "<geometry>"
                                      "<cylinder>"
                                        "<radius>0.5</radius>"
                                        "<length>1.0</length>"
                                      "</cylinder>"
                                    "</geometry>"
                                  "</collision>"
                                  "<visual name=\"cylinder_visual\">"
                                    "<geometry>"
                                      "<cylinder>"
                                        "<radius>0.5</radius>"
                                        "<length>1.0</length>"
                                      "</cylinder>"
                                    "</geometry>"
                                  "</visual>"
                                "</link>"
                              "</model>"
                            "</sdf>");
  }
  req.set_data(sdfString);
  this->dataPtr->node.Request(this->dataPtr->service, req, cb);
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::Shapes,
                    ignition::gui::Plugin)
