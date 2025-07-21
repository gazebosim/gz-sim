/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#include "ExportOccupancy.hh"
#include <gz/msgs/entity_factory.pb.h>
#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/plugin/Register.hh>

#include <sstream>

using namespace gz;
using namespace sim;

class gz::sim::ExportOccupancyUiPrivate
{
};

ExportOccupancyUi::ExportOccupancyUi() : dataPtr(std::make_unique<ExportOccupancyUiPrivate>())
{
  gui::App()->Engine()->rootContext()->setContextProperty(
    "exportOccupancy", this);
}

ExportOccupancyUi::~ExportOccupancyUi()
{

}

void ExportOccupancyUi::LoadConfig(
    const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
    this->title = "Export Occupancy";

  gui::App()->findChild<gui::MainWindow *>()->installEventFilter(this);
}

void ExportOccupancyUi::Update(const UpdateInfo &,
    EntityComponentManager &_ecm)
{

}

void ExportOccupancyUi::StartExport(double _samples, double _range, double _rangeRes, double _angularRes,
  double _distanceFromGround, double _gridResolution, std::size_t _numWidth, std::size_t _numHeight)
{
  gz::msgs::EntityFactory factoryReq;
  std::stringstream ss;
  ss << R"(
    <model name="model_with_lidar">
      <pose>0 0 )" << _distanceFromGround << R"( 0 0 0</pose>
      <link name="link">
          <inertial>
            <mass>0.1</mass>
            <inertia>
              <ixx>0.000166667</ixx>
              <iyy>0.000166667</iyy>
              <izz>0.000166667</izz>
            </inertia>
          </inertial>
          <collision name="collision">
            <geometry>
            <box>
              <size>0.1 0.1 0.1</size>
            </box>
            </geometry>
          </collision>
          <visual name="visual">
            <geometry>
              <box>
                <size>0.1 0.1 0.1</size>
              </box>
            </geometry>
          </visual>

          <sensor name='gpu_lidar' type='gpu_lidar'>
              <topic>freespace_explorer/scan</topic>
              <update_rate>10</update_rate>
              <lidar>
              <scan>
                <horizontal>
                <samples>)"<< _samples << R"(</samples>
                <resolution>)" << _angularRes << R"(</resolution>
                <min_angle>0</min_angle>
                <max_angle>6.28</max_angle>
                </horizontal>
              </scan>
              <range>
                <min>0.08</min>
                <max>)"<< _range << R"(</max>
                <resolution>)"<< _rangeRes <<R"(</resolution>
              </range>
              </lidar>
              <visualize>true</visualize>
          </sensor>
      </link>
      <static>true</static>

      <plugin
        filename="gz-sim-free-space-explorer-system"
        name="gz::sim::systems::FreeSpaceExplorer">
        <lidar_topic>freespace_explorer/scan</lidar_topic>
        <width>)" << _numWidth << R"(</width>
        <height>)" << _numHeight << R"(</height>
        <resolution>)" << _gridResolution << R"(</resolution>
        <sensor_link>link</sensor_link>
      </plugin>
    </model>
  )";
  factoryReq.set_sdf(ss.str());
}

// Register this plugin
GZ_ADD_PLUGIN(gz::sim::ExportOccupancyUi, gz::gui::Plugin)
