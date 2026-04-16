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

#include <gz/common/Image.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/Util.hh"
#include "gz/sim/World.hh"

#include <cstddef>
#include <functional>
#include <sstream>
#include <string>

#include <QFileDialog>

using namespace gz;
using namespace sim;

class gz::sim::ExportOccupancyUi::Implementation
{
  public: std::string worldName;
  public: gz::transport::Node node;
  public: gz::transport::Node::Publisher startPub;
  public: ImageProvider* provider;
  public: gz::msgs::Image lastOccupancy;
};

ExportOccupancyUi::ExportOccupancyUi() :
  dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
{
  gui::App()->Engine()->rootContext()->setContextProperty(
    "exportOccupancy", this);
  this->dataPtr->startPub =
    this->dataPtr->node.Advertise<gz::msgs::Boolean>("/start");
  this->dataPtr->provider = new ImageProvider();

  this->dataPtr->node.Subscribe("/scan_image",
      &ExportOccupancyUi::OnImageMsg,
      this);
}

ExportOccupancyUi::~ExportOccupancyUi()
{

}

void ExportOccupancyUi::RegisterImageProvider(const QString &_uniqueName)
{
  gz::gui::App()->Engine()->addImageProvider(_uniqueName,
                                    this->dataPtr->provider);
}

void ExportOccupancyUi::OnImageMsg(const gz::msgs::Image &img)
{
  this->dataPtr->lastOccupancy = img;
  unsigned int height = img.height();
  unsigned int width = img.width();
  QImage::Format qFormat = QImage::Format_RGB888;
  QImage image = QImage(width, height, qFormat);
  image = QImage(reinterpret_cast<const uchar *>(
        img.data().c_str()), width, height,
        3 * width, qFormat);
  this->dataPtr->provider->SetImage(image);
  emit this->newImage();
}

void ExportOccupancyUi::LoadConfig(
    const tinyxml2::XMLElement */*_pluginElem*/)
{
  if (this->title.empty())
    this->title = "Export Occupancy";

  gui::App()->findChild<gui::MainWindow *>()->installEventFilter(this);
}

void ExportOccupancyUi::Update(const UpdateInfo &,
    EntityComponentManager &_ecm)
{
  auto world = World(worldEntity(_ecm));
  if (!world.Valid(_ecm))
  {
    gzerr << "Could not get running world" << std::endl;
    return;
  }
  if (!world.Name(_ecm).has_value())
  {
    return;
  }
  this->dataPtr->worldName = world.Name(_ecm).value();
}

void ExportOccupancyUi::StartExport(double _samples, double _range,
  double _rangeRes, double _angularRes,
  double _distanceFromGround, double _gridResolution,
  std::size_t _numWidth, std::size_t _numHeight)
{
  gz::msgs::EntityFactory factoryReq;
  std::stringstream ss;
  ss << R"(
    <sdf version="1.6">
    <model name="model_with_lidar">
      <pose>1.5 1.5 )" << _distanceFromGround << R"( 0 0 0</pose>
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
    </sdf>
  )";

  factoryReq.set_sdf(ss.str());

  gz::transport::Node node;
  std::function<void(const msgs::Boolean &, const bool)> cb =
        [](const msgs::Boolean &/*_rep*/, const bool _result)
    {};
  node.Request("/world/" + this->dataPtr->worldName + "/create",
      factoryReq, cb);
}

void ExportOccupancyUi::StartExploration()
{
  gz::msgs::Boolean start;
  start.set_data(true);
  this->dataPtr->startPub.Publish(start);
}

void ExportOccupancyUi::Save()
{
  auto last = this->dataPtr->lastOccupancy;
  common::Image image;
  image.SetFromData(
    reinterpret_cast<unsigned char*>(
      const_cast<char*>(last.data().data())),
    last.width(), last.height(),
    common::Image::PixelFormatType::RGB_INT8);

  QString fileName = QFileDialog::getSaveFileName(
    nullptr,
    tr("Save File"),
    QDir::homePath(),
    tr("PNG Files (*.png);"));
  if (fileName.isEmpty())
  {
    return;
  }
  image.SavePNG(fileName.toStdString());
}
// Register this plugin
GZ_ADD_PLUGIN(gz::sim::ExportOccupancyUi, gz::gui::Plugin)
