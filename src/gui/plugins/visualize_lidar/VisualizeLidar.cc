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

#include "VisualizeLidar.hh"

#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>

#include <gz/plugin/Register.hh>

#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>

#include <gz/transport/Node.hh>

#include <gz/gui/Application.hh>
#include <gz/gui/Conversions.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>

#include "gz/sim/components/Name.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/rendering/RenderUtil.hh"

#include "gz/rendering/RenderTypes.hh"
#include "gz/rendering/RenderingIface.hh"
#include "gz/rendering/RenderEngine.hh"
#include "gz/rendering/Scene.hh"
#include "gz/rendering/LidarVisual.hh"

#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/Util.hh"

#include "gz/msgs/laserscan.pb.h"

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE
{
  /// \brief Private data class for VisualizeLidar
  class VisualizeLidarPrivate
  {
    /// \brief Transport node
    public: transport::Node node;

    /// \brief Scene Pointer
    public: rendering::ScenePtr scene;

    /// \brief Pointer to LidarVisual
    public: rendering::LidarVisualPtr lidar;

    /// \brief Pose of the lidar visual
    public: math::Pose3d lidarPose{math::Pose3d::Zero};

    /// \brief Topic name to subscribe
    public: std::string topicName;

    /// \brief List of topics publishing LaserScan messages.
    public: QStringList topicList;

    /// \brief Entity representing the sensor in the world
    public: sim::Entity lidarEntity;

    /// \brief Minimum range for the visual
    public: double minVisualRange{0.0};

    /// \brief Maximum range for the visual
    public: double maxVisualRange{0.0};

    /// \brief Mutex for variable mutated by the checkbox and spinboxes
    /// callbacks.
    /// The variables are: msg, visualType, minVisualRange and
    /// maxVisualRange
    public: std::mutex serviceMutex;

    /// \brief Visual type for lidar visual
    public: rendering::LidarVisualType visualType{
                            rendering::LidarVisualType::LVT_TRIANGLE_STRIPS};

    /// \brief Initialization flag
    public: bool initialized{false};

    /// \brief Reset visual flag
    public: bool resetVisual{false};

    /// \brief lidar visual display dirty flag
    public: bool visualDirty{false};

    /// \brief lidar sensor entity dirty flag
    public: bool lidarEntityDirty{false};
  };
}
}
}

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
VisualizeLidar::VisualizeLidar()
  : GuiSystem(), dataPtr(new VisualizeLidarPrivate)
{
}

/////////////////////////////////////////////////
VisualizeLidar::~VisualizeLidar()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->scene->DestroyVisual(this->dataPtr->lidar);
}

/////////////////////////////////////////////////
void VisualizeLidar::LoadLidar()
{
  auto scene = rendering::sceneFromFirstRenderEngine();
  if (!scene)
    return;

  // Create lidar visual
  gzdbg << "Creating lidar visual" << std::endl;

  auto root = scene->RootVisual();
  this->dataPtr->lidar = scene->CreateLidarVisual();
  if (!this->dataPtr->lidar)
  {
    gzwarn << "Failed to create lidar, visualize lidar plugin won't work."
           << std::endl;

    scene->DestroyVisual(this->dataPtr->lidar);

    gz::gui::App()->findChild<
        gz::gui::MainWindow *>()->removeEventFilter(this);
  }
  else
  {
    this->dataPtr->scene = scene;
    root->AddChild(this->dataPtr->lidar);
    this->dataPtr->initialized = true;
  }
}

/////////////////////////////////////////////////
void VisualizeLidar::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Visualize lidar";

  gz::gui::App()->findChild<
    gz::gui::MainWindow *>()->installEventFilter(this);
}

/////////////////////////////////////////////////
bool VisualizeLidar::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gz::gui::events::Render::kType)
  {
    // This event is called in the RenderThread, so it's safe to make
    // rendering calls here

    std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
    if (!this->dataPtr->initialized)
    {
      this->LoadLidar();
    }

    if (this->dataPtr->lidar)
    {
      if (this->dataPtr->resetVisual)
      {
        this->dataPtr->lidar->ClearPoints();
        this->dataPtr->resetVisual = false;
      }
      if (this->dataPtr->visualDirty)
      {
        this->dataPtr->lidar->SetWorldPose(this->dataPtr->lidarPose);
        this->dataPtr->lidar->Update();
        this->dataPtr->visualDirty = false;
      }
    }
    else
    {
      gzerr << "Lidar pointer is not set" << std::endl;
    }
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

//////////////////////////////////////////////////
void VisualizeLidar::Update(const UpdateInfo &,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("VisualizeLidar::Update");

  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);

  if (this->dataPtr->topicName.empty())
    return;

  if (this->dataPtr->lidarEntityDirty)
  {
    std::string topic = this->dataPtr->topicName;
    auto lidarEnt =
        _ecm.EntityByComponents(components::SensorTopic(topic));
    if (lidarEnt == kNullEntity)
    {
      if (topic[0] == '/')
        topic = topic.substr(1);
      lidarEnt =
        _ecm.EntityByComponents(components::SensorTopic(topic));
    }

    static bool informed{false};
    if (lidarEnt == kNullEntity)
    {
      if (!informed)
      {
        gzerr << "The lidar entity with topic '['" <<  this->dataPtr->topicName
              << "'] could not be found. "
              << "Error displaying lidar visual. " << std::endl;
        informed = true;
      }
      return;
    }
    informed = false;
    this->dataPtr->lidarEntity = lidarEnt;
    this->dataPtr->lidarEntityDirty = false;
  }

  if (!_ecm.HasEntity(this->dataPtr->lidarEntity))
  {
    this->dataPtr->resetVisual = true;
    this->dataPtr->topicName = "";
    return;
  }

  // Only update lidarPose if the lidarEntity exists and the lidar is
  // initialized and the sensor message is yet to arrive.
  //
  // If we update the worldpose on the physics thread **after** the sensor
  // data arrives, the visual is offset from the obstacle if the sensor is
  // moving fast.
  if (!this->dataPtr->lidarEntityDirty && this->dataPtr->initialized &&
      !this->dataPtr->visualDirty)
  {
    this->dataPtr->lidarPose = worldPose(this->dataPtr->lidarEntity, _ecm);
  }
}

//////////////////////////////////////////////////
void VisualizeLidar::UpdateType(int _type)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  switch (_type)
  {
    case 0:
      this->dataPtr->visualType = rendering::LidarVisualType::LVT_NONE;
      break;

    case 1:
      this->dataPtr->visualType = rendering::LidarVisualType::LVT_RAY_LINES;
      break;

    case 2:
      this->dataPtr->visualType = rendering::LidarVisualType::LVT_POINTS;
      break;

    case 3:
      this->dataPtr->visualType =
                            rendering::LidarVisualType::LVT_TRIANGLE_STRIPS;
      break;

    default:
      this->dataPtr->visualType =
                            rendering::LidarVisualType::LVT_TRIANGLE_STRIPS;
      break;
  }
  this->dataPtr->lidar->SetType(this->dataPtr->visualType);
}

//////////////////////////////////////////////////
void VisualizeLidar::UpdateSize(int _size)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->lidar->SetSize(_size);
}

//////////////////////////////////////////////////
void VisualizeLidar::OnTopic(const QString &_topicName)
{
  std::string topic = _topicName.toStdString();
  if (this->dataPtr->topicName == topic)
    return;

  if (!this->dataPtr->topicName.empty() &&
      !this->dataPtr->node.Unsubscribe(this->dataPtr->topicName))
  {
    gzerr << "Unable to unsubscribe from topic ["
          << this->dataPtr->topicName <<"]" <<std::endl;
  }
  this->dataPtr->topicName = topic;

  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  // Reset visualization
  this->dataPtr->resetVisual = true;

  // Create new subscription
  if (!this->dataPtr->node.Subscribe(this->dataPtr->topicName,
                            &VisualizeLidar::OnScan, this))
  {
    gzerr << "Unable to subscribe to topic ["
          << this->dataPtr->topicName << "]\n";
    return;
  }
  this->dataPtr->visualDirty = false;
  gzmsg << "Subscribed to " << this->dataPtr->topicName << std::endl;

  this->dataPtr->lidarEntityDirty = true;
}

//////////////////////////////////////////////////
void VisualizeLidar::UpdateNonHitting(bool _value)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->lidar->SetDisplayNonHitting(_value);
}

//////////////////////////////////////////////////
void VisualizeLidar::DisplayVisual(bool _value)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->lidar->SetVisible(_value);
  gzmsg << "Lidar Visual Display " << ((_value) ? "ON." : "OFF.")
        << std::endl;
}

/////////////////////////////////////////////////
void VisualizeLidar::OnRefresh()
{
  gzmsg << "Refreshing topic list for LaserScan messages." << std::endl;

  // Clear
  this->dataPtr->topicList.clear();

  // Get updated list
  std::vector<std::string> allTopics;
  this->dataPtr->node.TopicList(allTopics);
  for (const auto &topic : allTopics)
  {
    std::vector<transport::MessagePublisher> publishers;
    std::vector<transport::MessagePublisher> subscribers;
    this->dataPtr->node.TopicInfo(topic, publishers, subscribers);
    for (const auto &pub : publishers)
    {
      if (pub.MsgTypeName() == "gz.msgs.LaserScan")
      {
        this->dataPtr->topicList.push_back(QString::fromStdString(topic));
        break;
      }
    }
  }
  if (!this->dataPtr->topicList.empty())
  {
    this->OnTopic(this->dataPtr->topicList.at(0));
  }

  this->TopicListChanged();
}

/////////////////////////////////////////////////
QStringList VisualizeLidar::TopicList() const
{
  return this->dataPtr->topicList;
}

/////////////////////////////////////////////////
void VisualizeLidar::SetTopicList(const QStringList &_topicList)
{
  this->dataPtr->topicList = _topicList;
  this->TopicListChanged();
}

//////////////////////////////////////////////////
void VisualizeLidar::OnScan(const msgs::LaserScan &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  if (this->dataPtr->initialized)
  {
    this->dataPtr->lidar->SetVerticalRayCount(_msg.vertical_count());
    this->dataPtr->lidar->SetHorizontalRayCount(_msg.count());
    this->dataPtr->lidar->SetMinHorizontalAngle(_msg.angle_min());
    this->dataPtr->lidar->SetMaxHorizontalAngle(_msg.angle_max());
    this->dataPtr->lidar->SetMinVerticalAngle(_msg.vertical_angle_min());
    this->dataPtr->lidar->SetMaxVerticalAngle(_msg.vertical_angle_max());

    this->dataPtr->lidar->SetPoints(std::vector<double>(
                                  _msg.ranges().begin(),
                                  _msg.ranges().end()));

    if (!math::equal(this->dataPtr->maxVisualRange, _msg.range_max()))
    {
      this->dataPtr->maxVisualRange = _msg.range_max();
      this->dataPtr->lidar->SetMaxRange(this->dataPtr->maxVisualRange);
      this->MaxRangeChanged();
    }
    if (!math::equal(this->dataPtr->minVisualRange, _msg.range_min()))
    {
      this->dataPtr->minVisualRange = _msg.range_min();
      this->dataPtr->lidar->SetMinRange(this->dataPtr->minVisualRange);
      this->MinRangeChanged();
    }

    this->dataPtr->visualDirty = true;
  }
}

//////////////////////////////////////////////////
QString VisualizeLidar::MaxRange() const
{
  return QString::fromStdString(std::to_string(this->dataPtr->maxVisualRange));
}

//////////////////////////////////////////////////
QString VisualizeLidar::MinRange() const
{
  return QString::fromStdString(std::to_string(this->dataPtr->minVisualRange));
}

// Register this plugin
GZ_ADD_PLUGIN(gz::sim::VisualizeLidar,
                    gz::gui::Plugin)
