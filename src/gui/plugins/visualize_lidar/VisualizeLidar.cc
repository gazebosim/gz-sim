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

#include <string>
#include <utility>
#include <vector>

#include <sdf/Link.hh>
#include <sdf/Model.hh>

#include <ignition/common/Console.hh>
#include <ignition/common/Profiler.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include <ignition/transport/Node.hh>

#include <ignition/gui/Conversions.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/gui/GuiEvents.hh"
#include "ignition/gazebo/rendering/RenderUtil.hh"

#include "ignition/rendering/RenderTypes.hh"
#include "ignition/rendering/RenderingIface.hh"
#include "ignition/rendering/RenderEngine.hh"
#include "ignition/rendering/Scene.hh"
#include "ignition/rendering/LidarVisual.hh"

#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/Util.hh"

#include "ignition/msgs/laserscan.pb.h"
#include "VisualizeLidar.hh"

namespace ignition
{
namespace gazebo
{
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE
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

    /// \brief Visual type for lidar visual
    public: rendering::LidarVisualType visualType{
                            rendering::LidarVisualType::LVT_TRIANGLE_STRIPS};

    /// \brief URI sequence to the lidar link
    public: std::string lidarString{""};

    /// \brief LaserScan message from sensor
    public: msgs::LaserScan msg;

    /// \brief Pose of the lidar visual
    public: math::Pose3d lidarPose{math::Pose3d::Zero};

    /// \brief Topic name to subscribe
    public: std::string topicName{""};

    /// \brief List of topics publishing LaserScan messages.
    public: QStringList topicList;

    /// \brief Entity representing the sensor in the world
    public: gazebo::Entity lidarEntity;

    /// \brief Minimum range for the visual
    public: double minVisualRange{0.0};

    /// \brief Maximum range for the visual
    public: double maxVisualRange{0.0};

    /// \brief Mutex for variable mutated by the checkbox and spinboxes
    /// callbacks.
    /// The variables are: msg, visualType, minVisualRange and
    /// maxVisualRange
    public: std::mutex serviceMutex;

    /// \brief Initialization flag
    public: bool initialized{false};

    /// \brief Reset visual flag
    public: bool resetVisual{false};

    /// \brief lidar visual display dirty flag
    public: bool visualDirty{false};

    /// \brief lidar sensor entity dirty flag
    public: bool lidarEntityDirty{true};
  };
}
}
}

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
VisualizeLidar::VisualizeLidar()
  : GuiSystem(), dataPtr(new VisualizeLidarPrivate)
{
  // no ops
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
  auto loadedEngNames = rendering::loadedEngines();
  if (loadedEngNames.empty())
    return;

  // assume there is only one engine loaded
  auto engineName = loadedEngNames[0];
  if (loadedEngNames.size() > 1)
  {
    igndbg << "More than one engine is available. "
      << "VisualizeLidar plugin will use engine ["
        << engineName << "]" << std::endl;
  }
  auto engine = rendering::engine(engineName);
  if (!engine)
  {
    ignerr << "Internal error: failed to load engine [" << engineName
      << "]. VisualizeLidar plugin won't work." << std::endl;
    return;
  }

  if (engine->SceneCount() == 0)
    return;

  // assume there is only one scene
  // load scene
  auto scene = engine->SceneByIndex(0);
  if (!scene)
  {
    ignerr << "Internal error: scene is null." << std::endl;
    return;
  }

  if (!scene->IsInitialized() || scene->VisualCount() == 0)
  {
    return;
  }

  // Create lidar visual
  igndbg << "Creating lidar visual" << std::endl;

  auto root = scene->RootVisual();
  this->dataPtr->lidar = scene->CreateLidarVisual();
  if (!this->dataPtr->lidar)
  {
    ignwarn << "Failed to create lidar, visualize lidar plugin won't work."
            << std::endl;

    scene->DestroyVisual(this->dataPtr->lidar);

    ignition::gui::App()->findChild<
        ignition::gui::MainWindow *>()->removeEventFilter(this);
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

  ignition::gui::App()->findChild<
    ignition::gui::MainWindow *>()->installEventFilter(this);
}

/////////////////////////////////////////////////
bool VisualizeLidar::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == ignition::gazebo::gui::events::Render::kType)
  {
    // This event is called in Scene3d's RenderThread, so it's safe to make
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
      ignerr << "Lidar pointer is not set" << std::endl;
    }
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

//////////////////////////////////////////////////
void VisualizeLidar::Update(const UpdateInfo &,
    EntityComponentManager &_ecm)
{
  IGN_PROFILE("VisualizeLidar::Update");

  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);

  if (this->dataPtr->lidarEntityDirty)
  {
    auto lidarURIVec = common::split(common::trimmed(
                  this->dataPtr->lidarString), "::");
    if (lidarURIVec.size() > 0)
    {
      auto baseEntity = _ecm.EntityByComponents(
          components::Name(lidarURIVec[0]));
      if (!baseEntity)
      {
        ignerr << "Error entity " << lidarURIVec[0]
            << " doesn't exist and cannot be used to set lidar visual pose"
            << std::endl;
        return;
      }
      else
      {
        auto parent = baseEntity;
        bool success = false;
        for (size_t i = 0u; i < lidarURIVec.size()-1; i++)
        {
          auto children = _ecm.EntitiesByComponents(
                            components::ParentEntity(parent));
          bool foundChild = false;
          for (auto child : children)
          {
            std::string nextstring = lidarURIVec[i+1];
            std::string childname = std::string(
                            _ecm.Component<components::Name>(child)->Data());
            if (nextstring.compare(childname) == 0)
            {
              parent = child;
              foundChild = true;
              if (i+1 == lidarURIVec.size()-1)
              {
                success = true;
              }
              break;
            }
          }
          if (!foundChild)
          {
            ignerr << "The entity could not be found."
                  << "Error displaying lidar visual" <<std::endl;
            return;
          }
        }
        if (success)
        {
          this->dataPtr->lidarEntity = parent;
          this->dataPtr->lidarEntityDirty = false;
        }
      }
    }
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
void VisualizeLidar::OnTopic(const QString &_topicName)
{
  std::lock_guard<std::mutex>(this->dataPtr->serviceMutex);
  if (!this->dataPtr->topicName.empty() &&
      !this->dataPtr->node.Unsubscribe(this->dataPtr->topicName))
  {
    ignerr << "Unable to unsubscribe from topic ["
           << this->dataPtr->topicName <<"]" <<std::endl;
  }
  this->dataPtr->topicName = _topicName.toStdString();

  // Reset visualization
  this->dataPtr->resetVisual = true;

  // Create new subscription
  if (!this->dataPtr->node.Subscribe(this->dataPtr->topicName,
                            &VisualizeLidar::OnScan, this))
  {
    ignerr << "Unable to subscribe to topic ["
           << this->dataPtr->topicName << "]\n";
    return;
  }
  this->dataPtr->visualDirty = false;
  ignmsg << "Subscribed to " << this->dataPtr->topicName << std::endl;
}

//////////////////////////////////////////////////
void VisualizeLidar::UpdateNonHitting(bool _value)
{
  std::lock_guard<std::mutex>(this->dataPtr->serviceMutex);
  this->dataPtr->lidar->SetDisplayNonHitting(_value);
}

//////////////////////////////////////////////////
void VisualizeLidar::DisplayVisual(bool _value)
{
  std::lock_guard<std::mutex>(this->dataPtr->serviceMutex);
  this->dataPtr->lidar->SetVisible(_value);
  ignmsg << "Lidar Visual Display " << ((_value) ? "ON." : "OFF.")
         << std::endl;
}

/////////////////////////////////////////////////
void VisualizeLidar::OnRefresh()
{
  std::lock_guard<std::mutex>(this->dataPtr->serviceMutex);
  ignmsg << "Refreshing topic list for LaserScan messages." << std::endl;

  // Clear
  this->dataPtr->topicList.clear();

  // Get updated list
  std::vector<std::string> allTopics;
  this->dataPtr->node.TopicList(allTopics);
  for (auto topic : allTopics)
  {
    std::vector<transport::MessagePublisher> publishers;
    this->dataPtr->node.TopicInfo(topic, publishers);
    for (auto pub : publishers)
    {
      if (pub.MsgTypeName() == "ignition.msgs.LaserScan")
      {
        this->dataPtr->topicList.push_back(QString::fromStdString(topic));
        break;
      }
    }
  }
  if (this->dataPtr->topicList.size() > 0)
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
  std::lock_guard<std::mutex>(this->dataPtr->serviceMutex);
  if (this->dataPtr->initialized)
  {
    this->dataPtr->msg = std::move(_msg);
    this->dataPtr->lidar->SetVerticalRayCount(
                                  this->dataPtr->msg.vertical_count());
    this->dataPtr->lidar->SetHorizontalRayCount(
                                  this->dataPtr->msg.count());
    this->dataPtr->lidar->SetMinHorizontalAngle(
                                  this->dataPtr->msg.angle_min());
    this->dataPtr->lidar->SetMaxHorizontalAngle(
                                  this->dataPtr->msg.angle_max());
    this->dataPtr->lidar->SetMinVerticalAngle(
                                  this->dataPtr->msg.vertical_angle_min());
    this->dataPtr->lidar->SetMaxVerticalAngle(
                                  this->dataPtr->msg.vertical_angle_max());

    this->dataPtr->lidar->SetPoints(std::vector<double>(
                                  this->dataPtr->msg.ranges().begin(),
                                  this->dataPtr->msg.ranges().end()));

    this->dataPtr->visualDirty = true;

    for (auto data_values : this->dataPtr->msg.header().data())
    {
      if (data_values.key() == "frame_id")
      {
        if (this->dataPtr->lidarString.compare(
                common::trimmed(data_values.value(0))) != 0)
        {
          this->dataPtr->lidarString = common::trimmed(data_values.value(0));
          this->dataPtr->lidarEntityDirty = true;
          this->dataPtr->maxVisualRange = this->dataPtr->msg.range_max();
          this->dataPtr->minVisualRange = this->dataPtr->msg.range_min();
          this->dataPtr->lidar->SetMaxRange(this->dataPtr->maxVisualRange);
          this->dataPtr->lidar->SetMinRange(this->dataPtr->minVisualRange);
          this->MinRangeChanged();
          this->MaxRangeChanged();
          break;
        }
      }
    }
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
IGNITION_ADD_PLUGIN(ignition::gazebo::VisualizeLidar,
                    ignition::gui::Plugin)
