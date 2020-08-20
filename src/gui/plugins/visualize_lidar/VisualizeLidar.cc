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
#include <vector>

#include <sdf/Link.hh>
#include <sdf/Model.hh>

#include <ignition/common/Animation.hh>
#include <ignition/common/Console.hh>
#include <ignition/common/KeyFrame.hh>
#include <ignition/common/MeshManager.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/common/Uuid.hh>
#include <ignition/common/VideoEncoder.hh>

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
    public: rendering::LidarVisualType visualType;

    /// \brief URI sequence to the lidar link
    public: std::string lidarString{""};

    /// \brief LaserScan message from sensor
    public: msgs::LaserScan msg;

    /// \brief Pose of the lidar visual
    public: math::Pose3d lidarPose{math::Pose3d::Zero};

    /// \brief Current state of the checkbox
    public: bool checkboxState{false};

    /// \brief Topic name to subscribe
    public: std::string topicName;

    /// \brief Entity representing the sensor in the world
    public: gazebo::Entity lidarEntity;

    /// \brief Minimum range for the visual
    public: double minVisualRange;

    /// \brief Maximum range for the visual
    public: double maxVisualRange;

    /// \brief Mutex for variable mutated by the checkbox and spinboxes
    /// callbacks.
    /// The variables are: msg, visualType, minVisualRange and
    /// maxVisualRange
    public: std::mutex serviceMutex;

    /// \brief Initialization flag
    public: bool initialized{false};

    /// \brief lidar visual display dirty flag
    public: bool visualDirty{false};

    /// \brief lidar sensor entity dirty flag
    public: bool lidarEntityDirty{false};

    /// \brief Name of the world
    public: std::string worldName;
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
VisualizeLidar::~VisualizeLidar() = default;

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
    ignwarn << "Failed to create lidar, lidar visual plugin won't work."
            << std::endl;

    ignition::gui::App()->findChild<
        ignition::gui::MainWindow *>()->removeEventFilter(this);
    return;
  }
  if (this->dataPtr->lidar)
  {
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
        for (auto i = 0ul; i < lidarURIVec.size()-1; i++)
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
          if (foundChild == false)
          {
            ignerr << "The entity could not be found."
                  << "Error displaying lidar visual" <<std::endl;
            return;
          }
        }
        if (success == true)
        {
          this->dataPtr->lidarEntity = parent;
          this->dataPtr->lidarEntityDirty = false;
        }
      }
    }
  }
  if (this->dataPtr->lidarEntity)
  {
    this->dataPtr->lidarPose = worldPose(this->dataPtr->lidarEntity, _ecm);
  }
}

//////////////////////////////////////////////////
void VisualizeLidar::UpdateType(int _type)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  switch (_type) {
    case 0: this->dataPtr->visualType =
                      rendering::LidarVisualType::LVT_NONE;
            break;
    case 1: this->dataPtr->visualType =
                      rendering::LidarVisualType::LVT_RAY_LINES;
            break;
    case 2: this->dataPtr->visualType =
                      rendering::LidarVisualType::LVT_POINTS;
            break;
    case 3: this->dataPtr->visualType =
                      rendering::LidarVisualType::LVT_TRIANGLE_STRIPS;
            break;
    default: this->dataPtr->visualType =
                      rendering::LidarVisualType::LVT_TRIANGLE_STRIPS;
            break;
  }
  this->dataPtr->lidar->SetType(this->dataPtr->visualType);
}

//////////////////////////////////////////////////
void VisualizeLidar::SetTopicName(const QString &_topicName)
{
  std::lock_guard<std::mutex>(this->dataPtr->serviceMutex);
  this->dataPtr->node.Unsubscribe(this->dataPtr->topicName);
  this->dataPtr->topicName = _topicName.toStdString();

  // Reset visualization
  this->ResetLidarVisual();

  // Create new subscription
  if (!this->dataPtr->node.Subscribe(this->dataPtr->topicName,
                            &VisualizeLidar::OnScan, this))
  {
    ignerr << "Input subscriber could not be created for topic ["
           << this->dataPtr->topicName << "]\n";
    return;
  }
  ignmsg << "Subscribed to " << this->dataPtr->topicName << std::endl;
  this->TopicNameChanged();
}

//////////////////////////////////////////////////
QString VisualizeLidar::TopicName() const
{
  return QString::fromStdString(this->dataPtr->topicName);
}

//////////////////////////////////////////////////
void VisualizeLidar::UpdateNonHitting(bool _value)
{
  std::lock_guard<std::mutex>(this->dataPtr->serviceMutex);
  this->dataPtr->lidar->SetDisplayNonHitting(_value);
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
        }
      }
    }
  }
}

//////////////////////////////////////////////////
void VisualizeLidar::ResetLidarVisual()
{
  this->dataPtr->lidar->ClearPoints();
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
