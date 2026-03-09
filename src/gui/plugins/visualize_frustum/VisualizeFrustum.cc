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

#include "VisualizeFrustum.hh"

#include <cmath>
#include <cstddef>
#include <memory>
#include <mutex>
#include <ostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/common/StringUtils.hh>

#include <gz/plugin/Register.hh>

#include <gz/math/Pose3.hh>

#include <gz/transport/Node.hh>

#include <gz/gui/Application.hh>
#include <gz/gui/Conversions.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>

#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/rendering/RenderUtil.hh"

#include "gz/rendering/RenderTypes.hh"
#include "gz/rendering/RenderingIface.hh"
#include "gz/rendering/RenderEngine.hh"
#include "gz/rendering/Scene.hh"
#include "gz/rendering/FrustumVisual.hh"

#include "gz/sim/Util.hh"

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE
{
  /// \brief Private data class for VisualizeFrustum
  class VisualizeFrustumPrivate
  {
    /// \brief Transport node
    public: transport::Node node;

    /// \brief Scene Pointer
    public: rendering::ScenePtr scene;

    /// \brief Pointer to FrustumVisual
    public: rendering::FrustumVisualPtr frustum;

    /// \brief URI sequence to the frustum link
    public: std::string frustumString;

    /// \brief Pose of the frustum visual
    public: math::Pose3d frustumPose{math::Pose3d::Zero};

    /// \brief Topic name to subscribe
    public: std::string topicName;

    /// \brief List of topics publishing supported camera messages.
    public: QStringList topicList;

    /// \brief Mapping of topic names to message types.
    public: std::unordered_map<std::string, std::string> topicTypes;

    /// \brief Default near clip for CameraInfo-derived frustums.
    public: double defaultNearClip{0.1};

    /// \brief Default far clip for CameraInfo-derived frustums.
    public: double defaultFarClip{10.0};

    /// \brief Entity representing the sensor in the world
    public: sim::Entity frustumEntity;

    /// \brief Mutex for variables mutated by callbacks.
    public: std::mutex serviceMutex;

    /// \brief Initialization flag
    public: bool initialized{false};

    /// \brief Reset visual flag
    public: bool resetVisual{false};

    /// \brief Frustum visual display dirty flag
    public: bool visualDirty{false};

    /// \brief Frustum sensor entity dirty flag
    public: bool frustumEntityDirty{true};
  };
}
}
}

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
VisualizeFrustum::VisualizeFrustum()
  : GuiSystem(), dataPtr(new VisualizeFrustumPrivate)
{
}

/////////////////////////////////////////////////
VisualizeFrustum::~VisualizeFrustum()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  if (this->dataPtr->scene && this->dataPtr->frustum)
    this->dataPtr->scene->DestroyVisual(this->dataPtr->frustum);
}

/////////////////////////////////////////////////
std::string VisualizeFrustum::FrameIdFromHeader(
    const msgs::Header &_header) const
{
  for (const auto &dataValues : _header.data())
  {
    if (dataValues.key() == "frame_id" && dataValues.value_size() > 0)
    {
      return common::trimmed(dataValues.value(0));
    }
  }
  return "";
}

/////////////////////////////////////////////////
bool VisualizeFrustum::FrustumDataFromLogicalCamera(
    const msgs::LogicalCameraSensor &_msg,
    FrustumData &_data) const
{
  _data.nearClip = _msg.near_clip();
  _data.farClip = _msg.far_clip();
  _data.horizontalFov = _msg.horizontal_fov();
  _data.aspectRatio = _msg.aspect_ratio();
  _data.frameId = this->FrameIdFromHeader(_msg.header());
  _data.valid = !_data.frameId.empty();

  return _data.valid;
}

/////////////////////////////////////////////////
bool VisualizeFrustum::FrustumDataFromCameraInfo(
    const msgs::CameraInfo &_msg,
    FrustumData &_data) const
{
  if (_msg.width() == 0 || _msg.height() == 0)
    return false;

  const auto &intrinsics = _msg.intrinsics();
  if (intrinsics.k_size() < 9)
    return false;

  const double width = static_cast<double>(_msg.width());
  const double height = static_cast<double>(_msg.height());
  const double fx = intrinsics.k(0);

  if (fx <= 0.0)
    return false;

  _data.aspectRatio = width / height;
  _data.horizontalFov = 2.0 * std::atan(width / (2.0 * fx));
  _data.nearClip = this->dataPtr->defaultNearClip;
  _data.farClip = this->dataPtr->defaultFarClip;
  _data.frameId = this->FrameIdFromHeader(_msg.header());
  _data.valid = !_data.frameId.empty();

  return _data.valid;
}

/////////////////////////////////////////////////
void VisualizeFrustum::ApplyFrustumData(const FrustumData &_data)
{
  if (!this->dataPtr->initialized || !this->dataPtr->frustum || !_data.valid)
    return;

  this->dataPtr->frustum->SetNearClipPlane(_data.nearClip);
  this->dataPtr->frustum->SetFarClipPlane(_data.farClip);
  this->dataPtr->frustum->SetHFOV(_data.horizontalFov);
  this->dataPtr->frustum->SetAspectRatio(_data.aspectRatio);
  this->dataPtr->visualDirty = true;

  if (this->dataPtr->frustumString != _data.frameId)
  {
    this->dataPtr->frustumString = _data.frameId;
    this->dataPtr->frustumEntityDirty = true;
  }
}

/////////////////////////////////////////////////
void VisualizeFrustum::LoadFrustum()
{
  auto scene = rendering::sceneFromFirstRenderEngine();
  if (!scene)
  {
    gzerr << "Internal error: scene is null." << std::endl;
    return;
  }

  // Create frustum visual
  gzdbg << "Creating frustum visual" << std::endl;
  auto root = scene->RootVisual();

  // Create a frustum visual
  // \todo(iche033) uncomment and use official API in gz-rendering10
  // this->dataPtr->frustum = scene->CreateFrustumVisual();
  this->dataPtr->frustum =
    std::dynamic_pointer_cast<rendering::FrustumVisual>(
      scene->Extension()->CreateExt("frustum_visual"));
  if (!this->dataPtr->frustum)
  {
    gzwarn << "Failed to create frustum, visualize frustum plugin won't work."
           << std::endl;

    scene->DestroyVisual(this->dataPtr->frustum);

    gz::gui::App()->findChild<
        gz::gui::MainWindow *>()->removeEventFilter(this);
  }
  else
  {
    this->dataPtr->scene = scene;
    root->AddChild(this->dataPtr->frustum);
    this->dataPtr->initialized = true;
  }
}

/////////////////////////////////////////////////
void VisualizeFrustum::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Visualize frustum";

  gz::gui::App()->findChild<
    gz::gui::MainWindow *>()->installEventFilter(this);
}

/////////////////////////////////////////////////
bool VisualizeFrustum::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gz::gui::events::Render::kType)
  {
    // This event is called in the RenderThread, so it's safe to make
    // rendering calls here

    std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
    if (!this->dataPtr->initialized)
    {
      this->LoadFrustum();
    }

    if (this->dataPtr->frustum)
    {
      if (this->dataPtr->resetVisual)
      {
        this->dataPtr->resetVisual = false;
      }
      if (this->dataPtr->visualDirty)
      {
        this->dataPtr->frustum->SetWorldPose(this->dataPtr->frustumPose);
        this->dataPtr->frustum->Update();
        this->dataPtr->visualDirty = false;
      }
    }
    else
    {
      gzerr << "Frustum pointer is not set" << std::endl;
    }
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

//////////////////////////////////////////////////
void VisualizeFrustum::Update(const UpdateInfo &,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("VisualizeFrustum::Update");

  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);

  if (this->dataPtr->frustumEntityDirty)
  {
    const auto frustumURIVec = common::split(common::trimmed(
                  this->dataPtr->frustumString), "::");
    if (!frustumURIVec.empty())
    {
      const auto baseEntity = _ecm.EntityByComponents(
          components::Name(frustumURIVec[0]));
      if (!baseEntity)
      {
        gzerr << "Error entity " << frustumURIVec[0]
              << " doesn't exist and cannot be used to set frustum visual pose"
              << std::endl;
        return;
      }
      else
      {
        auto parent = baseEntity;
        bool success = false;
        for (size_t i = 0u; i < frustumURIVec.size()-1; ++i)
        {
          const auto children = _ecm.EntitiesByComponents(
                            components::ParentEntity(parent));
          bool foundChild = false;
          for (const auto child : children)
          {
            const auto &nextstring = frustumURIVec[i+1];
            auto comp = _ecm.Component<components::Name>(child);
            if (!comp)
            {
              continue;
            }
            const auto &childname = comp->Data();
            if (nextstring == childname)
            {
              parent = child;
              foundChild = true;
              if (i+1 == frustumURIVec.size()-1)
              {
                success = true;
              }
              break;
            }
          }
          if (!foundChild)
          {
            gzerr << "The entity could not be found."
                  << "Error displaying frustum visual" << std::endl;
            return;
          }
        }
        if (success)
        {
          this->dataPtr->frustumEntity = parent;
          this->dataPtr->frustumEntityDirty = false;
        }
      }
    }
  }

  // Only update frustumPose if the frustumEntity exists and the frustum is
  // initialized and the sensor message is yet to arrive.
  //
  // If we update the worldPose on the physics thread **after** the sensor
  // data arrives, the visual is offset from the obstacle if the sensor is
  // moving fast.
  if (!this->dataPtr->frustumEntityDirty && this->dataPtr->initialized &&
                                          !this->dataPtr->visualDirty)
  {
    this->dataPtr->frustumPose = worldPose(this->dataPtr->frustumEntity, _ecm);
  }
}

//////////////////////////////////////////////////
void VisualizeFrustum::OnTopic(const QString &_topicName)
{
  if (!this->dataPtr->topicName.empty() &&
      !this->dataPtr->node.Unsubscribe(this->dataPtr->topicName))
  {
    gzerr << "Unable to unsubscribe from topic ["
          << this->dataPtr->topicName << "]" << std::endl;
  }

  this->dataPtr->topicName = _topicName.toStdString();

  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);

  this->dataPtr->resetVisual = true;
  this->dataPtr->visualDirty = false;

  const auto typeIt =
      this->dataPtr->topicTypes.find(this->dataPtr->topicName);
  if (typeIt == this->dataPtr->topicTypes.end())
  {
    gzerr << "Unknown message type for topic ["
          << this->dataPtr->topicName << "]" << std::endl;
    return;
  }

  bool subscribed = false;
  if (typeIt->second == "gz.msgs.LogicalCameraSensor")
  {
    subscribed = this->dataPtr->node.Subscribe(
        this->dataPtr->topicName,
        &VisualizeFrustum::OnScan,
        this);
  }
  else if (typeIt->second == "gz.msgs.CameraInfo")
  {
    subscribed = this->dataPtr->node.Subscribe(
        this->dataPtr->topicName,
        &VisualizeFrustum::OnCameraInfo,
        this);
  }

  if (!subscribed)
  {
    gzerr << "Unable to subscribe to topic ["
          << this->dataPtr->topicName << "]" << std::endl;
  }
}

//////////////////////////////////////////////////
void VisualizeFrustum::DisplayVisual(bool _value)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  if (this->dataPtr->frustum)
  {
    this->dataPtr->frustum->SetVisible(_value);
    gzdbg << "Frustum Visual Display " << (_value ? "ON." : "OFF.")
          << std::endl;
  }
}

/////////////////////////////////////////////////
void VisualizeFrustum::OnRefresh()
{
  this->dataPtr->topicList.clear();
  this->dataPtr->topicTypes.clear();

  std::vector<std::string> allTopics;
  this->dataPtr->node.TopicList(allTopics);
  for (const auto &topic : allTopics)
  {
    std::vector<transport::MessagePublisher> publishers;
    std::vector<transport::MessagePublisher> subscribers;
    this->dataPtr->node.TopicInfo(topic, publishers, subscribers);
    for (const auto &pub : publishers)
    {
      const auto &msgType = pub.MsgTypeName();
      if (msgType == "gz.msgs.LogicalCameraSensor" ||
          msgType == "gz.msgs.CameraInfo")
      {
        this->dataPtr->topicList.push_back(QString::fromStdString(topic));
        this->dataPtr->topicTypes[topic] = msgType;
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
QStringList VisualizeFrustum::TopicList() const
{
  return this->dataPtr->topicList;
}

/////////////////////////////////////////////////
void VisualizeFrustum::SetTopicList(const QStringList &_topicList)
{
  this->dataPtr->topicList = _topicList;
  this->TopicListChanged();
}

//////////////////////////////////////////////////
void VisualizeFrustum::OnScan(const msgs::LogicalCameraSensor &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);

  FrustumData data;
  if (!this->FrustumDataFromLogicalCamera(_msg, data))
  {
    gzwarn << "Unable to compute frustum from LogicalCameraSensor message."
           << std::endl;
    return;
  }

  this->ApplyFrustumData(data);
}

//////////////////////////////////////////////////
void VisualizeFrustum::OnCameraInfo(const msgs::CameraInfo &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);

  FrustumData data;
  if (!this->FrustumDataFromCameraInfo(_msg, data))
  {
    gzwarn << "Unable to compute frustum from CameraInfo message."
           << std::endl;
    return;
  }

  this->ApplyFrustumData(data);
}

// Register this plugin
GZ_ADD_PLUGIN(gz::sim::VisualizeFrustum,
              gz::gui::Plugin)