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

#include "VisualizeFrustum.hh"

#include <string>
#include <utility>
#include <vector>

#include <sdf/Link.hh>
#include <sdf/Model.hh>

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
#include "gz/rendering/FrustumVisual.hh"

#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Pose.hh"
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
    public: std::string frustumString{""};

    /// \brief LaserScan message from sensor
    public: msgs::LogicalCameraSensor msg;

    /// \brief Pose of the frustum visual
    public: math::Pose3d frustumPose{math::Pose3d::Zero};

    /// \brief Topic name to subscribe
    public: std::string topicName{""};

    /// \brief List of topics publishing LaserScan messages.
    public: QStringList topicList;

    /// \brief Entity representing the sensor in the world
    public: sim::Entity frustumEntity;

    /// \brief Mutex for variable mutated by the checkbox and spinboxes
    /// callbacks.
    /// The variables are: msg
    public: std::mutex serviceMutex;

    /// \brief Initialization flag
    public: bool initialized{false};

    /// \brief Reset visual flag
    public: bool resetVisual{false};

    /// \brief frustum visual display dirty flag
    public: bool visualDirty{false};

    /// \brief frustum sensor entity dirty flag
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
  // no ops
}

/////////////////////////////////////////////////
VisualizeFrustum::~VisualizeFrustum()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->scene->DestroyVisual(this->dataPtr->frustum);
}

/////////////////////////////////////////////////
void VisualizeFrustum::LoadFrustum()
{
  auto loadedEngNames = rendering::loadedEngines();
  if (loadedEngNames.empty())
    return;

  // assume there is only one engine loaded
  auto engineName = loadedEngNames[0];
  if (loadedEngNames.size() > 1)
  {
    gzdbg << "More than one engine is available. "
          << "VisualizeFrustum plugin will use engine ["
          << engineName << "]" << std::endl;
  }
  auto engine = rendering::engine(engineName);
  if (!engine)
  {
    gzerr << "Internal error: failed to load engine [" << engineName
          << "]. VisualizeFrustum plugin won't work." << std::endl;
    return;
  }

  if (engine->SceneCount() == 0)
    return;

  // assume there is only one scene
  // load scene
  auto scene = engine->SceneByIndex(0);
  if (!scene)
  {
    gzerr << "Internal error: scene is null." << std::endl;
    return;
  }

  if (!scene->IsInitialized() || scene->VisualCount() == 0)
  {
    return;
  }

  // Create frustum visual
  gzdbg << "Creating frustum visual" << std::endl;
  auto root = scene->RootVisual();

  this->dataPtr->frustum = scene->CreateFrustumVisual();
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
  GZ_PROFILE("VisualizeFrusum::Update");

  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);

  if (this->dataPtr->frustumEntityDirty)
  {
    auto frustumURIVec = common::split(common::trimmed(
                  this->dataPtr->frustumString), "::");
    if (frustumURIVec.size() > 0)
    {
      auto baseEntity = _ecm.EntityByComponents(
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
        for (size_t i = 0u; i < frustumURIVec.size()-1; i++)
        {
          auto children = _ecm.EntitiesByComponents(
                            components::ParentEntity(parent));
          bool foundChild = false;
          for (auto child : children)
          {
            std::string nextstring = frustumURIVec[i+1];
            auto comp = _ecm.Component<components::Name>(child);
            if (!comp)
            {
              continue;
            }
            std::string childname = std::string(
                            comp->Data());
            if (nextstring.compare(childname) == 0)
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
                  << "Error displaying frustum visual" <<std::endl;
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
  // If we update the worldpose on the physics thread **after** the sensor
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
          << this->dataPtr->topicName <<"]" <<std::endl;
  }

  this->dataPtr->topicName = _topicName.toStdString();
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);

  // Reset visualization
  this->dataPtr->resetVisual = true;

  // Create new subscription
  if (!this->dataPtr->node.Subscribe(this->dataPtr->topicName,
                            &VisualizeFrustum::OnScan, this))
  {
    gzerr << "Unable to subscribe to topic ["
          << this->dataPtr->topicName << "]\n";
    return;
  }
  this->dataPtr->visualDirty = false;
}

//////////////////////////////////////////////////
void VisualizeFrustum::DisplayVisual(bool _value)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->frustum->SetVisible(_value);
  gzerr << "Frustum Visual Display " << ((_value) ? "ON." : "OFF.") << std::endl;
}

/////////////////////////////////////////////////
void VisualizeFrustum::OnRefresh()
{
  gzmsg << "Refreshing topic list for Logical Camera Sensor messages." << std::endl;

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
      if (pub.MsgTypeName() == "gz.msgs.LogicalCameraSensor")
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
  if (this->dataPtr->initialized)
  {
    this->dataPtr->msg = std::move(_msg);

    this->dataPtr->frustum->SetNear(this->dataPtr->msg.near_clip());
    this->dataPtr->frustum->SetFar(this->dataPtr->msg.far_clip());
    this->dataPtr->frustum->SetFOV(this->dataPtr->msg.horizontal_fov());
    this->dataPtr->frustum->SetAspectRatio(this->dataPtr->msg.aspect_ratio());

    this->dataPtr->visualDirty = true;

    for (auto data_values : this->dataPtr->msg.header().data())
    {
      if (data_values.key() == "frame_id")
      {
        if (this->dataPtr->frustumString.compare(
                common::trimmed(data_values.value(0))) != 0)
        {
          this->dataPtr->frustumString = common::trimmed(data_values.value(0));
          this->dataPtr->frustumEntityDirty = true;
          break;
        }
      }
    }
  }
}

// Register this plugin
GZ_ADD_PLUGIN(gz::sim::VisualizeFrustum,
                    gz::gui::Plugin)
