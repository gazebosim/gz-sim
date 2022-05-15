/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#define _LIBCPP_ENABLE_THREAD_SAFETY_ANNOTATIONS

#include "GlobalIlluminationVct.hh"

#include <string>
#include <utility>
#include <vector>

#include <sdf/Link.hh>
#include <sdf/Model.hh>

#include <ignition/common/Console.hh>
#include <ignition/common/Profiler.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <ignition/transport/Node.hh>

#include <ignition/gui/Application.hh>
#include <ignition/gui/Conversions.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/gui/MainWindow.hh>

#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/rendering/RenderUtil.hh"

#include "ignition/rendering/GlobalIlluminationVct.hh"
#include "ignition/rendering/LidarVisual.hh"
#include "ignition/rendering/RenderEngine.hh"
#include "ignition/rendering/RenderTypes.hh"
#include "ignition/rendering/RenderingIface.hh"
#include "ignition/rendering/Scene.hh"

#include "ignition/gazebo/Util.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Sensor.hh"

#include "ignition/msgs/laserscan.pb.h"

#if defined(__clang__)
#  define THREAD_ANNOTATION_ATTRIBUTE__(x) __attribute__((x))
#else
#  define THREAD_ANNOTATION_ATTRIBUTE__(x)  // no-op
#endif
#define GUARDED_BY(x) THREAD_ANNOTATION_ATTRIBUTE__(guarded_by(x))
#define PT_GUARDED_BY(x) THREAD_ANNOTATION_ATTRIBUTE__(pt_guarded_by(x))
#define REQUIRES(...) \
  THREAD_ANNOTATION_ATTRIBUTE__(requires_capability(__VA_ARGS__))

// clang-format off
namespace ignition
{
namespace gazebo
{
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE
{
  /// \brief Private data class for GlobalIlluminationVct
  class GlobalIlluminationVctPrivate
  {
    /// \brief Transport node
    public: transport::Node node;

    /// \brief Scene Pointer
    public: rendering::ScenePtr scene;

    /// \brief Pointer to GlobalIlluminationVct
    public: rendering::GlobalIlluminationVctPtr gi GUARDED_BY(serviceMutex);

    /// \brief TBD
    public: bool enabled GUARDED_BY(serviceMutex){false};

    /// \brief See rendering::Ogre2GlobalIlluminationVct::SetResolution
    public: uint32_t resolution[3] GUARDED_BY(serviceMutex){16u, 16u, 16u};

#ifdef VCT_DISABLED
    /// \brief URI sequence to the lidar link
    public: std::string lidarString{""};

    /// \brief Pose of the lidar visual
    public: math::Pose3d lidarPose{math::Pose3d::Zero};

    /// \brief Topic name to subscribe
    public: std::string topicName{""};
#endif

#ifdef VCT_DISABLED
    /// \brief Entity representing the sensor in the world
    public: gazebo::Entity lidarEntity;
#endif

    /// \brief Mutex for variable mutated by the checkbox and spinboxes
    /// callbacks.
    /// The variables are: msg, minVisualRange and
    /// maxVisualRange
    public: std::mutex serviceMutex;

    /// \brief Initialization flag
    public: bool initialized{false};

    /// \brief Reset visual flag
    public: bool resetVisual{false};

    /// \brief lidar visual display dirty flag
    public: bool visualDirty GUARDED_BY(serviceMutex){false};

    /// \brief lidar sensor entity dirty flag
    public: bool lidarEntityDirty{true};
  };
}
}
}
// clang-format on

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
GlobalIlluminationVct::GlobalIlluminationVct() :
  GuiSystem(),
  dataPtr(new GlobalIlluminationVctPrivate)
{
  // no ops
}

/////////////////////////////////////////////////
GlobalIlluminationVct::~GlobalIlluminationVct()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->gi.reset();
}

/////////////////////////////////////////////////
void GlobalIlluminationVct::LoadGlobalIlluminationVct()
  REQUIRES(this->dataPtr->serviceMutex)
{
  auto loadedEngNames = rendering::loadedEngines();
  if (loadedEngNames.empty())
  {
    return;
  }

  // assume there is only one engine loaded
  auto engineName = loadedEngNames[0];
  if (loadedEngNames.size() > 1)
  {
    igndbg << "More than one engine is available. "
           << "GlobalIlluminationVct plugin will use engine [" << engineName
           << "]" << std::endl;
  }
  auto engine = rendering::engine(engineName);
  if (!engine)
  {
    ignerr << "Internal error: failed to load engine [" << engineName
           << "]. GlobalIlluminationVct plugin won't work." << std::endl;
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
  igndbg << "Creating GlobalIlluminationVct" << std::endl;

  auto root = scene->RootVisual();
  this->dataPtr->gi = scene->CreateGlobalIlluminationVct();
  if (!this->dataPtr->gi)
  {
    ignwarn << "Failed to create GlobalIlluminationVct, GI plugin won't work."
            << std::endl;

    ignition::gui::App()
      ->findChild<ignition::gui::MainWindow *>()
      ->removeEventFilter(this);
  }
  else
  {
    this->dataPtr->gi->SetParticipatingVisuals(
      rendering::GlobalIlluminationBase::DYNAMIC_VISUALS |
      rendering::GlobalIlluminationBase::STATIC_VISUALS);
    this->dataPtr->scene = scene;
    this->dataPtr->initialized = true;
  }
}

/////////////////////////////////////////////////
void GlobalIlluminationVct::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Global Illumination (VCT)";

  ignition::gui::App()
    ->findChild<ignition::gui::MainWindow *>()
    ->installEventFilter(this);
}

/////////////////////////////////////////////////
bool GlobalIlluminationVct::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == ignition::gui::events::Render::kType)
  {
    // This event is called in Scene3d's RenderThread, so it's safe to make
    // rendering calls here

    std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
    if (!this->dataPtr->initialized)
    {
      this->LoadGlobalIlluminationVct();
    }

    if (this->dataPtr->gi)
    {
      if (this->dataPtr->resetVisual)
      {
#ifdef VCT_DISABLED
        this->dataPtr->lidar->ClearPoints();
#endif
        this->dataPtr->resetVisual = false;
      }
      if (this->dataPtr->visualDirty)
      {
        if (this->dataPtr->enabled)
        {
          this->dataPtr->scene->SetActiveGlobalIllumination(this->dataPtr->gi);
        }
        else
        {
          this->dataPtr->scene->SetActiveGlobalIllumination(nullptr);
        }
#ifdef VCT_DISABLED
        this->dataPtr->lidar->SetWorldPose(this->dataPtr->lidarPose);
        this->dataPtr->lidar->Update();
#endif
        this->dataPtr->visualDirty = false;
      }
    }
    else
    {
      ignerr << "GI pointer is not set" << std::endl;
    }
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

//////////////////////////////////////////////////
void GlobalIlluminationVct::Update(const UpdateInfo &,
                                   EntityComponentManager &_ecm)
{
  IGN_PROFILE("GlobalIlluminationVct::Update");

  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);

  if (this->dataPtr->lidarEntityDirty)
  {
#ifdef VCT_DISABLED
    auto lidarURIVec = common::split(common::trimmed(this->dataPtr->gi), "::");
    if (lidarURIVec.size() > 0)
    {
      auto baseEntity =
        _ecm.EntityByComponents(components::Name(lidarURIVec[0]));
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
        for (size_t i = 0u; i < lidarURIVec.size() - 1; i++)
        {
          auto children =
            _ecm.EntitiesByComponents(components::ParentEntity(parent));
          bool foundChild = false;
          for (auto child : children)
          {
            std::string nextstring = lidarURIVec[i + 1];
            auto comp = _ecm.Component<components::Name>(child);
            if (!comp)
            {
              continue;
            }
            std::string childname = std::string(comp->Data());
            if (nextstring.compare(childname) == 0)
            {
              parent = child;
              foundChild = true;
              if (i + 1 == lidarURIVec.size() - 1)
              {
                success = true;
              }
              break;
            }
          }
          if (!foundChild)
          {
            ignerr << "The entity could not be found."
                   << "Error displaying lidar visual" << std::endl;
            return;
          }
        }
        if (success)
        {
#  ifdef VCT_DISABLED
          this->dataPtr->lidarEntity = parent;
#  endif
          this->dataPtr->lidarEntityDirty = false;
        }
      }
    }
#endif
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
#ifdef VCT_DISABLED
    this->dataPtr->lidarPose = worldPose(this->dataPtr->lidarEntity, _ecm);
#endif
  }
}

//////////////////////////////////////////////////
void GlobalIlluminationVct::UpdateDebugVisualizationMode(int _mode)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);

  rendering::GlobalIlluminationVct::DebugVisualizationMode
    debugVisualizationMode = rendering::GlobalIlluminationVct::DVM_None;

  if (_mode >= rendering::GlobalIlluminationVct::DVM_Albedo &&
      _mode <= rendering::GlobalIlluminationVct::DVM_None)
  {
    debugVisualizationMode =
      static_cast<rendering::GlobalIlluminationVct::DebugVisualizationMode>(
        _mode);
  }

  this->dataPtr->gi->SetDebugVisualization(debugVisualizationMode);
}

//////////////////////////////////////////////////
void GlobalIlluminationVct::UpdateResolution(int _axis, uint32_t _res)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->resolution[_axis] = _res;
  this->dataPtr->visualDirty = true;
}

//////////////////////////////////////////////////
void GlobalIlluminationVct::OnTopic(const QString &_topicName)
{
#ifdef VCT_DISABLED
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  if (!this->dataPtr->topicName.empty() &&
      !this->dataPtr->node.Unsubscribe(this->dataPtr->topicName))
  {
    ignerr << "Unable to unsubscribe from topic [" << this->dataPtr->topicName
           << "]" << std::endl;
  }
  this->dataPtr->topicName = _topicName.toStdString();

  // Reset visualization
  this->dataPtr->resetVisual = true;

  // Create new subscription
  if (!this->dataPtr->node.Subscribe(this->dataPtr->topicName,
                                     &GlobalIlluminationVct::OnScan, this))
  {
    ignerr << "Unable to subscribe to topic [" << this->dataPtr->topicName
           << "]\n";
    return;
  }
  this->dataPtr->visualDirty = false;
  ignmsg << "Subscribed to " << this->dataPtr->topicName << std::endl;
#endif
}

//////////////////////////////////////////////////
void GlobalIlluminationVct::OnScan(const msgs::LaserScan &_msg)
{
#ifdef VCT_DISABLED
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  if (this->dataPtr->initialized)
  {
    this->dataPtr->msg = std::move(_msg);
    this->dataPtr->lidar->SetVerticalRayCount(
      this->dataPtr->msg.vertical_count());
    this->dataPtr->lidar->SetHorizontalRayCount(this->dataPtr->msg.count());
    this->dataPtr->lidar->SetMinHorizontalAngle(this->dataPtr->msg.angle_min());
    this->dataPtr->lidar->SetMaxHorizontalAngle(this->dataPtr->msg.angle_max());
    this->dataPtr->lidar->SetMinVerticalAngle(
      this->dataPtr->msg.vertical_angle_min());
    this->dataPtr->lidar->SetMaxVerticalAngle(
      this->dataPtr->msg.vertical_angle_max());

    this->dataPtr->lidar->SetPoints(std::vector<double>(
      this->dataPtr->msg.ranges().begin(), this->dataPtr->msg.ranges().end()));

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
#endif
}

//////////////////////////////////////////////////
void GlobalIlluminationVct::SetEnabled(const bool _enabled)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->enabled = _enabled;
  this->dataPtr->visualDirty = true;
  /*if (_enabled)
  {
    this->dataPtr->scene->SetActiveGlobalIllumination(this->dataPtr->gi);
  }
  else
  {
    this->dataPtr->scene->SetActiveGlobalIllumination(nullptr);
  }*/
}

//////////////////////////////////////////////////
bool GlobalIlluminationVct::Enabled() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  return this->dataPtr->enabled;
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::GlobalIlluminationVct,
                    ignition::gui::Plugin)
