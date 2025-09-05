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

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>

#include <gz/plugin/Register.hh>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <gz/transport/Node.hh>

#include <gz/gui/Application.hh>
#include <gz/gui/Conversions.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>

#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/rendering/RenderUtil.hh"

#include "gz/rendering/GlobalIlluminationVct.hh"
#include "gz/rendering/LidarVisual.hh"
#include "gz/rendering/RenderEngine.hh"
#include "gz/rendering/RenderTypes.hh"
#include "gz/rendering/RenderingIface.hh"
#include "gz/rendering/Scene.hh"

#include "gz/sim/Util.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Sensor.hh"

#include "gz/msgs/laserscan.pb.h"

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
namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE
{
  /// \brief Private data class for GlobalIlluminationVct
  class GZ_SIM_HIDDEN GlobalIlluminationVctPrivate
  {
    /// \brief Transport node
    public: transport::Node node;

    /// \brief Scene Pointer
    public: rendering::ScenePtr scene;

    /// \brief Pointer to GlobalIlluminationVct
    public: rendering::GlobalIlluminationVctPtr gi GUARDED_BY(serviceMutex);

    /// \brief Toggles this GI on/off. Only one can be active at the same time.
    public: bool enabled GUARDED_BY(serviceMutex){false};

    /// \brief See rendering::GlobalIlluminationVct::SetResolution
    public: uint32_t resolution[3] GUARDED_BY(serviceMutex){16u, 16u, 16u};

    /// \brief See rendering::GlobalIlluminationVct::SetOctantCount
    public: uint32_t octantCount[3] GUARDED_BY(serviceMutex){1u, 1u, 1u};

    /// \brief See rendering::GlobalIlluminationVct::SetBounceCount
    public: uint32_t bounceCount GUARDED_BY(serviceMutex){6u};

    /// \brief See rendering::GlobalIlluminationVct::SetHighQuality
    public: bool highQuality GUARDED_BY(serviceMutex){true};

    /// \brief See rendering::GlobalIlluminationVct::SetAnisotropic
    public: bool anisotropic GUARDED_BY(serviceMutex){true};

    /// \brief See rendering::GlobalIlluminationVct::SetConserveMemory
    public: bool conserveMemory GUARDED_BY(serviceMutex){false};

    /// \brief See rendering::GlobalIlluminationVct::DebugVisualizationMode
    public: float thinWallCounter GUARDED_BY(serviceMutex){ 1.0f };

    /// \brief See rendering::GlobalIlluminationVct::DebugVisualizationMode
    public: uint32_t debugVisMode GUARDED_BY(
      serviceMutex){ rendering::GlobalIlluminationVct::DVM_None };

    /// \brief Mutex for variable mutated by the checkbox and spinboxes
    /// callbacks.
    /// The variables are: msg, minVisualRange and
    /// maxVisualRange
    public: std::mutex serviceMutex;

    /// \brief Initialization flag
    public: bool initialized{false};

    /// \brief Reset visual flag
    public: bool resetVisual{false};

    /// \brief GI visual display dirty flag
    public: bool visualDirty GUARDED_BY(serviceMutex){false};

    /// \brief GI visual display dirty flag; but it is fast/quick to rebuild
    public: bool lightingDirty GUARDED_BY(serviceMutex){false};

    /// \brief GI debug visualization is dirty. Only used by GUI.
    /// Not in simulation.
    public: bool debugVisualizationDirty GUARDED_BY(serviceMutex){false};
  };
}
}
}
// clang-format on

using namespace gz;
using namespace sim;

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
    gzdbg << "More than one engine is available. "
          << "GlobalIlluminationVct plugin will use engine [" << engineName
          << "]" << std::endl;
  }
  auto engine = rendering::engine(engineName);
  if (!engine)
  {
    gzerr << "Internal error: failed to load engine [" << engineName
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
    gzerr << "Internal error: scene is null." << std::endl;
    return;
  }

  if (!scene->IsInitialized() || scene->VisualCount() == 0)
  {
    return;
  }

  // Create lidar visual
  gzdbg << "Creating GlobalIlluminationVct" << std::endl;

  auto root = scene->RootVisual();
  this->dataPtr->gi = scene->CreateGlobalIlluminationVct();
  if (!this->dataPtr->gi)
  {
    gzwarn << "Failed to create GlobalIlluminationVct, GI plugin won't work."
           << std::endl;

    gz::gui::App()->findChild<gz::gui::MainWindow *>()->removeEventFilter(this);
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

/// \brief XML helper to retrieve values and handle errors
/// \param[in] _elem XML element to read
/// \param[out] _valueToSet Value to set. Left unmodified on error
/// \return True if _valueToSet was successfully set
static bool GetXmlBool(const tinyxml2::XMLElement *_elem, bool &_valueToSet)
{
  bool value = false;

  if (_elem->QueryBoolText(&value) != tinyxml2::XML_SUCCESS)
  {
    gzerr << "Failed to parse <" << _elem->Name()
          << "> value: " << _elem->GetText() << std::endl;
    return false;
  }
  else
  {
    _valueToSet = value;
    return true;
  }
}

/// \brief XML helper to retrieve values and handle errors
/// \param[in] _elem XML element to read
/// \param[out] _valueToSet Value to set. Left unmodified on error
/// \return True if _valueToSet was successfully set
static bool GetXmlFloat(const tinyxml2::XMLElement *_elem, float &_valueToSet)
{
  float value = 0;

  if (_elem->QueryFloatText(&value) != tinyxml2::XML_SUCCESS)
  {
    gzerr << "Failed to parse <" << _elem->Name()
          << "> value: " << _elem->GetText() << std::endl;
    return false;
  }
  else
  {
    _valueToSet = value;
    return true;
  }
}

/// \brief XML helper to retrieve values and handle errors
/// \param[in] _elem XML element to read
/// \param[out] _valueToSet Value to set. Left unmodified on error
/// \return True if _valueToSet was successfully set
static bool GetXmlUint32(const tinyxml2::XMLElement *_elem,
                         uint32_t &_valueToSet)
{
  int value = 0;

  if (_elem->QueryIntText(&value) != tinyxml2::XML_SUCCESS)
  {
    gzerr << "Failed to parse <" << _elem->Name()
          << "> value: " << _elem->GetText() << std::endl;
    return false;
  }
  else
  {
    _valueToSet = static_cast<uint32_t>(value);
    return true;
  }
}

/// \brief XML helper to retrieve values and handle errors
/// \param[in] _elem XML element to read
/// \param[out] _valueToSet Values to set. Left unmodified on error.
/// Its array length must be >= 3
/// \return True if _valueToSet was successfully set
static bool GetXmlUint32x3(const tinyxml2::XMLElement *_elem,
                           uint32_t _valueToSet[3])
{
  std::istringstream stream(_elem->GetText());
  math::Vector3i values3;
  stream >> values3;

  _valueToSet[0] = static_cast<uint32_t>(values3.X());
  _valueToSet[1] = static_cast<uint32_t>(values3.Y());
  _valueToSet[2] = static_cast<uint32_t>(values3.Z());

  return true;
}

/////////////////////////////////////////////////
void GlobalIlluminationVct::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
    this->title = "Global Illumination (VCT)";

  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);

  if (auto elem = _pluginElem->FirstChildElement("enabled"))
  {
    GetXmlBool(elem, this->dataPtr->enabled);
  }
  if (auto elem = _pluginElem->FirstChildElement("highQuality"))
  {
    GetXmlBool(elem, this->dataPtr->highQuality);
  }
  if (auto elem = _pluginElem->FirstChildElement("anisotropic"))
  {
    GetXmlBool(elem, this->dataPtr->anisotropic);
  }
  if (auto elem = _pluginElem->FirstChildElement("conserveMemory"))
  {
    GetXmlBool(elem, this->dataPtr->conserveMemory);
  }
  if (auto elem = _pluginElem->FirstChildElement("resolution"))
  {
    GetXmlUint32x3(elem, this->dataPtr->resolution);
  }
  if (auto elem = _pluginElem->FirstChildElement("octantCount"))
  {
    GetXmlUint32x3(elem, this->dataPtr->octantCount);
  }
  if (auto elem = _pluginElem->FirstChildElement("bounceCount"))
  {
    GetXmlUint32(elem, this->dataPtr->bounceCount);
  }
  if (auto elem = _pluginElem->FirstChildElement("thinWallCounter"))
  {
    GetXmlFloat(elem, this->dataPtr->thinWallCounter);
  }
  if (auto elem = _pluginElem->FirstChildElement("debugVisMode"))
  {
    const std::string text = elem->GetText();
    if (text == "none")
    {
      this->dataPtr->debugVisMode = rendering::GlobalIlluminationVct::DVM_None;
    }
    else if (text == "albedo")
    {
      this->dataPtr->debugVisMode =
        rendering::GlobalIlluminationVct::DVM_Albedo;
    }
    else if (text == "normal")
    {
      this->dataPtr->debugVisMode =
        rendering::GlobalIlluminationVct::DVM_Normal;
    }
    else if (text == "emissive")
    {
      this->dataPtr->debugVisMode =
        rendering::GlobalIlluminationVct::DVM_Emissive;
    }
    else if (text == "lighting")
    {
      this->dataPtr->debugVisMode =
        rendering::GlobalIlluminationVct::DVM_Lighting;
    }
    else
    {
      GetXmlUint32(elem, this->dataPtr->debugVisMode);
    }
  }

  gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(this);
}

/////////////////////////////////////////////////
bool GlobalIlluminationVct::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gz::gui::events::Render::kType)
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
        this->dataPtr->resetVisual = false;
      }

      if (!this->dataPtr->visualDirty && !this->dataPtr->gi->Enabled() &&
          this->dataPtr->enabled)
      {
        // If we're here, GI was disabled externally. This can happen
        // if e.g. another GI solution was enabled (only one can be active)
        this->dataPtr->enabled = false;
        this->EnabledChanged();
      }

      if (this->dataPtr->visualDirty)
      {
        this->dataPtr->gi->SetResolution(this->dataPtr->resolution);
        this->dataPtr->gi->SetOctantCount(this->dataPtr->octantCount);
        this->dataPtr->gi->SetBounceCount(this->dataPtr->bounceCount);
        this->dataPtr->gi->SetHighQuality(this->dataPtr->highQuality);
        this->dataPtr->gi->SetAnisotropic(this->dataPtr->anisotropic);
        this->dataPtr->gi->SetThinWallCounter(this->dataPtr->thinWallCounter);
        this->dataPtr->gi->SetConserveMemory(this->dataPtr->conserveMemory);

        // Ogre-Next may crash if some of the settings above are
        // changed while visualizing is enabled.
        this->dataPtr->gi->SetDebugVisualization(
          rendering::GlobalIlluminationVct::DVM_None);

        if (this->dataPtr->enabled)
        {
          this->dataPtr->gi->Build();
          this->dataPtr->scene->SetActiveGlobalIllumination(this->dataPtr->gi);
        }
        else
        {
          this->dataPtr->scene->SetActiveGlobalIllumination(nullptr);
        }

        // Restore debug visualization to desired.
        this->dataPtr->gi->SetDebugVisualization(
          static_cast<rendering::GlobalIlluminationVct::DebugVisualizationMode>(
            this->dataPtr->debugVisMode));

        this->dataPtr->visualDirty = false;
        this->dataPtr->lightingDirty = false;
        this->dataPtr->debugVisualizationDirty = false;
      }
      else if (this->dataPtr->lightingDirty)
      {
        this->dataPtr->gi->SetBounceCount(this->dataPtr->bounceCount);
        this->dataPtr->gi->SetHighQuality(this->dataPtr->highQuality);
        this->dataPtr->gi->SetAnisotropic(this->dataPtr->anisotropic);
        this->dataPtr->gi->SetThinWallCounter(this->dataPtr->thinWallCounter);
        this->dataPtr->gi->SetConserveMemory(this->dataPtr->conserveMemory);

        if (this->dataPtr->gi->Enabled())
        {
          this->dataPtr->gi->SetDebugVisualization(
            rendering::GlobalIlluminationVct::DVM_None);

          this->dataPtr->gi->LightingChanged();

          this->dataPtr->gi->SetDebugVisualization(
            static_cast<
              rendering::GlobalIlluminationVct::DebugVisualizationMode>(
              this->dataPtr->debugVisMode));

          this->dataPtr->debugVisualizationDirty = false;
        }
        this->dataPtr->lightingDirty = false;
      }
      else if (this->dataPtr->debugVisualizationDirty)
      {
        if (this->dataPtr->enabled && this->dataPtr->gi->Enabled())
        {
          this->dataPtr->gi->SetDebugVisualization(
          static_cast<rendering::GlobalIlluminationVct::DebugVisualizationMode>(
          this->dataPtr->debugVisMode));
        }
        else
        {
          gzerr << "Trying to set debug visualization mode while GI is "
                << "disabled. Please enable GI first."
                << std::endl;
          // Always set to none when disabled to avoid crash
          this->dataPtr->gi->SetDebugVisualization(
            rendering::GlobalIlluminationVct::DVM_None);
        }
        this->dataPtr->debugVisualizationDirty = false;
      }
    }
    else
    {
      gzerr << "GI pointer is not set" << std::endl;
    }
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
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
void GlobalIlluminationVct::UpdateOctantCount(int _axis, uint32_t _count)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->octantCount[_axis] = _count;
  this->dataPtr->visualDirty = true;
}

//////////////////////////////////////////////////
void GlobalIlluminationVct::SetEnabled(const bool _enabled)
{
  bool needEmitDebugVisChanged = false;
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
    if (this->dataPtr->enabled && !_enabled)
    {
      // When disabling GI, force debugVisMode to None for safety
      this->dataPtr->debugVisMode = rendering::GlobalIlluminationVct::DVM_None;
      this->dataPtr->debugVisualizationDirty = true;
      needEmitDebugVisChanged = true;
    }
    this->dataPtr->enabled = _enabled;
    this->dataPtr->visualDirty = true;
  }
  if (needEmitDebugVisChanged)
  {
    this->DebugVisualizationModeChanged();
  }
}

//////////////////////////////////////////////////
bool GlobalIlluminationVct::Enabled() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  return this->dataPtr->enabled;
}

//////////////////////////////////////////////////
void GlobalIlluminationVct::SetResolutionX(const uint32_t _res)
{
  this->UpdateResolution(0, _res);
}

//////////////////////////////////////////////////
uint32_t GlobalIlluminationVct::ResolutionX() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  return this->dataPtr->resolution[0];
}

//////////////////////////////////////////////////
void GlobalIlluminationVct::SetResolutionY(const uint32_t _res)
{
  this->UpdateResolution(1, _res);
}

//////////////////////////////////////////////////
uint32_t GlobalIlluminationVct::ResolutionY() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  return this->dataPtr->resolution[1];
}

//////////////////////////////////////////////////
void GlobalIlluminationVct::SetResolutionZ(const uint32_t _res)
{
  this->UpdateResolution(2, _res);
}

//////////////////////////////////////////////////
uint32_t GlobalIlluminationVct::ResolutionZ() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  return this->dataPtr->resolution[2];
}

//////////////////////////////////////////////////
void GlobalIlluminationVct::SetOctantCountX(const uint32_t _octantCount)
{
  this->UpdateOctantCount(0, _octantCount);
}

//////////////////////////////////////////////////
uint32_t GlobalIlluminationVct::OctantCountX() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  return this->dataPtr->octantCount[0];
}

//////////////////////////////////////////////////
void GlobalIlluminationVct::SetOctantCountY(const uint32_t _octantCount)
{
  this->UpdateOctantCount(1, _octantCount);
}

//////////////////////////////////////////////////
uint32_t GlobalIlluminationVct::OctantCountY() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  return this->dataPtr->octantCount[1];
}

//////////////////////////////////////////////////
void GlobalIlluminationVct::SetOctantCountZ(const uint32_t _octantCount)
{
  this->UpdateOctantCount(2, _octantCount);
}

//////////////////////////////////////////////////
uint32_t GlobalIlluminationVct::OctantCountZ() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  return this->dataPtr->octantCount[2];
}

//////////////////////////////////////////////////
void GlobalIlluminationVct::SetBounceCount(const uint32_t _bounces)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->bounceCount = _bounces;
  this->dataPtr->lightingDirty = true;
}

//////////////////////////////////////////////////
uint32_t GlobalIlluminationVct::BounceCount() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  return this->dataPtr->bounceCount;
}

//////////////////////////////////////////////////
void GlobalIlluminationVct::SetHighQuality(const bool _quality)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->highQuality = _quality;
  this->dataPtr->lightingDirty = true;
}

//////////////////////////////////////////////////
bool GlobalIlluminationVct::HighQuality() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  return this->dataPtr->highQuality;
}

//////////////////////////////////////////////////
void GlobalIlluminationVct::SetAnisotropic(const bool _anisotropic)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->anisotropic = _anisotropic;
  this->dataPtr->lightingDirty = true;
}

//////////////////////////////////////////////////
bool GlobalIlluminationVct::Anisotropic() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  return this->dataPtr->anisotropic;
}

//////////////////////////////////////////////////
void GlobalIlluminationVct::SetConserveMemory(const bool _conserveMemory)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->conserveMemory = _conserveMemory;
  this->dataPtr->lightingDirty = true;
}

//////////////////////////////////////////////////
bool GlobalIlluminationVct::ConserveMemory() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  return this->dataPtr->conserveMemory;
}

//////////////////////////////////////////////////
void GlobalIlluminationVct::SetThinWallCounter(const float _thinWallCounter)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->thinWallCounter = _thinWallCounter;
  this->dataPtr->lightingDirty = true;
}

//////////////////////////////////////////////////
float GlobalIlluminationVct::ThinWallCounter() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  return this->dataPtr->thinWallCounter;
}

//////////////////////////////////////////////////
void GlobalIlluminationVct::SetDebugVisualizationMode(const uint32_t _visMode)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->debugVisMode = _visMode;
  this->dataPtr->debugVisualizationDirty = true;
}

//////////////////////////////////////////////////
uint32_t GlobalIlluminationVct::DebugVisualizationMode() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  return this->dataPtr->debugVisMode;
}

// Register this plugin
GZ_ADD_PLUGIN(gz::sim::GlobalIlluminationVct, gz::gui::Plugin)
