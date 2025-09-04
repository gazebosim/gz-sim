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

#include "GlobalIlluminationCiVct.hh"

#include "CiVctCascadePrivate.hh"

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

#include "gz/rendering/Camera.hh"
#include "gz/rendering/GlobalIlluminationCiVct.hh"
#include "gz/rendering/RenderEngine.hh"
#include "gz/rendering/RenderTypes.hh"
#include "gz/rendering/RenderingIface.hh"
#include "gz/rendering/Scene.hh"

#include "gz/sim/Util.hh"

#include "Tsa.hh"

// clang-format off
namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE
{
  /// \brief Private data only used when loading this plugin
  /// from XML. We must do this because we require Ogre-Next
  /// engine to be loaded and there are also thread synchronization
  /// issues (GUI vs internal data) that needs to be accounted
  struct GZ_SIM_HIDDEN GiCiVctXmlInitData
  {
    uint32_t resolution[3]{ 16u, 16u, 16u };
    uint32_t octantCount[3]{ 1u, 1u, 1u };
    float areaHalfSize[3]{ 5.0f, 5.0f, 5.0f };
    float thinWallCounter{ 1.0f };
  };

  /// \brief Private data class for GlobalIlluminationCiVct
  class GZ_SIM_HIDDEN GlobalIlluminationCiVctPrivate
  {
    /// \brief Transport node
    public: transport::Node node;

    /// \brief Scene Pointer
    public: rendering::ScenePtr scene GUARDED_BY(serviceMutex);

    /// \brief Each cascade created by GI.
    /// We directly access the data in CiVctCascade from UI thread
    /// because it's safe to do so:
    ///   - Ogre2 doesn't invoke any side effect (i.e. build must be called)
    ///   - Ogre2 won't issue rendering commands (all rendering must
    ///     happen in the main thread, regardless of whether it's protected)
    public: std::vector<std::unique_ptr<CiVctCascadePrivate>> cascades
      GUARDED_BY(serviceMutex);

    /// \brief Pointer to GlobalIlluminationCiVct
    public: rendering::GlobalIlluminationCiVctPtr gi GUARDED_BY(serviceMutex);

    /// \brief Toggles this GI on/off. Only one can be active at the same time.
    public: bool enabled GUARDED_BY(serviceMutex){false};

    /// \brief See GlobalIlluminationCiVct::ResetCascades.
    public: bool resetRequested GUARDED_BY(serviceMutex){false};

    /// \brief See rendering::GlobalIlluminationCiVct::SetBounceCount
    public: uint32_t bounceCount GUARDED_BY(serviceMutex){6u};

    /// \brief See rendering::GlobalIlluminationCiVct::SetHighQuality
    public: bool highQuality GUARDED_BY(serviceMutex){true};

    /// \brief See rendering::GlobalIlluminationCiVct::SetAnisotropic
    public: bool anisotropic GUARDED_BY(serviceMutex){true};

    /// \brief See rendering::GlobalIlluminationCiVct::DebugVisualizationMode
    public: uint32_t debugVisMode GUARDED_BY(
      serviceMutex){ rendering::GlobalIlluminationCiVct::DVM_None };

    /// \brief Camera from where the CIVCT cascades are centered around
    public: rendering::CameraPtr bindCamera GUARDED_BY(serviceMutex){ nullptr };

    /// \brief Available cameras for binding
    public: QStringList availableCameras;

    /// \brief Mutex for variable mutated by the checkbox and spinboxes
    /// callbacks.
    public: std::mutex serviceMutex;

    /// \brief Initialization flag
    public: bool initialized{false};

    /// \brief GI visual display dirty flag
    public: bool visualDirty GUARDED_BY(serviceMutex){false};

    /// \brief GI visual display dirty flag; but it is fast/quick to rebuild
    public: bool lightingDirty GUARDED_BY(serviceMutex){false};

    /// \brief GI debug visualization is dirty. Only used by GUI.
    /// Not in simulation.
    public: bool debugVisualizationDirty GUARDED_BY(serviceMutex){false};

    /// \brief See GiCiVctXmlInitData. Only used during XML initialization.
    /// It's never accessed by multiple threads at the same time.
    public: std::vector<GiCiVctXmlInitData> xmlInitData;
  };
}
}
}
// clang-format on

using namespace gz;
using namespace sim;

// Q_DECLARE_METATYPE(CiVctCascadePrivate);

/////////////////////////////////////////////////
GlobalIlluminationCiVct::GlobalIlluminationCiVct() :
  GuiSystem(),
  dataPtr(new GlobalIlluminationCiVctPrivate)
{
  // no ops
}

/////////////////////////////////////////////////
GlobalIlluminationCiVct::~GlobalIlluminationCiVct()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->gi.reset();
}

/////////////////////////////////////////////////
void GlobalIlluminationCiVct::LoadGlobalIlluminationCiVct()
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
          << "GlobalIlluminationCiVct plugin will use engine [" << engineName
          << "]" << std::endl;
  }
  auto engine = rendering::engine(engineName);
  if (!engine)
  {
    gzerr << "Internal error: failed to load engine [" << engineName
          << "]. GlobalIlluminationCiVct plugin won't work." << std::endl;
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

  // Create visual
  gzdbg << "Creating GlobalIlluminationCiVct" << std::endl;

  auto root = scene->RootVisual();
  this->dataPtr->gi = scene->CreateGlobalIlluminationCiVct();
  if (!this->dataPtr->gi)
  {
    gzerr << "Failed to create GlobalIlluminationCiVct, GI plugin won't work."
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

    if (this->dataPtr->xmlInitData.empty())
    {
      // Ensure we initialize with valid settings so the user
      // can just Enable us immediately.
      emit qmlAddCascade();
    }
    else
    {
      for (const GiCiVctXmlInitData &itor : this->dataPtr->xmlInitData)
      {
        emit qmlAddCascade2(itor.resolution[0], itor.resolution[1],
                            itor.resolution[2], itor.octantCount[0],
                            itor.octantCount[1], itor.octantCount[2],
                            itor.areaHalfSize[0], itor.areaHalfSize[1],
                            itor.areaHalfSize[2], itor.thinWallCounter);
      }
    }

    this->OnRefreshCamerasImpl();
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

static bool GetXmlFloatx3(const tinyxml2::XMLElement *_elem,
                          float _valueToSet[3])
{
  std::istringstream stream(_elem->GetText());
  math::Vector3f values3;
  stream >> values3;

  _valueToSet[0] = static_cast<float>(values3.X());
  _valueToSet[1] = static_cast<float>(values3.Y());
  _valueToSet[2] = static_cast<float>(values3.Z());

  return true;
}

/////////////////////////////////////////////////
void GlobalIlluminationCiVct::LoadConfig(
  const tinyxml2::XMLElement *_pluginElem)
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
  if (auto elem = _pluginElem->FirstChildElement("bounceCount"))
  {
    GetXmlUint32(elem, this->dataPtr->bounceCount);
  }
  if (auto elem = _pluginElem->FirstChildElement("debugVisMode"))
  {
    const std::string text = elem->GetText();
    if (text == "none")
    {
      this->dataPtr->debugVisMode =
        rendering::GlobalIlluminationCiVct::DVM_None;
    }
    else if (text == "albedo")
    {
      this->dataPtr->debugVisMode =
        rendering::GlobalIlluminationCiVct::DVM_Albedo;
    }
    else if (text == "normal")
    {
      this->dataPtr->debugVisMode =
        rendering::GlobalIlluminationCiVct::DVM_Normal;
    }
    else if (text == "emissive")
    {
      this->dataPtr->debugVisMode =
        rendering::GlobalIlluminationCiVct::DVM_Emissive;
    }
    else if (text == "lighting")
    {
      this->dataPtr->debugVisMode =
        rendering::GlobalIlluminationCiVct::DVM_Lighting;
    }
    else
    {
      GetXmlUint32(elem, this->dataPtr->debugVisMode);
    }
  }

  this->dataPtr->xmlInitData.clear();

  for (auto *elemCascade = _pluginElem->FirstChildElement("cascade");
       elemCascade != nullptr;
       elemCascade = elemCascade->NextSiblingElement("cascade"))
  {
    GiCiVctXmlInitData xmlInitData;
    if (auto elem = elemCascade->FirstChildElement("resolution"))
    {
      GetXmlUint32x3(elem, xmlInitData.resolution);
    }
    if (auto elem = elemCascade->FirstChildElement("octantCount"))
    {
      GetXmlUint32x3(elem, xmlInitData.octantCount);
    }
    if (auto elem = elemCascade->FirstChildElement("thinWallCounter"))
    {
      GetXmlFloat(elem, xmlInitData.thinWallCounter);
    }
    if (auto elem = elemCascade->FirstChildElement("areaHalfSize"))
    {
      GetXmlFloatx3(elem, xmlInitData.areaHalfSize);
    }

    this->dataPtr->xmlInitData.push_back(xmlInitData);
  }

  gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(this);
}

/////////////////////////////////////////////////
bool GlobalIlluminationCiVct::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gz::gui::events::Render::kType)
  {
    // This event is called in Scene3d's RenderThread, so it's safe to make
    // rendering calls here

    std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
    if (!this->dataPtr->initialized)
    {
      this->LoadGlobalIlluminationCiVct();
    }

    if (this->dataPtr->gi)
    {
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
        this->dataPtr->gi->SetBounceCount(this->dataPtr->bounceCount);
        this->dataPtr->gi->SetHighQuality(this->dataPtr->highQuality);

        if (this->dataPtr->gi->Started())
        {
          // Ogre-Next may crash if some of the settings above are
          // changed while visualizing is enabled.
          this->dataPtr->gi->SetDebugVisualization(
            rendering::GlobalIlluminationCiVct::DVM_None);
        }

        if (this->dataPtr->enabled)
        {
          if (!this->dataPtr->gi->Started())
          {
            this->dataPtr->gi->Bind(this->dataPtr->bindCamera);
            this->dataPtr->gi->Start(this->dataPtr->bounceCount,
                                     this->dataPtr->anisotropic);
            this->CascadesEditableChanged();
          }
          else
          {
            this->dataPtr->gi->NewSettings(this->dataPtr->bounceCount,
                                           this->dataPtr->anisotropic);
          }
          this->dataPtr->gi->Build();
          this->dataPtr->scene->SetActiveGlobalIllumination(this->dataPtr->gi);
        }
        else
        {
          this->dataPtr->scene->SetActiveGlobalIllumination(nullptr);
        }

        if (this->dataPtr->gi->Started())
        {
          // Restore debug visualization to desired.
          this->dataPtr->gi->SetDebugVisualization(
            static_cast<
              rendering::GlobalIlluminationCiVct::DebugVisualizationMode>(
              this->dataPtr->debugVisMode));
        }

        this->dataPtr->visualDirty = false;
        this->dataPtr->lightingDirty = false;
        this->dataPtr->debugVisualizationDirty = false;
      }
      else if (this->dataPtr->lightingDirty)
      {
        this->dataPtr->gi->SetBounceCount(this->dataPtr->bounceCount);
        this->dataPtr->gi->SetHighQuality(this->dataPtr->highQuality);

        if (this->dataPtr->gi->Enabled())
        {
          this->dataPtr->gi->SetDebugVisualization(
            rendering::GlobalIlluminationCiVct::DVM_None);

          this->dataPtr->gi->LightingChanged();

          this->dataPtr->gi->SetDebugVisualization(
            static_cast<
              rendering::GlobalIlluminationCiVct::DebugVisualizationMode>(
              this->dataPtr->debugVisMode));

          this->dataPtr->debugVisualizationDirty = false;
        }
        this->dataPtr->lightingDirty = false;
      }
      else if (this->dataPtr->debugVisualizationDirty)
      {
        if (this->dataPtr->gi->Started())
        {
          this->dataPtr->gi->SetDebugVisualization(
            static_cast<
              rendering::GlobalIlluminationCiVct::DebugVisualizationMode>(
              this->dataPtr->debugVisMode));
        }
        this->dataPtr->debugVisualizationDirty = false;
      }

      if (this->dataPtr->resetRequested)
      {
        if (this->dataPtr->gi->Enabled())
        {
          this->dataPtr->scene->SetActiveGlobalIllumination(nullptr);
          this->dataPtr->enabled = false;
          this->EnabledChanged();
        }

        this->dataPtr->gi->Reset();
        this->CascadesEditableChanged();

        this->dataPtr->resetRequested = false;
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
void GlobalIlluminationCiVct::UpdateDebugVisualizationMode(int _mode)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);

  rendering::GlobalIlluminationCiVct::DebugVisualizationMode
    debugVisualizationMode = rendering::GlobalIlluminationCiVct::DVM_None;

  if (_mode >= rendering::GlobalIlluminationCiVct::DVM_Albedo &&
      _mode <= rendering::GlobalIlluminationCiVct::DVM_None)
  {
    debugVisualizationMode =
      static_cast<rendering::GlobalIlluminationCiVct::DebugVisualizationMode>(
        _mode);
  }

  this->dataPtr->gi->SetDebugVisualization(debugVisualizationMode);
}

//////////////////////////////////////////////////
bool GlobalIlluminationCiVct::SetEnabled(const bool _enabled)
{
  if (_enabled && !this->ValidSettings())
  {
    this->EnabledChanged();
    return false;
  }

  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->enabled = _enabled;
  this->dataPtr->visualDirty = true;

  return _enabled;
}

//////////////////////////////////////////////////
bool GlobalIlluminationCiVct::Enabled() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  return this->dataPtr->enabled;
}

//////////////////////////////////////////////////
bool GlobalIlluminationCiVct::CascadesEditable() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  return !this->dataPtr->gi || !this->dataPtr->gi->Started();
}

//////////////////////////////////////////////////
void GlobalIlluminationCiVct::SetBounceCount(const uint32_t _bounces)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->bounceCount = _bounces;
  this->dataPtr->lightingDirty = true;
}

//////////////////////////////////////////////////
uint32_t GlobalIlluminationCiVct::BounceCount() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  return this->dataPtr->bounceCount;
}

//////////////////////////////////////////////////
void GlobalIlluminationCiVct::SetHighQuality(const bool _quality)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->highQuality = _quality;
  this->dataPtr->lightingDirty = true;
}

//////////////////////////////////////////////////
bool GlobalIlluminationCiVct::HighQuality() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  return this->dataPtr->highQuality;
}

//////////////////////////////////////////////////
void GlobalIlluminationCiVct::SetAnisotropic(const bool _anisotropic)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->anisotropic = _anisotropic;
  this->dataPtr->lightingDirty = true;
}

//////////////////////////////////////////////////
bool GlobalIlluminationCiVct::Anisotropic() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  return this->dataPtr->anisotropic;
}

//////////////////////////////////////////////////
void GlobalIlluminationCiVct::SetDebugVisualizationMode(const uint32_t _visMode)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  if (this->dataPtr->debugVisMode != _visMode)
  {
    this->dataPtr->debugVisMode = _visMode;
    this->dataPtr->debugVisualizationDirty = true;
  }
}

//////////////////////////////////////////////////
uint32_t GlobalIlluminationCiVct::DebugVisualizationMode() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  return this->dataPtr->debugVisMode;
}

//////////////////////////////////////////////////
void GlobalIlluminationCiVct::OnCamareBind(const QString &_cameraName)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);

  auto scene = this->dataPtr->scene.get();
  rendering::SensorPtr sensor = scene->SensorByName(_cameraName.toStdString());
  rendering::CameraPtr asCamera =
    std::dynamic_pointer_cast<rendering::Camera>(sensor);

  if (asCamera)
  {
    this->dataPtr->bindCamera = asCamera;
  }
  else
  {
    this->CameraListChanged();
  }
}

//////////////////////////////////////////////////
void GlobalIlluminationCiVct::OnRefreshCamerasImpl()
  REQUIRES(this->dataPtr->serviceMutex)
{
  auto scene = this->dataPtr->scene.get();
  if (!scene)
  {
    gzerr << "Scene is not initialized. "
          << "Cannot refresh camera list."
          << std::endl;
    return;
  }
  const unsigned int sensorCount = scene->SensorCount();
  for (unsigned int i = 0u; i < sensorCount; ++i)
  {
    rendering::SensorPtr sensor = scene->SensorByIndex(i);
    rendering::CameraPtr asCamera =
      std::dynamic_pointer_cast<rendering::Camera>(sensor);

    if (asCamera)
    {
      this->dataPtr->availableCameras.push_back(
        QString::fromStdString(asCamera->Name()));

      if (!this->dataPtr->bindCamera)
        this->dataPtr->bindCamera = asCamera;
    }
  }

  this->CameraListChanged();
}

//////////////////////////////////////////////////
void GlobalIlluminationCiVct::OnRefreshCameras()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->OnRefreshCamerasImpl();
}

//////////////////////////////////////////////////
QStringList GlobalIlluminationCiVct::CameraList()
{
  return this->dataPtr->availableCameras;
}

//////////////////////////////////////////////////
QObject *GlobalIlluminationCiVct::AddCascade()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);

  if (this->dataPtr->gi && this->dataPtr->gi->Started())
    return nullptr;

  rendering::CiVctCascade const *ref = nullptr;
  if (!this->dataPtr->cascades.empty())
    ref = this->dataPtr->cascades.back()->cascade.get();

  auto cascadeRendering = this->dataPtr->gi->AddCascade(ref);

  this->dataPtr->cascades.push_back(
    std::unique_ptr<CiVctCascadePrivate>(new CiVctCascadePrivate(
      this->dataPtr->serviceMutex, *this, cascadeRendering)));

  if (!ref)
  {
    this->dataPtr->cascades.back()->cascade->SetAreaHalfSize(
      gz::math::Vector3d(5.0, 5.0, 5.0));
    this->dataPtr->cascades.back()->cascade->SetThinWallCounter(1.0f);
  }
  else
  {
    this->dataPtr->cascades.back()->cascade->SetAreaHalfSize(
      this->dataPtr->cascades[this->dataPtr->cascades.size() - 1u]
        ->cascade->AreaHalfSize() *
      2.0);
  }

  return this->dataPtr->cascades.back().get();
}

//////////////////////////////////////////////////
void GlobalIlluminationCiVct::PopCascade()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  if (!this->dataPtr->cascades.empty())
  {
    if (this->dataPtr->gi && this->dataPtr->gi->Started())
      return;

    this->dataPtr->cascades.pop_back();
    this->dataPtr->gi->PopCascade();
  }
}

//////////////////////////////////////////////////
QObject *GlobalIlluminationCiVct::GetCascade(int _idx) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  return this->dataPtr->cascades[(size_t)_idx].get();
}

//////////////////////////////////////////////////
void GlobalIlluminationCiVct::ResetCascades()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->resetRequested = true;
}

//////////////////////////////////////////////////
bool GlobalIlluminationCiVct::ValidSettings() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);

  if (this->dataPtr->cascades.empty())
  {
    return false;
  }

  if (this->dataPtr->bindCamera == nullptr)
  {
    return false;
  }

  return true;
}

// Register this plugin
GZ_ADD_PLUGIN(gz::sim::GlobalIlluminationCiVct, gz::gui::Plugin)
