/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include "VisualizeForces.hh"

#include <chrono>
#include <queue>
#include <random>
#include <string>
#include <tuple>
#include <unordered_set>
#include <vector>

#include <gz/common/Profiler.hh>

#include <gz/gui/Application.hh>
#include <gz/gui/Conversions.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <gz/msgs/entity_wrench.pb.h>
#include <gz/msgs/entity_wrench_map.pb.h>

#include <gz/plugin/Register.hh>

#include <gz/rendering/ArrowVisual.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/RenderTypes.hh>
#include <gz/rendering/Scene.hh>

#include <gz/transport/Node.hh>

#include "gz/sim/components/EntityWrench.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/World.hh"

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE
{
/// \brief Private data class for VisualizeForces
class VisualizeForcesPrivate
{
  /// \brief Model used to synchronize with the GUI
  public: ForceListModel model;

  /// \brief Node for receiving incoming requests.
  public: transport::Node node;

  /// \brief Queue for incoming messages
  public: std::queue<std::tuple<math::Pose3d, msgs::EntityWrench>> queue;

  /// \brief Mutex for queue and other variables shared with render thread.
  public: std::mutex mutex;

  /// \brief Set of markers already drawn.
  public: std::unordered_set<std::string> onScreenMarkers;

  /// \brief Pointer to the scene.
  public: rendering::ScenePtr scene;

  /// \brief Visuals for each force marker.
  public: std::unordered_map<
      std::string, rendering::ArrowVisualPtr> visuals;

  /// \brief Set the scale. A scale of 1 => force of 1N has a marker length 1m.
  public: double scale{1.0};

  /// \brief System update period calculated from <update_rate>
  public: std::chrono::steady_clock::duration updatePeriod{0};

  /// \brief Last system update simulation time
  public: std::chrono::steady_clock::duration lastUpdateTime{0};


  /// \brief Destructor
  public: ~VisualizeForcesPrivate();

  /// \brief Constructor
  public: VisualizeForcesPrivate();

  /// \brief Get the wrench visuals - runs on GUI thread.
  /// \param[in] _ecm - the ecm.
  public: void RetrieveWrenchesFromEcm(EntityComponentManager &_ecm);

  /// \brief Render force visuals - runs on render thread.
  public: void OnRender();

  public: void AddVisual(const std::string &_ns,
      const math::Color &_color);

  public: void RemoveVisual(const std::string &_ns);

  public: void ClearVisuals();
};

/////////////////////////////////////////////////
VisualizeForcesPrivate::~VisualizeForcesPrivate()
{
  this->ClearVisuals();
}

/////////////////////////////////////////////////
VisualizeForcesPrivate::VisualizeForcesPrivate() = default;

/////////////////////////////////////////////////
void VisualizeForcesPrivate::RetrieveWrenchesFromEcm(
    EntityComponentManager &_ecm)
{
  std::lock_guard<std::mutex> lock(mutex);

#if 0
  {
    // Read components::EntityWrenches
    _ecm.Each<components::WorldPose, components::EntityWrenches>(
      [&](const Entity &_entity,
          components::WorldPose *_worldPose,
          components::EntityWrenches *_entityWrenches) -> bool
      {
      // Push all the wrenches for this entity to the queue.
      for (auto& [key, value] : _entityWrenches->Data())
      {
        this->queue.push({_worldPose->Data(), value});
      }

      // debugging
      // {
      //   std::stringstream ss;
      //   ss << "Received EntityWrenches for entity ["
      //     << _entity << "]\n"
      //     << "WorldPose [" << _worldPose->Data().Pos() << "]\n"
      //     << "Size " << _entityWrenches->Data().size() << "\n";
      //   for (auto& [key, value] : _entityWrenches->Data())
      //   {
      //     ss << "Key: " << key << "\n"
      //        << "Msg: " << value.DebugString() << "\n";
      //   }
      //   gzdbg << ss.str();
      // }

      return true;
    });
  }
#endif

#if 1
  {
    // Read components::EntityWrenchMap
    _ecm.Each<components::WorldPose, components::EntityWrenchMap>(
      [&](const Entity &_entity,
          components::WorldPose *_worldPose,
          components::EntityWrenchMap *_entityWrenchMap) -> bool
      {
      // Push all the wrenches for this entity to the queue.
      for (auto& [key, value] : _entityWrenchMap->Data().wrenches())
      {
        this->queue.push({_worldPose->Data(), value});
      }

      // debugging
      // {
      //   std::stringstream ss;
      //   ss << "Received EntityWrenchMap for entity ["
      //     << _entity << "]\n"
      //     << "WorldPose [" << _worldPose->Data().Pos() << "]\n"
      //     << "Size " << _entityWrenchMap->Data().wrenches().size() << "\n";
      //   for (auto& [key, value] : _entityWrenchMap->Data().wrenches())
      //   {
      //     ss << "Key: " << key << "\n"
      //        << "Msg: " << value.DebugString() << "\n";
      //   }
      //   gzdbg << ss.str();
      // }

      return true;
    });
  }
#endif

#if 0
  {
    /// \todo(srmainwaring) - for debugging - read components::EntityWrench
    _ecm.Each<components::WorldPose, components::EntityWrench>(
      [&](const Entity &_entity,
          components::WorldPose *_worldPose,
          components::EntityWrench *_entityWrench) -> bool
      {
      // Push the wrench for this entity to the queue.
      this->queue.push({_worldPose->Data(), _entityWrench->Data()});

      // debugging
      // {
      //   std::stringstream ss;
      //   ss << "Received EntityWrench for entity ["
      //     << _entity << "]\n"
      //     << "WorldPose [" << _worldPose->Data().Pos() << "]\n"
      //     << "Msg: " << _entityWrench->Data().DebugString() << "\n";
      //   gzdbg << ss.str();
      // }

      return true;
    });
  }
#endif
}

/////////////////////////////////////////////////
void VisualizeForcesPrivate::OnRender()
{
  // Apply lock as this is run on the render thread
  std::lock_guard<std::mutex> lock(mutex);

  if (nullptr == this->scene)
  {
    this->scene = rendering::sceneFromFirstRenderEngine();
    if (nullptr == this->scene)
    {
      return;
    }
    // this->sceneManager.SetScene(this->scene);
  }

  while (true)
  {
    // Get all messages off the queue
    math::Pose3d worldPose;
    msgs::EntityWrench wrenchMsg;
    {
      if(this->queue.empty())
      {
        return;
      }
      std::tie(worldPose, wrenchMsg) = this->queue.front();
      queue.pop();
    }
    // Check if we should render the force based on user's settings.
    auto color = this->model.getRenderColor(wrenchMsg);

    /// \todo(srmainwaring) is there a more efficient lookup method?
    std::string label;
    for (auto& values : wrenchMsg.header().data())
    {
      if (values.key() == "label")
        label = values.value(0);
    }

    // Namespace markers
    auto ns = "force/" + std::to_string(wrenchMsg.entity().id())
      + "/" + label;

    // Marker color if marker is on screen.
    if (!color.has_value())
    {
      // If the marker is already on screen delete it
      if (this->onScreenMarkers.count(ns))
      {
        this->onScreenMarkers.erase(ns);
        this->RemoveVisual(ns);
      }
      continue;
    }
    auto force = msgs::Convert(wrenchMsg.wrench().force());

    this->onScreenMarkers.insert(ns);
    if (this->visuals.find(ns) == this->visuals.end())
    {
      gzdbg << "Adding arrow visual [" << ns << "]\n"
            << "Color   [" << color.value() << "]\n"
            << "Thread  [" << QThread::currentThread() << "]\n";

      this->AddVisual(ns, color.value());
    }

    if (std::abs(force.Length()) > 1.0E-5)
    {
      math::Quaterniond quat;
      quat.SetFrom2Axes(math::Vector3d::UnitZ, force.Normalized());
      math::Pose3d rotation(math::Vector3d::Zero, quat);
      math::Pose3d forcePose(worldPose.Pos(), math::Quaterniond());

      auto visual = this->visuals[ns];
      visual->SetWorldPose(forcePose * rotation);
      visual->SetLocalScale(1.0, 1.0, force.Length() * this->scale);

      // gzdbg << "Wrench visual for entity ["
      //       << wrenchMsg.entity().id() << "]\n"
      //       << "Force     [" << force << "]\n"
      //       << "Position  [" << forcePose.Pos() << "]\n";

    }
  }
}

/////////////////////////////////////////////////
void VisualizeForcesPrivate::AddVisual(
    const std::string &_ns, const math::Color &_color)
{
  auto rootVisual = this->scene->RootVisual();

  rendering::MaterialPtr mat = this->scene->CreateMaterial();
  mat->SetAmbient(_color.R(), _color.G(), _color.B(), 1.0);
  mat->SetDiffuse(_color.R(), _color.G(), _color.B(), 1.0);
  mat->SetSpecular(0.5, 0.5, 0.5, 1.0);
  mat->SetShininess(50);
  mat->SetReflectivity(0);
  mat->SetCastShadows(false);

  auto visual = this->scene->CreateArrowVisual();
  // updateArrow(visual);
  visual->SetMaterial(mat);

  visual->ShowArrowHead(false);
  visual->ShowArrowShaft(true);
  visual->ShowArrowRotation(false);
  visual->SetVisibilityFlags(GZ_VISIBILITY_GUI);
  // visual->SetVisible(this->showForces);
  // visual->SetVisible(true);

  this->visuals[_ns] = visual;
  rootVisual->AddChild(this->visuals[_ns]);
}

/////////////////////////////////////////////////
void VisualizeForcesPrivate::RemoveVisual(const std::string &_ns)
{
  if (this->visuals.find(_ns) != this->visuals.end())
  {
    auto visual = this->visuals[_ns];
    if (visual != nullptr && visual->HasParent())
    {
      visual->Parent()->RemoveChild(visual);
      this->scene->DestroyVisual(visual, true);
      visual.reset();
    }
    this->visuals.erase(_ns);
  }
}

/////////////////////////////////////////////////
void VisualizeForcesPrivate::ClearVisuals()
{
  for (auto&& item : this->visuals)
  {
    auto visual = item.second;
    if (visual != nullptr && visual->HasParent())
    {
      visual->Parent()->RemoveChild(visual);
      this->scene->DestroyVisual(visual, true);
    }
  }
  this->visuals.clear();
}

/////////////////////////////////////////////////
/////////////////////////////////////////////////
ForceListModel::ForceListModel() = default;

/////////////////////////////////////////////////
std::optional<math::Color> ForceListModel::getRenderColor(
  msgs::EntityWrench &_wrench)
{
  auto pluginList = this->force_mapping.find(_wrench.entity().id());

  /// \todo(srmainwaring) is there a more efficient lookup method?
  std::string label;
  for (auto& values : _wrench.header().data())
  {
    if (values.key() == "label")
      label = values.value(0);
  }

  // add new force list for the entity
  if (pluginList == this->force_mapping.end())
  {

    auto color = retrieveOrAssignColor(label);
    beginInsertRows(QModelIndex(), this->forceInfo.size(),
        this->forceInfo.size());
    forceInfo.push_back(
      {
        _wrench.entity().name() + " (" +
        std::to_string(_wrench.entity().id()) +")",
        label,
        true
      }
    );

    force_mapping[_wrench.entity().id()][label] =
      {this->forceInfo.size() -1};
    endInsertRows();
    return color;
  }

  if(pluginList->second.count(label) == 0)
  {
    auto color = retrieveOrAssignColor(label);
    beginInsertRows(QModelIndex(), this->forceInfo.size(),
        this->forceInfo.size());
    this->forceInfo.push_back(
      {
        _wrench.entity().name() + " (" +
          std::to_string(_wrench.entity().id()) +")",
        label,
        true
      }
    );
    pluginList->second[label] = {this->forceInfo.size() -1};
    endInsertRows();
    return color;
  }

  const auto markerInfo = this->forceInfo[
      pluginList->second[label].index];

  if (!markerInfo.visible)
    return std::nullopt;

  return retrieveOrAssignColor(label);
}

/////////////////////////////////////////////////
void ForceListModel::setVisibility(int index, bool visible)
{
  if (index < 0 || static_cast<std::size_t>(index) > this->forceInfo.size())
    return;

  this->forceInfo[index].visible = visible;
  auto modelIndex = createIndex(index, 0);
  dataChanged(modelIndex, modelIndex, {ForceRoles::VisibleRole});
}

/////////////////////////////////////////////////
void ForceListModel::setColor(int index, QColor color)
{
  if (index < 0 || static_cast<std::size_t>(index) > this->forceInfo.size())
    return;

  auto plugin = this->forceInfo[index].pluginName;

  double r, g, b;
  color.getRgbF(&r, &g, &b);
  auto gzColor = math::Color{
    static_cast<float>(r),
    static_cast<float>(g),
    static_cast<float>(b)
  };
  this->colors[plugin] = gzColor;

  auto start = createIndex(0, 0);
  auto end = createIndex(this->forceInfo.size() - 1, 0);
  dataChanged(start, end, {ForceRoles::ColorRole});
}

/////////////////////////////////////////////////
math::Color ForceListModel::retrieveOrAssignColor(std::string _pluginname)
{
  auto color = this->colors.find(_pluginname);

  if (color == this->colors.end())
  {
    static std::default_random_engine e;
    static std::uniform_real_distribution<float> colorPicker(0.0f, 1.0f);

    float r = colorPicker(e);
    float g = colorPicker(e);
    float b = colorPicker(e);

    auto col = math::Color{r, g, b};
    this->colors[_pluginname] = col;
    return col;
  }

  return color->second;
}

/////////////////////////////////////////////////
QVariant ForceListModel::data(
  const QModelIndex &index, int role) const
{
  if (index.row() < 0 ||
    static_cast<std::size_t>(index.row()) > this->forceInfo.size())
    return QVariant();

  if (role == ForceRoles::LinkRole)
  {
    return QString::fromStdString(this->forceInfo[index.row()].linkName);
  }

  if (role == ForceRoles::PluginRole)
  {
    return QString::fromStdString(this->forceInfo[index.row()].pluginName);
  }

  if (role == ForceRoles::ColorRole)
  {
    const auto arrow = this->forceInfo[index.row()];
    const auto color = this->colors.find(arrow.pluginName)->second;

    QColor qcolor(color.R() * 255, color.G() * 255, color.B() *255);
    return qcolor;
  }

  if (role == ForceRoles::VisibleRole)
  {
    return forceInfo[index.row()].visible;
  }

  return QVariant();
}

/////////////////////////////////////////////////
int ForceListModel::rowCount(const QModelIndex &/*unused*/) const
{
  return forceInfo.size();
}

/////////////////////////////////////////////////
QHash<int, QByteArray> ForceListModel::roleNames() const
{
  return {
    std::pair(ForceRoles::LinkRole, "link"),
    std::pair(ForceRoles::PluginRole, "plugin"),
    std::pair(ForceRoles::ColorRole, "arrowColor"),
    std::pair(ForceRoles::VisibleRole, "isVisible")
  };
}

/////////////////////////////////////////////////
/////////////////////////////////////////////////
VisualizeForces::VisualizeForces()
  : GuiSystem(), dataPtr(new VisualizeForcesPrivate)
{
  gz::gui::App()->Engine()->rootContext()->setContextProperty(
    "ForceListModel", &this->dataPtr->model);
}

/////////////////////////////////////////////////
VisualizeForces::~VisualizeForces() = default;

/////////////////////////////////////////////////
void VisualizeForces::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
  {
    this->title = "Visualize forces";
  }

  // Parameters from SDF
  if (_pluginElem)
  {
    {
      auto elem = _pluginElem->FirstChildElement("scale");
      if (nullptr != elem && nullptr != elem->GetText())
      {
        double value(1.0);
        elem->QueryDoubleText(&value);
        this->dataPtr->scale = value;
      }
    }

    {
      double rate(10.0);
      auto elem = _pluginElem->FirstChildElement("update_rate");
      if (nullptr != elem && nullptr != elem->GetText())
      {
        elem->QueryDoubleText(&rate);
      }
      std::chrono::duration<double> period{rate > 0.0 ? 1.0 / rate : 0.0};
      this->dataPtr->updatePeriod = std::chrono::duration_cast<
          std::chrono::steady_clock::duration>(period);
    }
  }

  // Install filter to receive events from the main window.
  gz::gui::App()->findChild<
      gz::gui::MainWindow *>()->installEventFilter(this);
}

/////////////////////////////////////////////////
void VisualizeForces::Update(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("VisualizeForces::Update");
  if (_info.paused) return;

  // Throttle update rate
  auto elapsed = _info.simTime - this->dataPtr->lastUpdateTime;
  if (elapsed > std::chrono::steady_clock::duration::zero() &&
      elapsed < this->dataPtr->updatePeriod)
    return;
  this->dataPtr->lastUpdateTime = _info.simTime;

  this->dataPtr->RetrieveWrenchesFromEcm(_ecm);
}

/////////////////////////////////////////////////
bool VisualizeForces::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gz::gui::events::Render::kType)
  {
    this->dataPtr->OnRender();
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

// Register this plugin
GZ_ADD_PLUGIN(gz::sim::VisualizeForces,
              gz::gui::Plugin)
