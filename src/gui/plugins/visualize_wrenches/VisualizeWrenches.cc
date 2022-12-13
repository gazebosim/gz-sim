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


#include <queue>
#include <random>
#include <string>
#include <unordered_set>
#include <vector>

#include <gz/common/Profiler.hh>

#include <gz/plugin/Register.hh>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <gz/transport/Node.hh>

#include <gz/gui/Application.hh>
#include <gz/gui/Conversions.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>

#include <gz/rendering/ArrowVisual.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/Text.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>

#include "gz/sim/components/Gravity.hh"
#include "gz/sim/components/WrenchDebug.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/World.hh"
#include "gz/sim/Util.hh"
#include "VisualizeWrenches.hh"

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE
{
  struct NamedWrench
  {
    std::string name;
    components::WrenchDebugData data;
  };

  using WrenchMap =
    std::unordered_map<std::string, NamedWrench>;

   using VisualMap =
    std::unordered_map<std::string, gz::rendering::VisualPtr>;
  /// \brief Private data class for VisualizeWrenches
  class VisualizeWrenchesPrivate
  {
    /// \brief Model used to synchronize with the GUI
    public: ForceListModel model;

    /// \brief World entity
    public: Entity worldEntityId{kNullEntity};

    /// \brief Pointer to the camera being recorded
    public: gz::rendering::CameraPtr camera{nullptr};

    /// \brief Pointer to the 3D scene
    public: gz::rendering::ScenePtr scene{nullptr};

    /// \brief Wrench list
    public: std::unordered_map<Entity, WrenchMap> wrenchList;

    /// \brief Visuals list
    public: std::unordered_map<Entity, VisualMap> visualList;

    /// \brief Mutex
    public: std::mutex mtx;

    public: bool rerender{true};

    /////////////////////////////////////////////////
    /// \brief Default constructors
    public: VisualizeWrenchesPrivate()
    {
    }

    /////////////////////////////////////////////////
    /// \brief Get the wrench visuals
    /// \param[in] _ecm - the ecm.
    public: void RetrieveWrenchesFromEcm(EntityComponentManager &_ecm)
    {
      {
      std::lock_guard<std::mutex> lock(this->mtx);
      std::vector<Entity> entities;
      _ecm.Each<components::WrenchDebugList>(
      [&](const Entity &_entity,
          components::WrenchDebugList *_wrench_debug_list) -> bool
      {
        Link link(_entity);
        if (!_wrench_debug_list)
        {
          return true;
        }

        gzerr << _wrench_debug_list->Data().moments.size() << "\n";

        for(auto res: _wrench_debug_list->Data().moments)
        {
          gzerr << "Got " << res.label << "\n";
          this->wrenchList[_entity][res.label] =
            NamedWrench{
              link.Name(_ecm).value_or(""),
              res
            };
        }
        _wrench_debug_list->Data().moments.clear();
        return true;
      });
      }

      const components::Gravity *gravity =
        _ecm.Component<components::Gravity>(this->worldEntityId);

      if (!gravity)
      {
        return;
      }

      /// Add gravity for each link
      _ecm.Each<components::Link>(
      [&](const Entity &_entity,
          components::Link */*_link*/) -> bool
      {
        Link link(_entity);
        const auto *inertial = _ecm.Component<components::Inertial>(_entity);

        if (!inertial)
        {
          return true;
        }

        auto comPose = link.WorldInertialPose(_ecm);
        if (!comPose.has_value())
        {
          enableComponent<components::WorldPose>(_ecm, _entity);
          return true;
        }

        components::WrenchDebugData gravityWrench
        {
          "Gravity",
          inertial->Data().MassMatrix().Mass() * gravity->Data(),
          math::Vector3d(0, 0, 0),
          comPose.value().Pos()
        };

        auto name = link.Name(_ecm);

        wrenchList[_entity]["Gravity"] =
          NamedWrench {
            name.value_or(""),
            gravityWrench
          };

        return true;
      });

      this->rerender = true;
    }

    /////////////////////////////////////////////////
    rendering::VisualPtr CreateOrSpawnArrow(
      const Entity _entity, const std::string &_forceType)
    {
      auto visList = this->visualList.find(_entity);
      if (this->visualList.end() != visList)
      {
        auto arrowElem = visList->second.find(_forceType);
        if(arrowElem != visList->second.end())
        {
          return arrowElem->second;
        }
      }

      auto vis = this->scene->CreateArrowVisual();
      this->scene->RootVisual()->AddChild(vis);
      this->visualList[_entity][_forceType] = vis;

      return vis;
    }

    /////////////////////////////////////////////////
    void Remove(
      const Entity _entity, const std::string &_forceType)
    {
      auto ptr = this->visualList[_entity][_forceType];
      this->scene->RootVisual()->RemoveChild(ptr);
      this->visualList[_entity].erase(_forceType);
    }

    /////////////////////////////////////////////////
    void Initialize()
    {
      // Already initialized
      if (this->scene)
        return;

      this->scene = gz::rendering::sceneFromFirstRenderEngine();

      if (!this->scene)
        return;

      this->model.scene = this->scene;

      for (unsigned int i = 0; i < this->scene->NodeCount(); ++i)
      {
        auto cam = std::dynamic_pointer_cast<gz::rendering::Camera>(
          this->scene->NodeByIndex(i));
        if (cam && cam->HasUserData("user-camera") &&
            std::get<bool>(cam->UserData("user-camera")))
        {
          this->camera = cam;
          gzdbg << "Video Recorder plugin is recoding camera ["
                << this->camera->Name() << "]" << std::endl;
          break;
        }
      }

      if (!this->camera)
      {
        gzerr << "Camera is not available" << std::endl;
      }
    }

    /////////////////////////////////////////////////
    void OnPreRender()
    {
      this->Initialize();

      std::lock_guard<std::mutex> lock(this->mtx);

      if(!this->rerender)
        return;

      std::set<std::pair<Entity, std::string>> updatedWrenches;
      for (const auto &[entity, forceList]: this->wrenchList)
      {
        for (const auto &[forceType, forceData]: forceList)
        {
          auto color = this->model.getRenderColor(entity, forceData);

          if (!color.has_value())
            continue;

          auto visPtr = this->CreateOrSpawnArrow(entity, forceType);
          auto force = forceData.data.force.Length();
          visPtr->SetLocalScale(1, 1, force * this->model.getScale());
          visPtr->SetLocalPosition(forceData.data.position);
          visPtr->SetMaterial(color.value());

          auto axis = forceData.data.force.Cross(math::Vector3d(0, 0, 1));
          if (axis.Length() < 1e-6)
          {
            // Degenerate case, the arrow is parrallel to z
            axis = math::Vector3d(1, 0, 0);
          }

          auto angle =
            acos(forceData.data.force.Dot(math::Vector3d(0, 0, 1)) / force);
          math::Quaterniond qt(axis, angle);

          visPtr->SetLocalRotation(qt);

          updatedWrenches.emplace(entity, forceType);
        }
      }

      std::vector<std::pair<Entity, std::string>> toRemove;
      for(const auto &[entity, forceList]: this->visualList)
      {
        for(const auto &[forceType, _]: forceList)
        {
          if (updatedWrenches.count(std::pair{entity, forceType}) == 0)
          {
            toRemove.emplace_back(entity, forceType);
          }
        }
      }

      for (const auto &[entity, forceType]: toRemove)
      {
        this->Remove(entity, forceType);
      }

      this->rerender = false;
    }

  };

  /////////////////////////////////////////////////
  ForceListModel::ForceListModel()
  {

  }

  /////////////////////////////////////////////////
  std::optional<rendering::MaterialPtr> ForceListModel::getRenderColor(
    const Entity &_entity, const NamedWrench &_wrench)
  {
    if (!this->scene)
      return std::nullopt;
    auto pluginList = this->arrow_mapping.find(_entity);

    if (pluginList == this->arrow_mapping.end())
    {
      auto color = retrieveOrAssignColor(_wrench.data.label);
      beginInsertRows( QModelIndex(), this->arrows.size(), this->arrows.size());
      arrows.push_back(
        {
          _wrench.name + " (" +
          std::to_string(_entity) +")",
          _wrench.data.label,
          true
        }
      );

      arrow_mapping[_entity][_wrench.data.label] =
        {this->arrows.size() -1};
      endInsertRows();
      return color;
    }

    if(pluginList->second.count(_wrench.data.label) == 0)
    {
      auto color = retrieveOrAssignColor(_wrench.data.label);
      beginInsertRows(QModelIndex(), this->arrows.size(),  this->arrows.size());
      this->arrows.push_back(
        {
          _wrench.name + " (" +
            std::to_string(_entity) +")",
          _wrench.data.label,
          true
        }
      );
      pluginList->second[_wrench.data.label] = {this->arrows.size() -1};
      endInsertRows();
      return color;
    }

    const auto arrow =
      this->arrows[pluginList->second[_wrench.data.label].index];

    if (!arrow.visible)
      return std::nullopt;

    return retrieveOrAssignColor(_wrench.data.label);
  }

  /////////////////////////////////////////////////
  void ForceListModel::setVisibility(int index, bool visible)
  {
    if (index < 0 || static_cast<std::size_t>(index) > this->arrows.size())
      return;

    this->arrows[index].visible = visible;
    auto modelIndex = createIndex(index, 0);
    dataChanged(modelIndex, modelIndex, {ArrowRoles::VisibleRole});
  }

  /////////////////////////////////////////////////
  void ForceListModel::setColor(int index, QColor color)
  {
    if (index < 0 || static_cast<std::size_t>(index) > this->arrows.size())
      return;

    auto plugin = this->arrows[index].pluginName;

    double r, g, b;
    color.getRgbF(&r, &g, &b);
    auto ignColor = math::Color{
      static_cast<float>(r),
      static_cast<float>(g),
      static_cast<float>(b)
    };
    this->colors[plugin]->SetDiffuse(ignColor);
    this->colors[plugin]->SetAmbient(ignColor);

    auto start = createIndex(0, 0);
    auto end = createIndex(this->arrows.size() - 1, 0);
    dataChanged(start, end, {ArrowRoles::ColorRole});
  }

  /////////////////////////////////////////////////
  void ForceListModel::setScale(double _scale)
  {
    this->scale = _scale;
  }

  /////////////////////////////////////////////////
  double ForceListModel::getScale()
  {
    return pow(10, this->scale);
  }

  /////////////////////////////////////////////////
  rendering::MaterialPtr
    ForceListModel::retrieveOrAssignColor(std::string _pluginname)
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

      auto material = this->scene->CreateMaterial();
      material->SetAmbient(col);
      material->SetDiffuse(col);
      this->colors[_pluginname] = material;
      return material;
    }

    return color->second;
  }

  /////////////////////////////////////////////////
  QVariant ForceListModel::data(
    const QModelIndex &index, int role) const
  {
    if (index.row() < 0 ||
      static_cast<std::size_t>(index.row()) > this->arrows.size())
      return QVariant();

    if (role == ArrowRoles::LinkRole)
    {
      return QString::fromStdString(this->arrows[index.row()].linkName);
    }

    if (role == ArrowRoles::PluginRole)
    {
      return QString::fromStdString(this->arrows[index.row()].pluginName);
    }

    if (role == ArrowRoles::ColorRole)
    {
      const auto arrow = this->arrows[index.row()];
      const auto material = this->colors.find(arrow.pluginName)->second;
      const auto color = material->Diffuse();

      QColor qcolor(color.R() * 255, color.G() * 255, color.B() *255);
      return qcolor;
    }

    if (role == ArrowRoles::VisibleRole)
    {
      return arrows[index.row()].visible;
    }

    return QVariant();
  }

  /////////////////////////////////////////////////
  int ForceListModel::rowCount(const QModelIndex &/*unused*/) const
  {
    return arrows.size();
  }

  /////////////////////////////////////////////////
  QHash<int, QByteArray> ForceListModel::roleNames() const
  {
    return {
      std::pair(ArrowRoles::LinkRole, "link"),
      std::pair(ArrowRoles::PluginRole, "plugin"),
      std::pair(ArrowRoles::ColorRole, "arrowColor"),
      std::pair(ArrowRoles::VisibleRole, "isVisible")
    };
  }
}
}
}

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
VisualizeWrenches::VisualizeWrenches()
  : GuiSystem(), dataPtr(new VisualizeWrenchesPrivate)
{
  gz::gui::App()->Engine()->rootContext()->setContextProperty(
    "ForceListModel", &this->dataPtr->model);
}

/////////////////////////////////////////////////
VisualizeWrenches::~VisualizeWrenches() {
  for(const auto &[entity, forceList]: this->dataPtr->visualList)
  {
    for(const auto &[_, visual]: forceList)
    {
      this->dataPtr->scene->RootVisual()->RemoveChild(visual);
    }
  }
}

/////////////////////////////////////////////////
void VisualizeWrenches::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Visualize Wrenches";

  gz::gui::App()->findChild<
    gz::gui::MainWindow *>()->installEventFilter(this);
}

//////////////////////////////////////////////////
void VisualizeWrenches::Update(const UpdateInfo &/*unused*/,
    EntityComponentManager &_ecm)
{
  if (this->dataPtr->worldEntityId == kNullEntity)
  {
    this->dataPtr->worldEntityId = worldEntity(_ecm);
    enableComponent<components::WrenchDebugEnable>(_ecm,
      this->dataPtr->worldEntityId);
  }
  this->dataPtr->RetrieveWrenchesFromEcm(_ecm);
}

//////////////////////////////////////////////////
bool VisualizeWrenches::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gz::gui::events::PreRender::kType)
  {
    this->dataPtr->OnPreRender();
  }
  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}


// Register this plugin
GZ_ADD_PLUGIN(gz::sim::VisualizeWrenches,
                    gz::gui::Plugin)