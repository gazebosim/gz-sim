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


#include <ignition/common/Profiler.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <ignition/msgs/marker.pb.h>
#include <ignition/msgs/wrench_visual.pb.h>

#include <ignition/transport/Node.hh>

#include <ignition/gui/Application.hh>
#include <ignition/gui/Conversions.hh>
#include <ignition/gui/MainWindow.hh>

#include "ignition/gazebo/Link.hh"
#include "VisualizeForces.hh"

namespace ignition
{
namespace gazebo
{
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE
{
  /// \brief Private data class for VisualizeForces
  class VisualizeForcesPrivate
  {
    /// \brief Model used to synchronize with the GUI
    public: ForceListModel model;

    /// \brief Node for recieving incoming requests.
    public: transport::Node node;

    /// \brief queue for incoming messages
    public: std::queue<msgs::WrenchVisual> queue;

    /// \brief queue lock
    public: std::mutex mtx;

    /// \brief Set of markers already drawn.
    public: std::unordered_set<std::string> onScreenMarkers;

    /// \brief Default constructors
    public: VisualizeForcesPrivate()
    {
      this->node.Subscribe("/force_viz",
        &VisualizeForcesPrivate::VisualizeCallback, this);
    }

    /// \brief Handler for when we have to visuallize stuff. Simply enqueues
    /// items to be 
    /// \param _stamped - The incoming message
    public: void VisualizeCallback(const msgs::WrenchVisual& _stamped)
    {
      std::lock_guard<std::mutex> lock(mtx);
      queue.push(_stamped);
    }

    /// \brief Render the forces
    /// \param _ecm - The ecm
    public: void RenderForces(EntityComponentManager &_ecm)
    {
      while (true)
      {
        // Get all messages off the queue
        msgs::WrenchVisual wrenchMsg; 
        {
          std::lock_guard<std::mutex> lock(mtx);
          if(this->queue.empty())
          {
            return;
          }
          wrenchMsg = this->queue.front();
          queue.pop();
        }
        // Check if we should render the force based on user's settings.
        auto color = this->model.getRenderColor(wrenchMsg);

        // Namespace markers
        auto ns = "force/" + std::to_string(wrenchMsg.entity().id())
          + "/" + wrenchMsg.label();

        // Marker color if marker is on screen.
        if (!color.has_value())
        {
          // If the marker is already on screen delete it
          if (this->onScreenMarkers.count(ns))
          {
            this->onScreenMarkers.erase(ns);
            msgs::Marker marker;
            marker.set_ns(ns);
            marker.set_id(1);
            marker.set_action(msgs::Marker::DELETE_MARKER);
            this->node.Request("/marker", marker);
          }
          continue;
        }
        msgs::Marker marker;
        auto _force = msgs::Convert(wrenchMsg.wrench().force());

        this->onScreenMarkers.insert(ns);
        marker.set_ns(ns);
        marker.set_id(1);
        marker.set_action(msgs::Marker::ADD_MODIFY);
        marker.set_type(msgs::Marker::CYLINDER);

        marker.set_visibility(msgs::Marker::GUI);

        ignition::msgs::Set(marker.mutable_material()->mutable_ambient(), color.value());
        ignition::msgs::Set(marker.mutable_material()->mutable_diffuse(), color.value());

        Link link(wrenchMsg.entity().id());

        if (link.WorldInertialPose(_ecm).has_value() && std::abs(_force.Length()) > 1e-5)
        {
          // Get the center of mass from where the force will be exerted.
          auto linkPose = link.WorldInertialPose(_ecm).value();

          math::Quaterniond qt;
          qt.From2Axes(math::Vector3d::UnitZ, _force.Normalized());

          // translate cylinder up
          math::Pose3d translateCylinder(
            math::Vector3d(0, 0, _force.Length()/2), math::Quaterniond());
          math::Pose3d rotation(math::Vector3d(0, 0, 0), qt);
          math::Pose3d arrowPose(linkPose.Pos(), math::Quaterniond());
          ignition::msgs::Set(
            marker.mutable_pose(), arrowPose * rotation * translateCylinder);
          ignition::msgs::Set(
            marker.mutable_scale(), math::Vector3d(0.1, 0.1, _force.Length()));

          this->node.Request("/marker", marker);
        }
      }
    }  
  };

  /////////////////////////////////////////////////
  ForceListModel::ForceListModel()
  {

  }

  /////////////////////////////////////////////////
  std::optional<math::Color> ForceListModel::getRenderColor(msgs::WrenchVisual &_wrench)
  {
    auto pluginList = this->arrow_mapping.find(_wrench.entity().id());

    if (pluginList == this->arrow_mapping.end())
    {
      auto color = retrieveOrAssignColor(_wrench.label());
      beginInsertRows( QModelIndex(), this->arrows.size(), this->arrows.size());
      arrows.push_back(
        {
          _wrench.entity().name() + " (" + std::to_string(_wrench.entity().id()) +")",
          _wrench.label(),
          true
        }
      );

      arrow_mapping[_wrench.entity().id()][_wrench.label()] =
        {this->arrows.size() -1};
      endInsertRows();
      return color;
    }

    if(pluginList->second.count(_wrench.label()) == 0)
    {
      auto color = retrieveOrAssignColor(_wrench.label());
      beginInsertRows(QModelIndex(), this->arrows.size(),  this->arrows.size());
      this->arrows.push_back(
        {
          _wrench.entity().name() + " (" + std::to_string(_wrench.entity().id()) +")",
          _wrench.label(),
          true
        }
      );
      pluginList->second[_wrench.label()] = {this->arrows.size() -1};
      endInsertRows();
      return color;
    }

    const auto arrow = this->arrows[pluginList->second[_wrench.label()].index];
    
    if (!arrow.visible)
      return std::nullopt;
  
    return retrieveOrAssignColor(_wrench.label());
  }

  /////////////////////////////////////////////////
  void ForceListModel::setVisibility(int index, bool visible)
  {
    if (index < 0 || index > this->arrows.size())
      return;

    this->arrows[index].visible = visible;
    auto modelIndex = createIndex(index, 0);
    dataChanged(modelIndex, modelIndex, {ArrowRoles::VisibleRole});
  }

  /////////////////////////////////////////////////
  void ForceListModel::setColor(int index, QColor color)
  {
    if (index < 0 || index > this->arrows.size())
      return;

    
    auto plugin = this->arrows[index].pluginName;

    double r, g, b;
    color.getRgbF(&r, &g, &b);
    auto ignColor = math::Color{r, g, b};
    this->colors[plugin] = ignColor;

    auto start = createIndex(0, 0);
    auto end = createIndex(this->arrows.size() - 1, 0);
    dataChanged(start, end, {ArrowRoles::ColorRole});
  }

  /////////////////////////////////////////////////
  math::Color ForceListModel::retrieveOrAssignColor(std::string _pluginname)
  {
    auto color = this->colors.find(_pluginname);
    
    if (color == this->colors.end())
    {
      float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
      float g = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
      float b = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

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
    if (index.row() < 0 || index.row() > this->arrows.size())
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
      const auto color = this->colors.find(arrow.pluginName)->second;

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
  int ForceListModel::rowCount(const QModelIndex &parent) const
  {
    return arrows.size();
  }

  /////////////////////////////////////////////////
  QHash<int,QByteArray> ForceListModel::roleNames() const
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

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
VisualizeForces::VisualizeForces()
  : GuiSystem(), dataPtr(new VisualizeForcesPrivate)
{
  ignition::gui::App()->Engine()->rootContext()->setContextProperty(
    "ForceListModel", &this->dataPtr->model);
}

/////////////////////////////////////////////////
VisualizeForces::~VisualizeForces() = default;

/////////////////////////////////////////////////
void VisualizeForces::LoadConfig(const tinyxml2::XMLElement *)
{

}

//////////////////////////////////////////////////
void VisualizeForces::Update(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  this->dataPtr->RenderForces(_ecm);
}


// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::VisualizeForces,
                    ignition::gui::Plugin)
