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
#include <ignition/msgs/wrench_stamped.pb.h>

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
    public: std::queue<msgs::WrenchStamped> queue;

    /// \brief Default constructors
    public: VisualizeForcesPrivate()
    {
      this->node.Subscribe("/force_viz",
        &VisualizeForcesPrivate::VisualizeCallback, this);
    }

    /// \brief Handler for when we have to visuallize stuff. Simply enqueues
    /// items to be 
    /// \param _stamped - The incoming message
    public: void VisualizeCallback(const msgs::WrenchStamped& _stamped)
    {
      queue.push(_stamped);
    }

    /// \brief Render the forces
    /// \param _ecm - The ecm
    public: void RenderForces(EntityComponentManager &_ecm)
    {
      while (!this->queue.empty())
      {
        auto wrenchMsg = this->queue.front();
        queue.pop();

        auto color = this->model.getRenderColor(wrenchMsg);

        if (!color.has_value())
          continue;

        msgs::Marker marker;
        auto _force = msgs::Convert(wrenchMsg.wrench().force());

        auto ns = "force/" + std::to_string(wrenchMsg.entity()) + "/" + wrenchMsg.plugin();
        marker.set_ns(ns);
        marker.set_id(1);
        marker.set_action(msgs::Marker::ADD_MODIFY);
        marker.set_type(msgs::Marker::CYLINDER);

        marker.set_visibility(msgs::Marker::GUI);

        ignition::msgs::Set(marker.mutable_material()->mutable_ambient(), color.value());
        ignition::msgs::Set(marker.mutable_material()->mutable_diffuse(), color.value());

        Link link(wrenchMsg.entity());

        if (link.WorldInertialPose(_ecm).has_value() && std::abs(_force.Length()) > 1e-5)
        {
          auto linkPose = link.WorldInertialPose(_ecm).value();

          math::Quaterniond qt;
          qt.From2Axes(math::Vector3d::UnitZ, _force.Normalized());

          // translate cylinder up
          math::Pose3d translateCylinder(math::Vector3d(0, 0, _force.Length()/2), math::Quaterniond());
          math::Pose3d rotation(math::Vector3d(0, 0, 0), qt);
          math::Pose3d arrowPose(linkPose.Pos(), math::Quaterniond());
          ignition::msgs::Set(marker.mutable_pose(), arrowPose * rotation * translateCylinder);
          ignition::msgs::Set(marker.mutable_scale(), math::Vector3d(0.1, 0.1, _force.Length()));

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
  std::optional<math::Color> ForceListModel::getRenderColor(msgs::WrenchStamped _wrench)
  {
    auto pluginList = this->arrow_mapping.find(_wrench.entity());

    if (pluginList == this->arrow_mapping.end())
    {
      auto color = retrieveOrAssignColor(_wrench.plugin());
      beginInsertRows( QModelIndex(), arrows.size(), arrows.size());
      arrows.push_back(
        {
          std::to_string(_wrench.entity()),
          _wrench.plugin(),
          true
        }
      );

      arrow_mapping[_wrench.entity()][_wrench.plugin()] = {arrows.size() -1};
      endInsertRows();
      ignerr << _wrench.entity() << ", " <<_wrench.plugin() << "\n";
      return color;
    }

    if(pluginList->second.count(_wrench.plugin()) == 0)
    {
      auto color = retrieveOrAssignColor(_wrench.plugin());
      beginInsertRows( QModelIndex(), arrows.size(),  arrows.size());
      arrows.push_back(
        {
          std::to_string(_wrench.entity()),
          _wrench.plugin(),
          true
        }
      );
      pluginList->second[_wrench.plugin()] = {arrows.size() -1};
            ignerr << _wrench.entity() << ", " <<_wrench.plugin() << "\n";

      endInsertRows();
      return color;
    }

    const auto arrow = arrows[pluginList->second[_wrench.plugin()].index];
    
    if (!arrow.visible)
      return std::nullopt;
  
    return retrieveOrAssignColor(_wrench.plugin());
  }

  /////////////////////////////////////////////////
  math::Color ForceListModel::retrieveOrAssignColor(std::string _pluginname)
  {
    auto color = colors.find(_pluginname);
    
    if (color == colors.end())
    {
      float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
      float g = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
      float b = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

      auto col = math::Color{r, g, b};
      colors[_pluginname] = col;
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
