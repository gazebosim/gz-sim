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

#include <ignition/transport/Node.hh>

#include <ignition/gui/Application.hh>
#include <ignition/gui/Conversions.hh>
#include <ignition/gui/MainWindow.hh>

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
    public: ForceListModel model; 
  };

  ForceListModel::ForceListModel()
  {
    arrows = {
      ForceArrow{
        "be", "buoyancyEngine", "#440044", false
      },
      ForceArrow{
        "be", "buoyancyEngine", "#004444", false
      },
      ForceArrow{
        "be", "buoyancyEngine", "#444400", false
      },
    };
  }

  QVariant ForceListModel::data(
    const QModelIndex &index, int role) const
  {
    if (index.row() < 0 || index.row() > arrows.size())
      return QVariant();

    if (role == ArrowRoles::LinkRole)
    {
      return QString::fromStdString(arrows[index.row()].linkName);
    }

    if (role == ArrowRoles::PluginRole)
    {
      return QString::fromStdString(arrows[index.row()].pluginName);
    }

    if (role == ArrowRoles::ColorRole)
    {
      return QString::fromStdString(arrows[index.row()].color);
    }

    if (role == ArrowRoles::VisibleRole)
    {
      return arrows[index.row()].visible;
    }

    return QVariant();
  }

  int ForceListModel::rowCount(const QModelIndex &parent) const
  {
    return arrows.size();
  }
  
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
  
}


// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::VisualizeForces,
                    ignition::gui::Plugin)
