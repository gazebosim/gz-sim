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

#ifndef GZ_SIM_GUI_VISUALIZECONTACTS_HH_
#define GZ_SIM_GUI_VISUALIZECONTACTS_HH_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <gz/math/Color.hh>
#include <gz/msgs/entity_wrench.pb.h>

#include <gz/sim/gui/GuiSystem.hh>

#include <QAbstractListModel>

#include "gz/gui/qt.h"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE
{
class VisualizeForcesPrivate;

/// \brief Qt AbstactListModel for incoming arrows
class ForceListModel: public QAbstractListModel
{
  Q_OBJECT

  public:
    enum ForceRoles {
      LinkRole = Qt::UserRole + 1,
      PluginRole,
      ColorRole,
      VisibleRole
    };

    struct ForceInfo
    {
      std::string linkName;
      std::string pluginName;
      bool visible;
    };

    ForceListModel();

    ~ForceListModel() override = default;

    QVariant data(const QModelIndex &index,
      int role = Qt::DisplayRole) const override;
    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    QHash<int, QByteArray> roleNames() const override;

    std::optional<math::Color> getRenderColor(msgs::EntityWrench &_wrench);

    Q_INVOKABLE void setVisibility(int index, bool visible);
    Q_INVOKABLE void setColor(int index, QColor color);

  private:
    std::vector<ForceInfo> forceInfo;
    std::unordered_map<std::string, math::Color> colors;

    struct ForceIndex
    {
      std::size_t index;
    };

    /// \brief map between entities and a force map.
    /// The force map is between a label identifying the origin of the
    /// wtench (plugin label) and an index into the force information.
    std::unordered_map<Entity, std::unordered_map<std::string, ForceIndex>>
      force_mapping;

    /// \brief Retrieve the color assigned to a plugin
    /// \param[in] _pluginname - Name of plugin.
    math::Color retrieveOrAssignColor(std::string _pluginname);
};

/// \brief Visualize Force markers
class VisualizeForces : public gz::sim::GuiSystem
{
  Q_OBJECT

  /// \brief Constructor
  public: VisualizeForces();

  /// \brief Destructor
  public: ~VisualizeForces() override;

  // Documentation inherited
  public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  // Documentation inherited
  public: void Update(const UpdateInfo &_info,
      EntityComponentManager &_ecm) override;

  // Documentation inherited
  protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

  /// \internal
  /// \brief Pointer to private data
  private: std::unique_ptr<VisualizeForcesPrivate> dataPtr;
};

}
}
}

#endif
