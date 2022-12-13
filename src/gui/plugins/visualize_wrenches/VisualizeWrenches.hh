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

#ifndef GZ_SIM_GUI_VISUALIZEWRENCHES_HH_
#define GZ_SIM_GUI_VISUALIZEWRENCHES_HH_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <gz/math/Color.hh>

#include <gz/rendering/Material.hh>
#include <gz/rendering/Scene.hh>

#include <gz/sim/components/WrenchDebug.hh>

#include <gz/sim/gui/GuiSystem.hh>

#include <QAbstractListModel>

#include "gz/gui/qt.h"

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE
{
  struct NamedWrench;

  class VisualizeWrenchesPrivate;

  /// \brief Qt AbstactListModel for incoming arrows
  class ForceListModel: public QAbstractListModel
  {
    Q_OBJECT

    public:
      enum ArrowRoles {
        LinkRole = Qt::UserRole + 1,
        PluginRole,
        ColorRole,
        VisibleRole
      };

      struct ForceArrow
      {
        std::string linkName;
        std::string pluginName;
        bool visible;
      };
      ForceListModel();

      ~ForceListModel() override = default;

      QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;
      int rowCount(const QModelIndex &parent = QModelIndex()) const;
      QHash<int, QByteArray> roleNames() const;

      std::optional<rendering::MaterialPtr> getRenderColor(
        const Entity &_entity, const NamedWrench &_wrench);

      Q_INVOKABLE void setVisibility(int index, bool visible);
      Q_INVOKABLE void setColor(int index, QColor color);
      Q_INVOKABLE void setScale(double _scale);

      double getScale();

      rendering::ScenePtr scene;

    private:
      std::vector<ForceArrow> arrows;
      std::unordered_map<std::string, rendering::MaterialPtr> colors;

      struct ArrowInfo
      {
        std::size_t index;
      };
      std::unordered_map<Entity, std::unordered_map<std::string, ArrowInfo>>
        arrow_mapping;

      /// \brief Retrieve the color assigned to a plugin
      /// \param[in] _pluginname - Name of plugin.
      rendering::MaterialPtr retrieveOrAssignColor(std::string _pluginname);
      double scale;
  };

  /// \brief Visualize Force markers
  class VisualizeWrenches : public GuiSystem
  {
    Q_OBJECT

    /// \brief Constructor
    public: VisualizeWrenches();

    /// \brief Destructor
    public: ~VisualizeWrenches() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited
    public: void Update(const UpdateInfo &_info,
        EntityComponentManager &_ecm) override;

    // Documentation inherited
    public: bool eventFilter(QObject *_obj, QEvent *_event) override;

    /// \internal
    /// \brief Pointer to private data
    private: std::unique_ptr<VisualizeWrenchesPrivate> dataPtr;
  };
}
}
}

#endif