/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef IGNITION_GAZEBO_GUI_ENTITYTREE_HH_
#define IGNITION_GAZEBO_GUI_ENTITYTREE_HH_

#include <map>
#include <memory>

#include <ignition/gazebo/gui/GuiSystem.hh>

namespace ignition
{
namespace gazebo
{
  class ComponentInspectorPrivate;

  /// \brief TODO
  class TreeModel : public QStandardItemModel
  {
    Q_OBJECT

    /// \brief Constructor
    public: explicit TreeModel();

    /// \brief Destructor
    public: ~TreeModel() override = default;

    // Documentation inherited
    public: QHash<int, QByteArray> roleNames() const override;

    /// \brief Add an entity to the tree.
    /// \param[in] _entity Entity to be added
    /// \param[in] _entityName Name of entity to be added
    /// \param[in] _parentEntity Parent entity. By default, kNullEntity, which
    /// means it's a root entity.
    /// \param[in] _type Entity type
    public slots: void AddComponent(long _typeId,
        const QString &_typeName);

    public slots: void AddPose(
        double _x,
        double _y,
        double _z,
        double _roll,
        double _pitch,
        double _yaw);

    /// \brief Keep track of items in the tree
    private: std::map<QString, QStandardItem *> items;
  };

  /// \brief Displays a tree view with all the entities in the world.
  ///
  /// ## Configuration
  /// None
  class ComponentInspector : public ignition::gazebo::GuiSystem
  {
    Q_OBJECT

    /// \brief Constructor
    public: ComponentInspector();

    /// \brief Destructor
    public: ~ComponentInspector() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited
    public: void Update(const UpdateInfo &, EntityComponentManager &) override;

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<ComponentInspectorPrivate> dataPtr;
  };
}
}

#endif
