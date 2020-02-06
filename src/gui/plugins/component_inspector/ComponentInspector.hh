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
#include <ignition/math/Pose3.hh>

#include <ignition/gazebo/gui/GuiSystem.hh>

namespace ignition
{
namespace gazebo
{
  class ComponentInspectorPrivate;

  /// \brief Generic function to set data.
  /// \param[in] _item Item whose data will be set.
  /// \param[in] _data Data to set.
  template <class DataType>
  void setData(QStandardItem *_item, const DataType &_data)
  {
    if constexpr (traits::IsOutStreamable<std::ostream, DataType>::value)
    {
      std::stringstream ss;
      ss << _data;
      setData(_item, ss.str());
    }
    else
    {
      ignwarn << "Attempting to set unsupported data type to item ["
              << _item->text().toStdString() << "]" << std::endl;
    }
  }

  /// \brief Specialized to set string data.
  /// \param[in] _item Item whose data will be set.
  /// \param[in] _data Data to set.
  template<>
  void setData(QStandardItem *_item, const std::string &_data);

  /// \brief Specialized to set pose data.
  /// \param[in] _item Item whose data will be set.
  /// \param[in] _data Data to set.
  template<>
  void setData(QStandardItem *_item, const math::Pose3d &_data);

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

    /// \brief Static version of roleNames
    /// \return A hash connecting a unique ideitifier to a role name.
    public: static QHash<int, QByteArray> RoleNames();

    /// \brief Add a component type to the inspector.
    /// \param[in] _typeId Type of component to be added.
    /// \return Newly created item.
    public slots: QStandardItem *AddComponentType(long _typeId);

    /// \brief Remove a component type from the inspector.
    /// \param[in] _typeId Type of component to be removed.
    public slots: void RemoveComponentType(long _typeId);

    /// \brief Keep track of items in the tree, according to type ID.
    public: std::map<long, QStandardItem *> items;
  };

  /// \brief Displays a tree view with all the entities in the world.
  ///
  /// ## Configuration
  /// None
  class ComponentInspector : public ignition::gazebo::GuiSystem
  {
    Q_OBJECT

    /// \brief Entity
    Q_PROPERTY(
      int entity
      READ Entity
      WRITE SetEntity
      NOTIFY EntityChanged
    )

    /// \brief Constructor
    public: ComponentInspector();

    /// \brief Destructor
    public: ~ComponentInspector() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited
    public: void Update(const UpdateInfo &, EntityComponentManager &) override;

    // Documentation inherited
    protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

    /// \brief Get the message type as a string, for example
    /// 'ignition.msgs.StringMsg'
    /// \return Message type
    public: Q_INVOKABLE int Entity() const;

    /// \brief Set the message type from a string, for example
    /// 'ignition.msgs.StringMsg'
    /// \param[in] _entity Message type
    public: Q_INVOKABLE void SetEntity(const int &_entity);

    /// \brief Notify that message type has changed
    signals: void EntityChanged();

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<ComponentInspectorPrivate> dataPtr;
  };
}
}

#endif
