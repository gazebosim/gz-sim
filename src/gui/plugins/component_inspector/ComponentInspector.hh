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

#ifndef IGNITION_GAZEBO_GUI_COMPONENTINSPECTOR_HH_
#define IGNITION_GAZEBO_GUI_COMPONENTINSPECTOR_HH_

#include <map>
#include <memory>
#include <string>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/gui/GuiSystem.hh>
#include <ignition/gazebo/Types.hh>

Q_DECLARE_METATYPE(ignition::gazebo::ComponentTypeId)

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
    // cppcheck-suppress syntaxError
    // cppcheck-suppress unmatchedSuppression
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

  /// \brief Specialized to set vector data.
  /// \param[in] _item Item whose data will be set.
  /// \param[in] _data Data to set.
  template<>
  void setData(QStandardItem *_item, const math::Vector3d &_data);

  /// \brief Specialized to set boolean data.
  /// \param[in] _item Item whose data will be set.
  /// \param[in] _data Data to set.
  template<>
  void setData(QStandardItem *_item, const bool &_data);

  /// \brief Specialized to set integer data.
  /// \param[in] _item Item whose data will be set.
  /// \param[in] _data Data to set.
  template<>
  void setData(QStandardItem *_item, const int &_data);

  /// \brief Specialized to set double data.
  /// \param[in] _item Item whose data will be set.
  /// \param[in] _data Data to set.
  template<>
  void setData(QStandardItem *_item, const double &_data);

  /// \brief Specialized to set stream data.
  /// \param[in] _item Item whose data will be set.
  /// \param[in] _data Data to set.
  template<>
  void setData(QStandardItem *_item, const std::ostream &_data);

  /// \brief Set the unit of a given item.
  /// \param[in] _item Item whose unit will be set.
  /// \param[in] _unit Unit to be displayed, such as 'm' for meters.
  void setUnit(QStandardItem *_item, const std::string &_unit);

  /// \brief Model holding information about components, such as their type
  /// and data.
  class ComponentsModel : public QStandardItemModel
  {
    Q_OBJECT

    /// \brief Constructor
    public: explicit ComponentsModel();

    /// \brief Destructor
    public: ~ComponentsModel() override = default;

    // Documentation inherited
    public: QHash<int, QByteArray> roleNames() const override;

    /// \brief Static version of roleNames
    /// \return A hash connecting a unique identifier to a role name.
    public: static QHash<int, QByteArray> RoleNames();

    /// \brief Add a component type to the inspector.
    /// \param[in] _typeId Type of component to be added.
    /// \return Newly created item.
    public slots: QStandardItem *AddComponentType(
        ignition::gazebo::ComponentTypeId _typeId);

    /// \brief Remove a component type from the inspector.
    /// \param[in] _typeId Type of component to be removed.
    public slots: void RemoveComponentType(
        ignition::gazebo::ComponentTypeId _typeId);

    /// \brief Keep track of items in the tree, according to type ID.
    public: std::map<ComponentTypeId, QStandardItem *> items;
  };

  /// \brief Displays a tree view with all the entities in the world.
  ///
  /// ## Configuration
  /// None
  class ComponentInspector : public gazebo::GuiSystem
  {
    Q_OBJECT

    /// \brief Entity
    Q_PROPERTY(
      int entity
      READ Entity
      WRITE SetEntity
      NOTIFY EntityChanged
    )

    /// \brief Type
    Q_PROPERTY(
      QString type
      READ Type
      WRITE SetType
      NOTIFY TypeChanged
    )

    /// \brief Locked
    Q_PROPERTY(
      bool locked
      READ Locked
      WRITE SetLocked
      NOTIFY LockedChanged
    )

    /// \brief Paused
    Q_PROPERTY(
      bool paused
      READ Paused
      WRITE SetPaused
      NOTIFY PausedChanged
    )

    /// \brief Constructor
    public: ComponentInspector();

    /// \brief Destructor
    public: ~ComponentInspector() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited
    public: void Update(const UpdateInfo &, EntityComponentManager &) override;

    /// \brief Callback in Qt thread when pose changes.
    /// \param[in] _x X
    /// \param[in] _y Y
    /// \param[in] _z Z
    /// \param[in] _roll Roll
    /// \param[in] _pitch Pitch
    /// \param[in] _yaw Yaw
    public: Q_INVOKABLE void OnPose(double _x, double _y, double _z,
        double _roll, double _pitch, double _yaw);

    // Documentation inherited
    protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

    /// \brief Get the entity currently inspected.
    /// \return Entity ID.
    public: Q_INVOKABLE int Entity() const;

    /// \brief Set the entity currently inspected.
    /// \param[in] _entity Entity ID.
    public: Q_INVOKABLE void SetEntity(const int &_entity);

    /// \brief Notify that entity has changed.
    signals: void EntityChanged();

    /// \brief Get the type of entity currently inspected.
    /// \return Type, such as 'world' or 'model'
    public: Q_INVOKABLE QString Type() const;

    /// \brief Set the type of entity currently inspected.
    /// \param[in] _type Type, such as 'world' or 'model'.
    public: Q_INVOKABLE void SetType(const QString &_entity);

    /// \brief Notify that entity type has changed
    signals: void TypeChanged();

    /// \brief Get whether the inspector is currently locked on an entity.
    /// \return True for locked
    public: Q_INVOKABLE bool Locked() const;

    /// \brief Set whether the inspector is currently locked on an entity.
    /// \param[in] _locked True for locked.
    public: Q_INVOKABLE void SetLocked(bool _locked);

    /// \brief Notify that locked has changed.
    signals: void LockedChanged();

    /// \brief Get whether the inspector is currently paused for updates.
    /// \return True for paused.
    public: Q_INVOKABLE bool Paused() const;

    /// \brief Set whether the inspector is currently paused for updates.
    /// \param[in] _paused True for paused.
    public: Q_INVOKABLE void SetPaused(bool _paused);

    /// \brief Notify that paused has changed.
    signals: void PausedChanged();

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<ComponentInspectorPrivate> dataPtr;
  };
}
}

#endif
