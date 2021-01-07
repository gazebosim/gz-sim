/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef IGNITION_GAZEBO_GUI_JOINTPOSITIONCONTROLLER_HH_
#define IGNITION_GAZEBO_GUI_JOINTPOSITIONCONTROLLER_HH_

#include <map>
#include <memory>
#include <string>

#include <ignition/gazebo/gui/GuiSystem.hh>
#include <ignition/gazebo/Types.hh>

Q_DECLARE_METATYPE(ignition::gazebo::Entity)

namespace ignition
{
namespace gazebo
{
  class JointPositionControllerPrivate;

  /// \brief Model holding information about joints
  class JointsModel : public QStandardItemModel
  {
    Q_OBJECT

    /// \brief Constructor
    public: explicit JointsModel();

    /// \brief Destructor
    public: ~JointsModel() override = default;

    // Documentation inherited
    public: QHash<int, QByteArray> roleNames() const override;

    /// \brief Static version of roleNames
    /// \return A hash connecting a unique identifier to a role name.
    public: static QHash<int, QByteArray> RoleNames();

    /// \brief Add a component type to the inspector.
    /// \param[in] _typeId Type of component to be added.
    /// \return Newly created item.
    public slots: QStandardItem *AddJoint(Entity _entity);

    /// \brief Remove a joint from the list.
    /// \param[in] _entity
    public slots: void RemoveJoint(Entity _entity);

    /// \brief Clear all joints
    public slots: void Clear();

    /// \brief Keep track of items in the list, according to joint entity.
    public: std::map<Entity, QStandardItem *> items;
  };
  /// \brief
  ///
  /// ## Configuration
  /// None
  class JointPositionController : public gazebo::GuiSystem
  {
    Q_OBJECT

    /// \brief Model entity
    Q_PROPERTY(
      int modelEntity
      READ ModelEntity
      WRITE SetModelEntity
      NOTIFY ModelEntityChanged
    )

    /// \brief Model name
    Q_PROPERTY(
      QString modelName
      READ ModelName
      WRITE SetModelName
      NOTIFY ModelNameChanged
    )

    /// \brief Locked
    Q_PROPERTY(
      bool locked
      READ Locked
      WRITE SetLocked
      NOTIFY LockedChanged
    )

    /// \brief Constructor
    public: JointPositionController();

    /// \brief Destructor
    public: ~JointPositionController() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited
    public: void Update(const UpdateInfo &, EntityComponentManager &) override;

    /// \brief Callback in Qt thread when joint position changes.
    /// \param[in] _jointName Name of joint being commanded
    /// \param[in] _pos New joint position
    public: Q_INVOKABLE void OnCommand(const QString &_jointName, double _pos);

    // Documentation inherited
    protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

    /// \brief Get the entity currently inspected.
    /// \return ModelEntity ID.
    public: Q_INVOKABLE int ModelEntity() const;

    /// \brief Set the entity currently inspected.
    /// \param[in] _entity ModelEntity ID.
    public: Q_INVOKABLE void SetModelEntity(const int &_entity);

    /// \brief Notify that entity has changed.
    signals: void ModelEntityChanged();

    /// \brief Get the type of entity currently inspected.
    /// \return ModelName, such as 'world' or 'model'
    public: Q_INVOKABLE QString ModelName() const;

    /// \brief Set the type of entity currently inspected.
    /// \param[in] _type ModelName, such as 'world' or 'model'.
    public: Q_INVOKABLE void SetModelName(const QString &_entity);

    /// \brief Notify that entity type has changed
    signals: void ModelNameChanged();

    /// \brief Get whether the inspector is currently locked on an entity.
    /// \return True for locked
    public: Q_INVOKABLE bool Locked() const;

    /// \brief Set whether the inspector is currently locked on an entity.
    /// \param[in] _locked True for locked.
    public: Q_INVOKABLE void SetLocked(bool _locked);

    /// \brief Notify that locked has changed.
    signals: void LockedChanged();

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<JointPositionControllerPrivate> dataPtr;
  };
}
}

#endif
