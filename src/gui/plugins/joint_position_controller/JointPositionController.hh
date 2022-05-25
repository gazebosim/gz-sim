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

#ifndef GZ_SIM_GUI_JOINTPOSITIONCONTROLLER_HH_
#define GZ_SIM_GUI_JOINTPOSITIONCONTROLLER_HH_

#include <map>
#include <memory>
#include <string>

#include <gz/sim/gui/GuiSystem.hh>
#include <gz/sim/Types.hh>

Q_DECLARE_METATYPE(gz::sim::Entity)

namespace gz
{
namespace sim
{
namespace gui
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

    /// \brief Add a joint to the list.
    /// \param[in] _typeId Joint to be added.
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

  /// \brief Control position of all joints in the selected model.
  /// The model must have loaded an
  /// `gz::sim::systems::JointPositionController`
  /// for each joint to be controlled.
  ///
  /// This plugin publishes position command messages (`gz::msgs::Double`)
  /// on topics in the format `/model/<model_name>/joint/<joint_name/0/cmd_pos`.
  ///
  /// Only the 1st axis of each joint is considered. Joints without axes are
  /// ignored.
  ///
  /// When the lock button is checked, the model doesn't change even if it's
  /// deselected.
  ///
  /// ## Configuration
  ///
  /// `<model_name>`: Load the widget pointed at the given model, so it's not
  /// necessary to select it. If a model is given at startup, the plugin starts
  /// in locked mode.
  class JointPositionController : public sim::GuiSystem
  {
    Q_OBJECT

    /// \brief Model entity
    Q_PROPERTY(
      Entity modelEntity
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

    /// \brief Callback in Qt thread when user requests a reset.
    public: Q_INVOKABLE void OnReset();

    /// \brief Get the model currently controlled.
    /// \return Model entity ID.
    public: Q_INVOKABLE Entity ModelEntity() const;

    /// \brief Set the model currently controlled.
    /// \param[in] _entity Model entity ID.
    public: Q_INVOKABLE void SetModelEntity(Entity _entity);

    /// \brief Notify that entity has changed.
    signals: void ModelEntityChanged();

    /// \brief Get the name of model currently controlled.
    /// \return ModelName, such as 'world' or 'model'
    public: Q_INVOKABLE QString ModelName() const;

    /// \brief Set the name of model currently controlled.
    /// \param[in] _name ModelName, such as 'world' or 'model'.
    public: Q_INVOKABLE void SetModelName(const QString &_name);

    /// \brief Notify that model name has changed
    signals: void ModelNameChanged();

    /// \brief Get whether the controller is currently locked on a model.
    /// \return True for locked
    public: Q_INVOKABLE bool Locked() const;

    /// \brief Set whether the controller is currently locked on a model.
    /// \param[in] _locked True for locked.
    public: Q_INVOKABLE void SetLocked(bool _locked);

    /// \brief Notify that locked has changed.
    signals: void LockedChanged();

    // Documentation inherited
    protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<JointPositionControllerPrivate> dataPtr;
  };
}
}
}

#endif
