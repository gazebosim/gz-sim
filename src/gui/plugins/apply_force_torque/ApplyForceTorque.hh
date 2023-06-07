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

#ifndef GZ_GUI_APPLYFORCETORQUE_HH_
#define GZ_GUI_APPLYFORCETORQUE_HH_

#include <memory>

#include <gz/sim/gui/GuiSystem.hh>

namespace gz
{
namespace sim
{
  class ApplyForceTorquePrivate;

  /// \brief Publish wrench to "/world/apply_link_wrench/wrench" topic.
  ///
  /// ## Configuration
  /// This plugin doesn't accept any custom configuration.
  class ApplyForceTorque : public gz::sim::GuiSystem
  {
    Q_OBJECT

    /// \brief Model name
    Q_PROPERTY(
      QString modelName
      READ ModelName
      NOTIFY ModelNameChanged
    )

    /// \brief Link list
    Q_PROPERTY(
      QStringList linkNameList
      READ LinkNameList
      NOTIFY LinkNameListChanged
    )

    /// \brief Link index
    Q_PROPERTY(
      int linkIndex
      READ LinkIndex
      WRITE SetLinkIndex
      NOTIFY LinkIndexChanged
    )

    /// \brief Constructor
    public: ApplyForceTorque();

    /// \brief Destructor
    public: ~ApplyForceTorque() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited
    protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

    // Documentation inherited
    public: void Update(const UpdateInfo &_info,
      EntityComponentManager &_ecm) override;

    /// \brief Get the name of the selected model
    public: Q_INVOKABLE QString ModelName() const;

    /// \brief Notify that the model name changed
    signals: void ModelNameChanged();

    /// \brief Get the name of the links of the selected model
    public: Q_INVOKABLE QStringList LinkNameList() const;

    /// \brief Notify that the link list changed
    signals: void LinkNameListChanged();

    /// \brief Get index of the link in the list
    public: Q_INVOKABLE int LinkIndex() const;

    /// \brief Notify that the link index changed
    signals: void LinkIndexChanged();

    /// \brief Set the index of the link in the list
    public: Q_INVOKABLE void SetLinkIndex(int _linkIndex);

    /// \brief Set components of force
    /// \param[in] _x X component of force
    /// \param[in] _y Y component of force
    /// \param[in] _z Z component of force
    public: Q_INVOKABLE void UpdateForce(double _x, double _y, double _z);

    /// \brief Set components of force offset
    /// \param[in] _x X component of force offset
    /// \param[in] _y Y component of force offset
    /// \param[in] _z Z component of force offset
    public: Q_INVOKABLE void UpdateOffset(double _x, double _y, double _z);

    /// \brief Set components of torque
    /// \param[in] _x X component of torque
    /// \param[in] _y Y component of torque
    /// \param[in] _z Z component of torque
    public: Q_INVOKABLE void UpdateTorque(double _x, double _y, double _z);

    /// \brief Apply the specified force
    public: Q_INVOKABLE void ApplyForce();

    /// \brief Apply the specified torque
    public: Q_INVOKABLE void ApplyTorque();

    /// \brief Apply the specified force and torque
    public: Q_INVOKABLE void ApplyAll();

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<ApplyForceTorquePrivate> dataPtr;
  };
}
}

#endif
