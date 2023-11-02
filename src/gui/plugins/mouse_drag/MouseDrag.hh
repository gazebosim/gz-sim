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

#ifndef GZ_GUI_MOUSEDRAG_HH
#define GZ_GUI_MOUSEDRAG_HH

#include <memory>

#include <gz/sim/gui/GuiSystem.hh>

namespace gz
{
namespace sim
{
  class MouseDragPrivate;

  /// \brief Translate and rotate links by dragging them with the mouse.
  /// Automatically loads the ApplyLinkWrench system.
  ///
  /// ## Configuration
  ///
  /// * \<rotation_stiffness\> : Stiffness of rotation mode, defaults to 100
  /// * \<position_stiffness\> : Stiffness of translation mode, defaults to 100
  class MouseDrag : public gz::sim::GuiSystem
  {
    Q_OBJECT

    /// \brief Stiffness of rotation mode
    Q_PROPERTY(
      double rotStiffness
      READ RotStiffness
      WRITE SetRotStiffness
      NOTIFY RotStiffnessChanged
    )

    /// \brief Stiffness of translation mode
    Q_PROPERTY(
      double posStiffness
      READ PosStiffness
      WRITE SetPosStiffness
      NOTIFY PosStiffnessChanged
    )

    /// \brief Constructor
    public: MouseDrag();

    /// \brief Destructor
    public: ~MouseDrag() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited
    protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

    // Documentation inherited
    public: void Update(const UpdateInfo &_info,
      EntityComponentManager &_ecm) override;

    /// \brief Callback when COM switch is pressed
    /// \param[in] _checked True if force should be applied to center of mass
    public slots: void OnSwitchCOM(const bool _checked);

    /// \brief Get the rotational stiffness
    /// \return The rotational stiffness
    public: Q_INVOKABLE double RotStiffness() const;

    /// \brief Notify that the rotational stiffness changed
    signals: void RotStiffnessChanged();

    /// \brief Set the rotational stiffness
    /// \param[in] _rotStiffness The new rotational stiffness
    public: Q_INVOKABLE void SetRotStiffness(double _rotStiffness);

    /// \brief Get the translational stiffness
    /// \return The translational stiffness
    public: Q_INVOKABLE double PosStiffness() const;

    /// \brief Notify that the translational stiffness changed
    signals: void PosStiffnessChanged();

    /// \brief Set the translational stiffness
    /// \param[in] _posStiffness The new translational stiffness
    public: Q_INVOKABLE void SetPosStiffness(double _posStiffness);

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<MouseDragPrivate> dataPtr;
  };
}
}

#endif
