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

#ifndef IGNITION_GAZEBO_GUI_3DPLOT_HH_
#define IGNITION_GAZEBO_GUI_3DPLOT_HH_

#include <memory>

#include <ignition/gazebo/gui/GuiSystem.hh>

#include "ignition/gui/qt.h"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE
{
  class Plot3DPrivate;

  /// \brief
  class Plot3D : public ignition::gazebo::GuiSystem
  {
    Q_OBJECT

    /// \brief Target entity
    Q_PROPERTY(
      Entity targetEntity
      READ TargetEntity
      WRITE SetTargetEntity
      NOTIFY TargetEntityChanged
    )

    /// \brief Target name
    Q_PROPERTY(
      QString targetName
      READ TargetName
      WRITE SetTargetName
      NOTIFY TargetNameChanged
    )

    /// \brief Locked
    Q_PROPERTY(
      bool locked
      READ Locked
      WRITE SetLocked
      NOTIFY LockedChanged
    )

    /// \brief Constructor
    public: Plot3D();

    /// \brief Destructor
    public: ~Plot3D() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited
    public: void Update(const UpdateInfo &_info,
        EntityComponentManager &_ecm) override;

    /// \brief Get the target currently controlled.
    /// \return Target entity ID.
    public: Q_INVOKABLE Entity TargetEntity() const;

    /// \brief Set the target currently controlled.
    /// \param[in] _entity Target entity ID.
    public: Q_INVOKABLE void SetTargetEntity(Entity _entity);

    /// \brief Notify that entity has changed.
    signals: void TargetEntityChanged();

    /// \brief Get the name of target currently controlled.
    /// \return TargetName, such as 'world' or 'target'
    public: Q_INVOKABLE QString TargetName() const;

    /// \brief Set the name of target currently controlled.
    /// \param[in] _name TargetName, such as 'world' or 'target'.
    public: Q_INVOKABLE void SetTargetName(const QString &_name);

    /// \brief Notify that target name has changed
    signals: void TargetNameChanged();

    /// \brief Get whether the controller is currently locked on a target.
    /// \return True for locked
    public: Q_INVOKABLE bool Locked() const;

    /// \brief Set whether the controller is currently locked on a target.
    /// \param[in] _locked True for locked.
    public: Q_INVOKABLE void SetLocked(bool _locked);

    /// \brief Notify that locked has changed.
    signals: void LockedChanged();

    /// \brief Callback to update offset
    /// \param[in] _x Offset on X axis
    /// \param[in] _y Offset on Y axis
    /// \param[in] _z Offset on Z axis
    public: Q_INVOKABLE void SetOffset(double _x, double _y, double _z);

    /// \brief Callback to update color
    /// \param[in] _r Red
    /// \param[in] _g Green
    /// \param[in] _b Blue
    public: Q_INVOKABLE void SetColor(double _r, double _g, double _b);

    /// \brief Callback to set minimum distance between points.
    /// \param[in] _dist Minimum distance
    public: Q_INVOKABLE void SetMinDistance(double _dist);

    /// \brief Callback to set maximum number of points
    /// \param[in] _max Maximum number of points in the plot.
    public: Q_INVOKABLE void SetMaxPoints(int _max);

    // Documentation inherited
    protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

    /// \internal
    /// \brief Pointer to private data
    private: std::unique_ptr<Plot3DPrivate> dataPtr;
  };
}
}
}

#endif
