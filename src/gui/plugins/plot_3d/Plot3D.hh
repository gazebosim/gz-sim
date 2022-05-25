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

#ifndef GZ_SIM_GUI_3DPLOT_HH_
#define GZ_SIM_GUI_3DPLOT_HH_

#include <memory>

#include <gz/sim/gui/GuiSystem.hh>

#include "gz/gui/qt.h"

namespace gz
{
namespace sim
{
namespace gui
{
  class Plot3DPrivate;

  /// \brief Plot the trajectory of an entity into the 3D scene.
  ///
  /// This plugin can be instantiated multiple times to plot various entities.
  ///
  /// The plugin is automatically attached to the currently selected entity,
  /// unless it is locked on an entity.
  ///
  /// ## Configuration
  ///
  /// * `<entity_name>` (optional): Plot the given entity at startup. Accepts
  /// names scoped with `::`, for example `my_model::my_link`. If not provided,
  /// the plugin starts not attached to any entity, and will attach to the
  /// next selected entity.
  ///
  /// * `<color>` (optional): RGB color of line, defaults to blue.
  ///
  /// * `<offset>` (optional): XYZ offset from the entity's origin to plot from,
  /// expressed in the entity's frame. Defaults to zero.
  ///
  /// * `<minimum_distance>` (optional): The minimum distance between points to
  /// plot. A new point will not be plotted until the entity has moved beyond
  /// this distance from the previous point. Defaults to 0.05 m.
  ///
  /// * `<maximum_points> (optional)`: Maximum number of points on the plot.
  /// After this number is reached, the older points start being deleted.
  /// Defaults to 1000.
  ///
  class Plot3D : public gz::sim::GuiSystem
  {
    Q_OBJECT

    /// \brief Target entity
    Q_PROPERTY(
      Entity targetEntity
      READ TargetEntity
      WRITE SetTargetEntity
      NOTIFY TargetEntityChanged
    )

    /// \brief Target entity scoped name
    Q_PROPERTY(
      QString targetName
      READ TargetName
      WRITE SetTargetName
      NOTIFY TargetNameChanged
    )

    /// \brief Whether the plugin is locked on an entity
    Q_PROPERTY(
      bool locked
      READ Locked
      WRITE SetLocked
      NOTIFY LockedChanged
    )

    /// \brief XYZ offset to the entity's origin to plot
    Q_PROPERTY(
      QVector3D offset
      READ Offset
      WRITE SetOffset
      NOTIFY OffsetChanged
    )

    /// \brief Plot line color
    Q_PROPERTY(
      QVector3D color
      READ Color
      WRITE SetColor
      NOTIFY ColorChanged
    )

    /// \brief Minimum distance between plot points
    Q_PROPERTY(
      double minDistance
      READ MinDistance
      WRITE SetMinDistance
      NOTIFY MinDistanceChanged
    )

    /// \brief Maximum number of total points on the plot
    Q_PROPERTY(
      int maxPoints
      READ MaxPoints
      WRITE SetMaxPoints
      NOTIFY MaxPointsChanged
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

    /// \brief Get whether the plugin is currently locked on a target.
    /// \return True for locked
    public: Q_INVOKABLE bool Locked() const;

    /// \brief Set whether the plugin is currently locked on a target.
    /// \param[in] _locked True for locked.
    public: Q_INVOKABLE void SetLocked(bool _locked);

    /// \brief Notify that locked has changed.
    signals: void LockedChanged();

    /// \brief Get the offset in the target's frame.
    /// \return The current offset.
    public: Q_INVOKABLE QVector3D Offset() const;

    /// \brief Set the offset.
    /// \param[in] _offset The offset in the target's frame
    public: Q_INVOKABLE void SetOffset(const QVector3D &_offset);

    /// \brief Notify that the offset has changed.
    signals: void OffsetChanged();

    /// \brief Get the color of the plot.
    /// \return The current color.
    public: Q_INVOKABLE QVector3D Color() const;

    /// \brief Set the color of the plot.
    /// \param[in] _color New color.
    public: Q_INVOKABLE void SetColor(const QVector3D &_color);

    /// \brief Notify that the color has changed.
    signals: void ColorChanged();

    /// \brief Get the minimum distance between points.
    /// \return The current minimum distance.
    public: Q_INVOKABLE double MinDistance() const;

    /// \brief Set the minimum distance between points. If the target moved
    /// less than this distance, the latest position won't be plotted.
    /// \param[in] _minDistance New distance.
    public: Q_INVOKABLE void SetMinDistance(double _minDistance);

    /// \brief Notify that the minimum distance has changed.
    signals: void MinDistanceChanged();

    /// \brief Get the maximum number of points.
    /// \return The current maximum points.
    public: Q_INVOKABLE int MaxPoints() const;

    /// \brief Set the maximum number of points. If the plot has more than
    /// this number, older points start being removed.
    /// \param[in] _maxPoints Maximum number of points.
    public: Q_INVOKABLE void SetMaxPoints(int _maxPoints);

    /// \brief Notify that the maximum points has changed.
    signals: void MaxPointsChanged();

    // Documentation inherited
    protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

    /// \brief Clear plot
    private: void ClearPlot();

    /// \internal
    /// \brief Pointer to private data
    private: std::unique_ptr<Plot3DPrivate> dataPtr;
  };
}
}
}

#endif
