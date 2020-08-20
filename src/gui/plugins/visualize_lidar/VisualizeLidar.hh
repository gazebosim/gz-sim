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

#ifndef IGNITION_GAZEBO_GUI_VISUALIZELIDAR_HH_
#define IGNITION_GAZEBO_GUI_VISUALIZELIDAR_HH_

#include <memory>

#include "ignition/msgs/laserscan.pb.h"
#include "ignition/gazebo/gui/GuiSystem.hh"
#include "ignition/gui/qt.h"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE
{
  class VisualizeLidarPrivate;

  /// \brief Visualize the LaserScan message returned by the sensors. Use the
  /// checkbox to turn visualization of non-hitting rays on or off and
  /// the textfield to select the message to be visualised. The combobox is
  /// used to select the type of visual for the sensor data.
  class VisualizeLidar : public ignition::gazebo::GuiSystem
  {
    Q_OBJECT

    /// \brief Message topic
    Q_PROPERTY(
      QString topicName
      READ TopicName
      WRITE SetTopicName
      NOTIFY TopicNameChanged
    )

    /// \brief Min Range
    Q_PROPERTY(
      QString minRange
      READ MinRange
      NOTIFY MinRangeChanged
    )

    /// \brief Max Range
    Q_PROPERTY(
      QString maxRange
      READ MaxRange
      NOTIFY MaxRangeChanged
    )

    /// \brief Constructor
    public: VisualizeLidar();

    /// \brief Destructor
    public: ~VisualizeLidar() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation Inherited
    public: bool eventFilter(QObject *_obj, QEvent *_event);

    // Documentation inherited
    public: void Update(const UpdateInfo &,
                        EntityComponentManager &_ecm) override;

    /// \brief Callback function to get data from the message
    /// \param[in]_msg LidarSensor message
    public: void OnScan(const msgs::LaserScan &_msg);

    /// \brief Reset and clear visual
    void ResetLidarVisual();

    /// \brief Load the scene and attach LidarVisual to the scene
    public: void LoadLidar();

    /// \brief Set visual type of LidarVisual
    /// \param[in] _type Index of selected visual type
    public slots: void UpdateType(int _type);

    /// \brief Set topic to subscribe for LidarSensor data
    /// \param[in] _topicName Name of selected topic
    public: Q_INVOKABLE void SetTopicName(const QString &_topicName);

    /// \brief Get the name of the topic currently beign visualised.
    /// \return Name such as "/lidar"
    public: Q_INVOKABLE QString TopicName() const;

    /// \brief Set whether to display non-hitting rays
    /// \param[in] _value Boolean value for displaying non hitting rays
    public slots: void UpdateNonHitting(bool _value);

    /// \brief Notify that topic name has changed
    signals: void TopicNameChanged();

    /// \brief Notify that topic name has changed
    signals: void MinRangeChanged();

    /// \brief Notify that topic name has changed
    signals: void MaxRangeChanged();

    /// \brief Get the maximum range of the lidar sensor (in metres).
    /// \return Range, the maximum distance sensed by the sensor.
    public: Q_INVOKABLE QString MaxRange() const;

    /// \brief Get the minimum range of the lidar sensor (in metres).
    /// \return Range, the minimum distance sensed by the sensor.
    public: Q_INVOKABLE QString MinRange() const;

    /// \internal
    /// \brief Pointer to private data
    private: std::unique_ptr<VisualizeLidarPrivate> dataPtr;
  };
}
}
}
#endif
