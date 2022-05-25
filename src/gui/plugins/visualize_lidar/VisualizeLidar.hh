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

#ifndef GZ_SIM_GUI_VISUALIZELIDAR_HH_
#define GZ_SIM_GUI_VISUALIZELIDAR_HH_

#include <memory>

#include "gz/msgs/laserscan.pb.h"
#include "gz/sim/gui/GuiSystem.hh"
#include "gz/gui/qt.h"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE
{
  class VisualizeLidarPrivate;

  /// \brief Visualize the LaserScan message returned by the sensors. Use the
  /// checkbox to turn visualization of non-hitting rays on or off and
  /// the textfield to select the message to be visualised. The combobox is
  /// used to select the type of visual for the sensor data.
  class VisualizeLidar : public gz::sim::GuiSystem
  {
    Q_OBJECT

    /// \brief Topic list
    Q_PROPERTY(
      QStringList topicList
      READ TopicList
      WRITE SetTopicList
      NOTIFY TopicListChanged
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
    public: bool eventFilter(QObject *_obj, QEvent *_event) override;

    // Documentation inherited
    public: void Update(const UpdateInfo &,
                        EntityComponentManager &_ecm) override;

    /// \brief Callback function to get data from the message
    /// \param[in]_msg LidarSensor message
    public: void OnScan(const msgs::LaserScan &_msg);

    /// \brief Load the scene and attach LidarVisual to the scene
    public: void LoadLidar();

    /// \brief Set visual type of LidarVisual
    /// \param[in] _type Index of selected visual type
    public: Q_INVOKABLE void UpdateType(int _type);

    /// \brief Set lidar visualization size
    /// \param[in] _size Size of lidar visualization
    public: Q_INVOKABLE void UpdateSize(int _size);

    /// \brief Get the topic list as a string
    /// \return Message type
    public: Q_INVOKABLE QStringList TopicList() const;

    /// \brief Set the topic list from a string, for example
    /// 'gz.msgs.StringMsg'
    /// \param[in] _topicList Message type
    public: Q_INVOKABLE void SetTopicList(const QStringList &_topicList);

    /// \brief Notify that topic list has changed
    signals: void TopicListChanged();

    /// \brief Set topic to subscribe for LidarSensor data
    /// \param[in] _topicName Name of selected topic
    public: Q_INVOKABLE void OnTopic(const QString &_topicName);

    /// \brief Set whether to display non-hitting rays
    /// \param[in] _value Boolean value for displaying non hitting rays
    public: Q_INVOKABLE void UpdateNonHitting(bool _value);

    /// \brief Set whether to display the lidar visual
    /// \param[in] _value Boolean value for displaying the visual
    public: Q_INVOKABLE void DisplayVisual(bool _value);

    /// \brief Callback when refresh button is pressed.
    public: Q_INVOKABLE void OnRefresh();

    /// \brief Notify that minimum range has changed
    signals: void MinRangeChanged();

    /// \brief Notify that maximum range has changed
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
