/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#ifndef GZ_SIM_GUI_VISUALIZEFRUSTUM_HH_
#define GZ_SIM_GUI_VISUALIZEFRUSTUM_HH_

#include <memory>

#include "gz/msgs/logical_camera_sensor.pb.h"
#include "gz/sim/gui/GuiSystem.hh"
#include "gz/gui/qt.h"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE
{
  class VisualizeFrustumPrivate;

  /// \brief Visualize the frustum from a LogicalCameraSensor message.
  class VisualizeFrustum : public gz::sim::GuiSystem
  {
    Q_OBJECT

    /// \brief Topic list
    Q_PROPERTY(
      QStringList topicList
      READ TopicList
      WRITE SetTopicList
      NOTIFY TopicListChanged
    )

    /// \brief Constructor
    public: VisualizeFrustum();

    /// \brief Destructor
    public: ~VisualizeFrustum() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation Inherited
    public: bool eventFilter(QObject *_obj, QEvent *_event) override;

    // Documentation inherited
    public: void Update(const UpdateInfo &,
                        EntityComponentManager &_ecm) override;

    /// \brief Callback function to get data from the message
    /// \param[in] _msg LogicalCameraSensor message with frustum information
    public: void OnScan(const msgs::LogicalCameraSensor &_msg);

    /// \brief Load the scene and attach FrustumVisual to the scene
    public: void LoadFrustum();

    /// \brief Get the list of topics as a string
    /// \return Message type
    public: Q_INVOKABLE QStringList TopicList() const;

    /// \brief Set the list of topics from a string, for example
    /// 'gz.msgs.StringMsg'
    /// \param[in] _topicList Message type
    public: Q_INVOKABLE void SetTopicList(const QStringList &_topicList);

    /// \brief Notify that topic list has changed
    signals: void TopicListChanged();

    /// \brief Set topic to subscribe for FrustumSensor data
    /// \param[in] _topicName Name of selected topic
    public: Q_INVOKABLE void OnTopic(const QString &_topicName);

    /// \brief Set whether to display the frustum visual
    /// \param[in] _value Boolean value for displaying the visual
    public: Q_INVOKABLE void DisplayVisual(bool _value);

    /// \brief Callback when refresh button is pressed.
    public: Q_INVOKABLE void OnRefresh();

    /// \internal
    /// \brief Pointer to private data
    private: std::unique_ptr<VisualizeFrustumPrivate> dataPtr;
  };
}
}
}
#endif
