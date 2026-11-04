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
#include <string>

#include "gz/msgs/camera_info.pb.h"
#include "gz/msgs/header.pb.h"
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

  /// \brief Visualize the frustum from supported camera sensor messages.
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

    /// \brief Frustum parameters extracted from a supported sensor message.
    public: struct FrustumData
    {
      /// \brief Near clip plane
      double nearClip{0.1};

      /// \brief Far clip plane
      double farClip{10.0};

      /// \brief Horizontal field of view in radians
      double horizontalFov{1.0471975512};

      /// \brief Aspect ratio
      double aspectRatio{1.3333333333};

      /// \brief Frame id used to resolve the sensor entity
      std::string frameId;

      /// \brief Whether this data is valid
      bool valid{false};
    };

    /// \brief Constructor
    public: VisualizeFrustum();

    /// \brief Destructor
    public: ~VisualizeFrustum() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited
    public: bool eventFilter(QObject *_obj, QEvent *_event) override;

    // Documentation inherited
    public: void Update(const UpdateInfo &,
                        EntityComponentManager &_ecm) override;

    /// \brief Callback function to get data from a LogicalCameraSensor message.
    /// \param[in] _msg LogicalCameraSensor message with frustum information
    public: void OnScan(const msgs::LogicalCameraSensor &_msg);

    /// \brief Callback function to get data from a CameraInfo message.
    /// \param[in] _msg CameraInfo message used to compute frustum information
    public: void OnCameraInfo(const msgs::CameraInfo &_msg);

    /// \brief Load the scene and attach FrustumVisual to the scene
    public: void LoadFrustum();

    /// \brief Get the list of topics as a string list
    /// \return Supported topic list
    public: Q_INVOKABLE QStringList TopicList() const;

    /// \brief Set the list of topics
    /// \param[in] _topicList Topic list
    public: Q_INVOKABLE void SetTopicList(const QStringList &_topicList);

    /// \brief Notify that topic list has changed
    signals: void TopicListChanged();

    /// \brief Set topic to subscribe for frustum data
    /// \param[in] _topicName Name of selected topic
    public: Q_INVOKABLE void OnTopic(const QString &_topicName);

    /// \brief Set whether to display the frustum visual
    /// \param[in] _value Boolean value for displaying the visual
    public: Q_INVOKABLE void DisplayVisual(bool _value);

    /// \brief Callback when refresh button is pressed.
    public: Q_INVOKABLE void OnRefresh();

    /// \brief Get frame id from a message header.
    /// \param[in] _header Message header
    /// \return Frame id string, empty if not found
    private: std::string FrameIdFromHeader(const msgs::Header &_header) const;

    /// \brief Extract frustum data from a LogicalCameraSensor message.
    /// \param[in] _msg Logical camera message
    /// \param[out] _data Extracted frustum data
    /// \return True if frustum data was extracted successfully
    private: bool FrustumDataFromLogicalCamera(
        const msgs::LogicalCameraSensor &_msg,
        FrustumData &_data) const;

    /// \brief Extract frustum data from a CameraInfo message.
    /// \param[in] _msg Camera info message
    /// \param[out] _data Extracted frustum data
    /// \return True if frustum data was extracted successfully
    private: bool FrustumDataFromCameraInfo(
        const msgs::CameraInfo &_msg,
        FrustumData &_data) const;

    /// \brief Apply frustum parameters to the rendering visual.
    /// \param[in] _data Frustum data
    private: void ApplyFrustumData(const FrustumData &_data);

    /// \internal
    /// \brief Pointer to private data
    private: std::unique_ptr<VisualizeFrustumPrivate> dataPtr;
  };
}
}
}
#endif
