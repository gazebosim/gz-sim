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

#ifndef GZ_SIM_GUI_VIEWANGLE_HH_
#define GZ_SIM_GUI_VIEWANGLE_HH_

#include <gz/msgs/pose.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/gui_camera.pb.h>

#include <memory>

#include <gz/gui/Plugin.hh>

namespace gz
{
namespace sim
{
  class ViewAnglePrivate;

  /// \brief Provides buttons for viewing angles
  ///
  /// ## Configuration
  /// \<service\> : Set the service to receive view angle requests.
  /// MinimalScene. Defaults to true.
  class ViewAngle : public gz::gui::Plugin
  {
    Q_OBJECT

    /// \brief gui camera pose (QList order is x, y, z, roll, pitch, yaw)
    Q_PROPERTY(
      QList<double> camPose
      READ CamPose
      NOTIFY CamPoseChanged
    )

    /// \brief gui camera near/far clipping distance (QList order is near, far)
    Q_PROPERTY(
      QList<double> camClipDist
      READ CamClipDist
      NOTIFY CamClipDistChanged
    )

    /// \brief view controller index for qml side (0: orbit; 1: ortho)
    Q_PROPERTY(
      int viewControlIndex
      READ ViewControlIndex
      NOTIFY ViewControlIndexChanged
    )

    /// \brief gui camera horizontal fov
    Q_PROPERTY(
      double horizontalFOV
      READ HorizontalFOV
      NOTIFY CamHorizontalFOVChanged
    )

    /// \brief Constructor
    public: ViewAngle();

    /// \brief Destructor
    public: ~ViewAngle() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited
    private: bool eventFilter(QObject *_obj, QEvent *_event) override;

    /// \brief Callback in Qt thread when angle mode changes.
    /// \param[in] _x The x component of the directional vector for the camera
    /// to assume.  All 0s for x, y, and z indicate the initial camera pose.
    /// \param[in] _y The y component of the directional vector for the camera
    /// to assume.  All 0s for x, y, and z indicate the initial camera pose.
    /// \param[in] _z The z component of the directional vector for the camera
    /// to assume.  All 0s for x, y, and z indicate the initial camera pose.
    public slots: void OnAngleMode(int _x, int _y, int _z);

    /// \brief Callback in Qt thread when camera view controller changes.
    /// \param[in] _mode New camera view controller
    public slots: void OnViewControl(const QString &_controller);

    /// \brief Callback in Qt thread when camera view reference visual state
    /// changes.
    /// \param[in] _enable True to enable camera view control reference visual,
    /// false to hide it
    public slots: void OnViewControlReferenceVisual(bool _enable);

    /// \brief Callback in Qt thread when camera view controller changes.
    /// \param[in] _sensitivity View control sensitivity vlaue
    public slots: void OnViewControlSensitivity(double _sensitivity);

    /// \brief Updates gui camera's Horizontal fov
    /// \param[in] _horizontalFOV Horizontal fov
    public slots: void SetHorizontalFOV(double _horizontalFOV);

    /// \brief Get the current gui horizontal fov.
    public: Q_INVOKABLE double HorizontalFOV() const;

    /// \brief Notify that the gui camera's horizontal fov changed
    signals: void CamHorizontalFOVChanged();

    /// \brief Get the current gui camera pose.
    public: Q_INVOKABLE QList<double> CamPose() const;

    /// \brief Notify that the gui camera pose has changed.
    signals: void CamPoseChanged();

    /// \brief Callback to update gui camera pose
    /// \param[in] _x, _y, _z cartesion coordinates
    /// \param[in] _roll, _pitch, _yaw principal coordinates
    public slots: void SetCamPose(double _x, double _y, double _z,
                               double _roll, double _pitch, double _yaw);

    /// \brief Callback for retrieving gui camera pose
    /// \param[in] _msg Pose message
    public: void CamPoseCb(const msgs::Pose &_msg);

    /// \brief Move to model service received
    /// \param[in] _msg GUI camera message
    /// \param[in] _res Response
    public: bool OnMoveToModelService(const gz::msgs::GUICamera &_msg,
      gz::msgs::Boolean &_res);

    /// \brief Get the current gui camera's near and far clipping distances
    public: Q_INVOKABLE QList<double> CamClipDist() const;

    /// \brief Notify that the gui camera's near/far clipping distances changed
    signals: void CamClipDistChanged();

    /// \brief Updates gui camera's near/far clipping distances
    /// \param[in] _near Near clipping plane distance
    /// \param[in] _far Far clipping plane distance
    public slots: void SetCamClipDist(double _near, double _far);

    /// \brief Get the current index for view controller (on qml side)
    public: int ViewControlIndex() const;

    /// \brief Notify that the camera's view controller has changed
    signals: void ViewControlIndexChanged();

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<ViewAnglePrivate> dataPtr;
  };
}
}

#endif
