/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef IGNITION_GAZEBO_GUI_SCENE3D_HH_
#define IGNITION_GAZEBO_GUI_SCENE3D_HH_

#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/gui_camera.pb.h>
#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/msgs/vector3d.pb.h>
#include <ignition/msgs/video_record.pb.h>

#include <string>
#include <memory>
#include <mutex>

#include <sdf/Root.hh>

#include <ignition/math/Color.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include <ignition/common/MouseEvent.hh>
#include <ignition/common/KeyEvent.hh>

#include <ignition/rendering/Camera.hh>

#include <ignition/gazebo/gui/GuiSystem.hh>

#include "ignition/gui/qt.h"


namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
  class IgnRendererPrivate;
  class RenderWindowItemPrivate;
  class Scene3DPrivate;
  class RenderUtil;

  /// \brief Creates a new ignition rendering scene or adds a user-camera to an
  /// existing scene. It is possible to orbit the camera around the scene with
  /// the mouse. Use other plugins to manage objects in the scene.
  ///
  /// ## Configuration
  ///
  /// * \<engine\> : Optional render engine name, defaults to 'ogre'.
  /// * \<scene\> : Optional scene name, defaults to 'scene'. The plugin will
  ///               create a scene with this name if there isn't one yet. If
  ///               there is already one, a new camera is added to it.
  /// * \<ambient_light\> : Optional color for ambient light, defaults to
  ///                       (0.3, 0.3, 0.3, 1.0)
  /// * \<background_color\> : Optional background color, defaults to
  ///                          (0.3, 0.3, 0.3, 1.0)
  /// * \<camera_pose\> : Optional starting pose for the camera, defaults to
  ///                     (0, 0, 5, 0, 0, 0)
  /// * \<camera_follow\> :
  ///     * \<p_gain\>    : Camera follow movement p gain.
  ///     * \<target\>    : Target to follow.
  /// * \<fullscreen\> : Optional starting the window in fullscreen.
  class Scene3D : public ignition::gazebo::GuiSystem
  {
    Q_OBJECT

    /// \brief Text for popup error
    Q_PROPERTY(
      QString errorPopupText
      READ ErrorPopupText
      WRITE SetErrorPopupText
      NOTIFY ErrorPopupTextChanged
    )

    /// \brief Constructor
    public: Scene3D();

    /// \brief Destructor
    public: ~Scene3D() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited
    public: void Update(const UpdateInfo &_info,
        EntityComponentManager &_ecm) override;

    /// \brief Callback when receives a drop event.
    /// \param[in] _drop Dropped string.
    /// \param[in] _mouseX x coordinate of mouse position.
    /// \param[in] _mouseY y coordinate of mouse position.
    public slots: void OnDropped(const QString &_drop,
        int _mouseX, int _mouseY);

    /// \brief Callback when the mouse hovers to a new position.
    /// \param[in] _mouseX x coordinate of the hovered mouse position.
    /// \param[in] _mouseY y coordinate of the hovered mouse position.
    public slots: void OnHovered(int _mouseX, int _mouseY);

    /// \brief Callback when the mouse enters the render window to
    /// focus the window for mouse/key events
    public slots: void OnFocusWindow();

    // Documentation inherited
    protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

    /// \brief Callback for a transform mode request
    /// \param[in] _msg Request message to set a new transform mode
    /// \param[in] _res Response data
    /// \return True if the request is received
    private: bool OnTransformMode(const msgs::StringMsg &_msg,
        msgs::Boolean &_res);

    /// \brief Callback for a record video request
    /// \param[in] _msg Request message to enable/disable video recording.
    /// \param[in] _res Response data
    /// \return True if the request is received
    private: bool OnRecordVideo(const msgs::VideoRecord &_msg,
        msgs::Boolean &_res);

    /// \brief Callback for a move to request
    /// \param[in] _msg Request message to set the target to move to.
    /// \param[in] _res Response data
    /// \return True if the request is received
    private: bool OnMoveTo(const msgs::StringMsg &_msg,
        msgs::Boolean &_res);

    /// \brief Callback for a follow request
    /// \param[in] _msg Request message to set the target to follow.
    /// \param[in] _res Response data
    /// \return True if the request is received
    private: bool OnFollow(const msgs::StringMsg &_msg,
        msgs::Boolean &_res);

    /// \brief Callback for a follow offset request
    /// \param[in] _msg Request message to set the camera's follow offset.
    /// \param[in] _res Response data
    /// \return True if the request is received
    private: bool OnFollowOffset(const msgs::Vector3d &_msg,
        msgs::Boolean &_res);

    /// \brief Callback for a view angle request
    /// \param[in] _msg Request message to set the camera to.
    /// \param[in] _res Response data
    /// \return True if the request is received
    private: bool OnViewAngle(const msgs::Vector3d &_msg,
        msgs::Boolean &_res);

    /// \brief Callback for a move to pose request.
    /// \param[in] _msg GUICamera request message.
    /// \param[in] _res Response data
    /// \return True if the request is received
    private: bool OnMoveToPose(const msgs::GUICamera &_msg,
                 msgs::Boolean &_res);

    /// \brief Callback for view collisions request
    /// \param[in] _msg Request message to set the target to view collisions
    /// \param[in] _res Response data
    /// \return True if the request is received
    private: bool OnViewCollisions(const msgs::StringMsg &_msg,
        msgs::Boolean &_res);

    /// \brief Get the text for the popup error message
    /// \return The error text
    public: Q_INVOKABLE QString ErrorPopupText() const;

    /// \brief Set the text for the popup error message
    /// \param[in] _errorTxt The error text
    public: Q_INVOKABLE void SetErrorPopupText(const QString &_errorTxt);

    /// \brief Notify the popup error text has changed
    signals: void ErrorPopupTextChanged();

    /// \brief Notify that an error has occurred (opens popup)
    /// Note that the function name needs to start with lowercase in order for
    /// the connection to work on the QML side
    signals: void popupError();

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<Scene3DPrivate> dataPtr;
  };

  class RenderSync;

  /// \brief Ign-rendering renderer.
  /// All ign-rendering calls should be performed inside this class as it makes
  /// sure that opengl calls in the underlying render engine do not interfere
  /// with QtQuick's opengl render operations. The main Render function will
  /// render to an offscreen texture and notify via signal and slots when it's
  /// ready to be displayed.
  class IgnRenderer : public QObject
  {
    Q_OBJECT

    ///  \brief Constructor
    public: IgnRenderer();

    ///  \brief Destructor
    public: ~IgnRenderer() override;

    ///  \brief Main render function
    /// \param[in] _renderSync RenderSync to safely
    /// synchronize Qt and worker thread (this)
    public: void Render(RenderSync *_renderSync);

    /// \brief Initialize the render engine
    public: void Initialize();

    /// \brief Destroy camera associated with this renderer
    public: void Destroy();

    /// \brief Set the renderer
    public: class RenderUtil *RenderUtil() const;

    /// \brief Set the transform mode
    /// \param[in] _mode New transform mode to set to
    public: void SetTransformMode(const std::string &_mode);

    /// \brief Set the model to hover over the scene.
    /// \param[in] _model Sdf string of the model to load in for the user.
    public: void SetModel(const std::string &_model);

    /// \brief Set the path to the model to hover over the scene.
    /// \param[in] _filePath Sdf path of the model to load in for the user.
    public: void SetModelPath(const std::string &_filePath);

    /// \brief Set if the dropdown menu is enabled or disabled.
    /// \param[in] _enableDropdownMenu The boolean to enable or disable
    /// the dropdown menu
    public: void SetDropdownMenuEnabled(bool _enableDropdownMenu);

    /// \brief Set whether to record video
    /// \param[in] _record True to start video recording, false to stop.
    /// \param[in] _format Video encoding format: "mp4", "ogv"
    /// \param[in] _savePath Path to save the recorded video.
    public: void SetRecordVideo(bool _record, const std::string &_format,
        const std::string &_savePath);

    /// \brief Set whether to record video using sim time as timestamp
    /// \param[in] _true True record video using sim time
    public: void SetRecordVideoUseSimTime(bool _useSimTime);

    /// \brief Set whether to record video in lockstep mode
    /// \param[in] _true True to record video in lockstep mode
    public: void SetRecordVideoLockstep(bool _lockstep);

    /// \brief Set video recorder bitrate in bps
    /// \param[in] _bitrate Bit rate to set to
    public: void SetRecordVideoBitrate(unsigned int _bitrate);

    /// \brief Move the user camera to move to the speficied target
    /// \param[in] _target Target to move the camera to
    public: void SetMoveTo(const std::string &_target);

    /// \brief Move the user camera to follow the speficied target
    /// \param[in] _target Target to follow
    /// \param[in] _waitForTarget True to continuously look for the target
    /// to follow. A typical use case is when following a target that is not
    ///  present on startup but spawned later into simulation
    public: void SetFollowTarget(const std::string &_target,
        bool _waitForTarget = false);

    /// \brief Set the viewing angle of the camera
    /// \param[in] _direction The pose to assume relative to the entit(y/ies).
    /// (0, 0, 0) indicates to return the camera back to the home pose
    /// originally loaded in from the sdf
    public: void SetViewAngle(const math::Vector3d &_direction);

    /// \brief Set the world pose of the camera
    /// \param[in] _pose The world pose to set the camera to.
    public: void SetMoveToPose(const math::Pose3d &_pose);

    /// \brief View collisions of the specified target
    /// \param[in] _target Target to view collisions
    public: void SetViewCollisionsTarget(const std::string &_target);

    /// \brief Set the p gain for the camera follow movement
    /// \param[in] _gain Camera follow p gain.
    public: void SetFollowPGain(double _gain);

    /// \brief True to set the camera to follow the target in world frame,
    /// false to follow in target's local frame
    /// \param[in] _gain Camera follow p gain.
    public: void SetFollowWorldFrame(bool _worldFrame);

    /// \brief Set the camera follow offset position
    /// \param[in] _offset Camera follow offset position.
    public: void SetFollowOffset(const math::Vector3d &_offset);

    /// \brief Get the target which the user camera is following
    /// \return Target being followed
    public: std::string FollowTarget() const;

    /// \brief Get whether the camera is following the entity in world frame.
    /// \return True if follow mode is in world frame, false if local frame
    public: bool FollowWorldFrame() const;

    /// \brief Get the camera follow offset position
    /// \return Camera follow offset position.
    public: math::Vector3d FollowOffset() const;

    /// \brief Set the initial user camera pose
    /// \param[in] _pose Pose to set the camera to
    public: void SetInitCameraPose(const math::Pose3d &_pose);

    /// \brief New mouse event triggered
    /// \param[in] _e New mouse event
    /// \param[in] _drag Mouse move distance
    public: void NewMouseEvent(const common::MouseEvent &_e,
        const math::Vector2d &_drag = math::Vector2d::Zero);

    /// \brief New hover event triggered.
    /// \param[in] _hoverPos Mouse hover screen position
    public: void NewHoverEvent(const math::Vector2i &_hoverPos);

    /// \brief Handle key press event for snapping
    /// \param[in] _e The key event to process.
    public: void HandleKeyPress(QKeyEvent *_e);

    /// \brief Handle key release event for snapping
    /// \param[in] _e The key event to process.
    public: void HandleKeyRelease(QKeyEvent *_e);

    /// \brief Set the XYZ snap values.
    /// \param[in] _xyz The XYZ snap values
    public: void SetXYZSnap(const math::Vector3d &_xyz);

    /// \brief Get the XYZ snap values.
    /// \return XYZ snapping values as a Vector3d
    public: math::Vector3d XYZSnap() const;

    /// \brief Set the RPY snap values.
    /// \param[in] _rpy The RPY snap values
    public: void SetRPYSnap(const math::Vector3d &_rpy);

    /// \brief Get the RPY snap values.
    /// \return RPY snapping values as a Vector3d
    public: math::Vector3d RPYSnap() const;

    /// \brief Set the scale snap values.
    /// \param[in] _scale The scale snap values
    public: void SetScaleSnap(const math::Vector3d &_scale);

    /// \brief Get the scale snap values.
    /// \return Scale snapping values as a Vector3d
    public: math::Vector3d ScaleSnap() const;

    /// \brief Snaps a point at intervals of a fixed distance. Currently used
    /// to give a snapping behavior when moving models with a mouse.
    /// \param[in] _point Input point to snap.
    /// \param[in] _snapVals The snapping values to use for each corresponding
    /// coordinate in _point
    /// \param[in] _sensitivity Sensitivity of a point snapping, in terms of a
    /// percentage of the interval.
    public: void SnapPoint(
                math::Vector3d &_point,
                math::Vector3d &_snapVals, double _sensitivity = 0.4) const;

    /// \brief Request entity selection. This queues the selection to be handled
    /// later in the render thread.
    /// \param[in] _selectedEntity Entity to be selected, or `kNullEntity`.
    /// \param[in] _deselectAll True if all entities should be deselected.
    /// \param[in] _sendEvent True to emit an event to other widgets.
    public: void RequestSelectionChange(Entity _selectedEntity,
        bool _deselectAll, bool _sendEvent);

    /// \brief Snaps a value at intervals of a fixed distance. Currently used
    /// to give a snapping behavior when moving models with a mouse.
    /// \param[in] _coord Input coordinate point.
    /// \param[in] _interval Fixed distance interval at which the point is
    /// snapped.
    /// \param[in] _sensitivity Sensitivity of a point snapping, in terms of a
    /// percentage of the interval.
    /// \return Snapped coordinate point.
    private: double SnapValue(
                 double _coord, double _interval, double _sensitivity) const;

    /// \brief Constraints the passed in axis to the currently selected axes.
    /// \param[in] _axis The axis to constrain.
    private: void XYZConstraint(math::Vector3d &_axis);

    /// \brief Handle mouse events
    private: void HandleMouseEvent();

    /// \brief Handle mouse event for context menu
    private: void HandleMouseContextMenu();

    /// \brief Handle mouse event for view control
    private: void HandleMouseViewControl();

    /// \brief Handle mouse event for transform control
    private: void HandleMouseTransformControl();

    /// \brief Handle entity selection requests
    private: void HandleEntitySelection();

    /// \brief Handle model placement requests
    private: void HandleModelPlacement();

    /// \brief Broadcasts the currently hovered 3d scene location.
    private: void BroadcastHoverPos();

    /// \brief Broadcasts a left click within the scene
    private: void BroadcastLeftClick();

    /// \brief Broadcasts a right click within the scene
    private: void BroadcastRightClick();

    /// \brief Generate a unique entity id.
    /// \return The unique entity id
    private: Entity UniqueId();

    /// \brief Generate a preview of a resource.
    /// \param[in] _sdf The SDF to be previewed.
    /// \return True on success, false if failure
    public: bool GeneratePreview(const sdf::Root &_sdf);

    /// \brief Delete the visuals generated while an entity is being spawned.
    public: void TerminateSpawnPreview();

    /// \brief Retrieve the point on a plane at z = 0 in the 3D scene hit by a
    /// ray cast from the given 2D screen coordinates.
    /// \param[in] _screenPod 2D coordinates on the screen, in pixels.
    /// \return 3D coordinates of a point in the 3D scene.
    public: math::Vector3d ScreenToPlane(const math::Vector2i &_screenPos)
        const;

    /// \brief Retrieve the first point on a surface in the 3D scene hit by a
    /// ray cast from the given 2D screen coordinates.
    /// \param[in] _screenPos 2D coordinates on the screen, in pixels.
    /// \return 3D coordinates of a point in the 3D scene.
    public: math::Vector3d ScreenToScene(const math::Vector2i &_screenPos)
        const;

    /// \brief Get the current camera pose.
    /// \return Pose of the camera.
    public: math::Pose3d CameraPose() const;

    /// \brief Callback when a move to animation is complete
    private: void OnMoveToComplete();

    /// \brief Callback when a view angle animation is complete
    private: void OnViewAngleComplete();

    /// \brief Callback when a move to  pose animation is complete
    private: void OnMoveToPoseComplete();

    /// \brief Process a node's selection
    /// \param[in] _node The node to be selected.
    /// \param[in] _sendEvent True to notify other widgets. This should be true
    /// when the selection is initiated from this plugin.
    private: void UpdateSelectedEntity(const rendering::NodePtr &_node,
        bool _sendEvent);

    /// \brief Deselect all entities.
    /// \param[in] _sendEvent True to notify other widgets. This should be true
    /// when the deselection is initiated from this plugin.
    private: void DeselectAllEntities(bool _sendEvent);

    /// \brief Signal fired when context menu event is triggered
    signals: void ContextMenuRequested(QString _entity);

    /// \brief When fired, the follow target changed. May not be fired for
    /// every target change.
    /// \param[in] _target Target to follow
    /// \param[in] _waitForTarget True to continuously look for the target
    signals: void FollowTargetChanged(const std::string &_target,
        bool _waitForTarget);

    /// \brief Render texture id
    /// Values is constantly constantly cycled/swapped/changed
    /// from a worker thread
    /// Don't read this directly
    public: GLuint textureId;

    /// \brief Initial Camera pose
    public: math::Pose3d cameraPose = math::Pose3d(0, 0, 2, 0, 0.4, 0);

    /// \brief Name of the world
    public: std::string worldName;

    /// \brief Camera visibility mask
    public: uint32_t visibilityMask = 0xFFFFFFFFu;

    /// \brief True if engine has been initialized;
    public: bool initialized = false;

    /// \brief Render texture size
    public: QSize textureSize = QSize(1024, 1024);

    /// \brief Flag to indicate texture size has changed.
    public: bool textureDirty = false;

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<IgnRendererPrivate> dataPtr;
  };

  /// \brief Rendering thread
  class RenderThread : public QThread
  {
    Q_OBJECT

    /// \brief Constructor
    public: RenderThread();

    /// \brief Render the next frame
    /// \param[in] _renderSync RenderSync to safely
    /// synchronize Qt and worker thread (this)
    public slots: void RenderNext(RenderSync *renderSync);

    /// \brief Shutdown the thread and the render engine
    public slots: void ShutDown();

    /// \brief Slot called to update render texture size
    public slots: void SizeChanged();

    /// \brief Signal to indicate that a frame has been rendered and ready
    /// to be displayed
    /// \param[in] _id GLuid of the opengl texture
    /// \param[in] _size Size of the texture
    signals: void TextureReady(uint _id, const QSize &_size);

    /// \brief Offscreen surface to render to
    public: QOffscreenSurface *surface = nullptr;

    /// \brief OpenGL context to be passed to the render engine
    public: QOpenGLContext *context = nullptr;

    /// \brief Ign-rendering renderer
    public: IgnRenderer ignRenderer;
  };


  /// \brief A QQUickItem that manages the render window
  class RenderWindowItem : public QQuickItem
  {
    Q_OBJECT

    /// \brief Constructor
    /// \param[in] _parent Parent item
    public: explicit RenderWindowItem(QQuickItem *_parent = nullptr);

    /// \brief Destructor
    public: ~RenderWindowItem() override;

    /// \brief Set the renderer
    public: class RenderUtil *RenderUtil() const;

    /// \brief Set the initial user camera pose
    /// \param[in] _pose Pose to set the camera to
    public: void SetCameraPose(const math::Pose3d &_pose);

    /// \brief Get the user camera pose.
    /// \return Pose of the user camera.
    public: math::Pose3d CameraPose() const;

    /// \brief Set the initial user camera pose
    /// \param[in] _pose Pose to set the camera to
    public: void SetInitCameraPose(const math::Pose3d &_pose);

    /// \brief Set the user camera visibility mask
    /// \param[in] _mask Visibility mask to set to
    public: void SetVisibilityMask(uint32_t _mask);

    /// \brief Set the transform mode
    /// \param[in] _mode New transform mode to set to
    public: void SetTransformMode(const std::string &_mode);

    /// \brief Set the model to hover.
    /// \param[in] _model Sdf string of the model to load in for the user.
    public: void SetModel(const std::string &_model);

    /// \brief Set the path of the model to hover.
    /// \param[in] _filePath File path of the model to load in for the user.
    public: void SetModelPath(const std::string &_filePath);

    /// \brief Set if the dropdown menu is enabled or disabled.
    /// \param[in] _enableDropdownMenu The boolean to enable or disable
    /// the menu
    public: void SetDropdownMenuEnabled(bool _enableDropdownMenu);

    /// \brief Set whether to record video
    /// \param[in] _record True to start video recording, false to stop.
    /// \param[in] _format Video encoding format: "mp4", "ogv"
    /// \param[in] _savePath Path to save the recorded video.
    public: void SetRecordVideo(bool _record, const std::string &_format,
        const std::string &_savePath);

    /// \brief Set whether to record video using sim time as timestamp
    /// \param[in] _true True record video using sim time
    public: void SetRecordVideoUseSimTime(bool _useSimTime);

    /// \brief Set whether to record video in lockstep mode
    /// \param[in] _true True to record video in lockstep mode
    public: void SetRecordVideoLockstep(bool _lockstep);

    /// \brief Set video recorder bitrate in bps
    /// \param[in] _bitrate Bit rate to set to
    public: void SetRecordVideoBitrate(unsigned int _bitrate);

    /// \brief Move the user camera to move to the specified target
    /// \param[in] _target Target to move the camera to
    public: void SetMoveTo(const std::string &_target);

    /// \brief Move the user camera to follow the specified target
    /// \param[in] _target Target to follow
    /// \param[in] _waitForTarget True to continuously look for the target
    /// to follow. A typical use case is follow a target that is not present
    /// on startup but spawned later into simulation
    public Q_SLOTS: void SetFollowTarget(const std::string &_target,
        bool _waitForTarget = false);

    /// \brief Set the viewing angle of the camera
    /// \param[in] _direction The pose to assume relative to the entit(y/ies).
    /// (0, 0, 0) indicates to return the camera back to the home pose
    /// originally loaded in from the sdf
    public: void SetViewAngle(const math::Vector3d &_direction);

    /// \brief Set the pose of the camera
    /// \param[in] _pose The new camera pose in the world frame.
    public: void SetMoveToPose(const math::Pose3d &_pose);

    /// \brief View collisions of the specified target
    /// \param[in] _target Target to view collisions
    public: void SetViewCollisionsTarget(const std::string &_target);

    /// \brief Set the p gain for the camera follow movement
    /// \param[in] _gain Camera follow p gain.
    public: void SetFollowPGain(double _gain);

    /// \brief True to set the camera to follow the target in world frame,
    /// false to follow in target's local frame
    /// \param[in] _gain Camera follow p gain.
    public: void SetFollowWorldFrame(bool _worldFrame);

    /// \brief Set the camera follow offset position
    /// \param[in] _offset Camera follow offset position.
    public: void SetFollowOffset(const math::Vector3d &_offset);

    /// \brief Set the world name
    /// \param[in] _name Name of the world to set to.
    public: void SetWorldName(const std::string &_name);

    /// \brief An update function to apply the rules of selection to the
    /// passed in node. The rules are as follows:
    /// - If control is held, append the node to the selected entity list.
    /// - If control is not held, deselect all other entities.
    /// - If the user is currently in a transform mode, attach the control
    ///   to the latest selected node and deselect all others.
    /// Note that this function emits events to update other widgets.
    /// \param[in] _entity The entity to select.
    /// \param[in] _sendEvent True to notify other widgets. This should be true
    /// when the selection is initiated from this plugin.
    public: void UpdateSelectedEntity(Entity _entity,
        bool _sendEvent);

    /// \brief Deselect all the currently selected entities within
    /// the RenderUtil class.
    /// \param[in] _sendEvent True to notify other widgets. This should be true
    /// when the deselection is initiated from this plugin.
    public: void DeselectAllEntities(bool _sendEvent);

    /// \brief Set the XYZ snap values from the user input.
    /// \param[in] _xyz The XYZ snap values
    public: void SetXYZSnap(const math::Vector3d &_xyz);

    /// \brief Set the RPY snap values from the user input.
    /// \param[in] _rpy The RPY snap values
    public: void SetRPYSnap(const math::Vector3d &_rpy);

    /// \brief Set the scale snap values from the user input.
    /// \param[in] _scale The scale snap values
    public: void SetScaleSnap(const math::Vector3d &_scale);

    /// \brief Retrieve the first point on a surface in the 3D scene hit by a
    /// ray cast from the given 2D screen coordinates.
    /// \param[in] _screenPos 2D coordinates on the screen, in pixels.
    /// \return 3D coordinates of a point in the 3D scene.
    public: math::Vector3d ScreenToScene(const math::Vector2i &_screenPos);

    /// \brief Called when the mouse hovers to a new position.
    /// \param[in] _hoverPos 2D coordinates of the hovered mouse position on
    /// the render window.
    public: void OnHovered(const ignition::math::Vector2i &_hoverPos);

    /// \brief Get whether the renderer is initialized. The renderer is
    /// initialized when the context is created and the render thread is
    /// started.
    /// \return True if the renderer is initialized.
    public: bool RendererInitialized() const;

    /// \brief Slot called when thread is ready to be started
    public Q_SLOTS: void Ready();

    /// \brief Handle key press event for snapping
    /// \param[in] _e The key event to process.
    public: void HandleKeyPress(QKeyEvent *_e);

    /// \brief Handle key release event for snapping
    /// \param[in] _e The key event to process.
    public: void HandleKeyRelease(QKeyEvent *_e);

    // Documentation inherited
    protected: void mousePressEvent(QMouseEvent *_e) override;

    // Documentation inherited
    protected: void mouseReleaseEvent(QMouseEvent *_e) override;

    // Documentation inherited
    protected: void mouseMoveEvent(QMouseEvent *_e) override;

    // Documentation inherited
    protected: void wheelEvent(QWheelEvent *_e) override;

    /// \brief Overrides the paint event to render the render engine
    /// camera view
    /// \param[in] _oldNode The node passed in previous updatePaintNode
    /// function. It represents the visual representation of the item.
    /// \param[in] _data The node transformation data.
    /// \return Updated node.
    private: QSGNode *updatePaintNode(QSGNode *_oldNode,
        QQuickItem::UpdatePaintNodeData *_data) override;

    /// \brief Signal fired to open context menu
    /// Note that the function name needs to start with lowercase in order for
    /// the connection to work on the QML side
    /// \param[in] _entity Scoped name of entity.
    signals: void openContextMenu(QString _entity); // NOLINT

    /// \brief Qt callback when context menu request is received
    /// \param[in] _entity Scoped name of entity.
    public slots: void OnContextMenuRequested(QString _entity);

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<RenderWindowItemPrivate> dataPtr;
  };

  /// \brief Texture node for displaying the render texture from ign-renderer
  class TextureNode : public QObject, public QSGSimpleTextureNode
  {
    Q_OBJECT

    /// \brief Constructor
    /// \param[in] _window Parent window
    /// \param[in] _renderSync RenderSync to safely
    /// synchronize Qt (this) and worker thread
    public: explicit TextureNode(QQuickWindow *_window,
                                 RenderSync &_renderSync);

    /// \brief Destructor
    public: ~TextureNode() override;

    /// \brief This function gets called on the FBO rendering thread and will
    ///  store the texture id and size and schedule an update on the window.
    /// \param[in] _id OpenGL render texture Id
    /// \param[in] _size Texture size
    public slots: void NewTexture(uint _id, const QSize &_size);

    /// \brief Before the scene graph starts to render, we update to the
    /// pending texture
    public slots: void PrepareNode();

    /// \brief Signal emitted when the texture is being rendered and renderer
    /// can start rendering next frame
    /// \param[in] _renderSync RenderSync to send to the worker thread
    signals: void TextureInUse(RenderSync *_renderSync);

    /// \brief Signal emitted when a new texture is ready to trigger window
    /// update
    signals: void PendingNewTexture();

    /// \brief OpenGL texture id
    public: uint id = 0;

    /// \brief Texture size
    public: QSize size = QSize(0, 0);

    /// \brief Mutex to protect the texture variables
    public: QMutex mutex;

    /// \brief See RenderSync
    public: RenderSync &renderSync;

    /// \brief Qt's scene graph texture
    public: QSGTexture *texture = nullptr;

    /// \brief Qt quick window
    public: QQuickWindow *window = nullptr;
  };
}
}
}

#endif
