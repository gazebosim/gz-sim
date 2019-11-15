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
#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/msgs/video_record.pb.h>

#include <string>
#include <memory>
#include <mutex>

#include <ignition/math/Color.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include <ignition/common/MouseEvent.hh>

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
  class Scene3D : public ignition::gazebo::GuiSystem
  {
    Q_OBJECT

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
    public slots: void OnDropped(const QString &_drop);

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

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<Scene3DPrivate> dataPtr;
  };

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
    public: void Render();

    /// \brief Initialize the render engine
    public: void Initialize();

    /// \brief Destroy camera associated with this renderer
    public: void Destroy();

    /// \brief Set the renderer
    public: class RenderUtil *RenderUtil() const;

    /// \brief Set the transform mode
    /// \param[in] _mode New transform mode to set to
    public: void SetTransformMode(const std::string &_mode);

    /// \brief Set whether to record video
    /// \param[in] _record True to start video recording, false to stop.
    /// \param[in] _format Video encoding format: "mp4", "ogv"
    /// \param[in] _savePath Path to save the recorded video.
    public: void SetRecordVideo(bool _record, const std::string &_format,
        const std::string &_savePath);

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

    /// \brief New mouse event triggered
    /// \param[in] _e New mouse event
    /// \param[in] _drag Mouse move distance
    public: void NewMouseEvent(const common::MouseEvent &_e,
        const math::Vector2d &_drag = math::Vector2d::Zero);

    /// \brief Handle mouse events
    private: void HandleMouseEvent();

    /// \brief Handle mouse event for context menu
    private: void HandleMouseContextMenu();

    /// \brief Handle mouse event for view control
    private: void HandleMouseViewControl();

    /// \brief Handle mouse event for transform control
    private: void HandleMouseTransformControl();

    /// \brief Retrieve the first point on a surface in the 3D scene hit by a
    /// ray cast from the given 2D screen coordinates.
    /// \param[in] _screenPos 2D coordinates on the screen, in pixels.
    /// \return 3D coordinates of a point in the 3D scene.
    private: math::Vector3d ScreenToScene(const math::Vector2i &_screenPos)
        const;

    /// \brief Callback when a move to animation is complete
    private: void OnMoveToComplete();

    /// \brief Signal fired when context menu event is triggered
    signals: void ContextMenuRequested(QString _entity);

    /// \brief Render texture id
    public: GLuint textureId = 0u;

    /// \brief Initial Camera pose
    public: math::Pose3d cameraPose = math::Pose3d(0, 0, 2, 0, 0.4, 0);

    /// \brief Name of the world
    public: std::string worldName;

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
    public slots: void RenderNext();

    /// \brief Shutdown the thread and the render engine
    public slots: void ShutDown();

    /// \brief Slot called to update render texture size
    public slots: void SizeChanged();

    /// \brief Signal to indicate that a frame has been rendered and ready
    /// to be displayed
    /// \param[in] _id GLuid of the opengl texture
    /// \param[in] _size Size of the texture
    signals: void TextureReady(int _id, const QSize &_size);

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

    /// \brief Set the transform mode
    /// \param[in] _mode New transform mode to set to
    public: void SetTransformMode(const std::string &_mode);

    /// \brief Set whether to record video
    /// \param[in] _record True to start video recording, false to stop.
    /// \param[in] _format Video encoding format: "mp4", "ogv"
    /// \param[in] _savePath Path to save the recorded video.
    public: void SetRecordVideo(bool _record, const std::string &_format,
        const std::string &_savePath);

    /// \brief Move the user camera to move to the speficied target
    /// \param[in] _target Target to move the camera to
    public: void SetMoveTo(const std::string &_target);

    /// \brief Move the user camera to follow the speficied target
    /// \param[in] _target Target to follow
    /// \param[in] _waitForTarget True to continuously look for the target
    /// to follow. A typical use case is follow a target that is not present
    /// on startup but spawned later into simulation
    public: void SetFollowTarget(const std::string &_target,
        bool _waitForTarget = false);

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

    /// \brief Slot called when thread is ready to be started
    public Q_SLOTS: void Ready();

    // Documentation inherited
    protected: void mousePressEvent(QMouseEvent *_e) override;

    // Documentation inherited
    protected: void mouseReleaseEvent(QMouseEvent *_e) override;

    // Documentation inherited
    protected: void mouseMoveEvent(QMouseEvent *_e) override;

    // Documentation inherited
    protected: void wheelEvent(QWheelEvent *_e) override;

    // Documentation inherited
    protected: void keyReleaseEvent(QKeyEvent *_e) override;

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
    public: explicit TextureNode(QQuickWindow *_window);

    /// \brief Destructor
    public: ~TextureNode() override;

    /// \brief This function gets called on the FBO rendering thread and will
    ///  store the texture id and size and schedule an update on the window.
    /// \param[in] _id OpenGL render texture Id
    /// \param[in] _size Texture size
    public slots: void NewTexture(int _id, const QSize &_size);

    /// \brief Before the scene graph starts to render, we update to the
    /// pending texture
    public slots: void PrepareNode();

    /// \brief Signal emitted when the texture is being rendered and renderer
    /// can start rendering next frame
    signals: void TextureInUse();

    /// \brief Signal emitted when a new texture is ready to trigger window
    /// update
    signals: void PendingNewTexture();

    /// \brief OpenGL texture id
    public: int id = 0;

    /// \brief Texture size
    public: QSize size = QSize(0, 0);

    /// \brief Mutex to protect the texture variables
    public: QMutex mutex;

    /// \brief Qt's scene graph texture
    public: QSGTexture *texture = nullptr;

    /// \brief Qt quick window
    public: QQuickWindow *window = nullptr;
  };
}
}
}

#endif
