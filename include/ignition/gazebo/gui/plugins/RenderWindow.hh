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

#ifndef IGNITION_GAZEBO_RENDERWINDOW_HH_
#define IGNITION_GAZEBO_RENDERWINDOW_HH_

#include <string>
#include <memory>
#include <mutex>

#include <ignition/math/Color.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include <ignition/common/MouseEvent.hh>

#include <ignition/rendering/Camera.hh>
#include <ignition/rendering/OrbitViewController.hh>

#include "ignition/gui/qt.h"

#include <ignition/gazebo/gui/GuiSystem.hh>

namespace ignition
{
namespace gazebo
{
  class IgnRendererPrivate;
  class RenderWindowItemPrivate;
  class RenderWindowPrivate;

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
  class RenderWindow : public GuiPlugin
  {
    Q_OBJECT

    /// \brief Constructor
    public: RenderWindow();

    /// \brief Destructor
    public: virtual ~RenderWindow();

    // Documentation inherited
    public: virtual void LoadConfig(const tinyxml2::XMLElement *_pluginElem)
        override;

    public: void PostUpdate(const UpdateInfo &_info,
        const EntityComponentManager &_ecm) override;


    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<RenderWindowPrivate> dataPtr;
  };

  /// \brief Ign-rendering renderer.
  /// All ign-rendering calls should be performed inside this class as it makes
  /// sure that opengl calls in the underlying render engine do not interfere
  /// with QtQuick's opengl render operations. The main Render function will
  /// render to an offscreen texture and notify via signal and slots when it's
  /// ready to be displayed.
  class IgnRenderer
  {
    ///  \brief Constructor
    public: IgnRenderer();

    ///  \brief Destructor
    public: ~IgnRenderer();

    ///  \brief Main render function
    public: void Render();

    /// \brief Initialize the render engine
    public: void Initialize();

    /// \brief Destroy camera associated with this renderer
    public: void Destroy();

    /// \brief New mouse event triggered
    /// \param[in] _e New mouse event
    /// \param[in] _drag Mouse move distance
    public: void NewMouseEvent(const common::MouseEvent &_e,
        const math::Vector2d &_drag = math::Vector2d::Zero);

    /// \brief Handle mouse event for view control
    private: void HandleMouseEvent();

    /// \brief Retrieve the first point on a surface in the 3D scene hit by a
    /// ray cast from the given 2D screen coordinates.
    /// \param[in] _screenPos 2D coordinates on the screen, in pixels.
    /// \return 3D coordinates of a point in the 3D scene.
    private: math::Vector3d ScreenToScene(const math::Vector2i &_screenPos)
        const;

    /// \brief Render texture id
    public: GLuint textureId = 0u;

    /// \brief Render engine to use
    public: std::string engineName = "ogre";

    /// \brief Unique scene name
    public: std::string sceneName = "scene";

    /// \brief Initial Camera pose
    public: math::Pose3d cameraPose = math::Pose3d(0, 0, 2, 0, 0.4, 0);

    /// \brief Scene background color
    public: math::Color backgroundColor = math::Color::Black;

    /// \brief Ambient color
    public: math::Color ambientLight = math::Color(0.3, 0.3, 0.3, 1.0);

    /// \brief True if engine has been initialized;
    public: bool initialized = false;

    /// \brief Render texture size
    public: QSize textureSize = QSize(1024, 1024);

    /// \brief Flag to indicate texture size has changed.
    public: bool textureDirty = false;

    /// \brief Scene service. If not empty, a request will be made to get the
    /// scene information using this service and the renderer will populate the
    /// scene based on the response data
    public: std::string sceneService;

    /// \brief Scene pose topic. If not empty, a node will subcribe to this
    /// topic to get pose updates of objects in the scene
    public: std::string poseTopic;

    /// \brief Ign-transport deletion topic name
    public: std::string deletionTopic;

    /// \brief Ign-transport scene topic name
    /// New scene messages will be published to this topic when an entities are
    /// added
    public: std::string sceneTopic;

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
    public: virtual ~RenderWindowItem();

    /// \brief Set background color of render window
    /// \param[in] _color Color of render window background
    public: void SetBackgroundColor(const math::Color &_color);

    /// \brief Set ambient light of render window
    /// \param[in] _ambient Color of ambient light
    public: void SetAmbientLight(const math::Color &_ambient);

    /// \brief Set engine name used to create the render window
    /// \param[in] _name Name of render engine
    public: void SetEngineName(const std::string &_name);

    /// \brief Set name of scene created inside the render window
    /// \param[in] _name Name of scene
    public: void SetSceneName(const std::string &_name);

    /// \brief Set the initial pose the render window camera
    /// \param[in] _pose Initical camera pose
    public: void SetCameraPose(const math::Pose3d &_pose);

    /// \brief Set scene service to use in this render window
    /// A service call will be made using ign-transport to get scene
    /// data using this service
    /// \param[in] _service Scene service name
    public: void SetSceneService(const std::string &_service);

    /// \brief Set pose topic to use for updating objects in the scene
    /// The renderer will subscribe to this topic to get pose messages of
    /// visuals in the scene
    /// \param[in] _topic Pose topic
    public: void SetPoseTopic(const std::string &_topic);

    /// \brief Set deletion topic to use for deleting objects from the scene
    /// The renderer will subscribe to this topic to get notified when entities
    /// in the scene get deleted
    /// \param[in] _topic Deletion topic
    public: void SetDeletionTopic(const std::string &_topic);

    /// \brief Set the scene topic to use for updating objects in the scene
    /// The renderer will subscribe to this topic to get updates scene messages
    /// \param[in] _topic Scene topic
    public: void SetSceneTopic(const std::string &_topic);

    /// \brief Slot called when thread is ready to be started
    public Q_SLOTS: void Ready();

    // Documentation inherited
    protected: virtual void mousePressEvent(QMouseEvent *_e) override;

    // Documentation inherited
    protected: virtual void mouseReleaseEvent(QMouseEvent *_e) override;

    // Documentation inherited
    protected: virtual void mouseMoveEvent(QMouseEvent *_e) override;

    // Documentation inherited
    protected: virtual void wheelEvent(QWheelEvent *_e) override;

    /// \brief Overrides the paint event to render the render engine
    /// camera view
    /// \param[in] _oldNode The node passed in previous updatePaintNode
    /// function. It represents the visual representation of the item.
    /// \param[in] _data The node transformation data.
    /// \return Updated node.
    private: QSGNode *updatePaintNode(QSGNode *_oldNode,
        QQuickItem::UpdatePaintNodeData *_data) override;

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

#endif
