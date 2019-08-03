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

#include <cmath>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <ignition/common/Console.hh>
#include <ignition/common/MeshManager.hh>
#include <ignition/common/Profiler.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include <ignition/rendering/OrbitViewController.hh>
#include <ignition/rendering/RayQuery.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/TransformController.hh>

#include <ignition/transport/Node.hh>

#include <ignition/gui/Conversions.hh>
#include <ignition/gui/Application.hh>

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/rendering/RenderUtil.hh"

#include "Scene3D.hh"

namespace ignition
{
namespace gazebo
{
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
  /// \brief Private data class for IgnRenderer
  class IgnRendererPrivate
  {
    /// \brief Flag to indicate if mouse event is dirty
    public: bool mouseDirty = false;

    /// \brief Mouse event
    public: common::MouseEvent mouseEvent;

    /// \brief Mouse move distance since last event.
    public: math::Vector2d drag;

    /// \brief Mutex to protect mouse events
    public: std::mutex mutex;

    /// \brief User camera
    public: rendering::CameraPtr camera;

    /// \brief Camera orbit controller
    public: rendering::OrbitViewController viewControl;

    /// \brief Transform controller for models
    public: rendering::TransformController transformControl;

    /// \brief Transform space: local or world
    public: rendering::TransformSpace transformSpace =
        rendering::TransformSpace::TS_LOCAL;

    /// \brief Transform mode: none, translation, rotation, or scale
    public: rendering::TransformMode transformMode =
        rendering::TransformMode::TM_NONE;

    /// \brief Ray query for mouse clicks
    public: rendering::RayQueryPtr rayQuery;

    /// \brief View control focus target
    public: math::Vector3d target;

    /// \brief Rendering utility
    public: RenderUtil renderUtil;

    /// \brief Transport node for making transform control requests
    public: transport::Node node;

    /// \brief Name of service for setting entity pose
    public: std::string poseCmdService;
  };

  /// \brief Private data class for RenderWindowItem
  class RenderWindowItemPrivate
  {
    /// \brief Keep latest mouse event
    public: common::MouseEvent mouseEvent;

    /// \brief Render thread
    public : RenderThread *renderThread = nullptr;

    //// \brief List of threads
    public: static QList<QThread *> threads;
  };

  /// \brief Private data class for Scene3D
  class Scene3DPrivate
  {
    /// \brief Transport node
    public: transport::Node node;

    /// \brief Name of the world
    public: std::string worldName;

    /// \brief Rendering utility
    public: RenderUtil *renderUtil = nullptr;

    /// \brief Transform mode service
    public: std::string transformModeService;
  };
}
}
}

using namespace ignition;
using namespace gazebo;

QList<QThread *> RenderWindowItemPrivate::threads;

/////////////////////////////////////////////////
IgnRenderer::IgnRenderer()
  : dataPtr(new IgnRendererPrivate)
{
}


/////////////////////////////////////////////////
IgnRenderer::~IgnRenderer() = default;

////////////////////////////////////////////////
RenderUtil *IgnRenderer::RenderUtil() const
{
  return &this->dataPtr->renderUtil;
}

/////////////////////////////////////////////////
void IgnRenderer::Render()
{
  IGN_PROFILE_THREAD_NAME("RenderThread");
  IGN_PROFILE("IgnRenderer::Render");
  if (this->textureDirty)
  {
    this->dataPtr->camera->SetImageWidth(this->textureSize.width());
    this->dataPtr->camera->SetImageHeight(this->textureSize.height());
    this->dataPtr->camera->SetAspectRatio(this->textureSize.width() /
        static_cast<double>(this->textureSize.height()));
    // setting the size should cause the render texture to be rebuilt
    {
      IGN_PROFILE("IgnRenderer::Render Pre-render camera");
      this->dataPtr->camera->PreRender();
    }
    this->textureId = this->dataPtr->camera->RenderTextureGLId();
    this->textureDirty = false;
  }

  // update the scene
  this->dataPtr->renderUtil.SetTransformActive(
      this->dataPtr->transformControl.Active());
  this->dataPtr->renderUtil.Update();

  // view control
  this->HandleMouseEvent();

  // update and render to texture
  {
    IGN_PROFILE("IgnRenderer::Render Update camera");
    this->dataPtr->camera->Update();
  }
}

/////////////////////////////////////////////////
void IgnRenderer::HandleMouseEvent()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->HandleMouseTransformControl();
  this->HandleMouseViewControl();
}

/////////////////////////////////////////////////
void IgnRenderer::HandleMouseTransformControl()
{
  // set transform configuration
  this->dataPtr->transformControl.SetTransformMode(
      this->dataPtr->transformMode);

  if (!this->dataPtr->transformControl.Camera())
    this->dataPtr->transformControl.SetCamera(this->dataPtr->camera);

  // stop and detach transform controller if mode is none or no entity is
  // selected
  if (this->dataPtr->transformMode == rendering::TransformMode::TM_NONE ||
      (this->dataPtr->transformControl.Node() &&
      !this->dataPtr->renderUtil.SelectedEntity()))
  {
    if (this->dataPtr->transformControl.Active())
      this->dataPtr->transformControl.Stop();

    this->dataPtr->transformControl.Detach();
    this->dataPtr->renderUtil.SetSelectedEntity(
        rendering::VisualPtr());
  }
  else
  {
    // TODO(anyone) make key events work
    // shift indicates world space transformation
    // this->dataPtr->transformSpace = (this->dataPtr->keyEvent.Shift()) ?
    //     rendering::TransformSpace::TS_WORLD :
    //     rendering::TransformSpace::TS_LOCAL;
    this->dataPtr->transformControl.SetTransformSpace(
        this->dataPtr->transformSpace);
  }

  // update gizmo visual
  this->dataPtr->transformControl.Update();

  // check for mouse events
  if (!this->dataPtr->mouseDirty)
    return;

  // handle transform control
  if (this->dataPtr->mouseEvent.Button() == common::MouseEvent::LEFT)
  {
    if (this->dataPtr->mouseEvent.Type() == common::MouseEvent::PRESS
        && this->dataPtr->transformControl.Node())
    {
      // get the visual at mouse position
      rendering::VisualPtr visual = this->dataPtr->camera->VisualAt(
            this->dataPtr->mouseEvent.PressPos());

      if (visual)
      {
        // check if the visual is an axis in the gizmo visual
        math::Vector3d axis =
            this->dataPtr->transformControl.AxisById(visual->Id());
        if (axis != ignition::math::Vector3d::Zero)
        {
          // start the transform process
          this->dataPtr->transformControl.SetActiveAxis(axis);
          this->dataPtr->transformControl.Start();
          this->dataPtr->mouseDirty = false;
        }
        else
          return;
      }
    }
    else if (this->dataPtr->mouseEvent.Type() == common::MouseEvent::RELEASE)
    {
      if (this->dataPtr->transformControl.Active())
      {
        if (this->dataPtr->transformControl.Node())
        {
          std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
              [](const ignition::msgs::Boolean &/*_rep*/, const bool _result)
          {
            if (!_result)
              ignerr << "Error setting pose" << std::endl;
          };
          rendering::NodePtr node = this->dataPtr->transformControl.Node();
          ignition::msgs::Pose req;
          req.set_name(node->Name());
          msgs::Set(req.mutable_position(), node->WorldPosition());
          msgs::Set(req.mutable_orientation(), node->WorldRotation());
          if (this->dataPtr->poseCmdService.empty())
          {
            this->dataPtr->poseCmdService = "/world/" + this->worldName
                + "/set_pose";
          }
          this->dataPtr->node.Request(this->dataPtr->poseCmdService, req, cb);
        }

        this->dataPtr->transformControl.Stop();
        this->dataPtr->mouseDirty = false;
      }
      else
      {
        rendering::VisualPtr v = this->dataPtr->camera->VisualAt(
              this->dataPtr->mouseEvent.Pos());

        rendering::VisualPtr visual = this->dataPtr->camera->Scene()->VisualAt(
              this->dataPtr->camera,
              this->dataPtr->mouseEvent.Pos());

        if (!visual)
          return;

        // check if the visual is an axis in the gizmo visual
        math::Vector3d axis =
            this->dataPtr->transformControl.AxisById(visual->Id());
        if (axis == ignition::math::Vector3d::Zero)
        {
          auto topVis =
              this->dataPtr->renderUtil.SceneManager().TopLevelVisual(visual);
          // TODO(anyone) Check plane geometry instead of hardcoded name!
          if (topVis && topVis->Name() != "ground_plane")
          {
            this->dataPtr->transformControl.Attach(topVis);
            this->dataPtr->renderUtil.SetSelectedEntity(topVis);
            this->dataPtr->mouseDirty = false;
            return;
          }
        }
      }
    }
  }

  if (this->dataPtr->mouseEvent.Type() == common::MouseEvent::MOVE
      && this->dataPtr->transformControl.Active())
  {
    // compute the the start and end mouse positions in normalized coordinates
    double imageWidth = static_cast<double>(
        this->dataPtr->camera->ImageWidth());
    double imageHeight = static_cast<double>(
        this->dataPtr->camera->ImageHeight());
    double nx = 2.0 * this->dataPtr->mouseEvent.PressPos().X() /
      imageWidth - 1.0;
    double ny = 1.0 - 2.0 * this->dataPtr->mouseEvent.PressPos().Y() /
      imageHeight;
    double nxEnd = 2.0 * this->dataPtr->mouseEvent.Pos().X() /
      imageWidth - 1.0;
    double nyEnd = 1.0 - 2.0 * this->dataPtr->mouseEvent.Pos().Y() /
      imageHeight;
    math::Vector2d start(nx, ny);
    math::Vector2d end(nxEnd, nyEnd);

    // get the current active axis
    math::Vector3d axis = this->dataPtr->transformControl.ActiveAxis();

    // compute 3d transformation from 2d mouse movement
    if (this->dataPtr->transformControl.Mode() ==
        rendering::TransformMode::TM_TRANSLATION)
    {
      math::Vector3d distance =
          this->dataPtr->transformControl.TranslationFrom2d(axis, start, end);
      this->dataPtr->transformControl.Translate(distance);
    }
    else if (this->dataPtr->transformControl.Mode() ==
        rendering::TransformMode::TM_ROTATION)
    {
      math::Quaterniond rotation =
          this->dataPtr->transformControl.RotationFrom2d(axis, start, end);
      this->dataPtr->transformControl.Rotate(rotation);
    }
    else if (this->dataPtr->transformControl.Mode() ==
        rendering::TransformMode::TM_SCALE)
    {
      // note: scaling is limited to local space
      math::Vector3d scale =
          this->dataPtr->transformControl.ScaleFrom2d(axis, start, end);
      this->dataPtr->transformControl.Scale(scale);
    }
    this->dataPtr->drag = 0;
    this->dataPtr->mouseDirty = false;
  }
}


/////////////////////////////////////////////////
void IgnRenderer::HandleMouseViewControl()
{
  if (!this->dataPtr->mouseDirty)
    return;

  this->dataPtr->viewControl.SetCamera(this->dataPtr->camera);

  if (this->dataPtr->mouseEvent.Type() == common::MouseEvent::SCROLL)
  {
    this->dataPtr->target =
        this->ScreenToScene(this->dataPtr->mouseEvent.Pos());
    this->dataPtr->viewControl.SetTarget(this->dataPtr->target);
    double distance = this->dataPtr->camera->WorldPosition().Distance(
        this->dataPtr->target);
    double amount = -this->dataPtr->drag.Y() * distance / 5.0;
    this->dataPtr->viewControl.Zoom(amount);
  }
  else
  {
    if (this->dataPtr->mouseEvent.Type() == common::MouseEvent::PRESS)
    {
      this->dataPtr->target = this->ScreenToScene(
          this->dataPtr->mouseEvent.PressPos());
      this->dataPtr->viewControl.SetTarget(this->dataPtr->target);
    }

    // Pan with left button
    if (this->dataPtr->mouseEvent.Buttons() & common::MouseEvent::LEFT)
    {
      this->dataPtr->viewControl.Pan(this->dataPtr->drag);
    }
    // Orbit with middle button
    else if (this->dataPtr->mouseEvent.Buttons() & common::MouseEvent::MIDDLE)
    {
      this->dataPtr->viewControl.Orbit(this->dataPtr->drag);
    }
    else if (this->dataPtr->mouseEvent.Buttons() & common::MouseEvent::RIGHT)
    {
      double hfov = this->dataPtr->camera->HFOV().Radian();
      double vfov = 2.0f * atan(tan(hfov / 2.0f) /
          this->dataPtr->camera->AspectRatio());
      double distance = this->dataPtr->camera->WorldPosition().Distance(
          this->dataPtr->target);
      double amount = ((-this->dataPtr->drag.Y() /
          static_cast<double>(this->dataPtr->camera->ImageHeight()))
          * distance * tan(vfov/2.0) * 6.0);
      this->dataPtr->viewControl.Zoom(amount);
    }
  }
  this->dataPtr->drag = 0;
  this->dataPtr->mouseDirty = false;
}

/////////////////////////////////////////////////
void IgnRenderer::Initialize()
{
  if (this->initialized)
    return;

  this->dataPtr->renderUtil.SetUseCurrentGLContext(true);
  this->dataPtr->renderUtil.Init();

  rendering::ScenePtr scene = this->dataPtr->renderUtil.Scene();
  auto root = scene->RootVisual();

  // Camera
  this->dataPtr->camera = scene->CreateCamera();
  root->AddChild(this->dataPtr->camera);
  this->dataPtr->camera->SetLocalPose(this->cameraPose);
  this->dataPtr->camera->SetImageWidth(this->textureSize.width());
  this->dataPtr->camera->SetImageHeight(this->textureSize.height());
  this->dataPtr->camera->SetAntiAliasing(8);
  this->dataPtr->camera->SetHFOV(M_PI * 0.5);
  // setting the size and calling PreRender should cause the render texture to
  //  be rebuilt
  this->dataPtr->camera->PreRender();
  this->textureId = this->dataPtr->camera->RenderTextureGLId();

  // Ray Query
  this->dataPtr->rayQuery = this->dataPtr->camera->Scene()->CreateRayQuery();

  this->initialized = true;
}

/////////////////////////////////////////////////
void IgnRenderer::Destroy()
{
  auto engine = rendering::engine(this->dataPtr->renderUtil.EngineName());
  if (!engine)
    return;
  auto scene = engine->SceneByName(this->dataPtr->renderUtil.SceneName());
  if (!scene)
    return;
  scene->DestroySensor(this->dataPtr->camera);

  // If that was the last sensor, destroy scene
  if (scene->SensorCount() == 0)
  {
    igndbg << "Destroy scene [" << scene->Name() << "]" << std::endl;
    engine->DestroyScene(scene);

    // TODO(anyone) If that was the last scene, terminate engine?
  }
}

/////////////////////////////////////////////////
void IgnRenderer::SetTransformMode(const std::string &_mode)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  if (_mode == "select")
    this->dataPtr->transformMode = rendering::TransformMode::TM_NONE;
  else if (_mode == "translate")
    this->dataPtr->transformMode = rendering::TransformMode::TM_TRANSLATION;
  else if (_mode == "rotate")
    this->dataPtr->transformMode = rendering::TransformMode::TM_ROTATION;
  else if (_mode == "scale")
    this->dataPtr->transformMode = rendering::TransformMode::TM_SCALE;
  else
    ignerr << "Unknown transform mode: [" << _mode << "]" << std::endl;
}

/////////////////////////////////////////////////
void IgnRenderer::NewMouseEvent(const common::MouseEvent &_e,
    const math::Vector2d &_drag)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->mouseEvent = _e;
  this->dataPtr->drag += _drag;
  this->dataPtr->mouseDirty = true;
}

/////////////////////////////////////////////////
math::Vector3d IgnRenderer::ScreenToScene(
    const math::Vector2i &_screenPos) const
{
  // Normalize point on the image
  double width = this->dataPtr->camera->ImageWidth();
  double height = this->dataPtr->camera->ImageHeight();

  double nx = 2.0 * _screenPos.X() / width - 1.0;
  double ny = 1.0 - 2.0 * _screenPos.Y() / height;

  // Make a ray query
  this->dataPtr->rayQuery->SetFromCamera(
      this->dataPtr->camera, math::Vector2d(nx, ny));

  auto result = this->dataPtr->rayQuery->ClosestPoint();
  if (result)
    return result.point;

  // Set point to be 10m away if no intersection found
  return this->dataPtr->rayQuery->Origin() +
      this->dataPtr->rayQuery->Direction() * 10;
}

/////////////////////////////////////////////////
RenderThread::RenderThread()
{
  RenderWindowItemPrivate::threads << this;
}

/////////////////////////////////////////////////
void RenderThread::RenderNext()
{
  this->context->makeCurrent(this->surface);

  if (!this->ignRenderer.initialized)
  {
    // Initialize renderer
    this->ignRenderer.Initialize();
  }

  // check if engine has been successfully initialized
  if (!this->ignRenderer.initialized)
  {
    ignerr << "Unable to initialize renderer" << std::endl;
    return;
  }

  this->ignRenderer.Render();

  emit TextureReady(this->ignRenderer.textureId, this->ignRenderer.textureSize);
}

/////////////////////////////////////////////////
void RenderThread::ShutDown()
{
  this->context->makeCurrent(this->surface);

  this->ignRenderer.Destroy();

  this->context->doneCurrent();
  delete this->context;

  // schedule this to be deleted only after we're done cleaning up
  this->surface->deleteLater();

  // Stop event processing, move the thread to GUI and make sure it is deleted.
  this->moveToThread(QGuiApplication::instance()->thread());
}


/////////////////////////////////////////////////
void RenderThread::SizeChanged()
{
  auto item = qobject_cast<QQuickItem *>(this->sender());
  if (!item)
  {
    ignerr << "Internal error, sender is not QQuickItem." << std::endl;
    return;
  }

  if (item->width() <= 0 || item->height() <= 0)
    return;

  this->ignRenderer.textureSize = QSize(item->width(), item->height());
  this->ignRenderer.textureDirty = true;
}

/////////////////////////////////////////////////
TextureNode::TextureNode(QQuickWindow *_window)
    : window(_window)
{
  // Our texture node must have a texture, so use the default 0 texture.
  this->texture = this->window->createTextureFromId(0, QSize(1, 1));
  this->setTexture(this->texture);
}

/////////////////////////////////////////////////
TextureNode::~TextureNode()
{
  delete this->texture;
}

/////////////////////////////////////////////////
void TextureNode::NewTexture(int _id, const QSize &_size)
{
  this->mutex.lock();
  this->id = _id;
  this->size = _size;
  this->mutex.unlock();

  // We cannot call QQuickWindow::update directly here, as this is only allowed
  // from the rendering thread or GUI thread.
  emit PendingNewTexture();
}

/////////////////////////////////////////////////
void TextureNode::PrepareNode()
{
  this->mutex.lock();
  int newId = this->id;
  QSize sz = this->size;
  this->id = 0;
  this->mutex.unlock();
  if (newId)
  {
    delete this->texture;
    // note: include QQuickWindow::TextureHasAlphaChannel if the rendered
    // content has alpha.
    this->texture = this->window->createTextureFromId(
        newId, sz, QQuickWindow::TextureIsOpaque);
    this->setTexture(this->texture);

    this->markDirty(DirtyMaterial);

    // This will notify the rendering thread that the texture is now being
    // rendered and it can start rendering to the other one.
    emit TextureInUse();
  }
}

/////////////////////////////////////////////////
RenderWindowItem::RenderWindowItem(QQuickItem *_parent)
  : QQuickItem(_parent), dataPtr(new RenderWindowItemPrivate)
{
  this->setAcceptedMouseButtons(Qt::AllButtons);
  this->setFlag(ItemHasContents);
  this->dataPtr->renderThread = new RenderThread();
}

/////////////////////////////////////////////////
RenderWindowItem::~RenderWindowItem() = default;

/////////////////////////////////////////////////
void RenderWindowItem::Ready()
{
  this->dataPtr->renderThread->surface = new QOffscreenSurface();
  this->dataPtr->renderThread->surface->setFormat(
      this->dataPtr->renderThread->context->format());
  this->dataPtr->renderThread->surface->create();

  this->dataPtr->renderThread->ignRenderer.textureSize =
      QSize(std::max({this->width(), 1.0}), std::max({this->height(), 1.0}));

  this->dataPtr->renderThread->moveToThread(this->dataPtr->renderThread);

  this->connect(this, &QObject::destroyed,
      this->dataPtr->renderThread, &RenderThread::ShutDown,
      Qt::QueuedConnection);

  this->connect(this, &QQuickItem::widthChanged,
      this->dataPtr->renderThread, &RenderThread::SizeChanged);
  this->connect(this, &QQuickItem::heightChanged,
      this->dataPtr->renderThread, &RenderThread::SizeChanged);

  this->dataPtr->renderThread->start();
  this->update();
}

/////////////////////////////////////////////////
QSGNode *RenderWindowItem::updatePaintNode(QSGNode *_node,
    QQuickItem::UpdatePaintNodeData *)
{
  auto node = static_cast<TextureNode *>(_node);

  if (!this->dataPtr->renderThread->context)
  {
    QOpenGLContext *current = this->window()->openglContext();
    // Some GL implementations require that the currently bound context is
    // made non-current before we set up sharing, so we doneCurrent here
    // and makeCurrent down below while setting up our own context.
    current->doneCurrent();

    this->dataPtr->renderThread->context = new QOpenGLContext();
    this->dataPtr->renderThread->context->setFormat(current->format());
    this->dataPtr->renderThread->context->setShareContext(current);
    this->dataPtr->renderThread->context->create();
    this->dataPtr->renderThread->context->moveToThread(
        this->dataPtr->renderThread);

    current->makeCurrent(this->window());

    QMetaObject::invokeMethod(this, "Ready");
    return nullptr;
  }

  if (!node)
  {
    node = new TextureNode(this->window());

    // Set up connections to get the production of render texture in sync with
    // vsync on the rendering thread.
    //
    // When a new texture is ready on the rendering thread, we use a direct
    // connection to the texture node to let it know a new texture can be used.
    // The node will then emit PendingNewTexture which we bind to
    // QQuickWindow::update to schedule a redraw.
    //
    // When the scene graph starts rendering the next frame, the PrepareNode()
    // function is used to update the node with the new texture. Once it
    // completes, it emits TextureInUse() which we connect to the rendering
    // thread's RenderNext() to have it start producing content into its render
    // texture.
    //
    // This rendering pipeline is throttled by vsync on the scene graph
    // rendering thread.

    this->connect(this->dataPtr->renderThread, &RenderThread::TextureReady,
        node, &TextureNode::NewTexture, Qt::DirectConnection);
    this->connect(node, &TextureNode::PendingNewTexture, this->window(),
        &QQuickWindow::update, Qt::QueuedConnection);
    this->connect(this->window(), &QQuickWindow::beforeRendering, node,
        &TextureNode::PrepareNode, Qt::DirectConnection);
    this->connect(node, &TextureNode::TextureInUse, this->dataPtr->renderThread,
        &RenderThread::RenderNext, Qt::QueuedConnection);

    // Get the production of FBO textures started..
    QMetaObject::invokeMethod(this->dataPtr->renderThread, "RenderNext",
        Qt::QueuedConnection);
  }

  node->setRect(this->boundingRect());

  return node;
}

////////////////////////////////////////////////
RenderUtil *RenderWindowItem::RenderUtil() const
{
  return this->dataPtr->renderThread->ignRenderer.RenderUtil();
}

/////////////////////////////////////////////////
Scene3D::Scene3D()
  : GuiSystem(), dataPtr(new Scene3DPrivate)
{
  qmlRegisterType<RenderWindowItem>("RenderWindow", 1, 0, "RenderWindow");
}


/////////////////////////////////////////////////
Scene3D::~Scene3D() = default;

/////////////////////////////////////////////////
void Scene3D::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  auto renderWindow = this->PluginItem()->findChild<RenderWindowItem *>();
  if (!renderWindow)
  {
    ignerr << "Unable to find Render Window item. "
           << "Render window will not be created" << std::endl;
    return;
  }

  if (this->title.empty())
    this->title = "3D Scene";

  this->dataPtr->renderUtil = renderWindow->RenderUtil();

  renderWindow->forceActiveFocus();

  // Custom parameters
  if (_pluginElem)
  {
    if (auto elem = _pluginElem->FirstChildElement("engine"))
    {
      std::string engineName = elem->GetText();
      if (!engineName.empty())
      {
        this->dataPtr->renderUtil->SetEngineName(engineName);

        // there is a problem with displaying ogre2 render textures that are in
        // sRGB format. Workaround for now is to apply gamma correction
        // manually. There maybe a better way to solve the problem by making
        // OpenGL calls..
        if (engineName == std::string("ogre2"))
          this->PluginItem()->setProperty("gammaCorrect", true);
      }
    }

    if (auto elem = _pluginElem->FirstChildElement("scene"))
    {
      this->dataPtr->renderUtil->SetSceneName(elem->GetText());
    }

    if (auto elem = _pluginElem->FirstChildElement("ambient_light"))
    {
      math::Color ambient;
      std::stringstream colorStr;
      colorStr << std::string(elem->GetText());
      colorStr >> ambient;
      this->dataPtr->renderUtil->SetAmbientLight(ambient);
    }

    if (auto elem = _pluginElem->FirstChildElement("background_color"))
    {
      math::Color bgColor;
      std::stringstream colorStr;
      colorStr << std::string(elem->GetText());
      colorStr >> bgColor;
      this->dataPtr->renderUtil->SetBackgroundColor(bgColor);
    }

    if (auto elem = _pluginElem->FirstChildElement("camera_pose"))
    {
      math::Pose3d pose;
      std::stringstream poseStr;
      poseStr << std::string(elem->GetText());
      poseStr >> pose;
      renderWindow->SetCameraPose(pose);
    }
  }

  this->dataPtr->transformModeService =
      "/gui/transform_mode";
  this->dataPtr->node.Advertise(this->dataPtr->transformModeService,
      &Scene3D::OnTransformMode, this);
  ignmsg << "Transform mode service on ["
         << this->dataPtr->transformModeService << "]" << std::endl;
}

//////////////////////////////////////////////////
void Scene3D::Update(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  IGN_PROFILE("Scene3D::Update");
  if (this->dataPtr->worldName.empty())
  {
    // TODO(anyone) Only one scene is supported for now
    _ecm.EachNew<components::World, components::Name>(
        [&](const Entity &/*_entity*/,
          const components::World * /* _world */ ,
          const components::Name *_name)->bool
        {
          this->dataPtr->worldName = _name->Data();
          return true;
        });

    RenderWindowItem *renderWindow =
        this->PluginItem()->findChild<RenderWindowItem *>();
    renderWindow->SetWorldName(this->dataPtr->worldName);
  }

  this->dataPtr->renderUtil->UpdateFromECM(_info, _ecm);
}

/////////////////////////////////////////////////
bool Scene3D::OnTransformMode(const msgs::StringMsg &_msg,
  msgs::Boolean &_res)
{
  RenderWindowItem *renderWindow =
      this->PluginItem()->findChild<RenderWindowItem *>();
  renderWindow->SetTransformMode(_msg.data());

  _res.set_data(true);
  return true;
}

/////////////////////////////////////////////////
void RenderWindowItem::SetTransformMode(const std::string &_mode)
{
  this->dataPtr->renderThread->ignRenderer.SetTransformMode(_mode);
}

/////////////////////////////////////////////////
void RenderWindowItem::SetCameraPose(const math::Pose3d &_pose)
{
  this->dataPtr->renderThread->ignRenderer.cameraPose = _pose;
}

/////////////////////////////////////////////////
void RenderWindowItem::SetWorldName(const std::string &_name)
{
  this->dataPtr->renderThread->ignRenderer.worldName = _name;
}

/////////////////////////////////////////////////
void RenderWindowItem::mousePressEvent(QMouseEvent *_e)
{
  auto event = gui::convert(*_e);
  event.SetPressPos(event.Pos());
  this->dataPtr->mouseEvent = event;
  this->dataPtr->mouseEvent.SetType(common::MouseEvent::PRESS);

  this->dataPtr->renderThread->ignRenderer.NewMouseEvent(
      this->dataPtr->mouseEvent);
}

////////////////////////////////////////////////
void RenderWindowItem::mouseReleaseEvent(QMouseEvent *_e)
{
  this->dataPtr->mouseEvent = gui::convert(*_e);
  this->dataPtr->mouseEvent.SetType(common::MouseEvent::RELEASE);

  this->dataPtr->renderThread->ignRenderer.NewMouseEvent(
      this->dataPtr->mouseEvent);
}

////////////////////////////////////////////////
void RenderWindowItem::mouseMoveEvent(QMouseEvent *_e)
{
  auto event = gui::convert(*_e);
  event.SetPressPos(this->dataPtr->mouseEvent.PressPos());

  if (!event.Dragging())
    return;

  auto dragInt = event.Pos() - this->dataPtr->mouseEvent.Pos();
  auto dragDistance = math::Vector2d(dragInt.X(), dragInt.Y());

  this->dataPtr->mouseEvent = event;
  this->dataPtr->mouseEvent.SetType(common::MouseEvent::MOVE);
  this->dataPtr->renderThread->ignRenderer.NewMouseEvent(
      this->dataPtr->mouseEvent, dragDistance);
}

////////////////////////////////////////////////
void RenderWindowItem::wheelEvent(QWheelEvent *_e)
{
  this->dataPtr->mouseEvent.SetType(common::MouseEvent::SCROLL);
  this->dataPtr->mouseEvent.SetPos(_e->x(), _e->y());
  double scroll = (_e->angleDelta().y() > 0) ? -1.0 : 1.0;
  this->dataPtr->renderThread->ignRenderer.NewMouseEvent(
      this->dataPtr->mouseEvent, math::Vector2d(scroll, scroll));
}

///////////////////////////////////////////////////
// void Scene3D::resizeEvent(QResizeEvent *_e)
// {
//  if (this->dataPtr->renderWindow)
//  {
//    this->dataPtr->renderWindow->OnResize(_e->size().width(),
//                                          _e->size().height());
//  }
//
//  if (this->dataPtr->camera)
//  {
//    this->dataPtr->camera->SetAspectRatio(
//        static_cast<double>(this->width()) / this->height());
//    this->dataPtr->camera->SetHFOV(M_PI * 0.5);
//  }
// }
//

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::Scene3D,
                    ignition::gui::Plugin)
