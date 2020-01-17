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

#include <ignition/common/Animation.hh>
#include <ignition/common/Console.hh>
#include <ignition/common/KeyFrame.hh>
#include <ignition/common/MeshManager.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/common/VideoEncoder.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include <ignition/rendering/Image.hh>
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

Q_DECLARE_METATYPE(std::string)

namespace ignition
{
namespace gazebo
{
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
  //
  /// \brief Helper class for animating a user camera to move to a target entity
  /// todo(anyone) Move this functionality to rendering::Camera class in
  /// ign-rendering3
  class MoveToHelper
  {
    /// \brief Move the camera to look at the specified target
    /// param[in] _camera Camera to be moved
    /// param[in] _target Target to look at
    /// param[in] _duration Duration of the move to animation, in seconds.
    /// param[in] _onAnimationComplete Callback function when animation is
    /// complete
    public: void MoveTo(const rendering::CameraPtr &_camera,
        const rendering::NodePtr &_target, double _duration,
        std::function<void()> _onAnimationComplete);

    /// \brief Add time to the animation.
    /// \param[in] _time Time to add in seconds
    public: void AddTime(double _time);

    /// \brief Get whether the move to helper is idle, i.e. no animation
    /// is being executed.
    /// \return True if idle, false otherwise
    public: bool Idle() const;

    /// \brief Pose animation object
    public: std::unique_ptr<common::PoseAnimation> poseAnim;

    /// \brief Pointer to the camera being moved
    public: rendering::CameraPtr camera;

    /// \brief Callback function when animation is complete.
    public: std::function<void()> onAnimationComplete;
  };

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

    /// \brief True to record a video from the user camera
    public: bool recordVideo = false;

    /// \brief Video encoding format
    public: std::string recordVideoFormat;

    /// \brief Path to save the recorded video
    public: std::string recordVideoSavePath;

    /// \brief Target to move the user camera to
    public: std::string moveToTarget;

    /// \brief Helper object to move user camera
    public: MoveToHelper moveToHelper;

    /// \brief Target to follow
    public: std::string followTarget;

    /// \brief Wait for follow target
    public: bool followTargetWait = false;

    /// \brief Offset of camera from taget being followed
    public: math::Vector3d followOffset = math::Vector3d(-5, 0, 3);

    /// \brief Flag to indicate the follow offset needs to be updated
    public: bool followOffsetDirty = false;

    /// \brief Follow P gain
    public: double followPGain = 0.01;

    /// \brief True follow the target at an offset that is in world frame,
    /// false to follow in target's local frame
    public: bool followWorldFrame = false;

    /// \brief Last move to animation time
    public: std::chrono::time_point<std::chrono::system_clock> prevMoveToTime;

    /// \brief Image from user camera
    public: rendering::Image cameraImage;

    /// \brief Video encoder
    public: common::VideoEncoder videoEncoder;

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

    /// \brief Record video service
    public: std::string recordVideoService;

    /// \brief Move to service
    public: std::string moveToService;

    /// \brief Follow service
    public: std::string followService;
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

  // reset follow mode if target node got removed
  if (!this->dataPtr->followTarget.empty())
  {
    rendering::ScenePtr scene = this->dataPtr->renderUtil.Scene();
    rendering::NodePtr target = scene->NodeByName(this->dataPtr->followTarget);
    if (!target && !this->dataPtr->followTargetWait)
    {
      this->dataPtr->camera->SetFollowTarget(nullptr);
      this->dataPtr->camera->SetTrackTarget(nullptr);
      this->dataPtr->followTarget.clear();
      emit FollowTargetChanged(std::string(), false);
    }
  }

  // update and render to texture
  {
    IGN_PROFILE("IgnRenderer::Render Update camera");
    this->dataPtr->camera->Update();
  }

  // record video is requested
  {
    IGN_PROFILE("IgnRenderer::Render Record Video");
    if (this->dataPtr->recordVideo)
    {
      unsigned int width = this->dataPtr->camera->ImageWidth();
      unsigned int height = this->dataPtr->camera->ImageHeight();

      if (this->dataPtr->cameraImage.Width() != width ||
          this->dataPtr->cameraImage.Height() != height)
      {
        this->dataPtr->cameraImage = this->dataPtr->camera->CreateImage();
      }

      // Video recorder is on. Add more frames to it
      if (this->dataPtr->videoEncoder.IsEncoding())
      {
        this->dataPtr->camera->Copy(this->dataPtr->cameraImage);
        this->dataPtr->videoEncoder.AddFrame(
            this->dataPtr->cameraImage.Data<unsigned char>(), width, height);
      }
      // Video recorder is idle. Start recording.
      else
      {
        this->dataPtr->videoEncoder.Start(this->dataPtr->recordVideoFormat,
            this->dataPtr->recordVideoSavePath, width, height);
      }
    }
    else if (this->dataPtr->videoEncoder.IsEncoding())
    {
      this->dataPtr->videoEncoder.Stop();
    }
  }

  // Move To
  {
    IGN_PROFILE("IgnRenderer::Render MoveTo");
    if (!this->dataPtr->moveToTarget.empty())
    {
      if (this->dataPtr->moveToHelper.Idle())
      {
        rendering::ScenePtr scene = this->dataPtr->renderUtil.Scene();
        rendering::NodePtr target = scene->NodeByName(
            this->dataPtr->moveToTarget);
        if (target)
        {
          this->dataPtr->moveToHelper.MoveTo(this->dataPtr->camera, target, 0.5,
              std::bind(&IgnRenderer::OnMoveToComplete, this));
          this->dataPtr->prevMoveToTime = std::chrono::system_clock::now();
        }
        else
        {
          ignerr << "Unable to move to target. Target: '"
                 << this->dataPtr->moveToTarget << "' not found" << std::endl;
          this->dataPtr->moveToTarget.clear();
        }
      }
      else
      {
        auto now = std::chrono::system_clock::now();
        std::chrono::duration<double> dt = now - this->dataPtr->prevMoveToTime;
        this->dataPtr->moveToHelper.AddTime(dt.count());
        this->dataPtr->prevMoveToTime = now;
      }
    }
  }

  // Follow
  {
    IGN_PROFILE("IgnRenderer::Render Follow");
    if (!this->dataPtr->moveToTarget.empty())
      return;
    rendering::NodePtr followTarget = this->dataPtr->camera->FollowTarget();
    if (!this->dataPtr->followTarget.empty())
    {
      rendering::ScenePtr scene = this->dataPtr->renderUtil.Scene();
      rendering::NodePtr target = scene->NodeByName(
          this->dataPtr->followTarget);
      if (target)
      {
        if (!followTarget || target != followTarget)
        {
          this->dataPtr->camera->SetFollowTarget(target,
              this->dataPtr->followOffset,
              this->dataPtr->followWorldFrame);
          this->dataPtr->camera->SetFollowPGain(this->dataPtr->followPGain);

          this->dataPtr->camera->SetTrackTarget(target);
          // found target, no need to wait anymore
          this->dataPtr->followTargetWait = false;
        }
        else if (this->dataPtr->followOffsetDirty)
        {
          math::Vector3d offset =
              this->dataPtr->camera->WorldPosition() - target->WorldPosition();
          if (!this->dataPtr->followWorldFrame)
          {
            offset = target->WorldRotation().RotateVectorReverse(offset);
          }
          this->dataPtr->camera->SetFollowOffset(offset);
          this->dataPtr->followOffsetDirty = false;
        }
      }
      else if (!this->dataPtr->followTargetWait)
      {
        ignerr << "Unable to follow target. Target: '"
               << this->dataPtr->followTarget << "' not found" << std::endl;
        this->dataPtr->followTarget.clear();
      }
    }
    else if (followTarget)
    {
      this->dataPtr->camera->SetFollowTarget(nullptr);
      this->dataPtr->camera->SetTrackTarget(nullptr);
    }
  }
}

/////////////////////////////////////////////////
void IgnRenderer::HandleMouseEvent()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->HandleMouseContextMenu();
  this->HandleMouseTransformControl();
  this->HandleMouseViewControl();
}


/////////////////////////////////////////////////
void IgnRenderer::HandleMouseContextMenu()
{
  if (!this->dataPtr->mouseDirty)
    return;

  if (!this->dataPtr->mouseEvent.Dragging() &&
      this->dataPtr->mouseEvent.Type() == common::MouseEvent::RELEASE &&
      this->dataPtr->mouseEvent.Button() == common::MouseEvent::RIGHT)
  {
    math::Vector2i dt =
      this->dataPtr->mouseEvent.PressPos() - this->dataPtr->mouseEvent.Pos();

    // check for click with some tol for mouse movement
    if (dt.Length() > 5.0)
      return;

    rendering::VisualPtr visual = this->dataPtr->camera->Scene()->VisualAt(
          this->dataPtr->camera,
          this->dataPtr->mouseEvent.Pos());

    if (!visual)
      return;

    // get model visual
    while (visual->HasParent() && visual->Parent() !=
        visual->Scene()->RootVisual())
    {
      visual = std::dynamic_pointer_cast<rendering::Visual>(visual->Parent());
    }

    emit ContextMenuRequested(visual->Name().c_str());
    this->dataPtr->mouseDirty = false;
  }
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
    auto imageWidth = static_cast<double>(this->dataPtr->camera->ImageWidth());
    auto imageHeight = static_cast<double>(
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

  math::Vector3d camWorldPos;
  if (!this->dataPtr->followTarget.empty())
    this->dataPtr->camera->WorldPosition();

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


  if (!this->dataPtr->followTarget.empty())
  {
    math::Vector3d dPos = this->dataPtr->camera->WorldPosition() - camWorldPos;
    if (dPos != math::Vector3d::Zero)
    {
      this->dataPtr->followOffsetDirty = true;
    }
  }
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
void IgnRenderer::SetRecordVideo(bool _record, const std::string &_format,
    const std::string &_savePath)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->recordVideo = _record;
  this->dataPtr->recordVideoFormat = _format;
  this->dataPtr->recordVideoSavePath = _savePath;
}

/////////////////////////////////////////////////
void IgnRenderer::SetMoveTo(const std::string &_target)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->moveToTarget = _target;
}

/////////////////////////////////////////////////
void IgnRenderer::SetFollowTarget(const std::string &_target,
    bool _waitForTarget)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->followTarget = _target;
  this->dataPtr->followTargetWait = _waitForTarget;
}

/////////////////////////////////////////////////
void IgnRenderer::SetFollowPGain(double _gain)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->followPGain = _gain;
}

/////////////////////////////////////////////////
void IgnRenderer::SetFollowWorldFrame(bool _worldFrame)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->followWorldFrame = _worldFrame;
}

/////////////////////////////////////////////////
bool IgnRenderer::FollowWorldFrame() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->followWorldFrame;
}

/////////////////////////////////////////////////
void IgnRenderer::SetFollowOffset(const math::Vector3d &_offset)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->followOffset = _offset;
}

/////////////////////////////////////////////////
math::Vector3d IgnRenderer::FollowOffset() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->followOffset;
}

/////////////////////////////////////////////////
std::string IgnRenderer::FollowTarget() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->followTarget;
}

/////////////////////////////////////////////////
void IgnRenderer::OnMoveToComplete()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->moveToTarget.clear();
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
  qRegisterMetaType<std::string>();
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
  // FIXME(anyone) Ogre 1/2 singletons crash when there's an attempt to load
  // this plugin twice, so shortcut here. Ideally this would be caught at
  // Ignition Rendering.
  static bool done{false};
  if (done)
  {
    return;
  }
  done = true;

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

  this->connect(&this->dataPtr->renderThread->ignRenderer,
      &IgnRenderer::ContextMenuRequested,
      this, &RenderWindowItem::OnContextMenuRequested, Qt::QueuedConnection);

  this->connect(&this->dataPtr->renderThread->ignRenderer,
      &IgnRenderer::FollowTargetChanged,
      this, &RenderWindowItem::SetFollowTarget, Qt::QueuedConnection);

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

///////////////////////////////////////////////////
void RenderWindowItem::OnContextMenuRequested(QString _entity)
{
  emit openContextMenu(std::move(_entity));
}

math::Vector3d RenderWindowItem::ScreenToScene(const math::Vector2i &_screenPos)
{
  return this->dataPtr->renderThread->ignRenderer.ScreenToScene(_screenPos);
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
  // FIXME(anyone) Ogre 1/2 singletons crash when there's an attempt to load
  // this plugin twice, so shortcut here. Ideally this would be caught at
  // Ignition Rendering.
  static bool done{false};
  if (done)
  {
    ignerr << "Only one Scene3D is supported per process at the moment."
           << std::endl;
    return;
  }
  done = true;

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

    if (auto elem = _pluginElem->FirstChildElement("camera_follow"))
    {
      if (auto gainElem = elem->FirstChildElement("p_gain"))
      {
        double gain;
        std::stringstream gainStr;
        gainStr << std::string(gainElem->GetText());
        gainStr >> gain;
        if (gain >= 0 && gain <= 1.0)
          renderWindow->SetFollowPGain(gain);
        else
          ignerr << "Camera follow p gain outside of range [0, 1]" << std::endl;
      }

      if (auto targetElem = elem->FirstChildElement("target"))
      {
        std::stringstream targetStr;
        targetStr << std::string(targetElem->GetText());
        renderWindow->SetFollowTarget(targetStr.str(), true);
      }

      if (auto worldFrameElem = elem->FirstChildElement("world_frame"))
      {
        std::string worldFrameStr =
            common::lowercase(worldFrameElem->GetText());
        if (worldFrameStr == "true" || worldFrameStr == "1")
          renderWindow->SetFollowWorldFrame(true);
        else if (worldFrameStr == "false" || worldFrameStr == "0")
          renderWindow->SetFollowWorldFrame(false);
        else
        {
          ignerr << "Faild to parse <world_frame> value: " << worldFrameStr
                 << std::endl;
        }
      }

      if (auto offsetElem = elem->FirstChildElement("offset"))
      {
        math::Vector3d offset;
        std::stringstream offsetStr;
        offsetStr << std::string(offsetElem->GetText());
        offsetStr >> offset;
        renderWindow->SetFollowOffset(offset);
      }
    }
  }

  // transform mode
  this->dataPtr->transformModeService =
      "/gui/transform_mode";
  this->dataPtr->node.Advertise(this->dataPtr->transformModeService,
      &Scene3D::OnTransformMode, this);
  ignmsg << "Transform mode service on ["
         << this->dataPtr->transformModeService << "]" << std::endl;

  // video recorder
  this->dataPtr->recordVideoService =
      "/gui/record_video";
  this->dataPtr->node.Advertise(this->dataPtr->recordVideoService,
      &Scene3D::OnRecordVideo, this);
  ignmsg << "Record video service on ["
         << this->dataPtr->recordVideoService << "]" << std::endl;

  // move to
  this->dataPtr->moveToService = "/gui/move_to";
  this->dataPtr->node.Advertise(this->dataPtr->moveToService,
      &Scene3D::OnMoveTo, this);
  ignmsg << "Move to service on ["
         << this->dataPtr->moveToService << "]" << std::endl;

  // follow
  this->dataPtr->followService = "/gui/follow";
  this->dataPtr->node.Advertise(this->dataPtr->followService,
      &Scene3D::OnFollow, this);
  ignmsg << "Follow service on ["
         << this->dataPtr->followService << "]" << std::endl;
}

//////////////////////////////////////////////////
void Scene3D::Update(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  if (nullptr == this->dataPtr->renderUtil)
    return;

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

    auto renderWindow = this->PluginItem()->findChild<RenderWindowItem *>();
    renderWindow->SetWorldName(this->dataPtr->worldName);
  }

  this->dataPtr->renderUtil->UpdateFromECM(_info, _ecm);
}

/////////////////////////////////////////////////
bool Scene3D::OnTransformMode(const msgs::StringMsg &_msg,
  msgs::Boolean &_res)
{
  auto renderWindow = this->PluginItem()->findChild<RenderWindowItem *>();
  renderWindow->SetTransformMode(_msg.data());

  _res.set_data(true);
  return true;
}

/////////////////////////////////////////////////
bool Scene3D::OnRecordVideo(const msgs::VideoRecord &_msg,
  msgs::Boolean &_res)
{
  auto renderWindow = this->PluginItem()->findChild<RenderWindowItem *>();

  bool record = _msg.start() && !_msg.stop();
  renderWindow->SetRecordVideo(record, _msg.format(), _msg.save_filename());

  _res.set_data(true);
  return true;
}

/////////////////////////////////////////////////
bool Scene3D::OnMoveTo(const msgs::StringMsg &_msg,
  msgs::Boolean &_res)
{
  auto renderWindow = this->PluginItem()->findChild<RenderWindowItem *>();

  renderWindow->SetMoveTo(_msg.data());

  _res.set_data(true);
  return true;
}

/////////////////////////////////////////////////
bool Scene3D::OnFollow(const msgs::StringMsg &_msg,
  msgs::Boolean &_res)
{
  auto renderWindow = this->PluginItem()->findChild<RenderWindowItem *>();

  renderWindow->SetFollowTarget(_msg.data());

  _res.set_data(true);
  return true;
}

/////////////////////////////////////////////////
void Scene3D::OnDropped(const QString &_drop, int _mouseX, int _mouseY)
{
  if (_drop.toStdString().empty())
  {
    ignwarn << "Dropped empty entity URI." << std::endl;
    return;
  }

  std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
      [](const ignition::msgs::Boolean &_res, const bool _result)
  {
    if (!_result || !_res.data())
      ignerr << "Error creating dropped entity." << std::endl;
  };

  auto renderWindow = this->PluginItem()->findChild<RenderWindowItem *>();
  math::Vector3d pos = renderWindow->ScreenToScene({_mouseX, _mouseY});

  msgs::EntityFactory req;
  req.set_sdf_filename(_drop.toStdString());
  req.set_allow_renaming(true);
  msgs::Set(req.mutable_pose(),
      math::Pose3d(pos.X(), pos.Y(), pos.Z(), 1, 0, 0, 0));

  this->dataPtr->node.Request("/world/" + this->dataPtr->worldName + "/create",
      req, cb);
}

/////////////////////////////////////////////////
void RenderWindowItem::SetTransformMode(const std::string &_mode)
{
  this->dataPtr->renderThread->ignRenderer.SetTransformMode(_mode);
}

/////////////////////////////////////////////////
void RenderWindowItem::SetRecordVideo(bool _record, const std::string &_format,
    const std::string &_savePath)
{
  this->dataPtr->renderThread->ignRenderer.SetRecordVideo(_record, _format,
      _savePath);
}

/////////////////////////////////////////////////
void RenderWindowItem::SetMoveTo(const std::string &_target)
{
  this->dataPtr->renderThread->ignRenderer.SetMoveTo(_target);
}

/////////////////////////////////////////////////
void RenderWindowItem::SetFollowTarget(const std::string &_target,
    bool _waitForTarget)
{
  this->setProperty("message", _target.empty() ? "" :
      "Press Escape to exit Follow mode");
  this->dataPtr->renderThread->ignRenderer.SetFollowTarget(_target,
      _waitForTarget);
}

/////////////////////////////////////////////////
void RenderWindowItem::SetFollowPGain(double _gain)
{
  this->dataPtr->renderThread->ignRenderer.SetFollowPGain(_gain);
}

/////////////////////////////////////////////////
void RenderWindowItem::SetFollowWorldFrame(bool _worldFrame)
{
  this->dataPtr->renderThread->ignRenderer.SetFollowWorldFrame(_worldFrame);
}

/////////////////////////////////////////////////
void RenderWindowItem::SetFollowOffset(const math::Vector3d &_offset)
{
  this->dataPtr->renderThread->ignRenderer.SetFollowOffset(_offset);
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
  this->forceActiveFocus();

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
  auto event = gui::convert(*_e);
  event.SetPressPos(this->dataPtr->mouseEvent.PressPos());
  this->dataPtr->mouseEvent = event;
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
  this->forceActiveFocus();

  this->dataPtr->mouseEvent.SetType(common::MouseEvent::SCROLL);
  this->dataPtr->mouseEvent.SetPos(_e->x(), _e->y());
  double scroll = (_e->angleDelta().y() > 0) ? -1.0 : 1.0;
  this->dataPtr->renderThread->ignRenderer.NewMouseEvent(
      this->dataPtr->mouseEvent, math::Vector2d(scroll, scroll));
}

////////////////////////////////////////////////
void RenderWindowItem::keyReleaseEvent(QKeyEvent *_e)
{
  if (_e->key() == Qt::Key_Escape)
  {
    if (!this->dataPtr->renderThread->ignRenderer.FollowTarget().empty())
    {
      this->SetFollowTarget(std::string());
      this->setProperty("message", "");

      _e->accept();
    }
  }
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

////////////////////////////////////////////////
void MoveToHelper::MoveTo(const rendering::CameraPtr &_camera,
    const rendering::NodePtr &_target,
    double _duration, std::function<void()> _onAnimationComplete)
{
  this->camera = _camera;
  this->poseAnim = std::make_unique<common::PoseAnimation>(
      "move_to", _duration, false);
  this->onAnimationComplete = std::move(_onAnimationComplete);

  math::Pose3d start = _camera->WorldPose();

  // todo(anyone) implement bounding box function in rendering to get
  // target size and center.
  // Assume fixed size and target world position is its center
  math::Box targetBBox(1.0, 1.0, 1.0);
  math::Vector3d targetCenter = _target->WorldPosition();
  math::Vector3d dir = targetCenter - start.Pos();
  dir.Correct();
  dir.Normalize();

  // distance to move
  double maxSize = targetBBox.Size().Max();
  double dist = start.Pos().Distance(targetCenter) - maxSize;

  // Scale to fit in view
  double hfov = this->camera->HFOV().Radian();
  double offset = maxSize*0.5 / std::tan(hfov/2.0);

  // End position and rotation
  math::Vector3d endPos = start.Pos() + dir*(dist - offset);
  math::Quaterniond endRot =
      math::Matrix4d::LookAt(endPos, targetCenter).Rotation();
  math::Pose3d end(endPos, endRot);

  common::PoseKeyFrame *key = this->poseAnim->CreateKeyFrame(0);
  key->Translation(start.Pos());
  key->Rotation(start.Rot());

  key = this->poseAnim->CreateKeyFrame(_duration);
  key->Translation(end.Pos());
  key->Rotation(end.Rot());
}

////////////////////////////////////////////////
void MoveToHelper::AddTime(double _time)
{
  if (!this->camera || !this->poseAnim)
    return;

  common::PoseKeyFrame kf(0);

  this->poseAnim->AddTime(_time);
  this->poseAnim->InterpolatedKeyFrame(kf);

  math::Pose3d offset(kf.Translation(), kf.Rotation());

  this->camera->SetWorldPose(offset);

  if (this->poseAnim->Length() <= this->poseAnim->Time())
  {
    if (this->onAnimationComplete)
    {
      this->onAnimationComplete();
    }
    this->camera.reset();
    this->poseAnim.reset();
    this->onAnimationComplete = nullptr;
  }
}

////////////////////////////////////////////////
bool MoveToHelper::Idle() const
{
  return this->poseAnim == nullptr;
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::Scene3D,
                    ignition::gui::Plugin)
