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

#include "Scene3D.hh"

#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <limits>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/Visual.hh>

#include <ignition/common/Animation.hh>
#include <ignition/common/Console.hh>
#include <ignition/common/KeyFrame.hh>
#include <ignition/common/MeshManager.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/common/StringUtils.hh>
#include <ignition/common/Uuid.hh>
#include <ignition/common/VideoEncoder.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include <ignition/rendering/Image.hh>
#include <ignition/rendering/OrbitViewController.hh>
#include <ignition/rendering/MoveToHelper.hh>
#include <ignition/rendering/RayQuery.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/TransformController.hh>

#include <ignition/transport/Node.hh>

#include <ignition/utils/SuppressWarning.hh>

#include <ignition/gui/Conversions.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/RenderEngineGuiPlugin.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/gui/GuiEvents.hh"
#include "ignition/gazebo/rendering/RenderUtil.hh"

/// \brief condition variable for lockstepping video recording
/// todo(anyone) avoid using a global condition variable when we support
/// multiple viewports in the future.
std::condition_variable g_renderCv;

Q_DECLARE_METATYPE(std::string)
Q_DECLARE_METATYPE(ignition::gazebo::RenderSync*)

namespace ignition
{
namespace gazebo
{
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
  /// \brief Helper to store selection requests to be handled in the render
  /// thread by `IgnRenderer::HandleEntitySelection`.
  struct SelectionHelper
  {
    /// \brief Entity to be selected
    Entity selectEntity{kNullEntity};

    /// \brief Deselect all entities
    bool deselectAll{false};

    /// \brief True to send an event and notify all widgets
    bool sendEvent{false};
  };

  /// \brief Private data class for IgnRenderer
  class IgnRendererPrivate
  {
    /// \brief Flag to indicate if mouse event is dirty
    public: bool mouseDirty = false;

    /// \brief Flag to indicate if hover event is dirty
    public: bool hoverDirty = false;

    /// \brief Mouse event
    public: common::MouseEvent mouseEvent;

    /// \brief Key event
    public: common::KeyEvent keyEvent;

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

    /// \brief Use sim time as timestamp during video recording
    /// By default (false), video encoding is done using real time.
    public: bool recordVideoUseSimTime = false;

    /// \brief Lockstep gui with ECM when recording
    public: bool recordVideoLockstep = false;

    /// \brief Video recorder bitrate (bps)
    public: unsigned int recordVideoBitrate = 2070000;

    /// \brief Previous camera update time during video recording
    /// only used in lockstep mode and recording in sim time.
    public: std::chrono::steady_clock::time_point recordVideoUpdateTime;

    /// \brief Start tiem of video recording
    public: std::chrono::steady_clock::time_point recordStartTime;

    /// \brief Camera pose publisher
    public: transport::Node::Publisher recorderStatsPub;

    /// \brief Target to move the user camera to
    public: std::string moveToTarget;

    /// \brief Helper object to move user camera
    public: ignition::rendering::MoveToHelper moveToHelper;

    /// \brief Target to view collisions
    public: std::string viewCollisionsTarget;

    /// \brief Helper object to select entities. Only the latest selection
    /// request is kept.
    public: SelectionHelper selectionHelper;

    /// \brief Target to follow
    public: std::string followTarget;

    /// \brief Wait for follow target
    public: bool followTargetWait = false;

    /// \brief Offset of camera from target being followed
    public: math::Vector3d followOffset = math::Vector3d(-5, 0, 3);

    /// \brief Flag to indicate the follow offset needs to be updated
    public: bool followOffsetDirty = false;

    /// \brief Flag to indicate the follow offset has been updated
    public: bool newFollowOffset = true;

    /// \brief Follow P gain
    public: double followPGain = 0.01;

    /// \brief True follow the target at an offset that is in world frame,
    /// false to follow in target's local frame
    public: bool followWorldFrame = false;

    /// \brief Flag for indicating whether we are in view angle mode or not
    public: bool viewAngle = false;

    /// \brief Flag for indicating whether we are spawning or not.
    public: bool isSpawning = false;

    /// \brief Flag for indicating whether the user is currently placing a
    /// resource with the shapes plugin or not
    public: bool isPlacing = false;

    /// \brief Atomic bool indicating whether the dropdown menu
    /// is currently enabled or disabled.
    public: std::atomic_bool dropdownMenuEnabled = true;

    /// \brief The SDF string of the resource to be used with plugins that spawn
    /// entities.
    public: std::string spawnSdfString;

    /// \brief Path of an SDF file, to be used with plugins that spawn entities.
    public: std::string spawnSdfPath;

    /// \brief The pose of the spawn preview.
    public: ignition::math::Pose3d spawnPreviewPose =
            ignition::math::Pose3d::Zero;

    /// \brief The currently hovered mouse position in screen coordinates
    public: math::Vector2i mouseHoverPos = math::Vector2i::Zero;

    /// \brief The visual generated from the spawnSdfString / spawnSdfPath
    public: rendering::NodePtr spawnPreview = nullptr;

    /// \brief A record of the ids currently used by the entity spawner
    /// for easy deletion of visuals later
    public: std::vector<Entity> previewIds;

    /// \brief The pose set during a view angle button press that holds
    /// the pose the camera should assume relative to the entit(y/ies).
    /// The vector (0, 0, 0) indicates to return the camera back to the home
    /// pose originally loaded from the sdf.
    public: math::Vector3d viewAngleDirection = math::Vector3d::Zero;

    /// \brief The pose set from the move to pose service.
    public: std::optional<math::Pose3d> moveToPoseValue;

    /// \brief Last move to animation time
    public: std::chrono::time_point<std::chrono::system_clock> prevMoveToTime;

    /// \brief Image from user camera
    public: rendering::Image cameraImage;

    /// \brief Video encoder
    public: common::VideoEncoder videoEncoder;

    /// \brief Ray query for mouse clicks
    public: rendering::RayQueryPtr rayQuery;

    /// \brief View control focus target
    public: math::Vector3d target = math::Vector3d(
        math::INF_D, math::INF_D, math::INF_D);

    /// \brief Rendering utility
    public: RenderUtil renderUtil;

    /// \brief Transport node for making transform control requests
    public: transport::Node node;

    /// \brief Name of service for setting entity pose
    public: std::string poseCmdService;

    /// \brief Name of service for creating entity
    public: std::string createCmdService;

    /// \brief The starting world pose of a clicked visual.
    public: ignition::math::Vector3d startWorldPos = math::Vector3d::Zero;

    /// \brief Flag to keep track of world pose setting used
    /// for button translating.
    public: bool isStartWorldPosSet = false;

    /// \brief Where the mouse left off - used to continue translating
    /// smoothly when switching axes through keybinding and clicking
    /// Updated on an x, y, or z, press or release and a mouse press
    public: math::Vector2i mousePressPos = math::Vector2i::Zero;

    /// \brief Flag to indicate whether the x key is currently being pressed
    public: bool xPressed = false;

    /// \brief Flag to indicate whether the y key is currently being pressed
    public: bool yPressed = false;

    /// \brief Flag to indicate whether the z key is currently being pressed
    public: bool zPressed = false;

    /// \brief Flag to indicate whether the escape key has been released.
    public: bool escapeReleased = false;

    /// \brief ID of thread where render calls can be made.
    public: std::thread::id renderThreadId;

    /// \brief The xyz values by which to snap the object.
    public: math::Vector3d xyzSnap = math::Vector3d::One;

    /// \brief The rpy values by which to snap the object.
    public: math::Vector3d rpySnap = {45, 45, 45};

    /// \brief The scale values by which to snap the object.
    public: math::Vector3d scaleSnap = math::Vector3d::One;
  };

  /// \brief Qt and Ogre rendering is happening in different threads
  /// The original sample 'textureinthread' from Qt used a double-buffer
  /// scheme so that the worker (Ogre) thread write to FBO A, while
  /// Qt is displaying FBO B.
  ///
  /// However Qt's implementation doesn't handle all the edge cases
  /// (like resizing a window), and also it increases our VRAM
  /// consumption in multiple ways (since we have to double other
  /// resources as well or re-architect certain parts of the code
  /// to avoid it)
  ///
  /// Thus we just serialize both threads so that when Qt reaches
  /// drawing preparation, it halts and Ogre worker thread starts rendering,
  /// then resumes when Ogre is done.
  ///
  /// This code is admitedly more complicated than it should be
  /// because Qt's synchronization using signals and slots causes
  /// deadlocks when other means of synchronization are introduced.
  /// The whole threaded loop should be rewritten.
  ///
  /// All RenderSync does is conceptually:
  ///
  /// \code
  ///   TextureNode::PrepareNode()
  ///   {
  ///     renderSync.WaitForWorkerThread(); // Qt thread
  ///       // WaitForQtThreadAndBlock();
  ///       // Now worker thread begins executing what's between
  ///       // ReleaseQtThreadFromBlock();
  ///     continue with qt code...
  ///   }
  /// \endcode
  ///
  ///
  /// For more info see
  /// https://github.com/ignitionrobotics/ign-rendering/issues/304
  class RenderSync
  {
    /// \brief Cond. variable to synchronize rendering on specific events
    /// (e.g. texture resize) or for debugging (e.g. keep
    /// all API calls sequential)
    public: std::mutex mutex;

    /// \brief Cond. variable to synchronize rendering on specific events
    /// (e.g. texture resize) or for debugging (e.g. keep
    /// all API calls sequential)
    public: std::condition_variable cv;

    public: enum class RenderStallState
            {
              /// Qt is stuck inside WaitForWorkerThread
              /// Worker thread can proceed
              WorkerCanProceed,
              /// Qt is stuck inside WaitForWorkerThread
              /// Worker thread is between WaitForQtThreadAndBlock
              /// and ReleaseQtThreadFromBlock
              WorkerIsProceeding,
              /// Worker is stuck inside WaitForQtThreadAndBlock
              /// Qt can proceed
              QtCanProceed,
              /// Do not block
              ShuttingDown,
            };

    /// \brief See TextureNode::RenderSync::RenderStallState
    public: RenderStallState renderStallState =
        RenderStallState::QtCanProceed /*GUARDED_BY(sharedRenderMutex)*/;

    /// \brief Must be called from worker thread when we want to block
    /// \param[in] lock Acquired lock. Must be based on this->mutex
    public: void WaitForQtThreadAndBlock(std::unique_lock<std::mutex> &_lock);

    /// \brief Must be called from worker thread when we are done
    /// \param[in] lock Acquired lock. Must be based on this->mutex
    public: void ReleaseQtThreadFromBlock(std::unique_lock<std::mutex> &_lock);

    /// \brief Must be called from Qt thread periodically
    public: void WaitForWorkerThread();

    /// \brief Must be called from GUI thread when shutting down
    public: void Shutdown();
  };

  /// \brief Private data class for RenderWindowItem
  class RenderWindowItemPrivate
  {
    /// \brief Keep latest mouse event
    public: common::MouseEvent mouseEvent;

    /// \brief Render thread
    public : RenderThread *renderThread = nullptr;

    /// \brief See RenderSync
    public: RenderSync renderSync;

    //// \brief Set to true after the renderer is initialized
    public: bool rendererInit = false;

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

    /// \brief Follow offset service
    public: std::string followOffsetService;

    /// \brief View angle service
    public: std::string viewAngleService;

    /// \brief Move to pose service
    public: std::string moveToPoseService;

    /// \brief Shapes service
    public: std::string shapesService;

    /// \brief Camera pose topic
    public: std::string cameraPoseTopic;

    /// \brief Camera pose publisher
    public: transport::Node::Publisher cameraPosePub;

    /// \brief lockstep ECM updates with rendering
    public: bool recordVideoLockstep = false;

    /// \brief True to indicate video recording in progress
    public: bool recording = false;

    /// \brief mutex to protect the recording variable
    public: std::mutex recordMutex;

    /// \brief mutex to protect the render condition variable
    /// Used when recording in lockstep mode.
    public: std::mutex renderMutex;

    /// \brief View collisions service
    public: std::string viewCollisionsService;

    /// \brief Text for popup error message
    public: QString errorPopupText;
  };
}
}
}

using namespace ignition;
using namespace gazebo;

QList<QThread *> RenderWindowItemPrivate::threads;

/////////////////////////////////////////////////
void RenderSync::WaitForQtThreadAndBlock(std::unique_lock<std::mutex> &_lock)
{
  this->cv.wait(_lock, [this]
  { return this->renderStallState == RenderStallState::WorkerCanProceed ||
           this->renderStallState == RenderStallState::ShuttingDown; });

  this->renderStallState = RenderStallState::WorkerIsProceeding;
}

/////////////////////////////////////////////////
void RenderSync::ReleaseQtThreadFromBlock(std::unique_lock<std::mutex> &_lock)
{
  this->renderStallState = RenderStallState::QtCanProceed;
  _lock.unlock();
  this->cv.notify_one();
}

/////////////////////////////////////////////////
void RenderSync::WaitForWorkerThread()
{
  std::unique_lock<std::mutex> lock(this->mutex);

  // Wait until we're clear to go
  this->cv.wait( lock, [this]
  {
    return this->renderStallState == RenderStallState::QtCanProceed ||
           this->renderStallState == RenderStallState::ShuttingDown;
  } );

  // Worker thread asked us to wait!
  this->renderStallState = RenderStallState::WorkerCanProceed;
  lock.unlock();
  // Wake up worker thread
  this->cv.notify_one();
  lock.lock();

  // Wait until we're clear to go
  this->cv.wait( lock, [this]
  {
    return this->renderStallState == RenderStallState::QtCanProceed ||
           this->renderStallState == RenderStallState::ShuttingDown;
  } );
}

/////////////////////////////////////////////////
void RenderSync::Shutdown()
{
  {
    std::unique_lock<std::mutex> lock(this->mutex);

    this->renderStallState = RenderStallState::ShuttingDown;

    lock.unlock();
    this->cv.notify_one();
  }
}

/////////////////////////////////////////////////
IgnRenderer::IgnRenderer()
  : dataPtr(new IgnRendererPrivate)
{
  this->dataPtr->moveToHelper.SetInitCameraPose(this->cameraPose);

  // recorder stats topic
  std::string recorderStatsTopic = "/gui/record_video/stats";
  this->dataPtr->recorderStatsPub =
    this->dataPtr->node.Advertise<msgs::Time>(recorderStatsTopic);
  ignmsg << "Video recorder stats topic advertised on ["
         << recorderStatsTopic << "]" << std::endl;
}


/////////////////////////////////////////////////
IgnRenderer::~IgnRenderer() = default;

////////////////////////////////////////////////
RenderUtil *IgnRenderer::RenderUtil() const
{
  return &this->dataPtr->renderUtil;
}

/////////////////////////////////////////////////
void IgnRenderer::Render(RenderSync *_renderSync)
{
  rendering::ScenePtr scene = this->dataPtr->renderUtil.Scene();
  if (!scene)
  {
    ignwarn << "Scene is null. The render step will not occur in Scene3D."
      << std::endl;
    return;
  }

  this->dataPtr->renderThreadId = std::this_thread::get_id();

  IGN_PROFILE_THREAD_NAME("RenderThread");
  IGN_PROFILE("IgnRenderer::Render");

  std::unique_lock<std::mutex> lock(_renderSync->mutex);
  _renderSync->WaitForQtThreadAndBlock(lock);

  if (this->textureDirty)
  {
    // TODO(anyone) If SwapFromThread gets implemented,
    // then we only need to lock when texture is dirty
    // (but we still need to lock the whole routine if
    // debugging from RenderDoc or if user is not willing
    // to sacrifice VRAM)
    //
    // std::unique_lock<std::mutex> lock(renderSync->mutex);
    // _renderSync->WaitForQtThreadAndBlock(lock);
    this->dataPtr->camera->SetImageWidth(this->textureSize.width());
    this->dataPtr->camera->SetImageHeight(this->textureSize.height());
    this->dataPtr->camera->SetAspectRatio(this->textureSize.width() /
        static_cast<double>(this->textureSize.height()));
    // setting the size should cause the render texture to be rebuilt
    {
      IGN_PROFILE("IgnRenderer::Render Pre-render camera");
      this->dataPtr->camera->PreRender();
    }
    this->textureDirty = false;

    // TODO(anyone) See SwapFromThread comments
    // _renderSync->ReleaseQtThreadFromBlock(lock);
  }

  // texture id could change so get the value in every render update
  this->textureId = this->dataPtr->camera->RenderTextureGLId();

  // update the scene
  this->dataPtr->renderUtil.SetTransformActive(
      this->dataPtr->transformControl.Active());
  this->dataPtr->renderUtil.Update();

  // view control
  this->HandleMouseEvent();

  // Entity selection
  this->HandleEntitySelection();

  // reset follow mode if target node got removed
  if (!this->dataPtr->followTarget.empty())
  {
    rendering::NodePtr target = scene->NodeByName(this->dataPtr->followTarget);
    if (!target && !this->dataPtr->followTargetWait)
    {
      this->dataPtr->camera->SetFollowTarget(nullptr);
      this->dataPtr->camera->SetTrackTarget(nullptr);
      this->dataPtr->followTarget.clear();
      emit FollowTargetChanged(std::string(), false);
    }
  }

  // check if recording is in lockstep mode and if it is using sim time
  // if so, there is no need to update camera if sim time has not advanced
  bool update = true;
  if (this->dataPtr->recordVideoLockstep &&
      this->dataPtr->recordVideoUseSimTime &&
      this->dataPtr->videoEncoder.IsEncoding())
  {
    std::chrono::steady_clock::time_point t =
        std::chrono::steady_clock::time_point(
        this->dataPtr->renderUtil.SimTime());
    if (t - this->dataPtr->recordVideoUpdateTime == std::chrono::seconds(0))
      update = false;
    else
      this->dataPtr->recordVideoUpdateTime = t;
  }

  // update and render to texture
  if (update)
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

        std::chrono::steady_clock::time_point t =
            std::chrono::steady_clock::now();
        if (this->dataPtr->recordVideoUseSimTime)
        {
          t = std::chrono::steady_clock::time_point(
              this->dataPtr->renderUtil.SimTime());
        }
        bool frameAdded = this->dataPtr->videoEncoder.AddFrame(
            this->dataPtr->cameraImage.Data<unsigned char>(), width, height, t);

        if (frameAdded)
        {
          // publish recorder stats
          if (this->dataPtr->recordStartTime ==
              std::chrono::steady_clock::time_point(
              std::chrono::duration(std::chrono::seconds(0))))
          {
            // start time, i.e. time when first frame is added
            this->dataPtr->recordStartTime = t;
          }

          std::chrono::steady_clock::duration dt;
          dt = t - this->dataPtr->recordStartTime;
          int64_t sec, nsec;
          std::tie(sec, nsec) = ignition::math::durationToSecNsec(dt);
          msgs::Time msg;
          msg.set_sec(sec);
          msg.set_nsec(nsec);
          this->dataPtr->recorderStatsPub.Publish(msg);
        }
      }
      // Video recorder is idle. Start recording.
      else
      {
        if (this->dataPtr->recordVideoUseSimTime)
          ignmsg << "Recording video using sim time." << std::endl;
        if (this->dataPtr->recordVideoLockstep)
        {
          ignmsg << "Recording video in lockstep mode" << std::endl;
          if (!this->dataPtr->recordVideoUseSimTime)
          {
            ignwarn << "It is recommended to set <use_sim_time> to true "
                    << "when recording video in lockstep mode." << std::endl;
          }
        }
        ignmsg << "Recording video using bitrate: "
               << this->dataPtr->recordVideoBitrate <<  std::endl;
        this->dataPtr->videoEncoder.Start(this->dataPtr->recordVideoFormat,
            this->dataPtr->recordVideoSavePath, width, height, 25,
            this->dataPtr->recordVideoBitrate);
        this->dataPtr->recordStartTime = std::chrono::steady_clock::time_point(
            std::chrono::duration(std::chrono::seconds(0)));
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

  // Move to pose
  {
    IGN_PROFILE("IgnRenderer::Render MoveToPose");
    if (this->dataPtr->moveToPoseValue)
    {
      if (this->dataPtr->moveToHelper.Idle())
      {
        this->dataPtr->moveToHelper.MoveTo(this->dataPtr->camera,
            *(this->dataPtr->moveToPoseValue),
            0.5, std::bind(&IgnRenderer::OnMoveToPoseComplete, this));
        this->dataPtr->prevMoveToTime = std::chrono::system_clock::now();
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
    {
      _renderSync->ReleaseQtThreadFromBlock(lock);
      return;
    }
    rendering::NodePtr followTarget = this->dataPtr->camera->FollowTarget();
    if (!this->dataPtr->followTarget.empty())
    {
      rendering::NodePtr target = scene->NodeByName(
          this->dataPtr->followTarget);
      if (target)
      {
        if (!followTarget || target != followTarget
              || this->dataPtr->newFollowOffset)
        {
          this->dataPtr->camera->SetFollowTarget(target,
              this->dataPtr->followOffset,
              this->dataPtr->followWorldFrame);
          this->dataPtr->camera->SetFollowPGain(this->dataPtr->followPGain);

          this->dataPtr->camera->SetTrackTarget(target);
          // found target, no need to wait anymore
          this->dataPtr->followTargetWait = false;
          this->dataPtr->newFollowOffset = false;
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

  // View Angle
  {
    IGN_PROFILE("IgnRenderer::Render ViewAngle");
    if (this->dataPtr->viewAngle)
    {
      if (this->dataPtr->moveToHelper.Idle())
      {
        const std::vector<Entity> &selectedEntities =
          this->dataPtr->renderUtil.SelectedEntities();

        // Look at the origin if no entities are selected
        math::Vector3d lookAt = math::Vector3d::Zero;
        if (!selectedEntities.empty())
        {
          for (const auto &entity : selectedEntities)
          {
            rendering::NodePtr node =
                this->dataPtr->renderUtil.SceneManager().NodeById(entity);

            if (!node)
              continue;

            math::Vector3d nodePos = node->WorldPose().Pos();
            lookAt += nodePos;
          }
          lookAt /= selectedEntities.size();
        }

        this->dataPtr->moveToHelper.LookDirection(this->dataPtr->camera,
            this->dataPtr->viewAngleDirection, lookAt,
            0.5, std::bind(&IgnRenderer::OnViewAngleComplete, this));
        this->dataPtr->prevMoveToTime = std::chrono::system_clock::now();
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

  // Shapes
  {
    IGN_PROFILE("IgnRenderer::Render Shapes");
    if (this->dataPtr->isSpawning)
    {
      // Generate spawn preview
      rendering::VisualPtr rootVis = scene->RootVisual();
      sdf::Root root;
      if (!this->dataPtr->spawnSdfString.empty())
      {
        root.LoadSdfString(this->dataPtr->spawnSdfString);
      }
      else if (!this->dataPtr->spawnSdfPath.empty())
      {
        root.Load(this->dataPtr->spawnSdfPath);
      }
      else
      {
        ignwarn << "Failed to spawn: no SDF string or path" << std::endl;
      }
      this->dataPtr->isPlacing = this->GeneratePreview(root);
      this->dataPtr->isSpawning = false;
    }
  }

  // Escape action, clear all selections and terminate any
  // spawned previews if escape button is released
  {
    if (this->dataPtr->escapeReleased)
    {
      this->DeselectAllEntities(true);
      this->TerminateSpawnPreview();
      this->dataPtr->escapeReleased = false;
    }
  }

  // View collisions
  {
    IGN_PROFILE("IgnRenderer::Render ViewCollisions");
    if (!this->dataPtr->viewCollisionsTarget.empty())
    {
      rendering::NodePtr targetNode =
          scene->NodeByName(this->dataPtr->viewCollisionsTarget);
      auto targetVis = std::dynamic_pointer_cast<rendering::Visual>(targetNode);

      if (targetVis)
      {
        Entity targetEntity =
            std::get<int>(targetVis->UserData("gazebo-entity"));
        this->dataPtr->renderUtil.ViewCollisions(targetEntity);
      }
      else
      {
        ignerr << "Unable to find node name ["
               << this->dataPtr->viewCollisionsTarget
               << "] to view collisions" << std::endl;
      }

      this->dataPtr->viewCollisionsTarget.clear();
    }
  }

  if (ignition::gui::App())
  {
    ignition::gui::events::Render event;
    ignition::gui::App()->sendEvent(
        ignition::gui::App()->findChild<ignition::gui::MainWindow *>(),
        &event);

    IGN_UTILS_WARN_IGNORE__DEPRECATED_DECLARATION
    ignition::gazebo::gui::events::Render oldEvent;
    ignition::gui::App()->sendEvent(
        ignition::gui::App()->findChild<ignition::gui::MainWindow *>(),
        &oldEvent);
    IGN_UTILS_WARN_RESUME__DEPRECATED_DECLARATION
  }

  // only has an effect in video recording lockstep mode
  // this notifes ECM to continue updating the scene
  g_renderCv.notify_one();

  // TODO(anyone) implement a SwapFromThread for parallel command generation
  // See https://github.com/ignitionrobotics/ign-rendering/issues/304
  // if( bForcedSerialization )
  //   this->dataPtr->camera->SwapFromThread();
  // else
  //  _renderSync->ReleaseQtThreadFromBlock(lock);
  _renderSync->ReleaseQtThreadFromBlock(lock);
}

/////////////////////////////////////////////////
bool IgnRenderer::GeneratePreview(const sdf::Root &_sdf)
{
  // Terminate any pre-existing spawned entities
  this->TerminateSpawnPreview();

  if (nullptr == _sdf.Model() && nullptr == _sdf.Light())
  {
    ignwarn << "Only model entities can be spawned at the moment." << std::endl;
    this->TerminateSpawnPreview();
    return false;
  }

  if (_sdf.Model())
  {
    // Only preview first model
    sdf::Model model = *(_sdf.Model());
    this->dataPtr->spawnPreviewPose = model.RawPose();
    model.SetName(ignition::common::Uuid().String());
    Entity modelId = this->UniqueId();
    if (!modelId)
    {
      this->TerminateSpawnPreview();
      return false;
    }
    this->dataPtr->spawnPreview =
      this->dataPtr->renderUtil.SceneManager().CreateModel(
          modelId, model,
          this->dataPtr->renderUtil.SceneManager().WorldId());

    this->dataPtr->previewIds.push_back(modelId);
    for (auto j = 0u; j < model.LinkCount(); j++)
    {
      sdf::Link link = *(model.LinkByIndex(j));
      link.SetName(ignition::common::Uuid().String());
      Entity linkId = this->UniqueId();
      if (!linkId)
      {
        this->TerminateSpawnPreview();
        return false;
      }
      this->dataPtr->renderUtil.SceneManager().CreateLink(
          linkId, link, modelId);
      this->dataPtr->previewIds.push_back(linkId);
      for (auto k = 0u; k < link.VisualCount(); k++)
      {
       sdf::Visual visual = *(link.VisualByIndex(k));
       visual.SetName(ignition::common::Uuid().String());
       Entity visualId = this->UniqueId();
       if (!visualId)
       {
         this->TerminateSpawnPreview();
         return false;
       }
       this->dataPtr->renderUtil.SceneManager().CreateVisual(
           visualId, visual, linkId);
       this->dataPtr->previewIds.push_back(visualId);
      }
    }
  }
  else if (_sdf.Light())
  {
    // Only preview first model
    sdf::Light light = *(_sdf.Light());
    this->dataPtr->spawnPreviewPose = light.RawPose();
    light.SetName(ignition::common::Uuid().String());
    Entity lightVisualId = this->UniqueId();
    if (!lightVisualId)
    {
      this->TerminateSpawnPreview();
      return false;
    }
    Entity lightId = this->UniqueId();
    if (!lightId)
    {
      this->TerminateSpawnPreview();
      return false;
    }
    this->dataPtr->spawnPreview =
      this->dataPtr->renderUtil.SceneManager().CreateLight(
          lightId, light,
          this->dataPtr->renderUtil.SceneManager().WorldId());
    this->dataPtr->renderUtil.SceneManager().CreateLightVisual(
        lightVisualId, light, lightId);



    this->dataPtr->previewIds.push_back(lightId);
    this->dataPtr->previewIds.push_back(lightVisualId);
  }
  return true;
}

/////////////////////////////////////////////////
void IgnRenderer::TerminateSpawnPreview()
{
  for (auto _id : this->dataPtr->previewIds)
    this->dataPtr->renderUtil.SceneManager().RemoveEntity(_id);
  this->dataPtr->previewIds.clear();
  this->dataPtr->isPlacing = false;
}

/////////////////////////////////////////////////
Entity IgnRenderer::UniqueId()
{
  auto timeout = 100000u;
  for (auto i = 0u; i < timeout; ++i)
  {
    Entity id = std::numeric_limits<uint64_t>::max() - i;
    if (!this->dataPtr->renderUtil.SceneManager().HasEntity(id))
      return id;
  }
  return kNullEntity;
}

/////////////////////////////////////////////////
void IgnRenderer::HandleMouseEvent()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->BroadcastHoverPos();
  this->BroadcastLeftClick();
  this->BroadcastRightClick();
  this->HandleMouseContextMenu();
  this->HandleModelPlacement();
  this->HandleMouseTransformControl();
  this->HandleMouseViewControl();
}

/////////////////////////////////////////////////
void IgnRenderer::BroadcastHoverPos()
{
  if (this->dataPtr->hoverDirty)
  {
    math::Vector3d pos = this->ScreenToScene(this->dataPtr->mouseHoverPos);

    ignition::gui::events::HoverToScene hoverToSceneEvent(pos);
    ignition::gui::App()->sendEvent(
        ignition::gui::App()->findChild<ignition::gui::MainWindow *>(),
        &hoverToSceneEvent);
  }
}

/////////////////////////////////////////////////
void IgnRenderer::BroadcastLeftClick()
{
  if (this->dataPtr->mouseEvent.Button() == common::MouseEvent::LEFT &&
      this->dataPtr->mouseEvent.Type() == common::MouseEvent::RELEASE &&
      !this->dataPtr->mouseEvent.Dragging() && this->dataPtr->mouseDirty)
  {
    math::Vector3d pos = this->ScreenToScene(this->dataPtr->mouseEvent.Pos());

    ignition::gui::events::LeftClickToScene leftClickToSceneEvent(pos);
    ignition::gui::App()->sendEvent(
        ignition::gui::App()->findChild<ignition::gui::MainWindow *>(),
        &leftClickToSceneEvent);
  }
}

/////////////////////////////////////////////////
void IgnRenderer::BroadcastRightClick()
{
  if (this->dataPtr->mouseEvent.Button() == common::MouseEvent::RIGHT &&
      this->dataPtr->mouseEvent.Type() == common::MouseEvent::RELEASE &&
      !this->dataPtr->mouseEvent.Dragging() && this->dataPtr->mouseDirty)
  {
    // If the dropdown menu is disabled, quash the mouse event
    if (!this->dataPtr->dropdownMenuEnabled)
      this->dataPtr->mouseDirty = false;

    math::Vector3d pos = this->ScreenToScene(this->dataPtr->mouseEvent.Pos());

    ignition::gui::events::RightClickToScene rightClickToSceneEvent(pos);
    ignition::gui::App()->sendEvent(
        ignition::gui::App()->findChild<ignition::gui::MainWindow *>(),
        &rightClickToSceneEvent);
  }
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

////////////////////////////////////////////////
void IgnRenderer::HandleKeyPress(QKeyEvent *_e)
{
  if (_e->isAutoRepeat())
    return;

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  this->dataPtr->keyEvent.SetKey(_e->key());
  this->dataPtr->keyEvent.SetText(_e->text().toStdString());

  this->dataPtr->keyEvent.SetControl(
    (_e->modifiers() & Qt::ControlModifier));
  this->dataPtr->keyEvent.SetShift(
    (_e->modifiers() & Qt::ShiftModifier));
  this->dataPtr->keyEvent.SetAlt(
    (_e->modifiers() & Qt::AltModifier));

  this->dataPtr->mouseEvent.SetControl(this->dataPtr->keyEvent.Control());
  this->dataPtr->mouseEvent.SetShift(this->dataPtr->keyEvent.Shift());
  this->dataPtr->mouseEvent.SetAlt(this->dataPtr->keyEvent.Alt());
  this->dataPtr->keyEvent.SetType(common::KeyEvent::PRESS);

  // Update the object and mouse to be placed at the current position
  // only for x, y, and z key presses
  if (_e->key() == Qt::Key_X ||
      _e->key() == Qt::Key_Y ||
      _e->key() == Qt::Key_Z ||
      _e->key() == Qt::Key_Shift)
  {
    this->dataPtr->transformControl.Start();
    this->dataPtr->mousePressPos = this->dataPtr->mouseEvent.Pos();
  }

  // fullscreen
  if (_e->key() == Qt::Key_F11)
  {
    if (ignition::gui::App()->findChild
        <ignition::gui::MainWindow *>()->QuickWindow()->visibility()
        == QWindow::FullScreen)
    {
      ignition::gui::App()->findChild
        <ignition::gui::MainWindow *>()->QuickWindow()->showNormal();
    }
    else
    {
      ignition::gui::App()->findChild
        <ignition::gui::MainWindow *>()->QuickWindow()->showFullScreen();
    }
  }

  switch (_e->key())
  {
    case Qt::Key_X:
      this->dataPtr->xPressed = true;
      break;
    case Qt::Key_Y:
      this->dataPtr->yPressed = true;
      break;
    case Qt::Key_Z:
      this->dataPtr->zPressed = true;
      break;
    default:
      break;
  }
}

////////////////////////////////////////////////
void IgnRenderer::HandleKeyRelease(QKeyEvent *_e)
{
  if (_e->isAutoRepeat())
    return;

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  this->dataPtr->keyEvent.SetKey(0);

  this->dataPtr->keyEvent.SetControl(
    (_e->modifiers() & Qt::ControlModifier)
    && (_e->key() != Qt::Key_Control));
  this->dataPtr->keyEvent.SetShift(
    (_e->modifiers() & Qt::ShiftModifier)
    && (_e->key() != Qt::Key_Shift));
  this->dataPtr->keyEvent.SetAlt(
    (_e->modifiers() & Qt::AltModifier)
    && (_e->key() != Qt::Key_Alt));

  this->dataPtr->mouseEvent.SetControl(this->dataPtr->keyEvent.Control());
  this->dataPtr->mouseEvent.SetShift(this->dataPtr->keyEvent.Shift());
  this->dataPtr->mouseEvent.SetAlt(this->dataPtr->keyEvent.Alt());
  this->dataPtr->keyEvent.SetType(common::KeyEvent::RELEASE);

  // Update the object and mouse to be placed at the current position
  // only for x, y, and z key presses
  if (_e->key() == Qt::Key_X ||
      _e->key() == Qt::Key_Y ||
      _e->key() == Qt::Key_Z ||
      _e->key() == Qt::Key_Shift)
  {
    this->dataPtr->transformControl.Start();
    this->dataPtr->mousePressPos = this->dataPtr->mouseEvent.Pos();
    this->dataPtr->isStartWorldPosSet = false;
  }

  switch (_e->key())
  {
    case Qt::Key_X:
      this->dataPtr->xPressed = false;
      break;
    case Qt::Key_Y:
      this->dataPtr->yPressed = false;
      break;
    case Qt::Key_Z:
      this->dataPtr->zPressed = false;
      break;
    case Qt::Key_Escape:
      this->dataPtr->escapeReleased = true;
      break;
    default:
      break;
  }
}

/////////////////////////////////////////////////
void IgnRenderer::HandleModelPlacement()
{
  if (!this->dataPtr->isPlacing)
    return;

  if (this->dataPtr->spawnPreview && this->dataPtr->hoverDirty)
  {
    math::Vector3d pos = this->ScreenToPlane(this->dataPtr->mouseHoverPos);
    pos.Z(this->dataPtr->spawnPreview->WorldPosition().Z());
    this->dataPtr->spawnPreview->SetWorldPosition(pos);
    this->dataPtr->hoverDirty = false;
  }
  if (this->dataPtr->mouseEvent.Button() == common::MouseEvent::LEFT &&
      this->dataPtr->mouseEvent.Type() == common::MouseEvent::RELEASE &&
      !this->dataPtr->mouseEvent.Dragging() && this->dataPtr->mouseDirty)
  {
    // Delete the generated visuals
    this->TerminateSpawnPreview();

    math::Pose3d modelPose = this->dataPtr->spawnPreviewPose;
    std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
        [](const ignition::msgs::Boolean &/*_rep*/, const bool _result)
    {
      if (!_result)
        ignerr << "Error creating model" << std::endl;
    };
    math::Vector3d pos = this->ScreenToPlane(this->dataPtr->mouseEvent.Pos());
    pos.Z(modelPose.Pos().Z());
    msgs::EntityFactory req;
    if (!this->dataPtr->spawnSdfString.empty())
    {
      req.set_sdf(this->dataPtr->spawnSdfString);
    }
    else if (!this->dataPtr->spawnSdfPath.empty())
    {
      req.set_sdf_filename(this->dataPtr->spawnSdfPath);
    }
    else
    {
      ignwarn << "Failed to find SDF string or file path" << std::endl;
    }
    req.set_allow_renaming(true);
    msgs::Set(req.mutable_pose(), math::Pose3d(pos, modelPose.Rot()));

    if (this->dataPtr->createCmdService.empty())
    {
      this->dataPtr->createCmdService = "/world/" + this->worldName
          + "/create";
    }
    this->dataPtr->createCmdService = transport::TopicUtils::AsValidTopic(
        this->dataPtr->createCmdService);
    if (this->dataPtr->createCmdService.empty())
    {
      ignerr << "Failed to create valid create command service for world ["
             << this->worldName <<"]" << std::endl;
      return;
    }

    this->dataPtr->node.Request(this->dataPtr->createCmdService, req, cb);
    this->dataPtr->isPlacing = false;
    this->dataPtr->mouseDirty = false;
    this->dataPtr->spawnSdfString.clear();
    this->dataPtr->spawnSdfPath.clear();
  }
}

/////////////////////////////////////////////////
void IgnRenderer::HandleEntitySelection()
{
  if (this->dataPtr->selectionHelper.deselectAll)
  {
    this->DeselectAllEntities(this->dataPtr->selectionHelper.sendEvent);

    this->dataPtr->selectionHelper = SelectionHelper();
  }
  else if (this->dataPtr->selectionHelper.selectEntity != kNullEntity)
  {
    auto node = this->dataPtr->renderUtil.SceneManager().NodeById(
      this->dataPtr->selectionHelper.selectEntity);
    this->UpdateSelectedEntity(node,
        this->dataPtr->selectionHelper.sendEvent);

    this->dataPtr->selectionHelper = SelectionHelper();
  }
}

/////////////////////////////////////////////////
void IgnRenderer::DeselectAllEntities(bool _sendEvent)
{
  if (this->dataPtr->renderThreadId != std::this_thread::get_id())
  {
    ignwarn << "Making render calls from outside the render thread"
            << std::endl;
  }

  this->dataPtr->renderUtil.DeselectAllEntities();

  if (_sendEvent)
  {
    ignition::gazebo::gui::events::DeselectAllEntities deselectEvent;
    ignition::gui::App()->sendEvent(
        ignition::gui::App()->findChild<ignition::gui::MainWindow *>(),
        &deselectEvent);
  }
}

/////////////////////////////////////////////////
double IgnRenderer::SnapValue(
    double _coord, double _interval, double _sensitivity) const
{
  double snap = _interval * _sensitivity;
  double rem = fmod(_coord, _interval);
  double minInterval = _coord - rem;

  if (rem < 0)
  {
    minInterval -= _interval;
  }

  double maxInterval = minInterval + _interval;

  if (_coord < (minInterval + snap))
  {
    _coord = minInterval;
  }
  else if (_coord > (maxInterval - snap))
  {
    _coord = maxInterval;
  }

  return _coord;
}

/////////////////////////////////////////////////
void IgnRenderer::SnapPoint(
    ignition::math::Vector3d &_point, math::Vector3d &_snapVals,
    double _sensitivity) const
{
  if (_snapVals.X() <= 0 || _snapVals.Y() <= 0 || _snapVals.Z() <= 0)
  {
    ignerr << "Interval distance must be greater than 0"
        << std::endl;
    return;
  }

  if (_sensitivity < 0 || _sensitivity > 1.0)
  {
    ignerr << "Sensitivity must be between 0 and 1" << std::endl;
    return;
  }

  _point.X() = this->SnapValue(_point.X(), _snapVals.X(), _sensitivity);
  _point.Y() = this->SnapValue(_point.Y(), _snapVals.Y(), _sensitivity);
  _point.Z() = this->SnapValue(_point.Z(), _snapVals.Z(), _sensitivity);
}

/////////////////////////////////////////////////
void IgnRenderer::XYZConstraint(math::Vector3d &_axis)
{
  math::Vector3d translationAxis = math::Vector3d::Zero;

  if (this->dataPtr->xPressed)
  {
    translationAxis += math::Vector3d::UnitX;
  }

  if (this->dataPtr->yPressed)
  {
    translationAxis += math::Vector3d::UnitY;
  }

  if (this->dataPtr->zPressed)
  {
    translationAxis += math::Vector3d::UnitZ;
  }

  if (translationAxis != math::Vector3d::Zero)
  {
    _axis = translationAxis;
  }
}

/////////////////////////////////////////////////
void IgnRenderer::HandleMouseTransformControl()
{
  if (this->dataPtr->renderThreadId != std::this_thread::get_id())
  {
    ignwarn << "Making render calls from outside the render thread"
            << std::endl;
  }

  // set transform configuration
  this->dataPtr->transformControl.SetTransformMode(
      this->dataPtr->transformMode);

  if (!this->dataPtr->transformControl.Camera())
    this->dataPtr->transformControl.SetCamera(this->dataPtr->camera);

  // stop and detach transform controller if mode is none or no entity is
  // selected
  if (this->dataPtr->transformMode == rendering::TransformMode::TM_NONE ||
      (this->dataPtr->transformControl.Node() &&
      this->dataPtr->renderUtil.SelectedEntities().empty()))
  {
    if (this->dataPtr->transformControl.Active())
      this->dataPtr->transformControl.Stop();

    this->dataPtr->transformControl.Detach();
  }
  else
  {
    // shift indicates world space transformation
    this->dataPtr->transformSpace = (this->dataPtr->keyEvent.Shift()) ?
        rendering::TransformSpace::TS_WORLD :
        rendering::TransformSpace::TS_LOCAL;
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
      this->dataPtr->mousePressPos = this->dataPtr->mouseEvent.Pos();
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
      this->dataPtr->isStartWorldPosSet = false;
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
          this->dataPtr->poseCmdService = transport::TopicUtils::AsValidTopic(
              this->dataPtr->poseCmdService);
          if (this->dataPtr->poseCmdService.empty())
          {
            ignerr << "Failed to create valid pose command service for world ["
                   << this->worldName <<"]" << std::endl;
            return;
          }
          this->dataPtr->node.Request(this->dataPtr->poseCmdService, req, cb);
        }

        this->dataPtr->transformControl.Stop();
        this->dataPtr->mouseDirty = false;
      }
      // Select entity
      else if (!this->dataPtr->mouseEvent.Dragging())
      {
        rendering::VisualPtr visual = this->dataPtr->camera->Scene()->VisualAt(
              this->dataPtr->camera,
              this->dataPtr->mouseEvent.Pos());

        if (!visual)
        {
          // Hit the background, deselect all
          if (!this->dataPtr->mouseEvent.Dragging())
          {
            this->DeselectAllEntities(true);
          }
          return;
        }

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
            // Highlight entity and notify other widgets
            this->UpdateSelectedEntity(topVis, true);

            this->dataPtr->mouseDirty = false;
            return;
          }
          // Don't deselect after dragging, user may be orbiting the camera
          else if (!this->dataPtr->mouseEvent.Dragging())
          {
            // Hit the ground, deselect all
            this->DeselectAllEntities(true);
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
    double nx = 2.0 *
      this->dataPtr->mousePressPos.X() /
      imageWidth - 1.0;
    double ny = 1.0 - 2.0 *
      this->dataPtr->mousePressPos.Y() /
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
      Entity nodeId = this->dataPtr->renderUtil.SelectedEntities().front();
      rendering::NodePtr target =
          this->dataPtr->renderUtil.SceneManager().NodeById(nodeId);
      if (!target)
      {
        ignwarn << "Failed to find node with ID [" << nodeId << "]"
                << std::endl;
        return;
      }
      this->XYZConstraint(axis);
      if (!this->dataPtr->isStartWorldPosSet)
      {
        this->dataPtr->isStartWorldPosSet = true;
        this->dataPtr->startWorldPos =
          target->WorldPosition();
      }
      ignition::math::Vector3d worldPos =
        target->WorldPosition();
      math::Vector3d distance =
        this->dataPtr->transformControl.TranslationFrom2d(axis, start, end);
      if (this->dataPtr->keyEvent.Control())
      {
        // Translate to world frame for snapping
        distance += this->dataPtr->startWorldPos;
        math::Vector3d snapVals = this->XYZSnap();

        // Constrain snap values to a minimum of 1e-4
        snapVals.X() = std::max(1e-4, snapVals.X());
        snapVals.Y() = std::max(1e-4, snapVals.Y());
        snapVals.Z() = std::max(1e-4, snapVals.Z());

        SnapPoint(distance, snapVals);

        // Translate back to entity frame
        distance -= this->dataPtr->startWorldPos;
        distance *= axis;
      }
      this->dataPtr->transformControl.Translate(distance);
    }
    else if (this->dataPtr->transformControl.Mode() ==
        rendering::TransformMode::TM_ROTATION)
    {
      math::Quaterniond rotation =
          this->dataPtr->transformControl.RotationFrom2d(axis, start, end);

      if (this->dataPtr->keyEvent.Control())
      {
        math::Vector3d currentRot = rotation.Euler();
        math::Vector3d snapVals = this->RPYSnap();

        if (snapVals.X() <= 1e-4)
        {
          snapVals.X() = IGN_PI/4;
        }
        else
        {
          snapVals.X() = IGN_DTOR(snapVals.X());
        }
        if (snapVals.Y() <= 1e-4)
        {
          snapVals.Y() = IGN_PI/4;
        }
        else
        {
          snapVals.Y() = IGN_DTOR(snapVals.Y());
        }
        if (snapVals.Z() <= 1e-4)
        {
          snapVals.Z() = IGN_PI/4;
        }
        else
        {
          snapVals.Z() = IGN_DTOR(snapVals.Z());
        }

        SnapPoint(currentRot, snapVals);
        rotation = math::Quaterniond::EulerToQuaternion(currentRot);
      }
      this->dataPtr->transformControl.Rotate(rotation);
    }
    else if (this->dataPtr->transformControl.Mode() ==
        rendering::TransformMode::TM_SCALE)
    {
      this->XYZConstraint(axis);
      // note: scaling is limited to local space
      math::Vector3d scale =
          this->dataPtr->transformControl.ScaleFrom2d(axis, start, end);
      if (this->dataPtr->keyEvent.Control())
      {
        math::Vector3d snapVals = this->ScaleSnap();

        if (snapVals.X() <= 1e-4)
          snapVals.X() = 0.1;
        if (snapVals.Y() <= 1e-4)
          snapVals.Y() = 0.1;
        if (snapVals.Z() <= 1e-4)
          snapVals.Z() = 0.1;

        SnapPoint(scale, snapVals);
      }
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

  if (this->dataPtr->renderThreadId != std::this_thread::get_id())
  {
    ignwarn << "Making render calls from outside the render thread"
            << std::endl;
  }

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
    if (this->dataPtr->mouseEvent.Type() == common::MouseEvent::PRESS ||
        // the rendering thread may miss the press event due to
        // race condition when doing a drag operation (press and move, where
        // the move event overrides the press event before it is processed)
        // so we double check to see if target is set or not
        (this->dataPtr->mouseEvent.Type() == common::MouseEvent::MOVE &&
        this->dataPtr->mouseEvent.Dragging() &&
        std::isinf(this->dataPtr->target.X())))
    {
      this->dataPtr->target = this->ScreenToScene(
          this->dataPtr->mouseEvent.PressPos());
      this->dataPtr->viewControl.SetTarget(this->dataPtr->target);
    }
    // unset the target on release (by setting to inf)
    else if (this->dataPtr->mouseEvent.Type() == common::MouseEvent::RELEASE)
    {
      this->dataPtr->target = ignition::math::INF_D;
    }

    // Pan with left button
    if (this->dataPtr->mouseEvent.Buttons() & common::MouseEvent::LEFT)
    {
      if (Qt::ShiftModifier == QGuiApplication::queryKeyboardModifiers())
        this->dataPtr->viewControl.Orbit(this->dataPtr->drag);
      else
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
    this->dataPtr->followOffsetDirty = true;
}

/////////////////////////////////////////////////
void IgnRenderer::Initialize()
{
  if (this->initialized)
    return;

  this->dataPtr->renderUtil.SetUseCurrentGLContext(true);
  this->dataPtr->renderUtil.Init();

  rendering::ScenePtr scene = this->dataPtr->renderUtil.Scene();
  if (!scene)
    return;

  auto root = scene->RootVisual();

  // Camera
  this->dataPtr->camera = scene->CreateCamera();
  root->AddChild(this->dataPtr->camera);
  this->dataPtr->camera->SetLocalPose(this->cameraPose);
  this->dataPtr->camera->SetImageWidth(this->textureSize.width());
  this->dataPtr->camera->SetImageHeight(this->textureSize.height());
  this->dataPtr->camera->SetAntiAliasing(8);
  this->dataPtr->camera->SetHFOV(M_PI * 0.5);
  this->dataPtr->camera->SetVisibilityMask(this->visibilityMask);
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
void IgnRenderer::SetXYZSnap(const math::Vector3d &_xyz)
{
  this->dataPtr->xyzSnap = _xyz;
}

/////////////////////////////////////////////////
math::Vector3d IgnRenderer::XYZSnap() const
{
  return this->dataPtr->xyzSnap;
}

/////////////////////////////////////////////////
void IgnRenderer::SetRPYSnap(const math::Vector3d &_rpy)
{
  this->dataPtr->rpySnap = _rpy;
}

/////////////////////////////////////////////////
math::Vector3d IgnRenderer::RPYSnap() const
{
  return this->dataPtr->rpySnap;
}

/////////////////////////////////////////////////
void IgnRenderer::SetScaleSnap(const math::Vector3d &_scale)
{
  this->dataPtr->scaleSnap = _scale;
}

/////////////////////////////////////////////////
math::Vector3d IgnRenderer::ScaleSnap() const
{
  return this->dataPtr->scaleSnap;
}
/////////////////////////////////////////////////
void IgnRenderer::UpdateSelectedEntity(const rendering::NodePtr &_node,
    bool _sendEvent)
{
  if (!_node)
    return;

  if (this->dataPtr->renderThreadId != std::this_thread::get_id())
  {
    ignwarn << "Making render calls from outside the render thread"
            << std::endl;
  }

  bool deselectedAll{false};

  // Deselect all if control is not being held
  if (!(QGuiApplication::keyboardModifiers() & Qt::ControlModifier) &&
      !this->dataPtr->renderUtil.SelectedEntities().empty())
  {
    // Notify other widgets regardless of _sendEvent, because this is a new
    // decision from this widget
    this->DeselectAllEntities(true);
    deselectedAll = true;
  }

  // Attach control if in a transform mode - control is attached to:
  // * latest selection
  // * top-level nodes (model, light...)
  if (this->dataPtr->transformMode != rendering::TransformMode::TM_NONE)
  {
    auto topNode =
        this->dataPtr->renderUtil.SceneManager().TopLevelNode(_node);
    if (topNode == _node)
    {
      this->dataPtr->transformControl.Attach(_node);

      // When attached, we want only one entity selected
      // Notify other widgets regardless of _sendEvent, because this is a new
      // decision from this widget
      this->DeselectAllEntities(true);
      deselectedAll = true;
    }
    else
    {
      this->dataPtr->transformControl.Detach();
    }
  }

  // Select new entity
  this->dataPtr->renderUtil.SetSelectedEntity(_node);

  // Notify other widgets of the currently selected entities
  if (_sendEvent || deselectedAll)
  {
    ignition::gazebo::gui::events::EntitiesSelected selectEvent(
        this->dataPtr->renderUtil.SelectedEntities());
    ignition::gui::App()->sendEvent(
        ignition::gui::App()->findChild<ignition::gui::MainWindow *>(),
        &selectEvent);
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

  // Update selected entities if transform control is changed
  if (!this->dataPtr->renderUtil.SelectedEntities().empty())
  {
    Entity entity = this->dataPtr->renderUtil.SelectedEntities().back();
    this->dataPtr->selectionHelper = {entity, false, false};
  }
}

/////////////////////////////////////////////////
void IgnRenderer::SetModel(const std::string &_model)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->isSpawning = true;
  this->dataPtr->spawnSdfString = _model;
}

/////////////////////////////////////////////////
void IgnRenderer::SetModelPath(const std::string &_filePath)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->isSpawning = true;
  this->dataPtr->spawnSdfPath = _filePath;
}

/////////////////////////////////////////////////
void IgnRenderer::SetDropdownMenuEnabled(bool _enableDropdownMenu)
{
  this->dataPtr->dropdownMenuEnabled = _enableDropdownMenu;
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
void IgnRenderer::SetRecordVideoUseSimTime(bool _useSimTime)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->recordVideoUseSimTime = _useSimTime;
}

/////////////////////////////////////////////////
void IgnRenderer::SetRecordVideoLockstep(bool _useSimTime)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->recordVideoLockstep = _useSimTime;
}

/////////////////////////////////////////////////
void IgnRenderer::SetRecordVideoBitrate(unsigned int _bitrate)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->recordVideoBitrate = _bitrate;
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
void IgnRenderer::SetViewAngle(const math::Vector3d &_direction)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->viewAngle = true;
  this->dataPtr->viewAngleDirection = _direction;
}

/////////////////////////////////////////////////
void IgnRenderer::SetMoveToPose(const math::Pose3d &_pose)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->moveToPoseValue = _pose;
}

/////////////////////////////////////////////////
void IgnRenderer::SetViewCollisionsTarget(const std::string &_target)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->viewCollisionsTarget = _target;
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
void IgnRenderer::SetInitCameraPose(const math::Pose3d &_pose)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->moveToHelper.SetInitCameraPose(_pose);
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

  if (!this->dataPtr->followTarget.empty())
    this->dataPtr->newFollowOffset = true;
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
void IgnRenderer::OnViewAngleComplete()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->viewAngle = false;
}

/////////////////////////////////////////////////
void IgnRenderer::OnMoveToPoseComplete()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->moveToPoseValue.reset();
}

/////////////////////////////////////////////////
void IgnRenderer::NewHoverEvent(const math::Vector2i &_hoverPos)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->mouseHoverPos = _hoverPos;
  this->dataPtr->hoverDirty = true;
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
math::Vector3d IgnRenderer::ScreenToPlane(
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

  ignition::math::Planed plane(ignition::math::Vector3d(0, 0, 1), 0);

  math::Vector3d origin = this->dataPtr->rayQuery->Origin();
  math::Vector3d direction = this->dataPtr->rayQuery->Direction();
  double distance = plane.Distance(origin, direction);
  return origin + direction * distance;
}

/////////////////////////////////////////////////
math::Pose3d IgnRenderer::CameraPose() const
{
  if (this->dataPtr->camera)
    return this->dataPtr->camera->WorldPose();
  return math::Pose3d::Zero;
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

////////////////////////////////////////////////
void IgnRenderer::RequestSelectionChange(Entity _selectedEntity,
    bool _deselectAll, bool _sendEvent)
{
  this->dataPtr->selectionHelper = {_selectedEntity, _deselectAll, _sendEvent};
}

/////////////////////////////////////////////////
RenderThread::RenderThread()
{
  RenderWindowItemPrivate::threads << this;
  qRegisterMetaType<std::string>();
  qRegisterMetaType<RenderSync*>("RenderSync*");
}

/////////////////////////////////////////////////
void RenderThread::RenderNext(RenderSync *_renderSync)
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

  this->ignRenderer.Render(_renderSync);

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
  this->exit();
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
TextureNode::TextureNode(QQuickWindow *_window, RenderSync &_renderSync)
    : renderSync(_renderSync), window(_window)
{
  // Our texture node must have a texture, so use the default 0 texture.
#if QT_VERSION < QT_VERSION_CHECK(5, 14, 0)
  this->texture = this->window->createTextureFromId(0, QSize(1, 1));
#else
  void * nativeLayout;
  this->texture = this->window->createTextureFromNativeObject(
      QQuickWindow::NativeObjectTexture, &nativeLayout, 0, QSize(1, 1),
      QQuickWindow::TextureIsOpaque);
#endif
  this->setTexture(this->texture);
}

/////////////////////////////////////////////////
TextureNode::~TextureNode()
{
  delete this->texture;
}

/////////////////////////////////////////////////
void TextureNode::NewTexture(uint _id, const QSize &_size)
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
  uint newId = this->id;
  QSize sz = this->size;
  this->id = 0;
  this->mutex.unlock();
  if (newId)
  {
    delete this->texture;
    // note: include QQuickWindow::TextureHasAlphaChannel if the rendered
    // content has alpha.
#if QT_VERSION < QT_VERSION_CHECK(5, 14, 0)
    this->texture = this->window->createTextureFromId(
        newId, sz, QQuickWindow::TextureIsOpaque);
#else
    // TODO(anyone) Use createTextureFromNativeObject
    // https://github.com/ignitionrobotics/ign-gui/issues/113
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
    this->texture = this->window->createTextureFromId(
        newId, sz, QQuickWindow::TextureIsOpaque);
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#endif
    this->setTexture(this->texture);

    this->markDirty(DirtyMaterial);

    // This will notify the rendering thread that the texture is now being
    // rendered and it can start rendering to the other one.
    // emit TextureInUse(&this->renderSync); See comment below
  }

  // NOTE: The original code from Qt samples only emitted when
  // newId is not null.
  //
  // This is correct... for their case.
  // However we need to synchronize the threads when resolution changes,
  // and we're also currently doing everything in lockstep (i.e. both Qt
  // and worker thread are serialized,
  // see https://github.com/ignitionrobotics/ign-rendering/issues/304 )
  //
  // We need to emit even if newId == 0 because it's safe as long as both
  // threads are forcefully serialized and otherwise we may get a
  // deadlock (this func. called twice in a row with the worker thread still
  // finishing the 1st iteration, may result in a deadlock for newer versions
  // of Qt; as WaitForWorkerThread will be called with no corresponding
  // WaitForQtThreadAndBlock as the worker thread thinks there are
  // no more jobs to do.
  //
  // If we want these to run in worker thread and stay resolution-synchronized,
  // we probably should use a different method of signals and slots
  // to send work to the worker thread and get results back
  emit TextureInUse(&this->renderSync);

  this->renderSync.WaitForWorkerThread();
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
RenderWindowItem::~RenderWindowItem()
{
  this->dataPtr->renderSync.Shutdown();
  QMetaObject::invokeMethod(this->dataPtr->renderThread,
                            "ShutDown",
                            Qt::QueuedConnection);

  this->dataPtr->renderThread->wait();
}

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

  this->connect(this, &QQuickItem::widthChanged,
      this->dataPtr->renderThread, &RenderThread::SizeChanged);
  this->connect(this, &QQuickItem::heightChanged,
      this->dataPtr->renderThread, &RenderThread::SizeChanged);

  this->dataPtr->renderThread->start();
  this->update();

  this->dataPtr->rendererInit = true;
}

/////////////////////////////////////////////////
bool RenderWindowItem::RendererInitialized() const
{
  return this->dataPtr->rendererInit;
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
    if (this->RenderUtil()->EngineName() == "ogre2")
    {
      // Although it seems unbelievable, we can request another format for a
      // shared context; it is needed because Qt selects by default a compat
      // context which is much less likely to provide OpenGL 3.3 (only closed
      // NVidia drivers). This means there will be mismatch between what is
      // reported by QSG_INFO=1 and by OGRE.
      auto surfaceFormat = QSurfaceFormat();
      surfaceFormat.setMajorVersion(3);
      surfaceFormat.setMinorVersion(3);
      surfaceFormat.setProfile(QSurfaceFormat::CoreProfile);
      this->dataPtr->renderThread->context->setFormat(surfaceFormat);
    }
    else
    {
      this->dataPtr->renderThread->context->setFormat(current->format());
    }
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
    node = new TextureNode(this->window(), this->dataPtr->renderSync);

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
                              Qt::QueuedConnection,
                              Q_ARG( RenderSync*, &node->renderSync ));
  }

  node->setRect(this->boundingRect());

  return node;
}

///////////////////////////////////////////////////
void RenderWindowItem::OnContextMenuRequested(QString _entity)
{
  emit openContextMenu(std::move(_entity));
}

///////////////////////////////////////////////////
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

    if (auto elem = _pluginElem->FirstChildElement("sky"))
    {
      this->dataPtr->renderUtil->SetSkyEnabled(true);
      if (!elem->NoChildren())
        ignwarn << "Child elements of <sky> are not supported yet" << std::endl;
    }

    if (auto elem = _pluginElem->FirstChildElement("camera_pose"))
    {
      math::Pose3d pose;
      std::stringstream poseStr;
      poseStr << std::string(elem->GetText());
      poseStr >> pose;
      renderWindow->SetInitCameraPose(pose);
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

    if (auto elem = _pluginElem->FirstChildElement("record_video"))
    {
      if (auto useSimTimeElem = elem->FirstChildElement("use_sim_time"))
      {
        bool useSimTime = false;
        if (useSimTimeElem->QueryBoolText(&useSimTime) != tinyxml2::XML_SUCCESS)
        {
          ignerr << "Faild to parse <use_sim_time> value: "
                 << useSimTimeElem->GetText() << std::endl;
        }
        else
        {
          renderWindow->SetRecordVideoUseSimTime(useSimTime);
        }
      }
      if (auto lockstepElem = elem->FirstChildElement("lockstep"))
      {
        bool lockstep = false;
        if (lockstepElem->QueryBoolText(&lockstep) != tinyxml2::XML_SUCCESS)
        {
          ignerr << "Failed to parse <lockstep> value: "
                 << lockstepElem->GetText() << std::endl;
        }
        else
        {
          renderWindow->SetRecordVideoLockstep(lockstep);
        }
      }
      if (auto bitrateElem = elem->FirstChildElement("bitrate"))
      {
        unsigned int bitrate = 0u;
        std::stringstream bitrateStr;
        bitrateStr << std::string(bitrateElem->GetText());
        bitrateStr >> bitrate;
        if (bitrate > 0u)
        {
          renderWindow->SetRecordVideoBitrate(bitrate);
        }
        else
        {
          ignerr << "Video recorder bitrate must be larger than 0"
                 << std::endl;
        }
      }
    }

    if (auto elem = _pluginElem->FirstChildElement("fullscreen"))
    {
      auto fullscreen = false;
      elem->QueryBoolText(&fullscreen);
      if (fullscreen)
      {
        ignition::gui::App()->findChild
          <ignition::gui::MainWindow *>()->QuickWindow()->showFullScreen();
      }
    }

    if (auto elem = _pluginElem->FirstChildElement("visibility_mask"))
    {
      uint32_t visibilityMask = 0xFFFFFFFFu;
      std::stringstream visibilityMaskStr;
      visibilityMaskStr << std::string(elem->GetText());
      bool isHex = common::lowercase(
          visibilityMaskStr.str()).compare(0, 2, "0x") == 0;
      if (isHex)
        visibilityMaskStr >> std::hex >> visibilityMask;
      else
        visibilityMaskStr >> visibilityMask;
      renderWindow->SetVisibilityMask(visibilityMask);
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

  // follow offset
  this->dataPtr->followOffsetService = "/gui/follow/offset";
  this->dataPtr->node.Advertise(this->dataPtr->followOffsetService,
      &Scene3D::OnFollowOffset, this);
  ignmsg << "Follow offset service on ["
         << this->dataPtr->followOffsetService << "]" << std::endl;

  // view angle
  this->dataPtr->viewAngleService =
      "/gui/view_angle";
  this->dataPtr->node.Advertise(this->dataPtr->viewAngleService,
      &Scene3D::OnViewAngle, this);
  ignmsg << "View angle service on ["
         << this->dataPtr->viewAngleService << "]" << std::endl;

  // move to pose service
  this->dataPtr->moveToPoseService =
      "/gui/move_to/pose";
  this->dataPtr->node.Advertise(this->dataPtr->moveToPoseService,
      &Scene3D::OnMoveToPose, this);
  ignmsg << "Move to pose service on ["
         << this->dataPtr->moveToPoseService << "]" << std::endl;

  // camera position topic
  this->dataPtr->cameraPoseTopic = "/gui/camera/pose";
  this->dataPtr->cameraPosePub =
    this->dataPtr->node.Advertise<msgs::Pose>(this->dataPtr->cameraPoseTopic);
  ignmsg << "Camera pose topic advertised on ["
         << this->dataPtr->cameraPoseTopic << "]" << std::endl;

  // view collisions service
  this->dataPtr->viewCollisionsService = "/gui/view/collisions";
  this->dataPtr->node.Advertise(this->dataPtr->viewCollisionsService,
      &Scene3D::OnViewCollisions, this);
  ignmsg << "View collisions service on ["
         << this->dataPtr->viewCollisionsService << "]" << std::endl;

  ignition::gui::App()->findChild<
      ignition::gui::MainWindow *>()->QuickWindow()->installEventFilter(this);
  ignition::gui::App()->findChild<
      ignition::gui::MainWindow *>()->installEventFilter(this);
}

//////////////////////////////////////////////////
void Scene3D::Update(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  if (nullptr == this->dataPtr->renderUtil)
    return;

  IGN_PROFILE("Scene3D::Update");
  auto renderWindow = this->PluginItem()->findChild<RenderWindowItem *>();
  if (this->dataPtr->worldName.empty())
  {
    // TODO(anyone) Only one scene is supported for now
    Entity worldEntity;
    _ecm.Each<components::World, components::Name>(
        [&](const Entity &_entity,
          const components::World * /* _world */ ,
          const components::Name *_name)->bool
        {
          this->dataPtr->worldName = _name->Data();
          worldEntity = _entity;
          return true;
        });

    if (!this->dataPtr->worldName.empty())
    {
      renderWindow->SetWorldName(this->dataPtr->worldName);
      auto renderEngineGuiComp =
        _ecm.Component<components::RenderEngineGuiPlugin>(worldEntity);
      if (renderEngineGuiComp && !renderEngineGuiComp->Data().empty())
      {
        this->dataPtr->renderUtil->SetEngineName(renderEngineGuiComp->Data());
      }
      else
      {
        igndbg << "RenderEngineGuiPlugin component not found, "
          "render engine won't be set from the ECM " << std::endl;
      }
    }
  }

  if (this->dataPtr->cameraPosePub.HasConnections())
  {
    msgs::Pose poseMsg = msgs::Convert(renderWindow->CameraPose());
    this->dataPtr->cameraPosePub.Publish(poseMsg);
  }
  this->dataPtr->renderUtil->UpdateECM(_info, _ecm);
  this->dataPtr->renderUtil->UpdateFromECM(_info, _ecm);

  // check if video recording is enabled and if we need to lock step
  // ECM updates with GUI rendering during video recording
  std::unique_lock<std::mutex> lock(this->dataPtr->recordMutex);
  if (this->dataPtr->recording && this->dataPtr->recordVideoLockstep &&
      renderWindow->RendererInitialized())
  {
    std::unique_lock<std::mutex> lock2(this->dataPtr->renderMutex);
    g_renderCv.wait(lock2);
  }
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

  std::unique_lock<std::mutex> lock(this->dataPtr->recordMutex);
  this->dataPtr->recording = record;
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
bool Scene3D::OnFollowOffset(const msgs::Vector3d &_msg,
  msgs::Boolean &_res)
{
  auto renderWindow = this->PluginItem()->findChild<RenderWindowItem *>();

  math::Vector3d offset = msgs::Convert(_msg);
  renderWindow->SetFollowOffset(offset);

  _res.set_data(true);
  return true;
}

/////////////////////////////////////////////////
bool Scene3D::OnViewAngle(const msgs::Vector3d &_msg,
  msgs::Boolean &_res)
{
  auto renderWindow = this->PluginItem()->findChild<RenderWindowItem *>();

  renderWindow->SetViewAngle(msgs::Convert(_msg));

  _res.set_data(true);
  return true;
}

/////////////////////////////////////////////////
bool Scene3D::OnMoveToPose(const msgs::GUICamera &_msg, msgs::Boolean &_res)
{
  auto renderWindow = this->PluginItem()->findChild<RenderWindowItem *>();

  math::Pose3d pose = msgs::Convert(_msg.pose());

  // If there is no orientation in the message, then set a Rot value in the
  // math::Pose3d object to infinite. This will prevent the orientation from
  // being used when positioning the camera.
  // See the MoveToHelper::MoveTo function
  if (!_msg.pose().has_orientation())
    pose.Rot().X() = math::INF_D;

  // If there is no position in the message, then set a Pos value in the
  // math::Pose3d object to infinite. This will prevent the orientation from
  // being used when positioning the camera.
  // See the MoveToHelper::MoveTo function
  if (!_msg.pose().has_position())
    pose.Pos().X() = math::INF_D;

  renderWindow->SetMoveToPose(pose);

  _res.set_data(true);
  return true;
}

/////////////////////////////////////////////////
bool Scene3D::OnViewCollisions(const msgs::StringMsg &_msg,
  msgs::Boolean &_res)
{
  auto renderWindow = this->PluginItem()->findChild<RenderWindowItem *>();

  renderWindow->SetViewCollisionsTarget(_msg.data());

  _res.set_data(true);
  return true;
}

/////////////////////////////////////////////////
void Scene3D::OnHovered(int _mouseX, int _mouseY)
{
  auto renderWindow = this->PluginItem()->findChild<RenderWindowItem *>();
  renderWindow->OnHovered({_mouseX, _mouseY});
}

/////////////////////////////////////////////////
void Scene3D::OnDropped(const QString &_drop, int _mouseX, int _mouseY)
{
  if (_drop.toStdString().empty())
  {
    this->SetErrorPopupText("Dropped empty entity URI.");
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
  std::string dropStr = _drop.toStdString();
  if (QUrl(_drop).isLocalFile())
  {
    // mesh to sdf model
    common::rtrim(dropStr);

    if (!common::MeshManager::Instance()->IsValidFilename(dropStr))
    {
      QString errTxt = QString::fromStdString("Invalid URI: " + dropStr +
        "\nOnly Fuel URLs or mesh file types DAE, OBJ, and STL are supported.");
      this->SetErrorPopupText(errTxt);
      return;
    }

    // Fixes whitespace
    dropStr = common::replaceAll(dropStr, "%20", " ");

    std::string filename = common::basename(dropStr);
    std::vector<std::string> splitName = common::split(filename, ".");

    std::string sdf = "<?xml version='1.0'?>"
      "<sdf version='" + std::string(SDF_PROTOCOL_VERSION) + "'>"
        "<model name='" + splitName[0] + "'>"
          "<link name='link'>"
            "<visual name='visual'>"
              "<geometry>"
                "<mesh>"
                  "<uri>" + dropStr + "</uri>"
                "</mesh>"
              "</geometry>"
            "</visual>"
            "<collision name='collision'>"
              "<geometry>"
                "<mesh>"
                  "<uri>" + dropStr + "</uri>"
                "</mesh>"
              "</geometry>"
            "</collision>"
          "</link>"
        "</model>"
      "</sdf>";

    req.set_sdf(sdf);
  }
  else
  {
    // model from fuel
    req.set_sdf_filename(dropStr);
  }

  req.set_allow_renaming(true);
  msgs::Set(req.mutable_pose(),
      math::Pose3d(pos.X(), pos.Y(), pos.Z(), 1, 0, 0, 0));

  this->dataPtr->node.Request("/world/" + this->dataPtr->worldName + "/create",
      req, cb);
}

/////////////////////////////////////////////////
QString Scene3D::ErrorPopupText() const
{
  return this->dataPtr->errorPopupText;
}

/////////////////////////////////////////////////
void Scene3D::SetErrorPopupText(const QString &_errorTxt)
{
  this->dataPtr->errorPopupText = _errorTxt;
  this->ErrorPopupTextChanged();
  this->popupError();
}

/////////////////////////////////////////////////
void Scene3D::OnFocusWindow()
{
  auto renderWindow = this->PluginItem()->findChild<RenderWindowItem *>();
  renderWindow->forceActiveFocus();
}

/////////////////////////////////////////////////
void RenderWindowItem::SetXYZSnap(const math::Vector3d &_xyz)
{
  this->dataPtr->renderThread->ignRenderer.SetXYZSnap(_xyz);
}

/////////////////////////////////////////////////
void RenderWindowItem::SetRPYSnap(const math::Vector3d &_rpy)
{
  this->dataPtr->renderThread->ignRenderer.SetRPYSnap(_rpy);
}

/////////////////////////////////////////////////
void RenderWindowItem::SetScaleSnap(const math::Vector3d &_scale)
{
  this->dataPtr->renderThread->ignRenderer.SetScaleSnap(_scale);
}

/////////////////////////////////////////////////
bool Scene3D::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == QEvent::KeyPress)
  {
    QKeyEvent *keyEvent = static_cast<QKeyEvent*>(_event);
    if (keyEvent)
    {
      auto renderWindow = this->PluginItem()->findChild<RenderWindowItem *>();
      renderWindow->HandleKeyPress(keyEvent);
    }
  }
  else if (_event->type() == QEvent::KeyRelease)
  {
    QKeyEvent *keyEvent = static_cast<QKeyEvent*>(_event);
    if (keyEvent)
    {
      auto renderWindow = this->PluginItem()->findChild<RenderWindowItem *>();
      renderWindow->HandleKeyRelease(keyEvent);
    }
  }
  else if (_event->type() ==
      ignition::gazebo::gui::events::EntitiesSelected::kType)
  {
    auto selectedEvent =
        reinterpret_cast<ignition::gazebo::gui::events::EntitiesSelected *>(
        _event);
    if (selectedEvent)
    {
      for (const auto &entity : selectedEvent->Data())
      {
        // If the event is from the user, update render util state
        if (!selectedEvent->FromUser())
          continue;

        auto node = this->dataPtr->renderUtil->SceneManager().NodeById(entity);

        if (nullptr == node)
        {
          // If an unknown entity has been selected, and control is not pressed,
          // deselect all known selected entities
          if (!(QGuiApplication::keyboardModifiers() & Qt::ControlModifier))
          {
            auto renderWindow =
                this->PluginItem()->findChild<RenderWindowItem *>();
            renderWindow->DeselectAllEntities(false);
          }
          continue;
        }

        auto renderWindow = this->PluginItem()->findChild<RenderWindowItem *>();
        renderWindow->UpdateSelectedEntity(entity, false);
      }
    }
  }
  else if (_event->type() ==
           ignition::gazebo::gui::events::DeselectAllEntities::kType)
  {
    auto deselectEvent =
        reinterpret_cast<ignition::gazebo::gui::events::DeselectAllEntities *>(
        _event);

    // If the event is from the user, update render util state
    if (deselectEvent && deselectEvent->FromUser())
    {
      auto renderWindow = this->PluginItem()->findChild<RenderWindowItem *>();
      renderWindow->DeselectAllEntities(false);
    }
  }
  else if (_event->type() ==
      ignition::gui::events::SnapIntervals::kType)
  {
    auto snapEvent = reinterpret_cast<ignition::gui::events::SnapIntervals *>(
        _event);
    if (snapEvent)
    {
      auto renderWindow = this->PluginItem()->findChild<RenderWindowItem *>();
      renderWindow->SetXYZSnap(snapEvent->Position());
      renderWindow->SetRPYSnap(snapEvent->Rotation());
      renderWindow->SetScaleSnap(snapEvent->Scale());
    }
  }
  else if (_event->type() ==
      ignition::gui::events::SpawnFromDescription::kType)
  {
    auto spawnPreviewEvent = reinterpret_cast<
        ignition::gui::events::SpawnFromDescription *>(_event);
    if (spawnPreviewEvent)
    {
      auto renderWindow = this->PluginItem()->findChild<RenderWindowItem *>();
      renderWindow->SetModel(spawnPreviewEvent->Description());
    }
  }
  else if (_event->type() == ignition::gui::events::SpawnFromPath::kType)
  {
    auto spawnPreviewPathEvent =
      reinterpret_cast<ignition::gui::events::SpawnFromPath *>(_event);
    if (spawnPreviewPathEvent)
    {
      auto renderWindow = this->PluginItem()->findChild<RenderWindowItem *>();
      renderWindow->SetModelPath(spawnPreviewPathEvent->FilePath());
    }
  }
  else if (_event->type() ==
      ignition::gui::events::DropdownMenuEnabled::kType)
  {
    auto dropdownMenuEnabledEvent =
      reinterpret_cast<ignition::gui::events::DropdownMenuEnabled *>(_event);
    if (dropdownMenuEnabledEvent)
    {
      auto renderWindow = this->PluginItem()->findChild<RenderWindowItem *>();
      renderWindow->SetDropdownMenuEnabled(
          dropdownMenuEnabledEvent->MenuEnabled());
    }
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

/////////////////////////////////////////////////
void RenderWindowItem::UpdateSelectedEntity(Entity _entity,
    bool _sendEvent)
{
  this->dataPtr->renderThread->ignRenderer.RequestSelectionChange(
      _entity, false, _sendEvent);
}

/////////////////////////////////////////////////
void RenderWindowItem::SetTransformMode(const std::string &_mode)
{
  this->dataPtr->renderThread->ignRenderer.SetTransformMode(_mode);
}

/////////////////////////////////////////////////
void RenderWindowItem::SetModel(const std::string &_model)
{
  this->dataPtr->renderThread->ignRenderer.SetModel(_model);
}

/////////////////////////////////////////////////
void RenderWindowItem::SetModelPath(const std::string &_filePath)
{
  this->dataPtr->renderThread->ignRenderer.SetModelPath(_filePath);
}

/////////////////////////////////////////////////
void RenderWindowItem::SetDropdownMenuEnabled(bool _enableDropdownMenu)
{
  this->dataPtr->renderThread->ignRenderer.SetDropdownMenuEnabled(
      _enableDropdownMenu);
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
void RenderWindowItem::DeselectAllEntities(bool _sendEvent)
{
  this->dataPtr->renderThread->ignRenderer.RequestSelectionChange(
      kNullEntity, true, _sendEvent);
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
void RenderWindowItem::SetViewAngle(const math::Vector3d &_direction)
{
  this->dataPtr->renderThread->ignRenderer.SetViewAngle(_direction);
}

/////////////////////////////////////////////////
void RenderWindowItem::SetMoveToPose(const math::Pose3d &_pose)
{
  this->dataPtr->renderThread->ignRenderer.SetMoveToPose(_pose);
}

/////////////////////////////////////////////////
void RenderWindowItem::SetViewCollisionsTarget(const std::string &_target)
{
  this->dataPtr->renderThread->ignRenderer.SetViewCollisionsTarget(_target);
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
math::Pose3d RenderWindowItem::CameraPose() const
{
  if (this->dataPtr->renderThread)
    return this->dataPtr->renderThread->ignRenderer.CameraPose();
  return math::Pose3d::Zero;
}

/////////////////////////////////////////////////
void RenderWindowItem::SetInitCameraPose(const math::Pose3d &_pose)
{
  this->dataPtr->renderThread->ignRenderer.SetInitCameraPose(_pose);
}

/////////////////////////////////////////////////
void RenderWindowItem::SetWorldName(const std::string &_name)
{
  this->dataPtr->renderThread->ignRenderer.worldName = _name;
}

/////////////////////////////////////////////////
void RenderWindowItem::SetRecordVideoUseSimTime(bool _useSimTime)
{
  this->dataPtr->renderThread->ignRenderer.SetRecordVideoUseSimTime(
      _useSimTime);
}

/////////////////////////////////////////////////
void RenderWindowItem::SetRecordVideoLockstep(bool _lockstep)
{
  this->dataPtr->renderThread->ignRenderer.SetRecordVideoLockstep(
      _lockstep);
}

/////////////////////////////////////////////////
void RenderWindowItem::SetRecordVideoBitrate(unsigned int _bitrate)
{
  this->dataPtr->renderThread->ignRenderer.SetRecordVideoBitrate(
      _bitrate);
}

/////////////////////////////////////////////////
void RenderWindowItem::SetVisibilityMask(uint32_t _mask)
{
  this->dataPtr->renderThread->ignRenderer.visibilityMask = _mask;
}

/////////////////////////////////////////////////
void RenderWindowItem::OnHovered(const ignition::math::Vector2i &_hoverPos)
{
  this->dataPtr->renderThread->ignRenderer.NewHoverEvent(_hoverPos);
}

/////////////////////////////////////////////////
void RenderWindowItem::mousePressEvent(QMouseEvent *_e)
{
  this->forceActiveFocus();

  auto event = ignition::gui::convert(*_e);
  event.SetPressPos(event.Pos());
  this->dataPtr->mouseEvent = event;
  this->dataPtr->mouseEvent.SetType(common::MouseEvent::PRESS);

  this->dataPtr->renderThread->ignRenderer.NewMouseEvent(
      this->dataPtr->mouseEvent);
}

////////////////////////////////////////////////
void RenderWindowItem::mouseReleaseEvent(QMouseEvent *_e)
{
  auto event = ignition::gui::convert(*_e);
  event.SetPressPos(this->dataPtr->mouseEvent.PressPos());

  // A release at the end of a drag
  if (this->dataPtr->mouseEvent.Type() == common::MouseEvent::MOVE)
    event.SetDragging(this->dataPtr->mouseEvent.Dragging());

  this->dataPtr->mouseEvent = event;
  this->dataPtr->mouseEvent.SetType(common::MouseEvent::RELEASE);

  this->dataPtr->renderThread->ignRenderer.NewMouseEvent(
      this->dataPtr->mouseEvent);
}

////////////////////////////////////////////////
void RenderWindowItem::mouseMoveEvent(QMouseEvent *_e)
{
  auto event = ignition::gui::convert(*_e);

  if (!event.Dragging())
    return;

  event.SetPressPos(this->dataPtr->mouseEvent.PressPos());

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
#if QT_VERSION < QT_VERSION_CHECK(5, 14, 0)
  this->dataPtr->mouseEvent.SetPos(_e->x(), _e->y());
#else
  this->dataPtr->mouseEvent.SetPos(_e->position().x(), _e->position().y());
#endif
  double scroll = (_e->angleDelta().y() > 0) ? -1.0 : 1.0;
  this->dataPtr->renderThread->ignRenderer.NewMouseEvent(
      this->dataPtr->mouseEvent, math::Vector2d(scroll, scroll));
}

////////////////////////////////////////////////
void RenderWindowItem::HandleKeyPress(QKeyEvent *_e)
{
  this->dataPtr->renderThread->ignRenderer.HandleKeyPress(_e);
}

////////////////////////////////////////////////
void RenderWindowItem::HandleKeyRelease(QKeyEvent *_e)
{
  this->dataPtr->renderThread->ignRenderer.HandleKeyRelease(_e);

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

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::Scene3D,
                    ignition::gui::Plugin)
