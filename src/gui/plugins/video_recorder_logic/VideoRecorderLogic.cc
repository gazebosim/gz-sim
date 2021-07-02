/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <string>

#include <ignition/common/Console.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/common/VideoEncoder.hh>

#include <ignition/gui/Application.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/gui/MainWindow.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/rendering/Camera.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>

#include <ignition/transport/Node.hh>

#include "VideoRecorderLogic.hh"

/// \brief condition variable for lockstepping video recording
/// todo(anyone) avoid using a global condition variable when we support
/// multiple viewports in the future.
std::condition_variable g_renderCv;

namespace ignition::gazebo::plugins
{
  class VideoRecorderLogicPrivate
  {
    /// \brief Update the Video Recorder Plugin on the latest state of the ECM.
    public: void OnRender();

    // Initialize Video Recorder Logic plugin
    public: void Initialize();

    /// \brief Callback for a record video request
    /// \param[in] _msg Request message to enable/disable video recording.
    /// \param[in] _res Response data
    /// \return True if the request is received
    public: bool OnRecordVideo(
      const msgs::VideoRecord &_msg, msgs::Boolean &_res);

    /// \brief Ignition communication node.
    public: transport::Node node;

    /// \brief Pointer to the camera being moved
    public: rendering::CameraPtr camera{nullptr};

    /// \brief Pointer to the camera being moved
    public: rendering::ScenePtr scene{nullptr};

    /// \brief Video encoder
    public: common::VideoEncoder videoEncoder;

    /// \brief Image from user camera
    public: rendering::Image cameraImage;

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

    /// \brief Record stats topic name
    public: std::string recorderStatsTopic = "/gui/record_video/stats";

    /// \brief Record video service
    public: std::string recordVideoService = "/gui/record_video";

    /// \brief True to indicate video recording in progress
    public: bool recording = false;

    /// \brief mutex to protect the recording variable
    public: std::mutex recordMutex;

    /// \brief mutex to protect the render condition variable
    /// Used when recording in lockstep mode.
    public: std::mutex renderMutex;

    /// \brief Total time elapsed in simulation. This will not increase while
    /// paused.
    public: std::chrono::steady_clock::duration simTime{0};
  };
}

using namespace ignition;
using namespace gazebo;
using namespace plugins;

void VideoRecorderLogicPrivate::Initialize()
{
  if (!this->scene)
  {
    this->scene = rendering::sceneFromFirstRenderEngine();
    if (!this->scene)
      return;

    for (unsigned int i = 0; i < scene->NodeCount(); ++i)
    {
      auto cam = std::dynamic_pointer_cast<rendering::Camera>(
        scene->NodeByIndex(i));
      if (cam)
      {
        if (std::get<bool>(cam->UserData("user-camera")))
        {
          this->camera = cam;
          igndbg << "Video Recorder plugin is recoding camera ["
                 << this->camera->Name() << "]" << std::endl;
          break;
        }
      }
    }

    if (!this->camera)
    {
      ignerr << "InteractiveViewControl camera is not available" << std::endl;
      return;
    }

    // video recorder
    this->node.Advertise(this->recordVideoService,
        &VideoRecorderLogicPrivate::OnRecordVideo, this);
    ignmsg << "Record video service on ["
           << this->recordVideoService << "]" << std::endl;

    // recorder stats topic
    this->recorderStatsPub =
      this->node.Advertise<msgs::Time>(this->recorderStatsTopic);
    ignmsg << "Video recorder stats topic advertised on ["
           << this->recorderStatsTopic << "]" << std::endl;
  }
}

/////////////////////////////////////////////////
bool VideoRecorderLogicPrivate::OnRecordVideo(const msgs::VideoRecord &_msg,
  msgs::Boolean &_res)
{
  bool record = _msg.start() && !_msg.stop();

  this->recordVideo = record;
  this->recordVideoFormat = _msg.format();
  this->recordVideoSavePath = _msg.save_filename();

  _res.set_data(true);

  std::unique_lock<std::mutex> lock(this->recordMutex);
  this->recording = record;
  return true;
}

/////////////////////////////////////////////////
void VideoRecorderLogicPrivate::OnRender()
{
  this->Initialize();

  // check if recording is in lockstep mode and if it is using sim time
  // if so, there is no need to update camera if sim time has not advanced
  bool update = true;
  if (this->recordVideoLockstep &&
      this->recordVideoUseSimTime &&
      this->videoEncoder.IsEncoding())
  {
    std::chrono::steady_clock::time_point t =
        std::chrono::steady_clock::time_point(
        this->simTime);
    if (t - this->recordVideoUpdateTime == std::chrono::seconds(0))
      update = false;
    else
      this->recordVideoUpdateTime = t;
  }

  // update and render to texture
  if (update)
  {
    IGN_PROFILE("IgnRenderer::Render Update camera");
    this->camera->Update();
  }

  // record video is requested
  {
    IGN_PROFILE("IgnRenderer::Render Record Video");
    if (this->recordVideo)
    {
      unsigned int width = this->camera->ImageWidth();
      unsigned int height = this->camera->ImageHeight();

      if (this->cameraImage.Width() != width ||
          this->cameraImage.Height() != height)
      {
        this->cameraImage = this->camera->CreateImage();
      }

      // Video recorder is on. Add more frames to it
      if (this->videoEncoder.IsEncoding())
      {
        this->camera->Copy(this->cameraImage);

        std::chrono::steady_clock::time_point t =
            std::chrono::steady_clock::now();
        if (this->recordVideoUseSimTime)
        {
          t = std::chrono::steady_clock::time_point(
              this->simTime);
        }
        bool frameAdded = this->videoEncoder.AddFrame(
            this->cameraImage.Data<unsigned char>(), width, height, t);

        if (frameAdded)
        {
          // publish recorder stats
          if (this->recordStartTime ==
              std::chrono::steady_clock::time_point(
              std::chrono::duration(std::chrono::seconds(0))))
          {
            // start time, i.e. time when first frame is added
            this->recordStartTime = t;
          }

          std::chrono::steady_clock::duration dt;
          dt = t - this->recordStartTime;
          int64_t sec, nsec;
          std::tie(sec, nsec) = ignition::math::durationToSecNsec(dt);
          msgs::Time msg;
          msg.set_sec(sec);
          msg.set_nsec(nsec);
          this->recorderStatsPub.Publish(msg);
        }
      }
      // Video recorder is idle. Start recording.
      else
      {
        if (this->recordVideoUseSimTime)
          ignmsg << "Recording video using sim time." << std::endl;
        if (this->recordVideoLockstep)
        {
          ignmsg << "Recording video in lockstep mode" << std::endl;
          if (!this->recordVideoUseSimTime)
          {
            ignwarn << "It is recommended to set <use_sim_time> to true "
                    << "when recording video in lockstep mode." << std::endl;
          }
        }
        ignmsg << "Recording video using bitrate: "
               << this->recordVideoBitrate <<  std::endl;
        this->videoEncoder.Start(this->recordVideoFormat,
            this->recordVideoSavePath, width, height, 25,
            this->recordVideoBitrate);
        this->recordStartTime = std::chrono::steady_clock::time_point(
            std::chrono::duration(std::chrono::seconds(0)));
      }
    }
    else if (this->videoEncoder.IsEncoding())
    {
      this->videoEncoder.Stop();
    }
  }
  // only has an effect in video recording lockstep mode
  // this notifes ECM to continue updating the scene
  g_renderCv.notify_one();
}

/////////////////////////////////////////////////
VideoRecorderLogic::VideoRecorderLogic()
  : GuiSystem(), dataPtr(std::make_unique<VideoRecorderLogicPrivate>())
{
}

/////////////////////////////////////////////////
VideoRecorderLogic::~VideoRecorderLogic() = default;

//////////////////////////////////////////////////
void VideoRecorderLogic::Update(const UpdateInfo &_info,
    EntityComponentManager & /*_ecm*/)
{
  this->dataPtr->simTime = _info.simTime;

  // check if video recording is enabled and if we need to lock step
  // ECM updates with GUI rendering during video recording
  std::unique_lock<std::mutex> lock(this->dataPtr->recordMutex);
  if (this->dataPtr->recording && this->dataPtr->recordVideoLockstep)
  {
    std::unique_lock<std::mutex> lock2(this->dataPtr->renderMutex);
    g_renderCv.wait(lock2);
  }
}

/////////////////////////////////////////////////
void VideoRecorderLogic::LoadConfig(const tinyxml2::XMLElement * _pluginElem)
{
  if (this->title.empty())
    this->title = "Video recorder";

  // Custom parameters
  if (_pluginElem)
  {
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
          this->dataPtr->recordVideoUseSimTime = useSimTime;
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
          this->dataPtr->recordVideoLockstep = lockstep;
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
          this->dataPtr->recordVideoBitrate = bitrate;
        }
        else
        {
          ignerr << "Video recorder bitrate must be larger than 0"
                 << std::endl;
        }
      }
    }
  }

  ignition::gui::App()->findChild<
      ignition::gui::MainWindow *>()->installEventFilter(this);
}

/////////////////////////////////////////////////
bool VideoRecorderLogic::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == ignition::gui::events::Render::kType)
  {
    this->dataPtr->OnRender();
  }
  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::plugins::VideoRecorderLogic,
                    ignition::gui::Plugin)
