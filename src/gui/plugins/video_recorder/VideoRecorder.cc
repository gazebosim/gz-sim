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

#include "VideoRecorder.hh"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/video_record.pb.h>

#include <iostream>
#include <string>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/common/Profiler.hh>
#include <gz/common/VideoEncoder.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/Publisher.hh>

/// \brief condition variable for lockstepping video recording
/// todo(anyone) avoid using a global condition variable when we support
/// multiple viewports in the future.
std::condition_variable g_renderCv;

namespace gz::sim
{
  class VideoRecorderPrivate
  {
    /// \brief Capture a video frame in the render thread.
    public: void OnRender();

    /// \brief Initialize rendering and transport.
    public: void Initialize();

    /// \brief Gazebo communication node.
    public: transport::Node node;

    /// \brief Pointer to the camera being recorded
    public: rendering::CameraPtr camera{nullptr};

    /// \brief Pointer to the 3D scene
    public: rendering::ScenePtr scene{nullptr};

    /// \brief Video encoder
    public: common::VideoEncoder videoEncoder;

    /// \brief Image from user camera
    public: rendering::Image cameraImage;

    /// \brief True to record a video from the user camera
    public: bool recordVideo = false;

    /// \brief Video encoding format
    public: std::string format;

    /// \brief Use sim time as timestamp during video recording
    /// By default (false), video encoding is done using real time.
    public: bool useSimTime = false;

    /// \brief Lockstep gui with ECM when recording
    public: bool lockstep = false;

    /// \brief Video recorder bitrate (bps)
    public: unsigned int bitrate = 2070000;

    /// \brief Start tiem of video recording
    public: std::chrono::steady_clock::time_point startTime;

    /// \brief Camera pose publisher
    public: transport::Node::Publisher recorderStatsPub;

    /// \brief Record stats topic name
    public: std::string recorderStatsTopic = "/gui/record_video/stats";

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

    /// \brief Filename of the recorded video
    public: std::string filename;
  };
}

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
void VideoRecorderPrivate::Initialize()
{
  // Already initialized
  if (this->scene)
    return;

  this->scene = rendering::sceneFromFirstRenderEngine();
  if (!this->scene)
    return;

  for (unsigned int i = 0; i < this->scene->NodeCount(); ++i)
  {
    auto cam = std::dynamic_pointer_cast<rendering::Camera>(
      this->scene->NodeByIndex(i));
    if (cam && cam->HasUserData("user-camera") &&
        std::get<bool>(cam->UserData("user-camera")))
    {
      this->camera = cam;
      gzdbg << "Video Recorder plugin is recoding camera ["
             << this->camera->Name() << "]" << std::endl;
      break;
    }
  }

  if (!this->camera)
  {
    gzerr << "Camera is not available" << std::endl;
    return;
  }

  // recorder stats topic
  this->recorderStatsPub =
    this->node.Advertise<msgs::Time>(this->recorderStatsTopic);
  gzmsg << "Video recorder stats topic advertised on ["
         << this->recorderStatsTopic << "]" << std::endl;
}

/////////////////////////////////////////////////
void VideoRecorderPrivate::OnRender()
{
  this->Initialize();

  // record video is requested
  {
    GZ_PROFILE("VideoRecorder Record Video");
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
        if (this->useSimTime)
        {
          t = std::chrono::steady_clock::time_point(
              this->simTime);
        }
        bool frameAdded = this->videoEncoder.AddFrame(
            this->cameraImage.Data<unsigned char>(), width, height, t);

        if (frameAdded)
        {
          // publish recorder stats
          if (this->startTime ==
              std::chrono::steady_clock::time_point(
              std::chrono::duration(std::chrono::seconds(0))))
          {
            // start time, i.e. time when first frame is added
            this->startTime = t;
          }

          std::chrono::steady_clock::duration dt;
          dt = t - this->startTime;
          int64_t sec, nsec;
          std::tie(sec, nsec) = gz::math::durationToSecNsec(dt);
          msgs::Time msg;
          msg.set_sec(sec);
          msg.set_nsec(nsec);
          this->recorderStatsPub.Publish(msg);
        }
      }
      // Video recorder is idle. Start recording.
      else
      {
        if (this->useSimTime)
          gzmsg << "Recording video using sim time." << std::endl;
        if (this->lockstep)
        {
          gzmsg << "Recording video in lockstep mode" << std::endl;
          if (!this->useSimTime)
          {
            gzwarn << "It is recommended to set <use_sim_time> to true "
                    << "when recording video in lockstep mode." << std::endl;
          }
        }
        gzmsg << "Recording video using bitrate: "
               << this->bitrate <<  std::endl;
        this->videoEncoder.Start(this->format,
            this->filename, width, height, 25,
            this->bitrate);
        this->startTime = std::chrono::steady_clock::time_point(
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
VideoRecorder::VideoRecorder()
  : GuiSystem(), dataPtr(std::make_unique<VideoRecorderPrivate>())
{
}

/////////////////////////////////////////////////
VideoRecorder::~VideoRecorder() = default;

//////////////////////////////////////////////////
void VideoRecorder::Update(const UpdateInfo &_info,
    EntityComponentManager & /*_ecm*/)
{
  this->dataPtr->simTime = _info.simTime;

  // check if video recording is enabled and if we need to lock step
  // ECM updates with GUI rendering during video recording
  std::unique_lock<std::mutex> lock(this->dataPtr->recordMutex);
  if (this->dataPtr->recording && this->dataPtr->lockstep)
  {
    std::unique_lock<std::mutex> lock2(this->dataPtr->renderMutex);
    g_renderCv.wait(lock2);
  }
}

/////////////////////////////////////////////////
void VideoRecorder::LoadConfig(const tinyxml2::XMLElement * _pluginElem)
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
          gzerr << "Faild to parse <use_sim_time> value: "
                 << useSimTimeElem->GetText() << std::endl;
        }
        else
        {
          this->dataPtr->useSimTime = useSimTime;
        }
      }
      if (auto lockstepElem = elem->FirstChildElement("lockstep"))
      {
        bool lockstep = false;
        if (lockstepElem->QueryBoolText(&lockstep) != tinyxml2::XML_SUCCESS)
        {
          gzerr << "Failed to parse <lockstep> value: "
                 << lockstepElem->GetText() << std::endl;
        }
        else
        {
          this->dataPtr->lockstep = lockstep;
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
          this->dataPtr->bitrate = bitrate;
        }
        else
        {
          gzerr << "Video recorder bitrate must be larger than 0"
                 << std::endl;
        }
      }
    }
  }

  gz::gui::App()->findChild<
      gz::gui::MainWindow *>()->installEventFilter(this);
}

/////////////////////////////////////////////////
bool VideoRecorder::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gui::events::Render::kType)
  {
    this->dataPtr->OnRender();
  }
  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

/////////////////////////////////////////////////
void VideoRecorder::OnStart(const QString &_format)
{
  std::unique_lock<std::mutex> lock(this->dataPtr->recordMutex);
  this->dataPtr->format = _format.toStdString();
  this->dataPtr->filename = "ign_recording." + this->dataPtr->format;
  this->dataPtr->recordVideo = true;
  this->dataPtr->recording = true;
}

/////////////////////////////////////////////////
void VideoRecorder::OnStop()
{
  this->dataPtr->recordVideo = false;
  this->dataPtr->recording = false;
}

/////////////////////////////////////////////////
void VideoRecorder::OnSave(const QString &_url)
{
  std::string path = QUrl(_url).toLocalFile().toStdString();

  // If we cannot find an extension in the user entered file name,
  // append the format of the selected codec
  if (common::basename(path).find(".") == std::string::npos)
  {
    // Get the user selected file extension
    std::string filenameBaseName = common::basename(this->dataPtr->filename);
    std::string::size_type filenameExtensionIndex =
      filenameBaseName.rfind(".");
    std::string fileExtension =
      filenameBaseName.substr(filenameExtensionIndex + 1);

    // Append file extension to the user entered path
    path += "." + fileExtension;
  }

  bool result = common::moveFile(this->dataPtr->filename, path);

  if (!result)
  {
    gzerr  << "Unable to rename file from[" << this->dataPtr->filename
      << "] to [" << path << "]" << std::endl;
  }
  else
  {
    gzmsg << "Video saved to: " << path << std::endl;
  }
}

/////////////////////////////////////////////////
void VideoRecorder::OnCancel()
{
  if (common::exists(this->dataPtr->filename))
    std::remove(this->dataPtr->filename.c_str());
}

// Register this plugin
GZ_ADD_PLUGIN(VideoRecorder,
              gz::gui::Plugin)
