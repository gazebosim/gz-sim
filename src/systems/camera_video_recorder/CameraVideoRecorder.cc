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

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/time.pb.h>
#include <gz/msgs/video_record.pb.h>

#include <regex>
#include <set>
#include <string>
#include <unordered_map>

#include <gz/common/Profiler.hh>
#include <gz/common/VideoEncoder.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/rendering/Camera.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>

#include "gz/sim/rendering/RenderUtil.hh"
#include "gz/sim/rendering/Events.hh"
#include "gz/sim/rendering/MarkerManager.hh"

#include "gz/sim/components/Camera.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/Conversions.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Events.hh"
#include "gz/sim/Util.hh"

#include "CameraVideoRecorder.hh"

using namespace gz;
using namespace sim;
using namespace systems;

// Private data class.
class gz::sim::systems::CameraVideoRecorderPrivate
{
  /// \brief Callback for the video recorder service
  public: bool OnRecordVideo(const msgs::VideoRecord &_msg,
      msgs::Boolean &_res);

  /// \brief Callback invoked in the rendering thread after a render update
  public: void OnPostRender();

  /// \brief Callback for new images
  public: void OnImage(const msgs::Image &_msg);

  /// \brief Transport node
  public: transport::Node node;

  /// \brief Mutex to protect updates
  public: std::mutex updateMutex;

  /// \brief Connection to the post-render event.
  public: common::ConnectionPtr postRenderConn;

  /// \brief Pointer to the event manager
  public: EventManager *eventMgr = nullptr;

  //// \brief Pointer to the rendering scene
  public: rendering::ScenePtr scene;

  /// \brief Pointer to the rendering camera
  public: rendering::CameraPtr camera;

  /// \brief Name of service for recording video
  public: std::string service;

  /// \brief Camera entity.
  public: Entity entity;

  /// \brief Name of the camera
  public: std::string cameraName;

  /// \brief Image from user camera
  public: rendering::Image cameraImage;

  /// \brief Video encoder
  public: common::VideoEncoder videoEncoder;

  /// \brief Video encoding format
  public: std::string recordVideoFormat;

  /// \brief Path to save the recorded video
  public: std::string recordVideoSavePath;

  /// \brief Temporary filename of the video being encoded
  public: std::string tmpVideoFilename;

  /// \brief True to record a video from the rendering camera
  public: bool recordVideo = false;

  /// \brief Topic that the sensor publishes to
  public: std::string sensorTopic;

  /// \brief Video recording statistics publisher
  public: transport::Node::Publisher recorderStatsPub;

  /// \brief Start time of video recording.
  public: std::chrono::steady_clock::time_point recordStartTime;

  /// \brief Current simulation time.
  public: std::chrono::steady_clock::duration simTime{0};

  /// \brief Use sim time as timestamp during video recording
  /// By default (false), video encoding is done using real time.
  public: bool recordVideoUseSimTime = false;

  /// \brief Video recorder bitrate (bps). This is rougly 2Mbps which
  /// produces decent video quality while not generating overly large
  /// video files.
  ///
  /// Another point of reference is at:
  /// https://support.google.com/youtube/answer/1722171?hl=en#zippy=%2Cbitrate
  public: unsigned int recordVideoBitrate = 2070000;

  /// \brief Recording frames per second.
  public: unsigned int fps = 25;

  /// \brief Marker manager
  public: MarkerManager markerManager;
};

//////////////////////////////////////////////////
void CameraVideoRecorderPrivate::OnImage(const msgs::Image &)
{
  // No work is done here. We need to subscribe to the sensor to make it active.
}

//////////////////////////////////////////////////
bool CameraVideoRecorderPrivate::OnRecordVideo(const msgs::VideoRecord &_msg,
    msgs::Boolean &_res)
{
  std::lock_guard<std::mutex> lock(this->updateMutex);
  bool record = _msg.start() && !_msg.stop();
  this->recordVideo = record;

  if (this->recordVideo)
  {
    this->recordVideoFormat = _msg.format();
    this->recordVideoSavePath = _msg.save_filename();

    if (this->recordVideoFormat.empty())
    {
      this->recordVideoFormat = "mp4";
    }
    else
    {
      this->recordVideoFormat = common::lowercase(this->recordVideoFormat);
      if (this->recordVideoFormat != "mp4" && this->recordVideoFormat!= "ogv" &&
          this->recordVideoFormat != "avi")
      {
        gzerr << "Video encoding format: '" << this->recordVideoFormat
               << "' not supported. Available formats are: mp4, ogv, and avi."
               << std::endl;
        _res.set_data(false);
        return true;
      }
    }
    // create filename with timestamped suffix if path is not specified
    if (this->recordVideoSavePath.empty())
    {
      std::string str = common::systemTimeISO();
      std::string prefix = this->cameraName;
      prefix = std::regex_replace(prefix, std::regex("::"), "_");
      this->recordVideoSavePath = prefix + "_" + str + "." +
          this->recordVideoFormat;
    }

    // encode to tmp file and only move to the user specified path once
    // recording is done
    this->tmpVideoFilename = std::to_string(this->entity) + "."
         + this->recordVideoFormat;
  }

  // Set up the render connection so we can do the encoding in the rendering
  // thread
  if (!this->postRenderConn)
  {
    this->postRenderConn =
        this->eventMgr->Connect<events::PostRender>(
        std::bind(&CameraVideoRecorderPrivate::OnPostRender, this));
  }

  _res.set_data(true);
  return true;
}


//////////////////////////////////////////////////
CameraVideoRecorder::CameraVideoRecorder()
  : System(), dataPtr(std::make_unique<CameraVideoRecorderPrivate>())
{
}

//////////////////////////////////////////////////
void CameraVideoRecorder::Configure(
    const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm, EventManager &_eventMgr)
{
  auto cameraEntComp =
      _ecm.Component<components::Camera>(_entity);
  if (!cameraEntComp)
  {
    gzerr << "The camera video recorder system can only be attached to a "
           << "camera sensor." << std::endl;
  }

  this->dataPtr->entity = _entity;

  // video recorder service topic name
  if (_sdf->HasElement("service"))
  {
    this->dataPtr->service = transport::TopicUtils::AsValidTopic(
        _sdf->Get<std::string>("service"));
    if (this->dataPtr->service.empty())
    {
      gzerr << "Service [" << _sdf->Get<std::string>("service")
             << "] not valid. Ignoring." << std::endl;
    }
  }
  this->dataPtr->eventMgr = &_eventMgr;

  // get sensor topic
  sdf::Sensor sensorSdf = cameraEntComp->Data();
  std::string topic  = sensorSdf.Topic();
  if (topic.empty())
  {
    auto scoped = scopedName(_entity, _ecm);
    topic = transport::TopicUtils::AsValidTopic(scoped + "/image");
    if (topic.empty())
    {
      gzerr << "Failed to generate valid topic for entity [" << scoped
             << "]" << std::endl;
    }
  }
  this->dataPtr->sensorTopic = topic;

  // Get whether sim time should be used for recording.
  this->dataPtr->recordVideoUseSimTime = _sdf->Get<bool>("use_sim_time",
      this->dataPtr->recordVideoUseSimTime).first;

  // Get video recoder bitrate param
  this->dataPtr->recordVideoBitrate = _sdf->Get<unsigned int>("bitrate",
      this->dataPtr->recordVideoBitrate).first;

  this->dataPtr->fps = _sdf->Get<unsigned int>("fps", this->dataPtr->fps).first;

  // recorder stats topic
  std::string recorderStatsTopic = this->dataPtr->sensorTopic + "/stats";
  this->dataPtr->recorderStatsPub =
    this->dataPtr->node.Advertise<msgs::Time>(recorderStatsTopic);
  gzmsg << "Camera Video recorder stats topic advertised on ["
    << recorderStatsTopic << "]" << std::endl;
}

//////////////////////////////////////////////////
void CameraVideoRecorderPrivate::OnPostRender()
{
  // get scene
  if (!this->scene)
  {
    this->scene = rendering::sceneFromFirstRenderEngine();
    this->markerManager.SetTopic(this->sensorTopic + "/marker");
    this->markerManager.Init(this->scene);
  }

  // return if scene not ready or no sensors available.
  if (!this->scene->IsInitialized() ||
      this->scene->SensorCount() == 0)
  {
    return;
  }

  // get camera
  if (!this->camera)
  {
    auto sensor = this->scene->SensorByName(this->cameraName);
    if (!sensor)
    {
      gzerr << "Unable to find sensor: " << this->cameraName
             << std::endl;
      return;
    }
    this->camera = std::dynamic_pointer_cast<rendering::Camera>(sensor);
    if (!this->camera)
    {
      gzerr << "Sensor: " << this->cameraName << " is not a caemra"
             << std::endl;
      return;
    }

    return;
  }

  std::lock_guard<std::mutex> lock(this->updateMutex);

  this->markerManager.SetSimTime(this->simTime);
  this->markerManager.Update();

  // record video
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
      std::chrono::steady_clock::time_point t;
      if (this->recordVideoUseSimTime)
        t = std::chrono::steady_clock::time_point(this->simTime);
      else
        t = std::chrono::steady_clock::now();

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
        std::tie(sec, nsec) = math::durationToSecNsec(dt);
        msgs::Time msg;
        msg.set_sec(sec);
        msg.set_nsec(nsec);
        this->recorderStatsPub.Publish(msg);
      }
    }
    // Video recorder is idle. Start recording.
    else
    {
      // hack! subscribe to the camera sensor topic in order to make the sensor
      // active, otherwise the sensor thinks there are no subscribers and so
      // does not actually render.
      // todo(anyone) Make it possible to get a pointer to the sensor object so
      // we can connect to its new image event
      this->node.Subscribe(this->sensorTopic,
          &CameraVideoRecorderPrivate::OnImage, this);

      this->videoEncoder.Start(this->recordVideoFormat,
          this->tmpVideoFilename, width, height, this->fps,
          this->recordVideoBitrate);

      this->recordStartTime = std::chrono::steady_clock::time_point(
            std::chrono::duration(std::chrono::seconds(0)));

      gzmsg << "Start video recording on [" << this->service << "]. "
             << "Encoding to tmp file: ["
             << this->tmpVideoFilename << "]" << std::endl;
    }
  }
  else if (this->videoEncoder.IsEncoding())
  {
    // unsubscribe to let the sensor become inactive if there are no
    // other connections
    this->node.Unsubscribe(this->sensorTopic);

    // stop encoding
    this->videoEncoder.Stop();

    gzmsg << "Stop video recording on [" << this->service << "]." << std::endl;

    if (common::exists(this->tmpVideoFilename))
    {
      std::string parentPath = common::parentPath(this->recordVideoSavePath);

      // move the tmp video file to user specified path
      if (parentPath != this->recordVideoSavePath &&
          !common::exists(parentPath) && !common::createDirectory(parentPath))
      {
        gzerr << "Unable to create directory[" << parentPath
          << "]. Video file[" << this->tmpVideoFilename
          << "] will not be moved." << std::endl;
      }
      else
      {
        common::moveFile(this->tmpVideoFilename, this->recordVideoSavePath);

        // Remove old temp file, if it exists.
        std::remove(this->tmpVideoFilename.c_str());

        gzmsg << "Saving tmp video[" << this->tmpVideoFilename << "] file to ["
               << this->recordVideoSavePath << "]" << std::endl;
      }
    }

   this->scene.reset();
   this->camera.reset();
    // reset the event connection to prevent unnecessary render callbacks
    this->postRenderConn.reset();
  }
}

//////////////////////////////////////////////////
void CameraVideoRecorder::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  this->dataPtr->simTime = _info.simTime;
  if (!this->dataPtr->cameraName.empty())
    return;

  this->dataPtr->cameraName =
      removeParentScope(scopedName(this->dataPtr->entity, _ecm, "::", false),
      "::");

  if (this->dataPtr->cameraName.empty())
    return;

  if (this->dataPtr->service.empty())
  {
    auto scoped = scopedName(this->dataPtr->entity, _ecm);
    this->dataPtr->service = transport::TopicUtils::AsValidTopic(scoped +
        "/record_video");
    if (this->dataPtr->service.empty())
    {
      gzerr << "Failed to create valid service for [" << scoped << "]"
             << std::endl;
    }
    return;
  }

  this->dataPtr->node.Advertise(this->dataPtr->service,
       &CameraVideoRecorderPrivate::OnRecordVideo, this->dataPtr.get());
  gzmsg << "Record video service on ["
       << this->dataPtr->service << "]" << std::endl;
}

GZ_ADD_PLUGIN(CameraVideoRecorder,
                    System,
                    CameraVideoRecorder::ISystemConfigure,
                    CameraVideoRecorder::ISystemPostUpdate)

// Add plugin alias so that we can refer to the plugin without the version
// namespace
GZ_ADD_PLUGIN_ALIAS(CameraVideoRecorder,
                          "gz::sim::systems::CameraVideoRecorder")
