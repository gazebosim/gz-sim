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

#include <regex>
#include <set>
#include <unordered_map>

#include <ignition/common/Profiler.hh>
#include <ignition/common/VideoEncoder.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <ignition/rendering/Camera.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>

#include "ignition/gazebo/rendering/Events.hh"

#include "ignition/gazebo/components/Camera.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/Conversions.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Events.hh"
#include "ignition/gazebo/Util.hh"

#include "CameraVideoRecorder.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

// Private data class.
class ignition::gazebo::systems::CameraVideoRecorderPrivate
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
  public: ignition::common::ConnectionPtr postRenderConn;

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
        ignerr << "Video encoding format: '" << this->recordVideoFormat
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
    ignerr << "The camera video recorder system can only be attached to a "
           << "camera sensor." << std::endl;
  }

  this->dataPtr->entity = _entity;

  // video recorder service topic name
  if (_sdf->HasElement("service"))
  {
    this->dataPtr->service = _sdf->Get<std::string>("service");
  }
  this->dataPtr->eventMgr = &_eventMgr;

  // get sensor topic
  sdf::Sensor sensorSdf = cameraEntComp->Data();
  std::string topic  = sensorSdf.Topic();
  if (topic.empty())
    topic = scopedName(_entity, _ecm) + "/image";
  this->dataPtr->sensorTopic = topic;
}

//////////////////////////////////////////////////
void CameraVideoRecorderPrivate::OnPostRender()
{
  // get scene
  if (!this->scene)
  {
    auto loadedEngNames = rendering::loadedEngines();
    if (loadedEngNames.empty())
      return;

    // assume there is only one engine loaded
    auto engineName = loadedEngNames[0];
    if (loadedEngNames.size() > 1)
    {
      igndbg << "More than one engine is available. "
        << "Camera video recorder system will use engine ["
          << engineName << "]" << std::endl;
    }
    auto engine = rendering::engine(engineName);
    if (!engine)
    {
      ignerr << "Internal error: failed to load engine [" << engineName
        << "]. Camera video recorder system won't work." << std::endl;
      return;
    }

    if (engine->SceneCount() == 0)
      return;

    // assume there is only one scene
    // load scene
    auto s = engine->SceneByIndex(0);
    if (!s)
    {
      ignerr << "Internal error: scene is null." << std::endl;
      return;
    }
    this->scene = s;
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
      ignerr << "Unable to find sensor: " << this->cameraName
             << std::endl;
      return;
    }
    this->camera = std::dynamic_pointer_cast<rendering::Camera>(sensor);
    if (!this->camera)
    {
      ignerr << "Sensor: " << this->cameraName << " is not a caemra"
             << std::endl;
      return;
    }

    return;
  }

  std::lock_guard<std::mutex> lock(this->updateMutex);

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
      this->videoEncoder.AddFrame(
          this->cameraImage.Data<unsigned char>(), width, height);
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
          this->tmpVideoFilename, width, height);

      ignmsg << "Start video recording on [" << this->service << "]. "
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

    // move the tmp video file to user specified path
    if (common::exists(this->tmpVideoFilename))
    {
      common::moveFile(this->tmpVideoFilename,
          this->recordVideoSavePath);

      // Remove old temp file, if it exists.
      std::remove(this->tmpVideoFilename.c_str());
    }
    ignmsg << "Stop video recording on [" << this->service << "]. "
           << "Saving file to: [" << this->recordVideoSavePath << "]"
           << std::endl;

    // reset the event connection to prevent unnecessary render callbacks
    this->postRenderConn.reset();
  }
}

//////////////////////////////////////////////////
void CameraVideoRecorder::PostUpdate(const UpdateInfo &,
    const EntityComponentManager &_ecm)
{
  if (!this->dataPtr->cameraName.empty())
    return;

  this->dataPtr->cameraName =
      removeParentScope(scopedName(this->dataPtr->entity, _ecm, "::", false),
      "::");

  if (this->dataPtr->cameraName.empty())
    return;

  if (this->dataPtr->service.empty())
  {
    this->dataPtr->service = scopedName(this->dataPtr->entity, _ecm) +
      "/record_video";
  }

  this->dataPtr->node.Advertise(this->dataPtr->service,
       &CameraVideoRecorderPrivate::OnRecordVideo, this->dataPtr.get());
  ignmsg << "Record video service on ["
       << this->dataPtr->service << "]" << std::endl;
}

IGNITION_ADD_PLUGIN(CameraVideoRecorder,
                    ignition::gazebo::System,
                    CameraVideoRecorder::ISystemConfigure,
                    CameraVideoRecorder::ISystemPostUpdate)

// Add plugin alias so that we can refer to the plugin without the version
// namespace
IGNITION_ADD_PLUGIN_ALIAS(CameraVideoRecorder,
                          "ignition::gazebo::systems::CameraVideoRecorder")
