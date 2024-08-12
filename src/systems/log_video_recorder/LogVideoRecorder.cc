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

#include "LogVideoRecorder.hh"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/log_playback_control.pb.h>
#include <gz/msgs/scene.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/video_record.pb.h>

#include <chrono>
#include <set>
#include <string>

#include <gz/common/Profiler.hh>
#include <gz/math/AxisAlignedBox.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Static.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/Conversions.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Events.hh"

using namespace std::chrono_literals;

using namespace gz;
using namespace sim;
using namespace systems;

// Private data class.
class gz::sim::systems::LogVideoRecorderPrivate
{
  /// \brief Rewind the log
  public: void Rewind();

  /// \brief Play the log
  public: void Play();

  /// \brief Follow the specified entity
  /// \param[in] _entity Name of entity to follow
  public: void Follow(const std::string &_entity);

  /// \brief Start/stop video recording
  /// \param[in] _record True to start, false to stop
  public: void Record(bool _record);

  /// \brief Transport node.
  public: transport::Node node;

  /// \brief Pose publisher.
  public: transport::Node::Publisher pub;

  /// \brief Keep the name of the world entity
  public: std::string worldName;

  /// Models to follow and record videos
  public: std::set<std::string> modelsToRecord;

  /// Models for which videos have already been recorded
  public: std::set<std::string> modelsRecorded;

  /// \brief Video record service name
  public: std::string videoRecordService;

  /// \brief Move to service name
  public: std::string moveToService;

  /// \brief Follow service name
  public: std::string followService;

  /// \brief Playback control service name
  public: std::string playbackControlService;

  /// \brief True to indicate recording in progress
  public: bool recording{false};

  /// \brief Pointer to the event manager
  public: EventManager *eventManager{nullptr};

  /// \brief Request to rewind log playback
  public: bool rewindRequested{false};

  /// \brief Request to stop video recording
  public: bool recordStopRequested{false};

  /// \brief Time when video recording stop request is made
  public: std::chrono::time_point<std::chrono::system_clock> recordStopTime;

  /// \brief Video encoding format
  public: std::string videoFormat{"mp4"};

  /// \brief Filename of temp video recording
  public: std::string tmpVideoFilename =
      "tmp_video_recording." + this->videoFormat;

  /// \brief Name of model currently being followed and recorded
  public: std::string modelName;

  /// \brief Publisher for status msgs
  public: transport::Node::Publisher statusPub;

  /// \brief Status message in string
  public: msgs::StringMsg statusMsg;

  /// \brief Regions to look for entities when recording playback videos
  public: std::vector<math::AxisAlignedBox> regions;

  /// \brief Time when system is loaded
  public: std::chrono::time_point<std::chrono::system_clock> loadTime;

  /// \brief Sim time to start recording
  public: std::chrono::steady_clock::duration startTime;

  /// \brief Sim time to stop recording
  public: std::chrono::steady_clock::duration endTime;

  /// \brief Auto exit when log playback recording ends
  public: bool exitOnFinish = false;
};

//////////////////////////////////////////////////
LogVideoRecorder::LogVideoRecorder()
  : System(), dataPtr(std::make_unique<LogVideoRecorderPrivate>())
{
}

//////////////////////////////////////////////////
void LogVideoRecorder::Configure(
    const Entity &, const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &, EventManager &_eventMgr)
{
  this->dataPtr->eventManager = &_eventMgr;

  // For move to service requests
  this->dataPtr->moveToService = "/gui/move_to";

  // For follow service requests
  this->dataPtr->followService = "/gui/follow";

  // For video record requests
  this->dataPtr->videoRecordService = "/gui/record_video";

  // todo(anyone) get world name of log playback sdf and not the world the
  // state log is recorded for
  this->dataPtr->worldName = "default";
  this->dataPtr->playbackControlService = "/world/" + this->dataPtr->worldName
      + "/playback/control";

  // publishes status msgs of the log video recorder
  this->dataPtr->statusPub =
      this->dataPtr->node.Advertise<msgs::StringMsg>(
      "/log_video_recorder/status");

  if (_sdf->HasElement("entity"))
  {
    auto entityElem = _sdf->FindElement("entity");
    while (entityElem)
    {
      this->dataPtr->modelsToRecord.insert(entityElem->Get<std::string>());
      entityElem = entityElem->GetNextElement("entity");
    }
  }

  if (_sdf->HasElement("region"))
  {
    auto regionElem = _sdf->FindElement("region");
    while (regionElem)
    {
      auto min = regionElem->Get<math::Vector3d>("min");
      auto max = regionElem->Get<math::Vector3d>("max");
      math::AxisAlignedBox box(min, max);
      this->dataPtr->regions.push_back(box);

      regionElem = regionElem->GetNextElement("region");
    }
  }

  if (_sdf->HasElement("start_time"))
  {
    auto t = _sdf->Get<double>("start_time");
    this->dataPtr->startTime =
        std::chrono::milliseconds(static_cast<int64_t>(t * 1000.0));
  }

  if (_sdf->HasElement("end_time"))
  {
    auto t = _sdf->Get<double>("end_time");
    std::chrono::milliseconds ms(static_cast<int64_t>(t * 1000.0));
    if (this->dataPtr->startTime > ms)
    {
      gzerr << "<start_time> cannot be larger than <end_time>" << std::endl;
    }
    else
    {
      this->dataPtr->endTime = ms;
    }
  }

  if (_sdf->HasElement("exit_on_finish"))
  {
    this->dataPtr->exitOnFinish = _sdf->Get<bool>("exit_on_finish");
  }

  this->dataPtr->loadTime = std::chrono::system_clock::now();
}

//////////////////////////////////////////////////
void LogVideoRecorder::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("LogVideoRecorder::PostUpdate");

  // record videos for models in the specified regions.
  if (!this->dataPtr->regions.empty())
  {
    _ecm.Each<components::Model, components::Name, components::Pose>(
        [&](const Entity &, const components::Model *,
            const components::Name *_nameComp,
            const components::Pose *_poseComp) -> bool
        {
          math::Pose3d p = _poseComp->Data();
          for (const auto &box : this->dataPtr->regions)
          {
            if (box.Contains(p.Pos()))
            {
              const std::string &name = _nameComp->Data();
              if (this->dataPtr->modelsRecorded.find(name) ==
                  this->dataPtr->modelsRecorded.end())
              {
                this->dataPtr->modelsToRecord.insert(name);
              }
            }
          }
          return true;
        });
  }

  // play for a few seconds before doing anything
  std::chrono::time_point<std::chrono::system_clock> t =
      std::chrono::system_clock::now();
  if (t - this->dataPtr->loadTime < std::chrono::seconds(5))
    return;

  // start sim
  if (_info.simTime < std::chrono::milliseconds(1))
  {
    if (_info.paused)
      this->dataPtr->Play();

    this->dataPtr->rewindRequested = false;
    return;
  }
  else if (this->dataPtr->rewindRequested)
  {
    return;
  }

  // do not start recording until start time is reached.
  // todo(anyone) use seek to fast forward to start time?
  if (_info.simTime < this->dataPtr->startTime)
  {
    if (_info.paused)
    {
      gzdbg << "Warning: Playback is either manually paused or <start_time> "
             << "is smaller than total log playback time!"
             << std::endl;
    }
    return;
  }

  // Video recording stopped. We need to save a copy of the video file
  if (this->dataPtr->recordStopRequested)
  {
    // give it some time for video encording to write the finalize encoding
    std::chrono::time_point<std::chrono::system_clock> now =
      std::chrono::system_clock::now();
    if (now - this->dataPtr->recordStopTime < std::chrono::seconds(5))
      return;

    if (common::exists(this->dataPtr->tmpVideoFilename))
    {
      std::string filename =
          this->dataPtr->modelName + "." + this->dataPtr->videoFormat;
      common::moveFile(this->dataPtr->tmpVideoFilename, filename);

      // Remove old temp file, if it exists.
      std::remove(this->dataPtr->tmpVideoFilename.c_str());
    }
    this->dataPtr->Rewind();
    this->dataPtr->recordStopRequested = false;
    return;
  }


  // plugin is idle, look for next model to record video for
  if (!this->dataPtr->recording)
  {
    if (!this->dataPtr->modelsToRecord.empty())
    {
      auto modelIt = this->dataPtr->modelsToRecord.begin();
      std::string model = *modelIt;
      this->dataPtr->modelsRecorded.insert(model);
      this->dataPtr->modelsToRecord.erase(modelIt);
      this->dataPtr->Follow(model);
      this->dataPtr->Record(true);
      this->dataPtr->Play();
      this->dataPtr->recording = true;
      this->dataPtr->modelName = model;
      return;
    }
    else if (this->dataPtr->modelsRecorded.empty())
    {
      // never recorded before so wait for models to appear
      if (_info.paused)
        this->dataPtr->Play();
    }
    else
    {
      // No more models to record.
      if (this->dataPtr->statusMsg.data().empty())
        gzdbg << "Finish Recording" << std::endl;
      this->dataPtr->statusMsg.set_data("end");
      this->dataPtr->statusPub.Publish(this->dataPtr->statusMsg);

      if (this->dataPtr->exitOnFinish)
        exit(0);
    }
  }

  // check if we need to stop recording
  if (this->dataPtr->recording)
  {
    // stop when either end of log reached (indicated by paused state)
    // or sim time has gone past specified end time
    if (_info.paused ||
        (this->dataPtr->endTime.count() > 0 &&
        _info.simTime > this->dataPtr->endTime))
    {
      this->dataPtr->Record(false);
      this->dataPtr->recording = false;
    }
  }

  // rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzdbg << "Detected Rewind." << std::endl;
  }
}

//////////////////////////////////////////////////
void LogVideoRecorderPrivate::Rewind()
{
  std::function<void(const msgs::Boolean &, const bool)> cb =
      [](const msgs::Boolean &/*_rep*/, const bool _result)
  {
    if (!_result)
      gzerr << "Error sending rewind request" << std::endl;
  };


  msgs::LogPlaybackControl req;
  req.set_rewind(true);
  if (this->node.Request(this->playbackControlService, req, cb))
  {
    gzdbg << "Rewind Playback " << std::endl;
    this->rewindRequested = true;
  }
}

//////////////////////////////////////////////////
void LogVideoRecorderPrivate::Play()
{
  this->eventManager->Emit<events::Pause>(false);
  gzdbg << "Play log " << std::endl;
}

//////////////////////////////////////////////////
void LogVideoRecorderPrivate::Follow(const std::string &_entity)
{
  std::function<void(const msgs::Boolean &, const bool)> cb =
      [](const msgs::Boolean &/*_rep*/, const bool _result)
  {
    if (!_result)
      gzerr << "Error sending follow request" << std::endl;
  };

  msgs::StringMsg req;
  req.set_data(_entity);
  if (this->node.Request(this->followService, req, cb))
  {
    gzdbg << "Following entity: " << _entity << std::endl;
  }
}

//////////////////////////////////////////////////
void LogVideoRecorderPrivate::Record(bool _record)
{
  std::function<void(const msgs::Boolean &, const bool)> cb =
      [](const msgs::Boolean &/*_rep*/, const bool _result)
  {
    if (!_result)
      gzerr << "Error sending record request" << std::endl;
  };

  msgs::VideoRecord req;

  if (_record)
  {
    std::string filename = this->tmpVideoFilename;
    req.set_start(true);
    req.set_format(this->videoFormat);
    req.set_save_filename(filename);
    gzdbg << "Recording video " << filename << std::endl;
  }
  else
  {
    gzdbg << "Stopping video recorder" << std::endl;
    req.set_stop(true);
    this->recordStopRequested = true;
    this->recordStopTime = std::chrono::system_clock::now();
  }
  this->node.Request(this->videoRecordService, req, cb);
}

GZ_ADD_PLUGIN(LogVideoRecorder,
                    System,
                    LogVideoRecorder::ISystemConfigure,
                    LogVideoRecorder::ISystemPostUpdate)

// Add plugin alias so that we can refer to the plugin without the version
// namespace
GZ_ADD_PLUGIN_ALIAS(LogVideoRecorder,
                          "gz::sim::systems::LogVideoRecorder")
