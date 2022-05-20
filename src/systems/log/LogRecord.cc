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

#include "LogRecord.hh"

#include <sys/stat.h>
#include <gz/msgs/stringmsg.pb.h>

#include <string>
#include <fstream>
#include <ctime>
#include <set>
#include <list>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/common/Profiler.hh>
#include <gz/common/SystemPaths.hh>
#include <gz/common/Util.hh>
#include <gz/fuel_tools/Zip.hh>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/log/Log.hh>
#include <gz/transport/log/Recorder.hh>

#include <sdf/Collision.hh>
#include <sdf/Element.hh>
#include <sdf/Geometry.hh>
#include <sdf/Link.hh>
#include <sdf/Mesh.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/Visual.hh>
#include <sdf/World.hh>

#include "gz/sim/components/Geometry.hh"
#include "gz/sim/components/Light.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Material.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/SourceFilePath.hh"
#include "gz/sim/components/Visual.hh"
#include "gz/sim/components/World.hh"

#include "gz/sim/Util.hh"

using namespace gz;
using namespace gz::sim;
using namespace gz::sim::systems;

// Private data class.
class gz::sim::systems::LogRecordPrivate
{
  /// \brief Start recording
  /// \param[in] _logPath Path to record to.
  /// \param[in] _cmpPath Path to compress recorded files to.
  /// \return True if any recorder has been started successfully.
  public: bool Start(const std::string &_logPath = std::string(""),
    const std::string &_cmpPath = std::string(""));

  /// \brief Append an extension to the end of a directory path.
  /// \param[in] _dir Path of a directory
  /// \param[in] _ext Extension to append, starting with "."
  /// \return Path with extension appended to the end.
  public: std::string AppendExtension(const std::string &_dir,
    const std::string &_ext);

  /// \brief Get whether the model meshes and materials are saved when
  /// recording.
  /// \return True if model meshes and materials are saved when recording.
  public: bool RecordResources() const;

  /// \brief Set whether to save model meshes and materials when recording.
  /// \param[in] _record True to save model resources when recording.
  public: void SetRecordResources(bool _record);

  /// \brief Save model resources while recording a log, such as meshes
  /// and textures.
  /// \param[in] _ecm Reference to an instance of entity component manager
  public: void LogModelResources(const EntityComponentManager &_ecm);

  /// \brief Return true if all the models are saved successfully.
  /// \param[in] _models List of absolute paths of model SDFs to save
  /// \return True if all the models are saved successfully, and false if
  /// there are errors saving the models.
  public: bool SaveModels(const std::set<std::string> &_models);

  /// \brief Compress model resource files and state file into one file.
  public: void CompressStateAndResources();

  /// \brief Indicator of whether any recorder instance has ever been started.
  /// Currently, only one instance is allowed. This enforcement may be removed
  /// in the future.
  public: static bool started;

  /// \brief Indicator of whether this instance has been started
  public: bool instStarted{false};

  /// \brief Ignition transport recorder
  public: transport::log::Recorder recorder;

  /// \brief Directory in which to place log file
  public: std::string logPath{""};

  /// \brief File path to write compressed file
  public: std::string cmpPath{""};

  /// \brief Clock used to timestamp recorded messages with sim time.
  /// This is not the timestamp on the header, rather a logging-specific stamp.
  /// This stamp is used by LogPlayback to step through logs.
  /// In case there's disagreement between these stamps, the one in the
  /// header should be the most accurate.
  public: std::unique_ptr<transport::NetworkClock> clock;

  /// \brief Name of this world
  public: std::string worldName{""};

  /// \brief SDF of this plugin
  public: std::shared_ptr<const sdf::Element> sdf{nullptr};

  /// \brief Transport node for publishing SDF string to be recorded
  public: transport::Node node;

  /// \brief Publisher for SDF string
  public: transport::Node::Publisher sdfPub;

  /// \brief Publisher for state changes
  public: transport::Node::Publisher statePub;

  /// \brief Message holding SDF string of world
  public: msgs::StringMsg sdfMsg;

  /// \brief Whether the SDF has already been published
  public: bool sdfPublished{false};

  /// \brief Record with model resources
  public: bool recordResources{false};

  /// \brief Compress log files at the end
  public: bool compress{false};

  /// \brief List of saved models if record with resources is enabled.
  public: std::set<std::string> savedModels;
};

bool LogRecordPrivate::started{false};

//////////////////////////////////////////////////
std::string LogRecordPrivate::AppendExtension(const std::string &_dir,
  const std::string &_ext)
{
  std::string rv = std::string(_dir);
  size_t sepIdx = _dir.find_last_of(common::separator(""));
  // Remove the separator at end of path
  if (sepIdx == _dir.length() - 1)
    rv = _dir.substr(0, _dir.length() - 1);
  rv += _ext;
  return rv;
}

//////////////////////////////////////////////////
LogRecord::LogRecord()
  : System(), dataPtr(std::make_unique<LogRecordPrivate>())
{
}

//////////////////////////////////////////////////
LogRecord::~LogRecord()
{
  if (this->dataPtr->instStarted)
  {
    // Use ign-transport directly
    this->dataPtr->recorder.Stop();

    if (this->dataPtr->compress)
      this->dataPtr->CompressStateAndResources();
    this->dataPtr->savedModels.clear();

    LogRecordPrivate::started = false;
    gzmsg << "Stopping recording" << std::endl;
  }
}

//////////////////////////////////////////////////
void LogRecord::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm, EventManager &/*_eventMgr*/)
{
  this->dataPtr->sdf = _sdf;

  this->dataPtr->worldName = _ecm.Component<components::Name>(_entity)->Data();

  this->dataPtr->SetRecordResources(_sdf->Get<bool>("record_resources",
    false).first);

  this->dataPtr->compress = _sdf->Get<bool>("compress", false).first;
  this->dataPtr->cmpPath = _sdf->Get<std::string>("compress_path", "").first;

  // If plugin is specified in both the SDF tag and on command line, only
  //   activate one recorder.
  if (!LogRecordPrivate::started)
  {
    auto logPath = _sdf->Get<std::string>("record_path");
    // Path is initialized by server if record is set from command line options.
    //   Otherwise, path is loaded from SDF. If a path is not specified in
    //   SDF, initialize to default here.
    if (logPath.empty())
    {
      logPath = gzLogDirectory();
    }

    this->dataPtr->Start(logPath, this->dataPtr->cmpPath);
  }
  else
  {
    gzwarn << "A LogRecord instance has already been started. "
      << "Will not start another.\n";
  }
}

//////////////////////////////////////////////////
bool LogRecordPrivate::Start(const std::string &_logPath,
  const std::string &_cmpPath)
{
  // Only start one recorder instance
  if (LogRecordPrivate::started)
  {
    gzwarn << "A LogRecord instance has already been started. "
      << "Will not start another.\n";
    return true;
  }

  this->logPath = _logPath;

  // Define path for compressed file
  if (_cmpPath.empty())
    // This case happens if plugin is enabled only from SDF and no command
    //   line arguments enable recording plugin. Then compress path is not
    //   set from Server.
    this->cmpPath = this->AppendExtension(this->logPath, ".zip");
  else
    this->cmpPath = _cmpPath;

  // The ServerConfig takes care of specifying a default log record path.
  // This if-statement is reached if the record plugin is only specified in
  //   SDF, and a <path> is not specified.
  if (this->logPath.empty() ||
      (common::exists(this->logPath) && !common::isDirectory(this->logPath)))
  {
    gzerr << "Unspecified or invalid log path[" << this->logPath << "]. "
      << "Recording will not take place." << std::endl;
    return false;
  }

  LogRecordPrivate::started = true;

  if (this->recordResources)
  {
    gzmsg << "Resources will be recorded\n";
  }

  // Create log directory
  if (!common::exists(this->logPath))
  {
    common::createDirectories(this->logPath);
  }

  // Use directory basename as topic name, to be able to retrieve at playback
  std::string sdfTopic = "/" + common::basename(this->logPath) + "/sdf";
  auto validSdfTopic = transport::TopicUtils::AsValidTopic(sdfTopic);
  if (!validSdfTopic.empty())
  {
    this->sdfPub = this->node.Advertise(validSdfTopic,
        this->sdfMsg.GetTypeName());
  }
  else
  {
    gzerr << "Failed to generate valid topic to publish SDF. Tried ["
           << sdfTopic << "]." << std::endl;
  }

  // TODO(louise) Combine with SceneBroadcaster's state topic
  std::string stateTopic = "/world/" + this->worldName + "/changed_state";
  auto validStateTopic = transport::TopicUtils::AsValidTopic(stateTopic);
  if (!validStateTopic.empty())
  {
    this->statePub = this->node.Advertise<msgs::SerializedStateMap>(
        validStateTopic);
  }
  else
  {
    gzerr << "Failed to generate valid topic to publish state. Tried ["
           << stateTopic << "]." << std::endl;
  }

  // Append file name
  std::string dbPath = common::joinPaths(this->logPath, "state.tlog");
  if (common::exists(dbPath))
  {
    gzmsg << "Overwriting existing file [" << dbPath << "]\n";
    common::removeFile(dbPath);
  }
  gzmsg << "Recording to log file [" << dbPath << "]" << std::endl;

  // Add default topics if no topics were specified.
  igndbg << "Recording default topic[" << sdfTopic << "].\n";
  igndbg << "Recording default topic[" << stateTopic << "].\n";
  this->recorder.AddTopic(sdfTopic);
  this->recorder.AddTopic(stateTopic);

  // Get the topics to record, if any.
  if (this->sdf->HasElement("record_topic"))
  {
    auto ptr = const_cast<sdf::Element *>(this->sdf.get());
    sdf::ElementPtr recordTopicElem = ptr->GetElement("record_topic");

    // This is used to determine if a topic is a regular expression.
    std::regex regexMatch(".*[\\*\\?\\[\\]\\(\\)\\.]+.*");
    while (recordTopicElem)
    {
      std::string topic = recordTopicElem->Get<std::string>();
      if (std::regex_match(topic, regexMatch))
      {
        this->recorder.AddTopic(std::regex(topic));
        igndbg << "Recording topic[" << topic << "] as regular expression.\n";
      }
      else
      {
        this->recorder.AddTopic(topic);
        igndbg << "Recording topic[" << topic << "] as plain topic.\n";
      }
      recordTopicElem = recordTopicElem->GetNextElement("record_topic");
    }
  }

  // Timestamp messages with sim time and republish that time on
  // a ~/log/clock topic (which we don't really need).
  // Note that the message headers should also have a timestamp
  auto clockTopic = "/world/" + this->worldName + "/log/clock";
  this->clock = std::make_unique<transport::NetworkClock>(clockTopic,
      transport::NetworkClock::TimeBase::SIM);
  this->recorder.Sync(this->clock.get());

  // This calls Log::Open() and loads sql schema
  if (this->recorder.Start(dbPath) ==
      gz::transport::log::RecorderError::SUCCESS)
  {
    this->instStarted = true;
    return true;
  }
  else
    return false;
}

//////////////////////////////////////////////////
bool LogRecordPrivate::RecordResources() const
{
  return this->recordResources;
}

//////////////////////////////////////////////////
void LogRecordPrivate::SetRecordResources(const bool _record)
{
  this->recordResources = _record;
}

//////////////////////////////////////////////////
void LogRecordPrivate::LogModelResources(const EntityComponentManager &_ecm)
{
  if (!this->recordResources)
    return;

  std::set<std::string> modelSdfPaths;

  // Loop through geometries in world
  _ecm.EachNew<components::Geometry>(
      [&](const Entity &_entity,
      const components::Geometry *_geoComp) -> bool
  {
    const sdf::Geometry &geoSdf = _geoComp->Data();
    if (geoSdf.Type() == sdf::GeometryType::MESH)
    {
      // Geometry component is in visual or collision entity, which should have
      // link as parent, which has model as parent.
      Entity linkEntity = _ecm.ParentEntity(_entity);
      if (linkEntity != kNullEntity)
      {
        Entity modelEntity = _ecm.ParentEntity(linkEntity);
        if (modelEntity != kNullEntity)
        {
          if (_ecm.EntityHasComponentType(modelEntity,
            components::SourceFilePath::typeId))
          {
            const auto *pathComp =
              _ecm.Component<components::SourceFilePath>(modelEntity);

            if (pathComp != nullptr)
            {
              const std::string & modelPath = pathComp->Data();
              if (!modelPath.empty())
              {
                modelSdfPaths.insert(modelPath);
              }
            }
          }
        }
      }
    }

    return true;
  });

  if (!this->SaveModels(modelSdfPaths))
  {
    gzwarn << "Failed to save model resources during logging\n";
  }
}

//////////////////////////////////////////////////
bool LogRecordPrivate::SaveModels(const std::set<std::string> &_models)
{
  if (!this->recordResources)
    return false;

  if (_models.empty())
    return true;

  bool saveError = false;
  std::set<std::string> diff;
  std::set_difference(_models.begin(), _models.end(),
      this->savedModels.begin(),
      this->savedModels.end(),
      std::inserter(diff, diff.begin()));

  // Compare between _uri and _modelDir. If _modelDir is the prefix of _uri,
  // then remove the prefix portion such that the new URI is a relative path
  // with respect to _modelDir.
  // Returns the relative URI wrt _modelDir, if _modelDir is a prefix of _uri.
  auto convertToRelativePath = [&](const std::string &_uri,
    const std::string &_modelDir) -> std::string
  {
    if (_uri.empty())
      return _uri;

    std::string rt = _uri;

    const std::string prefix = "file://";
    if (_uri.compare(0, prefix.length(), prefix) == 0)
    {
      // Strip prefix
      rt = _uri.substr(prefix.length());
    }

    // If the URI is an absolute path
    if (rt[0] == '/')
    {
      // If model directory is in the prefix of URI, then this is a valid URI
      // pointing to a file inside the model directory.
      if (rt.compare(0, _modelDir.length(), _modelDir) == 0)
      {
        // Convert to relative path by truncating the model directory
        rt = rt.substr(_modelDir.length());

        // Remove extra slashes in front, so this does not get misinterpreted
        // as absolute path
        while (rt[0] == '/')
        {
          rt = rt.substr(1);
        }
        rt = prefix + rt;
      }
      else
      {
        gzerr << "Saving resource files at URI pointing to outside the model "
               << "directory is currently not supported [" << rt << "]"
               << std::endl;
        saveError = true;
        return _uri;
      }
    }

    return rt;
  };

  for (auto &file : diff)
  {
    if (file.empty())
      continue;

    this->savedModels.insert(file);

    bool fileFound = false;
    std::string prefix = "file://";
    std::string fileName = file;

    std::string modelPath;
    if (fileName.compare(0, prefix.length(), prefix) == 0)
    {
      // Strip prefix
      fileName = file.substr(prefix.length());
    }

    if (fileName[0] == '/')
    {
      // search in gazebo path
      modelPath = common::findFile(fileName);
      if (!modelPath.empty())
      {
        fileFound = true;
      }
    }

    // Copy resource
    // TODO(anyone): support shared resources. Currently, the entire model
    // directory is copied, which ensures that meshes and textures in the
    // model directory are copied.
    if (fileFound)
    {
      std::string srcPath = common::parentPath(modelPath);
      std::string destPath = common::joinPaths(this->logPath, srcPath);
      std::string destModelPath = common::joinPaths(this->logPath, modelPath);

      // Read model SDF
      sdf::Root root;
      root.Load(modelPath);

      // Look for URIs in SDF and convert them to paths relative to the model
      // directory
      const sdf::Model *model = root.Model();
      for (uint64_t li = 0; li < model->LinkCount(); li++)
      {
        const sdf::Link *link = model->LinkByIndex(li);
        for (uint64_t ci = 0; ci < link->CollisionCount(); ci++)
        {
          const sdf::Collision *collision = link->CollisionByIndex(ci);
          const sdf::Geometry *geometry = collision->Geom();
          const sdf::Mesh *mesh = geometry->MeshShape();
          if (mesh != nullptr)
          {
            // Replace path with relative path
            std::string relPath = convertToRelativePath(mesh->Uri(), srcPath);
            sdf::ElementPtr meshElem = mesh->Element();
            if (meshElem->HasElement("uri"))
            {
              sdf::ElementPtr uriElem = meshElem->GetElement("uri");
              uriElem->Set(relPath);
            }
          }
        }
        for (uint64_t vi = 0; vi < link->VisualCount(); vi++)
        {
          const sdf::Visual *visual = link->VisualByIndex(vi);
          const sdf::Geometry *geometry = visual->Geom();
          const sdf::Mesh *mesh = geometry->MeshShape();
          if (mesh != nullptr)
          {
            // Replace path with relative path
            std::string relPath = convertToRelativePath(mesh->Uri(), srcPath);
            sdf::ElementPtr meshElem = mesh->Element();
            if (meshElem->HasElement("uri"))
            {
              sdf::ElementPtr uriElem = meshElem->GetElement("uri");
              uriElem->Set(relPath);
            }
          }
        }
      }

      // Copy entire model directory
      if (!common::createDirectories(destPath) ||
          !common::copyDirectory(srcPath, destPath))
      {
        gzerr << "Failed to copy model directory from [" << srcPath
               << "] to [" << destPath << "]" << std::endl;
        saveError = true;
      }
      else
      {
        // Overwrite model SDF with newly generated SDF with relative paths
        std::ofstream ofs(destModelPath);
        ofs << root.Element()->ToString("").c_str();
        ofs.close();
      }
    }
    else
    {
      gzerr << "File: " << file << " not found!" << std::endl;
      saveError = true;
    }
  }

  return !saveError;
}

//////////////////////////////////////////////////
void LogRecordPrivate::CompressStateAndResources()
{
  if (common::exists(this->cmpPath))
  {
    gzmsg << "Removing existing file [" << this->cmpPath << "].\n";
    common::removeFile(this->cmpPath);
  }

  // Compress directory
  if (fuel_tools::Zip::Compress(this->logPath, this->cmpPath))
  {
    gzmsg << "Compressed log file and resources to [" << this->cmpPath
           << "].\nRemoving recorded directory [" << this->logPath << "]."
           << std::endl;
    // Remove directory after compressing successfully
    common::removeAll(this->logPath);
  }
  else
  {
    gzerr << "Failed to compress log file and resources to ["
           << this->cmpPath << "]. Keeping recorded directory ["
           << this->logPath << "]." << std::endl;
  }
}

//////////////////////////////////////////////////
void LogRecord::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &)
{
  IGN_PROFILE("LogRecord::PreUpdate");
  // Safe guard to prevent seg faults if recorder could not be started
  if (!this->dataPtr->instStarted)
    return;
  this->dataPtr->clock->SetTime(_info.simTime);
}

//////////////////////////////////////////////////
void LogRecord::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  IGN_PROFILE("LogRecord::PostUpdate");

  // Safe guard to prevent seg faults if recorder could not be started
  if (!this->dataPtr->instStarted)
    return;

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  // Publish only once
  if (!this->dataPtr->sdfPublished)
  {
    // Construct message with SDF string
    auto worldEntity = _ecm.EntityByComponents(components::World());
    if (worldEntity == kNullEntity)
    {
      gzerr << "Could not find the world entity\n";
    }
    else
    {
      auto worldSdfComp = _ecm.Component<components::WorldSdf>(worldEntity);
      if (worldSdfComp == nullptr || worldSdfComp->Data().Element() == nullptr)
      {
        gzerr << "Could not load world SDF data\n";
      }
      else
      {
        this->dataPtr->sdfMsg.set_data(
            worldSdfComp->Data().Element()->ToString(""));

        this->dataPtr->sdfPub.Publish(this->dataPtr->sdfMsg);
        this->dataPtr->sdfPublished = true;
      }
    }
  }

  // TODO(louise) Use the SceneBroadcaster's topic once that publishes
  // the changed state
  // \todo(anyone) A potential enhancement here is have a keyframe mechanism
  // to store complete state periodically, and then store incremental from
  // that. It would reduce some of the compute on replaying
  // (especially in tools like plotting or seeking through logs).
  msgs::SerializedStateMap stateMsg;
  _ecm.ChangedState(stateMsg);
  if (!stateMsg.entities().empty())
    this->dataPtr->statePub.Publish(stateMsg);

  // If there are new models loaded, save meshes and textures
  if (this->dataPtr->RecordResources() && _ecm.HasNewEntities())
    this->dataPtr->LogModelResources(_ecm);
}

IGNITION_ADD_PLUGIN(gz::sim::systems::LogRecord,
                    gz::sim::System,
                    LogRecord::ISystemConfigure,
                    LogRecord::ISystemPreUpdate,
                    LogRecord::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(gz::sim::systems::LogRecord,
                          "gz::sim::systems::LogRecord")
