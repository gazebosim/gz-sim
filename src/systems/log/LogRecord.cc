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
#include <ignition/msgs/stringmsg.pb.h>

#include <string>
#include <fstream>
#include <ctime>
#include <set>
#include <list>
// #include <system>
#include <filesystem>
// #include <algorithm/string/predicate.hpp>

#include <ignition/common/Filesystem.hh>
#include <ignition/common/SystemPaths.hh>
#include <ignition/common/Util.hh>
#include <ignition/fuel_tools/Zip.hh>
#include <ignition/msgs/Utility.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/transport/log/Log.hh>
#include <ignition/transport/log/Recorder.hh>

#include <sdf/World.hh>
#include <sdf/Geometry.hh>
#include <sdf/Mesh.hh>

#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Material.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Visual.hh"

using namespace ignition;
using namespace ignition::gazebo::systems;

// Private data class.
class ignition::gazebo::systems::LogRecordPrivate
{
  /// \brief Start recording
  /// \param[in] _logPath Path to record to.
  /// \return True if any recorder has been started successfully.
  public: bool Start(const std::string &_logPath = std::string(""));

  /// \brief Default directory to record to
  public: static std::string DefaultRecordPath();

  /// \brief Save model resources while recording a log, such as meshes
  /// and textures.
  public: void LogModelResources(EntityComponentManager &_ecm);

  /// \brief Return true if all the models are saved successfully.
  /// \return True if all the models are saved successfully.
  public: bool SaveModels(const std::set<std::string> &models);

  /// \brief Return true if all the files are saved successfully.
  /// \return True if all the files are saved successfully, and false if
  /// there are errors saving the files.
  public: bool SaveFiles(const std::set<std::string> &resources);

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

  /// \brief Clock used to timestamp recorded messages with sim time
  /// coming from /clock topic. This is not the timestamp on the header,
  /// rather a logging-specific stamp.
  /// In case there's disagreement between these stamps, the one in the
  /// header should be used.
  public: std::unique_ptr<transport::NetworkClock> clock;

  /// \brief Name of this world
  public: std::string worldName;

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

  /// \brief Record with model resources.
  public: bool recordResources = true;

  /// \brief List of saved models if record with resources is enabled.
  public: std::set<std::string> savedModels;

  /// \brief List of saved files if record with resources is enabled.
  public: std::set<std::string> savedFiles;
};

bool LogRecordPrivate::started{false};

//////////////////////////////////////////////////
std::string LogRecordPrivate::DefaultRecordPath()
{
  std::string home;
  common::env(IGN_HOMEDIR, home);

  std::string timestamp = common::systemTimeISO();

  std::string path = common::joinPaths(home,
    ".ignition", "gazebo", "log", timestamp);

  return path;
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

    this->dataPtr->CompressStateAndResources();

    this->dataPtr->savedModels.clear();
    this->dataPtr->savedFiles.clear();

    ignmsg << "Stopping recording" << std::endl;
  }
}

//////////////////////////////////////////////////
void LogRecord::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm, EventManager &/*_eventMgr*/)
{
  this->dataPtr->sdf = _sdf;

  this->dataPtr->worldName = _ecm.Component<components::Name>(_entity)->Data();

  // If plugin is specified in both the SDF tag and on command line, only
  //   activate one recorder.
  if (!LogRecordPrivate::started)
  {
    // Get directory path from SDF param
    this->dataPtr->Start(_sdf->Get<std::string>("path"));
  }
  else
  {
    ignwarn << "A LogRecord instance has already been started. "
      << "Will not start another.\n";
  }
}

//////////////////////////////////////////////////
bool LogRecord::RecordResources() const
{
  return this->dataPtr->recordResources;
}

//////////////////////////////////////////////////
void LogRecord::SetRecordResources(const bool _record)
{
  this->dataPtr->recordResources = _record;
}

//////////////////////////////////////////////////
bool LogRecordPrivate::Start(const std::string &_logPath)
{
  // Only start one recorder instance
  if (LogRecordPrivate::started)
  {
    ignwarn << "A LogRecord instance has already been started. "
      << "Will not start another.\n";
    return true;
  }
  LogRecordPrivate::started = true;

  this->logPath = _logPath;

  // If unspecified, or specified is not a directory, use default directory
  if (this->logPath.empty() ||
      (common::exists(this->logPath) && !common::isDirectory(this->logPath)))
  {
    this->logPath = this->DefaultRecordPath();
    ignmsg << "Unspecified or invalid log path to record to. "
      << "Recording to default location [" << this->logPath << "]" << std::endl;
  }

  // If directory already exists, do not overwrite
  if (common::exists(this->logPath))
  {
    this->logPath = common::uniqueDirectoryPath(this->logPath);
    ignwarn << "Log path already exists on disk! "
      << "Recording instead to [" << this->logPath << "]" << std::endl;
  }

  // Create log directory
  if (!common::exists(this->logPath))
  {
    common::createDirectories(this->logPath);
  }

  // Go up to root of SDF, to record entire SDF file
  sdf::ElementPtr sdfRoot = this->sdf->GetParent();
  while (sdfRoot->GetParent() != nullptr)
  {
    sdfRoot = sdfRoot->GetParent();
  }

  // Construct message with SDF string
  this->sdfMsg.set_data(sdfRoot->ToString(""));

  // Use directory basename as topic name, to be able to retrieve at playback
  std::string sdfTopic = "/" + common::basename(this->logPath) + "/sdf";
  this->sdfPub = this->node.Advertise(sdfTopic, this->sdfMsg.GetTypeName());

  // TODO(louise) Combine with SceneBroadcaster's state topic
  std::string stateTopic = "/world/" + this->worldName + "/changed_state";
  this->statePub = this->node.Advertise<msgs::SerializedStateMap>(stateTopic);

  // Append file name
  std::string dbPath = common::joinPaths(this->logPath, "state.tlog");
  ignmsg << "Recording to log file [" << dbPath << "]" << std::endl;

  // Use ign-transport directly
  sdf::ElementPtr sdfWorld = sdfRoot->GetElement("world");
  this->recorder.AddTopic("/world/" + this->worldName + "/dynamic_pose/info");
  this->recorder.AddTopic(sdfTopic);
  this->recorder.AddTopic(stateTopic);
  // this->recorder.AddTopic(std::regex(".*"));

  // Timestamp messages with sim time from clock topic
  // Note that the message headers should also have a timestamp
  auto clockTopic = "/world/" + this->worldName + "/clock";
  this->clock = std::make_unique<transport::NetworkClock>(clockTopic,
      transport::NetworkClock::TimeBase::SIM);
  this->recorder.Sync(this->clock.get());

  // This calls Log::Open() and loads sql schema
  if (this->recorder.Start(dbPath) ==
      ignition::transport::log::RecorderError::SUCCESS)
  {
    this->instStarted = true;
    return true;
  }
  else
    return false;
}

//////////////////////////////////////////////////
void LogRecordPrivate::LogModelResources(EntityComponentManager &_ecm)
{
  if (!this->recordResources)
    return;

  std::set<std::string> modelNames;
  std::set<std::string> fileNames;
  auto addModelResource = [&](const std::string &_uri)
  {
    if (_uri.empty())
      return;

    const std::string modelPrefix = "model://";
    const std::string filePrefix = "file://";
    std::string prefix;
    if (_uri.find(modelPrefix) == 0)
    {
      // First directory after prefix is the model name
      std::string modelName = _uri.substr(modelPrefix.size(),
        _uri.find("/", modelPrefix.size()) - modelPrefix.size());
      modelNames.insert(modelName);
    }
    else if (_uri.find(filePrefix) == 0 || _uri[0] == '/')
    {
      fileNames.insert(_uri);
    }
  };

  igndbg << "LogModelResources()" << std::endl;

  // Loop through geometries in world
  _ecm.EachNew<components::Geometry>(
      [&](const Entity &/*_entity*/, components::Geometry *geoComp) -> bool
  {
    sdf::Geometry geoSdf = geoComp->Data();
    if (geoSdf.Type() == sdf::GeometryType::MESH)
    {
      std::string meshUri = geoSdf.MeshShape()->Uri();
      if (!meshUri.empty())
      {
        addModelResource(meshUri);
      }
      igndbg << meshUri << std::endl;
    }

    return true;
  });

  // Loop through materials in world
  _ecm.EachNew<components::Material>(
      [&](const Entity &/*_entity*/, components::Material *matComp) -> bool
  {
    sdf::Material matSdf = matComp->Data();
    std::string matUri = matSdf.ScriptUri();
    if (!matUri.empty())
    {
      addModelResource(matUri);
    }
    igndbg << matUri << std::endl;

    return true;
  });

  if (!this->SaveModels(modelNames) ||
      !this->SaveFiles(fileNames))
  {
    ignwarn << "Failed to save model resources during logging\n";
  }
}

//////////////////////////////////////////////////
bool LogRecordPrivate::SaveModels(const std::set<std::string> &_models)
{
  std::set<std::string> diff;
  std::set_difference(_models.begin(), _models.end(),
      this->savedModels.begin(), this->savedModels.end(),
      std::inserter(diff, diff.begin()));

  for (auto &model : diff)
  {
    if (model.empty())
      continue;

    this->savedModels.insert(model);

    // Look for the specified file in Gazebo model paths
    std::string srcModelPath = common::findFile(model);
    if (!srcModelPath.empty())
    {
      // Copy file to recording directory
      std::string destModelPath = common::joinPaths(this->logPath, model);
      if (!common::copyDirectory(srcModelPath, destModelPath))
      {
        ignerr << "Failed to copy model from '" << srcModelPath
               << "' to '" << destModelPath << "'" << std::endl;
      }
    }
    else
    {
      ignwarn << "Model: " << model << " not found, "
        << "please check the value of env variable GAZEBO_MODEL_PATH\n";
    }
  }
  return true;
}

//////////////////////////////////////////////////
bool LogRecordPrivate::SaveFiles(const std::set<std::string> &_files)
{
  if (_files.empty())
    return false;

  bool saveError = false;
  std::set<std::string> diff;
  std::set_difference(_files.begin(), _files.end(),
      this->savedFiles.begin(),
      this->savedFiles.end(),
      std::inserter(diff, diff.begin()));

  for (auto &file : diff)
  {
    if (file.empty())
      continue;

    this->savedFiles.insert(file);

    bool fileFound = false;
    std::string prefix = "file://";
    std::string fileName = file;

    std::string srcPath;
    if (fileName.find(prefix) == 0u)
    {
      // strip prefix
      fileName = file.substr(prefix.size());

      // search in gazebo path
      srcPath = common::findFile(fileName);
      if (!srcPath.empty())
      {
        fileFound = true;
      }
    }

    // copy resource
    // NOTE: if file is a mesh, e.g. box.dae, it could contain reference to
    // to texture files in other directories. A hacky workaround is to copy
    // entire model dir
    if (fileFound)
    {
      // HACK! copy entire model dir if mesh
      // A better way is to get the model's root directory, with a diff to
      //   environment variable. But for absolute paths in e.g. downloaded
      //   fuel files in ~/.ignition, that is not possible.
      size_t meshIdx = fileName.find("/meshes/");
      if (meshIdx != std::string::npos)
      {
        // Entire path up to meshes directory
        std::string modelPath = fileName.substr(0, meshIdx);
        std::string destPath = common::joinPaths(this->logPath, modelPath);

        size_t meshIdx_src = srcPath.find("/meshes/");
        srcPath = srcPath.substr(0, meshIdx_src);

        igndbg << "source: " << srcPath << std::endl;
        igndbg << "dest: " << destPath << std::endl;

        if (!common::createDirectories(destPath) ||
            !common::copyDirectory(srcPath, destPath))
        {
          ignerr << "Failed to copy model directory from '" << srcPath
                 << "' to '" << destPath << "'" << std::endl;
          saveError = true;
        }
      }
      // else copy only the specified file
      else
      {
        srcPath = common::joinPaths(srcPath, fileName);
        std::string destPath = common::joinPaths(this->logPath, fileName);
        if (common::createDirectories(common::parentPath(destPath)))
          common::copyFile(srcPath, destPath);
        else
        {
          ignerr << "Failed to copy file from '" << srcPath
                 << "' to '" << destPath << "'" << std::endl;
          saveError = true;
        }
      }
    }
    else
    {
      ignerr << "File: " << file << " not found!" << std::endl;
      saveError = true;
    }
  }

  return !saveError;
}

//////////////////////////////////////////////////
void LogRecordPrivate::CompressStateAndResources()
{
  std::string cmp_dest = this->logPath;

  size_t sep_idx = this->logPath.find(common::separator(""));
  // Remove the separator at end of path
  if (sep_idx == this->logPath.length() - 1)
    cmp_dest = this->logPath.substr(0, this->logPath.length() - 1);
  cmp_dest += ".zip";

  // Compress directory
  if (fuel_tools::Zip::Compress(this->logPath, cmp_dest))
  {
    ignmsg << "Compressed log file and resources to [" << cmp_dest
           << "]" << std::endl;
  }
  else
  {
    ignerr << "Failed to compressed log file and resources to [" << cmp_dest
           << "]" << std::endl;
  }
}

//////////////////////////////////////////////////
void LogRecord::Update(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  // If there are new models loaded, save meshes and textures
  if (_ecm.HasNewEntities())
    this->dataPtr->LogModelResources(_ecm);
}

//////////////////////////////////////////////////
void LogRecord::PostUpdate(const UpdateInfo &,
    const EntityComponentManager &_ecm)
{
  // Publish only once
  if (!this->dataPtr->sdfPublished)
  {
    this->dataPtr->sdfPub.Publish(this->dataPtr->sdfMsg);
    this->dataPtr->sdfPublished = true;
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
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::LogRecord,
                    ignition::gazebo::System,
                    LogRecord::ISystemConfigure,
                    LogRecord::ISystemUpdate,
                    LogRecord::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ignition::gazebo::systems::LogRecord,
                          "ignition::gazebo::systems::LogRecord")
