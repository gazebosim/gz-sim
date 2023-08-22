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

#include <gz/msgs/sdf_generator_config.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>

#include <fstream>

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/gui/Application.hh>
#include <gz/msgs/Utility.hh>

#include "GuiFileHandler.hh"

using namespace gz;
using namespace sim;
using namespace sim::gui;

/////////////////////////////////////////////////
void GuiFileHandler::SaveWorldAs(const QString &_fileUrl,
                                 QObject *_config)
{
  GZ_PROFILE("GuiFileHandler::SaveWorldAs");
  QUrl url(_fileUrl);

  bool status = false;
  std::stringstream statusMsg;

  std::string suffix = ".sdf";
  if (url.fileName().endsWith(".sdf"))
    suffix = "";

  std::string localPath = url.toLocalFile().toStdString() + suffix;
  std::string service{"/gazebo/worlds"};
  msgs::StringMsg_V worldsMsg;

  bool result{false};
  unsigned int timeout{5000};
  bool ret = this->node.Request(service, timeout, worldsMsg, result);
  if (!ret || !result)
  {
    statusMsg << "Service call to " << service
                  << " failed. Cannot save world.\n";
  }
  // TODO(addisu) Support saving multiple worlds
  else if (worldsMsg.data_size() > 0)
  {
    const auto &worldName = worldsMsg.data(0);
    const std::string sdfGenService{std::string("/world/") + worldName +
                                    "/generate_world_sdf"};
    msgs::StringMsg genWorldSdf;
    msgs::SdfGeneratorConfig req;

    auto *globalConfig = req.mutable_global_entity_gen_config();
    msgs::Set(globalConfig->mutable_expand_include_tags(),
              _config->property("expandIncludeTags").toBool());
    msgs::Set(globalConfig->mutable_save_fuel_version(),
              _config->property("saveFuelModelVersion").toBool());

    bool serviceCall =
        this->node.Request(sdfGenService, req, timeout, genWorldSdf, result);
    if (serviceCall && result && !genWorldSdf.data().empty())
    {
      gzdbg << "Saving world: " << worldName << " to: " << localPath << "\n";
      std::ofstream fs(localPath, std::ios::out);
      if (fs.is_open())
      {
        fs << genWorldSdf.data();
        status = true;
        statusMsg << "World saved to " << localPath << "\n";
      }
      else
      {
        statusMsg << "File: " << localPath << " could not be opened for "
                      << "saving. Please check that the directory containg the "
                      << "file exists and the correct permissions are set.\n";
      }
    }
    else
    {
      if (!serviceCall)
      {
        statusMsg << "Service call for generating world SDFormat timed out\n";
      }
      statusMsg << "Unknown error occured when saving the world. Please check "
                << "the console output of gz-sim\n";
    }
  }

  if (!status)
  {
    gzerr << statusMsg.str();
  }
  else
  {
    gzmsg << statusMsg.str();
  }
  emit newSaveWorldStatus(status, QString::fromStdString(statusMsg.str()));
}
