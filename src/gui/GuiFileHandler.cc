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

#include <ignition/msgs/sdf_generator_config.pb.h>

#include <fstream>

#include <ignition/common/Console.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/gui/Application.hh>

// Include all components so they have first-class support
#include "ignition/gazebo/components/components.hh"
#include "ignition/gazebo/Conversions.hh"

#include "GuiFileHandler.hh"

using namespace ignition;
using namespace gazebo;
using namespace gazebo::gui;

/////////////////////////////////////////////////
void GuiFileHandler::SaveWorldAs(const QString &_fileUrl,
                                 QObject *_config)
{
  QUrl url(_fileUrl);

  std::string suffix = ".sdf";
  if (url.fileName().endsWith(".sdf"))
    suffix = "";

  std::string localPath = url.toLocalFile().toStdString() + suffix;
  std::string service{"/gazebo/worlds"};
  ignition::msgs::StringMsg_V worldsMsg;

  bool result{false};
  unsigned int timeout{5000};
  this->node.Request(service, timeout, worldsMsg, result);
  // TODO(addisu) Support saving multiple worlds
  if (worldsMsg.data_size() > 0)
  {
    const auto &worldName = worldsMsg.data(0);
    const std::string sdfGenService{std::string("/world/") + worldName +
                                    "/generate_world_sdf"};
    msgs::StringMsg genWorldSdf;
    msgs::SdfGeneratorConfig req;

    auto *globalConfig = req.mutable_global_model_gen_config();
    msgs::Set(globalConfig->mutable_expand_include_tags(),
              _config->property("expandIncludeTags").toBool());
    msgs::Set(globalConfig->mutable_save_fuel_model_version(),
              _config->property("saveFuelModelVersion").toBool());

    bool serviceCall =
        this->node.Request(sdfGenService, req, timeout, genWorldSdf, result);
    if (serviceCall && result && !genWorldSdf.data().empty())
    {
      igndbg << "Saving world: " << worldName << " to: " << localPath << "\n";
      std::ofstream fs(localPath, std::ios::out);
      if (fs.is_open())
      {
        fs << genWorldSdf.data();
      }
      else
      {
        ignerr << "File " << localPath << " could not be opened for saving.\n";
      }
    }
    else
    {
      if (!serviceCall)
      {
        ignerr << "Service call for generating world SDFormat timed out\n";
      }
      ignerr << "World could not be saved\n";
    }
  }
}

