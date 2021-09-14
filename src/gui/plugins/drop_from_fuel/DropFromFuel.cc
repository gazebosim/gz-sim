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

#include "DropFromFuel.hh"

#include <ignition/math/Vector2.hh>

#include <ignition/msgs/boolean.pb.h>

#include <ignition/common/Console.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/common/MeshManager.hh>

#include <ignition/gui/Application.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/gui/Helpers.hh>
#include <ignition/gui/MainWindow.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Utils.hh>

#include <ignition/transport/Node.hh>

#include <sdf/Root.hh>

namespace ignition::gazebo
{
  class DropFromFuelPrivate
  {
    /// \brief Update the 3D scene with new entities
    public: void OnRender();

    /// \brief User camera
    public: rendering::CameraPtr camera{nullptr};

    /// \brief Pointer to the rendering scene
    public: rendering::ScenePtr scene{nullptr};

    /// \brief Ray query for mouse clicks
    public: rendering::RayQueryPtr rayQuery{nullptr};

    /// \brief Transport node for making transform control requests
    public: ignition::transport::Node node;

    /// \brief Name of the world
    public: std::string worldName;
  };
}

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
void DropFromFuelPrivate::OnRender()
{
  if (nullptr == this->scene)
  {
    this->scene = rendering::sceneFromFirstRenderEngine();
    if (nullptr == this->scene)
    {
      return;
    }

    for (unsigned int i = 0; i < this->scene->NodeCount(); ++i)
    {
      auto cam = std::dynamic_pointer_cast<rendering::Camera>(
        this->scene->NodeByIndex(i));
      if (cam)
      {
        if (std::get<bool>(cam->UserData("user-camera")))
        {
          this->camera = cam;

          // Ray Query
          this->rayQuery = this->camera->Scene()->CreateRayQuery();

          igndbg << "DropFromFuel plugin is using camera ["
                 << this->camera->Name() << "]" << std::endl;
          break;
        }
      }
    }
  }
}

/////////////////////////////////////////////////
DropFromFuel::DropFromFuel()
  : ignition::gui::Plugin(),
  dataPtr(std::make_unique<DropFromFuelPrivate>())
{
}

/////////////////////////////////////////////////
DropFromFuel::~DropFromFuel() = default;

/////////////////////////////////////////////////
void DropFromFuel::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Drop from fuel";

  // World name from window, to construct default topics and services
  auto worldNames = gui::worldNames();
  if (!worldNames.empty())
    this->dataPtr->worldName = worldNames[0].toStdString();

  ignition::gui::App()->findChild
    <ignition::gui::MainWindow *>()->installEventFilter(this);
}

////////////////////////////////////////////////
bool DropFromFuel::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == ignition::gui::events::Render::kType)
  {
    this->dataPtr->OnRender();
  }
  else if (_event->type() == ignition::gui::events::DropOnScene::kType)
  {
    auto dropOnSceneEvent =
      reinterpret_cast<ignition::gui::events::DropOnScene *>(_event);
    if (dropOnSceneEvent)
    {
      if (dropOnSceneEvent->DropName().empty())
      {
        ignerr << "Dropped empty entity URI.\n";
      }
      else
      {
        if (this->dataPtr->camera && this->dataPtr->rayQuery)
        {
          std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
              [](const ignition::msgs::Boolean &_res, const bool _result)
          {
            if (!_result || !_res.data())
              ignerr << "Error creating dropped entity." << std::endl;
          };

          math::Vector3d pos = ignition::rendering::screenToScene(
            dropOnSceneEvent->Mouse(),
            this->dataPtr->camera,
            this->dataPtr->rayQuery);

          msgs::EntityFactory req;
          std::string dropStr = dropOnSceneEvent->DropName();
          if (QUrl(QString(dropStr.c_str())).isLocalFile())
          {
            // mesh to sdf model
            common::rtrim(dropStr);

            if (!common::MeshManager::Instance()->IsValidFilename(dropStr))
            {
              ignerr << "Invalid URI: " + dropStr +
                "\nOnly Fuel URLs or mesh file types DAE, OBJ, and STL are supported.";
              return QObject::eventFilter(_obj, _event);
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
      }
    }
  }

  return QObject::eventFilter(_obj, _event);
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::DropFromFuel,
                    ignition::gui::Plugin)
