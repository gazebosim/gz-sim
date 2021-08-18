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

#include "Shapes.hh"

#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/stringmsg.pb.h>

#include <algorithm>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <ignition/common/Console.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/common/Uuid.hh>

#include <ignition/gui/Application.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/gui/Helpers.hh>
#include <ignition/gui/MainWindow.hh>

#include <ignition/math/Vector2.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/rendering/Camera.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/RayQuery.hh>
#include <ignition/rendering/Visual.hh>
#include <ignition/rendering/Scene.hh>

#include <ignition/transport/Node.hh>
#include <ignition/transport/Publisher.hh>

#include <sdf/Root.hh>

#include "ignition/gazebo/rendering/RenderUtil.hh"
#include "ignition/gazebo/rendering/SceneManager.hh"

namespace ignition::gazebo
{
  class ShapesPrivate
  {
    /// \brief Handle shapes
    public: void handleShapes();

    /// \brief Delete the visuals generated while an entity is being spawned.
    public: void TerminateSpawnPreview();

    /// \brief Generate a preview of a resource.
    /// \param[in] _sdf The SDF to be previewed.
    /// \return True on success, false if failure
    public: bool GeneratePreview(const sdf::Root &_sdf);

    /// \brief Handle model placement requests
    public: void HandleModelPlacement();

    /// \brief Retrieve the point on a plane at z = 0 in the 3D scene hit by a
    /// ray cast from the given 2D screen coordinates.
    /// \param[in] _screenPos 2D coordinates on the screen, in pixels.
    /// \param[in] _camera User camera
    /// \param[in] _rayQuery Ray query for mouse clicks
    /// \param[in] _offset Offset along the plane normal
    /// \return 3D coordinates of a point in the 3D scene.
    math::Vector3d ScreenToPlane(
      const ignition::math::Vector2i &_screenPos,
      const ignition::rendering::CameraPtr &_camera,
      const ignition::rendering::RayQueryPtr &_rayQuery,
      const float offset = 0.0);

    /// \brief Generate a unique entity id.
    /// \return The unique entity id
    ignition::gazebo::Entity UniqueId();

    /// \brief Ignition communication node.
    public: transport::Node node;

    /// \brief Transform control service name
    public: std::string service;

    /// \brief Enable legacy features for plugin to work with GzScene3D.
    /// Disable them to work with the new MinimalScene plugin.
    public: bool legacy{true};

    /// \brief Flag for indicating whether we are spawning or not.
    public: bool isSpawning = false;

    /// \brief Flag for indicating whether the user is currently placing a
    /// resource with the shapes plugin or not
    public: bool isPlacing = false;

    /// \brief The SDF string of the resource to be used with plugins that spawn
    /// entities.
    public: std::string spawnSdfString;

    /// \brief Pointer to the rendering scene
    public: ignition::rendering::ScenePtr scene{nullptr};

    /// \brief A record of the ids currently used by the entity spawner
    /// for easy deletion of visuals later
    public: std::vector<ignition::gazebo::Entity> previewIds;

    /// \brief The visual generated from the spawnSdfString
    public: ignition::rendering::NodePtr spawnPreview{nullptr};

    /// \brief Scene manager
    public: ignition::gazebo::SceneManager sceneManager;

    /// \brief The pose of the spawn preview.
    public: ignition::math::Pose3d spawnPreviewPose =
            ignition::math::Pose3d::Zero;

    /// \brief Mouse event
    public: common::MouseEvent mouseEvent;

    /// \brief Flag to indicate if mouse event is dirty
    public: bool mouseDirty = false;

    /// \brief Flag to indicate if hover event is dirty
    public: bool hoverDirty = false;

    /// \brief The currently hovered mouse position in screen coordinates
    public: math::Vector2i mouseHoverPos = math::Vector2i::Zero;

    /// \brief Ray query for mouse clicks
    public: rendering::RayQueryPtr rayQuery{nullptr};

    /// \brief User camera
    public: rendering::CameraPtr camera;

    /// \brief Name of service for creating entity
    public: std::string createCmdService;

    /// \brief Name of the world
    public: std::string worldName;
  };
}

using namespace ignition;
using namespace gazebo;

// TODO(ahcorde): Replace this when this PR is on ign-rendering6
/////////////////////////////////////////////////
math::Vector3d ShapesPrivate::ScreenToPlane(
    const math::Vector2i &_screenPos,
    const rendering::CameraPtr &_camera,
    const rendering::RayQueryPtr &_rayQuery,
    const float offset)
{
  // Normalize point on the image
  double width = _camera->ImageWidth();
  double height = _camera->ImageHeight();

  double nx = 2.0 * _screenPos.X() / width - 1.0;
  double ny = 1.0 - 2.0 * _screenPos.Y() / height;

  // Make a ray query
  _rayQuery->SetFromCamera(
      _camera, math::Vector2d(nx, ny));

  ignition::math::Planed plane(ignition::math::Vector3d(0, 0, 1), offset);

  math::Vector3d origin = _rayQuery->Origin();
  math::Vector3d direction = _rayQuery->Direction();
  double distance = plane.Distance(origin, direction);
  return origin + direction * distance;
}

/////////////////////////////////////////////////
void ShapesPrivate::HandleModelPlacement()
{
  if (!this->isPlacing)
    return;

  if (this->spawnPreview && this->hoverDirty)
  {
    math::Vector3d pos = this->ScreenToPlane(
      this->mouseHoverPos, this->camera, this->rayQuery);
    pos.Z(this->spawnPreview->WorldPosition().Z());
    this->spawnPreview->SetWorldPosition(pos);
    this->hoverDirty = false;
  }
  if (this->mouseEvent.Button() == common::MouseEvent::LEFT &&
      this->mouseEvent.Type() == common::MouseEvent::RELEASE &&
      !this->mouseEvent.Dragging() && this->mouseDirty)
  {
    // Delete the generated visuals
    this->TerminateSpawnPreview();

    math::Pose3d modelPose = this->spawnPreviewPose;
    std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
        [](const ignition::msgs::Boolean &/*_rep*/, const bool _result)
    {
      if (!_result)
        ignerr << "Error creating model" << std::endl;
    };
    math::Vector3d pos = this->ScreenToPlane(
      this->mouseEvent.Pos(), this->camera, this->rayQuery);
    pos.Z(modelPose.Pos().Z());
    msgs::EntityFactory req;
    if (!this->spawnSdfString.empty())
    {
      req.set_sdf(this->spawnSdfString);
    }
    else
    {
      ignwarn << "Failed to find SDF string or file path" << std::endl;
    }
    req.set_allow_renaming(true);
    msgs::Set(req.mutable_pose(), math::Pose3d(pos, modelPose.Rot()));

    if (this->createCmdService.empty())
    {
      this->createCmdService = "/world/" + this->worldName
          + "/create";
    }
    this->createCmdService = transport::TopicUtils::AsValidTopic(
        this->createCmdService);
    if (this->createCmdService.empty())
    {
      ignerr << "Failed to create valid create command service for world ["
             << this->worldName <<"]" << std::endl;
      return;
    }

    this->node.Request(this->createCmdService, req, cb);
    this->isPlacing = false;
    this->mouseDirty = false;
    this->spawnSdfString.clear();
  }
}

/////////////////////////////////////////////////
ignition::gazebo::Entity ShapesPrivate::UniqueId()
{
  auto timeout = 100000u;
  for (auto i = 0u; i < timeout; ++i)
  {
    Entity id = std::numeric_limits<uint64_t>::max() - i;
    if (!this->sceneManager.HasEntity(id))
      return id;
  }
  return kNullEntity;
}

/////////////////////////////////////////////////
void ShapesPrivate::handleShapes()
{
  if (nullptr == this->scene)
  {
    this->scene = rendering::sceneFromFirstRenderEngine();
    if (nullptr == this->scene)
    {
      return;
    }
    this->sceneManager.SetScene(this->scene);

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

          igndbg << "Shapes plugin is using camera ["
                 << this->camera->Name() << "]" << std::endl;
          break;
        }
      }
    }
  }

  // Shapes
  IGN_PROFILE("IgnRenderer::Render Shapes");
  if (this->isSpawning)
  {
    // Generate spawn preview
    rendering::VisualPtr rootVis = this->scene->RootVisual();
    sdf::Root root;
    if (!this->spawnSdfString.empty())
    {
      root.LoadSdfString(this->spawnSdfString);
    }
    else
    {
      ignwarn << "Failed to spawn: no SDF string or path" << std::endl;
    }
    this->isPlacing = this->GeneratePreview(root);
    this->isSpawning = false;
  }

  this->HandleModelPlacement();
}

/////////////////////////////////////////////////
void ShapesPrivate::TerminateSpawnPreview()
{
  for (auto _id : this->previewIds)
    this->sceneManager.RemoveEntity(_id);
  this->previewIds.clear();
  this->isPlacing = false;
}

/////////////////////////////////////////////////
bool ShapesPrivate::GeneratePreview(const sdf::Root &_sdf)
{
  // Terminate any pre-existing spawned entities
  this->TerminateSpawnPreview();

  if (nullptr == _sdf.Model())
  {
    ignwarn << "Only model entities can be spawned at the moment." << std::endl;
    this->TerminateSpawnPreview();
    return false;
  }

  if (_sdf.Model())
  {
    // Only preview first model
    sdf::Model model = *(_sdf.Model());
    this->spawnPreviewPose = model.RawPose();
    model.SetName(ignition::common::Uuid().String());
    Entity modelId = this->UniqueId();
    if (!modelId)
    {
      this->TerminateSpawnPreview();
      return false;
    }
    this->spawnPreview =
      this->sceneManager.CreateModel(
          modelId, model,
          this->sceneManager.WorldId());

    this->previewIds.push_back(modelId);
    for (auto j = 0u; j < model.LinkCount(); j++)
    {
      sdf::Link link = *(model.LinkByIndex(j));
      link.SetName(ignition::common::Uuid().String());
      Entity linkId = this->UniqueId();
      if (!linkId)
      {
        this->TerminateSpawnPreview();
        return false;
      }
      this->sceneManager.CreateLink(
          linkId, link, modelId);
      this->previewIds.push_back(linkId);
      for (auto k = 0u; k < link.VisualCount(); k++)
      {
       sdf::Visual visual = *(link.VisualByIndex(k));
       visual.SetName(ignition::common::Uuid().String());
       Entity visualId = this->UniqueId();
       if (!visualId)
       {
         this->TerminateSpawnPreview();
         return false;
       }
       this->sceneManager.CreateVisual(
           visualId, visual, linkId);
       this->previewIds.push_back(visualId);
      }
    }
  }
  return true;
}

/////////////////////////////////////////////////
Shapes::Shapes()
  : ignition::gui::Plugin(),
  dataPtr(std::make_unique<ShapesPrivate>())
{
}

/////////////////////////////////////////////////
Shapes::~Shapes() = default;

/////////////////////////////////////////////////
void Shapes::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
    this->title = "Shapes";

  if (_pluginElem)
  {
    if (auto elem = _pluginElem->FirstChildElement("legacy"))
    {
      elem->QueryBoolText(&this->dataPtr->legacy);
    }
  }

  if (this->dataPtr->legacy)
  {
    igndbg << "Legacy mode is enabled; this plugin must be used with "
           << "GzScene3D." << std::endl;
  }
  else
  {
    igndbg << "Legacy mode is disabled; this plugin must be used with "
           << "MinimalScene." << std::endl;
  }

  // World name from window, to construct default topics and services
  auto worldNames = gui::worldNames();
  if (!worldNames.empty())
    this->dataPtr->worldName = worldNames[0].toStdString();

  // For shapes requests
  ignition::gui::App()->findChild
    <ignition::gui::MainWindow *>()->installEventFilter(this);
}

/////////////////////////////////////////////////
void Shapes::OnMode(const QString &_mode)
{
  std::string modelSdfString = _mode.toStdString();
  std::transform(modelSdfString.begin(), modelSdfString.end(),
                 modelSdfString.begin(), ::tolower);

  if (modelSdfString == "box")
  {
    modelSdfString = std::string("<?xml version=\"1.0\"?>"
                                 "<sdf version=\"1.8\">"
                                   "<model name=\"box\">"
                                     "<pose>0 0 0.5 0 0 0</pose>"
                                     "<link name=\"box_link\">"
                                       "<inertial>"
                                         "<inertia>"
                                           "<ixx>0.16666</ixx>"
                                           "<ixy>0</ixy>"
                                           "<ixz>0</ixz>"
                                           "<iyy>0.16666</iyy>"
                                           "<iyz>0</iyz>"
                                           "<izz>0.16666</izz>"
                                         "</inertia>"
                                         "<mass>1.0</mass>"
                                       "</inertial>"
                                       "<collision name=\"box_collision\">"
                                         "<geometry>"
                                           "<box>"
                                             "<size>1 1 1</size>"
                                           "</box>"
                                         "</geometry>"
                                       "</collision>"
                                       "<visual name=\"box_visual\">"
                                         "<geometry>"
                                           "<box>"
                                             "<size>1 1 1</size>"
                                           "</box>"
                                         "</geometry>"
                                       "</visual>"
                                     "</link>"
                                   "</model>"
                                 "</sdf>");
  }
  else if (modelSdfString == "sphere")
  {
    modelSdfString = std::string("<?xml version=\"1.0\"?>"
                                 "<sdf version=\"1.8\">"
                                   "<model name=\"sphere\">"
                                     "<pose>0 0 0.5 0 0 0</pose>"
                                     "<link name=\"sphere_link\">"
                                       "<inertial>"
                                         "<inertia>"
                                           "<ixx>0.1</ixx>"
                                           "<ixy>0</ixy>"
                                           "<ixz>0</ixz>"
                                           "<iyy>0.1</iyy>"
                                           "<iyz>0</iyz>"
                                           "<izz>0.1</izz>"
                                         "</inertia>"
                                         "<mass>1.0</mass>"
                                       "</inertial>"
                                       "<collision name=\"sphere_collision\">"
                                         "<geometry>"
                                           "<sphere>"
                                             "<radius>0.5</radius>"
                                           "</sphere>"
                                         "</geometry>"
                                       "</collision>"
                                       "<visual name=\"sphere_visual\">"
                                         "<geometry>"
                                           "<sphere>"
                                             "<radius>0.5</radius>"
                                           "</sphere>"
                                         "</geometry>"
                                       "</visual>"
                                     "</link>"
                                   "</model>"
                                 "</sdf>");
  }
  else if (modelSdfString == "cylinder")
  {
    modelSdfString = std::string("<?xml version=\"1.0\"?>"
                                 "<sdf version=\"1.8\">"
                                   "<model name=\"cylinder\">"
                                     "<pose>0 0 0.5 0 0 0</pose>"
                                     "<link name=\"cylinder_link\">"
                                       "<inertial>"
                                         "<inertia>"
                                           "<ixx>0.1458</ixx>"
                                           "<ixy>0</ixy>"
                                           "<ixz>0</ixz>"
                                           "<iyy>0.1458</iyy>"
                                           "<iyz>0</iyz>"
                                           "<izz>0.125</izz>"
                                         "</inertia>"
                                         "<mass>1.0</mass>"
                                       "</inertial>"
                                       "<collision name=\"cylinder_collision\">"
                                         "<geometry>"
                                           "<cylinder>"
                                             "<radius>0.5</radius>"
                                             "<length>1.0</length>"
                                           "</cylinder>"
                                         "</geometry>"
                                       "</collision>"
                                       "<visual name=\"cylinder_visual\">"
                                         "<geometry>"
                                           "<cylinder>"
                                             "<radius>0.5</radius>"
                                             "<length>1.0</length>"
                                           "</cylinder>"
                                         "</geometry>"
                                       "</visual>"
                                     "</link>"
                                   "</model>"
                                 "</sdf>");
  }
  else if (modelSdfString == "capsule")
  {
    modelSdfString = std::string("<?xml version=\"1.0\"?>"
                                 "<sdf version=\"1.8\">"
                                   "<model name=\"capsule\">"
                                     "<pose>0 0 0.5 0 0 0</pose>"
                                     "<link name=\"capsule_link\">"
                                       "<inertial>"
                                         "<inertia>"
                                           "<ixx>0.074154</ixx>"
                                           "<ixy>0</ixy>"
                                           "<ixz>0</ixz>"
                                           "<iyy>0.074154</iyy>"
                                           "<iyz>0</iyz>"
                                           "<izz>0.018769</izz>"
                                         "</inertia>"
                                         "<mass>1.0</mass>"
                                       "</inertial>"
                                       "<collision name=\"capsule_collision\">"
                                         "<geometry>"
                                           "<capsule>"
                                             "<radius>0.2</radius>"
                                             "<length>0.6</length>"
                                           "</capsule>"
                                         "</geometry>"
                                       "</collision>"
                                       "<visual name=\"capsule_visual\">"
                                         "<geometry>"
                                           "<capsule>"
                                             "<radius>0.2</radius>"
                                             "<length>0.6</length>"
                                           "</capsule>"
                                         "</geometry>"
                                       "</visual>"
                                     "</link>"
                                   "</model>"
                                 "</sdf>");
  }
  else if (modelSdfString == "ellipsoid")
  {
    modelSdfString = std::string("<?xml version=\"1.0\"?>"
                                 "<sdf version=\"1.8\">"
                                   "<model name=\"ellipsoid\">"
                                     "<pose>0 0 0.5 0 0 0</pose>"
                                     "<link name=\"ellipsoid_link\">"
                                       "<inertial>"
                                         "<inertia>"
                                           "<ixx>0.068</ixx>"
                                           "<ixy>0</ixy>"
                                           "<ixz>0</ixz>"
                                           "<iyy>0.058</iyy>"
                                           "<iyz>0</iyz>"
                                           "<izz>0.026</izz>"
                                         "</inertia>"
                                         "<mass>1.0</mass>"
                                       "</inertial>"
                                     "<collision name=\"ellipsoid_collision\">"
                                         "<geometry>"
                                           "<ellipsoid>"
                                             "<radii>0.2 0.3 0.5</radii>"
                                           "</ellipsoid>"
                                         "</geometry>"
                                       "</collision>"
                                       "<visual name=\"ellipsoid_visual\">"
                                         "<geometry>"
                                           "<ellipsoid>"
                                             "<radii>0.2 0.3 0.5</radii>"
                                           "</ellipsoid>"
                                         "</geometry>"
                                       "</visual>"
                                     "</link>"
                                   "</model>"
                                 "</sdf>");
  }
  else
  {
    ignwarn << "Invalid model string " << modelSdfString << "\n";
    ignwarn << "The valid options are:\n";
    ignwarn << " - box\n";
    ignwarn << " - sphere\n";
    ignwarn << " - capsule\n";
    ignwarn << " - cylinder\n";
    ignwarn << " - ellipsoid\n";
    return;
  }

  // Emit event to GzScene3D in legacy mode
  if (this->dataPtr->legacy)
  {
    ignition::gui::events::SpawnFromDescription event(modelSdfString);
    ignition::gui::App()->sendEvent(
        ignition::gui::App()->findChild<ignition::gui::MainWindow *>(),
        &event);
  }
  else
  {
    this->dataPtr->spawnSdfString = modelSdfString;
    this->dataPtr->isSpawning = true;
  }
}

/////////////////////////////////////////////////
bool Shapes::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == ignition::gui::events::Render::kType)
  {
    this->dataPtr->handleShapes();
  }
  else if (_event->type() == ignition::gui::events::LeftClickOnScene::kType)
  {
    ignition::gui::events::LeftClickOnScene *_e =
      static_cast<ignition::gui::events::LeftClickOnScene*>(_event);
    this->dataPtr->mouseEvent = _e->Mouse();
    this->dataPtr->mouseDirty = true;
  }
  else if (_event->type() == ignition::gui::events::HoverOnScene::kType)
  {
    ignition::gui::events::HoverOnScene *_e =
      static_cast<ignition::gui::events::HoverOnScene*>(_event);
    this->dataPtr->mouseHoverPos = _e->Point();
    this->dataPtr->hoverDirty = true;
  }

  return QObject::eventFilter(_obj, _event);
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::Shapes,
                    ignition::gui::Plugin)
