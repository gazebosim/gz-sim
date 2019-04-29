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
#include <unistd.h>

#include <cmath>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <ignition/common/Console.hh>
#include <ignition/common/MeshManager.hh>
#include <ignition/common/MouseEvent.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include <ignition/msgs.hh>

#include <ignition/rendering.hh>
#include <ignition/rendering/gziface/SceneManager.hh>

#include <ignition/transport/Node.hh>

#include <ignition/gui/Conversions.hh>
#include <ignition/gui/Application.hh>

#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/gui/plugins/RenderWindow.hh"
#include "ignition/gazebo/gui/plugins/Rendering.hh"


////////////////// remove me
#include "ignition/gazebo/components/Camera.hh"
#include "ignition/gazebo/components/DepthCamera.hh"
#include "ignition/gazebo/components/GpuLidar.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Material.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Scene.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/World.hh"
////////////////// remove me


namespace ignition
{
namespace gazebo
{
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
/*  /// \brief Scene manager class for loading and managing objects in the scene
  class SceneManager
  {
    /// \brief Constructor
    public: SceneManager();

    /// \brief Constructor
    /// \param[in] _service Ign transport scene service name
    /// \param[in] _poseTopic Ign transport pose topic name
    /// \param[in] _deletionTopic Ign transport deletion topic name
    /// \param[in] _sceneTopic Ign transport scene topic name
    /// \param[in] _scene Pointer to the rendering scene
    public: SceneManager(const std::string &_service,
                         const std::string &_poseTopic,
                         const std::string &_deletionTopic,
                         const std::string &_sceneTopic,
                         rendering::ScenePtr _scene);

    /// \brief Load the scene manager
    /// \param[in] _service Ign transport service name
    /// \param[in] _poseTopic Ign transport pose topic name
    /// \param[in] _deletionTopic Ign transport deletion topic name
    /// \param[in] _sceneTopic Ign transport scene topic name
    /// \param[in] _scene Pointer to the rendering scene
    public: void Load(const std::string &_service,
                      const std::string &_poseTopic,
                      const std::string &_deletionTopic,
                      const std::string &_sceneTopic,
                      rendering::ScenePtr _scene);

    /// \brief Make the scene service request and populate the scene
    public: void Request();

    /// \brief Update the scene based on pose msgs received
    public: void Update();

    /// \brief Callback function for the pose topic
    /// \param[in] _msg Pose vector msg
    private: void OnPoseVMsg(const msgs::Pose_V &_msg);

    /// \brief Load the scene from a scene msg
    /// \param[in] _msg Scene msg
    private: void LoadScene(const msgs::Scene &_msg);

    /// \brief Callback function for the request topic
    /// \param[in] _msg Deletion message
    private: void OnDeletionMsg(const msgs::UInt32_V &_msg);

    /// \brief Load the scene from a scene msg
    /// \param[in] _msg Scene msg
    private: void OnSceneSrvMsg(const msgs::Scene &_msg, const bool result);

    /// \brief Called when there's an entity is added to the scene
    /// \param[in] _msg Scene msg
    private: void OnSceneMsg(const msgs::Scene &_msg);

    /// \brief Load the model from a model msg
    /// \param[in] _msg Model msg
    /// \return Model visual created from the msg
    private: rendering::VisualPtr LoadModel(const msgs::Model &_msg);

    /// \brief Load a link from a link msg
    /// \param[in] _msg Link msg
    /// \return Link visual created from the msg
    private: rendering::VisualPtr LoadLink(const msgs::Link &_msg);

    /// \brief Load a visual from a visual msg
    /// \param[in] _msg Visual msg
    /// \return Visual visual created from the msg
    private: rendering::VisualPtr LoadVisual(const msgs::Visual &_msg);

    /// \brief Load a geometry from a geometry msg
    /// \param[in] _msg Geometry msg
    /// \param[out] _scale Geometry scale that will be set based on msg param
    /// \param[out] _localPose Additional local pose to be applied after the
    /// visual's pose
    /// \return Geometry object created from the msg
    private: rendering::GeometryPtr LoadGeometry(const msgs::Geometry &_msg,
        math::Vector3d &_scale, math::Pose3d &_localPose);

    /// \brief Load a material from a material msg
    /// \param[in] _msg Material msg
    /// \return Material object created from the msg
    private: rendering::MaterialPtr LoadMaterial(const msgs::Material &_msg);

    /// \brief Load a light from a light msg
    /// \param[in] _msg Light msg
    /// \return Light object created from the msg
    private: rendering::LightPtr LoadLight(const msgs::Light &_msg);

    /// \brief Delete an entity
    /// \param[in] _entity Entity to delete
    private: void DeleteEntity(const unsigned int _entity);

    //// \brief Ign-transport scene service name
    private: std::string service;

    //// \brief Ign-transport pose topic name
    private: std::string poseTopic;

    //// \brief Ign-transport deletion topic name
    private: std::string deletionTopic;

    //// \brief Ign-transport scene topic name
    private: std::string sceneTopic;

    //// \brief Pointer to the rendering scene
    private: rendering::ScenePtr scene;

    //// \brief Mutex to protect the pose msgs
    private: std::mutex mutex;

    /// \brief Map of entity id to pose
    private: std::map<unsigned int, math::Pose3d> poses;

    /// \brief Map of entity id to initial local poses
    /// This is currently used to handle the normal vector in plane visuals. In
    /// general, this can be used to store any local transforms between the
    /// parent Visual and geometry.
    private: std::map<unsigned int, math::Pose3d> localPoses;

    /// \brief Map of visual id to visual pointers.
    private: std::map<unsigned int, rendering::VisualPtr::weak_type> visuals;

    /// \brief Map of light id to light pointers.
    private: std::map<unsigned int, rendering::LightPtr::weak_type> lights;

    /// Entities to be deleted
    private: std::vector<unsigned int> toDeleteEntities;

    /// \brief Keeps the a list of unprocessed scene messages
    private: std::vector<msgs::Scene> sceneMsgs;

    /// \brief Transport node for making service request and subscribing to
    /// pose topic
    private: ignition::transport::Node node;
  };
*/

  /// \brief Private data class for IgnRenderer
  class IgnRendererPrivate
  {
    /// \brief Flag to indicate if mouse event is dirty
    public: bool mouseDirty = false;

    /// \brief Mouse event
    public: common::MouseEvent mouseEvent;

    /// \brief Mouse move distance since last event.
    public: math::Vector2d drag;

    /// \brief Mutex to protect mouse events
    public: std::mutex mutex;

    /// \brief User camera
    public: rendering::CameraPtr camera;

    /// \brief Camera orbit controller
    public: rendering::OrbitViewController viewControl;

    /// \brief Ray query for mouse clicks
    public: rendering::RayQueryPtr rayQuery;

    /// \brief Scene requester to get scene info
    // public: SceneManager sceneManager;

    /// \brief View control focus target
    public: math::Vector3d target;

    public: RenderUtil *renderUtil = nullptr;
  };

  /// \brief Private data class for RenderWindowItem
  class RenderWindowItemPrivate
  {
    /// \brief Keep latest mouse event
    public: common::MouseEvent mouseEvent;

    /// \brief Render thread
    public : RenderThread *renderThread = nullptr;

    //// \brief List of threads
    public: static QList<QThread *> threads;
  };

  /// \brief Private data class for RenderWindow
  class RenderWindowPrivate
  {
    public: RenderUtil renderUtil;
  };
}
}
}

using namespace ignition;
using namespace gazebo;

QList<QThread *> RenderWindowItemPrivate::threads;

/*
/////////////////////////////////////////////////
SceneManager::SceneManager()
{
}

/////////////////////////////////////////////////
SceneManager::SceneManager(const std::string &_service,
                           const std::string &_poseTopic,
                           const std::string &_deletionTopic,
                           const std::string &_sceneTopic,
                           rendering::ScenePtr _scene)
{
  this->Load(_service, _poseTopic, _deletionTopic, _sceneTopic, _scene);
}

/////////////////////////////////////////////////
void SceneManager::Load(const std::string &_service,
                        const std::string &_poseTopic,
                        const std::string &_deletionTopic,
                        const std::string &_sceneTopic,
                        rendering::ScenePtr _scene)
{
  this->service = _service;
  this->poseTopic = _poseTopic;
  this->deletionTopic = _deletionTopic;
  this->sceneTopic = _sceneTopic;
  this->scene = _scene;
}

/////////////////////////////////////////////////
void SceneManager::Request()
{
  // wait for the service to be advertized
  std::vector<transport::ServicePublisher> publishers;
  const std::chrono::duration<double> sleepDuration{1.0};
  const std::size_t tries = 30;
  for (std::size_t i = 0; i < tries; ++i)
  {
    this->node.ServiceInfo(this->service, publishers);
    if (publishers.size() > 0)
      break;
    std::this_thread::sleep_for(sleepDuration);
    igndbg << "Waiting for service " << this->service << "\n";
  }

  if (publishers.empty() ||
      !this->node.Request(this->service, &SceneManager::OnSceneSrvMsg, this))
  {
    ignerr << "Error making service request to " << this->service << std::endl;
  }
}

/////////////////////////////////////////////////
void SceneManager::OnPoseVMsg(const msgs::Pose_V &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  for (int i = 0; i < _msg.pose_size(); ++i)
  {
    math::Pose3d pose = msgs::Convert(_msg.pose(i));

    // apply additional local poses if available
    const auto it = this->localPoses.find(_msg.pose(i).id());
    if (it != this->localPoses.end())
    {
      pose = pose * it->second;
    }

    this->poses[_msg.pose(i).id()] = pose;
  }
}

/////////////////////////////////////////////////
void SceneManager::OnDeletionMsg(const msgs::UInt32_V &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  std::copy(_msg.data().begin(), _msg.data().end(),
            std::back_inserter(this->toDeleteEntities));
}

/////////////////////////////////////////////////
void SceneManager::Update()
{
  // process msgs
  std::lock_guard<std::mutex> lock(this->mutex);

  for (const auto &msg : this->sceneMsgs)
  {
    this->LoadScene(msg);
  }
  this->sceneMsgs.clear();

  for (const auto &entity : this->toDeleteEntities)
  {
    this->DeleteEntity(entity);
  }
  this->toDeleteEntities.clear();


  for (auto pIt = this->poses.begin(); pIt != this->poses.end();)
  {
    auto vIt = this->visuals.find(pIt->first);
    if (vIt != this->visuals.end())
    {
      auto visual = vIt->second.lock();
      if (visual)
      {
        visual->SetLocalPose(pIt->second);
      }
      else
      {
        this->visuals.erase(vIt);
      }
      this->poses.erase(pIt++);
    }
    else
    {
      auto lIt = this->lights.find(pIt->first);
      if (lIt != this->lights.end())
      {
        auto light = lIt->second.lock();
        if (light)
        {
          light->SetLocalPose(pIt->second);
        }
        else
        {
          this->lights.erase(lIt);
        }
        this->poses.erase(pIt++);
      }
      else
      {
        ++pIt;
      }
    }
  }

  // Note we are clearing the pose msgs here but later on we may need to
  // consider the case where pose msgs arrive before scene/visual msgs
  this->poses.clear();
}


/////////////////////////////////////////////////
void SceneManager::OnSceneMsg(const msgs::Scene &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->sceneMsgs.push_back(_msg);
}

/////////////////////////////////////////////////
void SceneManager::OnSceneSrvMsg(const msgs::Scene &_msg, const bool result)
{
  if (!result)
  {
    ignerr << "Error making service request to " << this->service
           << std::endl;
    return;
  }

  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->sceneMsgs.push_back(_msg);
  }

  if (!this->poseTopic.empty())
  {
    if (!this->node.Subscribe(this->poseTopic, &SceneManager::OnPoseVMsg, this))
    {
      ignerr << "Error subscribing to pose topic: " << this->poseTopic
        << std::endl;
    }
  }
  else
  {
    ignwarn << "The pose topic, set via <pose_topic>, for the RenderWindow plugin "
      << "is missing or empty. Please set this topic so that the RenderWindow "
      << "can receive and process pose information.\n";
  }

  if (!this->deletionTopic.empty())
  {
    if (!this->node.Subscribe(this->deletionTopic, &SceneManager::OnDeletionMsg,
          this))
    {
      ignerr << "Error subscribing to deletion topic: " << this->deletionTopic
        << std::endl;
    }
  }
  else
  {
    ignwarn << "The deletion topic, set via <deletion_topic>, for the "
      << "RenderWindow plugin is missing or empty. Please set this topic so that "
      << "the RenderWindow can receive and process deletion information.\n";
  }

  if (!this->sceneTopic.empty())
  {
    if (!this->node.Subscribe(
          this->sceneTopic, &SceneManager::OnSceneMsg, this))
    {
      ignerr << "Error subscribing to scene topic: " << this->sceneTopic
             << std::endl;
    }
  }
  else
  {
    ignwarn << "The scene topic, set via <scene_topic>, for the "
      << "RenderWindow plugin is missing or empty. Please set this topic so that "
      << "the RenderWindow can receive and process scene information.\n";
  }
}

void SceneManager::LoadScene(const msgs::Scene &_msg)
{
  rendering::VisualPtr rootVis = this->scene->RootVisual();

  // load models
  for (int i = 0; i < _msg.model_size(); ++i)
  {
    // Only add if it's not already loaded
    if (this->visuals.find(_msg.model(i).id()) == this->visuals.end())
    {
      rendering::VisualPtr modelVis = this->LoadModel(_msg.model(i));
      if (modelVis)
        rootVis->AddChild(modelVis);
      else
        ignerr << "Failed to load model: " << _msg.model(i).name() << std::endl;
    }
  }

  // load lights
  for (int i = 0; i < _msg.light_size(); ++i)
  {
    if (this->lights.find(_msg.light(i).id()) == this->lights.end())
    {
      rendering::LightPtr light = this->LoadLight(_msg.light(i));
      if (light)
        rootVis->AddChild(light);
      else
        ignerr << "Failed to load light: " << _msg.light(i).name() << std::endl;
    }
  }
}

/////////////////////////////////////////////////
rendering::VisualPtr SceneManager::LoadModel(const msgs::Model &_msg)
{
  rendering::VisualPtr modelVis = this->scene->CreateVisual();
  if (_msg.has_pose())
    modelVis->SetLocalPose(msgs::Convert(_msg.pose()));
  this->visuals[_msg.id()] = modelVis;

  // load links
  for (int i = 0; i < _msg.link_size(); ++i)
  {
    rendering::VisualPtr linkVis = this->LoadLink(_msg.link(i));
    if (linkVis)
      modelVis->AddChild(linkVis);
    else
      ignerr << "Failed to load link: " << _msg.link(i).name() << std::endl;
  }

  // load nested models
  for (int i = 0; i < _msg.model_size(); ++i)
  {
    rendering::VisualPtr nestedModelVis = this->LoadModel(_msg.model(i));
    if (nestedModelVis)
      modelVis->AddChild(nestedModelVis);
    else
      ignerr << "Failed to load nested model: " << _msg.model(i).name()
             << std::endl;
  }

  return modelVis;
}

/////////////////////////////////////////////////
rendering::VisualPtr SceneManager::LoadLink(const msgs::Link &_msg)
{
  rendering::VisualPtr linkVis = this->scene->CreateVisual();
  if (_msg.has_pose())
    linkVis->SetLocalPose(msgs::Convert(_msg.pose()));
  this->visuals[_msg.id()] = linkVis;

  // load visuals
  for (int i = 0; i < _msg.visual_size(); ++i)
  {
    rendering::VisualPtr visualVis = this->LoadVisual(_msg.visual(i));
    if (visualVis)
      linkVis->AddChild(visualVis);
    else
      ignerr << "Failed to load visual: " << _msg.visual(i).name() << std::endl;
  }

  // load lights
  for (int i = 0; i < _msg.light_size(); ++i)
  {
    rendering::LightPtr light = this->LoadLight(_msg.light(i));
    if (light)
      linkVis->AddChild(light);
    else
      ignerr << "Failed to load light: " << _msg.light(i).name() << std::endl;
  }

  return linkVis;
}

/////////////////////////////////////////////////
rendering::VisualPtr SceneManager::LoadVisual(const msgs::Visual &_msg)
{
  if (!_msg.has_geometry())
    return rendering::VisualPtr();

  rendering::VisualPtr visualVis = this->scene->CreateVisual();
  this->visuals[_msg.id()] = visualVis;

  math::Vector3d scale = math::Vector3d::One;
  math::Pose3d localPose;
  rendering::GeometryPtr geom =
      this->LoadGeometry(_msg.geometry(), scale, localPose);

  if (_msg.has_pose())
    visualVis->SetLocalPose(msgs::Convert(_msg.pose()) * localPose);
  else
    visualVis->SetLocalPose(localPose);

  if (geom)
  {
    // store the local pose
    this->localPoses[_msg.id()] = localPose;

    visualVis->AddGeometry(geom);
    visualVis->SetLocalScale(scale);

    // set material
    rendering::MaterialPtr material{nullptr};
    if (_msg.has_material())
    {
      material = this->LoadMaterial(_msg.material());
    }
    // Don't set a default material for meshes because they
    // may have their own
    // TODO(anyone) support overriding mesh material
    else if (_msg.geometry().has_mesh())
    {
      material = geom->Material();
    }
    else
    {
      // create default material
      material = this->scene->Material("ign-grey");
      if (!material)
      {
        material = this->scene->CreateMaterial("ign-grey");
        material->SetAmbient(0.3, 0.3, 0.3);
        material->SetDiffuse(0.7, 0.7, 0.7);
        material->SetSpecular(1.0, 1.0, 1.0);
        material->SetRoughness(0.2);
        material->SetMetalness(1.0);
      }
    }

    material->SetTransparency(_msg.transparency());

    // TODO(anyone) Get roughness and metalness from message instead
    // of giving a default value.
    material->SetRoughness(0.3);
    material->SetMetalness(0.3);

    geom->SetMaterial(material);
  }
  else
  {
    ignerr << "Failed to load geometry for visual: " << _msg.name()
           << std::endl;
  }

  return visualVis;
}

/////////////////////////////////////////////////
rendering::GeometryPtr SceneManager::LoadGeometry(const msgs::Geometry &_msg,
    math::Vector3d &_scale, math::Pose3d &_localPose)
{
  math::Vector3d scale = math::Vector3d::One;
  math::Pose3d localPose = math::Pose3d::Zero;
  rendering::GeometryPtr geom{nullptr};
  if (_msg.has_box())
  {
    geom = this->scene->CreateBox();
    if (_msg.box().has_size())
      scale = msgs::Convert(_msg.box().size());
  }
  else if (_msg.has_cylinder())
  {
    geom = this->scene->CreateCylinder();
    scale.X() = _msg.cylinder().radius() * 2;
    scale.Y() = scale.X();
    scale.Z() = _msg.cylinder().length();
  }
  else if (_msg.has_plane())
  {
    geom = this->scene->CreatePlane();

    if (_msg.plane().has_size())
    {
      scale.X() = _msg.plane().size().x();
      scale.Y() = _msg.plane().size().y();
    }

    if (_msg.plane().has_normal())
    {
      // Create a rotation for the plane mesh to account for the normal vector.
      // The rotation is the angle between the +z(0,0,1) vector and the
      // normal, which are both expressed in the local (Visual) frame.
      math::Vector3d normal = msgs::Convert(_msg.plane().normal());
      localPose.Rot().From2Axes(math::Vector3d::UnitZ, normal.Normalized());
    }
  }
  else if (_msg.has_sphere())
  {
    geom = this->scene->CreateSphere();
    scale.X() = _msg.sphere().radius() * 2;
    scale.Y() = scale.X();
    scale.Z() = scale.X();
  }
  else if (_msg.has_mesh())
  {
    if (_msg.mesh().filename().empty())
    {
      ignerr << "Mesh geometry missing filename" << std::endl;
      return geom;
    }
    rendering::MeshDescriptor descriptor;

    // Assume absolute path to mesh file
    descriptor.meshName = _msg.mesh().filename();

    ignition::common::MeshManager* meshManager =
        ignition::common::MeshManager::Instance();
    descriptor.mesh = meshManager->Load(descriptor.meshName);
    geom = this->scene->CreateMesh(descriptor);

    scale = msgs::Convert(_msg.mesh().scale());
  }
  else
  {
    ignerr << "Unsupported geometry type" << std::endl;
  }
  _scale = scale;
  _localPose = localPose;
  return geom;
}

/////////////////////////////////////////////////
rendering::MaterialPtr SceneManager::LoadMaterial(const msgs::Material &_msg)
{
  rendering::MaterialPtr material = this->scene->CreateMaterial();
  if (_msg.has_ambient())
  {
    material->SetAmbient(msgs::Convert(_msg.ambient()));
  }
  if (_msg.has_diffuse())
  {
    material->SetDiffuse(msgs::Convert(_msg.diffuse()));
  }
  if (_msg.has_specular())
  {
    material->SetDiffuse(msgs::Convert(_msg.specular()));
  }
  if (_msg.has_emissive())
  {
    material->SetEmissive(msgs::Convert(_msg.emissive()));
  }

  return material;
}

/////////////////////////////////////////////////
rendering::LightPtr SceneManager::LoadLight(const msgs::Light &_msg)
{
  rendering::LightPtr light;

  switch (_msg.type())
  {
    case msgs::Light_LightType_POINT:
      light = this->scene->CreatePointLight();
      break;
    case msgs::Light_LightType_SPOT:
    {
      light = this->scene->CreateSpotLight();
      rendering::SpotLightPtr spotLight =
          std::dynamic_pointer_cast<rendering::SpotLight>(light);
      spotLight->SetInnerAngle(_msg.spot_inner_angle());
      spotLight->SetOuterAngle(_msg.spot_outer_angle());
      spotLight->SetFalloff(_msg.spot_falloff());
      break;
    }
    case msgs::Light_LightType_DIRECTIONAL:
    {
      light = this->scene->CreateDirectionalLight();
      rendering::DirectionalLightPtr dirLight =
          std::dynamic_pointer_cast<rendering::DirectionalLight>(light);

      if (_msg.has_direction())
        dirLight->SetDirection(msgs::Convert(_msg.direction()));
      break;
    }
    default:
      ignerr << "Light type not supported" << std::endl;
      return light;
  }

  if (_msg.has_pose())
    light->SetLocalPose(msgs::Convert(_msg.pose()));

  if (_msg.has_diffuse())
    light->SetDiffuseColor(msgs::Convert(_msg.diffuse()));

  if (_msg.has_specular())
    light->SetSpecularColor(msgs::Convert(_msg.specular()));

  light->SetAttenuationConstant(_msg.attenuation_constant());
  light->SetAttenuationLinear(_msg.attenuation_linear());
  light->SetAttenuationQuadratic(_msg.attenuation_quadratic());
  light->SetAttenuationRange(_msg.range());

  light->SetCastShadows(_msg.cast_shadows());

  this->lights[_msg.id()] = light;
  return light;
}

/////////////////////////////////////////////////
void SceneManager::DeleteEntity(const unsigned int _entity)
{
  if (this->visuals.find(_entity) != this->visuals.end())
  {
    auto visual = this->visuals[_entity].lock();
    if (visual)
    {
      this->scene->DestroyVisual(visual, true);
    }
    this->visuals.erase(_entity);
  }
  else if (this->lights.find(_entity) != this->lights.end())
  {
    auto light = this->lights[_entity].lock();
    if (light)
    {
      this->scene->DestroyLight(light, true);
    }
    this->lights.erase(_entity);
  }
}
*/

/////////////////////////////////////////////////
IgnRenderer::IgnRenderer()
  : dataPtr(new IgnRendererPrivate)
{
}


/////////////////////////////////////////////////
IgnRenderer::~IgnRenderer()
{
}

////////////////////////////////////////////////
void IgnRenderer::SetRenderUtil(RenderUtil *_renderer)
{
  this->dataPtr->renderUtil = _renderer;
}

/////////////////////////////////////////////////
void IgnRenderer::Render()
{
  if (this->textureDirty)
  {
    this->dataPtr->camera->SetImageWidth(this->textureSize.width());
    this->dataPtr->camera->SetImageHeight(this->textureSize.height());
    this->dataPtr->camera->SetAspectRatio(this->textureSize.width() /
        this->textureSize.height());
    // setting the size should cause the render texture to be rebuilt
    this->dataPtr->camera->PreRender();
    this->textureId = this->dataPtr->camera->RenderTextureGLId();
    this->textureDirty = false;
  }

  // update the scene
//  this->dataPtr->sceneManager.Update();
  this->dataPtr->renderUtil->Update();

  // view control
  this->HandleMouseEvent();

  // update and render to texture
  this->dataPtr->camera->Update();
}

/////////////////////////////////////////////////
void IgnRenderer::HandleMouseEvent()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  if (!this->dataPtr->mouseDirty)
    return;

  this->dataPtr->viewControl.SetCamera(this->dataPtr->camera);

  if (this->dataPtr->mouseEvent.Type() == common::MouseEvent::SCROLL)
  {
    this->dataPtr->target =
        this->ScreenToScene(this->dataPtr->mouseEvent.Pos());
    this->dataPtr->viewControl.SetTarget(this->dataPtr->target);
    double distance = this->dataPtr->camera->WorldPosition().Distance(
        this->dataPtr->target);
    double amount = -this->dataPtr->drag.Y() * distance / 5.0;
    this->dataPtr->viewControl.Zoom(amount);
  }
  else
  {
    if (this->dataPtr->drag == math::Vector2d::Zero)
    {
      this->dataPtr->target = this->ScreenToScene(
          this->dataPtr->mouseEvent.PressPos());
      this->dataPtr->viewControl.SetTarget(this->dataPtr->target);
    }

    // Pan with left button
    if (this->dataPtr->mouseEvent.Buttons() & common::MouseEvent::LEFT)
    {
      this->dataPtr->viewControl.Pan(this->dataPtr->drag);
    }
    // Orbit with middle button
    else if (this->dataPtr->mouseEvent.Buttons() & common::MouseEvent::MIDDLE)
    {
      this->dataPtr->viewControl.Orbit(this->dataPtr->drag);
    }
    else if (this->dataPtr->mouseEvent.Buttons() & common::MouseEvent::RIGHT)
    {
      double hfov = this->dataPtr->camera->HFOV().Radian();
      double vfov = 2.0f * atan(tan(hfov / 2.0f) /
          this->dataPtr->camera->AspectRatio());
      double distance = this->dataPtr->camera->WorldPosition().Distance(
          this->dataPtr->target);
      double amount = ((-this->dataPtr->drag.Y() /
          static_cast<double>(this->dataPtr->camera->ImageHeight()))
          * distance * tan(vfov/2.0) * 6.0);
      this->dataPtr->viewControl.Zoom(amount);
    }
  }
  this->dataPtr->drag = 0;
  this->dataPtr->mouseDirty = false;
}

/////////////////////////////////////////////////
void IgnRenderer::Initialize()
{
  if (this->initialized)
    return;

  if (!this->dataPtr->renderUtil)
  {
    ignerr << "Renderer not set " << std::endl;
    return;
  }

  this->dataPtr->renderUtil->SetUseCurrentGLContext(true);
  this->dataPtr->renderUtil->Init();

  rendering::ScenePtr scene = this->dataPtr->renderUtil->Scene();
  auto root = scene->RootVisual();

  // Camera
  this->dataPtr->camera = scene->CreateCamera();
  root->AddChild(this->dataPtr->camera);
  this->dataPtr->camera->SetLocalPose(this->dataPtr->renderUtil->CameraPose());
  this->dataPtr->camera->SetImageWidth(this->textureSize.width());
  this->dataPtr->camera->SetImageHeight(this->textureSize.height());
  this->dataPtr->camera->SetAntiAliasing(8);
  this->dataPtr->camera->SetHFOV(M_PI * 0.5);
  // setting the size and calling PreRender should cause the render texture to
  //  be rebuilt
  this->dataPtr->camera->PreRender();
  this->textureId = this->dataPtr->camera->RenderTextureGLId();

/*  // Make service call to populate scene
  if (!this->sceneService.empty())
  {
    this->dataPtr->sceneManager.Load(this->sceneService, this->poseTopic,
                                     this->deletionTopic, this->sceneTopic,
                                     scene);
    this->dataPtr->sceneManager.Request();
  }
*/
  std::cerr << "loading gazebo scene " <<  ::getpid() << std::endl;

  // Ray Query
  this->dataPtr->rayQuery = this->dataPtr->camera->Scene()->CreateRayQuery();

  this->initialized = true;
}

/////////////////////////////////////////////////
void IgnRenderer::Destroy()
{
  auto engine = rendering::engine(this->dataPtr->renderUtil->EngineName());
  if (!engine)
    return;
  auto scene = engine->SceneByName(this->dataPtr->renderUtil->SceneName());
  if (!scene)
    return;
  scene->DestroySensor(this->dataPtr->camera);

  // If that was the last sensor, destroy scene
  if (scene->SensorCount() == 0)
  {
    igndbg << "Destroy scene [" << scene->Name() << "]" << std::endl;
    engine->DestroyScene(scene);

    // TODO(anyone) If that was the last scene, terminate engine?
  }
}

/////////////////////////////////////////////////
void IgnRenderer::NewMouseEvent(const common::MouseEvent &_e,
    const math::Vector2d &_drag)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->mouseEvent = _e;
  this->dataPtr->drag += _drag;
  this->dataPtr->mouseDirty = true;
}

/////////////////////////////////////////////////
math::Vector3d IgnRenderer::ScreenToScene(
    const math::Vector2i &_screenPos) const
{
  // Normalize point on the image
  double width = this->dataPtr->camera->ImageWidth();
  double height = this->dataPtr->camera->ImageHeight();

  double nx = 2.0 * _screenPos.X() / width - 1.0;
  double ny = 1.0 - 2.0 * _screenPos.Y() / height;

  // Make a ray query
  this->dataPtr->rayQuery->SetFromCamera(
      this->dataPtr->camera, math::Vector2d(nx, ny));

  auto result = this->dataPtr->rayQuery->ClosestPoint();
  if (result)
    return result.point;

  // Set point to be 10m away if no intersection found
  return this->dataPtr->rayQuery->Origin() +
      this->dataPtr->rayQuery->Direction() * 10;
}

/////////////////////////////////////////////////
RenderThread::RenderThread()
{
  RenderWindowItemPrivate::threads << this;
}

/////////////////////////////////////////////////
void RenderThread::RenderNext()
{
  this->context->makeCurrent(this->surface);

  if (!this->ignRenderer.initialized)
  {
    // Initialize renderer
    this->ignRenderer.Initialize();
  }

  // check if engine has been successfully initialized
  if (!this->ignRenderer.initialized)
  {
    ignerr << "Unable to initialize renderer" << std::endl;
    return;
  }

  this->ignRenderer.Render();

  emit TextureReady(this->ignRenderer.textureId, this->ignRenderer.textureSize);
}

/////////////////////////////////////////////////
void RenderThread::ShutDown()
{
  this->context->makeCurrent(this->surface);

  this->ignRenderer.Destroy();

  this->context->doneCurrent();
  delete this->context;

  // schedule this to be deleted only after we're done cleaning up
  this->surface->deleteLater();

  // Stop event processing, move the thread to GUI and make sure it is deleted.
  this->moveToThread(QGuiApplication::instance()->thread());
}


/////////////////////////////////////////////////
void RenderThread::SizeChanged()
{
  auto item = qobject_cast<QQuickItem *>(this->sender());
  if (!item)
  {
    ignerr << "Internal error, sender is not QQuickItem." << std::endl;
    return;
  }

  if (item->width() <= 0 || item->height() <= 0)
    return;

  this->ignRenderer.textureSize = QSize(item->width(), item->height());
  this->ignRenderer.textureDirty = true;
}

/////////////////////////////////////////////////
TextureNode::TextureNode(QQuickWindow *_window)
    : window(_window)
{
  // Our texture node must have a texture, so use the default 0 texture.
  this->texture = this->window->createTextureFromId(0, QSize(1, 1));
  this->setTexture(this->texture);
}

/////////////////////////////////////////////////
TextureNode::~TextureNode()
{
  delete this->texture;
}

/////////////////////////////////////////////////
void TextureNode::NewTexture(int _id, const QSize &_size)
{
  this->mutex.lock();
  this->id = _id;
  this->size = _size;
  this->mutex.unlock();

  // We cannot call QQuickWindow::update directly here, as this is only allowed
  // from the rendering thread or GUI thread.
  emit PendingNewTexture();
}

/////////////////////////////////////////////////
void TextureNode::PrepareNode()
{
  this->mutex.lock();
  int newId = this->id;
  QSize sz = this->size;
  this->id = 0;
  this->mutex.unlock();
  if (newId)
  {
    delete this->texture;
    // note: include QQuickWindow::TextureHasAlphaChannel if the rendered
    // content has alpha.
    this->texture = this->window->createTextureFromId(
        newId, sz, QQuickWindow::TextureIsOpaque);
    this->setTexture(this->texture);

    this->markDirty(DirtyMaterial);

    // This will notify the rendering thread that the texture is now being
    // rendered and it can start rendering to the other one.
    emit TextureInUse();
  }
}

/////////////////////////////////////////////////
RenderWindowItem::RenderWindowItem(QQuickItem *_parent)
  : QQuickItem(_parent), dataPtr(new RenderWindowItemPrivate)
{
  this->setAcceptedMouseButtons(Qt::AllButtons);
  this->setFlag(ItemHasContents);
  this->dataPtr->renderThread = new RenderThread();
}

/////////////////////////////////////////////////
RenderWindowItem::~RenderWindowItem()
{
}

/////////////////////////////////////////////////
void RenderWindowItem::Ready()
{
  this->dataPtr->renderThread->surface = new QOffscreenSurface();
  this->dataPtr->renderThread->surface->setFormat(
      this->dataPtr->renderThread->context->format());
  this->dataPtr->renderThread->surface->create();

  this->dataPtr->renderThread->ignRenderer.textureSize =
      QSize(std::max({this->width(), 1.0}), std::max({this->height(), 1.0}));

  this->dataPtr->renderThread->moveToThread(this->dataPtr->renderThread);

  this->connect(this, &QObject::destroyed,
      this->dataPtr->renderThread, &RenderThread::ShutDown,
      Qt::QueuedConnection);

  this->connect(this, &QQuickItem::widthChanged,
      this->dataPtr->renderThread, &RenderThread::SizeChanged);
  this->connect(this, &QQuickItem::heightChanged,
      this->dataPtr->renderThread, &RenderThread::SizeChanged);

  this->dataPtr->renderThread->start();
  this->update();
}

/////////////////////////////////////////////////
QSGNode *RenderWindowItem::updatePaintNode(QSGNode *_node,
    QQuickItem::UpdatePaintNodeData *)
{
  TextureNode *node = static_cast<TextureNode *>(_node);

  if (!this->dataPtr->renderThread->context)
  {
    QOpenGLContext *current = this->window()->openglContext();
    // Some GL implementations require that the currently bound context is
    // made non-current before we set up sharing, so we doneCurrent here
    // and makeCurrent down below while setting up our own context.
    current->doneCurrent();

    this->dataPtr->renderThread->context = new QOpenGLContext();
    this->dataPtr->renderThread->context->setFormat(current->format());
    this->dataPtr->renderThread->context->setShareContext(current);
    this->dataPtr->renderThread->context->create();
    this->dataPtr->renderThread->context->moveToThread(
        this->dataPtr->renderThread);

    current->makeCurrent(this->window());

    QMetaObject::invokeMethod(this, "Ready");
    return nullptr;
  }

  if (!node)
  {
    node = new TextureNode(this->window());

    // Set up connections to get the production of render texture in sync with
    // vsync on the rendering thread.
    //
    // When a new texture is ready on the rendering thread, we use a direct
    // connection to the texture node to let it know a new texture can be used.
    // The node will then emit PendingNewTexture which we bind to
    // QQuickWindow::update to schedule a redraw.
    //
    // When the scene graph starts rendering the next frame, the PrepareNode()
    // function is used to update the node with the new texture. Once it
    // completes, it emits TextureInUse() which we connect to the rendering
    // thread's RenderNext() to have it start producing content into its render
    // texture.
    //
    // This rendering pipeline is throttled by vsync on the scene graph
    // rendering thread.

    this->connect(this->dataPtr->renderThread, &RenderThread::TextureReady,
        node, &TextureNode::NewTexture, Qt::DirectConnection);
    this->connect(node, &TextureNode::PendingNewTexture, this->window(),
        &QQuickWindow::update, Qt::QueuedConnection);
    this->connect(this->window(), &QQuickWindow::beforeRendering, node,
        &TextureNode::PrepareNode, Qt::DirectConnection);
    this->connect(node, &TextureNode::TextureInUse, this->dataPtr->renderThread,
        &RenderThread::RenderNext, Qt::QueuedConnection);

    // Get the production of FBO textures started..
    QMetaObject::invokeMethod(this->dataPtr->renderThread, "RenderNext",
        Qt::QueuedConnection);
  }

  node->setRect(this->boundingRect());

  return node;
}

////////////////////////////////////////////////
void RenderWindowItem::SetRenderUtil(RenderUtil *_renderer)
{
  this->dataPtr->renderThread->ignRenderer.SetRenderUtil(_renderer);
}

/*
/////////////////////////////////////////////////
void RenderWindowItem::SetBackgroundColor(const math::Color &_color)
{
  this->dataPtr->renderThread->ignRenderer.backgroundColor = _color;
}

/////////////////////////////////////////////////
void RenderWindowItem::SetAmbientLight(const math::Color &_ambient)
{
  this->dataPtr->renderThread->ignRenderer.ambientLight = _ambient;
}

/////////////////////////////////////////////////
void RenderWindowItem::SetEngineName(const std::string &_name)
{
  this->dataPtr->renderThread->ignRenderer.engineName = _name;
}

/////////////////////////////////////////////////
void RenderWindowItem::SetSceneName(const std::string &_name)
{
  this->dataPtr->renderThread->ignRenderer.sceneName = _name;
}

/////////////////////////////////////////////////
void RenderWindowItem::SetCameraPose(const math::Pose3d &_pose)
{
  this->dataPtr->renderThread->ignRenderer.cameraPose = _pose;
}

/////////////////////////////////////////////////
void RenderWindowItem::SetSceneService(const std::string &_service)
{
  this->dataPtr->renderThread->ignRenderer.sceneService = _service;
}

/////////////////////////////////////////////////
void RenderWindowItem::SetPoseTopic(const std::string &_topic)
{
  this->dataPtr->renderThread->ignRenderer.poseTopic = _topic;
}

/////////////////////////////////////////////////
void RenderWindowItem::SetDeletionTopic(const std::string &_topic)
{
  this->dataPtr->renderThread->ignRenderer.deletionTopic = _topic;
}

/////////////////////////////////////////////////
void RenderWindowItem::SetSceneTopic(const std::string &_topic)
{
  this->dataPtr->renderThread->ignRenderer.sceneTopic = _topic;
}
*/

/////////////////////////////////////////////////
RenderWindow::RenderWindow()
  : GuiPlugin(), dataPtr(new RenderWindowPrivate)
{
  qmlRegisterType<RenderWindowItem>("RenderWindow", 1, 0, "RenderWindow");
}


/////////////////////////////////////////////////
RenderWindow::~RenderWindow()
{
}

/////////////////////////////////////////////////
void RenderWindow::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  RenderWindowItem *renderWindow =
      this->PluginItem()->findChild<RenderWindowItem *>();
  if (!renderWindow)
  {
    ignerr << "Unable to find Render Window item. "
           << "Render window will not be created" << std::endl;
    return;
  }

  if (this->title.empty())
    this->title = "3D Scene";

  // Custom parameters
  if (_pluginElem)
  {
    if (auto elem = _pluginElem->FirstChildElement("engine"))
    {
      //renderWindow->SetEngineName(elem->GetText());
      std::string engineName = elem->GetText();
      if (!engineName.empty())
      {
        this->dataPtr->renderUtil.SetEngineName(engineName);

        // there is a problem with displaying ogre2 render textures that are in
        // sRGB format. Workaround for now is to apply gamma correction
        // manually. There maybe a better way to solve the problem by making
        // OpenGL calls..
        if (engineName == std::string("ogre2"))
          this->PluginItem()->setProperty("gammaCorrect", true);
      }
    }

    if (auto elem = _pluginElem->FirstChildElement("scene"))
    {
      //renderWindow->SetSceneName(elem->GetText());
      this->dataPtr->renderUtil.SetSceneName(elem->GetText());
    }

    if (auto elem = _pluginElem->FirstChildElement("ambient_light"))
    {
      math::Color ambient;
      std::stringstream colorStr;
      colorStr << std::string(elem->GetText());
      colorStr >> ambient;
      // renderWindow->SetAmbientLight(ambient);
      this->dataPtr->renderUtil.SetAmbientLight(ambient);
    }

    if (auto elem = _pluginElem->FirstChildElement("background_color"))
    {
      math::Color bgColor;
      std::stringstream colorStr;
      colorStr << std::string(elem->GetText());
      colorStr >> bgColor;
      // renderWindow->SetBackgroundColor(bgColor);
      this->dataPtr->renderUtil.SetBackgroundColor(bgColor);
    }

    if (auto elem = _pluginElem->FirstChildElement("camera_pose"))
    {
      math::Pose3d pose;
      std::stringstream poseStr;
      poseStr << std::string(elem->GetText());
      poseStr >> pose;
      // renderWindow->SetCameraPose(pose);
      this->dataPtr->renderUtil.SetCameraPose(pose);
    }

/*
    if (auto elem = _pluginElem->FirstChildElement("service"))
    {
      std::string service = elem->GetText();
      renderWindow->SetSceneService(service);
    }

    if (auto elem = _pluginElem->FirstChildElement("pose_topic"))
    {
      std::string topic = elem->GetText();
      renderWindow->SetPoseTopic(topic);
    }

    if (auto elem = _pluginElem->FirstChildElement("deletion_topic"))
    {
      std::string topic = elem->GetText();
      renderWindow->SetDeletionTopic(topic);
    }

    if (auto elem = _pluginElem->FirstChildElement("scene_topic"))
    {
      std::string topic = elem->GetText();
      renderWindow->SetSceneTopic(topic);
    }
*/
  }

  renderWindow->SetRenderUtil(&this->dataPtr->renderUtil);
}

//////////////////////////////////////////////////
void RenderWindow::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  // todo(anyone)
  // populate the scene using ecm
  _ecm.EachNew</*components::Model,*/ components::Name, /*components::Pose,*/
            components::ParentEntity>(
      [&](const Entity &_entity,
//          const components::Model *,
          const components::Name *_name,
//          const components::Pose *_pose,
          const components::ParentEntity *_parent)->bool
      {

        auto modelComp = _ecm.Component<components::Model>(_entity);
        if (modelComp)
          std::cerr << "creating render entities model " << _name->Data() << std::endl;
        else std::cerr << "not a model: " << _name->Data() << std::endl;
        return true;
      });

 _ecm.EachNew<components::Name>(
    [&](const Entity &_entity,
        const components::Name *_name)->bool
  {
    Entity parentEntity{kNullEntity};

    auto parentComp = _ecm.Component<components::ParentEntity>(_entity);
    if (parentComp)
    {
      parentEntity = parentComp->Data();
    }

    std::cerr << "each new entities model " << _name->Data() << std::endl;
    return true;
  });



//  this->dataPtr->renderUtil.UpdateFromECM(_info, _ecm);
}


/////////////////////////////////////////////////
void RenderWindowItem::mousePressEvent(QMouseEvent *_e)
{
  auto event = gui::convert(*_e);
  event.SetPressPos(event.Pos());
  this->dataPtr->mouseEvent = event;

  this->dataPtr->renderThread->ignRenderer.NewMouseEvent(
      this->dataPtr->mouseEvent);
}

////////////////////////////////////////////////
void RenderWindowItem::mouseReleaseEvent(QMouseEvent *_e)
{
  this->dataPtr->mouseEvent = gui::convert(*_e);

  this->dataPtr->renderThread->ignRenderer.NewMouseEvent(
      this->dataPtr->mouseEvent);
}

////////////////////////////////////////////////
void RenderWindowItem::mouseMoveEvent(QMouseEvent *_e)
{
  auto event = gui::convert(*_e);
  event.SetPressPos(this->dataPtr->mouseEvent.PressPos());

  if (!event.Dragging())
    return;

  auto dragInt = event.Pos() - this->dataPtr->mouseEvent.Pos();
  auto dragDistance = math::Vector2d(dragInt.X(), dragInt.Y());

  this->dataPtr->renderThread->ignRenderer.NewMouseEvent(event, dragDistance);
  this->dataPtr->mouseEvent = event;
}

////////////////////////////////////////////////
void RenderWindowItem::wheelEvent(QWheelEvent *_e)
{
  this->dataPtr->mouseEvent.SetType(common::MouseEvent::SCROLL);
  this->dataPtr->mouseEvent.SetPos(_e->x(), _e->y());
  double scroll = (_e->angleDelta().y() > 0) ? -1.0 : 1.0;
  this->dataPtr->renderThread->ignRenderer.NewMouseEvent(
      this->dataPtr->mouseEvent, math::Vector2d(scroll, scroll));
}


///////////////////////////////////////////////////
// void RenderWindow::resizeEvent(QResizeEvent *_e)
// {
//  if (this->dataPtr->renderWindow)
//  {
//    this->dataPtr->renderWindow->OnResize(_e->size().width(),
//                                          _e->size().height());
//  }
//
//  if (this->dataPtr->camera)
//  {
//    this->dataPtr->camera->SetAspectRatio(
//        static_cast<double>(this->width()) / this->height());
//    this->dataPtr->camera->SetHFOV(M_PI * 0.5);
//  }
// }
//

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::RenderWindow,
                    ignition::gui::Plugin)
