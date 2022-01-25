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
#include "ModelPhotoShoot.hh"

#include <ignition/common/Image.hh>
#include <ignition/gazebo/rendering/Events.hh>
#include <ignition/msgs.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/Visual.hh>

using namespace ignition;
using namespace gazebo;
using namespace systems;

ModelPhotoShoot::ModelPhotoShoot()
    : factoryPub(std::make_shared<ignition::transport::Node>()),
      takePicture(true) {}

ModelPhotoShoot::~ModelPhotoShoot() {}

void ModelPhotoShoot::Configure(const ignition::gazebo::Entity &_entity,
                                const std::shared_ptr<const sdf::Element> &_sdf,
                                ignition::gazebo::EntityComponentManager &_ecm,
                                ignition::gazebo::EventManager &_eventMgr) {

  // Read the configuration values from the sdf
  this->modelLocation = _sdf->Get<std::string>("model_uri");
  if (this->modelLocation.empty()) {
    ignerr << "Please specify the model location through the <model_location>"
              "tag in the sdf file.\n";
    return;
  }
  std::string save_data_location =
      _sdf->Get<std::string>("translation_data_file");
  if (save_data_location.empty()) {
    igndbg << "No data location specified, skipping translaiton data"
              "saving.\n";
  } else {
    igndbg << "Saving translation data to: " << save_data_location << std::endl;
    this->savingFile.open(save_data_location);
  }

  this->connection = _eventMgr.Connect<ignition::gazebo::events::PostRender>(
      std::bind(&ModelPhotoShoot::PerformPostRenderingOperations, this));

  LoadModel(_entity, _ecm);
}

void ModelPhotoShoot::LoadModel(
    const ignition::gazebo::Entity &_entity,
    ignition::gazebo::EntityComponentManager &_ecm) {
  auto world = std::make_shared<ignition::gazebo::Model>(_entity);
  std::string worldName = world->Name(_ecm);

  sdf::SDFPtr modelSdf(new sdf::SDF());

  if (!sdf::init(modelSdf)) {
    ignerr << "ERROR: SDF parsing the xml failed\n";
    return;
  }

  if (!sdf::readFile(this->modelLocation, modelSdf)) {
    ignerr << "Error: SDF parsing the xml failed\n";
    return;
  }

  sdf::ElementPtr modelElem = modelSdf->Root()->GetElement("model");
  this->modelName = modelElem->Get<std::string>("name");

  // Create entity
  std::string service = "/world/" + worldName + "/create";
  ignition::msgs::EntityFactory request;
  request.set_sdf(modelSdf->ToString());
  request.set_name(this->modelName);

  ignition::msgs::Boolean response;
  bool result;
  uint32_t timeout = 5000;
  bool executed =
      this->factoryPub->Request(service, request, timeout, response, result);
  if (executed) {
    if (result && response.data()) {
      igndbg << "Requested creation of entity: " << this->modelName.c_str()
             << std::endl;
    } else {
      ignerr << "Failed request to create entity.\n"
             << request.DebugString().c_str();
    }
  } else {
    ignerr << "Request to create entity from service "
           << request.DebugString().c_str() << "timer out ...\n";
  }
}

void ModelPhotoShoot::PerformPostRenderingOperations() {
  igndbg << "PerformPostRenderingOperations\n";

  ignition::rendering::ScenePtr scene =
      ignition::rendering::sceneFromFirstRenderEngine();
  ignition::rendering::VisualPtr modelVisual =
      scene->VisualByName(this->modelName);

  ignition::rendering::VisualPtr root = scene->RootVisual();

  if (modelVisual && takePicture) {

    scene->SetAmbientLight(0.3, 0.3, 0.3);
    scene->SetBackgroundColor(0.3, 0.3, 0.3);

    // create directional light
    ignition::rendering::DirectionalLightPtr light0 =
        scene->CreateDirectionalLight();
    light0->SetDirection(-0.5, 0.5, -1);
    light0->SetDiffuseColor(0.8, 0.8, 0.8);
    light0->SetSpecularColor(0.5, 0.5, 0.5);
    root->AddChild(light0);

    // create point light
    ignition::rendering::PointLightPtr light2 = scene->CreatePointLight();
    light2->SetDiffuseColor(0.5, 0.5, 0.5);
    light2->SetSpecularColor(0.5, 0.5, 0.5);
    light2->SetLocalPosition(3, 5, 5);
    root->AddChild(light2);

    for (unsigned int i = 0; i < scene->NodeCount(); ++i) {
      auto camera = std::dynamic_pointer_cast<ignition::rendering::Camera>(
          scene->NodeByIndex(i));
      if (nullptr != camera)
        std::cout << camera->Name() << std::endl;
      if (nullptr != camera && camera->Name() == "photo_shoot::link::camera") {

        // Set the model pose
        ignition::math::AxisAlignedBox bbox = modelVisual->LocalBoundingBox();
        ignition::rendering::WireBoxPtr wireBox = scene->CreateWireBox();

        double scaling = 1.0 / bbox.Size().Max();

        // Compute the model translation.
        ignition::math::Vector3d trans = bbox.Center();
        trans *= -scaling;

        if (savingFile.is_open())
          savingFile << "Translation: " << trans << std::endl;

        // Normalize the size of the visual
        //         modelVisual->SetLocalScale(ignition::math::Vector3d(scaling,
        //         scaling, scaling));
        modelVisual->SetWorldPose(
            ignition::math::Pose3d(trans.X(), trans.Y(), trans.Z(), 0, 0, 0));

        ignition::math::Pose3d pose;

        // Perspective view
        pose.Pos().Set(1.6, -1.6, 1.2);
        pose.Rot().Euler(0, IGN_DTOR(30), IGN_DTOR(-225));
        SavePicture(camera, pose, "1");

        // Top view
        pose.Pos().Set(0, 0, 2.2);
        pose.Rot().Euler(0, IGN_DTOR(90), 0);
        SavePicture(camera, pose, "2");

        // Front view
        pose.Pos().Set(2.2, 0, 0);
        pose.Rot().Euler(0, 0, IGN_DTOR(-180));
        SavePicture(camera, pose, "3");

        // Side view
        pose.Pos().Set(0, 2.2, 0);
        pose.Rot().Euler(0, 0, IGN_DTOR(-90));
        SavePicture(camera, pose, "4");

        // Back view
        pose.Pos().Set(-2.2, 0, 0);
        pose.Rot().Euler(0, 0, 0);
        SavePicture(camera, pose, "5");

        takePicture = false;
      }
    }
  }
}

void ModelPhotoShoot::SavePicture(const ignition::rendering::CameraPtr camera,
                                  const ignition::math::Pose3d pose,
                                  const std::string name) {
  unsigned int width = camera->ImageWidth();
  unsigned int height = camera->ImageHeight();

  ignition::common::Image image;

  camera->SetWorldPose(pose);
  auto cameraImage = camera->CreateImage();
  camera->Capture(cameraImage);
  auto formatStr = ignition::rendering::PixelUtil::Name(camera->ImageFormat());
  auto format = ignition::common::Image::ConvertPixelFormat(formatStr);
  image.SetFromData(cameraImage.Data<unsigned char>(), width, height, format);
  std::string fullname = name + ".png";
  image.SavePNG(fullname);

  igndbg << "Saved image to [" << fullname << "]" << std::endl;
}

IGNITION_ADD_PLUGIN(ModelPhotoShoot, ignition::gazebo::System,
                    ModelPhotoShoot::ISystemConfigure)

IGNITION_ADD_PLUGIN_ALIAS(ModelPhotoShoot,
                          "ignition::gazebo::systems::ModelPhotoShoot")
