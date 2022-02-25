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

#include <random>
#include <string>
#include <vector>

#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/JointAxis.hh"
#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/components/JointPositionLimitsCmd.hh"
#include "ignition/gazebo/components/JointPositionReset.hh"
#include <ignition/common/Image.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/rendering/Events.hh>
#include <ignition/msgs.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/Visual.hh>
#include <ignition/rendering/WireBox.hh>

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Private ModelPhotoShoot data class.
class ignition::gazebo::systems::ModelPhotoShootPrivate
{
  /// \brief Callback for pos rendering operations.
  public: void PerformPostRenderingOperations();

  /// \brief Save a pitcture with the camera from the given pose.
  public: void SavePicture (const ignition::rendering::CameraPtr _camera,
                    const ignition::math::Pose3d _pose,
                    const std::string _fileName) const;

  /// \brief Name of the loaded model.
  public: std::string modelName;

  /// \brief model
  public: std::shared_ptr<ignition::gazebo::Model> model;

  /// \brief Connection to pre-render event callback.
  public: ignition::common::ConnectionPtr connection{nullptr};

  /// \brief Boolean to control we only take the pictures once.
  public: bool takePicture{true};

  /// \brief Boolean to control if joints should adopt random poses.
  public: bool randomPoses{false};

  /// \brief File to save translation and scaling info.
  public: std::ofstream savingFile;
};

//////////////////////////////////////////////////
ModelPhotoShoot::ModelPhotoShoot()
    : System(), dataPtr(std::make_unique<ModelPhotoShootPrivate>())
{
}

//////////////////////////////////////////////////
void ModelPhotoShoot::Configure(const ignition::gazebo::Entity &_entity,
                                const std::shared_ptr<const sdf::Element> &_sdf,
                                ignition::gazebo::EntityComponentManager &_ecm,
                                ignition::gazebo::EventManager &_eventMgr)
{
  std::string save_data_location =
      _sdf->Get<std::string>("translation_data_file");
  if (save_data_location.empty())
  {
    igndbg << "No data location specified, skipping translaiton data"
              "saving.\n";
  }
  else
  {
    igndbg << "Saving translation data to: " << save_data_location << std::endl;
    this->dataPtr->savingFile.open(save_data_location);
  }

  if (_sdf->HasElement("random_joints_pose"))
  {
    this->dataPtr->randomPoses = _sdf->Get<bool>("random_joints_pose");
  }

  this->dataPtr->connection =
      _eventMgr.Connect<ignition::gazebo::events::PostRender>(std::bind(
          &ModelPhotoShootPrivate::PerformPostRenderingOperations,
          this->dataPtr.get()));

  this->dataPtr->model = std::make_shared<ignition::gazebo::Model>(_entity);
  this->dataPtr->modelName = this->dataPtr->model->Name(_ecm);
}

//////////////////////////////////////////////////
void ModelPhotoShoot::PreUpdate(
    const ignition::gazebo::UpdateInfo &,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  if (this->dataPtr->randomPoses)
  {
    std::vector<gazebo::Entity> joints = this->dataPtr->model->Joints(_ecm);
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    for (auto joint = joints.begin(); joint != joints.end(); ++joint)
    {

      auto jointNameComp = _ecm.Component<components::Name>(*joint);
      if (jointNameComp->Data() != "World")
      {
        // Using the JointAxis component to extract the joint pose limits
        auto jointAxisComp = _ecm.Component<components::JointAxis>(*joint);

        float mean =
            (jointAxisComp->Data().Lower() + jointAxisComp->Data().Upper()) / 2;
        float stdv = (jointAxisComp->Data().Upper() - mean) / 3;
        std::normal_distribution<float> distribution(mean, stdv);
        float jointPose = distribution(generator);
        _ecm.SetComponentData<components::JointPositionReset>(*joint,
                                                              {jointPose});
        if (this->dataPtr->savingFile.is_open())
        {
          this->dataPtr->savingFile << jointNameComp->Data() << ": "
                                    << jointPose << std::endl;
        }
      }
    }
    // Only set random joint poses once
    this->dataPtr->randomPoses = false;
  }
}

//////////////////////////////////////////////////
void ModelPhotoShootPrivate::PerformPostRenderingOperations()
{

  ignition::rendering::ScenePtr scene =
      ignition::rendering::sceneFromFirstRenderEngine();
  ignition::rendering::VisualPtr modelVisual =
      scene->VisualByName(this->modelName);

  ignition::rendering::VisualPtr root = scene->RootVisual();

  if (modelVisual && this->takePicture)
  {

    scene->SetAmbientLight(0.3, 0.3, 0.3);

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

    for (unsigned int i = 0; i < scene->NodeCount(); ++i)
    {
      auto camera = std::dynamic_pointer_cast<ignition::rendering::Camera>(
          scene->NodeByIndex(i));
      if (nullptr != camera && camera->Name() == "photo_shoot::link::camera")
      {

        // Set the model pose
        ignition::math::AxisAlignedBox bbox = modelVisual->LocalBoundingBox();
        double scaling = 1.0 / bbox.Size().Max();
        // Compute the model translation.
        ignition::math::Vector3d trans = -bbox.Center();
        if (this->savingFile.is_open()) {
          this->savingFile << "Translation: " << trans << std::endl;
          this->savingFile << "Scaling: " << scaling << std::endl;
        }
        // Normalize the size of the visual
        modelVisual->SetWorldPose(
            ignition::math::Pose3d(trans.X(), trans.Y(), trans.Z(), 0, 0, 0));

        ignition::math::Pose3d pose;
        // Perspective view
        pose.Pos().Set(1.6 / scaling, -1.6 / scaling, 1.2 / scaling);
        pose.Rot().Euler(0, IGN_DTOR(30), IGN_DTOR(-225));
        SavePicture(camera, pose, "1.png");

        // Top view
        pose.Pos().Set(0, 0, 2.2 / scaling);
        pose.Rot().Euler(0, IGN_DTOR(90), 0);
        SavePicture(camera, pose, "2.png");

        // Front view
        pose.Pos().Set(2.2 / scaling, 0, 0);
        pose.Rot().Euler(0, 0, IGN_DTOR(-180));
        SavePicture(camera, pose, "3.png");

        // Side view
        pose.Pos().Set(0, 2.2 / scaling, 0);
        pose.Rot().Euler(0, 0, IGN_DTOR(-90));
        SavePicture(camera, pose, "4.png");

        // Back view
        pose.Pos().Set(-2.2 / scaling, 0, 0);
        pose.Rot().Euler(0, 0, 0);
        SavePicture(camera, pose, "5.png");

        this->takePicture = false;
      }
    }
  }
}

//////////////////////////////////////////////////
void ModelPhotoShootPrivate::SavePicture(
                                  const ignition::rendering::CameraPtr _camera,
                                  const ignition::math::Pose3d _pose,
                                  const std::string _fileName) const
{
  unsigned int width = _camera->ImageWidth();
  unsigned int height = _camera->ImageHeight();

  ignition::common::Image image;

  _camera->SetWorldPose(_pose);
  auto cameraImage = _camera->CreateImage();
  _camera->Capture(cameraImage);
  auto formatStr = ignition::rendering::PixelUtil::Name(_camera->ImageFormat());
  auto format = ignition::common::Image::ConvertPixelFormat(formatStr);
  image.SetFromData(cameraImage.Data<unsigned char>(), width, height, format);
  std::string fullName = _fileName;
  image.SavePNG(fullName);

  igndbg << "Saved image to [" << fullName << "]" << std::endl;
}

IGNITION_ADD_PLUGIN(ModelPhotoShoot, ignition::gazebo::System,
                    ModelPhotoShoot::ISystemConfigure,
                    ModelPhotoShoot::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ModelPhotoShoot,
                          "ignition::gazebo::systems::ModelPhotoShoot")
