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

#include <gz/common/Image.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Visual.hh>

#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointAxis.hh"
#include "gz/sim/components/JointType.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointPositionReset.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/rendering/Events.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Private ModelPhotoShoot data class.
class gz::sim::systems::ModelPhotoShootPrivate
{
  /// \brief Callback for pos rendering operations.
  public: void PerformPostRenderingOperations();

  /// \brief Save a pitcture with the camera from the given pose.
  public: void SavePicture (const gz::rendering::CameraPtr _camera,
                    const gz::math::Pose3d &_pose,
                    const std::string &_fileName) const;

  /// \brief Name of the loaded model.
  public: std::string modelName;

  /// \brief model
  public: std::shared_ptr<gz::sim::Model> model;

  /// \brief model world pose
  public: gz::math::Pose3d modelPose3D;

  /// \brief Connection to pre-render event callback.
  public: gz::common::ConnectionPtr connection{nullptr};

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
void ModelPhotoShoot::Configure(const gz::sim::Entity &_entity,
                                const std::shared_ptr<const sdf::Element> &_sdf,
                                gz::sim::EntityComponentManager &_ecm,
                                gz::sim::EventManager &_eventMgr)
{
  std::string saveDataLocation =
      _sdf->Get<std::string>("translation_data_file");
  if (saveDataLocation.empty())
  {
    gzdbg << "No data location specified, skipping translaiton data"
              "saving.\n";
  }
  else
  {
    gzdbg << "Saving translation data to: "
        << saveDataLocation << std::endl;
    this->dataPtr->savingFile.open(saveDataLocation);
  }

  if (_sdf->HasElement("random_joints_pose"))
  {
    this->dataPtr->randomPoses = _sdf->Get<bool>("random_joints_pose");
  }

  this->dataPtr->connection =
      _eventMgr.Connect<gz::sim::events::PostRender>(std::bind(
          &ModelPhotoShootPrivate::PerformPostRenderingOperations,
          this->dataPtr.get()));

  this->dataPtr->model = std::make_shared<gz::sim::Model>(_entity);
  this->dataPtr->modelName = this->dataPtr->model->Name(_ecm);
  // Get the pose of the model
  this->dataPtr->modelPose3D =
      gz::sim::worldPose(this->dataPtr->model->Entity(), _ecm);
}

//////////////////////////////////////////////////
void ModelPhotoShoot::PreUpdate(
    const gz::sim::UpdateInfo &,
    gz::sim::EntityComponentManager &_ecm)
{
  if (this->dataPtr->randomPoses)
  {
    std::vector<sim::Entity> joints = this->dataPtr->model->Joints(_ecm);
    unsigned seed =
        std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    for (const auto &joint : joints)
    {
      auto jointNameComp = _ecm.Component<components::Name>(joint);
      if (jointNameComp)
      {
        auto jointType = _ecm.Component<components::JointType>(joint)->Data();
        if (jointType != sdf::JointType::FIXED)
        {
          if (jointType == sdf::JointType::REVOLUTE  ||
              jointType == sdf::JointType::PRISMATIC)
          {
            // Using the JointAxis component to extract the joint pose limits
            auto jointAxisComp = _ecm.Component<components::JointAxis>(joint);
            if (jointAxisComp)
            {
              std::uniform_real_distribution<double> distribution(
                  jointAxisComp->Data().Lower(),
                  jointAxisComp->Data().Upper());
              double jointPose = distribution(generator);
              _ecm.SetComponentData<components::JointPositionReset>(
                  joint, {jointPose});

              // Create a JointPosition component if it doesn't exist.
              if (nullptr == _ecm.Component<components::JointPosition>(joint))
              {
                _ecm.CreateComponent(joint, components::JointPosition());
                _ecm.SetComponentData<components::JointPosition>(
                    joint, {jointPose});
              }

              if (this->dataPtr->savingFile.is_open())
              {
                this->dataPtr->savingFile << jointNameComp->Data() << ": "
                                          << std::setprecision(17)
                                          << jointPose << std::endl;
              }
            }
            else
            {
              gzerr << "No jointAxisComp found, ignoring joint: " <<
                  jointNameComp->Data() << std::endl;
            }
          }
          else
          {
            gzerr << "Model Photo Shoot only supports single axis joints. "
                "Skipping joint: "<< jointNameComp->Data() << std::endl;
          }
        }
        else
        {
          gzdbg << "Ignoring fixed joint: " << jointNameComp->Data() <<
              std::endl;
        }
      }
      else
      {
          gzerr << "No jointNameComp found on entity: " << joint <<
              std:: endl;
      }
    }
    // Only set random joint poses once
    this->dataPtr->randomPoses = false;
  }
}

//////////////////////////////////////////////////
void ModelPhotoShootPrivate::PerformPostRenderingOperations()
{
  gz::rendering::ScenePtr scene =
      gz::rendering::sceneFromFirstRenderEngine();
  gz::rendering::VisualPtr modelVisual =
      scene->VisualByName(this->modelName);

  gz::rendering::VisualPtr root = scene->RootVisual();

  if (modelVisual && this->takePicture)
  {
    scene->SetAmbientLight(0.3, 0.3, 0.3);

    // create directional light
    gz::rendering::DirectionalLightPtr light0 =
        scene->CreateDirectionalLight();
    light0->SetDirection(-0.5, 0.5, -1);
    light0->SetDiffuseColor(0.8, 0.8, 0.8);
    light0->SetSpecularColor(0.5, 0.5, 0.5);
    root->AddChild(light0);

    // create point light
    gz::rendering::PointLightPtr light2 = scene->CreatePointLight();
    light2->SetDiffuseColor(0.5, 0.5, 0.5);
    light2->SetSpecularColor(0.5, 0.5, 0.5);
    light2->SetLocalPosition(3, 5, 5);
    root->AddChild(light2);

    for (unsigned int i = 0; i < scene->NodeCount(); ++i)
    {
      auto camera = std::dynamic_pointer_cast<gz::rendering::Camera>(
          scene->NodeByIndex(i));
      if (nullptr != camera && camera->Name() == "photo_shoot::link::camera")
      {
        // Compute the translation we have to apply to the cameras to
        // center the model in the image.
        gz::math::AxisAlignedBox bbox = modelVisual->LocalBoundingBox();
        double scaling = 1.0 / bbox.Size().Max();
        gz::math::Vector3d bboxCenter = bbox.Center();
        gz::math::Vector3d translation =
            bboxCenter + this->modelPose3D.Pos();
        if (this->savingFile.is_open()) {
          this->savingFile << "Translation: " << translation << std::endl;
          this->savingFile << "Scaling: " << scaling << std::endl;
        }

        gz::math::Pose3d pose;
        // Perspective view
        pose.Pos().Set(1.6 / scaling + translation.X(),
                       -1.6 / scaling + translation.Y(),
                       1.2 / scaling + translation.Z());
        pose.Rot().SetFromEuler(0, GZ_DTOR(30), GZ_DTOR(-225));
        SavePicture(camera, pose, "1.png");

        // Top view
        pose.Pos().Set(0 + translation.X(),
                       0 + translation.Y(),
                       2.2 / scaling + translation.Z());
        pose.Rot().SetFromEuler(0, GZ_DTOR(90), 0);
        SavePicture(camera, pose, "2.png");

        // Front view
        pose.Pos().Set(2.2 / scaling + translation.X(),
                       0 + translation.Y(),
                       0 + translation.Z());
        pose.Rot().SetFromEuler(0, 0, GZ_DTOR(-180));
        SavePicture(camera, pose, "3.png");

        // Side view
        pose.Pos().Set(0 + translation.X(),
                       2.2 / scaling + translation.Y(),
                       0 + translation.Z());
        pose.Rot().SetFromEuler(0, 0, GZ_DTOR(-90));
        SavePicture(camera, pose, "4.png");

        // Back view
        pose.Pos().Set(-2.2 / scaling + translation.X(),
                       0 + translation.Y(),
                       0 + translation.Z());
        pose.Rot().SetFromEuler(0, 0, 0);
        SavePicture(camera, pose, "5.png");

        this->takePicture = false;
      }
    }
  }
}

//////////////////////////////////////////////////
void ModelPhotoShootPrivate::SavePicture(
                                  const gz::rendering::CameraPtr _camera,
                                  const gz::math::Pose3d &_pose,
                                  const std::string &_fileName) const
{
  unsigned int width = _camera->ImageWidth();
  unsigned int height = _camera->ImageHeight();
  gz::common::Image image;

  _camera->SetWorldPose(_pose);
  auto cameraImage = _camera->CreateImage();
  _camera->Capture(cameraImage);
  auto formatStr =
      gz::rendering::PixelUtil::Name(_camera->ImageFormat());
  auto format = gz::common::Image::ConvertPixelFormat(formatStr);
  image.SetFromData(cameraImage.Data<unsigned char>(), width, height, format);
  image.SavePNG(_fileName);

  gzdbg << "Saved image to [" << _fileName << "]" << std::endl;
}

GZ_ADD_PLUGIN(ModelPhotoShoot, gz::sim::System,
                    ModelPhotoShoot::ISystemConfigure,
                    ModelPhotoShoot::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(ModelPhotoShoot,
                          "gz::sim::systems::ModelPhotoShoot")
