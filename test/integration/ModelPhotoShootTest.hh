/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the \"License\");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an \"AS IS\" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef IGNITION_GAZEBO_TEST_INTEGRATION_MODELPHOTOSHOOTTEST_HH_
#define IGNITION_GAZEBO_TEST_INTEGRATION_MODELPHOTOSHOOTTEST_HH_

#include <gtest/gtest.h>

#include <stdio.h>
#include <string>
#include <fstream>
#include <map>
#include <memory>
#include <vector>

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/JointAxis.hh"
#include "ignition/gazebo/components/JointType.hh"
#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"
#include "ignition/gazebo/TestFixture.hh"
#include "ignition/gazebo/rendering/Events.hh"
#include "ignition/gazebo/Model.hh"

#include <ignition/common/Image.hh>
#include <ignition/rendering/Camera.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/Visual.hh>


#include "helpers/UniqueTestDirectoryEnv.hh"
#include "helpers/EnvTestFixture.hh"

using namespace ignition;
using namespace gazebo;
using namespace std::chrono_literals;

/// \brief Saves an image from a camera in a given position.
/// \param[in] _camera Camera to use for the picture.
/// \param[in] _pose Pose for the camera.
/// \param[in] _fileName Filename to save the image.
void SavePicture(const ignition::rendering::CameraPtr _camera,
                const ignition::math::Pose3d &_pose,
                const std::string &_fileName)
{
  unsigned int width = _camera->ImageWidth();
  unsigned int height = _camera->ImageHeight();
  ignition::common::Image image;

  _camera->SetWorldPose(_pose);
  auto cameraImage = _camera->CreateImage();
  _camera->Capture(cameraImage);
  auto formatStr =
      ignition::rendering::PixelUtil::Name(_camera->ImageFormat());
  auto format = ignition::common::Image::ConvertPixelFormat(formatStr);
  image.SetFromData(cameraImage.Data<unsigned char>(), width, height, format);
  image.SavePNG(_fileName);
}

/// \brief Tests that two png files have the same values.
/// \param[in] _filename First png file.
/// \param[in] _testFilename Second png file.
void testImages(const std::string &_imageFile,
                const std::string &_testImageFile)
{
  std::string imageFilePath = common::joinPaths(common::cwd(), _imageFile);
  ignition::common::Image image(imageFilePath);
  std::string testImageFilePath =
      common::joinPaths(common::cwd(), _testImageFile);
  ignition::common::Image testImage(testImageFilePath);

  EXPECT_TRUE(image.Valid());
  EXPECT_TRUE(testImage.Valid());
  EXPECT_EQ(image.Width(), testImage.Width());
  EXPECT_EQ(image.Height(), testImage.Height());
  EXPECT_EQ(image.PixelFormat(), testImage.PixelFormat());
  unsigned int dataLength;
  unsigned char *imageData = nullptr;
  image.Data(&imageData, dataLength);
  unsigned int testDataLenght;
  unsigned char *testImageData = nullptr;
  image.Data(&testImageData, testDataLenght);
  ASSERT_EQ(dataLength, testDataLenght);
  ASSERT_EQ(memcmp(imageData, testImageData, dataLength), 0);

  // Deleting files so they do not affect future tests
  EXPECT_EQ(remove(imageFilePath.c_str()), 0);
  EXPECT_EQ(remove(testImageFilePath.c_str()), 0);
}

/// \brief Test ModelPhotoShootTest system.
class ModelPhotoShootTest : public InternalFixture<::testing::Test>
{
  protected: void SetUp() override
  {
    EXPECT_TRUE(common::chdir(test::UniqueTestDirectoryEnv::Path()));
    InternalFixture<::testing::Test>::SetUp();
  }
  /// \brief PostRender callback.
  public: void OnPostRender()
  {
    if (takeTestPics)
    {
      ignition::rendering::ScenePtr scene =
        ignition::rendering::sceneFromFirstRenderEngine();
      for (unsigned int i = 0; i < scene->NodeCount(); ++i)
      {
        auto camera = std::dynamic_pointer_cast<ignition::rendering::Camera>(
            scene->NodeByIndex(i));
        if (nullptr != camera && camera->Name() == "photo_shoot::link::camera")
        {
          ignition::math::Pose3d pose;
          // Perspective view
          pose.Pos().Set(1.6 / scaling + translation.X(),
                        -1.6 / scaling + translation.Y(),
                        1.2 / scaling + translation.Z());
          pose.Rot().Euler(0, IGN_DTOR(30), IGN_DTOR(-225));
          SavePicture(camera, pose, "1_test.png");
          // Top view
          pose.Pos().Set(0 + translation.X(),
                        0 + translation.Y(),
                        2.2 / scaling + translation.Z());
          pose.Rot().Euler(0, IGN_DTOR(90), 0);
          SavePicture(camera, pose, "2_test.png");

          // Front view
          pose.Pos().Set(2.2 / scaling + translation.X(),
                        0 + translation.Y(),
                        0 + translation.Z());
          pose.Rot().Euler(0, 0, IGN_DTOR(-180));
          SavePicture(camera, pose, "3_test.png");

          // Side view
          pose.Pos().Set(0 + translation.X(),
                        2.2 / scaling + translation.Y(),
                        0 + translation.Z());
          pose.Rot().Euler(0, 0, IGN_DTOR(-90));
          SavePicture(camera, pose, "4_test.png");

          // Back view
          pose.Pos().Set(-2.2 / scaling + translation.X(),
                        0 + translation.Y(),
                        0 + translation.Z());
          pose.Rot().Euler(0, 0, 0);
          SavePicture(camera, pose, "5_test.png");
        }
      }
      takeTestPics = false;
    }
  }

  /// \brief Loads the pose values generated by the Model Photo Shoot plugin.
  /// \param[in] _poseFile File containing the generated poses.
  protected: void LoadPoseValues(std::string _poseFile = "poses.txt")
  {
    std::string poseFilePath = common::joinPaths(common::cwd(), _poseFile);
    std::ifstream poseFile (poseFilePath);
    std::string line;
    ASSERT_TRUE(poseFile.is_open());
    while (getline(poseFile, line) )
    {
      std::istringstream iss(line);
      std::string word;
      while (getline( iss, word, ' ' ))
      {
        if (word == "Translation:")
        {
          float tr_x, tr_y, tr_z;
          getline( iss, word, ' ' );
          tr_x = std::stof(word);
          getline( iss, word, ' ' );
          tr_y = std::stof(word);
          getline( iss, word, ' ' );
          tr_z = std::stof(word);
          this->translation = {tr_x, tr_y, tr_z};
          break;
        }
        else
        {
          if (word == "Scaling:")
          {
            getline( iss, word, ' ' );
            this->scaling = std::stod(word);
            break;
          }
          else
          {
            std::string jointName = line.substr(0, line.find(": "));
            std::string jointPose = line.substr(line.find(": ")+2);
            jointPositions[jointName] = std::stod(jointPose);
          }
        }
      }
    }
    poseFile.close();
    EXPECT_EQ(remove(poseFilePath.c_str()), 0);
  }

  /// \brief Tests the Model Photo Shoot plugin with a given sdf world.
  /// \param[in] _sdfWorld SDF World to use for the test.
  protected: void ModelPhotoShootTestCmd(const std::string _sdfWorld)
  {
    // First run of the server generating images through the plugin.
    TestFixture fixture(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
        _sdfWorld));
    fixture.Server()->SetUpdatePeriod(1ns);

    common::ConnectionPtr postRenderConn;
    fixture.OnConfigure([&](
      const Entity &,
      const std::shared_ptr<const sdf::Element> &,
      EntityComponentManager &,
      EventManager &_eventMgr)
    {
      postRenderConn = _eventMgr.Connect<gazebo::events::PostRender>(
            std::bind(&ModelPhotoShootTest::OnPostRender, this));
    }).Finalize();

    fixture.Server()->Run(true, 50, false);
    this->LoadPoseValues();

    fixture.OnPreUpdate([&](const gazebo::UpdateInfo &,
                            gazebo::EntityComponentManager &_ecm)
    {
      if(!jointPositions.empty() && this->checkRandomJoints)
      {
        _ecm.Each<components::Model>(
        [&](const ignition::gazebo::Entity &_entity,
            const components::Model *) -> bool
        {
          auto modelName = _ecm.Component<components::Name>(_entity);
          if (modelName->Data() == "r2")
          {
            this->model = std::make_shared<ignition::gazebo::Model>(_entity);
          }
          return true;
        });
        std::vector<gazebo::Entity> joints = this->model->Joints(_ecm);
        for (const auto &joint : joints)
        {
          auto jointNameComp = _ecm.Component<components::Name>(joint);
          std::map<std::string, double>::iterator it =
              jointPositions.find(jointNameComp->Data());
          if(it != jointPositions.end())
          {
            auto jointType = _ecm.Component<components::JointType>
                (joint)->Data();
            ASSERT_TRUE(jointType == sdf::JointType::REVOLUTE ||
                  jointType == sdf::JointType::PRISMATIC);
            auto jointAxis = _ecm.Component<components::JointAxis>(joint);
            ASSERT_GE(it->second, jointAxis->Data().Lower());
            ASSERT_LE(it->second, jointAxis->Data().Upper());
            auto jointPosition =
                _ecm.Component<components::JointPosition>(joint);
            ASSERT_NE(jointPosition, nullptr);
            ASSERT_DOUBLE_EQ(jointPosition->Data()[0], it->second);
          }
        }
        this->checkRandomJoints = false;
      }
    }).Finalize();

    this->takeTestPics = true;

    const auto end_time = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(3000);
    while (takeTestPics && end_time > std::chrono::steady_clock::now())
    {
      fixture.Server()->Run(true, 1, false);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    testImages("1.png", "1_test.png");
    testImages("2.png", "2_test.png");
    testImages("3.png", "3_test.png");
    testImages("4.png", "4_test.png");
    testImages("5.png", "5_test.png");
  }

  private: bool takeTestPics{false};
  private: bool checkRandomJoints{true};
  private: double scaling;
  private: ignition::math::Vector3d translation;
  private: std::map<std::string, double> jointPositions;
  private: std::shared_ptr<ignition::gazebo::Model> model;
};

#endif
