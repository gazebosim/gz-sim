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

#ifndef GZ_SIM_TEST_INTEGRATION_MODELPHOTOSHOOTTEST_HH_
#define GZ_SIM_TEST_INTEGRATION_MODELPHOTOSHOOTTEST_HH_

#include <gtest/gtest.h>

#include <stdio.h>
#include <string>
#include <fstream>
#include <map>
#include <memory>
#include <vector>

#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/JointAxis.hh"
#include "gz/sim/components/JointType.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/TestFixture.hh"
#include "gz/sim/rendering/Events.hh"
#include "gz/sim/Model.hh"

#include <gz/common/Image.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>


#include "helpers/UniqueTestDirectoryEnv.hh"
#include "helpers/EnvTestFixture.hh"

#include "test_config.hh"

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

/// \brief Saves an image from a camera in a given position.
/// \param[in] _camera Camera to use for the picture.
/// \param[in] _pose Pose for the camera.
/// \param[in] _fileName Filename to save the image.
void SavePicture(const gz::rendering::CameraPtr _camera,
                const gz::math::Pose3d &_pose,
                const std::string &_fileName)
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
}

/// \brief Tests that two png files have the same values.
/// \param[in] _filename First png file.
/// \param[in] _testFilename Second png file.
void testImages(const std::string &_imageFile,
                const std::string &_testImageFile)
{
  std::string imageFilePath = common::joinPaths(common::cwd(), _imageFile);
  gz::common::Image image(imageFilePath);
  std::string testImageFilePath =
      common::joinPaths(common::cwd(), _testImageFile);
  gz::common::Image testImage(testImageFilePath);

  EXPECT_TRUE(image.Valid());
  EXPECT_TRUE(testImage.Valid());
  EXPECT_EQ(image.Width(), testImage.Width());
  EXPECT_EQ(image.Height(), testImage.Height());
  EXPECT_EQ(image.PixelFormat(), testImage.PixelFormat());
  // Images should be almost equal (They might have
  // minimal color differences on a few pixels)
  unsigned int equalPixels = 0;
  unsigned int totalPixels = testImage.Width() * testImage.Height();
  for (unsigned int x = 0; x < image.Width(); x++)
  {
    for (unsigned int y = 0; y < image.Height(); y++)
    {
      if (image.Pixel(x, y) == testImage.Pixel(x, y))
      {
        equalPixels++;
      }
    }
  }
  ASSERT_GT((float)equalPixels/(float)totalPixels, 0.99);

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
      gz::rendering::ScenePtr scene =
        gz::rendering::sceneFromFirstRenderEngine();
      for (unsigned int i = 0; i < scene->NodeCount(); ++i)
      {
        auto camera = std::dynamic_pointer_cast<gz::rendering::Camera>(
            scene->NodeByIndex(i));
        if (nullptr != camera && camera->Name() == "photo_shoot::link::camera")
        {
          gz::math::Pose3d pose;
          // Perspective view
          pose.Pos().Set(1.6 / scaling + translation.X(),
                        -1.6 / scaling + translation.Y(),
                        1.2 / scaling + translation.Z());
          pose.Rot().SetFromEuler(0, GZ_DTOR(30), GZ_DTOR(-225));
          SavePicture(camera, pose, "1_test.png");
          // Top view
          pose.Pos().Set(0 + translation.X(),
                        0 + translation.Y(),
                        2.2 / scaling + translation.Z());
          pose.Rot().SetFromEuler(0, GZ_DTOR(90), 0);
          SavePicture(camera, pose, "2_test.png");

          // Front view
          pose.Pos().Set(2.2 / scaling + translation.X(),
                        0 + translation.Y(),
                        0 + translation.Z());
          pose.Rot().SetFromEuler(0, 0, GZ_DTOR(-180));
          SavePicture(camera, pose, "3_test.png");

          // Side view
          pose.Pos().Set(0 + translation.X(),
                        2.2 / scaling + translation.Y(),
                        0 + translation.Z());
          pose.Rot().SetFromEuler(0, 0, GZ_DTOR(-90));
          SavePicture(camera, pose, "4_test.png");

          // Back view
          pose.Pos().Set(-2.2 / scaling + translation.X(),
                        0 + translation.Y(),
                        0 + translation.Z());
          pose.Rot().SetFromEuler(0, 0, 0);
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

    common::ConnectionPtr postRenderConn;
    fixture.OnConfigure([&](
      const Entity &,
      const std::shared_ptr<const sdf::Element> &,
      EntityComponentManager &,
      EventManager &_eventMgr)
    {
      postRenderConn = _eventMgr.Connect<sim::events::PostRender>(
            std::bind(&ModelPhotoShootTest::OnPostRender, this));
    }).Finalize();

    fixture.Server()->SetUpdatePeriod(1ns);

    for (int i = 0; i < 50; ++i)
    {
      fixture.Server()->RunOnce(true);
    }
    this->LoadPoseValues();

    fixture.OnPreUpdate([&](const sim::UpdateInfo &,
                            sim::EntityComponentManager &_ecm)
    {
      if(!jointPositions.empty() && this->checkRandomJoints)
      {
        _ecm.Each<components::Model>(
        [&](const gz::sim::Entity &_entity,
            const components::Model *) -> bool
        {
          auto modelName = _ecm.Component<components::Name>(_entity);
          if (modelName->Data() == "r2")
          {
            this->model = std::make_shared<gz::sim::Model>(_entity);
          }
          return true;
        });
        std::vector<sim::Entity> joints = this->model->Joints(_ecm);
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
    });

    this->takeTestPics = true;

    const auto end_time = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(3000);
    while (takeTestPics && end_time > std::chrono::steady_clock::now())
    {
      fixture.Server()->RunOnce(true);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    testImages("1.png", "1_test.png");
    testImages("2.png", "2_test.png");
    testImages("3.png", "3_test.png");
    testImages("4.png", "4_test.png");
    testImages("5.png", "5_test.png");

    postRenderConn.reset();
  }

  private: bool takeTestPics{false};
  private: bool checkRandomJoints{true};
  private: double scaling;
  private: gz::math::Vector3d translation;
  private: std::map<std::string, double> jointPositions;
  private: std::shared_ptr<gz::sim::Model> model;
};

#endif
