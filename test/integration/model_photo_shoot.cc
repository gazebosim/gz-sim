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

#include <gtest/gtest.h>

#include <stdio.h>
#include <fstream>

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"
#include "ignition/gazebo/rendering/Events.hh"

#include <ignition/common/Image.hh>
#include <ignition/rendering/Camera.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/Visual.hh>

#include "helpers/UniqueTestDirectoryEnv.hh"
#include "plugins/MockSystem.hh"
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
void testImages(const std::string &_filename,
                const std::string &_testFilename)
{
  ignition::common::Image image(std::string(PROJECT_BINARY_PATH) +
      "/test/integration/" + _filename);
  ignition::common::Image testImage(std::string(PROJECT_BINARY_PATH) +
      "/test/integration/" + _testFilename);
  EXPECT_TRUE(image.Valid());
  EXPECT_TRUE(testImage.Valid());
  EXPECT_EQ(image.Width(), testImage.Width());
  EXPECT_EQ(image.Height(), testImage.Height());
  EXPECT_EQ(image.PixelFormat(), testImage.PixelFormat());
  unsigned int dataLength;
  unsigned char *imageData;
  image.Data(&imageData, dataLength);
  unsigned int testDataLenght;
  unsigned char *testImageData;
  image.Data(&testImageData, testDataLenght);
  ASSERT_EQ(dataLength, testDataLenght);
  ASSERT_EQ(memcmp(imageData, testImageData, dataLength), 0);

  // Deleting files so they do not affect future tests
  EXPECT_EQ(remove(_filename.c_str()), 0);
  EXPECT_EQ(remove(_testFilename.c_str()), 0);
}

/// \brief Test ModelPhotoShootTest system.
class ModelPhotoShootTest : public InternalFixture<::testing::Test>
{
  /// \brief PostRender callback.
  public: void OnPostRender()
  {
    if (!testPicturesTaken)
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
      testPicturesTaken = true;
    }
  }

  /// \brief Loads the pose values generated by the Model Photo Shoot plugin.
  /// \param[in] _poseFile File containing the generated poses.
  protected: void LoadPoseValues(std::string _poseFile = "poses.txt")
  {
    std::ifstream poseFile (_poseFile);
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
        if (word == "Scaling:")
        {
          getline( iss, word, ' ' );
          this->scaling = std::stod(word);
          break;
        }
      }
    }
    poseFile.close();
    EXPECT_EQ(remove(_poseFile.c_str()), 0);
  }

  // Documentation inherited
  protected: void SetUp() override
  {
    InternalFixture::SetUp();

    auto plugin = sm.LoadPlugin("libMockSystem.so",
                                "ignition::gazebo::MockSystem",
                                nullptr);
    EXPECT_TRUE(plugin.has_value());
    this->systemPtr = plugin.value();
    this->mockSystem = static_cast<gazebo::MockSystem *>(
        systemPtr->QueryInterface<gazebo::System>());
  }

  /// \brief Tests the Model Photo Shoot plugin with a given sdf world.
  /// \param[in] _sdfWorld SDF World to use for the test.
  protected: void modelPhotoShootTestCmd(const std::string _sdfWorld)
  {
    // First run of the server generating images through the plugin.
    this->serverConfig.SetResourceCache(test::UniqueTestDirectoryEnv::Path());
    this->serverConfig.SetSdfFile(
        common::joinPaths(PROJECT_SOURCE_PATH, _sdfWorld));

    this->server = std::make_unique<Server>(this->serverConfig);
    EXPECT_FALSE(this->server->Running());
    EXPECT_FALSE(*this->server->Running(0));

    this->server->SetUpdatePeriod(1ns);
    this->server->Run(true, 50, false);

    common::ConnectionPtr postRenderConn;

    // A pointer to the ecm. This will be valid once we run the mock system.
    gazebo::EntityComponentManager *ecm = nullptr;
    this->mockSystem->preUpdateCallback =
      [&ecm](const gazebo::UpdateInfo &, gazebo::EntityComponentManager &_ecm)
      {
        ecm = &_ecm;
      };
    this->mockSystem->configureCallback =
      [&](const gazebo::Entity &,
            const std::shared_ptr<const sdf::Element> &,
            gazebo::EntityComponentManager &,
            gazebo::EventManager &_eventMgr)
      {
        postRenderConn = _eventMgr.Connect<gazebo::events::PostRender>(
            std::bind(&ModelPhotoShootTest::OnPostRender, this));
      };
    this->LoadPoseValues();
    this->server->AddSystem(this->systemPtr);

    while (!testPicturesTaken)
    {
      this->server->Run(true, 1, false);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    testImages("1.png", "1_test.png");
    testImages("2.png", "2_test.png");
    testImages("3.png", "3_test.png");
    testImages("4.png", "4_test.png");
    testImages("5.png", "5_test.png");
  }

  public: ServerConfig serverConfig;
  public: std::unique_ptr<Server> server;
  public: ignition::gazebo::SystemPluginPtr systemPtr;
  public: gazebo::MockSystem *mockSystem;

  private: gazebo::SystemLoader sm;
  private: bool testPicturesTaken{false};
  private: double scaling;
  private: ignition::math::Vector3d translation;
};

// Test the Model Photo Shoot plugin on the example world.
TEST_F(ModelPhotoShootTest, ModelPhotoShootExampleWorld)
{
  this->modelPhotoShootTestCmd("examples/worlds/model_photo_shoot.sdf");
}
