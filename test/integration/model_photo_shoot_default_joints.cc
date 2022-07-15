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

#include "ModelPhotoShootTest.hh"

#include <gz/utils/ExtraTestMacros.hh>

// Test the Model Photo Shoot plugin on the example world.
TEST_F(ModelPhotoShootTest,
       GZ_UTILS_TEST_ENABLED_ONLY_ON_LINUX(ModelPhotoShootDefaultJoints))
{
  this->ModelPhotoShootTestCmd(
      "examples/worlds/model_photo_shoot.sdf");
}

int main(int _argc, char **_argv)
{
  ::testing::InitGoogleTest(&_argc, _argv);
  ::testing::AddGlobalTestEnvironment(
      new test::UniqueTestDirectoryEnv("model_photo_shoot_test"));
  return RUN_ALL_TESTS();
}
