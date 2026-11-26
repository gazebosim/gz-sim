/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include <QDir>
#include <QTemporaryDir>
#include <QUrl>

#include <gz/gui/Application.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "ResourceSpawner.hh"

int g_argc = 1;
char *g_argv[] =
{
  reinterpret_cast<char *>(const_cast<char *>("dummy")),
};

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
TEST(ResourceSpawnerTest, BuildFuelThumbnailUrl)
{
  EXPECT_EQ(
      "https://fuel.gazebosim.org/1.0/openrobotics/models/Oak%20tree"
      "/tip/files/thumbnails/1.png",
      ResourceSpawner::BuildFuelThumbnailUrl(
          "https://fuel.gazebosim.org/", "1.0", "openrobotics", "Oak tree"));

  EXPECT_EQ(
      "https://fuel.gazebosim.org/1.0/my%20org/models/My_Model"
      "/tip/files/thumbnails/1.png",
      ResourceSpawner::BuildFuelThumbnailUrl(
          "https://fuel.gazebosim.org", "", "my org", "My_Model"));

  EXPECT_EQ("", ResourceSpawner::BuildFuelThumbnailUrl("", "1.0", "a", "b"));
  EXPECT_EQ("", ResourceSpawner::BuildFuelThumbnailUrl(
      "https://x", "1.0", "", "b"));
  EXPECT_EQ("", ResourceSpawner::BuildFuelThumbnailUrl(
      "https://x", "1.0", "a", ""));
}

/////////////////////////////////////////////////
TEST(ResourceSpawnerTest, GZ_UTILS_TEST_ENABLED_ONLY_ON_LINUX(SetThumbnail))
{
  auto app = std::make_unique<gui::Application>(g_argc, g_argv);
  ASSERT_NE(nullptr, app);

  ResourceSpawner spawner;

  QTemporaryDir tempDir;
  ASSERT_TRUE(tempDir.isValid());
  const std::string thumbDir = tempDir.path().toStdString();

  // Create a file without extension, an uppercase PNG, and a later-sorted png.
  {
    std::ofstream(thumbDir + "/README") << "ignore";
    std::ofstream(thumbDir + "/2.png") << "img2";
    std::ofstream(thumbDir + "/1.PNG") << "img1";
  }

  Resource res;
  spawner.SetThumbnail(thumbDir, res);
  EXPECT_EQ(thumbDir + "/1.PNG", res.thumbnailPath);
  EXPECT_EQ(QUrl::fromLocalFile(QString::fromStdString(res.thumbnailPath))
                .toString().toStdString(),
            res.thumbnailUrl);

  // Non-existent directory leaves resource untouched.
  Resource emptyRes;
  spawner.SetThumbnail(thumbDir + "/missing", emptyRes);
  EXPECT_TRUE(emptyRes.thumbnailPath.empty());
  EXPECT_TRUE(emptyRes.thumbnailUrl.empty());
}

/////////////////////////////////////////////////
TEST(ResourceSpawnerTest,
    GZ_UTILS_TEST_ENABLED_ONLY_ON_LINUX(ResourceModelUpdateByName))
{
  auto app = std::make_unique<gui::Application>(g_argc, g_argv);
  ASSERT_NE(nullptr, app);

  ResourceModel model;
  Resource r1;
  r1.owner = "openrobotics";
  r1.name = "Husky";
  r1.isFuel = true;
  r1.isDownloaded = false;
  r1.isDownloading = false;
  model.AddResource(r1);

  r1.isDownloading = true;
  EXPECT_TRUE(model.UpdateResourceByName("openrobotics", "Husky", r1));
  EXPECT_FALSE(model.UpdateResourceByName("openrobotics", "Missing", r1));

  const auto roles = model.roleNames();
  auto *item = model.invisibleRootItem()->child(0);
  ASSERT_NE(nullptr, item);
  EXPECT_TRUE(item->data(roles.key("isDownloading")).toBool());
  EXPECT_FALSE(item->data(roles.key("isDownloaded")).toBool());
}
