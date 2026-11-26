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

/////////////////////////////////////////////////
TEST(ResourceSpawnerTest,
    GZ_UTILS_TEST_ENABLED_ONLY_ON_LINUX(FavoritesAndRecents))
{
  auto app = std::make_unique<gui::Application>(g_argc, g_argv);
  ASSERT_NE(nullptr, app);

  ResourceSpawner spawner;
  const QString owner = "test_owner_unit";
  const QString name = "test_model_unit";

  const bool initialFav = spawner.IsFavorite(owner, name);
  spawner.ToggleFavorite(owner, name);
  EXPECT_NE(initialFav, spawner.IsFavorite(owner, name));
  // Restore original state so QSettings stays clean.
  spawner.ToggleFavorite(owner, name);
  EXPECT_EQ(initialFav, spawner.IsFavorite(owner, name));
}

/////////////////////////////////////////////////
TEST(ResourceSpawnerTest, InferCategory)
{
  EXPECT_EQ("drone", ResourceSpawner::InferCategory("X3 UAV"));
  EXPECT_EQ("robot", ResourceSpawner::InferCategory("Clearpath Husky"));
  EXPECT_EQ("sensor", ResourceSpawner::InferCategory(
      "MyUnit", "Depth camera module"));
  EXPECT_EQ("vehicle", ResourceSpawner::InferCategory("Prius Hybrid"));
  EXPECT_EQ("nature", ResourceSpawner::InferCategory("Oak tree"));
  EXPECT_EQ("building", ResourceSpawner::InferCategory("Warehouse Depot"));
  EXPECT_EQ("furniture", ResourceSpawner::InferCategory("Picking_Shelves"));
  EXPECT_EQ("light", ResourceSpawner::InferCategory("Street Lamp"));
  EXPECT_EQ("prop", ResourceSpawner::InferCategory("Coke Can"));
  EXPECT_EQ("robot", ResourceSpawner::InferCategory(
      "CustomModel", "", {"ugv", "outdoor"}));
  EXPECT_EQ("generic", ResourceSpawner::InferCategory("FooBarBaz"));
}

/////////////////////////////////////////////////
TEST(ResourceSpawnerTest, FormatFileSize)
{
  EXPECT_EQ("", ResourceSpawner::FormatFileSize(0));
  EXPECT_EQ("512 B", ResourceSpawner::FormatFileSize(512));
  EXPECT_EQ("1.5 KB", ResourceSpawner::FormatFileSize(1536));
  EXPECT_EQ("4.2 MB", ResourceSpawner::FormatFileSize(4404019));
  EXPECT_EQ("2.0 GB", ResourceSpawner::FormatFileSize(2147483648ULL));
}

/////////////////////////////////////////////////
TEST(ResourceSpawnerTest,
    GZ_UTILS_TEST_ENABLED_ONLY_ON_LINUX(PopularSortAndCategorySearch))
{
  auto app = std::make_unique<gui::Application>(g_argc, g_argv);
  ASSERT_NE(nullptr, app);

  ResourceSpawner spawner;

  Resource a;
  a.name = "Alpha";
  a.category = "robot";
  a.downloadCount = 100;

  Resource b;
  b.name = "Beta";
  b.category = "sensor";
  b.downloadCount = 5000;

  Resource c;
  c.name = "Gamma";
  c.category = "robot";
  c.downloadCount = 1200;

  std::vector<Resource> list = {a, b, c};
  spawner.OnSortChosen("Popular");
  spawner.SortResources(list);
  ASSERT_EQ(3u, list.size());
  EXPECT_EQ("Beta", list[0].name);
  EXPECT_EQ("Gamma", list[1].name);
  EXPECT_EQ("Alpha", list[2].name);

  spawner.OnSearchEntered("robot");
  spawner.FilterResources(list);
  ASSERT_EQ(2u, list.size());
  EXPECT_EQ("Gamma", list[0].name);
  EXPECT_EQ("Alpha", list[1].name);
}
