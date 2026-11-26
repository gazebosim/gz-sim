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

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <QCoreApplication>
#include <QDir>
#include <QQmlApplicationEngine>
#include <QQmlContext>
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

namespace
{
/////////////////////////////////////////////////
Resource FuelResource(const std::string &_name, uint32_t _downloadCount)
{
  Resource resource;
  resource.name = _name;
  resource.owner = "alice";
  resource.isFuel = true;
  resource.sdfPath = "https://fuel.gazebosim.org/1.0/alice/models/" + _name;
  resource.downloadCount = _downloadCount;
  return resource;
}

/////////////////////////////////////////////////
std::vector<std::string> Names(const std::vector<Resource> &_resources)
{
  std::vector<std::string> names;
  for (const auto &resource : _resources)
    names.push_back(resource.name);
  return names;
}

/////////////////////////////////////////////////
std::vector<std::string> Names(const ResourceModel &_model)
{
  std::vector<std::string> names;
  const int nameRole = _model.roleNames().key("name");
  for (int row = 0; row < _model.rowCount(); ++row)
  {
    names.push_back(
        _model.item(row)->data(nameRole).toString().toStdString());
  }
  return names;
}
}  // namespace

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
    GZ_UTILS_TEST_ENABLED_ONLY_ON_LINUX(DownloadFailureIsReported))
{
  auto app = std::make_unique<gui::Application>(g_argc, g_argv);
  ASSERT_NE(nullptr, app);

  ResourceSpawner spawner;
  int errors{0};
  QString lastError;
  QObject::connect(&spawner, &ResourceSpawner::resourceSpawnerError,
      [&errors, &lastError](const QString &_msg)
      {
        ++errors;
        lastError = _msg;
      });

  // Process events until the queued download results arrive
  auto waitForErrors = [&errors](int _count)
  {
    for (int i = 0; i < 500 && errors < _count; ++i)
    {
      QCoreApplication::processEvents();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  };

  // An invalid model URL fails before any network access. The second
  // request is ignored because the first one is still in progress.
  spawner.OnDownloadFuelResource("not_a_model_url", "box", "alice", 0, true);
  spawner.OnDownloadFuelResource("not_a_model_url", "box", "alice", 0, true);
  waitForErrors(1);
  EXPECT_EQ(1, errors);
  EXPECT_TRUE(lastError.contains("box")) << lastError.toStdString();

  // Once the download finished, it can be retried
  spawner.OnDownloadFuelResource("not_a_model_url", "box", "alice", 0, true);
  waitForErrors(2);
  EXPECT_EQ(2, errors);
}

/////////////////////////////////////////////////
TEST(ResourceSpawnerTest, GZ_UTILS_TEST_ENABLED_ONLY_ON_LINUX(SortResources))
{
  auto app = std::make_unique<gui::Application>(g_argc, g_argv);
  ASSERT_NE(nullptr, app);

  ResourceSpawner spawner;
  const std::vector<Resource> loaded{FuelResource("b", 5),
      FuelResource("C", 50), FuelResource("a", 10), FuelResource("d", 50)};

  // "Popular" is the default: most downloaded first, and resources with the
  // same download count keep the order in which they were loaded
  auto resources = loaded;
  spawner.SortResources(resources);
  EXPECT_EQ((std::vector<std::string>{"C", "d", "a", "b"}), Names(resources));

  resources = loaded;
  spawner.OnSortChosen("Most Recent");
  spawner.SortResources(resources);
  EXPECT_EQ((std::vector<std::string>{"b", "C", "a", "d"}), Names(resources));

  resources = loaded;
  spawner.OnSortChosen("A - Z");
  spawner.SortResources(resources);
  EXPECT_EQ((std::vector<std::string>{"a", "b", "C", "d"}), Names(resources));

  resources = loaded;
  spawner.OnSortChosen("Z - A");
  spawner.SortResources(resources);
  EXPECT_EQ((std::vector<std::string>{"d", "C", "b", "a"}), Names(resources));
}

/////////////////////////////////////////////////
TEST(ResourceSpawnerTest,
    GZ_UTILS_TEST_ENABLED_ONLY_ON_LINUX(ResourcesLoadedWhileShown))
{
  auto app = std::make_unique<gui::Application>(g_argc, g_argv);
  ASSERT_NE(nullptr, app);

  ResourceSpawner spawner;
  auto *model = qobject_cast<ResourceModel *>(app->Engine()->rootContext()
      ->contextProperty("ResourceList").value<QObject *>());
  ASSERT_NE(nullptr, model);

  // Resources that arrive from Fuel while their owner is shown are inserted
  // in "Popular" order
  spawner.OnOwnerClicked("alice");
  spawner.DisplayResources();
  for (const auto &resource : {FuelResource("office chair", 5),
      FuelResource("table", 50), FuelResource("armchair", 10),
      FuelResource("lamp", 50), FuelResource("chair", 10)})
  {
    spawner.UpdateOwnerListModel(resource);
  }
  EXPECT_EQ((std::vector<std::string>{
      "table", "lamp", "armchair", "chair", "office chair"}), Names(*model));

  // With a search keyword, only the matching ones are inserted
  spawner.OnSearchEntered("chair");
  spawner.DisplayResources();
  spawner.UpdateOwnerListModel(FuelResource("desk", 99));
  spawner.UpdateOwnerListModel(FuelResource("high chair", 20));
  const std::vector<std::string> expected{
      "high chair", "armchair", "chair", "office chair"};
  EXPECT_EQ(expected, Names(*model));

  // Same result as displaying all the loaded resources again
  spawner.DisplayResources();
  EXPECT_EQ(expected, Names(*model));
}
