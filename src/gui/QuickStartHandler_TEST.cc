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

#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryFile>
#include <QVariantList>
#include <QVariantMap>

#include <string>

#include "QuickStartHandler.hh"
#include "test_config.hh"

using namespace gz;
using namespace sim::gui;

/////////////////////////////////////////////////
TEST(QuickStartHandlerTest, DefaultCatalogLoaded)
{
  QuickStartHandler handler;
  const QVariantList categories = handler.Categories();
  const QVariantList allWorlds = handler.Worlds();

  // The loader skips duplicate ids, unknown categories and worlds that
  // cannot be resolved, so every entry of the bundled catalog must load
  QFile catalog(":/Gazebo/quickstart_worlds.json");
  ASSERT_TRUE(catalog.open(QIODevice::ReadOnly));
  const QJsonObject raw = QJsonDocument::fromJson(catalog.readAll()).object();
  EXPECT_EQ(raw.value("categories").toArray().size(), categories.size());
  EXPECT_EQ(raw.value("worlds").toArray().size(), allWorlds.size());

  const QVariantList featured = handler.FeaturedWorlds();
  EXPECT_EQ(featured.size(), 6);

  for (const QVariant &catVar : categories)
  {
    const QVariantMap cat = catVar.toMap();
    EXPECT_FALSE(cat.value("name").toString().isEmpty());
    EXPECT_EQ(cat.value("count").toInt(),
              handler.Worlds(cat.value("id").toString()).size());
  }

  const QDir examplesDir(
      QString::fromStdString(std::string(PROJECT_SOURCE_PATH) +
                             "/examples/worlds"));
  for (const QVariant &worldVar : allWorlds)
  {
    const QVariantMap world = worldVar.toMap();
    if (world.value("source").toString() != "local")
      continue;

    const std::string id = world.value("id").toString().toStdString();
    const QString uri = handler.ResolveWorldUri(world);
    QFile sdf(examplesDir.filePath(uri));
    EXPECT_TRUE(sdf.exists())
        << "Local example world missing: " << uri.toStdString();
    // Worlds tagged as offline must not download models from Fuel
    if (world.value("tags").toStringList().contains("offline") &&
        sdf.open(QIODevice::ReadOnly))
    {
      EXPECT_FALSE(sdf.readAll().contains("fuel.gazebosim.org"))
          << "Offline world [" << id << "] downloads from Fuel";
    }
  }
}

/////////////////////////////////////////////////
TEST(QuickStartHandlerTest, LoadCatalogSkipsInvalidEntries)
{
  QuickStartHandler handler;
  QTemporaryFile file;
  ASSERT_TRUE(file.open());
  file.write(R"({"categories": [{"id": "Mobile"}, {"id": "mobile"}, {},
                                {"id": "all"}, {"id": "featured"}],
    "worlds": [
      {"id": "a", "title": "A", "category": "MOBILE", "source": "Local",
       "fileName": "a.sdf", "featured": true},
      {"id": "e", "title": "E", "category": "featured", "source": "local",
       "fileName": "e.sdf"},
      {"id": "a", "title": "Dup", "category": "mobile", "source": "local",
       "fileName": "b.sdf"},
      {"id": "b", "title": "B", "category": "x", "source": "local",
       "fileName": "b.sdf"},
      {"id": "c", "title": "C", "category": "mobile", "source": "fuel"},
      {"id": "d", "category": "mobile", "source": "local", "fileName": "d"},
      42]})");
  file.flush();
  ASSERT_TRUE(handler.LoadCatalog(file.fileName()));
  EXPECT_EQ(2, handler.Worlds().size());
  EXPECT_EQ(1, handler.Worlds("Mobile", "LOCAL").size());
  // "e" is featured because of its category, "a" because of its flag
  EXPECT_EQ(2, handler.FeaturedWorlds().size());

  const QVariantList categories = handler.Categories();
  ASSERT_EQ(2, categories.size());
  EXPECT_EQ(1, categories[0].toMap().value("count").toInt());
  EXPECT_EQ(2, categories[1].toMap().value("count").toInt());
}

/////////////////////////////////////////////////
TEST(QuickStartHandlerTest, FilterByCategorySourceAndQuery)
{
  QuickStartHandler handler;

  const QVariantList mobileWorlds = handler.Worlds("mobile");
  EXPECT_GT(mobileWorlds.size(), 0);
  for (const QVariant &w : mobileWorlds)
  {
    EXPECT_EQ("mobile", w.toMap().value("category").toString());
  }

  const QVariantList localOnly = handler.Worlds("all", "local");
  EXPECT_GT(localOnly.size(), 0);
  for (const QVariant &w : localOnly)
  {
    EXPECT_EQ("local", w.toMap().value("source").toString());
  }

  const QVariantList fuelOnly = handler.Worlds("all", "fuel");
  EXPECT_GT(fuelOnly.size(), 0);
  for (const QVariant &w : fuelOnly)
  {
    EXPECT_EQ("fuel", w.toMap().value("source").toString());
  }

  const QVariantList pandaMatches = handler.Worlds("all", "all", "panda");
  EXPECT_EQ(1, pandaMatches.size());
  EXPECT_EQ("panda_joint_control",
            pandaMatches.first().toMap().value("id").toString());

  // Case-insensitive tag search
  const QVariantList uavMatches = handler.Worlds("all", "all", "UAV");
  EXPECT_GT(uavMatches.size(), 0);
}

/////////////////////////////////////////////////
TEST(QuickStartHandlerTest, ResolveWorldUriAndInvalidCatalog)
{
  QuickStartHandler handler;

  QVariantMap localWorld;
  localWorld["source"] = "local";
  localWorld["fileName"] = "empty.sdf";
  EXPECT_EQ("empty.sdf", handler.ResolveWorldUri(localWorld));

  QVariantMap fuelWorld;
  fuelWorld["source"] = "fuel";
  fuelWorld["owner"] = "OpenRobotics";
  fuelWorld["fuelName"] = "Quadcopter teleop";
  EXPECT_EQ("https://fuel.gazebosim.org/1.0/OpenRobotics/worlds/"
            "Quadcopter teleop",
            handler.ResolveWorldUri(fuelWorld));

  QVariantMap brokenFuel;
  brokenFuel["source"] = "fuel";
  brokenFuel["owner"] = "OpenRobotics";
  EXPECT_TRUE(handler.ResolveWorldUri(brokenFuel).isEmpty());

  // Loading an invalid file should fail without corrupting existing catalog
  const int prevCount = handler.Worlds().size();
  EXPECT_FALSE(handler.LoadCatalog("/nonexistent/path/catalog.json"));
  EXPECT_EQ(prevCount, handler.Worlds().size());

  QTemporaryFile badJson;
  ASSERT_TRUE(badJson.open());
  badJson.write("{ not valid json");
  badJson.flush();
  EXPECT_FALSE(handler.LoadCatalog(badJson.fileName()));
  EXPECT_EQ(prevCount, handler.Worlds().size());

  QTemporaryFile noValidWorlds;
  ASSERT_TRUE(noValidWorlds.open());
  noValidWorlds.write(R"({"categories": [], "worlds": [{"id": "a"}]})");
  noValidWorlds.flush();
  EXPECT_FALSE(handler.LoadCatalog(noValidWorlds.fileName()));
  EXPECT_EQ(prevCount, handler.Worlds().size());
}
