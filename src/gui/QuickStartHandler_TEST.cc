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
#include <QTemporaryFile>
#include <QVariantList>
#include <QVariantMap>

#include <set>
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
  EXPECT_GE(categories.size(), 7);

  const QVariantList allWorlds = handler.Worlds();
  EXPECT_GT(allWorlds.size(), 15);

  const QVariantList featured = handler.FeaturedWorlds();
  EXPECT_EQ(featured.size(), 6);

  // Ensure every category id is unique and has a non-empty name
  std::set<std::string> categoryIds;
  for (const QVariant &catVar : categories)
  {
    const QVariantMap cat = catVar.toMap();
    const std::string id = cat.value("id").toString().toStdString();
    EXPECT_FALSE(id.empty());
    EXPECT_FALSE(cat.value("name").toString().isEmpty());
    EXPECT_TRUE(categoryIds.insert(id).second) << "Duplicate category: " << id;
  }

  // Ensure every world references a valid category and has a valid URI
  std::set<std::string> worldIds;
  const QDir examplesDir(
      QString::fromStdString(std::string(PROJECT_SOURCE_PATH) +
                             "/examples/worlds"));
  for (const QVariant &worldVar : allWorlds)
  {
    const QVariantMap world = worldVar.toMap();
    const std::string id = world.value("id").toString().toStdString();
    EXPECT_FALSE(id.empty());
    EXPECT_TRUE(worldIds.insert(id).second) << "Duplicate world id: " << id;

    const std::string cat = world.value("category").toString().toStdString();
    EXPECT_TRUE(categoryIds.count(cat) > 0)
        << "World [" << id << "] has unknown category [" << cat << "]";

    const QString uri = handler.ResolveWorldUri(world);
    EXPECT_FALSE(uri.isEmpty()) << "World [" << id << "] has empty URI";

    const QString source = world.value("source").toString();
    if (source == "local")
    {
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
    else if (source == "fuel")
    {
      EXPECT_TRUE(uri.startsWith("https://fuel.gazebosim.org/1.0/"))
          << "Invalid Fuel URI: " << uri.toStdString();
    }
    else
    {
      FAIL() << "Unknown world source [" << source.toStdString()
             << "] for world [" << id << "]";
    }
  }
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
}
