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
#include <QTemporaryDir>
#include <QTemporaryFile>
#include <QUrl>
#include <QVariantList>
#include <QVariantMap>

#include <ostream>
#include <string>

#include "QuickStartHandler.hh"
#include "test_config.hh"

using namespace gz;
using namespace sim::gui;

QT_BEGIN_NAMESPACE
/// \brief Print QString values as text in test failure messages.
void PrintTo(const QString &_str, std::ostream *_os)
{
  *_os << '"' << _str.toStdString() << '"';
}
QT_END_NAMESPACE

/// \brief Expect a non-empty list of worlds whose _key field is _value.
void ExpectAllWorldsHave(const QVariantList &_worlds, const QString &_key,
                         const QString &_value)
{
  EXPECT_FALSE(_worlds.isEmpty()) << _key.toStdString();
  for (const QVariant &world : _worlds)
    EXPECT_EQ(_value, world.toMap().value(_key).toString());
}

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

  // The dialog shows the featured worlds as a fixed 3x2 grid of thumbnails
  const QDir examplesDir(
      QString::fromStdString(std::string(PROJECT_SOURCE_PATH) +
                             "/examples/worlds"));
  const QVariantList featured = handler.FeaturedWorlds();
  EXPECT_EQ(6, featured.size());
  for (const QVariant &worldVar : featured)
  {
    const QString thumb = worldVar.toMap().value("thumbnail").toString();
    EXPECT_TRUE(QFile::exists(examplesDir.filePath("thumbnails/" + thumb)))
        << "Missing thumbnail [" << thumb.toStdString() << "]";
  }

  for (const QVariant &catVar : categories)
  {
    const QVariantMap cat = catVar.toMap();
    EXPECT_FALSE(cat.value("name").toString().isEmpty());
    EXPECT_EQ(cat.value("count").toInt(),
              handler.Worlds(cat.value("id").toString()).size());
  }

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

  ExpectAllWorldsHave(handler.Worlds("mobile"), "category", "mobile");
  ExpectAllWorldsHave(handler.Worlds("all", "local"), "source", "local");
  ExpectAllWorldsHave(handler.Worlds("all", "fuel"), "source", "fuel");

  const QVariantList pandaMatches = handler.Worlds("all", "all", "panda");
  ASSERT_EQ(1, pandaMatches.size());
  EXPECT_EQ("panda_joint_control",
            pandaMatches.first().toMap().value("id").toString());

  // "starter" only appears in the tags of the empty world
  const QVariantList tagMatches = handler.Worlds("all", "all", " STARTER ");
  ASSERT_EQ(1, tagMatches.size());
  EXPECT_EQ("empty", tagMatches.first().toMap().value("id").toString());
}

/////////////////////////////////////////////////
TEST(QuickStartHandlerTest, ResolveThumbnailUrl)
{
  QTemporaryDir prefix;
  ASSERT_TRUE(prefix.isValid());
  qputenv("GZ_SIM_INSTALL_PREFIX", prefix.path().toUtf8());
  QuickStartHandler handler;
  const QDir thumbs(handler.WorldsPath() + "/thumbnails");
  ASSERT_TRUE(thumbs.mkpath("."));
  QFile png(thumbs.filePath("Owner.My World.png"));
  ASSERT_TRUE(png.open(QIODevice::WriteOnly));
  png.close();

  QVariantMap world;
  world["thumbnail"] = "Owner.My World.png";
  EXPECT_EQ(QUrl::fromLocalFile(png.fileName()).toString(),
            handler.ResolveThumbnailUrl(world));
  world["thumbnail"] = "missing.png";
  EXPECT_TRUE(handler.ResolveThumbnailUrl(world).isEmpty());
  qunsetenv("GZ_SIM_INSTALL_PREFIX");
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

  // Invalid catalogs are rejected and the loaded one is kept
  const QVariantList prevWorlds = handler.Worlds();
  const QVariantList prevCategories = handler.Categories();
  EXPECT_FALSE(handler.LoadCatalog("/nonexistent/path/catalog.json"));
  for (const char *json : {"{ not valid json", "[]", "{}",
                           R"({"categories": [], "worlds": {}})",
                           R"({"categories": [], "worlds": [{"id": "a"}]})"})
  {
    QTemporaryFile file;
    ASSERT_TRUE(file.open());
    file.write(json);
    file.flush();
    EXPECT_FALSE(handler.LoadCatalog(file.fileName())) << json;
  }
  EXPECT_TRUE(prevWorlds == handler.Worlds());
  EXPECT_TRUE(prevCategories == handler.Categories());
}

/////////////////////////////////////////////////
TEST(QuickStartHandlerTest, RecentWorldsAndLocalUrl)
{
  QuickStartHandler handler;
  handler.SetSettingsGroup("QuickStartTestIsolated");
  handler.ClearRecentWorlds();

  EXPECT_TRUE(handler.RecentWorlds().isEmpty());

  // Empty URI is ignored
  handler.RecordRecentWorld("   ", "Ignored");
  EXPECT_TRUE(handler.RecentWorlds().isEmpty());

  // Record 6 worlds; should cap at 5, most recent first
  for (int i = 1; i <= 6; ++i)
  {
    handler.RecordRecentWorld(
        QString("world_%1.sdf").arg(i),
        QString("World %1").arg(i));
  }
  QVariantList recents = handler.RecentWorlds();
  ASSERT_EQ(5, recents.size());
  EXPECT_EQ("world_6.sdf", recents[0].toMap().value("uri").toString());
  EXPECT_EQ("world_2.sdf", recents[4].toMap().value("uri").toString());

  // Re-recording an existing world moves it to the front without duplicating
  handler.RecordRecentWorld("world_3.sdf", "World 3 Updated");
  recents = handler.RecentWorlds();
  ASSERT_EQ(5, recents.size());
  EXPECT_EQ("world_3.sdf", recents[0].toMap().value("uri").toString());
  EXPECT_EQ("World 3 Updated", recents[0].toMap().value("title").toString());

  // Persistence across handler instances
  QuickStartHandler secondHandler;
  secondHandler.SetSettingsGroup("QuickStartTestIsolated");
  EXPECT_EQ(5, secondHandler.RecentWorlds().size());
  EXPECT_EQ("world_3.sdf",
            secondHandler.RecentWorlds()[0].toMap().value("uri").toString());

  // LocalPathFromUrl
  EXPECT_EQ("/tmp/example.sdf",
            handler.LocalPathFromUrl(QUrl::fromLocalFile("/tmp/example.sdf")));
  EXPECT_EQ("https://fuel.gazebosim.org/1.0/OpenRobotics/worlds/Empty",
            handler.LocalPathFromUrl(
                QUrl("https://fuel.gazebosim.org/1.0/OpenRobotics/worlds/"
                     "Empty")));

  // Cleanup test settings
  handler.ClearRecentWorlds();
  EXPECT_TRUE(handler.RecentWorlds().isEmpty());
}
