/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include "QuickStartHandler.hh"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QUrl>

#include "gz/sim/InstallationDirectories.hh"
#include <gz/transport/Node.hh>
#include <gz/common/Console.hh>

using namespace gz;
using namespace sim::gui;

namespace
{
constexpr const char *kDefaultCatalogResource =
    ":/Gazebo/quickstart_worlds.json";
constexpr const char *kFuelWorldsBaseUrl = "https://fuel.gazebosim.org/1.0/";

/// \brief String fields copied from each catalog world entry.
constexpr const char *kWorldStringFields[] = {
    "id", "title", "description", "category", "source", "owner",
    "fileName", "fuelName", "thumbnail", "fuelThumbnail"};
}

QuickStartHandler::QuickStartHandler()
{
  std::vector<transport::MessagePublisher> publishers;
  std::vector<transport::MessagePublisher> subscribers;
  this->node.TopicInfo("/stats", publishers, subscribers);
  if (!publishers.empty())
  {
    gzerr << "Detected an already running world on start" << std::endl;
  }

  this->LoadCatalog(QString::fromUtf8(kDefaultCatalogResource));
}

/////////////////////////////////////////////////
QString QuickStartHandler::WorldsPath() const
{
  std::string worldsPathLocal = gz::sim::getWorldInstallDir();
  return QString::fromUtf8(worldsPathLocal.c_str());
}

/////////////////////////////////////////////////
std::string QuickStartHandler::StartingWorld() const
{
  return this->startingWorld;
}

/////////////////////////////////////////////////
QString QuickStartHandler::Distribution() const
{
  return QString::fromUtf8(GZ_DISTRIBUTION);
}

/////////////////////////////////////////////////
QString QuickStartHandler::SimVersion() const
{
  return QString::fromUtf8(GZ_SIM_VERSION_FULL);
}

/////////////////////////////////////////////////
void QuickStartHandler::SetStartingWorld(const QString &_url)
{
  this->startingWorld = _url.toStdString();
}

/////////////////////////////////////////////////
void QuickStartHandler::SetShowAgain(const bool _showAgain)
{
  this->showAgain = !_showAgain;
}

/////////////////////////////////////////////////
bool QuickStartHandler::ShowAgain() const
{
  return this->showAgain;
}

/////////////////////////////////////////////////
bool QuickStartHandler::LoadCatalog(const QString &_catalogPath)
{
  QFile file(_catalogPath);
  if (!file.open(QIODevice::ReadOnly))
  {
    gzerr << "Failed to open QuickStart catalog ["
          << _catalogPath.toStdString() << "]" << std::endl;
    return false;
  }

  QJsonParseError parseErr;
  const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &parseErr);
  if (parseErr.error != QJsonParseError::NoError || !doc.isObject())
  {
    gzerr << "Failed to parse QuickStart catalog ["
          << _catalogPath.toStdString() << "]: "
          << parseErr.errorString().toStdString() << std::endl;
    return false;
  }

  const QJsonObject root = doc.object();
  if (!root.value("categories").isArray() || !root.value("worlds").isArray())
  {
    gzerr << "QuickStart catalog [" << _catalogPath.toStdString()
          << "] is missing required 'categories' or 'worlds' arrays"
          << std::endl;
    return false;
  }

  QVariantList parsedWorlds;
  QHash<QString, int> categoryCounts;
  int featuredCount = 0;

  const QJsonArray worldsArray = root.value("worlds").toArray();
  for (const QJsonValue &val : worldsArray)
  {
    if (!val.isObject())
      continue;

    const QJsonObject obj = val.toObject();
    QVariantMap entry;
    for (const char *field : kWorldStringFields)
      entry[field] = obj.value(field).toString();

    if (entry["id"].toString().isEmpty() ||
        entry["title"].toString().isEmpty())
    {
      continue;
    }

    entry["featured"] = obj.value("featured").toBool();
    QStringList tags;
    const QJsonArray tagsArray = obj.value("tags").toArray();
    for (const QJsonValue &tagVal : tagsArray)
    {
      if (tagVal.isString())
        tags.append(tagVal.toString());
    }
    entry["tags"] = tags;

    parsedWorlds.append(entry);
    const QString cat = entry["category"].toString();
    if (!cat.isEmpty())
      ++categoryCounts[cat];
    if (entry["featured"].toBool())
      ++featuredCount;
  }

  QVariantList parsedCategories;
  const QJsonArray categoriesArray = root.value("categories").toArray();
  for (const QJsonValue &val : categoriesArray)
  {
    if (!val.isObject())
      continue;

    const QJsonObject obj = val.toObject();
    const QString id = obj.value("id").toString();
    if (id.isEmpty())
      continue;

    QVariantMap cat;
    cat["id"] = id;
    cat["name"] = obj.value("name").toString();
    cat["icon"] = obj.value("icon").toString();
    cat["count"] = (id == "featured")
        ? featuredCount
        : categoryCounts.value(id, 0);
    parsedCategories.append(cat);
  }

  this->categories = parsedCategories;
  this->worlds = parsedWorlds;
  return true;
}

/////////////////////////////////////////////////
QVariantList QuickStartHandler::Categories() const
{
  return this->categories;
}

/////////////////////////////////////////////////
QVariantList QuickStartHandler::Worlds(
    const QString &_category,
    const QString &_source,
    const QString &_query) const
{
  const QString catFilter = _category.trimmed().toLower();
  const QString srcFilter = _source.trimmed().toLower();
  const QString queryFilter = _query.trimmed().toLower();

  QVariantList result;
  for (const QVariant &item : this->worlds)
  {
    const QVariantMap world = item.toMap();

    if (!catFilter.isEmpty() && catFilter != "all")
    {
      if (catFilter == "featured")
      {
        if (!world.value("featured").toBool())
          continue;
      }
      else if (world.value("category").toString().toLower() != catFilter)
      {
        continue;
      }
    }

    if (!srcFilter.isEmpty() && srcFilter != "all")
    {
      if (world.value("source").toString().toLower() != srcFilter)
        continue;
    }

    if (!queryFilter.isEmpty())
    {
      bool matched =
          world.value("title").toString().toLower().contains(queryFilter) ||
          world.value("description").toString().toLower().contains(
              queryFilter) ||
          world.value("owner").toString().toLower().contains(queryFilter) ||
          world.value("category").toString().toLower().contains(queryFilter) ||
          world.value("fileName").toString().toLower().contains(queryFilter) ||
          world.value("fuelName").toString().toLower().contains(queryFilter);

      if (!matched)
      {
        const QStringList tags = world.value("tags").toStringList();
        for (const QString &tag : tags)
        {
          if (tag.toLower().contains(queryFilter))
          {
            matched = true;
            break;
          }
        }
      }

      if (!matched)
        continue;
    }

    result.append(world);
  }

  return result;
}

/////////////////////////////////////////////////
QVariantList QuickStartHandler::FeaturedWorlds() const
{
  return this->Worlds("featured", "all", "");
}

/////////////////////////////////////////////////
QString QuickStartHandler::ResolveWorldUri(const QVariantMap &_world) const
{
  const QString source = _world.value("source").toString().toLower();
  if (source == "local")
    return _world.value("fileName").toString();

  if (source == "fuel")
  {
    const QString owner = _world.value("owner").toString();
    const QString fuelName = _world.value("fuelName").toString();
    if (owner.isEmpty() || fuelName.isEmpty())
      return "";
    return QString::fromUtf8(kFuelWorldsBaseUrl) + owner + "/worlds/" +
           fuelName;
  }

  return "";
}

/////////////////////////////////////////////////
QString QuickStartHandler::ResolveThumbnailUrl(const QVariantMap &_world) const
{
  const QString thumb = _world.value("thumbnail").toString();
  if (thumb.isEmpty())
    return "";

  const QString fullPath =
      QDir(this->WorldsPath()).filePath("thumbnails/" + thumb);
  if (!QFileInfo::exists(fullPath))
    return "";

  return QUrl::fromLocalFile(fullPath).toString();
}
