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
#include <QSet>
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

/// \brief World fields, besides tags, matched by the search query.
constexpr const char *kSearchableFields[] = {
    "title", "description", "owner", "category", "fileName", "fuelName"};

/// \brief Whether a filter value accepts every entry.
bool IsWildcard(const QString &_filter)
{
  return _filter.isEmpty() || _filter == "all";
}

/// \brief Whether a searchable field or a tag of a world contains a query.
/// \param[in] _world Catalog world entry.
/// \param[in] _query Lower-case search query.
bool MatchesQuery(const QVariantMap &_world, const QString &_query)
{
  for (const char *field : kSearchableFields)
  {
    if (_world.value(field).toString().toLower().contains(_query))
      return true;
  }

  const QStringList tags = _world.value("tags").toStringList();
  for (const QString &tag : tags)
  {
    if (tag.toLower().contains(_query))
      return true;
  }
  return false;
}
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
  const std::string path = _catalogPath.toStdString();
  QFile file(_catalogPath);
  if (!file.open(QIODevice::ReadOnly))
  {
    gzerr << "Failed to open QuickStart catalog [" << path << "]: "
          << file.errorString().toStdString() << std::endl;
    return false;
  }

  QJsonParseError parseErr;
  const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &parseErr);
  if (parseErr.error != QJsonParseError::NoError)
  {
    gzerr << "Failed to parse QuickStart catalog [" << path << "] at offset "
          << parseErr.offset << ": "
          << parseErr.errorString().toStdString() << std::endl;
    return false;
  }

  // doc.object() is empty when the root is not an object
  const QJsonObject root = doc.object();
  if (!root.value("categories").isArray() || !root.value("worlds").isArray())
  {
    gzerr << "QuickStart catalog [" << path << "] must be a JSON object "
          << "with 'categories' and 'worlds' arrays" << std::endl;
    return false;
  }

  // Parse categories first so that worlds can be checked against them
  QVariantList parsedCategories;
  QHash<QString, int> categoryCounts;
  const QJsonArray categoriesArray = root.value("categories").toArray();
  for (int i = 0; i < categoriesArray.size(); ++i)
  {
    const QJsonObject obj = categoriesArray.at(i).toObject();
    // Ids are case-insensitive and "all" means "no category filter"
    const QString id = obj.value("id").toString().toLower();
    if (id.isEmpty() || id == "all" || categoryCounts.contains(id))
    {
      gzerr << "QuickStart catalog [" << path << "]: skipping categories["
            << i << "] with a missing, reserved or duplicate id" << std::endl;
      continue;
    }
    categoryCounts.insert(id, 0);

    QVariantMap cat;
    cat["id"] = id;
    cat["name"] = obj.value("name").toString();
    cat["icon"] = obj.value("icon").toString();
    parsedCategories.append(cat);
  }

  QVariantList parsedWorlds;
  QSet<QString> worldIds;
  int featuredCount = 0;
  const QJsonArray worldsArray = root.value("worlds").toArray();
  for (int i = 0; i < worldsArray.size(); ++i)
  {
    const QJsonObject obj = worldsArray.at(i).toObject();
    QVariantMap entry;
    for (const char *field : kWorldStringFields)
      entry[field] = obj.value(field).toString();

    // Category and source are matched case-insensitively, and a world in
    // the "featured" category is always featured
    entry["category"] = entry["category"].toString().toLower();
    entry["source"] = entry["source"].toString().toLower();
    entry["featured"] = obj.value("featured").toBool() ||
        entry["category"] == "featured";
    QStringList tags;
    const QJsonArray tagsArray = obj.value("tags").toArray();
    for (const QJsonValue &tagVal : tagsArray)
    {
      if (tagVal.isString())
        tags.append(tagVal.toString());
    }
    entry["tags"] = tags;

    const QString id = entry["id"].toString();
    const QString cat = entry["category"].toString();
    std::string error;
    if (id.isEmpty() || entry["title"].toString().isEmpty())
      error = "missing id or title";
    else if (worldIds.contains(id))
      error = "duplicate id";
    else if (!categoryCounts.contains(cat))
      error = "unknown category [" + cat.toStdString() + "]";
    else if (this->ResolveWorldUri(entry).isEmpty())
      error = "unknown source or missing fileName, owner or fuelName";
    if (!error.empty())
    {
      gzerr << "QuickStart catalog [" << path << "]: skipping worlds[" << i
            << "] [" << id.toStdString() << "]: " << error << std::endl;
      continue;
    }

    worldIds.insert(id);
    ++categoryCounts[cat];
    if (entry["featured"].toBool())
      ++featuredCount;
    parsedWorlds.append(entry);
  }

  if (parsedWorlds.isEmpty())
  {
    gzerr << "QuickStart catalog [" << path << "] has no valid worlds"
          << std::endl;
    return false;
  }

  for (QVariant &catVar : parsedCategories)
  {
    QVariantMap cat = catVar.toMap();
    const QString id = cat["id"].toString();
    cat["count"] = id == "featured" ? featuredCount : categoryCounts.value(id);
    catVar = cat;
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

    if (!IsWildcard(catFilter))
    {
      const bool inCategory = (catFilter == "featured")
          ? world.value("featured").toBool()
          : world.value("category").toString() == catFilter;
      if (!inCategory)
        continue;
    }

    if (!IsWildcard(srcFilter) &&
        world.value("source").toString() != srcFilter)
    {
      continue;
    }

    if (!queryFilter.isEmpty() && !MatchesQuery(world, queryFilter))
      continue;

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
