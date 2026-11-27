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
#ifndef GZ_SIM_GUI_QUICKSTARTHANDLER_HH_
#define GZ_SIM_GUI_QUICKSTARTHANDLER_HH_

#include <QObject>
#include <QString>
#include <QVariantList>
#include <QVariantMap>
#include <string>

#include "gz/sim/config.hh"
#include "gz/sim/gui/Export.hh"

#include <gz/transport/Node.hh>
#include <gz/common/Console.hh>

#include <memory>
#include <vector>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace gui
{
/// \brief Class for handling quick start dialog
class GZ_SIM_GUI_VISIBLE QuickStartHandler : public QObject
{
  Q_OBJECT

  /// \brief Constructor. Checks for an already running world and loads the
  /// bundled world catalog.
  public: QuickStartHandler();

  /// \brief Get worlds path
  /// \return worlds directory path
  public: Q_INVOKABLE QString WorldsPath() const;

  /// \brief Get the distribution name
  /// \return Distribution name, such as 'Citadel'
  public: Q_INVOKABLE QString Distribution() const;

  /// \brief Get Gazebo Sim version
  /// \return Version
  public: Q_INVOKABLE QString SimVersion() const;

  /// \brief Set starting world
  /// \param[in] _url Url to the world file.
  public: Q_INVOKABLE void SetStartingWorld(const QString &_url);

  /// \brief Get starting world url from GUI.
  /// \return World url
  public: Q_INVOKABLE std::string StartingWorld() const;

  /// \brief Set the flag to show quick start menu again.
  /// \param[in] _showQuickStartOpts True to show.
  public: Q_INVOKABLE void SetShowAgain(const bool _showAgain);

  /// \brief Show again option.
  /// \return True to show again.
  public: Q_INVOKABLE bool ShowAgain() const;

  /// \brief Load the world catalog, replacing the current one on success.
  ///
  /// The catalog is a JSON object with two arrays:
  /// * "categories": objects with "id", "name" and "icon". Ids are
  ///   case-insensitive, "all" is reserved, and "featured" lists the worlds
  ///   that are featured.
  /// * "worlds": objects with "id", "title", "description", "category",
  ///   "featured", "tags", a "thumbnail" file in WorldsPath()/thumbnails, and
  ///   either "source": "local" with an SDF "fileName", or "source": "fuel"
  ///   with "owner", "fuelName" and a "fuelThumbnail" path inside the files
  ///   of the Fuel world. Worlds in the "featured" category are featured.
  /// Invalid, duplicate or unresolvable entries are skipped with an error.
  /// \param[in] _catalogPath Catalog file, such as
  /// ":/Gazebo/quickstart_worlds.json".
  /// \return False, keeping the current catalog, if the file cannot be read,
  /// is not a catalog, or has no valid world.
  public: bool LoadCatalog(const QString &_catalogPath);

  /// \brief Get the catalog categories.
  /// \return One QVariantMap per category, with keys id, name, icon and
  /// count, the number of worlds that Worlds(id) returns.
  public: Q_INVOKABLE QVariantList Categories() const;

  /// \brief Get the catalog worlds, in catalog order, that pass all filters.
  ///
  /// Filters are trimmed and case-insensitive.
  /// \param[in] _category Category id, "featured" for the featured worlds,
  /// or "" or "all" for every category.
  /// \param[in] _source "local", "fuel", or "" or "all" for both.
  /// \param[in] _query Text that the title, description, owner, category,
  /// fileName, fuelName or a tag must contain, or "" for no search.
  /// \return One QVariantMap per world, with the catalog keys.
  public: Q_INVOKABLE QVariantList Worlds(
      const QString &_category = "",
      const QString &_source = "all",
      const QString &_query = "") const;

  /// \brief Get curated worlds marked as featured.
  /// \return Featured world entries.
  public: Q_INVOKABLE QVariantList FeaturedWorlds() const;

  /// \brief Get the world to launch for a catalog world: the "fileName" of a
  /// local world, which is not checked on disk, or
  /// https://fuel.gazebosim.org/1.0/OWNER/worlds/FUEL_NAME for a Fuel
  /// world.
  /// \param[in] _world World entry, as returned by Worlds().
  /// \return The world to launch, or an empty string if the source is
  /// unknown or one of its fields is missing.
  public: Q_INVOKABLE QString ResolveWorldUri(
      const QVariantMap &_world) const;

  /// \brief Get the local thumbnail of a catalog world, the "thumbnail" file
  /// in WorldsPath()/thumbnails.
  /// \param[in] _world World entry, as returned by Worlds().
  /// \return file:// URL of the thumbnail, or an empty string if the world
  /// has none or the file does not exist.
  public: Q_INVOKABLE QString ResolveThumbnailUrl(
      const QVariantMap &_world) const;

  /// \brief Show the quick start menu again.
  private: bool showAgain{true};

  /// \brief Get starting world url.
  private: std::string startingWorld{""};

  /// \brief Catalog categories, as returned by Categories().
  private: QVariantList categories;

  /// \brief Catalog worlds, as returned by Worlds().
  private: QVariantList worlds;

  // Add a member for the transport node
  private: gz::transport::Node node;
};
}
}
}
}
#endif
