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

#include <QtCore>
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

  // Explicitly declare the constructor
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

  /// \brief Load world catalog from a JSON file or Qt resource path.
  /// \param[in] _catalogPath Path to catalog JSON (e.g. Qt resource path).
  /// \return True if the catalog was loaded and parsed successfully.
  public: bool LoadCatalog(const QString &_catalogPath);

  /// \brief Get list of robotics categories defined in the catalog.
  /// Each entry is a QVariantMap with keys: id, name, icon, count.
  /// \return Category metadata list.
  public: Q_INVOKABLE QVariantList Categories() const;

  /// \brief Get curated worlds filtered by category, source, and search query.
  /// \param[in] _category Category id ("", "all", "featured", or category id).
  /// \param[in] _source Source filter ("all", "local", or "fuel").
  /// \param[in] _query Case-insensitive search query matched against title,
  /// description, owner, tags, and file/fuel name.
  /// \return Matching world entries (each a QVariantMap).
  public: Q_INVOKABLE QVariantList Worlds(
      const QString &_category = "",
      const QString &_source = "all",
      const QString &_query = "") const;

  /// \brief Get curated worlds marked as featured.
  /// \return Featured world entries.
  public: Q_INVOKABLE QVariantList FeaturedWorlds() const;

  /// \brief Resolve a catalog world entry into a launchable SDF filename or
  /// Gazebo Fuel URL.
  /// \param[in] _world World metadata map from the catalog.
  /// \return Local SDF filename or Fuel URL, or empty string if invalid.
  public: Q_INVOKABLE QString ResolveWorldUri(
      const QVariantMap &_world) const;

  /// \brief Resolve a catalog world entry into a local thumbnail file URL
  /// if available on disk.
  /// \param[in] _world World metadata map from the catalog.
  /// \return Local file:// URL if thumbnail exists on disk, else empty string.
  public: Q_INVOKABLE QString ResolveThumbnailUrl(
      const QVariantMap &_world) const;

  /// \brief Show the quick start menu again.
  private: bool showAgain{true};

  /// \brief Get starting world url.
  private: std::string startingWorld{""};

  /// \brief Loaded catalog categories.
  private: QVariantList categories;

  /// \brief Loaded catalog worlds.
  private: QVariantList worlds;

  // Add a member for the transport node
  private: gz::transport::Node node;
};
}
}
}
}
#endif
