/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_SERVERCONFIG_HH_
#define IGNITION_GAZEBO_SERVERCONFIG_HH_

#include <chrono>
#include <list>
#include <memory>
#include <optional> // NOLINT(*)
#include <string>
#include <sdf/Element.hh>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    // Forward declarations.
    class ServerConfigPrivate;

    /// \class ServerConfig ServerConfig.hh ignition/gazebo/ServerConfig.hh
    /// \brief Configuration parameters for a Server. An instance of this
    /// object can be used to construct a Server with a particular
    /// configuration.
    class IGNITION_GAZEBO_VISIBLE ServerConfig
    {
      class PluginInfoPrivate;
      /// \brief Information about a plugin that should be loaded by the
      /// server.
      /// \detail Currently supports attaching a plugin to an entity given its
      /// type and name, but it can't tell apart multiple entities with the same
      /// name in different parts of the entity tree.
      /// \sa const std::list<PluginInfo> &Plugins() const
      public: class PluginInfo
      {
        /// \brief Default constructor.
        public: PluginInfo();

        /// \brief Destructor.
        public: ~PluginInfo();

        /// \brief Constructor with plugin information specified.
        /// \param[in] _entityName Name of the entity which should receive
        /// this plugin. The name is used in conjuction with _entityType to
        /// uniquely identify an entity.
        /// \param[in] _entityType Entity type which should receive  this
        /// plugin. The type is used in conjuction with _entityName to
        /// uniquely identify an entity.
        /// \param[in] _filename Plugin library filename.
        /// \param[in] _name Name of the interface within the plugin library
        /// to load.
        /// \param[in] _sdf Plugin XML elements associated with this plugin.
        public: PluginInfo(const std::string &_entityName,
                           const std::string &_entityType,
                           const std::string &_filename,
                           const std::string &_name,
                           const sdf::ElementPtr &_sdf);

        /// \brief Copy constructor.
        /// \param[in] _info Plugin to copy.
        public: PluginInfo(const PluginInfo &_info);

        /// \brief Equal operator.
        /// \param[in] _info PluginInfo to copy.
        /// \return Reference to this class.
        public: PluginInfo &operator=(const PluginInfo &_info);

        /// \brief Get the name of the entity which should receive
        /// this plugin. The name is used in conjuction with _entityType to
        /// uniquely identify an entity.
        /// \return Entity name.
        public: const std::string &EntityName() const;

        /// \brief Set the name of the entity which should receive
        /// this plugin. The name is used in conjuction with _entityType to
        /// uniquely identify an entity.
        /// \param[in] _entityName Entity name.
        public: void SetEntityName(const std::string &_entityName);

        /// \brief Get the entity type which should receive  this
        /// plugin. The type is used in conjuction with EntityName to
        /// uniquely identify an entity.
        /// \return Entity type string.
        public: const std::string &EntityType() const;

        /// \brief Set the type of the entity which should receive this
        /// plugin. The type is used in conjuction with EntityName to
        /// uniquely identify an entity.
        /// \param[in] _entityType Entity type string.
        public: void SetEntityType(const std::string &_entityType);

        /// \brief Get the plugin library filename.
        /// \return Plugin library filename.
        public: const std::string &Filename() const;

        /// \brief Set the type of the entity which should receive this
        /// plugin. The type is used in conjuction with EntityName to
        /// uniquely identify an entity.
        /// \param[in] _entityType Entity type string.
        public: void SetFilename(const std::string &_filename);

        /// \brief Name of the interface within the plugin library
        /// to load.
        /// \return Interface name.
        public: const std::string &Name() const;

        /// \brief Set the name of the interface within the plugin library
        /// to load.
        /// \param[in] _name Interface name.
        public: void SetName(const std::string &_name);

        /// \brief Plugin XML elements associated with this plugin.
        /// \return SDF pointer.
        public: const sdf::ElementPtr &Sdf() const;

        /// \brief Set the plugin XML elements associated with this plugin.
        /// \param[in] _sdf SDF pointer, it will be cloned.
        public: void SetSdf(const sdf::ElementPtr &_sdf);

        /// \brief Private data pointer
        private: std::unique_ptr<ServerConfig::PluginInfoPrivate> dataPtr;
      };

      /// \brief Constructor
      public: ServerConfig();

      /// \brief Copy constructor.
      /// \param[in] _config ServerConfig to copy.
      public: ServerConfig(const ServerConfig &_config);

      /// \brief Destructor
      public: ~ServerConfig();

      /// \brief Set an SDF file to be used with the server.
      ///
      /// Setting the SDF file will override any value set by `SetSdfString`.
      ///
      /// \param[in] _file Full path to an SDF file.
      /// \return (reserved for future use)
      public: bool SetSdfFile(const std::string &_file);

      /// \brief Get the SDF file that has been set. An empty string will be
      /// returned if an SDF file has not been set.
      /// \return The full path to the SDF file, or empty string.
      public: std::string SdfFile() const;

      /// \brief Set an SDF string to be used by the server.
      ///
      /// Setting the SDF string will override any value set by `SetSdfFile`.
      ///
      /// \param[in] _file Full path to an SDF file.
      /// \return (reserved for future use)
      public: bool SetSdfString(const std::string &_sdfString);

      /// \brief Get the SDF String that has been set. An empty string will
      /// be returned if an SDF string has not been set.
      /// \return The full contents of the SDF string, or empty string.
      public: std::string SdfString() const;

      /// \brief Set the update rate in Hertz. Value <=0 are ignored.
      /// \param[in] _hz The desired update rate of the server in Hertz.
      public: void SetUpdateRate(const double &_hz);

      /// \brief Get the update rate in Hertz.
      /// \return The desired update rate of the server in Hertz, or nullopt if
      /// an UpdateRate has not been set.
      public: std::optional<double> UpdateRate() const;

      /// \brief Get whether the server is using the level system
      /// \return True if the server is set to use the level system
      public: bool UseLevels() const;

      /// \brief Get whether the server is using the level system.
      /// \param[in] _levels Value to set.
      public: void SetUseLevels(const bool _levels);

      /// \brief Get whether the server is using the distributed sim system
      /// \return True if the server is set to use the distributed simulation
      /// system
      /// \sa SetNetworkRole(const std::string &_role)
      public: bool UseDistributedSimulation() const;

      /// \brief Set the number of network secondary servers that the
      /// primary server should expect. This value is valid only when
      /// SetNetworkRole("primary") is also used.
      /// \param[in] _secondaries Number of secondary servers.
      /// \sa SetNetworkRole(const std::string &_role)
      /// \sa NetworkRole() const
      public: void SetNetworkSecondaries(unsigned int _secondaries);

      /// \brief Get the number of secondary servers that a primary server
      /// should expect.
      /// \return Number of secondary servers.
      /// \sa SetNetworkSecondaries(unsigned int _secondaries)
      public: unsigned int NetworkSecondaries() const;

      /// \brief Set the network role, which is one of [primary, secondary].
      /// If primary is used, then make sure to also set the numer of
      /// network secondaries via
      /// SetNetworkSecondaries(unsigned int _secondaries).
      /// \param[in] _role Network role, one of [primary, secondary].
      /// \note Setting a network role enables distributed simulation.
      /// \sa SetNetworkSecondaries(unsigned int _secondaries)
      public: void SetNetworkRole(const std::string &_role);

      /// \brief Get the network role. See
      /// SetNetworkRole(const std::string &_role) for more information
      /// about distributed simulation and network roles.
      /// \return The network role.
      /// \sa SetNetworkRole(const std::string &_role)
      public: std::string NetworkRole() const;

      /// \brief Get whether the server is recording states
      /// \return True if the server is set to record states
      public: bool UseLogRecord() const;

      /// \brief Set whether the server is recording states
      /// \param[in] _record Value to set
      public: void SetUseLogRecord(const bool _record);

      /// \brief Get path to place recorded states
      /// \return Path to place recorded states
      public: const std::string LogRecordPath() const;

      /// \brief Set path to place recorded states
      /// \param[in] _recordPath Path to place recorded states
      public: void SetLogRecordPath(const std::string &_recordPath);

      /// \brief Get whether to ignore the path specified in SDF.
      /// \return Whether to ignore the path specified in SDF
      /// \TODO(anyone) Deprecate on Dome, SDF path will always be ignored.
      public: bool LogIgnoreSdfPath() const;

      /// \brief Set whether to ignore the path specified in SDF. Path in SDF
      /// should be ignored if a record path is specified on the command line,
      /// for example.
      /// \param[in] _ignore Whether to ignore the path specified in SDF
      /// \TODO(anyone) Deprecate on Dome, SDF path will always be ignored.
      public: void SetLogIgnoreSdfPath(bool _ignore);

      /// \brief Get path to recorded states to play back
      /// \return Path to recorded states
      public: const std::string LogPlaybackPath() const;

      /// \brief Set path to recorded states to play back
      /// \param[in] _playbackPath Path to recorded states
      public: void SetLogPlaybackPath(const std::string &_playbackPath);

      /// \brief Get file path to compress log files to
      /// \return File path to compress log files to
      public: std::string LogRecordCompressPath() const;

      /// \brief Set file path to compress log files to
      /// \param[in] _path File path to compress log files to
      public: void SetLogRecordCompressPath(const std::string &_path);

      /// \brief The given random seed.
      /// \return The random seed or 0 if not specified.
      public: unsigned int Seed() const;

      /// \brief Set the random seed.
      /// \param[in] _seed The seed.
      public: void SetSeed(unsigned int _seed);

      /// \brief Get the update period duration.
      /// \return The desired update period, or nullopt if
      /// an UpdateRate has not been set.
      public: std::optional<std::chrono::steady_clock::duration>
              UpdatePeriod() const;

      /// \brief Path to where simulation resources, such as models downloaded
      /// from fuel.ignitionrobotics.org, should be stored.
      /// \return Path to a location on disk. An empty string indicates that
      /// the default value will be used, which is currently
      /// ~/.ignition/fuel.
      public: const std::string &ResourceCache() const;

      /// \brief Set the path to where simulation resources, such as models
      /// downloaded from fuel.ignitionrobotics.org, should be stored.
      /// \param[in] _path Path to a location on disk. An empty string
      /// indicates that the default value will be used, which is currently
      /// ~/.ignition/fuel.
      public: void SetResourceCache(const std::string &_path);

      /// \brief Instruct simulation to attach a plugin to a specific
      /// entity when simulation starts.
      /// \param[in] _info Information about the plugin to load.
      public: void AddPlugin(const PluginInfo &_info);

      /// \brief Get all the plugins that should be loaded.
      /// \return A list of all the plugins specified via
      /// AddPlugin(const PluginInfo &).
      public: const std::list<PluginInfo> &Plugins() const;

      /// \brief Equal operator.
      /// \param[in] _cfg ServerConfig to copy.
      /// \return Reference to this class.
      public: ServerConfig &operator=(const ServerConfig &_cfg);

      /// \brief Get the timestamp of this ServerConfig. This is the system
      /// time when this ServerConfig was created. The timestamp is used
      /// internally to create log file paths so that both state and console
      /// logs are co-located.
      /// \return Time when this ServerConfig was created.
      public: const std::chrono::time_point<std::chrono::system_clock> &
              Timestamp() const;

      /// \brief Private data pointer
      private: std::unique_ptr<ServerConfigPrivate> dataPtr;
    };
    }
  }
}

#endif
