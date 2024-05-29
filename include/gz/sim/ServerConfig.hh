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
#ifndef GZ_SIM_SERVERCONFIG_HH_
#define GZ_SIM_SERVERCONFIG_HH_

#include <chrono>
#include <list>
#include <memory>
#include <optional> // NOLINT(*)
#include <string>
#include <vector>
#include <sdf/Element.hh>
#include <sdf/Plugin.hh>
#include <sdf/Root.hh>
#include <gz/sim/config.hh>
#include <gz/sim/Export.hh>

namespace gz
{
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE {
    // Forward declarations.
    class ServerConfigPrivate;

    /// \class ServerConfig ServerConfig.hh gz/sim/ServerConfig.hh
    /// \brief Configuration parameters for a Server. An instance of this
    /// object can be used to construct a Server with a particular
    /// configuration.
    class GZ_SIM_VISIBLE ServerConfig
    {
      /// \brief Type of SDF source.
      public: enum class SourceType
      {
        // No source specified.
        kNone,

        // The source is an SDF Root object.
        kSdfRoot,

        // The source is an SDF file.
        kSdfFile,

        // The source is an SDF string.
        kSdfString,
      };

      /// \brief SDF error behavior
      public: enum class SdfErrorBehavior
      {
        /// \brief Exit the server immediately
        EXIT_IMMEDIATELY,
        /// \brief Continue loading the server if possible
        CONTINUE_LOADING
      };

      class PluginInfoPrivate;
      /// \brief Information about a plugin that should be loaded by the
      /// server.
      /// \details Currently supports attaching a plugin to an entity given its
      /// type and name, but it can't tell apart multiple entities with the same
      /// name in different parts of the entity tree.
      /// \sa const std::list<PluginInfo> &Plugins() const
      public: class GZ_SIM_VISIBLE PluginInfo
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
        /// \param[in] _plugin SDF Plugin library information.
        public: PluginInfo(const std::string &_entityName,
                           const std::string &_entityType,
                           const sdf::Plugin &_plugin);

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

        /// \brief Get the SDF plugin information.
        /// \return The SDF Plugin object.
        public: const sdf::Plugin &Plugin() const;

        /// \brief Get a mutable version of the SDF plugin information.
        /// \return The SDF Plugin object.
        public: sdf::Plugin &Plugin();

        /// \brief Set the SDF plugin information.
        /// \param[in] _plugin The SDF Plugin object to use.
        public: void SetPlugin(const sdf::Plugin &_plugin) const;

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
      /// \return True if the file was set, false if the file was not set.
      /// The file will not be set if the provide _file string is empty.
      public: bool SetSdfFile(const std::string &_file);

      /// \brief Get the SDF file that has been set. An empty string will be
      /// returned if an SDF file has not been set.
      /// \return The full path to the SDF file, or empty string.
      public: std::string SdfFile() const;

      /// \brief Set an SDF string to be used by the server.
      ///
      /// Setting the SDF string will override any value set by `SetSdfFile`.
      ///
      /// \param[in] _sdfString Full path to an SDF file.
      /// \return (reserved for future use)
      public: bool SetSdfString(const std::string &_sdfString);

      /// \brief Get the SDF String that has been set. An empty string will
      /// be returned if an SDF string has not been set.
      /// \return The full contents of the SDF string, or empty string.
      public: std::string SdfString() const;

      /// \brief Set the SDF Root DOM object. The sdf::Root object will take
      /// precendence over ServerConfig::SdfString() and
      /// ServerConfig::SdfFile().
      /// \param[in] _root SDF Root object to use.
      public: void SetSdfRoot(const sdf::Root &_root) const;

      /// \brief Get the SDF Root DOM object.
      /// \return SDF Root object to use, or std::nullopt if the sdf::Root
      /// has not been set via ServerConfig::SetSdfRoot().
      public: std::optional<sdf::Root> &SdfRoot() const;

      /// \brief Set the update rate in Hertz. Value <=0 are ignored.
      /// \param[in] _hz The desired update rate of the server in Hertz.
      public: void SetUpdateRate(const double &_hz);

      /// \brief Get the update rate in Hertz.
      /// \return The desired update rate of the server in Hertz, or nullopt if
      /// an UpdateRate has not been set.
      public: std::optional<double> UpdateRate() const;

      /// \brief Set the initial simulation time in seconds.
      /// \param[in] _initialSimTime The desired initial simulation time in
      /// seconds.
      public: void SetInitialSimTime(const double &_initialSimTime) const;

      /// \brief Get the initial simulation time in seconds.
      /// \return The initial simulation time in seconds.
      public: double InitialSimTime() const;

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

      /// \brief Get time period to record states
      /// \return Time period to record states
      public: std::chrono::steady_clock::duration LogRecordPeriod() const;

      /// \brief Set time period to record states
      /// \param[in] _period Time period to record states
      public: void SetLogRecordPeriod(
          const std::chrono::steady_clock::duration &_period);

      /// \brief Add a topic to record.
      /// \param[in] _topic Topic name, which can include wildcards.
      public: void AddLogRecordTopic(const std::string &_topic);

      /// \brief Clear topics to record. This will remove all topics set
      /// using AddLogRecordTopic.
      public: void ClearLogRecordTopics();

      /// \brief Get the topics to record that were added using
      /// AddLogRecordTopic.
      /// \return The topics to record.
      public: const std::vector<std::string> &LogRecordTopics() const;

      /// \brief Get path to recorded states to play back
      /// \return Path to recorded states
      public: const std::string LogPlaybackPath() const;

      /// \brief Set path to recorded states to play back
      /// \param[in] _playbackPath Path to recorded states
      public: void SetLogPlaybackPath(const std::string &_playbackPath);

      /// \brief Get whether meshes and material files are recorded
      /// \return True if resources should be recorded.
      public: bool LogRecordResources() const;

      /// \brief Set whether meshes and material files are recorded
      /// \param[in] _recordResources Value to set
      public: void SetLogRecordResources(bool _recordResources);

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
      /// from fuel.gazebosim.org, should be stored.
      /// \return Path to a location on disk. An empty string indicates that
      /// the default value will be used, which is currently
      /// ~/.gz/fuel.
      public: const std::string &ResourceCache() const;

      /// \brief Set the path to where simulation resources, such as models
      /// downloaded from fuel.gazebosim.org, should be stored.
      /// \param[in] _path Path to a location on disk. An empty string
      /// indicates that the default value will be used, which is currently
      /// ~/.gz/fuel.
      public: void SetResourceCache(const std::string &_path);

      /// \brief Physics engine plugin library to load.
      /// \return File containing physics engine library.
      public: const std::string &PhysicsEngine() const;

      /// \brief Set the physics engine plugin library.
      /// \param[in] _physicsEngine File containing physics engine library.
      public: void SetPhysicsEngine(const std::string &_physicsEngine);

      /// \brief Render engine plugin library to load.
      /// \return File containing render engine library.
      public: const std::string &RenderEngineServer() const;

      /// \brief Render engine plugin library to load.
      /// \return File containing render engine library.
      public: const std::string &RenderEngineGui() const;

      /// \brief Set the headless mode
      /// \param[in] _headless Set to true to enable headless mode.
      public: void SetHeadlessRendering(const bool _headless);

      /// \brief Get the headless mode
      /// \return True if headless mode is enable, false otherwise.
      public: bool HeadlessRendering() const;

      /// \brief Set the render engine server plugin library.
      /// \param[in] _renderEngineServer File containing render engine library.
      public: void SetRenderEngineServer(
                  const std::string &_renderEngineServer);

      /// \brief Set the render engine server API backend.
      /// \param[in] _apiBackend See --render-engine-server-api-backend for
      /// possible options
      public: void SetRenderEngineServerApiBackend(
                  const std::string &_apiBackend);

      /// \return Api backend for server. See SetRenderEngineServerApiBackend()
      const std::string &RenderEngineServerApiBackend() const;

      /// \brief Set the render engine gui plugin library.
      /// \param[in] _renderEngineGui File containing render engine library.
      public: void SetRenderEngineGui(const std::string &_renderEngineGui);

      /// \brief Set the render engine gui API backend.
      /// \param[in] _apiBackend See --render-engine-gui-api-backend for
      /// possible options
      public: void SetRenderEngineGuiApiBackend(
                  const std::string &_apiBackend);

      /// \return Api backend for gui. See SetRenderEngineGuiApiBackend()
      public: const std::string &RenderEngineGuiApiBackend() const;

      /// \brief Set the server behavior when SDF errors are encountered while
      //// loading the server.
      /// \param[in] _behavior Server behavior when SDF errors are encounted.
      public: void SetBehaviorOnSdfErrors(SdfErrorBehavior _behavior);

      /// \brief Get the behavior when SDF errors are encountered while
      //// loading the server.
      /// \return Server behavior when SDF errors are encounted.
      public: SdfErrorBehavior BehaviorOnSdfErrors() const;

      /// \brief Instruct simulation to attach a plugin to a specific
      /// entity when simulation starts.
      /// \param[in] _info Information about the plugin to load.
      public: void AddPlugin(const PluginInfo &_info);

      /// \brief Add multiple plugins to the simulation
      /// \param[in] _plugins List of Information about the plugin to load.
      public: void AddPlugins(const std::list<PluginInfo> &_plugins);

      /// \brief Generate PluginInfo for Log recording based on the
      /// internal state of this ServerConfig object:
      /// \sa UseLogRecord
      /// \sa LogRecordPath
      /// \sa LogRecordResources
      /// \sa LogRecordCompressPath
      /// \sa LogRecordTopics
      public: PluginInfo LogRecordPlugin() const;

      /// \brief Generate PluginInfo for Log playback based on the
      /// internal state of this ServerConfig object:
      /// \sa LogPlaybackPath
      public: PluginInfo LogPlaybackPlugin() const;

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

      /// \brief Get the type of source
      /// \return The source type.
      public: SourceType Source() const;

      /// \brief Private data pointer
      private: std::unique_ptr<ServerConfigPrivate> dataPtr;
    };

    /// \brief Parse plugins from XML configuration file.
    /// \param[in] _fname Absolute path to the configuration file to parse.
    /// \return A list of all of the plugins found in the configuration file
    std::list<ServerConfig::PluginInfo>
    GZ_SIM_VISIBLE
    parsePluginsFromFile(const std::string &_fname);

    /// \brief Parse plugins from XML configuration string.
    /// \param[in] _str XML configuration content to parse
    /// \return A list of all of the plugins found in the configuration string.
    std::list<ServerConfig::PluginInfo>
    GZ_SIM_VISIBLE
    parsePluginsFromString(const std::string &_str);

    /// \brief Load plugin information, following ordering.
    ///
    /// This method is used when no plugins are found in an SDF
    /// file to load either a default or custom set of plugins.
    ///
    /// The following order is used to resolve:
    /// 1. Config file located at GZ_SIM_SERVER_CONFIG_PATH environment
    ///    variable.
    ///   * If GZ_SIM_SERVER_CONFIG_PATH is set but empty, no plugins
    ///     are loaded.
    /// 2. File at ${GZ_HOMEDIR}/.gz/sim/server.config
    /// 3. File at ${GZ_DATA_INSTALL_DIR}/server.config
    ///
    /// If any of the above files exist but are empty, resolution
    /// stops and the plugin list will be empty.
    ///
    //
    /// \param[in] _isPlayback Is the server in playback mode. If so, fallback
    /// to playback_server.config.
    //
    /// \return A list of plugins to load, based on above ordering
    std::list<ServerConfig::PluginInfo>
    GZ_SIM_VISIBLE
    loadPluginInfo(bool _isPlayback = false);
    }
  }
}

#endif
