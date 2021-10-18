
#ifndef IGNITION_GAZEBO_PYTHON__HELPER_SYSTEM_HPP_
#define IGNITION_GAZEBO_PYTHON__HELPER_SYSTEM_HPP_

#include <functional>

#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Export.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/ServerConfig.hh"

namespace ignition
{
namespace gazebo
{
  class Callback
  {
  public:
      Callback(){}

      virtual ~Callback(){}
      virtual void call_update(
        const ignition::gazebo::UpdateInfo &,
        const ignition::gazebo::EntityComponentManager &){}
      virtual void call_update_no_const(
        const ignition::gazebo::UpdateInfo &,
        ignition::gazebo::EntityComponentManager &){}
      virtual void call_configure_callback(
        const ignition::gazebo::Entity &,
        ignition::gazebo::EntityComponentManager &){
          std::cerr << "/* error message */" << '\n';
        }
  };

  class HelperFixturePrivate;

    class HelperFixture
    {
      /// \brief Constructor
      /// \param[in] _path Path to SDF file.
      public: explicit HelperFixture(const std::string &_path);

      /// \brief Constructor
      /// \param[in] _config Server config file
      public: explicit HelperFixture(const ServerConfig &_config);

      /// \brief Destructor
      public: virtual ~HelperFixture();

      public: void OnPostUpdate(Callback &_callback);

      public: void OnPreUpdate(Callback &_callback);

      public: void OnUpdate(Callback &_callback);

      public: void OnConfigure(Callback &_callback);

      /// \brief Finalize all the functions and add fixture to server.
      /// Finalize must be called before running the server, otherwise none of the
      /// `On*` functions will be called.
      /// The `OnConfigure` callback is called immediately on finalize.
      public: HelperFixture &Finalize();

      /// \brief Get pointer to underlying server.
      public: std::shared_ptr<ignition::gazebo::Server> Server() const;

      /// \internal
      /// \brief Pointer to private data.
      // TODO(chapulina) Use IGN_UTILS_UNIQUE_IMPL_PTR(dataPtr) when porting to v6
      private: std::shared_ptr<HelperFixturePrivate> dataPtr;
    };

  /// \brief System that is inserted into the simulation loop to observe the ECM.
  class HelperSystem :
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate,
    public ISystemUpdate,
    public ISystemPostUpdate
  {
  public:
    HelperSystem();
private:
    // Documentation inherited
    public: void Configure(
                  const Entity &_entity,
                  const std::shared_ptr<const sdf::Element> &_sdf,
                  ignition::gazebo::EntityComponentManager &_ecm,
                  ignition::gazebo::EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(const UpdateInfo &_info,
                  ignition::gazebo::EntityComponentManager &_ecm) override;

    // Documentation inherited
    public: void Update(const UpdateInfo &_info,
                  ignition::gazebo::EntityComponentManager &_ecm) override;

    // Documentation inherited
    public: void PostUpdate(const UpdateInfo &_info,
                  const ignition::gazebo::EntityComponentManager &_ecm) override;

    /// \brief Function to call every post-update
    Callback* callback_post_udpate = nullptr;

    /// \brief Function to call every pre-update
    Callback* callback_pre_udpate = nullptr;

    /// \brief Function to call every update
    Callback* callback_udpate = nullptr;

    /// \brief Function to call every time  we configure a world
    Callback* callback_configure = nullptr;
  };
}
}

#endif
