#include "ignition/gazebo/HelperFixture.hh"

#include <Python.h>

namespace ignition
{
namespace gazebo
{
  HelperSystem::HelperSystem()
  {
    // Py_Initialize();
    // PyEval_InitThreads();
    // PyEval_ReleaseLock();
  }

  /////////////////////////////////////////////////
  void HelperSystem::Configure(
                  const Entity &_entity,
                  const std::shared_ptr<const sdf::Element> &_sdf,
                  ignition::gazebo::EntityComponentManager &_ecm,
                  EventManager &_eventMgr)
  {
    if (callback_configure)
    {
      callback_configure->call_configure_callback(_entity, _ecm);
    }
  }

  /////////////////////////////////////////////////
  void HelperSystem::PreUpdate(const UpdateInfo &_info,
        ignition::gazebo::EntityComponentManager &_ecm)
  {
    if (callback_pre_udpate)
    {
      PyGILState_STATE state;
      state = PyGILState_Ensure();
      callback_pre_udpate->call_update_no_const(_info, _ecm);
      PyGILState_Release(state);
    }
  }

  /////////////////////////////////////////////////
  void HelperSystem::Update(const UpdateInfo &_info,
        ignition::gazebo::EntityComponentManager &_ecm)
  {
    if (callback_udpate)
    {
      PyGILState_STATE state;
      state = PyGILState_Ensure();
      callback_udpate->call_update_no_const(_info, _ecm);
      PyGILState_Release(state);
    }
  }

  /////////////////////////////////////////////////
  void HelperSystem::PostUpdate(const UpdateInfo &_info,
      const ignition::gazebo::EntityComponentManager &_ecm)
  {
    if (callback_post_udpate)
    {
      PyGILState_STATE state;
      state = PyGILState_Ensure();
      callback_post_udpate->call_update(_info, _ecm);
      PyGILState_Release(state);
    }
  }

  //////////////////////////////////////////////////
  class HelperFixturePrivate
  {
    /// \brief Initialize fixture
    /// \param[in] _config Server config
    public: void Init(const ServerConfig &_config);

    /// \brief Pointer to underlying server
    public: std::shared_ptr<gazebo::Server> server{nullptr};

    /// \brief Pointer to underlying Helper interface
    public: std::shared_ptr<HelperSystem> helperSystem{nullptr};

    /// \brief Flag to make sure Finalize is only called once
    public: bool finalized{false};
  };

  //////////////////////////////////////////////////
  HelperFixture::HelperFixture(const std::string &_path)
    : dataPtr(std::make_shared<HelperFixturePrivate>())
  {
    ServerConfig config;
    config.SetSdfFile(_path);
    this->dataPtr->Init(config);
  }

  //////////////////////////////////////////////////
  HelperFixture::HelperFixture(const ServerConfig &_config)
    : dataPtr(new HelperFixturePrivate())
  {
    this->dataPtr->Init(_config);
  }

  //////////////////////////////////////////////////
  HelperFixture::~HelperFixture()
  {
    dataPtr = nullptr;
  }

  //////////////////////////////////////////////////
  void HelperFixturePrivate::Init(const ServerConfig &_config)
  {
    this->helperSystem = std::make_shared<HelperSystem>();
    this->server = std::make_shared<gazebo::Server>(_config);
  }

  //////////////////////////////////////////////////
  HelperFixture &HelperFixture::Finalize()
  {
    if (this->dataPtr->finalized)
    {
      ignwarn << "Fixture has already been finalized, this only needs to be done"
              << " once." << std::endl;
      return *this;
    }

    this->dataPtr->server->AddSystem(this->dataPtr->helperSystem);

    this->dataPtr->finalized = true;
    return *this;
  }

  void HelperFixture::OnPostUpdate(Callback &_callback)
  {
    this->dataPtr->helperSystem->callback_post_udpate = &_callback;
  }

  void HelperFixture::OnPreUpdate(Callback &_callback)
  {
    this->dataPtr->helperSystem->callback_pre_udpate = &_callback;
  }

  void HelperFixture::OnUpdate(Callback &_callback)
  {
    this->dataPtr->helperSystem->callback_udpate = &_callback;
  }

  void HelperFixture::OnConfigure(Callback &_callback)
  {
    this->dataPtr->helperSystem->callback_configure = &_callback;
  }

  //////////////////////////////////////////////////
  std::shared_ptr<gazebo::Server> HelperFixture::Server() const
  {
    return this->dataPtr->server;
  }
}
}
