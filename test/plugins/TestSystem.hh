#ifndef IGNITION_GAZEBO_TEST_TEST_SYSTEM_HH_
#define IGNITION_GAZEBO_TEST_TEST_SYSTEM_HH_

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/EntityQueryRegistrar.hh>
#include <ignition/gazebo/System.hh>

namespace ignition
{
  namespace gazebo
  {
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    class TestSystem: public System
    {
      public: TestSystem();

      public: virtual ~TestSystem();

      public: virtual void Init(EntityQueryRegistrar &_registrar) override final;
    };
    }
  }
}


#endif

