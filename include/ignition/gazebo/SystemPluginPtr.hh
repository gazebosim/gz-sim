#ifndef IGNITION_GAZEBO_SYSTEMPLUGINPTR_HH_
#define IGNITION_GAZEBO_SYSTEMPLUGINPTR_HH_

#include <ignition/plugin/SpecializedPluginPtr.hh>
#include <ignition/gazebo/System.hh>

namespace ignition
{
  namespace gazebo
  {
    using SystemPluginPtr = ignition::plugin::SpecializedPluginPtr<
      System, ISystemPreUpdate, ISystemUpdate, ISystemPostUpdate>;
  }
}


#endif

