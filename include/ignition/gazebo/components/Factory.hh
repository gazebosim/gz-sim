/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_COMPONENTS_FACTORY_HH_
#define IGNITION_GAZEBO_COMPONENTS_FACTORY_HH_

#include <map>
#include <memory>
#include <vector>
#include <ignition/gazebo/Component.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace components
{
  //
  /// \typedef FactoryFn
  /// \brief Prototype for component factory generation
  typedef std::unique_ptr<Component> (*FactoryFn) ();

  /// \class ToDo.
  class IGNITION_GAZEBO_VISIBLE Factory
  {
    /// \brief Register a component.
    /// \param[in] _compType Type of component to register.
    /// \param[in] _factoryfn Function that generates the component.
    public: static void Register(const std::string &_compType,
                                 FactoryFn _factoryfn)
    {
      // Create the compMap if it's null
      if (!compMap)
        compMap = new std::map<std::string, FactoryFn>;

      (*compMap)[_compType] = _factoryfn;
    }

    /// \brief Create a new instance of a component.
    /// \param[in] _compType Type of component to create.
    /// \return Pointer to a component. Null if the component
    /// type could not be handled.
    public: template<typename T>
            static std::unique_ptr<T> New(const std::string &_compType)
            {
              return std::unique_ptr<T>(
                  static_cast<T*>(New(_compType).release()));
            }

    /// \brief Create a new instance of a component.
    /// \param[in] _compType Type of component to create.
    /// \return Pointer to a component. Null if the component
    /// type could not be handled.
    public: static std::unique_ptr<Component> New(const std::string &_compType)
    {
      std::unique_ptr<Component> comp;

      std::string type;
      // Convert "ignition.gazebo.components." to "ign_gazebo_components.".
      if (_compType.find("ignition.gazebo.components.") == 0)
      {
        type = "ign_gazebo_components." + _compType.substr(27);
      }
      // Convert ".ignition.msgs." to "ign_msgs.".
      else if (_compType.find(".ignition.gazebo.components.") == 0)
      {
        type = "ign_gazebo_components." + _compType.substr(28);
      }
      else
      {
        // Fix typenames that are missing "ign_gazebo_components." at the beginning.
        if (_compType.find("ign_gazebo_components.") != 0)
          type = "ign_gazebo_components.";
        type += _compType;
      }

      // Create a new message if a FactoryFn has been assigned to the message type
      if (compMap->find(type) != compMap->end())
        comp = ((*compMap)[type]) ();

      return comp;
    }

    /// \brief Get all the component types
    /// \param[out] _types Vector of strings of the component types.
    public: static void Types(std::vector<std::string> &_types);

    /// \brief A list of registered component types
    private: inline static std::map<std::string, FactoryFn> *compMap = nullptr;
  };

  // Initialization of static members,
  //std::map<std::string, FactoryFn> *Factory::compMap = NULL;

  /// \brief Static message registration macro
  ///
  /// Use this macro to register messages.
  /// \param[in] _compType Component type name.
  /// \param[in] _classname Class name for message.
  #define IGN_GAZEBO_REGISTER_COMPONENT(_compType, _classname) \
  IGNITION_GAZEBO_VISIBLE \
  std::unique_ptr<ignition::gazebo::Component> New##_classname() \
  { \
    return std::unique_ptr<ignition::gazebo::components::_classname>(\
        new ignition::gazebo::components::_classname); \
  } \
  class IGNITION_GAZEBO_VISIBLE IgnGazeboComponents##_classname \
  { \
    public: IgnGazeboComponents##_classname() \
    { \
      ignition::gazebo::components::Factory::Register(\
        _compType, New##_classname);\
    } \
  }; \
  static IgnGazeboComponents##_classname\
    IgnitionGazeboComponentsInitializer##_classname;
  }
}
}
}
#endif
