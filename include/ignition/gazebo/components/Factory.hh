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
#include <string>
#include <vector>
#include <ignition/gazebo/components/Component.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace components
{
  /// \typedef FactoryFn
  /// \brief Prototype for component factory generation
  using FactoryFn = std::unique_ptr<Component> (*)();

  /// \brief A factory that generates a component based on a string type.
  class IGNITION_GAZEBO_VISIBLE Factory
  {
    /// \brief Register a component.
    /// \param[in] _compType Type of component to register.
    /// \param[in] _factoryfn Function that generates the component.
    public: static void Register(const std::string &_compType,
                                 FactoryFn _factoryfn)
    {
      compMap[_compType] = _factoryfn;
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
      std::string type;
      const std::string kCompStr  = "ign_gazebo_components.";
      const std::string kCompStr1 = "ignition.gazebo.components.";
      const std::string kCompStr2 = ".ignition.gazebo.components.";
      // Convert "ignition.gazebo.components." to "ign_gazebo_components.".
      if (_compType.compare(0, kCompStr1.size(), kCompStr1) == 0)
      {
        type = kCompStr + _compType.substr(kCompStr1.size());
      }
      // Convert ".ignition.gazebo.components" to "ign_gazebo_components.".
      else if (_compType.compare(0, kCompStr2.size(), kCompStr2) == 0)
      {
        type = kCompStr + _compType.substr(kCompStr2.size());
      }
      else
      {
        // Fix typenames that are missing "ign_gazebo_components."
        // at the beginning.
        if (_compType.compare(0, kCompStr.size(), kCompStr) != 0)
          type = kCompStr;
        type += _compType;
      }

      // Create a new component if a FactoryFn has been assigned to this type.
      std::unique_ptr<Component> comp;
      auto it = compMap.find(type);
      if (it != compMap.end())
        comp = it->second();

      return comp;
    }

    /// \brief Get all the component types.
    /// \param[out] _types Vector of strings of the component types.
    public: static std::vector<std::string> Components()
    {
      std::vector<std::string> types;

      // Return the list of all known component types.
      for (const auto & [name, funct] : compMap)
        types.push_back(name);

      return types;
    }

    /// \brief A list of registered component types
    private: inline static std::map<std::string, FactoryFn> compMap;
  };

  /// \brief Static component registration macro.
  ///
  /// Use this macro to register components.
  /// \param[in] _compType Component type name.
  /// \param[in] _classname Class name for component.
  #define IGN_GAZEBO_REGISTER_COMPONENT(_compType, _classname) \
  IGNITION_GAZEBO_VISIBLE \
  std::unique_ptr<ignition::gazebo::components::Component> New##_classname() \
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
