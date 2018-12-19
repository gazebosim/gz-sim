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
#ifndef IGNITION_GAZEBO_COMPONENTS_TAGWRAPPER_HH_
#define IGNITION_GAZEBO_COMPONENTS_TAGWRAPPER_HH_

#include <memory>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace components
{
  /// \brief This class to be used to create simple components that represent
  /// just a "tag", while avoiding a lot of boilerplate code. The Identifier
  /// must be a unique type so that type aliases can be used to create new
  /// components. However the type does not need to be defined anywhere eg.
  ///
  ///     using Joint = TagWrapper<class JointTag>;
  ///
  template <typename Identifier>
  class TagWrapper
  {
    /// \brief Equality operator, always returns true, since tags don't have
    /// data.
    /// \param[in] _tagWrapper TagWrapper to compare to.
    /// \return True if equal.
    public: bool operator==(const TagWrapper &_tagWrapper) const;

    /// \brief Inequality operator, always returns false, since tags don't have
    /// data.
    /// \param[in] _tagWrapper TagWrapper to compare to.
    /// \return True if different.
    public: bool operator!=(const TagWrapper &_tagWrapper) const;
  };

  //////////////////////////////////////////////////
  template <typename Identifier>
  bool TagWrapper<Identifier>::operator==(const TagWrapper<Identifier> &) const
  {
    return true;
  }

  //////////////////////////////////////////////////
  template <typename Identifier>
  bool TagWrapper<Identifier>::operator!=(const TagWrapper<Identifier> &) const
  {
    return false;
  }
}
}
}
}
#endif
