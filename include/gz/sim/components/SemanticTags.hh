/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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
#ifndef GZ_SIM_COMPONENTS_SEMANTIC_TAGS_HH_
#define GZ_SIM_COMPONENTS_SEMANTIC_TAGS_HH_

#include <gz/msgs/stringmsg_v.pb.h>

#include <istream>
#include <ostream>
#include <string>
#include <vector>

#include "gz/sim/components/Component.hh"
#include "gz/sim/components/Factory.hh"
#include "gz/sim/config.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE
{
namespace serializers
{
class SemanticTagsSerializer
{
  public:
  static std::ostream &Serialize(std::ostream &_out,
                                 const std::vector<std::string> &_tags)
  {
    msgs::StringMsg_V msg;
    for (const auto &tag : _tags)
    {
      msg.add_data(tag);
    }
    msg.SerializeToOstream(&_out);
    return _out;
  }

  public:
  static std::istream &Deserialize(std::istream &_in,
                                   std::vector<std::string> &_tags)
  {
    msgs::StringMsg_V msg;
    msg.ParsePartialFromIstream(&_in);
    _tags.clear();
    for (const auto &item : msg.data())
    {
      _tags.push_back(item);
    }
    return _in;
  }
};
}  // namespace serializers
//
namespace components
{

/// \brief A component that holds the semantic tag of an entity. This is
/// meant to be used by systems such as EntitySemantics to assign tags to
/// entities. See
/// https://github.com/ros-simulation/simulation_interfaces/blob/1.0.0/msg/TagsFilter.msg
using SemanticTags = Component<std::vector<std::string>, class SemanticTagsTag,
                              serializers::SemanticTagsSerializer>;
GZ_SIM_REGISTER_COMPONENT("gz_sim_components.SemanticTags", SemanticTags)
}  // namespace components
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz
#endif
