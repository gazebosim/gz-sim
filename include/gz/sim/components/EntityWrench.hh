/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#ifndef GZ_SIM_COMPONENTS_ENTITYWRENCH_HH_
#define GZ_SIM_COMPONENTS_ENTITYWRENCH_HH_

#include <string>
#include <unordered_map>

#include <gz/msgs/entity_wrench.pb.h>
#include <gz/msgs/entity_wrench_map.pb.h>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Serialization.hh>
#include <gz/sim/config.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace serializers
{
  class EntityWrenchesSerializer
  {
    /// \brief Serialization for `unordered_map<string, msgs::EntityWrench>`
    /// \param[in] _out Output stream.
    /// \param[in] _data The data to stream.
    /// \return The stream.
    public: static std::ostream &Serialize(std::ostream &_out,
      const std::unordered_map<std::string, msgs::EntityWrench> &_data)
    {
      _out << _data.size();
      for (const auto& [key, value] : _data)
      {
        // serialize to string so we can determine size
        std::string str = value.SerializeAsString();
        _out << " " << key;
        _out << " " << str.size();
        _out  << str;

        // debug info
        // gzdbg << value.DebugString() << "\n";
      }
      return _out;
    }

    /// \brief Deserialization for `unordered_set<string, msgs::EntityWrench>`
    /// \param[in] _in Input stream.
    /// \param[out] _data The data to populate.
    /// \return The stream.
    public: static std::istream &Deserialize(std::istream &_in,
      std::unordered_map<std::string, msgs::EntityWrench> &_data)
    {
      size_t size;
      _in >> size;
      for (size_t i = 0; i < size; ++i)
      {
        // read key and msg size
        std::string key;
        size_t str_size;

        _in >> key;
        _in >> str_size;

        // construct string buffer to required size
        std::string str(str_size, '\0');
        _in.read(&str[0], str_size);

        // parse message
        msgs::EntityWrench msg;
        msg.ParseFromString(str);
        _data.insert({key, msg});

        // debug info
        // gzdbg << msg.DebugString() << "\n";
      }
      return _in;
    }
  };
}

namespace components
{
  /// \brief A component type that contains a wrench and an entity expressed
  /// in the world frame to apply the wrench to. It is represented by
  /// gz::msgs::EntityWrench.
  /// The wrench uses SI units (N for force and N⋅m for torque).
  using EntityWrench =
      Component<msgs::EntityWrench, class EntityWrenchTag,
        serializers::MsgSerializer>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.EntityWrench",
                            EntityWrench)

  /// \brief A component type that contains an map of wrenches
  /// for an entity. The wrenches are represented by gz::msgs::EntityWrenchMap.
  using EntityWrenchMap =
      Component<msgs::EntityWrenchMap, class EntityWrenchMapTag,
          serializers::MsgSerializer>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.EntityWrenchMap",
                            EntityWrenchMap)

  /// \brief A component type that contains an unordered map of wrenches
  /// for an entity. The wrenches are represented by gz::msgs::EntityWrench.
  /// The key is a string label.
  using EntityWrenches =
      Component<std::unordered_map<std::string, msgs::EntityWrench>,
          class EntityWrenchesTag,
          serializers::EntityWrenchesSerializer>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.EntityWrenches",
                            EntityWrenches)

}
}
}  // namespace sim
}  // namespace gz

#endif  // GZ_SIM_COMPONENTS_ENTITYWRENCH_HH_
