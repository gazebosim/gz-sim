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
#ifndef IGNITION_GAZEBO_COMPONENTS_ACTOR_HH_
#define IGNITION_GAZEBO_COMPONENTS_ACTOR_HH_

#include <ignition/msgs/actor.pb.h>

#include <chrono>
#include <string>

#include <sdf/Actor.hh>

#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/components/Serialization.hh>
#include <ignition/gazebo/Conversions.hh>
#include <ignition/gazebo/config.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace serializers
{
  using ActorSerializer =
      serializers::ComponentToMsgSerializer<sdf::Actor, msgs::Actor>;

  class AnimationTimeSerializer
  {
    /// \brief Serialization for `std::chrono::steady_clock::duration`.
    /// \param[in] _out Output stream.
    /// \param[in] _time Time to stream
    /// \return The stream.
    public: static std::ostream &Serialize(std::ostream &_out,
                const std::chrono::steady_clock::duration &_time)
    {
      _out << std::chrono::duration_cast<std::chrono::milliseconds>(
          _time).count();
      return _out;
    }

    /// \brief Deserialization for `std::chrono::steady_clock::duration`.
    /// \param[in] _in Input stream.
    /// \param[out] _time Time to populate
    /// \return The stream.
    public: static std::istream &Deserialize(std::istream &_in,
                std::chrono::steady_clock::duration &_time)
    {
      int64_t time;
      _in >> time;
      _time = std::chrono::duration<int64_t, std::milli>(time);
      return _in;
    }
  };
}

namespace components
{
  /// \brief This component contains actor source information. For more
  /// information on actors, see [SDF's Actor
  /// element](http://sdformat.org/spec?ver=1.6&elem=actor).
  using Actor =
      Component<sdf::Actor, class ActorTag, serializers::ActorSerializer>;
  IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.Actor", Actor)

  /// \brief Time in seconds within animation being currently played.
  using AnimationTime = Component<std::chrono::steady_clock::duration,
      class AnimationTimeTag, serializers::AnimationTimeSerializer>;
  IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.AnimationTime",
      AnimationTime)

  /// \brief Name of animation being currently played.
  using AnimationName = Component<std::string, class AnimationNameTag,
      serializers::StringSerializer>;
  IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.AnimationName",
      AnimationName)
}
}
}
}

#endif
