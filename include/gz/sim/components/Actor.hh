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
#ifndef GZ_SIM_COMPONENTS_ACTOR_HH_
#define GZ_SIM_COMPONENTS_ACTOR_HH_

#include <gz/msgs/actor.pb.h>
#include <gz/msgs/pose_v.pb.h>

#include <chrono>
#include <map>
#include <ratio>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/msgs/Utility.hh>
#include <sdf/Actor.hh>

#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Serialization.hh>
#include <gz/sim/Conversions.hh>
#include <gz/sim/config.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
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
      _out << std::chrono::duration_cast<std::chrono::nanoseconds>(
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
      _time = std::chrono::duration<int64_t, std::nano>(time);
      return _in;
    }
  };

  /// \brief Serializer for a map of bone names to poses, streamed as a
  /// msgs::Pose_V whose poses carry the bone name.
  class BoneTransformsSerializer
  {
    /// \brief Serialization for `std::map<std::string, math::Pose3d>`.
    /// \param[in] _out Output stream.
    /// \param[in] _transforms Bone transforms to stream
    /// \return The stream.
    public: static std::ostream &Serialize(std::ostream &_out,
                const std::map<std::string, math::Pose3d> &_transforms)
    {
      msgs::Pose_V msg;
      for (const auto &[name, pose] : _transforms)
      {
        auto *poseMsg = msg.add_pose();
        poseMsg->set_name(name);
        msgs::Set(poseMsg, pose);
      }
      msg.SerializeToOstream(&_out);
      return _out;
    }

    /// \brief Deserialization for `std::map<std::string, math::Pose3d>`.
    /// \param[in] _in Input stream.
    /// \param[out] _transforms Bone transforms to populate
    /// \return The stream.
    public: static std::istream &Deserialize(std::istream &_in,
                std::map<std::string, math::Pose3d> &_transforms)
    {
      msgs::Pose_V msg;
      msg.ParseFromIstream(&_in);
      _transforms.clear();
      for (const auto &poseMsg : msg.pose())
        _transforms[poseMsg.name()] = msgs::Convert(poseMsg);
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
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.Actor", Actor)

  /// \brief Time in seconds within animation being currently played.
  using AnimationTime = Component<std::chrono::steady_clock::duration,
      class AnimationTimeTag, serializers::AnimationTimeSerializer>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.AnimationTime",
      AnimationTime)

  /// \brief Name of animation being currently played.
  using AnimationName = Component<std::string, class AnimationNameTag,
      serializers::StringSerializer>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.AnimationName",
      AnimationName)

  /// \brief Local transforms of the actor's skeleton bones, keyed by bone
  /// name and given relative to each bone's parent. When present, the
  /// skeleton is posed from these transforms directly instead of playing the
  /// animation selected by AnimationName and AnimationTime, which lets a
  /// system drive every bone itself.
  using BoneTransforms = Component<std::map<std::string, math::Pose3d>,
      class BoneTransformsTag, serializers::BoneTransformsSerializer>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.BoneTransforms",
      BoneTransforms)
}
}
}
}

#endif
