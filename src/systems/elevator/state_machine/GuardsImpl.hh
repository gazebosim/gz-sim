/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

/*
 * \author Nick Lamprianidis <nlamprian@gmail.com>
 * \date January 2021
 */

#include <mutex>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE
{
namespace systems
{
/// \brief Guard that checks whether the cabin is at the target floor level.
/// \note The template parameter can invert the result if set to true.
template <bool invert>
class CabinAtTargetGuard
{
  /// \brief Function call operator
  /// \param[in] _fsm State machine with which the guard is associated
  public: template <typename Event, typename Fsm, typename Source,
                    typename Target>
  bool operator()(const Event &, Fsm &_fsm, Source &, Target &)
  {
    const auto &data = _fsm.Data();
    std::lock_guard<std::mutex> lock(data->system->mutex);
    bool at_target = data->targets.front() == data->system->state;
    return at_target ^ invert;
  }
};

/// \brief Guard that checks whether the target queue is empty.
/// \note The template parameter can invert the result if set to true.
template <bool invert>
class NoTargetGuard
{
  /// \brief Function call operator
  /// \param[in] _fsm State machine with which the guard is associated
  public: template <typename Event, typename Fsm, typename Source,
                    typename Target>
  bool operator()(const Event &, Fsm &_fsm, Source &, Target &)
  {
    const auto &data = _fsm.Data();
    std::lock_guard<std::mutex> lock(data->system->mutex);
    return data->targets.empty() ^ invert;
  }
};

}  // namespace systems
}  // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
}  // namespace gazebo
}  // namespace ignition
