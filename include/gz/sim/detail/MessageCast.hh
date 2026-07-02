/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
#ifndef GZ_SIM_DETAIL_MESSAGE_CAST_HH_
#define GZ_SIM_DETAIL_MESSAGE_CAST_HH_

#include <google/protobuf/message_lite.h>

namespace gz {
namespace sim {
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace detail {
namespace {

// A helper to do a dynamic cast from a generic proto message to a concrete
// type. In newer versions of protobuf, the helper `DynamicCastMessage` is
// available and should be used.
template <typename T>
const T* DoDynamicCastMessage(const google::protobuf::MessageLite* message)
{
#if GOOGLE_PROTOBUF_VERSION >= 6030000
  return google::protobuf::DynamicCastMessage<T>(message);
#else
  return dynamic_cast<const T*>(message);
#endif
}

// A helper to do a dynamic cast from a generic proto message to a concrete
// type. In newer versions of protobuf, the helper `DynamicCastMessage` is
// available and should be used.
template <typename T>
T* DoDynamicCastMessage(google::protobuf::MessageLite* message)
{
#if GOOGLE_PROTOBUF_VERSION >= 6030000
  return google::protobuf::DynamicCastMessage<T>(message);
#else
  return dynamic_cast<T*>(message);
#endif
}

// A helper to do a dynamic pointer cast from a shared_ptr to a generic proto
// message to a concrete type. In newer versions of protobuf, the helper
// `DynamicCastMessage` is available and should be used.
template <typename T>
std::shared_ptr<T> DoDynamicCastMessage(
    std::shared_ptr<google::protobuf::MessageLite> message)
{
#if GOOGLE_PROTOBUF_VERSION >= 6030000
  return google::protobuf::DynamicCastMessage<T>(std::move(message));
#else
  return std::dynamic_pointer_cast<T>(std::move(message));
#endif
}

}  // namespace
}  // namespace detail
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

#endif  // GZ_SIM_DETAIL_MESSAGE_CAST_HH_
