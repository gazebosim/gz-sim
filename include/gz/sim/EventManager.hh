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
#ifndef GZ_SIM_EVENTMANAGER_HH_
#define GZ_SIM_EVENTMANAGER_HH_

#include <functional>
#include <memory>
#include <typeinfo>
#include <unordered_map>
#include <utility>

#include <gz/common/Console.hh>
#include <gz/common/Event.hh>

#include <gz/sim/config.hh>
#include <gz/sim/Export.hh>
#include <gz/sim/Types.hh>

namespace gz
{
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE {
    // Forward declarations.
    class GZ_SIM_HIDDEN EventManagerPrivate;

    /// \brief The EventManager is used to send/receive notifications of
    /// simulator events.
    ///
    /// The simulator environment and corresponding systems can either connect
    /// to an Event or emit an Event as needed to signal actions that need to
    /// occur.
    ///
    /// See \ref gz::sim::events for a complete list of events.

    /// TODO: if visibility is added here the MSVC is unable to compile it.
    /// The use of smart pointer inside the unordered_map (events method) is
    /// the cause of it. Maybe a compiler bug?
    class EventManager
    {
      /// \brief Constructor
      public: EventManager() = default;

      public: EventManager(const EventManager &) = delete;
      public: EventManager &operator=(const EventManager &) = delete;

      /// \brief Destructor
      public: ~EventManager() = default;

      /// \brief Add a connection to an event.
      /// \param[in] _subscriber A std::function callback function. The function
      ///   signature must match that of the event (template parameter E).
      /// \return A Connection pointer, which will automatically call
      /// Disconnect when it goes out of scope.
      public: template <typename E>
              gz::common::ConnectionPtr
              Connect(const typename E::CallbackT &_subscriber)
              {
                if (this->events.find(typeid(E)) == this->events.end()) {
                  this->events[typeid(E)] = std::make_unique<E>();
                }

                E *eventPtr = dynamic_cast<E *>(this->events[typeid(E)].get());
                // All values in the map should be derived from Event,
                // so this shouldn't be an issue, but it doesn't hurt to check.
                if (eventPtr != nullptr)
                {
                  return eventPtr->Connect(_subscriber);
                }
                else
                {
                  gzerr << "Failed to connect event: "
                    << typeid(E).name() << std::endl;
                  return nullptr;
                }
              }

      /// \brief Emit an event signal to connected subscribers.
      /// \param[in] _args function arguments to be passed to the event
      /// callbacks. Must match the signature of the event type E.
      public: template <typename E, typename ... Args>
              void Emit(Args && ... _args)
              {
                if (this->events.find(typeid(E)) == this->events.end())
                {
                  // If there are no events of type E in the map, create it.
                  // But it also means there is nothing to signal.
                  //
                  // This is also needed to suppress unused function warnings
                  // for Events that are purely emitted, with no connections.
                  this->events[typeid(E)] = std::make_unique<E>();
                  return;
                }

                E *eventPtr = dynamic_cast<E *>(this->events[typeid(E)].get());
                // All values in the map should be derived from Event,
                // so this shouldn't be an issue, but it doesn't hurt to check.
                if (eventPtr != nullptr)
                {
                  eventPtr->Signal(std::forward<Args>(_args) ...);
                }
                else
                {
                  gzerr << "Failed to signal event: "
                    << typeid(E).name() << std::endl;
                }
              }

      /// \brief Get connection count for a particular event
      /// Connection count for the event
      public: template <typename E>
              unsigned int
              ConnectionCount()
              {
                if (this->events.find(typeid(E)) == this->events.end())
                {
                  return 0u;
                }

                E *eventPtr = dynamic_cast<E *>(this->events[typeid(E)].get());
                // All values in the map should be derived from Event,
                // so this shouldn't be an issue, but it doesn't hurt to check.
                if (eventPtr != nullptr)
                {
                  return eventPtr->ConnectionCount();
                }
                else
                {
                  gzerr << "Failed to get connection count for event: "
                    << typeid(E).name() << std::endl;
                  return 0u;
                }
              }

      /// \brief Convenience type for storing typeinfo references.
      private: using TypeInfoRef = std::reference_wrapper<const std::type_info>;

      /// \brief Hash functor for TypeInfoRef
      private: struct Hasher
               {
                 std::size_t operator()(TypeInfoRef _code) const
                 {
                   return _code.get().hash_code();
                 }
               };

      /// \brief Equality functor for TypeInfoRef
      private: struct EqualTo
               {
                 bool operator()(TypeInfoRef _lhs, TypeInfoRef _rhs) const
                 {
                   return _lhs.get() == _rhs.get();
                 }
               };

      /// \brief Container of used signals.
      private: std::unordered_map<TypeInfoRef,
                                  std::unique_ptr<gz::common::Event>,
                                  Hasher, EqualTo> events;
    };
    }
  }
}

#endif  // GZ_SIM_EVENTMANAGER_HH_
