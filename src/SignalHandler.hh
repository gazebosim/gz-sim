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
#ifndef IGNITION_GAZEBO_SIGNALHANDLER_HH_
#define IGNITION_GAZEBO_SIGNALHANDLER_HH_

#include <functional>
#include <vector>
#include <ignition/common/SingletonT.hh>

namespace ignition
{
  namespace gazebo
  {
    class SignalHandler
    {
      /// \brief Constructor
      public: SignalHandler();

      /// \brief Destructor.
      public: ~SignalHandler() = default;

      /// \brief Get whether the signal handlers were successfully
      /// initialized.
      /// \return True if the signal handlers were successfully created.
      public: bool Initialized() const;

      /// \brief Add a callback to execute when a signal is received.
      /// \param[in] _cb Callback to execute.
      public: void AddCallback(std::function<void(int)> _cb);

      /// \brief Handle a signal.
      /// \param[in] _sig Signal number
      private: void OnSignal(int _sig);

      /// \brief True if signal handlers were successfully initialized.
      private: bool initialized = false;

      /// \brief the callbacks to execute when a signal is received.
      private: std::vector<std::function<void(int)>> callbacks;
    };
  }
}

#endif
