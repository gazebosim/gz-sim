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

%module subscribeoptions
%{
#include "ignition/transport/config.hh"
#include "ignition/transport/Export.hh"
%}

namespace ignition
{
  namespace transport
  {
    class SubscribeOptions
    {
      public: SubscribeOptions();

      public: SubscribeOptions(const SubscribeOptions &_otherSubscribeOpts);

      public: ~SubscribeOptions();

      public: bool Throttled() const;

      public: void SetMsgsPerSec(const uint64_t _newMsgsPerSec);

      public: uint64_t MsgsPerSec() const;
  };
}
}
