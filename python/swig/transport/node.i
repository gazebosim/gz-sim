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

%module node
%{
#include <ignition/transport/Node.hh>
%}


namespace ignition
{
namespace transport
{
  class Node {
    public: Node();
    public: ~Node();

    public: template<typename MessageT>
    bool Subscribe(
        const std::string &_topic,
        std::function<void(const MessageT &_msg)> &_callback,
        const SubscribeOptions &_opts = SubscribeOptions());

  };
}
}
