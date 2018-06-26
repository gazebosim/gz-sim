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
#ifndef IGNITION_GAZEBO_TMPIFACE_HH_
#define IGNITION_GAZEBO_TMPIFACE_HH_

#include <memory>
#include <ignition/msgs.hh>

#include "ignition/gazebo/Export.hh"

namespace ignition
{
  namespace gazebo
  {
    class TmpIfacePrivate;

    /// \brief Temporary place to prototype transport interfaces while it's not
    /// clear where they will live.
    ///
    /// Move API from here to their appropriate locations once that's known.
    ///
    /// This class should be removed before releasing!
    class IGNITION_GAZEBO_VISIBLE TmpIface
    {
      /// \brief Constructor: advertize services and topics
      public: TmpIface();

      /// \brief Destructor
      public: ~TmpIface() = default;

      /// \brief World control service callback
      /// \param[in] _req Request
      /// \param[out] _res Response
      /// \return True for success
      private: bool OnWorldControl(const msgs::WorldControl &_req,
                                         msgs::Boolean &_res);

      /// \brief Server control service callback
      /// \param[in] _req Request
      /// \param[out] _res Response
      /// \return True for success
      private: bool OnServerControl(const msgs::ServerControl &_req,
                                          msgs::Boolean &_res);

      /// \brief Transport service that responds with the scene graph.
      /// \param[out] _rep Scene reply message.
      /// \return True if the service successfully completed.
      private: bool SceneService(ignition::msgs::Scene &_rep);

      /// \internal
      /// \brief Private data pointer
      private: std::unique_ptr<TmpIfacePrivate> dataPtr;
    };
  }
}
#endif
