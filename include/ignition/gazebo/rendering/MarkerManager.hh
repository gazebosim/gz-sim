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
#ifndef IGNITION_RENDERING_MARKERMANAGER_HH_
#define IGNITION_RENDERING_MARKERMANAGER_HH_

#include <memory>

#include "ignition/rendering/RenderTypes.hh"

namespace ignition
{
  namespace rendering
  {
    // Forwared declare private data class.
    class MarkerManagerPrivate;

    /// \cond
    /// \brief Creates, deletes, and maintains marker visuals. Only the
    /// Scene class should instantiate and use this class.
    class MarkerManager
    {
      /// \brief Constructor
      public: MarkerManager();

      /// \brief Destructor
      public: virtual ~MarkerManager();

      /// \brief Initialize the marker manager.
      /// \param[in] _scene Reference to the scene.
      /// \return True on success
      private: bool Init(Scene *_scene);

      /// \internal
      /// \brief Private data pointer
      private: std::unique_ptr<MarkerManagerPrivate> dataPtr;

      /// Make sure the Scene can access Init()
      private: friend class Scene;
    };
    /// \endcond
  }
}
#endif
