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
#ifndef GZ_SIM_TEST_TESTSYSTEM_HH_
#define GZ_SIM_TEST_TESTSYSTEM_HH_

#include <gz/sim/config.hh>
#include <gz/sim/System.hh>

namespace gz
{
  namespace sim
  {
    inline namespace GZ_SIM_VERSION_NAMESPACE {
    class TestSystem: public System
    {
      public: TestSystem();

      public: virtual ~TestSystem();
    };
    }
  }
}


#endif
