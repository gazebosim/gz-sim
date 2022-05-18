/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <gtest/gtest.h>

#include <ignition/sim/System.hh>
#include <ignition/utils/SuppressWarning.hh>

IGN_UTILS_WARN_IGNORE__DEPRECATED_DECLARATION

/////////////////////////////////////////////////
// Make sure the ignition namespace still works
TEST(Deprecated, IgnitionNamespace)
{
  ignition::sim::System system;
}

IGN_UTILS_WARN_RESUME__DEPRECATED_DECLARATION
