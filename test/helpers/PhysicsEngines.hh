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
#ifndef GZ_SIM_TEST_HELPERS_PHYSICSENGINES_HH_
#define GZ_SIM_TEST_HELPERS_PHYSICSENGINES_HH_

/// \brief Declares the physics engines that the tests in a file are known to
/// pass with, e.g.
///
/// ```
/// GZ_SIM_TEST_PHYSICS_ENGINES("dartsim", "bullet-featherstone");
/// ```
///
/// Each name matches a gz-physics engine plugin, e.g. bullet-featherstone for
/// gz-physics-bullet-featherstone-plugin. CMake reads this line from the source
/// file and runs the test once per engine, so write it on a single line at the
/// start of the line, at file scope. Files without it only run with DART, the
/// default engine.
#define GZ_SIM_TEST_PHYSICS_ENGINES(...) static_assert(true, "")

#endif
