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
/// file and runs the test once per engine, so write it on a single line at
/// file scope. Unknown engine names are a CMake error. Files without it only
/// run with DART, the default engine.
///
/// Each run selects its engine with the `_GZ_SIM_TEST_PHYSICS_ENGINE`
/// environment variable, e.g.
/// `_GZ_SIM_TEST_PHYSICS_ENGINE=gz-physics-bullet-featherstone-plugin`. The
/// Physics system only uses it when neither the server configuration nor the
/// SDF chooses an engine, and logs a message when it's ignored. The variable is
/// for these tests only, it isn't a supported interface.
///
/// Each run is labeled with its engine name. To run the tests of a single
/// engine, select its label, anchored so that `bullet` doesn't also match
/// `bullet-featherstone`:
///
/// ```
/// ctest -L '^bullet-featherstone$'
/// ```
///
/// Setting `_GZ_SIM_TEST_PHYSICS_ENGINE` before running `ctest` doesn't
/// restrict the tests to one engine: the runs of the files that declare their
/// engines set the variable themselves, while the other tests inherit it from
/// the shell. This is handy to try a new engine with the files that don't
/// declare it yet, and then add it to the declaration of those that pass:
///
/// ```
/// _GZ_SIM_TEST_PHYSICS_ENGINE=gz-physics-mujoco-plugin \
///   ctest -LE '^(dartsim|bullet-featherstone)$'
/// ```
#define GZ_SIM_TEST_PHYSICS_ENGINES(...) static_assert(true)

#endif
