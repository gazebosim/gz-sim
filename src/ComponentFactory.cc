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

// Definition translation unit for all components shipped with gz-sim.
//
// Component headers only *declare* components (GZ_SIM_DECLARE_COMPONENT
// expands to the ADL typeId/typeName helpers). Defining
// GZ_SIM_COMPONENT_DEFINITION_TU before including them expands the static
// Factory registration objects here — once per library instead of
// once per consumer translation unit, which is what makes including
// component headers cheap. Putting these registrations in the same
// translation unit as Factory::Instance() also ensures static linkers
// always pull in this object file whenever Factory::Instance() is referenced.
#define GZ_SIM_COMPONENT_DEFINITION_TU

#include "gz/sim/components/Factory.hh"

#include "gz/sim/components/components.hh"

using Factory = gz::sim::components::Factory;

Factory *Factory::Instance()
{
  static gz::utils::NeverDestroyed<Factory> instance;
  return &instance.Access();
}
