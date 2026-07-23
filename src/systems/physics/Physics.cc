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

#include "PhysicsPrivate.hh"
#include <gz/plugin/Register.hh>

//////////////////////////////////////////////////
Physics::Physics() : System(), dataPtr(std::make_unique<PhysicsPrivate>())
{
}

//////////////////////////////////////////////////
System::PriorityType Physics::ConfigurePriority()
{
  // Use constant from System.hh
  return ::gz::sim::systems::kPhysicsPriority;
}

//////////////////////////////////////////////////
void Physics::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
{
  std::string pluginLib;

  // 1. Engine from component (from command line / ServerConfig)
  auto engineComp = _ecm.Component<components::PhysicsEnginePlugin>(_entity);
  if (engineComp && !engineComp->Data().empty())
  {
    pluginLib = engineComp->Data();
  }
  // 2. Engine from SDF
  else if (_sdf->HasElement("engine"))
  {
    auto sdfClone = _sdf->Clone();
    auto engineElem = sdfClone->GetElement("engine");
    pluginLib = engineElem->Get<std::string>("filename", pluginLib).first;
  }

  // 3. Use DART by default
  if (pluginLib.empty())
  {
    pluginLib = "gz-physics-dartsim-plugin";
  }

  // Update component
  if (!engineComp)
  {
    _ecm.CreateComponent(_entity, components::PhysicsEnginePlugin(pluginLib));
  }
  else
  {
    engineComp->SetData(pluginLib,
        [](const std::string &_a, const std::string &_b){return _a == _b;});
  }

  // Check if entity names should be populated in contact points.
  auto contactsElement = _sdf->FindElement("contacts");
  if (contactsElement)
  {
    this->dataPtr->contactsEntityNames = contactsElement->Get<bool>(
      "include_entity_names", true).first;
  }

  // Check if fixed constraints should be enforced.
  this->dataPtr->enforceFixedConstraint =
      _sdf->Get<bool>("enforce_fixed_constraint",
      this->dataPtr->enforceFixedConstraint).first;

  plugin::Loader pluginLoader;
  if (isStaticPlugin(pluginLib))
  {
    const size_t prefixLen = staticPluginPrefixStr().size();
    const std::string pluginToInstantiate =
        pluginLib.substr(prefixLen);
    auto plugin = pluginLoader.Instantiate(pluginToInstantiate);
    if (!plugin)
    {
       gzerr << "Failed to load physics engine plugin: "
             << "(Reason: static plugin registry does not contain the "
             << "requested plugin)\n"
             << "- Requested plugin name: [" << pluginLib << "]\n";
      return;
    }

    this->dataPtr->engine = physics::RequestEngine<
      physics::FeaturePolicy3d,
      PhysicsPrivate::MinimumFeatureList>::From(plugin);

    if (!this->dataPtr->engine)
    {
      gzerr << "Failed to load physics engine from static plugin registry: "
            << "(Reason: Physics engine does not meet the minimum features "
            << "requirement)\n"
            << "- Requested plugin name: [" << pluginLib << "]\n";
      return;
    }
    gzdbg << "Loaded [" << pluginLib <<"] from the static plugin registry"
          << std::endl;
  }
  else
  {
    // Find engine shared library
    // Look in:
    // * Paths from environment variable
    // * Engines installed with gz-physics
    common::SystemPaths systemPaths;
    systemPaths.SetPluginPathEnv(this->dataPtr->pluginPathEnv);
    systemPaths.AddPluginPaths(gz::physics::getEngineInstallDir());

    auto pathToLib = systemPaths.FindSharedLibrary(pluginLib);

    if (pathToLib.empty())
    {
      gzerr << "Failed to find plugin [" << pluginLib
      << "]. Have you checked the " << this->dataPtr->pluginPathEnv
      << " environment variable?" << std::endl;

      return;
    }

    // Load engine plugin
    auto plugins = pluginLoader.LoadLib(pathToLib);
    if (plugins.empty())
    {
      gzerr << "Unable to load the [" << pathToLib << "] library.\n";
      return;
    }

    auto classNames = pluginLoader.PluginsImplementing<
        physics::ForwardStep::Implementation<
        physics::FeaturePolicy3d>>();
    if (classNames.empty())
    {
      gzerr << "No physics plugins implementing required interface found in "
            << "library [" << pathToLib << "]." << std::endl;
      return;
    }

    // Get the first plugin that works
    for (auto className : classNames)
    {
      auto plugin = pluginLoader.Instantiate(className);

      if (!plugin)
      {
        gzwarn << "Failed to instantiate [" << className << "] from ["
                << pathToLib << "]" << std::endl;
        continue;
      }

      this->dataPtr->engine = physics::RequestEngine<
        physics::FeaturePolicy3d,
        PhysicsPrivate::MinimumFeatureList>::From(plugin);

      if (nullptr != this->dataPtr->engine)
      {
        gzdbg << "Loaded [" << className << "] from library ["
               << pathToLib << "]" << std::endl;
        break;
      }

      auto missingFeatures = physics::RequestEngine<
          physics::FeaturePolicy3d,
          PhysicsPrivate::MinimumFeatureList>::MissingFeatureNames(plugin);

      std::stringstream msg;
      msg << "Plugin [" << className << "] misses required features:"
          << std::endl;
      for (auto feature : missingFeatures)
      {
        msg << "- " << feature << std::endl;
      }
      gzwarn << msg.str();
    }
    if (nullptr == this->dataPtr->engine)
    {
      gzerr << "Failed to load a valid physics engine from [" << pathToLib
             << "]."
             << std::endl;
      return;
    }
  }

  this->dataPtr->eventManager = &_eventMgr;
}

//////////////////////////////////////////////////
Physics::~Physics() = default;

//////////////////////////////////////////////////
void Physics::Update(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  GZ_PROFILE("Physics::Update");

  if (this->dataPtr->engine)
  {
    this->dataPtr->CreatePhysicsEntities(_ecm);
    this->dataPtr->UpdatePhysics(_ecm);
    gz::physics::ForwardStep::Output stepOutput;
    // Only step if not paused.
    if (!_info.paused)
    {
      stepOutput = this->dataPtr->Step(_info.dt);
    }
    auto changedLinks = this->dataPtr->ChangedLinks(_ecm, stepOutput);
    this->dataPtr->UpdateSim(_ecm, changedLinks);

    // Entities scheduled to be removed should be removed from physics after the
    // simulation step. Otherwise, since the to-be-removed entity still shows up
    // in the ECM::Each the UpdatePhysics and UpdateSim calls will have an error
    this->dataPtr->RemovePhysicsEntities(_ecm);
  }
}

//////////////////////////////////////////////////
void Physics::Reset(const UpdateInfo &, EntityComponentManager &_ecm)
{
  GZ_PROFILE("Physics::Reset");

  if (this->dataPtr->engine)
  {
    gzdbg << "Resetting Physics\n";
    this->dataPtr->ResetPhysics(_ecm);
  }
}

//////////////////////////////////////////////////
GZ_ADD_PLUGIN(Physics,
                    System,
                    Physics::ISystemConfigure,
                    Physics::ISystemReset,
                    Physics::ISystemUpdate)

GZ_ADD_PLUGIN_ALIAS(Physics, "gz::sim::systems::Physics")
