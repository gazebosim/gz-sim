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

/*
 * Development of this module has been funded by the Monterey Bay Aquarium
 * Research Institute (MBARI) and the David and Lucile Packard Foundation
 */

#ifndef GZ_SIM_TEST_HELPERS_TEST_FIXTURE_HH
#define GZ_SIM_TEST_HELPERS_TEST_FIXTURE_HH

#include <chrono>
#include <string>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/sim/components/Physics.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/transport/Node.hh>

#include "helpers/ModelManipulator.hh"
#include "helpers/ModelObserver.hh"

using namespace gz;

/// \brief An decorated TestFixture class
/// to ease unit an integration testing.
class TestFixture
{
  /// Constructor.
  /// \param[in] _worldPath Path to world SDF.
  public: TestFixture(const std::string &_worldPath)
    : fixture(_worldPath)
  {
    common::Console::SetVerbosity(4);
  }

  virtual ~TestFixture() = default;

  /// Pause the simulation in this fixture.
  public: void Pause() { this->paused = true; }

  /// Returns the underlying Gazebosim server.
  public: sim::Server *Simulator()
  {
    if (!this->initialized)
    {
      this->fixture
          .OnConfigure(
              [&](const sim::Entity &_entity,
                  const std::shared_ptr<const sdf::Element> &_sdf,
                  sim::EntityComponentManager &_ecm,
                  sim::EventManager &_eventManager)
              {
                sim::World world(sim::worldEntity(_ecm));
                auto physicsComponent = _ecm.Component<
                  sim::components::Physics>(world.Entity());
                this->maxStepSize = physicsComponent->Data().MaxStepSize();
                this->OnConfigure(_entity, _sdf, _ecm, _eventManager);
              })
          .OnPreUpdate(
              [&](const sim::UpdateInfo &_info,
                  sim::EntityComponentManager &_ecm)
              {
                this->OnPreUpdate(_info, _ecm);
              })
          .OnPostUpdate(
              [&](const sim::UpdateInfo &_info,
                  const sim::EntityComponentManager &_ecm)
              {
                this->OnPostUpdate(_info, _ecm);
                this->info = _info;
              })
          .Finalize();
      this->initialized = true;
    }
    return this->fixture.Server().get();
  }

  /// Advance the simulation one step forward.
  /// \return number of simulation steps taken.
  public: uint64_t Step()
  {
    this->Simulator()->RunOnce(this->paused);
    return 1u;
  }

  /// Advance the simulation.
  /// \param[in] _steps Number of simulation steps to take.
  /// \return number of simulation steps actually taken.
  public: uint64_t Step(uint64_t _steps)
  {
    static constexpr bool blocking = true;
    uint64_t initial_iterations = this->Iterations();
    this->Simulator()->Run(blocking, _steps, this->paused);
    return this->Iterations() - initial_iterations;
  }

  /// Advance the simulation.
  /// \param[in] _step Step in simulation time to take.
  /// \return number of simulation steps taken.
  public: uint64_t Step(const std::chrono::steady_clock::duration &_step)
  {
    static constexpr bool blocking = true;
    uint64_t iterations = 0u;
    // Fetch simulator early to ensure it is initialized
    auto simulator = this->Simulator();
    const auto deadline = this->info.simTime + _step;
    do {
      const double stepSize =
          std::chrono::duration<double>(deadline - this->info.simTime).count();
      uint64_t previous_iterations = this->Iterations();
      simulator->Run(
          blocking, std::ceil(stepSize / this->maxStepSize), this->paused);
      iterations += this->Iterations() - previous_iterations;
    } while (this->info.simTime < deadline);
    return iterations;
  }

  /// Returns the total number of simulation iterations so far.
  public: uint64_t Iterations() const { return this->info.iterations; }

  /// To be optionally overriden by subclasses.
  /// \see sim::TestFixture
  protected: virtual void OnConfigure(
      const sim::Entity &,
      const std::shared_ptr<const sdf::Element> &,
      sim::EntityComponentManager &,
      sim::EventManager &)
  {
  }

  /// To be optionally overriden by subclasses.
  /// \see sim::TestFixture
  protected: virtual void OnPreUpdate(
    const sim::UpdateInfo &,
    sim::EntityComponentManager &)
  {
  }

  /// To be optionally overriden by subclasses.
  /// \see sim::TestFixture
  protected: virtual void OnPostUpdate(
    const sim::UpdateInfo &,
    const sim::EntityComponentManager &)
  {
  }

  private: bool initialized{false};

  private: bool paused{false};

  private: double maxStepSize;

  private: sim::UpdateInfo info;

  private: sim::TestFixture fixture;
};

/// \brief A test fixture around a single model.
class TestFixtureWithModel : public TestFixture
{
  /// Constructor.
  /// \param[in] _worldPath Path to world SDF.
  /// \param[in] _modelName Name of the model.
  public: TestFixtureWithModel(
      const std::string &_worldPath,
      const std::string &_modelName)
    : TestFixture(_worldPath),
      manipulator(_modelName),
      observer(_modelName)
  {
  }

  /// Returns a manipulator for the model.
  public: ModelManipulator &Manipulator()
  {
    return this->manipulator;
  }

  /// Returns an observer for the model.
  public: ModelObserver &Observer()
  {
    return this->observer;
  }

  protected: void OnPreUpdate(
    const sim::UpdateInfo &_info,
    sim::EntityComponentManager &_ecm) override
  {
    this->manipulator.Update(_ecm);
    this->observer.PreUpdate(_info, _ecm);
  }

  protected: void OnPostUpdate(
    const sim::UpdateInfo &_info,
    const sim::EntityComponentManager &_ecm) override
  {
    this->observer.Update(_info, _ecm);
  }

  private: ModelManipulator manipulator;

  private: ModelObserver observer;
};

#endif // GZ_SIM_TEST_HELPERS_TEST_FIXTURE_HH
