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

// Cost of the planar collision queries (PlanarCollisionWorld::Move) for a
// fleet of robots in a warehouse: 2000 shelf boxes in 25 rows, robots in the
// aisles. The "ns_per_robot" counter is the cost of one robot in one step.
// "blocked_pct" is the share of robot-steps the module cut, and "min_sep_mm"
// the smallest separation seen (negative means an overlap).

#include <benchmark/benchmark.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

#include <gz/math/Pose3.hh>

#include "systems/simplified_physics/PlanarCollision.hh"

using namespace gz;
using namespace sim;
using namespace systems::simplified_physics;

namespace
{
constexpr int kShelfRows{25};
constexpr int kShelvesPerRow{80};
constexpr double kShelfPitch{1.2};
constexpr double kShelfDepth{0.6};

PlanarFootprint Box(double _cx, double _cy, double _sx, double _sy,
    double _zMax)
{
  PlanarFootprint fp;
  fp.points = {
      {_cx - 0.5 * _sx, _cy - 0.5 * _sy}, {_cx + 0.5 * _sx, _cy - 0.5 * _sy},
      {_cx + 0.5 * _sx, _cy + 0.5 * _sy}, {_cx - 0.5 * _sx, _cy + 0.5 * _sy}};
  fp.zMin = 0.0;
  fp.zMax = _zMax;
  return fp;
}

/// \brief What the robots do.
struct Config
{
  /// \brief Number of robots.
  int robots{500};

  /// \brief Distance between robot centers in an aisle, in meters.
  double spacing{4.0};

  /// \brief Speed in m/s, and turn rate in rad/s.
  double speed{1.0};
  double turn{0.0};

  /// \brief Step in seconds.
  double dt{0.001};

  /// \brief Width of the aisles, in meters.
  double aisle{2.4};

  /// \brief Extra speed of the robot in the back of a line, per robot ahead
  /// of it, in m/s. Followers then run into the robot ahead.
  double push{0.0};

  /// \brief Speed toward the +y shelves, in m/s.
  double drift{0.0};

  /// \brief Distance of the robots from the aisle center toward the +y
  /// shelves, in meters.
  double offset{0.0};

  /// \brief True to stop the robot at the head of every line, so that the
  /// robots behind it queue at the same speed.
  bool stopHead{false};

  /// \brief Shapes per robot: 1 is a box, 5 adds four wheels (a box each).
  int shapes{1};

  PlanarResponse response{PlanarResponse::Stop};
};

/// \brief The warehouse and its robots.
struct Warehouse
{
  PlanarCollisionWorld world;
  std::vector<PlanarMotionRequest> requests;
  std::vector<PlanarMotionResult> results;
  Config config;

  explicit Warehouse(const Config &_config)
    : config(_config)
  {
    const double rowPitch = kShelfDepth + _config.aisle;
    Entity id = 100000;
    for (int r = 0; r < kShelfRows; ++r)
    {
      for (int s = 0; s < kShelvesPerRow; ++s)
      {
        this->world.AddStaticFootprints(id++, {Box(s * kShelfPitch,
            r * rowPitch, 1.0, kShelfDepth, 2.4)});
      }
    }
    // Robots in the aisles between the rows, all driving in +x. The aisle
    // between rows r and r + 1 is centered at (r + 0.5) * rowPitch.
    const int aisles = kShelfRows - 1;
    const int perAisle = (_config.robots + aisles - 1) / aisles;
    std::vector<PlanarFootprint> body{Box(0, 0, 0.7, 0.5, 0.4)};
    if (_config.shapes > 1)
    {
      // Wheels at the corners, inside the chassis footprint.
      for (const double x : {-0.25, 0.25})
      {
        for (const double y : {-0.2, 0.2})
          body.push_back(Box(x, y, 0.2, 0.1, 0.15));
      }
    }
    for (int i = 0; i < _config.robots; ++i)
    {
      const int aisle = i % aisles;
      const int slot = i / aisles;
      const Entity e = static_cast<Entity>(i + 1);
      this->world.AddMovableFootprints(e, body,
          math::Pose3d(2.0 + slot * _config.spacing,
            (aisle + 0.5) * rowPitch + _config.offset, 0, 0, 0, 0));
      PlanarMotionRequest req;
      req.model = e;
      req.response = _config.response;
      req.desired.x = (_config.speed + _config.push * (perAisle - 1 - slot)) *
          _config.dt;
      if (_config.stopHead && slot == perAisle - 1)
        req.desired.x = 0.0;
      req.desired.y = _config.drift * _config.dt;
      req.desired.yaw = _config.turn * _config.dt;
      this->requests.push_back(req);
    }
  }

  /// \brief Smallest separation of any robot to anything, in meters,
  /// capped at the contact margin.
  double MinSeparation() const
  {
    double sep = PlanarCollisionOptions().contactMargin;
    std::vector<PlanarContact> contacts;
    for (const auto &req : this->requests)
    {
      this->world.ContactsOf(req.model, contacts);
      sep = std::accumulate(contacts.begin(), contacts.end(), sep,
          [](double _s, const PlanarContact &_c)
          {
            return std::min(_s, _c.separation);
          });
    }
    return sep;
  }
};

/// \brief Run the benchmark loop.
///
/// Before the timed loop, an identical warehouse runs the same number of
/// steps outside the timing. It checks that the robots start clear of
/// everything and never overlap, and counts how many robot-steps are blocked,
/// so that the figure is known to measure the case it names.
void RunMoves(benchmark::State &_state, const Config &_config)
{
  {
    Warehouse probe(_config);
    if (probe.MinSeparation() < 0.0)
    {
      _state.SkipWithError("robots start overlapping");
      return;
    }
    double worst = 1.0;
    double blockedSteps = 0.0;
    const int steps = static_cast<int>(_state.max_iterations);
    for (int i = 0; i < steps; ++i)
    {
      probe.world.Move(probe.requests, probe.results);
      blockedSteps += static_cast<double>(std::count_if(
          probe.results.begin(), probe.results.end(),
          [](const PlanarMotionResult &_r) { return _r.blocked; }));
      if (i % 100 == 99)
        worst = std::min(worst, probe.MinSeparation());
    }
    _state.counters["blocked_pct"] = 100.0 * blockedSteps /
        (static_cast<double>(steps) * _config.robots);
    // In millimeters. The contact margin (1 mm) means "no contact".
    _state.counters["min_sep_mm"] = 1e3 * worst;
  }

  Warehouse w(_config);
  for (auto _ : _state)
  {
    w.world.Move(w.requests, w.results);
    benchmark::DoNotOptimize(w.results.data());
  }
  // Nanoseconds per robot and step.
  _state.counters["ns_per_robot"] = benchmark::Counter(_config.robots / 1e9,
      benchmark::Counter::kIsIterationInvariantRate |
      benchmark::Counter::kInvert);
}
}  // namespace

/// \brief Robots drive down the aisles. No contact.
static void BM_PlanarCollisionFree(benchmark::State &_state)
{
  Config c;
  c.robots = static_cast<int>(_state.range(0));
  RunMoves(_state, c);
}
BENCHMARK(BM_PlanarCollisionFree)->Arg(100)->Arg(500)->Arg(1000)->Iterations(
    2000);

/// \brief The same, with every robot turning a little.
static void BM_PlanarCollisionTurning(benchmark::State &_state)
{
  Config c;
  c.robots = static_cast<int>(_state.range(0));
  c.turn = 0.02;
  RunMoves(_state, c);
}
BENCHMARK(BM_PlanarCollisionTurning)->Arg(500)->Iterations(2000);

/// \brief Robots queue nose to tail, 0.5 mm apart, and each runs faster than
/// the one ahead: a robot-robot contact in most steps.
static void BM_PlanarCollisionQueue(benchmark::State &_state)
{
  Config c;
  c.robots = static_cast<int>(_state.range(0));
  c.spacing = 0.7005;
  c.push = 0.05;
  RunMoves(_state, c);
}
BENCHMARK(BM_PlanarCollisionQueue)->Arg(500)->Iterations(2000);

/// \brief Robots queue behind a stopped robot at the same speed: the case
/// that needs one pass per robot (21 robots per line).
static void BM_PlanarCollisionQueueEqualSpeed(benchmark::State &_state)
{
  Config c;
  c.robots = static_cast<int>(_state.range(0));
  c.spacing = 0.7005;
  c.stopHead = true;
  RunMoves(_state, c);
}
BENCHMARK(BM_PlanarCollisionQueueEqualSpeed)->Arg(500)->Iterations(2000);

/// \brief Aisles 4 cm wider than the robots (2 cm each side). The robots
/// start 0.3 mm from the +y shelves and drift toward them while they drive,
/// so they slide along the shelves in almost every step.
static void BM_PlanarCollisionTightAisleSlide(benchmark::State &_state)
{
  Config c;
  c.robots = static_cast<int>(_state.range(0));
  c.aisle = 0.54;
  c.drift = 0.05;
  c.offset = 0.02 - 0.0003;
  c.response = PlanarResponse::Slide;
  RunMoves(_state, c);
}
BENCHMARK(BM_PlanarCollisionTightAisleSlide)->Arg(500)->Iterations(2000);

/// \brief The same, and every robot also tries to turn. A box that touches a
/// shelf cannot turn, so the turn is dropped in each step.
static void BM_PlanarCollisionTightAisleTurn(benchmark::State &_state)
{
  Config c;
  c.robots = static_cast<int>(_state.range(0));
  c.aisle = 0.54;
  c.drift = 0.05;
  c.offset = 0.02 - 0.0003;
  c.turn = 0.5;
  c.response = PlanarResponse::Slide;
  RunMoves(_state, c);
}
BENCHMARK(BM_PlanarCollisionTightAisleTurn)->Arg(500)->Iterations(2000);

/// \brief The tight aisle again, with five shapes per robot (chassis and
/// four wheels).
static void BM_PlanarCollisionTightAisleFiveShapes(benchmark::State &_state)
{
  Config c;
  c.robots = static_cast<int>(_state.range(0));
  c.aisle = 0.54;
  c.drift = 0.05;
  c.offset = 0.02 - 0.0003;
  c.shapes = 5;
  c.response = PlanarResponse::Slide;
  RunMoves(_state, c);
}
BENCHMARK(BM_PlanarCollisionTightAisleFiveShapes)->Arg(500)->Iterations(2000);

BENCHMARK_MAIN();
