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

#include <benchmark/benchmark.h>

#include <algorithm>
#include <cmath>
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

/// \brief The warehouse and its robots.
struct Warehouse
{
  PlanarCollisionWorld world;
  std::vector<PlanarMotionRequest> requests;
  std::vector<PlanarMotionResult> results;

  /// \param[in] _robots Number of robots.
  /// \param[in] _spacing Distance between robots in an aisle, in meters.
  /// \param[in] _speed Speed in m/s.
  /// \param[in] _turn Turn rate in rad/s.
  /// \param[in] _dt Step in seconds.
  /// \param[in] _aisle Width of the aisles, in meters.
  /// \param[in] _push Extra speed of the robot in the back of a line, per
  /// robot ahead of it, in m/s. Followers then run into the robot ahead.
  /// \param[in] _drift Speed toward the shelves, in m/s.
  /// \param[in] _response Response of the robots.
  Warehouse(int _robots, double _spacing, double _speed, double _turn,
      double _dt, double _aisle = 2.4, double _push = 0.0,
      double _drift = 0.0, PlanarResponse _response = PlanarResponse::Stop)
  {
    const double rowPitch = kShelfDepth + _aisle;
    Entity id = 100000;
    for (int r = 0; r < kShelfRows; ++r)
    {
      for (int s = 0; s < kShelvesPerRow; ++s)
      {
        this->world.AddStaticFootprints(id++, {Box(s * kShelfPitch,
            r * rowPitch, 1.0, kShelfDepth, 2.4)});
      }
    }
    // Robots in the aisles between the rows, all driving in +x.
    const int aisles = kShelfRows - 1;
    const int perAisle = (_robots + aisles - 1) / aisles;
    for (int i = 0; i < _robots; ++i)
    {
      const int aisle = i % aisles;
      const int slot = i / aisles;
      const Entity e = static_cast<Entity>(i + 1);
      this->world.AddMovableFootprints(e, {Box(0, 0, 0.7, 0.5, 0.4)},
          math::Pose3d(2.0 + slot * _spacing, (aisle + 0.5) * rowPitch +
            0.5 * kShelfDepth, 0, 0, 0, 0));
      PlanarMotionRequest req;
      req.model = e;
      req.response = _response;
      req.desired.x = (_speed + _push * (perAisle - 1 - slot)) * _dt;
      req.desired.y = _drift * _dt;
      req.desired.yaw = _turn * _dt;
      this->requests.push_back(req);
    }
  }
};

void RunMoves(benchmark::State &_state, Warehouse &_w, int _robots)
{
  for (auto _ : _state)
  {
    _w.world.Move(_w.requests, _w.results);
    benchmark::DoNotOptimize(_w.results.data());
  }
  // Nanoseconds per robot and step.
  _state.counters["ns_per_robot"] = benchmark::Counter(_robots / 1e9,
      benchmark::Counter::kIsIterationInvariantRate |
      benchmark::Counter::kInvert);
  _state.counters["blocked"] = static_cast<double>(std::count_if(
      _w.results.begin(), _w.results.end(),
      [](const PlanarMotionResult &_r) { return _r.blocked; }));
}
}  // namespace

/// \brief Robots drive down the aisles. No contact.
static void BM_PlanarCollisionFree(benchmark::State &_state)
{
  const int robots = static_cast<int>(_state.range(0));
  Warehouse w(robots, 4.0, 1.0, 0.0, 0.001);
  RunMoves(_state, w, robots);
}
BENCHMARK(BM_PlanarCollisionFree)->Arg(100)->Arg(500)->Arg(1000)->Iterations(
    2000);

/// \brief The same, with every robot turning a little.
static void BM_PlanarCollisionTurning(benchmark::State &_state)
{
  const int robots = static_cast<int>(_state.range(0));
  Warehouse w(robots, 4.0, 1.0, 0.02, 0.001);
  RunMoves(_state, w, robots);
}
BENCHMARK(BM_PlanarCollisionTurning)->Arg(500)->Iterations(2000);

/// \brief Robots queue nose to tail, 2 cm apart, and each runs faster than
/// the one ahead: robot-robot contact in every pair, every step.
static void BM_PlanarCollisionQueue(benchmark::State &_state)
{
  const int robots = static_cast<int>(_state.range(0));
  Warehouse w(robots, 0.72, 1.0, 0.0, 0.001, 2.4, 0.05);
  RunMoves(_state, w, robots);
}
BENCHMARK(BM_PlanarCollisionQueue)->Arg(500)->Iterations(2000);

/// \brief Aisles 4 cm wider than the robots, and every robot drifts toward
/// the shelves while it drives: every step is a contact with a shelf, and
/// the robots slide along it.
static void BM_PlanarCollisionTightAisleSlide(benchmark::State &_state)
{
  const int robots = static_cast<int>(_state.range(0));
  Warehouse w(robots, 4.0, 1.0, 0.0, 0.001, 0.54, 0.0, 0.05,
      PlanarResponse::Slide);
  RunMoves(_state, w, robots);
}
BENCHMARK(BM_PlanarCollisionTightAisleSlide)->Arg(500)->Iterations(2000);

BENCHMARK_MAIN();
