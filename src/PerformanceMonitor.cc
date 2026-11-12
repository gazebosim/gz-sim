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

#include "PerformanceMonitor.hh"

#ifndef _WIN32
#include <time.h>
#endif

#include <algorithm>
#include <iomanip>
#include <map>
#include <sstream>
#include <unordered_set>
#include <utility>

#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/msgs/convert/StdTypes.hh>
#include <gz/sensors/SensorTiming.hh>
#include <gz/math/Helpers.hh>
#include <sdf/Collision.hh>
#include <sdf/Geometry.hh>
#include <sdf/Joint.hh>
#include <sdf/Mesh.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/ChildLinkName.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/Geometry.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointType.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/ParentLinkName.hh"
#include "gz/sim/components/Static.hh"
#include "gz/sim/components/Visual.hh"
#include "gz/sim/components/World.hh"

#include "SystemInternal.hh"
#include "SystemManager.hh"

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

namespace
{
using Duration = std::chrono::steady_clock::duration;

/// \brief The simulation counts as limited below this fraction of its target.
/// The tolerance keeps the jitter of pacing from being reported as a limit.
constexpr double kLimitedRtfRatio = 0.97;

/// \brief A system that is on the CPU for less than this fraction of its wall
/// time is waiting for something rather than computing.
constexpr double kBlockedCpuRatio = 0.5;

/// \brief Fraction of the render thread time blocked on the GPU above which
/// the GPU is the limit. Between this and kCpuLimitedRatio neither of them
/// dominates: halving the cost of just one wouldn't gain much.
constexpr double kGpuLimitedRatio = 0.65;

/// \brief Fraction of the render thread time blocked on the GPU below which
/// the CPU is the limit.
constexpr double kCpuLimitedRatio = 0.35;

/// \brief CPU time is measured only once in this many iterations, since
/// reading it is a system call. It is a prime number so that it doesn't alias
/// with periodic work: a sensor that renders every 4 steps would always be
/// sampled in the same phase with a period of 16.
constexpr uint64_t kCpuSamplePeriod = 17u;

/// \brief The first frames of the sensors are not representative, so that
/// much is discarded before the first interval starts.
constexpr auto kWarmUpTime = 2s;

/// \brief Number of entries of each table of the console report.
constexpr size_t kMaxReportRows = 5u;

/// \brief Cost of the contacts of a model, which is this weight times the
/// square of its contact points, in equivalent degrees of freedom. The contacts
/// of a model are solved together so their cost grows faster than their count.
/// Measured with DART on piles of 36, 300 and 1000 contact points, against
/// 300 free bodies: it is within a factor of 1.5 of the step time in the three.
constexpr double kContactWeight = 2.0;

/// \brief Weight of a collision shape in equivalent degrees of freedom. Every
/// shape in the world costs time on every step, even a static one. Measured with
/// DART: 2900 static boxes took 1.6 ms per step.
constexpr double kCollisionWeight = 2.0;

/// \brief Weight of a thousand triangles of a mesh collision that moves and
/// touches something, in equivalent degrees of freedom. Such a mesh is checked
/// triangle by triangle on every step. Static meshes cost nothing of the kind.
/// Measured once with DART: a 7700 triangle mesh spinning around a pivot took
/// 2.8 ms per step, while 3.3 million static triangles took none.
constexpr double kContactingKiloTriangleWeight = 1250.0;

/// \brief Whether a monitor is collecting, see PerformanceMonitorEnabled.
std::atomic<bool> performanceMonitorEnabled{false};

/// \brief Convert a duration to milliseconds.
/// \param[in] _time The duration
/// \return The milliseconds
double ms(Duration _time)
{
  return std::chrono::duration<double, std::milli>(_time).count();
}

/// \brief Compute a ratio of durations.
/// \param[in] _part The numerator
/// \param[in] _whole The denominator
/// \return The ratio, zero if the denominator isn't positive
double ratio(Duration _part, Duration _whole)
{
  if (_whole <= Duration::zero())
    return 0.0;
  return std::chrono::duration<double>(_part).count() /
      std::chrono::duration<double>(_whole).count();
}

/// \brief Scale a duration.
/// \param[in] _time The duration
/// \param[in] _factor The factor
/// \return The scaled duration
Duration scale(Duration _time, double _factor)
{
  return std::chrono::duration_cast<Duration>(
      std::chrono::duration<double, Duration::period>(
        static_cast<double>(_time.count()) * _factor));
}

/// \brief Read the CPU time of the calling thread.
/// \return The time, negative if it is not available in this platform.
std::chrono::nanoseconds threadCpuTime()
{
#ifndef _WIN32
  timespec ts;
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts) == 0)
  {
    return std::chrono::seconds(ts.tv_sec) +
        std::chrono::nanoseconds(ts.tv_nsec);
  }
#endif
  return std::chrono::nanoseconds(-1);
}

/// \brief Total time of a pass of the render thread.
/// \param[in] _times The pass
/// \return Sum of its stages
Duration renderTotal(const events::RenderFrameTimes &_times)
{
  return _times.sceneUpdate + _times.preRender + _times.sensors +
      _times.postRender;
}

/// \brief Get the time that the render thread spends on the scene itself, as
/// opposed to drawing it for the sensors.
/// \param[in] _times Times of the render thread
/// \return The time
Duration sceneWork(const events::RenderFrameTimes &_times)
{
  return _times.sceneUpdate + _times.preRender;
}

/// \brief Compute the cost of a model in the physics engine in equivalent
/// degrees of freedom.
/// \param[in] _model The model
/// \return The cost
double physicsCost(const PerformanceModelSample &_model)
{
  return static_cast<double>(_model.dof) +
      kCollisionWeight * _model.collisions +
      kContactWeight * _model.contacts * _model.contacts +
      kContactingKiloTriangleWeight *
      (_model.contactingMeshTriangles / 1000.0);
}

/// \brief Get the number of triangles of a mesh that is already loaded.
/// Meshes that nothing has loaded are not loaded here: it would be a costly
/// side effect, and if nothing needed them they aren't a load either.
/// \param[in] _mesh The mesh
/// \return The number of triangles
uint64_t meshTriangles(const sdf::Mesh &_mesh)
{
  auto *meshManager = common::MeshManager::Instance();
  const std::string path = asFullPath(_mesh.Uri(), _mesh.FilePath());
  if (!meshManager->HasMesh(path))
    return 0u;
  const common::Mesh *mesh = meshManager->MeshByName(path);
  return mesh ? mesh->IndexCount() / 3u : 0u;
}

/// \brief Get the degrees of freedom added by a joint.
/// \param[in] _type Type of the joint
/// \return The degrees of freedom
unsigned int jointDof(sdf::JointType _type)
{
  switch (_type)
  {
    case sdf::JointType::BALL:
      return 3u;
    case sdf::JointType::REVOLUTE2:
    case sdf::JointType::UNIVERSAL:
      return 2u;
    case sdf::JointType::FIXED:
    case sdf::JointType::INVALID:
      return 0u;
    default:
      return 1u;
  }
}

/// \brief Find the top level model that contains an entity.
/// \param[in] _entity The entity
/// \param[in] _ecm Entities of the simulation
/// \return Name of the model, empty if the entity is not inside of one
std::string topLevelModelName(Entity _entity,
    const EntityComponentManager &_ecm)
{
  const Entity model = topLevelModel(_entity, _ecm);
  if (!_ecm.Component<components::Model>(model))
    return std::string();

  const auto *name = _ecm.Component<components::Name>(model);
  return name ? name->Data() : std::string();
}

/// \brief Get the plugin name without its namespaces.
/// \param[in] _name The plugin name
/// \return The last element of the name
std::string shortSystemName(const std::string &_name)
{
  const auto pos = _name.rfind("::");
  return pos == std::string::npos ? _name : _name.substr(pos + 2);
}

/// \brief Add a time to a message.
/// \param[in, out] _msg The message
/// \param[in] _name Name of the entry
/// \param[in] _time The time
/// \param[in] _wall Wall time of the interval
void addTime(msgs::Diagnostics &_msg, const std::string &_name,
    Duration _time, Duration _wall)
{
  auto *entry = _msg.add_time();
  entry->set_name(_name);
  *entry->mutable_elapsed() = msgs::Convert(_time);
  *entry->mutable_wall() = msgs::Convert(_wall);
}

/// \brief Add a key and value to the header of a message.
/// \param[in, out] _msg The message
/// \param[in] _key The key
/// \param[in] _value The value
void addData(msgs::Diagnostics &_msg, const std::string &_key,
    const std::string &_value)
{
  auto *data = _msg.mutable_header()->add_data();
  data->set_key(_key);
  data->add_value(_value);
}
}  // namespace

/////////////////////////////////////////////////
PerformanceSchedule::Shares PerformanceSchedule::Step(Duration _busy,
    Duration _period, const Shares &_shares)
{
  const auto owe = [this](const void *_system, Duration _time)
  {
    if (_time <= Duration::zero())
      return;
    auto it = std::find_if(this->owed.begin(), this->owed.end(),
        [_system](const auto &_owed) { return _owed.first == _system; });
    if (it == this->owed.end())
      this->owed.emplace_back(_system, _time);
    else
      it->second += _time;
  };

  const Duration overrun = _busy - _period;
  if (overrun > Duration::zero())
  {
    // The step puts the loop further behind, split like the step time.
    Duration accounted{0};
    for (const auto &[system, time] : _shares)
    {
      owe(system, scale(overrun, ratio(time, _busy)));
      accounted += time;
    }
    owe(nullptr, scale(overrun,
        ratio(std::max(_busy - accounted, Duration::zero()), _busy)));
    this->debt += overrun;
  }
  else if (this->debt > Duration::zero())
  {
    // The slack of the step makes up for as much as it can, of what each
    // one owes in proportion.
    const Duration left = std::max(this->debt + overrun, Duration::zero());
    const double keep = ratio(left, this->debt);
    for (auto &share : this->owed)
      share.second = scale(share.second, keep);
    this->debt = left;
  }

  Shares lost;
  if (this->debt > _period)
  {
    // The loop starts a new schedule, the time behind the old one is lost.
    lost = std::move(this->owed);
    this->Clear();
  }
  else if (this->debt <= Duration::zero())
  {
    this->Clear();
  }
  return lost;
}

/////////////////////////////////////////////////
void PerformanceSchedule::Clear()
{
  this->debt = Duration::zero();
  this->owed.clear();
}

/////////////////////////////////////////////////
Duration PerformanceSchedule::Debt() const
{
  return this->debt;
}

/////////////////////////////////////////////////
std::vector<PerformanceModelSample> sim::CollectPerformanceModels(
    const EntityComponentManager &_ecm,
    const std::unordered_map<Entity, double> &_contacts)
{
  std::vector<PerformanceModelSample> models;
  _ecm.Each<components::Model, components::Name, components::ParentEntity>(
      [&](const Entity &_entity, const components::Model *,
          const components::Name *_name,
          const components::ParentEntity *_parent) -> bool
      {
        if (!_ecm.Component<components::World>(_parent->Data()))
          return true;

        PerformanceModelSample model;
        model.name = _name->Data();

        // Neither a static model nested inside of one that isn't, nor a link
        // welded to the world, can move.
        const auto descendants = _ecm.Descendants(_entity);
        std::unordered_set<Entity> fixedEntities;
        const auto fix = [&](const Entity _fixed)
        {
          const auto nested = _ecm.Descendants(_fixed);
          fixedEntities.insert(nested.begin(), nested.end());
        };
        for (const Entity entity : descendants)
        {
          const auto *staticComp = _ecm.Component<components::Static>(entity);
          if (_ecm.Component<components::Model>(entity) && staticComp &&
              staticComp->Data())
          {
            fix(entity);
          }

          const auto *type = _ecm.Component<components::JointType>(entity);
          const auto *parent = _ecm.Component<components::ParentLinkName>(entity);
          const auto *child = _ecm.Component<components::ChildLinkName>(entity);
          if (type && parent && child &&
              type->Data() == sdf::JointType::FIXED && parent->Data() == "world")
          {
            fixedEntities.insert(entity);
            const Entity link = _ecm.EntityByComponents(
                components::ParentEntity(_ecm.ParentEntity(entity)),
                components::Name(child->Data()), components::Link());
            if (link != kNullEntity)
              fix(link);
          }
        }

        int links = 0;
        int joints = 0;
        unsigned int jointsDof = 0u;
        for (const Entity entity : descendants)
        {
          const bool isFixed = fixedEntities.count(entity) > 0u;
          if (_ecm.Component<components::Link>(entity))
          {
            if (!isFixed)
              ++links;
          }
          else if (_ecm.Component<components::Joint>(entity))
          {
            if (isFixed)
              continue;
            ++joints;
            if (auto *type = _ecm.Component<components::JointType>(entity))
              jointsDof += jointDof(type->Data());
          }
          else if (_ecm.Component<components::Collision>(entity))
          {
            ++model.collisions;
            // Solving a contact is work for the body that can move, not for
            // the floor under it.
            auto contactsIt = _contacts.find(entity);
            if (!isFixed && contactsIt != _contacts.end() &&
                contactsIt->second > 0.0)
            {
              model.contacts += contactsIt->second;
              auto *element =
                  _ecm.Component<components::CollisionElement>(entity);
              if (element && element->Data().Geom() &&
                  element->Data().Geom()->MeshShape())
              {
                model.contactingMeshTriangles +=
                    meshTriangles(*element->Data().Geom()->MeshShape());
              }
            }
          }
          else if (_ecm.Component<components::Visual>(entity))
          {
            ++model.visuals;
            auto *geometry = _ecm.Component<components::Geometry>(entity);
            if (geometry && geometry->Data().MeshShape())
            {
              model.visualTriangles +=
                  meshTriangles(*geometry->Data().MeshShape());
            }
          }
        }

        // Each kinematic tree that isn't attached to the world floats.
        model.dof = jointsDof + 6u * static_cast<unsigned int>(
            std::max(links - joints, 0));
        models.push_back(std::move(model));
        return true;
      });
  return models;
}

//////////////////////////////////////////////////
bool gz::sim::PerformanceMonitorEnabled()
{
  return performanceMonitorEnabled.load(std::memory_order_relaxed);
}

//////////////////////////////////////////////////
PerformanceReport gz::sim::AnalyzePerformance(const PerformanceWindow &_window)
{
  PerformanceReport report;
  report.wall = _window.wall;
  report.busy = std::max(_window.wall - _window.sleep, Duration::zero());
  report.iterations = _window.iterations;
  report.desiredRtf = _window.desiredRtf;
  report.period = _window.period;
  report.rtf = ratio(_window.simTime, _window.wall);
  report.render = _window.render;
  report.renderFrames = _window.renderFrames;
  report.limited = _window.desiredRtf <= 0.0 ||
      report.rtf < _window.desiredRtf * kLimitedRtfRatio;

  report.systems = _window.systems;
  std::sort(report.systems.begin(), report.systems.end(),
      [](const auto &_a, const auto &_b)
      {
        return _a.critical > _b.critical;
      });

  report.sensors = _window.sensors;
  std::sort(report.sensors.begin(), report.sensors.end(),
      [](const auto &_a, const auto &_b)
      {
        return _a.total > _b.total;
      });

  // Split the time that the simulation thread was busy, and the part of it
  // that the steps lost to their schedule.
  Duration accounted{0};
  Duration sceneCopy{0};
  for (const auto &system : report.systems)
  {
    accounted += system.critical;
    const std::string name = shortSystemName(system.name);
    if (name == "Physics")
    {
      report.physics += system.critical;
      report.late.physics += system.late;
    }
    else if (name == "Sensors")
    {
      // Blocked on the render thread, or copying the entity changes for it.
      report.renderWait += std::min(system.critical,
          _window.render.simThreadWait);
      sceneCopy += std::max(system.critical - _window.render.simThreadWait,
          Duration::zero());
      report.late.rendering += system.late;
    }
    else
    {
      report.otherSystems += system.critical;
      report.late.otherSystems += system.late;
    }
  }
  report.otherSystems += sceneCopy;
  report.engine = std::max(report.busy - accounted, Duration::zero());
  report.late.loop = _window.lateLoop;

  // At a target real time factor the loop sleeps away the rest of the slot of
  // a step that fits in it, so that step costs no real time however long it
  // is. Only the time past the slot lowers the RTF, and the loop doesn't make
  // up for a step that falls a whole slot behind. Bursty work that makes a
  // few steps very late (render stalls) costs the RTF, steady work that fits
  // (physics well under the slot) doesn't. Rank by that time then. As fast as
  // possible, all of the step is past its zero slot and both rankings agree.
  report.rankedByLateTime = report.limited && _window.desiredRtf > 0.0 &&
      report.late.Total() > Duration::zero();
  const auto rankTime = [&report](const PerformanceSystemSample &_system)
  {
    return report.rankedByLateTime ? _system.late : _system.critical;
  };

  const PerformanceSystemSample *slowestOther = nullptr;
  for (const auto &system : report.systems)
  {
    const std::string name = shortSystemName(system.name);
    if (name != "Physics" && name != "Sensors" &&
        (!slowestOther || rankTime(system) > rankTime(*slowestOther)))
    {
      slowestOther = &system;
    }
  }

  // Attribute load to the models.
  std::map<std::string, PerformanceModelLoad> loads;
  double totalPhysicsCost = 0.0;
  uint64_t totalVisuals = 0u;
  for (const auto &model : _window.models)
  {
    totalVisuals += model.visuals;
    loads[model.name].name = model.name;
    loads[model.name].visuals = model.visuals;
    loads[model.name].dof = model.dof;
    loads[model.name].contacts = model.contacts;
    loads[model.name].visualTriangles = model.visualTriangles;
    totalPhysicsCost += physicsCost(model);
  }
  for (const auto &model : _window.models)
  {
    if (totalPhysicsCost > 0.0)
    {
      loads[model.name].physics =
          scale(report.physics, physicsCost(model) / totalPhysicsCost);
    }
    if (totalVisuals > 0u)
    {
      loads[model.name].scene = scale(sceneWork(_window.render),
          static_cast<double>(model.visuals) / totalVisuals);
    }
  }
  for (const auto &system : report.systems)
  {
    if (!system.model.empty())
    {
      loads[system.model].name = system.model;
      loads[system.model].plugins += system.critical;
    }
  }
  for (const auto &sensor : report.sensors)
  {
    if (!sensor.model.empty())
    {
      loads[sensor.model].name = sensor.model;
      loads[sensor.model].sensors += sensor.total;
    }
  }
  for (auto &[name, load] : loads)
    report.models.push_back(std::move(load));
  std::stable_sort(report.models.begin(), report.models.end(),
      [](const auto &_a, const auto &_b)
      {
        return _a.Total() > _b.Total();
      });

  // Find the location: the biggest consumer of the step time, or of the time
  // past the slot.
  const Duration slowestOtherTime =
      slowestOther ? rankTime(*slowestOther) : Duration::zero();
  Duration biggest = report.rankedByLateTime ? report.late.physics :
      report.physics;
  report.location = PerformanceLocation::kPhysics;
  const Duration renderingTime = report.rankedByLateTime ?
      report.late.rendering : report.renderWait + sceneCopy;
  if (renderingTime > biggest)
  {
    biggest = renderingTime;
    report.location = PerformanceLocation::kRendering;
  }
  if (slowestOtherTime > biggest)
  {
    biggest = slowestOtherTime;
    report.location = PerformanceLocation::kSystem;
  }
  const Duration loopTime = report.rankedByLateTime ? report.late.loop :
      report.engine;
  if (loopTime > biggest)
  {
    biggest = loopTime;
    report.location = PerformanceLocation::kEngine;
  }
  if (biggest <= Duration::zero())
  {
    report.location = PerformanceLocation::kNone;
    return report;
  }

  // Find who is responsible, and on which resource.
  PerformanceResource resource = PerformanceResource::kCpu;
  switch (report.location)
  {
    case PerformanceLocation::kPhysics:
    {
      const auto top = std::max_element(report.models.begin(),
          report.models.end(), [](const auto &_a, const auto &_b)
          {
            return _a.physics < _b.physics;
          });
      if (top != report.models.end() && top->physics > Duration::zero())
      {
        // An estimate can only point at a model that stands out. The largest
        // of many small shares is not worth sending anybody after.
        Duration total = Duration::zero();
        for (const auto &model : report.models)
          total += model.physics;
        if (2 * top->physics > total)
        {
          report.culprit = top->name;
        }
        else
        {
          const auto percent = std::lround(
              100.0 * std::chrono::duration<double>(top->physics).count() /
              std::chrono::duration<double>(total).count());
          report.culprit = "no model dominates, largest is " + top->name +
              " at " + std::to_string(percent) + "%";
        }
      }
      break;
    }
    case PerformanceLocation::kRendering:
    {
      const Duration renderTime = renderTotal(_window.render);
      if (_window.renderFrames == 0u)
      {
        resource = PerformanceResource::kUnknown;
      }
      else if (_window.render.softwareRenderer)
      {
        resource = PerformanceResource::kCpu;
      }
      else if (_window.render.gpuWait <= Duration::zero())
      {
        resource = PerformanceResource::kUnknown;
      }
      else if (ratio(_window.render.gpuWait, renderTime) >= kGpuLimitedRatio)
      {
        resource = PerformanceResource::kGpu;
      }
      else if (ratio(_window.render.gpuWait, renderTime) > kCpuLimitedRatio)
      {
        resource = PerformanceResource::kCpuAndGpu;
      }

      Duration sensorsTotal{0};
      for (const auto &sensor : report.sensors)
        sensorsTotal += sensor.total;

      if (sceneWork(_window.render) > sensorsTotal)
      {
        report.culprit = "scene update";
        // Name the model when most of the scene is one model.
        const auto top = std::max_element(report.models.begin(),
            report.models.end(), [](const auto &_a, const auto &_b)
            {
              return _a.visuals < _b.visuals;
            });
        if (top != report.models.end() && 2u * top->visuals > totalVisuals)
          report.culprit += ", mostly model " + top->name;
      }
      else if (!report.sensors.empty())
      {
        report.culprit = report.sensors.front().name;
      }

      // Not waiting for the render thread at all, the time goes to copying
      // the entity changes for it in the simulation thread.
      if (sceneCopy > report.renderWait)
      {
        resource = PerformanceResource::kCpu;
        report.culprit = "scene update";
      }
      break;
    }
    case PerformanceLocation::kSystem:
    {
      report.culprit = slowestOther->name;
      if (!slowestOther->model.empty())
        report.culprit += " on model " + slowestOther->model;
      if (slowestOther->sampledWall > Duration::zero() &&
          ratio(slowestOther->sampledCpu, slowestOther->sampledWall) <
          kBlockedCpuRatio)
      {
        resource = PerformanceResource::kExternal;
      }
      break;
    }
    case PerformanceLocation::kEngine:
      report.culprit = "simulation loop";
      break;
    default:
      break;
  }

  if (report.limited)
    report.resource = resource;

  return report;
}

//////////////////////////////////////////////////
std::string PerformanceReport::ResourceName() const
{
  switch (this->resource)
  {
    case PerformanceResource::kCpu:
      return "cpu";
    case PerformanceResource::kGpu:
      return "gpu";
    case PerformanceResource::kCpuAndGpu:
      return "cpu_and_gpu";
    case PerformanceResource::kExternal:
      return "external";
    case PerformanceResource::kUnknown:
      return "unknown";
    default:
      return "none";
  }
}

//////////////////////////////////////////////////
std::string PerformanceReport::LocationName() const
{
  switch (this->location)
  {
    case PerformanceLocation::kPhysics:
      return "physics";
    case PerformanceLocation::kRendering:
      return "rendering";
    case PerformanceLocation::kSystem:
      return "system";
    case PerformanceLocation::kEngine:
      return "engine";
    default:
      return "none";
  }
}

//////////////////////////////////////////////////
std::string PerformanceReport::ToString() const
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(2);

  const double steps = std::max<double>(
      static_cast<double>(this->iterations), 1.0);
  const auto perStep = [steps](Duration _time)
  {
    return ms(_time) / steps;
  };
  const auto percent = [this](Duration _time)
  {
    return static_cast<int>(100.0 * ratio(_time, this->busy) + 0.5);
  };

  out << "Performance report, last " << std::setprecision(1)
      << std::chrono::duration<double>(this->wall).count() << " s, "
      << this->iterations << " steps: RTF " << std::setprecision(2)
      << this->rtf;
  if (this->desiredRtf > 0.0)
    out << " (target " << this->desiredRtf << ")";
  else
    out << " (as fast as possible)";
  out << "\n";

  if (this->limited)
  {
    out << "  Bottleneck: ";
    switch (this->resource)
    {
      case PerformanceResource::kCpu:
        out << "CPU";
        if (this->location == PerformanceLocation::kRendering &&
            this->render.softwareRenderer)
        {
          out << " (software rasterizer, there is no GPU in use)";
        }
        break;
      case PerformanceResource::kGpu:
        out << "GPU";
        break;
      case PerformanceResource::kCpuAndGpu:
        out << "CPU and GPU, neither one dominates";
        break;
      case PerformanceResource::kExternal:
        out << "not computing, blocked waiting for something external";
        break;
      case PerformanceResource::kUnknown:
        out << "unknown CPU/GPU, render engine does not report GPU waits";
        break;
      default:
        out << "none";
        break;
    }
    out << ", in " << this->LocationName();
    if (!this->culprit.empty())
    {
      out << ": " << this->culprit;
      if (this->location == PerformanceLocation::kPhysics)
        out << " (estimated)";
    }
    out << "\n";
  }
  else
  {
    out << "  No bottleneck: the simulation keeps up with its target."
        << " Biggest consumer: " << this->LocationName();
    if (!this->culprit.empty())
      out << " (" << this->culprit << ")";
    out << "\n";
  }

  out << "  Step time " << perStep(this->busy) << " ms: rendering wait "
      << perStep(this->renderWait) << " ms ("
      << percent(this->renderWait) << "%), physics "
      << perStep(this->physics) << " ms ("
      << percent(this->physics) << "%), other systems "
      << perStep(this->otherSystems) << " ms ("
      << percent(this->otherSystems) << "%), loop "
      << perStep(this->engine) << " ms ("
      << percent(this->engine) << "%)\n";

  if (this->rankedByLateTime)
  {
    const Duration lateTotal = this->late.Total();
    const auto latePercent = [lateTotal](Duration _time)
    {
      return static_cast<int>(100.0 * ratio(_time, lateTotal) + 0.5);
    };
    out << "  Time past the " << ms(this->period) << " ms slot "
        << perStep(lateTotal)
        << " ms/step, which is what lowers the RTF: rendering "
        << perStep(this->late.rendering) << " ms ("
        << latePercent(this->late.rendering) << "%), physics "
        << perStep(this->late.physics) << " ms ("
        << latePercent(this->late.physics) << "%), other systems "
        << perStep(this->late.otherSystems) << " ms ("
        << latePercent(this->late.otherSystems) << "%), loop "
        << perStep(this->late.loop) << " ms ("
        << latePercent(this->late.loop) << "%)\n";
  }

  if (this->renderFrames > 0u)
  {
    const double frames = static_cast<double>(this->renderFrames);
    const Duration total = renderTotal(this->render);
    out << "  Render thread " << ms(total) / frames << " ms/frame ("
        << this->renderFrames << " frames): GPU wait "
        << ms(this->render.gpuWait) / frames << " ms ("
        << static_cast<int>(100.0 * ratio(this->render.gpuWait, total) + 0.5)
        << "%), scene update " << ms(this->render.sceneUpdate) / frames
        << " ms, pre render " << ms(this->render.preRender) / frames
        << " ms, sensors " << ms(this->render.sensors) / frames
        << " ms, post render " << ms(this->render.postRender) / frames
        << " ms\n";
  }

  if (!this->sensors.empty())
  {
    out << "  Sensors, slowest first:\n";
    for (size_t i = 0; i < std::min(kMaxReportRows, this->sensors.size()); ++i)
    {
      const auto &sensor = this->sensors[i];
      const double updates = static_cast<double>(sensor.updates);
      out << "    " << sensor.name << ": " << ms(sensor.total) / updates
          << " ms/update x " << sensor.updates << " (CPU "
          << ms(sensor.total - sensor.gpuWait) / updates << " ms, GPU wait "
          << ms(sensor.gpuWait) / updates << " ms)\n";
    }
  }

  if (!this->models.empty())
  {
    out << "  Models, biggest load first (physics and scene are estimated):\n";
    for (size_t i = 0; i < std::min(kMaxReportRows, this->models.size()); ++i)
    {
      const auto &model = this->models[i];
      out << "    " << model.name << ": sensors " << perStep(model.sensors)
          << " ms/step, plugins " << perStep(model.plugins)
          << " ms/step, physics ~" << perStep(model.physics)
          << " ms/step, scene ~" << perStep(model.scene)
          << " ms/step (" << model.dof << " DOF, "
          << static_cast<uint64_t>(std::lround(model.contacts))
          << " contacts, " << model.visuals << " visuals";
      if (model.visualTriangles > 0u)
        out << ", " << model.visualTriangles << " mesh triangles";
      out << ")\n";
    }
  }

  if (!this->systems.empty())
  {
    out << "  Systems, slowest first:\n";
    for (size_t i = 0; i < std::min(kMaxReportRows, this->systems.size()); ++i)
    {
      const auto &system = this->systems[i];
      out << "    " << system.name;
      if (!system.model.empty())
        out << " on model " << system.model;
      out << ": " << perStep(system.critical) << " ms/step\n";
    }
  }
  return out.str();
}

//////////////////////////////////////////////////
msgs::Diagnostics PerformanceReport::ToMsg() const
{
  msgs::Diagnostics msg;
  msg.set_real_time_factor(this->rtf);
  *msg.mutable_real_time() = msgs::Convert(this->wall);

  addData(msg, "limited", this->limited ? "true" : "false");
  addData(msg, "resource", this->ResourceName());
  addData(msg, "location", this->LocationName());
  addData(msg, "culprit", this->culprit);
  addData(msg, "desired_rtf", std::to_string(this->desiredRtf));
  addData(msg, "iterations", std::to_string(this->iterations));
  addData(msg, "render_frames", std::to_string(this->renderFrames));
  addData(msg, "software_renderer",
      this->render.softwareRenderer ? "true" : "false");

  addTime(msg, "step/busy", this->busy, this->wall);
  addTime(msg, "step/physics", this->physics, this->wall);
  addTime(msg, "step/render_wait", this->renderWait, this->wall);
  addTime(msg, "step/other_systems", this->otherSystems, this->wall);
  addTime(msg, "step/loop", this->engine, this->wall);

  addData(msg, "ranked_by_late_time",
      this->rankedByLateTime ? "true" : "false");
  addTime(msg, "late/physics", this->late.physics, this->wall);
  addTime(msg, "late/rendering", this->late.rendering, this->wall);
  addTime(msg, "late/other_systems", this->late.otherSystems, this->wall);
  addTime(msg, "late/loop", this->late.loop, this->wall);

  addTime(msg, "render/total", renderTotal(this->render), this->wall);
  addTime(msg, "render/gpu_wait", this->render.gpuWait, this->wall);
  addTime(msg, "render/scene_update", this->render.sceneUpdate, this->wall);
  addTime(msg, "render/pre_render", this->render.preRender, this->wall);
  addTime(msg, "render/sensors", this->render.sensors, this->wall);
  addTime(msg, "render/post_render", this->render.postRender, this->wall);

  for (const auto &sensor : this->sensors)
  {
    addTime(msg, "sensor/" + sensor.name, sensor.total, this->wall);
    addTime(msg, "sensor/" + sensor.name + "/gpu_wait", sensor.gpuWait,
        this->wall);
  }
  for (const auto &system : this->systems)
  {
    addTime(msg, "system/" + system.name +
        (system.model.empty() ? "" : "@" + system.model), system.critical,
        this->wall);
  }
  for (const auto &model : this->models)
  {
    addTime(msg, "model/" + model.name + "/sensors", model.sensors,
        this->wall);
    addTime(msg, "model/" + model.name + "/plugins", model.plugins,
        this->wall);
    addTime(msg, "model/" + model.name + "/physics_estimate", model.physics,
        this->wall);
    addTime(msg, "model/" + model.name + "/scene_estimate", model.scene,
        this->wall);
  }
  return msg;
}

//////////////////////////////////////////////////
PerformanceMonitor::SystemTimer::SystemTimer(PerformanceMonitor *_monitor,
    const void *_system, PerformanceSystemPhase _phase)
    : monitor(_monitor), system(_system), phase(_phase)
{
  if (!this->monitor)
    return;
  if (this->monitor->sampleCpu.load(std::memory_order_relaxed))
    this->cpuStart = threadCpuTime();
  this->start = std::chrono::steady_clock::now();
}

//////////////////////////////////////////////////
PerformanceMonitor::SystemTimer::~SystemTimer()
{
  if (!this->monitor)
    return;
  const auto wall = std::chrono::steady_clock::now() - this->start;

  auto cpu = std::chrono::nanoseconds(-1);
  if (this->cpuStart.count() >= 0)
    cpu = threadCpuTime() - this->cpuStart;

  this->monitor->Record(this->system, this->phase, wall, cpu);
}

//////////////////////////////////////////////////
PerformanceMonitor::PerformanceMonitor(
    std::chrono::steady_clock::duration _reportPeriod,
    bool _parallelPostUpdates)
    : reportPeriod(_reportPeriod), parallelPostUpdates(_parallelPostUpdates)
{
  sensors::SetSensorTimingEnabled(true);

  this->now = std::chrono::steady_clock::now();
  this->windowStart = this->now;
}

//////////////////////////////////////////////////
std::unique_ptr<PerformanceMonitor> PerformanceMonitor::Create(
    std::chrono::steady_clock::duration _reportPeriod,
    bool _parallelPostUpdates)
{
  bool expected{false};
  if (!performanceMonitorEnabled.compare_exchange_strong(expected, true))
    return nullptr;

  return std::unique_ptr<PerformanceMonitor>(
      new PerformanceMonitor(_reportPeriod, _parallelPostUpdates));
}

//////////////////////////////////////////////////
PerformanceMonitor::~PerformanceMonitor()
{
  performanceMonitorEnabled = false;
  sensors::SetSensorTimingEnabled(false);
}

//////////////////////////////////////////////////
void PerformanceMonitor::Record(const void *_system,
    PerformanceSystemPhase _phase, std::chrono::steady_clock::duration _wall,
    std::chrono::nanoseconds _cpu)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  auto &accumulator = this->systems[_system];
  accumulator.wall += _wall;

  if (_phase == PerformanceSystemPhase::kPostUpdate &&
      this->parallelPostUpdates)
  {
    this->stepPostUpdates.emplace_back(_system, _wall);
  }
  else
  {
    accumulator.critical += _wall;
    this->stepCritical.emplace_back(_system, _wall);
  }

  if (_cpu.count() >= 0)
  {
    accumulator.sampledWall += _wall;
    accumulator.sampledCpu +=
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(_cpu);
  }
}

//////////////////////////////////////////////////
void PerformanceMonitor::EndPostUpdate()
{
  std::lock_guard<std::mutex> lock(this->mutex);
  if (this->stepPostUpdates.empty())
    return;

  // In parallel, only the slowest post update delays the step.
  const auto slowest = std::max_element(this->stepPostUpdates.begin(),
      this->stepPostUpdates.end(), [](const auto &_a, const auto &_b)
      {
        return _a.second < _b.second;
      });
  this->systems[slowest->first].critical += slowest->second;
  this->stepCritical.push_back(*slowest);
  this->stepPostUpdates.clear();
}

//////////////////////////////////////////////////
void PerformanceMonitor::OnRenderFrame(const events::RenderFrameTimes &_times)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->render.sceneUpdate += _times.sceneUpdate;
  this->render.preRender += _times.preRender;
  this->render.sensors += _times.sensors;
  this->render.postRender += _times.postRender;
  this->render.gpuWait += _times.gpuWait;
  this->render.simThreadWait += _times.simThreadWait;
  this->render.softwareRenderer = _times.softwareRenderer;
  ++this->renderFrames;
}

//////////////////////////////////////////////////
void PerformanceMonitor::OnContactSample(
    const std::unordered_map<Entity, unsigned int> &_contacts)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  for (const auto &[entity, count] : _contacts)
    this->contacts[entity] += count;
  ++this->contactSamples;
}

//////////////////////////////////////////////////
void PerformanceMonitor::Restart()
{
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->systems.clear();
    this->stepPostUpdates.clear();
    this->lateLoop = Duration::zero();
    this->render = {};
    this->renderFrames = 0u;
    this->contacts.clear();
    this->contactSamples = 0u;
  }
  sensors::TakeSensorTimings();
  this->windowStart = this->now;
  this->windowSimStart = this->simTime;
  this->sleep = Duration::zero();
  this->iterations = 0u;
}

//////////////////////////////////////////////////
void PerformanceMonitor::EndIteration(
    const std::chrono::steady_clock::time_point &_paceStart,
    const std::chrono::steady_clock::time_point &_paceEnd,
    const UpdateInfo &_info, double _desiredRtf)
{
  // A world reset takes the simulation time back to the start.
  const bool rewound = _info.simTime < this->simTime;
  const auto stepStart = this->now;

  this->now = _paceEnd;
  this->simTime = _info.simTime;
  this->desiredRtf = _desiredRtf;

  // The time that the loop lost to its schedule in this step, split among who
  // spent it. Steps that aren't part of an interval are discarded with the
  // rest of their measurements by Restart.
  this->period = _desiredRtf > 0.0 ?
      scale(_info.dt, 1.0 / _desiredRtf) : Duration::zero();
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (_info.paused)
    {
      // The loop doesn't keep a schedule while paused, it starts a new one
      // when it resumes.
      this->schedule.Clear();
    }
    else
    {
      const Duration busy = _paceStart - stepStart;
      for (const auto &[system, time] :
          this->schedule.Step(busy, this->period, this->stepCritical))
      {
        if (nullptr != system)
          this->systems[system].late += time;
        else
          this->lateLoop += time;
      }
    }
    this->stepCritical.clear();
  }

  // Time paused says nothing about performance, and there is nothing to
  // measure against. Start over when the simulation resumes.
  if (_info.paused)
  {
    this->wasPaused = true;
    return;
  }

  if (this->wasPaused || rewound)
  {
    this->wasPaused = false;
    this->warmingUp = true;
    this->Restart();
  }

  if (this->warmingUp)
  {
    if (this->now - this->windowStart >= kWarmUpTime)
    {
      this->warmingUp = false;
      this->Restart();
    }
    return;
  }

  this->sleep += _paceEnd - _paceStart;
  ++this->iterations;
  this->sampleCpu.store(this->iterations % kCpuSamplePeriod == 0u,
      std::memory_order_relaxed);
}

//////////////////////////////////////////////////
void PerformanceMonitor::StartRun(
    const std::chrono::steady_clock::time_point &_now)
{
  this->windowStart += _now - this->now;
  this->now = _now;
  // The loop starts a new schedule when it runs again.
  this->schedule.Clear();
}

//////////////////////////////////////////////////
bool PerformanceMonitor::ReportDue() const
{
  return !this->warmingUp && !this->wasPaused && this->iterations > 0u &&
      this->now - this->windowStart >= this->reportPeriod;
}

//////////////////////////////////////////////////
PerformanceReport PerformanceMonitor::TakeReport(
    const SystemManager &_systemMgr, const EntityComponentManager &_ecm)
{
  PerformanceWindow window;
  window.wall = this->now - this->windowStart;
  window.sleep = this->sleep;
  window.simTime = this->simTime - this->windowSimStart;
  window.iterations = this->iterations;
  window.desiredRtf = this->desiredRtf;
  window.period = this->period;

  std::unordered_map<const void *, SystemAccumulator> systemTimes;
  std::unordered_map<Entity, double> averageContacts;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    systemTimes = this->systems;
    window.lateLoop = this->lateLoop;
    window.render = this->render;
    window.renderFrames = this->renderFrames;
    if (this->contactSamples > 0u)
    {
      for (const auto &[entity, count] : this->contacts)
      {
        averageContacts[entity] = static_cast<double>(count) /
            static_cast<double>(this->contactSamples);
      }
    }
  }

  // One system may implement several phases through different interfaces.
  for (const auto &system : _systemMgr.Systems())
  {
    PerformanceSystemSample sample;
    sample.name = system.name.empty() ? "unnamed system" : system.name;
    sample.model = topLevelModelName(system.parentEntity, _ecm);

    bool found = false;
    const void *interfaces[] = {system.preupdate, system.update,
        system.postupdate};
    for (const void *interface : interfaces)
    {
      auto it = systemTimes.find(interface);
      if (nullptr == interface || it == systemTimes.end())
        continue;
      found = true;
      sample.wall += it->second.wall;
      sample.critical += it->second.critical;
      sample.late += it->second.late;
      sample.sampledWall += it->second.sampledWall;
      sample.sampledCpu += it->second.sampledCpu;
    }
    if (found)
      window.systems.push_back(std::move(sample));
  }

  for (auto &timing : sensors::TakeSensorTimings())
  {
    PerformanceSensorSample sample;
    // The simulation names the sensors model::link::sensor.
    sample.name = timing.name;
    const auto scope = timing.name.find("::");
    if (scope != std::string::npos)
      sample.model = timing.name.substr(0, scope);
    sample.updates = timing.updates;
    sample.total = std::chrono::duration_cast<Duration>(timing.total);
    sample.gpuWait = std::chrono::duration_cast<Duration>(timing.gpuWait);
    window.sensors.push_back(std::move(sample));
  }

  window.models = CollectPerformanceModels(_ecm, averageContacts);

  this->Restart();
  return AnalyzePerformance(window);
}
