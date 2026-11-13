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
#ifndef GZ_SIM_GUI_PERFORMANCEOVERLAYDATA_HH_
#define GZ_SIM_GUI_PERFORMANCEOVERLAYDATA_HH_

#include <gz/msgs/diagnostics.pb.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "gz/sim/config.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE
{
  /// \brief Rows kept for each list of the overlay.
  constexpr std::size_t kPerformanceOverlayRows{3u};

  /// \brief Rows below this share of the wall time would all read 0%.
  constexpr double kPerformanceOverlayMinShare{0.005};

  /// \brief Below this share of the wall time, the render thread is too idle
  /// for its split between CPU and GPU to mean anything.
  constexpr double kPerformanceOverlayMinRender{0.05};

  /// \brief A sensor of a performance report. Shares are fractions of the
  /// wall time of the report.
  struct PerformanceOverlaySensor
  {
    /// \brief Scoped name, model::link::sensor
    std::string name;

    /// \brief Time spent updating the sensor
    double share{0.0};

    /// \brief Part of that time spent waiting for the GPU
    double gpuShare{0.0};

    /// \brief Model the sensor belongs to, empty if the name has no scope.
    /// \return The top level model name
    public: std::string Model() const
    {
      const auto scope = this->name.find("::");
      return scope == std::string::npos ? "" : this->name.substr(0, scope);
    }

    /// \brief Short name for display, model::sensor
    /// \return The name without the link
    public: std::string Label() const
    {
      const auto scope = this->name.rfind("::");
      if (scope == std::string::npos)
        return this->name;
      return this->Model() + this->name.substr(scope);
    }
  };

  /// \brief What a top level model costs. Shares are fractions of the wall
  /// time of the report.
  struct PerformanceOverlayModel
  {
    /// \brief Name of the model
    std::string name;

    /// \brief Estimated share of the physics
    double physics{0.0};

    /// \brief Estimated share of the scene update
    double scene{0.0};

    /// \brief Measured time of its sensors
    double sensors{0.0};

    /// \brief Measured time of its plugins
    double plugins{0.0};

    /// \brief Sum of all costs
    /// \return The share of the wall time
    public: double Total() const
    {
      return this->physics + this->scene + this->sensors + this->plugins;
    }
  };

  /// \brief A system of a performance report
  struct PerformanceOverlaySystem
  {
    /// \brief Name of the system, without its namespaces
    std::string name;

    /// \brief Time on the critical path of the step, as a fraction of the
    /// wall time of the report.
    double share{0.0};
  };

  /// \brief A performance report ready to display. It is read from the message
  /// published on /world/<name>/performance/report.
  struct PerformanceOverlayReport
  {
    /// \brief Real time factor measured
    double rtf{0.0};

    /// \brief Real time factor requested, 0 when running as fast as possible
    double desiredRtf{0.0};

    /// \brief Length of the report in seconds
    double windowSeconds{0.0};

    /// \brief True if the simulation does not reach its target
    bool limited{false};

    /// \brief True if rendering runs on a software rasterizer
    bool softwareRenderer{false};

    /// \brief cpu, gpu, cpu_and_gpu, external, unknown or none
    std::string resource;

    /// \brief physics, rendering, system, engine or none
    std::string location;

    /// \brief The sensor, model or system named by the monitor
    std::string culprit;

    /// \brief Steps in the report
    uint64_t iterations{0u};

    /// \brief Frames rendered in the report
    uint64_t renderFrames{0u};

    /// \brief Share of the wall time on physics
    double physics{0.0};

    /// \brief Share of the wall time waiting for rendering
    double renderWait{0.0};

    /// \brief Share of the wall time on other systems
    double otherSystems{0.0};

    /// \brief Share of the wall time on the simulation loop
    double loop{0.0};

    /// \brief Share of the wall time the render thread was working
    double renderTotal{0.0};

    /// \brief Share of the wall time the render thread waited for the GPU
    double renderGpuWait{0.0};

    /// \brief Slowest sensors first
    std::vector<PerformanceOverlaySensor> sensors;

    /// \brief Most expensive models first
    std::vector<PerformanceOverlayModel> models;

    /// \brief Slowest systems first, without physics and sensors
    std::vector<PerformanceOverlaySystem> systems;

    /// \brief The verdict in a few words
    /// \return Text to display
    public: std::string Headline() const
    {
      if (!this->limited)
        return "Keeping up";
      if (this->resource == "cpu")
        return "Limited by the CPU";
      if (this->resource == "gpu")
        return "Limited by the GPU";
      if (this->resource == "cpu_and_gpu")
        return "Limited by CPU and GPU";
      if (this->resource == "external")
        return "Waiting on something external";
      return "Limited, resource unknown";
    }

    /// \brief Where the time goes and who is responsible
    /// \return Text to display
    public: std::string Detail() const
    {
      std::string where;
      if (this->location == "physics")
        where = "Physics";
      else if (this->location == "rendering")
        where = "Rendering";
      else if (this->location == "system")
        where = "System";
      else if (this->location == "engine")
        where = "Simulation loop";

      std::string detail = where;
      if (!this->culprit.empty())
      {
        detail += (detail.empty() ? "" : ": ") + this->culprit;
        if (this->location == "physics")
          detail += " (estimated)";
      }
      if (!this->limited && !detail.empty())
        return "Largest use is " + detail;
      return detail;
    }

    /// \brief Share of the render thread blocked on the GPU
    /// \return Number between 0 and 1
    public: double GpuWaitRatio() const
    {
      if (this->renderTotal <= 0.0)
        return 0.0;
      return std::clamp(this->renderGpuWait / this->renderTotal, 0.0, 1.0);
    }
  };

  namespace performance_overlay
  {
    /// \brief Convert a message time to seconds
    /// \param[in] _time The time
    /// \return Seconds
    inline double Seconds(const msgs::Time &_time)
    {
      return static_cast<double>(_time.sec()) +
          static_cast<double>(_time.nsec()) * 1e-9;
    }

    /// \brief Remove a prefix from a string
    /// \param[in, out] _text String to change
    /// \param[in] _prefix The prefix
    /// \return True if the string started with the prefix
    inline bool ConsumePrefix(std::string &_text, const std::string &_prefix)
    {
      if (_text.compare(0, _prefix.size(), _prefix) != 0)
        return false;
      _text.erase(0, _prefix.size());
      return true;
    }

    /// \brief Remove a suffix from a string
    /// \param[in, out] _text String to change
    /// \param[in] _suffix The suffix
    /// \return True if the string ended with the suffix
    inline bool ConsumeSuffix(std::string &_text, const std::string &_suffix)
    {
      if (_text.size() < _suffix.size() ||
          _text.compare(_text.size() - _suffix.size(), _suffix.size(),
          _suffix) != 0)
      {
        return false;
      }
      _text.resize(_text.size() - _suffix.size());
      return true;
    }

    /// \brief Remove the namespaces and the model of a system entry
    /// \param[in] _name Name in the report, namespace::Name\@model
    /// \return The bare name
    inline std::string SystemName(const std::string &_name)
    {
      std::string name = _name.substr(0, _name.find('@'));
      const auto scope = name.rfind("::");
      return scope == std::string::npos ? name : name.substr(scope + 2);
    }

    /// \brief Store one entry of the message that isn't a fixed field
    /// \param[in] _name Name of the entry
    /// \param[in] _share Share of the wall time
    /// \param[in, out] _models Models by name
    /// \param[in, out] _sensors Sensors by name
    /// \param[in, out] _report Report to fill
    inline void StoreEntry(std::string _name, const double _share,
        std::map<std::string, PerformanceOverlayModel> &_models,
        std::map<std::string, PerformanceOverlaySensor> &_sensors,
        PerformanceOverlayReport &_report)
    {
      if (ConsumePrefix(_name, "sensor/"))
      {
        const bool gpuWait = ConsumeSuffix(_name, "/gpu_wait");
        auto &sensor = _sensors[_name];
        sensor.name = _name;
        (gpuWait ? sensor.gpuShare : sensor.share) = _share;
      }
      else if (ConsumePrefix(_name, "system/"))
      {
        const std::string name = SystemName(_name);
        if (name != "Physics" && name != "Sensors")
          _report.systems.push_back({name, _share});
      }
      else if (ConsumePrefix(_name, "model/"))
      {
        double PerformanceOverlayModel::*field{nullptr};
        if (ConsumeSuffix(_name, "/physics_estimate"))
          field = &PerformanceOverlayModel::physics;
        else if (ConsumeSuffix(_name, "/scene_estimate"))
          field = &PerformanceOverlayModel::scene;
        else if (ConsumeSuffix(_name, "/sensors"))
          field = &PerformanceOverlayModel::sensors;
        else if (ConsumeSuffix(_name, "/plugins"))
          field = &PerformanceOverlayModel::plugins;
        else
          return;
        auto &model = _models[_name];
        model.name = _name;
        model.*field = _share;
      }
    }

    /// \brief Sort a list, largest first, and drop what would read 0%
    /// \param[in, out] _rows The list
    /// \param[in] _value Function that returns the value of a row
    template <typename RowT, typename ValueT>
    void Rank(std::vector<RowT> &_rows, ValueT _value)
    {
      std::stable_sort(_rows.begin(), _rows.end(),
          [&](const RowT &_a, const RowT &_b)
          {
            return _value(_a) > _value(_b);
          });
      _rows.erase(std::remove_if(_rows.begin(), _rows.end(),
          [&](const RowT &_row)
          {
            return _value(_row) < kPerformanceOverlayMinShare;
          }), _rows.end());
      if (_rows.size() > kPerformanceOverlayRows)
        _rows.resize(kPerformanceOverlayRows);
    }
  }  // namespace performance_overlay

  /// \brief Read a performance report message
  /// \param[in] _msg Message from the performance monitor
  /// \return The report
  inline PerformanceOverlayReport ParsePerformanceReport(
      const msgs::Diagnostics &_msg)
  {
    using performance_overlay::Seconds;

    PerformanceOverlayReport report;
    report.rtf = _msg.real_time_factor();
    report.windowSeconds = Seconds(_msg.real_time());

    for (const auto &data : _msg.header().data())
    {
      if (data.value_size() < 1)
        continue;
      const std::string &value = data.value(0);
      if (data.key() == "limited")
        report.limited = value == "true";
      else if (data.key() == "software_renderer")
        report.softwareRenderer = value == "true";
      else if (data.key() == "resource")
        report.resource = value;
      else if (data.key() == "location")
        report.location = value;
      else if (data.key() == "culprit")
        report.culprit = value;
      else if (data.key() == "desired_rtf")
        report.desiredRtf = std::strtod(value.c_str(), nullptr);
      else if (data.key() == "iterations")
        report.iterations = std::strtoull(value.c_str(), nullptr, 10);
      else if (data.key() == "render_frames")
        report.renderFrames = std::strtoull(value.c_str(), nullptr, 10);
    }

    const std::map<std::string, double PerformanceOverlayReport::*> fields{
      {"step/physics", &PerformanceOverlayReport::physics},
      {"step/render_wait", &PerformanceOverlayReport::renderWait},
      {"step/other_systems", &PerformanceOverlayReport::otherSystems},
      {"step/loop", &PerformanceOverlayReport::loop},
      {"render/total", &PerformanceOverlayReport::renderTotal},
      {"render/gpu_wait", &PerformanceOverlayReport::renderGpuWait}};

    std::map<std::string, PerformanceOverlayModel> models;
    std::map<std::string, PerformanceOverlaySensor> sensors;
    for (const auto &entry : _msg.time())
    {
      const double wall = Seconds(entry.wall());
      const double share = wall > 0.0 ? Seconds(entry.elapsed()) / wall : 0.0;
      const auto field = fields.find(entry.name());
      if (field != fields.end())
        report.*(field->second) = share;
      else
        performance_overlay::StoreEntry(entry.name(), share, models, sensors,
            report);
    }

    for (auto &sensor : sensors)
      report.sensors.push_back(std::move(sensor.second));
    for (auto &model : models)
      report.models.push_back(std::move(model.second));

    performance_overlay::Rank(report.sensors,
        [](const PerformanceOverlaySensor &_row) { return _row.share; });
    performance_overlay::Rank(report.models,
        [](const PerformanceOverlayModel &_row) { return _row.Total(); });
    performance_overlay::Rank(report.systems,
        [](const PerformanceOverlaySystem &_row) { return _row.share; });
    return report;
  }
}
}  // namespace sim
}  // namespace gz

#endif
