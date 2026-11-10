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

#include "PerformanceOverlay.hh"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/diagnostics.pb.h>

#include <chrono>
#include <functional>
#include <string>
#include <vector>

#include <gz/gui/Application.hh>
#include <gz/gui/Helpers.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/gui/GuiEvents.hh"

#include "PerformanceOverlayData.hh"

using namespace gz;
using namespace sim;

namespace
{
/// \brief Reports kept for the history
constexpr int kHistorySize{40};

/// \brief Time for the server to answer before saying that it can't
constexpr std::chrono::seconds kServiceTimeout{3};

/// \brief Room left at the top of the 3D view, where the default layout has
/// its toolbars.
constexpr double kSceneToolbarsHeight{104.0};

/// \brief Gap between the panel and the edge of the 3D view
constexpr double kSceneMargin{8.0};

/// \brief Convert a report to what the QML side reads
/// \param[in] _report The report
/// \return Map of values
QVariantMap toVariant(const PerformanceOverlayReport &_report)
{
  QVariantMap map;
  map["headline"] = QString::fromStdString(_report.Headline());
  map["detail"] = QString::fromStdString(_report.Detail());
  map["limited"] = _report.limited;
  map["resource"] = QString::fromStdString(_report.resource);
  map["location"] = QString::fromStdString(_report.location);
  map["rtf"] = _report.rtf;
  map["desiredRtf"] = _report.desiredRtf;
  map["windowSeconds"] = _report.windowSeconds;
  map["iterations"] = static_cast<qulonglong>(_report.iterations);
  map["renderFrames"] = static_cast<qulonglong>(_report.renderFrames);
  map["softwareRenderer"] = _report.softwareRenderer;
  map["physics"] = _report.physics;
  map["renderWait"] = _report.renderWait;
  map["otherSystems"] = _report.otherSystems;
  map["loop"] = _report.loop;
  map["showRenderThread"] =
      _report.renderTotal >= kPerformanceOverlayMinRender;
  map["gpuWaitRatio"] = _report.GpuWaitRatio();

  QVariantList sensors;
  for (const auto &sensor : _report.sensors)
  {
    QVariantMap row;
    row["name"] = QString::fromStdString(sensor.name);
    row["label"] = QString::fromStdString(sensor.Label());
    row["model"] = QString::fromStdString(sensor.Model());
    row["share"] = sensor.share;
    row["gpuShare"] = sensor.gpuShare;
    sensors.append(row);
  }
  map["sensors"] = sensors;

  QVariantList models;
  for (const auto &model : _report.models)
  {
    QVariantMap row;
    row["name"] = QString::fromStdString(model.name);
    row["physics"] = model.physics;
    row["scene"] = model.scene;
    row["sensors"] = model.sensors;
    row["plugins"] = model.plugins;
    row["total"] = model.Total();
    models.append(row);
  }
  map["models"] = models;

  QVariantList systems;
  for (const auto &system : _report.systems)
  {
    QVariantMap row;
    row["name"] = QString::fromStdString(system.name);
    row["share"] = system.share;
    systems.append(row);
  }
  map["systems"] = systems;

  return map;
}

/// \brief Float a card at the top right corner of the 3D view
/// \param[in] _card The card of the plugin
void floatOverScene(QQuickItem *_card)
{
  auto *scene = gz::gui::findFirstByProperty(
      gz::gui::App()->Engine()->findChildren<QQuickItem *>(),
      "pluginName", QString("3D View"));
  if (!scene)
    return;

  _card->setProperty("state", "floating");
  _card->setParentItem(scene);
  QMetaObject::invokeMethod(_card, "clearAnchors");
  auto *anchors = qvariant_cast<QObject *>(_card->property("anchors"));
  anchors->setProperty("right", scene->property("right"));
  anchors->setProperty("top", scene->property("top"));
  anchors->setProperty("topMargin", kSceneToolbarsHeight);
  anchors->setProperty("rightMargin", kSceneMargin);
  _card->setProperty("anchored", true);
  _card->setProperty("z", 1.0);
}
}  // namespace

/// \brief Private data of the plugin. Everything but the transport callbacks
/// runs on the Qt thread, and those hand their results over to it.
class gz::sim::PerformanceOverlayPrivate
{
  /// \brief Show a notice
  /// \param[in] _plugin The plugin
  /// \param[in] _notice The text
  public: void SetNotice(PerformanceOverlay *_plugin, const QString &_notice)
  {
    if (this->notice == _notice)
      return;
    this->notice = _notice;
    emit _plugin->NoticeChanged();
  }

  /// \brief Store a new report, on the Qt thread
  /// \param[in] _plugin The plugin
  /// \param[in] _report The report
  public: void OnReport(PerformanceOverlay *_plugin,
      const PerformanceOverlayReport &_report)
  {
    this->report = toVariant(_report);

    QVariantMap sample;
    sample["rtf"] = _report.rtf;
    sample["limited"] = _report.limited;
    sample["location"] = QString::fromStdString(_report.location);
    this->history.append(sample);
    while (this->history.size() > kHistorySize)
      this->history.removeFirst();

    this->SetNotice(_plugin, "");
    emit _plugin->ReportChanged();
  }

  /// \brief Store the answer of the server, on the Qt thread
  /// \param[in] _plugin The plugin
  /// \param[in] _enable What was requested
  /// \param[in] _accepted True if the server is now in that state
  public: void OnMonitoring(PerformanceOverlay *_plugin, bool _enable,
      bool _accepted)
  {
    this->waitingForService = false;
    this->monitoring = _enable && _accepted;
    emit _plugin->MonitoringChanged();

    if (_enable && !_accepted)
    {
      this->SetNotice(_plugin,
          "Not monitored: another world in this process already is.");
    }
    else if (this->monitoring && this->report.isEmpty())
    {
      this->SetNotice(_plugin, "Measuring, the first report is on its way.");
    }
    else if (!this->monitoring)
    {
      this->SetNotice(_plugin, "The monitor is off.");
    }
    else
    {
      this->SetNotice(_plugin, "");
    }
  }

  /// \brief Communication with the server
  public: transport::Node node;

  /// \brief Name of the world, empty until it is known
  public: std::string worldName;

  /// \brief Last report
  public: QVariantMap report;

  /// \brief Last reports, oldest first
  public: QVariantList history;

  /// \brief Why there is nothing new to show
  public: QString notice{"Connecting to the server."};

  /// \brief True if the server is monitoring
  public: bool monitoring{false};

  /// \brief True if the simulation is paused
  public: bool paused{false};

  /// \brief True to float over the scene once the card has a parent
  public: bool placeOverScene{false};

  /// \brief True while the answer of the server is pending
  public: bool waitingForService{false};

  /// \brief When the server was asked
  public: std::chrono::steady_clock::time_point serviceRequestTime;

  /// \brief Model to select on the next update
  public: std::string modelToSelect;
};

/////////////////////////////////////////////////
PerformanceOverlay::PerformanceOverlay()
  : GuiSystem(), dataPtr(std::make_unique<PerformanceOverlayPrivate>())
{
}

/////////////////////////////////////////////////
PerformanceOverlay::~PerformanceOverlay() = default;

/////////////////////////////////////////////////
void PerformanceOverlay::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
    this->title = "Performance";

  this->dataPtr->placeOverScene =
      !_pluginElem || !_pluginElem->FirstChildElement("gz-gui");
}

/////////////////////////////////////////////////
void PerformanceOverlay::Update(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  if (this->dataPtr->placeOverScene && this->CardItem()->parentItem())
  {
    this->dataPtr->placeOverScene = false;
    floatOverScene(this->CardItem());
  }

  if (this->dataPtr->worldName.empty())
  {
    _ecm.Each<components::World, components::Name>(
        [&](const Entity &, const components::World *,
            const components::Name *_name)
        {
          this->dataPtr->worldName = _name->Data();
          return false;
        });
    if (this->dataPtr->worldName.empty())
      return;

    const std::function<void(const msgs::Diagnostics &)> onReport =
        [this](const msgs::Diagnostics &_msg)
        {
          QMetaObject::invokeMethod(this,
              [this, report = ParsePerformanceReport(_msg)]()
              {
                this->dataPtr->OnReport(this, report);
              }, Qt::QueuedConnection);
        };
    this->dataPtr->node.Subscribe(
        "/world/" + this->dataPtr->worldName + "/performance/report",
        onReport);
    this->SetMonitoring(true);
  }

  if (this->dataPtr->waitingForService &&
      std::chrono::steady_clock::now() - this->dataPtr->serviceRequestTime >
      kServiceTimeout)
  {
    this->dataPtr->waitingForService = false;
    this->dataPtr->SetNotice(this,
        "This server does not report its performance.");
  }

  if (_info.paused != this->dataPtr->paused)
  {
    this->dataPtr->paused = _info.paused;
    emit this->PausedChanged();
  }

  if (this->dataPtr->modelToSelect.empty())
    return;

  Entity selected{kNullEntity};
  _ecm.Each<components::Model, components::Name, components::ParentEntity>(
      [&](const Entity &_entity, const components::Model *,
          const components::Name *_name,
          const components::ParentEntity *_parent)
      {
        if (_name->Data() != this->dataPtr->modelToSelect ||
            !_ecm.Component<components::World>(_parent->Data()))
        {
          return true;
        }
        selected = _entity;
        return false;
      });
  this->dataPtr->modelToSelect.clear();
  if (selected == kNullEntity)
    return;

  auto *window = gz::gui::App()->findChild<gz::gui::MainWindow *>();
  gui::events::DeselectAllEntities deselect(true);
  gz::gui::App()->sendEvent(window, &deselect);
  gui::events::EntitiesSelected select({selected}, true);
  gz::gui::App()->sendEvent(window, &select);
}

/////////////////////////////////////////////////
QVariantMap PerformanceOverlay::Report() const
{
  return this->dataPtr->report;
}

/////////////////////////////////////////////////
QVariantList PerformanceOverlay::History() const
{
  return this->dataPtr->history;
}

/////////////////////////////////////////////////
QString PerformanceOverlay::Notice() const
{
  return this->dataPtr->notice;
}

/////////////////////////////////////////////////
bool PerformanceOverlay::Monitoring() const
{
  return this->dataPtr->monitoring;
}

/////////////////////////////////////////////////
bool PerformanceOverlay::Paused() const
{
  return this->dataPtr->paused;
}

/////////////////////////////////////////////////
void PerformanceOverlay::SetMonitoring(bool _enable)
{
  if (this->dataPtr->worldName.empty())
    return;

  msgs::Boolean request;
  request.set_data(_enable);
  std::function<void(const msgs::Boolean &, const bool)> onAnswer =
      [this, _enable](const msgs::Boolean &_answer, const bool _result)
      {
        const bool accepted = _result && _answer.data() == _enable;
        QMetaObject::invokeMethod(this, [this, _enable, accepted]()
            {
              this->dataPtr->OnMonitoring(this, _enable, accepted);
            }, Qt::QueuedConnection);
      };

  this->dataPtr->waitingForService = true;
  this->dataPtr->serviceRequestTime = std::chrono::steady_clock::now();
  this->dataPtr->node.Request(
      "/world/" + this->dataPtr->worldName + "/performance/enable", request,
      onAnswer);
}

/////////////////////////////////////////////////
void PerformanceOverlay::SelectModel(const QString &_name)
{
  this->dataPtr->modelToSelect = _name.toStdString();
}

// Register this plugin
GZ_ADD_PLUGIN(gz::sim::PerformanceOverlay,
              gz::gui::Plugin)
