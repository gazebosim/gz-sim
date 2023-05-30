/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <gz/msgs/marker.pb.h>

#include <mutex>
#include <string>

#include <gz/plugin/Register.hh>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <gz/transport/Node.hh>

#include <gz/gui/Application.hh>
#include <gz/gui/Conversions.hh>
#include <gz/gui/MainWindow.hh>

#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/gui/GuiEvents.hh"
#include "gz/sim/Util.hh"

#include "Plot3D.hh"

namespace gz::sim::gui
{
  /// \brief Private data class for Plot3D
  class Plot3DPrivate
  {
    /// \brief Transport node.
    public: transport::Node node;

    /// \brief Whether currently locked on a given entity
    public: bool locked{false};

    /// \brief Current target entity.
    public: Entity targetEntity{kNullEntity};

    /// \brief Name of the target entity.
    public: std::string targetName;

    /// \brief Whether the target entity has been changed.
    public: bool targetEntityDirty{false};

    /// \brief Whether the target name has been changed.
    public: bool targetNameDirty{false};

    /// \brief Current marker message.
    public: msgs::Marker markerMsg;

    /// \brief Marker color.
    public: math::Color color{math::Color::Blue};

    /// \brief Previous plotted position.
    public: math::Vector3d prevPos;

    /// \brief Offset from entity origin to place plot point.
    /// The offset is expressed in the entity's frame.
    public: math::Vector3d offset;

    /// \brief Minimum distance between points. If the object has moved by less
    /// than this distance, a new point isn't plotted.
    public: double minDistance{0.05};

    /// \brief Maximum number of points in the plot. When the plot reaches this
    /// size, older points are deleted.
    public: int maxPoints{1000};

    /// \brief Protects variables that are updated by the user.
    public: std::mutex mutex;
  };
}

using namespace gz;
using namespace gz::sim;
using namespace gz::sim::gui;

/////////////////////////////////////////////////
Plot3D::Plot3D()
  : GuiSystem(), dataPtr(std::make_unique<Plot3DPrivate>())
{
  qRegisterMetaType<Entity>("Entity");
}

/////////////////////////////////////////////////
Plot3D::~Plot3D()
{
  this->ClearPlot();
}

/////////////////////////////////////////////////
void Plot3D::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
    this->title = "3D Plot";

  // Parameters from SDF
  if (_pluginElem)
  {
    auto nameElem = _pluginElem->FirstChildElement("entity_name");
    if (nullptr != nameElem && nullptr != nameElem->GetText())
    {
      this->dataPtr->targetName = nameElem->GetText();
      this->dataPtr->targetNameDirty = true;
      this->SetLocked(true);
    }

    auto offsetElem = _pluginElem->FirstChildElement("offset");
    if (nullptr != offsetElem && nullptr != offsetElem->GetText())
    {
      std::stringstream offsetStr;
      offsetStr << std::string(offsetElem->GetText());
      offsetStr >> this->dataPtr->offset;
      this->OffsetChanged();
    }

    auto colorElem = _pluginElem->FirstChildElement("color");
    if (nullptr != colorElem && nullptr != colorElem->GetText())
    {
      std::stringstream colorStr;
      colorStr << std::string(colorElem->GetText());
      colorStr >> this->dataPtr->color;
      this->ColorChanged();
    }

    auto distElem = _pluginElem->FirstChildElement("minimum_distance");
    if (nullptr != distElem && nullptr != distElem->GetText())
    {
      distElem->QueryDoubleText(&this->dataPtr->minDistance);
      this->MinDistanceChanged();
    }

    auto ptsElem = _pluginElem->FirstChildElement("maximum_points");
    if (nullptr != ptsElem && nullptr != ptsElem->GetText())
    {
      ptsElem->QueryIntText(&this->dataPtr->maxPoints);
      this->MaxPointsChanged();
    }
  }

  gz::gui::App()->findChild<
      gz::gui::MainWindow *>()->installEventFilter(this);
}

/////////////////////////////////////////////////
void Plot3D::ClearPlot()
{
  // Clear previous plot
  if (this->dataPtr->markerMsg.point().size() > 0)
  {
    this->dataPtr->markerMsg.set_action(msgs::Marker::DELETE_MARKER);
    this->dataPtr->node.Request("/marker", this->dataPtr->markerMsg);
  }
}

//////////////////////////////////////////////////
void Plot3D::Update(const UpdateInfo &, EntityComponentManager &_ecm)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  bool newTarget{false};

  // New target by name, get entity
  if (this->dataPtr->targetNameDirty)
  {
    auto entities = entitiesFromScopedName(this->dataPtr->targetName, _ecm);
    if (entities.empty())
    {
      // Keep trying
      return;
    }

    Entity entity = *(entities.begin());

    if (kNullEntity == entity)
    {
      // Keep trying
      return;
    }

    this->dataPtr->targetEntity = entity;
    this->dataPtr->targetNameDirty = false;
    newTarget = true;
  }

  // New target by entity, get name
  if (this->dataPtr->targetEntityDirty)
  {
    this->dataPtr->targetEntityDirty = false;

    auto name = _ecm.ComponentData<components::Name>(
        this->dataPtr->targetEntity);
    if (!name)
    {
      this->dataPtr->targetName.clear();
      return;
    }
    this->dataPtr->targetName = name.value();

    newTarget = true;
  }

  if (newTarget)
  {
    this->ClearPlot();

    // Reset message
    this->dataPtr->markerMsg.Clear();
    this->dataPtr->markerMsg.set_ns("plot_" + this->dataPtr->targetName);
    this->dataPtr->markerMsg.set_id(this->dataPtr->targetEntity);
    this->dataPtr->markerMsg.set_action(msgs::Marker::ADD_MODIFY);
    this->dataPtr->markerMsg.set_type(msgs::Marker::LINE_STRIP);
    this->dataPtr->markerMsg.set_visibility(msgs::Marker::GUI);

    // Update view
    this->TargetEntityChanged();
    this->TargetNameChanged();
  }

  // Get entity pose
  auto pose = worldPose(this->dataPtr->targetEntity, _ecm);
  math::Pose3d offsetPose;
  offsetPose.Set(this->dataPtr->offset, math::Vector3d::Zero);

  auto point = (pose * offsetPose).Pos();

  // Only add points if the distance is past a threshold.
  if (point.Distance(this->dataPtr->prevPos) < this->dataPtr->minDistance)
    return;

  this->dataPtr->prevPos = point;
  msgs::Set(this->dataPtr->markerMsg.add_point(), point);

  // Reduce message array
  if (this->dataPtr->markerMsg.point_size() > this->dataPtr->maxPoints)
    this->dataPtr->markerMsg.mutable_point()->DeleteSubrange(0, 5);

  msgs::Set(this->dataPtr->markerMsg.mutable_material()->mutable_ambient(),
    this->dataPtr->color);
  msgs::Set(this->dataPtr->markerMsg.mutable_material()->mutable_diffuse(),
    this->dataPtr->color);

  // Request
  this->dataPtr->node.Request("/marker", this->dataPtr->markerMsg);
}

/////////////////////////////////////////////////
bool Plot3D::eventFilter(QObject *_obj, QEvent *_event)
{
  if (!this->dataPtr->locked)
  {
    if (_event->type() == sim::gui::events::EntitiesSelected::kType)
    {
      auto event = reinterpret_cast<gui::events::EntitiesSelected *>(_event);
      if (event && !event->Data().empty())
      {
        this->SetTargetEntity(*event->Data().begin());
      }
    }

    if (_event->type() == sim::gui::events::DeselectAllEntities::kType)
    {
      auto event = reinterpret_cast<gui::events::DeselectAllEntities *>(
          _event);
      if (event)
      {
        this->SetTargetEntity(kNullEntity);
      }
    }
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

/////////////////////////////////////////////////
Entity Plot3D::TargetEntity() const
{
  return this->dataPtr->targetEntity;
}

/////////////////////////////////////////////////
void Plot3D::SetTargetEntity(Entity _entity)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->targetEntity = _entity;
  this->dataPtr->targetEntityDirty = true;
  this->TargetEntityChanged();

  if (this->dataPtr->targetEntity == kNullEntity)
  {
    this->dataPtr->targetName.clear();
  }
}

/////////////////////////////////////////////////
QString Plot3D::TargetName() const
{
  return QString::fromStdString(this->dataPtr->targetName);
}

/////////////////////////////////////////////////
void Plot3D::SetTargetName(const QString &_targetName)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->targetName = _targetName.toStdString();
  this->dataPtr->targetNameDirty = true;
  this->TargetNameChanged();
}

/////////////////////////////////////////////////
bool Plot3D::Locked() const
{
  return this->dataPtr->locked;
}

/////////////////////////////////////////////////
void Plot3D::SetLocked(bool _locked)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->locked = _locked;
  this->LockedChanged();
}

/////////////////////////////////////////////////
QVector3D Plot3D::Offset() const
{
  return QVector3D(
      this->dataPtr->offset.X(),
      this->dataPtr->offset.Y(),
      this->dataPtr->offset.Z());
}

/////////////////////////////////////////////////
void Plot3D::SetOffset(const QVector3D &_offset)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->offset.Set(_offset.x(), _offset.y(), _offset.z());
  this->OffsetChanged();
}

/////////////////////////////////////////////////
QVector3D Plot3D::Color() const
{
  return QVector3D(
      this->dataPtr->color.R(),
      this->dataPtr->color.G(),
      this->dataPtr->color.B());
}

/////////////////////////////////////////////////
void Plot3D::SetColor(const QVector3D &_color)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->color.Set(_color.x(), _color.y(), _color.z());
  this->ColorChanged();
}

/////////////////////////////////////////////////
double Plot3D::MinDistance() const
{
  return this->dataPtr->minDistance;
}

/////////////////////////////////////////////////
void Plot3D::SetMinDistance(double _minDistance)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->minDistance = _minDistance;
  this->MinDistanceChanged();
}

/////////////////////////////////////////////////
int Plot3D::MaxPoints() const
{
  return this->dataPtr->maxPoints;
}

/////////////////////////////////////////////////
void Plot3D::SetMaxPoints(int _maxPoints)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->maxPoints = _maxPoints;
  this->MaxPointsChanged();
}

// Register this plugin
GZ_ADD_PLUGIN(Plot3D,
              gz::gui::Plugin)
