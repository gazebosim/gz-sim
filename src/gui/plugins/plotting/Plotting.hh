/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#ifndef IGNITION_GUI_PLUGINS_PLOTTING_HH_
#define IGNITION_GUI_PLUGINS_PLOTTING_HH_

#include <ignition/gui/qt.h>
#include <ignition/gui/Application.hh>
#include <ignition/gui/PlottingInterface.hh>
#include <ignition/gazebo/gui/GuiSystem.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include <map>
#include <string>
#include <memory>

#include "ignition/gazebo/components/Actor.hh"
#include "ignition/gazebo/components/AngularAcceleration.hh"
#include "ignition/gazebo/components/AngularVelocity.hh"
#include "ignition/gazebo/components/CastShadows.hh"
#include "ignition/gazebo/components/ChildLinkName.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/Factory.hh"
#include "ignition/gazebo/components/Gravity.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/Level.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/LinearAcceleration.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/LinearVelocitySeed.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/MagneticField.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/ParentLinkName.hh"
#include "ignition/gazebo/components/Performer.hh"
#include "ignition/gazebo/components/PerformerAffinity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/PoseCmd.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/Static.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/WindMode.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

namespace ignition {

namespace gazebo {

class PlotComponentPrivate;

class PlotComponent
{
  /// \brief Constructor
  /// \param[in] _type component data type (Pose3d, Vector3d, double)
  /// \param [in] _entity entity id of that component
  /// \param [in] _typeId type identifier unique to each component type
  public: PlotComponent(std::string _type, uint64_t _entity, uint64_t _typeId);

  /// \brief Add a registered chart to the attribute
  /// \param[in] _attribute component attribute to add the chart to it
  /// \param[in] _chart chart ID to be added to the attribute
  public: void RegisterChart(std::string _attribute, int _chart);

  /// \brief Remove a registered chart from the attribute
  /// \param[in] _attribute component attribute to remove the chart from it
  /// \param[in] _chart chart ID to be removed from the attribute
  public: void UnRegisterChart(std::string _attribute, int _chart);

  /// \brief Check if any of the component attributes has any chart
  /// \return true if any attribute has a chart
  /// false if all attributes are empty from the charts
  public: bool HasCharts();

  /// \brief Set a value of specefic component attribute
  /// \param[in] _attribute component attribute to set its value
  /// ex : yaw attribute in Pose3d type Component
  /// \param[in] _value value to be set to the attribute
  public: void SetAttributeValue(std::string _attribute, const double &_value);

  /// \brief Get all attributes of the component
  /// \return component attributes
  public: std::map<std::string, ignition::gui::PlotData*> Data() const;

  /// \brief Get the Component entity ID
  /// \return entity ID
  public: uint64_t Entity();

  /// \brief Get the Component type ID
  /// \return component type ID
  public: uint64_t TypeId();

  /// \brief dataPtr holds Abstraction data of PlottingPrivate
  private: std::unique_ptr<PlotComponentPrivate> dataPtr;
};

class PlottingPrivate;

class Plotting : public ignition::gazebo::GuiSystem
{
  Q_OBJECT
  /// \brief Constructor
  public: Plotting();

  /// \brief Destructor
  public: ~Plotting();

    /// \brief called every simulation iteration to access components
  public: void Update(const ignition::gazebo::UpdateInfo &_info,
                      ignition::gazebo::EntityComponentManager &_ecm) override;

  /// \brief Set the Component data of giving id to the giving vector
  /// \param [in] _Id Component Key of the components map
  /// \param [in] _vector Vector Data to be set to the component
  public: void SetData(std::string _Id,
                       const ignition::math::Vector3d &_vector);

  /// \brief Set the Component data of giving id to the giving pose
  /// \param [in] _Id Component Key of the components map
  /// \param [in] _pose Position Data to be set to the component
  public: void SetData(std::string _Id,
                       const ignition::math::Pose3d &_pose);

  /// \brief Set the Component data of giving id to the giving number
  /// \param [in] _Id Component Key of the components map
  /// \param [in] _value double Data to be set to the component
  /// valid for types (double, float, int, bool)
  public: void SetData(std::string _Id, const double &_value);

  /// \brief Add a chart to a specefic component attribute
  /// \param [in] _entity entity id in the simulation
  /// \param [in] _typeId type identifier unique to each component type
  /// \param [in] _attribute component attribute to add the chart to it
  /// ex: x attribute in Pose3d Component
  /// \param [in] _chart chart ID to be registered
  public slots: void RegisterChartToComponent(uint64_t _entity,
                                              uint64_t _typeId,
                                              std::string _type,
                                              std::string _attribute,
                                              int _chart);

  /// \brief Remove a chart from a specefic component attribute
  /// \param [in] _entity entity id in the simulation
  /// \param [in] _typeId type identifier unique to each component type
  /// \param [in] _attribute component attribute to remove the chart from it
  /// ex: x attribute in Pose3d Component
  /// \param [in] _chart chart ID to be unregistered
  public slots: void UnRegisterChartToComponent(uint64_t _entity,
                                                uint64_t _typeId,
                                                std::string _attribute,
                                                int _chart);

  /// \brief dataPtr holds Abstraction data of PlottingPrivate
  private: std::unique_ptr<PlottingPrivate> dataPtr;
};
}
}

#endif
