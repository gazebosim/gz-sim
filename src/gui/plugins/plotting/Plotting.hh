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
#ifndef GZ_GUI_PLUGINS_PLOTTING_HH_
#define GZ_GUI_PLUGINS_PLOTTING_HH_

#include <gz/gui/qt.h>
#include <gz/gui/Application.hh>
#include <gz/gui/PlottingInterface.hh>
#include <gz/sim/gui/GuiSystem.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/SphericalCoordinates.hh>
#include <gz/msgs/light.pb.h>

#include "sdf/Physics.hh"

#include <map>
#include <string>
#include <memory>

namespace gz {

namespace sim {

class PlotComponentPrivate;

/// \brief A container of the component data that keeps track of the registered
/// attributes and update their values and their registered charts
class PlotComponent
{
  /// \brief Constructor
  /// \param[in] _type component data type (Pose3d, Vector3d, double)
  /// \param [in] _entity entity id of that component
  /// \param [in] _typeId type identifier unique to each component type
  public: PlotComponent(const std::string &_type,
                        gz::sim::Entity _entity,
                        ComponentTypeId _typeId);

  /// \brief Destructor
  public: ~PlotComponent();

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
  public: std::map<std::string, std::shared_ptr<gz::gui::PlotData>>
    Data() const;

  /// \brief Get the Component entity ID
  /// \return Entity ID
  public: gz::sim::Entity Entity();

  /// \brief Get the Component type ID
  /// \return component type ID
  public: ComponentTypeId TypeId();

  /// \brief dataPtr holds Abstraction data of PlottingPrivate
  private: std::unique_ptr<PlotComponentPrivate> dataPtr;
};

class PlottingPrivate;

/// \brief Physics data plotting handler that keeps track of the
/// registered components, update them and update the plot
class Plotting : public gz::sim::GuiSystem
{
  Q_OBJECT
  /// \brief Constructor
  public: Plotting();

  /// \brief Destructor
  public: ~Plotting();

  // Documentation inherited
  public: void LoadConfig(const tinyxml2::XMLElement *) override;

  // Documentation inherited
  public: void Update(const gz::sim::UpdateInfo &_info,
                      gz::sim::EntityComponentManager &_ecm) override;

  /// \brief Set the Component data of given id to the given vector
  /// \param [in] _Id Component Key of the components map
  /// \param [in] _vector Vector Data to be set to the component
  public: void SetData(std::string _Id,
                       const gz::math::Vector3d &_vector);

   /// \brief Set the Component data of given id to the given light
   /// \param [in] _Id Component Key of the components map
   /// \param [in] _light Vector Data to be set to the component
   public: void SetData(std::string _Id,
                        const msgs::Light &_light);

  /// \brief Set the Component data of given id to the given pose
  /// \param [in] _Id Component Key of the components map
  /// \param [in] _pose Position Data to be set to the component
  public: void SetData(std::string _Id,
                       const gz::math::Pose3d &_pose);

  /// \brief Set the Component data of given id to the given spherical
  /// coordinates.
  /// \param [in] _Id Component Key of the components map
  /// \param [in] _sc Data to be set to the component
  public: void SetData(std::string _Id, const math::SphericalCoordinates &_sc);

  /// \brief Set the Component data of given id to the given
  /// physics properties
  /// \param [in] _Id Component Key of the components map
  /// \param [in] _value physics Data to be set to the component
  public: void SetData(std::string _Id, const sdf::Physics &_physics);

  /// \brief Set the Component data of given id to the given number
  /// \param [in] _Id Component Key of the components map
  /// \param [in] _value double Data to be set to the component
  /// valid for types (double, float, int, bool)
  public: void SetData(std::string _Id, const double &_value);

  /// \brief Add a chart to a specefic component attribute
  /// \param[in] _entity entity id in the simulation
  /// \param[in] _typeId type identifier unique to each component type
  /// \param[in] _type Component Datatype ("Pose3d","Vector3d","double")
  /// \param[in] _attribute component attribute to add the chart to it
  /// ex: x attribute in Pose3d Component will be "x"
  /// \param [in] _chart chart ID to be registered
  public slots: void RegisterChartToComponent(uint64_t _entity,
                                              uint64_t _typeId,
                                              std::string _type,
                                              std::string _attribute,
                                              int _chart);

  /// \brief Remove a chart from a specefic component attribute
  /// \param[in] _entity entity id in the simulation
  /// \param[in] _typeId type identifier unique to each component type
  /// \param[in] _attribute component attribute to remove the chart from it
  /// ex: x attribute in Pose3d Component will be "x"
  /// \param[in] _chart chart ID to be unregistered
  public slots: void UnRegisterChartFromComponent(uint64_t _entity,
                                                  uint64_t _typeId,
                                                  std::string _attribute,
                                                  int _chart);

  /// \brief Get Component Name based on its type Id
  /// \param[in] _typeId type Id of the component
  /// \return Component name
  public slots: std::string ComponentName(const uint64_t &_typeId);

  /// \brief dataPtr holds Abstraction data of PlottingPrivate
  private: std::unique_ptr<PlottingPrivate> dataPtr;
};
}
}

#endif
