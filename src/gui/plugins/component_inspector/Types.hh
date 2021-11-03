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

#ifndef IGNITION_GAZEBO_GUI_COMPONENTINSPECTOR_TYPES_HH_
#define IGNITION_GAZEBO_GUI_COMPONENTINSPECTOR_TYPES_HH_

#include <map>

#include <sdf/Noise.hh>

#include <ignition/gazebo/gui/GuiSystem.hh>
#include <ignition/gazebo/Types.hh>

Q_DECLARE_METATYPE(ignition::gazebo::ComponentTypeId)

namespace ignition
{
namespace gazebo
{
  using UpdateCallback = std::function<void(EntityComponentManager &)>;

  /// \param[out] _noise Noise to set
  void setNoise(sdf::Noise &_noise,
      double _mean, double _meanBias, double _stdDev,
      double _stdDevBias, double _dynamicBiasStdDev,
      double _dynamicBiasCorrelationTime);

  /// \brief Generic function to set data.
  /// \param[in] _item Item whose data will be set.
  /// \param[in] _data Data to set.
  template <class DataType>
  void setData(QStandardItem *_item, const DataType &_data)
  {
    // cppcheck-suppress syntaxError
    // cppcheck-suppress unmatchedSuppression
    if constexpr (traits::IsOutStreamable<std::ostream, DataType>::value)
    {
      std::stringstream ss;
      ss << _data;
      setData(_item, ss.str());
    }
    else
    {
      ignwarn << "Attempting to set unsupported data type to item ["
              << _item->text().toStdString() << "]" << std::endl;
    }
  }

  /// \brief Model holding information about components, such as their type
  /// and data.
  class ComponentsModel : public QStandardItemModel
  {
    Q_OBJECT

    /// \brief Constructor
    public: explicit ComponentsModel();

    /// \brief Destructor
    public: ~ComponentsModel() override = default;

    // Documentation inherited
    public: QHash<int, QByteArray> roleNames() const override;

    /// \brief Static version of roleNames
    /// \return A hash connecting a unique identifier to a role name.
    public: static QHash<int, QByteArray> RoleNames();

    /// \brief Add a component type to the inspector.
    /// \param[in] _typeId Type of component to be added.
    /// \return Newly created item.
    public slots: QStandardItem *AddComponentType(
        ignition::gazebo::ComponentTypeId _typeId);

    /// \brief Remove a component type from the inspector.
    /// \param[in] _typeId Type of component to be removed.
    public slots: void RemoveComponentType(
        ignition::gazebo::ComponentTypeId _typeId);

    /// \brief Keep track of items in the tree, according to type ID.
    public: std::map<ComponentTypeId, QStandardItem *> items;
  };

}
}
#endif
