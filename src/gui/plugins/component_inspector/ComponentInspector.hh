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

#ifndef GZ_SIM_GUI_COMPONENTINSPECTOR_HH_
#define GZ_SIM_GUI_COMPONENTINSPECTOR_HH_

#include <map>
#include <memory>
#include <string>

#include <sdf/Material.hh>
#include <sdf/Physics.hh>

#include <gz/math/SphericalCoordinates.hh>
#include <gz/math/Vector3.hh>
#include <gz/transport/Node.hh>

#include <gz/sim/components/Component.hh>
#include <gz/sim/gui/GuiSystem.hh>
#include <gz/sim/Types.hh>

#include "Types.hh"

#include <gz/msgs/light.pb.h>

Q_DECLARE_METATYPE(gz::sim::ComponentTypeId)

namespace gz
{
namespace sim
{
  class ComponentInspectorPrivate;

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
      gzwarn << "Attempting to set unsupported data type to item ["
              << _item->text().toStdString() << "]" << std::endl;
    }
  }

  /// \brief Specialized to set string data.
  /// \param[in] _item Item whose data will be set.
  /// \param[in] _data Data to set.
  template<>
  void setData(QStandardItem *_item, const std::string &_data);

  /// \brief Specialized to set light data.
  /// \param[in] _item Item whose data will be set.
  /// \param[in] _data Data to set.
  template<>
  void setData(QStandardItem *_item, const msgs::Light &_data);

  /// \brief Specialized to set vector data.
  /// \param[in] _item Item whose data will be set.
  /// \param[in] _data Data to set.
  template<>
  void setData(QStandardItem *_item, const math::Vector3d &_data);

  /// \brief Specialized to set Physics data.
  /// \param[in] _item Item whose data will be set.
  /// \param[in] _data Data to set.
  template<>
  void setData(QStandardItem *_item, const sdf::Physics &_data);

  /// \brief Specialized to set Spherical Coordinates data.
  /// \param[in] _item Item whose data will be set.
  /// \param[in] _data Data to set.
  template<>
  void setData(QStandardItem *_item, const math::SphericalCoordinates &_data);

  /// \brief Specialized to set boolean data.
  /// \param[in] _item Item whose data will be set.
  /// \param[in] _data Data to set.
  template<>
  void setData(QStandardItem *_item, const bool &_data);

  /// \brief Specialized to set integer data.
  /// \param[in] _item Item whose data will be set.
  /// \param[in] _data Data to set.
  template<>
  void setData(QStandardItem *_item, const int &_data);

  /// \brief Specialized to set double data.
  /// \param[in] _item Item whose data will be set.
  /// \param[in] _data Data to set.
  template<>
  void setData(QStandardItem *_item, const double &_data);

  /// \brief Specialized to set stream data.
  /// \param[in] _item Item whose data will be set.
  /// \param[in] _data Data to set.
  template<>
  void setData(QStandardItem *_item, const std::ostream &_data);

  /// \brief Specialized to set material data.
  /// \param[in] _item Item whose data will be set.
  /// \param[in] _data Data to set.
  template<>
  void setData(QStandardItem *_item, const sdf::Material &_data);


  /// \brief Set the unit of a given item.
  /// \param[in] _item Item whose unit will be set.
  /// \param[in] _unit Unit to be displayed, such as 'm' for meters.
  void setUnit(QStandardItem *_item, const std::string &_unit);

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
        gz::sim::ComponentTypeId _typeId);

    /// \brief Remove a component type from the inspector.
    /// \param[in] _typeId Type of component to be removed.
    public slots: void RemoveComponentType(
        gz::sim::ComponentTypeId _typeId);

    /// \brief Keep track of items in the tree, according to type ID.
    public: std::map<ComponentTypeId, QStandardItem *> items;
  };

  /// \brief Displays a tree view with all the entities in the world.
  ///
  /// ## Configuration
  /// None
  class ComponentInspector : public sim::GuiSystem
  {
    Q_OBJECT

    /// \brief Entity
    Q_PROPERTY(
      Entity entity
      READ GetEntity
      WRITE SetEntity
      NOTIFY EntityChanged
    )

    /// \brief Type
    Q_PROPERTY(
      QString type
      READ Type
      WRITE SetType
      NOTIFY TypeChanged
    )

    /// \brief Locked
    Q_PROPERTY(
      bool locked
      READ Locked
      WRITE SetLocked
      NOTIFY LockedChanged
    )

    /// \brief Paused
    Q_PROPERTY(
      bool paused
      READ Paused
      WRITE SetPaused
      NOTIFY PausedChanged
    )

    /// \brief Nested Model
    Q_PROPERTY(
      bool nestedModel
      READ NestedModel
      NOTIFY NestedModelChanged
    )

    /// \brief System display name list
    Q_PROPERTY(
      QStringList systemNameList
      READ SystemNameList
      WRITE SetSystemNameList
      NOTIFY SystemNameListChanged
    )

    /// \brief Constructor
    public: ComponentInspector();

    /// \brief Destructor
    public: ~ComponentInspector() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited
    public: void Update(const UpdateInfo &, EntityComponentManager &) override;

    /// \brief Add a callback that's called whenever there are updates from the
    /// ECM to the view, for a given component type.
    /// \param[in] _id The component type id
    /// \param[in] _cb Function that's called when there are updates.
    public: void AddUpdateViewCb(ComponentTypeId _id,
                inspector::UpdateViewCb _cb);

    /// \brief Callback in Qt thread when specular changes.
    /// \param[in] _rSpecular specular red
    /// \param[in] _gSpecular specular green
    /// \param[in] _bSpecular specular blue
    /// \param[in] _aSpecular specular alpha
    /// \param[in] _rDiffuse Diffuse red
    /// \param[in] _gDiffuse Diffuse green
    /// \param[in] _bDiffuse Diffuse blue
    /// \param[in] _aDiffuse Diffuse alpha
    /// \param[in] _attRange Range attenuation
    /// \param[in] _attLinear Linear attenuation
    /// \param[in] _attConstant Constant attenuation
    /// \param[in] _attQuadratic Quadratic attenuation
    /// \param[in] _castShadows Specify if this light should cast shadows
    /// \param[in] _directionX X direction of the light
    /// \param[in] _directionY Y direction of the light
    /// \param[in] _directionZ Z direction of the light
    /// \param[in] _innerAngle Inner angle of the spotlight
    /// \param[in] _outerAngle Outer angle of the spotlight
    /// \param[in] _falloff Falloff of the spotlight
    /// \param[in] _intensity Intensity of the light
    /// \param[in] _type light type
    /// \param[in] _isLightOn is light on
    /// \param[in] _visualizeVisual is visual enabled
    public: Q_INVOKABLE void OnLight(
      double _rSpecular, double _gSpecular, double _bSpecular,
      double _aSpecular, double _rDiffuse, double _gDiffuse,
      double _bDiffuse, double _aDiffuse, double _attRange,
      double _attLinear, double _attConstant, double _attQuadratic,
      bool _castShadows, double _directionX, double _directionY,
      double _directionZ, double _innerAngle, double _outerAngle,
      double _falloff, double _intensity, int _type, bool _isLightOn,
      bool _visualizeVisual);

    /// \brief Callback in Qt thread when physics' properties change.
    /// \param[in] _stepSize step size
    /// \param[in] _realTimeFactor real time factor
    public: Q_INVOKABLE void OnPhysics(double _stepSize,
        double _realTimeFactor);

    // \brief Callback in Qt thread when material color changes for a visual
    /// \param[in] _rAmbient ambient red
    /// \param[in] _gAmbient ambient green
    /// \param[in] _bAmbient ambient blue
    /// \param[in] _aAmbient ambient alpha
    /// \param[in] _rDiffuse diffuse red
    /// \param[in] _gDiffuse diffuse green
    /// \param[in] _bDiffuse diffuse blue
    /// \param[in] _aDiffuse diffuse alpha
    /// \param[in] _rSpecular specular red
    /// \param[in] _gSpecular specular green
    /// \param[in] _bSpecular specular blue
    /// \param[in] _aSpecular specular alpha
    /// \param[in] _rEmissive emissive red
    /// \param[in] _gEmissive emissive green
    /// \param[in] _bEmissive emissive blue
    /// \param[in] _aEmissive emissive alpha
    /// \param[in] _type if type is not empty, opens QColorDialog.
    /// The possible types are ambient, diffuse, specular, or emissive.
    /// \param[in] _currColor used for QColorDialog to show the current color
    /// in the open dialog.
    public: Q_INVOKABLE void OnMaterialColor(
      double _rAmbient, double _gAmbient, double _bAmbient,
      double _aAmbient, double _rDiffuse, double _gDiffuse,
      double _bDiffuse, double _aDiffuse, double _rSpecular,
      double _gSpecular, double _bSpecular, double _aSpecular,
      double _rEmissive, double _gEmissive, double _bEmissive,
      double _aEmissive, QString _type, QColor _currColor);

    /// \brief Callback in Qt thread when spherical coordinates change.
    /// \param[in] _surface Surface model
    /// \param[in] _latitude Latitude in degrees
    /// \param[in] _longitude Longitude in degrees
    /// \param[in] _elevation Elevation in meters
    /// \param[in] _heading Heading in degrees
    public: Q_INVOKABLE void OnSphericalCoordinates(QString _surface,
        double _latitude, double _longitude, double _elevation,
        double _heading);

    /// \brief Get whether the entity is a nested model or not
    /// \return True if the entity is a nested model, false otherwise
    public: Q_INVOKABLE bool NestedModel() const;

    /// \brief Notify that is nested model property has changed
    signals: void NestedModelChanged();

    // Documentation inherited
    protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

    /// \brief Get the entity currently inspected.
    /// \return Entity ID.
    public: Q_INVOKABLE Entity GetEntity() const;

    /// \brief Set the entity currently inspected.
    /// \param[in] _entity Entity ID.
    public: Q_INVOKABLE void SetEntity(const Entity &_entity);

    /// \brief Notify that entity has changed.
    signals: void EntityChanged();

    /// \brief Get the type of entity currently inspected.
    /// \return Type, such as 'world' or 'model'
    public: Q_INVOKABLE QString Type() const;

    /// \brief Set the type of entity currently inspected.
    /// \param[in] _type Type, such as 'world' or 'model'.
    public: Q_INVOKABLE void SetType(const QString &_entity);

    /// \brief Notify that entity type has changed
    signals: void TypeChanged();

    /// \brief Get whether the inspector is currently locked on an entity.
    /// \return True for locked
    public: Q_INVOKABLE bool Locked() const;

    /// \brief Set whether the inspector is currently locked on an entity.
    /// \param[in] _locked True for locked.
    public: Q_INVOKABLE void SetLocked(bool _locked);

    /// \brief Notify that locked has changed.
    signals: void LockedChanged();

    /// \brief Get whether the inspector is currently paused for updates.
    /// \return True for paused.
    public: Q_INVOKABLE bool Paused() const;

    /// \brief Set whether the inspector is currently paused for updates.
    /// \param[in] _paused True for paused.
    public: Q_INVOKABLE void SetPaused(bool _paused);

    /// \brief Notify that paused has changed.
    signals: void PausedChanged();

    /// \brief Name of world entity
    /// \return World name
    public: const std::string &WorldName() const;

    /// \brief Node for communication
    /// \return Transport node
    public: transport::Node &TransportNode();

    /// \brief Query system plugin info.
    public: Q_INVOKABLE void QuerySystems();

    /// \brief Get the system plugin display name list
    /// \return A list of names that are potentially system plugins
    public: Q_INVOKABLE QStringList SystemNameList() const;

    /// \brief Set the system plugin display name list
    /// \param[in] _systempFilenameList A list of system plugin display names
    public: Q_INVOKABLE void SetSystemNameList(
        const QStringList &_systemNameList);

    /// \brief Notify that system plugin display name list has changed
    signals: void SystemNameListChanged();

    /// \brief Callback when a new system is to be added to an entity
    /// \param[in] _name Name of system
    /// \param[in] _filename Filename of system
    /// \param[in] _innerxml Inner XML content of the system
    public: Q_INVOKABLE void OnAddSystem(const QString &_name,
        const QString &_filename, const QString &_innerxml);

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<ComponentInspectorPrivate> dataPtr;
  };
}
}

#endif
