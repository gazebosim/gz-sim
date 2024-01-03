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

#ifndef GZ_SIM_GUI_COMPONENTINSPECTOREDITOR_HH_
#define GZ_SIM_GUI_COMPONENTINSPECTOREDITOR_HH_

#include <map>
#include <memory>
#include <string>

#include <sdf/Material.hh>
#include <sdf/Physics.hh>
#include <sdf/Joint.hh>

#include <gz/math/SphericalCoordinates.hh>
#include <gz/math/Vector3.hh>

#include <gz/sim/components/Component.hh>
#include <gz/sim/gui/GuiSystem.hh>
#include <gz/sim/Types.hh>

#include <gz/msgs/light.pb.h>

#include "Types.hh"
Q_DECLARE_METATYPE(gz::sim::ComponentTypeId)

namespace gz
{
namespace sim
{
  class ComponentInspectorEditorPrivate;

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
  class ComponentInspectorEditor : public sim::GuiSystem
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

    /// \brief Type
    Q_PROPERTY(
      QStringList modelParentLinks
      READ ModelParentLinks
      NOTIFY ModelLinksChanged
    )

    /// \brief Type
    Q_PROPERTY(
      QStringList modelChildLinks
      READ ModelChildLinks
      NOTIFY ModelLinksChanged
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

    /// \brief Simulation paused
    Q_PROPERTY(
      bool simPaused
      READ SimPaused
      NOTIFY SimPausedChanged
    )

    /// \brief Nested Model
    Q_PROPERTY(
      bool nestedModel
      READ NestedModel
      NOTIFY NestedModelChanged
    )

    /// \brief Constructor
    public: ComponentInspectorEditor();

    /// \brief Destructor
    public: ~ComponentInspectorEditor() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited
    public: void Update(const UpdateInfo &, EntityComponentManager &) override;

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
    public: Q_INVOKABLE void OnLight(
      double _rSpecular, double _gSpecular, double _bSpecular,
      double _aSpecular, double _rDiffuse, double _gDiffuse,
      double _bDiffuse, double _aDiffuse, double _attRange,
      double _attLinear, double _attConstant, double _attQuadratic,
      bool _castShadows, double _directionX, double _directionY,
      double _directionZ, double _innerAngle, double _outerAngle,
      double _falloff, double _intensity, int _type);

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
    public: Q_INVOKABLE void SetEntity(const sim::Entity &_entity);

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

    /// \brief Get whether simulation is currently paused.
    /// \return True for paused.
    public: Q_INVOKABLE bool SimPaused() const;

    /// \brief Notify that simulation paused state has changed.
    signals: void SimPausedChanged();

    /// \brief Set whether simulation is currently paused.
    /// \param[in] _paused True for paused.
    public: void SetSimPaused(bool _paused);

    /// \brief Get whether the inspector is currently paused for updates.
    /// \return True for paused.
    public: Q_INVOKABLE bool Paused() const;

    /// \brief Set whether the inspector is currently paused for updates.
    /// \param[in] _paused True for paused.
    public: Q_INVOKABLE void SetPaused(bool _paused);

    /// \brief Notify that paused has changed.
    signals: void PausedChanged();

    /// \brief Callback in Qt thread when an entity is to be added
    /// \param[in] _entity Entity to add, e.g. box, sphere, cylinder, etc
    /// \param[in] _type Entity type, e.g. link, visual, collision, etc
    public: Q_INVOKABLE void OnAddEntity(const QString &_entity,
                const QString &_type);

    /// \brief Callback in Qt thread when a joint is to be added
    /// \param[in] _jointType Type of joint to add (revolute, fixed, etc)
    /// \param[in] _parentLink Name of the link to be the parent link
    /// \param[in] _childLink Name of the link to be the child link
    public: Q_INVOKABLE void OnAddJoint(const QString &_jointType,
                const QString &_parentLink,
                const QString &_childLink);

    /// \brief Return the list of availabe links that are suitable for
    /// parent links in joints if a model is selected.
    /// \return List of available links.
    public: Q_INVOKABLE QStringList ModelParentLinks() const;

    /// \brief Return the list of availabe links that are suitable for
    /// child links in joints if a model is selected.
    /// \return List of available links.
    public: Q_INVOKABLE QStringList ModelChildLinks() const;

    /// \brief Set the list of availabe links when a model is selected.
    /// \param[in] _modelLinks List of available links.
    public: Q_INVOKABLE void SetModelLinks(const QStringList &_modelLinks);

    /// \brief Notify that locked has changed.
    signals: void ModelLinksChanged();

    /// \brief Callback to insert a new entity
    /// \param[in] _entity Entity to add, e.g. box, sphere, cylinder, etc
    /// \param[in] _type Entity type, e.g. link, visual, collision, etc
    /// \param[in] _mesh Mesh file to load.
    public: Q_INVOKABLE void OnLoadMesh(const QString &_entity,
                const QString &_type, const QString &_mesh);


    /// \brief Add a callback that will be executed during the next Update.
    /// \param[in] _cb The callback to run.
    public: void AddUpdateCallback(UpdateCallback _cb);

    /// \brief Register a component creator. A component creator is
    /// responsible for selecting the correct QML and setting the
    /// appropriate data for a ComponentTypeId.
    /// \param[in] _id The component type id to associate with the creation
    /// function.
    /// \param[in] _creatorFn Function to call in order to create the QML
    /// component.
    public: void RegisterComponentCreator(ComponentTypeId _id,
                ComponentCreator _creatorFn);

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<ComponentInspectorEditorPrivate> dataPtr;
  };
}
}

#endif
