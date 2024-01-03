/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef GZ_SIM_GUI_ENTITYTREE_HH_
#define GZ_SIM_GUI_ENTITYTREE_HH_

#include <map>
#include <memory>
#include <vector>

#include <gz/sim/Entity.hh>
#include <gz/sim/gui/GuiSystem.hh>

namespace gz
{
namespace sim
{
  class EntityTreePrivate;

  /// \brief TODO
  class TreeModel : public QStandardItemModel
  {
    Q_OBJECT

    /// \brief Constructor
    public: explicit TreeModel();

    /// \brief Destructor
    public: ~TreeModel() override = default;

    // Documentation inherited
    public: QHash<int, QByteArray> roleNames() const override;

    /// \brief Add an entity to the tree.
    /// \param[in] _entity Entity to be added
    /// \param[in] _entityName Name of entity to be added
    /// \param[in] _parentEntity Parent entity. By default, kNullEntity, which
    /// means it's a root entity.
    /// \param[in] _type Entity type
    public slots: void AddEntity(Entity _entity,
        const QString &_entityName,
        Entity _parentEntity = kNullEntity,
        const QString &_type = QString());

    /// \brief Remove an entity from the tree.
    /// \param[in] _entity Entity to be removed
    public slots: void RemoveEntity(Entity _entity);

    /// \brief Get the entity type of a tree item at specified index
    /// \param[in] _index Model index
    /// \return Type of entity
    public: Q_INVOKABLE QString EntityType(const QModelIndex &_index) const;

    /// \brief Get the scoped name of a tree item at specified index
    /// \param[in] _index Model index
    /// \return Scoped name of the entity
    public: Q_INVOKABLE QString ScopedName(const QModelIndex &_index) const;

    /// \brief Get the entity ID of a tree item at specified index
    /// \param[in] _index Model index
    /// \return Entity ID
    public: Q_INVOKABLE Entity EntityId(const QModelIndex &_index) const;

    /// \brief Keep track of which item corresponds to which entity.
    private: std::map<Entity, QStandardItem *> entityItems;

    /// \brief Entity information used to queue the pending entities
    struct EntityInfo
    {
      /// \brief Entity ID
      // cppcheck-suppress unmatchedSuppression
      // cppcheck-suppress unusedStructMember
      Entity entity;

      /// \brief Entity name
      QString name;

      /// \brief Parent ID
      // cppcheck-suppress unmatchedSuppression
      // cppcheck-suppress unusedStructMember
      Entity parentEntity;

      /// \brief Entity type
      QString type;
    };

    /// \brief If an entity is added before its parent, we queue it in this
    /// vector until their parent shows up or they are deleted.
    private: std::vector<EntityInfo> pendingEntities;
  };

  /// \brief Displays a tree view with all the entities in the world.
  ///
  /// ## Configuration
  /// None
  class EntityTree : public gz::sim::GuiSystem
  {
    Q_OBJECT

    /// \brief Constructor
    public: EntityTree();

    /// \brief Destructor
    public: ~EntityTree() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited
    public: void Update(const UpdateInfo &, EntityComponentManager &) override;

    /// \brief Callback when an entity has been selected. This should be
    /// called from QML.
    /// \param[in] _entity Entity being selected.
    public: Q_INVOKABLE void OnEntitySelectedFromQml(Entity _entity);

    /// \brief Callback when all entities have been deselected.
    /// This should be called from QML.
    public: Q_INVOKABLE void DeselectAllEntities();

    /// \brief Callback to insert a new entity
    /// \param[in] _type Type of entity to insert
    public: Q_INVOKABLE void OnInsertEntity(const QString &_type);

    /// \brief Callback to insert a new entity
    /// \param[in] _mesh Mesh file to create a model from.
    public: Q_INVOKABLE void OnLoadMesh(const QString &_mesh);

    // Documentation inherited
    protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<EntityTreePrivate> dataPtr;
  };
}
}

#endif
