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
#ifndef GZ_SIM_UTIL_HH_
#define GZ_SIM_UTIL_HH_

#include <gz/msgs/entity.pb.h>

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <gz/common/Mesh.hh>
#include <gz/math/AxisAlignedBox.hh>
#include <gz/math/Pose3.hh>
#include <sdf/Mesh.hh>

#include "gz/sim/components/Environment.hh"
#include "gz/sim/config.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Export.hh"
#include "gz/sim/Types.hh"


namespace gz
{
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE {
    //
    /// \brief Helper function to compute world pose of an entity
    /// \param[in] _entity Entity to get the world pose for
    /// \param[in] _ecm Immutable reference to ECM.
    /// \return World pose of entity
    math::Pose3d GZ_SIM_VISIBLE worldPose(const Entity &_entity,
        const EntityComponentManager &_ecm);

    /// \brief Helper function to compute world velocity of an entity
    /// \param[in] _entity Entity to get the world pose for
    /// \param[in] _ecm Immutable reference to ECM.
    /// \return World pose of entity
    math::Vector3d GZ_SIM_VISIBLE relativeVel(const Entity &_entity,
        const EntityComponentManager &_ecm);

    /// \brief Helper function to generate scoped name for an entity.
    /// \param[in] _entity Entity to get the name for.
    /// \param[in] _ecm Immutable reference to ECM.
    /// \param[in] _delim Delimiter to put between names, defaults to "/".
    /// \param[in] _includePrefix True to include the type prefix before the
    /// entity name
    std::string GZ_SIM_VISIBLE scopedName(const Entity &_entity,
      const EntityComponentManager &_ecm, const std::string &_delim = "/",
      bool _includePrefix = true);

    /// \brief Helper function to get an entity given its scoped name.
    /// The scope may start at any level by default. For example, in this
    /// hierarchy:
    ///
    /// world_name
    ///  model_name
    ///    link_name
    ///
    /// All these names will return the link entity:
    ///
    /// * world_name::model_name::link_name
    /// * model_name::link_name
    /// * link_name
    ///
    /// \param[in] _scopedName Entity's scoped name.
    /// \param[in] _ecm Immutable reference to ECM.
    /// \param[in] _relativeTo Entity that the scoped name is relative to. The
    /// scoped name does not include the name of this entity. If not provided,
    /// the scoped name could be relative to any entity.
    /// \param[in] _delim Delimiter between names, defaults to "::", it can't
    /// be empty.
    /// \return All entities that match the scoped name and relative to
    /// requirements, or an empty set otherwise.
    std::unordered_set<Entity> GZ_SIM_VISIBLE entitiesFromScopedName(
      const std::string &_scopedName, const EntityComponentManager &_ecm,
      Entity _relativeTo = kNullEntity,
      const std::string &_delim = "::");

    /// \brief Generally, each entity will be of some specific high-level type,
    /// such as World, Sensor, Collision, etc, and one type only.
    /// The entity type is usually marked by having some component that
    /// represents that type, such as `gz::sim::components::Visual`.
    ///
    /// This function returns the type ID of the given entity's type, which
    /// can be checked against different types. For example, if the
    /// entity is a model, this will be true:
    ///
    /// `sim::components::Model::typeId == entityTypeId(entity, ecm)`
    ///
    /// In case the entity isn't of any known type, this will return
    /// `gz::sim::kComponentTypeIdInvalid`.
    ///
    /// In case the entity has more than one type, only one of them will be
    /// returned. This is not standard usage.
    ///
    /// \param[in] _entity Entity to get the type for.
    /// \param[in] _ecm Immutable reference to ECM.
    /// \return ID of entity's type-defining components.
    ComponentTypeId GZ_SIM_VISIBLE entityTypeId(const Entity &_entity,
        const EntityComponentManager &_ecm);

    /// \brief Generally, each entity will be of some specific high-level type,
    /// such as "world", "sensor", "collision", etc, and one type only.
    ///
    /// This function returns a lowercase string for each type. For example,
    /// "light", "actor", etc.
    ///
    /// In case the entity isn't of any known type, this will return an empty
    /// string.
    ///
    /// In case the entity has more than one type, only one of them will be
    /// returned. This is not standard usage.
    ///
    /// Note that this is different from  component type names.
    ///
    /// \param[in] _entity Entity to get the type for.
    /// \param[in] _ecm Immutable reference to ECM.
    /// \return ID of entity's type-defining components.
    std::string GZ_SIM_VISIBLE entityTypeStr(const Entity &_entity,
        const EntityComponentManager &_ecm);

    /// \brief Get the world to which the given entity belongs.
    /// \param[in] _entity Entity to get the world for.
    /// \param[in] _ecm Immutable reference to ECM.
    /// \return World entity ID.
    Entity GZ_SIM_VISIBLE worldEntity(const Entity &_entity,
        const EntityComponentManager &_ecm);

    /// \brief Get the first world entity that's found.
    /// \param[in] _ecm Immutable reference to ECM.
    /// \return World entity ID.
    Entity GZ_SIM_VISIBLE worldEntity(
        const EntityComponentManager &_ecm);

    /// \brief Helper function to remove a parent scope from a given name.
    /// This removes the first name found before the delimiter.
    /// \param[in] _name Input name possibly generated by scopedName.
    /// \param[in] _delim Delimiter between names.
    /// \return A new string with the parent scope removed.
    std::string GZ_SIM_VISIBLE removeParentScope(
        const std::string &_name, const std::string &_delim);

    /// \brief Combine a URI and a file path into a full path.
    /// If the URI is already a full path or contains a scheme, it won't be
    /// modified.
    /// If the URI is a relative path, the file path will be prepended.
    /// \param[in] _uri URI, which can have a scheme, or be full or relative
    /// paths.
    /// \param[in] _filePath The path to a file in disk.
    /// \return The full path URI.
    std::string GZ_SIM_VISIBLE asFullPath(const std::string &_uri,
        const std::string &_filePath);

    /// \brief Get resource paths based on latest environment variables.
    /// \return All paths in the GZ_SIM_RESOURCE_PATH variable.
    std::vector<std::string> GZ_SIM_VISIBLE resourcePaths();

    /// \brief Add resource paths based on latest environment variables.
    /// This will update the SDF and Gazebo environment variables, and
    /// optionally add more paths to the list.
    /// \param[in] _paths Optional paths to add.
    void GZ_SIM_VISIBLE addResourcePaths(
        const std::vector<std::string> &_paths = {});

    /// \brief Get the top level model of an entity
    /// \param[in] _entity Input entity
    /// \param[in] _ecm Constant reference to ECM.
    /// \return Entity of top level model. If _entity has no top level model,
    /// kNullEntity is returned.
    gz::sim::Entity GZ_SIM_VISIBLE topLevelModel(
        const Entity &_entity,
        const EntityComponentManager &_ecm);

    /// \brief Helper function to generate a valid transport topic, given
    /// a list of topics ordered by preference. The generated topic will be,
    /// in this order:
    ///
    /// 1. The first topic unchanged, if valid.
    /// 2. A valid version of the first topic, if possible.
    /// 3. The second topic unchanged, if valid.
    /// 4. A valid version of the second topic, if possible.
    /// 5. ...
    /// 6. If no valid topics could be generated, return an empty string.
    ///
    /// \param[in] _topics Topics ordered by preference.
    std::string GZ_SIM_VISIBLE validTopic(
        const std::vector<std::string> &_topics);

    /// \brief Helper function that returns a valid Gazebo Transport topic
    /// consisting of the scoped name for the provided entity.
    ///
    /// For example, if the provided entity has a scoped name of
    /// `my_model::my_link::my_sensor` then the resulting topic name will
    /// be `/model/my_model/link/my_link/sensor/my_sensor`. If _excludeWorld
    /// is false, then the topic name will be prefixed by `/world/WORLD_NAME/`,
    /// where `WORLD_NAME` is the name of the world.
    ///
    /// \param[in] _entity The entity to generate the topic name for.
    /// \param[in] _ecm The entity component manager.
    /// \param[in] _excludeWorld True to exclude the world name from the topic.
    /// \return A Gazebo Transport topic name based on the scoped name of
    /// the provided entity, or empty string if a topic name could not be
    /// generated.
    std::string GZ_SIM_VISIBLE topicFromScopedName(
        const Entity &_entity,
        const EntityComponentManager &_ecm,
        bool _excludeWorld = true);

    /// \brief Convert an SDF world filename string, such as "shapes.sdf", to
    /// full system file path.
    /// The provided SDF filename may be a Fuel URI, relative path, name
    /// of an installed Gazebo world filename, or an absolute path.
    /// \param[in] _sdfFile An SDF world filename such as:
    ///    1. "shapes.sdf" - This is referencing an installed world file.
    ///    2. "../shapes.sdf" - This is referencing a relative world file.
    ///    3. "/home/user/shapes.sdf" - This is reference an absolute world
    ///       file.
    ///    4. "https://fuel.gazebosim.org/1.0/openrobotics/worlds/shapes.sdf"
    /// This is referencing a Fuel URI. This will download the world file.
    /// \param[in] _fuelResourceCache Path to a Fuel resource cache, if
    /// known.
    /// \return Full path to the SDF world file. An empty string is returned
    /// if the file could not be found.
    std::string GZ_SIM_VISIBLE resolveSdfWorldFile(
        const std::string &_sdfFilename,
        const std::string &_fuelResourceCache = "");

    /// \brief Helper function to "enable" a component (i.e. create it if it
    /// doesn't exist) or "disable" a component (i.e. remove it if it exists).
    /// \param[in] _ecm Mutable reference to the ECM
    /// \param[in] _entity Entity whose component is being enabled
    /// \param[in] _enable True to enable (create), false to disable (remove).
    /// Defaults to true.
    /// \param[in] _comp The component to create if neeeded. Defaults to a
    /// default-constructed component.
    /// \return True if a component was created or removed, false if nothing
    /// changed.
    template <class ComponentType>
    bool enableComponent(EntityComponentManager &_ecm,
        Entity _entity,
        bool _enable = true,
        const ComponentType &_comp = ComponentType())
    {
      bool changed{false};

      auto exists = _ecm.Component<ComponentType>(_entity);
      if (_enable && !exists)
      {
        _ecm.CreateComponent(_entity, _comp);
        changed = true;
      }
      else if (!_enable && exists)
      {
        _ecm.RemoveComponent<ComponentType>(_entity);
        changed = true;
      }
      return changed;
    }

    /// \brief Helper function to get an entity from an entity message.
    /// The returned entity is not guaranteed to exist.
    ///
    /// The message is used as follows:
    ///
    ///     if id not null
    ///       use id
    ///     else if name not null and type not null
    ///       use name + type
    ///     else
    ///       error
    ///     end
    ///
    /// \param[in] _ecm Entity component manager
    /// \param[in] _msg Entity message
    /// \return Entity ID, or kNullEntity if a matching entity couldn't be
    /// found.
    Entity GZ_SIM_VISIBLE entityFromMsg(
      const EntityComponentManager &_ecm, const msgs::Entity &_msg);

    /// \brief Get the spherical coordinates for an entity.
    /// \param[in] _entity Entity whose coordinates we want.
    /// \param[in] _ecm Entity component manager
    /// \return The entity's latitude (deg), longitude (deg) and elevation (m).
    /// If the entity doesn't have a pose, or the world's spherical coordinates
    /// haven't been defined, this will return nullopt.
    std::optional<math::Vector3d> GZ_SIM_VISIBLE sphericalCoordinates(
        Entity _entity, const EntityComponentManager &_ecm);

    /// \brief Get grid field coordinates based on a world position in cartesian
    /// coordinate frames.
    /// \param[in] _ecm Entity Component Manager
    /// \param[in] _worldPosition world position
    /// \param[in] _gridField Gridfield you are interested in.
    std::optional<math::Vector3d> GZ_SIM_VISIBLE getGridFieldCoordinates(
      const EntityComponentManager &_ecm,
      const math::Vector3d& _worldPosition,
      const std::shared_ptr<components::EnvironmentalData>& _gridField);

    /// \brief Load a mesh from a Mesh SDF DOM
    /// \param[in] _meshSdf Mesh SDF DOM
    /// \return The loaded mesh or null if the mesh can not be loaded.
    GZ_SIM_VISIBLE const common::Mesh *loadMesh(const sdf::Mesh &_meshSdf);

    /// \brief Optimize input mesh.
    /// \param[in] _meshSdf Mesh SDF DOM with mesh optimization parameters
    /// \param[in] _mesh Input mesh to optimize.
    /// \return The optimized mesh or null if the mesh can not be optimized.
    GZ_SIM_VISIBLE const common::Mesh *optimizeMesh(const sdf::Mesh &_meshSdf,
        const common::Mesh &_mesh);

    /// \brief Transform an axis-aligned bounding box by a pose.
    /// \param[in] _aabb Axis-aligned bounding box to transform.
    /// \param[in] _pose Pose to transform the bounding box by.
    /// \return The axis-aligned bounding box in the pose target frame.
    GZ_SIM_VISIBLE math::AxisAlignedBox transformAxisAlignedBox(
      const math::AxisAlignedBox & _aabb,
      const math::Pose3d & _pose);

    /// \brief Compute the axis-aligned bounding box of a mesh.
    /// \param _sdfMesh Mesh SDF DOM.
    /// \return The AABB of the mesh in its local frame.
    GZ_SIM_VISIBLE std::optional<math::AxisAlignedBox> meshAxisAlignedBox(
      const sdf::Mesh &_sdfMesh);

    /// \brief Environment variable holding resource paths.
    const std::string kResourcePathEnv{"GZ_SIM_RESOURCE_PATH"};

    /// \brief Environment variable used by SDFormat to find URIs inside
    /// `<include>`
    const std::string kSdfPathEnv{"SDF_PATH"};

    /// \brief Environment variable holding server config paths.
    const std::string kServerConfigPathEnv{"GZ_SIM_SERVER_CONFIG_PATH"};

    /// \brief Environment variable holding paths to custom rendering engine
    /// plugins.
    const std::string kRenderPluginPathEnv{"GZ_SIM_RENDER_ENGINE_PATH"};
    }
  }
}
#endif
