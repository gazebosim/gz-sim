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
#ifndef GZ_SIM_SYSTEMS_BREADCRUMBS_HH_
#define GZ_SIM_SYSTEMS_BREADCRUMBS_HH_

#include <memory>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <sdf/Element.hh>
#include <sdf/Geometry.hh>
#include <sdf/Root.hh>

#include <gz/transport/Node.hh>
#include <gz/math/Pose3.hh>

#include "gz/sim/Model.hh"
#include "gz/sim/SdfEntityCreator.hh"
#include "gz/sim/System.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  /// \brief A system for creating Breadcrumbs in the form of models that can
  /// get deployed/spawned at the location of the model to which this system is
  /// attached. Each breadcrumb is a complete sdf::Model. When deployed, the
  /// pose of the breadcrumb model is offset from the containing model by the
  /// pose specified in the `<pose>` element of the breadcrumb model. A name is
  /// generated for the breadcrumb by appending the current count of deployments
  /// to the name specified in the breadcrumb `<model>` element. The model
  /// specified in the `<breadcrumb>` parameter serves as a template for
  /// deploying multiple breadcrumbs of the same type. Including models from
  /// Fuel is accomplished by creating a `<model>` that includes the Fuel
  /// model using the `<include>` tag.
  /// See the example in examples/worlds/breadcrumbs.sdf.
  ///
  /// ## System Parameters
  ///
  /// - `<topic>`: Custom topic to be used to deploy breadcrumbs. If topic is
  /// not set, the default topic with the following pattern would be used
  /// `/model/<model_name>/breadcrumbs/<breadcrumb_name>/deploy`. The topic
  /// type is gz.msgs.Empty
  /// - `<max_deployments>`: The maximum number of times this breadcrumb can be
  /// deployed. Once this many are deployed, publishing on the deploy topic
  /// will have no effect. If a negative number is set, the maximum deployment
  /// will be unbounded. If a value of zero is used, then the breadcrumb system
  /// will be disabled. A zero value is useful for situations where SDF files
  /// are programmatically created. The remaining deployment count is available
  /// on the `<topic>/remaining` topic.
  /// - `<disable_physics_time>`: The time in which the breadcrumb entity's
  /// dynamics remain enabled. After his specified time, the breadcrumb will
  /// be made static. If this value is <= 0 or the param is not specified, the
  /// breadcrumb model's dynamics will not be modified.
  /// - `<performer_volume>`: Geometry that represents the bounding volume of
  /// the performer. Only `<geometry><box>` is supported currently. When this
  /// parameter is present, the deployed models will be performers.
  /// - `<allow_renaming>`: If true, the deployed model will be renamed if
  /// another model with the same name already exists in the world. If false
  /// and there is another model with the same name, the breadcrumb will not
  /// be deployed. Defaults to false.
  /// - `<breadcrumb>`: This is the model used as a template for deploying
  /// breadcrumbs.
  /// - `<topic_statistics>`: If true, then topic statistics are enabled on
  /// `<topic>` and error messages will be generated when messages are
  /// dropped. Default to false.
  class Breadcrumbs
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor
    public: Breadcrumbs() = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Callback to deployment topic
    private: void OnDeploy(const msgs::Empty &_msg);

    /// \brief Make an entity static
    /// \param[in] _entity Entity to make static
    /// \param[in] _ecm Entity component manager
    /// \return True if operation is successful, false otherwise
    public: bool MakeStatic(Entity _entity, EntityComponentManager &_ecm);

    /// \brief Set to true after initialization with valid parameters
    private: bool initialized{false};

    /// \brief Gazebo communication node.
    private: transport::Node node;

    /// \brief Model interface
    private: Model model{kNullEntity};

    /// \brief World entity
    private: Entity worldEntity{kNullEntity};

    /// \brief Creator interface
    public: std::unique_ptr<SdfEntityCreator> creator{nullptr};

    /// \brief The number of deployments allowed for the model
    private: int maxDeployments{-1};

    /// \brief The current number of deployments
    private: int numDeployments{0};

    /// \brief sdf::Root of the model to be deployed
    private: sdf::Root modelRoot;

    /// \brief Whether the deployed models will be performers
    private: bool isPerformer{false};

    /// \brief Whether the deployed model should be renamed if a model with the
    /// same name already exists
    private: bool allowRenaming{false};

    /// \brief Bounding volume of the performer
    private: std::optional<sdf::Geometry> performerGeometry;

    /// \brief Pending commands
    private: std::vector<bool> pendingCmds;

    /// \brief List of entities that need geometry updates.
    private: std::set<Entity> pendingGeometryUpdate;

    /// \brief Mutex to protect pending commands
    private: std::mutex pendingCmdsMutex;

    /// \brief Time when the entity should be made static after they are spawned
    private: std::chrono::steady_clock::duration disablePhysicsTime =
        std::chrono::steady_clock::duration::zero();

    /// \brief A map of auto static entities and time when they are spawned.
    private: std::unordered_map<Entity, std::chrono::steady_clock::duration>
        autoStaticEntities;

    /// \brief SDF DOM of a static model with empty link
    private: sdf::Model staticModelToSpawn;

    /// \brief Publishes remaining deployments.
    public: transport::Node::Publisher remainingPub;

    /// \brief True when topic statistics are enabled.
    public: bool topicStatistics{false};

    /// \brief Name of the deploy topic.
    public: std::string topic;
  };
  }
}
}
}

#endif
