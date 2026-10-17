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
#ifndef GZ_SIM_SYSTEMS_SIMPLIFIED_PHYSICS_PLANAR_COLLISION_HH_
#define GZ_SIM_SYSTEMS_SIMPLIFIED_PHYSICS_PLANAR_COLLISION_HH_

#include <cstddef>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector2.hh>
#include <gz/utils/ImplPtr.hh>

#include <sdf/Geometry.hh>

#include "gz/sim/Entity.hh"
#include "gz/sim/config.hh"

#include "PlanarMotion.hh"

namespace gz::common
{
  class Mesh;
}

namespace gz::sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
class EntityComponentManager;

namespace systems::simplified_physics
{
  /// \brief Shape of one 2.5D footprint.
  enum class PlanarShapeKind
  {
    /// \brief A convex polygon, counter-clockwise, 3 to 8 vertices.
    Polygon,

    /// \brief A circle. `points` holds the center.
    Circle,

    /// \brief A capsule, which is a segment grown by `radius`. `points`
    /// holds the two end points.
    Capsule,
  };

  /// \brief A collision shape projected onto the ground plane, with the
  /// height interval it covers.
  ///
  /// The footprint is the "2.5D" shape the planar collision code works with.
  /// Two footprints collide if the 2D shapes overlap and the height intervals
  /// overlap.
  struct PlanarFootprint
  {
    /// \brief Kind of shape.
    // cppcheck-suppress unusedStructMember
    PlanarShapeKind kind{PlanarShapeKind::Polygon};

    /// \brief Polygon vertices (counter-clockwise), circle center, or the
    /// two capsule end points, in meters.
    // cppcheck-suppress unusedStructMember
    std::vector<math::Vector2d> points;

    /// \brief Radius of a circle or capsule, in meters. Zero for polygons.
    // cppcheck-suppress unusedStructMember
    double radius{0.0};

    /// \brief Lowest z the shape covers, in meters.
    // cppcheck-suppress unusedStructMember
    double zMin{0.0};

    /// \brief Highest z the shape covers, in meters.
    // cppcheck-suppress unusedStructMember
    double zMax{0.0};

    /// \brief The collision entity the footprint came from, or kNullEntity.
    // cppcheck-suppress unusedStructMember
    Entity collision{kNullEntity};
  };

  /// \brief The footprints of one SDF geometry.
  ///
  /// How each geometry is projected:
  ///
  /// - box: convex hull of the 8 projected corners, a rectangle if the box
  ///   is upright.
  /// - cylinder: circle if the axis is vertical, else the hull of the two
  ///   projected end disks.
  /// - sphere: circle.
  /// - capsule: circle if the axis is vertical, else a 2D capsule.
  /// - ellipsoid: polygon that encloses the projected ellipse (16 tangent
  ///   lines), or a circle if the projection is round.
  /// - cone: the footprint of its bounding cylinder, a superset.
  /// - mesh: convex hull of all vertices. A concave mesh gets filled in.
  ///   `<submesh>` and `<center_submesh>` are ignored, as in the physics
  ///   system: the hull covers the whole mesh file.
  /// - polyline: convex hull of all points of all polylines.
  /// - plane, heightmap, empty: none. They are the floor, not obstacles.
  ///
  /// A convex hull with more than 8 vertices is split into several convex
  /// polygons of at most 8 vertices that cover it exactly. Hulls with fewer
  /// than 3 vertices (a zero-thickness box, for example) become a capsule of
  /// radius 1 mm. Features smaller than 1e-9 m are dropped.
  ///
  /// \param[in] _geometry The geometry.
  /// \param[in] _pose Pose of the geometry in the frame the footprints are
  /// expressed in.
  /// \return Zero or more footprints, with `collision` unset.
  std::vector<PlanarFootprint> FootprintsFromGeometry(
      const sdf::Geometry &_geometry, const math::Pose3d &_pose);

  /// \brief The footprint of a mesh, which is the convex hull of its
  /// vertices (see FootprintsFromGeometry).
  /// \param[in] _mesh The mesh.
  /// \param[in] _scale Scale applied to the vertices.
  /// \param[in] _pose Pose of the mesh in the target frame.
  /// \return One or more footprints, or none for a mesh without vertices.
  std::vector<PlanarFootprint> FootprintsFromMesh(const common::Mesh &_mesh,
      const math::Vector3d &_scale, const math::Pose3d &_pose);

  /// \brief The footprints of all collisions of a model, including the
  /// collisions of nested models.
  ///
  /// A collision is skipped if its `collide_bitmask` is zero. The footprints
  /// come in the order of ascending collision entity id, so the result does
  /// not depend on the order the ECM stores entities in.
  ///
  /// \param[in] _ecm The entity component manager.
  /// \param[in] _model A model entity.
  /// \param[in] _inWorldFrame True to express the footprints in the world
  /// frame, false to express them in the model frame.
  /// \return The footprints, with `collision` set.
  std::vector<PlanarFootprint> ModelFootprints(
      const EntityComponentManager &_ecm, Entity _model, bool _inWorldFrame);

  /// \brief How a body reacts when its motion runs into an obstacle.
  enum class PlanarResponse
  {
    /// \brief The body stops where it first touches the obstacle.
    Stop,

    /// \brief The body keeps the part of the motion along the obstacle. The
    /// part into the obstacle is dropped. A body that is blocked by two
    /// obstacles stops.
    Slide,
  };

  /// \brief Most threads a PlanarCollisionWorld uses.
  constexpr int kMaxPlanarCollisionThreads{64};

  /// \brief Settings of a PlanarCollisionWorld.
  struct PlanarCollisionOptions
  {
    /// \brief Bottom of the height band, in world z. Collisions below it
    /// are floor, not obstacles. Default 1 cm.
    // cppcheck-suppress unusedStructMember
    double bandMin{0.01};

    /// \brief Top of the height band, in world z. Collisions above it are
    /// ceiling, not obstacles. Default 2 m.
    // cppcheck-suppress unusedStructMember
    double bandMax{2.0};

    /// \brief Gap a body keeps to what it stopped at, in meters. A body
    /// that stops at a wall rests this far from it, so float rounding in the
    /// distance query never leaves it inside. Default 0.1 mm.
    // cppcheck-suppress unusedStructMember
    double contactSkin{1e-4};

    /// \brief Pairs closer than this are reported as contacts, in meters.
    /// Default 1 mm.
    // cppcheck-suppress unusedStructMember
    double contactMargin{1e-3};

    /// \brief The most obstacles a Slide motion bends around in one step.
    /// Clamped to 1..8. Default 4.
    // cppcheck-suppress unusedStructMember
    int maxSlideIterations{4};

    /// \brief The fewest passes of the robot-robot resolution. Clamped to
    /// 1..16. Default 4. A group of robots that are linked by pairs gets as
    /// many passes as it has robots (at most 64), because a queue at equal
    /// speed settles one robot per pass.
    // cppcheck-suppress unusedStructMember
    int maxRobotPasses{4};

    /// \brief Collect contacts in Move. They cost extra, so it is off by
    /// default.
    // cppcheck-suppress unusedStructMember
    bool reportContacts{false};

    /// \brief Threads that Move uses for the per-robot queries, counting the
    /// calling thread. 1 runs everything on the calling thread and creates
    /// no pool. Clamped to 1 to kMaxPlanarCollisionThreads. The result does
    /// not depend on this number, bit for bit. Move uses the threads only
    /// when it moves enough robots to pay for waking them (at least 64). The
    /// pool is created by the first such call. Default 1.
    // cppcheck-suppress unusedStructMember
    int threads{1};
  };

  /// \brief A contact between two collision shapes.
  struct PlanarContact
  {
    /// \brief The model that was moved. If both models were moved, the one
    /// with the lower entity id.
    // cppcheck-suppress unusedStructMember
    Entity modelA{kNullEntity};

    /// \brief The other model.
    // cppcheck-suppress unusedStructMember
    Entity modelB{kNullEntity};

    /// \brief The collision of `modelA`, or kNullEntity if the footprint
    /// was added without one.
    // cppcheck-suppress unusedStructMember
    Entity collisionA{kNullEntity};

    /// \brief The collision of `modelB`, or kNullEntity.
    // cppcheck-suppress unusedStructMember
    Entity collisionB{kNullEntity};

    /// \brief Contact point in the world frame, in meters.
    math::Vector2d point;

    /// \brief Unit normal in the world frame. It points from A to B.
    math::Vector2d normal;

    /// \brief Distance between the shapes at the point, in meters. Negative
    /// if they overlap.
    // cppcheck-suppress unusedStructMember
    double separation{0.0};

    /// \brief Penetration depth in meters: `max(0, -separation)`.
    // cppcheck-suppress unusedStructMember
    double depth{0.0};
  };

  /// \brief A request to move one model.
  struct PlanarMotionRequest
  {
    /// \brief A movable model that was added to the world.
    // cppcheck-suppress unusedStructMember
    Entity model{kNullEntity};

    /// \brief The motion the model would make without obstacles. The model
    /// moves from its current pose, which the world holds. The path is a
    /// straight line in x and y and a uniform turn in yaw. Callers that
    /// integrate an arc should split long steps, the chord and the arc differ
    /// by O(step^2).
    // cppcheck-suppress unusedStructMember
    PlanarDelta desired;

    /// \brief Reaction to obstacles.
    // cppcheck-suppress unusedStructMember
    PlanarResponse response{PlanarResponse::Stop};
  };

  /// \brief The result for one PlanarMotionRequest.
  struct PlanarMotionResult
  {
    /// \brief The motion the model made. The world holds the new pose.
    // cppcheck-suppress unusedStructMember
    PlanarDelta achieved;

    /// \brief True if an obstacle or another robot cut the motion.
    // cppcheck-suppress unusedStructMember
    bool blocked{false};

    /// \brief The model that cut the motion first, or kNullEntity.
    // cppcheck-suppress unusedStructMember
    Entity blockedBy{kNullEntity};

    /// \brief False if the request named a model that is not movable in
    /// this world, named a model twice, or had a motion that is not finite.
    /// The motion is then zero.
    // cppcheck-suppress unusedStructMember
    bool known{true};
  };

  /// \brief Collision queries for planar bodies, built on the collision code
  /// of Box2D v3.
  ///
  /// The class is the collision part of the `planar_kinematic` mode. It holds
  /// footprints (2.5D shapes, see PlanarFootprint) of static scenery and of
  /// movable models, and moves bodies through them. It uses Box2D's dynamic
  /// AABB tree, GJK distance and contact manifolds. It does not create a
  /// Box2D world and does not step dynamics, so it keeps no global state and
  /// every instance is independent.
  ///
  /// \section planar_collision_models Models
  ///
  /// - A static model (scenery) never moves. Each collision becomes a shape
  ///   in the static tree, in the world frame.
  /// - A movable model has a pose held by this class. Its collisions form one
  ///   rigid compound in the model frame. A movable model that is not part of
  ///   a Move call is an obstacle at its current pose. This is how carts,
  ///   pallets and bodies of other physics engines enter. Update their pose
  ///   with SetPose before each Move.
  ///
  /// A model must have at most 65535 shapes. Only x, y, z and yaw of a pose
  /// are used. Roll and pitch are dropped when the footprints are made.
  ///
  /// The world holds the pose of a movable model. The caller must call
  /// SetPose before Move for every model that something else moved: a
  /// teleport, or a body of another physics engine. A static model that is
  /// teleported needs to be added again. Coordinates are stored as floats in
  /// the broadphase, so the supported range is about 100 km from the origin.
  ///
  /// \section planar_collision_motion Motion
  ///
  /// Move() takes one desired planar motion per robot and returns the motion
  /// the robot achieves.
  ///
  /// 1. Every robot is swept against the static scenery and against movable
  ///    models that do not move in this call. The sweep is conservative
  ///    advancement with Box2D's GJK distance: the robot advances by the
  ///    distance to the nearest shape divided by a bound on the closing
  ///    speed, until the gap is `contactSkin`. A robot never ends inside a
  ///    shape it started outside of. While a robot turns and is within the
  ///    skin of a shape, the bound is too loose to advance by, so the rest of
  ///    the segment is certified with the Lipschitz bound of the distance:
  ///    it changes by at most the speed of the fastest point of the robot
  ///    plus that of the shape. Intervals that fail the bound are halved
  ///    until the points move less than 0.4 `contactSkin`. A robot in this
  ///    case stops at least a quarter of the skin from the shape, so it
  ///    cannot pass through a feature of any size. If the split runs out of
  ///    its budget the robot stops early. A `Stop` robot
  ///    ends there. A `Slide` robot drops the part of the remaining motion
  ///    that points into the obstacle and continues, up to
  ///    `maxSlideIterations` times. If a turn runs into an obstacle the turn
  ///    is dropped and the robot keeps sliding. The path of a robot is a
  ///    polyline in time with up to `maxSlideIterations + 2` segments.
  /// 2. Robots that move in the same call are checked against each other in
  ///    Jacobi passes. In a pass, the first contact time of every pair is
  ///    computed from the paths of the previous pass, and every robot that
  ///    approaches its partner (or turns) is cut at the contact time. A cut
  ///    robot stops, even in `Slide` mode, so a robot that touches a slower
  ///    one moves in steps of full speed and zero when it is cut, and its
  ///    wheel joints show it. If the passes do not settle, the robots still
  ///    in conflict stay where they were, and the freeze is repeated until
  ///    no robot that approaches another one is left moving. No robot ends a
  ///    step overlapping another that it started outside of.
  ///
  /// A robot that touches a wall cannot turn in place, and a Slide robot that
  /// drives along a wall turns only by the part of the turn that clears it,
  /// because a rigid body pivoting about its origin sweeps into the wall.
  /// A differential-drive AMR at a wall can look stuck in yaw until it backs
  /// off.
  ///
  /// A robot that rests at the skin distance from an obstacle can move away,
  /// move along it, or turn if the turn does not bring any point closer than
  /// the skin. A model that starts inside an obstacle (spawned there) cannot
  /// move into it further. It can move out.
  ///
  /// \section planar_collision_precision Precision
  ///
  /// Poses are doubles. Box2D works in floats, so each query is done in the
  /// frame of the robot, where the numbers are small. The unit test stops a
  /// robot within 0.2 mm of a wall 5 km from the origin.
  ///
  /// \section planar_collision_determinism Determinism
  ///
  /// The result depends on the shapes that were added, their slot order (it
  /// breaks ties between obstacles hit at the same time) and the poses, and
  /// on nothing else: not on the order of the requests, tree layout, hash
  /// order or the number of threads. The same inputs give bit-identical
  /// outputs in one build on one machine; `std::sin` and `std::cos` come from
  /// the C library, which can differ between platforms. Box2D is built with
  /// `-ffp-contract=off`, and so is this file.
  ///
  /// \section planar_collision_threads Threads
  ///
  /// With `PlanarCollisionOptions::threads` above 1, Move() runs the per-robot
  /// work on a pool that the world owns: the broadphase query, the sweep
  /// against scenery and the sweep of each robot pair. Each of them reads the
  /// trees and the poses and writes only to the slot of its own robot or
  /// pair. The tree updates, the merge of the candidate pairs, the Jacobi
  /// update of the caps and the write-back stay on the calling thread, in the
  /// order of the requests. A pass reads the caps of the previous pass only.
  /// The result is therefore the same, bit for bit, for any number of threads.
  /// Nothing in Move() mutates a tree while the pool runs. Move() itself is
  /// not thread-safe: one thread at a time may call it on a world.
  ///
  /// \section planar_collision_contacts Contacts
  ///
  /// With `reportContacts` set, Move() also lists the contacts of every
  /// moved model at its final pose. A contact is a pair of shapes closer than
  /// `contactMargin`, with up to two points each.
  class PlanarCollisionWorld
  {
    /// \brief Constructor.
    /// \param[in] _options Settings.
    public: explicit PlanarCollisionWorld(
        const PlanarCollisionOptions &_options = PlanarCollisionOptions());

    /// \brief Destructor.
    public: ~PlanarCollisionWorld();

    /// \brief Add the collisions of a model as static scenery.
    ///
    /// Shapes outside the height band are dropped. Adding a model that is
    /// already in the world replaces it.
    ///
    /// \param[in] _ecm The entity component manager.
    /// \param[in] _model A model entity.
    /// \return True if at least one shape was added.
    public: bool AddStaticModel(const EntityComponentManager &_ecm,
        Entity _model);

    /// \brief Add the collisions of a model as a movable body. The pose is
    /// the model's world pose in the ECM.
    ///
    /// Adding a model that is already in the world replaces it.
    ///
    /// \param[in] _ecm The entity component manager.
    /// \param[in] _model A model entity.
    /// \return True if at least one shape was added.
    public: bool AddMovableModel(const EntityComponentManager &_ecm,
        Entity _model);

    /// \brief Add static scenery from footprints in the world frame. For
    /// callers that have no ECM, and for tests.
    /// \param[in] _model Id of the scenery.
    /// \param[in] _footprints Footprints in the world frame. The z interval
    /// is clipped to the height band.
    /// \return True if at least one shape was added.
    public: bool AddStaticFootprints(Entity _model,
        const std::vector<PlanarFootprint> &_footprints);

    /// \brief Add a movable body from footprints in the model frame.
    /// \param[in] _model Id of the body.
    /// \param[in] _footprints Footprints in the model frame.
    /// \param[in] _pose Initial pose.
    /// \return True if at least one shape was added.
    public: bool AddMovableFootprints(Entity _model,
        const std::vector<PlanarFootprint> &_footprints,
        const math::Pose3d &_pose);

    /// \brief Remove a model, static or movable. Unknown ids are ignored.
    /// \param[in] _model Id of the model.
    public: void RemoveModel(Entity _model);

    /// \brief True if the model is in the world.
    /// \param[in] _model Id of the model.
    /// \return True if the model was added and not removed.
    public: bool HasModel(Entity _model) const;

    /// \brief True if the model is in the world and movable.
    /// \param[in] _model Id of the model.
    /// \return True if the model is movable.
    public: bool IsMovable(Entity _model) const;

    /// \brief Number of shapes of a model, after splitting and clipping.
    /// \param[in] _model Id of the model.
    /// \return Number of shapes, or zero for an unknown model.
    public: std::size_t ShapeCount(Entity _model) const;

    /// \brief Number of models, static and movable.
    /// \return The number of models.
    public: std::size_t ModelCount() const;

    /// \brief Set the pose of a movable model, for a teleport or to follow
    /// a body that another system moves. Ignored for other models.
    /// \param[in] _model Id of the model.
    /// \param[in] _pose New pose (x, y, z and yaw are used).
    public: void SetPose(Entity _model, const math::Pose3d &_pose);

    /// \brief The pose the world holds for a movable model. Roll and pitch
    /// are zero.
    /// \param[in] _model Id of the model.
    /// \return The pose, or the identity for an unknown or static model.
    public: math::Pose3d Pose(Entity _model) const;

    /// \brief Move robots through the world. See the class description.
    ///
    /// The world holds the achieved poses afterwards. Each model may appear
    /// once in `_requests`. Models that are movable but absent from
    /// `_requests` stay where they are and act as obstacles.
    ///
    /// \param[in] _requests The desired motions.
    /// \param[out] _results One result per request, in the same order.
    public: void Move(const std::vector<PlanarMotionRequest> &_requests,
        std::vector<PlanarMotionResult> &_results);

    /// \brief The contacts found by the last Move call, if
    /// `reportContacts` is set. They are sorted by model ids, collision
    /// ids, then point order.
    /// \return The contacts.
    public: const std::vector<PlanarContact> &Contacts() const;

    /// \brief Find the contacts of a movable model at its current pose,
    /// against everything else. Independent of Move.
    /// \param[in] _model Id of a movable model.
    /// \param[out] _contacts Receives the contacts, A being `_model`.
    public: void ContactsOf(Entity _model,
        std::vector<PlanarContact> &_contacts) const;

    /// \brief Private data pointer.
    GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}
}
}

#endif
