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
#include "PlanarCollision.hh"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include <gz/common/MeshManager.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <sdf/Box.hh>
#include <sdf/Capsule.hh>
#include <sdf/Collision.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Ellipsoid.hh>
#include <sdf/Geometry.hh>
#include <sdf/Mesh.hh>
#include <sdf/Polyline.hh>
#include <sdf/Sphere.hh>
#include <sdf/Surface.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/Geometry.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/World.hh"

using namespace gz;
using namespace sim;
using namespace systems::simplified_physics;

namespace
{
/// \brief Gap a body keeps to what it stopped at.
const double kSkin{PlanarCollisionOptions().contactSkin};

PlanarFootprint BoxFp(double _cx, double _cy, double _sx, double _sy,
    double _zMin = 0.05, double _zMax = 0.5, double _yaw = 0.0)
{
  PlanarFootprint fp;
  fp.kind = PlanarShapeKind::Polygon;
  const double c = std::cos(_yaw);
  const double s = std::sin(_yaw);
  for (const auto &[x, y] : std::vector<std::pair<double, double>>{
      {-1, -1}, {1, -1}, {1, 1}, {-1, 1}})
  {
    const double lx = 0.5 * x * _sx;
    const double ly = 0.5 * y * _sy;
    fp.points.emplace_back(_cx + c * lx - s * ly, _cy + s * lx + c * ly);
  }
  fp.zMin = _zMin;
  fp.zMax = _zMax;
  return fp;
}

PlanarFootprint CircleFp(double _cx, double _cy, double _r,
    double _zMin = 0.05, double _zMax = 0.5)
{
  PlanarFootprint fp;
  fp.kind = PlanarShapeKind::Circle;
  fp.points = {math::Vector2d(_cx, _cy)};
  fp.radius = _r;
  fp.zMin = _zMin;
  fp.zMax = _zMax;
  return fp;
}

PlanarMotionRequest Req(Entity _model, double _dx, double _dy, double _dyaw,
    PlanarResponse _response = PlanarResponse::Stop)
{
  PlanarMotionRequest r;
  r.model = _model;
  r.desired.x = _dx;
  r.desired.y = _dy;
  r.desired.yaw = _dyaw;
  r.response = _response;
  return r;
}

/// \brief Area of a polygon.
double Area(const std::vector<math::Vector2d> &_p)
{
  double a = 0.0;
  for (std::size_t i = 0; i < _p.size(); ++i)
  {
    const auto &p = _p[i];
    const auto &q = _p[(i + 1) % _p.size()];
    a += p.X() * q.Y() - q.X() * p.Y();
  }
  return 0.5 * a;
}

/// \brief Total area of the polygons of footprints.
double TotalArea(const std::vector<PlanarFootprint> &_fps)
{
  return std::accumulate(_fps.begin(), _fps.end(), 0.0,
      [](double _sum, const PlanarFootprint &_fp)
      {
        return _sum + Area(_fp.points);
      });
}

/// \brief Smallest and largest coordinates of the points of footprints.
void Bounds(const std::vector<PlanarFootprint> &_fps, double &_x0,
    double &_y0, double &_x1, double &_y1)
{
  _x0 = _y0 = 1e300;
  _x1 = _y1 = -1e300;
  for (const auto &fp : _fps)
  {
    const double r = fp.radius;
    for (const auto &p : fp.points)
    {
      _x0 = std::min(_x0, p.X() - r);
      _x1 = std::max(_x1, p.X() + r);
      _y0 = std::min(_y0, p.Y() - r);
      _y1 = std::max(_y1, p.Y() + r);
    }
  }
}

sdf::Geometry BoxGeometry(const math::Vector3d &_size)
{
  sdf::Box box;
  box.SetSize(_size);
  sdf::Geometry g;
  g.SetType(sdf::GeometryType::BOX);
  g.SetBoxShape(box);
  return g;
}

sdf::Geometry CylinderGeometry(double _r, double _l)
{
  sdf::Cylinder cyl;
  cyl.SetRadius(_r);
  cyl.SetLength(_l);
  sdf::Geometry g;
  g.SetType(sdf::GeometryType::CYLINDER);
  g.SetCylinderShape(cyl);
  return g;
}
}  // namespace

/////////////////////////////////////////////////
TEST(PlanarFootprints, Box)
{
  // A box turned by 30 degrees: the footprint is the turned rectangle.
  const math::Pose3d pose(1.0, 2.0, 0.5, 0, 0, GZ_DTOR(30));
  auto fps = FootprintsFromGeometry(BoxGeometry({2.0, 0.4, 1.0}), pose);
  ASSERT_EQ(1u, fps.size());
  EXPECT_EQ(PlanarShapeKind::Polygon, fps[0].kind);
  ASSERT_EQ(4u, fps[0].points.size());
  EXPECT_NEAR(0.8, Area(fps[0].points), 1e-9);
  EXPECT_NEAR(0.0, fps[0].zMin, 1e-12);
  EXPECT_NEAR(1.0, fps[0].zMax, 1e-12);
  // Counter-clockwise.
  EXPECT_GT(Area(fps[0].points), 0.0);

  // A box on its side: the vertical extent follows the turn.
  fps = FootprintsFromGeometry(BoxGeometry({2.0, 0.4, 1.0}),
      math::Pose3d(0, 0, 1.0, GZ_PI / 2, 0, 0));
  ASSERT_EQ(1u, fps.size());
  EXPECT_NEAR(0.8, fps[0].zMin, 1e-9);
  EXPECT_NEAR(1.2, fps[0].zMax, 1e-9);
  EXPECT_NEAR(1.0 * 2.0, Area(fps[0].points), 1e-9);

  // A tilted box has a hexagon as shadow.
  fps = FootprintsFromGeometry(BoxGeometry({1.0, 1.0, 1.0}),
      math::Pose3d(0, 0, 0, 0, GZ_DTOR(20), GZ_DTOR(10)));
  ASSERT_EQ(1u, fps.size());
  EXPECT_LE(fps[0].points.size(), 8u);
  EXPECT_GE(fps[0].points.size(), 4u);
}

/////////////////////////////////////////////////
TEST(PlanarFootprints, CylinderSphereCapsule)
{
  auto fps = FootprintsFromGeometry(CylinderGeometry(0.3, 1.0),
      math::Pose3d(1, 2, 0.5, 0, 0, 0));
  ASSERT_EQ(1u, fps.size());
  EXPECT_EQ(PlanarShapeKind::Circle, fps[0].kind);
  EXPECT_NEAR(0.3, fps[0].radius, 1e-12);
  EXPECT_NEAR(1.0, fps[0].points[0].X(), 1e-12);
  EXPECT_NEAR(0.0, fps[0].zMin, 1e-12);
  EXPECT_NEAR(1.0, fps[0].zMax, 1e-12);

  // A wheel: the axis is horizontal, the footprint is a rectangle 0.2 long
  // and 0.6 wide, turned with the wheel.
  fps = FootprintsFromGeometry(CylinderGeometry(0.3, 0.2),
      math::Pose3d(0, 0, 0.3, GZ_PI / 2, 0, GZ_DTOR(90)));
  ASSERT_GE(fps.size(), 1u);
  double x0, y0, x1, y1;
  Bounds(fps, x0, y0, x1, y1);
  // The axis points along x after the 90 degree turn about z.
  EXPECT_NEAR(-0.1, x0, 1e-9);
  EXPECT_NEAR(0.1, x1, 1e-9);
  EXPECT_NEAR(-0.3, y0, 1e-9);
  EXPECT_NEAR(0.3, y1, 1e-9);
  const double area = TotalArea(fps);
  EXPECT_NEAR(0.2 * 0.6, area, 1e-9);
  EXPECT_NEAR(0.0, fps[0].zMin, 1e-9);
  EXPECT_NEAR(0.6, fps[0].zMax, 1e-9);

  sdf::Sphere sphere;
  sphere.SetRadius(0.25);
  sdf::Geometry g;
  g.SetType(sdf::GeometryType::SPHERE);
  g.SetSphereShape(sphere);
  fps = FootprintsFromGeometry(g, math::Pose3d(0, 1, 1, 0, 0, 0));
  ASSERT_EQ(1u, fps.size());
  EXPECT_EQ(PlanarShapeKind::Circle, fps[0].kind);
  EXPECT_NEAR(0.25, fps[0].radius, 1e-12);
  EXPECT_NEAR(0.75, fps[0].zMin, 1e-12);
  EXPECT_NEAR(1.25, fps[0].zMax, 1e-12);

  sdf::Capsule capsule;
  capsule.SetRadius(0.1);
  capsule.SetLength(0.6);
  sdf::Geometry cg;
  cg.SetType(sdf::GeometryType::CAPSULE);
  cg.SetCapsuleShape(capsule);
  fps = FootprintsFromGeometry(cg, math::Pose3d(0, 0, 1, 0, 0, 0));
  ASSERT_EQ(1u, fps.size());
  EXPECT_EQ(PlanarShapeKind::Circle, fps[0].kind);
  EXPECT_NEAR(0.1, fps[0].radius, 1e-12);
  EXPECT_NEAR(0.6, fps[0].zMin, 1e-12);
  EXPECT_NEAR(1.4, fps[0].zMax, 1e-12);

  // Lying on its side along x: a 2D capsule.
  fps = FootprintsFromGeometry(cg, math::Pose3d(0, 0, 1, 0, GZ_PI / 2, 0));
  ASSERT_EQ(1u, fps.size());
  EXPECT_EQ(PlanarShapeKind::Capsule, fps[0].kind);
  EXPECT_NEAR(0.1, fps[0].radius, 1e-12);
  ASSERT_EQ(2u, fps[0].points.size());
  EXPECT_NEAR(0.6, fps[0].points[0].Distance(fps[0].points[1]), 1e-9);
  EXPECT_NEAR(0.9, fps[0].zMin, 1e-9);
  EXPECT_NEAR(1.1, fps[0].zMax, 1e-9);
}

/////////////////////////////////////////////////
TEST(PlanarFootprints, EllipsoidAndNonObstacles)
{
  sdf::Ellipsoid e;
  e.SetRadii({0.5, 0.25, 0.4});
  sdf::Geometry g;
  g.SetType(sdf::GeometryType::ELLIPSOID);
  g.SetEllipsoidShape(e);
  auto fps = FootprintsFromGeometry(g, math::Pose3d(0, 0, 1, 0, 0, 0));
  ASSERT_GE(fps.size(), 1u);
  double x0, y0, x1, y1;
  Bounds(fps, x0, y0, x1, y1);
  EXPECT_NEAR(-0.5, x0, 1e-9);
  EXPECT_NEAR(0.5, x1, 1e-9);
  EXPECT_NEAR(-0.25, y0, 1e-9);
  EXPECT_NEAR(0.25, y1, 1e-9);
  EXPECT_NEAR(0.6, fps[0].zMin, 1e-9);
  EXPECT_NEAR(1.4, fps[0].zMax, 1e-9);
  // The polygon encloses the ellipse and is not much bigger.
  const double area = TotalArea(fps);
  EXPECT_GE(area, GZ_PI * 0.5 * 0.25);
  EXPECT_LE(area, 1.05 * GZ_PI * 0.5 * 0.25);

  // A sphere-shaped ellipsoid is a circle.
  e.SetRadii({0.3, 0.3, 0.3});
  g.SetEllipsoidShape(e);
  fps = FootprintsFromGeometry(g, math::Pose3d::Zero);
  ASSERT_EQ(1u, fps.size());
  EXPECT_EQ(PlanarShapeKind::Circle, fps[0].kind);
  EXPECT_NEAR(0.3, fps[0].radius, 1e-9);

  // Plane, heightmap and empty are the floor.
  for (auto type : {sdf::GeometryType::PLANE, sdf::GeometryType::HEIGHTMAP,
      sdf::GeometryType::EMPTY})
  {
    sdf::Geometry floor;
    floor.SetType(type);
    EXPECT_TRUE(FootprintsFromGeometry(floor, math::Pose3d::Zero).empty());
  }
}

/////////////////////////////////////////////////
TEST(PlanarFootprints, PolylineAndMesh)
{
  sdf::Polyline polyline;
  polyline.SetHeight(2.0);
  for (const auto &p : std::vector<math::Vector2d>{
      {0, 0}, {2, 0}, {2, 1}, {1, 1}, {1, 2}, {0, 2}})
  {
    polyline.AddPoint(p);
  }
  sdf::Geometry g;
  g.SetType(sdf::GeometryType::POLYLINE);
  g.SetPolylineShape({polyline});
  auto fps = FootprintsFromGeometry(g, math::Pose3d(0, 0, 0.1, 0, 0, 0));
  ASSERT_EQ(1u, fps.size());
  // The L shape is filled in: convex hull of 6 points is a pentagon.
  EXPECT_EQ(5u, fps[0].points.size());
  EXPECT_NEAR(3.5, Area(fps[0].points), 1e-9);
  EXPECT_NEAR(0.1, fps[0].zMin, 1e-12);
  EXPECT_NEAR(2.1, fps[0].zMax, 1e-12);

  // A mesh: hull of the vertices, scaled and moved.
  auto *manager = common::MeshManager::Instance();
  manager->CreateBox("planar_collision_box", math::Vector3d(1, 2, 3),
      math::Vector2d(1, 1));
  const common::Mesh *mesh = manager->MeshByName("planar_collision_box");
  ASSERT_NE(nullptr, mesh);
  fps = FootprintsFromMesh(*mesh, math::Vector3d(2, 1, 1),
      math::Pose3d(10, 0, 0, 0, 0, 0));
  ASSERT_EQ(1u, fps.size());
  EXPECT_EQ(4u, fps[0].points.size());
  EXPECT_NEAR(2.0 * 2.0, Area(fps[0].points), 1e-9);
  double x0, y0, x1, y1;
  Bounds(fps, x0, y0, x1, y1);
  EXPECT_NEAR(9.0, x0, 1e-9);
  EXPECT_NEAR(11.0, x1, 1e-9);
  EXPECT_NEAR(-1.5, fps[0].zMin, 1e-9);
  EXPECT_NEAR(1.5, fps[0].zMax, 1e-9);
}

/////////////////////////////////////////////////
TEST(PlanarFootprints, LargeHullIsSplitExactly)
{
  // A cylinder at 30 degrees from the vertical has a 32-corner shadow.
  auto fps = FootprintsFromGeometry(CylinderGeometry(0.3, 1.0),
      math::Pose3d(0, 0, 1.0, GZ_DTOR(30), 0, 0));
  ASSERT_GT(fps.size(), 1u);
  double area = 0.0;
  for (const auto &fp : fps)
  {
    EXPECT_EQ(PlanarShapeKind::Polygon, fp.kind);
    EXPECT_GE(fp.points.size(), 3u);
    EXPECT_LE(fp.points.size(), 8u);
    EXPECT_GT(Area(fp.points), 0.0);
    // Convex: every turn is to the left.
    const auto &p = fp.points;
    for (std::size_t i = 0; i < p.size(); ++i)
    {
      const auto &a = p[i];
      const auto &b = p[(i + 1) % p.size()];
      const auto &c = p[(i + 2) % p.size()];
      EXPECT_GT((b.X() - a.X()) * (c.Y() - b.Y()) -
          (b.Y() - a.Y()) * (c.X() - b.X()), 0.0);
    }
    area += Area(fp.points);
  }
  // The shadow of a cylinder tilted by a: a rectangle 2r wide and L sin(a)
  // long, plus an elliptic cap at each end (ellipse axes r, r cos(a)). The
  // polygon encloses it and is within a few percent.
  const double a = GZ_DTOR(30);
  const double exact = 2 * 0.3 * 1.0 * std::sin(a) +
      GZ_PI * 0.3 * 0.3 * std::cos(a);
  EXPECT_GE(area, exact);
  EXPECT_LE(area, 1.03 * exact);
}

/////////////////////////////////////////////////
TEST(PlanarFootprints, ThinBoxIsKept)
{
  // A box of zero thickness has a segment as shadow.
  auto fps = FootprintsFromGeometry(BoxGeometry({4.0, 0.0, 2.0}),
      math::Pose3d(0, 0, 1, 0, 0, 0));
  ASSERT_EQ(1u, fps.size());
  EXPECT_EQ(PlanarShapeKind::Capsule, fps[0].kind);
  EXPECT_NEAR(4.0, fps[0].points[0].Distance(fps[0].points[1]), 1e-9);
}

/////////////////////////////////////////////////
/// \brief A world with a model that has one link and the given collisions.
class EcmFixture : public ::testing::Test
{
  public: void SetUp() override
  {
    this->worldEntity = this->ecm.CreateEntity();
    this->ecm.CreateComponent(this->worldEntity, components::World());
  }

  /// \brief Create a model with one link (at the model origin) and one
  /// collision with a geometry.
  public: Entity AddModel(const std::string &_name, const math::Pose3d &_pose,
      const sdf::Geometry &_geom, const math::Pose3d &_collisionPose,
      Entity _parent = kNullEntity)
  {
    const Entity model = this->ecm.CreateEntity();
    this->ecm.CreateComponent(model, components::Model());
    this->ecm.CreateComponent(model, components::Name(_name));
    this->ecm.CreateComponent(model, components::Pose(_pose));
    this->ecm.SetParentEntity(model, _parent == kNullEntity ? this->worldEntity
        : _parent);
    this->AddLinkWithCollision(model, _geom, _collisionPose);
    return model;
  }

  public: Entity AddLinkWithCollision(Entity _model, const sdf::Geometry &_geom,
      const math::Pose3d &_collisionPose)
  {
    const Entity link = this->ecm.CreateEntity();
    this->ecm.CreateComponent(link, components::Link());
    this->ecm.CreateComponent(link, components::Name("link"));
    this->ecm.CreateComponent(link, components::Pose(math::Pose3d::Zero));
    this->ecm.SetParentEntity(link, _model);
    const Entity col = this->ecm.CreateEntity();
    this->ecm.CreateComponent(col, components::Collision());
    this->ecm.CreateComponent(col, components::Name("collision"));
    this->ecm.CreateComponent(col, components::Pose(_collisionPose));
    this->ecm.CreateComponent(col, components::Geometry(_geom));
    this->ecm.SetParentEntity(col, link);
    return col;
  }

  public: EntityComponentManager ecm;
  public: Entity worldEntity{kNullEntity};
};

/////////////////////////////////////////////////
TEST_F(EcmFixture, ModelFootprintsFrames)
{
  const Entity model = this->AddModel("m", math::Pose3d(5, 0, 0, 0, 0,
      GZ_PI / 2), BoxGeometry({1, 1, 1}), math::Pose3d(1, 0, 0.5, 0, 0, 0));
  // A nested model inside it.
  const Entity nested = this->AddModel("n", math::Pose3d(0, 2, 0, 0, 0, 0),
      CylinderGeometry(0.2, 1.0), math::Pose3d(0, 0, 0.5, 0, 0, 0), model);

  const auto inWorld = ModelFootprints(this->ecm, model, true);
  ASSERT_EQ(2u, inWorld.size());
  // The box center is 1 m ahead of the model, which looks along +y.
  EXPECT_EQ(PlanarShapeKind::Polygon, inWorld[0].kind);
  double x0, y0, x1, y1;
  Bounds({inWorld[0]}, x0, y0, x1, y1);
  EXPECT_NEAR(4.5, x0, 1e-9);
  EXPECT_NEAR(5.5, x1, 1e-9);
  EXPECT_NEAR(0.5, y0, 1e-9);
  EXPECT_NEAR(1.5, y1, 1e-9);
  // The nested model sits at (0, 2) in the frame of its parent: world (3, 0).
  EXPECT_EQ(PlanarShapeKind::Circle, inWorld[1].kind);
  EXPECT_NEAR(3.0, inWorld[1].points[0].X(), 1e-9);
  EXPECT_NEAR(0.0, inWorld[1].points[0].Y(), 1e-9);
  EXPECT_NE(kNullEntity, inWorld[1].collision);
  EXPECT_NE(inWorld[0].collision, inWorld[1].collision);
  static_cast<void>(nested);

  const auto local = ModelFootprints(this->ecm, model, false);
  ASSERT_EQ(2u, local.size());
  Bounds({local[0]}, x0, y0, x1, y1);
  EXPECT_NEAR(0.5, x0, 1e-9);
  EXPECT_NEAR(1.5, x1, 1e-9);
  EXPECT_NEAR(-0.5, y0, 1e-9);
  EXPECT_NEAR(0.5, y1, 1e-9);
}

/////////////////////////////////////////////////
TEST_F(EcmFixture, CollideBitmaskZeroIsSkipped)
{
  const Entity model = this->AddModel("m", math::Pose3d::Zero,
      BoxGeometry({1, 1, 1}), math::Pose3d(0, 0, 0.5, 0, 0, 0));
  EXPECT_EQ(1u, ModelFootprints(this->ecm, model, true).size());

  // Give the collision an element that switches contact off.
  const Entity col = this->AddLinkWithCollision(model,
      BoxGeometry({1, 1, 1}), math::Pose3d(3, 0, 0.5, 0, 0, 0));
  sdf::Collision element;
  sdf::Surface surface;
  sdf::Contact contact;
  contact.SetCollideBitmask(0);
  surface.SetContact(contact);
  element.SetSurface(surface);
  this->ecm.CreateComponent(col, components::CollisionElement(element));
  EXPECT_EQ(1u, ModelFootprints(this->ecm, model, true).size());
}

/////////////////////////////////////////////////
TEST_F(EcmFixture, WorldFromEcm)
{
  // Wall: a static box. Robot: a movable box that looks along +x.
  const Entity wall = this->AddModel("wall", math::Pose3d(2.0, 0, 0, 0, 0, 0),
      BoxGeometry({0.2, 4.0, 1.0}), math::Pose3d(0, 0, 0.5, 0, 0, 0));
  const Entity robot = this->AddModel("robot", math::Pose3d(0, 0, 0, 0, 0, 0),
      BoxGeometry({0.6, 0.4, 0.3}), math::Pose3d(0, 0, 0.15, 0, 0, 0));

  PlanarCollisionWorld pworld;
  EXPECT_TRUE(pworld.AddStaticModel(this->ecm, wall));
  EXPECT_TRUE(pworld.AddMovableModel(this->ecm, robot));
  EXPECT_EQ(2u, pworld.ModelCount());
  EXPECT_TRUE(pworld.IsMovable(robot));
  EXPECT_FALSE(pworld.IsMovable(wall));

  std::vector<PlanarMotionResult> results;
  pworld.Move({Req(robot, 3.0, 0.0, 0.0)}, results);
  ASSERT_EQ(1u, results.size());
  // The wall face is at 1.9 and the robot front is 0.3 ahead of its origin.
  EXPECT_NEAR(1.6, results[0].achieved.x, 1e-3);
  EXPECT_TRUE(results[0].blocked);
  EXPECT_EQ(wall, results[0].blockedBy);
  EXPECT_NEAR(1.6, pworld.Pose(robot).Pos().X(), 1e-3);

  pworld.RemoveModel(wall);
  EXPECT_FALSE(pworld.HasModel(wall));
  pworld.Move({Req(robot, 1.0, 0.0, 0.0)}, results);
  EXPECT_NEAR(1.0, results[0].achieved.x, 1e-12);
  EXPECT_FALSE(results[0].blocked);
}

/////////////////////////////////////////////////
/// \brief Worlds with a wall at x = 2 (face at 1.9) and a robot box.
class WallFixture : public ::testing::Test
{
  public: void SetUp() override
  {
    // Wall 0.2 thick, 10 m long.
    this->world.AddStaticFootprints(this->wall,
        {BoxFp(2.0, 0.0, 0.2, 10.0)});
    // Robot 0.6 long (x), 0.4 wide.
    this->world.AddMovableFootprints(this->robot,
        {BoxFp(0, 0, 0.6, 0.4, 0.05, 0.3)}, math::Pose3d::Zero);
  }

  public: PlanarMotionResult Move(const PlanarMotionRequest &_r)
  {
    std::vector<PlanarMotionResult> results;
    this->world.Move({_r}, results);
    return results.at(0);
  }

  public: PlanarCollisionWorld world;
  public: const Entity wall{100};
  public: const Entity robot{200};
};

/////////////////////////////////////////////////
TEST_F(WallFixture, StopAtWall)
{
  // A big step: no tunneling through the 0.2 m wall.
  const auto r = this->Move(Req(this->robot, 5.0, 0.0, 0.0));
  EXPECT_TRUE(r.blocked);
  EXPECT_EQ(this->wall, r.blockedBy);
  EXPECT_TRUE(r.known);
  // The front is at 1.9 - skin.
  EXPECT_NEAR(1.6 - kSkin, r.achieved.x, 1e-3);
  EXPECT_LE(r.achieved.x, 1.6);
  EXPECT_GE(r.achieved.x, 1.6 - 1.5 * kSkin);
  EXPECT_EQ(0.0, r.achieved.y);
  EXPECT_EQ(0.0, r.achieved.yaw);

  // Pushing on keeps it there.
  const auto again = this->Move(Req(this->robot, 1.0, 0.0, 0.0));
  EXPECT_TRUE(again.blocked);
  EXPECT_EQ(0.0, again.achieved.x);

  // Driving away works.
  const auto away = this->Move(Req(this->robot, -0.5, 0.0, 0.0));
  EXPECT_FALSE(away.blocked);
  EXPECT_NEAR(-0.5, away.achieved.x, 1e-12);
}

/////////////////////////////////////////////////
TEST_F(WallFixture, StopOnAngledApproach)
{
  const auto r = this->Move(Req(this->robot, 3.0, 1.0, 0.0));
  EXPECT_TRUE(r.blocked);
  // It stops where the front touches: at x 1.6 after 1.6/3 of the motion.
  EXPECT_NEAR(1.6, r.achieved.x, 1e-3);
  EXPECT_NEAR(1.6 / 3.0, r.achieved.y, 1e-3);
}

/////////////////////////////////////////////////
TEST_F(WallFixture, SlideAlongWall)
{
  const auto r = this->Move(
      Req(this->robot, 3.0, 1.0, 0.0, PlanarResponse::Slide));
  EXPECT_TRUE(r.blocked);
  EXPECT_NEAR(1.6, r.achieved.x, 1e-3);
  // The part along the wall is kept: the robot covers the 1 m in y.
  // It only loses the time spent before the wall: it moved at 1/3 m/s in y
  // for the whole step because the slide keeps the tangential velocity.
  EXPECT_NEAR(1.0, r.achieved.y, 1e-3);
  EXPECT_EQ(0.0, r.achieved.yaw);

  // Sliding along an already touched wall.
  const auto along = this->Move(
      Req(this->robot, 1.0, -0.5, 0.0, PlanarResponse::Slide));
  EXPECT_TRUE(along.blocked);
  EXPECT_NEAR(0.0, along.achieved.x, 1e-3);
  EXPECT_NEAR(-0.5, along.achieved.y, 1e-3);
}

/////////////////////////////////////////////////
TEST_F(WallFixture, SlideIntoCorner)
{
  // A second wall across the way: face at y = 1.0 above the robot.
  this->world.AddStaticFootprints(300,
      {BoxFp(0.0, 1.1, 10.0, 0.2)});
  const auto r = this->Move(
      Req(this->robot, 3.0, 3.0, 0.0, PlanarResponse::Slide));
  EXPECT_TRUE(r.blocked);
  // The corner holds it: x 1.6, y 1.0 - 0.2 (half width).
  EXPECT_NEAR(1.6, r.achieved.x, 2e-3);
  EXPECT_NEAR(0.8, r.achieved.y, 2e-3);
  // It does not end inside either wall.
  std::vector<PlanarContact> contacts;
  this->world.ContactsOf(this->robot, contacts);
  for (const auto &c : contacts)
    EXPECT_GE(c.separation, 0.0);
}

/////////////////////////////////////////////////
TEST_F(WallFixture, RotationIntoWall)
{
  // The reach of the turned robot along x is 0.3 cos|yaw| + 0.2 sin|yaw|,
  // at most 0.36. With the origin at 1.4 the wall face (1.9) is 0.5 away:
  // the turn is free.
  this->world.SetPose(this->robot, math::Pose3d(1.4, 0, 0, 0, 0, 0));
  auto r = this->Move(Req(this->robot, 0.0, 0.0, 1.0));
  EXPECT_FALSE(r.blocked);
  EXPECT_NEAR(1.0, r.achieved.yaw, 1e-12);

  // Closer: the origin at 1.55 is 0.35 from the face. The turn stops where
  // 0.3 cos + 0.2 sin reaches 0.35 - skin.
  this->world.SetPose(this->robot, math::Pose3d(1.55, 0, 0, 0, 0, 0));
  const double limit = 0.35 - kSkin;
  double lo = 0.0, hi = 1.0;
  for (int i = 0; i < 60; ++i)
  {
    const double mid = 0.5 * (lo + hi);
    if (0.3 * std::cos(mid) + 0.2 * std::sin(mid) < limit)
      lo = mid;
    else
      hi = mid;
  }
  r = this->Move(Req(this->robot, 0.0, 0.0, 1.0));
  EXPECT_TRUE(r.blocked);
  EXPECT_NEAR(lo, r.achieved.yaw, 2e-3);
  EXPECT_EQ(0.0, r.achieved.x);

  // Turning the other way is blocked the same way, symmetric.
  this->world.SetPose(this->robot, math::Pose3d(1.55, 0, 0, 0, 0, 0));
  r = this->Move(Req(this->robot, 0.0, 0.0, -1.0));
  EXPECT_TRUE(r.blocked);
  EXPECT_NEAR(-lo, r.achieved.yaw, 2e-3);

  // With Slide, a turn that cannot go on is dropped, the translation along
  // the wall continues.
  this->world.SetPose(this->robot, math::Pose3d(1.55, 0, 0, 0, 0, 0));
  r = this->Move(Req(this->robot, 0.0, 0.5, 1.0, PlanarResponse::Slide));
  EXPECT_TRUE(r.blocked);
  EXPECT_LT(r.achieved.yaw, 1.0);
  EXPECT_NEAR(0.5, r.achieved.y, 2e-3);
}

/////////////////////////////////////////////////
TEST_F(WallFixture, RoundRobotWithOffsetOriginTurnsAwayFromWall)
{
  // A round body of radius 0.2 whose center is 0.1 ahead of the origin.
  const Entity round = 201;
  this->world.AddMovableFootprints(round, {CircleFp(0.1, 0.0, 0.2)},
      math::Pose3d(0.0, 3.0, 0, 0, 0, 0));
  // Drive into the wall at 1.9: the front is 0.3 from the origin.
  std::vector<PlanarMotionResult> results;
  this->world.Move({Req(round, 5.0, 0.0, 0.0)}, results);
  EXPECT_TRUE(results[0].blocked);
  EXPECT_NEAR(1.6, results[0].achieved.x, 1e-3);
  // Resting within the skin of the wall, turn through a whole circle. The
  // center orbits the origin and moves away from the wall, then comes back
  // to the same place at the end: the last part would touch, so go half way.
  this->world.Move({Req(round, 0.0, 0.0, GZ_PI)}, results);
  EXPECT_FALSE(results[0].blocked);
  EXPECT_NEAR(GZ_PI, results[0].achieved.yaw, 1e-12);
  // And back: the front returns to the wall and the turn stops there.
  this->world.Move({Req(round, 0.0, 0.0, -GZ_PI)}, results);
  EXPECT_NEAR(-GZ_PI, results[0].achieved.yaw, 2e-3);
}

/////////////////////////////////////////////////
TEST_F(WallFixture, RoundRobotTurnsInPlaceAtWall)
{
  // A circle around its origin does not change shape when it turns.
  const Entity round = 201;
  this->world.AddMovableFootprints(round, {CircleFp(0.0, 0.0, 0.3)},
      math::Pose3d(0.0, 3.0, 0, 0, 0, 0));
  std::vector<PlanarMotionResult> results;
  this->world.Move({Req(round, 5.0, 0.0, 0.0)}, results);
  EXPECT_NEAR(1.6, results[0].achieved.x, 1e-3);
  this->world.Move({Req(round, 0.0, 0.0, 5.0)}, results);
  EXPECT_FALSE(results[0].blocked);
  EXPECT_NEAR(5.0, results[0].achieved.yaw, 1e-12);
}

/////////////////////////////////////////////////
TEST_F(WallFixture, FlushRectangleCannotTurn)
{
  const auto stop = this->Move(Req(this->robot, 5.0, 0.0, 0.0));
  EXPECT_NEAR(1.6, stop.achieved.x, 1e-3);
  const auto r = this->Move(Req(this->robot, 0.0, 0.0, 0.5));
  EXPECT_TRUE(r.blocked);
  EXPECT_NEAR(0.0, r.achieved.yaw, 1e-3);
}

/////////////////////////////////////////////////
TEST_F(WallFixture, StartInsideCanLeaveOnly)
{
  // Spawn the robot 1 cm into the wall.
  this->world.SetPose(this->robot, math::Pose3d(1.61, 0, 0, 0, 0, 0));
  const auto in = this->Move(Req(this->robot, 0.1, 0.0, 0.0));
  EXPECT_TRUE(in.blocked);
  EXPECT_EQ(0.0, in.achieved.x);
  const auto out = this->Move(Req(this->robot, -0.5, 0.0, 0.0));
  EXPECT_FALSE(out.blocked);
  EXPECT_NEAR(-0.5, out.achieved.x, 1e-12);

  // The depth is reported.
  this->world.SetPose(this->robot, math::Pose3d(1.61, 0, 0, 0, 0, 0));
  std::vector<PlanarContact> contacts;
  this->world.ContactsOf(this->robot, contacts);
  ASSERT_FALSE(contacts.empty());
  for (const auto &c : contacts)
  {
    EXPECT_NEAR(0.01, c.depth, 1e-4);
    EXPECT_NEAR(-0.01, c.separation, 1e-4);
    EXPECT_EQ(this->robot, c.modelA);
    EXPECT_EQ(this->wall, c.modelB);
    EXPECT_NEAR(1.0, c.normal.X(), 1e-6);
    EXPECT_NEAR(0.0, c.normal.Y(), 1e-6);
  }
}

/////////////////////////////////////////////////
TEST_F(WallFixture, ContactsAfterMove)
{
  PlanarCollisionOptions options;
  options.reportContacts = true;
  PlanarCollisionWorld fresh(options);
  fresh.AddStaticFootprints(this->wall, {BoxFp(2.0, 0.0, 0.2, 10.0)});
  fresh.AddMovableFootprints(this->robot, {BoxFp(0, 0, 0.6, 0.4, 0.05, 0.3)},
      math::Pose3d::Zero);
  std::vector<PlanarMotionResult> results;
  fresh.Move({Req(this->robot, 5.0, 0.0, 0.0)}, results);
  const auto &contacts = fresh.Contacts();
  ASSERT_EQ(2u, contacts.size());
  for (const auto &c : contacts)
  {
    EXPECT_EQ(this->robot, c.modelA);
    EXPECT_EQ(this->wall, c.modelB);
    EXPECT_NEAR(1.0, c.normal.X(), 1e-6);
    EXPECT_NEAR(0.0, c.normal.Y(), 1e-6);
    EXPECT_NEAR(kSkin, c.separation, 1e-5);
    EXPECT_EQ(0.0, c.depth);
    // The points are the front corners of the robot.
    // Box2D puts the point halfway between the two shapes, on the wall.
    EXPECT_NEAR(1.9, c.point.X(), 2e-3);
    EXPECT_NEAR(0.2, std::abs(c.point.Y()), 1e-3);
  }
  EXPECT_LT(contacts[0].point.Y(), contacts[1].point.Y());

  // Contacts are replaced by the next call.
  fresh.Move({Req(this->robot, -1.0, 0.0, 0.0)}, results);
  EXPECT_TRUE(fresh.Contacts().empty());
}

/////////////////////////////////////////////////
TEST_F(WallFixture, HeightBand)
{
  // A shelf above the band, one below it and a table whose top is above
  // the robot do not block.
  PlanarCollisionWorld fresh;
  fresh.AddStaticFootprints(1, {BoxFp(2.0, 0.0, 0.2, 10.0, 2.5, 3.0)});
  fresh.AddStaticFootprints(2, {BoxFp(2.0, 0.0, 0.2, 10.0, 0.0, 0.005)});
  EXPECT_EQ(0u, fresh.ShapeCount(1));
  EXPECT_EQ(0u, fresh.ShapeCount(2));
  EXPECT_FALSE(fresh.HasModel(1));
  // A deck at 0.5..0.6 over a robot that is 0.3 high.
  fresh.AddStaticFootprints(3, {BoxFp(2.0, 0.0, 0.2, 10.0, 0.5, 0.6)});
  EXPECT_EQ(1u, fresh.ShapeCount(3));
  fresh.AddMovableFootprints(this->robot, {BoxFp(0, 0, 0.6, 0.4, 0.05, 0.3)},
      math::Pose3d::Zero);
  std::vector<PlanarMotionResult> results;
  fresh.Move({Req(this->robot, 4.0, 0.0, 0.0)}, results);
  EXPECT_FALSE(results[0].blocked);
  EXPECT_NEAR(4.0, results[0].achieved.x, 1e-12);

  // Lifted by 0.3 the robot (z 0.35 to 0.6) meets the deck.
  fresh.SetPose(this->robot, math::Pose3d(0, 0, 0.3, 0, 0, 0));
  fresh.Move({Req(this->robot, 4.0, 0.0, 0.0)}, results);
  EXPECT_TRUE(results[0].blocked);
  EXPECT_NEAR(1.6, results[0].achieved.x, 1e-3);
}

/////////////////////////////////////////////////
TEST_F(WallFixture, MovableObstacle)
{
  // A cart that does not move in the call is an obstacle at its pose.
  const Entity cart = 300;
  this->world.AddMovableFootprints(cart, {BoxFp(0, 0, 1.0, 1.0, 0.05, 0.4)},
      math::Pose3d(1.0, 0.0, 0, 0, 0, 0));
  auto r = this->Move(Req(this->robot, 5.0, 0.0, 0.0));
  EXPECT_TRUE(r.blocked);
  EXPECT_EQ(cart, r.blockedBy);
  // Cart back is at 0.5, robot front at 0.5 - skin: x = 0.2.
  EXPECT_NEAR(0.2, r.achieved.x, 1e-3);

  // The cart moves away (another system moved it).
  this->world.SetPose(cart, math::Pose3d(10.0, 0.0, 0, 0, 0, 0));
  r = this->Move(Req(this->robot, 5.0, 0.0, 0.0));
  EXPECT_TRUE(r.blocked);
  EXPECT_EQ(this->wall, r.blockedBy);
  EXPECT_NEAR(1.6, this->world.Pose(this->robot).Pos().X(), 1e-3);
}

/////////////////////////////////////////////////
TEST(PlanarCollision, UnknownAndDuplicateRequests)
{
  PlanarCollisionWorld world;
  world.AddStaticFootprints(1, {BoxFp(5, 0, 1, 1)});
  world.AddMovableFootprints(2, {BoxFp(0, 0, 1, 1)}, math::Pose3d::Zero);
  std::vector<PlanarMotionResult> results;
  world.Move({Req(99, 1, 0, 0), Req(1, 1, 0, 0), Req(2, 1, 0, 0),
      Req(2, 1, 0, 0)}, results);
  ASSERT_EQ(4u, results.size());
  EXPECT_FALSE(results[0].known);
  EXPECT_FALSE(results[1].known);
  EXPECT_TRUE(results[2].known);
  EXPECT_FALSE(results[3].known);
  EXPECT_EQ(1.0, results[2].achieved.x);
  EXPECT_EQ(0.0, results[3].achieved.x);
  EXPECT_EQ(1.0, world.Pose(2).Pos().X());

  // Adding the same id again replaces the model.
  world.AddMovableFootprints(2, {BoxFp(0, 0, 1, 1)},
      math::Pose3d(3, 0, 0, 0, 0, 0));
  EXPECT_EQ(2u, world.ModelCount());
  EXPECT_EQ(3.0, world.Pose(2).Pos().X());
  world.RemoveModel(77);
  EXPECT_EQ(2u, world.ModelCount());
}

/////////////////////////////////////////////////
TEST(PlanarCollision, FarFromTheOrigin)
{
  // Box2D works in floats, which resolve 0.5 mm at 5 km. The sweeps run in
  // the frame of the robot, so the stop distance does not suffer.
  const double ox = 5000.0;
  const double oy = -3000.0;
  PlanarCollisionWorld world;
  world.AddStaticFootprints(1, {BoxFp(ox + 2.0, oy, 0.2, 10.0)});
  world.AddMovableFootprints(2, {BoxFp(0, 0, 0.6, 0.4)},
      math::Pose3d(ox, oy, 0, 0, 0, 0));
  std::vector<PlanarMotionResult> results;
  world.Move({Req(2, 5.0, 0.0, 0.0)}, results);
  EXPECT_TRUE(results[0].blocked);
  EXPECT_GE(results[0].achieved.x, 1.6 - 2.0 * kSkin);
  EXPECT_LE(results[0].achieved.x, 1.6);

  // Turning against the wall stops at the same angle as near the origin.
  world.SetPose(2, math::Pose3d(ox + 1.55, oy, 0, 0, 0, 0));
  world.Move({Req(2, 0.0, 0.0, 1.0)}, results);
  const double farYaw = results[0].achieved.yaw;
  PlanarCollisionWorld nearWorld;
  nearWorld.AddStaticFootprints(1, {BoxFp(2.0, 0.0, 0.2, 10.0)});
  nearWorld.AddMovableFootprints(2, {BoxFp(0, 0, 0.6, 0.4)},
      math::Pose3d(1.55, 0, 0, 0, 0, 0));
  nearWorld.Move({Req(2, 0.0, 0.0, 1.0)}, results);
  EXPECT_NEAR(results[0].achieved.yaw, farYaw, 1e-4);
}

/////////////////////////////////////////////////
TEST(PlanarCollision, CirclesAndCapsules)
{
  PlanarCollisionWorld world;
  // A post (circle) and a capsule-shaped bar.
  world.AddStaticFootprints(1, {CircleFp(3.0, 0.0, 0.25)});
  PlanarFootprint bar;
  bar.kind = PlanarShapeKind::Capsule;
  bar.points = {math::Vector2d(0.0, 3.0), math::Vector2d(4.0, 3.0)};
  bar.radius = 0.1;
  bar.zMin = 0.0 + 0.1;
  bar.zMax = 1.0;
  world.AddStaticFootprints(2, {bar});

  world.AddMovableFootprints(3, {CircleFp(0, 0, 0.3)}, math::Pose3d::Zero);
  std::vector<PlanarMotionResult> results;
  world.Move({Req(3, 5.0, 0.0, 0.0)}, results);
  EXPECT_NEAR(3.0 - 0.25 - 0.3 - kSkin, results[0].achieved.x, 1e-3);
  EXPECT_EQ(1u, world.ModelCount() - 2u);

  // Up into the bar: the bar's lower edge is at y = 2.9.
  world.SetPose(3, math::Pose3d(1.0, 0.0, 0, 0, 0, 0));
  world.Move({Req(3, 0.0, 5.0, 0.0)}, results);
  EXPECT_NEAR(2.9 - 0.3 - kSkin, results[0].achieved.y, 1e-3);

  // A box robot into the post: stops on its face.
  world.AddMovableFootprints(4, {BoxFp(0, 0, 0.6, 0.6)},
      math::Pose3d(0, 0, 0, 0, 0, 0));
  world.Move({Req(4, 5.0, 0.0, 0.0)}, results);
  EXPECT_NEAR(3.0 - 0.25 - 0.3 - kSkin, results[0].achieved.x, 1e-3);
}

/////////////////////////////////////////////////
TEST(PlanarCollision, TwoRobotsHeadOn)
{
  PlanarCollisionWorld world;
  const Entity a = 1, b = 2;
  world.AddMovableFootprints(a, {BoxFp(0, 0, 0.6, 0.4)},
      math::Pose3d(-1.0, 0, 0, 0, 0, 0));
  world.AddMovableFootprints(b, {BoxFp(0, 0, 0.6, 0.4)},
      math::Pose3d(1.0, 0, 0, 0, 0, 0));
  std::vector<PlanarMotionResult> results;
  // Both drive 1 m: they would meet at 0.7 of the step.
  world.Move({Req(a, 1.0, 0.0, 0.0), Req(b, -1.0, 0.0, 0.0)}, results);
  EXPECT_TRUE(results[0].blocked);
  EXPECT_TRUE(results[1].blocked);
  EXPECT_EQ(b, results[0].blockedBy);
  EXPECT_EQ(a, results[1].blockedBy);
  EXPECT_NEAR(0.7, results[0].achieved.x, 1e-3);
  EXPECT_NEAR(-0.7, results[1].achieved.x, 1e-3);
  // They rest at the skin distance, not inside each other.
  const double gap = world.Pose(b).Pos().X() - world.Pose(a).Pos().X() - 0.6;
  EXPECT_GE(gap, 0.0);
  EXPECT_NEAR(0.0, gap, 2.0 * kSkin + 1e-5);

  // Next step they stay apart.
  world.Move({Req(a, 1.0, 0.0, 0.0), Req(b, -1.0, 0.0, 0.0)}, results);
  EXPECT_EQ(0.0, results[0].achieved.x);
  EXPECT_EQ(0.0, results[1].achieved.x);

  // One contact pair, reported once, from the robot with the lower id.
  std::vector<PlanarContact> contacts;
  world.ContactsOf(a, contacts);
  ASSERT_EQ(2u, contacts.size());
  for (const auto &c : contacts)
  {
    EXPECT_EQ(a, c.modelA);
    EXPECT_EQ(b, c.modelB);
    EXPECT_NEAR(1.0, c.normal.X(), 1e-6);
    EXPECT_NEAR(0.0, c.normal.Y(), 1e-6);
    EXPECT_NEAR(kSkin, c.separation, 5e-5);
  }

  PlanarCollisionOptions options;
  options.reportContacts = true;
  PlanarCollisionWorld reporting(options);
  reporting.AddMovableFootprints(a, {BoxFp(0, 0, 0.6, 0.4)},
      math::Pose3d(-1.0, 0, 0, 0, 0, 0));
  reporting.AddMovableFootprints(b, {BoxFp(0, 0, 0.6, 0.4)},
      math::Pose3d(1.0, 0, 0, 0, 0, 0));
  // Listed in the other order: the contacts are the same.
  reporting.Move({Req(b, -1.0, 0.0, 0.0), Req(a, 1.0, 0.0, 0.0)}, results);
  EXPECT_EQ(a, results[0].blockedBy);
  EXPECT_EQ(b, results[1].blockedBy);
  ASSERT_EQ(2u, reporting.Contacts().size());
  EXPECT_EQ(a, reporting.Contacts()[0].modelA);
  EXPECT_EQ(b, reporting.Contacts()[0].modelB);
}

/////////////////////////////////////////////////
TEST(PlanarCollision, OneRobotChasesAnother)
{
  PlanarCollisionWorld world;
  const Entity a = 1, b = 2;
  world.AddMovableFootprints(a, {BoxFp(0, 0, 0.6, 0.4)},
      math::Pose3d(0.0, 0, 0, 0, 0, 0));
  world.AddMovableFootprints(b, {BoxFp(0, 0, 0.6, 0.4)},
      math::Pose3d(1.0, 0, 0, 0, 0, 0));
  std::vector<PlanarMotionResult> results;
  // A drives 2 m, B drives 0.5 m the same way. A catches up and is cut, B
  // is not.
  world.Move({Req(a, 2.0, 0.0, 0.0), Req(b, 0.5, 0.0, 0.0)}, results);
  EXPECT_TRUE(results[0].blocked);
  EXPECT_FALSE(results[1].blocked);
  EXPECT_NEAR(0.5, results[1].achieved.x, 1e-12);
  // A reaches B's back after 4/15 of the step and stops there, at 8/15.
  // It does not follow B for the rest of the step.
  EXPECT_NEAR(8.0 / 15.0, results[0].achieved.x, 2e-3);

  // A robot that is cut makes the next one wait, in the same call.
  PlanarCollisionWorld line;
  line.AddMovableFootprints(1, {BoxFp(0, 0, 0.6, 0.4)},
      math::Pose3d(0.0, 0, 0, 0, 0, 0));
  line.AddMovableFootprints(2, {BoxFp(0, 0, 0.6, 0.4)},
      math::Pose3d(1.0, 0, 0, 0, 0, 0));
  line.AddMovableFootprints(3, {BoxFp(0, 0, 0.6, 0.4)},
      math::Pose3d(2.0, 0, 0, 0, 0, 0));
  line.AddStaticFootprints(9, {BoxFp(5.0, 0, 0.2, 4.0)});
  line.Move({Req(1, 5.0, 0.0, 0.0), Req(2, 5.0, 0.0, 0.0),
      Req(3, 5.0, 0.0, 0.0)}, results);
  // Robot 3 meets the wall (face 4.9, its front is 0.3 ahead), the others
  // queue up behind it at 0.6 spacing.
  EXPECT_NEAR(4.6, line.Pose(3).Pos().X(), 2e-3);
  EXPECT_NEAR(4.0, line.Pose(2).Pos().X(), 5e-3);
  EXPECT_NEAR(3.4, line.Pose(1).Pos().X(), 1e-2);
  for (Entity e : {1, 2, 3})
  {
    std::vector<PlanarContact> contacts;
    line.ContactsOf(e, contacts);
    for (const auto &c : contacts)
      EXPECT_GE(c.separation, 0.0);
  }
}

/////////////////////////////////////////////////
TEST(PlanarCollision, SlideAndRobotsAreOrderIndependent)
{
  auto run = [](const std::vector<int> &_order)
  {
    PlanarCollisionWorld world;
    world.AddStaticFootprints(100, {BoxFp(3.0, 0.0, 0.2, 10.0)});
    world.AddMovableFootprints(1, {BoxFp(0, 0, 0.6, 0.4)},
        math::Pose3d(1.0, 0.0, 0, 0, 0, 0));
    world.AddMovableFootprints(2, {BoxFp(0, 0, 0.6, 0.4)},
        math::Pose3d(1.0, 0.8, 0, 0, 0, 0));
    world.AddMovableFootprints(3, {CircleFp(0, 0, 0.25)},
        math::Pose3d(0.0, 0.4, 0, 0, 0, 0));
    std::vector<PlanarMotionRequest> reqs{
      Req(1, 2.0, 0.5, 0.2, PlanarResponse::Slide),
      Req(2, 2.0, -0.5, -0.2, PlanarResponse::Slide),
      Req(3, 3.0, 0.1, 0.0, PlanarResponse::Stop)};
    std::vector<PlanarMotionRequest> ordered(_order.size());
    std::transform(_order.begin(), _order.end(), ordered.begin(),
        [&](int _i) { return reqs[static_cast<std::size_t>(_i)]; });
    std::vector<PlanarMotionResult> results;
    world.Move(ordered, results);
    std::vector<math::Pose3d> poses{world.Pose(1), world.Pose(2),
        world.Pose(3)};
    return poses;
  };
  const auto base = run({0, 1, 2});
  for (const auto &order : std::vector<std::vector<int>>{
      {2, 1, 0}, {1, 2, 0}, {0, 2, 1}})
  {
    const auto poses = run(order);
    for (std::size_t i = 0; i < base.size(); ++i)
    {
      // Bit-identical, not just close.
      EXPECT_EQ(base[i].Pos().X(), poses[i].Pos().X());
      EXPECT_EQ(base[i].Pos().Y(), poses[i].Pos().Y());
      EXPECT_EQ(base[i].Rot().Yaw(), poses[i].Rot().Yaw());
    }
  }
  // Something happened: the first robot ends up against the wall.
  EXPECT_LT(base[0].Pos().X(), 3.0 - 0.1 - 0.3);
  EXPECT_GT(base[0].Pos().X(), 1.5);
}

/////////////////////////////////////////////////
/// \brief A warehouse-like random scene for the determinism and no-overlap
/// tests.
struct Scene
{
  PlanarCollisionWorld world;
  std::vector<PlanarMotionRequest> requests;

  explicit Scene(unsigned _seed, int _robots, bool _slide)
  {
    std::mt19937 gen(_seed);
    std::uniform_real_distribution<double> uni(-1.0, 1.0);
    // Shelves in rows.
    Entity id = 1000;
    for (int row = 0; row < 6; ++row)
    {
      for (int col = 0; col < 10; ++col)
      {
        this->world.AddStaticFootprints(id++, {BoxFp(-9.0 + 2.0 * col,
            -7.5 + 3.0 * row, 1.2, 0.5, 0.0, 1.8, 0.1 * uni(gen))});
      }
    }
    for (int i = 0; i < _robots; ++i)
    {
      const Entity e = static_cast<Entity>(i + 1);
      PlanarFootprint body = (i % 2 == 0) ? BoxFp(0, 0, 0.7, 0.5, 0.05, 0.4) :
          CircleFp(0.05, 0, 0.3, 0.05, 0.4);
      // On a grid in the aisles, clear of the shelves and of each other.
      this->world.AddMovableFootprints(e, {body},
          math::Pose3d(-8.25 + 1.5 * (i % 12), -6.0 + 3.0 * ((i / 12) % 5),
            0, 0, 0, 3.0 * uni(gen)));
      this->requests.push_back(Req(e, 0.0, 0.0, 0.0,
          _slide ? PlanarResponse::Slide : PlanarResponse::Stop));
    }
  }

  /// \brief Set the motions for a step. The motion of a robot depends on its
  /// id and the step only, not on its place in the list of requests.
  void Step(int _step, double _speed)
  {
    for (auto &r : this->requests)
    {
      std::mt19937 gen(static_cast<unsigned>(1000003 * _step + r.model));
      std::uniform_real_distribution<double> uni(-1.0, 1.0);
      const double yaw = this->world.Pose(r.model).Rot().Yaw();
      const double v = _speed * (0.5 + 0.5 * uni(gen));
      r.desired.x = v * std::cos(yaw);
      r.desired.y = v * std::sin(yaw);
      r.desired.yaw = 0.05 * uni(gen);
    }
  }
};

/// \brief Run a scene and return the achieved motions, by robot.
/// \param[in] _slide Response of the robots.
/// \param[in] _reversed True to list the requests in reverse order.
std::vector<std::vector<double>> RunScene(bool _slide, bool _reversed)
{
  Scene scene(42, 60, _slide);
  std::vector<std::vector<double>> trace(61);
  std::vector<PlanarMotionResult> results;
  for (int step = 0; step < 150; ++step)
  {
    // Large steps (up to 0.15 m) stress the sweeps.
    scene.Step(step, 0.15);
    if (_reversed)
      std::reverse(scene.requests.begin(), scene.requests.end());
    scene.world.Move(scene.requests, results);
    for (std::size_t i = 0; i < results.size(); ++i)
    {
      auto &t = trace[scene.requests[i].model];
      t.push_back(results[i].achieved.x);
      t.push_back(results[i].achieved.y);
      t.push_back(results[i].achieved.yaw);
    }
    // Nobody is inside anything, at every step.
    for (const auto &req : scene.requests)
    {
      std::vector<PlanarContact> contacts;
      scene.world.ContactsOf(req.model, contacts);
      for (const auto &c : contacts)
      {
        EXPECT_GE(c.separation, -1e-6) << "robot " << req.model << " step "
            << step;
      }
    }
  }
  return trace;
}

TEST(PlanarCollision, DeterministicAndNoOverlap)
{
  for (const bool slide : {false, true})
  {
    const auto first = RunScene(slide, false);
    const auto again = RunScene(slide, false);
    const auto reversed = RunScene(slide, true);
    std::size_t moved = 0;
    for (std::size_t i = 1; i < first.size(); ++i)
    {
      ASSERT_EQ(first[i].size(), again[i].size());
      ASSERT_EQ(first[i].size(), reversed[i].size());
      // Bit-identical, also when the requests come in another order.
      EXPECT_EQ(0, std::memcmp(first[i].data(), again[i].data(),
          sizeof(double) * first[i].size())) << "robot " << i;
      EXPECT_EQ(0, std::memcmp(first[i].data(), reversed[i].data(),
          sizeof(double) * first[i].size())) << "robot " << i;
      moved += std::count_if(first[i].begin(), first[i].end(),
          [](double _v) { return std::abs(_v) > 1e-3; });
    }
    EXPECT_GT(moved, 1000u);
  }
}

/////////////////////////////////////////////////
TEST(PlanarCollision, RandomMovesNeverPenetrateAndStopAtContact)
{
  std::mt19937 gen(2024);
  std::uniform_real_distribution<double> uni(-1.0, 1.0);
  int blocked = 0;
  int turned = 0;
  for (int trial = 0; trial < 400; ++trial)
  {
    PlanarCollisionWorld world;
    // A few obstacles around the origin: boxes, circles, a thin wall.
    for (int i = 0; i < 6; ++i)
    {
      const double x = 2.5 * uni(gen);
      const double y = 2.5 * uni(gen);
      if (i % 3 == 0)
      {
        world.AddStaticFootprints(10 + i, {CircleFp(x, y, 0.1 + 0.2 * std::abs(
            uni(gen)))});
      }
      else if (i % 3 == 1)
      {
        world.AddStaticFootprints(10 + i, {BoxFp(x, y,
            0.2 + 0.8 * std::abs(uni(gen)), 0.2 + 0.8 * std::abs(uni(gen)),
            0.05, 0.5, 3.0 * uni(gen))});
      }
      else
      {
        world.AddStaticFootprints(10 + i, {BoxFp(x, y, 0.01, 2.0, 0.05, 0.5,
            3.0 * uni(gen))});
      }
    }
    // The robot starts clear of everything: look for a free pose.
    const Entity robot = 1;
    bool placed = false;
    for (int attempt = 0; attempt < 50 && !placed; ++attempt)
    {
      const PlanarFootprint body = (trial % 2 == 0) ? BoxFp(0.05, 0, 0.5, 0.3)
          : CircleFp(0.1, 0.0, 0.2);
      world.AddMovableFootprints(robot, {body},
          math::Pose3d(2.5 * uni(gen), 2.5 * uni(gen), 0, 0, 0,
            3.0 * uni(gen)));
      std::vector<PlanarContact> contacts;
      world.ContactsOf(robot, contacts);
      placed = contacts.empty();
    }
    if (!placed)
      continue;

    for (int step = 0; step < 10; ++step)
    {
      const bool slide = step % 2 == 1;
      const PlanarMotionRequest req = Req(robot, 0.6 * uni(gen),
          0.6 * uni(gen), (step % 3 == 0 ? 0.0 : 1.5 * uni(gen)),
          slide ? PlanarResponse::Slide : PlanarResponse::Stop);
      std::vector<PlanarMotionResult> results;
      world.Move({req}, results);
      std::vector<PlanarContact> contacts;
      world.ContactsOf(robot, contacts);
      for (const auto &c : contacts)
      {
        ASSERT_GE(c.separation, -1e-6) << "trial " << trial << " step "
            << step;
      }
      if (!results[0].blocked)
      {
        // Free motion is the desired motion, exactly.
        ASSERT_EQ(req.desired.x, results[0].achieved.x);
        ASSERT_EQ(req.desired.y, results[0].achieved.y);
        ASSERT_EQ(req.desired.yaw, results[0].achieved.yaw);
      }
      else
      {
        ++blocked;
        if (std::abs(req.desired.yaw) > 0.0)
          ++turned;
        if (!slide && results[0].achieved.x * req.desired.x +
            results[0].achieved.y * req.desired.y >= 0.0)
        {
          // A stopped robot rests at the skin distance or closer to what
          // stopped it.
          ASSERT_FALSE(contacts.empty()) << "trial " << trial << " step "
              << step;
        }
      }
    }
  }
  // The test must exercise blocking, with and without a turn.
  EXPECT_GT(blocked, 100);
  EXPECT_GT(turned, 30);
}

/////////////////////////////////////////////////
/// \brief Smallest separation of a model to anything within the contact
/// margin, or the margin if it touches nothing.
double MinSeparation(const PlanarCollisionWorld &_world, Entity _model)
{
  std::vector<PlanarContact> contacts;
  _world.ContactsOf(_model, contacts);
  return std::accumulate(contacts.begin(), contacts.end(),
      PlanarCollisionOptions().contactMargin,
      [](double _s, const PlanarContact &_c)
      {
        return std::min(_s, _c.separation);
      });
}

/// \brief Smallest separation of a model along a straight motion from
/// _before by _delta, sampled _n times. The model is put back where it was.
double SweptMinSeparation(PlanarCollisionWorld &_world, Entity _model,
    const math::Pose3d &_before, const PlanarDelta &_delta, int _n)
{
  const math::Pose3d after = _world.Pose(_model);
  double sep = PlanarCollisionOptions().contactMargin;
  for (int k = 0; k <= _n; ++k)
  {
    const double u = static_cast<double>(k) / _n;
    _world.SetPose(_model, math::Pose3d(_before.Pos().X() + u * _delta.x,
        _before.Pos().Y() + u * _delta.y, 0, 0, 0,
        _before.Rot().Yaw() + u * _delta.yaw));
    sep = std::min(sep, MinSeparation(_world, _model));
  }
  _world.SetPose(_model, after);
  return sep;
}

/////////////////////////////////////////////////
/// \brief Ten robots in a queue, all asked for the same motion, against a
/// wall. More robots than passes: robot k is cut only after robot k-1 is.
TEST(PlanarCollision, LongQueueDoesNotOverlap)
{
  PlanarCollisionWorld world;
  world.AddStaticFootprints(1000, {BoxFp(10.0, 0.0, 0.2, 4.0)});
  const int n = 10;
  std::vector<PlanarMotionRequest> reqs;
  for (int i = 0; i < n; ++i)
  {
    const Entity e = static_cast<Entity>(i + 1);
    world.AddMovableFootprints(e, {BoxFp(0, 0, 0.6, 0.4)},
        math::Pose3d(9.9 - 0.3 - 0.0005 - i * 0.601, 0, 0, 0, 0, 0));
    reqs.push_back(Req(e, 0.05, 0.0, 0.0));
  }
  std::vector<PlanarMotionResult> results;
  world.Move(reqs, results);
  for (int i = 0; i < n; ++i)
    EXPECT_GE(MinSeparation(world, i + 1), -1e-6) << "robot " << i + 1;
  // The head stops at the wall, and the queue behind it stops.
  for (int i = 0; i < n; ++i)
    EXPECT_LT(results[static_cast<std::size_t>(i)].achieved.x, 0.0501);
}

/////////////////////////////////////////////////
/// \brief Twenty robots at the same speed drive into a wall, 1 ms steps.
TEST(PlanarCollision, EqualSpeedQueueDoesNotOverlap)
{
  PlanarCollisionWorld world;
  world.AddStaticFootprints(1000, {BoxFp(10.0, 0.0, 0.2, 4.0)});
  const int n = 20;
  std::vector<PlanarMotionRequest> reqs;
  for (int i = 0; i < n; ++i)
  {
    const Entity e = static_cast<Entity>(i + 1);
    world.AddMovableFootprints(e, {BoxFp(0, 0, 0.6, 0.4)},
        math::Pose3d(9.0 - i * 0.62, 0, 0, 0, 0, 0));
    reqs.push_back(Req(e, 0.001, 0.0, 0.0));
  }
  std::vector<PlanarMotionResult> results;
  double worst = 1.0;
  for (int step = 0; step < 1500; ++step)
  {
    world.Move(reqs, results);
    for (int i = 0; i < n; ++i)
        worst = std::min(worst, MinSeparation(world, i + 1));
  }
  EXPECT_GE(worst, -1e-6);
  // The head reached the wall and the queue closed up behind it.
  EXPECT_NEAR(9.9 - 0.3, world.Pose(1).Pos().X(), 2e-3);
  EXPECT_GT(world.Pose(2).Pos().X(), 9.0 - 0.62 + 0.1);
}

/////////////////////////////////////////////////
/// \brief Random groups of robots: boxes and circles, equal and unequal
/// speeds, turns, Stop and Slide, scenery. Nobody ends a step inside
/// anything.
TEST(PlanarCollision, RandomRobotGroupsNeverOverlap)
{
  std::mt19937 gen(77);
  std::uniform_real_distribution<double> uni(-1.0, 1.0);
  int blocked = 0;
  int groups = 0;
  for (int trial = 0; trial < 60; ++trial)
  {
    PlanarCollisionWorld world;
    world.AddStaticFootprints(900, {BoxFp(3.0, 0.0, 0.2, 8.0)});
    world.AddStaticFootprints(901, {BoxFp(0.0, 2.0, 6.0, 0.2)});
    const int n = 4 + trial % 14;
    std::vector<Entity> robots;
    // A row of robots 1 mm to 5 cm apart, or a loose cluster.
    const bool row = trial % 2 == 0;
    double x = 2.5;
    for (int i = 0; i < n; ++i)
    {
      const Entity e = static_cast<Entity>(i + 1);
      const bool box = (i + trial) % 3 != 0;
      const PlanarFootprint body = box ? BoxFp(0, 0, 0.6, 0.4) :
          CircleFp(0, 0, 0.25);
      math::Pose3d pose;
      if (row)
      {
        const double half = box ? 0.3 : 0.25;
        x -= half;
        pose = math::Pose3d(x, 0.0, 0, 0, 0, 0);
        x -= half + 0.001 + 0.05 * std::abs(uni(gen));
      }
      else
      {
        pose = math::Pose3d(-1.5 + 1.0 * (i % 4), -1.5 + 0.8 * (i / 4),
            0, 0, 0, 3.0 * uni(gen));
      }
      world.AddMovableFootprints(e, {body}, pose);
      robots.push_back(e);
    }
    // Skip groups that start inside each other or the scenery.
    bool clear = true;
    for (const Entity e : robots)
      clear = clear && MinSeparation(world, e) >= 0.0;
    if (!clear)
      continue;
    ++groups;

    const double speed = 0.05 + 0.1 * std::abs(uni(gen));
    for (int step = 0; step < 40; ++step)
    {
      std::vector<PlanarMotionRequest> reqs;
      for (const Entity e : robots)
      {
        const bool equal = trial % 4 < 2;
        const double v = equal ? speed : speed * (0.5 + 0.5 * uni(gen));
        const double dir = row ? 0.0 : 3.0 * uni(gen);
        reqs.push_back(Req(e, v * std::cos(dir), v * std::sin(dir),
            (step % 3 == 0) ? 0.1 * uni(gen) : 0.0,
            (step + trial) % 2 ? PlanarResponse::Slide :
            PlanarResponse::Stop));
      }
      if (trial % 3 == 0)
        std::reverse(reqs.begin(), reqs.end());
      std::vector<PlanarMotionResult> results;
      world.Move(reqs, results);
      blocked += static_cast<int>(std::count_if(results.begin(),
          results.end(),
          [](const PlanarMotionResult &_r) { return _r.blocked; }));
      for (const Entity e : robots)
      {
        ASSERT_GE(MinSeparation(world, e), -1e-6) << "trial " << trial
            << " step " << step << " robot " << e;
      }
    }
  }
  EXPECT_GT(groups, 30);
  EXPECT_GT(blocked, 500);
}

/////////////////////////////////////////////////
/// \brief A 2 m bar turns about its center past a 1 cm post. Every turn
/// beyond a quarter turn must stop: at 90 degrees the bar lies on the post.
TEST(PlanarCollision, TurningBarDoesNotPassThroughPost)
{
  int stopped = 0;
  for (int i = 0; i < 351; ++i)
  {
    const double turn = 1.7 + (3.0 - 1.7) * i / 350.0;
    for (const double sign : {1.0, -1.0})
    {
      PlanarCollisionWorld world;
      world.AddMovableFootprints(1, {BoxFp(0, 0, 2.0, 0.02)},
          math::Pose3d::Zero);
      // The post sits on the side the bar turns toward.
      world.AddStaticFootprints(2, {CircleFp(0.0, sign * 0.975, 0.005)});
      std::vector<PlanarMotionResult> results;
      world.Move({Req(1, 0.0, 0.0, sign * turn)}, results);
      EXPECT_TRUE(results[0].blocked) << "turn " << turn;
      EXPECT_LT(std::abs(results[0].achieved.yaw), GZ_PI / 2)
          << "turn " << turn;
      EXPECT_GE(MinSeparation(world, 1), -1e-6) << "turn " << turn;
      stopped += results[0].blocked ? 1 : 0;
    }
  }
  EXPECT_EQ(702, stopped);
}

/////////////////////////////////////////////////
/// \brief The same bar, turning while it travels, past a thin wall and with
/// several step sizes. The achieved path never goes through.
TEST(PlanarCollision, TurningRobotSweptPathStaysClear)
{
  std::mt19937 gen(5);
  std::uniform_real_distribution<double> uni(-1.0, 1.0);
  int blocked = 0;
  for (int trial = 0; trial < 300; ++trial)
  {
    PlanarCollisionWorld world;
    // Thin features: a 2 mm wall, a post, a thin box.
    world.AddStaticFootprints(10, {BoxFp(0.9 * uni(gen), 1.0 + 0.3 * uni(gen),
        1.0, 0.002, 0.05, 0.5, 3.0 * uni(gen))});
    world.AddStaticFootprints(11, {CircleFp(0.8 * uni(gen), 0.8 * uni(gen),
        0.004)});
    world.AddStaticFootprints(12, {BoxFp(0.8 * uni(gen), -1.0 * uni(gen),
        0.01, 0.4, 0.05, 0.5, 3.0 * uni(gen))});
    const PlanarFootprint body = BoxFp(0, 0, 1.6, 0.03);
    const math::Pose3d start(0.0, 0.0, 0, 0, 0, 3.0 * uni(gen));
    world.AddMovableFootprints(1, {body}, start);
    if (MinSeparation(world, 1) < 0.0)
      continue;
    std::vector<PlanarMotionResult> results;
    world.Move({Req(1, 0.2 * uni(gen), 0.2 * uni(gen), 3.0 * uni(gen))},
        results);
    blocked += results[0].blocked ? 1 : 0;
    EXPECT_GE(SweptMinSeparation(world, 1, start,
        results[0].achieved, 400), -(0.5 * kSkin + 1e-6))
        << "trial " << trial;
  }
  EXPECT_GT(blocked, 100);
}

/////////////////////////////////////////////////
/// \brief The contact list is sorted by model ids, whatever the order of the
/// requests.
TEST(PlanarCollision, ContactsDoNotDependOnRequestOrder)
{
  PlanarCollisionOptions options;
  options.reportContacts = true;
  auto run = [&](bool _reversed)
  {
    PlanarCollisionWorld world(options);
    world.AddStaticFootprints(100, {BoxFp(2.0, 0.0, 0.2, 10.0)});
    world.AddMovableFootprints(1, {BoxFp(0, 0, 0.6, 0.4)},
        math::Pose3d(0.0, -1.0, 0, 0, 0, 0));
    world.AddMovableFootprints(2, {BoxFp(0, 0, 0.6, 0.4)},
        math::Pose3d(0.0, 1.0, 0, 0, 0, 0));
    std::vector<PlanarMotionRequest> reqs{Req(1, 5.0, 0.0, 0.0),
        Req(2, 5.0, 0.0, 0.0)};
    if (_reversed)
      std::reverse(reqs.begin(), reqs.end());
    std::vector<PlanarMotionResult> results;
    world.Move(reqs, results);
    return world.Contacts();
  };
  const auto a = run(false);
  const auto b = run(true);
  ASSERT_EQ(a.size(), b.size());
  ASSERT_GE(a.size(), 4u);
  for (std::size_t i = 0; i < a.size(); ++i)
  {
    EXPECT_EQ(a[i].modelA, b[i].modelA);
    EXPECT_EQ(a[i].modelB, b[i].modelB);
    if (i > 0)
    {
      EXPECT_LE(a[i - 1].modelA, a[i].modelA);
    }
  }
}

/////////////////////////////////////////////////
/// \brief A robot that the scenery would stop late is stopped first by a
/// robot, and the result names the robot.
TEST(PlanarCollision, BlockedByNamesTheFirstCut)
{
  PlanarCollisionWorld world;
  world.AddStaticFootprints(100, {BoxFp(1.2, 0.0, 0.2, 4.0)});
  world.AddMovableFootprints(1, {BoxFp(0, 0, 0.6, 0.4)}, math::Pose3d::Zero);
  world.AddMovableFootprints(2, {BoxFp(0, 0, 0.6, 0.4)},
      math::Pose3d(0.7, 0.0, 0, 0, 0, 0));
  std::vector<PlanarMotionResult> results;
  world.Move({Req(1, 1.0, 0.0, 0.0), Req(2, 0.001, 0.0, 0.0)}, results);
  EXPECT_TRUE(results[0].blocked);
  EXPECT_EQ(2u, results[0].blockedBy);
}

/////////////////////////////////////////////////
/// \brief A floor slab whose top is at the bottom of the band is not an
/// obstacle.
TEST(PlanarCollision, SlabEndingAtTheBandIsDropped)
{
  PlanarCollisionOptions options;
  options.bandMin = 0.0;
  PlanarCollisionWorld world(options);
  EXPECT_FALSE(world.AddStaticFootprints(1,
      {BoxFp(0, 0, 100.0, 100.0, -0.1, 0.0)}));
  PlanarCollisionWorld world2;
  EXPECT_FALSE(world2.AddStaticFootprints(1,
      {BoxFp(0, 0, 100.0, 100.0, -0.1, 0.01)}));
  EXPECT_FALSE(world2.AddStaticFootprints(2,
      {BoxFp(0, 0, 100.0, 100.0, 2.0, 2.5)}));
  EXPECT_TRUE(world2.AddStaticFootprints(3,
      {BoxFp(0, 0, 100.0, 100.0, -0.1, 0.02)}));
}
