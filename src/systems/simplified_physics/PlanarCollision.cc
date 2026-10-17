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

#include <box2d/box2d.h>
#include <box2d/collision.h>
#include <box2d/math_functions.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <numeric>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gz/common/Mesh.hh>
#include <gz/common/SubMesh.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Quaternion.hh>
#include <sdf/Box.hh>
#include <sdf/Capsule.hh>
#include <sdf/Collision.hh>
#include <sdf/Cone.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Ellipsoid.hh>
#include <sdf/Mesh.hh>
#include <sdf/Polyline.hh>
#include <sdf/Sphere.hh>
#include <sdf/Surface.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/Geometry.hh"

namespace gz::sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems::simplified_physics
{
namespace
{
using Vec2 = math::Vector2d;

constexpr double kInf{std::numeric_limits<double>::infinity()};

/// \brief Points and features closer than this are the same, in meters.
constexpr double kTiny{1e-9};

/// \brief Radius of the capsule that stands for a shape without area.
constexpr double kThinRadius{1e-3};

/// \brief Sides of the polygons that approximate a disk or an ellipse.
constexpr int kDiskSides{16};

/// \brief A closing speed below this fraction of the motion counts as zero.
/// It absorbs the rounding of a normal that was computed in floats.
constexpr double kApproachEps{1e-6};

/// \brief Iteration limit of one conservative advancement.
constexpr int kMaxAdvance{64};

/// \brief After this many steps a turning pair is certified by interval
/// splitting instead.
constexpr int kSlowAfter{10};

/// \brief Limit on the distance queries of one interval split. A split that
/// runs out stops the motion at the start of the interval it was on.
constexpr int kMaxLooks{1024};

/// \brief Margin around the box of a movable model in the broadphase, in
/// meters. A model that stays inside it needs no tree update.
constexpr double kFatMargin{0.25};

/// \brief Padding of boxes in the static tree, in meters. It covers the
/// rounding of coordinates to floats.
constexpr double kTreePad{5e-3};

/// \brief Most passes between robots, however large a group of linked robots
/// is. The freeze that follows makes the result safe whatever this is.
constexpr int kMaxRobotPassesHard{64};

/// \brief Most knots of a path: the start, one per bend of a slide, and the
/// end.
constexpr int kMaxKnots{12};

/// \brief Index of a "whole model" entry in a tree user data.
constexpr uint32_t kWholeModel{0xFFFF};

// ---------------------------------------------------------------------------
// Footprints.

/// \brief True if the value is exactly zero.
bool IsZero(double _v)
{
  return std::fpclassify(_v) == FP_ZERO;
}

/// \brief A point in the frame of a pose, expressed in the parent frame.
math::Vector3d Transform(const math::Pose3d &_pose, const math::Vector3d &_p)
{
  return _pose.Pos() + _pose.Rot() * _p;
}

double Cross(const Vec2 &_o, const Vec2 &_a, const Vec2 &_b)
{
  return (_a.X() - _o.X()) * (_b.Y() - _o.Y()) -
         (_a.Y() - _o.Y()) * (_b.X() - _o.X());
}

/// \brief Convex hull of a point set, counter-clockwise, without collinear
/// or repeated vertices. Andrew's monotone chain.
std::vector<Vec2> ConvexHull(std::vector<Vec2> _points)
{
  // Snap to a 0.1 nm grid, so that points that are equal up to rounding
  // sort as equal. Otherwise a vertical side with rounding noise in x has its
  // corners removed as nearly collinear.
  for (auto &p : _points)
  {
    p.Set(std::round(p.X() * 1e10) / 1e10, std::round(p.Y() * 1e10) / 1e10);
  }
  std::sort(_points.begin(), _points.end(),
      [](const Vec2 &_a, const Vec2 &_b)
      {
        return _a.X() < _b.X() ||
            (!(_b.X() < _a.X()) && _a.Y() < _b.Y());
      });
  std::vector<Vec2> unique = std::move(_points);
  unique.erase(std::unique(unique.begin(), unique.end(),
      [](const Vec2 &_a, const Vec2 &_b)
      {
        return std::abs(_a.X() - _b.X()) <= kTiny &&
            std::abs(_a.Y() - _b.Y()) <= kTiny;
      }), unique.end());
  if (unique.size() < 3u)
    return unique;

  std::vector<Vec2> hull(2 * unique.size());
  std::size_t k = 0;
  for (std::size_t i = 0; i < unique.size(); ++i)
  {
    while (k >= 2 && Cross(hull[k - 2], hull[k - 1], unique[i]) <= 1e-12)
      --k;
    hull[k++] = unique[i];
  }
  const std::size_t lower = k + 1;
  for (std::size_t i = unique.size() - 1; i > 0; --i)
  {
    while (k >= lower &&
        Cross(hull[k - 2], hull[k - 1], unique[i - 1]) <= 1e-12)
    {
      --k;
    }
    hull[k++] = unique[i - 1];
  }
  hull.resize(k - 1);
  return hull;
}

/// \brief Add the footprints for a set of projected points: their convex
/// hull, split into polygons of at most 8 vertices.
void AddHull(const std::vector<Vec2> &_points, double _zMin, double _zMax,
    std::vector<PlanarFootprint> &_out)
{
  const auto hull = ConvexHull(_points);
  if (hull.empty())
    return;

  PlanarFootprint base;
  base.zMin = _zMin;
  base.zMax = _zMax;

  if (hull.size() == 1u)
  {
    base.kind = PlanarShapeKind::Circle;
    base.points = {hull[0]};
    base.radius = kThinRadius;
    _out.push_back(base);
    return;
  }
  if (hull.size() == 2u)
  {
    base.kind = PlanarShapeKind::Capsule;
    base.points = {hull[0], hull[1]};
    base.radius = kThinRadius;
    _out.push_back(base);
    return;
  }

  base.kind = PlanarShapeKind::Polygon;
  const std::size_t kMax = B2_MAX_POLYGON_VERTICES;
  if (hull.size() <= kMax)
  {
    base.points = hull;
    _out.push_back(base);
    return;
  }

  // Fan out from vertex 0. Each piece is a sub-polygon of a convex polygon,
  // so it is convex, and the pieces cover the hull exactly.
  base.points.assign(hull.begin(), hull.begin() + kMax);
  _out.push_back(base);
  std::size_t start = kMax - 1;
  while (start < hull.size() - 1)
  {
    const std::size_t stop = std::min(start + kMax - 2, hull.size() - 1);
    base.points.clear();
    base.points.push_back(hull[0]);
    for (std::size_t i = start; i <= stop; ++i)
      base.points.push_back(hull[i]);
    _out.push_back(base);
    start = stop;
  }
}

/// \brief Add a circle footprint.
void AddCircle(const Vec2 &_center, double _radius, double _zMin, double _zMax,
    std::vector<PlanarFootprint> &_out)
{
  PlanarFootprint fp;
  fp.kind = PlanarShapeKind::Circle;
  fp.points = {_center};
  fp.radius = std::max(_radius, kTiny);
  fp.zMin = _zMin;
  fp.zMax = _zMax;
  _out.push_back(fp);
}

/// \brief The corners of the polygon around a disk of radius _r, whose
/// sides are tangent to the disk at angles k * 2 pi / n. The result
/// encloses the disk and touches it at those angles.
void DiskPoints(const math::Vector3d &_center, const math::Vector3d &_u,
    const math::Vector3d &_v, double _r, std::vector<Vec2> &_out)
{
  const double outer = _r / std::cos(GZ_PI / kDiskSides);
  for (int k = 0; k < kDiskSides; ++k)
  {
    const double a = (k + 0.5) * 2.0 * GZ_PI / kDiskSides;
    const auto p = _center + _u * (outer * std::cos(a)) +
        _v * (outer * std::sin(a));
    _out.emplace_back(p.X(), p.Y());
  }
}

void BoxFootprints(const math::Vector3d &_size, const math::Pose3d &_pose,
    std::vector<PlanarFootprint> &_out)
{
  std::vector<Vec2> pts;
  double zMin = kInf;
  double zMax = -kInf;
  for (int sx = -1; sx <= 1; sx += 2)
  {
    for (int sy = -1; sy <= 1; sy += 2)
    {
      for (int sz = -1; sz <= 1; sz += 2)
      {
        const auto p = Transform(_pose, math::Vector3d(
            0.5 * sx * _size.X(), 0.5 * sy * _size.Y(),
            0.5 * sz * _size.Z()));
        pts.emplace_back(p.X(), p.Y());
        zMin = std::min(zMin, p.Z());
        zMax = std::max(zMax, p.Z());
      }
    }
  }
  AddHull(pts, zMin, zMax, _out);
}

void CylinderFootprints(double _radius, double _length,
    const math::Pose3d &_pose, std::vector<PlanarFootprint> &_out)
{
  const math::Vector3d axis = _pose.Rot() * math::Vector3d::UnitZ;
  const math::Vector3d c = _pose.Pos();
  const double horizontal =
      std::sqrt(std::max(0.0, 1.0 - axis.Z() * axis.Z()));
  const double zHalf =
      std::abs(axis.Z()) * 0.5 * _length + _radius * horizontal;
  if (horizontal < kTiny)
  {
    AddCircle(Vec2(c.X(), c.Y()), _radius, c.Z() - zHalf, c.Z() + zHalf, _out);
    return;
  }
  // Two orthonormal vectors across the axis. u is horizontal, so the disk
  // is tangent to the hull at the extreme of the footprint across the axis.
  const math::Vector3d u = axis.Cross(math::Vector3d::UnitZ).Normalized();
  const math::Vector3d v = axis.Cross(u);
  std::vector<Vec2> pts;
  DiskPoints(c + axis * (0.5 * _length), u, v, _radius, pts);
  DiskPoints(c - axis * (0.5 * _length), u, v, _radius, pts);
  AddHull(pts, c.Z() - zHalf, c.Z() + zHalf, _out);
}

void CapsuleFootprints(double _radius, double _length,
    const math::Pose3d &_pose, std::vector<PlanarFootprint> &_out)
{
  const math::Vector3d axis = _pose.Rot() * math::Vector3d::UnitZ;
  const math::Vector3d p1 = _pose.Pos() + axis * (0.5 * _length);
  const math::Vector3d p2 = _pose.Pos() - axis * (0.5 * _length);
  const double zMin = std::min(p1.Z(), p2.Z()) - _radius;
  const double zMax = std::max(p1.Z(), p2.Z()) + _radius;
  const Vec2 a(p1.X(), p1.Y());
  const Vec2 b(p2.X(), p2.Y());
  if (a.Distance(b) < kTiny)
  {
    AddCircle(Vec2(_pose.Pos().X(), _pose.Pos().Y()), _radius, zMin, zMax,
        _out);
    return;
  }
  PlanarFootprint fp;
  fp.kind = PlanarShapeKind::Capsule;
  fp.points = {a, b};
  fp.radius = _radius;
  fp.zMin = zMin;
  fp.zMax = zMax;
  _out.push_back(fp);
}

void EllipsoidFootprints(const math::Vector3d &_radii,
    const math::Pose3d &_pose, std::vector<PlanarFootprint> &_out)
{
  // The support function of the ellipsoid in a world direction d is
  // | diag(radii) R^T d |.
  const math::Matrix3d rot(_pose.Rot());
  auto support = [&](const math::Vector3d &_d)
  {
    const math::Vector3d local = rot.Transposed() * _d;
    return std::sqrt(std::pow(_radii.X() * local.X(), 2) +
        std::pow(_radii.Y() * local.Y(), 2) +
        std::pow(_radii.Z() * local.Z(), 2));
  };
  const math::Vector3d c = _pose.Pos();
  const double zHalf = support(math::Vector3d::UnitZ);

  double h[kDiskSides];
  double hMin = kInf;
  double hMax = -kInf;
  for (int k = 0; k < kDiskSides; ++k)
  {
    const double a = k * 2.0 * GZ_PI / kDiskSides;
    h[k] = support(math::Vector3d(std::cos(a), std::sin(a), 0.0));
    hMin = std::min(hMin, h[k]);
    hMax = std::max(hMax, h[k]);
  }
  if (hMax - hMin < kTiny)
  {
    AddCircle(Vec2(c.X(), c.Y()), hMax, c.Z() - zHalf, c.Z() + zHalf, _out);
    return;
  }
  // Intersect the tangent lines n_k . x = c . n_k + h_k of neighboring
  // directions.
  std::vector<Vec2> pts;
  for (int k = 0; k < kDiskSides; ++k)
  {
    const int j = (k + 1) % kDiskSides;
    const double a0 = k * 2.0 * GZ_PI / kDiskSides;
    const double a1 = j * 2.0 * GZ_PI / kDiskSides;
    const double n0x = std::cos(a0), n0y = std::sin(a0);
    const double n1x = std::cos(a1), n1y = std::sin(a1);
    const double b0 = h[k] + c.X() * n0x + c.Y() * n0y;
    const double b1 = h[j] + c.X() * n1x + c.Y() * n1y;
    const double det = n0x * n1y - n0y * n1x;
    pts.emplace_back((b0 * n1y - b1 * n0y) / det, (n0x * b1 - n1x * b0) / det);
  }
  AddHull(pts, c.Z() - zHalf, c.Z() + zHalf, _out);
}

}  // namespace

std::vector<PlanarFootprint> FootprintsFromMesh(const common::Mesh &_mesh,
    const math::Vector3d &_scale, const math::Pose3d &_pose)
{
  std::vector<Vec2> pts;
  double zMin = kInf;
  double zMax = -kInf;
  for (unsigned int i = 0; i < _mesh.SubMeshCount(); ++i)
  {
    auto sub = _mesh.SubMeshByIndex(i).lock();
    if (!sub)
      continue;
    for (unsigned int k = 0; k < sub->VertexCount(); ++k)
    {
      const auto v = sub->Vertex(k);
      const auto p = Transform(_pose, math::Vector3d(
          v.X() * _scale.X(), v.Y() * _scale.Y(), v.Z() * _scale.Z()));
      pts.emplace_back(p.X(), p.Y());
      zMin = std::min(zMin, p.Z());
      zMax = std::max(zMax, p.Z());
    }
  }
  std::vector<PlanarFootprint> out;
  if (!pts.empty())
    AddHull(pts, zMin, zMax, out);
  return out;
}

std::vector<PlanarFootprint> FootprintsFromGeometry(
    const sdf::Geometry &_geometry, const math::Pose3d &_pose)
{
  std::vector<PlanarFootprint> out;
  switch (_geometry.Type())
  {
    case sdf::GeometryType::BOX:
      if (_geometry.BoxShape())
        BoxFootprints(_geometry.BoxShape()->Size(), _pose, out);
      break;
    case sdf::GeometryType::CYLINDER:
      if (_geometry.CylinderShape())
      {
        CylinderFootprints(_geometry.CylinderShape()->Radius(),
            _geometry.CylinderShape()->Length(), _pose, out);
      }
      break;
    case sdf::GeometryType::CONE:
      // The bounding cylinder is a superset of the cone.
      if (_geometry.ConeShape())
      {
        CylinderFootprints(_geometry.ConeShape()->Radius(),
            _geometry.ConeShape()->Length(), _pose, out);
      }
      break;
    case sdf::GeometryType::SPHERE:
      if (_geometry.SphereShape())
      {
        const double r = _geometry.SphereShape()->Radius();
        const auto &c = _pose.Pos();
        AddCircle(Vec2(c.X(), c.Y()), r, c.Z() - r, c.Z() + r, out);
      }
      break;
    case sdf::GeometryType::CAPSULE:
      if (_geometry.CapsuleShape())
      {
        CapsuleFootprints(_geometry.CapsuleShape()->Radius(),
            _geometry.CapsuleShape()->Length(), _pose, out);
      }
      break;
    case sdf::GeometryType::ELLIPSOID:
      if (_geometry.EllipsoidShape())
      {
        EllipsoidFootprints(_geometry.EllipsoidShape()->Radii(), _pose, out);
      }
      break;
    case sdf::GeometryType::POLYLINE:
    {
      std::vector<Vec2> pts;
      double zMin = kInf;
      double zMax = -kInf;
      for (const auto &polyline : _geometry.PolylineShape())
      {
        for (const auto &p2 : polyline.Points())
        {
          for (const double z : {0.0, polyline.Height()})
          {
            const auto p = Transform(_pose, math::Vector3d(p2.X(), p2.Y(), z));
            pts.emplace_back(p.X(), p.Y());
            zMin = std::min(zMin, p.Z());
            zMax = std::max(zMax, p.Z());
          }
        }
      }
      if (!pts.empty())
        AddHull(pts, zMin, zMax, out);
      break;
    }
    case sdf::GeometryType::MESH:
      if (_geometry.MeshShape())
      {
        const common::Mesh *mesh = loadMesh(*_geometry.MeshShape());
        if (mesh)
        {
          out = FootprintsFromMesh(*mesh, _geometry.MeshShape()->Scale(),
              _pose);
        }
      }
      break;
    default:
      // Plane, heightmap and empty geometries are the floor.
      break;
  }
  return out;
}

std::vector<PlanarFootprint> ModelFootprints(
    const EntityComponentManager &_ecm, Entity _model, bool _inWorldFrame)
{
  std::vector<Entity> collisions;
  const auto descendants = _ecm.Descendants(_model);
  std::copy_if(descendants.begin(), descendants.end(),
      std::back_inserter(collisions), [&](Entity _e)
      {
        return _ecm.EntityHasComponentType(_e, components::Collision::typeId);
      });
  std::sort(collisions.begin(), collisions.end());

  const math::Pose3d inverseModel = worldPose(_model, _ecm).Inverse();
  std::vector<PlanarFootprint> out;
  for (const Entity e : collisions)
  {
    const auto *geometry = _ecm.Component<components::Geometry>(e);
    if (!geometry)
      continue;
    const auto *element = _ecm.Component<components::CollisionElement>(e);
    if (element)
    {
      const auto *surface = element->Data().Surface();
      if (surface && surface->Contact() &&
          surface->Contact()->CollideBitmask() == 0u)
      {
        continue;
      }
    }
    math::Pose3d pose = worldPose(e, _ecm);
    if (!_inWorldFrame)
      pose = inverseModel * pose;
    for (auto &fp : FootprintsFromGeometry(geometry->Data(), pose))
    {
      fp.collision = e;
      out.push_back(std::move(fp));
    }
  }
  return out;
}

namespace
{
// ---------------------------------------------------------------------------
// Box2D shapes.

/// \brief A planar pose with its cosine and sine.
struct Pose2
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double c{1.0};
  double s{0.0};
};

Pose2 MakePose(double _x, double _y, double _yaw)
{
  return {_x, _y, _yaw, std::cos(_yaw), std::sin(_yaw)};
}

/// \brief One convex shape of a model, ready for Box2D.
struct Shape
{
  PlanarShapeKind kind{PlanarShapeKind::Polygon};
  b2Polygon polygon{};
  b2Capsule capsule{};
  b2Circle circle{};
  b2ShapeProxy proxy{};

  /// \brief Position of the shape's local frame in the model frame. Static
  /// scenery keeps its shapes near their own origin, so that float
  /// coordinates stay small. A movable model has zero here.
  Vec2 origin;

  /// \brief Center of the circle around the shape, in the model frame.
  Vec2 center;

  /// \brief Radius of the circle around the shape.
  double boundRadius{0.0};

  /// \brief Largest distance of a core point (the vertices, or the segment
  /// end points, or the center) from the model origin. The shape is the core
  /// grown by its radius, and a turn about the origin moves the core only.
  double coreRadius{0.0};

  /// \brief Height interval, in the model frame (or world frame for
  /// scenery).
  double zMin{0.0};
  double zMax{0.0};

  Entity collision{kNullEntity};
};

b2Vec2 ToB2(const Vec2 &_v)
{
  return {static_cast<float>(_v.X()), static_cast<float>(_v.Y())};
}

/// \brief Build a Box2D polygon from counter-clockwise vertices relative to
/// _ref. It fills the b2Polygon fields as b2MakePolygon does, but does not
/// merge vertices closer than Box2D's linear slop, which would collapse
/// thin walls.
b2Polygon MakePolygon(const std::vector<Vec2> &_vertices, const Vec2 &_ref)
{
  b2Polygon poly{};
  poly.count = static_cast<int>(_vertices.size());
  poly.radius = 0.0f;
  std::vector<Vec2> v(_vertices.size());
  std::transform(_vertices.begin(), _vertices.end(), v.begin(),
      [&](const Vec2 &_p) { return _p - _ref; });
  double area = 0.0;
  Vec2 centroid;
  for (std::size_t i = 0; i < v.size(); ++i)
  {
    const Vec2 &a = v[i];
    const Vec2 &b = v[(i + 1) % v.size()];
    poly.vertices[i] = ToB2(a);
    Vec2 edge = b - a;
    const double len = edge.Length();
    poly.normals[i] = {static_cast<float>(edge.Y() / len),
        static_cast<float>(-edge.X() / len)};
    // Triangle fan about v[0].
    if (i >= 1 && i + 1 < v.size())
    {
      const Vec2 e1 = a - v[0];
      const Vec2 e2 = b - v[0];
      const double tri = 0.5 * (e1.X() * e2.Y() - e1.Y() * e2.X());
      area += tri;
      centroid += (v[0] + a + b) * (tri / 3.0);
    }
  }
  if (std::abs(area) > 0.0)
    centroid /= area;
  else
    centroid = v[0];
  poly.centroid = ToB2(centroid);
  return poly;
}

/// \brief Make a shape from a footprint.
/// \param[in] _fp The footprint.
/// \param[in] _scenery True for static scenery, whose shapes get a local
/// origin at their center.
Shape MakeShape(const PlanarFootprint &_fp, bool _scenery)
{
  Shape s;
  s.kind = _fp.kind;
  s.collision = _fp.collision;
  s.zMin = _fp.zMin;
  s.zMax = _fp.zMax;

  Vec2 mean = std::accumulate(_fp.points.begin(), _fp.points.end(), Vec2());
  mean /= static_cast<double>(_fp.points.size());
  const Vec2 ref = _scenery ? mean : Vec2::Zero;
  s.origin = ref;

  s.coreRadius = std::accumulate(_fp.points.begin(), _fp.points.end(), 0.0,
      [](double _r, const Vec2 &_p) { return std::max(_r, _p.Length()); });

  std::vector<b2Vec2> local(_fp.points.size());
  std::transform(_fp.points.begin(), _fp.points.end(), local.begin(),
      [&](const Vec2 &_p) { return ToB2(_p - ref); });

  switch (_fp.kind)
  {
    case PlanarShapeKind::Polygon:
      s.polygon = MakePolygon(_fp.points, ref);
      s.proxy = b2MakeProxy(local.data(), static_cast<int>(local.size()), 0.0f);
      s.center = mean;
      for (const auto &p : _fp.points)
        s.boundRadius = std::max(s.boundRadius, p.Distance(mean));
      break;
    case PlanarShapeKind::Circle:
      s.circle.center = local[0];
      s.circle.radius = static_cast<float>(_fp.radius);
      s.proxy = b2MakeProxy(local.data(), 1, s.circle.radius);
      s.center = _fp.points[0];
      s.boundRadius = _fp.radius;
      break;
    case PlanarShapeKind::Capsule:
      s.capsule.center1 = local[0];
      s.capsule.center2 = local[1];
      s.capsule.radius = static_cast<float>(_fp.radius);
      s.proxy = b2MakeProxy(local.data(), 2, s.capsule.radius);
      s.center = mean;
      s.boundRadius = 0.5 * _fp.points[0].Distance(_fp.points[1]) + _fp.radius;
      break;
    default:
      break;
  }
  return s;
}

/// \brief Transform of a shape of B in the frame of A, in floats.
b2Transform Relative(const Pose2 &_a, const Pose2 &_b, const Vec2 &_bOrigin)
{
  const double wx = _b.x + _b.c * _bOrigin.X() - _b.s * _bOrigin.Y();
  const double wy = _b.y + _b.s * _bOrigin.X() + _b.c * _bOrigin.Y();
  const double dx = wx - _a.x;
  const double dy = wy - _a.y;
  b2Transform xf;
  xf.p = {static_cast<float>(_a.c * dx + _a.s * dy),
      static_cast<float>(-_a.s * dx + _a.c * dy)};
  xf.q = {static_cast<float>(_b.c * _a.c + _b.s * _a.s),
      static_cast<float>(_b.s * _a.c - _b.c * _a.s)};
  return xf;
}

/// \brief Distance between two shapes, with B at _xfB in the frame of A.
/// Zero if they overlap. The normal points from A to B and is valid when the
/// distance is positive.
b2DistanceOutput DistanceAt(const Shape &_a, const Shape &_b,
    const b2Transform &_xfB, b2SimplexCache &_cache)
{
  b2DistanceInput in;
  in.proxyA = _a.proxy;
  in.proxyB = _b.proxy;
  in.transformA = b2Transform_identity;
  in.transformB = _xfB;
  // Ask for the distance of the cores and subtract the radii here. With
  // useRadii Box2D ignores the radii when the cores are closer than half a
  // millimeter, which is the range the sweeps work in.
  in.useRadii = false;
  b2DistanceOutput out = b2ShapeDistance(&in, &_cache, nullptr, 0);
  out.distance = std::max(0.0f,
      out.distance - _a.proxy.radius - _b.proxy.radius);
  return out;
}

/// \brief Contact manifold of two shapes, with B at _xfB in the frame of A.
/// The normal points from A to B.
b2Manifold Collide(const Shape &_a, const Shape &_b, const b2Transform &_xfB)
{
  const b2Transform xfA = b2Transform_identity;
  auto rank = [](PlanarShapeKind _k)
  {
    return _k == PlanarShapeKind::Polygon ? 0 :
        (_k == PlanarShapeKind::Capsule ? 1 : 2);
  };
  if (rank(_a.kind) > rank(_b.kind))
  {
    // Box2D takes the shapes in the order polygon, capsule, circle.
    b2Manifold m = Collide(_b, _a, b2InvMulTransforms(_xfB, xfA));
    // The manifold is in the frame of B. Move it to the frame of A.
    m.normal = b2RotateVector(_xfB.q, b2Neg(m.normal));
    for (int i = 0; i < m.pointCount; ++i)
      m.points[i].point = b2TransformPoint(_xfB, m.points[i].point);
    return m;
  }
  switch (_a.kind)
  {
    case PlanarShapeKind::Polygon:
      switch (_b.kind)
      {
        case PlanarShapeKind::Polygon:
          return b2CollidePolygons(&_a.polygon, xfA, &_b.polygon, _xfB);
        case PlanarShapeKind::Capsule:
          return b2CollidePolygonAndCapsule(
              &_a.polygon, xfA, &_b.capsule, _xfB);
        default:
          return b2CollidePolygonAndCircle(&_a.polygon, xfA, &_b.circle, _xfB);
      }
    case PlanarShapeKind::Capsule:
      if (_b.kind == PlanarShapeKind::Capsule)
        return b2CollideCapsules(&_a.capsule, xfA, &_b.capsule, _xfB);
      return b2CollideCapsuleAndCircle(&_a.capsule, xfA, &_b.circle, _xfB);
    default:
      return b2CollideCircles(&_a.circle, xfA, &_b.circle, _xfB);
  }
}

// ---------------------------------------------------------------------------
// Paths.

/// \brief Motion of a model over the step, as a polyline in time. Time runs
/// from 0 to 1 over the step. The pose is linear in time between knots.
struct Path
{
  int count{1};
  double time[kMaxKnots]{};
  Pose2 pose[kMaxKnots]{};
};

/// \brief A pose on a path with the velocity of its segment, per unit of
/// step time.
struct Sample
{
  Pose2 pose;
  double vx{0.0};
  double vy{0.0};
  double w{0.0};

  /// \brief End of the segment the sample is on.
  double tNext{kInf};
};

/// \brief Evaluate a path at time _t. After _cap the model holds still at
/// the pose it had at _cap.
Sample Evaluate(const Path &_path, double _cap, double _t)
{
  Sample s;
  const int last = _path.count - 1;
  if (_path.count == 1)
  {
    s.pose = _path.pose[0];
    return s;
  }
  const double t = std::min(_t, _cap);
  if (t >= _path.time[last])
  {
    s.pose = _path.pose[last];
    return s;
  }
  int k = 0;
  while (k + 2 < _path.count && t >= _path.time[k + 1])
    ++k;
  const double t0 = _path.time[k];
  const double dt = _path.time[k + 1] - t0;
  const Pose2 &p0 = _path.pose[k];
  const Pose2 &p1 = _path.pose[k + 1];
  const double u = (t - t0) / dt;
  s.pose = MakePose(p0.x + u * (p1.x - p0.x), p0.y + u * (p1.y - p0.y),
      p0.yaw + u * (p1.yaw - p0.yaw));
  if (_t < _cap)
  {
    s.vx = (p1.x - p0.x) / dt;
    s.vy = (p1.y - p0.y) / dt;
    s.w = (p1.yaw - p0.yaw) / dt;
    s.tNext = std::min(_path.time[k + 1], _cap);
  }
  return s;
}

/// \brief Result of a sweep of two shapes.
struct PairHit
{
  bool hit{false};

  /// \brief Time of the contact.
  double t{1.0};

  /// \brief Unit normal in the world frame, from A to B.
  double nx{0.0};
  double ny{0.0};

  /// \brief How fast A (B) moves toward the other along the normal, with
  /// the bound of its turn.
  double approachA{0.0};
  double approachB{0.0};
};

/// \brief Settings the sweep needs.
struct SweepSettings
{
  double skin{1e-4};
};

/// \brief First time two shapes come within the skin distance, in
/// [0, _tEnd], by conservative advancement.
///
/// At each step the distance d between the shapes and a normal n come from
/// Box2D's GJK. The shapes cannot get closer along n than the bound
/// (vA - vB) . n + |wA| rA + |wB| rB per unit time, where r is the distance
/// of the shape's core points from the model origin. So the time can advance
/// by (d - skin) / bound without a contact. In a segment where the bound is
/// not positive the shapes cannot touch. When the gap is within the skin the
/// answer is exact for a motion without turning. With a turn the bound is
/// loose, so the function samples the true distance within the skin, and
/// after a few steps that do not converge.
PairHit PairSweep(const SweepSettings &_set, const Shape &_a, const Path &_pa,
    double _capA, const Shape &_b, const Path &_pb, double _capB, double _tEnd)
{
  PairHit result;
  const double skin = _set.skin;
  const double tol = 0.25 * skin;
  b2SimplexCache cache{};

  struct State
  {
    Sample a;
    Sample b;
    b2Transform xf;
    double dist{0.0};
    bool overlap{false};
    double nx{0.0};
    double ny{0.0};
    bool valid{false};
  };

  // Distance, normal and samples at time _t.
  auto look = [&](double _t) {
    State st;
    st.a = Evaluate(_pa, _capA, _t);
    st.b = Evaluate(_pb, _capB, _t);
    st.xf = Relative(st.a.pose, st.b.pose, _b.origin);
    const b2DistanceOutput out = DistanceAt(_a, _b, st.xf, cache);
    st.dist = out.distance;
    st.overlap = out.distance <= 0.0f;
    b2Vec2 n = out.normal;
    st.valid = true;
    if (st.overlap)
    {
      const b2Manifold m = Collide(_a, _b, st.xf);
      if (m.pointCount == 0)
      {
        st.valid = false;
      }
      n = m.normal;
    }
    st.nx = st.a.pose.c * n.x - st.a.pose.s * n.y;
    st.ny = st.a.pose.s * n.x + st.a.pose.c * n.y;
    return st;
  };

  auto fill = [&](double _t, const State &_st) {
    result.hit = true;
    result.t = _t;
    result.nx = _st.nx;
    result.ny = _st.ny;
    result.approachA = _st.a.vx * _st.nx + _st.a.vy * _st.ny +
        std::abs(_st.a.w) * _a.coreRadius;
    result.approachB = -(_st.b.vx * _st.nx + _st.b.vy * _st.ny) +
        std::abs(_st.b.w) * _b.coreRadius;
  };

  double t = 0.0;
  State st;
  for (int it = 0; it < kMaxAdvance; ++it)
  {
    st = look(t);
    const double segEnd = std::min({st.a.tNext, st.b.tNext, _tEnd});
    const double rot = std::abs(st.a.w) * _a.coreRadius +
        std::abs(st.b.w) * _b.coreRadius;
    const double scale = std::hypot(st.a.vx, st.a.vy) +
        std::hypot(st.b.vx, st.b.vy) + rot;

    // Move on to the next segment without a hit.
    auto next = [&]() {
      if (segEnd >= _tEnd || segEnd <= t)
        return false;
      t = segEnd;
      return true;
    };

    if (!st.valid)
    {
      if (next())
        continue;
      return result;
    }
    const double closing = (st.a.vx - st.b.vx) * st.nx +
        (st.a.vy - st.b.vy) * st.ny + rot;
    if (IsZero(scale) || closing <= kApproachEps * scale)
    {
      if (next())
        continue;
      return result;
    }
    if (st.overlap)
    {
      fill(t, st);
      return result;
    }
    const double gap = st.dist - skin;
    // The bound of a turn is loose, so the advancement can converge slowly
    // when a turning shape ends the segment close to another. Then sample.
    const bool slow = it >= kSlowAfter && rot > 0.0;
    if (gap > tol && !slow)
    {
      const double dt = gap / closing;
      if (t + dt >= segEnd)
      {
        if (next())
          continue;
        return result;
      }
      t += dt;
      continue;
    }
    if (rot <= 0.0)
    {
      fill(t, st);
      return result;
    }

    // Within the skin and turning, or advancing too slowly. The bound of the
    // advancement is no use here, so certify the rest of the segment with the
    // Lipschitz bound of the distance instead. The distance changes by at most
    // `scale` per unit time, so an interval [t0, t1] with distances d0 and d1
    // stays above 0.5 (d0 + d1 - scale (t1 - t0)). Intervals that fail the
    // test are halved until they are shorter than the resolution, where the
    // distance can dip at most half the resolution below the smaller end.
    // The distance may not fall below `floor`. It is the skin less the
    // rounding of the distance, which lets a robot at rest in the skin turn
    // without a false hit. If the floor followed the current distance, every
    // step could sink 2e-7 deeper, so it never goes below a quarter of the
    // skin: a robot closer than that to a shape may not get closer at all.
    // The floor must stay above zero, because the distance of overlapping
    // shapes is reported as zero.
    const double minGap = 0.25 * skin;
    const double minAllowed = st.dist >= minGap ?
        std::max(std::min(st.dist, skin) - 2e-7, minGap) : st.dist;
    const double resolution = 0.4 * skin;
    int budget = kMaxLooks;
    double lo = t;
    double hi = t;
    // Returns true if the distance falls below minAllowed in [_t0, _t1],
    // with lo and hi bracketing the first drop. The distance at _t0 is at
    // least minAllowed.
    std::function<bool(double, double, double, double)> search =
        [&](double _t0, double _d0, double _t1, double _d1)
    {
      const bool endsOk = _d1 >= minAllowed;
      const double span = scale * (_t1 - _t0);
      if (endsOk && 0.5 * (_d0 + _d1 - span) >= minAllowed)
        return false;
      if (span <= resolution || budget <= 0)
      {
        if (endsOk && span <= resolution)
          return false;
        // Either the end is too close, or the budget ran out. Stop at the
        // start, which is allowed: stopping early never passes through.
        lo = _t0;
        hi = endsOk ? _t0 : _t1;
        return true;
      }
      const double tm = 0.5 * (_t0 + _t1);
      --budget;
      const double dm = look(tm).dist;
      if (search(_t0, _d0, tm, dm))
        return true;
      // The left half is clear, so dm >= minAllowed.
      return search(tm, dm, _t1, _d1);
    };
    --budget;
    const bool bad = search(t, st.dist, segEnd, look(segEnd).dist);
    if (!bad)
    {
      if (next())
        continue;
      return result;
    }
    for (int k = 0; k < 24 && scale * (hi - lo) > 1e-6; ++k)
    {
      const double mid = 0.5 * (lo + hi);
      if (look(mid).dist >= minAllowed)
        lo = mid;
      else
        hi = mid;
    }
    fill(lo, look(lo));
    return result;
  }
  // Out of iterations. Stop where the advancement got to.
  if (st.valid)
    fill(t, st);
  return result;
}

}  // namespace

// ---------------------------------------------------------------------------
// The world.

namespace
{
/// \brief A model in the world.
struct Model
{
  Entity entity{kNullEntity};
  uint32_t slot{0};
  bool isStatic{false};
  std::vector<Shape> shapes;

  /// \brief Proxies in the static tree, one per shape.
  std::vector<int> staticProxies;

  /// \brief Pose of a movable model. Scenery is at the identity.
  Pose2 pose;
  double z{0.0};

  /// \brief Box around the shapes in the model frame, and the radius of the
  /// circle around the origin that covers them.
  double minX{0.0};
  double maxX{0.0};
  double minY{0.0};
  double maxY{0.0};
  double radius{0.0};

  /// \brief Proxy in the movable tree and its (fat) box.
  int proxy{-1};
  b2AABB fat{};

  /// \brief Index of the request in the current Move call, or -1.
  int request{-1};
};

b2AABB MakeAabb(double _minX, double _minY, double _maxX, double _maxY)
{
  b2AABB box;
  box.lowerBound = {static_cast<float>(_minX), static_cast<float>(_minY)};
  box.upperBound = {static_cast<float>(_maxX), static_cast<float>(_maxY)};
  return box;
}

bool Contains(const b2AABB &_outer, const b2AABB &_inner)
{
  return _outer.lowerBound.x <= _inner.lowerBound.x &&
      _outer.lowerBound.y <= _inner.lowerBound.y &&
      _inner.upperBound.x <= _outer.upperBound.x &&
      _inner.upperBound.y <= _outer.upperBound.y;
}

/// \brief Box of a movable model at a pose.
void ModelBox(const Model &_m, const Pose2 &_p, double &_x0, double &_y0,
    double &_x1, double &_y1)
{
  const double cx = 0.5 * (_m.minX + _m.maxX);
  const double cy = 0.5 * (_m.minY + _m.maxY);
  const double hx = 0.5 * (_m.maxX - _m.minX);
  const double hy = 0.5 * (_m.maxY - _m.minY);
  const double wx = _p.x + _p.c * cx - _p.s * cy;
  const double wy = _p.y + _p.s * cx + _p.c * cy;
  const double ex = std::abs(_p.c) * hx + std::abs(_p.s) * hy;
  const double ey = std::abs(_p.s) * hx + std::abs(_p.c) * hy;
  _x0 = wx - ex;
  _x1 = wx + ex;
  _y0 = wy - ey;
  _y1 = wy + ey;
}

bool Collect(int, uint64_t _userData, void *_context)
{
  static_cast<std::vector<uint64_t> *>(_context)->push_back(_userData);
  return true;
}

uint64_t Pack(uint32_t _slot, uint32_t _shape)
{
  return (static_cast<uint64_t>(_slot) << 16) | _shape;
}

/// \brief What a robot did in one Move call.
struct Mover
{
  /// \brief Forget the last call. The vectors keep their memory.
  void Reset()
  {
    this->model = nullptr;
    this->cap = 1.0;
    this->pathLen = 0.0;
    this->pathRot = 0.0;
    this->stationary = false;
    this->blocked = false;
    this->blockedBy = kNullEntity;
    this->blockedAt = kInf;
    this->cutTime = kInf;
    this->cutBy = kNullEntity;
    this->path.count = 1;
    this->path.time[0] = 0.0;
    this->staticCandidates.clear();
    this->obstacleCandidates.clear();
  }

  Model *model{nullptr};
  PlanarMotionRequest request;
  Pose2 start;
  Path path;
  double cap{1.0};
  double pathLen{0.0};
  double pathRot{0.0};
  bool stationary{false};
  bool blocked{false};
  Entity blockedBy{kNullEntity};

  /// \brief Step time of the first hit against scenery or a model that does
  /// not move.
  double blockedAt{kInf};

  /// \brief The earliest time another robot cut the path, and which one.
  double cutTime{kInf};
  Entity cutBy{kNullEntity};
  double x0{0.0};
  double y0{0.0};
  double x1{0.0};
  double y1{0.0};
  std::vector<uint64_t> staticCandidates;
  std::vector<uint64_t> obstacleCandidates;
};

/// \brief Contact sort order.
bool ContactLess(const PlanarContact &_a, const PlanarContact &_b)
{
  if (_a.modelA != _b.modelA)
    return _a.modelA < _b.modelA;
  if (_a.modelB != _b.modelB)
    return _a.modelB < _b.modelB;
  if (_a.collisionA != _b.collisionA)
    return _a.collisionA < _b.collisionA;
  if (_a.collisionB != _b.collisionB)
    return _a.collisionB < _b.collisionB;
  if (_a.point.X() < _b.point.X() || _b.point.X() < _a.point.X())
    return _a.point.X() < _b.point.X();
  return _a.point.Y() < _b.point.Y();
}
}  // namespace

/// \brief Private data of PlanarCollisionWorld.
class PlanarCollisionWorld::Implementation
{
  /// \brief Constructor.
  public: explicit Implementation(const PlanarCollisionOptions &_options)
      : options(_options)
  {
    this->options.contactSkin = std::max(this->options.contactSkin, 1e-6);
    this->options.contactMargin =
        std::max(this->options.contactMargin, this->options.contactSkin);
    this->options.maxSlideIterations =
        std::clamp(this->options.maxSlideIterations, 1, 8);
    this->options.maxRobotPasses =
        std::clamp(this->options.maxRobotPasses, 1, 16);
    this->staticTree = b2DynamicTree_Create();
    this->moverTree = b2DynamicTree_Create();
    this->settings.skin = this->options.contactSkin;
  }

  /// \brief Destructor.
  public: ~Implementation()
  {
    b2DynamicTree_Destroy(&this->staticTree);
    b2DynamicTree_Destroy(&this->moverTree);
  }

  public: Implementation(const Implementation &) = delete;
  public: Implementation &operator=(const Implementation &) = delete;

  /// \brief Remove a model and free its slot.
  public: void Remove(Entity _entity)
  {
    auto it = this->byEntity.find(_entity);
    if (it == this->byEntity.end())
      return;
    Model *model = it->second;
    for (const int proxy : model->staticProxies)
      b2DynamicTree_DestroyProxy(&this->staticTree, proxy);
    if (model->proxy >= 0)
      b2DynamicTree_DestroyProxy(&this->moverTree, model->proxy);
    this->staticDirty = this->staticDirty || model->isStatic;
    this->freeSlots.push_back(model->slot);
    this->slots[model->slot].reset();
    this->byEntity.erase(it);
  }

  /// \brief Take a slot for a new model.
  public: Model *Insert(Entity _entity, bool _static)
  {
    this->Remove(_entity);
    uint32_t slot;
    if (!this->freeSlots.empty())
    {
      slot = this->freeSlots.back();
      this->freeSlots.pop_back();
    }
    else
    {
      slot = static_cast<uint32_t>(this->slots.size());
      this->slots.emplace_back();
    }
    auto model = std::make_unique<Model>();
    model->entity = _entity;
    model->slot = slot;
    model->isStatic = _static;
    Model *raw = model.get();
    this->slots[slot] = std::move(model);
    this->byEntity[_entity] = raw;
    return raw;
  }

  public: bool AddStatic(Entity _entity,
      const std::vector<PlanarFootprint> &_footprints)
  {
    Model *model = this->Insert(_entity, true);
    for (auto fp : _footprints)
    {
      // A shape that only touches the band from below or above is outside it.
      const bool outside = fp.zMax <= this->options.bandMin ||
          fp.zMin >= this->options.bandMax;
      fp.zMin = std::max(fp.zMin, this->options.bandMin);
      fp.zMax = std::min(fp.zMax, this->options.bandMax);
      if (outside || fp.zMin > fp.zMax || fp.points.empty() ||
          model->shapes.size() >= kWholeModel)
      {
        continue;
      }
      fp.zMax = std::max(fp.zMax, fp.zMin + 2e-6);
      fp.zMin -= 1e-6;
      model->shapes.push_back(MakeShape(fp, true));
    }
    if (model->shapes.empty())
    {
      this->Remove(_entity);
      return false;
    }
    for (std::size_t i = 0; i < model->shapes.size(); ++i)
    {
      model->staticProxies.push_back(b2DynamicTree_CreateProxy(
          &this->staticTree, TightBox(model->shapes[i]), 1u,
          Pack(model->slot, static_cast<uint32_t>(i))));
    }
    this->staticDirty = true;
    return true;
  }

  /// \brief Axis-aligned box of a shape in the model frame, padded.
  public: static b2AABB TightBox(const Shape &_s)
  {
    // A tight box from the core points and the radius.
    const b2ShapeProxy &p = _s.proxy;
    Vec2 low(_s.origin.X() + p.points[0].x, _s.origin.Y() + p.points[0].y);
    Vec2 high = low;
    for (int i = 1; i < p.count; ++i)
    {
      const Vec2 v(_s.origin.X() + p.points[i].x,
          _s.origin.Y() + p.points[i].y);
      low.Set(std::min(low.X(), v.X()), std::min(low.Y(), v.Y()));
      high.Set(std::max(high.X(), v.X()), std::max(high.Y(), v.Y()));
    }
    return MakeAabb(low.X() - p.radius - kTreePad,
        low.Y() - p.radius - kTreePad, high.X() + p.radius + kTreePad,
        high.Y() + p.radius + kTreePad);
  }

  public: bool AddMovable(Entity _entity,
      const std::vector<PlanarFootprint> &_footprints,
      const math::Pose3d &_pose)
  {
    Model *model = this->Insert(_entity, false);
    model->minX = model->minY = kInf;
    model->maxX = model->maxY = -kInf;
    for (const auto &fp : _footprints)
    {
      if (fp.points.empty() || model->shapes.size() >= kWholeModel)
        continue;
      PlanarFootprint copy = fp;
      if (copy.zMax - copy.zMin < 2e-6)
      {
        copy.zMax += 1e-6;
        copy.zMin -= 1e-6;
      }
      model->shapes.push_back(MakeShape(copy, false));
      const Shape &s = model->shapes.back();
      const b2AABB box = TightBox(s);
      model->minX = std::min(model->minX,
          static_cast<double>(box.lowerBound.x) + kTreePad);
      model->minY = std::min(model->minY,
          static_cast<double>(box.lowerBound.y) + kTreePad);
      model->maxX = std::max(model->maxX,
          static_cast<double>(box.upperBound.x) - kTreePad);
      model->maxY = std::max(model->maxY,
          static_cast<double>(box.upperBound.y) - kTreePad);
      model->radius = std::max(model->radius,
          s.center.Length() + s.boundRadius);
    }
    if (model->shapes.empty())
    {
      this->Remove(_entity);
      return false;
    }
    model->pose = MakePose(_pose.Pos().X(), _pose.Pos().Y(),
        _pose.Rot().Yaw());
    model->z = _pose.Pos().Z();
    this->Refit(*model, true);
    return true;
  }

  /// \brief Make the fat box of a movable model cover _box. Creates the
  /// proxy if there is none.
  public: void EnsureFat(Model &_m, double _x0, double _y0, double _x1,
      double _y1, bool _force = false)
  {
    const b2AABB tight = MakeAabb(_x0 - kTreePad, _y0 - kTreePad,
        _x1 + kTreePad, _y1 + kTreePad);
    if (_m.proxy >= 0 && !_force && Contains(_m.fat, tight))
      return;
    _m.fat = MakeAabb(_x0 - kFatMargin, _y0 - kFatMargin, _x1 + kFatMargin,
        _y1 + kFatMargin);
    if (_m.proxy < 0)
    {
      _m.proxy = b2DynamicTree_CreateProxy(&this->moverTree, _m.fat, 1u,
          Pack(_m.slot, kWholeModel));
    }
    else
    {
      b2DynamicTree_MoveProxy(&this->moverTree, _m.proxy, _m.fat);
    }
  }

  /// \brief Fit the broadphase box to the current pose.
  public: void Refit(Model &_m, bool _force = false)
  {
    double x0, y0, x1, y1;
    ModelBox(_m, _m.pose, x0, y0, x1, y1);
    this->EnsureFat(_m, x0, y0, x1, y1, _force);
  }

  public: void RebuildStatic()
  {
    if (this->staticDirty)
    {
      if (b2DynamicTree_GetProxyCount(&this->staticTree) > 0)
        b2DynamicTree_Rebuild(&this->staticTree, true);
      this->staticDirty = false;
    }
  }

  /// \brief Do the height intervals of two shapes overlap inside the band?
  public: bool ZOverlap(const Model &_ma, const Shape &_a, const Model &_mb,
      const Shape &_b) const
  {
    const double lo = std::max({_a.zMin + _ma.z, _b.zMin + _mb.z,
        this->options.bandMin});
    const double hi = std::min({_a.zMax + _ma.z, _b.zMax + _mb.z,
        this->options.bandMax});
    return lo < hi;
  }

  /// \brief Can the motion of shape _a (a movable shape, on the path _pa)
  /// bring it within _margin of shape _b, which is fixed at _poseB or
  /// moves on _pb? A cheap test with circles around the shapes.
  public: static bool MayTouch(const Shape &_a, const Pose2 &_poseA,
      double _motionA, const Shape &_b, const Pose2 &_poseB,
      double _motionB, double _margin)
  {
    const double ax = _poseA.x + _poseA.c * _a.center.X() -
        _poseA.s * _a.center.Y();
    const double ay = _poseA.y + _poseA.s * _a.center.X() +
        _poseA.c * _a.center.Y();
    const double bx = _poseB.x + _poseB.c * _b.center.X() -
        _poseB.s * _b.center.Y();
    const double by = _poseB.y + _poseB.s * _b.center.X() +
        _poseB.c * _b.center.Y();
    const double reach = _a.boundRadius + _b.boundRadius + _margin +
        _motionA + _motionB;
    const double dx = ax - bx;
    const double dy = ay - by;
    return dx * dx + dy * dy <= reach * reach;
  }

  /// \brief Collect shapes and movable models near a box.
  public: void Query(double _x0, double _y0, double _x1, double _y1,
      std::vector<uint64_t> &_static, std::vector<uint64_t> &_movable) const
  {
    const b2AABB box = MakeAabb(_x0 - kTreePad, _y0 - kTreePad,
        _x1 + kTreePad, _y1 + kTreePad);
    _static.clear();
    _movable.clear();
    b2DynamicTree_Query(&this->staticTree, box, 1u, Collect, &_static);
    b2DynamicTree_Query(&this->moverTree, box, 1u, Collect, &_movable);
    std::sort(_static.begin(), _static.end());
    std::sort(_movable.begin(), _movable.end());
  }

  public: const Model *ModelAt(uint64_t _userData) const
  {
    return this->slots[_userData >> 16].get();
  }

  /// \brief First hit of a robot moving by _motion from _from against the
  /// fixed shapes in the candidate lists.
  public: PairHit SweepFixed(const Mover &_mover, const Pose2 &_from,
      double _dx, double _dy, double _dyaw, Entity &_blocker) const
  {
    const Model &a = *_mover.model;
    Path pa;
    pa.count = 2;
    pa.time[0] = 0.0;
    pa.time[1] = 1.0;
    pa.pose[0] = _from;
    pa.pose[1] = MakePose(_from.x + _dx, _from.y + _dy, _from.yaw + _dyaw);
    const double travel = std::hypot(_dx, _dy);
    const double turn = std::abs(_dyaw);
    const double margin = 2.0 * this->options.contactSkin;

    PairHit best;
    best.t = 1.0;
    Path pb;
    auto sweepModel = [&](const Model &_b, int _only)
    {
      pb.count = 1;
      pb.pose[0] = _b.pose;
      for (std::size_t j = 0; j < _b.shapes.size(); ++j)
      {
        if (_only >= 0 && static_cast<int>(j) != _only)
          continue;
        const Shape &sb = _b.shapes[j];
        for (const auto &sa : a.shapes)
        {
          if (!MayTouch(sa, _from, travel + turn * sa.center.Length(), sb,
                _b.pose, 0.0, margin) || !this->ZOverlap(a, sa, _b, sb))
          {
            continue;
          }
          const PairHit h = PairSweep(this->settings, sa, pa, 1.0, sb, pb,
              1.0, best.t);
          if (h.hit && h.t < best.t)
          {
            best = h;
            _blocker = _b.entity;
          }
        }
      }
    };
    for (const uint64_t u : _mover.staticCandidates)
      sweepModel(*this->ModelAt(u), static_cast<int>(u & 0xFFFF));
    for (const uint64_t u : _mover.obstacleCandidates)
      sweepModel(*this->ModelAt(u), -1);
    return best;
  }

  /// \brief Fill _contacts with the contacts of _a at its current pose.
  /// \param[in] _skipLowerRequests If true, other models with a request in
  /// this call and a lower entity id are left to their own turn.
  public: void ContactsFor(const Model &_a, bool _skipLowerRequests,
      std::vector<PlanarContact> &_contacts) const
  {
    double x0, y0, x1, y1;
    ModelBox(_a, _a.pose, x0, y0, x1, y1);
    const double m = this->options.contactMargin;
    std::vector<uint64_t> statics, movables;
    this->Query(x0 - m, y0 - m, x1 + m, y1 + m, statics, movables);

    const std::size_t first = _contacts.size();
    b2SimplexCache cache{};
    auto one = [&](const Model &_b, int _only)
    {
      if (&_b == &_a)
        return;
      if (_skipLowerRequests && _b.request >= 0 && _b.entity < _a.entity)
        return;
      for (std::size_t j = 0; j < _b.shapes.size(); ++j)
      {
        if (_only >= 0 && static_cast<int>(j) != _only)
          continue;
        const Shape &sb = _b.shapes[j];
        for (const auto &sa : _a.shapes)
        {
          if (!MayTouch(sa, _a.pose, 0.0, sb, _b.pose, 0.0, 2.0 * m) ||
              !this->ZOverlap(_a, sa, _b, sb))
          {
            continue;
          }
          const b2Transform xf = Relative(_a.pose, _b.pose, sb.origin);
          cache = b2SimplexCache{};
          const b2DistanceOutput d = DistanceAt(sa, sb, xf, cache);
          if (d.distance > m)
            continue;
          const b2Manifold man = Collide(sa, sb, xf);
          const double nx = _a.pose.c * man.normal.x -
              _a.pose.s * man.normal.y;
          const double ny = _a.pose.s * man.normal.x +
              _a.pose.c * man.normal.y;
          for (int k = 0; k < man.pointCount; ++k)
          {
            const double sep = man.points[k].separation;
            if (sep > m)
              continue;
            const b2Vec2 p = man.points[k].point;
            PlanarContact c;
            c.modelA = _a.entity;
            c.modelB = _b.entity;
            c.collisionA = sa.collision;
            c.collisionB = sb.collision;
            c.point = Vec2(_a.pose.x + _a.pose.c * p.x - _a.pose.s * p.y,
                _a.pose.y + _a.pose.s * p.x + _a.pose.c * p.y);
            c.normal = Vec2(nx, ny);
            c.separation = sep;
            c.depth = std::max(0.0, -sep);
            _contacts.push_back(c);
          }
        }
      }
    };
    for (const uint64_t u : statics)
      one(*this->ModelAt(u), static_cast<int>(u & 0xFFFF));
    for (const uint64_t u : movables)
      one(*this->ModelAt(u), -1);
    std::sort(_contacts.begin() + static_cast<std::ptrdiff_t>(first),
        _contacts.end(), ContactLess);
  }

  public: void Move(const std::vector<PlanarMotionRequest> &_requests,
      std::vector<PlanarMotionResult> &_results);

  public: PlanarCollisionOptions options;
  public: SweepSettings settings;
  public: b2DynamicTree staticTree;
  public: b2DynamicTree moverTree;
  public: bool staticDirty{false};
  public: std::vector<std::unique_ptr<Model>> slots;
  public: std::vector<uint32_t> freeSlots;
  public: std::unordered_map<Entity, Model *> byEntity;
  public: std::vector<PlanarContact> contacts;

  /// \brief Scratch memory kept between Move calls.
  public: std::vector<Mover> movers;
  public: std::vector<std::size_t> moverOf;
  public: std::vector<uint64_t> movable;
  public: std::vector<std::pair<int, int>> pairs;
  public: std::vector<double> caps;
  public: std::vector<double> newCaps;
  public: std::vector<std::pair<double, double>> pairCaps;
  public: std::vector<int> group;
  public: std::vector<int> groupSize;
};

void PlanarCollisionWorld::Implementation::Move(
    const std::vector<PlanarMotionRequest> &_requests,
    std::vector<PlanarMotionResult> &_results)
{
  _results.assign(_requests.size(), PlanarMotionResult());
  this->contacts.clear();
  this->RebuildStatic();

  const int maxSlide = this->options.maxSlideIterations;

  // Resolve the requests. The movers keep their memory between calls.
  std::size_t count = 0;
  this->moverOf.clear();
  for (std::size_t i = 0; i < _requests.size(); ++i)
  {
    const PlanarDelta &d = _requests[i].desired;
    auto it = this->byEntity.find(_requests[i].model);
    if (it == this->byEntity.end() || it->second->isStatic ||
        it->second->request >= 0 || !std::isfinite(d.x + d.y + d.yaw))
    {
      _results[i].known = false;
      continue;
    }
    Model *model = it->second;
    model->request = static_cast<int>(count);
    if (count == this->movers.size())
      this->movers.emplace_back();
    Mover &m = this->movers[count++];
    m.Reset();
    m.model = model;
    m.request = _requests[i];
    m.start = model->pose;
    this->moverOf.push_back(i);
  }

  // Paths, boxes and candidates. The box covers every path the sweep can
  // take: the straight path, the turn, and for Slide the bent paths, which
  // stay within the length of the desired translation.
  for (std::size_t k = 0; k < count; ++k)
  {
    Mover &m = this->movers[k];
    const PlanarDelta &d = m.request.desired;
    m.stationary = d.IsZero();
    const double len = std::hypot(d.x, d.y);
    double x0, y0, x1, y1;
    ModelBox(*m.model, m.start, x0, y0, x1, y1);
    const Pose2 end = MakePose(m.start.x + d.x, m.start.y + d.y,
        m.start.yaw + d.yaw);
    double ex0, ey0, ex1, ey1;
    ModelBox(*m.model, end, ex0, ey0, ex1, ey1);
    const double infl = m.model->radius * std::abs(d.yaw) +
        (m.request.response == PlanarResponse::Slide ? len : 0.0) +
        this->options.contactMargin;
    m.x0 = std::min(x0, ex0) - infl;
    m.y0 = std::min(y0, ey0) - infl;
    m.x1 = std::max(x1, ex1) + infl;
    m.y1 = std::max(y1, ey1) + infl;
    this->EnsureFat(*m.model, m.x0, m.y0, m.x1, m.y1);
    m.path.count = 1;
    m.path.time[0] = 0.0;
    m.path.pose[0] = m.start;
  }
  this->pairs.clear();
  for (std::size_t k = 0; k < count; ++k)
  {
    Mover &m = this->movers[k];
    if (m.stationary)
      continue;
    this->Query(m.x0, m.y0, m.x1, m.y1, m.staticCandidates, this->movable);
    for (const uint64_t u : this->movable)
    {
      const Model *other = this->ModelAt(u);
      if (other == m.model)
        continue;
      if (other->request < 0)
      {
        m.obstacleCandidates.push_back(u);
      }
      else
      {
        const int a = std::min(static_cast<int>(k), other->request);
        const int b = std::max(static_cast<int>(k), other->request);
        this->pairs.emplace_back(a, b);
      }
    }
  }
  std::sort(this->pairs.begin(), this->pairs.end());
  this->pairs.erase(std::unique(this->pairs.begin(), this->pairs.end()),
      this->pairs.end());

  // Pass 1: each robot against scenery and against models that do not move.
  for (std::size_t k = 0; k < count; ++k)
  {
    Mover &m = this->movers[k];
    if (m.stationary)
      continue;
    const PlanarDelta &d = m.request.desired;
    const bool slide = m.request.response == PlanarResponse::Slide;
    Pose2 cur = m.start;
    double remX = d.x, remY = d.y, remYaw = d.yaw;
    double time = 0.0;
    Vec2 planes[kMaxKnots];  // One per slide iteration; maxSlide is clamped.
    int planeCount = 0;
    for (int iter = 0; iter <= maxSlide; ++iter)
    {
      if (IsZero(remX) && IsZero(remY) && IsZero(remYaw))
        break;
      Entity blocker = kNullEntity;
      const PairHit h = this->SweepFixed(m, cur, remX, remY, remYaw, blocker);
      const double adv = h.hit ? h.t : 1.0;
      const double dur = (1.0 - time) * adv;
      const Pose2 next = MakePose(cur.x + adv * remX, cur.y + adv * remY,
          cur.yaw + adv * remYaw);
      if (dur > 0.0)
      {
        time += dur;
        m.path.time[m.path.count] = time;
        m.path.pose[m.path.count] = next;
        ++m.path.count;
      }
      cur = next;
      if (!h.hit)
        break;
      m.blocked = true;
      if (m.blockedBy == kNullEntity)
      {
        m.blockedBy = blocker;
        m.blockedAt = time;
      }
      if (!slide)
        break;
      // How far the fastest point of the robot got before the hit.
      const double reached = adv * (std::hypot(remX, remY) +
          std::abs(remYaw) * m.model->radius);
      remX *= 1.0 - adv;
      remY *= 1.0 - adv;
      remYaw *= 1.0 - adv;
      const Vec2 n(h.nx, h.ny);
      const double into = remX * n.X() + remY * n.Y();
      if (into > 0.0)
      {
        remX -= into * n.X();
        remY -= into * n.Y();
      }
      bool stuck = false;
      for (int p = 0; p < planeCount; ++p)
      {
        if (remX * planes[p].X() + remY * planes[p].Y() >
            kApproachEps * std::hypot(remX, remY))
        {
          stuck = true;
        }
      }
      if (stuck)
      {
        remX = remY = 0.0;
      }
      planes[planeCount++] = n;
      // A turn that gets nowhere runs into the obstacle. Drop it, or it
      // would use up the iterations in steps smaller than the skin.
      if (reached < 0.5 * this->settings.skin)
        remYaw = 0.0;
      if (std::hypot(remX, remY) < 1e-12 && IsZero(remYaw))
        break;
    }
    if (m.path.time[m.path.count - 1] < 1.0)
    {
      m.path.time[m.path.count] = 1.0;
      m.path.pose[m.path.count] = cur;
      ++m.path.count;
    }
    // Length and turn of the path, to bound how far any point can move.
    for (int s = 0; s + 1 < m.path.count; ++s)
    {
      m.pathLen += std::hypot(m.path.pose[s + 1].x - m.path.pose[s].x,
          m.path.pose[s + 1].y - m.path.pose[s].y);
      m.pathRot += std::abs(m.path.pose[s + 1].yaw - m.path.pose[s].yaw);
    }
  }

  // Pass 2: robots against each other, in Jacobi passes. Every pass reads the
  // caps of the previous pass.
  this->caps.assign(count, 1.0);
  this->pairCaps.assign(this->pairs.size(), {-1.0, -1.0});
  const double margin = 2.0 * this->options.contactSkin;
  auto pairPass = [&](bool _freeze) {
    this->newCaps = this->caps;
    bool changed = false;
    for (std::size_t p = 0; p < this->pairs.size(); ++p)
    {
      const auto &pr = this->pairs[p];
      // The lower entity id is A. The roles decide the frame the queries
      // run in, and the order of the requests must not change the rounding.
      std::size_t ia = static_cast<std::size_t>(pr.first);
      std::size_t ib = static_cast<std::size_t>(pr.second);
      if (this->movers[ib].model->entity < this->movers[ia].model->entity)
        std::swap(ia, ib);
      Mover &ma = this->movers[ia];
      Mover &mb = this->movers[ib];
      // The sweep of a pair depends on the two caps only. If they are the
      // ones of the last sweep, its cut is already in the caps.
      const auto &last = this->pairCaps[p];
      if (!(last.first < this->caps[ia]) && !(this->caps[ia] < last.first) &&
          !(last.second < this->caps[ib]) && !(this->caps[ib] < last.second))
      {
        continue;
      }
      this->pairCaps[p] = {this->caps[ia], this->caps[ib]};
      PairHit best;
      best.t = 1.0;
      for (const auto &sa : ma.model->shapes)
      {
        for (const auto &sb : mb.model->shapes)
        {
          if (!this->ZOverlap(*ma.model, sa, *mb.model, sb))
            continue;
          const double motionA = ma.pathLen + ma.pathRot * sa.center.Length();
          const double motionB = mb.pathLen + mb.pathRot * sb.center.Length();
          if (!MayTouch(sa, ma.start, motionA, sb, mb.start, motionB, margin))
            continue;
          const PairHit h = PairSweep(this->settings, sa, ma.path,
              this->caps[ia], sb, mb.path, this->caps[ib], best.t);
          if (h.hit && h.t < best.t)
            best = h;
        }
      }
      if (!best.hit)
        continue;
      const double eps = kApproachEps *
          std::max({std::abs(best.approachA), std::abs(best.approachB),
              1e-12});
      const double cut = _freeze ? 0.0 : best.t;
      // The earliest cut names the blocker, the lower entity id on a tie, so
      // that the answer does not depend on the order of the pairs.
      auto cutBy = [&](Mover &_m, std::size_t _i, Entity _other)
      {
        this->newCaps[_i] = std::min(this->newCaps[_i], cut);
        _m.blocked = true;
        if (cut < _m.cutTime || (!(_m.cutTime < cut) && _other < _m.cutBy))
        {
          _m.cutTime = cut;
          _m.cutBy = _other;
        }
      };
      if (best.approachA > eps)
        cutBy(ma, ia, mb.model->entity);
      if (best.approachB > eps)
        cutBy(mb, ib, ma.model->entity);
    }
    for (std::size_t i = 0; i < count; ++i)
    {
      if (this->newCaps[i] < this->caps[i])
        changed = true;
    }
    this->caps = this->newCaps;
    return changed;
  };
  if (!this->pairs.empty())
  {
    // A queue of robots at the same speed needs one pass per robot to settle:
    // robot k is cut only after robot k-1's cap is known. So allow as many
    // passes as the largest group of robots that are linked by pairs.
    int passes = this->options.maxRobotPasses;
    this->group.assign(count, 0);
    std::iota(this->group.begin(), this->group.end(), 0);
    std::function<int(int)> root = [&](int _i)
    {
      while (this->group[_i] != _i)
      {
        this->group[_i] = this->group[this->group[_i]];
        _i = this->group[_i];
      }
      return _i;
    };
    for (const auto &pr : this->pairs)
      this->group[root(pr.first)] = root(pr.second);
    this->groupSize.assign(count, 0);
    for (std::size_t i = 0; i < count; ++i)
    {
      const int r = root(static_cast<int>(i));
      passes = std::max(passes, ++this->groupSize[r]);
    }
    passes = std::min(passes, kMaxRobotPassesHard);

    bool settled = false;
    for (int pass = 0; pass < passes; ++pass)
    {
      if (!pairPass(false))
      {
        settled = true;
        break;
      }
    }
    if (!settled)
    {
      // Robots still in conflict stay where they were. A robot that is
      // frozen can put the one behind it in conflict, so repeat until nothing
      // changes. Each round freezes at least one more robot, so this ends
      // within `count` rounds, and no pair that approaches is left with a
      // moving robot.
      this->pairCaps.assign(this->pairs.size(), {-1.0, -1.0});
      while (pairPass(true)) {}
    }
  }

  // Write back.
  for (std::size_t k = 0; k < count; ++k)
  {
    Mover &m = this->movers[k];
    PlanarMotionResult &r = _results[this->moverOf[k]];
    if (!m.stationary)
    {
      Model &model = *m.model;
      if (m.blocked)
      {
        const Sample end = Evaluate(m.path, this->caps[k], 1.0);
        model.pose = end.pose;
        r.achieved.x = end.pose.x - m.start.x;
        r.achieved.y = end.pose.y - m.start.y;
        r.achieved.yaw = end.pose.yaw - m.start.yaw;
      }
      else
      {
        // Free motion is exactly the desired one, with no rounding from
        // adding and subtracting the pose.
        const PlanarDelta &d = m.request.desired;
        r.achieved = d;
        model.pose = MakePose(m.start.x + d.x, m.start.y + d.y,
            m.start.yaw + d.yaw);
      }
      this->Refit(model);
    }
    r.blocked = m.blocked;
    // The model that stopped the robot first. A tie goes to the scenery.
    r.blockedBy = (m.blockedBy != kNullEntity && m.blockedAt <= m.cutTime) ?
        m.blockedBy : m.cutBy;
  }

  if (this->options.reportContacts)
  {
    for (std::size_t k = 0; k < count; ++k)
      this->ContactsFor(*this->movers[k].model, true, this->contacts);
    // The blocks come in request order. Sort them all, so that the list does
    // not depend on it.
    std::sort(this->contacts.begin(), this->contacts.end(), ContactLess);
  }
  for (std::size_t k = 0; k < count; ++k)
    this->movers[k].model->request = -1;
}

PlanarCollisionWorld::PlanarCollisionWorld(
    const PlanarCollisionOptions &_options)
  : dataPtr(utils::MakeUniqueImpl<Implementation>(_options))
{
}

PlanarCollisionWorld::~PlanarCollisionWorld() = default;

bool PlanarCollisionWorld::AddStaticModel(const EntityComponentManager &_ecm,
    Entity _model)
{
  return this->dataPtr->AddStatic(_model, ModelFootprints(_ecm, _model, true));
}

bool PlanarCollisionWorld::AddMovableModel(const EntityComponentManager &_ecm,
    Entity _model)
{
  return this->dataPtr->AddMovable(_model,
      ModelFootprints(_ecm, _model, false), worldPose(_model, _ecm));
}

bool PlanarCollisionWorld::AddStaticFootprints(Entity _model,
    const std::vector<PlanarFootprint> &_footprints)
{
  return this->dataPtr->AddStatic(_model, _footprints);
}

bool PlanarCollisionWorld::AddMovableFootprints(Entity _model,
    const std::vector<PlanarFootprint> &_footprints,
    const math::Pose3d &_pose)
{
  return this->dataPtr->AddMovable(_model, _footprints, _pose);
}

void PlanarCollisionWorld::RemoveModel(Entity _model)
{
  this->dataPtr->Remove(_model);
}

bool PlanarCollisionWorld::HasModel(Entity _model) const
{
  return this->dataPtr->byEntity.count(_model) > 0u;
}

bool PlanarCollisionWorld::IsMovable(Entity _model) const
{
  auto it = this->dataPtr->byEntity.find(_model);
  return it != this->dataPtr->byEntity.end() && !it->second->isStatic;
}

std::size_t PlanarCollisionWorld::ShapeCount(Entity _model) const
{
  auto it = this->dataPtr->byEntity.find(_model);
  return it == this->dataPtr->byEntity.end() ? 0u :
      it->second->shapes.size();
}

std::size_t PlanarCollisionWorld::ModelCount() const
{
  return this->dataPtr->byEntity.size();
}

void PlanarCollisionWorld::SetPose(Entity _model, const math::Pose3d &_pose)
{
  auto it = this->dataPtr->byEntity.find(_model);
  if (it == this->dataPtr->byEntity.end() || it->second->isStatic)
    return;
  Model &m = *it->second;
  m.pose = MakePose(_pose.Pos().X(), _pose.Pos().Y(), _pose.Rot().Yaw());
  m.z = _pose.Pos().Z();
  this->dataPtr->Refit(m);
}

math::Pose3d PlanarCollisionWorld::Pose(Entity _model) const
{
  auto it = this->dataPtr->byEntity.find(_model);
  if (it == this->dataPtr->byEntity.end() || it->second->isStatic)
    return math::Pose3d::Zero;
  const Model &m = *it->second;
  return math::Pose3d(m.pose.x, m.pose.y, m.z, 0.0, 0.0, m.pose.yaw);
}

void PlanarCollisionWorld::Move(
    const std::vector<PlanarMotionRequest> &_requests,
    std::vector<PlanarMotionResult> &_results)
{
  this->dataPtr->Move(_requests, _results);
}

const std::vector<PlanarContact> &PlanarCollisionWorld::Contacts() const
{
  return this->dataPtr->contacts;
}

void PlanarCollisionWorld::ContactsOf(Entity _model,
    std::vector<PlanarContact> &_contacts) const
{
  _contacts.clear();
  auto it = this->dataPtr->byEntity.find(_model);
  if (it == this->dataPtr->byEntity.end() || it->second->isStatic)
    return;
  this->dataPtr->ContactsFor(*it->second, false, _contacts);
}
}
}
}
