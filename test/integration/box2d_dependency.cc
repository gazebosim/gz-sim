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

// Checks that Box2D v3 builds and links in the gz-sim tree, whether it comes
// from the vendored copy or from a system package (see GZ_SIM_BOX2D).

#include <gtest/gtest.h>

#include <box2d/box2d.h>

/////////////////////////////////////////////////
TEST(Box2DDependency, Version)
{
  const b2Version version = b2GetVersion();
  EXPECT_EQ(3, version.major);
  EXPECT_EQ(1, version.minor);
}

/////////////////////////////////////////////////
TEST(Box2DDependency, CollidePolygonsOverlapping)
{
  const b2Polygon boxA = b2MakeBox(0.5f, 0.5f);
  const b2Polygon boxB = b2MakeBox(0.5f, 0.5f);

  // Two unit boxes whose centers are 0.9 m apart along x overlap by 0.1 m.
  b2Transform xfB = b2Transform_identity;
  xfB.p = {0.9f, 0.0f};

  const b2Manifold manifold =
      b2CollidePolygons(&boxA, b2Transform_identity, &boxB, xfB);

  ASSERT_GT(manifold.pointCount, 0);
  EXPECT_NEAR(1.0f, manifold.normal.x, 1e-4f);
  EXPECT_NEAR(0.0f, manifold.normal.y, 1e-4f);
  for (int i = 0; i < manifold.pointCount; ++i)
  {
    EXPECT_NEAR(-0.1f, manifold.points[i].separation, 1e-3f);
  }
}

/////////////////////////////////////////////////
TEST(Box2DDependency, CollidePolygonsSeparated)
{
  const b2Polygon boxA = b2MakeBox(0.5f, 0.5f);
  const b2Polygon boxB = b2MakeBox(0.5f, 0.5f);

  b2Transform xfB = b2Transform_identity;
  xfB.p = {3.0f, 0.0f};

  const b2Manifold manifold =
      b2CollidePolygons(&boxA, b2Transform_identity, &boxB, xfB);
  EXPECT_EQ(0, manifold.pointCount);
}

/////////////////////////////////////////////////
TEST(Box2DDependency, ShapeDistance)
{
  const b2Polygon box = b2MakeBox(0.5f, 0.5f);

  b2DistanceInput input = {};
  input.proxyA = b2MakeProxy(box.vertices, box.count, box.radius);
  input.proxyB = b2MakeProxy(box.vertices, box.count, box.radius);
  input.transformA = b2Transform_identity;
  input.transformB = b2Transform_identity;
  // Box faces are at x = 0.5 and x = 2.5, so the gap is 2 m.
  input.transformB.p = {3.0f, 0.0f};
  input.useRadii = false;

  b2SimplexCache cache = {};
  const b2DistanceOutput output = b2ShapeDistance(&input, &cache, nullptr, 0);

  EXPECT_NEAR(2.0f, output.distance, 1e-4f);
  EXPECT_NEAR(0.5f, output.pointA.x, 1e-4f);
  EXPECT_NEAR(2.5f, output.pointB.x, 1e-4f);
}

/////////////////////////////////////////////////
TEST(Box2DDependency, ShapeCast)
{
  const b2Polygon box = b2MakeBox(0.5f, 0.5f);

  b2ShapeCastPairInput input = {};
  input.proxyA = b2MakeProxy(box.vertices, box.count, box.radius);
  input.proxyB = b2MakeProxy(box.vertices, box.count, box.radius);
  input.transformA = b2Transform_identity;
  input.transformB = b2Transform_identity;
  input.transformB.p = {3.0f, 0.0f};
  // Move B toward A by 4 m. The 2 m gap closes at half the translation, up to
  // Box2D's linear slop (5 mm).
  input.translationB = {-4.0f, 0.0f};
  input.maxFraction = 1.0f;
  input.canEncroach = false;

  const b2CastOutput output = b2ShapeCast(&input);

  ASSERT_TRUE(output.hit);
  EXPECT_NEAR(0.5f, output.fraction, 5e-3f);
  EXPECT_NEAR(1.0f, output.normal.x, 1e-3f);
}

/////////////////////////////////////////////////
TEST(Box2DDependency, WorldStep)
{
  b2WorldDef worldDef = b2DefaultWorldDef();
  worldDef.gravity = {0.0f, 0.0f};
  const b2WorldId world = b2CreateWorld(&worldDef);
  ASSERT_TRUE(b2World_IsValid(world));

  b2BodyDef bodyDef = b2DefaultBodyDef();
  bodyDef.type = b2_dynamicBody;
  bodyDef.linearVelocity = {1.0f, 0.0f};
  const b2BodyId body = b2CreateBody(world, &bodyDef);

  const b2Polygon box = b2MakeBox(0.5f, 0.5f);
  const b2ShapeDef shapeDef = b2DefaultShapeDef();
  b2CreatePolygonShape(body, &shapeDef, &box);

  // One second at 100 Hz with no forces and no damping.
  for (int i = 0; i < 100; ++i)
    b2World_Step(world, 0.01f, 4);

  const b2Vec2 position = b2Body_GetPosition(body);
  EXPECT_NEAR(1.0f, position.x, 1e-2f);
  EXPECT_NEAR(0.0f, position.y, 1e-4f);

  b2DestroyWorld(world);
}
