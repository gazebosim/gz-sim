/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <gtest/gtest.h>

#include "gz/math/Box.hh"

using namespace gz;

/////////////////////////////////////////////////
TEST(BoxTest, Constructor)
{
  // Default constructor
  {
    math::Boxd box;
    EXPECT_EQ(math::Vector3d::Zero, box.Size());
    EXPECT_EQ(math::Material(), box.Material());

    math::Boxd box2;
    EXPECT_EQ(box, box2);
  }

  // Individual dimension constructor
  {
    math::Boxd box(1.0, 2.0, 3.0);
    EXPECT_EQ(math::Vector3d(1.0, 2.0, 3.0), box.Size());
    EXPECT_EQ(math::Material(), box.Material());

    math::Boxd box2(1.0, 2.0, 3.0);
    EXPECT_EQ(box, box2);
  }

  // Vector dimension constructor
  {
    math::Boxd box(math::Vector3d(1.3, 2.5, 4.6));
    EXPECT_EQ(math::Vector3d(1.3, 2.5, 4.6), box.Size());
    EXPECT_EQ(math::Material(), box.Material());

    math::Boxd box2(math::Vector3d(1.3, 2.5, 4.6));
    EXPECT_EQ(box, box2);
  }

  // Dimension and mat constructor
  {
    math::Boxd box(1.0, 2.0, 5.0,
        math::Material(math::MaterialType::WOOD));
    EXPECT_EQ(math::Vector3d(1.0, 2.0, 5.0), box.Size());
    EXPECT_EQ(math::Material(math::MaterialType::WOOD), box.Material());

    math::Boxd box2(1.0, 2.0, 5.0,
        math::Material(math::MaterialType::WOOD));
    EXPECT_EQ(box, box2);
  }

  // Vector Dimension and mat constructor
  {
    math::Boxd box(math::Vector3d(2.2, 2.0, 10.0),
        math::Material(math::MaterialType::WOOD));
    EXPECT_EQ(math::Vector3d(2.2, 2.0, 10.0), box.Size());
    EXPECT_EQ(math::Material(math::MaterialType::WOOD), box.Material());

    math::Boxd box2(math::Vector3d(2.2, 2.0, 10.0),
        math::Material(math::MaterialType::WOOD));
    EXPECT_EQ(box, box2);
  }

  // Copy Constructor
  {
    math::Boxd box(math::Vector3d(2.2, 2.0, 10.0),
        math::Material(math::MaterialType::WOOD));
    math::Boxd box2(box);
    EXPECT_EQ(math::Vector3d(2.2, 2.0, 10.0), box2.Size());
    EXPECT_EQ(math::Material(math::MaterialType::WOOD), box2.Material());
  }

  // Move Constructor
  {
    math::Boxd box(math::Vector3d(2.2, 2.0, 10.0),
        math::Material(math::MaterialType::WOOD));
    math::Boxd box2(std::move(box));
    EXPECT_EQ(math::Vector3d(2.2, 2.0, 10.0), box2.Size());
    EXPECT_EQ(math::Material(math::MaterialType::WOOD), box2.Material());
  }
}

//////////////////////////////////////////////////
TEST(BoxTest, Mutators)
{
  math::Boxd box;
  box.SetSize(100.1, 2.3, 5.6);
  box.SetMaterial(math::Material(math::MaterialType::PINE));

  EXPECT_DOUBLE_EQ(100.1, box.Size().X());
  EXPECT_DOUBLE_EQ(2.3, box.Size().Y());
  EXPECT_DOUBLE_EQ(5.6, box.Size().Z());
  EXPECT_EQ(math::Material(math::MaterialType::PINE), box.Material());

  box.SetSize(math::Vector3d(3.4, 1.2, 0.5));
  EXPECT_DOUBLE_EQ(3.4, box.Size().X());
  EXPECT_DOUBLE_EQ(1.2, box.Size().Y());
  EXPECT_DOUBLE_EQ(0.5, box.Size().Z());
}

//////////////////////////////////////////////////
TEST(BoxTest, VolumeAndDensity)
{
  double mass = 1.0;
  math::Boxd box(1.0, 0.1, 10.4);
  double expectedVolume = 1.0 * 0.1 * 10.4;
  EXPECT_DOUBLE_EQ(expectedVolume, box.Volume());

  double expectedDensity = mass / expectedVolume;
  EXPECT_DOUBLE_EQ(expectedDensity, box.DensityFromMass(mass));

  // Bad density
  math::Boxd box2;
  EXPECT_GT(0.0, box2.DensityFromMass(mass));
}

//////////////////////////////////////////////////
TEST(BoxTest, Intersections)
{
  // No intersections
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(0.0, 0.0, 1.0), -5.0);
    EXPECT_EQ(box.Intersections(plane).size(), 0UL);
  }

  // Plane crosses 4 edges
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(0.0, 0.0, 1.0), 0);

    auto intersections = box.Intersections(plane);
    ASSERT_EQ(4UL, intersections.size());
    EXPECT_EQ(intersections.count(math::Vector3d(-1.0, -1.0, 0.0)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(-1.0, 1.0, 0.0)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(1.0, -1.0, 0.0)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(1.0, 1.0, 0.0)), 1UL);
  }

  // Plane coincides with box's face
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(0.0, 0.0, 1.0), 1.0);

    auto intersections = box.Intersections(plane);
    ASSERT_EQ(4UL, intersections.size());
    EXPECT_EQ(intersections.count(math::Vector3d(-1.0, -1.0, 1.0)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(-1.0, 1.0, 1.0)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(1.0, -1.0, 1.0)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(1.0, 1.0, 1.0)), 1UL);
  }

  // 3 intersections
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(1.0, 1.0, 1.0), 1.0);

    auto intersections = box.Intersections(plane);
    ASSERT_EQ(3UL, intersections.size());
    EXPECT_EQ(intersections.count(math::Vector3d(1.0, -1.0, 1.0)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(-1.0, 1.0, 1.0)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(1.0, 1.0, -1.0)), 1UL);
  }

  // 6 intersections
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(1.0, 1.0, 1.0), 0.5);

    auto intersections = box.Intersections(plane);
    ASSERT_EQ(6UL, intersections.size());
    EXPECT_EQ(intersections.count(math::Vector3d(-1.0, 1.0, 0.5)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(-1.0, 0.5, 1.0)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(1.0, -1.0, 0.5)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(0.5, -1.0, 1.0)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(1.0, 0.5, -1.0)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(0.5, 1.0, -1.0)), 1UL);
  }

  // 5 intersections
  // This is the plane above tilted further up
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(1.0, 1.0, 2.0), 0.5);

    auto intersections = box.Intersections(plane);
    ASSERT_EQ(5UL, intersections.size());
    EXPECT_EQ(intersections.count(math::Vector3d(-1.0, 1.0, 0.25)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(-1.0, -0.5, 1.0)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(1.0, -1.0, 0.25)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(-0.5, -1.0, 1.0)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(1.0, 1.0, -0.75)), 1UL);
  }
}

//////////////////////////////////////////////////
TEST(BoxTest, VolumeBelow)
{
  // Fully above
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(0.0, 0.0, 1.0), -5.0);
    EXPECT_DOUBLE_EQ(0.0, box.VolumeBelow(plane));
  }
  // Fully below
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(0.0, 0.0, 1.0), 20.0);
    EXPECT_DOUBLE_EQ(box.Volume(), box.VolumeBelow(plane));
  }
  // Fully below (because plane is rotated down)
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(0.0, 0.0, -1.0), 20.0);
    EXPECT_DOUBLE_EQ(box.Volume(), box.VolumeBelow(plane));
  }
  // Cut in half
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(0, 0, 1.0), 0);

    EXPECT_DOUBLE_EQ(box.Volume()/2, box.VolumeBelow(plane));
  }
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(0, 1, 0), 0);

    EXPECT_DOUBLE_EQ(box.Volume()/2, box.VolumeBelow(plane));
  }
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(-1, 0, 0), 0);

    EXPECT_DOUBLE_EQ(box.Volume()/2, box.VolumeBelow(plane));
  }
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(-1, -1, 0), 0);

    EXPECT_DOUBLE_EQ(box.Volume()/2, box.VolumeBelow(plane));
  }
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(0, 1, 1), 0);

    EXPECT_DOUBLE_EQ(box.Volume()/2, box.VolumeBelow(plane));
  }
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(1, 1, 1), 0);

    EXPECT_DOUBLE_EQ(box.Volume()/2, box.VolumeBelow(plane));
  }
  // Cut in 3/4
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(0, 0, 1.0), 0.5);

    EXPECT_DOUBLE_EQ(3*box.Volume()/4, box.VolumeBelow(plane));
  }
  // Opposites add to the total volume
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed planeA(math::Vector3d(0, 0, 1.0), 0.5);
    math::Planed planeB(math::Vector3d(0, 0, 1.0), -0.5);

    EXPECT_DOUBLE_EQ(box.Volume(),
        box.VolumeBelow(planeA) + box.VolumeBelow(planeB));
  }
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed planeA(math::Vector3d(0, 1.0, 1.0), 0.5);
    math::Planed planeB(math::Vector3d(0, 1.0, 1.0), -0.5);

    EXPECT_DOUBLE_EQ(box.Volume(),
        box.VolumeBelow(planeA) + box.VolumeBelow(planeB));
  }
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed planeA(math::Vector3d(-1, 1.0, 1.0), 0.5);
    math::Planed planeB(math::Vector3d(-1, 1.0, 1.0), -0.5);

    EXPECT_DOUBLE_EQ(box.Volume(),
        box.VolumeBelow(planeA) + box.VolumeBelow(planeB));
  }
}

//////////////////////////////////////////////////
TEST(BoxTest, CenterOfVolumeBelow)
{
  // Fully above
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(0.0, 0.0, 1.0), -5.0);
    EXPECT_FALSE(box.CenterOfVolumeBelow(plane).has_value());
  }

  // Fully below
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(0.0, 0.0, 1.0), 5.0);
    EXPECT_TRUE(box.CenterOfVolumeBelow(plane).has_value());
    EXPECT_EQ(box.CenterOfVolumeBelow(plane), math::Vector3d(0, 0, 0));
  }

  // Cut in half
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(0.0, 0.0, 1.0), 0);
    EXPECT_TRUE(box.CenterOfVolumeBelow(plane).has_value());
    EXPECT_EQ(box.CenterOfVolumeBelow(plane).value(),
      math::Vector3d(0, 0, -0.5));
  }

  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(0.0, 0.0, -1.0), 0);
    EXPECT_TRUE(box.CenterOfVolumeBelow(plane).has_value());
    EXPECT_EQ(box.CenterOfVolumeBelow(plane).value(),
      math::Vector3d(0, 0, 0.5));
  }
}

//////////////////////////////////////////////////
TEST(BoxTest, VerticesBelow)
{
  math::Boxd box(2.0, 2.0, 2.0);
  auto size = box.Size();

  math::Vector3d pXpYpZ{ size.X()/2,  size.Y()/2,  size.Z()/2};
  math::Vector3d nXpYpZ{-size.X()/2,  size.Y()/2,  size.Z()/2};
  math::Vector3d pXnYpZ{ size.X()/2, -size.Y()/2,  size.Z()/2};
  math::Vector3d nXnYpZ{-size.X()/2, -size.Y()/2,  size.Z()/2};
  math::Vector3d pXpYnZ{ size.X()/2,  size.Y()/2, -size.Z()/2};
  math::Vector3d nXpYnZ{-size.X()/2,  size.Y()/2, -size.Z()/2};
  math::Vector3d pXnYnZ{ size.X()/2, -size.Y()/2, -size.Z()/2};
  math::Vector3d nXnYnZ{-size.X()/2, -size.Y()/2, -size.Z()/2};

  // Fully above
  {
    math::Planed plane(math::Vector3d(0.0, 0.0, 1.0), -5.0);
    EXPECT_TRUE(box.VerticesBelow(plane).empty());
  }
  // Fully below
  {
    math::Planed plane(math::Vector3d(0.0, 0.0, 1.0), 20.0);
    EXPECT_EQ(8u, box.VerticesBelow(plane).size());
  }
  // Fully below (because plane is rotated down)
  {
    math::Planed plane(math::Vector3d(0.0, 0.0, -1.0), 20.0);
    EXPECT_EQ(8u, box.VerticesBelow(plane).size());
  }
  // 4 vertices
  {
    math::Planed plane(math::Vector3d(0, 0, 1.0), 0);

    auto vertices = box.VerticesBelow(plane);
    ASSERT_EQ(4u, vertices.size());

    EXPECT_EQ(vertices.count(nXnYnZ), 1UL);
    EXPECT_EQ(vertices.count(nXpYnZ), 1UL);
    EXPECT_EQ(vertices.count(pXnYnZ), 1UL);
    EXPECT_EQ(vertices.count(pXpYnZ), 1UL);
  }
  {
    math::Planed plane(math::Vector3d(0, 1, 0), 0.5);

    auto vertices = box.VerticesBelow(plane);
    ASSERT_EQ(4u, vertices.size());

    EXPECT_EQ(vertices.count(nXnYnZ), 1UL);
    EXPECT_EQ(vertices.count(nXnYpZ), 1UL);
    EXPECT_EQ(vertices.count(pXnYnZ), 1UL);
    EXPECT_EQ(vertices.count(pXnYpZ), 1UL);
  }
  {
    math::Planed plane(math::Vector3d(-1, 0, 0), -0.5);

    auto vertices = box.VerticesBelow(plane);
    ASSERT_EQ(4u, vertices.size());

    EXPECT_EQ(vertices.count(pXnYnZ), 1UL);
    EXPECT_EQ(vertices.count(pXnYpZ), 1UL);
    EXPECT_EQ(vertices.count(pXpYnZ), 1UL);
    EXPECT_EQ(vertices.count(pXpYpZ), 1UL);
  }
  {
    math::Planed plane(math::Vector3d(1, 1, 1), 0.0);

    auto vertices = box.VerticesBelow(plane);
    ASSERT_EQ(4u, vertices.size());

    EXPECT_EQ(vertices.count(nXnYnZ), 1UL);
    EXPECT_EQ(vertices.count(nXnYpZ), 1UL);
    EXPECT_EQ(vertices.count(nXpYnZ), 1UL);
    EXPECT_EQ(vertices.count(pXnYnZ), 1UL);
  }
  // 6 vertices
  {
    math::Planed plane(math::Vector3d(-1, -1, 0), 0.3);

    auto vertices = box.VerticesBelow(plane);
    ASSERT_EQ(6u, vertices.size());

    EXPECT_EQ(vertices.count(nXpYnZ), 1UL);
    EXPECT_EQ(vertices.count(nXpYpZ), 1UL);
    EXPECT_EQ(vertices.count(pXnYnZ), 1UL);
    EXPECT_EQ(vertices.count(pXnYpZ), 1UL);
    EXPECT_EQ(vertices.count(pXpYnZ), 1UL);
    EXPECT_EQ(vertices.count(pXpYpZ), 1UL);
  }
  {
    math::Planed plane(math::Vector3d(0, 1, 1), 0.9);

    auto vertices = box.VerticesBelow(plane);
    ASSERT_EQ(6u, vertices.size());

    EXPECT_EQ(vertices.count(nXnYnZ), 1UL);
    EXPECT_EQ(vertices.count(nXnYpZ), 1UL);
    EXPECT_EQ(vertices.count(pXnYpZ), 1UL);
    EXPECT_EQ(vertices.count(nXpYnZ), 1UL);
    EXPECT_EQ(vertices.count(pXnYnZ), 1UL);
    EXPECT_EQ(vertices.count(pXpYnZ), 1UL);
  }
  // 2 vertices
  {
    math::Planed plane(math::Vector3d(-1, -1, 0), -0.5);

    auto vertices = box.VerticesBelow(plane);
    ASSERT_EQ(2u, vertices.size());

    EXPECT_EQ(vertices.count(pXpYnZ), 1UL);
    EXPECT_EQ(vertices.count(pXpYpZ), 1UL);
  }
  // 7 vertices
  {
    math::Planed plane(math::Vector3d(1, 1, 1), 1.0);

    auto vertices = box.VerticesBelow(plane);
    ASSERT_EQ(7u, vertices.size());

    EXPECT_EQ(vertices.count(nXnYnZ), 1UL);
    EXPECT_EQ(vertices.count(nXnYpZ), 1UL);
    EXPECT_EQ(vertices.count(pXnYpZ), 1UL);
    EXPECT_EQ(vertices.count(nXpYnZ), 1UL);
    EXPECT_EQ(vertices.count(nXpYpZ), 1UL);
    EXPECT_EQ(vertices.count(pXnYnZ), 1UL);
    EXPECT_EQ(vertices.count(pXpYnZ), 1UL);
  }
  // 1 vertex
  {
    math::Planed plane(math::Vector3d(1, 1, 1), -1.2);

    auto vertices = box.VerticesBelow(plane);
    ASSERT_EQ(1u, vertices.size());

    EXPECT_EQ(vertices.count(nXnYnZ), 1UL);
  }
}

//////////////////////////////////////////////////
TEST(BoxTest, Mass)
{
  double mass = 2.0;
  double l = 2.0;
  double w = 0.1;
  double h = 34.12;
  math::Boxd box(l, w, h);
  box.SetDensityFromMass(mass);

  math::MassMatrix3d massMat;
  double ixx = (1.0/12.0) * mass * (w*w + h*h);
  double iyy = (1.0/12.0) * mass * (l*l + h*h);
  double izz = (1.0/12.0) * mass * (l*l + w*w);

  math::MassMatrix3d expectedMassMat;
  expectedMassMat.SetInertiaMatrix(ixx, iyy, izz, 0.0, 0.0, 0.0);
  expectedMassMat.SetMass(mass);

  box.MassMatrix(massMat);
  EXPECT_EQ(expectedMassMat, massMat);
  EXPECT_DOUBLE_EQ(expectedMassMat.Mass(), massMat.Mass());
}
