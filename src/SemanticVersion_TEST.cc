/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include "ignition/math/SemanticVersion.hh"

using namespace ignition;
using namespace math;

/////////////////////////////////////////////////
TEST(SemVerTest, Prerelease)
{
  SemanticVersion a("0.1.0");
  SemanticVersion b("0.1.0-pr2");

  EXPECT_TRUE(b < a);
  EXPECT_TRUE(a.Prerelease().empty());
  EXPECT_EQ(b.Prerelease(), "pr2");

  EXPECT_EQ(b.Major(), a.Major());
  EXPECT_EQ(b.Minor(), a.Minor());
  EXPECT_EQ(b.Patch(), a.Patch());
  EXPECT_EQ(b.Build(), a.Build());

  EXPECT_EQ(a.Version(), "0.1.0");
  EXPECT_EQ(b.Version(), "0.1.0-pr2");
}

/////////////////////////////////////////////////
TEST(SemVerTest, Build)
{
  SemanticVersion a("0.1.0");
  SemanticVersion b("0.1.0+012345");

  EXPECT_TRUE(b == a);
  EXPECT_TRUE(a.Build().empty());
  EXPECT_EQ(b.Build(), "012345");

  EXPECT_EQ(b.Major(), a.Major());
  EXPECT_EQ(b.Minor(), a.Minor());
  EXPECT_EQ(b.Patch(), a.Patch());
  EXPECT_EQ(b.Prerelease(), a.Prerelease());

  EXPECT_EQ(a.Version(), "0.1.0");
  EXPECT_EQ(b.Version(), "0.1.0+012345");
}

/////////////////////////////////////////////////
TEST(SemVerTest, PrereleaseBuild)
{
  SemanticVersion a("0.1.0");
  SemanticVersion b("0.1.0-pr2");
  SemanticVersion c("0.1.0+012345");
  SemanticVersion d("0.1.0-pr2+012345");

  EXPECT_EQ(a.Version(), "0.1.0");
  EXPECT_EQ(b.Version(), "0.1.0-pr2");
  EXPECT_EQ(c.Version(), "0.1.0+012345");
  EXPECT_EQ(d.Version(), "0.1.0-pr2+012345");

  EXPECT_TRUE(b < a);
  EXPECT_TRUE(b < c);
  EXPECT_TRUE(b == d);
  EXPECT_TRUE(a == c);

  EXPECT_EQ(a.Major(), b.Major());
  EXPECT_EQ(a.Minor(), b.Minor());
  EXPECT_EQ(a.Patch(), b.Patch());
  EXPECT_TRUE(a.Prerelease().empty());
  EXPECT_TRUE(a.Build().empty());

  EXPECT_EQ(b.Major(), c.Major());
  EXPECT_EQ(b.Minor(), c.Minor());
  EXPECT_EQ(b.Patch(), c.Patch());
  EXPECT_EQ(b.Prerelease(), d.Prerelease());

  EXPECT_EQ(c.Major(), d.Major());
  EXPECT_EQ(c.Minor(), d.Minor());
  EXPECT_EQ(c.Patch(), d.Patch());
  EXPECT_EQ(c.Build(), d.Build());

  EXPECT_EQ(d.Build(), "012345");
  EXPECT_EQ(d.Prerelease(), "pr2");
}

/////////////////////////////////////////////////
TEST(SemVerTest, OperatorStreamOut)
{
  SemanticVersion a("0.1.0");
  SemanticVersion b("0.1.0-pr2");
  SemanticVersion c("0.1.0+012345");
  SemanticVersion d("0.1.0-pr2+012345");

  {
    std::ostringstream stream;
    stream << a;
    EXPECT_EQ(stream.str(), "0.1.0");
  }

  {
    std::ostringstream stream;
    stream << b;
    EXPECT_EQ(stream.str(), "0.1.0-pr2");
  }

  {
    std::ostringstream stream;
    stream << c;
    EXPECT_EQ(stream.str(), "0.1.0+012345");
  }

  {
    std::ostringstream stream;
    stream << d;
    EXPECT_EQ(stream.str(), "0.1.0-pr2+012345");
  }
}

/////////////////////////////////////////////////
TEST(SemVerTest, Operators)
{
  SemanticVersion a("0.1.0");
  SemanticVersion b("1.0.0");
  SemanticVersion c("1.0.0");
  SemanticVersion c2("1.0.2");
  SemanticVersion d("0.2.0");

  // check that the short form is the same as the long one
  SemanticVersion aa("0.1");
  EXPECT_EQ(aa.Version(), "0.1.0");

  // check second constructor
  SemanticVersion c2b(1, 0, 2);
  EXPECT_TRUE(c2 == c2b);

  EXPECT_FALSE(a < a);
  EXPECT_FALSE(b < a);
  EXPECT_TRUE(a < d);
  EXPECT_FALSE(d < a);

  EXPECT_TRUE(a < b);
  EXPECT_TRUE(a <= b);

  // equality
  EXPECT_TRUE(a != b);
  EXPECT_TRUE(b == c);
  EXPECT_FALSE(a == b);
  EXPECT_FALSE(a != a);

  // other operators
  EXPECT_TRUE(b <= c);
  EXPECT_TRUE(b >= c);
  EXPECT_TRUE(c2 > c);
  EXPECT_FALSE(c2 < c);
  EXPECT_TRUE(b == b);
}

/////////////////////////////////////////////////
TEST(SemVerTest, AssignCopy)
{
  SemanticVersion a("0.1+pr2");
  SemanticVersion b("0.2");

  SemanticVersion aa(a);
  EXPECT_TRUE(a == aa);
  SemanticVersion aaa = aa;
  EXPECT_TRUE(a == aaa);
  // change a
  a = b;
  // aaa unchanged
  EXPECT_TRUE(aa == aaa);
  // a and aaa now different
  EXPECT_TRUE(a != aaa);
}

/////////////////////////////////////////////////
TEST(SemVerTest, Parse)
{
  SemanticVersion a;
  EXPECT_FALSE(a.Parse(""));
  EXPECT_FALSE(a.Parse("0.1.2+1-1"));
}

/////////////////////////////////////////////////
TEST(SemVerTest, Constructor)
{
  SemanticVersion a;

  EXPECT_EQ(a.Major(), 0u);
  EXPECT_EQ(a.Minor(), 0u);
  EXPECT_EQ(a.Patch(), 0u);
  EXPECT_TRUE(a.Prerelease().empty());
  EXPECT_TRUE(a.Build().empty());
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

