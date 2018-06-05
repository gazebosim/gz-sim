/*
 * Copyright (C) 2017 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.A()pache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gtest/gtest.h>
#include <cmath>
#include <ignition/math/Color.hh>

using namespace ignition;

/////////////////////////////////////////////////
TEST(Color, ConstColors)
{
  EXPECT_FLOAT_EQ(1.0f, math::Color::White.R());
  EXPECT_FLOAT_EQ(1.0f, math::Color::White.G());
  EXPECT_FLOAT_EQ(1.0f, math::Color::White.B());
  EXPECT_FLOAT_EQ(1.0f, math::Color::White.A());

  EXPECT_FLOAT_EQ(0.0f, math::Color::Black.R());
  EXPECT_FLOAT_EQ(0.0f, math::Color::Black.G());
  EXPECT_FLOAT_EQ(0.0f, math::Color::Black.B());
  EXPECT_FLOAT_EQ(1.0f, math::Color::Black.A());

  EXPECT_FLOAT_EQ(1.0f, math::Color::Red.R());
  EXPECT_FLOAT_EQ(0.0f, math::Color::Red.G());
  EXPECT_FLOAT_EQ(0.0f, math::Color::Red.B());
  EXPECT_FLOAT_EQ(1.0f, math::Color::Red.A());

  EXPECT_FLOAT_EQ(0.0f, math::Color::Green.R());
  EXPECT_FLOAT_EQ(1.0f, math::Color::Green.G());
  EXPECT_FLOAT_EQ(0.0f, math::Color::Green.B());
  EXPECT_FLOAT_EQ(1.0f, math::Color::Green.A());

  EXPECT_FLOAT_EQ(0.0f, math::Color::Blue.R());
  EXPECT_FLOAT_EQ(0.0f, math::Color::Blue.G());
  EXPECT_FLOAT_EQ(1.0f, math::Color::Blue.B());
  EXPECT_FLOAT_EQ(1.0f, math::Color::Blue.A());

  EXPECT_FLOAT_EQ(1.0f, math::Color::Yellow.R());
  EXPECT_FLOAT_EQ(1.0f, math::Color::Yellow.G());
  EXPECT_FLOAT_EQ(0.0f, math::Color::Yellow.B());
  EXPECT_FLOAT_EQ(1.0f, math::Color::Yellow.A());

  EXPECT_FLOAT_EQ(1.0f, math::Color::Magenta.R());
  EXPECT_FLOAT_EQ(0.0f, math::Color::Magenta.G());
  EXPECT_FLOAT_EQ(1.0f, math::Color::Magenta.B());
  EXPECT_FLOAT_EQ(1.0f, math::Color::Magenta.A());

  EXPECT_FLOAT_EQ(0.0f, math::Color::Cyan.R());
  EXPECT_FLOAT_EQ(1.0f, math::Color::Cyan.G());
  EXPECT_FLOAT_EQ(1.0f, math::Color::Cyan.B());
  EXPECT_FLOAT_EQ(1.0f, math::Color::Cyan.A());
}

/////////////////////////////////////////////////
TEST(Color, Color)
{
  math::Color clr0;
  EXPECT_FLOAT_EQ(0.0f, clr0.R());
  EXPECT_FLOAT_EQ(0.0f, clr0.G());
  EXPECT_FLOAT_EQ(0.0f, clr0.B());
  EXPECT_FLOAT_EQ(1.0f, clr0.A());
  EXPECT_EQ(clr0.AsRGBA(), 255u);
  clr0.A(0.0);
  EXPECT_EQ(clr0.AsRGBA(), 0u);

  math::Color clr(.1f, .2f, .3f, 1.0f);
  EXPECT_FLOAT_EQ(0.1f, clr.R());
  EXPECT_FLOAT_EQ(0.2f, clr.G());
  EXPECT_FLOAT_EQ(0.3f, clr.B());
  EXPECT_FLOAT_EQ(1.0f, clr.A());

  clr.Set(1, 0, 0, 0);
  EXPECT_EQ(clr.AsRGBA(), static_cast<uint32_t>(255) << 24);
  EXPECT_EQ(clr.AsBGRA(), static_cast<uint32_t>(255) << 8);
  EXPECT_EQ(clr.AsARGB(), static_cast<uint32_t>(255) << 16);
  EXPECT_EQ(clr.AsABGR(), static_cast<uint32_t>(255));
  clr0.SetFromRGBA(static_cast<uint32_t>(255) << 24);
  EXPECT_EQ(clr0, clr);
  clr0.SetFromBGRA(static_cast<uint32_t>(255) << 8);
  EXPECT_EQ(clr0, clr);
  clr0.SetFromARGB(static_cast<uint32_t>(255) << 16);
  EXPECT_EQ(clr0, clr);
  clr0.SetFromABGR(static_cast<uint32_t>(255));
  EXPECT_EQ(clr0, clr);

  clr.Set(0, 1, 0, 0);
  EXPECT_EQ(clr.AsRGBA(), static_cast<uint32_t>(255) << 16);
  EXPECT_EQ(clr.AsBGRA(), static_cast<uint32_t>(255) << 16);
  EXPECT_EQ(clr.AsARGB(), static_cast<uint32_t>(255) << 8);
  EXPECT_EQ(clr.AsABGR(), static_cast<uint32_t>(255) << 8);
  clr0.SetFromRGBA(static_cast<uint32_t>(255) << 16);
  EXPECT_EQ(clr0, clr);
  clr0.SetFromBGRA(static_cast<uint32_t>(255) << 16);
  EXPECT_EQ(clr0, clr);
  clr0.SetFromARGB(static_cast<uint32_t>(255) << 8);
  EXPECT_EQ(clr0, clr);
  clr0.SetFromABGR(static_cast<uint32_t>(255) << 8);
  EXPECT_EQ(clr0, clr);

  clr.Set(0, 0, 1, 0);
  EXPECT_EQ(clr.AsRGBA(), static_cast<uint32_t>(255) << 8);
  EXPECT_EQ(clr.AsBGRA(), static_cast<uint32_t>(255) << 24);
  EXPECT_EQ(clr.AsARGB(), static_cast<uint32_t>(255));
  EXPECT_EQ(clr.AsABGR(), static_cast<uint32_t>(255) << 16);
  clr0.SetFromRGBA(static_cast<uint32_t>(255) << 8);
  EXPECT_EQ(clr0, clr);
  clr0.SetFromBGRA(static_cast<uint32_t>(255) << 24);
  EXPECT_EQ(clr0, clr);
  clr0.SetFromARGB(static_cast<uint32_t>(255));
  EXPECT_EQ(clr0, clr);
  clr0.SetFromABGR(static_cast<uint32_t>(255) << 16);
  EXPECT_EQ(clr0, clr);

  clr.Set(0, 0, 0, 1);
  EXPECT_EQ(clr.AsRGBA(), static_cast<uint32_t>(255));
  EXPECT_EQ(clr.AsBGRA(), static_cast<uint32_t>(255));
  EXPECT_EQ(clr.AsARGB(), static_cast<uint32_t>(255) << 24);
  EXPECT_EQ(clr.AsABGR(), static_cast<uint32_t>(255) << 24);
  clr0.SetFromRGBA(static_cast<uint32_t>(255));
  EXPECT_EQ(clr0, clr);
  clr0.SetFromBGRA(static_cast<uint32_t>(255));
  EXPECT_EQ(clr0, clr);
  clr0.SetFromARGB(static_cast<uint32_t>(255) << 24);
  EXPECT_EQ(clr0, clr);
  clr0.SetFromABGR(static_cast<uint32_t>(255) << 24);
  EXPECT_EQ(clr0, clr);

  clr.Reset();
  EXPECT_FLOAT_EQ(0.0f, clr.R());
  EXPECT_FLOAT_EQ(0.0f, clr.G());
  EXPECT_FLOAT_EQ(0.0f, clr.B());
  EXPECT_FLOAT_EQ(1.0f, clr.A());

  clr.SetFromHSV(0, 0.5, 1.0);
  EXPECT_FLOAT_EQ(1.0f, clr.R());
  EXPECT_FLOAT_EQ(0.5f, clr.G());
  EXPECT_FLOAT_EQ(0.5f, clr.B());
  EXPECT_FLOAT_EQ(1.0f, clr.A());

  EXPECT_TRUE(clr.HSV() == math::Vector3f(6, 0.5, 1));

  clr.SetFromHSV(60, 0.0, 1.0);
  EXPECT_FLOAT_EQ(1.0f, clr.R());
  EXPECT_FLOAT_EQ(1.0f, clr.G());
  EXPECT_FLOAT_EQ(1.0f, clr.B());
  EXPECT_FLOAT_EQ(1.0f, clr.A());

  clr.SetFromHSV(120, 0.5, 1.0);
  EXPECT_FLOAT_EQ(0.5f, clr.R());
  EXPECT_FLOAT_EQ(1.0f, clr.G());
  EXPECT_FLOAT_EQ(0.5f, clr.B());
  EXPECT_FLOAT_EQ(1.0f, clr.A());

  clr.SetFromHSV(180, 0.5, 1.0);
  EXPECT_FLOAT_EQ(0.5f, clr.R());
  EXPECT_FLOAT_EQ(1.0f, clr.G());
  EXPECT_FLOAT_EQ(1.0f, clr.B());
  EXPECT_FLOAT_EQ(1.0f, clr.A());

  clr.SetFromHSV(240, 0.5, 1.0);
  EXPECT_FLOAT_EQ(0.5f, clr.R());
  EXPECT_FLOAT_EQ(0.5f, clr.G());
  EXPECT_FLOAT_EQ(1.0f, clr.B());
  EXPECT_FLOAT_EQ(1.0f, clr.A());

  clr.SetFromHSV(300, 0.5, 1.0);
  EXPECT_FLOAT_EQ(1.0f, clr[0]);
  EXPECT_FLOAT_EQ(0.5f, clr[1]);
  EXPECT_FLOAT_EQ(1.0f, clr[2]);
  EXPECT_FLOAT_EQ(1.0f, clr[3]);
  EXPECT_TRUE(std::isnan(clr[4]));

  clr.R() = 0.1f;
  clr.G() = 0.2f;
  clr.B() = 0.3f;
  clr.A() = 0.4f;
  EXPECT_FLOAT_EQ(0.1f, clr[0]);
  EXPECT_FLOAT_EQ(0.2f, clr[1]);
  EXPECT_FLOAT_EQ(0.3f, clr[2]);
  EXPECT_FLOAT_EQ(0.4f, clr[3]);

  clr.Set(0.1f, 0.2f, 0.3f, 0.4f);
  clr = clr + 0.2f;
  EXPECT_TRUE(clr == math::Color(0.3f, 0.4f, 0.5f, 0.6f));

  clr.Set(0.1f, 0.2f, 0.3f, 0.4f);
  clr += math::Color(0.2f, 0.2f, 0.2f, 0.2f);
  EXPECT_TRUE(clr == math::Color(0.3f, 0.4f, 0.5f, 0.6f));


  clr.Set(0.1f, 0.2f, 0.3f, 0.4f);
  clr = clr - 0.1f;
  EXPECT_TRUE(clr == math::Color(0.0f, 0.1f, 0.2f, 0.3f));

  clr.Set(0.1f, 0.2f, 0.3f, 0.4f);
  clr -= math::Color(0.1f, 0.1f, 0.1f, 0.1f);
  EXPECT_TRUE(clr == math::Color(0.0f, 0.1f, 0.2f, 0.3f));


  clr.Set(1.f, 1.f, 1.f, 1.f);
  clr = clr / 1.6f;
  EXPECT_TRUE(clr == math::Color(0.625f, 0.625f, 0.625f, 0.625f));

  clr.Set(1.f, 1.f, 1.f, 1.f);
  clr /= math::Color(1.f, 1.f, 1.f, 1.f);
  EXPECT_TRUE(clr == math::Color(1.f, 1.f, 1.f, 1.f));


  clr.Set(.1f, .2f, .3f, .4f);
  clr = clr * .1f;
  EXPECT_TRUE(clr == math::Color(0.01f, 0.02f, 0.03f, 0.04f));

  clr.Set(.1f, .2f, .3f, .4f);
  clr *= math::Color(0.1f, 0.1f, 0.1f, 0.1f);
  EXPECT_TRUE(clr == math::Color(0.01f, 0.02f, 0.03f, 0.04f));


  clr.SetFromYUV(0.5f, 0.2f, 0.8f);
  EXPECT_TRUE(math::equal(0.00553f, clr.R(), 1e-3f));
  EXPECT_TRUE(math::equal(0.0f, clr.G()));
  EXPECT_TRUE(math::equal(0.9064f, clr.B(), 1e-3f));
  EXPECT_TRUE(math::equal(0.04f, clr.A()));

  EXPECT_TRUE(clr.YUV() == math::Vector3f(0.104985f, 0.95227f, 0.429305f));

  clr = math::Color(1.0f, 0.0f, 0.5f, 1.0f) +
    math::Color(0.1f, 0.3f, 0.4f, 1.0f);
  EXPECT_TRUE(math::equal(0.00431373f, clr.R()));
  EXPECT_TRUE(math::equal(0.3f, clr.G()));
  EXPECT_TRUE(math::equal(0.9f, clr.B()));
  EXPECT_TRUE(math::equal(1.0f, clr.A()));

  clr = math::Color(1.0f, 0.0f, 0.5f, 1.0f) -
    math::Color(0.1f, 0.3f, 0.4f, 1.0f);
  EXPECT_TRUE(math::equal(0.9f, clr.R()));
  EXPECT_TRUE(math::equal(0.0f, clr.G()));
  EXPECT_TRUE(math::equal(0.1f, clr.B()));
  EXPECT_TRUE(math::equal(0.0f, clr.A()));

  clr = math::Color(0.5f, 0.2f, 0.4f, 0.6f) / 2.0f;
  EXPECT_TRUE(math::equal(0.25f, clr.R()));
  EXPECT_TRUE(math::equal(0.1f, clr.G()));
  EXPECT_TRUE(math::equal(0.2f, clr.B()));
  EXPECT_TRUE(math::equal(0.3f, clr.A()));
}


/////////////////////////////////////////////////
TEST(Color, MulOp)
{
  math::Color clr(0.0f, 0.01f, 0.2f, 1.0f);
  math::Color clr2(1.0f, 0.2f, 0.2f, 0.0f);
  math::Color clr3 = clr * clr2;

  EXPECT_FLOAT_EQ(clr3.R(), 0.0f);
  EXPECT_FLOAT_EQ(clr3.G(), 0.002f);
  EXPECT_FLOAT_EQ(clr3.B(), 0.04f);
  EXPECT_FLOAT_EQ(clr3.A(), 0.0f);
}

/////////////////////////////////////////////////
TEST(Color, DivisonOp)
{
  math::Color clr(0.0f, 0.01f, 0.2f, 1.0f);
  math::Color clr2 = clr / 0.2f;
  EXPECT_FLOAT_EQ(clr2.R(), 0.0f);
  EXPECT_FLOAT_EQ(clr2.G(), 0.05f);
  EXPECT_FLOAT_EQ(clr2.B(), 1.0f);
  EXPECT_FLOAT_EQ(clr2.A(), 1.0f);

  clr2 = clr / 2.0f;
  EXPECT_FLOAT_EQ(clr2.R(), 0.0f);
  EXPECT_FLOAT_EQ(clr2.G(), 0.005f);
  EXPECT_FLOAT_EQ(clr2.B(), 0.1f);
  EXPECT_FLOAT_EQ(clr2.A(), 0.5f);

  clr2.Set(0.0f, 0.2f, 0.4f, 0.5f);
  math::Color clr3 = clr / clr2;
  EXPECT_FLOAT_EQ(clr3.R(), 0.0f);
  EXPECT_FLOAT_EQ(clr3.G(), 0.05f);
  EXPECT_FLOAT_EQ(clr3.B(), 0.5f);
  EXPECT_FLOAT_EQ(clr3.A(), 1.0f);

  clr.Set(0.0f, 0.0f, 0.0f, 0.0f);
  clr2.Set(0.0f, 0.0f, 0.0f, 0.0f);
  clr3 = clr / clr2;
  EXPECT_FLOAT_EQ(clr3.R(), 0.0f);
  EXPECT_FLOAT_EQ(clr3.G(), 0.0f);
  EXPECT_FLOAT_EQ(clr3.B(), 0.0f);
  EXPECT_FLOAT_EQ(clr3.A(), 0.0f);
}

/////////////////////////////////////////////////
TEST(Color, ConstAndSet)
{
  const math::Color clr(0.1f, 0.2f, 0.3f, 0.4f);

  EXPECT_FLOAT_EQ(clr.R(), 0.1f);
  EXPECT_FLOAT_EQ(clr.G(), 0.2f);
  EXPECT_FLOAT_EQ(clr.B(), 0.3f);
  EXPECT_FLOAT_EQ(clr.A(), 0.4f);

  math::Color clr2;
  clr2.R(0.4f);
  clr2.G(0.3f);
  clr2.B(0.2f);
  clr2.A(0.1f);
  EXPECT_FLOAT_EQ(clr2.R(), 0.4f);
  EXPECT_FLOAT_EQ(clr2.G(), 0.3f);
  EXPECT_FLOAT_EQ(clr2.B(), 0.2f);
  EXPECT_FLOAT_EQ(clr2.A(), 0.1f);

  EXPECT_TRUE(clr2 != clr);
}

/////////////////////////////////////////////////
TEST(Color, OperatorStreamOut)
{
  math::Color c(0.1f, 0.2f, 0.3f, 0.5f);
  std::ostringstream stream;
  stream << c;
  EXPECT_EQ(stream.str(), "0.1 0.2 0.3 0.5");
}

/////////////////////////////////////////////////
TEST(Color, HSV)
{
  math::Color clr;
  math::Vector3f hsv = clr.HSV();
  EXPECT_FLOAT_EQ(hsv.X(), -1.0f);
  EXPECT_FLOAT_EQ(hsv.Y(), 0.0f);
  EXPECT_FLOAT_EQ(hsv.Z(), 0.0f);

  clr.Set(0.1f, 0.2f, 0.3f, 1.0f);
  hsv = clr.HSV();
  EXPECT_NEAR(hsv.X(), 3.5f, 1e-3);
  EXPECT_NEAR(hsv.Y(), 0.666667f, 1e-3);
  EXPECT_NEAR(hsv.Z(), 0.3f, 1e-3);

  clr.Set(0.3f, 0.2f, 0.1f, 1.0f);
  hsv = clr.HSV();
  EXPECT_NEAR(hsv.X(), 0.5f, 1e-3);
  EXPECT_NEAR(hsv.Y(), 0.666667f, 1e-3);
  EXPECT_NEAR(hsv.Z(), 0.3f, 1e-3);

  clr.SetFromHSV(60, 10, 5);
  EXPECT_NEAR(clr.R(), 0.0196078f, 1e-3);
  EXPECT_NEAR(clr.G(), 0.0196078f, 1e-3);
  EXPECT_NEAR(clr.B(), 0.0f, 1e-3);
  EXPECT_NEAR(clr.A(), 1.0, 1e-3);

  clr.SetFromHSV(360.0f, 0.5f, 0.6f);
  EXPECT_NEAR(clr.R(), 0.6f, 1e-3);
  EXPECT_NEAR(clr.G(), 0.3f, 1e-3);
  EXPECT_NEAR(clr.B(), 0.3f, 1e-3);
  EXPECT_NEAR(clr.A(), 1.0, 1e-3);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
