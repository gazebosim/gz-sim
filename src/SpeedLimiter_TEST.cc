/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "gz/math/Helpers.hh"
#include "gz/math/SpeedLimiter.hh"

using namespace gz;
using namespace math;
using namespace std::literals::chrono_literals;

/////////////////////////////////////////////////
TEST(SpeedLimiterTest, Default)
{
  // Unlimited by default, the velocity shouldn't change.
  SpeedLimiter limiter;

  EXPECT_DOUBLE_EQ(-std::numeric_limits<double>::infinity(),
      limiter.MinVelocity());
  EXPECT_DOUBLE_EQ(std::numeric_limits<double>::infinity(),
      limiter.MaxVelocity());
  EXPECT_DOUBLE_EQ(-std::numeric_limits<double>::infinity(),
      limiter.MinAcceleration());
  EXPECT_DOUBLE_EQ(std::numeric_limits<double>::infinity(),
      limiter.MaxAcceleration());
  EXPECT_DOUBLE_EQ(-std::numeric_limits<double>::infinity(),
      limiter.MinJerk());
  EXPECT_DOUBLE_EQ(std::numeric_limits<double>::infinity(),
      limiter.MaxJerk());

  double vel{5.0};
  EXPECT_DOUBLE_EQ(0.0, limiter.Limit(vel, 4.0, 3.0, 1ms));
  EXPECT_DOUBLE_EQ(5.0, vel);

  EXPECT_DOUBLE_EQ(0.0, limiter.LimitVelocity(vel));
  EXPECT_DOUBLE_EQ(5.0, vel);

  EXPECT_DOUBLE_EQ(0.0, limiter.LimitAcceleration(vel, 4.0, 1ms));
  EXPECT_DOUBLE_EQ(5.0, vel);

  EXPECT_DOUBLE_EQ(0.0, limiter.LimitJerk(vel, 4.0, 3.0, 1ms));
  EXPECT_DOUBLE_EQ(5.0, vel);

  vel = 0.0;
  EXPECT_NEAR(0.0, limiter.Limit(vel, 4.0, 3.0, 1ms), 1e-6);
  EXPECT_NEAR(0.0, vel, 1e-6);
}

/////////////////////////////////////////////////
TEST(SpeedLimiterTest, LimitVelocity)
{
  double minVel = -1.0;
  double maxVel = 4.0;

  SpeedLimiter limiter;
  limiter.SetMinVelocity(minVel);
  limiter.SetMaxVelocity(maxVel);

  EXPECT_DOUBLE_EQ(minVel, limiter.MinVelocity());
  EXPECT_DOUBLE_EQ(maxVel, limiter.MaxVelocity());

  // Within bounds
  double vel{1.0};
  EXPECT_DOUBLE_EQ(0.0, limiter.LimitVelocity(vel));
  EXPECT_DOUBLE_EQ(1.0, vel);

  // Above upper bound
  vel = 5.0;
  EXPECT_DOUBLE_EQ(-1.0, limiter.LimitVelocity(vel));
  EXPECT_DOUBLE_EQ(maxVel, vel);

  // Under lower bound
  vel = -2.0;
  EXPECT_DOUBLE_EQ(1.0, limiter.LimitVelocity(vel));
  EXPECT_DOUBLE_EQ(minVel, vel);
}

/////////////////////////////////////////////////
TEST(SpeedLimiterTest, LimitAcceleration)
{
  double minAcc = -2.0;
  double maxAcc = 4.0;

  SpeedLimiter limiter;
  limiter.SetMinAcceleration(minAcc);
  limiter.SetMaxAcceleration(maxAcc);

  EXPECT_DOUBLE_EQ(minAcc, limiter.MinAcceleration());
  EXPECT_DOUBLE_EQ(maxAcc, limiter.MaxAcceleration());

  auto dt = 1s;
  const double dtSec = std::chrono::duration<double>(dt).count();

  // Within bounds
  double vel{2.0};
  double velPrev = 1.0;
  EXPECT_GT(vel - velPrev, minAcc * dtSec);
  EXPECT_LT(vel - velPrev, maxAcc * dtSec);

  EXPECT_DOUBLE_EQ(0.0, limiter.LimitAcceleration(vel, velPrev, dt));
  EXPECT_DOUBLE_EQ(2.0, vel);

  // Above upper bound
  vel = 10.0;
  velPrev = 1.0;
  EXPECT_GT(vel - velPrev, minAcc * dtSec);
  EXPECT_GT(vel - velPrev, maxAcc * dtSec);

  EXPECT_DOUBLE_EQ(-5.0, limiter.LimitAcceleration(vel, velPrev, dt));
  EXPECT_DOUBLE_EQ(5.0, vel);
  EXPECT_DOUBLE_EQ(vel - velPrev, maxAcc * dtSec);

  // Under lower bound
  vel = -8.0;
  velPrev = -2.0;
  EXPECT_LT(vel - velPrev, minAcc * dtSec);
  EXPECT_LT(vel - velPrev, maxAcc * dtSec);

  EXPECT_DOUBLE_EQ(4.0, limiter.LimitAcceleration(vel, velPrev, dt));
  EXPECT_DOUBLE_EQ(-4.0, vel);
  EXPECT_DOUBLE_EQ(vel - velPrev, minAcc * dtSec);
}

/////////////////////////////////////////////////
TEST(SpeedLimiterTest, LimitJerk)
{
  double minJerk = -1.0;
  double maxJerk = 1.0;

  SpeedLimiter limiter;
  limiter.SetMinJerk(minJerk);
  limiter.SetMaxJerk(maxJerk);

  EXPECT_DOUBLE_EQ(minJerk, limiter.MinJerk());
  EXPECT_DOUBLE_EQ(maxJerk, limiter.MaxJerk());

  auto dt = 1s;
  const double dtSec = std::chrono::duration<double>(dt).count();

  // Within bounds
  double vel{2.0};
  double velPrev = 1.0;
  double velPrevPrev = 0.5;
  double acc = (vel - velPrev) / dtSec;
  double accPrev = (velPrev - velPrevPrev) / dtSec;
  EXPECT_GT(acc - accPrev, minJerk * dtSec);
  EXPECT_LT(acc - accPrev, maxJerk * dtSec);

  EXPECT_DOUBLE_EQ(0.0, limiter.LimitJerk(vel, velPrev, velPrevPrev, dt));
  EXPECT_DOUBLE_EQ(2.0, vel);

  // Above upper bound (desired: accel 4.0, jerk 3.0)
  vel = 6.0;
  velPrev = 2.0;
  velPrevPrev = 1.0;
  acc = (vel - velPrev) / dtSec;
  accPrev = (velPrev - velPrevPrev) / dtSec;
  EXPECT_GT(acc - accPrev, minJerk * dtSec);
  EXPECT_GT(acc - accPrev, maxJerk * dtSec);

  EXPECT_DOUBLE_EQ(-2.0, limiter.LimitJerk(vel, velPrev, velPrevPrev, dt));
  EXPECT_DOUBLE_EQ(4.0, vel);
  acc = (vel - velPrev) / dtSec;
  EXPECT_DOUBLE_EQ(acc - accPrev, maxJerk * dtSec);

  // Under lower bound
  vel = -6.0;
  velPrev = -2.0;
  velPrevPrev = -1.0;
  acc = (vel - velPrev) / dtSec;
  accPrev = (velPrev - velPrevPrev) / dtSec;
  EXPECT_LT(acc - accPrev, minJerk * dtSec);
  EXPECT_LT(acc - accPrev, maxJerk * dtSec);

  EXPECT_DOUBLE_EQ(2.0, limiter.LimitJerk(vel, velPrev, velPrevPrev, dt));
  EXPECT_DOUBLE_EQ(-4.0, vel);

  acc = vel - velPrev;
  EXPECT_DOUBLE_EQ(acc - accPrev, minJerk * dtSec);
}

/////////////////////////////////////////////////
TEST(SpeedLimiterTest, LimitAll)
{
  double minVel = -4.0;
  double maxVel = 4.0;
  double minAcc = -2.0;
  double maxAcc = 2.0;
  double minJerk = -1.0;
  double maxJerk = 1.0;

  SpeedLimiter limiter;
  limiter.SetMinVelocity(minVel);
  limiter.SetMaxVelocity(maxVel);
  limiter.SetMinAcceleration(minAcc);
  limiter.SetMaxAcceleration(maxAcc);
  limiter.SetMinJerk(minJerk);
  limiter.SetMaxJerk(maxJerk);

  EXPECT_DOUBLE_EQ(minVel, limiter.MinVelocity());
  EXPECT_DOUBLE_EQ(maxVel, limiter.MaxVelocity());
  EXPECT_DOUBLE_EQ(minAcc, limiter.MinAcceleration());
  EXPECT_DOUBLE_EQ(maxAcc, limiter.MaxAcceleration());
  EXPECT_DOUBLE_EQ(minJerk, limiter.MinJerk());
  EXPECT_DOUBLE_EQ(maxJerk, limiter.MaxJerk());

  auto dt = 1s;

  // Within bounds
  double vel{2.0};
  double velPrev = 1.0;
  double velPrevPrev = 0.5;
  EXPECT_DOUBLE_EQ(0.0, limiter.Limit(vel, velPrev, velPrevPrev, dt));
  EXPECT_DOUBLE_EQ(2.0, vel);

  // Above upper velocity bound
  vel = 5.0;
  velPrev = 2.0;
  velPrevPrev = 1.0;
  EXPECT_DOUBLE_EQ(-1.0, limiter.Limit(vel, velPrev, velPrevPrev, dt));
  EXPECT_DOUBLE_EQ(4.0, vel);

  // Above upper acceleration bound (desired accel: 3.0)
  vel = 4.0;
  velPrev = 1.0;
  velPrevPrev = 0.0;
  EXPECT_DOUBLE_EQ(-1.0, limiter.Limit(vel, velPrev, velPrevPrev, dt));
  EXPECT_DOUBLE_EQ(3.0, vel);

  // Above upper jerk bound (desired jerk: 1.5, accel: 1.8)
  vel = 2.1;
  velPrev = 0.3;
  velPrevPrev = 0.0;
  EXPECT_DOUBLE_EQ(-0.5, limiter.Limit(vel, velPrev, velPrevPrev, dt));
  EXPECT_DOUBLE_EQ(1.6, vel);

  // No change in zero time
  dt = 0s;
  vel = 2.1;

  EXPECT_DOUBLE_EQ(0.0, limiter.Limit(vel, velPrev, velPrevPrev, dt));
  EXPECT_DOUBLE_EQ(2.1, vel);
}
