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

#include "ignition/math/PID.hh"
#include "ignition/math/Helpers.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(PidTest, PID)
{
  math::PID pid;
  EXPECT_NEAR(pid.PGain(), 0, 1e-6);
  EXPECT_NEAR(pid.IGain(), 0, 1e-6);
  EXPECT_NEAR(pid.DGain(), 0, 1e-6);
  EXPECT_NEAR(pid.IMax(), 0, 1e-6);
  EXPECT_NEAR(pid.IMin(), 0, 1e-6);
  EXPECT_NEAR(pid.CmdMax(), -1.0, 1e-6);
  EXPECT_NEAR(pid.CmdMin(), 0, 1e-6);
  EXPECT_NEAR(pid.Cmd(), 0, 1e-6);

  math::PID pid2(1.0, 2.1, -4.5, 10.5, 1.4, 45, -35);
  EXPECT_NEAR(pid2.PGain(), 1.0, 1e-6);
  EXPECT_NEAR(pid2.IGain(), 2.1, 1e-6);
  EXPECT_NEAR(pid2.DGain(), -4.5, 1e-6);
  EXPECT_NEAR(pid2.IMax(), 10.5, 1e-6);
  EXPECT_NEAR(pid2.IMin(), 1.4, 1e-6);
  EXPECT_NEAR(pid2.CmdMax(), 45.0, 1e-6);
  EXPECT_NEAR(pid2.CmdMin(), -35.0, 1e-6);
  EXPECT_NEAR(pid2.Cmd(), 0.0, 1e-6);

  pid.SetPGain(pid2.PGain());
  pid.SetIGain(pid2.IGain());
  pid.SetDGain(pid2.DGain());
  pid.SetIMax(pid2.IMax());
  pid.SetIMin(pid2.IMin());
  pid.SetCmdMax(pid2.CmdMax());
  pid.SetCmdMin(pid2.CmdMin());
  pid.SetCmd(10.4);

  EXPECT_NEAR(pid.PGain(), pid2.PGain(), 1e-6);
  EXPECT_NEAR(pid.IGain(), pid2.IGain(), 1e-6);
  EXPECT_NEAR(pid.DGain(), pid2.DGain(), 1e-6);
  EXPECT_NEAR(pid.IMax(), pid2.IMax(), 1e-6);
  EXPECT_NEAR(pid.IMin(), pid2.IMin(), 1e-6);
  EXPECT_NEAR(pid.CmdMax(), pid2.CmdMax(), 1e-6);
  EXPECT_NEAR(pid.CmdMin(), pid2.CmdMin(), 1e-6);
  EXPECT_NEAR(pid.Cmd(), 10.4, 1e-6);

  math::PID pid3;
  pid3 = pid;
  EXPECT_NEAR(pid.PGain(), pid3.PGain(), 1e-6);
  EXPECT_NEAR(pid.IGain(), pid3.IGain(), 1e-6);
  EXPECT_NEAR(pid.DGain(), pid3.DGain(), 1e-6);
  EXPECT_NEAR(pid.IMax(), pid3.IMax(), 1e-6);
  EXPECT_NEAR(pid.IMin(), pid3.IMin(), 1e-6);
  EXPECT_NEAR(pid.CmdMax(), pid3.CmdMax(), 1e-6);
  EXPECT_NEAR(pid.CmdMin(), pid3.CmdMin(), 1e-6);
  EXPECT_NEAR(pid.Cmd(), pid3.Cmd(), 1e-6);

  double pe, ie, de;
  pid.Errors(pe, ie, de);
  EXPECT_NEAR(pe, 0.0, 1e-6);
  EXPECT_NEAR(ie, 0.0, 1e-6);
  EXPECT_NEAR(de, 0.0, 1e-6);
}

/////////////////////////////////////////////////
TEST(PidTest, EqualOperatorCornerCase)
{
  math::PID pid(1.0, 2.1, -4.5, 10.5, 1.4, 45, -35);
  EXPECT_NEAR(pid.PGain(), 1.0, 1e-6);
  EXPECT_NEAR(pid.IGain(), 2.1, 1e-6);
  EXPECT_NEAR(pid.DGain(), -4.5, 1e-6);
  EXPECT_NEAR(pid.IMax(), 10.5, 1e-6);
  EXPECT_NEAR(pid.IMin(), 1.4, 1e-6);
  EXPECT_NEAR(pid.CmdMax(), 45.0, 1e-6);
  EXPECT_NEAR(pid.CmdMin(), -35.0, 1e-6);
  EXPECT_NEAR(pid.Cmd(), 0.0, 1e-6);

  pid = pid;

  EXPECT_NEAR(pid.PGain(), 1.0, 1e-6);
  EXPECT_NEAR(pid.IGain(), 2.1, 1e-6);
  EXPECT_NEAR(pid.DGain(), -4.5, 1e-6);
  EXPECT_NEAR(pid.IMax(), 10.5, 1e-6);
  EXPECT_NEAR(pid.IMin(), 1.4, 1e-6);
  EXPECT_NEAR(pid.CmdMax(), 45.0, 1e-6);
  EXPECT_NEAR(pid.CmdMin(), -35.0, 1e-6);
  EXPECT_NEAR(pid.Cmd(), 0.0, 1e-6);
}

/////////////////////////////////////////////////
TEST(PidTest, Update)
{
  math::PID pid;
  pid.Init(1.0, 0.1, 0.5, 10, 0, 20, -20);

  double result = pid.Update(5.0, std::chrono::duration<double>(0.0));
  EXPECT_NEAR(result, 0.0, 1e-6);

  result = pid.Update(5.0, std::chrono::duration<double>(10.0));
  EXPECT_NEAR(result, -10.25, 1e-6);

  double pe, ie, de;
  pid.Errors(pe, ie, de);
  EXPECT_NEAR(pe, 5, 1e-6);
  EXPECT_NEAR(ie, 50, 1e-6);
  EXPECT_NEAR(de, 0.5, 1e-6);

  // Test max integral term
  pid.SetIMax(0.2);
  pid.SetIGain(10.0);
  result = pid.Update(5.0, std::chrono::duration<double>(10.0));
  EXPECT_NEAR(result, -5.2, 1e-6);
  pid.Errors(pe, ie, de);
  EXPECT_NEAR(pe, 5, 1e-6);
  EXPECT_NEAR(ie, 0.02, 1e-6);
  EXPECT_NEAR(de, 0.0, 1e-6);

  // Test min integral term
  pid.SetIMax(20);
  pid.SetIMin(1.4);
  pid.SetIGain(0.01);
  result = pid.Update(5.0, std::chrono::duration<double>(10.0));
  EXPECT_NEAR(result, -6.4, 1e-6);
  pid.Errors(pe, ie, de);
  EXPECT_NEAR(pe, 5, 1e-6);
  EXPECT_NEAR(ie, 140, 1e-6);
  EXPECT_NEAR(de, 0.0, 1e-6);
}
