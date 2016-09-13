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
TEST(PidTest, ConstructorDefault)
{
  const math::PID pid;
  EXPECT_DOUBLE_EQ(0.0, pid.PGain());
  EXPECT_DOUBLE_EQ(0.0, pid.IGain());
  EXPECT_DOUBLE_EQ(0.0, pid.DGain());
  EXPECT_DOUBLE_EQ(-1.0, pid.IMax());
  EXPECT_DOUBLE_EQ(0.0, pid.IMin());
  EXPECT_DOUBLE_EQ(-1.0, pid.CmdMax());
  EXPECT_DOUBLE_EQ(0.0, pid.CmdMin());
  EXPECT_DOUBLE_EQ(0.0, pid.CmdOffset());
  EXPECT_DOUBLE_EQ(0.0, pid.Cmd());

  double pe, ie, de;
  pid.Errors(pe, ie, de);
  EXPECT_DOUBLE_EQ(pe, 0.0);
  EXPECT_DOUBLE_EQ(ie, 0.0);
  EXPECT_DOUBLE_EQ(de, 0.0);
}

/////////////////////////////////////////////////
TEST(PidTest, CoverageExtra)
{
  // getting full destructor coverage
  math::PID *p = new math::PID;
  EXPECT_TRUE(p != NULL);
  delete p;
}

/////////////////////////////////////////////////
TEST(PidTest, SetValues)
{
  const math::PID pid2(1.0, 2.1, -4.5, 10.5, 1.4, 45, -35, 1.3);
  EXPECT_DOUBLE_EQ(1.0,  pid2.PGain());
  EXPECT_DOUBLE_EQ(2.1,  pid2.IGain());
  EXPECT_DOUBLE_EQ(-4.5, pid2.DGain());
  EXPECT_DOUBLE_EQ(10.5, pid2.IMax());
  EXPECT_DOUBLE_EQ(1.4,  pid2.IMin());
  EXPECT_DOUBLE_EQ(45,   pid2.CmdMax());
  EXPECT_DOUBLE_EQ(-35,  pid2.CmdMin());
  EXPECT_DOUBLE_EQ(1.3,  pid2.CmdOffset());
  EXPECT_DOUBLE_EQ(0.0,  pid2.Cmd());

  // Test Set*() functions
  {
    const double cmd = 10.4;
    math::PID pid;
    pid.SetPGain(pid2.PGain());
    pid.SetIGain(pid2.IGain());
    pid.SetDGain(pid2.DGain());
    pid.SetIMax(pid2.IMax());
    pid.SetIMin(pid2.IMin());
    pid.SetCmdMax(pid2.CmdMax());
    pid.SetCmdMin(pid2.CmdMin());
    pid.SetCmdOffset(pid2.CmdOffset());
    pid.SetCmd(cmd);

    EXPECT_DOUBLE_EQ(pid.PGain(), pid2.PGain());
    EXPECT_DOUBLE_EQ(pid.IGain(), pid2.IGain());
    EXPECT_DOUBLE_EQ(pid.DGain(), pid2.DGain());
    EXPECT_DOUBLE_EQ(pid.IMax(), pid2.IMax());
    EXPECT_DOUBLE_EQ(pid.IMin(), pid2.IMin());
    EXPECT_DOUBLE_EQ(pid.CmdMax(), pid2.CmdMax());
    EXPECT_DOUBLE_EQ(pid.CmdMin(), pid2.CmdMin());
    EXPECT_DOUBLE_EQ(pid.CmdOffset(), pid2.CmdOffset());
    EXPECT_DOUBLE_EQ(pid.Cmd(), cmd);
  }

  // Assignment operator
  {
    math::PID pid;
    pid = pid2;
    EXPECT_DOUBLE_EQ(pid.PGain(), pid2.PGain());
    EXPECT_DOUBLE_EQ(pid.IGain(), pid2.IGain());
    EXPECT_DOUBLE_EQ(pid.DGain(), pid2.DGain());
    EXPECT_DOUBLE_EQ(pid.IMax(), pid2.IMax());
    EXPECT_DOUBLE_EQ(pid.IMin(), pid2.IMin());
    EXPECT_DOUBLE_EQ(pid.CmdMax(), pid2.CmdMax());
    EXPECT_DOUBLE_EQ(pid.CmdMin(), pid2.CmdMin());
    EXPECT_DOUBLE_EQ(pid.CmdOffset(), pid2.CmdOffset());
    EXPECT_DOUBLE_EQ(pid.Cmd(), pid2.Cmd());
  }
}

/////////////////////////////////////////////////
TEST(PidTest, EqualOperatorCornerCase)
{
  math::PID pid(1.0, 2.1, -4.5, 10.5, 1.4, 45, -35, 1.23);
  EXPECT_DOUBLE_EQ(pid.PGain(), 1.0);
  EXPECT_DOUBLE_EQ(pid.IGain(), 2.1);
  EXPECT_DOUBLE_EQ(pid.DGain(), -4.5);
  EXPECT_DOUBLE_EQ(pid.IMax(), 10.5);
  EXPECT_DOUBLE_EQ(pid.IMin(), 1.4);
  EXPECT_DOUBLE_EQ(pid.CmdMax(), 45.0);
  EXPECT_DOUBLE_EQ(pid.CmdMin(), -35.0);
  EXPECT_DOUBLE_EQ(pid.CmdOffset(), 1.23);
  EXPECT_DOUBLE_EQ(pid.Cmd(), 0.0);

  pid = pid;

  EXPECT_DOUBLE_EQ(pid.PGain(), 1.0);
  EXPECT_DOUBLE_EQ(pid.IGain(), 2.1);
  EXPECT_DOUBLE_EQ(pid.DGain(), -4.5);
  EXPECT_DOUBLE_EQ(pid.IMax(), 10.5);
  EXPECT_DOUBLE_EQ(pid.IMin(), 1.4);
  EXPECT_DOUBLE_EQ(pid.CmdMax(), 45.0);
  EXPECT_DOUBLE_EQ(pid.CmdMin(), -35.0);
  EXPECT_DOUBLE_EQ(pid.CmdOffset(), 1.23);
  EXPECT_DOUBLE_EQ(pid.Cmd(), 0.0);
}

/////////////////////////////////////////////////
TEST(PidTest, Update)
{
  math::PID pid;
  pid.Init(1.0, 0.1, 0.5, 10, 0, 20, -20);

  double result = pid.Update(5.0, std::chrono::duration<double>(0.0));
  EXPECT_DOUBLE_EQ(result, 0.0);

  result = pid.Update(5.0, std::chrono::duration<double>(10.0));
  EXPECT_DOUBLE_EQ(result, -10.25);

  double pe, ie, de;
  pid.Errors(pe, ie, de);
  EXPECT_DOUBLE_EQ(pe, 5);
  EXPECT_DOUBLE_EQ(ie, 5);
  EXPECT_DOUBLE_EQ(de, 0.5);

  // Test max integral term
  pid.SetIMax(0.2);
  pid.SetIGain(10.0);
  result = pid.Update(5.0, std::chrono::duration<double>(10.0));
  EXPECT_DOUBLE_EQ(result, -5.2);
  pid.Errors(pe, ie, de);
  EXPECT_DOUBLE_EQ(pe, 5);
  EXPECT_DOUBLE_EQ(ie, 0.2);
  EXPECT_DOUBLE_EQ(de, 0.0);

  // Test min integral term
  pid.SetIMax(20);
  pid.SetIMin(1.4);
  pid.SetIGain(0.01);
  result = pid.Update(5.0, std::chrono::duration<double>(10.0));
  EXPECT_DOUBLE_EQ(result, -6.4);
  pid.Errors(pe, ie, de);
  EXPECT_DOUBLE_EQ(pe, 5);
  EXPECT_DOUBLE_EQ(ie, 1.4);
  EXPECT_DOUBLE_EQ(de, 0.0);
}

/////////////////////////////////////////////////
/// \brief Helper function for testing PID::Update
/// \param[in] _pid PID object.
/// \param[in] _result Expected PID output.
/// \param[in] _error Error input to Update.
/// \param[in] _dt Time interval.
/// \param[in] _pErr Expected proportional error.
/// \param[in] _iErr Expected integral error.
/// \param[in] _dErr Expected derivative error.
void UpdateTest(math::PID &_pid, const double _result, const double _error,
                const std::chrono::duration<double> &_dt,
                const double _pErr, const double _iErr, const double _dErr)
{
  EXPECT_DOUBLE_EQ(_result, _pid.Update(_error, _dt));
  double pErr, iErr, dErr;
  _pid.Errors(pErr, iErr, dErr);
  EXPECT_DOUBLE_EQ(pErr, _pErr);
  EXPECT_DOUBLE_EQ(iErr, _iErr);
  EXPECT_DOUBLE_EQ(dErr, _dErr);
}

/////////////////////////////////////////////////
TEST(PidTest, ZeroGains)
{
  // controller with zero gains, no command limits
  math::PID pid;

  std::cerr << "zero inputs, expect zero outputs" << std::endl;
  // repeat once to test derivative and integral error
  UpdateTest(pid, 0, 0, std::chrono::duration<double>(0), 0, 0, 0);
  UpdateTest(pid, 0, 0, std::chrono::duration<double>(0), 0, 0, 0);

  std::cerr << "dt = 0, no change since previous state" << std::endl;
  UpdateTest(pid, 0,  1, std::chrono::duration<double>(0), 0, 0, 0);
  UpdateTest(pid, 0,  1, std::chrono::duration<double>(0), 0, 0, 0);
  UpdateTest(pid, 0, -1, std::chrono::duration<double>(0), 0, 0, 0);
  UpdateTest(pid, 0, -1, std::chrono::duration<double>(0), 0, 0, 0);
  UpdateTest(pid, 0,  1, std::chrono::duration<double>(0), 0, 0, 0);
  UpdateTest(pid, 0,  1, std::chrono::duration<double>(0), 0, 0, 0);

  std::cerr << "dt > 0, but gains still zero" << std::endl;
  UpdateTest(pid, 0,  1, std::chrono::duration<double>(1),  1, 0, 1);
  UpdateTest(pid, 0,  1, std::chrono::duration<double>(1),  1, 0, 0);
  UpdateTest(pid, 0, -1, std::chrono::duration<double>(1), -1, 0, -2);
  UpdateTest(pid, 0, -1, std::chrono::duration<double>(1), -1, 0, 0);
  UpdateTest(pid, 0,  1, std::chrono::duration<double>(1),  1, 0, 2);
  UpdateTest(pid, 0,  1, std::chrono::duration<double>(1),  1, 0, 0);

  std::cerr << "dt = 0, no change since previous state" << std::endl;
  UpdateTest(pid, 0,  1, std::chrono::duration<double>(0), 1, 0, 0);
  UpdateTest(pid, 0,  1, std::chrono::duration<double>(0), 1, 0, 0);
  UpdateTest(pid, 0, -1, std::chrono::duration<double>(0), 1, 0, 0);
  UpdateTest(pid, 0, -1, std::chrono::duration<double>(0), 1, 0, 0);
  UpdateTest(pid, 0,  1, std::chrono::duration<double>(0), 1, 0, 0);
  UpdateTest(pid, 0,  1, std::chrono::duration<double>(0), 1, 0, 0);

  std::cerr << "dt < 0, but gains still zero" << std::endl;
  UpdateTest(pid, 0,  1, std::chrono::duration<double>(-1),  1, 0, 0);
  UpdateTest(pid, 0,  1, std::chrono::duration<double>(-1),  1, 0, 0);
  UpdateTest(pid, 0, -1, std::chrono::duration<double>(-1), -1, 0, 2);
  UpdateTest(pid, 0, -1, std::chrono::duration<double>(-1), -1, 0, 0);
  UpdateTest(pid, 0,  1, std::chrono::duration<double>(-1),  1, 0, -2);
  UpdateTest(pid, 0,  1, std::chrono::duration<double>(-1),  1, 0, 0);

  std::cerr << "Reset" << std::endl;
  pid.Reset();
  UpdateTest(pid, 0,  1, std::chrono::duration<double>(0), 0, 0, 0);

  std::cerr << "unclamp Cmd values" << std::endl;
  // CmdMax defaults to -1.0
  // setting CmdMin to -10.0, means output should now be -1.0
  // when time is non-zero
  pid.SetCmdMin(-10.0);
  EXPECT_DOUBLE_EQ(-10.0, pid.CmdMin());
  // command hasn't been updated yet
  EXPECT_DOUBLE_EQ(0.0, pid.Cmd());

  std::cerr << "dt = 0, still report cmd = 0" << std::endl;
  UpdateTest(pid, 0,  1, std::chrono::duration<double>(0), 0, 0, 0);
  UpdateTest(pid, 0,  1, std::chrono::duration<double>(0), 0, 0, 0);
  UpdateTest(pid, 0, -1, std::chrono::duration<double>(0), 0, 0, 0);
  UpdateTest(pid, 0, -1, std::chrono::duration<double>(0), 0, 0, 0);
  UpdateTest(pid, 0,  1, std::chrono::duration<double>(0), 0, 0, 0);
  UpdateTest(pid, 0,  1, std::chrono::duration<double>(0), 0, 0, 0);

  std::cerr << "dt > 0, report clamped value" << std::endl;
  UpdateTest(pid, -1,  1, std::chrono::duration<double>(1),  1, 0, 1);
  UpdateTest(pid, -1,  1, std::chrono::duration<double>(1),  1, 0, 0);
  UpdateTest(pid, -1, -1, std::chrono::duration<double>(1), -1, 0, -2);
  UpdateTest(pid, -1, -1, std::chrono::duration<double>(1), -1, 0, 0);
  UpdateTest(pid, -1,  1, std::chrono::duration<double>(1),  1, 0, 2);
  UpdateTest(pid, -1,  1, std::chrono::duration<double>(1),  1, 0, 0);

  std::cerr << "Reset" << std::endl;
  pid.Reset();
  UpdateTest(pid, 0,  1, std::chrono::duration<double>(0), 0, 0, 0);

  std::cerr << "unclamp iErr values" << std::endl;
  // IMax defaults to -1.0
  // setting IMin to -10.0, means output should now be -1.0
  // when time is non-zero
  pid.SetIMin(-10.0);
  EXPECT_DOUBLE_EQ(-10.0, pid.IMin());
  // iErr hasn't been updated yet
  {
    double pErr, iErr, dErr;
    pid.Errors(pErr, iErr, dErr);
    EXPECT_DOUBLE_EQ(0.0, iErr);
  }

  std::cerr << "dt = 0, still report iErr = 0" << std::endl;
  UpdateTest(pid, 0,  1, std::chrono::duration<double>(0), 0, 0, 0);
  UpdateTest(pid, 0,  1, std::chrono::duration<double>(0), 0, 0, 0);
  UpdateTest(pid, 0, -1, std::chrono::duration<double>(0), 0, 0, 0);
  UpdateTest(pid, 0, -1, std::chrono::duration<double>(0), 0, 0, 0);
  UpdateTest(pid, 0,  1, std::chrono::duration<double>(0), 0, 0, 0);
  UpdateTest(pid, 0,  1, std::chrono::duration<double>(0), 0, 0, 0);

  std::cerr << "dt > 0, report clamped value" << std::endl;
  UpdateTest(pid, -1,  1, std::chrono::duration<double>(1),  1, -1, 1);
  UpdateTest(pid, -1,  1, std::chrono::duration<double>(1),  1, -1, 0);
  UpdateTest(pid, -1, -1, std::chrono::duration<double>(1), -1, -1, -2);
  UpdateTest(pid, -1, -1, std::chrono::duration<double>(1), -1, -1, 0);
  UpdateTest(pid, -1,  1, std::chrono::duration<double>(1),  1, -1, 2);
  UpdateTest(pid, -1,  1, std::chrono::duration<double>(1),  1, -1, 0);

  std::cerr << "Reset" << std::endl;
  pid.Reset();
  UpdateTest(pid, 0,  1, std::chrono::duration<double>(0), 0, 0, 0);

  std::cerr << "set Cmd Offset" << std::endl;
  pid.SetCmdOffset(-20.0);
  EXPECT_DOUBLE_EQ(-20.0, pid.CmdOffset());
  // Cmd hasn't been updated yet
  EXPECT_DOUBLE_EQ(0.0, pid.Cmd());

  std::cerr << "dt = 0, still return 0" << std::endl;
  UpdateTest(pid, 0,  1, std::chrono::duration<double>(0), 0, 0, 0);
  UpdateTest(pid, 0,  1, std::chrono::duration<double>(0), 0, 0, 0);
  UpdateTest(pid, 0, -1, std::chrono::duration<double>(0), 0, 0, 0);
  UpdateTest(pid, 0, -1, std::chrono::duration<double>(0), 0, 0, 0);
  UpdateTest(pid, 0,  1, std::chrono::duration<double>(0), 0, 0, 0);
  UpdateTest(pid, 0,  1, std::chrono::duration<double>(0), 0, 0, 0);

  std::cerr << "dt > 0, report negative min value" << std::endl;
  UpdateTest(pid, -10,  1, std::chrono::duration<double>(1),  1, -1, 1);
  UpdateTest(pid, -10,  1, std::chrono::duration<double>(1),  1, -1, 0);
  UpdateTest(pid, -10, -1, std::chrono::duration<double>(1), -1, -1, -2);
  UpdateTest(pid, -10, -1, std::chrono::duration<double>(1), -1, -1, 0);
  UpdateTest(pid, -10,  1, std::chrono::duration<double>(1),  1, -1, 2);
  UpdateTest(pid, -10,  1, std::chrono::duration<double>(1),  1, -1, 0);
}

/////////////////////////////////////////////////
TEST(PidTest, Pcontrol)
{
  math::PID pid(1);
  std::chrono::duration<double> dt(1);
  const int N = 5;
  for (int i = 0; i < N; ++i)
  {
    double d = static_cast<double>(i);
    EXPECT_DOUBLE_EQ(-d, pid.Update(d, std::chrono::duration<double>(1)));
  }

  pid.SetPGain(2);
  for (int i = 0; i < N; ++i)
  {
    double d = static_cast<double>(i);
    EXPECT_DOUBLE_EQ(-2*d, pid.Update(d, std::chrono::duration<double>(1)));
  }
}

/////////////////////////////////////////////////
TEST(PidTest, Icontrol)
{
  math::PID pid(0, 1);
  std::chrono::duration<double> dt(1);
  const int N = 5;
  for (int i = 0; i < N; ++i)
  {
    double d = static_cast<double>(i+1);
    EXPECT_DOUBLE_EQ(-d, pid.Update(1, std::chrono::duration<double>(1)));
  }

  pid.SetIGain(2);
  double I0;
  {
    double pErr, iErr, dErr;
    pid.Errors(pErr, iErr, dErr);
    EXPECT_DOUBLE_EQ(N, iErr);
    I0 = iErr;
  }

  // confirm that changing gain doesn't cause jumps in integral control
  EXPECT_DOUBLE_EQ(-I0, pid.Update(0, std::chrono::duration<double>(1)));
  EXPECT_DOUBLE_EQ(-I0, pid.Update(0, std::chrono::duration<double>(1)));

  for (int i = 0; i < N; ++i)
  {
    double d = static_cast<double>(i+1);
    EXPECT_DOUBLE_EQ(-I0-2*d, pid.Update(1, std::chrono::duration<double>(1)));
  }
}

/////////////////////////////////////////////////
TEST(PidTest, Dcontrol)
{
  math::PID pid(0, 0, 1);
  std::chrono::duration<double> dt(1);
  EXPECT_DOUBLE_EQ(1, pid.Update(-1, std::chrono::duration<double>(1)));
  const int N = 5;
  for (int i = 0; i < N; ++i)
  {
    double d = static_cast<double>(i);
    EXPECT_DOUBLE_EQ(-1, pid.Update(d, std::chrono::duration<double>(1)));
  }

  pid.SetDGain(2);
  EXPECT_DOUBLE_EQ(10, pid.Update(-1, std::chrono::duration<double>(1)));
  for (int i = 0; i < N; ++i)
  {
    double d = static_cast<double>(i);
    EXPECT_DOUBLE_EQ(-2, pid.Update(d, std::chrono::duration<double>(1)));
  }
}
