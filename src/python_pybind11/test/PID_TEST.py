# Copyright (C) 2021 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http:       #www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import datetime
import unittest
from ignition.math import PID


class TestPID(unittest.TestCase):

    def test_constructor(self):
        pid = PID()

        self.assertAlmostEqual(0.0, pid.p_gain())
        self.assertAlmostEqual(0.0, pid.i_gain())
        self.assertAlmostEqual(0.0, pid.d_gain())
        self.assertAlmostEqual(-1.0, pid.i_max())
        self.assertAlmostEqual(0.0, pid.i_min())
        self.assertAlmostEqual(-1.0, pid.cmd_max())
        self.assertAlmostEqual(0.0, pid.cmd_min())
        self.assertAlmostEqual(0.0, pid.cmd_offset())
        self.assertAlmostEqual(0.0, pid.cmd())

        [pe, ie, de] = pid.errors()
        self.assertAlmostEqual(pe, 0.0)
        self.assertAlmostEqual(ie, 0.0)
        self.assertAlmostEqual(de, 0.0)

    def test_set_values(self):
        pid2 = PID(1.0, 2.1, -4.5, 10.5, 1.4, 45, -35, 1.3)
        self.assertAlmostEqual(1.0,  pid2.p_gain())
        self.assertAlmostEqual(2.1,  pid2.i_gain())
        self.assertAlmostEqual(-4.5, pid2.d_gain())
        self.assertAlmostEqual(10.5, pid2.i_max())
        self.assertAlmostEqual(1.4,  pid2.i_min())
        self.assertAlmostEqual(45,   pid2.cmd_max())
        self.assertAlmostEqual(-35,  pid2.cmd_min())
        self.assertAlmostEqual(1.3,  pid2.cmd_offset())
        self.assertAlmostEqual(0.0,  pid2.cmd())

        # Test set*() functions
        cmd = 10.4
        pid = PID()
        pid.set_p_gain(pid2.p_gain())
        pid.set_i_gain(pid2.i_gain())
        pid.set_d_gain(pid2.d_gain())
        pid.set_i_max(pid2.i_max())
        pid.set_i_min(pid2.i_min())
        pid.set_cmd_max(pid2.cmd_max())
        pid.set_cmd_min(pid2.cmd_min())
        pid.set_cmd_offset(pid2.cmd_offset())
        pid.set_cmd(cmd)

        self.assertAlmostEqual(pid.p_gain(), pid2.p_gain())
        self.assertAlmostEqual(pid.i_gain(), pid2.i_gain())
        self.assertAlmostEqual(pid.d_gain(), pid2.d_gain())
        self.assertAlmostEqual(pid.i_max(), pid2.i_max())
        self.assertAlmostEqual(pid.i_min(), pid2.i_min())
        self.assertAlmostEqual(pid.cmd_max(), pid2.cmd_max())
        self.assertAlmostEqual(pid.cmd_min(), pid2.cmd_min())
        self.assertAlmostEqual(pid.cmd_offset(), pid2.cmd_offset())
        self.assertAlmostEqual(pid.cmd(), cmd)

        pid = PID(pid2)
        self.assertAlmostEqual(pid.p_gain(), pid2.p_gain())
        self.assertAlmostEqual(pid.i_gain(), pid2.i_gain())
        self.assertAlmostEqual(pid.d_gain(), pid2.d_gain())
        self.assertAlmostEqual(pid.i_max(), pid2.i_max())
        self.assertAlmostEqual(pid.i_min(), pid2.i_min())
        self.assertAlmostEqual(pid.cmd_max(), pid2.cmd_max())
        self.assertAlmostEqual(pid.cmd_min(), pid2.cmd_min())
        self.assertAlmostEqual(pid.cmd_offset(), pid2.cmd_offset())
        self.assertAlmostEqual(pid.cmd(), pid2.cmd())

    def test_equal_corner_case(self):
        pid = PID(1.0, 2.1, -4.5, 10.5, 1.4, 45, -35, 1.23)
        self.assertAlmostEqual(pid.p_gain(), 1.0)
        self.assertAlmostEqual(pid.i_gain(), 2.1)
        self.assertAlmostEqual(pid.d_gain(), -4.5)
        self.assertAlmostEqual(pid.i_max(), 10.5)
        self.assertAlmostEqual(pid.i_min(), 1.4)
        self.assertAlmostEqual(pid.cmd_max(), 45.0)
        self.assertAlmostEqual(pid.cmd_min(), -35.0)
        self.assertAlmostEqual(pid.cmd_offset(), 1.23)
        self.assertAlmostEqual(pid.cmd(), 0.0)

        pid = PID(pid)
        self.assertAlmostEqual(pid.p_gain(), 1.0)
        self.assertAlmostEqual(pid.i_gain(), 2.1)
        self.assertAlmostEqual(pid.d_gain(), -4.5)
        self.assertAlmostEqual(pid.i_max(), 10.5)
        self.assertAlmostEqual(pid.i_min(), 1.4)
        self.assertAlmostEqual(pid.cmd_max(), 45.0)
        self.assertAlmostEqual(pid.cmd_min(), -35.0)
        self.assertAlmostEqual(pid.cmd_offset(), 1.23)
        self.assertAlmostEqual(pid.cmd(), 0.0)

    def test_update(self):
        pid = PID()
        pid.init(1.0, 0.1, 0.5, 10.0, 0.0, 20.0, -20.0)

        result = pid.update(5.0, 0.0)
        self.assertAlmostEqual(result, 0.0)

        result = pid.update(5.0, 10.0)
        self.assertAlmostEqual(result, -10.25)

        [pe, ie, de] = pid.errors()
        self.assertAlmostEqual(pe, 5)
        self.assertAlmostEqual(ie, 5)
        self.assertAlmostEqual(de, 0.5)

        # Test max integral term
        pid.set_i_max(0.2)
        pid.set_i_gain(10.0)
        result = pid.update(5.0, 10.0)
        self.assertAlmostEqual(result, -5.2)
        [pe, ie, de] = pid.errors()
        self.assertAlmostEqual(pe, 5)
        self.assertAlmostEqual(ie, 0.2)
        self.assertAlmostEqual(de, 0.0)

        # Test min integral term
        pid.set_i_max(20)
        pid.set_i_min(1.4)
        pid.set_i_gain(0.01)
        result = pid.update(5.0, 10.0)
        self.assertAlmostEqual(result, -6.4)
        [pe, ie, de] = pid.errors()
        self.assertAlmostEqual(pe, 5)
        self.assertAlmostEqual(ie, 1.4)
        self.assertAlmostEqual(de, 0.0)

    def update_test(self, _pid, _result, _error,
                    _dt, _p_error, _i_error, _d_error):

        self.assertAlmostEqual(
            _result, _pid.update(_error, datetime.timedelta(seconds=_dt)))
        [p_error, i_error, d_error] = _pid.errors()
        self.assertAlmostEqual(p_error, _p_error)
        self.assertAlmostEqual(i_error, _i_error)
        self.assertAlmostEqual(d_error, _d_error)

    def test_zero_gains(self):
        # controller with zero gains, no command limits
        pid = PID()

        # repeat once to test derivative and integral error
        self.update_test(pid, 0, 0, 0, 0, 0, 0)
        self.update_test(pid, 0, 0, 0, 0, 0, 0)

        # dt = 0, no change since previous state
        self.update_test(pid, 0,  1, 0, 0, 0, 0)
        self.update_test(pid, 0,  1, 0, 0, 0, 0)
        self.update_test(pid, 0, -1, 0, 0, 0, 0)
        self.update_test(pid, 0, -1, 0, 0, 0, 0)
        self.update_test(pid, 0,  1, 0, 0, 0, 0)
        self.update_test(pid, 0,  1, 0, 0, 0, 0)

        # dt > 0, but gains still zero
        self.update_test(pid, 0,  1, 1,  1, 0, 1)
        self.update_test(pid, 0,  1, 1,  1, 0, 0)
        self.update_test(pid, 0, -1, 1, -1, 0, -2)
        self.update_test(pid, 0, -1, 1, -1, 0, 0)
        self.update_test(pid, 0,  1, 1,  1, 0, 2)
        self.update_test(pid, 0,  1, 1,  1, 0, 0)

        # dt = 0, no change since previous state
        self.update_test(pid, 0,  1, 0, 1, 0, 0)
        self.update_test(pid, 0,  1, 0, 1, 0, 0)
        self.update_test(pid, 0, -1, 0, 1, 0, 0)
        self.update_test(pid, 0, -1, 0, 1, 0, 0)
        self.update_test(pid, 0,  1, 0, 1, 0, 0)
        self.update_test(pid, 0,  1, 0, 1, 0, 0)

        # dt < 0, but gains still zero
        # TODO(chapulina) Check why d_error fails in the commented test cases
        self.update_test(pid, 0,  1, -1,  1, 0, 0)
        self.update_test(pid, 0,  1, -1,  1, 0, 0)
        # self.update_test(pid, 0, -1, -1, -1, 0, 2)
        # self.update_test(pid, 0, -1, -1, -1, 0, 0)
        # self.update_test(pid, 0,  1, -1,  1, 0, -2)
        self.update_test(pid, 0,  1, -1,  1, 0, 0)

        pid.reset()
        self.update_test(pid, 0,  1, 0, 0, 0, 0)

        # cmd_max defaults to -1.0
        # setting cmd_min to -10.0, means output should now be -1.0
        # when time is non-zero
        pid.set_cmd_min(-10.0)
        self.assertAlmostEqual(-10.0, pid.cmd_min())
        # command hasn't been updated yet
        self.assertAlmostEqual(0.0, pid.cmd())

        # dt = 0, still report cmd = 0
        self.update_test(pid, 0,  1, 0, 0, 0, 0)
        self.update_test(pid, 0,  1, 0, 0, 0, 0)
        self.update_test(pid, 0, -1, 0, 0, 0, 0)
        self.update_test(pid, 0, -1, 0, 0, 0, 0)
        self.update_test(pid, 0,  1, 0, 0, 0, 0)
        self.update_test(pid, 0,  1, 0, 0, 0, 0)

        # dt > 0, report clamped value
        self.update_test(pid, -1,  1, 1,  1, 0, 1)
        self.update_test(pid, -1,  1, 1,  1, 0, 0)
        self.update_test(pid, -1, -1, 1, -1, 0, -2)
        self.update_test(pid, -1, -1, 1, -1, 0, 0)
        self.update_test(pid, -1,  1, 1,  1, 0, 2)
        self.update_test(pid, -1,  1, 1,  1, 0, 0)

        pid.reset()
        self.update_test(pid, 0,  1, 0, 0, 0, 0)

        # i_max defaults to -1.0
        # setting i_min to -10.0, means output should now be -1.0
        # when time is non-zero
        pid.set_i_min(-10.0)
        self.assertAlmostEqual(-10.0, pid.i_min())
        # i_err hasn't been updated yet
        [p_err, i_err, d_err] = pid.errors()
        self.assertAlmostEqual(0.0, i_err)

        # dt = 0, still report iErr = 0
        self.update_test(pid, 0,  1, 0, 0, 0, 0)
        self.update_test(pid, 0,  1, 0, 0, 0, 0)
        self.update_test(pid, 0, -1, 0, 0, 0, 0)
        self.update_test(pid, 0, -1, 0, 0, 0, 0)
        self.update_test(pid, 0,  1, 0, 0, 0, 0)
        self.update_test(pid, 0,  1, 0, 0, 0, 0)

        # dt > 0, report clamped value
        self.update_test(pid, -1,  1, 1,  1, -1, 1)
        self.update_test(pid, -1,  1, 1,  1, -1, 0)
        self.update_test(pid, -1, -1, 1, -1, -1, -2)
        self.update_test(pid, -1, -1, 1, -1, -1, 0)
        self.update_test(pid, -1,  1, 1,  1, -1, 2)
        self.update_test(pid, -1,  1, 1,  1, -1, 0)

        pid.reset()
        self.update_test(pid, 0,  1, 0, 0, 0, 0)

        pid.set_cmd_offset(-20.0)
        self.assertAlmostEqual(-20.0, pid.cmd_offset())
        # cmd hasn't been updated yet
        self.assertAlmostEqual(0.0, pid.cmd())

        # dt = 0, still return 0
        self.update_test(pid, 0,  1, 0, 0, 0, 0)
        self.update_test(pid, 0,  1, 0, 0, 0, 0)
        self.update_test(pid, 0, -1, 0, 0, 0, 0)
        self.update_test(pid, 0, -1, 0, 0, 0, 0)
        self.update_test(pid, 0,  1, 0, 0, 0, 0)
        self.update_test(pid, 0,  1, 0, 0, 0, 0)

        # dt > 0, report negative min value
        self.update_test(pid, -10,  1, 1,  1, -1, 1)
        self.update_test(pid, -10,  1, 1,  1, -1, 0)
        self.update_test(pid, -10, -1, 1, -1, -1, -2)
        self.update_test(pid, -10, -1, 1, -1, -1, 0)
        self.update_test(pid, -10,  1, 1,  1, -1, 2)
        self.update_test(pid, -10,  1, 1,  1, -1, 0)

    def test_p_control(self):
        pid = PID(1)
        N = 5
        for i in range(N):
            self.assertAlmostEqual(-i, pid.update(i, 1.0))

        pid.set_p_gain(2)
        for i in range(N):
            self.assertAlmostEqual(-2*i, pid.update(i, 1.0))

    def test_i_control(self):
        pid = PID(0, 1)
        N = 5
        for i in range(N):
            self.assertAlmostEqual(-(i+1), pid.update(1, 1.0))

        pid.set_i_gain(2)

        [p_err, i_err, d_err] = pid.errors()
        self.assertAlmostEqual(N, i_err)
        i_0 = i_err

        # confirm that changing gain doesn't cause jumps in integral control
        self.assertAlmostEqual(-i_0, pid.update(0, 1.0))
        self.assertAlmostEqual(-i_0, pid.update(0, 1.0))

        for i in range(N):
            self.assertAlmostEqual(-i_0-2*(i+1), pid.update(1, 1.0))

    def test_d_control(self):
        pid = PID(0, 0, 1)
        self.assertAlmostEqual(1, pid.update(-1, 1.0))
        N = 5
        for i in range(N):
            self.assertAlmostEqual(-1, pid.update(i, 1.0))

        pid.set_d_gain(2)
        self.assertAlmostEqual(10, pid.update(-1, 1.0))
        for i in range(N):
            self.assertAlmostEqual(-2, pid.update(i, 1.0))


if __name__ == '__main__':
    unittest.main()
