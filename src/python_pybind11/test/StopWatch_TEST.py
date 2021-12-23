# Copyright (C) 2021 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
import unittest
from datetime import datetime, timedelta

from ignition.math import Stopwatch


class TestBox(unittest.TestCase):
    # Helper function that runs a few tests
    def runTimer(self, _time):
        handleSteadyClock = timedelta(0)

        # Start the timer
        self.assertTrue(_time.start())
        # The timer should be running.
        self.assertTrue(_time.running())
        # The start time should be greater than the stop time.
        self.assertGreater(_time.start_time(), _time.stop_time())
        # The elapsed stop time should still be zero.
        self.assertEqual(timedelta(0), _time.elapsed_stop_time())

        # Wait for some time...
        time.sleep(1)
        # Now the elapsed time should be greater than or equal to the time slept.
        self.assertGreaterEqual(_time.elapsed_run_time() + handleSteadyClock, timedelta(seconds=1))

        # Stop the timer.
        self.assertTrue(_time.stop())
        # The timer should not be running.
        self.assertFalse(_time.running())
        # The stop time should be greater than the start time.
        self.assertGreater(_time.stop_time(), _time.start_time())
        # The elapsed time should still be greater than the time slept.
        self.assertGreaterEqual(_time.elapsed_run_time() + handleSteadyClock, timedelta(seconds=1))

        # Save the elapsed time.
        elapsedTime = _time.elapsed_run_time()

        # The timer is now stopped, let's sleep some more.
        time.sleep(1)
        # The elapsed stop time should be greater than or equal to the time
        # slept.
        self.assertGreaterEqual(_time.elapsed_stop_time() + handleSteadyClock, timedelta(seconds=1))
        # The elapsed time should be the same.
        self.assertEqual(elapsedTime, _time.elapsed_run_time())

        # Start the timer again.
        self.assertTrue(_time.start())
        # Store the elapsed stop time.
        elapsedStopTime = _time.elapsed_stop_time()
        # The timer should be running.
        self.assertTrue(_time.running())
        # Sleep for some time.
        time.sleep(1)
        # The elapsed stop time should remain the same
        self.assertEqual(elapsedStopTime, _time.elapsed_stop_time())
        # The elapsed time should be greater than the previous elapsed time.
        self.assertGreater(_time.elapsed_run_time(), elapsedTime)
        # The elapsed time should be greater than or equal to the the previous
        # two sleep times.
        self.assertGreaterEqual(_time.elapsed_run_time() + handleSteadyClock, timedelta(seconds=2))

    def test_constructor(self):
        watch = Stopwatch()

        self.assertFalse(watch.running())
        self.assertEqual(watch.stop_time(), watch.start_time())
        self.assertEqual(timedelta(0), watch.elapsed_run_time())
        self.assertEqual(timedelta(0), watch.elapsed_stop_time())

        self.runTimer(watch)

        watch2 = watch
        self.assertEqual(watch, watch2)

        watch3 = watch2
        self.assertEqual(watch, watch3)

    def test_equal_operator(self):
        watch = Stopwatch()
        watch2 = Stopwatch()
        watch3 = Stopwatch()
        self.assertEqual(watch, watch2)
        self.assertEqual(watch, watch3)

        self.runTimer(watch)
        self.runTimer(watch2)
        self.runTimer(watch3)

        self.assertNotEqual(watch, watch2)
        self.assertNotEqual(watch, watch3)

        watch2 = watch
        self.assertEqual(watch, watch2)

        watch3 = watch2
        self.assertEqual(watch, watch3)

    def test_start_stop_reset(self):
        watch = Stopwatch()

        self.runTimer(watch)

        watch.reset()

        self.assertFalse(watch.running())
        self.assertEqual(watch.stop_time(), watch.start_time())
        self.assertEqual(timedelta(0), watch.elapsed_run_time())
        self.assertEqual(timedelta(0), watch.elapsed_stop_time())

        self.runTimer(watch)

        self.assertTrue(watch.running())

        watch.start(True)
        self.assertTrue(watch.running())
        self.assertLess(watch.stop_time(), watch.start_time())
        self.assertNotEqual(timedelta(0), watch.elapsed_run_time())
        self.assertEqual(timedelta(0), watch.elapsed_stop_time())

    def test_fail_start_stop(self):
        watch = Stopwatch()

        # Can't stop while not running
        self.assertFalse(watch.stop())
        self.assertFalse(watch.running())

        # Can start while not running
        self.assertTrue(watch.start())
        self.assertTrue(watch.running())

        # Can't start while running
        self.assertFalse(watch.start())
        self.assertTrue(watch.running())

        # Can stop while running
        self.assertTrue(watch.stop())
        self.assertFalse(watch.running())

        # Can start while not running
        self.assertTrue(watch.start())
        self.assertTrue(watch.running())


if __name__ == '__main__':
    unittest.main()
