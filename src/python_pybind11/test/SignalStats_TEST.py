# Copyright (C) 2021 Open Source Robotics Foundation

# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#       http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import unittest
from ignition.math import Rand
from ignition.math import SignalMaxAbsoluteValue
from ignition.math import SignalMaximum
from ignition.math import SignalMean
from ignition.math import SignalMinimum
from ignition.math import SignalRootMeanSquare
from ignition.math import SignalStats
from ignition.math import SignalVariance


class TestSignalStats(unittest.TestCase):

    def test_signal_maximum_constructor(self):
        # Constructor
        max = SignalMaximum()
        self.assertAlmostEqual(max.value(), 0.0)
        self.assertEqual(max.count(), 0)
        self.assertEqual(max.short_name(), "max")

        # reset
        max.reset()
        self.assertAlmostEqual(max.value(), 0.0)
        self.assertEqual(max.count(), 0)

    def test_signal_maximum_constant_values(self):
        # Constant values, max should match
        max = SignalMaximum()
        self.assertAlmostEqual(max.value(), 0.0)
        self.assertEqual(max.count(), 0)

        value = 3.14159

        # Loop two times to verify reset
        for j in range(2):
            for i in range(1, 11):
                max.insert_data(value)
                self.assertAlmostEqual(max.value(), value)
                self.assertEqual(max.count(), i)

            # reset
            max.reset()
            self.assertAlmostEqual(max.value(), 0.0)
            self.assertEqual(max.count(), 0)

    def test_signal_maximum_alternating_values(self):
        # Values with alternating sign, increasing magnitude
        # Should always match positive value
        max = SignalMaximum()
        self.assertAlmostEqual(max.value(), 0.0)
        self.assertEqual(max.count(), 0)

        value = 3.14159

        # Loop two times to verify reset
        for j in range(2):

            for i in range(1, 11):
                max.insert_data(value * i)
                self.assertAlmostEqual(max.value(), value * i)
                max.insert_data(-value * i)
                self.assertAlmostEqual(max.value(), value * i)
                self.assertEqual(max.count(), i*2)

            # reset
            max.reset()
            self.assertAlmostEqual(max.value(), 0.0)
            self.assertEqual(max.count(), 0)

    def test_signal_mean_constructor(self):
        # Constructor
        mean = SignalMean()
        self.assertAlmostEqual(mean.value(), 0.0)
        self.assertEqual(mean.count(), 0)
        self.assertEqual(mean.short_name(), "mean")

        # reset
        mean.reset()
        self.assertAlmostEqual(mean.value(), 0.0)
        self.assertEqual(mean.count(), 0)

    def test_signal_mean_constant_values(self):
        # Constant values, mean should match
        mean = SignalMean()
        self.assertAlmostEqual(mean.value(), 0.0)
        self.assertEqual(mean.count(), 0)

        value = 3.14159

        # Loop two times to verify reset
        for j in range(2):
            for i in range(1, 11):
                mean.insert_data(value)
                self.assertAlmostEqual(mean.value(), value)
                self.assertEqual(mean.count(), i)

            # reset
            mean.reset()
            self.assertAlmostEqual(mean.value(), 0.0)
            self.assertEqual(mean.count(), 0)

    def test_signal_mean_alternating_values(self):
        # Values with alternating sign, increasing magnitude
        # Should be zero every other time
        mean = SignalMean()
        self.assertAlmostEqual(mean.value(), 0.0)
        self.assertEqual(mean.count(), 0)

        value = 3.14159

        # Loop two times to verify reset
        for j in range(2):
            for i in range(1, 11):
                mean.insert_data(value * i)
                mean.insert_data(-value * i)
                self.assertAlmostEqual(mean.value(), 0.0)
                self.assertEqual(mean.count(), i*2)

            # reset
            mean.reset()
            self.assertAlmostEqual(mean.value(), 0.0)
            self.assertEqual(mean.count(), 0)

    def test_signal_minimum_constructor(self):
        # Constructor
        min = SignalMinimum()
        self.assertAlmostEqual(min.value(), 0.0)
        self.assertEqual(min.count(), 0)
        self.assertEqual(min.short_name(), "min")

        # reset
        min.reset()
        self.assertAlmostEqual(min.value(), 0.0)
        self.assertEqual(min.count(), 0)

    def test_signal_minimum_constant_values(self):
        # Constant values, min should match
        min = SignalMinimum()
        self.assertAlmostEqual(min.value(), 0.0)
        self.assertEqual(min.count(), 0)

        value = 3.14159

        # Loop two times to verify reset
        for j in range(2):
            for i in range(1, 11):
                min.insert_data(value)
                self.assertAlmostEqual(min.value(), value)
                self.assertEqual(min.count(), i)

            # reset
            min.reset()
            self.assertAlmostEqual(min.value(), 0.0)
            self.assertEqual(min.count(), 0)

    def test_signal_minimum_alternating_values(self):
        # Values with alternating sign, increasing magnitude
        # Should always match negative value
        min = SignalMinimum()
        self.assertAlmostEqual(min.value(), 0.0)
        self.assertEqual(min.count(), 0)

        value = 3.14159

        # Loop two times to verify reset
        for j in range(2):
            for i in range(1, 11):
                min.insert_data(value * i)
                min.insert_data(-value * i)
                self.assertAlmostEqual(min.value(), -value * i)
                self.assertEqual(min.count(), i*2)

            # reset
            min.reset()
            self.assertAlmostEqual(min.value(), 0.0)
            self.assertEqual(min.count(), 0)

    def test_signal_root_mean_square(self):
        # Constructor
        rms = SignalRootMeanSquare()
        self.assertAlmostEqual(rms.value(), 0.0)
        self.assertEqual(rms.count(), 0)
        self.assertEqual(rms.short_name(), "rms")

        # reset
        rms.reset()
        self.assertAlmostEqual(rms.value(), 0.0)
        self.assertEqual(rms.count(), 0)

    def test_signal_root_mean_square_constant_values(self):
        # Constant values, rms should match
        rms = SignalRootMeanSquare()
        self.assertAlmostEqual(rms.value(), 0.0)
        self.assertEqual(rms.count(), 0)

        value = 3.14159

        # Loop two times to verify reset
        for j in range(2):
            for i in range(1, 11):
                rms.insert_data(value)
                self.assertAlmostEqual(rms.value(), value)
                self.assertEqual(rms.count(), i)

            # reset
            rms.reset()
            self.assertAlmostEqual(rms.value(), 0.0)
            self.assertEqual(rms.count(), 0)

    def test_signal_root_mean_square_alternating_values(self):
        # Values with alternating sign, same magnitude
        # rms should match absolute value every time
        rms = SignalRootMeanSquare()
        self.assertAlmostEqual(rms.value(), 0.0)
        self.assertEqual(rms.count(), 0)

        value = 3.14159

        # Loop two times to verify reset
        for j in range(2):
            for i in range(1, 11):
                rms.insert_data(value)
                self.assertAlmostEqual(rms.value(), value)
                self.assertEqual(rms.count(), i*2-1)

                rms.insert_data(-value)
                self.assertAlmostEqual(rms.value(), value)
                self.assertEqual(rms.count(), i*2)

            # reset
            rms.reset()
            self.assertAlmostEqual(rms.value(), 0.0)
            self.assertEqual(rms.count(), 0)

    def test_signal_max_absolute_value_constructor(self):
        # Constructor
        max = SignalMaxAbsoluteValue()
        self.assertAlmostEqual(max.value(), 0.0)
        self.assertEqual(max.count(), 0)
        self.assertEqual(max.short_name(), "maxAbs")

        # reset
        max.reset()
        self.assertAlmostEqual(max.value(), 0.0)
        self.assertEqual(max.count(), 0)

    def test_signal_max_absolute_value_constant_values(self):
        # Constant values, max should match
        max = SignalMaxAbsoluteValue()
        self.assertAlmostEqual(max.value(), 0.0)
        self.assertEqual(max.count(), 0)

        value = 3.14159

        # Loop two times to verify reset
        for j in range(2):
            for i in range(1, 11):
                max.insert_data(value)
                self.assertAlmostEqual(max.value(), value)
                self.assertEqual(max.count(), i)

            # reset
            max.reset()
            self.assertAlmostEqual(max.value(), 0.0)
            self.assertEqual(max.count(), 0)

    def test_signal_max_absolute_value_alternating_values(self):
        # Values with alternating sign, increasing magnitude
        # max should match absolute value every time
        max = SignalMaxAbsoluteValue()
        self.assertAlmostEqual(max.value(), 0.0)
        self.assertEqual(max.count(), 0)

        value = 3.14159

        # Loop two times to verify reset
        for j in range(2):
            for i in range(1, 11):
                max.insert_data(value * i)
                self.assertAlmostEqual(max.value(), value * i)
                self.assertEqual(max.count(), i*2-1)

                max.insert_data(-value * i)
                self.assertAlmostEqual(max.value(), value * i)
                self.assertEqual(max.count(), i*2)

            # reset
            max.reset()
            self.assertAlmostEqual(max.value(), 0.0)
            self.assertEqual(max.count(), 0)

    def test_signal_variance_constructor(self):
        var = SignalVariance()
        self.assertAlmostEqual(var.value(), 0.0)
        self.assertEqual(var.count(), 0)
        self.assertEqual(var.short_name(), "var")

        # reset
        var.reset()
        self.assertAlmostEqual(var.value(), 0.0)
        self.assertEqual(var.count(), 0)

    def test_signal_variance_one_value(self):
        # Add one value, expect 0.0 variance
        values = {0, 1.0, 10.0, -100.0}
        for value in values:
            var = SignalVariance()
            var.insert_data(value)
            self.assertEqual(var.count(), 1)
            self.assertAlmostEqual(0.0, var.value())

            # reset
            var.reset()
            self.assertAlmostEqual(0.0, var.value())
            self.assertEqual(var.count(), 0)

    def test_signal_variance_constant_values(self):
        # Constant values, expect 0.0 variance
        var = SignalVariance()
        value = 3.14159

        # Loop two times to verify reset
        for j in range(2):
            for i in range(1, 11):
                var.insert_data(value)
                self.assertAlmostEqual(0.0, var.value())
                self.assertEqual(var.count(), i)

            # reset
            var.reset()
            self.assertAlmostEqual(var.value(), 0.0)
            self.assertEqual(var.count(), 0)

    def test_signal_variance_random_values(self):
        # Random normally distributed values
        # The sample variance has the following variance:
        # 2 variance^2 / (count - 1)
        # en.wikipedia.org/wiki/Variance#Distribution_of_the_sample_variance
        # We will use 5 sigma (4e-5 chance of failure)
        var = SignalVariance()
        std_dev = 3.14159
        count = 10000
        sigma = 5.0
        for i in range(count):
            var.insert_data(Rand.dbl_normal(0.0, std_dev))

        variance = std_dev*std_dev
        sampleVariance2 = 2 * variance*variance / (count - 1)
        self.assertAlmostEqual(var.value(), variance,
                               delta=sigma*math.sqrt(sampleVariance2))

        # reset
        var.reset()
        self.assertAlmostEqual(var.value(), 0.0)
        self.assertEqual(var.count(), 0)

    def test_signal_stats_constructor(self):
        # Constructor
        stats = SignalStats()
        self.assertTrue(len(stats.map()) == 0)
        self.assertEqual(stats.count(), 0)

        stats2 = SignalStats(stats)
        self.assertEqual(stats.count(), stats2.count())

        # reset
        stats.reset()
        self.assertTrue(len(stats.map()) == 0)
        self.assertEqual(stats.count(), 0)

    def test_01_signal_stats_intern_statistic(self):
        # insert static
        stats = SignalStats()
        self.assertTrue(len(stats.map()) == 0)

        self.assertTrue(stats.insert_statistic("max"))
        self.assertFalse(stats.insert_statistic("max"))
        self.assertFalse(len(stats.map()) == 0)

        self.assertTrue(stats.insert_statistic("maxAbs"))
        self.assertFalse(stats.insert_statistic("maxAbs"))
        self.assertFalse(len(stats.map()) == 0)

        self.assertTrue(stats.insert_statistic("mean"))
        self.assertFalse(stats.insert_statistic("mean"))
        self.assertFalse(len(stats.map()) == 0)

        self.assertTrue(stats.insert_statistic("min"))
        self.assertFalse(stats.insert_statistic("min"))
        self.assertFalse(len(stats.map()) == 0)

        self.assertTrue(stats.insert_statistic("rms"))
        self.assertFalse(stats.insert_statistic("rms"))
        self.assertFalse(len(stats.map()) == 0)

        self.assertTrue(stats.insert_statistic("var"))
        self.assertFalse(stats.insert_statistic("var"))
        self.assertFalse(len(stats.map()) == 0)

        self.assertFalse(stats.insert_statistic("FakeStatistic"))

        # map with no data
        map = stats.map()
        self.assertFalse(len(map) == 0)
        self.assertEqual(len(map), 6)

        self.assertEqual("max" in map.keys(), 1)
        self.assertEqual("maxAbs" in map.keys(), 1)
        self.assertEqual("mean" in map.keys(), 1)
        self.assertEqual("min" in map.keys(), 1)
        self.assertEqual("rms" in map.keys(), 1)
        self.assertEqual("var" in map.keys(), 1)
        self.assertEqual("FakeStatistic" in map.keys(), 0)

        stats2 = SignalStats(stats)
        map2 = stats2.map()
        self.assertFalse(len(map2) == 0)
        self.assertEqual(len(map), len(map2))
        self.assertEqual("max" in map.keys(), "max" in map2.keys())
        self.assertEqual("maxAbs" in map.keys(), "maxAbs" in map2.keys())
        self.assertEqual("mean" in map.keys(), "mean" in map2.keys())
        self.assertEqual("min" in map.keys(), "min" in map2.keys())
        self.assertEqual("rms" in map.keys(), "rms" in map2.keys())
        self.assertEqual("var" in map.keys(), "var" in map2.keys())
        self.assertEqual("FakeStatistic" in map.keys(),
                         "FakeStatistic" in map2.keys())

    def test_02_signal_stats_intern_statistic(self):
        # insert statics
        stats = SignalStats()
        self.assertFalse(stats.insert_statistics(""))
        self.assertTrue(len(stats.map()) == 0)

        self.assertTrue(stats.insert_statistics("maxAbs,rms"))
        self.assertEqual(len(stats.map()), 2)
        self.assertFalse(stats.insert_statistics("maxAbs,rms"))
        self.assertFalse(stats.insert_statistics("maxAbs"))
        self.assertFalse(stats.insert_statistics("rms"))
        self.assertEqual(len(stats.map()), 2)

        self.assertFalse(stats.insert_statistics("mean,FakeStatistic"))
        self.assertEqual(len(stats.map()), 3)

        self.assertFalse(stats.insert_statistics("var,FakeStatistic"))
        self.assertEqual(len(stats.map()), 4)

        self.assertFalse(stats.insert_statistics("max,FakeStatistic"))
        self.assertEqual(len(stats.map()), 5)

        self.assertFalse(stats.insert_statistics("min,FakeStatistic"))
        self.assertEqual(len(stats.map()), 6)

        self.assertFalse(stats.insert_statistics("FakeStatistic"))
        self.assertEqual(len(stats.map()), 6)

        # map with no data
        map = stats.map()
        self.assertFalse(len(map) == 0)
        self.assertEqual(len(map), 6)
        self.assertEqual("max" in map.keys(), 1)
        self.assertEqual("maxAbs" in map.keys(), 1)
        self.assertEqual("mean" in map.keys(), 1)
        self.assertEqual("min" in map.keys(), 1)
        self.assertEqual("rms" in map.keys(), 1)
        self.assertEqual("var" in map.keys(), 1)
        self.assertEqual("FakeStatistic" in map.keys(), 0)

    def test_signal_stats_alternating_values(self):
        # Add some statistics
        stats = SignalStats()
        self.assertTrue(stats.insert_statistics("max,maxAbs,mean,min,rms"))
        self.assertEqual(len(stats.map()), 5)

        # No data yet
        self.assertEqual(stats.count(), 0)

        # Insert data with alternating signs
        value = 3.14159
        stats.insert_data(value)
        stats.insert_data(-value)
        self.assertEqual(stats.count(), 2)

        map = stats.map()
        self.assertAlmostEqual(map["max"], value)
        self.assertAlmostEqual(map["maxAbs"], value)
        self.assertAlmostEqual(map["min"], -value)
        self.assertAlmostEqual(map["rms"], value)
        self.assertAlmostEqual(map["mean"], 0.0)

        # test operator=
        copy = SignalStats(stats)
        self.assertEqual(copy.count(), 2)
        map = stats.map()
        self.assertEqual(len(map), 5)
        self.assertAlmostEqual(map["max"], value)
        self.assertAlmostEqual(map["maxAbs"], value)
        self.assertAlmostEqual(map["min"], -value)
        self.assertAlmostEqual(map["rms"], value)
        self.assertAlmostEqual(map["mean"], 0.0)

        stats.reset()
        self.assertEqual(len(stats.map()), 5)
        self.assertEqual(stats.count(), 0)
        map = stats.map()
        self.assertAlmostEqual(map["max"], 0.0)
        self.assertAlmostEqual(map["maxAbs"], 0.0)
        self.assertAlmostEqual(map["min"], 0.0)
        self.assertAlmostEqual(map["rms"], 0.0)
        self.assertAlmostEqual(map["mean"], 0.0)


if __name__ == '__main__':
    unittest.main()
