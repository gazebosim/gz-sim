#
# Copyright (C) 2023 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#
#
# In each iteration, for each vehicle, generate a random fin angle and thrust
# within reasonable limits, and send the command to the vehicle.
#
# Usage:
#   $ gz sim -r worlds/multi_lrauv_race.sdf
#   $ python3 multi_lrauv_race.py
#
# Note: You need to update the PYTHONPATH variable if it is not set
# before for other python examples. You can use then following:
# $ export PYTHONPATH=$PYTHONPATH:<path_to_gazebo_ws>/install/lib/python

from gz.msgs10.double_pb2 import Double
from gz.transport13 import Node

import random
import time


# Find joint limits from tethys model.sdf
def random_angle_within_limits(min=-0.261799, max=0.261799):
    return random.uniform(min, max)


# Nominal speed is thruster 300 rpm ~ 31.4 radians per second ~ 6.14 Newtons
def random_thrust_within_limits(min=-6.14, max=6.14):
    return random.uniform(min, max)


def main():
    # Set up node
    node = Node()

    # Set up publishers
    ns = ["tethys", "triton", "daphne"]
    rudder_pubs = [
        node.advertise(
            "/model/" + name + "/joint/vertical_fins_joint/0/cmd_pos", Double
        )
        for name in ns
    ]
    propeller_pubs = [
        node.advertise("/model/" + name + "/joint/propeller_joint/cmd_thrust", Double)
        for name in ns
    ]

    artificial_speedup = 1

    # Set up messages
    rudder_msg = Double()
    propeller_msg = Double()

    try:
        while True:
            for rudder_pub, propelled_pub, name in zip(rudder_pubs, propeller_pubs, ns):
                rudder_msg.data = random_angle_within_limits(-0.01, 0.01)
                rudder_pub.publish(rudder_msg)
                propeller_msg.data = random_thrust_within_limits(
                    -6.14 * artificial_speedup, 0
                )
                propelled_pub.publish(propeller_msg)
                print(
                    "Commanding: "
                    + name
                    + " ruddder angle "
                    + str(round(rudder_msg.data, 4))
                    + " rad, thrust "
                    + str(round(propeller_msg.data, 2))
                    + " Newtons."
                )
            print(
                "----------------------------------------------------------------------"
            )
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nProcess terminated")


if __name__ == "__main__":
    main()
