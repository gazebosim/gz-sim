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
# Check README for detailed instructions.
# Note: You need to update the PYTHONPATH variable if it is not set
# before for other python examples. You can use then following:
# $ export PYTHONPATH=$PYTHONPATH:<path_to_gazebo_ws>/install/lib/python
# Usage:
#   $ gz sim -r worlds/lrauv_control_demo.sdf
#   $ # python3 lrauv_control.py speed_m_s yaw_rad pitch_rad
#   $ python3 lrauv_control.py 0.5 0.78 0.174
#

from gz.msgs10.double_pb2 import Double
from gz.msgs10.odometry_pb2 import Odometry
from gz.math7 import Quaterniond, Vector3d
from gz.transport13 import Node

from threading import Lock

import math
import sys
import time


# Helper class for the PID controller for
# linear speed, pitch and yaw angles.
class Controller:
    def __init__(self):
        # Mutex to synchronize internal variables.
        self.controllerMutex = Lock()

        # Desired state.
        self.targetSpeed = 0
        self.targetYawAngle = 0
        self.targetPitchAngle = 0

        # Errors
        self.errorSpeed = 0
        self.errorSpeedIntegral = 0
        self.errorYawAngle = 0
        self.errorPitchAngle = 0

        # States to be tracked and controlled.
        self.speed = 0
        self.yawAngle = 0
        self.pitchAngle = 0

        # PID gains and error limits.
        # PI for speed.
        self.kSpeed = -30
        self.kISpeed = -0.5
        self.errorSpeedIntegralMax = 10

        # P for yaw and pitch control.
        self.kYawAngle = -0.5
        self.kPitchAngle = 0.6

    # Set the target states to be tracked,
    # i.e. linear speed (m/s), pitch and yaw angles (rad).
    def SetTargets(self, _speed, _yaw, _pitch):
        with self.controllerMutex:
            if _speed == 0 and (_yaw != 0 or _pitch != 0):
                print("Speed needs to be non zero for non zero pitch and yaw angles")
                return
            self.targetSpeed = float(_speed)
            self.targetYawAngle = float(_yaw)
            self.targetPitchAngle = float(_pitch)

    # Update the state of the vehicle.
    def UpdateState(self, _speed, _yaw, _pitch):
        with self.controllerMutex:
            self.speed = _speed
            self.yawAngle = _yaw
            self.pitchAngle = _pitch

            self.errorSpeed = self.targetSpeed - self.speed
            self.errorSpeedIntegral = min(
                self.errorSpeedIntegral + self.errorSpeed, self.errorSpeedIntegralMax
            )
            self.errorYawAngle = self.targetYawAngle - self.yawAngle
            self.errorPitchAngle = self.targetPitchAngle - self.pitchAngle

    # Generate control input to be applied to the thruster.
    def SpeedControl(self):
        return self.errorSpeed * self.kSpeed + self.errorSpeedIntegral * self.kISpeed

    # Generate control input to be supplied to the yaw rudders.
    def YawControl(self):
        return self.errorYawAngle * self.kYawAngle

    # Generate control input to be supplied to the pitch rudders.
    def PitchControl(self):
        return self.errorPitchAngle * self.kPitchAngle


def main():
    control = Controller()
    argc = len(sys.argv)
    if argc == 4:
        targetSpeed = sys.argv[1]
        targetYaw = sys.argv[2]
        targetPitch = sys.argv[3]

        # Target state : speed (m/s), yaw angle, pitch angle (rad).
        control.SetTargets(targetSpeed, targetYaw, targetPitch)

    node = Node()

    # Propeller command publisher.
    propellerTopicTethys = "/model/tethys/joint/propeller_joint/cmd_thrust"
    propellerPubTethys = node.advertise(propellerTopicTethys, Double)

    # Subscriber for vehicle pose.
    def cbPos(odometry_msg: Odometry):
        orientation = odometry_msg.pose.orientation

        q = Quaterniond(orientation.w, orientation.x, orientation.y, orientation.z)

        # Get the velocity of the vehicle.
        velocity = Vector3d(
            odometry_msg.twist.linear.x,
            odometry_msg.twist.linear.y,
            odometry_msg.twist.linear.z,
        )

        control.UpdateState(velocity.length(), q.yaw(), q.pitch())

    node.subscribe(Odometry, "/model/tethys/odometry", cbPos)

    # Rudder command publisher.
    rudderTopicTethys = "/model/tethys/joint/vertical_fins_joint/0/cmd_pos"
    rudderPubTethys = node.advertise(rudderTopicTethys, Double)

    # Fin command publisher.
    finTopicTethys = "/model/tethys/joint/horizontal_fins_joint/0/cmd_pos"
    finPubTethys = node.advertise(finTopicTethys, Double)

    while True:
        with control.controllerMutex:
            # Publish propeller command for speed.
            propellerMsg = Double()
            propellerMsg.data = control.SpeedControl()
            propellerPubTethys.publish(propellerMsg)

            # Publish rudder command for yaw.
            rudderMsg = Double()
            rudderMsg.data = control.YawControl()
            rudderPubTethys.publish(rudderMsg)

            # Publish fin command for pitch.
            finMsg = Double()
            finMsg.data = control.PitchControl()
            finPubTethys.publish(finMsg)

            # Print the states.
            print("-----------------------")
            print("States            ( target, current, error) : ")
            print(
                "Speed (m/s)       :  ",
                str(round(control.targetSpeed, 2)),
                "  ",
                str(round(float(control.speed), 2)),
                "  ",
                str(round(float(control.errorSpeed), 2)),
            )
            print(
                "Yaw angle (deg)   : ",
                str(round(control.targetYawAngle * 180 / math.pi, 2)),
                " ",
                str(round(control.yawAngle * 180 / math.pi, 2)),
                "  ",
                str(round(control.errorYawAngle * 180 / math.pi, 2)),
            )
            print(
                "Pitch angle (deg) : ",
                str(round(control.targetPitchAngle * 180 / math.pi, 2)),
                " ",
                str(round(control.pitchAngle * 180 / math.pi, 2)),
                "  ",
                str(round(control.errorPitchAngle * 180 / math.pi, 2)),
            )
        time.sleep(0.2)


if __name__ == "__main__":
    main()
