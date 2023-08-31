# Copyright (C) 2023 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
from os.path import dirname, realpath
import threading

# Add "../__file__" to sys.path to get gz_test_deps
sys.path.append(dirname(dirname(realpath(__file__))))

from gz_test_deps.sim import Model
from gz_test_deps.transport import Node
from gz_test_deps.msgs.pose_pb2 import Pose
from gz_test_deps.msgs.clock_pb2 import Clock


# Test system to be used with test/integration/python_system_loader.cc
class TestModelSystem(object):
    def __init__(self):
        self.node = Node()
        self.has_been_reset = False
        self.lock = threading.Lock()
        self.sim_time_from_clock = None

    def configure(self, entity, sdf, ecm, event_mgr):
        self.model = Model(entity)
        if not self.model.valid(ecm):
            raise RuntimeError(f"Model {entity} is invalid")

        self.target_pose = sdf.get_pose("target_pose")
        self.reset_pose = sdf.get_pose("reset_pose")
        self.pub = self.node.advertise(f"{self.model.name(ecm)}/pose", Pose)
        self.sub = self.node.subscribe(Clock, "/clock", self.clock_cb)

    def pre_update(self, info, ecm):
        if info.paused or self.has_been_reset:
            return

        self.model.set_world_pose_cmd(ecm, self.target_pose)

    def post_update(self, info, ecm):
        msg = Pose()
        msg.position.x = self.target_pose.x()
        msg.position.y = self.target_pose.y()
        msg.position.z = self.target_pose.z()
        with self.lock:
            if self.sim_time_from_clock is not None:
                stamp = msg.header.stamp
                stamp.sec = self.sim_time_from_clock.sec
                stamp.nsec = self.sim_time_from_clock.nsec
        self.pub.publish(msg)

    def reset(self, info, ecm):
        self.model.set_world_pose_cmd(ecm, self.reset_pose)
        self.has_been_reset = True

    def clock_cb(self, msg):
        with self.lock:
            self.sim_time_from_clock = msg.sim


def get_system():
    return TestModelSystem()
