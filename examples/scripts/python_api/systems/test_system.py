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

from gz.math7 import Vector3d
from gz.sim8 import Model, Link
import random


class TestSystem(object):
    def __init__(self):
        self.id = random.randint(1, 100)

    def configure(self, entity, sdf, ecm, event_mgr):
        self.model = Model(entity)
        self.link = Link(self.model.canonical_link(ecm))
        print("Configured on", entity)
        print("sdf name:", sdf.get_name())
        self.force = sdf.get_double("force")
        print(f"Applying {self.force} N on link {self.link.name(ecm)}")

    def pre_update(self, info, ecm):
        if info.paused:
            return

        if info.iterations % 3000 == 0:
            self.link.add_world_force(
                ecm, Vector3d(0, 0, self.force),
                Vector3d(random.random(), random.random(), 0))


def get_system():
    return TestSystem()
