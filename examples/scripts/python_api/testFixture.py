#!/usr/bin/python3
# Copyright (C) 2021 Open Source Robotics Foundation
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

# If you compiled Gazebo from source you should modify your
# `PYTHONPATH`:
#
# export PYTHONPATH=$PYTHONPATH:<path to ws>/install/lib/python
#
# Now you can run the example:
#
# python3 examples/scripts/python_api/testFixture.py

import os

from gz.common5 import set_verbosity
from gz.sim8 import TestFixture, World, world_entity
from gz.math7 import Vector3d

set_verbosity(4)

file_path = os.path.dirname(os.path.realpath(__file__))

fixture = TestFixture(os.path.join(file_path, 'gravity.sdf'))

post_iterations = 0
iterations = 0
pre_iterations = 0
first_iteration = True


def on_pre_udpate_cb(_info, _ecm):
    global pre_iterations
    global first_iteration
    pre_iterations += 1
    if first_iteration:
        first_iteration = False
        world_e = world_entity(_ecm)
        print('World entity is ', world_e)
        w = World(world_e)
        v = w.gravity(_ecm)
        print('Gravity ', v)
        modelEntity = w.model_by_name(_ecm, 'falling')
        print('Entity for falling model is: ', modelEntity)


def on_udpate_cb(_info, _ecm):
    global iterations
    iterations += 1


def on_post_udpate_cb(_info, _ecm):
    global post_iterations
    post_iterations += 1
    if _info.sim_time.seconds == 1:
        print('Post update sim time: ', _info.sim_time)


fixture.on_post_update(on_post_udpate_cb)
fixture.on_update(on_udpate_cb)
fixture.on_pre_update(on_pre_udpate_cb)
fixture.finalize()

server = fixture.server()
server.run(True, 1000, False)

print('iterations ', iterations)
print('post_iterations ', post_iterations)
print('pre_iterations ', pre_iterations)
