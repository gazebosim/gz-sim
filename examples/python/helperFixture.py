#!/usr/bin/python3

import os
import time

from ignition.common import set_verbosity
from ignition.gazebo import HelperFixture, World
from ignition.math import Vector3d


set_verbosity(4)

file_path = os.path.dirname(os.path.realpath(__file__))

helper = HelperFixture(os.path.join(file_path, 'gravity.sdf'))

post_iterations = 0
iterations = 0
pre_iterations = 0


def on_configure_cb(worldEntity, _ecm):
    print('World entity is ', worldEntity)
    w = World(worldEntity)
    v = w.gravity(_ecm)
    print('Gravity ', v)
    modelEntity = w.model_by_name(_ecm, 'falling')
    print('Entity for falling model is: ', modelEntity)


def on_post_udpate_cb(_info, _ecm):
    global post_iterations
    post_iterations += 1
    # print(_info.sim_time)


def on_pre_udpate_cb(_info, _ecm):
    global pre_iterations
    pre_iterations += 1


def on_udpate_cb(_info, _ecm):
    global iterations
    iterations += 1


helper = helper.on_post_update(on_post_udpate_cb)
helper = helper.on_update(on_udpate_cb)
helper = helper.on_pre_update(on_pre_udpate_cb)
helper = helper.on_configure(on_configure_cb)

helper = helper.finalize()

server = helper.server()
server.run(False, 1000, False)

while(server.is_running()):
    time.sleep(0.1)

print('iterations ', iterations)
print('post_iterations ', post_iterations)
print('pre_iterations ', pre_iterations)
