from gz.sim8 import UpdateInfo, EntityComponentManager, Model
from gz.math7 import Vector3d, Pose3d
import random

class TestSystem(object):
    def __init__(self):
        self.id = random.randint(1, 100)

    def configure(self, entity):
        self.model = Model(entity)
        print("Configured on", entity)

    def pre_update(self, info, ecm):
        if info.paused:
            return

        if info.iterations % 3000 == 0:
            print(f"{self.id} {info.real_time} pre_update")
            self.model.set_world_pose_cmd(ecm, Pose3d(0, 0, 5, 0,0,0))


def get_system():
    return TestSystem()
