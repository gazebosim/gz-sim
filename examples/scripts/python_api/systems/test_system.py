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
            print(f"{self.id} {info.real_time} pre_update")

            self.link.add_world_force(
                ecm, Vector3d(0, 0, self.force),
                Vector3d(random.random(), random.random(), 0))


def get_system():
    return TestSystem()
