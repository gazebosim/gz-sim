\page python_interfaces Python interfaces

## Overview

Gazebo provides a Python API to interact with the world.

For now, we provide a `TestFixture` class that allows to load a world file,
step simulation and check entities and components.

 - **Step 1**: Load a world with a fixture

```python
file_path = os.path.dirname(os.path.realpath(__file__))
fixture = TestFixture(os.path.join(file_path, 'gravity.sdf'))
```

 - **Step 2**: Write your `preupdate`, `update` or `postupdate` code:

```python
def on_post_udpate_cb(_info, _ecm):
  # <your code here>
  ...
```

 - **Step 3**: Register the function.

```python
fixture.on_post_update(on_post_udpate_cb)
```

  - **Step 4**: Be sure to call finalize before running the server.

```python
fixture.finalize()
```

  - **Step 5**: Run the server

```python
server.run(True, 1000, False)
```

## Run the example

In the
[examples/scripts/python_api](https://github.com/gazebosim/gz-sim/tree/gz-sim8/examples/scripts/python_api)
folder there is a Python script that shows how to make use of this API.

If you compiled Gazebo from source you should modify your `PYTHONPATH`:

```bash
export PYTHONPATH=$PYTHONPATH:<path to ws>/install/lib/python
```

Now you can run the example:

```bash
$ python3 examples/scripts/python_api/testFixture.py
[Msg] Loading SDF world file[/home/ahcorde/gz_fortress/src/ign-gazebo/examples/scripts/python_api/gravity.sdf].
[Dbg] [Physics.cc:789] Loaded [gz::physics::dartsim::Plugin] from library [/home/ahcorde/gz_fortress/install/lib/gz-physics-5/engine-plugins/libgz-physics-dartsim-plugin.so]
[Dbg] [SimulationRunner.cc:909] Loaded system [gz::sim::systems::Physics] for entity [1]
[Msg] Loaded level [3]
[Msg] Serving world controls on [/world/gravity/control], [/world/gravity/control/state] and [/world/gravity/playback/control]
[Msg] Serving GUI information on [/world/gravity/gui/info]
[Msg] World [gravity] initialized with [default_physics] physics profile.
[Msg] Serving world SDF generation service on [/world/gravity/generate_world_sdf]
[Msg] Serving world names on [/gazebo/worlds]
[Msg] Resource path add service on [/gazebo/resource_paths/add].
[Msg] Resource path get service on [/gazebo/resource_paths/get].
[Msg] Resource paths published on [/gazebo/resource_paths].
[Msg] Found no publishers on /stats, adding root stats topic
[Msg] Found no publishers on /clock, adding root clock topic
[Dbg] [SimulationRunner.cc:524] Creating PostUpdate worker threads: 2
[Dbg] [SimulationRunner.cc:537] Creating postupdate worker thread (0)
AddSystem1
World entity is  1
Gravity  0 0 -9.8
Entity for falling model is:  4
AddSystem2
iterations  1000
post_iterations  1000
pre_iterations  1000
```

# Gazebo Systems written in Python

Gazebo also provides a way to write systems in Python. This is done using the
`gz::sim::systems::PythonSystemLoader` system which loads a given python module
specified by its `<module_name>` parameter. The search path for the module
includes `GZ_SIM_SYSTEM_PLUGIN_PATH` as well as `PYTHONPATH`. The module is
expected to provide a function called `get_system` that returns an instance of
a class that implements the various interfaces in `gz::sim::System`.

Example python system:

<!-- TODO(azeey) Allow including python files in doxygen -->
<!-- \include examples/scripts/python_api/systems/test_system.py -->
```python
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
```

The system can be added to SDFormat model or world with:

```xml
<plugin filename="gz-sim-python-system-loader-system"
        name="gz::sim::systems::PythonSystemLoader">
    <module_name>test_system</module_name>
    <force>100</force>
    <!-- Extra xml parameters to pass to the module -->
</plugin>
```

asuming the name of the module is `test_system` and the directory containing
the module has been added to `GZ_SIM_SYSTEM_PLUGIN_PATH`,
