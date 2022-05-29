\page python_interfaces Python interfaces

# Overview

Gazebo provides a Python API to interact with world.

For now, we provide a `TestFixture` class that allows to load a world file,
step simulation and check entities and components.

 - **Step 1**: Load a world with a fixture

```{.python}
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
server.run(False, 1000, False)
while(server.is_running()):
    time.sleep(0.1)
```

# Run the example

In the
[examples/scripts/python_api](https://github.com/gazebosim/gz-sim/blob/ign-gazebo7/examples/scripts/python_api)
folder there is a Python script that shows how to make use of this API.

If you compiled Gazebo from source you should modify your `PYTHONPATH`:

```bash
export PYTHONPATH=$PYTHONPATH:<path to ws>/install/lib/python
```

Now you can run the example:

```bash
$ python3 examples/scripts/python_api/testFixture.py
[Msg] Loading SDF world file[/home/ahcorde/ignition_fortress/src/ign-gazebo/examples/scripts/python_api/gravity.sdf].
[Dbg] [Physics.cc:789] Loaded [gz::physics::dartsim::Plugin] from library [/home/ahcorde/ignition_fortress/install/lib/ign-physics-5/engine-plugins/libignition-physics-dartsim-plugin.so]
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
AddSystem1
World entity is  1
Gravity  0 0 -9.8
Entity for falling model is:  4
AddSystem2
[Msg] Found no publishers on /stats, adding root stats topic
[Msg] Found no publishers on /clock, adding root clock topic
[Dbg] [SimulationRunner.cc:524] Creating PostUpdate worker threads: 2
[Dbg] [SimulationRunner.cc:537] Creating postupdate worker thread (0)
iterations  1000
post_iterations  1000
pre_iterations  1000
```
