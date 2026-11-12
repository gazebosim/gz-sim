\page performance_monitor Finding what limits the real time factor

When a simulation runs slower than real time, the performance monitor tells
what is limiting it: the CPU or the GPU, in which part of the simulation, and
which sensor or model is responsible.

## Enabling the monitor

The monitor is off by default and costs nothing noticeable while off. Set
`GZ_SIM_PERFORMANCE_MONITOR` to the number of seconds between reports:

```bash
GZ_SIM_PERFORMANCE_MONITOR=5 gz sim -s -r my_world.sdf
```

Or start and stop it in a simulation that is already running:

```bash
gz service -s /world/my_world/performance/enable \
  --reqtype gz.msgs.Boolean --reptype gz.msgs.Boolean \
  --timeout 3000 --req 'data: true'
```

Rendering sensors only do work while something subscribes to them, so run the
simulation with the subscribers that you care about, like your ROS bridge.

Only one world per process can be monitored at a time. If a server runs several
worlds, the first one to start monitoring is the one monitored.

## Reading a report

The reports are printed to the console, whatever the verbosity, and published
as `gz.msgs.Diagnostics` on `/world/<world name>/performance/report`.

```
Performance report, last 5.0 s, 2417 steps: RTF 0.48 (target 1.00)
  Bottleneck: GPU, in rendering: big_cam::link::camera
  Step time 2.05 ms: rendering wait 1.74 ms (85%), physics 0.05 ms (2%), ...
  Render thread 8.12 ms/frame (1201 frames): GPU wait 7.32 ms (90%), ...
  Sensors, slowest first:
    big_cam::link::camera: 8.01 ms/update x 1201 (CPU 0.71 ms, GPU wait 7.30 ms)
  Models, biggest load first (physics and scene are estimated):
    big_cam: sensors 1.99 ms/step, plugins 0.00 ms/step, physics ~0.00 ms/step, scene ~0.00 ms/step (0 DOF, 0 contacts, 0 visuals)
  ...
```

The bottleneck line has a resource and a location:

| Resource | Meaning |
|---|---|
| `CPU` | Computation is the limit. Software rasterizers such as llvmpipe count here and the report says so. |
| `GPU` | The render thread spends most of its time (65% or more) blocked on the GPU, which includes copying images out of GPU memory. |
| `CPU and GPU` | The render thread spends comparable time computing and blocked on the GPU, so improving only one of them gains little. High resolution cameras in simple scenes usually end up here. |
| not computing | A system is blocked waiting for something outside of the simulation, like a socket or a lock-step partner. |
| unknown | Rendering is the limit, but the render engine does not report its GPU waits. |

| Location | Meaning |
|---|---|
| `physics` | The physics engine step. |
| `rendering` | The simulation waits for the render thread to finish the sensors. |
| `system` | A system other than physics and sensors, it is named in the report. |
| `engine` | The simulation loop, outside of any system. |

## How it measures

The monitor discards the first seconds, then accumulates over each interval:

- The time of each call into each system. When post updates run in parallel
  only the slowest one of each step counts, since it is the one delaying the
  step.
- The time of each sensor update, for all sensors.
- The stages of the render thread, and how much of them the CPU spent blocked
  on the GPU. Render engines add up the time spent at the points where the CPU
  has to wait for the GPU, like reading back a rendered image. GPU work that
  completes while the CPU is still busy isn't counted because it isn't limiting
  anything, so a GPU at 100% utilization that keeps up is not reported as the
  bottleneck.

The simulation counts as limited when it runs below 97% of its target real time
factor. A simulation with a target of 0 runs as fast as it can, so it always
has a bottleneck.

With a target real time factor, each step has a slot of wall time, the step
size divided by the target. A step that fits in its slot costs no real time:
the loop sleeps the rest of the slot away. A step that runs past its slot puts
the loop behind its schedule, and faster steps after it can make up for that.
Only when the loop falls more than a slot behind does it give up on the
schedule and start a new one, and the time that it was behind is lost for
good. That lost time is what lowers the RTF, so a limited simulation with a
target is blamed on it, and the report adds a `Time past the slot` line that
splits it among the systems that put the loop behind.
This matters when the load is bursty: physics that takes 0.45 ms of every
1 ms slot is the biggest part of the step time but costs nothing, while a
render stall of 11 ms in a few steps per second costs all of the lost time.
Likewise a system that runs 0.5 ms past the slot every other step costs
nothing if the step after it is fast enough to make up for it.
Without a target the two measures are the same.

## What is an estimate

Sensor, system and render thread times are measured. The share of each model in
the physics step is not: the physics engines step the whole world at once. It is
estimated from the degrees of freedom of the parts of the model that can move,
its collision shapes, how many contact points the moving parts have, and the
triangles of the moving meshes that are touching something. The report marks it
as such. Treat it as a pointer on where to look first. The weights were measured
on DART, one of them on a single case, and the contacts of a model are assumed to
be solved together, so a pile of many separate models that touch each other is
underestimated.

A model is only named as the physics bottleneck when it holds more than half of
the estimate. Otherwise the report says that no model dominates and gives the
largest share, since the cost is spread over the world and removing that one
model would gain little.

The `scene` share of each model is also an estimate: the time that the render
thread spends on the scene before drawing is split by the number of visuals.

The real time factor is also determined by more than one thing at a time.
Removing the bottleneck that is reported can reveal the next one, which you can
see in the percentages of the step time.
