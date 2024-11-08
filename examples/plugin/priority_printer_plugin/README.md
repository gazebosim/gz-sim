# Priority Printer

This example illustrates how to control the order of execution of System
PreUpdate and Update callbacks. As documented in
[gz/sim/System.hh](https://github.com/gazebosim/gz-sim/tree/main/include/gz/sim/System.hh),
the PreUpdate and Update phases are executed sequentially in the same
thread, and the order of execution of these phases can be
controlled by specifying a signed integer priority value for the System
in its XML configuration. The default priority value is zero, and
smaller values are executed earlier. Systems with the same priority
value are executed in the order in which they are loaded.

## Build

From the root of the `gz-sim` repository, do the following to build the example:

~~~
cd gz-sim/examples/plugins/priority_printer
mkdir build
cd build
cmake ..
make
~~~

This will generate the `PriorityPrinter` library under `build`.

## Run

Multiple instances of the `PriorityPrinter` plugin are added to the
[priority\_printer\_plugin.sdf](priority_printer_plugin.sdf) world file
with various priority values and unique labels corresponding to the order
in which the plugins are specified ("first" for the first plugin and so on).
Without priority values, the systems would be executed in the order they are
specified in XML ("first", then "second", etc.).
With the priority values specified, the systems with smallest integer priority
values are executed first. For systems with the same priority value, the
system that is specified earlier in the XML file will be executed first.

Before starting Gazebo, we must make sure it can find the plugin by doing:

~~~
cd gz-sim/examples/plugins/priority_printer
export GZ_SIM_SYSTEM_PLUGIN_PATH=`pwd`/build
~~~

Then load the example world and run for 5 iterations:

    gz sim -v 3 priority_printer_plugin.sdf -s -r --iterations 5

You should see green messages on the terminal like those given below.
Note that the system with priority `-100` was executed first, despite being
the fifth system in the XML ordering. There are two instances of systems with
the same priority value: the fourth and sixth systems with priority 0 (with
"unset" defaulting to 0) and the first and seventh systems with priority 100.
In each case, the system declared earlier in XML executed first.

```
[Msg] PreUpdate: Iteration 1, system priority -100, system label fifth
[Msg] PreUpdate: Iteration 1, system priority -10, system label third
[Msg] PreUpdate: Iteration 1, system priority unset, system label fourth
[Msg] PreUpdate: Iteration 1, system priority 0, system label sixth
[Msg] PreUpdate: Iteration 1, system priority 10, system label second
[Msg] PreUpdate: Iteration 1, system priority 100, system label first
[Msg] PreUpdate: Iteration 1, system priority 100, system label seventh
[Msg] Update: Iteration 1, system priority -100, system label fifth
[Msg] Update: Iteration 1, system priority -10, system label third
[Msg] Update: Iteration 1, system priority unset, system label fourth
[Msg] Update: Iteration 1, system priority 0, system label sixth
[Msg] Update: Iteration 1, system priority 10, system label second
[Msg] Update: Iteration 1, system priority 100, system label first
[Msg] Update: Iteration 1, system priority 100, system label seventh
[Msg] PreUpdate: Iteration 2, system priority -100, system label fifth
[Msg] PreUpdate: Iteration 2, system priority -10, system label third
[Msg] PreUpdate: Iteration 2, system priority unset, system label fourth
[Msg] PreUpdate: Iteration 2, system priority 0, system label sixth
[Msg] PreUpdate: Iteration 2, system priority 10, system label second
[Msg] PreUpdate: Iteration 2, system priority 100, system label first
[Msg] PreUpdate: Iteration 2, system priority 100, system label seventh
[Msg] Update: Iteration 2, system priority -100, system label fifth
[Msg] Update: Iteration 2, system priority -10, system label third
[Msg] Update: Iteration 2, system priority unset, system label fourth
[Msg] Update: Iteration 2, system priority 0, system label sixth
[Msg] Update: Iteration 2, system priority 10, system label second
[Msg] Update: Iteration 2, system priority 100, system label first
[Msg] Update: Iteration 2, system priority 100, system label seventh
[Msg] PreUpdate: Iteration 3, system priority -100, system label fifth
[Msg] PreUpdate: Iteration 3, system priority -10, system label third
[Msg] PreUpdate: Iteration 3, system priority unset, system label fourth
[Msg] PreUpdate: Iteration 3, system priority 0, system label sixth
[Msg] PreUpdate: Iteration 3, system priority 10, system label second
[Msg] PreUpdate: Iteration 3, system priority 100, system label first
[Msg] PreUpdate: Iteration 3, system priority 100, system label seventh
[Msg] Update: Iteration 3, system priority -100, system label fifth
[Msg] Update: Iteration 3, system priority -10, system label third
[Msg] Update: Iteration 3, system priority unset, system label fourth
[Msg] Update: Iteration 3, system priority 0, system label sixth
[Msg] Update: Iteration 3, system priority 10, system label second
[Msg] Update: Iteration 3, system priority 100, system label first
[Msg] Update: Iteration 3, system priority 100, system label seventh
[Msg] PreUpdate: Iteration 4, system priority -100, system label fifth
[Msg] PreUpdate: Iteration 4, system priority -10, system label third
[Msg] PreUpdate: Iteration 4, system priority unset, system label fourth
[Msg] PreUpdate: Iteration 4, system priority 0, system label sixth
[Msg] PreUpdate: Iteration 4, system priority 10, system label second
[Msg] PreUpdate: Iteration 4, system priority 100, system label first
[Msg] PreUpdate: Iteration 4, system priority 100, system label seventh
[Msg] Update: Iteration 4, system priority -100, system label fifth
[Msg] Update: Iteration 4, system priority -10, system label third
[Msg] Update: Iteration 4, system priority unset, system label fourth
[Msg] Update: Iteration 4, system priority 0, system label sixth
[Msg] Update: Iteration 4, system priority 10, system label second
[Msg] Update: Iteration 4, system priority 100, system label first
[Msg] Update: Iteration 4, system priority 100, system label seventh
[Msg] PreUpdate: Iteration 5, system priority -100, system label fifth
[Msg] PreUpdate: Iteration 5, system priority -10, system label third
[Msg] PreUpdate: Iteration 5, system priority unset, system label fourth
[Msg] PreUpdate: Iteration 5, system priority 0, system label sixth
[Msg] PreUpdate: Iteration 5, system priority 10, system label second
[Msg] PreUpdate: Iteration 5, system priority 100, system label first
[Msg] PreUpdate: Iteration 5, system priority 100, system label seventh
[Msg] Update: Iteration 5, system priority -100, system label fifth
[Msg] Update: Iteration 5, system priority -10, system label third
[Msg] Update: Iteration 5, system priority unset, system label fourth
[Msg] Update: Iteration 5, system priority 0, system label sixth
[Msg] Update: Iteration 5, system priority 10, system label second
[Msg] Update: Iteration 5, system priority 100, system label first
[Msg] Update: Iteration 5, system priority 100, system label seventh
```
