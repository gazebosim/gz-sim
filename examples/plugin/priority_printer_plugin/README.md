# Priority Printer

This example illustrates how to control the order of execution of System
PreUpdate and Update callbacks.

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

Multiple instances of the `PriorityPrinter` plugin are added to the world
with various priority values and unique labels in the
`priority_printer_plugin.sdf` file that's going to be loaded.

Before starting Gazebo, we must make sure it can find the plugin by doing:

~~~
cd gz-sim/examples/plugins/priority_printer
export GZ_SIM_SYSTEM_PLUGIN_PATH=`pwd`/build
~~~

Then load the example world and run for 5 iterations:

    gz sim -v 3 priority_printer_plugin.sdf -s -r --iterations 5

You should see green messages on the terminal like:

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
