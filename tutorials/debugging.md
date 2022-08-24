\page debugging Debugging

## Using GDB with the command line tools

The Gazebo command line tools are based on Ruby. If you want to use GDB to
debug a problem, which we highly encourage, then you'll need to run GDB against
the Ruby executable. Once in the GDB shell, you can run the `gz sim`
script to run an instance of Gazebo.

You'll likely want to debug the Gazebo server or GUI separately. Refer to the
following two sections for instructions concerning each case.

### Debugging the server

1. Find the `gz` executable:

    `which gz`

    The following instructions will assume the output was `/usr/bin/gz`, be
    sure to change it accordingly.

2. Launch `gdb`

    `gdb ruby`

3. Run the Gazebo server with the desired arguments. Make sure to use the
   `-s` argument. The following example runs the `shapes.sdf` world.

    `(gdb) r /usr/bin/gz sim -s shapes.sdf`

4. Use GDB as normal

### Debugging the GUI

1. Launch `gdb`

    `gdb ruby`

2. Run the Gazebo GUI with the desired arguments. Make sure to use the
   `-g` argument.

    `(gdb) r /usr/bin/gz sim -g`

3. In another terminal run the Gazebo server.

    `gz sim -s -v 4 -r shapes.sdf`

4. Use GDB as normal.
