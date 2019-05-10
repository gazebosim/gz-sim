# Distributed simulation demo

This demo shows how to run a simulation using both the distribution and the
levels features.

The demo world consists of 2 shapes rolling down a hill that is composed of
various blocks in a sequence. Each block is in a separate level, and is
enabled as a shape approaches it. The levels' volumes are represented by
dark blue boxes, and their buffer zones by light blue ones.

Each shape's physics is calculated by a "secondary" simulator, while one
"primary" simulator keeps them in sync and displays the unified information to
the GUI.

## Try it out

1. Open 2 terminals and run the following to start a simulation secondary for
   each performer:

        ./secondary.sh

1. On another terminal, start the simulation primary:

        ./primary.sh

    A window will pop up displaying the 2 shapes.

1. Press play and you should see the levels being loaded and unloaded as the
   shapes roll down.

## Standalone

You can run the same world in a standalone process as follows:

    ./standalone.sh

# Generating worlds

The SDF files in this demo have been generated from an ERB template.
This is how to generate each file:

* Generate the primary world:

        erb type=primary distributed_levels.sdf.erb > primary.sdf

* Generate the secondary world:

        erb type=secondary distributed_levels.sdf.erb > secondary.sdf

* Generate a standalone world that can be run without `--distributed`:

        erb type=standalone distributed_levels.sdf.erb > standalone.sdf
