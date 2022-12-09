# Distributed simulation demo

This demo shows how to run a simulation that is distributed across several
processes or even machines.

The demo world consists of 4 shapes rolling downhill. The sphere and box models
have one link each, and the cylinder model has 2 cylinder links, making a total of
3 models.

Each model's physics is calculated by a "secondary" simulator, while one
"primary" simulator keeps them in sync and displays the unified information to
the GUI.

## Try it out

1. Open 3 terminals and run the following to start a simulation secondary for
   each performer:

        ./secondary.sh

1. On another terminal, start the simulation primary:

        ./primary.sh

    A window will pop up displaying the 3 shapes.

1. Press play and you should see the shapes rolling down.

## Standalone

You can run the same world in a standalone process as follows:

    ./standalone.sh
