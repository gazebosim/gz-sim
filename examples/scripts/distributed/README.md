# Distributed simulation demo

This demo shows how to run a simulation that is distributed across several
processes or even machines.

The demo world consists of 3 shapes rolling downhill. Each shape's physics is
calculated by a "secondary" simulator, while one "primary" simulator keeps them
in sync and displays the unified information to the GUI.

## Try it out

1. On one terminal, start the simulation primary:

        ./primary.sh

    An empty window will appear. Simulation won't be initialized until all 3
    secondaries are initialized.

1. Open 3 other terminals and run the following to start a simulation secondary for each:

        ./secondary.sh

    Once you've started all 3 secondaries, you'll see the 3 shapes show up on the window.

1. Press play and you should see the shapes rolling down.

