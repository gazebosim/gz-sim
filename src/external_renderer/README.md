# External Renderer for Gazebo (Zenoh + Flatbuffers)

This directory contains a prototype for an external renderer that receives position updates over Zenoh and renders a red box using `gz-rendering`.

## Components

1.  **Producer App**: Generates a moving position (circular motion) and publishes it to the `external_renderer/position` topic using Zenoh and Flatbuffers.
2.  **Renderer App**: Initializes a `gz-rendering` scene with Ogre2, creates a camera and a red box, and updates the box's position based on received Zenoh messages.
3.  **Schema**: Defines the `Position` Flatbuffers table.

## Prerequisites

- `gz-rendering` (version corresponding to your Gazebo installation)
- `zenoh-cpp`
- `Flatbuffers` (`flatc` and libraries)
- `GLUT` (already a dependency in `package.xml`)
- `OpenGL`

## Building

```bash
mkdir build
cd build
cmake ..
make
```

## Running

1.  Start the renderer:
    ```bash
    ./renderer_app
    ```
2.  In another terminal, start the producer:
    ```bash
    ./producer_app
    ```

The red box in the renderer should start moving in a circle.
