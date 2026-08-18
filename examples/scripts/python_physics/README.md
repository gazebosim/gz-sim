# Python Physics Examples

> [!WARNING]
> This is a super basic integration and is intended for experimental purposes only.

This directory contains examples of using Python to implement physics or integrate external physics engines with Gazebo Sim.

## Contents

- `pbd_physics.py`: A Position Based Dynamics implementation in Python.
- `newton_physics.py`: Integration with the Newton physics engine.
- `genesis_physics.py`: Integration with the Genesis physics engine.

## Setup Instructions

To run the Genesis or Newton examples, you should create a virtual environment and install the necessary packages.

> [!IMPORTANT]
> The Python version in the virtual environment MUST match the version used to compile the Gazebo Python bindings. (These instructions assume Python 3.13 is being used).

1. Create a virtual environment:
   ```bash
   python3 -m venv .venv
   ```
2. Activate the virtual environment and install dependencies (e.g., for Genesis):
   ```bash
   source .venv/bin/activate
   # Install genesis and other required packages here
   ```

### Environment Variables

You need to set the `PYTHONPATH` to include:
1. The `site-packages` of your virtual environment (where Genesis/Newton are installed).
2. The Gazebo Python install path in your colcon install space.
3. This directory (so that Gazebo can find the Python modules).

## Running the Examples

Run these commands from the `src/gz-sim` directory or adjust the paths to the SDF files accordingly.

### 1. PBD Physics
```bash
PYTHONPATH=<path_to_colcon_install>/lib/python:examples/scripts/python_physics gz sim examples/scripts/python_physics/pbd_world.sdf -v4
```

### 2. Newton Physics
```bash
PYTHONPATH=<path_to_newton_venv>/lib/python3.13/site-packages:<path_to_colcon_install>/lib/python:examples/scripts/python_physics DYLD_LIBRARY_PATH=<path_to_colcon_install>/lib gz sim examples/scripts/python_physics/python_physics.sdf -v4
```

### 3. Genesis Physics
```bash
PYTHONPATH=<path_to_genesis_venv>/lib/python3.13/site-packages:<path_to_colcon_install>/lib/python:examples/scripts/python_physics DYLD_LIBRARY_PATH=<path_to_colcon_install>/lib gz sim examples/scripts/python_physics/genesis_world.sdf -v4
```
