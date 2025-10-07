# Example for Reinforcement Learning (RL) With Gazebo

This demo world shows you an example of how you can use SDFormat, Stable Baselines 3 and Gazebo to perform RL with python.
We start with a very simple cart-pole world. This world is defined in our sdf file `cart_pole.sdf`. It is analogous to the cart-pole world in gymnasium.

## Create a VENV

First create a virtual environment using python,
```
python3 -m venv venv
```
Let's activate it and install stablebaselines3 and pytorch.
```
. venv/bin/activate
```

Lets install our dependencies
```
pip install stable-baselines3[extra]
```
For visualization to work you will also need to:
```
pip uninstall opencv-python
pip install opencv-python-headless
```
This is because `opencv-python` brings in Qt5 by default.

In the same terminal you should add your gazebo python install directory to the `PYTHONPATH`.
If you built gazebo from source in the current working directory this would be:
```
# cd to the colcon workspace where you built gazebo
cd <path_to_colcon_workspace>
export PYTHONPATH=$PYTHONPATH:`pwd`/install/lib/python
. install/setup.bash
```

You will also need to set PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION to python due to version
mismatches.
```
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
```


## Exploring the environment

You can see the environment by using `gz sim cart_pole.sdf`.

## Perform RL

To perform RL take a look at `cart_pole_env.py`. We simply subclass `gym.Env` and
create a new gazebo system. Close any instance of gazebo you may be running.
To run the script, in your terminal with the venv sourced run:
```
python3 cart_pole_env.py
```
