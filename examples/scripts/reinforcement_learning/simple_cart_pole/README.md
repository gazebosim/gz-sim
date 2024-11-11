# Example for Reinforcement Learning (RL) With Gazebo

This demo world shows you an example of how you can use SDFormat, Ray-RLLIB and Gazebo to perform RL with python.
We start with a very simple cart-pole world. This world is defined in our sdf file `cart_pole.sdf`. It is analogous to
the

## Create a VENV

First create a virtual environment using python,
```
python3 -m venv venv
```
Lets activate it and install rayrllib and pytorch.
```
. venv/bin/activate
```

Lets install our dependencies
```
pip install stable-baselines3[extra]
```

In the same terminal you should add your gazebo python install directory to the `PYTHONPATH`
If you built gazebo from source in the current working directory this would be:
```
export PYTHONPATH=$PYTHONPATH:install/lib/python
```

You will also need to set PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION to python due to version
mis-matches.
```
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
```

## Exploring the environment

You can see the environment by using `gz sim cart_pole.sdf`.