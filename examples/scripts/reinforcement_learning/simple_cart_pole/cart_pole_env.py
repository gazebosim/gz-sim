
import os
import gymnasium as gym
import numpy as np

from gz.common import set_verbosity
from gz.sim import TestFixture, World, world_entity, Model, Link, get_install_prefix
from gz.math import Vector3d

from stable_baselines3 import PPO
import time
import subprocess

file_path = os.path.dirname(os.path.realpath(__file__))

def run_gui():
    """
    This function looks for your gz sim installation and looks for
    an instance of the gui client
    """
    if os.name == 'nt':
        base = os.path.join(get_install_prefix(), "libexec", "gz", "sim10","gz-sim-gui-client.exe")
    else:
        base = os.path.join(get_install_prefix(), "libexec", "gz", "sim10", "gz-sim-gui-client")
    subprocess.Popen(base)

class GzRewardScorer:
    """
    This Gazebo System is used to introspect and score the world.
    """
    def __init__(self):
        """
        We initialize a TestFixture: This is a simple fixture that is used
        to load our gazebo world. We also inject the code to be executed
        on each run.
        """
        self.fixture = TestFixture(os.path.join(file_path, 'cart_pole.sdf'))
        self.fixture.on_pre_update(self.on_pre_update)
        self.fixture.on_post_update(self.on_post_update)
        self.command = None # This variable is used as a bridge between Gymnasium and gazebo
        self.fixture.finalize()
        self.server = self.fixture.server()
        self.terminated = False

    def on_pre_update(self, info, ecm):
        """
        on_pre_update is used to command the model vehicle.
        """
        if info.paused:
            # In the event a pause comes in, we want to be able to not crash when visuallizing
            return
        world = World(world_entity(ecm))
        self.model = Model(world.model_by_name(ecm, "vehicle_green"))
        self.pole_entity = self.model.link_by_name(ecm, "pole")
        self.chassis_entity = self.model.link_by_name(ecm, "chassis")
        self.pole = Link(self.pole_entity)
        self.pole.enable_velocity_checks(ecm)
        self.chassis = Link(self.chassis_entity)
        self.chassis.enable_velocity_checks(ecm)
        if self.command == 1:
            self.chassis.add_world_force(ecm, Vector3d(2000, 0, 0))
        elif self.command == 0:
            self.chassis.add_world_force(ecm, Vector3d(-2000, 0, 0))

    def on_post_update(self, info, ecm):
        """
        on_post_update is used to read the current state of the world. We write the
        state to a local field.
        """
        if info.paused:
            # In the event a pause comes in, we want to be able to not crash when visuallizing
            return
        pole_pose = self.pole.world_pose(ecm).rot().euler().y()
        if self.pole.world_angular_velocity(ecm) is not None:
            pole_angular_vel = self.pole.world_angular_velocity(ecm).y()
        else:
            pole_angular_vel = 0
            print("Warning failed to get angular velocity")
        cart_pose = self.chassis.world_pose(ecm).pos().x()
        cart_vel = self.chassis.world_linear_velocity(ecm)

        if cart_vel is not None:
            cart_vel = cart_vel.x()
        else:
            cart_vel = 0
            print("Warning failed to get cart velocity")
        # Write the state to the environment
        self.state = np.array([cart_pose, cart_vel, pole_pose, pole_angular_vel], dtype=np.float32)
        if not self.terminated:
            self.terminated = pole_pose > 0.48 or pole_pose < -0.48 or cart_pose > 4.8 or cart_pose < -4.8

        if self.terminated:
            self.reward = 0.0
        else:
            self.reward = 1.0

    def step(self, action, paused=False):
        """
        Execute the server.

        There is a bit of nuance in this instance,
        our environment has control over every 5 simulation steps.
        We block the server till those 5 steps are completed.
        """
        self.command = action
        self.server.run(True, 5, paused)
        obs = self.state
        reward = self.reward
        return obs, reward, self.terminated, False, {}

    def reset(self):
        """
        This function simply resets the server
        """
        self.server.reset_all()
        self.command = None
        self.terminated = False
        obs, reward_, term_, tunc_, other_= self.step(None, paused=False)
        return obs, {}



class CustomCartPole(gym.Env):
    """
    Wrapper around GzRewardScorer that adapts the reward scorer to work with
    gymnasium.
    """
    def __init__(self, env_config):
        self.env = GzRewardScorer()
        self.action_space = gym.spaces.Discrete(2)
        self.observation_space = gym.spaces.Box(
            np.array([-10, float("-inf"), -0.418, -3.4028235e+38]),
            np.array([10, float("inf"), 0.418, 3.4028235e+38]),
            (4,), np.float32)

    def reset(self, seed=123):
        return self.env.reset()

    def step(self, action):
        obs, reward, done, truncated, info = self.env.step(action)
        return  obs, reward, done, truncated, info

env = CustomCartPole({})
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=25_000)

vec_env = model.get_env()
obs = vec_env.reset()

# Spawn your GUI and see the model you inferred
run_gui()
time.sleep(10)
for i in range(50000):
    action, _state = model.predict(obs, deterministic=True)
    obs, reward, done, info = vec_env.step(action)
