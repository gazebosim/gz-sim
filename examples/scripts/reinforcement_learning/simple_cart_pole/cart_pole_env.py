
import os
import gymnasium as gym
import numpy as np

from gz.common6 import set_verbosity
from gz.sim9 import TestFixture, World, world_entity, Model, Link
from gz.math8 import Vector3d
from gz.transport14 import Node
from gz.msgs11.world_control_pb2 import WorldControl
from gz.msgs11.world_reset_pb2 import WorldReset
from gz.msgs11.boolean_pb2 import Boolean

from stable_baselines3 import A2C

file_path = os.path.dirname(os.path.realpath(__file__))

class GzRewardScorer:
    def __init__(self):
        self.fixture = TestFixture(os.path.join(file_path, 'cart_pole.sdf'))
        self.fixture.on_pre_update(self.on_pre_update)
        self.fixture.on_post_update(self.on_post_update)
        self.command = None
        self.fixture.finalize()
        self.server = self.fixture.server()
        self.terminated = False

    def on_pre_update(self, info, ecm):

        world = World(world_entity(ecm))
        self.model = Model(world.model_by_name(ecm, "vehicle_green"))
        self.pole_entity = self.model.link_by_name(ecm, "pole")
        self.chassis_entity = self.model.link_by_name(ecm, "chassis")
        self.pole = Link(self.pole_entity)
        self.pole.enable_velocity_checks(ecm)
        self.chassis = Link(self.chassis_entity)
        self.chassis.enable_velocity_checks(ecm)
        if self.command == 1:
            self.chassis.add_world_force(Vector3d(0, 100, 0))
        elif self.command == 0:
            self.chassis.add_world_force(Vector3d(0, -100, 0))

    def on_post_update(self, info, ecm):
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

        self.state = np.array([cart_pose, cart_vel, pole_pose, pole_angular_vel], dtype=np.float32)
        if not self.terminated:
            self.terminated = pole_pose > 0.24 or pole_pose < -0.24 or cart_pose > 4.8 or cart_pose < -4.8

        if self.terminated:
            self.reward = 0.0
        else:
            self.reward = 1.0

    def step(self, action, paused=False):
        self.action = action
        self.server.run(True, 1, paused)
        obs = self.state
        reward = self.reward
        return obs, reward, self.terminated, False, {}

    def reset(self):
        self.server.reset_all()
        self.command = None
        self.terminated = False
        obs, reward_, term_, tunc_, other_= self.step(None, paused=False)
        return obs, {}



class CustomCartPole(gym.Env):
    def __init__(self, env_config):
        self.env = GzRewardScorer()
        #self.server =
        self.action_space = gym.spaces.Discrete(2)#self.env.action_space
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
model = A2C("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=10_000)

vec_env = model.get_env()
obs = vec_env.reset()
for i in range(5000):
    action, _state = model.predict(obs, deterministic=True)
    obs, reward, done, info = vec_env.step(action)
    # Nice to have spawn a gz sim client