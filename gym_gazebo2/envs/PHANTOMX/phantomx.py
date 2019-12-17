import gym
gym.logger.set_level(40) # hide warnings
import time
import numpy as np
import copy
import math
import os
import psutil
import signal
import sys
from gym import utils, spaces
from gym_gazebo2.utils import ut_generic, ut_launch_phantomx #ut_math, ut_gazebo, general_utils , ut_mara
from gym.utils import seeding
from gazebo_msgs.srv import SpawnEntity
import subprocess
import argparse
import transforms3d as tf3d

import threading
from enum import Enum

# ROS 2
import rclpy
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import Float32
from std_srvs.srv import Empty
from ros2pkg.api import get_prefix_path
from nav_msgs.msg import Odometry

from functools import partial

class Info:
    def __init__(self, env):
        self.env = env
        self.legs = ['lf', 'lm', 'lr', 'rf', 'rm', 'rr']
        self.reset = {}
        self.action = {}
        for leg in self.legs:
            self.action[leg] = []
            self.reset[leg] = False
        self.obs = []
        self.reward = []
        self.info = []
        self.done = []
        self.action_lock = threading.Lock()
        self.reset_lock = threading.Lock()

    def execute_action(self):
        if not len(np.concatenate([action for action in self.action.values()])) is len(self.legs) * 3:
            return
        self.obs, self.reward, self.done, self.info = \
            self.env.step(np.concatenate([action for action in self.action.values()]))
        self.action_lock.acquire()
        for leg in self.legs:
            self.action[leg] = []
        self.action_lock.release()

    def execute_reset(self):
        if not all(self.reset.values()):
            return
        self.obs = self.env.reset()
        self.reset_lock.acquire()
        for leg in self.legs:
            self.reset[leg] = False
        self.reset_lock.release()

    def set_action(self, leg_name, action):
        self.action_lock.acquire()
        self.action[leg_name] = action
        self.action_lock.release()

    def set_reset(self, leg_name):
        self.reset_lock.acquire()
        self.reset[leg_name] = True
        self.reset_lock.release()
        # self.reset_event.clear()
        #
        # if not all(reset for reset in self.reset.values()):
        #     self.reset_event.wait()
        #     return
        #
        # if not self.reset_event.is_set():
        #     self.obs = self.env.reset()
        #     for leg in self.legs:
        #         self.reset[leg] = False
        #     self.reset_event.set()


class PHANTOMXEnv(gym.Env):

    def __init__(self):

        self.info = Info(self)

        """
        Initialize the MARA environemnt
        """
        # Manage command line args
        args = ut_generic.getArgsParserMARA().parse_args()
        self.gzclient = args.gzclient
        self.realSpeed = args.realSpeed
        # self.realSpeed = True
        self.velocity = args.velocity
        self.multiInstance = args.multiInstance
        self.port = args.port

        self.leg_name = "main"

        # Set the path of the corresponding URDF file
        if self.realSpeed:
            urdf = "phantomx.urdf"
        else:
            urdf = "phantomx.urdf"

        urdfPath = get_prefix_path("mara_description") + "/share/mara_description/urdf/" + urdf

        # Launch phantomx in a new Process
        self.launch_subp = ut_launch_phantomx.startLaunchServiceProcess(
            ut_launch_phantomx.generateLaunchDescriptionPhantomX(
                self.gzclient, self.realSpeed, self.multiInstance, self.port, urdf))

        # Wait a bit for the spawn process.
        # TODO, replace sleep function.
        time.sleep(5)
        # Create the node after the new ROS_DOMAIN_ID is set in generate_launch_description()
        rclpy.init(args=None)

        self.node = rclpy.create_node(self.__class__.__name__)

        # class variables
        self.max_episode_steps = 1024
        self.iterator = 0
        self.reset_jnts = True
        self.ground_truth = None
        self.limb_odoms = {}
        self.max_torque = 2.8
        self.num_legs = 6

        low = np.reshape([-10.0, -10.0, -10.0] * 3 * 6
                         + [0.0] * 6
                         # + [-10, -10, -10] * 3 * 6
                         # + [-10, -10, -10] * 3 * 6
                         + [-2.0] * 6
                         # + [-10] * 1 + [-10] * 1 + [0] * 1 +
                         # + [-10] * 6
                         + [-2.8] * 18
                         #+ [-10.0, -10.0, -10.0] * 3 * 6
                         # + [0]
                         , -1)
        high = np.reshape([10.0, 10.0, 10.0] * 3 * 6
                          + [1.0] * 6
                          # + [10, 10, 10] * 3 * 6
                          # + [10, 10, 10] * 3 * 6
                          + [2.0] * 6
                          # + [10] * 1 + [10] * 1 + [2] * 1 +
                          # + [10] * 6
                          + [2.8] * 18
                          #+ [10.0, 10.0, 10.0] * 3 * 6
                          # + [1]
                          , -1)
        self.observation_space = spaces.Box(low, high)

        low = -self.max_torque * np.ones(self.num_legs*3)
        high = self.max_torque * np.ones(self.num_legs*3)
        self.action_space = spaces.Box(low, high)
        # low = -np.pi * np.ones(3*2)
        # high = np.pi * np.ones(3*2)
        # self.observation_space = spaces.Box(low, high)
        # low = -np.pi * np.ones(6)
        # high = np.pi * np.ones(6)
        # self.action_space = spaces.Box(low, high)

        self._contact_msgs = {}
        self._collision = False
        self._ground_truth = Odometry()

        self._goal_height = 0.13
        self.old_x = 0.0

        self._pubs = {}
        self.limb_odom_sc = {}
        self.contact_sc = {}
        self.legs = ['lf', 'lm', 'lr', 'rf', 'rm', 'rr']
        self.limbs = ['c1', 'thigh', 'tibia']
        for leg in self.legs:
            self.contact_sc["tibia_" + leg] = self.node.create_subscription(
                ContactsState, '/tibia_' + leg + '_collision', self.leg_contact_callback)
            self._contact_msgs["tibia_" + leg] = 0
            for limb in self.limbs:
                self._pubs[leg + "_" + limb] = self.node.create_publisher(Float32, 'j_' + limb + '_' + leg + '/force')
                self.limb_odoms[leg + "_" + limb] = None
                self.limb_odom_sc[leg + "_" + limb] = self.node.create_subscription(Odometry,
                                                                                    '/' + leg + "_" + limb + '/odom',
                                                                                    self.limb_odom_callback)

        self._ground_truth_sub = self.node.create_subscription(Odometry, '/odom', self.ground_truth_callback)
        self._body_ground_contact_sub = self.node.create_subscription(ContactsState, '/body_collision', self.body_contact_callback)
        self.reset_sim = self.node.create_client(Empty, '/reset_simulation')
        self.pause_sim = self.node.create_client(Empty, '/pause_physics')
        self.unpause_sim = self.node.create_client(Empty, '/unpause_physics')

        # Seed the environment
        self._body_ground_contact = 0
        self.idx = 0
        self.seed()
        self.buffer_dist_rewards = []
        self.buffer_tot_rewards = []

    def pause_physics(self):
        pause_future = self.pause_sim.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self.node, pause_future)

    def unpause_physics(self):
        pause_future = self.unpause_sim.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self.node, pause_future)

    def expand(self, v):
        if len(v.shape) is 1:
            v = v[np.newaxis, :]
        return v

    def quat_to_d6(self, q):
        ''' converts quaternion q [qw,qx,qy,qz] to SO3 representation [a0.T,a1.T,a2.T] '''
        if len(q.shape) is 2:
            w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
        else:
            w, x, y, z = q[np.newaxis, 0], q[np.newaxis, 1], q[np.newaxis, 2], q[np.newaxis, 3]

        # a_row_column
        a00 = 1 - 2 * y ** 2 - 2 * z ** 2
        a10 = 2 * x * y + 2 * z * w
        a20 = 2 * x * z - 2 * y * w
        a01 = 2 * x * y - 2 * z * w
        a11 = 1 - 2 * x ** 2 - 2 * z ** 2
        a21 = 2 * y * z + 2 * x * w
        a02 = 2 * x * z + 2 * y * w
        a12 = 2 * y * z - 2 * x * w
        a22 = 1 - 2 * x ** 2 - 2 * y ** 2

        R = np.vstack([a00, a10, a20, a01, a11, a21, a02, a12, a22]).T
        return self.expand(R)[:, :6]

    def limb_odom_callback(self, message):
        limb = message.child_frame_id.split('_')
        self.limb_odoms[limb[1] + "_" + limb[0]] = message

    def ground_truth_callback(self, message):
        self._ground_truth = message

    def leg_contact_callback(self, message):
        if message.states:
            if message.states[0].collision1_name.split("::")[0] == message.states[0].collision2_name.split("::")[0]:
                self._collision = True
                return
            self._contact_msgs[message.states[0].collision2_name.split("::")[1]] = 1

    def body_contact_callback(self, message):
        if message.states:
            if message.states[0].collision1_name ==\
                    "phantomx::base_link::base_link_fixed_joint_lump__MP_BODY_collision" and\
                    message.states[0].collision2_name == "ground_plane::link::collision":
                self._body_ground_contact = 1
        else:
            self._body_ground_contact = 0

    def set_episode_size(self, episode_size):
        self.max_episode_steps = episode_size

    def take_observation(self, action):
        """
        Take observation from the environment and return it.
        :return: state.
        """
        # # Take an observation

        self._ground_truth = None
        limb_odoms_set = False
        for leg in self.legs:
            for limb in self.limbs:
                self.limb_odoms[leg + "_" + limb] = None

        # Check that the observation is not prior to the action
        while self.ground_truth is None or limb_odoms_set is False:
            rclpy.spin_once(self.node)
            limb_odoms_set = True
            for key, value in self.limb_odoms.items():
                if value is None:
                    limb_odoms_set = False
            self.ground_truth = self._ground_truth
        limb_positions = []
        limb_orientations = []
        limb_twists_linear = []
        limb_twists_angular = []
        leg_contacts = []
        for leg in self.legs:
            leg_contacts.append(self._contact_msgs['tibia_' + leg])
            self._contact_msgs['tibia_' + leg] = 0
            for limb in self.limbs:
                limb_positions.append(self.limb_odoms[leg + "_" + limb].pose.pose.position.x)
                limb_positions.append(self.limb_odoms[leg + "_" + limb].pose.pose.position.y)
                limb_positions.append(self.limb_odoms[leg + "_" + limb].pose.pose.position.z)
                limb_twists_linear.append(self.limb_odoms[leg + "_" + limb].twist.twist.linear.x)
                limb_twists_linear.append(self.limb_odoms[leg + "_" + limb].twist.twist.linear.y)
                limb_twists_linear.append(self.limb_odoms[leg + "_" + limb].twist.twist.linear.z)
                limb_twists_angular.append(self.limb_odoms[leg + "_" + limb].twist.twist.angular.x)
                limb_twists_angular.append(self.limb_odoms[leg + "_" + limb].twist.twist.angular.y)
                limb_twists_angular.append(self.limb_odoms[leg + "_" + limb].twist.twist.angular.z)

        d6 = self.quat_to_d6(np.array([self.ground_truth.pose.pose.orientation.x,
                                         self.ground_truth.pose.pose.orientation.y,
                                         self.ground_truth.pose.pose.orientation.z,
                                         self.ground_truth.pose.pose.orientation.w]))

        if action is None:
            action = [0.0] * 18
        state = np.r_[np.reshape(limb_positions, -1),
                      # np.reshape(limb_twists_linear, -1),
                      # np.reshape(limb_twists_angular, -1),
                      np.reshape(leg_contacts, -1),
                      np.reshape(d6, -1),
                      # np.reshape([self.ground_truth.pose.pose.position.x,
                      #             self.ground_truth.pose.pose.position.y,
                      #             self.ground_truth.pose.pose.position.z,
                      #             self.ground_truth.twist.twist.linear.x,
                      #             self.ground_truth.twist.twist.linear.y,
                      #             self.ground_truth.twist.twist.linear.z,
                      #             self.ground_truth.twist.twist.angular.x,
                      #             self.ground_truth.twist.twist.angular.y,
                      #             self.ground_truth.twist.twist.angular.z,
                      #             ], -1),
                      action,
                      #np.reshape(limb_twists_linear, -1),
                      # [self._body_ground_contact]
                      ]
        return state

    def seed(self, seed=None):
        seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        """
        Implement the environment step abstraction. Execute action and returns:
            - action
            - observation
            - reward
            - done (status)
        """


        self.iterator += 1
        # Execute "action"
        msg = Float32()
        idx = 0
        # for leg in ['lf', 'lm', 'lr', 'rf', 'rm', 'rr']:
        #     for limb in ['c1', 'thigh', 'tibia']:
        #         msg.data = float(action[idx])
        #         self._pubs[leg + "_" + limb].publish(msg)
        #         idx += 1

        self._collision = False
        self.unpause_physics()
        for leg in ['lf', 'lm', 'lr', 'rf', 'rm', 'rr']:
            for limb in ['c1', 'thigh', 'tibia']:
                msg.data = float(action[idx % (self.num_legs * 3)])
                if self.num_legs is 2 and idx in [3, 12]:
                  msg.data *= -1
                if self.num_legs is 6 and idx in [0, 3, 6]:  # mirror c1 movements
                  msg.data *= -1
                self._pubs[leg + "_" + limb].publish(msg)
                idx += 1
        rclpy.spin_once(self.node)
        # Take an observation
        obs = self.take_observation(action)
        self.pause_physics()

        reward = self.ground_truth.twist.twist.linear.x  # reward for high forward velocity

        # Calculate if the env has been solved
        done = bool(self.iterator == self.max_episode_steps)
        collision = self._collision
        if collision:
            done = True
            reward = 0
        # self.buffer_dist_rewards.append(self.ground_truth.pose.pose.position.x)
        # self.buffer_tot_rewards.append(reward)
        info = {}
        # if self.iterator % self.max_episode_steps == 0:
        #     max_dist_tgt = max(self.buffer_dist_rewards)
        #     mean_dist_tgt = np.mean(self.buffer_dist_rewards)
        #     min_dist_tgt = min(self.buffer_dist_rewards)
        #     max_tot_rew = max(self.buffer_tot_rewards)
        #     mean_tot_rew = np.mean(self.buffer_tot_rewards)
        #     min_tot_rew = min(self.buffer_tot_rewards)
        #     num_coll = self.collided
        #
        #     info = {"infos": {"ep_dist_max": max_dist_tgt, "ep_dist_mean": mean_dist_tgt, "ep_dist_min": min_dist_tgt,
        #                       "ep_rew_max": max_tot_rew, "ep_rew_mean": mean_tot_rew, "ep_rew_min": min_tot_rew,
        #                       "num_coll": num_coll}}
        #     self.buffer_dist_rewards = []
        #     self.buffer_tot_rewards = []
        #     self.collided = 0

        # Return the corresponding observations, rewards, etc.
        return obs, reward, done, info

    def reset(self):
        """
        Reset the agent for a particular experiment condition.
        """

        self.unpause_physics()

        self.iterator = 0
        if self.reset_jnts is True:
            msg = Float32()
            for leg in ['lf', 'lm', 'lr', 'rf', 'rm', 'rr']:
                for limb in ['c1', 'thigh', 'tibia']:
                    msg.data = float(0.0)
                    self._pubs[leg + "_" + limb].publish(msg)
            # reset simulation
            while not self.reset_sim.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('/reset_simulation service not available, waiting again...')

            reset_future = self.reset_sim.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self.node, reset_future)
        time.sleep(0.2)
        self.ros_clock = rclpy.clock.Clock().now().nanoseconds

        # Take an observation
        obs = self.take_observation(None)

        self.pause_physics()
        # Return the corresponding observation
        return obs

    def close(self):
        print("Closing " + self.__class__.__name__ + " environment.")
        parent = psutil.Process(self.launch_subp.pid)
        for child in parent.children(recursive=True):
            child.kill()
        rclpy.shutdown()
        parent.kill()


class PHANTOMXLEGEnv(gym.Env):

    def __init__(self):
        self.leg_name = ""
        self.info = Info(self)
        self.max_episode_steps = 1024
        self.num_legs = 1
        self.max_torque = 2.8
        self.idx = 0

        low = np.reshape([-10.0, -10.0, -10.0] * 3 * 3
                         + [0.0] * 3
                         # + [-10, -10, -10] * 3 * 6
                         # + [-10, -10, -10] * 3 * 6
                         + [-2.0] * 6
                         # + [-10] * 1 + [-10] * 1 + [0] * 1 +
                         # + [-10] * 6
                         + [-2.8] * 3 * 2
                         #+ [-10.0, -10.0, -10.0] * 3 * 3
                         # + [0]
                         , -1)
        high = np.reshape([10.0, 10.0, 10.0] * 3 * 3
                          + [1.0] * 3
                          # + [10, 10, 10] * 3 * 6
                          # + [10, 10, 10] * 3 * 6
                          + [2.0] * 6
                          # + [10] * 1 + [10] * 1 + [2] * 1 +
                          # + [10] * 6
                          + [2.8] * 3 * 2
                          #+ [10.0, 10.0, 10.0] * 3 * 3
                          # + [1]
                          , -1)
        self.observation_space = spaces.Box(low, high)
        low = -self.max_torque * np.ones(self.num_legs*3)
        high = self.max_torque * np.ones(self.num_legs*3)
        self.action_space = spaces.Box(low, high)

    def set_episode_size(self, episode_size):
        self.max_episode_steps = episode_size

    def set_info(self, info):
        self.info = info

    def step(self, action):
        self.info.set_action(self.leg_name, action)
        while any(self.info.action[self.leg_name]):
            time.sleep(1/1000)
        return self.leg_obs(self.info.obs), self.info.reward, self.info.done, self.info.info

    def close(self):
        self.info.env.close()

    def reset(self):
        self.info.set_reset(self.leg_name)
        while self.info.reset[self.leg_name]:
            time.sleep(1/1000)
        return self.leg_obs(self.info.obs)

    def leg_obs(self, main_obs):
        obs = 0
        if self.leg_name is 'lf':
            # obs = np.delete(main_obs, np.s_[108:162])   MAKE THIS SMARTER! NUM_OF_VARIABLES PER LEG * LIMBS PER LEG!
            # positions
            obs = np.delete(main_obs, np.s_[18:27])
            obs = np.delete(obs, np.s_[27:45])
            # ground contacts
            obs = np.delete(obs, np.s_[29])
            obs = np.delete(obs, np.s_[30:32])
            # actions
            obs = np.delete(obs, np.s_[42:45])
            obs = np.delete(obs, np.s_[45:51])
            obs = np.delete(obs, np.s_[36:39])
            # linear velocities
            #obs = np.delete(obs, np.s_[72:90])
            #obs = np.delete(obs, np.s_[54:63])



        if self.leg_name is 'lm':
            # obs = np.delete(main_obs, np.s_[81:108])
            # obs = np.delete(obs, np.s_[108:135])
            obs = np.delete(main_obs, np.s_[27:54])

            obs = np.delete(obs, np.s_[30:33])

            obs = np.delete(obs, np.s_[45:54])
            obs = np.delete(obs, np.s_[39:42])

            #obs = np.delete(obs, np.s_[63:90])


        if self.leg_name is 'lr':
            # obs = np.delete(main_obs, np.s_[81:135])
            obs = np.delete(main_obs, np.s_[0:9])
            obs = np.delete(obs, np.s_[18:36])

            obs = np.delete(obs, np.s_[27])
            obs = np.delete(obs, np.s_[29:31])

            obs = np.delete(obs, np.s_[36:39])
            obs = np.delete(obs, np.s_[42:48])
            obs = np.delete(obs, np.s_[39:42])

            #obs = np.delete(obs, np.s_[63:81])
            #obs = np.delete(obs, np.s_[36:45])



        if self.leg_name is 'rf':
            # obs = np.delete(main_obs, np.s_[27:81])
            obs = np.delete(main_obs, np.s_[9:27])
            obs = np.delete(obs, np.s_[27:36])

            obs = np.delete(obs, np.s_[28:30])
            obs = np.delete(obs, np.s_[30])

            obs = np.delete(obs, np.s_[39:45])
            obs = np.delete(obs, np.s_[45:48])
            obs = np.delete(obs, np.s_[39:42])

            # obs = np.delete(obs, np.s_[81:90])
            # obs = np.delete(obs, np.s_[45:63])


        if self.leg_name is 'rm':
            # obs = np.delete(main_obs, np.s_[0:27])
            # obs = np.delete(obs, np.s_[27:54])
            obs = np.delete(main_obs, np.s_[0:27])

            obs = np.delete(obs, np.s_[27:30])

            obs = np.delete(obs, np.s_[36:45])
            obs = np.delete(obs, np.s_[39:42])

            #obs = np.delete(obs, np.s_[36:63])


        if self.leg_name is 'rr':
            # obs = np.delete(main_obs, np.s_[0:54])
            obs = np.delete(main_obs, np.s_[0:18])
            obs = np.delete(obs, np.s_[9:18])

            obs = np.delete(obs, np.s_[27:29])
            obs = np.delete(obs, np.s_[28])

            obs = np.delete(obs, np.s_[36:42])
            obs = np.delete(obs, np.s_[39:42])
            obs = np.delete(obs, np.s_[42:45])

            #obs = np.delete(obs, np.s_[63:72])
            #obs = np.delete(obs, np.s_[36:54])

        return obs
