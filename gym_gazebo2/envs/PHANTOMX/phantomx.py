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
from gym_gazebo2.utils import ut_generic, ut_launch_phantomx, ut_mara, ut_math, ut_gazebo, tree_urdf, general_utils
from gym.utils import seeding
from gazebo_msgs.srv import SpawnEntity
import subprocess
import argparse
import transforms3d as tf3d

# ROS 2
import rclpy
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint # Used for publishing mara joint angles.
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ContactState
from std_msgs.msg import Float32
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose
from ros2pkg.api import get_prefix_path
from builtin_interfaces.msg import Duration
from nav_msgs.msg import Odometry

# Algorithm specific
from PyKDL import ChainJntToJacSolver # For KDL Jacobians

class PHANTOMXEnv(gym.Env):

    def __init__(self):

        """
        Initialize the MARA environemnt
        """
        # Manage command line args
        args = ut_generic.getArgsParserMARA().parse_args()
        self.gzclient = args.gzclient
        self.realSpeed = args.realSpeed
        self.velocity = args.velocity
        self.multiInstance = args.multiInstance
        self.port = args.port

        # Set the path of the corresponding URDF file
        if self.realSpeed:
            urdf = "phantomx.urdf"
        else:
            urdf = "phantomx.urdf"

        urdfPath = get_prefix_path("mara_description") + "/share/mara_description/urdf/" + urdf

        # Launch phantomx in a new Process
        self.launch_subp = ut_launch_phantomx.startLaunchServiceProcess(dppo
            ut_launch_phantomx.generateLaunchDescriptionPhantomX(
                self.gzclient, self.realSpeed, self.multiInstance, self.port, urdf))

        # Wait a bit for the spawn process.
        # TODO, replace sleep function.
        time.sleep(5)
        # Create the node after the new ROS_DOMAIN_ID is set in generate_launch_description()
        rclpy.init(args=None)
        self.node = rclpy.create_node(self.__class__.__name__)

        # class variables
        self._observation_msg = None
        self.max_episode_steps = 512  #default value, can be updated from baselines
        self.iterator = 0
        self.reset_jnts = True
        self.ground_truth = None
        self.max_torque = 2.8
        self.num_legs = 6

        low = np.reshape([-0.523599, -1.5708, -1.5708] * 6 +
              [-5.6548668] * 6 * 3 +
              [-2] * 9 +
              [-3] * 1 + [0] * 1 +
              [-10] * 6, -1)
        high = np.reshape([0.523599, 1.5708, 0.6179939] * 6 +
               [5.6548668] * 6 * 3 +
               [2] * 9 +
               [3] * 1 + [0.5] * 1 +
               [10] * 6, -1)
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

        self._collision_msg = None

        self._ground_truth = Odometry()

        self._goal_height = 0.13
        self.old_x = 0.0
        JOINT_NAMES = []
        JOINT_SUBSCRIBER = '/joint_states'

        self._pubs = {}

        for leg in ['lf', 'lm', 'lr', 'rf', 'rm', 'rr']:
            for limb in ['c1', 'thigh', 'tibia']:
                self._pubs[leg + "_" + limb] = self.node.create_publisher(Float32, 'j_' + limb + '_' + leg + '/force')

        # Subscribe to the appropriate topics, taking into account the particular robot
        self._sub = self.node.create_subscription(JointState, JOINT_SUBSCRIBER,
                                                  self.observation_callback)
        self._ground_truth_sub = self.node.create_subscription(Odometry, '/odom',
                                                  self.ground_truth_callback)
        self._sub_coll = self.node.create_subscription(ContactState, '/gazebo_contacts', self.collision_callback)
        self.reset_sim = self.node.create_client(Empty, '/reset_simulation')

        # # Here idially we should find the control range of the robot. Unfortunatelly in ROS/KDL there is nothing like this.
        # # I have tested this with the mujoco enviroment and the output is always same low[-1.,-1.], high[1.,1.]
        # Seed the environment
        self.seed()
        self.buffer_dist_rewards = []
        self.buffer_tot_rewards = []
        self.collided = 0

    def expand(self, v):
        if len(v.shape) is 1:
            v = v[np.newaxis, :]
        return v

    def quat_to_so3(self, q):
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
        return self.expand(R)

    def ground_truth_callback(self, message):
        self._ground_truth = message

    def observation_callback(self, message):
        """
        Callback method for the subscriber of JointState
        """
        self._observation_msg = message

    def collision_callback(self, message):
        """
        Callback method for the subscriber of Collision data
        """
        if message.collision1_name != message.collision2_name:
            self._collision_msg = message

    def set_episode_size(self, episode_size):
        self.max_episode_steps = episode_size

    def take_observation(self):
        """
        Take observation from the environment and return it.
        :return: state.
        """
        # # Take an observation
        rclpy.spin_once(self.node)

        obs_message = self._observation_msg
        self.ground_truth = self._ground_truth
        # Check that the observation is not prior to the action
        while obs_message is None or self.ground_truth is None:
            rclpy.spin_once(self.node)
            obs_message = self._observation_msg
            self.ground_truth = self._ground_truth

        so3 = self.quat_to_so3(np.array([self.ground_truth.pose.pose.orientation.x,
                                  self.ground_truth.pose.pose.orientation.y,
                                  self.ground_truth.pose.pose.orientation.z,
                                  self.ground_truth.pose.pose.orientation.w]))

        state = np.r_[np.reshape(obs_message.position, -1),
                      np.reshape(obs_message.velocity, -1),
                      np.reshape(so3, -1),
                      np.reshape([self.ground_truth.pose.pose.position.y,
                                  self.ground_truth.pose.pose.position.z,
                                  self.ground_truth.twist.twist.linear.x,
                                  self.ground_truth.twist.twist.linear.y,
                                  self.ground_truth.twist.twist.linear.z,
                                  self.ground_truth.twist.twist.angular.x,
                                  self.ground_truth.twist.twist.angular.y,
                                  self.ground_truth.twist.twist.angular.z,
                                  ], -1), ]
        # state = np.r_[np.reshape(obs_message.position, -1),
        #               np.reshape(obs_message.velocity, -1),
        #               np.reshape([self.ground_truth.pose.pose.position.x,
        #                           self.ground_truth.pose.pose.position.y,
        #                           self.ground_truth.pose.pose.position.z], -1), ]

        return state

    def collision(self):
        # Reset if there is a collision
        if self._collision_msg is not None:
            while not self.reset_sim.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('/reset_simulation service not available, waiting again...')

            reset_future = self.reset_sim.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self.node, reset_future)
            self._collision_msg = None
            self.collided += 1
            return True
        else:
            return False

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
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

        for leg in ['lf', 'lm', 'lr', 'rf', 'rm', 'rr']:
            for limb in ['c1', 'thigh', 'tibia']:
                msg.data = float(action[idx % (self.num_legs * 3)])
                if self.num_legs is 2 and idx in [3, 12]:
                  msg.data *= -1
                if self.num_legs is 1 and idx in [0, 3, 6]:
                  msg.data *= -1
                self._pubs[leg + "_" + limb].publish(msg)
                idx += 1

        # Take an observation
        self._observation_msg = None
        self._ground_truth = None

        # collided = self.collision()

        obs = self.take_observation()

        #standing up reward
        # reward = (self._goal_height -
        #           np.sqrt((self.ground_truth.pose.pose.position.z - self._goal_height)**2)) / self._goal_height
        # r = 0
        # for a in action:
        #     r += np.sqrt(a**2) / 2.8
        # r = r / len(action) * 0.25
        #
        # reward -= r


        #walking forward reward
        reward = self.ground_truth.twist.twist.linear.x
        #reward = (self.ground_truth.pose.pose.position.x - self.old_x) * 10
        #self.old_x = self.ground_truth.pose.pose.position.x

        # Calculate if the env has been solved
        done = bool(self.iterator == self.max_episode_steps)
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
        self.iterator = 0

        if self.reset_jnts is True:
            # reset simulation
            while not self.reset_sim.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('/reset_simulation service not available, waiting again...')

            reset_future = self.reset_sim.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self.node, reset_future)

        self.ros_clock = rclpy.clock.Clock().now().nanoseconds

        # Take an observation
        obs = self.take_observation()

        # Return the corresponding observation
        return obs

    def close(self):
        print("Closing " + self.__class__.__name__ + " environment.")
        parent = psutil.Process(self.launch_subp.pid)
        for child in parent.children(recursive=True):
            child.kill()
        rclpy.shutdown()
        parent.kill()
