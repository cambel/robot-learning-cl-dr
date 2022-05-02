#!/usr/bin/env python
import argparse
import rospy
import numpy as np
from ur3e_openai.common import load_environment, clear_gym_params, load_ros_params
np.set_printoptions(suppress=True)
np.set_printoptions(linewidth=np.inf)
import ur_control.utils as utils
import sys
import signal
import timeit
def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


class Agent(object):

    def __init__(self, action_size, action_type=0):
        self.action_size = action_size
        self.action_type = action_type

    def act(self, obs):
        if self.action_type == 10:
            act = np.ones(self.action_size)
            act[6:12] = np.zeros(6)
            return act
        if self.action_type == 11:
            act = np.ones(self.action_size)
            act[3:] = np.zeros(self.action_size-3)
            return act
        if self.action_type == 12:
            act = np.ones(self.action_size)
            act[:3] = np.clip(np.random.normal(0.0, 1.0, size=(3,)), np.ones(3)*-1, np.ones(3))
            return act
        if self.action_type == 2:
            act = np.zeros(self.action_size)
            act[5] -= 1
            return act
        if self.action_type == -2:
            act = 0 * np.ones(self.action_size)
            act[:6] = 0
            act[18:] = 1
            return act
        if self.action_type == -1:
            return -1 * np.ones(self.action_size)
        if self.action_type == 0:
            return np.zeros(self.action_size)
        if self.action_type == 1:
            return np.ones(self.action_size)
        else:
            return np.clip(np.random.normal(0.0, 0.5, size=(self.action_size,)), np.ones(self.action_size)*-1, np.ones(self.action_size))

if __name__ == '__main__':

    rospy.init_node('ur3e_test_gym_env',
                    anonymous=True,
                    log_level=rospy.INFO)

    parser = argparse.ArgumentParser(description='Test force control')
    parser.add_argument('-e', '--env_id', type=int, help='environment ID', default=None)
    parser.add_argument('-a', '--action_type', type=int, help='Action type', default=0)
    parser.add_argument('-r', '--repetitions', type=int, help='repetitions', default=1)


    args = parser.parse_args(rospy.myargv()[1:])
    args = parser.parse_args()

    clear_gym_params('ur3e_gym')
    clear_gym_params('ur3e_force_control')

    param_file = None

    if args.env_id == 0:
        args.dir_suffix = "sim_peg_in_hole"
        param_file = "simulation/single_peg_in_hole.yaml"
    elif args.env_id == 1:
        args.dir_suffix = "sim_peg_in_hole"
        param_file = "simulation/multiple_peg_in_hole.yaml"
    elif args.env_id == 2:
        args.dir_suffix = "real_peg_in_hole"
        param_file = "real/peg_in_hole.yaml"
    else:
        raise Exception("invalid env_id")

    p = utils.TextColors()
    p.error("GYM Environment:{} ".format(param_file))
    
    load_ros_params(rospackage_name="ur3e_rl",
                    rel_path_from_package_to_file="config",
                    yaml_file_name=param_file)

    # Init OpenAI_ROS ENV
    # rospy.set_param('ur3e_gym/output_dir', '/root/dev/results')
    episode_lenght = rospy.get_param("ur3e_gym/steps_per_episode", 100)
    env = load_environment(rospy.get_param("ur3e_gym/env_id"),
                        max_episode_steps=episode_lenght)
    episodes = args.repetitions
    agent = Agent(env.n_actions, args.action_type)
    obs = None
    done = False
    steps = 0
    i = 0
    env.reset()
    start_time = timeit.default_timer()
    # moving agent
    print('>>>> START Moving <<<<')
    while i < episodes:
        if steps >= episode_lenght or done:
            print('>>>> Reset episode')
            end_time = timeit.default_timer()
            print('>>>> Time', end_time-start_time)
            done = False
            steps = 0
            i += 1
            env.reset()
            start_time = timeit.default_timer()
        action = agent.act(obs)
        obs, reward, done, info = env.step(action)

    # env.reset()
