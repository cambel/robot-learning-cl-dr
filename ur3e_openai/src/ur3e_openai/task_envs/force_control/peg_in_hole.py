
#!/usr/bin/env python

from copy import copy
import rospy
import numpy as np
from ur3e_openai.robot_envs.utils import get_board_color

from ur3e_openai.task_envs.ur3e_force_control import UR3eForceControlEnv
from ur_control import transformations
from ur_control.constants import FORCE_TORQUE_EXCEEDED
from ur_gazebo.basic_models import get_peg_board_model
from ur_gazebo.model import Model


def get_cl_range(range, curriculum_level):
    return [range[0], range[0] + (range[1] - range[0]) * curriculum_level]


def choose_peg_from_cl(curriculum_level):
    if curriculum_level <= 0.25:
        return PEG_SHAPES[0]
    elif curriculum_level <= 0.5:
        return PEG_SHAPES[1]
    elif curriculum_level <= 0.75:
        return PEG_SHAPES[2]
    else:
        return PEG_SHAPES[3]


PEG_SHAPES = ["cylinder", "hexagon", "cuboid", "triangular", "trapezoid", "star"]


class UR3ePegInHoleEnv2(UR3eForceControlEnv):
    """ Peg in hole with UR3e environment """

    def __init__(self):
        UR3eForceControlEnv.__init__(self)
        self.__load_env_params()

        if not self.real_robot:
            string_model = get_peg_board_model(
                kp=self.board_stiffness, mu=self.board_mu, 
                mu2=self.board_mu2, peg_shape=self.peg_shape,
                color=[0, 0.8, 0, 0.1])
            self.board_model = Model("board", self.board_initial_pose, file_type="string",
                                     string_model=string_model, model_id="target_board", reference_frame="base_link")

        # self.world_to_robot_base = transformations.pose_to_transform([0, 0, -0.7, 0, 0, np.pi/2])

        self.stage = 0
        self.current_board_pose = []
        self.x = np.zeros(6)

        self.current_peg_shape = ''

    def __load_env_params(self):
        prefix = "ur3e_gym"
        # Gazebo spawner parameters
        self.randomize_board_properties = rospy.get_param(prefix + "/randomize_board_properties", False)
        self.board_initial_pose = rospy.get_param(prefix + "/board_initial_pose", [])
        self.board_stiffness = rospy.get_param(prefix + "/board_stiffness", 1e5)
        self.board_mu = rospy.get_param(prefix + "/board_mu", 1.0)
        self.board_mu2 = rospy.get_param(prefix + "/board_mu2", 1.0)
        self.peg_shape = rospy.get_param(prefix + "/peg_shape", False)

        self.uncertainty_error = rospy.get_param(prefix + "/uncertainty_error", False)
        self.uncertainty_error_max_range = rospy.get_param(prefix + "/uncertainty_error_max_range", [0, 0, 0, 0, 0, 0])

        self.curriculum_level = rospy.get_param(prefix + "/initial_curriculum_level", 0.1)
        self.max_mu_range = rospy.get_param(prefix + "/max_mu_range", [1, 4])
        self.max_stiffness_range = rospy.get_param(prefix + "/max_stiffness_range", [5e5, 1e6])
        self.max_scale_range = rospy.get_param(prefix + "/max_scale_range", [1.05, 0.98])

        # How often to generate a new model, number of episodes
        self.refresh_rate = rospy.get_param(prefix + "/refresh_rate", False)
        self.normal_randomization = rospy.get_param(prefix + "/normal_randomization", True)
        self.basic_randomization = rospy.get_param(prefix + "/basic_randomization", False)
        self.random_type = rospy.get_param(prefix + "/random_type", "uniform")
        self.cl_upgrade_level = rospy.get_param(prefix + "/cl_upgrade_level", 0.8)
        self.cl_downgrade_level = rospy.get_param(prefix + "/cl_downgrade_level", 0.2)
        print(">>>>> ", self.random_type, self.curriculum_learning,
              self.progressive_cl, self.reward_based_on_cl, " <<<<<<")

    def _set_init_pose(self):
        if not self.real_robot:
            self.set_environment_conditions()
        else:
            self.position_threshold_cl = self.position_threshold
        xc = transformations.pose_euler_to_quaternion(
            self.ur3e_arm.end_effector(), [0, 0, -0.03, 0, 0, 0], ee_rotation=True)
        reset_time = 1.0 if not self.real_robot else 5.0
        self.ur3e_arm.set_target_pose(pose=xc, t=reset_time, wait=True)
        UR3eForceControlEnv._set_init_pose(self)

    def update_scene(self):
        if self.real_robot:
            return
        self.stage = 0
        if self.randomize_board_properties:
            board_pose = self.randomize_board_position()
            # Randomization type:
            # Uniform within the curriculum level's range
            if not self.curriculum_learning or self.random_type == "uniform":
                randomize_value = self.rng.uniform(low=0.0, high=1.0, size=4)
            # Normal within the curriculum level's range
            elif "normal" in self.random_type:
                mean = self.curriculum_level
                variance = 0.3
                randomize_value = self.rng.normal(loc=mean, scale=variance, size=4)
                randomize_value = np.clip(randomize_value, 0.0, 1.0)
                # Normal within the max range
                if self.random_type == "normal-full":
                    self.mu_range = self.max_mu_range
                    self.mu2_range = self.max_mu_range
                    self.stiffness_range = self.max_stiffness_range
                    self.scale_range = self.max_scale_range
                    # self.current_board_workspace = self.board_workspace
                    # self.position_threshold_cl = self.position_threshold

            mu = np.interp(randomize_value[0], [0., 1.], self.mu_range)
            mu2 = np.interp(randomize_value[1], [0., 1.], self.mu2_range)
            stiffness = np.interp(randomize_value[2], [0., 1.], self.stiffness_range)
            scale = np.interp(randomize_value[3], [0., 1.], self.scale_range)
            color = list(get_board_color(stiffness=stiffness,
                         stiff_lower=self.max_stiffness_range[0], stiff_upper=self.max_stiffness_range[1]))
            color[3] = 0.1
            string_model = get_peg_board_model(kp=stiffness, mu=mu, mu2=mu2, color=color,
                                               scale=scale, peg_shape=self.current_peg_shape)
            self.board_model = Model("board", board_pose, file_type="string",
                                     string_model=string_model, model_id="target_board", reference_frame="base_link")
            self.spawner.reset_model(self.board_model)
        else:
            board_pose = self.board_initial_pose
            self.board_model.set_pose(board_pose)
            self.spawner.update_model_state(self.board_model)

        self.current_board_pose = transformations.pose_euler_to_quat(board_pose)
        self.goal_offset = 0.0

    def set_environment_conditions(self):
        if self.curriculum_learning:
            if self.progressive_cl:
                if np.average(self.episode_hist) >= self.cl_upgrade_level and self.curriculum_level < 1.0:
                    self.curriculum_level += self.curriculum_level_step
                    self.logger.info("CL difficulty UP to %s, ep: %s" %
                                     (round(self.curriculum_level, 2), self.episode_num))
                    self.episode_hist = np.array([0, 1]*10)  # set 50%
                elif np.average(self.episode_hist) <= self.cl_downgrade_level and self.curriculum_level > self.curriculum_level_step:
                    self.curriculum_level -= self.curriculum_level_step
                    self.logger.info("CL difficulty DOWN to %s, ep: %s" %
                                     (round(self.curriculum_level, 2), self.episode_num))
                    self.episode_hist = np.array([0, 1]*10)  # set 50%
                self.curriculum_level = np.clip(self.curriculum_level, 0, 1)
                curriculum_level = self.curriculum_level
                self.logger.info("current CL difficulty: %s, %s, ep: %s" % (round(curriculum_level, 2),
                                 np.round(np.average(self.episode_hist), 2), self.episode_num))
            else:
                max_episodes = 200
                num_eps = self.episode_num + self.cumulative_episode_num  # current training session + previous ones
                curriculum_level = min((num_eps/max_episodes), 1.0)
                # curriculum_level = min((num_eps/max_episodes)**2, 1.0)
                self.logger.info(
                    "current CL difficulty: %s, %s, ep: %s" %
                    (round(curriculum_level, 2),
                     np.average(self.episode_hist),
                     self.episode_num))
            self.difficulty_ratio = copy(curriculum_level)
            self.mu_range = get_cl_range(self.max_mu_range, curriculum_level)
            self.mu2_range = get_cl_range(self.max_mu_range, curriculum_level)
            self.stiffness_range = get_cl_range(self.max_stiffness_range, curriculum_level)
            self.scale_range = get_cl_range(self.max_scale_range, curriculum_level)
            self.current_board_workspace = [
                [-max(self.max_board_workspace[i] * curriculum_level, self.min_board_workspace[i]),
                 max(self.max_board_workspace[i] * curriculum_level, self.min_board_workspace[i])] for i in range(6)]
            self.current_board_workspace[2] = [max(self.max_board_workspace[2]
                                                   * curriculum_level, self.min_board_workspace[2]), 0]
            self.position_threshold_cl = 0.005 + (1-curriculum_level) * self.max_position_threshold
            # set gripper peg

            if self.param_use_gazebo:
                old_peg_shape = rospy.get_param("/ur3e_gym/current_peg_shape", "")
                if self.peg_shape:
                    self.current_peg_shape = self.peg_shape  # multi peg training
                else:
                    self.current_peg_shape = choose_peg_from_cl(curriculum_level)
                if old_peg_shape != self.current_peg_shape:
                    rospy.set_param("/ur3e_gym/current_peg_shape", self.current_peg_shape)
                    self.robot_connection.peg_shape = self.current_peg_shape
                    self.robot_connection.need_reset_robot = True
                    self.robot_connection._reset_robot()
            else:
                self.current_peg_shape = self.peg_shape
        else:
            self.mu_range = self.max_mu_range
            self.mu2_range = self.max_mu_range
            self.stiffness_range = self.max_stiffness_range
            self.scale_range = self.max_scale_range
            self.current_board_workspace = self.board_workspace
            self.position_threshold_cl = self.position_threshold
            if self.param_use_gazebo:
                old_peg_shape = rospy.get_param("/ur3e_gym/current_peg_shape", None)
                if not old_peg_shape:
                    old_peg_shape = PEG_SHAPES[0]
                    self.current_peg_shape = PEG_SHAPES[0]
                elif self.peg_shape:
                    self.current_peg_shape = self.peg_shape  # ignore star
                else:
                    self.current_peg_shape = str(self.rng.choice(PEG_SHAPES[:-1]))  # ignore star
                rospy.set_param("/ur3e_gym/current_peg_shape", self.current_peg_shape)
                if old_peg_shape != self.current_peg_shape:
                    print(">> Changing gripper from", old_peg_shape, "to", self.current_peg_shape)
                    self.robot_connection.peg_shape = self.current_peg_shape
                    self.robot_connection.need_reset_robot = True
                    self.robot_connection._reset_robot()
            else:
                self.current_peg_shape = self.peg_shape
        self.robot_connection.need_reset_robot = False

    # def set_target_pose(self):
    #     if self.real_robot:
    #         return
    #     # print(self.object_current_pose)
    #     self.current_target_pose = np.copy(self.object_current_pose)
    #     # btn_pose = transformations.pose_euler_to_quaternion(self.object_current_pose, [0, 0, self.goal_offset, 0, 0, 0])
    #     # self.current_target_pose = np.copy(btn_pose)
    #     # self.current_target_pose[3:] = [1, 0, 0, 0]
    #     # self.current_target_pose[3:] = transformations.pose_euler_to_quaternion(
    #     #     self.current_target_pose, self.x, axes='sxyz')[3:]

    def randomize_board_position(self):
        if self.normal_randomization:
            rand = self.rng.random(size=6)
            rand = np.array([np.interp(rand[i], [0, 1.], self.current_board_workspace[i]) for i in range(6)])
            rand[3:] = np.deg2rad(rand[3:])
            self.x = rand
            pose = np.copy(self.board_initial_pose)
            pose += rand  # only x and y
        elif self.basic_randomization:
            options = []
            cbw = self.current_board_workspace
            options.append([cbw[0][0], cbw[1][0], 0, 0, 0, 0])
            options.append([cbw[0][0], cbw[1][1], 0, 0, 0, 0])
            options.append([cbw[0][1], cbw[1][0], 0, 0, 0, 0])
            options.append([cbw[0][1], cbw[1][1], 0, 0, 0, 0])
            options.append([cbw[0][0], cbw[1][0], 0, 0, 0, cbw[5][0]])
            options.append([cbw[0][0], cbw[1][1], 0, 0, 0, cbw[5][0]])
            options.append([cbw[0][1], cbw[1][0], 0, 0, 0, cbw[5][0]])
            options.append([cbw[0][1], cbw[1][1], 0, 0, 0, cbw[5][0]])
            rand = self.rng.choice(options)
            rand[3:] = np.deg2rad(rand[3:])
        else:
            rand = np.zeros(6)

        pose = np.copy(self.board_initial_pose)
        pose += rand  # only x and y
        return pose

    def _is_done(self, observations):
        pose_error = np.abs(observations[:6]*self.max_distance)
        if self.action_result == FORCE_TORQUE_EXCEEDED:
            self.logger.error("Collision!")

        if self.termination_on_negative_reward:
            if self.reward_based_on_cl:
                if self.cumulated_episode_reward <= self.termination_reward_threshold*self.difficulty_ratio:
                    rospy.loginfo("Fail on reward: %s" % (pose_error))
                    self.success_end = False
                    return True
            if self.cumulated_episode_reward <= self.termination_reward_threshold:
                rospy.loginfo("Fail on reward: %s" % (pose_error))
                self.success_end = False
                return True

        if self.two_steps:
            if self.stage == 0:
                position_reached = np.all(pose_error[:3] < 0.02)
            if self.stage == 1:
                position_reached = np.all(pose_error[:3] < self.position_threshold_cl)
            if position_reached and self.stage == 0:
                rospy.loginfo("First stage reached")
                self.goal_offset = 0.0
                self.stage = 1
                return False
            if position_reached and self.stage == 1:
                self.logger.info("goal reached: %s" % np.round(pose_error[:3], 4))
                self.success_end = True
            if self.step_count == self.steps_per_episode-1:
                self.logger.error("Fail!: %s" % np.round(pose_error[:3], 4))
            return (position_reached and self.stage == 1) \
                or self.action_result == FORCE_TORQUE_EXCEEDED  # Stop on collision
        else:
            position_reached = np.all(pose_error[:3] < self.position_threshold_cl)
            if self.real_robot and self.step_count % 100 == 0:
                print("----------- step:", self.step_count)
                print("dist error:", np.round(pose_error[:3], 4).tolist())
                print("motion act:", np.round(self.last_actions[:3].tolist(), 5).tolist(),
                      "\nposPID act:", np.round(self.last_actions[6:9].tolist(), 5).tolist(),
                      "\nforcePID act:", np.round(self.last_actions[12:15].tolist(), 5).tolist(),
                      "\nalpha act:", np.round(self.last_actions[18:21].tolist(), 5).tolist())
            if position_reached:
                self.logger.info("goal reached: %s" % np.round(pose_error[:3], 4))
                self.success_end = True
                if self.real_robot:
                    xc = transformations.pose_euler_to_quaternion(
                        self.ur3e_arm.end_effector(), [0, 0, 0.013, 0, 0, 0], ee_rotation=True)
                    reset_time = 5.0
                    # print("paused")
                    # input()
                    self.ur3e_arm.set_target_pose(pose=xc, t=reset_time, wait=True)
            if self.step_count == self.steps_per_episode-1:
                self.logger.error("Fail!: %s" % np.round(pose_error[:3], 4))
            return (position_reached) \
                or self.action_result == FORCE_TORQUE_EXCEEDED  # Stop on collision

    def _get_info(self):
        if self.action_result == FORCE_TORQUE_EXCEEDED:
            self.logger.error("Collision!")
            return "collision"
        return {}

    def _set_action(self, action):
        self.last_actions = action
        # self.set_target_pose()
        # print("target", np.round(self.current_target_pose, 4).tolist())
        self.action_result = self.controller.act(action, self.current_target_pose, self.action_type)
