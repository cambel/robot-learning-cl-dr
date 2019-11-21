import numpy as np
from ur_control.constants import FORCE_TORQUE_EXCEEDED


def dense_distance_force(self, obs, done):
    reward = 0

    r_distance = 1 - np.tanh(5.0 * np.linalg.norm(obs[:6]))

    wrench_size = self.wrench_hist_size*6
    force = np.reshape(obs[-wrench_size:], (6, -1))
    force = np.average(force, axis=1)
    norm_force_torque = np.linalg.norm(force)
    r_force = -1/(1 + np.exp(-norm_force_torque*15+5))

    r_collision = self.cost_collision if self.action_result == FORCE_TORQUE_EXCEEDED else 0.0
    position_reached = np.all(obs[:3] < self.position_threshold)
    r_done = self.cost_done if done and position_reached and self.action_result != FORCE_TORQUE_EXCEEDED else 0.0
    r_step = self.cost_step

    reward = self.w_dist*r_distance + self.w_force*r_force + r_collision + r_done + r_step
    return reward, [r_distance, r_force, r_collision, r_done]


def dense_distance_velocity_force(self, obs, done):
    reward = 0

    distance = np.linalg.norm(obs[:6])
    velocity = np.linalg.norm(obs[6:12])

    r_distance_velocity = (1-np.tanh(distance*5.)) * (1-velocity) + (velocity*0.5)**2

    wrench_size = self.wrench_hist_size*6
    force = np.reshape(obs[-wrench_size:], (6, -1))
    force = np.average(force, axis=1)
    norm_force_torque = np.linalg.norm(force)
    # r_force = - np.tanh(5*norm_force_torque) # monotonic penalization
    # s-shaped penalization, no penalization for lower values, max penalization for high values
    r_force = -1/(1 + np.exp(-norm_force_torque*15+5))
    # print(round(r_distance_velocity, 4), round(r_force, 4))

    r_collision = self.cost_collision if self.action_result == FORCE_TORQUE_EXCEEDED else 0.0
    # reward discounted by the percentage of steps left for the episode (encourage faster termination)
    # r_done = self.cost_done + (1-self.step_count/self.steps_per_episode) if done and self.action_result != FORCE_TORQUE_EXCEEDED else 0.0
    position_reached = np.all(obs[:3]*self.max_distance[:3] < self.position_threshold_cl)
    r_done = self.cost_done if done and position_reached and self.action_result != FORCE_TORQUE_EXCEEDED else 0.0

    # encourage faster termination
    r_step = self.cost_step

    reward = self.w_dist*r_distance_velocity + self.w_force*r_force + r_collision + r_done + r_step
    # print('r', round(reward, 4), round(r_distance_velocity, 4), round(r_force, 4), r_done)
    return reward, [r_distance_velocity, r_force, r_collision, r_done, r_step]
