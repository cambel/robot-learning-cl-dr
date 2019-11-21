# Copyright (c) 2018-2022, Cristian C Beltran-Hernandez.  All rights reserved.
#
# Cristian C Beltran-Hernandez and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from Cristian C Beltran-Hernandez is strictly prohibited.
#
# Author: Cristian C Beltran-Hernandez

from ur_control.constants import DONE, FORCE_TORQUE_EXCEEDED, SPEED_LIMIT_EXCEEDED, STOP_ON_TARGET_FORCE, TERMINATION_CRITERIA, IK_NOT_FOUND
from ur_control import conversions, transformations, spalg, utils
from ur_control.arm import Arm
import types
import numpy as np
import rospy
import sys
sys.path[:0] = ['/usr/local/lib/python3.6/dist-packages/']
import tf


# Returns the new average
# after including x
def getAvg(prev_avg, x, n):
    return ((prev_avg *
             n + x) /
            (n + 1))


class CompliantController(Arm):
    def __init__(self,
                 relative_to_ee=False,
                 namespace='',
                 **kwargs):
        """ Compliant controllertrajectory_time_compensation
            relative_to_ee bool: if True when moving in task-space move relative to the end-effector otherwise
                            move relative to the world coordinates
        """
        self.tf_listener = tf.TransformListener()
        Arm.__init__(self, namespace=namespace, **kwargs)

        self.relative_to_ee = relative_to_ee
        self.target_frame = None
        self.object_centric_transform = None

        # read publish rate if it does exist, otherwise set publish rate
        js_rate = utils.read_parameter(namespace + '/joint_state_controller/publish_rate', 500.0)
        self.rate = rospy.Rate(js_rate)

    def compute_step(self, motion_command, current_pose, adjusted_current_force, 
                     model, mtype, object_centric=False, ee_centric=False):
        """
            Compute next step based on the force control model, current pose and contact force.
            motion_command: list[6], additional 6DOF motion, can be completely independent as 
            in `parallel` and `parallel-guided` cases, or it can be bound by the selection matrix
            (alpha) to the position controller.
        """
        # Current Force in task-space
        if mtype == "parallel":
            dxf_force = model.control_force(adjusted_current_force)
            delta_pose = motion_command + dxf_force
        elif mtype == "parallel-guided":
            dxf_pos, dxf_force = model.control_position_orientation_raw(adjusted_current_force, current_pose)
            delta_pose = dxf_pos + motion_command + dxf_force
        elif mtype == "parallel-guided-with-alpha":
            dxf_pos, dxf_force = model.control_position_orientation_raw(adjusted_current_force, current_pose)
            if object_centric and ee_centric:
                pos = np.dot(model.alpha, -1*dxf_pos + motion_command)
            else:
                pos = np.dot(model.alpha, dxf_pos + motion_command)
            force = np.dot((np.identity(6) - model.alpha), dxf_force)
            delta_pose = pos + force
        elif mtype == "adm":
            dxf_force = model.control(adjusted_current_force)  # angular velocity
            delta_pose = motion_command + dxf_force
        elif mtype == "adm-guided":
            dxf_force = model.control(adjusted_current_force)  # angular velocity
            error = spalg.translation_rotation_error(model.position_target, current_pose)
            dxf_pos = model.position_pd.update(error=error, dt=model.dt)
            delta_pose = dxf_pos + motion_command + dxf_force
        else:
            raise ValueError("Invalid mtype")
        return delta_pose

    def execute_control(self, motion_command, model, max_force_torque, timeout=5.0, mtype="parallel",
                        object_centric=False, ee_centric=False):
        """
            motion command is the agent action, the delta translation/rotation
            the model will only consider the force and update the motion command accordingly
        """
        reduced_joint_speed = np.deg2rad([180, 180, 180, 250, 250, 250])
        # reduced_joint_speed = np.deg2rad([90, 90, 90, 125, 125, 250])

        linear_velocity_limit = 1.0

        initime = rospy.get_time()
        while not rospy.is_shutdown() \
                and (rospy.get_time() - initime) < timeout:

            # Transform wrench to the base_link frame
            current_wrench = self.get_ee_wrench(transformed=not object_centric)
            if object_centric:
                if not ee_centric:
                    model.target_position = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
                    current_pose_base_link = conversions.to_pose_stamped('base_link', self.end_effector())
                    ee_pos_oc_now_msg = conversions.transform_pose(
                        self.target_frame, self.object_centric_transform, current_pose_base_link)
                    current_pose = conversions.from_pose_to_list(ee_pos_oc_now_msg.pose)
                    current_wrench = spalg.convert_wrench(current_wrench, current_pose)
                else:
                    current_pose_obj_link = conversions.to_pose_stamped('target_board_tmp', [0.0, 0.0, 0, 0, 0, 0, 1])
                    obj_bc_now_msg = conversions.transform_pose(
                        "base_link", self.object_centric_transform, current_pose_obj_link)
                    rotation_matrix = transformations.pose_to_transform(self.end_effector())
                    ee_pos_oc_now_msg = conversions.transform_pose(
                        "gripper_tip_link", np.linalg.inv(rotation_matrix), obj_bc_now_msg)
                    current_pose = conversions.from_pose_to_list(ee_pos_oc_now_msg.pose)
            else:
                # Current position in task-space
                current_pose = self.end_effector()

            # Move in the opposite direction of the force
            adjusted_current_force = -1 * current_wrench

            # Enforce force/torque limits
            if np.any(np.abs(current_wrench) > max_force_torque):
                rospy.logerr('Maximum force/torque exceeded {}'.format(np.round(current_wrench, 3)))
                self.set_target_pose_flex(pose=current_pose, t=model.dt)
                result = FORCE_TORQUE_EXCEEDED
                break

            delta_pose = self.compute_step(motion_command, current_pose,
                                           adjusted_current_force, model, mtype, object_centric, ee_centric)

            magnitude = np.linalg.norm(delta_pose[:3])
            if magnitude > linear_velocity_limit:  # 1m/s
                delta_pose[:3] *= linear_velocity_limit/magnitude
            delta_pose[3:] = np.clip(delta_pose[3:], -np.pi/2, np.pi/2)

            if object_centric and ee_centric:
                xc = transformations.pose_from_angular_velocity(
                    np.array([0, 0, 0.0, 0, 0, 0, 1.0]), delta_pose, model.dt)
            else:
                xc = transformations.pose_from_angular_velocity(current_pose, delta_pose, model.dt)

            if object_centric:
                if not ee_centric:
                    cmd_pose = conversions.to_pose_stamped('target_board_tmp', xc)
                    cmd_base = conversions.transform_pose(
                        "base_link", np.linalg.inv(self.object_centric_transform), cmd_pose)
                else:
                    cmd_pose = conversions.to_pose_stamped('gripper_tip_link', xc)
                    rotation_matrix = transformations.pose_to_transform(self.end_effector())
                    cmd_base = conversions.transform_pose("gripper_tip_link", rotation_matrix, cmd_pose)

                # convert to base_link to compute IK
                xc = conversions.from_pose_to_list(cmd_base.pose)

            # execute motion
            result = self.cartesian_control(xc, model.dt, self.joint_angles(), reduced_joint_speed)

        return result

    def execute_hybrid_control(self, model, max_force_torque, timeout=5.0, object_centric=False):
        reduced_joint_speed = np.deg2rad([180, 180, 180, 250, 250, 250])
        linear_velocity_limit = 0.5

        initime = rospy.get_time()
        while not rospy.is_shutdown() \
                and (rospy.get_time() - initime) < timeout:

            # Transform wrench to the base_link frame
            current_wrench = self.get_ee_wrench(transformed=not object_centric)
            if object_centric:
                model.target_position = np.array([0, 0, 0, 0, 0, 0, 1])
                current_pose_base_link = conversions.to_pose_stamped('base_link', self.end_effector())
                ee_pos_oc_now_msg = conversions.transform_pose(
                    self.target_frame, self.object_centric_transform, current_pose_base_link)
                current_pose = conversions.from_pose_to_list(ee_pos_oc_now_msg.pose)
                current_wrench = spalg.convert_wrench(current_wrench, current_pose)
            else:
                # Current position in task-space
                current_pose = self.end_effector()
            # Move in the opposite direction of the force
            adjusted_current_force = -1 * current_wrench

            # Enforce force/torque limits
            if np.any(np.abs(current_wrench) > max_force_torque):
                rospy.logerr('Maximum force/torque exceeded {}'.format(np.round(current_wrench, 3)))
                self.set_target_pose_flex(pose=current_pose, t=model.dt)
                result = FORCE_TORQUE_EXCEEDED
                break

            # Current Force in task-space
            delta_pose, _, _ = model.control_position_orientation(
                adjusted_current_force, current_pose)  # angular velocity

            magnitude = np.linalg.norm(delta_pose[:3])
            if magnitude > linear_velocity_limit:  # 1m/s
                delta_pose[:3] *= linear_velocity_limit/magnitude
            delta_pose[3:] = np.clip(delta_pose[3:], -np.pi/2, np.pi/2)

            xc = transformations.pose_from_angular_velocity(current_pose, delta_pose, model.dt)

            if object_centric:
                cmd_pose = conversions.to_pose_stamped('target_board_tmp', xc)
                cmd_base = self.tf_listener.transformPose("base_link", cmd_pose)
                # execute motion
                xc = conversions.from_pose_to_list(cmd_base.pose)

            result = self.cartesian_control(xc, model.dt, self.joint_angles(), reduced_joint_speed)

        return result

    def set_hybrid_control_trajectory(self, trajectory, model, max_force_torque, timeout=5.0,
                                      stop_on_target_force=False, termination_criteria=None,
                                      displacement_epsilon=0.002, check_displacement_time=2.0,
                                      verbose=False, debug=False, time_compensation=True):
        """ Move the robot according to a hybrid controller model
            trajectory: array[array[7]] or array[7], can define a single target pose or a trajectory of multiple poses.
            model: force control model, see hybrid_controller.py 
            max_force_torque: array[6], max force/torque allowed before stopping controller
            timeout: float, maximum duration of controller's operation
            stop_on_target_force: bool: stop once the model's target force has been achieved, stopping controller when all non-zero target forces/torques are reached
            termination_criteria: lambda/function, special termination criteria based on current pose of the robot w.r.t the robot's base
            displacement_epsilon: float,  minimum displacement necessary to consider the robot in standby 
            check_displacement_time: float,  time interval to check whether the displacement has been larger than displacement_epsilon
        """

        reduced_speed = np.deg2rad([100, 100, 100, 250, 250, 250])

        xb = self.end_effector()
        failure_counter = 0

        ptp_index = 0
        q_last = self.joint_angles()

        trajectory_time_compensation = model.dt * 10. if time_compensation else 0.0  # Hyperparameter

        if trajectory.ndim == 1:  # just one point
            ptp_timeout = timeout
            model.set_goals(position=trajectory)
        else:  # trajectory
            ptp_timeout = timeout / float(len(trajectory)) - trajectory_time_compensation
            model.set_goals(position=trajectory[ptp_index])

        log = {SPEED_LIMIT_EXCEEDED: 0, IK_NOT_FOUND: 0}

        result = DONE

        standby_timer = rospy.get_time()
        standby_last_pose = self.end_effector()
        standby = False

        if debug:
            avg_step_time = 0.0
            step_num = 0

        # Timeout for motion
        initime = rospy.get_time()
        sub_inittime = rospy.get_time()
        while not rospy.is_shutdown() \
                and (rospy.get_time() - initime) < timeout:
            if debug:
                start_time = rospy.get_time()

            # Transform wrench to the base_link frame
            Wb = self.get_ee_wrench()
            # Current position in task-space
            xb = self.end_effector()

            if termination_criteria is not None:
                assert isinstance(
                    termination_criteria, types.LambdaType), "Invalid termination criteria, expecting lambda/function with one argument[current pose array[7]]"
                if termination_criteria(xb, standby):
                    rospy.loginfo("Termination criteria returned True, stopping force control")
                    result = TERMINATION_CRITERIA
                    break

            if (rospy.get_time() - sub_inittime) > ptp_timeout:
                sub_inittime = rospy.get_time()
                ptp_index += 1
                if ptp_index >= len(trajectory):
                    model.set_goals(position=trajectory[-1])
                elif not trajectory.ndim == 1:  # For some reason the timeout validation is not robust enough
                    model.set_goals(position=trajectory[ptp_index])

            Fb = -1 * Wb  # Move in the opposite direction of the force
            if stop_on_target_force and np.all(
                    np.abs(Fb)[model.target_force != 0] > np.abs(model.target_force)[model.target_force != 0]):
                rospy.loginfo('Target F/T reached {}'.format(np.round(Wb, 3)) + ' Stopping!')
                self.set_target_pose_flex(pose=xb, t=model.dt)
                result = STOP_ON_TARGET_FORCE
                break

            # Safety limits: max force
            if np.any(np.abs(Wb) > max_force_torque):
                rospy.logerr('Maximum force/torque exceeded {}'.format(np.round(Wb, 3)))
                self.set_target_pose_flex(pose=xb, t=model.dt)
                return FORCE_TORQUE_EXCEEDED

            # Current Force in task-space
            dxf, _, _ = model.control_position_orientation(Fb, xb)  # angular velocity

            xc = transformations.pose_from_angular_velocity(xb, dxf, dt=model.dt)

            # Avoid extra acceleration when a point failed due to IK or other violation
            # So, this corrects the allowed time for the next point
            dt = model.dt * (failure_counter+1)

            result = self.cartesian_control(xc, dt, q_last, reduced_speed)

            # Compensate the time allocated to the next command when there are failures
            # Especially important for following a motion trajectory
            for _ in range(failure_counter+1):
                self.rate.sleep()

            standby_time = (rospy.get_time() - standby_timer)
            if standby_time > check_displacement_time:
                displacement_dt = np.linalg.norm(standby_last_pose[:3] - self.end_effector()[:3])
                standby = displacement_dt < displacement_epsilon
                if standby and verbose:
                    rospy.logwarn("No more than %s displacement in the last %s seconds" %
                                  (round(displacement_dt, 6), check_displacement_time))
                standby_timer = rospy.get_time()
                standby_last_pose = self.end_effector()

            if debug:
                step_time = rospy.get_time() - start_time
                avg_step_time = step_time if avg_step_time == 0 else getAvg(avg_step_time, step_time, step_num)
                step_num += 1

        if debug:
            rospy.loginfo(">>> Force Control Aprox. time per step: %s <<<" % str(avg_step_time))
            hz = 1./avg_step_time if avg_step_time > 0 else 0.0
            rospy.loginfo(">>> Force Control Aprox. Frequency: %s <<<" % str(hz))
        if verbose:
            rospy.logwarn("Total # of commands ignored: %s" % log)
        return result

    def cartesian_control(self, pose, dt, q_last, reduced_speed, attempts=5):
        """
            Evaluate IK solution several times if it fails.
            Similarly, evaluate that the IK solution is viable
        """
        result = None
        q = self._solve_ik(pose, attempts=0, verbose=False)
        if q is None:
            if attempts > 0:
                return self.cartesian_control(pose, dt, q_last, reduced_speed, attempts-1)
            rospy.logwarn("IK not found")
            result = IK_NOT_FOUND
        else:
            q_speed = (q_last - q)/dt
            if np.any(np.abs(q_speed) > reduced_speed):
                scaling = np.deg2rad(160) / np.max(np.abs(q_speed))
                q_speed *= scaling
                q = q_last - q_speed*dt
            result = self.set_joint_positions_flex(position=q, t=dt)
            self.rate.sleep()
        return result
