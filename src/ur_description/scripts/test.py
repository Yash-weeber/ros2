#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from geometry_msgs.msg import PoseStamped
import pandas as pd
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory


class UR5eCleaningExecutor(Node):
    def __init__(self):
        super().__init__('ur5e_cleaning_executor')
        self.cb_group = ReentrantCallbackGroup()

        # 1. Action and Service Clients
        self._action_client = ActionClient(self, FollowJointTrajectory,
                                           '/joint_trajectory_controller/follow_joint_trajectory',
                                           callback_group=self.cb_group)
        self._ik_client = self.create_client(GetPositionIK, '/compute_ik', callback_group=self.cb_group)

        # 2. Hardware constants from your URDF
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.home_pos = [0.1698, -1.57, -1.1, -0.34, 1.57, 1.64]

        # Physical Offsets
        self.mop_handle_length = 0.5
        self.safety_z = 0.05  # 5cm above table for initial test

        pkg_share = get_package_share_directory('ur_description')
        self.csv_path = os.path.join(pkg_share, 'data', 'motion.csv')

        self.get_logger().info("Connecting to MoveIt...")
        self._ik_client.wait_for_service()
        self._action_client.wait_for_server()

        # Start move 2 seconds after clock starts
        self.timer = self.create_timer(2.0, self.execute_cleaning_move, callback_group=self.cb_group)

    def solve_ik(self, target_xyz, seed_joints):
        """Calls the MoveIt TRAC-IK solver defined in kinematics.yaml"""
        request = GetPositionIK.Request()
        req = PositionIKRequest()
        req.group_name = "ur_arm"  # Must match kinematics.yaml
        req.avoid_collisions = True

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = target_xyz
        # Point mop straight down (Orientation from your ur_for_ik.urdf)
        pose.pose.orientation.y = 0.7071
        pose.pose.orientation.w = 0.7071

        req.pose_stamped = pose
        req.robot_state.joint_state.name = self.joint_names
        req.robot_state.joint_state.position = seed_joints
        request.ik_request = req

        response = self._ik_client.call(request)
        if response and response.error_code.val == 1:
            full_map = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
            return [full_map[n] for n in self.joint_names]
        return None

    def execute_cleaning_move(self):
        self.timer.cancel()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        try:
            df = pd.read_csv(self.csv_path)
            last_joints, t = self.home_pos, 2.0

            # Start with home position
            goal.trajectory.points.append(JointTrajectoryPoint(positions=self.home_pos,
                                                               time_from_start=rclpy.duration.Duration(
                                                                   seconds=t).to_msg()))

            self.get_logger().info(f"Solving {len(df)} points from CSV...")
            for i, row in df.iterrows():
                # Correcting for your base_joint offset: xyz="0.0 -0.5 0.45"
                x_rob = float(row['x'])
                y_rob = float(row['y']) + 0.5
                z_rob = self.mop_handle_length + self.safety_z

                sol = self.solve_ik([x_rob, y_rob, z_rob], last_joints)
                if sol:
                    t += 0.2
                    goal.trajectory.points.append(JointTrajectoryPoint(positions=sol,
                                                                       time_from_start=rclpy.duration.Duration(
                                                                           seconds=t).to_msg()))
                    last_joints = sol

            self.get_logger().info(f"Sending {len(goal.trajectory.points)} valid points to Gazebo.")
            self._action_client.send_goal_async(goal)
        except Exception as e:
            self.get_logger().error(f"Error: {e}")


def main():
    rclpy.init()
    node = UR5eCleaningExecutor()
    # This prevents the "Waiting for service" hang
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__': main()