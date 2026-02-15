import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from trac_ik_python.trac_ik import IK
import pandas as pd
import numpy as np


class URCSVExecutor(Node):
    def __init__(self):
        super().__init__('ur_csv_trajectory_executor')

        # 1. Parameters & Configuration
        self.declare_parameter('csv_path', 'motion_data.csv')
        self.joints = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        # Your specific Home Position from MuJoCo
        self.home_positions = [0.16986918, -np.pi / 1.5, -1.1, -np.pi / 9, np.pi / 2, 1.64722557]
        self.dt = 0.02  # Matching your MuJoCo sample time (50Hz)

        # 2. Initialize TRAC-IK
        # Assumes the robot_description topic is being published by robot_state_publisher
        self.ik_solver = IK("base_link", "tool0")

        # 3. Publisher for the Controller
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Wait for simulation to start and then execute
        self.create_timer(2.0, self.execute_full_motion)

    def execute_full_motion(self):
        self.destroy_timer(self.timer)  # Run once

        full_trajectory = JointTrajectory()
        full_trajectory.joint_names = self.joints

        current_time = 2.0  # Start after 2 seconds to allow for settling

        # --- PHASE 1: Add Home Position ---
        home_point = JointTrajectoryPoint()
        home_point.positions = self.home_positions
        home_point.time_from_start = rclpy.duration.Duration(seconds=current_time).to_msg()
        full_trajectory.points.append(home_point)

        # Allow 2 seconds to move from origin to Home
        current_time += 2.0

        # --- PHASE 2: Process CSV for Motion ---
        csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        try:
            df = pd.read_csv(csv_path)
            self.get_logger().info(f"Loaded CSV with {len(df)} steps.")

            for index, row in df.iterrows():
                # Target: x, y from CSV, fixed z = 0.49
                # Orientation: Keep the tool pointing down (MuJoCo style)
                # Using a standard 'downward' quaternion for UR5e mop usage
                target_pos = [row['x'], row['y'], 0.49]
                target_ori = [0.0, 1.0, 0.0, 0.0]  # Adjust based on your mop's neutral orientation

                joint_angles = self.ik_solver.get_ik(
                    target_ori, target_pos,
                    self.home_positions  # Use home as seed for consistency
                )

                if joint_angles:
                    point = JointTrajectoryPoint()
                    point.positions = list(joint_angles)
                    point.time_from_start = rclpy.duration.Duration(seconds=current_time).to_msg()
                    full_trajectory.points.append(point)
                    current_time += self.dt  # Increment by your MuJoCo dt
                else:
                    self.get_logger().warn(f"IK failed for step {index}")

            self.get_logger().info("Publishing full trajectory to Gazebo...")
            self.trajectory_pub.publish(full_trajectory)

        except Exception as e:
            self.get_logger().error(f"Failed to process CSV: {e}")


def main():
    rclpy.init()
    node = URCSVExecutor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()