import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from moveit_msgs.msg import PositionIKRequest, RobotState
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np
import threading
import time
import csv
import os


class Ur5eIKMover(Node):
    def __init__(self):
        super().__init__('ur5e_ik_mover')

        # Declare use_sim_time parameter safely
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)

        # Publisher for the joint trajectory controller
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Subscriber for joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Clients for kinematics services
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')

        # HOME positions
        self.HOME_JOINT_POSITIONS = [
            0.16986918,  # shoulder_pan
            -np.pi / 1.5,  # shoulder_lift
            -1.1,  # elbow_joint
            -np.pi / 9,  # wrist_1
            np.pi / 2,  # wrist_2
            1.64722557  # wrist_3
        ]

        self.current_joint_positions = []

    def joint_state_callback(self, msg):
        # Update current joint positions safely
        if 'shoulder_pan_joint' in msg.name:
            temp_positions = []
            joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                           'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
            try:
                for joint_name in joint_names:
                    idx = msg.name.index(joint_name)
                    temp_positions.append(msg.position[idx])
                self.current_joint_positions = temp_positions
            except ValueError:
                pass

    def move_trajectory(self, list_of_joint_positions, time_step=0.1):
        """
        Publishes a continuous trajectory from a list of joint positions.
        list_of_joint_positions: List of lists (each inner list is 6 joint angles)
        time_step: Time (seconds) between each point
        """
        msg = JointTrajectory()
        msg.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        current_time = 0.0

        for joints in list_of_joint_positions:
            point = JointTrajectoryPoint()
            point.positions = joints

            # Increment time for each point so they play in sequence
            current_time += time_step
            point.time_from_start = Duration(seconds=current_time).to_msg()

            msg.points.append(point)

        self.get_logger().info(f"Publishing trajectory with {len(msg.points)} points...")
        self.publisher_.publish(msg)

    def move_robot(self, joint_positions, duration_sec=5.0):
        """Publishes a single point trajectory command"""
        msg = JointTrajectory()
        msg.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = Duration(seconds=duration_sec).to_msg()
        msg.points.append(point)
        self.publisher_.publish(msg)

    def get_fk(self, joint_positions):
        """Synchronous wrapper for FK"""
        if not self.fk_client.service_is_ready():
            self.get_logger().error("FK Service not ready!")
            return None

        request = GetPositionFK.Request()
        request.header = Header()
        request.header.frame_id = 'base_link'
        request.header.stamp = self.get_clock().now().to_msg()
        request.fk_link_names = ['tool0']  # Ensure this matches your URDF end-effector link

        request.robot_state = RobotState()
        request.robot_state.joint_state.name = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        request.robot_state.joint_state.position = joint_positions

        future = self.fk_client.call_async(request)

        try:
            while not future.done():
                time.sleep(0.01)

            response = future.result()
            if response.error_code.val == 1:  # SUCCESS
                return response.pose_stamped[0].pose
            else:
                self.get_logger().error(f"FK Failed code: {response.error_code.val}")
                return None
        except Exception as e:
            self.get_logger().error(f"FK Service Call Failed: {e}")
            return None

    def get_ik(self, target_pose):
        """Synchronous wrapper for IK"""
        if not self.ik_client.service_is_ready():
            self.get_logger().error("IK Service not ready!")
            return None

        ik_request = PositionIKRequest()
        ik_request.group_name = 'ur_manipulator'
        ik_request.robot_state.joint_state.name = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        # Seed with current or home position
        ik_request.robot_state.joint_state.position = self.current_joint_positions if self.current_joint_positions else self.HOME_JOINT_POSITIONS

        ik_request.pose_stamped = PoseStamped()
        ik_request.pose_stamped.header.frame_id = 'base_link'
        ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        ik_request.pose_stamped.pose = target_pose
        ik_request.timeout.sec = 5

        req = GetPositionIK.Request()
        req.ik_request = ik_request

        future = self.ik_client.call_async(req)

        try:
            while not future.done():
                time.sleep(0.01)

            response = future.result()
            if response.error_code.val == 1:
                return list(response.solution.joint_state.position)
            else:
                self.get_logger().error(f"IK Failed code: {response.error_code.val}")
                return None
        except Exception as e:
            self.get_logger().error(f"IK Service Call Failed: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = Ur5eIKMover()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        # --- Wait for connections ---
        node.get_logger().info("Waiting for FK/IK services...")
        if not node.ik_client.wait_for_service(timeout_sec=10.0) or not node.fk_client.wait_for_service(
                timeout_sec=10.0):
            node.get_logger().error("Services not available! Ensure MoveIt is running.")
            return

        node.get_logger().info("Waiting for Joint States...")
        while not node.current_joint_positions:
            time.sleep(0.1)

        # --- STEP 1: Go Home ---
        node.get_logger().info("--- STEP 1: Moving Home ---")
        node.move_robot(node.HOME_JOINT_POSITIONS)
        time.sleep(6.0)

        # --- STEP 2: SET ORIGIN (Get Home FK) ---
        node.get_logger().info("--- STEP 2: Setting HOME as ORIGIN ---")
        current_joints = node.current_joint_positions
        home_pose = node.get_fk(current_joints)

        if not home_pose:
            node.get_logger().error("Could not compute FK for Home. Exiting.")
            return

        # Store Home coordinates
        HOME_X = home_pose.position.x
        HOME_Y = home_pose.position.y
        HOME_Z = home_pose.position.z

        node.get_logger().info(f"ORIGIN SET AT: X={HOME_X:.4f}, Y={HOME_Y:.4f}, Z={HOME_Z:.4f}")

        # --- STEP 3: Read CSV and Calculate ALL Plans ---
        csv_path = '/home/yashrakesh/ros2_ws/src/ur_description/data/motion.csv'

        if not os.path.exists(csv_path):
            node.get_logger().error(f"CSV file not found at: {csv_path}")
            return

        node.get_logger().info(f"Reading motion points from: {csv_path}")

        # Buffer to store all successful joint solutions
        all_joint_solutions = []

        with open(csv_path, 'r') as csvfile:
            reader = csv.DictReader(csvfile)

            for i, row in enumerate(reader):
                try:
                    # ==========================================
                    # READ INPUT FROM CSV
                    # ==========================================
                    input_x = float(row['x'])/10
                    input_y = float(row['y'])/10
                    input_z = 0.567  # Default Z offset
                    # ==========================================

                    # --- Calculate Absolute Target ---
                    target_pose = Pose()
                    target_pose.position.x = HOME_X - input_x
                    target_pose.position.y = HOME_Y - input_y
                    target_pose.position.z = input_z

                    # Keep Home Orientation
                    target_pose.orientation = home_pose.orientation

                    # --- Solve IK ---
                    # Note: This might take a moment for many points, but it ensures safety
                    ik_solution = node.get_ik(target_pose)

                    if ik_solution:
                        # Slice to 6 joints (remove mop joints)
                        ur5e_joints_only = ik_solution[:6]
                        all_joint_solutions.append(ur5e_joints_only)
                        # Optional: Print every 10th point so logs aren't spammy
                        if i % 10 == 0:
                            node.get_logger().info(f"Planned Point {i + 1}...")
                    else:
                        node.get_logger().warn(f"No IK solution for Point {i + 1}. Skipping.")

                except ValueError as e:
                    node.get_logger().error(f"Error reading row {i}: {e}")
                    continue

        # --- STEP 4: Execute Full Trajectory ---
        if len(all_joint_solutions) > 0:
            node.get_logger().info(f"--- STEP 4: Executing Trajectory with {len(all_joint_solutions)} points ---")

            # Use a time step of 0.1s (100ms) per point.
            # If motion is too fast/jerky, increase this to 0.2 or 0.5
            node.move_trajectory(all_joint_solutions, time_step=0.01)

            # Wait enough time for the full trajectory to finish
            total_duration = len(all_joint_solutions) * 0.1
            time.sleep(total_duration + 0.1)

            node.get_logger().info("Trajectory Execution Complete.")
        else:
            node.get_logger().error("No valid points found in CSV!")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()