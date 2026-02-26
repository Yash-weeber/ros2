# import rclpy
# from rclpy.node import Node
# from moveit_msgs.srv import GetPositionIK
# from sensor_msgs.msg import JointState
# from geometry_msgs.msg import PoseStamped
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from rclpy.duration import Duration
# import time
#
#
# class MopIKDiagnostic(Node):
#     def __init__(self):
#         super().__init__('mop_ik_diagnostic')
#         # Service client for MoveIt Inverse Kinematics
#         self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
#
#         # Publisher for the active simulation controller
#         self.joint_pub = self.create_publisher(JointTrajectory, '/ur_1_controller/joint_trajectory', 10)
#
#         # Subscription to track the current position of the robot
#         self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
#
#         self.current_state = None
#         self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
#                             'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
#
#         # YOUR SPECIFIED HOME POSITION
#         self.ur1_home = [1.558, -1.24, 1.051, -1.425, -1.690, 0.033]
#         self.CONST_Z = 1.1
#
#     def joint_cb(self, msg):
#         # Identify the relevant joints from the state stream
#         if all(n in msg.name for n in self.joint_names):
#             self.current_state = msg
#
#     def go_home(self):
#         """Checks if the home position is safe before moving."""
#         print("ACTION: Verifying Home position safety...")
#
#         # Setup safety check request
#         req = GetPositionIK.Request()
#         req.ik_request.group_name = "ur_manipulator"
#         req.ik_request.robot_state.joint_state.name = self.joint_names
#         req.ik_request.robot_state.joint_state.position = self.ur1_home
#         req.ik_request.avoid_collisions = False  # SAFETY CHECK ACTIVE
#
#         # Call MoveIt to check for collisions at home
#         future = self.ik_client.call_async(req)
#         rclpy.spin_until_future_complete(self, future)
#         res = future.result()
#
#         if res and res.error_code.val == 1:
#             print("SUCCESS: Home position is safe. Moving now...")
#             self.execute(self.ur1_home, travel_time=3.0)
#         else:
#             # If this prints, your home position is physically hitting something in Gazebo
#             print(f"CRITICAL: Home position BLOCKED by collision (Code: {res.error_code.val})")
#             print("Suggestion: Change your ur1_home values to avoid the table/itself.")
#
#         time.sleep(3.0)
#
#     def test_move(self, x, y):
#         if not self.current_state:
#             print("WAITING: No joint states received yet. Is the simulator running?")
#             return
#
#         req = GetPositionIK.Request()
#         req.ik_request.group_name = "ur_manipulator"
#         req.ik_request.ik_link_name = "tool0"
#         req.ik_request.avoid_collisions = True  # SAFETY ENABLED
#         req.ik_request.robot_state.joint_state = self.current_state
#
#         target = PoseStamped()
#         target.header.frame_id = "world"
#         target.pose.position.x = x
#         target.pose.position.y = y
#         target.pose.position.z = self.CONST_Z
#         target.pose.orientation.x = 1.0
#         target.pose.orientation.w = 0.0
#
#         req.ik_request.pose_stamped = target
#
#         future = self.ik_client.call_async(req)
#         rclpy.spin_until_future_complete(self, future)
#         res = future.result()
#
#         if res and res.error_code.val == 1:
#             # Reorder solution joints to match our controller names
#             ordered_joints = [res.solution.joint_state.position[res.solution.joint_state.name.index(n)] for n in
#                               self.joint_names]
#             print(f"SUCCESS | Moving to X:{x} Y:{y} (Z:{self.CONST_Z})")
#             self.execute(ordered_joints)
#         else:
#             print(f"FAILED | Code: {res.error_code.val if res else 'Timeout'}")
#
#     def execute(self, joints, travel_time=2.0):
#         msg = JointTrajectory()
#         msg.joint_names = self.joint_names
#         p = JointTrajectoryPoint()
#         p.positions = joints
#         p.time_from_start = Duration(seconds=travel_time).to_msg()
#         msg.points.append(p)
#         self.joint_pub.publish(msg)
#
#
# def main():
#     rclpy.init()
#     node = MopIKDiagnostic()
#
#     print("Initializing system...")
#     while node.current_state is None:
#         rclpy.spin_once(node, timeout_sec=0.1)
#
#     # AUTOMATED HOMING WITH COLLISION CHECK
#     node.go_home()
#
#     print("\n--- MOP XY CONTROL (Z=1.1) ---")
#     try:
#         while rclpy.ok():
#             rclpy.spin_once(node, timeout_sec=0.1)
#             inp = input("Enter x y >> ").split()
#             if len(inp) == 2:
#                 try:
#                     node.test_move(float(inp[0]), float(inp[1]))
#                 except ValueError:
#                     print("Error: Use valid numbers.")
#             elif len(inp) == 1 and inp[0].lower() == 'exit':
#                 break
#     finally:
#         rclpy.shutdown()
#
#
# if __name__ == '__main__':
#     main()
#



# import rclpy
# from rclpy.node import Node
# from moveit_msgs.srv import GetPositionIK
# from sensor_msgs.msg import JointState
# from geometry_msgs.msg import PoseStamped
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from rclpy.duration import Duration
#
#
# class MopIKDiagnostic(Node):
#     def __init__(self):
#         super().__init__('mop_ik_diagnostic')
#         self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
#         # Target the active controller
#         self.joint_pub = self.create_publisher(JointTrajectory, '/ur_1_controller/joint_trajectory', 10)
#         self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
#
#         self.current_state = None
#         # Joint order strictly following the controller config
#         self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
#                             'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
#
#         # Hardcoded Z height as requested
#         self.CONST_Z = 1.1
#
#     def joint_cb(self, msg):
#         if all(n in msg.name for n in self.joint_names):
#             self.current_state = msg
#
#     def test_move(self, x, y):
#         if not self.current_state:
#             print("WAITING: No joint states received yet. Is the simulator running?")
#             return
#
#         req = GetPositionIK.Request()
#         req.ik_request.group_name = "ur_manipulator"
#         req.ik_request.ik_link_name = "tool0"
#         # KEEPING FALSE: To ignore mop-to-handle collisions as discussed
#         req.ik_request.avoid_collisions = False
#         req.ik_request.robot_state.joint_state = self.current_state
#
#         target = PoseStamped()
#         target.header.frame_id = "world"  # Using world frame as per working reference
#         target.pose.position.x = x
#         target.pose.position.y = y
#         target.pose.position.z = self.CONST_Z
#
#         # Mop Orientation: Downward
#         target.pose.orientation.x = 1.0
#         target.pose.orientation.w = 0.0
#
#         req.ik_request.pose_stamped = target
#
#         future = self.ik_client.call_async(req)
#         rclpy.spin_until_future_complete(self, future)
#         res = future.result()
#
#         if res and res.error_code.val == 1:
#             # Map joints to the order needed by ur_1_controller
#             ordered_joints = [res.solution.joint_state.position[res.solution.joint_state.name.index(n)] for n in
#                               self.joint_names]
#             print(f"SUCCESS | Moving to X:{x} Y:{y} (Z fixed at {self.CONST_Z})")
#             print(f"Solved Joints: {[round(j, 2) for j in ordered_joints]}")
#             self.execute(ordered_joints)
#         else:
#             print(f"FAILED | Code: {res.error_code.val if res else 'Timeout'}")
#             print(f"TIP: Point X:{x} Y:{y} at Z:{self.CONST_Z} may be out of reach.")
#
#     def execute(self, joints):
#         msg = JointTrajectory()
#         msg.joint_names = self.joint_names
#         p = JointTrajectoryPoint()
#         p.positions = joints
#         p.time_from_start = Duration(seconds=2.0).to_msg()
#         msg.points.append(p)
#         self.joint_pub.publish(msg)
#
#
# def main():
#     rclpy.init()
#     node = MopIKDiagnostic()
#     try:
#         while rclpy.ok():
#             rclpy.spin_once(node, timeout_sec=0.1)
#             inp = input("Enter x y >> ").split()
#             if len(inp) == 2:
#                 try:
#                     node.test_move(float(inp[0]), float(inp[1]))
#                 except ValueError:
#                     print("Error: Please enter valid numbers.")
#             elif len(inp) == 1 and inp[0].lower() == 'exit':
#                 break
#     finally:
#         rclpy.shutdown()
#
#
# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.duration import Duration
import csv
import os
import time


class MopIKDiagnostic(Node):
    def __init__(self):
        super().__init__('mop_ik_diagnostic')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.joint_pub = self.create_publisher(JointTrajectory, '/ur_1_controller/joint_trajectory', 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

        self.current_state = None
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        self.CONST_Z = 0.8

        # Path setup
        home_dir = os.path.expanduser('~')
        self.csv_path = os.path.join(home_dir, 'ros2_ws', 'data', 'motion_safe.csv')

    def joint_cb(self, msg):
        if all(n in msg.name for n in self.joint_names):
            self.current_state = msg

    def test_move(self, x, y):
        if not self.current_state:
            print("WAITING: No joint states received.")
            return False

        req = GetPositionIK.Request()
        req.ik_request.group_name = "ur_manipulator"
        req.ik_request.ik_link_name = "mop_tip"
        # Safety enabled now that SRDF/Macro is updated
        req.ik_request.avoid_collisions = True
        req.ik_request.robot_state.joint_state = self.current_state

        target = PoseStamped()
        target.header.frame_id = "world"
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = self.CONST_Z
        target.pose.orientation.x = 1.0
        target.pose.orientation.w = 0.0

        req.ik_request.pose_stamped = target

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

        if res and res.error_code.val == 1:
            ordered_joints = [res.solution.joint_state.position[res.solution.joint_state.name.index(n)] for n in
                              self.joint_names]
            print(f"SUCCESS | Point: {x}, {y}")
            self.execute(ordered_joints)
            return True
        else:
            print(f"FAILED | Point {x}, {y} (Code: {res.error_code.val if res else 'Timeout'})")
            return False

    def execute(self, joints):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        p = JointTrajectoryPoint()
        p.positions = joints
        p.time_from_start = Duration(seconds=1.5).to_msg()
        msg.points.append(p)
        self.joint_pub.publish(msg)
        # Sequence timing to ensure the robot reaches the target
        time.sleep(1.8)

    def run_csv(self):
        if not os.path.exists(self.csv_path):
            print(f"ERROR: File not found: {self.csv_path}")
            return

        print(f"READING MOTION: {self.csv_path}")
        with open(self.csv_path, mode='r') as f:
            # Using DictReader to handle columns by name
            reader = csv.DictReader(f)
            for row in reader:
                if not rclpy.ok(): break
                try:
                    # Extract specifically from your 'x' and 'y' columns
                    x_val = float(row['x'])
                    y_val = float(row['y'])
                    self.test_move(x_val, y_val)
                    rclpy.spin_once(self, timeout_sec=0.1)
                except (ValueError, KeyError) as e:
                    print(f"Skipping row due to error: {e}")
        print("\n--- SEQUENCE FINISHED ---")


def main():
    rclpy.init()
    node = MopIKDiagnostic()

    print("\n--- UR5e MOP CONTROL ---")
    print("1: Manual Entry")
    print("2: CSV Sequence (x,y)")
    mode = input("Select Mode >> ")

    try:
        if mode == '2':
            # Seed the state once before starting
            while node.current_state is None:
                rclpy.spin_once(node, timeout_sec=0.1)
            node.run_csv()
        else:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
                inp = input("Enter x y >> ").split()
                if len(inp) == 2:
                    node.test_move(float(inp[0]), float(inp[1]))
                elif len(inp) == 1 and inp[0].lower() == 'exit':
                    break
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()