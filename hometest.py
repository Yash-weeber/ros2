# # import rclpy
# # from rclpy.node import Node
# # from rclpy.duration import Duration
# # from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# # from moveit_msgs.srv import GetPositionIK, GetPositionFK
# # from moveit_msgs.msg import PositionIKRequest
# # from geometry_msgs.msg import PoseStamped, Pose, Quaternion
# # from sensor_msgs.msg import JointState
# # import threading
# # import time
# # import math
# # import numpy as np
# #
# #
# # def get_mop_down_orientation():
# #     # Points the tool straight down
# #     roll, pitch, yaw = math.radians(180), 0, 0
# #     qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
# #     qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
# #     qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
# #     qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
# #     return Quaternion(x=qx, y=qy, z=qz, w=qw)
# #
# #
# # class TracIKSolver(Node):
# #     def __init__(self):
# #         super().__init__('tracik_solver')
# #         self.publisher_ = self.create_publisher(JointTrajectory, '/ur_1_controller/joint_trajectory', 10)
# #         self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
# #         self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
# #         self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
# #
# #         self.JOINT_NAMES = [
# #             'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
# #             'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
# #         ]
# #         self.HOME_JOINTS = [-1.359, -2.221, -0.760, -1.823, 1.558, 0.033]
# #         self.current_joints = []
# #
# #     def joint_callback(self, msg):
# #         if 'shoulder_pan_joint' in msg.name:
# #             try:
# #                 temp = []
# #                 for n in self.JOINT_NAMES:
# #                     temp.append(msg.position[msg.name.index(n)])
# #                 self.current_joints = temp
# #             except ValueError:
# #                 pass
# #
# #     def send_trajectory(self, positions, duration=4.0):
# #         msg = JointTrajectory()
# #         msg.joint_names = self.JOINT_NAMES
# #         point = JointTrajectoryPoint()
# #         point.positions = positions
# #         point.time_from_start = Duration(seconds=duration).to_msg()
# #         msg.points.append(point)
# #         self.publisher_.publish(msg)
# #
# #     def move_to_home(self):
# #         print("[ACTION] Moving to Home Position...")
# #         self.send_trajectory(self.HOME_JOINTS, duration=4.0)
# #         time.sleep(4.5)
# #
# #     def get_ik(self, target_pose):
# #         req = GetPositionIK.Request()
# #         req.ik_request.group_name = 'ur_manipulator'
# #         req.ik_request.robot_state.joint_state.name = self.JOINT_NAMES
# #         req.ik_request.robot_state.joint_state.position = self.HOME_JOINTS
# #         req.ik_request.pose_stamped.header.frame_id = 'table_link'
# #         req.ik_request.timeout.sec = 1
# #
# #         # --- STRATEGY 1 CONFIGURATION ---
# #         req.ik_request.ik_link_name = "mop_head"  # Target the MOP TIP
# #         req.ik_request.avoid_collisions = False  # Allow tip to touch table
# #
# #         req.ik_request.pose_stamped.pose = target_pose
# #         future = self.ik_client.call_async(req)
# #
# #         start = time.time()
# #         while not future.done():
# #             time.sleep(0.01)
# #             if time.time() - start > 1.0: return None
# #
# #         try:
# #             res = future.result()
# #             if res.error_code.val == 1:
# #                 return list(res.solution.joint_state.position)[:6]
# #             else:
# #                 print(f"[TRAC-IK FAIL] Error Code: {res.error_code.val}")
# #         except Exception as e:
# #             print(f"[ERROR] Service Call Failed: {e}")
# #         return None
# #
# #
# # def main():
# #     rclpy.init()
# #     node = TracIKSolver()
# #
# #     executor = rclpy.executors.SingleThreadedExecutor()
# #     executor.add_node(node)
# #     thread = threading.Thread(target=executor.spin, daemon=True)
# #     thread.start()
# #
# #     print("[INFO] Waiting for connection...")
# #     while not node.current_joints: time.sleep(0.1)
# #
# #     node.move_to_home()
# #
# #     print("------------------------------------------------------------")
# #     print(" TRAC-IK SOLVER (Corrected Geometry)")
# #     print(" Origin: TABLE CENTER (0,0)")
# #     print("------------------------------------------------------------")
# #
# #     while True:
# #         try:
# #             raw = input("\nEnter Table X Y: ")
# #             if raw == 'q': break
# #             parts = raw.split()
# #             if len(parts) != 2: continue
# #
# #             tx, ty = map(float, parts)
# #
# #             # --- 1. Y-OFFSET FIX ---
# #             # Robot Base is at -0.2. Table Center is at 0.5.
# #             # You MUST add 0.7 to reach the table center.
# #             # Adding 0.2 only reaches empty space in front of the robot.
# #             rx = tx
# #             ry = ty + 0.2
# #
# #             # --- 2. Z-HEIGHT FIX ---
# #             # Since we target 'mop_head', we MUST target the table surface (-0.21).
# #             # If you set this to 0.45, the robot tries to reach 1.2m high and fails.
# #             rz = 0.45
# #
# #             dist = math.sqrt(rx ** 2 + ry ** 2 + rz ** 2)
# #             print(f"[INFO] Requesting: X={rx:.3f} Y={ry:.3f} Z={rz:.3f} (Reach: {dist:.3f}m)")
# #
# #             pose = Pose()
# #             pose.position.x = rx
# #             pose.position.y = ry
# #             pose.position.z = rz
# #             pose.orientation = get_mop_down_orientation()
# #
# #             sol = node.get_ik(pose)
# #
# #             if sol:
# #                 print("[SUCCESS] Moving...")
# #                 node.send_trajectory(sol)
# #                 time.sleep(4.0)
# #             else:
# #                 print("[FAILURE] Still Unreachable. Check if 'ry' is too far.")
# #
# #         except KeyboardInterrupt:
# #             break
# #         except ValueError:
# #             print("Invalid Input")
# #
# #     node.destroy_node()
# #     rclpy.shutdown()
# #
# #
# # if __name__ == '__main__':
# #     main()
#
#
# import rclpy
# from rclpy.node import Node
# from rclpy.duration import Duration
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from moveit_msgs.srv import GetPositionIK, GetPositionFK
# from moveit_msgs.msg import PositionIKRequest, RobotState
# from geometry_msgs.msg import PoseStamped, Pose, Quaternion
# from sensor_msgs.msg import JointState
# from std_msgs.msg import Header
# import numpy as np
# import threading
# import time
# import csv
# import os
# import math
#
#
# def quaternion_from_euler(roll, pitch, yaw):
#     qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
#     qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
#     qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
#     qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
#     return Quaternion(x=qx, y=qy, z=qz, w=qw)
#
#
# class Ur5eIKMover(Node):
#     def __init__(self):
#         super().__init__('ur5e_ik_mover')
#
#         self.publisher_ = self.create_publisher(JointTrajectory, '/ur_1_controller/joint_trajectory', 10)
#         self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
#         self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
#         self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
#
#         # Standard Home
#         self.HOME_JOINT_POSITIONS = [-1.359, -1.95, -0.760, -1.823, 1.558, 0.033]
#         self.current_joint_positions = []
#         self.JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
#                             'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
#
#     def joint_state_callback(self, msg):
#         if 'shoulder_pan_joint' in msg.name:
#             try:
#                 temp = []
#                 for n in self.JOINT_NAMES:
#                     temp.append(msg.position[msg.name.index(n)])
#                 self.current_joint_positions = temp
#             except ValueError:
#                 pass
#
#     def move_trajectory(self, list_of_joint_positions, time_step=0.1):
#         """Executes ALL points in one smooth motion"""
#         msg = JointTrajectory()
#         msg.joint_names = self.JOINT_NAMES
#         current_time = 0.0
#
#         for joints in list_of_joint_positions:
#             point = JointTrajectoryPoint()
#             point.positions = joints
#             current_time += time_step
#             point.time_from_start = Duration(seconds=current_time).to_msg()
#             msg.points.append(point)
#
#         self.get_logger().info(f"Publishing trajectory with {len(msg.points)} points...")
#         self.publisher_.publish(msg)
#
#     def move_robot(self, joint_positions, duration_sec=5.0):
#         msg = JointTrajectory()
#         msg.joint_names = self.JOINT_NAMES
#         point = JointTrajectoryPoint()
#         point.positions = joint_positions
#         point.time_from_start = Duration(seconds=duration_sec).to_msg()
#         msg.points.append(point)
#         self.publisher_.publish(msg)
#
#     def get_ik(self, target_pose):
#         req = GetPositionIK.Request()
#         req.ik_request.group_name = 'ur_manipulator'
#         req.ik_request.robot_state.joint_state.name = self.JOINT_NAMES
#         req.ik_request.robot_state.joint_state.position = self.current_joint_positions if self.current_joint_positions else self.HOME_JOINT_POSITIONS
#         req.ik_request.pose_stamped.header.frame_id = 'base_link'
#         req.ik_request.pose_stamped.pose = target_pose
#         req.ik_request.timeout.sec = 1
#
#         # --- CRITICAL FIXES FOR FLOPPY MOP ---
#         req.ik_request.ik_link_name = "tool0"  # Solve for WRIST (Solid)
#         req.ik_request.avoid_collisions = False  # Allow it
#
#         future = self.ik_client.call_async(req)
#         start = time.time()
#         while not future.done():
#             time.sleep(0.01)
#             if time.time() - start > 1.0: return None  # Timeout
#
#         try:
#             res = future.result()
#             if res.error_code.val == 1:
#                 return list(res.solution.joint_state.position)[:6]
#         except:
#             pass
#         return None
#
#
# def main(args=None):
#     rclpy.init(args=args)
#     node = Ur5eIKMover()
#     executor = rclpy.executors.SingleThreadedExecutor()
#     executor.add_node(node)
#     spin_thread = threading.Thread(target=executor.spin, daemon=True)
#     spin_thread.start()
#
#     # Wait for connection
#     while not node.current_joint_positions: time.sleep(0.1)
#
#     # 1. Move Home
#     node.get_logger().info("Moving Home...")
#     node.move_robot(node.HOME_JOINT_POSITIONS)
#     time.sleep(5.0)
#
#     # 2. Dynamic CSV Path
#     home_dir = os.path.expanduser('~')
#     csv_path = os.path.join(home_dir, 'ros2_ws', 'data', 'motion.csv')
#
#     if not os.path.exists(csv_path):
#         node.get_logger().error(f"CSV not found: {csv_path}")
#         return
#
#     node.get_logger().info(f"Processing CSV: {csv_path}")
#     all_joint_solutions = []
#
#     # 3. Read & Calculate
#     with open(csv_path, 'r') as csvfile:
#         reader = csv.DictReader(csvfile)
#         for i, row in enumerate(reader):
#             try:
#                 # --- YOUR MATH ---
#                 tx = float(row['x'])
#                 ty = float(row['y'])
#
#                 # Apply Offsets
#                 rx = tx
#                 ry = ty + 0.1
#                 rz = 0.6
#
#                 # Create Pose
#                 pose = Pose()
#                 pose.position.x = rx
#                 pose.position.y = ry
#                 pose.position.z = rz
#                 # Point wrist down
#                 pose.orientation = quaternion_from_euler(math.radians(180), 0, 0)
#
#                 # Solve IK
#                 sol = node.get_ik(pose)
#                 if sol:
#                     all_joint_solutions.append(sol)
#                     print(f"Planned Point {i}...", end='\r')
#
#             except Exception as e:
#                 print(f"Skipping row {i}: {e}")
#
#     # 4. Execute All
#     if all_joint_solutions:
#         node.get_logger().info(f"Executing {len(all_joint_solutions)} points...")
#         # time_step=0.1 means 10 points per second (smooth)
#         node.move_trajectory(all_joint_solutions, time_step=0.1)
#         time.sleep(len(all_joint_solutions) * 0.1 + 2.0)
#     else:
#         node.get_logger().error("No valid points found!")
#
#     node.destroy_node()
#     rclpy.shutdown()
#
#
# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from moveit_msgs.msg import PositionIKRequest, RobotState
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from sensor_msgs.msg import JointState
import numpy as np
import threading
import time
import csv
import os
import math


# --- 1. Math Helper ---
def quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


class Ur2IKMover(Node):
    def __init__(self):
        super().__init__('ur2_ik_mover')
        self.publisher_ = self.create_publisher(JointTrajectory, '/ur_2_controller/joint_trajectory', 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

        # UR2 Home
        self.HOME_JOINT_POSITIONS = [1.359, 0.033, 1.277, 0.033, 1.359, 0.099]
        self.current_joint_positions = []
        self.JOINT_NAMES = ['ur_2_shoulder_pan_joint', 'ur_2_shoulder_lift_joint', 'ur_2_elbow_joint',
                            'ur_2_wrist_1_joint', 'ur_2_wrist_2_joint', 'ur_2_wrist_3_joint']

    def joint_state_callback(self, msg):
        if 'ur_2_shoulder_pan_joint' in msg.name:
            try:
                self.current_joint_positions = [msg.position[msg.name.index(n)] for n in self.JOINT_NAMES]
            except ValueError:
                pass

    def move_robot(self, joint_positions, duration_sec=4.0):
        msg = JointTrajectory()
        msg.joint_names = self.JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = Duration(seconds=duration_sec).to_msg()
        msg.points.append(point)
        self.publisher_.publish(msg)

    def get_ik(self, target_pose):
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'ur_2_manipulator'
        req.ik_request.robot_state.joint_state.name = self.JOINT_NAMES
        req.ik_request.robot_state.joint_state.position = self.current_joint_positions if self.current_joint_positions else self.HOME_JOINT_POSITIONS
        req.ik_request.pose_stamped.header.frame_id = 'ur_2_base_link'
        req.ik_request.pose_stamped.pose = target_pose
        req.ik_request.timeout.sec = 1
        req.ik_request.ik_link_name = "ur_2_tool0"
        req.ik_request.avoid_collisions = False

        future = self.ik_client.call_async(req)
        start = time.time()
        while not future.done():
            time.sleep(0.01)
            if time.time() - start > 1.0: return None
        try:
            res = future.result()
            if res.error_code.val == 1: return list(res.solution.joint_state.position)[:6]
        except:
            pass
        return None


def main(args=None):
    rclpy.init(args=args)
    node = Ur2IKMover()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    t = threading.Thread(target=executor.spin, daemon=True)
    t.start()

    print("Waiting for Joint States...")
    while not node.current_joint_positions: time.sleep(0.1)

    print("Moving Home...")
    node.move_robot(node.HOME_JOINT_POSITIONS)
    time.sleep(5.0)

    print("\n=============================================")
    print("      UR2 GRAPH PLOTTER (URDF Corrected)    ")
    print("=============================================")
    print(" [1] Batch CSV Mode")
    print(" [2] Single Point Mode")
    mode = input("Select: ")

    if mode == '2':
        while True:
            raw = input("\nEnter Table X Y (e.g., 0 0): ")
            if raw == 'q': break
            try:
                tx, ty = map(float, raw.split())

                # --- THE CORRECT MATH FROM YOUR URDF ---
                # URDF Line 54: UR2 Base Y = 0.8
                # URDF Line 63: Table Center Y = 0.5
                # Diff = 0.3

                rx = -tx  # Flip X (180 deg rotation)
                ry = 0.3 - ty  # 0.3 is the GAP between Robot and Table Center
                rz = -0.0 # Maintain Wrist Height relative to base

                print(f"[MATH] Input({tx}, {ty}) -> RobotFrame({rx:.3f}, {ry:.3f}, {rz:.3f})")

                p = Pose()
                p.position.x = rx
                p.position.y = ry
                p.position.z = rz
                p.orientation = quaternion_from_euler(math.radians(180), 0, 0)

                sol = node.get_ik(p)
                if sol:
                    print("[SUCCESS] Moving...")
                    node.move_robot(sol)
                    time.sleep(4.0)
                else:
                    print("[ERROR] Out of Reach")
            except Exception as e:
                print(f"Invalid Input: {e}")

    # CSV MODE LOGIC
    elif mode == '1':
        home_dir = os.path.expanduser('~')
        csv_path = os.path.join(home_dir, 'ros2_ws', 'data', 'motion.csv')
        all_solutions = []

        with open(csv_path, 'r') as csvfile:
            reader = csv.DictReader(csvfile)
            for i, row in enumerate(reader):
                try:
                    tx = float(row['x'])
                    ty = float(row['y'])

                    rx = -tx
                    ry = 0.3 - ty
                    rz = 0

                    p = Pose()
                    p.position.x = rx
                    p.position.y = ry
                    p.position.z = rz
                    p.orientation = quaternion_from_euler(math.radians(180), 0, 0)

                    sol = node.get_ik(p)
                    if sol: all_solutions.append(sol)
                    if i % 50 == 0: print(f"Planned {i}...", end='\r')
                except:
                    pass

        if all_solutions:
            print(f"\nExecuting {len(all_solutions)} points...")
            node.move_trajectory(all_solutions, time_step=0.1)
            time.sleep(len(all_solutions) * 0.1 + 2.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()