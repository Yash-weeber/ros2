# # import rclpy
# # from rclpy.node import Node
# # from rclpy.duration import Duration
# # from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# # from moveit_msgs.srv import GetPositionFK
# # from moveit_msgs.msg import RobotState
# # from sensor_msgs.msg import JointState
# # from std_msgs.msg import Header
# # import threading
# # import time
# #
# #
# # class Ur5eHomeTest(Node):
# #     def __init__(self):
# #         super().__init__('ur5e_home_test')
# #
# #         # 1. Targeted Controller for Robot 1
# #         self.publisher_ = self.create_publisher(
# #             JointTrajectory,
# #             '/ur_1_controller/joint_trajectory',
# #             10
# #         )
# #
# #         # 2. Subscribe to Joint States to know when we are ready
# #         self.joint_state_sub = self.create_subscription(
# #             JointState,
# #             '/joint_states',
# #             self.joint_state_callback,
# #             10
# #         )
# #
# #         # 3. FK Client (to print the XYZ location of Home)
# #         self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
# #
# #         # 4. Exact Joint Values from your Screenshot
# #         self.HOME_JOINT_POSITIONS = [
# #             -1.359,  # shoulder_pan
# #             -2.221,  # shoulder_lift
# #             -0.760,  # elbow
# #             -1.823,  # wrist_1
# #             1.558,  # wrist_2
# #             0.033  # wrist_3
# #         ]
# #
# #         self.current_joint_positions = []
# #
# #     def joint_state_callback(self, msg):
# #         # Update current positions for Robot 1
# #         # Note: We check for 'shoulder_pan_joint' which is the standard name for Robot 1
# #         if 'shoulder_pan_joint' in msg.name:
# #             try:
# #                 temp_positions = []
# #                 # Order matters!
# #                 joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
# #                                'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
# #                 for name in joint_names:
# #                     idx = msg.name.index(name)
# #                     temp_positions.append(msg.position[idx])
# #                 self.current_joint_positions = temp_positions
# #             except ValueError:
# #                 pass
# #
# #     def move_to_home(self):
# #         msg = JointTrajectory()
# #         msg.joint_names = [
# #             'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
# #             'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
# #         ]
# #         point = JointTrajectoryPoint()
# #         point.positions = self.HOME_JOINT_POSITIONS
# #         point.time_from_start = Duration(seconds=5.0).to_msg()  # 5 second move time
# #         msg.points.append(point)
# #
# #         self.get_logger().info("Publishing Trajectory to /ur_1_controller...")
# #         self.publisher_.publish(msg)
# #
# #     def get_fk_for_home(self):
# #         if not self.fk_client.wait_for_service(timeout_sec=2.0):
# #             self.get_logger().warn("MoveIt FK Service not available. Cannot print XYZ coordinates.")
# #             return
# #
# #         request = GetPositionFK.Request()
# #         request.header = Header()
# #         request.header.frame_id = 'base_link'  # Robot 1 Base
# #         request.header.stamp = self.get_clock().now().to_msg()
# #         request.fk_link_names = ['tool0']
# #
# #         request.robot_state = RobotState()
# #         request.robot_state.joint_state.name = [
# #             'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
# #             'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
# #         ]
# #         request.robot_state.joint_state.position = self.HOME_JOINT_POSITIONS
# #
# #         future = self.fk_client.call_async(request)
# #
# #         # Simple blocking wait
# #         start = time.time()
# #         while not future.done():
# #             time.sleep(0.05)
# #             if time.time() - start > 3.0:
# #                 self.get_logger().error("FK Service Timeout.")
# #                 return
# #
# #         try:
# #             response = future.result()
# #             if response.error_code.val == 1:
# #                 pos = response.pose_stamped[0].pose.position
# #                 self.get_logger().info(f" HOME LOCATION CONFIRMED: X={pos.x:.4f}, Y={pos.y:.4f}, Z={pos.z:.4f}")
# #             else:
# #                 self.get_logger().error(f"FK Error Code: {response.error_code.val}")
# #         except Exception as e:
# #             self.get_logger().error(f"FK Exception: {e}")
# #
# #
# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = Ur5eHomeTest()
# #
# #     # Thread to handle callbacks
# #     executor = rclpy.executors.SingleThreadedExecutor()
# #     executor.add_node(node)
# #     spin_thread = threading.Thread(target=executor.spin, daemon=True)
# #     spin_thread.start()
# #
# #     try:
# #         # 1. Wait for Joint States (Confirm Controller is Alive)
# #         node.get_logger().info("Waiting for Robot 1 Joint States...")
# #         while not node.current_joint_positions:
# #             time.sleep(0.1)
# #         node.get_logger().info("Robot 1 Connected.")
# #
# #         # 2. Move to Home
# #         node.get_logger().info("--- Moving Robot 1 to Home Position ---")
# #         node.move_to_home()
# #
# #         # 3. Wait for move to finish (buffer time)
# #         time.sleep(6.0)
# #
# #         # 4. Print the Physical Location
# #         node.get_fk_for_home()
# #
# #         node.get_logger().info("--- Test Complete. Script Exiting. ---")
# #
# #     except KeyboardInterrupt:
# #         pass
# #     finally:
# #         node.destroy_node()
# #         rclpy.shutdown()
# #
# #
# # if __name__ == '__main__':
# #     main()
#
#
#
# # ------------------------------------------------------
# import rclpy
# from rclpy.node import Node
# from rclpy.duration import Duration
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from moveit_msgs.srv import GetPositionIK, GetPositionFK
# from moveit_msgs.msg import PositionIKRequest, RobotState
# from geometry_msgs.msg import PoseStamped, Pose, Quaternion
# from sensor_msgs.msg import JointState
# import threading
# import time
# import math
# import numpy as np
# import os
# import pandas as pd
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
# class FinalSmartSolver(Node):
#     def __init__(self):
#         super().__init__('final_smart_solver')
#
#         self.publisher_ = self.create_publisher(JointTrajectory, '/ur_1_controller/joint_trajectory', 10)
#         self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
#         self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
#         self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
#
#         self.JOINT_NAMES = [
#             'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
#             'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
#         ]
#         self.HOME_JOINTS = [-1.359, -1.95, -0.760, -1.823, 1.558, 0.033]
#         self.current_joints = []
#
#     def joint_callback(self, msg):
#         if 'shoulder_pan_joint' in msg.name:
#             try:
#                 temp = []
#                 for n in self.JOINT_NAMES:
#                     temp.append(msg.position[msg.name.index(n)])
#                 self.current_joints = temp
#             except ValueError:
#                 pass
#
#     def get_fk(self, joints):
#         req = GetPositionFK.Request()
#         req.header.frame_id = 'base_link'  # Getting coordinates relative to Robot Base
#         req.fk_link_names = ['mop_head']
#         req.robot_state.joint_state.name = self.JOINT_NAMES
#         req.robot_state.joint_state.position = joints
#
#         future = self.fk_client.call_async(req)
#         start = time.time()
#         while not future.done():
#             time.sleep(0.01)
#             if time.time() - start > 1.0: return None
#         try:
#             res = future.result()
#             if res.error_code.val == 1: return res.pose_stamped[0].pose
#         except:
#             pass
#         return None
#
#     def send_trajectory(self, positions, duration=4.0):
#         msg = JointTrajectory()
#         msg.joint_names = self.JOINT_NAMES
#         point = JointTrajectoryPoint()
#         point.positions = positions
#         point.time_from_start = Duration(seconds=duration).to_msg()
#         msg.points.append(point)
#         self.publisher_.publish(msg)
#
#     def move_to_home(self):
#         print("[ACTION] Moving to Home Position...")
#         self.send_trajectory(self.HOME_JOINTS, duration=4.0)
#         time.sleep(4.5)
#
#         pose = self.get_fk(self.HOME_JOINTS)
#         if pose:
#             print(
#                 f"[HOME] Robot Frame Coords: X={pose.position.x:.3f}, Y={pose.position.y:.3f}, Z={pose.position.z:.3f}")
#
#     def get_ik(self, target_pose):
#         req = GetPositionIK.Request()
#         req.ik_request.group_name = 'ur_manipulator'
#         req.ik_request.robot_state.joint_state.name = self.JOINT_NAMES
#         req.ik_request.robot_state.joint_state.position = self.HOME_JOINTS  # Seed with Home
#         req.ik_request.pose_stamped.header.frame_id = 'base_link'
#         req.ik_request.timeout.sec = 1
#         req.ik_request.avoid_collisions = False  # FORCE MOVE
#
#         # Try 8 wrist angles
#         for yaw in [0, 90, -90, 180, 45, -45, 135, -135]:
#             q = quaternion_from_euler(math.radians(180), 0, math.radians(yaw))
#             target_pose.orientation = q
#             req.ik_request.pose_stamped.pose = target_pose
#
#             future = self.ik_client.call_async(req)
#             start = time.time()
#             while not future.done():
#                 time.sleep(0.01)
#                 if time.time() - start > 0.5: break
#
#             try:
#                 res = future.result()
#                 if res.error_code.val == 1: return list(res.solution.joint_state.position)[:6]
#             except:
#                 pass
#         return None
#
#
# def main():
#     rclpy.init()
#     node = FinalSmartSolver()
#
#     executor = rclpy.executors.SingleThreadedExecutor()
#     executor.add_node(node)
#     thread = threading.Thread(target=executor.spin, daemon=True)
#     thread.start()
#
#     while not node.current_joints: time.sleep(0.1)
#
#     node.move_to_home()
#
#     print("------------------------------------------------------------")
#     print(" FINAL SMART SOLVER")
#     print(" Input: X Y (Relative to TABLE CENTER)")
#     print(" Z-Height: Automatically set to -0.21 (Table Surface)")
#     print("------------------------------------------------------------")
#     home_dir = os.path.expanduser('~')
#     csv_path = os.path.join(home_dir, 'ros2_ws', 'data', 'motion.csv')
#     if not os.path.exists(csv_path):
#         print(f"[ERROR] CSV not found at: {csv_path}")
#         return
#
#     print(f"[INFO] Reading motion file: {csv_path}")
#
#     try:
#         # Read the CSV file
#         df = pd.read_csv(csv_path)
#
#         # Loop through every row in the CSV
#         for index, row in df.iterrows():
#             # Get X and Y from the file
#             tx = float(row['x'])
#             ty = float(row['y'])
#
#             # --- YOUR MATH ---
#             rx = tx
#             ry = ty + 0.1  # Your offset
#             rz = 0.6  # Your Z height
#
#             print(f"[STEP {index}] Target: X={rx:.3f} Y={ry:.3f} Z={rz:.3f}")
#
#             pose = Pose()
#             pose.position.x = rx
#             pose.position.y = ry
#             pose.position.z = rz
#
#             # Calculate IK
#             sol = node.get_ik(pose)
#
#             if sol:
#                 print(f"   [SUCCESS] Moving...")
#                 # Move slightly faster (2.0s) since we have many points
#                 node.send_trajectory(sol, duration=2.0)
#                 time.sleep(2.1)
#             else:
#                 print("   [ERROR] Unreachable point in CSV.")
#
#     except Exception as e:
#         print(f"[ERROR] Something went wrong reading the CSV: {e}")
#
#     # while True:
#     #     try:
#     #         raw = input("\nEnter Table X Y: ")
#     #         if raw == 'q': break
#     #         parts = raw.split()
#     #         if len(parts) != 2: continue
#     #
#     #         tx, ty = map(float, parts)
#     #
#     #         # 1. Transform Table Center -> Robot Base
#     #         # Robot Base is Y=-0.2. Table Center is Y=0.5.
#     #         # We add 0.7 to Y.
#     #         rx = tx
#     #         ry = ty  + 0.1
#     #
#     #         # 2. Force Z Downwards
#     #         # Robot Base is 0.81m high. Table is 0.60m high.
#     #         # Difference is -0.21m.
#     #         rz = 0.6
#     #
#     #         dist = math.sqrt(rx ** 2 + ry ** 2 + rz ** 2)
#     #         print(f"[INFO] Target: X={rx:.3f} Y={ry:.3f} Z={rz:.3f} (Dist: {dist:.3f}m)")
#     #
#     #         pose = Pose()
#     #         pose.position.x = rx
#     #         pose.position.y = ry
#     #         pose.position.z = rz
#     #
#     #         sol = node.get_ik(pose)
#     #
#     #         if sol:
#     #             print("[SUCCESS] Moving...")
#     #             node.send_trajectory(sol)
#     #             time.sleep(4.0)
#     #         else:
#     #             print("[ERROR] Unreachable.")
#     #
#     #     except:
#     #         break
#
#     node.destroy_node()
#     rclpy.shutdown()
#
#
# if __name__ == '__main__':
#     main()



import os
import csv

def resample_trajectory():
    home_dir = os.path.expanduser('~')
    input_csv = os.path.join(home_dir, 'ros2_ws', 'data', 'motion.csv')
    output_csv = os.path.join(home_dir, 'ros2_ws', 'data', 'motion_safe.csv')

    # 1. Read Original Data
    original_points = []
    if not os.path.exists(input_csv):
        print(f"Error: {input_csv} not found!")
        return

    with open(input_csv, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            original_points.append({
                'iter': row['iter'],
                'step': row['step'],
                'x': float(row['x']),
                'y': float(row['y']),
                'timestamp': row['timestamp']
            })

    if not original_points:
        print("CSV is empty!")
        return

    # 2. Analyze Original Shape
    x_vals = [p['x'] for p in original_points]
    y_vals = [p['y'] for p in original_points]

    min_x, max_x = min(x_vals), max(x_vals)
    min_y, max_y = min(y_vals), max(y_vals)

    orig_width = max_x - min_x if max_x != min_x else 1.0
    orig_height = max_y - min_y if max_y != min_y else 1.0

    # 3. Define your NEW requested boundaries
    SAFE_MIN_X = -0.5
    SAFE_MAX_X = 0.5
    SAFE_MIN_Y = 0.0
    SAFE_MAX_Y = 0.6

    safe_width = SAFE_MAX_X - SAFE_MIN_X
    safe_height = SAFE_MAX_Y - SAFE_MIN_Y

    # Calculate scaling factor to preserve aspect ratio
    scale = min(safe_width / orig_width, safe_height / orig_height)

    # 4. Apply Scaling and Translation
    print(f"Original X Range: {min_x:.2f} to {max_x:.2f}")
    print(f"Original Y Range: {min_y:.2f} to {max_y:.2f}")
    print(f"Scaling all points by factor: {scale:.4f}")

    with open(output_csv, 'w', newline='') as f:
        fieldnames = ['iter', 'step', 'x', 'y', 'timestamp']
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for p in original_points:
            # Shift to origin (0,0) -> Scale -> Move to your specific Range
            # This ensures the drawing is centered within your max/min box
            new_x = (p['x'] - min_x) * scale + SAFE_MIN_X
            new_y = (p['y'] - min_y) * scale + SAFE_MIN_Y

            writer.writerow({
                'iter': p['iter'],
                'step': p['step'],
                'x': f"{new_x:.5f}",
                'y': f"{new_y:.5f}",
                'timestamp': p['timestamp']
            })

    print(f"\n[SUCCESS] Saved safe trajectory to: {output_csv}")
    print(f"New X Range: {SAFE_MIN_X} to {SAFE_MAX_X}")
    print(f"New Y Range: {SAFE_MIN_Y} to {SAFE_MAX_Y}")

if __name__ == '__main__':
    resample_trajectory()