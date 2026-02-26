# # # # import rclpy
# # # # from rclpy.node import Node
# # # # from rclpy.duration import Duration
# # # # from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# # # # from moveit_msgs.srv import GetPositionIK, GetPositionFK
# # # # from moveit_msgs.msg import PositionIKRequest, RobotState
# # # # from geometry_msgs.msg import PoseStamped, Pose, Quaternion
# # # # from sensor_msgs.msg import JointState
# # # # import numpy as np
# # # # import threading
# # # # import time
# # # # import csv
# # # # import os
# # # # import math
# # # #
# # # #
# # # # def quaternion_from_euler(roll, pitch, yaw):
# # # #     qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
# # # #     qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
# # # #     qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
# # # #     qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
# # # #     return Quaternion(x=qx, y=qy, z=qz, w=qw)
# # # #
# # # #
# # # # class DualRobotRelay(Node):
# # # #     def __init__(self):
# # # #         super().__init__('dual_robot_relay')
# # # #
# # # #         self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
# # # #         self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
# # # #         self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
# # # #
# # # #         # --- ROBOT 1 SETUP ---
# # # #         self.ur1_pub = self.create_publisher(JointTrajectory, '/ur_1_controller/joint_trajectory', 10)
# # # #         self.ur1_joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
# # # #                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
# # # #         self.ur1_home = [-1.359, -1.95, -0.760, -1.823, 1.558, 0.033]  # "Ready" position
# # # #         self.ur1_zero = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # "Flat/Start" position
# # # #         self.ur1_current = []
# # # #         self.ur1_group = 'ur_manipulator'
# # # #         self.ur1_base = 'base_link'
# # # #
# # # #         # --- ROBOT 2 SETUP ---
# # # #         self.ur2_pub = self.create_publisher(JointTrajectory, '/ur_2_controller/joint_trajectory', 10)
# # # #         self.ur2_joints = ['ur_2_shoulder_pan_joint', 'ur_2_shoulder_lift_joint', 'ur_2_elbow_joint',
# # # #                            'ur_2_wrist_1_joint', 'ur_2_wrist_2_joint', 'ur_2_wrist_3_joint']
# # # #         self.ur2_home = [1.359, 0.033, 1.277, 0.033, 1.359, 0.099]
# # # #         self.ur2_current = []
# # # #         self.ur2_group = 'ur_2_manipulator'
# # # #         self.ur2_base = 'ur_2_base_link'
# # # #
# # # #     def joint_state_callback(self, msg):
# # # #         if self.ur1_joints[0] in msg.name:
# # # #             try:
# # # #                 self.ur1_current = [msg.position[msg.name.index(n)] for n in self.ur1_joints]
# # # #             except:
# # # #                 pass
# # # #         if self.ur2_joints[0] in msg.name:
# # # #             try:
# # # #                 self.ur2_current = [msg.position[msg.name.index(n)] for n in self.ur2_joints]
# # # #             except:
# # # #                 pass
# # # #
# # # #     def get_ik(self, target_pose, group, base_link, current_joints, joint_names):
# # # #         req = GetPositionIK.Request()
# # # #         req.ik_request.group_name = group
# # # #         req.ik_request.robot_state.joint_state.name = joint_names
# # # #         req.ik_request.robot_state.joint_state.position = current_joints if current_joints else self.ur1_home
# # # #         req.ik_request.pose_stamped.header.frame_id = base_link
# # # #         req.ik_request.pose_stamped.pose = target_pose
# # # #         req.ik_request.timeout.sec = 1
# # # #         # IMPORTANT: Solving for tool0 (Wrist) because Mop is floppy
# # # #         req.ik_request.ik_link_name = "tool0" if group == 'ur_manipulator' else "ur_2_wrist_3_joint"
# # # #         req.ik_request.avoid_collisions = False
# # # #
# # # #         future = self.ik_client.call_async(req)
# # # #         start = time.time()
# # # #         while not future.done():
# # # #             time.sleep(0.01)
# # # #             if time.time() - start > 1.0: return None
# # # #
# # # #         try:
# # # #             res = future.result()
# # # #             if res.error_code.val == 1:
# # # #                 return list(res.solution.joint_state.position)
# # # #         except:
# # # #             pass
# # # #         return None
# # # #
# # # #     def execute_trajectory(self, publisher, joint_names, list_of_points, time_step=0.1):
# # # #         msg = JointTrajectory()
# # # #         msg.joint_names = joint_names
# # # #         current_time = 0.0
# # # #         for p in list_of_points:
# # # #             pt = JointTrajectoryPoint()
# # # #             if len(p) > 6:
# # # #                 pt.positions = p[:6]
# # # #             else:
# # # #                 pt.positions = p
# # # #             current_time += time_step
# # # #             pt.time_from_start = Duration(seconds=current_time).to_msg()
# # # #             msg.points.append(pt)
# # # #         publisher.publish(msg)
# # # #         return current_time
# # # #
# # # #     def run_robot_task(self, robot_name, publisher, joint_names, home_joints, end_joints, group, base_link, offset_y,offset_z):
# # # #         self.get_logger().info(f"--- STARTING {robot_name} ---")
# # # #
# # # #         # 1. Move to Home/Ready Position
# # # #         self.get_logger().info(f"[{robot_name}] Moving to Start...")
# # # #         self.execute_trajectory(publisher, joint_names, [home_joints], time_step=5.0)
# # # #         time.sleep(5.5)
# # # #
# # # #         # 2. Process CSV Path
# # # #         home_dir = os.path.expanduser('~')
# # # #         csv_path = os.path.join(home_dir, 'ros2_ws', 'data', 'motion.csv')
# # # #         trajectory_points = []
# # # #
# # # #         # Read CSV logic
# # # #         try:
# # # #             with open(csv_path, 'r') as csvfile:
# # # #                 reader = csv.DictReader(csvfile)
# # # #                 for i, row in enumerate(reader):
# # # #                     tx = float(row['x'])
# # # #                     ty = float(row['y'])
# # # #
# # # #                     rx = tx
# # # #                     ry = ty + offset_y
# # # #                     rz = offset_z # Wrist Height (Mop down)
# # # #
# # # #                     pose = Pose()
# # # #                     pose.position.x = rx
# # # #                     pose.position.y = ry
# # # #                     pose.position.z = rz
# # # #                     pose.orientation = quaternion_from_euler(math.radians(180), 0, 0)
# # # #
# # # #                     current = self.ur1_current if robot_name == "UR1" else self.ur2_current
# # # #                     if not current: current = home_joints
# # # #
# # # #                     sol = self.get_ik(pose, group, base_link, current, joint_names)
# # # #                     if sol:
# # # #                         trajectory_points.append(sol)
# # # #                         if i % 50 == 0: print(f"[{robot_name}] Planning {i}...", end='\r')
# # # #         except Exception as e:
# # # #             self.get_logger().error(f"CSV Error: {e}")
# # # #
# # # #         if trajectory_points:
# # # #             self.get_logger().info(f"\n[{robot_name}] Executing Path...")
# # # #             duration = self.execute_trajectory(publisher, joint_names, trajectory_points, time_step=0.1)
# # # #             time.sleep(duration + 2.0)
# # # #
# # # #             # 3. RELAY MOVEMENT: Move to the designated "End" Position (Zero for UR1)
# # # #             self.get_logger().info(f"[{robot_name}] Task Done. Moving to Rest Position: {end_joints}")
# # # #             self.execute_trajectory(publisher, joint_names, [end_joints], time_step=6.0)
# # # #             time.sleep(7.0)  # Wait for move to finish
# # # #             self.get_logger().info(f"[{robot_name}] IS CLEAR. RELAY READY.")
# # # #         else:
# # # #             self.get_logger().error(f"[{robot_name}] No valid path found!")
# # # #
# # # #
# # # # def main(args=None):
# # # #     rclpy.init(args=args)
# # # #     node = DualRobotRelay()
# # # #     executor = rclpy.executors.SingleThreadedExecutor()
# # # #     executor.add_node(node)
# # # #     t = threading.Thread(target=executor.spin, daemon=True)
# # # #     t.start()
# # # #
# # # #     print("Waiting for joint states...")
# # # #     time.sleep(2.0)
# # # #
# # # #     # ==========================
# # # #     #      THE RELAY LOGIC
# # # #     # ==========================
# # # #
# # # #     # STEP 1: UR1
# # # #     # Start: Home. End: ZERO (Flat/Original).
# # # #     node.run_robot_task(
# # # #         "UR1",
# # # #         node.ur1_pub,
# # # #         node.ur1_joints,
# # # #         home_joints=node.ur1_home,
# # # #         end_joints=node.ur1_zero,  # <--- UR1 GOES TO ALL ZEROS HERE
# # # #         group=node.ur1_group,
# # # #         base_link=node.ur1_base,
# # # #         offset_y=0.1,
# # # #         offset_z =0.6
# # # #     )
# # # #
# # # #     print("\n-------------------------------------------")
# # # #     print(" >>> RELAY TRIGGERED: HANDING OFF TO UR2 <<<")
# # # #     print("-------------------------------------------\n")
# # # #
# # # #     # STEP 2: UR2
# # # #     # Start: Home. End: Home (Or Zero if you prefer).
# # # #     node.run_robot_task(
# # # #         "UR2",
# # # #         node.ur2_pub,
# # # #         node.ur2_joints,
# # # #         home_joints=node.ur2_home,
# # # #         end_joints=node.ur2_home,  # UR2 stays at Home after done
# # # #         group=node.ur2_group,
# # # #         base_link=node.ur2_base,
# # # #         offset_y=-0.5,
# # # #         offset_z = 0.4
# # # #
# # # #     )
# # # #
# # # #     node.destroy_node()
# # # #     rclpy.shutdown()
# # # #
# # # #
# # # # if __name__ == '__main__':
# # # #     main()
# # #
# # #
# # #
# # #
# # #
# # #
# # #
# # # import rclpy
# # # from rclpy.node import Node
# # # from rclpy.duration import Duration
# # # from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# # # from moveit_msgs.srv import GetPositionIK, GetPositionFK
# # # from moveit_msgs.msg import PositionIKRequest, RobotState
# # # from geometry_msgs.msg import PoseStamped, Pose, Quaternion
# # # from sensor_msgs.msg import JointState
# # # import numpy as np
# # # import threading
# # # import time
# # # import csv
# # # import os
# # # import math
# # #
# # #
# # # def quaternion_from_euler(roll, pitch, yaw):
# # #     qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
# # #     qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
# # #     qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
# # #     qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
# # #     return Quaternion(x=qx, y=qy, z=qz, w=qw)
# # #
# # #
# # # class DualRobotRelay(Node):
# # #     def __init__(self):
# # #         super().__init__('dual_robot_relay')
# # #
# # #         self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
# # #         self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
# # #         self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
# # #
# # #         # --- ROBOT 1 SETUP ---
# # #         self.ur1_pub = self.create_publisher(JointTrajectory, '/ur_1_controller/joint_trajectory', 10)
# # #         self.ur1_joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
# # #                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
# # #         # self.ur1_home = [-1.359, -1.95, -0.760, -1.823, 1.558, 0.033]  # "Ready" position
# # #         self.ur1_home = [0.895, -1.558, 1.116, -1.160, -1.494, -0.033]  # "Ready" position
# # #         self.ur1_zero = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # "Flat/Start" position
# # #         self.ur1_current = []
# # #         self.ur1_group = 'ur_manipulator'
# # #         self.ur1_base = 'base_link'
# # #
# # #         # --- ROBOT 2 SETUP ---
# # #         self.ur2_pub = self.create_publisher(JointTrajectory, '/ur_2_controller/joint_trajectory', 10)
# # #         self.ur2_joints = ['ur_2_shoulder_pan_joint', 'ur_2_shoulder_lift_joint', 'ur_2_elbow_joint',
# # #                            'ur_2_wrist_1_joint', 'ur_2_wrist_2_joint', 'ur_2_wrist_3_joint']
# # #         # self.ur2_home = [1.359, 0.033, 1.277, 0.033, 1.359, 0.099]
# # #         self.ur2_home = [1.690, 0.099, 1.051, 0.431, 1.492, 0.00]
# # #         self.ur2_zero = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# # #         self.ur2_current = []
# # #         self.ur2_group = 'ur_2_manipulator'
# # #         self.ur2_base = 'ur_2_base_link'
# # #
# # #     def joint_state_callback(self, msg):
# # #         if self.ur1_joints[0] in msg.name:
# # #             try:
# # #                 self.ur1_current = [msg.position[msg.name.index(n)] for n in self.ur1_joints]
# # #             except:
# # #                 pass
# # #         if self.ur2_joints[0] in msg.name:
# # #             try:
# # #                 self.ur2_current = [msg.position[msg.name.index(n)] for n in self.ur2_joints]
# # #             except:
# # #                 pass
# # #
# # #     def get_ik(self, target_pose, group, base_link, current_joints, joint_names):
# # #         req = GetPositionIK.Request()
# # #         req.ik_request.group_name = group
# # #         req.ik_request.robot_state.joint_state.name = joint_names
# # #         req.ik_request.robot_state.joint_state.position = current_joints if current_joints else self.ur1_home
# # #         req.ik_request.pose_stamped.header.frame_id = base_link
# # #         req.ik_request.pose_stamped.pose = target_pose
# # #         req.ik_request.timeout.sec = 1
# # #         # IMPORTANT: Solving for tool0 (Wrist) because Mop is floppy
# # #         req.ik_request.ik_link_name = "tool0" if group == 'ur_manipulator' else "ur_2_wrist_3_joint"
# # #         req.ik_request.avoid_collisions = True
# # #
# # #         future = self.ik_client.call_async(req)
# # #         start = time.time()
# # #         while not future.done():
# # #             time.sleep(0.01)
# # #             if time.time() - start > 1.0: return None
# # #
# # #         try:
# # #             res = future.result()
# # #             if res.error_code.val == 1:
# # #                 return list(res.solution.joint_state.position)
# # #         except:
# # #             pass
# # #         return None
# # #
# # #     def execute_trajectory(self, publisher, joint_names, list_of_points, time_step=0.1):
# # #         msg = JointTrajectory()
# # #         msg.joint_names = joint_names
# # #         current_time = 0.0
# # #         for p in list_of_points:
# # #             pt = JointTrajectoryPoint()
# # #             if len(p) > 6:
# # #                 pt.positions = p[:6]
# # #             else:
# # #                 pt.positions = p
# # #             current_time += time_step
# # #             pt.time_from_start = Duration(seconds=current_time).to_msg()
# # #             msg.points.append(pt)
# # #         publisher.publish(msg)
# # #         return current_time
# # #
# # #     # MODIFICATION 1: Added execute_csv parameter
# # #     def run_robot_task(self, robot_name, publisher, joint_names, home_joints, end_joints, group, base_link, offset_y, offset_z, execute_csv=True):
# # #         self.get_logger().info(f"--- STARTING {robot_name} ---")
# # #
# # #         # 1. Move to Home/Ready Position
# # #         self.get_logger().info(f"[{robot_name}] Moving to Start...")
# # #         self.execute_trajectory(publisher, joint_names, [home_joints], time_step=5.0)
# # #         time.sleep(5.5)
# # #
# # #         # 2. Process CSV Path (ONLY IF execute_csv is True)
# # #         if execute_csv:
# # #             home_dir = os.path.expanduser('~')
# # #             csv_path = os.path.join(home_dir, 'ros2_ws', 'data', 'dmp_trajectory_feedback.csv')
# # #             trajectory_points = []
# # #
# # #             # Read CSV logic
# # #             try:
# # #                 with open(csv_path, 'r') as csvfile:
# # #                     reader = csv.DictReader(csvfile)
# # #                     for i, row in enumerate(reader):
# # #                         tx = float(row['x'])
# # #                         ty = float(row['y'])
# # #
# # #                         rx = tx
# # #                         ry = ty + offset_y
# # #                         rz = offset_z # Wrist Height (Mop down)
# # #
# # #                         pose = Pose()
# # #                         pose.position.x = rx
# # #                         pose.position.y = ry
# # #                         pose.position.z = rz
# # #                         pose.orientation = quaternion_from_euler(math.radians(180), 0, 0)
# # #
# # #                         current = self.ur1_current if robot_name == "UR1" else self.ur2_current
# # #                         if not current: current = home_joints
# # #
# # #                         sol = self.get_ik(pose, group, base_link, current, joint_names)
# # #                         if sol:
# # #                             trajectory_points.append(sol)
# # #                             if i % 50 == 0: print(f"[{robot_name}] Planning {i}...", end='\r')
# # #             except Exception as e:
# # #                 self.get_logger().error(f"CSV Error: {e}")
# # #
# # #             if trajectory_points:
# # #                 self.get_logger().info(f"\n[{robot_name}] Executing Path...")
# # #                 duration = self.execute_trajectory(publisher, joint_names, trajectory_points, time_step=0.1)
# # #                 time.sleep(duration + 2.0)
# # #             else:
# # #                 self.get_logger().error(f"[{robot_name}] No valid path found!")
# # #         else:
# # #             self.get_logger().info(f"[{robot_name}] Skipping CSV path execution as requested.")
# # #
# # #         # 3. RELAY MOVEMENT: Two-Step Parking Sequence (Always runs now)
# # #         self.get_logger().info(f"[{robot_name}] Task Done. Moving to Home Position first...")
# # #         self.execute_trajectory(publisher, joint_names, [home_joints], time_step=4.0)
# # #         time.sleep(4.5)  # Wait for the arm to reach Home
# # #
# # #         self.get_logger().info(f"[{robot_name}] Moving to Final Rest Position: {end_joints}")
# # #         self.execute_trajectory(publisher, joint_names, [end_joints], time_step=4.0)
# # #         time.sleep(5.0)  # Wait for the arm to finish folding down
# # #
# # #         self.get_logger().info(f"[{robot_name}] IS CLEAR. RELAY READY.")
# # #
# # #
# # # def main(args=None):
# # #     rclpy.init(args=args)
# # #     node = DualRobotRelay()
# # #     executor = rclpy.executors.SingleThreadedExecutor()
# # #     executor.add_node(node)
# # #     t = threading.Thread(target=executor.spin, daemon=True)
# # #     t.start()
# # #
# # #     print("Waiting for joint states...")
# # #     time.sleep(2.0)
# # #
# # #     # ==========================
# # #     #      THE RELAY LOGIC
# # #     # ==========================
# # #
# # #     # STEP 1: UR1
# # #     # Start: Home. End: ZERO (Flat/Original).
# # #     node.run_robot_task(
# # #         "UR1",
# # #         node.ur1_pub,
# # #         node.ur1_joints,
# # #         home_joints=node.ur1_home,
# # #         end_joints=node.ur1_zero,  # <--- UR1 GOES TO ALL ZEROS HERE
# # #         group=node.ur1_group,
# # #         base_link=node.ur1_base,
# # #         offset_y=0.1,
# # #         offset_z=0.6,
# # #         execute_csv=True  # MODIFICATION 2: Tell UR1 to execute the CSV
# # #     )
# # #
# # #     print("\n-------------------------------------------")
# # #     print(" >>> RELAY TRIGGERED: HANDING OFF TO UR2 <<<")
# # #     print("-------------------------------------------\n")
# # #
# # #     # STEP 2: UR2
# # #     # Start: Home. End: Home (Or Zero if you prefer).
# # #     node.run_robot_task(
# # #         "UR2",
# # #         node.ur2_pub,
# # #         node.ur2_joints,
# # #         home_joints=node.ur2_home,
# # #         end_joints=node.ur2_zero,  # UR2 stays at Home after done
# # #         group=node.ur2_group,
# # #         base_link=node.ur2_base,
# # #         offset_y=-0.5,
# # #         offset_z=0.4,
# # #         execute_csv=False  # MODIFICATION 3: Tell UR2 to skip the CSV
# # #     )
# # #
# # #     node.destroy_node()
# # #     rclpy.shutdown()
# # #
# # #
# # # if __name__ == '__main__':
# # #     main()
# #
# #
# # import rclpy
# # from rclpy.node import Node
# # from rclpy.duration import Duration
# # from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# # from moveit_msgs.srv import GetPositionIK, GetPositionFK
# # from moveit_msgs.msg import PositionIKRequest, RobotState
# # from geometry_msgs.msg import PoseStamped, Pose, Quaternion
# # from sensor_msgs.msg import JointState
# # import numpy as np
# # import threading
# # import time
# # import csv
# # import os
# # import math
# #
# #
# # def quaternion_from_euler(roll, pitch, yaw):
# #     qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
# #     qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
# #     qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
# #     qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
# #     return Quaternion(x=qx, y=qy, z=qz, w=qw)
# #
# #
# # class DualRobotRelay(Node):
# #     def __init__(self):
# #         super().__init__('dual_robot_relay')
# #
# #         self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
# #         self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
# #         self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
# #
# #         # --- ROBOT 1 SETUP ---
# #         self.ur1_pub = self.create_publisher(JointTrajectory, '/ur_1_controller/joint_trajectory', 10)
# #         self.ur1_joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
# #                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
# #         self.ur1_home = [1.558, -1.194, 1.051, -1.425, -1.690, 0.033]
# #         self.ur1_zero = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # "Flat/Start" position
# #         self.ur1_current = []
# #         self.ur1_group = 'ur_manipulator'
# #         self.ur1_base = 'base_link'
# #
# #         # --- ROBOT 2 SETUP ---
# #         self.ur2_pub = self.create_publisher(JointTrajectory, '/ur_2_controller/joint_trajectory', 10)
# #         self.ur2_joints = ['ur_2_shoulder_pan_joint', 'ur_2_shoulder_lift_joint', 'ur_2_elbow_joint',
# #                            'ur_2_wrist_1_joint', 'ur_2_wrist_2_joint', 'ur_2_wrist_3_joint']
# #         self.ur2_home = [1.690, 0.099, 1.051, 0.431, 1.492, 0.00]
# #         self.ur2_zero = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# #         self.ur2_current = []
# #         self.ur2_group = 'ur_2_manipulator'
# #         self.ur2_base = 'ur_2_base_link'
# #
# #     def joint_state_callback(self, msg):
# #         if self.ur1_joints[0] in msg.name:
# #             try:
# #                 self.ur1_current = [msg.position[msg.name.index(n)] for n in self.ur1_joints]
# #             except:
# #                 pass
# #         if self.ur2_joints[0] in msg.name:
# #             try:
# #                 self.ur2_current = [msg.position[msg.name.index(n)] for n in self.ur2_joints]
# #             except:
# #                 pass
# #
# #     def get_ik(self, target_pose, group, base_link, current_joints, joint_names):
# #         req = GetPositionIK.Request()
# #         req.ik_request.group_name = group
# #         req.ik_request.robot_state.joint_state.name = joint_names
# #         req.ik_request.robot_state.joint_state.position = current_joints if current_joints else self.ur1_home
# #         req.ik_request.pose_stamped.header.frame_id = base_link
# #         req.ik_request.pose_stamped.pose = target_pose
# #         req.ik_request.timeout.sec = 1
# #         req.ik_request.ik_link_name = "tool0" if group == 'ur_manipulator' else "ur_2_wrist_3_joint"
# #         req.ik_request.avoid_collisions = True
# #
# #         future = self.ik_client.call_async(req)
# #         start = time.time()
# #         while not future.done():
# #             time.sleep(0.01)
# #             if time.time() - start > 1.0: return None, -999  # Timeout Error
# #
# #         try:
# #             res = future.result()
# #             if res.error_code.val == 1:
# #                 # SUCCESS: Return joints and error code 1
# #                 return list(res.solution.joint_state.position), 1
# #             else:
# #                 # FAILED: Return None and the specific MoveIt error code
# #                 return None, res.error_code.val
# #         except:
# #             pass
# #         return None, -999
# #
# #     def execute_trajectory(self, publisher, joint_names, list_of_points, time_step=0.1):
# #         msg = JointTrajectory()
# #         msg.joint_names = joint_names
# #         current_time = 0.0
# #         for p in list_of_points:
# #             pt = JointTrajectoryPoint()
# #             if len(p) > 6:
# #                 pt.positions = p[:6]
# #             else:
# #                 pt.positions = p
# #             current_time += time_step
# #             pt.time_from_start = Duration(seconds=current_time).to_msg()
# #             msg.points.append(pt)
# #         publisher.publish(msg)
# #         return current_time
# #
# #     def run_robot_task(self, robot_name, publisher, joint_names, home_joints, end_joints, group, base_link, offset_y,
# #                        offset_z, execute_csv=True):
# #         self.get_logger().info(f"--- STARTING {robot_name} ---")
# #         collision_detected = False
# #
# #         # 1. Move to Home/Ready Position
# #         self.get_logger().info(f"[{robot_name}] Moving to Start...")
# #         self.execute_trajectory(publisher, joint_names, [home_joints], time_step=5.0)
# #         time.sleep(5.5)
# #
# #         # 2. Process CSV Path
# #         if execute_csv:
# #             home_dir = os.path.expanduser('~')
# #             csv_path = os.path.join(home_dir, 'ros2_ws', 'data', 'motion_safe.csv')
# #             trajectory_points = []
# #
# #             try:
# #                 with open(csv_path, 'r') as csvfile:
# #                     reader = csv.DictReader(csvfile)
# #                     for i, row in enumerate(reader):
# #                         tx = float(row['x'])
# #                         ty = float(row['y'])
# #
# #                         rx = tx
# #                         ry = ty
# #                         rz = offset_z
# #
# #                         pose = Pose()
# #                         pose.position.x = rx
# #                         pose.position.y = ry
# #                         pose.position.z = rz
# #                         pose.orientation = quaternion_from_euler(math.radians(180), 0, 0)
# #
# #                         current = self.ur1_current if robot_name == "UR1" else self.ur2_current
# #                         if not current: current = home_joints
# #
# #                         sol, err_code = self.get_ik(pose, group, base_link, current, joint_names)
# #
# #                         if sol:
# #                             trajectory_points.append(sol)
# #                             # Print updating computation status on the same line
# #                             print(f"[{robot_name}] [COMPUTING] ✓ Valid point {i} | X:{rx:.3f} Y:{ry:.3f}", end='\r')
# #                         else:
# #                             # IF COLLISION OR ERROR OCCURS:
# #                             print(f"\n\n[{robot_name}] 🛑 COLLISION OR REACH ERROR DETECTED!")
# #                             print(f"    Failed at CSV Row: {i}")
# #                             print(f"    Failed Location -> X: {rx:.3f}, Y: {ry:.3f}, Z: {rz:.3f}")
# #                             print(f"    MoveIt Error Code: {err_code} (Often -31 for NO_SOLUTION or -12 for COLLISION)")
# #                             print(
# #                                 f"    ACTION: Stopping computation here. Robot will move up to point {i - 1} and stop.\n")
# #                             collision_detected = True
# #                             break  # Stop reading CSV and move to execution
# #
# #             except Exception as e:
# #                 self.get_logger().error(f"CSV Error: {e}")
# #
# #             # Execute the valid points we managed to compute
# #             if trajectory_points:
# #                 print(f"\n[{robot_name}] Computing finished. Executing {len(trajectory_points)} valid points...")
# #                 duration = self.execute_trajectory(publisher, joint_names, trajectory_points, time_step=0.1)
# #                 time.sleep(duration + 2.0)
# #             else:
# #                 self.get_logger().error(f"[{robot_name}] No valid path found at all!")
# #
# #         else:
# #             self.get_logger().info(f"[{robot_name}] Skipping CSV path execution as requested.")
# #
# #         # 3. RELAY MOVEMENT / PARKING
# #         if collision_detected:
# #             # If we crashed, DO NOT park. Leave the robot at the crash site for inspection.
# #             self.get_logger().warn(
# #                 f"[{robot_name}] SKIPPING PARKING SEQUENCE. Robot remaining at failure point for inspection.")
# #             return
# #
# #         self.get_logger().info(f"[{robot_name}] Task Done. Moving to Home Position first...")
# #         self.execute_trajectory(publisher, joint_names, [home_joints], time_step=4.0)
# #         time.sleep(4.5)
# #
# #         self.get_logger().info(f"[{robot_name}] Moving to Final Rest Position: {end_joints}")
# #         self.execute_trajectory(publisher, joint_names, [end_joints], time_step=4.0)
# #         time.sleep(5.0)
# #
# #         self.get_logger().info(f"[{robot_name}] IS CLEAR. RELAY READY.")
# #
# #
# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = DualRobotRelay()
# #     executor = rclpy.executors.SingleThreadedExecutor()
# #     executor.add_node(node)
# #     t = threading.Thread(target=executor.spin, daemon=True)
# #     t.start()
# #
# #     print("Waiting for joint states...")
# #     time.sleep(2.0)
# #
# #     # STEP 1: UR1
# #     node.run_robot_task(
# #         "UR1",
# #         node.ur1_pub,
# #         node.ur1_joints,
# #         home_joints=node.ur1_home,
# #         end_joints=node.ur1_zero,
# #         group=node.ur1_group,
# #         base_link=node.ur1_base,
# #         offset_y=0.0,  # <--- Change to 0.0
# #         offset_z=0.55,  # <--- THIS IS THE PERFECT HEIGHT (0.06 Table + 0.47 Mop)
# #         execute_csv=True
# #     )
# #
# #     print("\n-------------------------------------------")
# #     print(" >>> RELAY TRIGGERED: HANDING OFF TO UR2 <<<")
# #     print("-------------------------------------------\n")
# #
# #     # STEP 2: UR2
# #     node.run_robot_task(
# #         "UR2",
# #         node.ur2_pub,
# #         node.ur2_joints,
# #         home_joints=node.ur2_home,
# #         end_joints=node.ur2_zero,
# #         group=node.ur2_group,
# #         base_link=node.ur2_base,
# #         offset_y=-0.5,
# #         offset_z=0.4,
# #         execute_csv=False
# #     )
# #
# #     node.destroy_node()
# #     rclpy.shutdown()
# #
# #
# # if __name__ == '__main__':
# #     main()
#
# import rclpy
# from rclpy.node import Node
# from rclpy.duration import Duration
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from moveit_msgs.srv import GetPositionIK, GetPositionFK
# from geometry_msgs.msg import PoseStamped, Pose, Quaternion
# from sensor_msgs.msg import JointState
# import threading
# import time
# import csv
# import os
# import math
#
#
# class DualRobotRelay(Node):
#     def __init__(self):
#         super().__init__('dual_robot_relay')
#
#         self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
#         self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
#         self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
#
#         # --- ROBOT 1 SETUP ---
#         self.ur1_pub = self.create_publisher(JointTrajectory, '/ur_1_controller/joint_trajectory', 10)
#         self.ur1_joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
#                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
#         # REVERTED TO YOUR PREVIOUS SAFE HOME POSE (-1.094 instead of -1.194)
#         self.ur1_home = [1.558, -1.194, 1.051, -1.425, -1.690, 0.033]
#         self.ur1_zero = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#         self.ur1_current = []
#         self.ur1_group = 'ur_manipulator'
#         self.ur1_base = 'base_link'
#
#         # --- ROBOT 2 SETUP ---
#         self.ur2_pub = self.create_publisher(JointTrajectory, '/ur_2_controller/joint_trajectory', 10)
#         self.ur2_joints = ['ur_2_shoulder_pan_joint', 'ur_2_shoulder_lift_joint', 'ur_2_elbow_joint',
#                            'ur_2_wrist_1_joint', 'ur_2_wrist_2_joint', 'ur_2_wrist_3_joint']
#         self.ur2_home = [1.359, 0.033, 1.277, 0.033, 1.359, 0.099]
#         self.ur2_zero = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#         self.ur2_current = []
#         self.ur2_group = 'ur_2_manipulator'
#         self.ur2_base = 'ur_2_base_link'
#
#     def joint_state_callback(self, msg):
#         if self.ur1_joints[0] in msg.name:
#             try:
#                 self.ur1_current = [msg.position[msg.name.index(n)] for n in self.ur1_joints]
#             except:
#                 pass
#         if self.ur2_joints[0] in msg.name:
#             try:
#                 self.ur2_current = [msg.position[msg.name.index(n)] for n in self.ur2_joints]
#             except:
#                 pass
#
#     def get_fk_pose(self, group, base_link, current_joints, joint_names):
#         req = GetPositionFK.Request()
#         req.header.frame_id = base_link
#         req.fk_link_names = ["tool0" if group == 'ur_manipulator' else "ur_2_wrist_3_joint"]
#         req.robot_state.joint_state.name = joint_names
#         req.robot_state.joint_state.position = current_joints
#
#         future = self.fk_client.call_async(req)
#         start = time.time()
#         while not future.done():
#             time.sleep(0.01)
#             if time.time() - start > 10.0: return None
#
#         try:
#             res = future.result()
#             if res.error_code.val == 1:
#                 return res.pose_stamped[0].pose
#         except:
#             pass
#         return None
#
#     def get_ik(self, target_pose, group, base_link, current_joints, joint_names, check_collision=True):
#         req = GetPositionIK.Request()
#         req.ik_request.group_name = group
#         req.ik_request.robot_state.joint_state.name = joint_names
#         req.ik_request.robot_state.joint_state.position = current_joints
#         req.ik_request.pose_stamped.header.frame_id = base_link
#         req.ik_request.pose_stamped.pose = target_pose
#
#         req.ik_request.timeout.sec = 5
#         req.ik_request.ik_link_name = "tool0" if group == 'ur_manipulator' else "ur_2_wrist_3_joint"
#         req.ik_request.avoid_collisions = check_collision
#
#         future = self.ik_client.call_async(req)
#         start = time.time()
#         while not future.done():
#             time.sleep(0.01)
#             if time.time() - start > 5.0: return None, -999
#
#         try:
#             res = future.result()
#             if res.error_code.val == 1:
#                 return list(res.solution.joint_state.position), 1
#             else:
#                 return None, res.error_code.val
#         except:
#             pass
#         return None, -999
#
#     def execute_trajectory(self, publisher, joint_names, list_of_points, time_step=0.1):
#         msg = JointTrajectory()
#         msg.joint_names = joint_names
#         current_time = 0.0
#         for p in list_of_points:
#             pt = JointTrajectoryPoint()
#             if len(p) > 6:
#                 pt.positions = p[:6]
#             else:
#                 pt.positions = p
#             current_time += time_step
#             pt.time_from_start = Duration(seconds=current_time).to_msg()
#             msg.points.append(pt)
#         publisher.publish(msg)
#         return current_time
#
#     def run_robot_task(self, robot_name, publisher, joint_names, home_joints, end_joints, group, base_link,
#                        execute_csv=True):
#         self.get_logger().info(f"--- STARTING {robot_name} ---")
#         collision_detected = False
#
#         self.get_logger().info(f"[{robot_name}] Moving to Start...")
#         self.execute_trajectory(publisher, joint_names, [home_joints], time_step=5.0)
#         time.sleep(5.5)
#
#         if execute_csv:
#             home_dir = os.path.expanduser('~')
#             csv_path = os.path.join(home_dir, 'ros2_ws', 'data', 'motion_safe.csv')
#             trajectory_points = []
#
#             self.get_logger().info(f"[{robot_name}] Locking to exact Home Anchor Pose...")
#             perfect_pose = self.get_fk_pose(group, base_link, home_joints, joint_names)
#
#             if not perfect_pose:
#                 self.get_logger().error("Failed to get FK pose! Exiting.")
#                 return
#
#             home_x = perfect_pose.position.x
#             home_y = perfect_pose.position.y
#             home_quat = perfect_pose.orientation
#
#             # AUTOMATIC HOVER OFFSET: Lift the drawing 5cm above the home pose to guarantee table clearance
#             home_z = perfect_pose.position.z + 0.05
#
#             self.get_logger().info(f"[{robot_name}] Sweeping Height Locked at -> Z:{home_z:.3f}")
#
#             # THE IK CHAINING FIX: Start the seed at home_joints
#             current_seed = self.ur1_current if robot_name == "UR1" else self.ur2_current
#             if not current_seed or len(current_seed) < 6:
#                 current_seed = home_joints
#
#             try:
#                 with open(csv_path, 'r') as csvfile:
#                     reader = csv.DictReader(csvfile)
#                     first_row = True
#                     shift_x = 0.0
#                     shift_y = 0.0
#
#                     for i, row in enumerate(reader):
#                         raw_x = float(row['x'])
#                         raw_y = float(row['y'])
#
#                         if first_row:
#                             shift_x = home_x - raw_x
#                             shift_y = home_y - raw_y
#                             first_row = False
#
#                         pose = Pose()
#                         pose.position.x = raw_x + shift_x
#                         pose.position.y = raw_y + shift_y
#                         pose.position.z = home_z
#                         pose.orientation = home_quat
#
#                         # Use current_seed to chain the math!
#                         sol, err_code = self.get_ik(pose, group, base_link, current_seed, joint_names,
#                                                     check_collision=True)
#
#                         if sol:
#                             trajectory_points.append(sol)
#                             current_seed = sol  # <--- CRITICAL FIX: Update the seed for the next point!
#                             print(
#                                 f"[{robot_name}] ✓ Point {i} OK | X:{pose.position.x:.3f} Y:{pose.position.y:.3f} Z:{pose.position.z:.3f}",
#                                 end='\r')
#                         else:
#                             print(f"\n\n[{robot_name}]  FAILED at Point {i}")
#                             print(f"MoveIt Error Code: {err_code}")
#                             collision_detected = True
#                             break
#
#             except Exception as e:
#                 self.get_logger().error(f"CSV Error: {e}")
#
#             if trajectory_points:
#                 print(f"\n[{robot_name}] Executing {len(trajectory_points)} points...")
#                 duration = self.execute_trajectory(publisher, joint_names, trajectory_points, time_step=0.1)
#                 time.sleep(duration + 2.0)
#
#         if collision_detected:
#             self.get_logger().warn(f"[{robot_name}] SKIPPING PARKING SEQUENCE. Inspect the error above.")
#             return
#
#         self.get_logger().info(f"[{robot_name}] Moving Home...")
#         self.execute_trajectory(publisher, joint_names, [home_joints], time_step=4.0)
#         time.sleep(4.5)
#         self.execute_trajectory(publisher, joint_names, [end_joints], time_step=4.0)
#         time.sleep(5.0)
#
#
# def main(args=None):
#     rclpy.init(args=args)
#     node = DualRobotRelay()
#     executor = rclpy.executors.SingleThreadedExecutor()
#     executor.add_node(node)
#     t = threading.Thread(target=executor.spin, daemon=True)
#     t.start()
#     time.sleep(2.0)
#
#     node.run_robot_task("UR1", node.ur1_pub, node.ur1_joints, node.ur1_home, node.ur1_zero, node.ur1_group,
#                         node.ur1_base, execute_csv=True)
#     print(
#         "\n-------------------------------------------\n >>> RELAY TRIGGERED: HANDING OFF TO UR2 <<<\n-------------------------------------------\n")
#     node.run_robot_task("UR2", node.ur2_pub, node.ur2_joints, node.ur2_home, node.ur2_zero, node.ur2_group,
#                         node.ur2_base, execute_csv=False)
#
#     node.destroy_node()
#     rclpy.shutdown()
#
#
# if __name__ == '__main__':
#     main()
#
#
# import rclpy
# from rclpy.node import Node
# from rclpy.duration import Duration
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from moveit_msgs.srv import GetPositionIK, GetPositionFK
# from geometry_msgs.msg import PoseStamped, Pose, Quaternion
# from sensor_msgs.msg import JointState
# import threading
# import time
# import os
# import math
#
#
# class DualRobotRelay(Node):
#     def __init__(self):
#         super().__init__('dual_robot_relay')
#
#         self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
#         self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
#         self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
#
#         # --- ROBOT 1 SETUP ---
#         # self.ur1_pub = self.create_publisher(JointTrajectory, '/ur_1_controller/joint_trajectory', 10)
#         self.ur1_pub = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)
#
#
#         self.ur1_joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
#                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
#         self.ur1_home = [1.558, -1.14, 1.051, -1.425, -1.690, 0.033]
#         self.ur1_zero = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#         self.ur1_current = []
#         self.ur1_group = 'ur_manipulator'
#         self.ur1_base = 'base_link'
#
#         # --- ROBOT 2 SETUP ---
#         self.ur2_pub = self.create_publisher(JointTrajectory, '/ur_2_controller/joint_trajectory', 10)
#         self.ur2_joints = ['ur_2_shoulder_pan_joint', 'ur_2_shoulder_lift_joint', 'ur_2_elbow_joint',
#                            'ur_2_wrist_1_joint', 'ur_2_wrist_2_joint', 'ur_2_wrist_3_joint']
#         self.ur2_home = [1.359, 0.033, 1.277, 0.033, 1.359, 0.099]
#         self.ur2_zero = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#         self.ur2_current = []
#         self.ur2_group = 'ur_2_manipulator'
#         self.ur2_base = 'ur_2_base_link'
#
#     def joint_state_callback(self, msg):
#         if self.ur1_joints[0] in msg.name:
#             try:
#                 self.ur1_current = [msg.position[msg.name.index(n)] for n in self.ur1_joints]
#             except:
#                 pass
#         if self.ur2_joints[0] in msg.name:
#             try:
#                 self.ur2_current = [msg.position[msg.name.index(n)] for n in self.ur2_joints]
#             except:
#                 pass
#
#     def get_fk_pose(self, group, base_link, current_joints, joint_names):
#         req = GetPositionFK.Request()
#         req.header.frame_id = base_link
#         req.fk_link_names = ["tool0" if group == 'ur_manipulator' else "ur_2_wrist_3_joint"]
#         req.robot_state.joint_state.name = joint_names
#         req.robot_state.joint_state.position = current_joints
#
#         future = self.fk_client.call_async(req)
#         start = time.time()
#         while not future.done():
#             time.sleep(0.01)
#             if time.time() - start > 10.0: return None
#
#         try:
#             res = future.result()
#             if res.error_code.val == 1:
#                 return res.pose_stamped[0].pose
#         except:
#             pass
#         return None
#
#     def get_ik(self, target_pose, group, base_link, current_joints, joint_names, check_collision=True):
#         req = GetPositionIK.Request()
#         req.ik_request.group_name = group
#         req.ik_request.robot_state.joint_state.name = joint_names
#         req.ik_request.robot_state.joint_state.position = current_joints
#         req.ik_request.pose_stamped.header.frame_id = base_link
#         req.ik_request.pose_stamped.pose = target_pose
#
#         req.ik_request.timeout.sec = 5
#         req.ik_request.ik_link_name = "tool0" if group == 'ur_manipulator' else "ur_2_wrist_3_joint"
#         req.ik_request.avoid_collisions = check_collision
#
#         future = self.ik_client.call_async(req)
#         start = time.time()
#         while not future.done():
#             time.sleep(0.01)
#             if time.time() - start > 5.0: return None, -999
#
#         try:
#             res = future.result()
#             if res.error_code.val == 1:
#                 return list(res.solution.joint_state.position), 1
#             else:
#                 return None, res.error_code.val
#         except:
#             pass
#         return None, -99
#
#     def execute_trajectory(self, publisher, joint_names, list_of_points, time_step=0.1):
#         msg = JointTrajectory()
#         msg.joint_names = joint_names
#         current_time = 0.0
#         for p in list_of_points:
#             pt = JointTrajectoryPoint()
#             if len(p) > 6:
#                 pt.positions = p[:6]
#             else:
#                 pt.positions = p
#             current_time += time_step
#             pt.time_from_start = Duration(seconds=current_time).to_msg()
#             msg.points.append(pt)
#         publisher.publish(msg)
#         return current_time
#
#     def run_robot_task(self, robot_name, publisher, joint_names, home_joints, end_joints, group, base_link,
#                        interactive_mode=True):
#         self.get_logger().info(f"--- STARTING {robot_name} ---")
#
#         self.get_logger().info(f"[{robot_name}] Moving to Start...")
#         self.execute_trajectory(publisher, joint_names, [home_joints], time_step=5.0)
#         time.sleep(5.5)
#
#         if interactive_mode:
#             self.get_logger().info(f"[{robot_name}] Locking to exact Home Anchor Pose...")
#             perfect_pose = self.get_fk_pose(group, base_link, home_joints, joint_names)
#
#             if not perfect_pose:
#                 self.get_logger().error("Failed to get FK pose! Exiting.")
#                 return
#
#             home_x = perfect_pose.position.x
#             home_y = perfect_pose.position.y
#             home_quat = perfect_pose.orientation
#
#             # AUTOMATIC HOVER OFFSET: Lift the drawing 5cm above the home pose to guarantee table clearance
#             home_z = perfect_pose.position.z - 0.2
#
#             self.get_logger().info(f"[{robot_name}] Sweeping Height Locked at -> Z:{home_z:.3f}")
#
#             # Ensure we have a valid starting seed
#             current_seed = self.ur1_current if robot_name == "UR1" else self.ur2_current
#             if not current_seed or len(current_seed) < 6:
#                 current_seed = home_joints
#
#             print(f"\n=======================================================")
#             print(f"   INTERACTIVE MODE ACTIVE for {robot_name}   ")
#             print(f"=======================================================")
#             print(f"  Reference Start Position -> X: {home_x:.3f} | Y: {home_y:.3f}")
#             print(f"  Type your desired X and Y coordinates separated by a space.")
#             print(f"  Example input:  0.4 0.1")
#             print(f"  Type 'quit' or 'q' when you are done to begin the handover.")
#
#             while True:
#                 try:
#                     user_input = input(f"[{robot_name}] Enter Target X Y (or 'quit'): ").strip()
#
#                     if user_input.lower() in ['quit', 'q', 'exit']:
#                         print(f"\n[{robot_name}] Exiting Interactive Mode. Preparing to park...")
#                         break
#
#                     parts = user_input.split()
#                     if len(parts) != 2:
#                         print(" Invalid input! Please enter exactly two numbers (X and Y), or type 'quit'.")
#                         continue
#
#                     target_x = float(parts[0])
#                     target_y = float(parts[1])
#
#                     # Build the pose request
#                     pose = Pose()
#                     pose.position.x = target_x
#                     pose.position.y = target_y
#                     pose.position.z = home_z
#                     pose.orientation = home_quat
#
#                     # Calculate IK and verify collisions
#                     sol, err_code = self.get_ik(pose, group, base_link, current_seed, joint_names, check_collision=True)
#
#                     if sol:
#                         print(f"[{robot_name}] ✓ Valid point! Moving to X:{target_x} Y:{target_y}...")
#                         # Move immediately point-to-point (3 seconds smooth motion)
#                         duration = self.execute_trajectory(publisher, joint_names, [sol], time_step=3.0)
#                         time.sleep(duration + 0.5)
#                         current_seed = sol  # Update seed so the next calculation originates from the new spot
#                     else:
#                         print(
#                             f"[{robot_name}]  Invalid point! MoveIt Error Code: {err_code}. (Collision or Unreachable)")
#
#                 except ValueError:
#                     print(" Invalid input! Please enter numerical values.")
#                 except KeyboardInterrupt:
#                     print(f"\n[{robot_name}] Interrupted by user. Exiting Interactive Mode.")
#                     break
#
#         # TWO-STEP PARKING SEQUENCE
#         self.get_logger().info(f"[{robot_name}] Moving Home...")
#         self.execute_trajectory(publisher, joint_names, [home_joints], time_step=4.0)
#         time.sleep(4.5)
#         self.execute_trajectory(publisher, joint_names, [end_joints], time_step=4.0)
#         time.sleep(5.0)
#
#
# def main(args=None):
#     rclpy.init(args=args)
#     node = DualRobotRelay()
#     executor = rclpy.executors.SingleThreadedExecutor()
#     executor.add_node(node)
#     t = threading.Thread(target=executor.spin, daemon=True)
#     t.start()
#     time.sleep(2.0)
#
#     # STEP 1: UR1 (Interactive Mode ON)
#     node.run_robot_task("UR1", node.ur1_pub, node.ur1_joints, node.ur1_home, node.ur1_zero, node.ur1_group,
#                         node.ur1_base, interactive_mode=True)
#     print(
#         "\n-------------------------------------------\n >>> RELAY TRIGGERED: HANDING OFF TO UR2 <<<\n-------------------------------------------\n")
#
#     # STEP 2: UR2 (Interactive Mode OFF - just goes Home -> Zero)
#     node.run_robot_task("UR2", node.ur2_pub, node.ur2_joints, node.ur2_home, node.ur2_zero, node.ur2_group,
#                         node.ur2_base, interactive_mode=False)
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
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from sensor_msgs.msg import JointState
import threading
import time
import os
import math


class SingleRobotControl(Node):
    def __init__(self):
        super().__init__('single_robot_control')

        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # --- ROBOT 1 SETUP ONLY ---
        # Using the scaled controller as per your working hardware configuration
        # self.ur1_pub = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)
        self.ur1_pub = self.create_publisher(JointTrajectory, '/ur_1_controller/joint_trajectory', 10)

        self.ur1_joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                           'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        # Preserving your specific Home and Zero positions
        self.ur1_home = [1.558, -1.14, 1.051, -1.425, -1.690, 0.033]
        self.ur1_zero = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.ur1_current = []
        self.ur1_group = 'ur_manipulator'
        self.ur1_base = 'base_link'

    def joint_state_callback(self, msg):
        # Filtering specifically for UR1 joint names
        if self.ur1_joints[0] in msg.name:
            try:
                self.ur1_current = [msg.position[msg.name.index(n)] for n in self.ur1_joints]
            except:
                pass

    def get_fk_pose(self, group, base_link, current_joints, joint_names):
        req = GetPositionFK.Request()
        req.header.frame_id = base_link
        req.fk_link_names = ["mop_tip"]
        req.robot_state.joint_state.name = joint_names
        req.robot_state.joint_state.position = current_joints

        future = self.fk_client.call_async(req)
        start = time.time()
        while not future.done():
            time.sleep(0.01)
            if time.time() - start > 10.0: return None

        try:
            res = future.result()
            if res.error_code.val == 1:
                return res.pose_stamped[0].pose
        except:
            pass
        return None

    def get_ik(self, target_pose, group, base_link, current_joints, joint_names, check_collision=True):
        req = GetPositionIK.Request()
        req.ik_request.group_name = group
        req.ik_request.robot_state.joint_state.name = joint_names
        req.ik_request.robot_state.joint_state.position = current_joints
        req.ik_request.pose_stamped.header.frame_id = base_link
        req.ik_request.pose_stamped.pose = target_pose

        req.ik_request.timeout.sec = 5
        req.ik_request.ik_link_name = "mop_tip"
        req.ik_request.avoid_collisions = check_collision

        future = self.ik_client.call_async(req)
        start = time.time()
        while not future.done():
            time.sleep(0.01)
            if time.time() - start > 5.0: return None, -999

        try:
            res = future.result()
            if res.error_code.val == 1:
                return list(res.solution.joint_state.position), 1
            else:
                return None, res.error_code.val
        except:
            pass
        return None, -99

    def execute_trajectory(self, publisher, joint_names, list_of_points, time_step=0.1):
        msg = JointTrajectory()
        msg.joint_names = joint_names
        current_time = 0.0
        for p in list_of_points:
            pt = JointTrajectoryPoint()
            # Handle cases where point might have velocity/acceleration data attached
            if len(p) > 6:
                pt.positions = p[:6]
            else:
                pt.positions = p
            current_time += time_step
            pt.time_from_start = Duration(seconds=current_time).to_msg()
            msg.points.append(pt)
        publisher.publish(msg)
        return current_time

    def run_robot_task(self, robot_name, publisher, joint_names, home_joints, end_joints, group, base_link,
                       interactive_mode=True):
        self.get_logger().info(f"--- STARTING {robot_name} ---")

        # Initial move to Start Position
        self.get_logger().info(f"[{robot_name}] Moving to Start...")
        self.execute_trajectory(publisher, joint_names, [home_joints], time_step=5.0)
        time.sleep(5.5)

        if interactive_mode:
            self.get_logger().info(f"[{robot_name}] Locking to exact Home Anchor Pose...")
            perfect_pose = self.get_fk_pose(group, base_link, home_joints, joint_names)

            if not perfect_pose:
                self.get_logger().error("Failed to get FK pose! Exiting.")
                return

            home_x = perfect_pose.position.x
            home_y = perfect_pose.position.y
            home_quat = perfect_pose.orientation

            # AUTOMATIC HOVER OFFSET: Maintain height relative to home pose
            home_z = perfect_pose.position.z -0.2
            self.get_logger().info(f"[{robot_name}] Sweeping Height Locked at -> Z:{home_z:.3f}")

            current_seed = self.ur1_current
            if not current_seed or len(current_seed) < 6:
                current_seed = home_joints

            print(f"\n=======================================================")
            print(f"   INTERACTIVE MODE ACTIVE for {robot_name}   ")
            print(f"=======================================================")
            print(f"  Reference Start Position -> X: {home_x:.3f} | Y: {home_y:.3f}")
            print(f"  Type desired X and Y coordinates (Example: 0.4 0.1)")
            print(f"  Type 'quit' or 'q' to end task and park.")

            while True:
                try:
                    user_input = input(f"[{robot_name}] Enter Target X Y (or 'quit'): ").strip()

                    if user_input.lower() in ['quit', 'q', 'exit']:
                        print(f"\n[{robot_name}] Exiting Interactive Mode. Preparing to park...")
                        break

                    parts = user_input.split()
                    if len(parts) != 2:
                        print(" Invalid input! Please enter X and Y, or 'quit'.")
                        continue

                    target_x = float(parts[0])
                    target_y = float(parts[1])

                    pose = Pose()
                    pose.position.x = target_x
                    pose.position.y = target_y
                    pose.position.z = home_z
                    pose.orientation = home_quat

                    # IK Calculation with Collision Checking
                    sol, err_code = self.get_ik(pose, group, base_link, current_seed, joint_names, check_collision=True)

                    if sol:
                        print(f"[{robot_name}] ✓ Valid point! Moving to X:{target_x} Y:{target_y}...")
                        duration = self.execute_trajectory(publisher, joint_names, [sol], time_step=3.0)
                        time.sleep(duration + 0.5)
                        current_seed = sol
                    else:
                        print(f"[{robot_name}] ✗ Invalid point! Error Code: {err_code}.")

                except ValueError:
                    print(" Invalid input! Please enter numerical values.")
                except KeyboardInterrupt:
                    break

        # PARKING SEQUENCE
        self.get_logger().info(f"[{robot_name}] Moving Home...")
        self.execute_trajectory(publisher, joint_names, [home_joints], time_step=4.0)
        time.sleep(4.5)
        self.execute_trajectory(publisher, joint_names, [end_joints], time_step=4.0)
        time.sleep(5.0)


def main(args=None):
    rclpy.init(args=args)
    node = SingleRobotControl()

    # Threaded executor to handle service calls while waiting for input
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    t = threading.Thread(target=executor.spin, daemon=True)
    t.start()

    time.sleep(2.0)

    # Execute main task for UR1
    node.run_robot_task("UR1", node.ur1_pub, node.ur1_joints, node.ur1_home, node.ur1_zero, node.ur1_group,
                        node.ur1_base, interactive_mode=True)

    print("\n--- Task Complete. Shutting down. ---")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()