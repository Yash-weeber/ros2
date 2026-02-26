# import rclpy
# from rclpy.node import Node
# from rclpy.duration import Duration
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from moveit_msgs.srv import GetPositionIK
# from geometry_msgs.msg import PoseStamped
# from sensor_msgs.msg import JointState
# import csv
# import os
# import time
# import threading
#
#
# class DualMopControlSystem(Node):
#     def __init__(self):
#         super().__init__('dual_mop_control_system')
#
#         # Clients & Publishers
#         self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
#
#         # Publishers for BOTH robots
#         self.ur1_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
#         self.ur2_pub = self.create_publisher(JointTrajectory, '/ur_2_controller/joint_trajectory', 10)
#
#         self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
#
#         # Joint Lists
#         self.ur1_joints = [
#             'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
#             'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
#         ]
#         self.ur2_joints = [
#             'ur_2_shoulder_pan_joint', 'ur_2_shoulder_lift_joint', 'ur_2_elbow_joint',
#             'ur_2_wrist_1_joint', 'ur_2_wrist_2_joint', 'ur_2_wrist_3_joint'
#         ]
#
#         # Home pose (Assuming same home configuration for both)
#         self.home_pose = [1.624, -0.895, 0.889, -1.359, -1.425, 0.00]
#
#         # Geometric Constants (Different heights for each robot)
#         # Robot 1 Heights
#         self.ur1_const_z = 0.65
#         self.ur1_hover_z = 0.75
#
#         # Robot 2 Heights
#         self.ur2_const_z = 1.0
#         self.ur2_hover_z = 1.1
#
#         # State tracking
#         self.current_joints_1 = None
#         self.current_joints_2 = None
#
#     def joint_cb(self, msg):
#         """Extracts joints for both robots simultaneously."""
#         try:
#             pos1 = []
#             for name in self.ur1_joints:
#                 idx = msg.name.index(name)
#                 pos1.append(msg.position[idx])
#             self.current_joints_1 = pos1
#         except ValueError:
#             pass
#
#         try:
#             pos2 = []
#             for name in self.ur2_joints:
#                 idx = msg.name.index(name)
#                 pos2.append(msg.position[idx])
#             self.current_joints_2 = pos2
#         except ValueError:
#             pass
#
#     def get_ik(self, x, y, z, seed, robot_id=1):
#         """Requests IK for the specified robot."""
#         if seed is None: return None
#
#         req = GetPositionIK.Request()
#         req.ik_request.avoid_collisions = True
#
#         # Switch parameters based on which robot we are calculating for
#         if robot_id == 1:
#             req.ik_request.group_name = "ur_manipulator"
#             req.ik_request.ik_link_name = "mop_tip"
#             req.ik_request.robot_state.joint_state.name = self.ur1_joints
#         else:
#             req.ik_request.group_name = "ur_2_manipulator"
#             # Using wrist_3_joint as tip since tool0 isn't explicitly in your SRDF chain
#             req.ik_request.ik_link_name = "ur_2_wrist_3_joint"
#             req.ik_request.robot_state.joint_state.name = self.ur2_joints
#
#         req.ik_request.robot_state.joint_state.position = seed
#
#         target = PoseStamped()
#         target.header.frame_id = "world"
#         target.pose.position.x = x
#         target.pose.position.y = y
#         target.pose.position.z = z
#
#         # Downward Orientation
#         target.pose.orientation.x = 1.0
#         target.pose.orientation.w = 0.0
#         req.ik_request.pose_stamped = target
#
#         future = self.ik_client.call_async(req)
#         rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
#         res = future.result()
#
#         if res and res.error_code.val == 1:
#             return list(res.solution.joint_state.position[:6])
#         return None
#
#     def move_to_joints(self, joint_goal, robot_id=1, duration=2.0):
#         """Builds and publishes trajectory for the target robot."""
#         msg = JointTrajectory()
#         msg.joint_names = self.ur1_joints if robot_id == 1 else self.ur2_joints
#
#         pt = JointTrajectoryPoint()
#         pt.positions = joint_goal
#         pt.time_from_start = Duration(seconds=duration).to_msg()
#         msg.points.append(pt)
#
#         if robot_id == 1:
#             self.ur1_pub.publish(msg)
#         else:
#             self.ur2_pub.publish(msg)
#
#     def run_manual(self):
#         """Live X/Y input. Commands both robots simultaneously."""
#         print("\n--- MANUAL MODE: Enter 'x y' or 'exit' ---")
#         while rclpy.ok():
#             user_input = input("Target (x y): ").split()
#             if not user_input or user_input[0].lower() == 'exit': break
#
#             try:
#                 x, y = float(user_input[0]), float(user_input[1])
#
#                 # Calculate for Robot 1
#                 r1_hover = self.get_ik(x, y, self.ur1_hover_z, self.current_joints_1, robot_id=1)
#                 r1_drop = self.get_ik(x, y, self.ur1_const_z, r1_hover, robot_id=1)
#
#                 # Calculate for Robot 2
#                 r2_hover = self.get_ik(x, y, self.ur2_hover_z, self.current_joints_2, robot_id=2)
#                 r2_drop = self.get_ik(x, y, self.ur2_const_z, r2_hover, robot_id=2)
#
#                 # Execute simultaneous moves if IK passed
#                 if r1_hover and r2_hover:
#                     print(f"Executing: Hovering to {x},{y}...")
#                     self.move_to_joints(r1_hover, robot_id=1, duration=1.5)
#                     self.move_to_joints(r2_hover, robot_id=2, duration=1.5)
#                     time.sleep(1.6)  # Wait for hover to finish
#
#                     print("Dropping to surface...")
#                     self.move_to_joints(r1_drop, robot_id=1, duration=1.0)
#                     self.move_to_joints(r2_drop, robot_id=2, duration=1.0)
#                     time.sleep(1.1)
#                 else:
#                     print("IK Failed for one or both robots: Point out of reach.")
#             except (ValueError, IndexError):
#                 print("Usage: 0.2 0.4")
#
#     def run_csv(self, file_path):
#         """Full CSV trajectory execution for both robots simultaneously."""
#         print(f"--- CSV MODE: Processing {file_path} ---")
#         if not os.path.exists(file_path):
#             print("Error: CSV file not found.")
#             return
#
#         r1_points, r2_points = [], []
#         r1_seed, r2_seed = self.home_pose, self.home_pose
#
#         # 1. 'Bake' the trajectory
#         with open(file_path, 'r') as f:
#             reader = csv.DictReader(f)
#             for row in reader:
#                 x, y = float(row['x']), float(row['y'])
#
#                 sol1 = self.get_ik(x, y, self.ur1_const_z, r1_seed, robot_id=1)
#                 sol2 = self.get_ik(x, y, self.ur2_const_z, r2_seed, robot_id=2)
#
#                 if sol1 and sol2:
#                     r1_points.append(sol1)
#                     r2_points.append(sol2)
#                     r1_seed, r2_seed = sol1, sol2
#
#                     # 2. Execute
#         if r1_points and r2_points:
#             print(f"Solving complete. Executing {len(r1_points)} points...")
#             msg1, msg2 = JointTrajectory(), JointTrajectory()
#             msg1.joint_names, msg2.joint_names = self.ur1_joints, self.ur2_joints
#
#             for i, (p1, p2) in enumerate(zip(r1_points, r2_points)):
#                 pt1, pt2 = JointTrajectoryPoint(), JointTrajectoryPoint()
#                 pt1.positions, pt2.positions = p1, p2
#
#                 t = Duration(seconds=(i + 1) * 0.1).to_msg()
#                 pt1.time_from_start, pt2.time_from_start = t, t
#
#                 msg1.points.append(pt1)
#                 msg2.points.append(pt2)
#
#             self.ur1_pub.publish(msg1)
#             self.ur2_pub.publish(msg2)
#             time.sleep(len(r1_points) * 0.1 + 2.0)
#         else:
#             print("No valid IK solutions found in CSV.")
#
#
# def main():
#     rclpy.init()
#     node = DualMopControlSystem()
#
#     # Spin the node in a background thread to prevent input() from blocking ROS
#     spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
#     spin_thread.start()
#
#     # Wait for sensor data synchronization from BOTH robots
#     while node.current_joints_1 is None or node.current_joints_2 is None:
#         time.sleep(0.1)
#
#     print("1: Manual X/Y")
#     print("2: CSV Sequence")
#     choice = input("Select: ")
#
#     print("Moving both robots to Home...")
#     node.move_to_joints(node.home_pose, robot_id=1, duration=4.0)
#     node.move_to_joints(node.home_pose, robot_id=2, duration=4.0)
#     time.sleep(4.1)
#
#     if choice == '1':
#         node.run_manual()
#     elif choice == '2':
#         csv_file = os.path.expanduser('~/ros2_ws/data/motion_safe.csv')
#         node.run_csv(csv_file)
#
#     rclpy.shutdown()
#     spin_thread.join()
#
#
# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import csv
import os
import time
import threading


class DualMopControlSystem(Node):
    def __init__(self):
        super().__init__('dual_mop_control_system')

        # Clients & Publishers
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.ur1_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.ur2_pub = self.create_publisher(JointTrajectory, '/ur_2_controller/joint_trajectory', 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

        # Joint Lists
        self.ur1_joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                           'wrist_3_joint']
        self.ur2_joints = ['ur_2_shoulder_pan_joint', 'ur_2_shoulder_lift_joint', 'ur_2_elbow_joint',
                           'ur_2_wrist_1_joint', 'ur_2_wrist_2_joint', 'ur_2_wrist_3_joint']

        # Poses
        self.home_pose = [1.624, -0.895, 0.889, -1.359, -1.425, 0.00]
        self.zero_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Heights
        self.ur1_const_z, self.ur1_hover_z = 0.65, 0.75
        self.ur2_const_z, self.ur2_hover_z = 1.0, 1.1

        self.current_joints_1 = None
        self.current_joints_2 = None

    def joint_cb(self, msg):
        """Extracts joints for both robots."""
        try:
            self.current_joints_1 = [msg.position[msg.name.index(name)] for name in self.ur1_joints]
        except (ValueError, IndexError):
            pass
        try:
            self.current_joints_2 = [msg.position[msg.name.index(name)] for name in self.ur2_joints]
        except (ValueError, IndexError):
            pass

    def verify_at_zero(self, robot_id):
        """Blocks until the robot is physically at 0.0 (Works for Real & Sim)."""
        print(f"Waiting for Robot {robot_id} to reach Zero Position...")
        tolerance = 0.01
        while rclpy.ok():
            current = self.current_joints_1 if robot_id == 1 else self.current_joints_2
            if current and all(abs(j) < tolerance for j in current):
                print(f"--- CONFIRMED: ROBOT {robot_id} IS AT ZERO POSITION ---")
                break
            time.sleep(0.2)

    def get_ik(self, x, y, z, seed, robot_id=1):
        """Requests IK using a manual check on future.done() to avoid Executor errors."""
        if seed is None: return None

        req = GetPositionIK.Request()
        req.ik_request.avoid_collisions = True

        if robot_id == 1:
            req.ik_request.group_name, req.ik_request.ik_link_name = "ur_manipulator", "mop_tip"
            req.ik_request.robot_state.joint_state.name = self.ur1_joints
        else:
            req.ik_request.group_name, req.ik_request.ik_link_name = "ur_2_manipulator", "ur_2_tool0"
            req.ik_request.robot_state.joint_state.name = self.ur2_joints

        req.ik_request.robot_state.joint_state.position = seed

        target = PoseStamped()
        target.header.frame_id = "world"
        target.pose.position.x, target.pose.position.y, target.pose.position.z = x, y, z
        target.pose.orientation.x, target.pose.orientation.w = 1.0, 0.0
        req.ik_request.pose_stamped = target

        # Call the service asynchronously
        future = self.ik_client.call_async(req)

        # Wait for the background thread to finish the service call
        start_wait = time.time()
        while not future.done():
            time.sleep(0.01)
            if time.time() - start_wait > 2.0:
                self.get_logger().error("IK Service Timeout!")
                return None

        res = future.result()
        if res and res.error_code.val == 1:
            return list(res.solution.joint_state.position[:6])
        else:
            return None

    def move_to_joints(self, joint_goal, robot_id=1, duration=3.0):
        """Your existing movement logic."""
        msg = JointTrajectory()
        msg.joint_names = self.ur1_joints if robot_id == 1 else self.ur2_joints
        msg.points.append(
            JointTrajectoryPoint(positions=joint_goal, time_from_start=Duration(seconds=duration).to_msg()))
        if robot_id == 1:
            self.ur1_pub.publish(msg)
        else:
            self.ur2_pub.publish(msg)
        time.sleep(duration + 0.1)

    def run_robot_phase(self, robot_id):
        """Handles the sequence for one robot then forces it to zero."""
        print(f"\n--- STARTING PHASE FOR ROBOT {robot_id} ---")

        # Initial Home Move
        self.move_to_joints(self.home_pose, robot_id, duration=4.0)

        print("1: Manual X/Y | 2: CSV Sequence")
        choice = input(f"Selection for Robot {robot_id}: ")

        if choice == '1':
            while rclpy.ok():
                inp = input(f"Robot {robot_id} Target (x y) or 'exit': ").split()
                if not inp or inp[0].lower() == 'exit': break
                try:
                    x, y = float(inp[0]), float(inp[1])
                    z = self.ur1_const_z if robot_id == 1 else self.ur2_const_z
                    seed = self.current_joints_1 if robot_id == 1 else self.current_joints_2
                    sol = self.get_ik(x, y, z, seed, robot_id)
                    if sol:
                        self.move_to_joints(sol, robot_id)
                    else:
                        print("IK Failed for these coordinates.")
                except:
                    pass

        elif choice == '2':
            csv_path = os.path.expanduser('~/ros2-master/data/motion_safe.csv')
            if not os.path.exists(csv_path):
                print(f"Error: CSV file not found at {csv_path}")
                return

            points = []
            # Start seed for CSV from the current position
            last_seed = self.current_joints_1 if robot_id == 1 else self.current_joints_2
            z_height = self.ur1_const_z if robot_id == 1 else self.ur2_const_z

            print(f"Calculating (Baking) Trajectory for Robot {robot_id}...")
            with open(csv_path, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    sol = self.get_ik(float(row['x']), float(row['y']), z_height, last_seed, robot_id)
                    if sol:
                        points.append(sol)
                        last_seed = sol

            if points:
                print(f"Baking complete. Executing {len(points)} points...")
                msg = JointTrajectory()
                msg.joint_names = self.ur1_joints if robot_id == 1 else self.ur2_joints
                for i, p in enumerate(points):
                    msg.points.append(
                        JointTrajectoryPoint(positions=p, time_from_start=Duration(seconds=(i + 1) * 0.2).to_msg()))

                if robot_id == 1:
                    self.ur1_pub.publish(msg)
                else:
                    self.ur2_pub.publish(msg)

                # Wait for trajectory to finish
                time.sleep(len(points) * 0.2 + 2.0)
            else:
                print("No valid IK solutions found in CSV.")

        # END OF ACTIVITY: MANDATORY RETURN TO ZERO
        print(f"Action Complete. Moving Robot {robot_id} to ZERO POSITION...")
        self.move_to_joints(self.zero_pose, robot_id, duration=5.0)
        self.verify_at_zero(robot_id)


def main():
    rclpy.init()
    node = DualMopControlSystem()

    # Spin thread handles communication so get_ik future.done() works
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Wait for joints to be populated
    while node.current_joints_1 is None or node.current_joints_2 is None:
        time.sleep(0.1)

    # RELAY STEP 1: Robot 1 executes its manual or CSV sequence
    node.run_robot_phase(robot_id=1)

    # RELAY STEP 2: Mandatory 6 second delay after R1 is verified at zero
    print("Robot 1 is at Zero. Cooling down for 6 seconds...")
    time.sleep(6.0)

    # RELAY STEP 3: Robot 2 activates for its manual or CSV sequence
    node.run_robot_phase(robot_id=2)

    print("Relay sequence finished. Both robots verified at zero.")

    rclpy.shutdown()
    spin_thread.join()


if __name__ == '__main__':
    main()