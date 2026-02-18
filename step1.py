


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


# import rclpy
# from rclpy.node import Node
# from moveit_msgs.srv import GetPositionIK
# from sensor_msgs.msg import JointState
# from geometry_msgs.msg import PoseStamped
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from rclpy.duration import Duration
# import csv
# import os
# import time
#
#
# class MopIKDiagnostic(Node):
#     def __init__(self):
#         super().__init__('mop_ik_diagnostic')
#         self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
#         self.joint_pub = self.create_publisher(JointTrajectory, '/ur_1_controller/joint_trajectory', 10)
#         self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
#
#         self.current_state = None
#         self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
#                             'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
#
#         self.CONST_Z = 1.1
#
#         # Path setup
#         home_dir = os.path.expanduser('~')
#         self.csv_path = os.path.join(home_dir, 'ros2_ws', 'data', 'motion_safe.csv')
#
#     def joint_cb(self, msg):
#         if all(n in msg.name for n in self.joint_names):
#             self.current_state = msg
#
#     def test_move(self, x, y):
#         if not self.current_state:
#             print("WAITING: No joint states received.")
#             return False
#
#         req = GetPositionIK.Request()
#         req.ik_request.group_name = "ur_manipulator"
#         req.ik_request.ik_link_name = "tool0"
#         # Safety enabled now that SRDF/Macro is updated
#         req.ik_request.avoid_collisions = True
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
#             ordered_joints = [res.solution.joint_state.position[res.solution.joint_state.name.index(n)] for n in
#                               self.joint_names]
#             print(f"SUCCESS | Point: {x}, {y}")
#             self.execute(ordered_joints)
#             return True
#         else:
#             print(f"FAILED | Point {x}, {y} (Code: {res.error_code.val if res else 'Timeout'})")
#             return False
#
#     def execute(self, joints):
#         msg = JointTrajectory()
#         msg.joint_names = self.joint_names
#         p = JointTrajectoryPoint()
#         p.positions = joints
#         p.time_from_start = Duration(seconds=1.5).to_msg()
#         msg.points.append(p)
#         self.joint_pub.publish(msg)
#         # Sequence timing to ensure the robot reaches the target
#         time.sleep(1.8)
#
#     def run_csv(self):
#         if not os.path.exists(self.csv_path):
#             print(f"ERROR: File not found: {self.csv_path}")
#             return
#
#         print(f"READING MOTION: {self.csv_path}")
#         with open(self.csv_path, mode='r') as f:
#             # Using DictReader to handle columns by name
#             reader = csv.DictReader(f)
#             for row in reader:
#                 if not rclpy.ok(): break
#                 try:
#                     # Extract specifically from your 'x' and 'y' columns
#                     x_val = float(row['x'])
#                     y_val = float(row['y'])
#                     self.test_move(x_val, y_val)
#                     rclpy.spin_once(self, timeout_sec=0.1)
#                 except (ValueError, KeyError) as e:
#                     print(f"Skipping row due to error: {e}")
#         print("\n--- SEQUENCE FINISHED ---")
#
#
# def main():
#     rclpy.init()
#     node = MopIKDiagnostic()
#
#     print("\n--- UR5e MOP CONTROL ---")
#     print("1: Manual Entry")
#     print("2: CSV Sequence (x,y)")
#     mode = input("Select Mode >> ")
#
#     try:
#         if mode == '2':
#             # Seed the state once before starting
#             while node.current_state is None:
#                 rclpy.spin_once(node, timeout_sec=0.1)
#             node.run_csv()
#         else:
#             while rclpy.ok():
#                 rclpy.spin_once(node, timeout_sec=0.1)
#                 inp = input("Enter x y >> ").split()
#                 if len(inp) == 2:
#                     node.test_move(float(inp[0]), float(inp[1]))
#                 elif len(inp) == 1 and inp[0].lower() == 'exit':
#                     break
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
from geometry_msgs.msg import PoseStamped, Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker
from gazebo_msgs.srv import SpawnEntity  # To inject 3D visuals into Gazebo
from rclpy.duration import Duration
import csv
import os
import time


class MopIKDiagnostic(Node):
    def __init__(self):
        super().__init__('mop_ik_diagnostic')

        # Clients
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')

        # Publishers & Subscribers
        self.joint_pub = self.create_publisher(JointTrajectory, '/ur_1_controller/joint_trajectory', 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

        # Robot State
        self.current_state = None
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        # Geometry Constants
        self.CONST_Z = 1.1  # Robot target height
        self.VIZ_Z = 0.92  # Gazebo visual height (just above table)

        # Internal Counters/Storage
        self.marker_count = 0
        self.path_points = []

        # Path setup
        home_dir = os.path.expanduser('~')
        self.csv_path = os.path.join(home_dir, 'ros2_ws', 'data', 'motion_safe.csv')

    def joint_cb(self, msg):
        if all(n in msg.name for n in self.joint_names):
            self.current_state = msg

    def spawn_gazebo_marker(self, x, y):
        """Spawns a small visual disk in the Gazebo world as a 'breadcrumb'."""
        if not self.spawn_client.service_is_ready():
            return

        # Simple SDF for a thin, semi-transparent blue disk (No Collision)
        sdf = f"""
        <sdf version="1.6">
          <model name="marker_{self.marker_count}">
            <static>true</static>
            <link name="link">
              <visual name="visual">
                <geometry>
                  <cylinder>
                    <radius>0.04</radius>
                    <length>0.002</length>
                  </cylinder>
                </geometry>
                <material>
                  <ambient>0.0 0.5 1.0 0.6</ambient>
                  <diffuse>0.0 0.5 1.0 0.6</diffuse>
                </material>
              </visual>
            </link>
          </model>
        </sdf>
        """

        request = SpawnEntity.Request()
        request.name = f"marker_{self.marker_count}"
        request.xml = sdf
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = self.VIZ_Z

        self.spawn_client.call_async(request)
        self.marker_count += 1

    def publish_rviz_marker(self, x, y):
        """Updates the persistent contact trail in RViz."""
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "mop_contact_points"
        marker.id = 1
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD

        p = Point()
        p.x = x
        p.y = y
        p.z = self.VIZ_Z

        self.path_points.append(p)
        marker.points = self.path_points

        # Marker Appearance
        marker.scale.x = 0.10
        marker.scale.y = 0.10
        marker.scale.z = 0.01
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 0.7, 1.0, 0.5

        self.marker_pub.publish(marker)

    def test_move(self, x, y, spawn_viz=True):
        if not self.current_state:
            print("WAITING: No joint states received.")
            return False

        req = GetPositionIK.Request()
        req.ik_request.group_name = "ur_manipulator"
        req.ik_request.ik_link_name = "tool0"
        req.ik_request.avoid_collisions = True  # Respecting our SRDF ignore rules
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
            print(f"SUCCESS | Point: {x:.3f}, {y:.3f}")

            # Execute physical move
            self.execute_trajectory(ordered_joints)

            # Update Visuals
            self.publish_rviz_marker(x, y)
            if spawn_viz:
                self.spawn_gazebo_marker(x, y)
            return True
        else:
            print(f"FAILED | Point {x}, {y} (Code: {res.error_code.val if res else 'Timeout'})")
            return False

    def execute_trajectory(self, joints):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        p = JointTrajectoryPoint()
        p.positions = joints
        p.time_from_start = Duration(seconds=1.2).to_msg()
        msg.points.append(p)
        self.joint_pub.publish(msg)
        time.sleep(1.4)  # Control loop stabilization

    def run_csv(self):
        if not os.path.exists(self.csv_path):
            print(f"ERROR: File not found: {self.csv_path}")
            return

        print(f"READING MOTION: {self.csv_path}")
        with open(self.csv_path, mode='r') as f:
            reader = csv.DictReader(f)
            step_counter = 0
            for row in reader:
                if not rclpy.ok(): break
                try:
                    x_val = float(row['x'])
                    y_val = float(row['y'])

                    # Optimization: Spawn in Gazebo every 5 points to prevent lag
                    do_spawn = (step_counter % 5 == 0)
                    self.test_move(x_val, y_val, spawn_viz=do_spawn)

                    step_counter += 1
                    rclpy.spin_once(self, timeout_sec=0.1)
                except (ValueError, KeyError) as e:
                    print(f"Skipping row: {e}")
        print("\n--- CSV SEQUENCE COMPLETE ---")


def main():
    rclpy.init()
    node = MopIKDiagnostic()

    print("\n--- UR5e MOP CLEANING (Gazebo + RViz Viz) ---")
    print("1: Manual Point Entry")
    print("2: CSV Sequence Mode")
    mode = input("Select Mode >> ")

    try:
        if mode == '2':
            # Pre-seed state
            print("Waiting for robot joint states...")
            while node.current_state is None:
                rclpy.spin_once(node, timeout_sec=0.1)
            print("Robot state received! Opening CSV...")
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