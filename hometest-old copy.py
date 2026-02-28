import sys

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import csv
import os
import numpy as np
import time
import threading
from moveit_msgs.msg import MoveItErrorCodes
import math


class DualMopControlSystem(Node):
    def __init__(self):
        super().__init__('dual_mop_control_system')

        # Clients & Publishers
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        # self.ur1_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.ur1_pub = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)
        self.ur2_pub = self.create_publisher(JointTrajectory, '/ur_2_controller/joint_trajectory', 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

        # Joint Lists
        self.ur1_joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                           'wrist_3_joint']
        self.ur2_joints = ['ur_2_shoulder_pan_joint', 'ur_2_shoulder_lift_joint', 'ur_2_elbow_joint',
                           'ur_2_wrist_1_joint', 'ur_2_wrist_2_joint', 'ur_2_wrist_3_joint']

        # Poses
        # self.home_pose = [1.624-3.14, -0.895, 0.889, -1.359, -1.425, 0.00]
        rest_qs_r1 = [13.53, -57.08, 64.66, -144.75, -88.89, 28.75]
        rest_qs_r2 = [0, -57.08, 64.66, -144.75, -88.89, 28.75]
        # home_qs_r1 = [-57.41, -92, 115, -136.1, -85.39, -10.33]
        home_qs_r1 = [-59.17, -69.74, 96.13, -122.94, -89.73, 9.94]
        home_qs_r2 = [0, -74, 138, -174, -82, 0]
        self.home_pose_r1 = self.deg_to_rad(home_qs_r1)
        self.home_pose_r2 = self.deg_to_rad(home_qs_r2)
        # self.home_pose = [148/180 * np.pi, -101/180 * np.pi, -113/180 * np.pi, -5.7/180 * np.pi, 72.58/180 * np.pi, 219.5/180 * np.pi]
        # self.home_pose  = [-1.500063721333639, -2.2403162161456507, 2.4708008766174316, -0.12481147447694951, 1.21921968460083, 3.2130491733551025]


        
        self.rest_pose_r1 = self.deg_to_rad(rest_qs_r1)  # Convert to radians
        self.rest_pose_r2 = self.deg_to_rad(rest_qs_r2)  # Convert to radians

        # Heights
        self.ur1_const_z, self.ur1_hover_z = 0.76, 0.756
        self.ur2_const_z, self.ur2_hover_z = 1.0, 1.1

        self.current_joints_1 = None
        self.current_joints_2 = None
        
    def deg_to_rad(self, degrees_list):
        """Converts a list of 6 degrees to radians"""
        return [math.radians(d) for d in degrees_list]

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
            if current and all(abs(j - k) < tolerance for j, k in zip(current, self.rest_pose_r1 if robot_id == 1 else self.rest_pose_r2)):
                print(f"--- CONFIRMED: ROBOT {robot_id} IS AT REST POSITION ---")
                break
            time.sleep(0.2)

    def _reorder_positions(self, target_names, source_names, source_positions):
        src = dict(zip(source_names, source_positions))
        return [src[name] for name in target_names]

    def get_ik(self, x, y, z, seed, robot_id=1):
        """Requests IK using a manual check on future.done() to avoid Executor errors."""
        if seed is None:
            return None

        req = GetPositionIK.Request()
        req.ik_request.avoid_collisions = True

        if robot_id == 1:
            req.ik_request.group_name, req.ik_request.ik_link_name = "ur_manipulator", "mop_tip"
            req.ik_request.robot_state.joint_state.name = self.ur1_joints
            controller_joint_order = self.ur1_joints
        else:
            req.ik_request.group_name, req.ik_request.ik_link_name = "ur_2_manipulator", "ur_2_tool0"
            req.ik_request.robot_state.joint_state.name = self.ur2_joints
            controller_joint_order = self.ur2_joints

        req.ik_request.robot_state.joint_state.position = seed

        target = PoseStamped()
        target.header.frame_id = "world"
        target.pose.position.x, target.pose.position.y, target.pose.position.z = x, y, z
        target.pose.orientation.x, target.pose.orientation.w = 1.0, 0.0
        req.ik_request.pose_stamped = target

        future = self.ik_client.call_async(req)

        start_wait = time.time()
        while not future.done():
            time.sleep(0.01)
            if time.time() - start_wait > 2.0:
                self.get_logger().error("IK Service Timeout!")
                return None

        res = future.result()
        if res is None:
            self.get_logger().error("IK response is None")
            return None

        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"IK failed. robot_id={robot_id}, group={req.ik_request.group_name}, "
                f"link={req.ik_request.ik_link_name}, frame={target.header.frame_id}, "
                f"error_code={res.error_code.val}"
            )
            return None

        names = list(res.solution.joint_state.name)
        positions = list(res.solution.joint_state.position)
        try:
            return self._reorder_positions(controller_joint_order, names, positions)
        except KeyError as e:
            self.get_logger().error(f"IK result missing joint: {e}")
            return None
        return None

    def move_to_joints(self, joint_goal, robot_id=1, duration=3.0):
        """Publish trajectory with strict controller joint ordering."""
        controller_joint_names = self.ur1_joints if robot_id == 1 else self.ur2_joints

        if not isinstance(joint_goal, (list, tuple)) or len(joint_goal) != len(controller_joint_names):
            self.get_logger().error(
                f"Invalid joint_goal for robot {robot_id}. "
                f"Expected {len(controller_joint_names)} values in controller order: {controller_joint_names}"
            )
            return

        msg = JointTrajectory()
        msg.joint_names = controller_joint_names

        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in joint_goal]
        pt.time_from_start = Duration(seconds=duration).to_msg()
        msg.points.append(pt)

        if robot_id == 1:
            self.ur1_pub.publish(msg)
        else:
            self.ur2_pub.publish(msg)

        time.sleep(duration + 0.1)

    def run_robot_phase(self, robot_id):
        """Handles the sequence for one robot then forces it to zero."""
        print(f"\n--- STARTING PHASE FOR ROBOT {robot_id} ---")

        # Initial Home Move
        self.move_to_joints(self.home_pose_r1 if robot_id == 1 else self.home_pose_r2, robot_id, duration=4.0)

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
                        print(f"IK Success! Moving Robot {robot_id} to {x}, {y}...")
                        self.move_to_joints(sol, robot_id)
                    else:
                        print("IK Failed for these coordinates.")
                except:
                    pass

        elif choice == '2':
            csv_path = os.path
            csv_path = os.path.expanduser('/mnt/sdc/GitHub/ros2/data/motion_safe.csv')
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

        # END OF ACTIVITY: MANDATORY RETURN TO REST POSITION
        print(f"Action Complete. Moving Robot {robot_id} to REST POSITION...")
        self.move_to_joints(self.rest_pose_r1 if robot_id == 1 else self.rest_pose_r2, robot_id, duration=5.0)
        self.verify_at_zero(robot_id)

    def send_goal(self):
        self.get_logger().info('Waiting for ur_2_controller action server...')
        self._action_client.wait_for_server()

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names



        home_rest_deg = [-16.57, -71.79, 127.75, -230.10, -86.40, 192.74]   #2.89530, -0.96376, 0.98408, -3.08301, -1.68612, 2.91085
        #[165.85, -55.20, 56.40, -176.72, -96.61, 166.75] #[2.89530,−0.96376,0.98408,−3.08301,−1.68612,2.91085]
        work_point_deg = [88.82,2.22,0.01,-176.72,-96.61,166.75] #[1.55057,0.03874,0.00017,−3.08301,−1.68612,2.91085]
        motion_pt1_deg = [62.82,2.22,0.01,-176.72,-96.61,166.75] #[1.09622,0.03874,0.00017,−3.08301,−1.68612,2.91085]
        motion_pt2_deg = [117.3,2.22,0.01,-176.72,-96.61,166.75] #[2.04730,0.03874,0.00017,−3.08301,−1.68612,2.91085]
        motion_pt3_deg = [-17.66, -73.09, 121.40, -231.08, -92.83, 181.31]# [-0.3083, -1.2760, 2.1188, -4.0330, -1.6203, 3.1644]

        motion_pt4_deg = [72.91, -63.50, 101.99, -217.47, -92.74, 98.49]# [1.2726, -1.1083, 1.7802, -3.7961, -1.6188, 1.7191]

        motion_pt6_deg = [-17.74, -76.03, 122.21, -150.27, -92.92, 181.31]# [-0.3097, -1.3274, 2.1329, -2.6228, -1.6219, 3.1644]
        motion_pt7_deg = [33.27, -75.79, 95.34, -212.50, -95.39, 179.87]# [0.5808, -1.3232, 1.6641, -3.7093, -1.6648, 3.1393]
        scoop_deg =[-11.11, -60.74, 124.15, -194.51, -92.74, 174.65]# [-0.1939, -1.0602, 2.1673, -3.3954, -1.6188, 3.0481]
        work_safe_deg =[84.32, -40.01, 44.64, -194.52, -92.74, 174.65]# [1.4718, -0.6983, 0.7792, -3.3956, -1.6188, 3.0481]



        # Convert to radians
        points_rad = [
            self.deg_to_rad(home_rest_deg),#same side home
            self.deg_to_rad(work_safe_deg),#hover over the table
            self.deg_to_rad(work_point_deg),#at the table touching
            self.deg_to_rad(motion_pt1_deg),#move right first for sweep
            self.deg_to_rad(motion_pt2_deg),#moves left for second sweep
            self.deg_to_rad(motion_pt3_deg),#hover over box


            self.deg_to_rad(motion_pt6_deg),#scoops in start
            self.deg_to_rad(scoop_deg),#scoops the beads
            self.deg_to_rad(motion_pt7_deg),# goes over table
            self.deg_to_rad(motion_pt4_deg),# dumps the beads




            self.deg_to_rad(home_rest_deg) #goes to home again
            # Return to home
        ]

        # Time durations (in seconds) for each point from the start
        # Adjust these to make the robot move faster or slower!
        time_from_start = [5.0,10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0,55.0]

        # Build the trajectory points
        for i, angles in enumerate(points_rad):
            point = JointTrajectoryPoint()
            point.positions = angles

            # Set timing
            duration = Duration()
            duration.sec = int(time_from_start[i])
            duration.nanosec = int((time_from_start[i] - int(time_from_start[i])) * 1e9)
            point.time_from_start = duration

            goal_msg.trajectory.points.append(point)

        self.get_logger().info('Sending trajectory to robot 2...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Trajectory rejected by controller!')
            sys.exit(0)

        self.get_logger().info('Trajectory accepted. Robot is moving!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Motion finished with error code: {result.error_code}')
        sys.exit(0)

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