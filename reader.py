import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import sys


class UR2MotionExecutor(Node):
    def __init__(self):
        super().__init__('ur2_motion_executor')

        # Connect directly to the active ur_2_controller
        self._action_client = ActionClient(
            self, FollowJointTrajectory, '/ur_2_controller/follow_joint_trajectory')

        # Exact joint names from your XACRO
        self.joint_names = [
            'ur_2_shoulder_pan_joint',
            'ur_2_shoulder_lift_joint',
            'ur_2_elbow_joint',
            'ur_2_wrist_1_joint',
            'ur_2_wrist_2_joint',
            'ur_2_wrist_3_joint'
        ]

    def deg_to_rad(self, degrees_list):
        """Converts a list of 6 degrees to radians"""
        return [math.radians(d) for d in degrees_list]

    def send_goal(self):
        self.get_logger().info('Waiting for ur_2_controller action server...')
        self._action_client.wait_for_server()

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names



        home_rest_deg = [-16.57, -71.79, 127.75, -230.10, -86.40, 192.74]#[165.85, -55.20, 56.40, -176.72, -96.61, 166.75] #[2.89530,−0.96376,0.98408,−3.08301,−1.68612,2.91085]
        work_point_deg = [88.82,2.22,0.01,-176.72,-96.61,166.75] #[1.55057,0.03874,0.00017,−3.08301,−1.68612,2.91085]
        motion_pt1_deg = [62.82,2.22,0.01,-176.72,-96.61,166.75] #[1.09622,0.03874,0.00017,−3.08301,−1.68612,2.91085]
        motion_pt2_deg = [117.3,2.22,0.01,-176.72,-96.61,166.75] #[2.04730,0.03874,0.00017,−3.08301,−1.68612,2.91085]
        motion_pt3_deg = [-17.66, -73.09, 121.40, -231.08, -92.83, 181.31]
        motion_pt4_deg = [72.91, -63.50, 101.99, -217.47, -92.74, 98.49]

        motion_pt6_deg = [-17.74, -76.03, 122.21, -150.27, -92.92, 181.31]
        motion_pt7_deg = [33.27, -75.79, 95.34, -212.50, -95.39, 179.87]
        scoop_deg =[-11.11, -60.74, 124.15, -194.51, -92.74, 174.65]
        work_safe_deg =[84.32, -40.01, 44.64, -194.52, -92.74, 174.65]



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


def main(args=None):
    rclpy.init(args=args)
    action_client = UR2MotionExecutor()
    action_client.send_goal()

    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass
    finally:
        action_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()