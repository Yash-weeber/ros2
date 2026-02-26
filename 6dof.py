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

        # Simulation uses /ur_1_controller; Real hardware uses /scaled_joint_trajectory_controller
        self.ur1_pub = self.create_publisher(JointTrajectory, '/ur_1_controller/joint_trajectory', 10)

        self.ur1_joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                           'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        self.ur1_home = [1.558, -1.14, 1.051, -1.425, -1.690, 0.033]
        self.ur1_zero = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.ur1_current = []
        self.ur1_group = 'ur_manipulator'
        self.ur1_base = 'base_link'
        self.target_frame = "mop_tip"

    def joint_state_callback(self, msg):
        if all(n in msg.name for n in self.ur1_joints):
            try:
                self.ur1_current = [msg.position[msg.name.index(n)] for n in self.ur1_joints]
            except Exception:
                pass

    def get_fk_pose(self, group, base_link, current_joints, joint_names):
        req = GetPositionFK.Request()
        req.header.frame_id = base_link
        req.fk_link_names = [self.target_frame]
        req.robot_state.joint_state.name = joint_names
        req.robot_state.joint_state.position = current_joints

        future = self.fk_client.call_async(req)
        start = time.time()
        while not future.done():
            time.sleep(0.01)
            if time.time() - start > 5.0: return None

        res = future.result()
        return res.pose_stamped[0].pose if res and res.error_code.val == 1 else None

    def solve_ik_with_sampling(self, x, y, z, current_joints):
        """
        Samples different rotations to find a valid 6-DOF solution.
        This prevents 'forcing' a single orientation that might be unreachable.
        """
        # Testing 8 angles (0, 45, 90... 315 degrees) around the vertical axis
        yaws = [i * (math.pi / 4) for i in range(8)]

        for yaw in yaws:
            target_pose = Pose()
            target_pose.position.x = x
            target_pose.position.y = y
            target_pose.position.z = z

            # Quaternion for Mop Down (Roll=PI) with variable Yaw
            target_pose.orientation.x = 0.0
            target_pose.orientation.y = 1.0
            target_pose.orientation.z = 0.0
            target_pose.orientation.w = math.cos(yaw / 2.0)

            req = GetPositionIK.Request()
            req.ik_request.group_name = self.ur1_group
            req.ik_request.robot_state.joint_state.name = self.ur1_joints
            req.ik_request.robot_state.joint_state.position = current_joints
            req.ik_request.pose_stamped.header.frame_id = self.ur1_base
            req.ik_request.pose_stamped.pose = target_pose
            req.ik_request.ik_link_name = self.target_frame
            req.ik_request.avoid_collisions = True

            future = self.ik_client.call_async(req)
            start = time.time()
            while not future.done():
                time.sleep(0.01)
                if time.time() - start > 1.0: break

            if future.done():
                res = future.result()
                if res and res.error_code.val == 1:
                    return list(res.solution.joint_state.position), 1

        return None, -1

    def execute_trajectory(self, publisher, joint_names, joints, time_step=3.0):
        msg = JointTrajectory()
        msg.joint_names = joint_names
        pt = JointTrajectoryPoint()
        pt.positions = joints
        pt.time_from_start = Duration(seconds=time_step).to_msg()
        msg.points.append(pt)
        publisher.publish(msg)
        return time_step

    def run_robot_task(self, robot_name, publisher, joint_names, home_joints, end_joints, group, base_link):
        self.get_logger().info(f"--- STARTING AUTOMATIC 6-DOF TASK ---")

        # Start Move
        self.execute_trajectory(publisher, joint_names, home_joints, time_step=5.0)
        time.sleep(5.5)

        perfect_pose = self.get_fk_pose(group, base_link, home_joints, joint_names)
        if not perfect_pose:
            self.get_logger().error("FK Failed! Cannot lock Z height.")
            return

        # Height locked based on the mop_tip position in your ASU setup
        home_z = perfect_pose.position.z - 0.2
        self.get_logger().info(f"Reachability height locked at Z: {home_z:.3f}")

        while rclpy.ok():
            try:
                user_input = input(f"\nEnter Target X Y (or 'q' to quit): ").strip()
                if user_input.lower() in ['quit', 'q']: break

                parts = user_input.split()
                if len(parts) != 2: continue

                tx, ty = float(parts[0]), float(parts[1])
                # Use current position or home as the starting point for the solver
                seed = self.ur1_current if self.ur1_current else home_joints

                # IK handles the orientation automatically now
                sol, err = self.solve_ik_with_sampling(tx, ty, home_z, seed)

                if sol:
                    print(f"✓ Valid 6-DOF solution found! Moving tip to X:{tx} Y:{ty}...")
                    self.execute_trajectory(publisher, joint_names, sol, time_step=3.0)
                    time.sleep(3.5)
                else:
                    print(f"✗ Point unreachable. Solver could not find a safe orientation.")

            except Exception as e:
                print(f"Error: {e}")

        # Park
        self.execute_trajectory(publisher, joint_names, home_joints, time_step=4.0)
        time.sleep(4.5)
        self.execute_trajectory(publisher, joint_names, end_joints, time_step=4.0)


def main(args=None):
    rclpy.init(args=args)
    node = SingleRobotControl()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    t = threading.Thread(target=executor.spin, daemon=True)
    t.start()

    node.run_robot_task("UR1", node.ur1_pub, node.ur1_joints, node.ur1_home, node.ur1_zero, node.ur1_group,
                        node.ur1_base)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()