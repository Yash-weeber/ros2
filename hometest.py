import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import csv
import os
import numpy as np
import time
import threading
from moveit_msgs.msg import MoveItErrorCodes
import argparse
import math


class DualMopControlSystem(Node):
    def __init__(self):
        super().__init__('dual_mop_control_system')

        # Clients & Publishers
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/compute_ik service not available")

        self.ur1_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.ur2_pub = self.create_publisher(JointTrajectory, '/ur_2_controller/joint_trajectory', 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

        # Joint Lists
        self.ur1_joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.ur2_joints = ['ur_2_shoulder_pan_joint', 'ur_2_shoulder_lift_joint', 'ur_2_elbow_joint', 'ur_2_wrist_1_joint', 'ur_2_wrist_2_joint', 'ur_2_wrist_3_joint']

        self.robot_cfg = {
            1: {"group": "ur_manipulator", "ik_links": ["mop_tip", "tool0"], "joints": self.ur1_joints, "z": 0.65},
            2: {"group": "ur_2_manipulator", "ik_links": ["ur_2_tool0"], "joints": self.ur2_joints, "z": 1.0},
        }

        # Poses
        rest_qs = [13.53, -57.08, 64.66, -144.75, -88.89, 28.75]
        home_qs = [-65, -74, 138, -174, -82, 0]
        # self.home_pose = [148/180*np.pi, -101/180*np.pi, -113/180*np.pi, -5.7/180*np.pi, 72.58/180*np.pi, 219.5/180*np.pi]
        self.home_pose = [q * np.pi / 180 for q in home_qs]
        self.zero_pose = [q * np.pi / 180 for q in rest_qs]

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

    def _reorder_positions(self, target_names, source_names, source_positions):
        src = dict(zip(source_names, source_positions))
        return [src[name] for name in target_names]

    def _nearest_equivalent(self, ref, target):
        """Map target angle to equivalent closest to ref (prevents big wraps)."""
        while target - ref > math.pi:
            target -= 2.0 * math.pi
        while target - ref < -math.pi:
            target += 2.0 * math.pi
        return target

    def _get_seed(self, robot_id):
        return self.current_joints_1 if robot_id == 1 else self.current_joints_2

    def get_ik(self, x, y, z, seed, robot_id=1):
        if seed is None:
            return None

        cfg = self.robot_cfg[robot_id]
        target = PoseStamped()
        target.header.frame_id = "world"
        target.pose.position.x, target.pose.position.y, target.pose.position.z = x, y, z
        target.pose.orientation.x = 0.0
        target.pose.orientation.y = 0.0
        target.pose.orientation.z = 0.0
        target.pose.orientation.w = 1.0

        for ik_link in cfg["ik_links"]:
            req = GetPositionIK.Request()
            req.ik_request.group_name = cfg["group"]
            req.ik_request.ik_link_name = ik_link
            req.ik_request.avoid_collisions = True
            req.ik_request.timeout.sec = 1
            req.ik_request.robot_state.joint_state.name = cfg["joints"]
            req.ik_request.robot_state.joint_state.position = seed
            req.ik_request.pose_stamped = target

            future = self.ik_client.call_async(req)
            start_wait = time.time()
            while not future.done():
                time.sleep(0.01)
                if time.time() - start_wait > 3.0:
                    self.get_logger().error(f"IK timeout (robot={robot_id}, link={ik_link})")
                    break

            if not future.done():
                continue

            res = future.result()
            if not res:
                continue

            if res.error_code.val == MoveItErrorCodes.SUCCESS:
                names = list(res.solution.joint_state.name)
                positions = list(res.solution.joint_state.position)
                try:
                    ordered = self._reorder_positions(cfg["joints"], names, positions)
                    return [self._nearest_equivalent(seed[i], ordered[i]) for i in range(len(ordered))]
                except KeyError as e:
                    self.get_logger().error(f"IK missing joint: {e}")
                    return None

        self.get_logger().error(
            f"IK failed. robot_id={robot_id}, group={cfg['group']}, frame=world, code=NO_IK_SOLUTION"
        )
        return None

    def load_cartesian_points_from_csv(self, csv_path):
        pts = []
        with open(csv_path, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                pts.append({
                    "x": float(row["x"]),
                    "y": float(row["y"]),
                    "z": float(row["z"]) if "z" in row and row["z"] != "" else None
                })
        return pts

    def execute_cartesian_points(self, robot_id, points, dt=0.2):
        cfg = self.robot_cfg[robot_id]
        seed = self._get_seed(robot_id)
        if seed is None:
            self.get_logger().error("No joint state seed yet.")
            return

        traj = JointTrajectory()
        traj.joint_names = cfg["joints"]

        t = dt
        valid = 0
        for p in points:
            z = cfg["z"] if p["z"] is None else p["z"]
            sol = self.get_ik(p["x"], p["y"], z, seed, robot_id)
            if sol is None:
                continue
            pt = JointTrajectoryPoint()
            pt.positions = sol
            pt.time_from_start = Duration(seconds=t).to_msg()
            traj.points.append(pt)
            seed = sol
            t += dt
            valid += 1

        if valid == 0:
            self.get_logger().error("No valid IK points to execute.")
            return

        if robot_id == 1:
            self.ur1_pub.publish(traj)
        else:
            self.ur2_pub.publish(traj)

        time.sleep(valid * dt + 1.0)

    def move_to_joints(self, joint_goal, robot_id=1, duration=3.0):
        """Send a single joint-space target in controller joint order."""
        cfg = self.robot_cfg[robot_id]
        joint_names = cfg["joints"]

        if not isinstance(joint_goal, (list, tuple)) or len(joint_goal) != len(joint_names):
            self.get_logger().error(
                f"Invalid joint goal for robot {robot_id}: "
                f"expected {len(joint_names)} values, got {len(joint_goal) if joint_goal is not None else 'None'}"
            )
            return

        msg = JointTrajectory()
        msg.joint_names = joint_names

        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in joint_goal]
        pt.time_from_start = Duration(seconds=duration).to_msg()
        msg.points.append(pt)

        if robot_id == 1:
            self.ur1_pub.publish(msg)
        else:
            self.ur2_pub.publish(msg)

        time.sleep(duration + 0.1)

    def run_robot_phase(self, robot_id, mode="manual", csv_path=None):
        print(f"\n--- STARTING PHASE FOR ROBOT {robot_id} ---")
        self.move_to_joints(self.home_pose, robot_id, duration=4.0)

        if mode == "csv":
            if not csv_path or not os.path.exists(csv_path):
                self.get_logger().error(f"CSV not found: {csv_path}")
                return
            pts = self.load_cartesian_points_from_csv(csv_path)
            self.execute_cartesian_points(robot_id, pts, dt=0.2)
        else:
            while rclpy.ok():
                inp = input(f"Robot {robot_id} target (x y [z]) or 'exit': ").strip()
                if inp.lower() == "exit":
                    break
                try:
                    vals = [float(v) for v in inp.split()]
                    if len(vals) < 2:
                        continue
                    x, y = vals[0], vals[1]
                    z = vals[2] if len(vals) > 2 else self.robot_cfg[robot_id]["z"]
                    seed = self._get_seed(robot_id)
                    sol = self.get_ik(x, y, z, seed, robot_id)
                    if sol:
                        self.move_to_joints(sol, robot_id, duration=2.0)
                except Exception as e:
                    self.get_logger().error(f"Input parse error: {e}")

        print(f"Returning Robot {robot_id} to ZERO POSITION...")
        self.move_to_joints(self.zero_pose, robot_id, duration=5.0)
        self.verify_at_zero(robot_id)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", choices=["1", "2", "both"], default="1")
    parser.add_argument("--mode", choices=["manual", "csv"], default="manual")
    parser.add_argument("--csv", default="/mnt/sdc/GitHub/ros2/data/motion_safe.csv")
    args = parser.parse_args()

    rclpy.init()
    node = DualMopControlSystem()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    while node.current_joints_1 is None or node.current_joints_2 is None:
        time.sleep(0.1)

    if args.robot in ("1", "both"):
        node.run_robot_phase(robot_id=1, mode=args.mode, csv_path=args.csv)

    if args.robot in ("2", "both"):
        if args.robot == "both":
            time.sleep(2.0)
        node.run_robot_phase(robot_id=2, mode=args.mode, csv_path=args.csv)

    rclpy.shutdown()
    spin_thread.join()


if __name__ == '__main__':
    main()