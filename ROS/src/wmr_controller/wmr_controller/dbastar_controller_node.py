#!/usr/bin/env python3
import copy
from concurrent.futures import ThreadPoolExecutor

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist, Vector3
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from wmr_interfaces.msg import AABB2DArray
from wmr_controller.aabb_utils import (
    aabb_map_from_messages,
    compare_aabb_maps,
    planner_obstacles,
)
import numpy as np
import sys
import os
import time
import glob
import yaml
venv_pattern = os.path.expanduser('~/wmr-ros/ROS/.venv/lib/python3.*/site-packages')
venv_matches = glob.glob(venv_pattern)
if venv_matches:
    venv_path = venv_matches[0]
    if venv_path not in sys.path:
        sys.path.insert(0, venv_path)

# Add wmr-simulator scripts to path
# When installed, the scripts should be in share/wmr_controller/deps/wmr-simulator/scripts
import ament_index_python.packages
try:
    package_share_directory = ament_index_python.packages.get_package_share_directory('wmr_controller')
    wmr_sim_path = os.path.join(package_share_directory, 'deps/wmr-simulator/scripts')

except Exception:
    wmr_sim_path = None

# If not found in share (e.g. not installed yet or running from source differently), try relative path
if not wmr_sim_path or not os.path.exists(wmr_sim_path):
     # When running from source: src/wmr_controller/wmr_controller/wmr_controller_node.py
     # Target: src/wmr_controller/deps/wmr-simulator/scripts
     wmr_sim_path = os.path.join(os.path.dirname(__file__), '../deps/wmr-simulator/scripts')

if os.path.exists(wmr_sim_path):
    sys.path.insert(0, wmr_sim_path)
    from controller import Controller
    from estimator import DiffDriveEstimator
else:
    print(f"Error: Could not find wmr-simulator scripts at {wmr_sim_path}")

realtime_dbastar_path = None
default_problem_path = None
try:
    package_share_directory = ament_index_python.packages.get_package_share_directory('wmr_controller')
    realtime_dbastar_path = os.path.join(package_share_directory, 'external/realtime-dbastar/baselines/wmr-simulator/scripts')
    default_problem_path = os.path.join(package_share_directory, 'external/realtime-dbastar/baselines/wmr-simulator/problems/benchmark/benchmark.yaml')
except Exception:
    pass

if not realtime_dbastar_path or not os.path.exists(realtime_dbastar_path):
    realtime_dbastar_path = os.path.join(os.path.dirname(__file__), '../external/realtime-dbastar/baselines/wmr-simulator/scripts')
if not default_problem_path or not os.path.exists(default_problem_path):
    default_problem_path = os.path.join(os.path.dirname(__file__), '../external/realtime-dbastar/baselines/wmr-simulator/problems/benchmark/benchmark.yaml')

if not os.path.exists(default_problem_path):
    print(f"Error: Could not find problem file at {default_problem_path}")

if os.path.exists(realtime_dbastar_path):
    sys.path.insert(0, realtime_dbastar_path)
    from benchmark import _deep_merge
    from simulator import _run_smag_once, goal_reached, obstacles_of, dynamic_obstacles_of, vanishing_obstacles_of, displacements_of
else:
    print(f"Error: Could not find scripts at {realtime_dbastar_path}")


class dbastarControllerNode(Node):
    def __init__(self):
        super().__init__('dbastar_controller_node')
        
        # Parameters
        self.declare_parameter('robot_name', 'Pololu09')
        self.declare_parameter('mocap_topic', '/poses')
        self.declare_parameter('cmd_unicycle_topic', '/cmd_unicycle')
        self.declare_parameter('control_dt', 0.1)
        self.declare_parameter('problem', default_problem_path)
        self.declare_parameter('instance', '1.3_0.5_2.3562_empty')
        self.declare_parameter('obstacle_topic', '/obstacles_aabb')
        self.declare_parameter('obstacle_change_tolerance', 0.02)
        self.declare_parameter('displacement_thr', 0.1)

        self.robot_name = str(self.get_parameter('robot_name').value)
        mocap_topic = str(self.get_parameter('mocap_topic').value)
        cmd_unicycle_topic = str(self.get_parameter('cmd_unicycle_topic').value)
        control_dt = float(self.get_parameter('control_dt').value)
        problem_path = str(self.get_parameter('problem').value)
        instance_name = str(self.get_parameter('instance').value)
        obstacle_topic = str(self.get_parameter('obstacle_topic').value)
        self.obstacle_change_tolerance = float(
            self.get_parameter('obstacle_change_tolerance').value
        )
        displacement_thr = float(self.get_parameter('displacement_thr').value)

        if control_dt <= 0.0:
            raise ValueError("ROS parameter 'control_dt' must be greater than zero")
        if not os.path.isfile(problem_path):
            raise FileNotFoundError(f"Problem file does not exist: {problem_path}")
        if self.obstacle_change_tolerance < 0.0:
            raise ValueError("obstacle_change_tolerance must be non-negative")

        with open(problem_path, 'r') as f:
            cfg = yaml.safe_load(f)
        shared = {k: v for k, v in cfg.items() if k != "instances"}
        out = {}
        for inst in cfg.get("instances") or []:
            inst = dict(inst)
            name = str(inst.pop("name"))
            # Environment tweaks written inline next to the goal (obstacles/min/max),
            # or a full `environment:` dict, layered over the shared environment.
            inline_env = dict(inst.pop("environment", {}) or {})
            for k in ("min", "max", "obstacles", "dynamic_obstacles", "vanishing_obstacles",
                    "displacements"):
                if k in inst:
                    inline_env[k] = inst.pop(k)
            prob = _deep_merge(shared, inst)             # inst now carries goal (+ dbastar/etc overrides)
            prob["environment"] = _deep_merge(cfg.get("environment") or {}, inline_env)
            out[name] = prob
        
        if instance_name not in out:
            available = ", ".join(out.keys())
            raise ValueError(
                f"Unknown benchmark instance '{instance_name}'. Available: {available}"
            )
        problem = out[instance_name]
        
        self.trajectory_dt = float(problem["dbastar"]["dt"])
        if self.trajectory_dt <= 0.0:
            raise ValueError(
                "dbastar.dt must be greater than zero"
            )
        self.controller_dt = control_dt

        if displacement_thr <= 0.0:
            raise ValueError(
                "displacement_thr distance must be absolute value"
            )
        self.displacement_thr = displacement_thr
        self.recovering_from_displacement = False
        self.still_count = 0
        self.prev_pose_2d = None
        self.pose_predicted = None # predicted next pose based on current pose and control command
        self.disturbance_vec = np.array([0.0, 0.0])

        # if not np.isclose(self.controller_dt, self.trajectory_dt):
        #     self.get_logger().warn(
        #         f"control_dt={control_dt:g} Hz overrides dbastar.dt="
        #         f"{self.trajectory_dt:g} s with dt={self.controller_dt:g} s"
        #     )
        #     problem["dbastar"]["dt"] = self.dt
        self.thr = float(problem["dbastar"]["goal_threshold"])
        self.steps_max = int(problem["sim_time"] / self.controller_dt)
        static = obstacles_of(problem)
        dyn, van = dynamic_obstacles_of(problem), vanishing_obstacles_of(problem)
        self.shoves = list(displacements_of(problem))
        self.known = list(static) + [v["box"] for v in van]
        self.hidden, self.present = list(dyn), list(van)
        # self.obs_p = self.ctrl.pack_obstacles(self.known) if self.ctrl.M else None
        self.t_compute, self.reached, self.reveals, self.vanishes = 0.0, False, [], []
        self.shifts, self.jumps = [], []
        

        self.problem = problem
        self.start = list(self.problem["start"])
        self.goal = list(self.problem["goal"])

        self.i = 0

        self.traj = []
        self.log_wheel_cmd = []

        # TODO remove planning here and plan as soon as the first mocap pose arrives?
        self.get_logger().info("Running planner once")
        result = _run_smag_once(
            self.problem,
            self.start,
            self.goal
        )

        if not result["reached"]:
            raise RuntimeError(
                "DBA* failed to generate a valid trajectory"
            )

        self.t_compute = result["search_time_s"] or 0.0
        self.setpoints = np.asarray(
            result["plan"],
            dtype=float
        )

        if self.setpoints.size == 0:
            raise RuntimeError(
                "DBA* returned an empty trajectory"
            )

        if self.setpoints.ndim == 1:
            self.setpoints = self.setpoints.reshape(1, -1)

        if self.setpoints.shape[1] < 6:
            raise RuntimeError(
                f"Expected trajectory columns "
                f"[x, y, theta, vx, vy, w], "
                f"but got shape {self.setpoints.shape}"
            )

        if not np.all(np.isfinite(self.setpoints)):
            raise RuntimeError(
                "Trajectory contains NaN or infinity"
            )

        self.get_logger().info(
            f"Loaded {len(self.setpoints)} setpoints; "
            f"trajectory_dt={self.trajectory_dt:g} s"
        )

        # self.setpoints = _smag_plan()
        print(f"Loaded {len(self.setpoints)} setpoints from planner")

        # Robot parameters for pololu robots
        robot_cfg = self.problem["robotcfg"]
        self.robot_param = {
            'wheel_radius':
                float(robot_cfg["wheel_radius"]),
            'base_diameter':
                float(robot_cfg["base_diameter"]),
        }

        #init controller from wmr-simulator
        dbastar_cfg = self.problem["dbastar"]
        self.kx = float(dbastar_cfg["kx"])
        self.ky = float(dbastar_cfg["ky"])
        self.kth = float(dbastar_cfg["kth"])
        self.get_logger().info(f'Controller gains: kx={self.kx}, ky={self.ky}, kth={self.kth}')
        # controller_gains = [self.kx, self.ky, self.kth, 0.01, 0.01, 0.01, 0.01]  # [kx, ky, kth, kpr, kpl, kir, kil] 
        controller_gains = [4.0, 8.0, 3.0, 0.0, 0.0, 0.0, 0.0]  # [kx, ky, kth, kpr, kpl, kir, kil]
        self.wheel_vel_upper_limits = float(robot_cfg.get("max_vel_leftwheel", 30.0))  # load wheel speed limits from benchmark.yaml
        self.wheel_vel_lower_limits = float(robot_cfg.get("min_vel_leftwheel", -30.0))  # load wheel speed limits from benchmark.yaml
        cmd_limits = (self.wheel_vel_lower_limits, self.wheel_vel_upper_limits)  # Wheel speed limits (rad/s) for pololu robots (seems to be lower than actual wheelspeed limits)
        self.controller = Controller(self.robot_param, gains=controller_gains, 
                                    cmd_limits=cmd_limits, dt=self.controller_dt)
        
        self.control_step = 0
        self.reference_index = 0
        #init estimator from wmr-simulator
        estimator_cfg = {
            "type": "dr",  # Dead reckoning for now (or "kf" for Kalman filter)
            "wheel_radius": self.robot_param['wheel_radius'],
            "base_diameter": self.robot_param['base_diameter'],
            "start": [0.0, 0.0, 0.0],  # Will be updated from first mocap pose
            "noise_pos": 0.001,
            "noise_angle": 0.01,
            "enc_angle_noise": 0.0,
            "proc_pos_std": 0.01,
            "proc_theta_std": 0.01
        }
        self.estimator = DiffDriveEstimator(estimator_cfg, self.controller_dt)

        # AABB snapshots are reusable ROS inputs.  Planning stays single-threaded
        # because every SMAG invocation writes the same traj/*.csv files.
        self.observed_obstacles = {
            f"yaml_{index}": tuple(box)
            for index, box in enumerate(obstacles_of(self.problem))
        }
        self.replanning = False
        self.replan_requested = False
        self.replan_pending_until_pose = False
        self.replan_future = None
        self.planning_failed = False
        self.planner_executor = ThreadPoolExecutor(max_workers=1)
        

        # QoS for mocap (BEST_EFFORT like controller_interface)
        mocap_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # QoS for control commands (BEST_EFFORT, depth=1)
        cmd_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to mocap poses
        self.mocap_sub = self.create_subscription(
            NamedPoseArray,
            mocap_topic,
            self.poses_callback,
            mocap_qos
        )

        self.obstacle_sub = self.create_subscription(
            AABB2DArray,
            obstacle_topic,
            self.obstacles_callback,
            mocap_qos
        )
        
        #publish velocity commands (v, w) on /cmd_unicycle topic 
        self.cmd_pub = self.create_publisher(Vector3, cmd_unicycle_topic, cmd_qos)
        
        # Timer for control loop
        self.timer = self.create_timer(self.controller_dt, self.control_loop)
        
        # State
        self.latest_pose = None
        self.initialized = False
        self.wheel_speeds = (0.0, 0.0)  # Estimated wheel speeds (ur, ul)
        
        self.get_logger().info(
            f'DBA* controller started for robot: {self.robot_name} @ {1.0/self.controller_dt:g} Hz; '
            f'mocap={mocap_topic}, obstacles={obstacle_topic}, '
            f'cmd={cmd_unicycle_topic}, instance={instance_name}'
        )
    

    def stop_robot(self):
        self.cmd_pub.publish(Vector3())
        self.timer.cancel()
        self.stopped = True
    
    def poses_callback(self, msg: NamedPoseArray):
        """Callback for motion capture poses"""

        #idea: mocap publishes at its own rate. The last pose that was incoming is stored for usage at the defined control loop execution rate.
        for pose in msg.poses:
            if pose.name == self.robot_name:
                self.latest_pose = pose.pose

                #init pose. Take zero if mocap doesn't have a pose yet.
                if not self.initialized:
                    x0 = pose.pose.position.x
                    y0 = pose.pose.position.y
                    # extract yaw from quaternion
                    q = pose.pose.orientation
                    th0 = self.estimator._wrap_to_pi(np.arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2)))
                    self.estimator._init_state(x0, y0, th0)
                    self.initialized = True
                    self.get_logger().info(f'Initialized at x={x0:.3f}, y={y0:.3f}, theta={th0:.3f}')
                    if self.replan_pending_until_pose:
                        self.replan_pending_until_pose = False
                        self.get_logger().info(
                            "Robot pose is now available; starting the pending replan"
                        )
                        self.request_replan()
                break


    def current_robot_pose(self):
        if self.latest_pose is None:
            return None

        pose = self.latest_pose
        q = pose.orientation
        theta = self.estimator._wrap_to_pi(
            np.arctan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y ** 2 + q.z ** 2),
            )
        )
        return [float(pose.position.x), float(pose.position.y), float(theta)]

    def obstacles_callback(self, msg: AABB2DArray):
        """Consume a complete, current AABB snapshot."""
        try:
            new_obstacles = aabb_map_from_messages(msg.boxes)
            changes = compare_aabb_maps(
                self.observed_obstacles,
                new_obstacles,
                self.obstacle_change_tolerance,
            )
        except ValueError as error:
            self.get_logger().warn(f"Ignoring invalid AABB snapshot: {error}")
            return

        if not changes.changed:
            return

        self.observed_obstacles = new_obstacles
        self.get_logger().info(
            f"AABB map changed: added={list(changes.added)}, "
            f"removed={list(changes.removed)}, moved={list(changes.moved)}"
        )
        if self.initialized:
            self.request_replan()
        else:
            self.replan_pending_until_pose = True
            self.get_logger().info(
                "AABB change stored; waiting for the first robot pose before replanning"
            )

    def validate_plan_result(self, result):
        if not result.get("reached", False):
            raise RuntimeError("DBA* failed to generate a valid trajectory")

        setpoints = np.asarray(result.get("plan", []), dtype=float)
        if setpoints.size == 0:
            raise RuntimeError("DBA* returned an empty trajectory")
        if setpoints.ndim == 1:
            setpoints = setpoints.reshape(1, -1)
        if setpoints.ndim != 2 or setpoints.shape[1] < 6:
            raise RuntimeError(
                "Expected trajectory columns [x, y, theta, vx, vy, w], "
                f"got shape {setpoints.shape}"
            )
        if not np.all(np.isfinite(setpoints)):
            raise RuntimeError("Trajectory contains NaN or infinity")
        return setpoints

    def make_replan_problem(self, start):
        problem = copy.deepcopy(self.problem)
        problem["start"] = list(start)
        problem["environment"]["obstacles"] = planner_obstacles(
            self.observed_obstacles
        )
        problem["environment"]["dynamic_obstacles"] = []
        problem["environment"]["vanishing_obstacles"] = []
        return problem

    def run_replan(self, problem, start, goal):
        result = _run_smag_once(problem, start, goal)
        return {
            "problem": problem,
            "start": list(start),
            "setpoints": self.validate_plan_result(result),
            "search_time_s": result.get("search_time_s") or 0.0,
        }

    def request_replan(self):
        start = self.current_robot_pose()
        if start is None:
            self.get_logger().warn("Cannot replan without a robot pose")
            return
        if self.replanning:
            self.replan_requested = True
            self.get_logger().info(
                "A replan is already running; queued another replan"
            )
            return

        problem = self.make_replan_problem(start)
        self.replanning = True
        self.replan_requested = False
        self.planning_failed = False
        self.controller.ir = 0.0
        self.controller.il = 0.0
        self.wheel_speeds = (0.0, 0.0)
        self.cmd_pub.publish(Vector3())
        self.pose_predicted = None
        self.get_logger().info(
            f"Starting DBA* replan from {start} with "
            f"{len(problem['environment']['obstacles'])} AABBs"
        )
        self.replan_future = self.planner_executor.submit(
            self.run_replan, problem, start, list(self.goal)
        )

    def update_replanning(self):
        if not self.replanning:
            return False

        self.cmd_pub.publish(Vector3())
        if self.replan_future is None or not self.replan_future.done():
            return True

        try:
            replanned = self.replan_future.result()
        except Exception as error:
            self.get_logger().error(f"DBA* replanning failed: {error}")
            self.replanning = False
            self.replan_future = None
            self.planning_failed = True
            return True

        self.problem = replanned["problem"]
        self.start = replanned["start"]
        self.setpoints = replanned["setpoints"]
        self.t_compute += replanned["search_time_s"]
        self.control_step = 0
        self.reference_index = 0
        self.controller.ir = 0.0
        self.controller.il = 0.0
        self.wheel_speeds = (0.0, 0.0)
        self.replanning = False
        self.replan_future = None
        self.planning_failed = False

        self.get_logger().info(
            f"Replanning succeeded; loaded {len(self.setpoints)} setpoints"
        )
        if self.replan_requested:
            self.replan_requested = False
            self.request_replan()
        return True
    
    def control_loop(self):
        """copied from simulator.py and adapted to run inside a ros2 node """
        if self.planning_failed:
            self.cmd_pub.publish(Vector3())
            return

        if self.update_replanning():
            return

        if not self.initialized or self.latest_pose is None:
            self.get_logger().warn('Waiting for first mocap pose...', throttle_duration_sec=2.0)
            self.cmd_pub.publish(Vector3())
            return
        
        #get true pose from mocap (replaces robot.get_pose() in simulator)
        x_true = self.latest_pose.position.x
        y_true = self.latest_pose.position.y
        q = self.latest_pose.orientation
        theta_true = self.estimator._wrap_to_pi(np.arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2)))
        pose_true = (x_true, y_true, theta_true)
        self.traj.append(pose_true)

        self.reached = goal_reached(pose_true, self.goal, self.thr, float(self.problem["dbastar"]["goal_error_tolerance"]))
        # stop if goal reached
        if self.reached:
            self.get_logger().info("Goal reached")
            self.stop_robot()
            return

        if self.control_step >= self.steps_max:
            self.get_logger().warn(
                "Maximum control duration reached"
            )
            self.stop_robot()
            return

        pose_true_2d = np.array(pose_true[0:2])
        if self.recovering_from_displacement:
            current_vel = np.linalg.norm(pose_true_2d - self.prev_pose_2d) / self.controller_dt
            if current_vel < 0.01:
                self.still_count +=1
            else: 
                self.still_count = 0

            if self.still_count >= 3:
                self.get_logger().info('Robot settled, replanning')
                self.recovering_from_displacement = False
                self.request_replan()
                return
            self.prev_pose_2d = pose_true_2d
            return

        # trigger replan if shoved aka the predicted next state is too far away from the current state
        if self.pose_predicted is not None:
            step_error = pose_true_2d - self.pose_predicted
            self.disturbance_vec = (0.5 * self.disturbance_vec) + step_error
            dist = np.linalg.norm(self.disturbance_vec)
            self.get_logger().info(f'Distance to predicted pose: {dist:.3f}')
            if dist >= self.displacement_thr:
                self.get_logger().info(f'Displacement detected. Waiting to settle')
                self.cmd_pub.publish(Vector3())
                self.disturbance_vec = np.array([0.0, 0.0]) #reset error
                self.recovering_from_displacement = True
                self.still_count = 0
                self.prev_pose_2d = pose_true_2d
                return
        
        #get true wheel speeds (in simulator: robot.get_wheel_speeds())
        #TODO: get robot log data eventually to use encoder readings for real wheel speeds
        #for now: wheel speed command is assumed to be true
        # ur_true, ul_true = self.wheel_speeds
        
        #self.estimator.update(ur_true, ul_true, pose_true)
        
        #pose_est = self.estimator.get_est_pose()  # (x_hat, y_hat, theta_hat)
        # ur_hat, ul_hat = self.estimator.get_est_wheel_speeds()
        # wheel_est = (ur_hat, ul_hat)

        #compute control commands using estimated states, use mocap pose as "true" pose instead of estimating it
        # ref_state = self.setpoints[self.i]
        elapsed_time = self.control_step * self.controller_dt
        reference_index = int(
            round(elapsed_time / self.trajectory_dt)
        )
        if reference_index >= len(self.setpoints):
            self.get_logger().warn(
                "Reference trajectory finished before reaching the goal"
            )
            self.stop_robot()
            return
        ref_state = self.setpoints[reference_index]
        self.reference_index = reference_index
        ref_state_full = np.asarray([
            ref_state[0],  # x
            ref_state[1],  # y
            ref_state[2],  # theta
            ref_state[3],  # vx
            ref_state[4],  # vy
            ref_state[5],  # w
            0.0,     # ax
            0.0,     # ay
        ])

        ur_cmd, ul_cmd = self.controller.compute(ref_state_full, pose_true, self.wheel_speeds) # this time it should really be ur, ul :D
        self.control_step += 1
        
        #convert wheel speeds to (v, w) for publishing
        #TODO: is it maybe better to publish r & l for pololu?
        r = self.robot_param['wheel_radius']
        L = self.robot_param['base_diameter']
        v = r * (ur_cmd + ul_cmd) / 2.0
        w = r * (ur_cmd - ul_cmd) / L
        # self.get_logger().info(f'v={v:.3f}, w={w:.3f}')
        # self.get_logger().info(f'x={ref_state[0]:.3f}, y={ref_state[1]:.3f}, theta={ref_state[2]:.3f}')
        # self.get_logger().info(f'ur={ur_cmd:.3f}, ul={ul_cmd:.3f}')

        self.pose_predicted = pose_true_2d + np.array([v * np.cos(pose_true[2]), v * np.sin(pose_true[2])]) * self.controller_dt
        #publish control actions --> controller interface expects (v, w) and sends it to pololu like x box controller inputs
        cmd = Vector3()
        cmd.x = v
        cmd.y = w
        cmd.z = 0.0
        self.cmd_pub.publish(cmd)
        
        # Store commanded wheel speeds as "true" for next iteration
        # (in simulator, robot.step() updates these, i simply use commanded values here)
        self.wheel_speeds = (ur_cmd, ul_cmd)
        self.log_wheel_cmd.append(self.wheel_speeds)


def main(args=None):
    rclpy.init(args=args)
    node = dbastarControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
