"""ROS 2 simulation reaching node using only standard message types."""
import json
import time
import uuid
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Empty, Float64, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from .fusion import FusionConfig, FusionState, GtsamPoseGraph, SimulationReference, Observation
from .kinematics import ArmModel
from .planning import PlanningRequest, solve_request
from .se3 import from_pose, inverse, to_pose
from .teleop import BimanualClutch, chain_reach


def pose_array(pose):
    p, q = pose.position, pose.orientation
    return [p.x, p.y, p.z, q.x, q.y, q.z, q.w]


def stamp_seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


def duration_msg(seconds, message):
    nanoseconds = int(round(seconds*1e9))
    message.sec, message.nanosec = divmod(nanoseconds, 1_000_000_000)


class DeicticControl(Node):
    def __init__(self):
        super().__init__('deictic_control')
        defaults = {
            'urdf': '', 'synthetic_test': False, 'synthetic_reference_backend': False,
            'allow_execution': False, 'allow_legacy_execute': False, 'mock_joint_states': False,
            'allow_teleoperation': False,
            'teleop_human_arm_length': .60, 'teleop_translation_scale': 0.,
            'synthetic_base_from_headset_world': [-1.2, 0., -.9, 0., 0., 0., 1.],
            'tool_offset': [0., -.10, 0.], 'table_top': -.15,
            'registration_timeout': 1., 'joint_timeout': .5, 'plan_timeout': 10.,
            'preview_target_tolerance': .005,
            'max_keyframes': 10000, 'synthetic_registration_interval': .5,
            'max_goal_age': 1., 'max_joint_speed': .35, 'max_joint_acceleration': .7,
            'obstacles_json': '[]',
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.param = lambda key: self.get_parameter(key).value
        tolerance = self.param('preview_target_tolerance')
        if not np.isfinite(tolerance) or tolerance <= 0:
            raise ValueError('preview_target_tolerance must be finite and positive')
        synthetic = self.param('synthetic_test')
        if self.param('synthetic_reference_backend'):
            if not synthetic:
                raise ValueError('synthetic_reference_backend requires synthetic_test')
            backend = SimulationReference()
        else:
            try:
                backend = GtsamPoseGraph(max_keyframes=self.param('max_keyframes'))
            except ImportError:
                backend = None
                self.get_logger().error('GTSAM unavailable; frame fusion and goal commits disabled')
        self.fusion = FusionState(backend, FusionConfig(allow_synthetic=synthetic,
                                  max_registration_age=self.param('registration_timeout')))
        urdf = self.param('urdf')
        if not urdf:
            raise ValueError('Set urdf parameter to the pinned models/K1/K1_22dof.urdf')
        self.arm = ArmModel(urdf, tool_offset=self.param('tool_offset'))
        self.left_arm = ArmModel(urdf, tip='left_elbow_yaw_link', tool_offset=[0., .10, 0.])
        self.obstacles = json.loads(self.param('obstacles_json'))
        if any(len(b) != 6 or not np.all(np.isfinite(b)) or np.any(np.array(b[:3]) >= b[3:]) for b in self.obstacles):
            raise ValueError('Obstacle boxes require finite xmin,ymin,zmin,xmax,ymax,zmax')
        self.both_arm_names = self.left_arm.names+self.arm.names
        human_length, scale = self.param('teleop_human_arm_length'), self.param('teleop_translation_scale')
        if not np.isfinite(human_length) or human_length <= 0 or not np.isfinite(scale) or scale < 0:
            raise ValueError('Invalid teleop human arm length or translation scale')
        scale = scale or min(chain_reach(self.left_arm), chain_reach(self.arm))/human_length
        self.teleop = BimanualClutch((self.left_arm, self.arm), speed=self.param('max_joint_speed'),
                                    acceleration=self.param('max_joint_acceleration'),
                                    table_top=self.param('table_top'), obstacles=self.obstacles,
                                    translation_scale=scale)
        self.teleop_joints = self.teleop_joint_time = None
        self.teleop_session_id, self.teleop_sequence = str(uuid.uuid4()), 0
        self.teleop_output_initialized = False
        self.teleop_relay_status = None
        self.teleop_relay_received = self.teleop_acquired = float('-inf')
        self.teleop_acquire_sequence = -1
        self.joints = self.joint_time = self.plan = self.plan_time = None
        self.plan_transform = self.plan_base_target = self.plan_epoch = None
        self.plan_left_joints = None
        self.plan_goal_stamp = None
        self.planning_generation = 0
        self.pending_request = self.planner_request = self.planner_future = None
        self.planner_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='k1-planner')
        self.active_goal = self.active_started = self.active_duration = None
        self.operation = 'awaiting_feedback'
        self.tracking_inhibited = False
        self.last_tracking_event_stamp = float('-inf')
        self.headset = np.eye(4)
        self.command_pub = self.create_publisher(JointTrajectory, '/k1/arm_controller/joint_trajectory', 10)
        self.teleop_pub = self.create_publisher(String, '/k1/teleop/command', 10)
        self.preview_pub = self.create_publisher(JointTrajectory, '/deictic/preview', 10)
        self.status_pub = self.create_publisher(String, '/deictic/status', 10)
        self.time_sync_pub = self.create_publisher(Float64MultiArray, '/deictic/time_sync/reply', 10)
        self.transform_pub = self.create_publisher(TransformStamped, '/deictic/base_from_headset_world', 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(PoseStamped, '/deictic/headset_pose', self.on_headset, 10)
        self.create_subscription(String, '/deictic/registration', self.on_registration, 10)
        self.create_subscription(String, '/deictic/registration_failure', self.on_failure, 10)
        self.create_subscription(PoseStamped, '/deictic/goal', self.on_goal, 10)
        self.create_subscription(Empty, '/deictic/execute', self.on_execute, 10)
        self.create_subscription(String, '/deictic/execute_request', self.on_execute_request, 10)
        self.create_subscription(Empty, '/deictic/cancel', self.on_cancel, 10)
        self.create_subscription(Float64, '/deictic/time_sync/request', self.on_time_sync, 10)
        self.create_subscription(String, '/deictic/teleop/input', self.on_teleop, 1)
        self.create_subscription(String, '/k1/teleop/relay_status', self.on_teleop_relay_status, 1)
        self.create_timer(.1, self.publish_status)
        self.create_timer(.02, self.poll_planner)
        self.create_timer(.05, self.teleop_tick)
        if synthetic:
            self.synthetic_transform = from_pose(self.param('synthetic_base_from_headset_world'))
            interval = self.param('synthetic_registration_interval')
            if interval <= 0 or interval >= self.param('registration_timeout'):
                raise ValueError('Synthetic registration interval must be positive and below the freshness timeout')
            self.create_timer(interval, self.synthetic_observation)
            self.get_logger().warning('SYNTHETIC TEST: known simulated alignment; no physical camera registration')
        self.mock_active = self.param('mock_joint_states')
        if self.mock_active:
            if not synthetic:
                raise ValueError('mock_joint_states requires synthetic_test')
            self.mock_q, self.mock_trajectory, self.mock_start = np.array([0., 1., 0., 0.]), None, None
            self.mock_pub = self.create_publisher(JointState, '/joint_states', 10)
            self.ee_pub = self.create_publisher(PoseStamped, '/k1/end_effector_pose', 10)
            self.create_subscription(JointTrajectory, '/k1/arm_controller/joint_trajectory', self.on_mock_command, 10)
            self.create_timer(1/60, self.mock_tick)
        self.get_logger().info('K1 position-only reaching ready; execution=' + str(self.param('allow_execution')))

    def now(self):
        # Wall clock matches Unity UTC stamps. Do not enable use_sim_time here.
        return time.time()

    def monotonic(self):
        return time.monotonic()

    def teleop_guard(self):
        if not self.param('allow_teleoperation') or not self.param('allow_execution'):
            return 'teleop_execution_disabled'
        if self.param('mock_joint_states'):
            return 'teleop_mock_bimanual_unsupported'
        if self.tracking_inhibited:
            return 'teleop_headset_tracking_unavailable'
        if self.teleop_joint_time is None or self.now()-self.teleop_joint_time > self.param('joint_timeout'):
            return 'teleop_both_joint_feedback_stale'
        if self.teleop_pub.get_subscription_count() < 1:
            return 'teleop_no_simulation_relay'
        relay = self.teleop_relay_status
        if (relay is None or self.now()-relay['stamp'] > .30 or relay['stamp']-self.now() > .05
                or self.monotonic()-self.teleop_relay_received > .30):
            return 'teleop_relay_ack_stale'
        if not relay['ready']:
            return 'teleop_relay_not_ready: '+relay['reason']
        if self.teleop.active and self.monotonic()-self.teleop_acquired >= .20:
            if (not relay['active'] or relay['owner_session_id'] != self.teleop_session_id
                    or relay['last_sequence'] is None or relay['last_sequence'] < self.teleop_acquire_sequence):
                return 'teleop_relay_lease_lost: '+relay['reason']
        elif not self.teleop.active and relay['active'] and relay['owner_session_id'] != self.teleop_session_id:
            return 'teleop_relay_owned_by_other_controller'
        return None

    def on_teleop_relay_status(self, message):
        try:
            if len(message.data) > 4096:
                raise ValueError('oversized ack')
            data = json.loads(message.data)
            if (type(data.get('schema_version')) is not int or data['schema_version'] != 1
                    or type(data.get('ready')) is not bool or type(data.get('active')) is not bool
                    or type(data.get('stamp')) not in (int, float) or not np.isfinite(data['stamp'])
                    or not isinstance(data.get('reason'), str) or len(data['reason']) > 256):
                raise ValueError('invalid ack')
            owner, seq = data.get('owner_session_id'), data.get('last_sequence')
            if owner is not None and (not isinstance(owner, str) or str(uuid.UUID(owner)) != owner):
                raise ValueError('invalid ack owner')
            if seq is not None and (type(seq) is not int or seq < 0):
                raise ValueError('invalid ack sequence')
            if data['active'] and (owner is None or seq is None):
                raise ValueError('missing active ack owner')
            if self.now()-data['stamp'] > .30 or data['stamp']-self.now() > .05 or data['stamp'] <= 0:
                return
            if self.teleop_relay_status is not None and data['stamp'] <= self.teleop_relay_status['stamp']:
                return  # Duplicate acknowledgements cannot renew the receipt lease.
            self.teleop_relay_status, self.teleop_relay_received = data, self.monotonic()
        except (ValueError, TypeError, AttributeError):
            self.teleop_relay_status = None

    def send_teleop_command(self, active):
        if self.teleop_joints is None:
            return
        positions = self.teleop.command if active else self.teleop_joints
        message = dict(schema_version=1, session_id=self.teleop_session_id,
                       sequence=self.teleop_sequence, stamp=self.now(), active=active,
                       names=self.both_arm_names, positions=positions.reshape(-1).tolist())
        self.teleop_sequence += 1
        self.teleop_pub.publish(String(data=json.dumps(message, allow_nan=False)))
        self.teleop_output_initialized = True

    def stop_teleop(self, reason):
        was_active = self.teleop.active
        self.teleop.stop(reason)
        if was_active:
            self.send_teleop_command(False)
            self.operation = 'teleop_hold: '+reason

    def on_teleop(self, message):
        was_active = self.teleop.active
        result = self.teleop.ingest(message.data, self.now(), self.monotonic(),
                                   self.teleop_joints, self.teleop_guard())
        if was_active and not self.teleop.active:
            self.send_teleop_command(False)
            self.operation = 'teleop_hold: '+self.teleop.reason
        if result == 'began':
            self.drop_plan()
            self.active_goal = self.active_started = self.active_duration = None
            # Same-topic ordered false/true also establishes a fresh relay lease
            # if the relay started after this controller's initial handshake.
            self.send_teleop_command(False)
            # Claim ownership immediately at the measured pose. This prevents a
            # legacy trajectory from starting during the wait for the next tick.
            self.teleop_acquired, self.teleop_acquire_sequence = self.monotonic(), self.teleop_sequence
            self.send_teleop_command(True)
            self.operation = 'teleop_bimanual_pose_ik'
        self.publish_status()

    def teleop_tick(self):
        was_active = self.teleop.active
        started = self.monotonic()
        command = self.teleop.tick(self.now(), started, self.teleop_joints, self.teleop_guard())
        if command is not None:
            # IK may take long enough for its copied input/feedback to expire.
            # Never hide that age by giving the solved target a new UTC stamp.
            now, finished = self.now(), self.monotonic()
            guard = self.teleop_guard()
            if not 0 <= finished-started <= .15:
                guard = 'teleop_servo_deadline'
            elif (finished-self.teleop.received > self.teleop.timeout
                    or now-self.teleop.input_stamp > self.teleop.timeout
                    or self.teleop.input_stamp-now > .05):
                guard = 'teleop_input_timeout'
            if guard is not None:
                self.stop_teleop(guard)
                return
            self.send_teleop_command(True)
            self.operation = 'teleop_'+self.teleop.state+': '+self.teleop.reason
        elif was_active and not self.teleop.active:
            self.send_teleop_command(False)
            self.operation = 'teleop_hold: '+self.teleop.reason

    def on_time_sync(self, message):
        received = self.now()
        if np.isfinite(message.data):
            self.time_sync_pub.publish(Float64MultiArray(data=[message.data, received, self.now()]))

    def drop_plan(self, reason=None, publish_clear=True):
        had_plan = self.plan is not None
        had_pending = (self.pending_request is not None or
                       (self.planner_request is not None and
                        self.planner_request.generation == self.planning_generation))
        self.planning_generation += 1
        if self.planner_request is not None:
            self.planner_request.cancelled.set()
        if self.pending_request is not None:
            self.pending_request.cancelled.set()
        self.pending_request = None
        self.plan = self.plan_time = self.plan_goal_stamp = None
        self.plan_transform = self.plan_base_target = self.plan_epoch = None
        self.plan_left_joints = None
        if reason is not None and (had_plan or had_pending):
            self.operation = 'preview_invalidated: '+reason
        if publish_clear and (had_plan or had_pending):
            self.preview_pub.publish(JointTrajectory())

    def check_plan_alignment(self):
        if self.plan_left_joints is not None:
            if (self.teleop_joint_time is None or self.now()-self.teleop_joint_time > self.param('joint_timeout')
                    or np.max(np.abs(self.teleop_joints[0]-self.plan_left_joints)) > .03):
                raise ValueError('held_left_arm_changed_recommit_goal')
        tolerance = self.param('preview_target_tolerance')
        # ROS parameters can also change at runtime; NaN/inf must not disable
        # the comparison after startup validation has completed.
        if not np.isfinite(tolerance) or tolerance <= 0:
            raise ValueError('invalid_preview_target_tolerance')
        if self.plan_epoch != self.fusion.epoch:
            raise ValueError('alignment_epoch_changed_recommit_goal')
        if self.plan_transform is None or self.plan_base_target is None or self.fusion.transform is None:
            raise ValueError('no_alignment_bound_preview')
        # Always compare with the original preview binding: small updates cannot
        # accumulate without limit by repeatedly rebasing the accepted preview.
        point = np.append(self.plan_base_target, 1.)
        moved = (self.fusion.transform @ inverse(self.plan_transform) @ point)[:3]
        distance = np.linalg.norm(moved-self.plan_base_target)
        if not np.isfinite(distance) or distance > tolerance:
            raise ValueError('alignment_moved_target_recommit_goal')

    def maintain_preview_alignment(self):
        if self.plan is not None:
            try:
                self.check_plan_alignment()
            except ValueError as error:
                self.drop_plan()
                self.operation = 'preview_invalidated: '+str(error)

    def on_headset(self, message):
        if message.header.frame_id != 'headset_world':
            return
        try:
            self.headset = from_pose(pose_array(message.pose))
        except ValueError:
            pass

    def on_joints(self, message):
        now = self.now()
        stamp = stamp_seconds(message.header.stamp) or now
        if (len(message.name) != len(message.position) or len(set(message.name)) != len(message.name)
                or now-stamp > self.param('joint_timeout') or stamp-now > .05):
            return
        lookup = dict(zip(message.name, message.position))
        if (stamp_seconds(message.header.stamp) > 0 and all(n in lookup for n in self.both_arm_names)
                and (self.teleop_joint_time is None or stamp > self.teleop_joint_time)):
            both = np.array([lookup[n] for n in self.both_arm_names]).reshape(2, 4)
            lower, upper = np.array([self.left_arm.lower, self.arm.lower]), np.array([self.left_arm.upper, self.arm.upper])
            if np.all(np.isfinite(both)) and np.all(both >= lower-.03) and np.all(both <= upper+.03):
                self.teleop_joints, self.teleop_joint_time = both, stamp
                if self.param('allow_teleoperation') and not self.teleop_output_initialized:
                    self.send_teleop_command(False)
        if any(n not in lookup for n in self.arm.names):
            return
        q = np.array([lookup[n] for n in self.arm.names])
        if not np.all(np.isfinite(q)) or np.any(q < self.arm.lower-.03) or np.any(q > self.arm.upper+.03):
            return
        self.joints, self.joint_time = q, now

    def on_registration(self, message):
        if self.tracking_inhibited:
            self.fusion.invalidate('headset_tracking_unavailable')
            return
        try:
            observation = Observation.parse(message.data)
            if not self.fusion.ingest(observation, self.now()):
                self.drop_plan(self.fusion.reason)
            else:
                self.maintain_preview_alignment()
        except (KeyError, TypeError, ValueError) as error:
            self.fusion.invalidate('invalid_registration: '+str(error))
            self.drop_plan(self.fusion.reason)

    def on_failure(self, message):
        try:
            data = json.loads(message.data)
            reason = str(data['reason'])
            if data.get('source') == 'quest_tracking':
                stamp = float(data['stamp'])
                if not np.isfinite(stamp) or stamp <= self.last_tracking_event_stamp:
                    return
                if self.now()-stamp > 2. or stamp-self.now() > .25:
                    return
                events = {'Head tracking lost': 'lost', 'Head tracking restored': 'restored',
                          'Application paused': 'paused', 'Application resumed': 'resumed',
                          'Headset recentered': 'recentered', 'Headset tracking origin changed': 'origin_changed'}
                event = data.get('event_type', data.get('event', events.get(reason, 'unknown')))
                self.last_tracking_event_stamp = stamp
                self.tracking_inhibited = event not in ('restored', 'resumed', 'recentered', 'origin_changed')
                self.on_cancel(None)
                self.fusion.reset('headset_epoch_reset: '+reason, minimum_stamp=stamp)
                self.operation = 'awaiting_fresh_headset_registration'
                return
        except (KeyError, ValueError, TypeError):
            reason = 'registration_failed'
        self.fusion.invalidate(reason)
        self.drop_plan(reason)

    def can_plan(self):
        if self.teleop.active:
            raise ValueError('teleoperation_active_release_clutch')
        if not self.fusion.snapshot(self.now())['can_commit']:
            raise ValueError(self.fusion.reason)
        if self.joint_time is None or self.now()-self.joint_time > self.param('joint_timeout'):
            raise ValueError('joint_feedback_stale')
        if (((self.param('allow_teleoperation') and not self.param('mock_joint_states'))
                or self.teleop_joints is not None)
                and (self.teleop_joint_time is None or self.now()-self.teleop_joint_time > self.param('joint_timeout'))):
            raise ValueError('held_left_arm_feedback_stale')

    def on_goal(self, message):
        self.drop_plan()
        try:
            plan_timeout = self.validated_plan_timeout()
            self.can_plan()
            if self.active_goal is not None:
                raise ValueError('robot_busy_cancel_before_replanning')
            stamp = stamp_seconds(message.header.stamp)
            if (message.header.stamp.nanosec >= 1_000_000_000 or not stamp
                    or self.now()-stamp > self.param('max_goal_age') or stamp-self.now() > .05):
                raise ValueError('goal_timestamp_out_of_range')
            if message.header.frame_id != 'base_link':
                raise ValueError('goal_frame_must_be_base_link')
            target = from_pose(pose_array(message.pose))[:3, 3]
            self.pending_request = PlanningRequest.capture(
                self.planning_generation, (message.header.stamp.sec, message.header.stamp.nanosec),
                self.now(), time.monotonic()+plan_timeout,
                self.fusion.epoch, self.fusion.transform, target, self.joints,
                self.arm, self.param('max_joint_speed'), self.param('max_joint_acceleration'),
                self.param('table_top'), self.obstacles,
                self.left_arm if self.teleop_joints is not None else None,
                self.teleop_joints[0] if self.teleop_joints is not None else None)
            self.operation = 'planning_position_only'
            self.start_pending_planner()
        except (ValueError, np.linalg.LinAlgError) as error:
            self.drop_plan()
            self.operation = 'goal_rejected: '+str(error)
        self.publish_status()

    def validated_plan_timeout(self):
        timeout = float(self.param('plan_timeout'))
        if not np.isfinite(timeout) or timeout <= 0:
            raise ValueError('invalid_plan_timeout')
        return timeout

    def start_pending_planner(self):
        # A running job and one latest request are the entire queue. Never
        # submit another future until the previous worker has completed.
        if self.planner_future is None and self.pending_request is not None:
            self.planner_request, self.pending_request = self.pending_request, None
            self.planner_future = self.planner_pool.submit(solve_request, self.planner_request)

    def poll_planner(self):
        if self.planner_future is None or not self.planner_future.done():
            return
        future, request = self.planner_future, self.planner_request
        self.planner_future = self.planner_request = None
        if request.generation == self.planning_generation and not request.cancelled.is_set():
            try:
                if time.monotonic() > request.deadline:
                    raise ValueError('planning_request_expired')
                plan = future.result()
                self.can_plan()
                if self.active_goal is not None:
                    raise ValueError('robot_busy_cancel_before_replanning')
                if np.max(np.abs(self.joints-request.joints)) > .03:
                    raise ValueError('robot_moved_during_planning')
                self.plan = plan
                self.plan_time = self.now()
                self.plan_transform = request.transform.copy()
                self.plan_base_target = request.target.copy()
                self.plan_epoch = request.epoch
                self.plan_left_joints = request.held_joints.copy() if request.held_joints is not None else None
                self.plan_goal_stamp = request.goal_stamp
                self.check_plan_alignment()
                preview = self.plan_message(plan)
                preview.header.stamp.sec, preview.header.stamp.nanosec = request.goal_stamp
                self.preview_pub.publish(preview)
                self.operation = 'preview_ready_position_only'
            except Exception as error:
                self.drop_plan()
                self.operation = 'goal_rejected: '+str(error)
            self.publish_status()
        self.start_pending_planner()

    def destroy_node(self):
        self.stop_teleop('controller_shutdown')
        self.drop_plan(publish_clear=False)
        # IK iterations and path samples check the event cooperatively. Join
        # before destroying ROS resources; no worker callback publishes results.
        self.planner_pool.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()

    def plan_message(self, plan):
        message = JointTrajectory()
        message.header.frame_id = 'base_link'
        message.joint_names = plan.names
        for t, q, v, a in zip(plan.times, plan.positions, plan.velocities, plan.accelerations):
            point = JointTrajectoryPoint()
            point.positions, point.velocities, point.accelerations = q.tolist(), v.tolist(), a.tolist()
            duration_msg(t, point.time_from_start)
            message.points.append(point)
        return message

    def on_execute(self, _):
        if not self.param('allow_legacy_execute'):
            self.operation = 'execute_rejected: legacy_empty_execute_disabled'
            self.publish_status()
            return
        self.execute_current_plan()

    def on_execute_request(self, message):
        try:
            data = json.loads(message.data)
            sec, nanosec = data['goal_stamp_sec'], data['goal_stamp_nanosec']
            if (data.get('schema_version') != 1 or type(sec) is not int or type(nanosec) is not int
                    or not 0 <= sec <= 2_147_483_647 or not 0 <= nanosec < 1_000_000_000):
                raise ValueError('invalid_preview_token')
            if self.plan is None or (sec, nanosec) != self.plan_goal_stamp:
                raise ValueError('preview_token_mismatch')
        except (ValueError, KeyError, TypeError):
            # A delayed execute for A must neither execute nor erase a newer B.
            self.operation = 'execute_rejected: preview_token_mismatch'
            self.publish_status()
            return
        self.execute_current_plan()

    def execute_current_plan(self):
        try:
            self.can_plan()
            if not self.param('allow_execution'):
                raise ValueError('execution_disabled')
            if self.plan is None or self.now()-self.plan_time > self.validated_plan_timeout():
                raise ValueError('no_fresh_preview')
            self.check_plan_alignment()
            if np.max(np.abs(self.joints-self.plan.positions[0])) > .03:
                raise ValueError('robot_moved_recommit_goal')
            if self.command_pub.get_subscription_count() < 1:
                raise ValueError('no_simulation_command_subscriber')
            self.command_pub.publish(self.plan_message(self.plan))
            self.active_goal = self.plan.positions[-1].copy()
            self.active_started, self.active_duration = self.now(), self.plan.times[-1]
            self.operation = 'trajectory_sent_simulation'
            self.drop_plan()
        except ValueError as error:
            self.operation = 'execute_rejected: '+str(error)
            self.drop_plan()
        self.publish_status()

    def on_cancel(self, _):
        was_teleop = self.teleop.active
        self.stop_teleop('cancel_requested')
        self.drop_plan()
        self.active_goal = self.active_started = self.active_duration = None
        if self.joints is not None and not was_teleop:
            message, point = JointTrajectory(), JointTrajectoryPoint()
            message.joint_names = self.arm.names
            point.positions = self.joints.tolist()
            duration_msg(.1, point.time_from_start)
            message.points = [point]
            self.command_pub.publish(message)
        self.operation = 'cancel_hold_sent'

    def publish_status(self):
        now = self.now()
        status = self.fusion.snapshot(now)
        guard = self.teleop_guard()
        if self.teleop.active and guard is not None:
            self.stop_teleop(guard)
        if self.joint_time is None or now-self.joint_time > self.param('joint_timeout'):
            status['can_commit'] = False
            if status['reason'] == 'tracking':
                status['reason'] = 'joint_feedback_stale'
            self.drop_plan(status['reason'])
            if self.active_goal is not None:
                self.on_cancel(None)
                self.operation = 'execution_aborted_feedback_stale'
        elif self.active_goal is not None:
            elapsed = now-self.active_started
            if elapsed >= self.active_duration and np.max(np.abs(self.joints-self.active_goal)) <= .025:
                self.active_goal = self.active_started = self.active_duration = None
                self.operation = 'execution_succeeded'
            elif elapsed > self.active_duration+4.:
                self.on_cancel(None)
                self.operation = 'execution_aborted_goal_tolerance'
        if not status['can_commit']:
            self.drop_plan(status['reason'])
        if self.teleop.active:
            if status['can_commit']:
                status['reason'] = 'bimanual_teleoperation_active'
            status['can_commit'] = False
        status.update(operation=self.operation, execution_enabled=self.param('allow_execution'),
                      position_only=True, preview_ready=self.plan is not None,
                      executing=self.active_goal is not None,
                      graph_keyframes=getattr(self.fusion.backend, 'index', None),
                      graph_capacity=self.param('max_keyframes'),
                      headset_tracking_ready=not self.tracking_inhibited,
                      robot_feedback='mock' if self.param('mock_joint_states') else 'external',
                      teleop_active=self.teleop.active, teleop_state=self.teleop.state,
                      teleop_limited=self.teleop.state == 'limited',
                      teleop_reason=guard or self.teleop.reason,
                      teleop_protocol_version=2,
                      teleop_translation_scale=self.teleop.translation_scale,
                      teleop_last_fault=self.teleop.last_fault,
                      teleop_relay_ready=(self.teleop_relay_status is not None and
                          self.teleop_relay_status['ready'] and
                          -.05 <= now-self.teleop_relay_status['stamp'] <= .30 and
                          self.monotonic()-self.teleop_relay_received <= .30),
                      teleop_position_error=max(self.teleop.measured_position_errors) if self.teleop.measured_position_errors else None,
                      teleop_orientation_error=max(self.teleop.measured_orientation_errors) if self.teleop.measured_orientation_errors else None,
                      teleop_measured_position_errors=self.teleop.measured_position_errors,
                      teleop_commanded_position_errors=self.teleop.commanded_position_errors,
                      teleop_measured_orientation_errors=self.teleop.measured_orientation_errors,
                      teleop_commanded_orientation_errors=self.teleop.commanded_orientation_errors,
                      teleop_solve_duration=self.teleop.solve_duration,
                      teleop_ready=(guard is None and self.teleop.armed))
        self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))
        if status['base_from_headset_world'] is not None:
            p = status['base_from_headset_world']
            transform = TransformStamped()
            duration_msg(now, transform.header.stamp)
            transform.header.frame_id, transform.child_frame_id = 'base_link', 'headset_world'
            tr, rot = transform.transform.translation, transform.transform.rotation
            tr.x, tr.y, tr.z = p[:3]
            rot.x, rot.y, rot.z, rot.w = p[3:]
            self.transform_pub.publish(transform)

    def synthetic_observation(self):
        if self.tracking_inhibited:
            return
        # Purely simulated measurement, advertised as such on every status update.
        camera = self.arm.fk(self.joints if self.joints is not None else np.array([0.,1.,0.,0.]))
        relative = inverse(camera) @ self.synthetic_transform @ self.headset
        obs = Observation(self.now(), self.headset.copy(), camera, relative, 100, 100, 0., 'simulation_ground_truth')
        if not self.fusion.ingest(obs, self.now()):
            self.drop_plan(self.fusion.reason)
        else:
            self.maintain_preview_alignment()

    def on_mock_command(self, message):
        if message.joint_names == self.arm.names and message.points:
            self.mock_trajectory, self.mock_start = message, self.now()

    def mock_tick(self):
        if self.mock_trajectory:
            points = self.mock_trajectory.points
            times = [stamp_seconds(p.time_from_start) for p in points]
            elapsed = self.now()-self.mock_start
            for j in range(len(self.mock_q)):
                self.mock_q[j] = np.interp(elapsed, times, [p.positions[j] for p in points])
            if elapsed >= times[-1]:
                self.mock_trajectory = None
        message = JointState()
        duration_msg(self.now(), message.header.stamp)
        message.name, message.position = self.arm.names, self.mock_q.tolist()
        self.mock_pub.publish(message)
        ee = PoseStamped()
        ee.header = message.header
        ee.header.frame_id = 'base_link'
        p = to_pose(self.arm.fk(self.mock_q))
        ee.pose.position.x, ee.pose.position.y, ee.pose.position.z = p[:3]
        ee.pose.orientation.x, ee.pose.orientation.y, ee.pose.orientation.z, ee.pose.orientation.w = p[3:]
        self.ee_pub.publish(ee)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DeicticControl()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.on_cancel(None)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
