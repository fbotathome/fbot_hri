import math
from collections import deque

import numpy as np
import rclpy
import tf2_ros
import tf2_geometry_msgs
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from fbot_vision_msgs.msg import Detection3DArray
from fbot_vision_msgs.srv import LookAtDescription3D, LookAtPlace3D, LookAtTarget3D
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from .PyDynamixel import DxlCommProtocol2, JointProtocol2

CAMERA_JOINTS = ('head_pan_joint', 'head_tilt_joint')
HEAD_JOINTS = ('horizontal_neck_joint', 'vertical_neck_joint')

# KeyPoint3D ids (COCO order)
HEAD_KEYPOINTS = (0, 1, 2, 3, 4)
SHOULDER_KEYPOINTS = (5, 6)


class NeckController(Node):
    """
    Drives the camera pan/tilt (Dynamixel 8/9) and the animatronic head (61/62).

    Angles in the command space are degrees with 180 = straight ahead:
    [horizontal, vertical], vertical > 180 looks up.

    - The motor state is read with one sync read at `state_rate` and published on
      boris_head/joint_states, so the camera TF follows the real neck position.
    - Detections (lookat) are transformed into `lookat.fixed_frame` with the TF at the
      image stamp, filtered, and followed by one control loop at `control_rate`.
    - In the target mode another node owns the identity of what is followed (e.g. the person
      following state) and streams the point to look at; the neck only filters and pursues it,
      holds the last direction when the stream stops and never picks a target by itself.
    - updateNeck / updateNeckByPoint and the end of every look-at mode move the neck to a fixed
      angle with the same smooth pursuit (mode 'move', move.speed_deg / move.accel_deg), never
      with one direct motor goal (the motor profile is fast and accelerates hard).
    - The neck only starts moving when the target is more than `start_threshold_deg`
      away and stops correcting once it is within `stop_threshold_deg` (hysteresis),
      with velocity/acceleration profiles on the motors.
    """

    def __init__(self):
        super().__init__('neck_controller')
        self.declareParameters()
        self.readParameters()

        self.motors: dict[str, JointProtocol2] = {}
        self.neck_comm = None
        try:
            self.setupMotors()
        except RuntimeError as e:
            self.get_logger().error(str(e))
            return

        self.pause = False

        # measured state, joint positions in rad (0 = centre), velocities in rad/s
        self.joint_pos = {name: 0.0 for name in self.motors}
        self.joint_vel = {name: 0.0 for name in self.motors}
        self.state_ok = False
        self.moving = None

        # last commands, in the command space (deg)
        self.camera_goal = None
        self.head_goal = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # lookat / lookatplace state
        self.mode = None                 # None | 'lookat' | 'place' | 'target' (point modes) | 'move'
        self.correcting = False          # hysteresis state of the control loop
        self.sub_lookat = None
        self.sub_target = None
        self.pending_points = deque(maxlen=5)   # (receive time, PointStamped), target mode
        self.lookat_identifier = None
        self.lookat_label = ''
        self.mode_initial_angle = list(self.initial_angle)
        self.lost_timeout = self.default_lost_timeout
        self.returned_home = False
        self.pending = deque(maxlen=3)   # (receive time, Detection3DArray)
        self.target = None               # filtered target in fixed_frame (np.array 3)
        self.target_vel = np.zeros(3)    # estimated target velocity in fixed_frame (m/s)
        self.target_time = None          # node time (s) of the last accepted detection
        self.setpoint = None             # pursuit setpoint, command space (deg)
        self.setpoint_vel = np.zeros(2)
        self.target_id = None
        self.jump_candidates = []
        self.acquire_candidates = []     # (time, point) of detections confirming a new target
        # point modes: the target is kept in point_frame (fixed_frame for a stream, the request
        # frame for a place, so map corrections are followed) and expires at point_deadline
        self.point_frame = None
        self.point_deadline = None
        self.point_limits = (None, None)  # (max speed, max accel) of the pursuit, None = lookat defaults
        # move mode: smooth move to a fixed angle (updateNeck, updateNeckByPoint, end of a look-at)
        self.move_target = None
        self.move_speed = None

        self.pub_joint_states = self.create_publisher(JointState, 'boris_head/joint_states', 10)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_moving = self.create_publisher(Bool, 'neck/moving', latched)
        self.pub_target_marker = self.create_publisher(Marker, 'neck/lookat_target', 10)

        self.sub_emergency_button = self.create_subscription(Bool, 'emergency_button', self.emergencyButtonCallback, 10)
        self.sub_update_neck = self.create_subscription(Float64MultiArray, 'updateNeck', self.updateNeckCallback, 10)
        self.sub_update_neck_by_point = self.create_subscription(PointStamped, 'updateNeckByPoint', self.updateNeckByPointCallback, 10)

        self.srv_start_lookat = self.create_service(LookAtDescription3D, 'lookat_start', self.lookAtStart)
        self.srv_stop_lookat = self.create_service(Empty, 'lookat_stop', self.lookAtStop)
        self.srv_start_lookatplace = self.create_service(LookAtPlace3D, 'lookatplace_start', self.lookAtPlaceStart)
        self.srv_stop_lookatplace = self.create_service(Empty, 'lookatplace_stop', self.lookAtPlaceStop)
        self.srv_start_lookattarget = self.create_service(LookAtTarget3D, 'lookattarget_start', self.lookAtTargetStart)
        self.srv_stop_lookattarget = self.create_service(Empty, 'lookattarget_stop', self.lookAtTargetStop)

        self.sendGoal(self.initial_angle, force_head=True)
        self.state_timer = self.create_timer(1.0 / self.state_rate, self.readState)
        self.control_timer = self.create_timer(1.0 / self.control_rate, self.controlLoop)
        self.get_logger().info(f'NeckController ready: {len(self.motors)} motors on {self.neck_port}.')

    # ------------------------------------------------------------------ parameters

    def declareParameters(self) -> None:
        self.declare_parameter('neck_port', '/dev/ttyNECK')
        self.declare_parameter('baudrate', 57600)
        self.declare_parameter('horizontal_neck_joint.id', 62)
        self.declare_parameter('horizontal_neck_joint.min_angle', 120)
        self.declare_parameter('horizontal_neck_joint.max_angle', 240)
        self.declare_parameter('vertical_neck_joint.id', 61)
        self.declare_parameter('vertical_neck_joint.min_angle', 150)
        self.declare_parameter('vertical_neck_joint.max_angle', 190)
        self.declare_parameter('head_pan_joint.id', 8)
        self.declare_parameter('head_tilt_joint.id', 9)
        self.declare_parameter('initial_angle', [180.0, 180.0])

        self.declare_parameter('camera.profile_velocity', 60)
        self.declare_parameter('camera.profile_acceleration', 15)
        self.declare_parameter('camera.pursuit_profile_velocity', 150)
        self.declare_parameter('head.profile_velocity', 40)
        self.declare_parameter('head.profile_acceleration', 8)

        self.declare_parameter('state_rate', 30.0)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('moving_threshold', 0.05)

        self.declare_parameter('lookat.fixed_frame', 'odom')
        self.declare_parameter('lookat.head_frame', 'camera_link_static')
        self.declare_parameter('lookat.tf_wait', 0.3)
        self.declare_parameter('lookat.max_detection_age', 1.5)
        self.declare_parameter('lookat.filter_time_constant', 0.25)
        self.declare_parameter('lookat.max_jump', 0.8)
        self.declare_parameter('lookat.jump_confirmations', 3)
        self.declare_parameter('lookat.switch_target_after', 1.5)
        self.declare_parameter('lookat.acquire_confirmations', 3)
        self.declare_parameter('lookat.acquire_window', 1.0)
        self.declare_parameter('lookat.lost_timeout', 3.0)
        self.declare_parameter('lookat.min_lost_timeout', 2.0)
        self.declare_parameter('lookat.velocity_gain', 0.3)
        self.declare_parameter('lookat.max_target_speed', 1.5)
        self.declare_parameter('lookat.max_extrapolation', 0.6)
        self.declare_parameter('lookat.start_threshold_deg', 4.0)
        self.declare_parameter('lookat.stop_threshold_deg', 1.0)
        self.declare_parameter('lookat.gain', 2.5)
        self.declare_parameter('lookat.max_speed_deg', 45.0)
        self.declare_parameter('lookat.max_accel_deg', 90.0)
        self.declare_parameter('lookat.return_speed_deg', 25.0)
        self.declare_parameter('move.speed_deg', 30.0)
        self.declare_parameter('move.accel_deg', 60.0)
        self.declare_parameter('lookat.min_command_step_deg', 0.2)
        self.declare_parameter('lookat.head_deadband_deg', 10.0)
        self.declare_parameter('lookat.keypoint_min_score', 0.5)
        self.declare_parameter('lookat.shoulder_to_head', 0.25)
        self.declare_parameter('lookat.person_z_limits', [0.4, 2.3])
        self.declare_parameter('lookat.target.topic', 'neck/target_point')
        self.declare_parameter('lookat.target.lost_timeout', 15.0)
        self.declare_parameter('lookat.target.max_speed_deg', 90.0)
        self.declare_parameter('lookat.target.max_accel_deg', 180.0)

    def readParameters(self) -> None:
        p = lambda name: self.get_parameter(name).value
        self.neck_port = p('neck_port')
        self.baudrate = p('baudrate')
        self.motors_config = {
            'horizontal_neck_joint': {'id': p('horizontal_neck_joint.id'),
                                      'min_angle': p('horizontal_neck_joint.min_angle'),
                                      'max_angle': p('horizontal_neck_joint.max_angle')},
            'vertical_neck_joint': {'id': p('vertical_neck_joint.id'),
                                    'min_angle': p('vertical_neck_joint.min_angle'),
                                    'max_angle': p('vertical_neck_joint.max_angle')},
            'head_pan_joint': {'id': p('head_pan_joint.id')},
            'head_tilt_joint': {'id': p('head_tilt_joint.id')},
        }
        self.initial_angle = list(p('initial_angle'))
        self.camera_profile = (p('camera.profile_velocity'), p('camera.profile_acceleration'),
                               p('camera.pursuit_profile_velocity'))
        self.head_profile = (p('head.profile_velocity'), p('head.profile_acceleration'))
        self.state_rate = p('state_rate')
        self.control_rate = p('control_rate')
        self.moving_threshold = p('moving_threshold')

        self.fixed_frame = p('lookat.fixed_frame')
        self.head_frame = p('lookat.head_frame')
        self.tf_wait = p('lookat.tf_wait')
        self.max_detection_age = p('lookat.max_detection_age')
        self.filter_tau = p('lookat.filter_time_constant')
        self.max_jump = p('lookat.max_jump')
        self.jump_confirmations = p('lookat.jump_confirmations')
        self.switch_target_after = p('lookat.switch_target_after')
        self.acquire_confirmations = p('lookat.acquire_confirmations')
        self.acquire_window = p('lookat.acquire_window')
        self.default_lost_timeout = p('lookat.lost_timeout')
        self.min_lost_timeout = p('lookat.min_lost_timeout')
        self.velocity_gain = p('lookat.velocity_gain')
        self.max_target_speed = p('lookat.max_target_speed')
        self.max_extrapolation = p('lookat.max_extrapolation')
        self.gain = p('lookat.gain')
        self.max_speed = p('lookat.max_speed_deg')
        self.max_accel = p('lookat.max_accel_deg')
        self.return_speed = p('lookat.return_speed_deg')
        self.move_speed_default = p('move.speed_deg')
        self.move_accel = p('move.accel_deg')
        self.start_threshold = p('lookat.start_threshold_deg')
        self.stop_threshold = p('lookat.stop_threshold_deg')
        self.min_command_step = p('lookat.min_command_step_deg')
        self.head_deadband = p('lookat.head_deadband_deg')
        self.keypoint_min_score = p('lookat.keypoint_min_score')
        self.shoulder_to_head = p('lookat.shoulder_to_head')
        self.person_z_limits = list(p('lookat.person_z_limits'))
        self.target_topic = p('lookat.target.topic')
        self.target_lost_timeout = p('lookat.target.lost_timeout')
        self.target_max_speed = p('lookat.target.max_speed_deg')
        self.target_max_accel = p('lookat.target.max_accel_deg')

    def setupMotors(self) -> None:
        try:
            self.neck_comm = DxlCommProtocol2(self.baudrate, self.neck_port)
        except Exception:
            raise RuntimeError(f'Failed to initialize NeckController: Neck port {self.neck_port} failed to connect. See readme for more details.')

        # A sync read fails as a whole when one id does not answer, so only motors that answer
        # a ping are read (the animatronic head, 61/62, is not always mounted).
        self.present_motors = []
        for name, props in self.motors_config.items():
            motor = JointProtocol2(props['id'])
            self.neck_comm.attachJoint(motor)
            self.motors[name] = motor
            _, result, _ = motor.pack_handler.ping(motor.socket, motor.servo_id)
            if result != 0:
                self.get_logger().warn(f'{name} (id {props["id"]}) does not answer; it is commanded but not read.')
                continue
            self.present_motors.append(motor)
            motor.enableTorque()
            velocity, acceleration = self.camera_profile[:2] if name in CAMERA_JOINTS else self.head_profile
            motor.setVelocityLimit(velocity)
            motor.setProfileAcceleration(acceleration)
        self.camera_profile_units = self.camera_profile[0]
        if not any(self.motors[name] in self.present_motors for name in CAMERA_JOINTS):
            raise RuntimeError(f'Failed to initialize NeckController: no camera motor answers on {self.neck_port}.')

    # ------------------------------------------------------------------ motor I/O

    def now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def clampCommand(self, angles):
        h = self.motors_config['horizontal_neck_joint']
        v = self.motors_config['vertical_neck_joint']
        return [min(h['max_angle'], max(h['min_angle'], angles[0])),
                min(v['max_angle'], max(v['min_angle'], angles[1]))]

    def setCameraProfileVelocity(self, pursuit: bool) -> None:
        """
        @brief Profile velocity of the camera motors. The pursuit already limits speed and
               acceleration and streams small goals at control_rate, so the motors get a fast
               profile (a slow one makes them brake towards every goal: ticks); direct goals
               (updateNeck, start/stop) keep the slow default profile.
        """
        units = self.camera_profile[2] if pursuit else self.camera_profile[0]
        if units == self.camera_profile_units:
            return
        motors = [self.motors['head_pan_joint'], self.motors['head_tilt_joint']]
        self.neck_comm.syncWriteProfileVelocities([(m, units) for m in motors if m in self.present_motors])
        self.camera_profile_units = units

    def sendGoal(self, angles, force_head=False, pursuit=False) -> None:
        """
        @brief Sends [horizontal, vertical] (deg) to the camera motors and, when it moved more than
               the head dead band (or force_head), to the animatronic head. A direct goal (not from
               the pursuit) resets the pursuit setpoint to it.
        """
        if not angles or len(angles) < 2:
            return
        angles = self.clampCommand(angles)
        self.setpoint = np.array(angles)
        if not pursuit:
            self.setpoint_vel = np.zeros(2)
        if self.pause:
            return
        horizontal, vertical = np.radians(angles[0]), np.radians(angles[1])
        commands = [(self.motors['head_pan_joint'], horizontal),
                    (self.motors['head_tilt_joint'], 2 * np.pi - vertical)]
        if force_head or self.head_goal is None or max(abs(a - b) for a, b in zip(angles, self.head_goal)) > self.head_deadband:
            commands += [(self.motors['horizontal_neck_joint'], horizontal),
                         (self.motors['vertical_neck_joint'], vertical)]
            self.head_goal = list(angles)
        try:
            self.setCameraProfileVelocity(pursuit)
            self.neck_comm.syncWriteGoalAngles(commands)
        except Exception as e:
            self.get_logger().error(f'Failed to send neck goal: {e}', throttle_duration_sec=2.0)
            return
        self.camera_goal = list(angles)

    def readState(self) -> None:
        """
        @brief Reads every motor with one sync read and publishes boris_head/joint_states.
        """
        stamp = self.get_clock().now()
        try:
            states = self.neck_comm.syncReadStates(self.present_motors)
        except Exception as e:
            self.get_logger().error(f'Neck sync read failed: {e}', throttle_duration_sec=2.0)
            return
        if not states:
            self.get_logger().warn('Neck sync read: no motor answered.', throttle_duration_sec=2.0)
            return

        msg = JointState()
        msg.header.stamp = stamp.to_msg()
        for name, motor in self.motors.items():
            if motor.servo_id in states:
                angle, velocity = states[motor.servo_id]
                position = angle - np.pi
                if name == 'vertical_neck_joint':
                    position, velocity = -position, -velocity
                self.joint_pos[name] = position
                self.joint_vel[name] = velocity
            msg.name.append(name)
            msg.position.append(self.joint_pos[name])
            msg.velocity.append(self.joint_vel[name])
            msg.effort.append(0.0)
        self.state_ok = True
        self.pub_joint_states.publish(msg)

        moving = any(abs(self.joint_vel[n]) > self.moving_threshold for n in CAMERA_JOINTS)
        if moving != self.moving:
            self.moving = moving
            self.pub_moving.publish(Bool(data=moving))

    def measuredCommand(self):
        """
        @brief Current camera position in the command space (deg).
        """
        pan = self.joint_pos['head_pan_joint']
        tilt = self.joint_pos['head_tilt_joint']       # tilt motor = 2*pi - vertical
        return [math.degrees(pan + np.pi), math.degrees(np.pi - tilt)]

    # ------------------------------------------------------------------ simple commands

    def emergencyButtonCallback(self, msg) -> None:
        self.pause = not msg.data

    def updateNeckCallback(self, msg) -> None:
        if self.mode not in (None, 'move'):
            self.get_logger().warn(f'updateNeck ignored: {self.mode} is active.', throttle_duration_sec=2.0)
            return
        if len(msg.data) < 2:
            self.get_logger().error(f'updateNeck: expected [horizontal, vertical], got {list(msg.data)}.')
            return
        self.startMove(list(msg.data)[:2], self.move_speed_default)

    def updateNeckByPointCallback(self, msg) -> None:
        """@brief Looks at a point: the angles are computed once, from the robot pose at reception."""
        if self.mode not in (None, 'move'):
            self.get_logger().warn(f'updateNeckByPoint ignored: {self.mode} is active.', throttle_duration_sec=2.0)
            return
        angles = self.anglesToPoint(msg)
        if angles is None:
            self.get_logger().error('updateNeckByPoint: failed to transform the point.')
            return
        self.startMove(angles, self.move_speed_default)

    def anglesToPoint(self, point_stamped: PointStamped):
        """
        @brief Command angles (deg) that point the camera at a point, using the latest TF.
        """
        try:
            ps = PointStamped()
            ps.header.frame_id = point_stamped.header.frame_id
            ps.point = point_stamped.point
            tr = self.tf_buffer.lookup_transform(self.head_frame, ps.header.frame_id, Time())
        except Exception as e:
            self.get_logger().error(f'Transform {ps.header.frame_id} -> {self.head_frame} failed: {e}',
                                    throttle_duration_sec=2.0)
            return None
        p = tf2_geometry_msgs.do_transform_point(ps, tr).point
        horizontal = math.pi + math.atan2(p.y, p.x)
        vertical = math.pi + math.atan2(p.z, math.hypot(p.x, p.y))
        return [math.degrees(horizontal), math.degrees(vertical)]

    # ------------------------------------------------------------------ lookat (detections)

    def lookAtStart(self, req: LookAtDescription3D.Request, res: LookAtDescription3D.Response):
        """
        @brief Follows a detection from req.recognitions3d_topic. req.label filters by label,
               req.global_id > 0 follows that id. req.timeout: seconds without a valid detection
               before returning to req.initial_angle (0 -> lookat.lost_timeout).
        """
        self.stopMode()
        self.syncSetpoint()
        self.lookat_label = req.label
        self.lookat_identifier = req.global_id if req.global_id > 0 else None
        self.mode_initial_angle = list(req.initial_angle) if len(req.initial_angle) == 2 else list(self.initial_angle)
        self.lost_timeout = max(req.timeout if req.timeout > 0 else self.default_lost_timeout, self.min_lost_timeout)
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sub_lookat = self.create_subscription(Detection3DArray, req.recognitions3d_topic, self.lookAtRecogCallback, qos)
        self.mode = 'lookat'
        self.returned_home = False      # no target yet: the loop moves smoothly to the initial angle
        self.get_logger().info(
            f"lookAt started: topic='{req.recognitions3d_topic}', label='{req.label}', "
            f"global_id={self.lookat_identifier}, lost_timeout={self.lost_timeout:.1f}s.")
        return res

    def lookAtStop(self, req: Empty.Request, res: Empty.Response):
        if self.mode == 'lookat':
            self.startMove(self.mode_initial_angle, self.return_speed, 'lookAt stopped')
        return res

    def lookAtRecogCallback(self, msg: Detection3DArray) -> None:
        if self.mode == 'lookat':
            self.pending.append((self.now(), msg))

    def processPending(self) -> None:
        """
        @brief Transforms queued detections with the TF at their image stamp. A detection waits
               up to lookat.tf_wait for that TF (the neck state may still be on its way).
        """
        now = self.now()
        while self.pending:
            received, msg = self.pending[0]
            if not msg.detections:
                self.pending.popleft()
                continue
            header = msg.detections[0].header
            stamp = Time.from_msg(header.stamp)
            age = now - stamp.nanoseconds / 1e9
            if age > self.max_detection_age:
                self.get_logger().warn(f'lookAt: dropping detection {age:.2f} s old.', throttle_duration_sec=5.0)
                self.pending.popleft()
                continue
            try:
                tr = self.tf_buffer.lookup_transform(self.fixed_frame, header.frame_id, stamp)
            except tf2_ros.ExtrapolationException as e:
                if now - received < self.tf_wait:
                    return      # TF at that stamp not there yet: try again next cycle
                self.get_logger().warn(f'lookAt: no TF at the detection stamp: {e}', throttle_duration_sec=5.0)
                self.pending.popleft()
                continue
            except Exception as e:
                self.get_logger().error(f'lookAt: transform {header.frame_id} -> {self.fixed_frame} failed: {e}',
                                        throttle_duration_sec=5.0)
                self.pending.popleft()
                continue
            self.pending.popleft()
            self.updateTarget(msg.detections, tr, now)

    def aimPoint(self, det, tr):
        """
        @brief Point to look at for one detection, in fixed_frame. Persons: median of the head
               keypoints, else shoulders raised by lookat.shoulder_to_head, else the bbox centre.
        @return (np.array 3, source) or (None, reason)
        """
        def transform(x, y, z):
            ps = PointStamped()
            ps.point.x, ps.point.y, ps.point.z = float(x), float(y), float(z)
            q = tf2_geometry_msgs.do_transform_point(ps, tr).point
            return np.array([q.x, q.y, q.z])

        def keypoints(ids):
            pts = np.array([[k.x, k.y, k.z] for k in det.pose
                            if k.id in ids and k.score >= self.keypoint_min_score and k.z > 0.1])
            if len(pts) == 0:
                return None
            depth = np.median(pts[:, 2])
            pts = pts[np.abs(pts[:, 2] - depth) < 0.3]
            return np.median(pts, axis=0) if len(pts) else None

        point, source = None, 'bbox'
        head = keypoints(HEAD_KEYPOINTS)
        if head is not None:
            point, source = transform(*head), 'head'
        else:
            shoulders = keypoints(SHOULDER_KEYPOINTS)
            if shoulders is not None:
                point, source = transform(*shoulders) + np.array([0.0, 0.0, self.shoulder_to_head]), 'shoulders'
        if point is None:
            c = det.bbox3d.center.position
            if c.z <= 0.0:
                return None, 'no depth'
            point = transform(c.x, c.y, c.z)

        if det.label == 'person' and not (self.person_z_limits[0] <= point[2] <= self.person_z_limits[1]):
            return None, f'{source} at z={point[2]:.2f} m'
        return point, source

    def updateTarget(self, detections, tr, now) -> None:
        candidates = []
        for det in detections:
            if self.lookat_label and det.label != self.lookat_label:
                continue
            if self.lookat_identifier is not None and det.global_id != self.lookat_identifier:
                continue
            point, source = self.aimPoint(det, tr)
            if point is None:
                self.get_logger().info(f'lookAt: rejected {det.label} {det.global_id}: {source}',
                                       throttle_duration_sec=2.0)
                continue
            candidates.append((det, point, source))
        if not candidates:
            return

        lost_for = float('inf') if self.target_time is None else now - self.target_time
        chosen = None
        if self.target is not None and lost_for < self.switch_target_after:
            same_id = [c for c in candidates if self.target_id is not None and c[0].global_id == self.target_id]
            if same_id:
                chosen = same_id[0]
            else:
                # no id (faces) or new id: the candidate closest to the current target
                chosen = min(candidates, key=lambda c: np.linalg.norm(c[1] - self.target))
        else:
            # no target (or lost for long enough): the person closest to the camera
            origin = tr.transform.translation
            o = np.array([origin.x, origin.y, origin.z])
            chosen = min(candidates, key=lambda c: np.linalg.norm(c[1] - o))
            self.target = None

        det, point, source = chosen
        if self.target is None:
            # a new target needs acquire_confirmations detections at the same place within
            # acquire_window: single false detections (e.g. at the image border, seen only when
            # the neck passes by) would otherwise pull the neck there again and again
            self.acquire_candidates = [(t, p) for t, p in self.acquire_candidates
                                       if now - t < self.acquire_window and np.linalg.norm(p - point) < self.max_jump / 2]
            self.acquire_candidates.append((now, point))
            if len(self.acquire_candidates) < self.acquire_confirmations:
                return
            self.target = np.mean([p for _, p in self.acquire_candidates], axis=0)
            self.target_vel = np.zeros(3)
        else:
            jump = np.linalg.norm(point - self.target)
            if jump > self.max_jump:
                # accept a jump only when several detections in a row agree on it
                self.jump_candidates = [p for p in self.jump_candidates if np.linalg.norm(p - point) < self.max_jump / 2]
                self.jump_candidates.append(point)
                if len(self.jump_candidates) < self.jump_confirmations:
                    return
                self.target = np.mean(self.jump_candidates, axis=0)
                self.target_vel = np.zeros(3)
            else:
                self.filterTarget(point, now)
        self.jump_candidates = []
        self.acquire_candidates = []
        self.target_time = now
        self.target_id = det.global_id if det.global_id > 0 else None
        self.returned_home = False
        self.publishTargetMarker(self.target, source)

    def filterTarget(self, point, now) -> None:
        """
        @brief Alpha-beta filter: position + horizontal velocity, used to keep moving between
               measurements (they arrive at only a few Hz).
        """
        dt = max(now - self.target_time, 1e-3)
        predicted = self.target + self.target_vel * dt
        residual = point - predicted
        alpha = 1.0 - math.exp(-dt / self.filter_tau)
        self.target = predicted + alpha * residual
        self.target_vel = self.target_vel + self.velocity_gain * residual / dt
        self.target_vel[2] = 0.0
        speed = np.linalg.norm(self.target_vel)
        if speed > self.max_target_speed:
            self.target_vel *= self.max_target_speed / speed

    def publishTargetMarker(self, point, text, frame=None) -> None:
        m = Marker()
        m.header.frame_id = frame or self.fixed_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'neck_lookat'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, point)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.12
        m.color.g = 1.0 if text == 'head' else 0.5
        m.color.r = 0.0 if text == 'head' else 1.0
        m.color.a = 1.0
        m.lifetime = Duration(seconds=1.0).to_msg()
        self.pub_target_marker.publish(m)

    # ------------------------------------------------------------------ point modes (lookatplace, lookattarget)
    # Both make the neck look at a point given from outside (not chosen by the neck): one fixed
    # point (place) or a stream of points (target). They share the target, the prediction and the
    # pursuit; they differ in where the point comes from, its frame and when it expires.

    def startPointMode(self, mode, initial_angle, frame, limits) -> None:
        self.stopMode()
        self.syncSetpoint()
        self.mode_initial_angle = list(initial_angle) if len(initial_angle) == 2 else list(self.initial_angle)
        self.point_frame = frame
        self.point_limits = limits
        self.mode = mode

    def setPointTarget(self, point, now, static, lifetime) -> None:
        """
        @brief Updates the target of a point mode with a point already in point_frame. A static
               point, the first one or a jump (the publisher owns the identity: a jump is a new
               target position, not an outlier) resets the filter; otherwise alpha-beta filter.
               The target expires `lifetime` seconds from now.
        """
        if static or self.target is None or np.linalg.norm(point - self.target) > self.max_jump:
            self.target = point
            self.target_vel = np.zeros(3)
        else:
            self.filterTarget(point, now)
        self.target_time = now
        self.point_deadline = now + lifetime
        self.returned_home = False
        self.publishTargetMarker(self.target, self.mode, self.point_frame)

    def stopPointMode(self, mode, name) -> None:
        if self.mode == mode:
            self.startMove(self.mode_initial_angle, self.return_speed, f'{name} stopped')

    def lookAtPlaceStart(self, req: LookAtPlace3D.Request, res: LookAtPlace3D.Response):
        """
        @brief Keeps looking at a fixed pose while the robot moves, for req.timeout seconds
               (0 -> 10 min), then returns to req.initial_angle and ends the mode. The point stays
               in the pose frame (e.g. map) and is transformed with the latest TF every cycle.
        """
        self.startPointMode('place', req.initial_angle, req.pose.header.frame_id, (None, None))
        p = req.pose.pose.position
        timeout = req.timeout if req.timeout > 0 else 600.0
        self.setPointTarget(np.array([p.x, p.y, p.z]), self.now(), static=True, lifetime=timeout)
        self.get_logger().info(f"lookAtPlace started: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) in "
                               f"'{self.point_frame}', timeout={timeout:.0f}s.")
        return res

    def lookAtPlaceStop(self, req: Empty.Request, res: Empty.Response):
        self.stopPointMode('place', 'lookAtPlace')
        return res

    def lookAtTargetStart(self, req: LookAtTarget3D.Request, res: LookAtTarget3D.Response):
        """
        @brief Follows the points published on req.target_topic (PointStamped, any frame). The
               publisher decides what is followed; the neck filters the points and pursues them.
               Without new points the neck keeps looking at the last one (in fixed_frame, so it
               compensates the base motion) and returns to req.initial_angle after req.timeout
               seconds (0 -> lookat.target.lost_timeout); the mode stays active. A point with a
               zero stamp is a fixed direction (latest TF, no velocity estimate), e.g. to scan
               for a lost target.
        """
        self.startPointMode('target', req.initial_angle, self.fixed_frame,
                            (self.target_max_speed, self.target_max_accel))
        topic = req.target_topic or self.target_topic
        self.lost_timeout = max(req.timeout if req.timeout > 0 else self.target_lost_timeout, self.min_lost_timeout)
        self.sub_target = self.create_subscription(PointStamped, topic, self.targetPointCallback, 10)
        self.returned_home = False      # no points yet: the loop moves smoothly to the initial angle
        self.get_logger().info(f"lookAtTarget started: topic='{topic}', lost_timeout={self.lost_timeout:.1f}s.")
        return res

    def lookAtTargetStop(self, req: Empty.Request, res: Empty.Response):
        self.stopPointMode('target', 'lookAtTarget')
        return res

    def targetPointCallback(self, msg: PointStamped) -> None:
        if self.mode == 'target':
            self.pending_points.append((self.now(), msg))

    def processPendingPoints(self) -> None:
        """
        @brief Moves the queued points into fixed_frame (TF at their stamp, waiting up to
               lookat.tf_wait for it; latest TF for a zero stamp) and updates the target.
        """
        now = self.now()
        while self.pending_points:
            received, msg = self.pending_points[0]
            stamp = Time.from_msg(msg.header.stamp)
            static = stamp.nanoseconds == 0
            point = np.array([msg.point.x, msg.point.y, msg.point.z])
            if msg.header.frame_id != self.fixed_frame:
                try:
                    tr = self.tf_buffer.lookup_transform(self.fixed_frame, msg.header.frame_id, stamp)
                except tf2_ros.ExtrapolationException as e:
                    if now - received < self.tf_wait:
                        return      # TF at that stamp not there yet: try again next cycle
                    self.get_logger().warn(f'lookAtTarget: no TF at the point stamp: {e}', throttle_duration_sec=5.0)
                    self.pending_points.popleft()
                    continue
                except Exception as e:
                    self.get_logger().error(f'lookAtTarget: transform {msg.header.frame_id} -> {self.fixed_frame} '
                                            f'failed: {e}', throttle_duration_sec=5.0)
                    self.pending_points.popleft()
                    continue
                q = tf2_geometry_msgs.do_transform_point(msg, tr).point
                point = np.array([q.x, q.y, q.z])
            self.pending_points.popleft()
            self.setPointTarget(point, now, static, self.lost_timeout)

    # ------------------------------------------------------------------ control loop

    def stopMode(self) -> None:
        if self.sub_lookat is not None:
            self.destroy_subscription(self.sub_lookat)
            self.sub_lookat = None
        if self.sub_target is not None:
            self.destroy_subscription(self.sub_target)
            self.sub_target = None
        self.pending_points.clear()
        self.mode = None
        self.correcting = False
        self.pending.clear()
        self.target = None
        self.target_time = None
        self.target_id = None
        self.jump_candidates = []
        self.acquire_candidates = []
        self.point_frame = None
        self.point_deadline = None
        self.point_limits = (None, None)

    def controlLoop(self) -> None:
        if self.mode is None:
            return
        if self.pause:
            # the motors do not get goals: drop the setpoint so the pursuit restarts from the
            # measured position instead of jumping to where the setpoint went meanwhile
            self.setpoint = None
            return
        try:
            if self.mode == 'move':
                self.moveStep()
                return
            if self.mode == 'lookat':
                self.processPending()
                if self.target is None or self.now() - self.target_time > self.lost_timeout:
                    self.returnHome('lookAt: target lost, returning to the initial angle.')
                    return
                self.pursueTarget(self.fixed_frame)
                return

            if self.mode == 'target':
                self.processPendingPoints()
            if self.target is None or self.now() > self.point_deadline:
                if self.mode == 'place':
                    self.startMove(self.mode_initial_angle, self.return_speed, 'lookAtPlace timed out')
                else:
                    self.returnHome('lookAtTarget: no points, returning to the initial angle.')
                return
            self.pursueTarget(self.point_frame, *self.point_limits)
        except Exception as e:
            self.get_logger().error(f'Neck control loop: {e}', throttle_duration_sec=2.0)

    def syncSetpoint(self) -> None:
        """@brief Restarts the pursuit from the measured position when the neck is at rest."""
        if self.mode is None and self.state_ok:
            self.setpoint = np.array(self.measuredCommand())
            self.setpoint_vel = np.zeros(2)

    def startMove(self, angles, speed, reason=None) -> None:
        """
        @brief Moves the neck to a fixed angle with the smooth pursuit (speed in deg/s,
               move.accel_deg) instead of one direct goal: a direct goal runs with the motor
               profile (fast, strong acceleration) and looks abrupt. Used by updateNeck,
               updateNeckByPoint and at the end of every look-at mode. A new move while one is
               running only changes its goal (the velocity carries over); a look-at start ends it.
        """
        if self.mode != 'move':
            self.stopMode()
            self.syncSetpoint()
            self.mode = 'move'
        self.move_target = self.clampCommand(angles)
        self.move_speed = speed
        self.correcting = True          # go all the way to stop_threshold, even from a few degrees away
        if reason:
            self.get_logger().info(f'{reason}, returning smoothly to the initial angle.')

    def moveStep(self) -> None:
        target = self.move_target
        self.track(target, max_speed=self.move_speed, max_accel=self.move_accel, braking=True)
        if self.setpoint is None or not self.state_ok:
            return
        if (not self.correcting and np.max(np.abs(self.setpoint_vel)) < 1e-3) or \
                np.max(np.abs(np.array(target) - self.setpoint)) < 0.05:
            self.stopMode()
            self.sendGoal(target, force_head=True)      # last < stop_threshold, and the animatronic head

    def returnHome(self, message) -> None:
        """@brief Slow pursuit back to the initial angle (not a direct jump); the mode stays active."""
        if not self.returned_home:
            self.get_logger().info(message)
            self.returned_home = True
            self.target = None
        self.track(self.clampCommand(self.mode_initial_angle), max_speed=self.return_speed, max_accel=self.move_accel)

    def pursueTarget(self, frame, max_speed=None, max_accel=None) -> None:
        """
        @brief Points the neck at the target predicted up to max_extrapolation ahead. After that the
               prediction stays where it is, so the neck keeps looking at the place where the target
               was last seen while the base moves (the target is in a frame that does not move with it).
        """
        horizon = min(self.now() - self.target_time, self.max_extrapolation)
        predicted = self.target + self.target_vel * horizon
        ps = PointStamped()
        ps.header.frame_id = frame
        ps.point.x, ps.point.y, ps.point.z = map(float, predicted)
        desired = self.anglesToPoint(ps)
        if desired is not None:
            self.track(self.clampCommand(desired), max_speed=max_speed, max_accel=max_accel)

    def track(self, desired, max_speed=None, max_accel=None, braking=False) -> None:
        """
        @brief Smooth pursuit. A setpoint moves towards the desired angles with a proportional
               velocity (lookat.gain) limited by lookat.max_speed_deg / max_accel_deg and is sent
               every cycle, so the motors get a ramp instead of steps. Hysteresis: the pursuit
               starts when the target is start_threshold away and ends within stop_threshold;
               outside the pursuit the velocity ramps down to zero. braking (fixed goals): the
               speed is also limited to sqrt(2 * accel * error), so the neck brakes at max_accel
               and stops at the goal instead of crawling the last degrees (gain only: ~1 s more).
        """
        if not self.state_ok:
            return
        dt = 1.0 / self.control_rate
        if self.setpoint is None:
            self.setpoint = np.array(self.measuredCommand())
            self.setpoint_vel = np.zeros(2)
        desired = np.array(desired)
        error = desired - self.setpoint
        if not self.correcting:
            if np.max(np.abs(error)) >= self.start_threshold:
                self.correcting = True
        elif np.max(np.abs(error)) < self.stop_threshold:
            self.correcting = False

        speed = self.max_speed if max_speed is None else max_speed
        accel = self.max_accel if max_accel is None else max_accel
        if not self.correcting:
            target_vel = np.zeros(2)
        elif braking:
            target_vel = np.sign(error) * np.minimum(speed, np.sqrt(2.0 * accel * np.abs(error)))
        else:
            target_vel = np.clip(self.gain * error, -speed, speed)
        max_dv = accel * dt
        self.setpoint_vel += np.clip(target_vel - self.setpoint_vel, -max_dv, max_dv)
        if not self.correcting and np.max(np.abs(self.setpoint_vel)) < 1e-3:
            return
        self.setpoint = self.setpoint + self.setpoint_vel * dt
        if self.camera_goal is None or np.max(np.abs(self.setpoint - np.array(self.camera_goal))) > self.min_command_step:
            self.sendGoal(list(self.setpoint), pursuit=True)


def main(args=None):
    rclpy.init(args=args)
    node = NeckController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
