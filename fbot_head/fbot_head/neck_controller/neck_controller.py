import math
import rclpy
import tf2_ros
import numpy as np
import tf2_geometry_msgs
from copy import deepcopy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from fbot_vision_msgs.msg import Detection3DArray
from fbot_vision_msgs.srv import LookAtDescription3D, LookAtPlace3D
from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from .PyDynamixel import DxlCommProtocol2, JointProtocol2


class NeckController(Node):
    def __init__(self, pause=False):
        """
        @brief A node for controlling the neck and head joints. It provides functionality 
        for updating neck positions, looking at specific points, and handling emergency stops.
        @param pause: If True, the node will not send any data to the motors.
        """
        super().__init__('neck_controller')

        self.motors_config = {
            'horizontal_neck_joint':{
                'current_angle': np.pi,
                'id': 62,
                'min_angle': 120,
                'max_angle': 240
            },
            'vertical_neck_joint':{
                'current_angle': np.pi,
                'id': 61,
                'min_angle': 150,
                'max_angle': 200
            },
            'head_pan_joint':{
                'current_angle': np.pi,
                'id': 8,
            },
            'head_tilt_joint': {
                'current_angle': np.pi,
                'id': 9,
            }
        }

        self.vel_limit = 800
        self.neck_port = "/dev/ttyNECK"
        self.motors: dict[str, JointProtocol2] = {}
        self.neck_comm = None
        try:
            self.setupMotors()
        except RuntimeError as e:
            # Fail fast: a half-initialized node (no subscriptions/timers) would
            # otherwise spin uselessly and crash later with AttributeErrors.
            self.get_logger().fatal(str(e))
            raise

        self.pause = pause
        self.lock_updateNeck = False

        self.sub_emergency_button = self.create_subscription(Bool, 'emergency_button', self.emergencyButtonCallback, 10)
        self.sub_update_neck = self.create_subscription(Float64MultiArray, "updateNeck", self.updateNeckCallback, 10)
        self.sub_update_neck_by_point = self.create_subscription(PointStamped, "updateNeckByPoint", self.updateNeckByPointCallback, 10)

        self.pub_lookat_point_marker = self.create_publisher(MarkerArray, "updateNeckByPoint/marker", 10)
        self.lookat_point_index = 0


        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.neck_updated = None

        self.joints_dict = { 
            'horizontal_neck_joint': (0., 0., 0.),
            'vertical_neck_joint':   (0., 0., 0.),
            'head_pan_joint':        (0., 0., 0.),
            'head_tilt_joint':       (0., 0., 0.)
        }

        self.pub_joint_states = self.create_publisher(JointState, 'boris_head/joint_states', 10)
        self.seq = 0

        self.srv_start_lookat = self.create_service(LookAtDescription3D, 'lookat_start', self.lookAtStart)
        self.srv_stop_lookat = self.create_service(Empty, 'lookat_stop', self.lookAtStop)

        self.srv_start_lookatplace = self.create_service(LookAtPlace3D, 'lookatplace_start', self.lookAtPlaceStart)
        self.srv_stop_lookatplace = self.create_service(Empty, 'lookatplace_stop', self.lookAtPlaceStop)

        self.lookatplace_point: PointStamped = None
        self.lookatplace_timer = None
        self.lookatplace_timeout_timer = None
        self.lookatplace_initial_angle = None
        self.lookatplace_default_rate = 5.0

        self.sub_lookat = None
        self.lookat_description_identifier: dict = None
        self.lookat_pose: PoseStamped = None
        self.last_stopped_time = None
        self.look_at_timeout = float("inf")
        self.lookat_timer = None 
        self.lookat_timeout_callback = None
        self.frame = 'map'
        self.look_at_topic = ''

        self.current_angle = [0.0, 0.0]
        self.initial_angle = [180.0, 180.0]
        self.updateNeck(self.initial_angle)
        self.joints_publish_timer = self.create_timer(5, self.updateJointsDict)

        self.get_logger().info(
            f"NeckController ready: {len(self.motors)} motors on {self.neck_port}, "
            f"paused={self.pause}.")

    def setupMotors(self) -> None:
        """
        @brief Initializes the Dynamixel motors, sets up the communication with the motors and configures their torque and velocity limits.
        """
        try:
            self.neck_comm = DxlCommProtocol2(self.neck_port)

        except Exception as e:
            raise RuntimeError(f"Failed to initialize NeckController: Neck port {self.neck_port} failed to connect. See readme for more details.")

        for motor_name, props in self.motors_config.items():
            
            self.motors[motor_name] = JointProtocol2(props['id'])

            self.neck_comm.attachJoint(self.motors[motor_name])

            self.motors[motor_name].enableTorque()

            self.motors[motor_name].setVelocityLimit(self.vel_limit)


    def emergencyButtonCallback(self, msg) -> None:
        """
        @brief Sets the pause state based on the emergency button's state.
        @param msg: (std_msgs.msg.Bool) The message containing the emergency button state.
        """
        new_pause = not msg.data
        if new_pause != self.pause:
            self.get_logger().warn(
                f"Emergency button changed: motors {'PAUSED' if new_pause else 'RESUMED'}.")
        self.pause = new_pause

    def updateNeckCallback(self, msg) -> None:
        """
        @brief Updates the neck's position based on the received angles.
        @param msg: (std_msgs.msg.Float64MultiArray) The message containing the new neck angles.
        """
        data = msg.data
        self.updateNeck(list(data), from_updateNeckCallback=True)

    def updateNeckByPointCallback(self, msg) -> None:
        """
        @brief Computes the neck angles required to look at the given point and updates the neck's position.
        @param msg: (geometry_msgs.msg.PointStamped) The message containing the target point.
        """
        self.updateNeckByPoint(msg)

    def updateNeckByPoint(self, msg) -> bool:
        """
        @brief Transforms the given point into the head frame, computes the neck angles required to
               look at it and updates the neck's position.
        @param msg: (geometry_msgs.msg.PointStamped) The target point, in any frame.
        @return: (bool) True if the neck was updated, False if the transform could not be computed.
        """
        transform = self.computeTFTransform(source_header = msg.header)
        if not transform:
            self.get_logger().error("Failed to compute transform for updateNeckByPoint.")
            return False
        ps = tf2_geometry_msgs.do_transform_point(msg, transform).point
        self.get_logger().info(
            f"updateNeckByPoint #{self.lookat_point_index}: target ({ps.x:.2f}, {ps.y:.2f}, "
            f"{ps.z:.2f}) in '{transform.header.frame_id}' (from '{msg.header.frame_id}').")
        self.publishLookAtPointMarker(ps, transform.header.frame_id)
        angle_msg = self.computeNeckStateByPoint(ps)
        self.updateNeck(angle_msg, from_updateNeckCallback=True)
        return True

    def publishLookAtPointMarker(self, point, frame_id, lifetime=5.0) -> None:
        """
        @brief Publishes a marker representing the point that updateNeckByPoint is looking at,
               in the frame the point was transformed to. Each call increments the look index.
        @param point: (geometry_msgs.msg.Point) The target point, already in frame_id.
        @param frame_id: (str) The frame the point is expressed in.
        @param lifetime: (float) How long the marker stays visible, in seconds.
        """
        index = self.lookat_point_index
        self.lookat_point_index += 1

        stamp = self.get_clock().now().to_msg()
        marker_lifetime = Duration(seconds=lifetime).to_msg()

        sphere = Marker()
        sphere.header.frame_id = frame_id
        sphere.header.stamp = stamp
        sphere.ns = "updateNeckByPoint"
        sphere.id = 2 * index
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = point.x
        sphere.pose.position.y = point.y
        sphere.pose.position.z = point.z
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 0.1
        sphere.scale.y = 0.1
        sphere.scale.z = 0.1
        sphere.color.r = 0.0
        sphere.color.g = 1.0
        sphere.color.b = 0.0
        sphere.color.a = 1.0
        sphere.lifetime = marker_lifetime

        text = Marker()
        text.header.frame_id = frame_id
        text.header.stamp = stamp
        text.ns = "updateNeckByPoint_index"
        text.id = 2 * index + 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = point.x
        text.pose.position.y = point.y
        text.pose.position.z = point.z + 0.12
        text.pose.orientation.w = 1.0
        text.scale.z = 0.1
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = str(index)
        text.lifetime = marker_lifetime

        self.pub_lookat_point_marker.publish(MarkerArray(markers=[sphere, text]))

    def updateNeck(self, data:list[float], from_updateNeckCallback = False) -> None:
        """
        @brief Updates the neck's position based on the given angles.
        @param data: (list[float]) The list of angles for the neck joints.
        @param from_updateNeckCallback: (bool) Whether the update was triggered by a callback (default: False).
        """
        if data:
            pos_horizontal = np.radians(min(self.motors_config['horizontal_neck_joint']['max_angle'], max(self.motors_config['horizontal_neck_joint']['min_angle'], data[0])))
            pos_vertical = np.radians(min(self.motors_config['vertical_neck_joint']['max_angle'], max(self.motors_config['vertical_neck_joint']['min_angle'], data[1])))

            self.motors_config['horizontal_neck_joint']['current_angle']=pos_horizontal
            self.motors_config['vertical_neck_joint']['current_angle']=pos_vertical
            self.motors_config['head_pan_joint']['current_angle']=pos_horizontal
            self.motors_config['head_tilt_joint']['current_angle']=2*np.pi - pos_vertical

        if self.pause:
            return

        if not self.lock_updateNeck and (not from_updateNeckCallback or self.sub_lookat is None):

            for key in self.motors:
                self.motors[key].sendGoalAngle(self.motors_config[key]['current_angle'])

            if data:
                self.current_angle = data

            self.updateJointsDict()
            self.get_logger().warn(f"UpdatedNeck to: {data}")


    def updateJointsDict(self) -> None:
        """
        @brief Updates the joint state dictionary and publishes the joint states.
        """
        msg = JointState()

        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []

        for key in self.joints_dict:
            position = self.motors[key].receiveCurrAngle() - np.pi
            if key=='vertical_neck_joint': position = -(position)
            self.joints_dict[key] = (position, 0., 0.)

            p, v, e = self.joints_dict[key]
            msg.name.append(key)
            msg.position.append(p)
            msg.velocity.append(v)
            msg.effort.append(e)
            
        try:
            self.pub_joint_states.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish joint states: {str(e)}")

        if (self.joints_dict['head_pan_joint'][1]<=0.3) and (self.joints_dict['head_tilt_joint'][1]<=0.3):
            if self.last_stopped_time is None:
                self.last_stopped_time = msg.header.stamp
        else:
            self.last_stopped_time = None

    def computeTFTransform(self, source_header=None, target_frame='femtobolt_link_static', lastest=True):
        """
        @brief Computes the transform between two frames.
        @param target_frame: (str) The target frame ID.
        @param source_header: (std_msgs.msg.Header) The source frame header.
        @param lastest: (bool) Whether to use the latest transform (default: False).
        """
        if not source_header:
            self.get_logger().error('source_header parameter was not given.')
            return
        transform= None    
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_header.frame_id, rclpy.time.Time() if lastest else source_header.stamp)
            return transform
            
        except Exception as e:
            self.get_logger().error(f"Transform lookup failed: {str(e)}")
    
        return transform
    
    def computeNeckStateByPoint(self, point):
        """
        @brief Computes the neck angles required to look at a given point.
        @param point: (geometry_msgs.msg.Point) The target point.
        """
        horizontal = math.pi + math.atan2(point.y, point.x)
        dist = np.hypot(point.x, point.y)
        z = point.z
        if self.look_at_topic and 'tracking' in self.look_at_topic:
            z = 0.15 if dist > 1.7 else 1.2
        vertical = math.pi + math.atan2(z, dist) #ajuste vertical 
        return [math.degrees(horizontal), math.degrees(vertical)]

    def lookAtStart(self, req : LookAtDescription3D.Request, res : LookAtDescription3D.Response): 
        """
        @brief Stops the "look at" service and resets the neck position.
        @param req: (std_srvs.srv.Empty.Request) The service request.
        """
        self.lookat_description_identifier = {'global_id': req.global_id, 'id': req.id, 'label': req.label}
        self.lookat_initial_angle = list(req.initial_angle)
        self.updateNeck(self.lookat_initial_angle)
        self.sub_lookat = self.create_subscription(Detection3DArray, req.recognitions3d_topic, self.lookAtRecogCallback, 10)
        self.look_at_topic = req.recognitions3d_topic
        self.last_pose  = None
        self.last_pose_time = 0.
        self.lookat_pose = None
        # self.last_stopped_time = None
        self.look_at_timeout = 10*60.0 if req.timeout == 0 else req.timeout

        if self.lookat_timer:
            self.lookat_timer.cancel()
        self.lookat_timer = self.create_timer(self.look_at_timeout, self.lookAtTimeout)

        self.get_logger().info(
            f"lookAt started: target={self.lookat_description_identifier}, "
            f"topic='{req.recognitions3d_topic}', timeout={self.look_at_timeout:.0f}s.")

        return res

    def lookAtRecogCallback(self, msg):
        """
        @brief Callback for processing recognition messages and updating the neck position.
        @param msg: (fbot_vision_msgs.msg.Detection3DArray) The message containing the detected objects.
        """
        if self.lookat_description_identifier is None:
            return

        selected_desc = self.selectDescription(msg.detections)

        if selected_desc is not None:
            header = selected_desc.header
            if self.last_stopped_time is not None and Time.from_msg(header.stamp) >= Time.from_msg(self.last_stopped_time):
                transform = self.computeTFTransform(header, self.frame)

                lookat_pose = PoseStamped()
                lookat_pose.header = header
                lookat_pose.pose = selected_desc.bbox3d.center

                self.lookat_pose = tf2_geometry_msgs.do_transform_pose_stamped(lookat_pose, transform)
                                    
                if self.lookat_timer:
                    self.lookat_timer.reset()

                transform = self.computeTFTransform(self.lookat_pose.header, lastest=True)
                if not transform:
                    self.get_logger().error("Transform not found, cannot compute look at point.")
                    self.lookat_pose = None
                    return
                
                ps = tf2_geometry_msgs.do_transform_pose_stamped(self.lookat_pose, transform).pose.position
                self.publishLookAtPointMarker(ps, transform.header.frame_id)
                distance = 0.
                time = self.get_clock().now().nanoseconds / 1e9
                delta = float("inf")
                if self.last_pose != None:
                    new = np.array([ps.x, ps.y, ps.z])
                    previus = np.array([self.last_pose.x, self.last_pose.y, self.last_pose.z])
                    distance = np.linalg.norm(new - previus)
                    delta = time - self.last_pose_time

                if distance < max(1.5 * delta, 1.5):
                    lookat_neck = self.computeNeckStateByPoint(ps)
                    self.last_pose = deepcopy(ps)
                    if abs(lookat_neck[0] - self.current_angle[0]) > 1.0 or abs(lookat_neck[1] - self.current_angle[1]) > 1.0:
                        self.updateNeck(lookat_neck)

                self.last_pose_time = time
                self.lookat_pose = None


    def lookAtTimeout(self):
        """
        @brief Callback triggered when the "look at" service times out.
        """
        self.get_logger().info(
            f"lookAt timed out after {self.look_at_timeout:.0f}s, returning to initial angle.")
        self.updateNeck(data=self.lookat_initial_angle)

    def getCloserDescription(self, descriptions):
        """
        @brief Finds the closest description from a list of detected objects.
        @param descriptions: (list[fbot_vision_msgs.msg.Detection3D]) The list of detected objects.
        """
        min_desc = None
        min_dist = float('inf')
        for desc in descriptions:
            p = desc.bbox3d.center.position
            dist = np.linalg.norm([p.x, p.y, p.z])
            if dist < min_dist:
                min_desc = desc
                min_dist = dist
        
        return min_desc

    # TODO: implement for another parameters like global_id or local_id
    def selectDescription(self, descriptions):
        """
        @brief Selects a description based on the target criteria.
        @param descriptions: (list[fbot_vision_msgs.msg.Detection3D]) The list of detected objects.
        """
        selected_descriptions = []

        desired_label = self.lookat_description_identifier['label']

        if desired_label != '':
            for desc in descriptions:
                if desc.label == desired_label:
                   selected_descriptions.append(desc)
        else:
            selected_descriptions = descriptions

        desc = self.getCloserDescription(selected_descriptions)

        return desc

    def lookAtStop(self, req: Empty.Request, res: Empty.Response):
        """
        @brief Stops the "look at" service and resets the neck position.
        @param req: (std_srvs.srv.Empty.Request) The service request.
        """
        was_active = self.sub_lookat is not None
        if self.sub_lookat is not None:
            self.destroy_subscription(self.sub_lookat)
            self.sub_lookat = None
            self.updateNeck(self.initial_angle)

        if self.lookat_timer is not None:
            self.lookat_timer.cancel()
            self.lookat_timer = None

        self.lookat_description_identifier = None
        self.look_at_topic = ''
        if was_active:
            self.get_logger().info("lookAt stopped, neck returned to initial angle.")

        return res

    def lookAtPlaceStart(self, req: LookAtPlace3D.Request, res: LookAtPlace3D.Response):
        """
        @brief Starts continuously looking at a fixed point/place. Unlike lookAt, the target is not
               tracked from detections: the same point is repeatedly re-transformed into the head
               frame so the robot keeps looking at it while it (or its base) moves.
        @param req: (LookAtPlace3D.Request) The service request containing the target point, timeout,
                    optional initial angle and update rate.
        @param res: (LookAtPlace3D.Response) The service response.
        """
        # Stop any previous place-looking loop so we don't run two timers at once.
        self.stopLookAtPlace()

        # The tracking loop only needs a position, so keep the pose's point (and frame) internally.
        self.lookatplace_point = PointStamped()
        self.lookatplace_point.header = req.pose.header
        self.lookatplace_point.point = req.pose.pose.position
        self.lookatplace_initial_angle = list(req.initial_angle) if req.initial_angle else None
        if self.lookatplace_initial_angle:
            self.updateNeck(self.lookatplace_initial_angle)

        rate = req.rate if req.rate > 0 else self.lookatplace_default_rate
        timeout = 10 * 60.0 if req.timeout == 0 else req.timeout

        self.lookatplace_timer = self.create_timer(1.0 / rate, self.lookAtPlaceUpdate)
        self.lookatplace_timeout_timer = self.create_timer(timeout, self.lookAtPlaceTimeout)

        p = self.lookatplace_point.point
        self.get_logger().info(
            f"lookAtPlace started: point ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) in "
            f"'{self.lookatplace_point.header.frame_id}', rate={rate:.1f}Hz, timeout={timeout:.0f}s.")

        return res

    def lookAtPlaceUpdate(self):
        """
        @brief Timer callback that re-transforms the stored place point and updates the neck so the
               robot keeps looking at it as it moves.
        """
        if self.lookatplace_point is None:
            return
        # Use the latest transform (not the stored stamp) so tracking follows the robot's motion.
        self.lookatplace_point.header.stamp = rclpy.time.Time().to_msg()
        self.updateNeckByPoint(self.lookatplace_point)

    def lookAtPlaceTimeout(self):
        """
        @brief Callback triggered when the lookAtPlace service times out. Stops the loop and returns
               the neck to its initial angle.
        """
        self.get_logger().info("lookAtPlace timed out, returning to initial angle.")
        self.stopLookAtPlace()
        self.updateNeck(self.lookatplace_initial_angle or self.initial_angle)

    def stopLookAtPlace(self):
        """
        @brief Cancels the place-looking timers and clears the stored target, without moving the neck.
        """
        if self.lookatplace_timer is not None:
            self.lookatplace_timer.cancel()
            self.destroy_timer(self.lookatplace_timer)
            self.lookatplace_timer = None
        if self.lookatplace_timeout_timer is not None:
            self.lookatplace_timeout_timer.cancel()
            self.destroy_timer(self.lookatplace_timeout_timer)
            self.lookatplace_timeout_timer = None
        self.lookatplace_point = None

    def lookAtPlaceStop(self, req: Empty.Request, res: Empty.Response):
        """
        @brief Stops the lookAtPlace service and resets the neck position.
        @param req: (std_srvs.srv.Empty.Request) The service request.
        @param res: (std_srvs.srv.Empty.Response) The service response.
        """
        was_active = self.lookatplace_timer is not None
        self.stopLookAtPlace()
        if was_active:
            self.updateNeck(self.lookatplace_initial_angle or self.initial_angle)
            self.get_logger().info("lookAtPlace stopped, neck returned to initial angle.")

        return res

def main(args=None):
    rclpy.init(args=args)
    node = NeckController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()