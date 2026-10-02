import math

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Pose, PoseArray
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class BallDetectorNode(Node):
    """
    RGB-D perception pipeline:

        RGB image -> HSV -> contour filtering -> pixel center
        depth + CameraInfo -> 3D camera point
        3D camera point + TF2 -> map-frame point
    """

    def __init__(self):
        super().__init__("ball_detector_node")

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        names = [
            "image_topic", "depth_topic", "camera_info_topic",
            "camera_frame", "map_frame", "h_min", "h_max",
            "s_min", "s_max", "v_min", "v_max", "min_area",
            "max_area", "min_radius_px", "min_depth_m", "max_depth_m",
            "morphology_kernel_size", "sync_queue_size", "sync_slop_sec",
        ]
        defaults = [
            "/camera/image_raw", "/camera/depth/image_raw", "/camera/camera_info",
            "camera_optical_frame", "map", 25, 90, 80, 255, 60, 255,
            30.0, 50000.0, 2.0, 0.15, 8.0, 5, 10, 0.10,
        ]
        for name, default in zip(names, defaults):
            self.declare_parameter(name, default)

        self.image_topic = self.get_parameter("image_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.map_frame = self.get_parameter("map_frame").value

        self.h_min = int(self.get_parameter("h_min").value)
        self.h_max = int(self.get_parameter("h_max").value)
        self.s_min = int(self.get_parameter("s_min").value)
        self.s_max = int(self.get_parameter("s_max").value)
        self.v_min = int(self.get_parameter("v_min").value)
        self.v_max = int(self.get_parameter("v_max").value)

        self.min_area = float(self.get_parameter("min_area").value)
        self.max_area = float(self.get_parameter("max_area").value)
        self.min_radius_px = float(self.get_parameter("min_radius_px").value)
        self.min_depth = float(self.get_parameter("min_depth_m").value)
        self.max_depth = float(self.get_parameter("max_depth_m").value)
        self.kernel_size = int(self.get_parameter("morphology_kernel_size").value)

        self.camera_info = None

    # --------------- SUBSCRIBERS ------------------
        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10,
        )

        rgb_sub = message_filters.Subscriber(self, Image, self.image_topic)
        depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub],
            queue_size=int(self.get_parameter("sync_queue_size").value),
            slop=float(self.get_parameter("sync_slop_sec").value),
        )
        self.sync.registerCallback(self.image_callback)

    # # ------------- PUBLISHERS -----------------

        self.camera_pub = self.create_publisher(
            PoseArray, "/detected_balls_camera", 10
        )
        self.map_pub = self.create_publisher(
            PoseArray, "/detected_balls", 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, "/detected_ball_markers", 10
        )
        self.debug_pub = self.create_publisher(
            Image, "/ball_image", 10
        )
        self.mask_pub = self.create_publisher(
            Image, "/ball_mask", 10
        )

        self.get_logger().info("Ball detector started")

    def camera_info_callback(self, msg):
        self.camera_info = msg

    # Kamera 2 boyutlu algılıyor, bunu derinlik ve piksel bilgileri sayesinde
    # 3 boyutlu haritamıza, gerçeğe dönüştürüyoruz.
    # u: satır, v: sütun, z: derinlik  
    @staticmethod
    def project_pixel_to_3d(u, v, depth_m, fx, fy, cx, cy):
        if depth_m <= 0:
            raise ValueError("Depth must be positive")
        return (
            (u - cx) * depth_m / fx,
            (v - cy) * depth_m / fy,
            depth_m,
        )

    @staticmethod
    def depth_at_pixel(depth_image, u, v):
        h, w = depth_image.shape[:2]
        if not (0 <= u < w and 0 <= v < h):
            return None

        r = 2
        patch = depth_image[
            max(0, v-r):min(h, v+r+1),
            max(0, u-r):min(w, u+r+1),
        ].astype(np.float32)
        valid = patch[np.isfinite(patch) & (patch > 0.0)]
        if valid.size == 0:
            return None

        depth = float(np.median(valid))
        if depth_image.dtype == np.uint16:
            depth *= 0.001
        return depth

    def image_callback(self, rgb_msg, depth_msg):
        if self.camera_info is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().error(f"cv_bridge conversion failed: {exc}")
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array([self.h_min, self.s_min, self.v_min], dtype=np.uint8)
        upper = np.array([self.h_max, self.s_max, self.v_max], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

        kernel = np.ones((self.kernel_size, self.kernel_size), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        fx = float(self.camera_info.k[0])
        fy = float(self.camera_info.k[4])
        cx = float(self.camera_info.k[2])
        cy = float(self.camera_info.k[5])

        camera_poses = PoseArray()
        camera_poses.header.stamp = rgb_msg.header.stamp
        camera_poses.header.frame_id = self.camera_frame

        map_poses = PoseArray()
        map_poses.header.stamp = rgb_msg.header.stamp
        map_poses.header.frame_id = self.map_frame

        markers = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        transform = None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.camera_frame,
                rclpy.time.Time.from_msg(rgb_msg.header.stamp),
            )
        except Exception:
            pass

        marker_id = 0

        for contour in contours:
            area = cv2.contourArea(contour)
            if not self.min_area <= area <= self.max_area:
                continue

            perimeter = cv2.arcLength(contour, True)
            if perimeter <= 0.0:
                continue

            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            if circularity < 0.45:
                continue

            (u, v), radius = cv2.minEnclosingCircle(contour)
            if radius < self.min_radius_px:
                continue

            ui, vi = int(round(u)), int(round(v))
            depth_m = self.depth_at_pixel(depth, ui, vi)
            if depth_m is None or not self.min_depth <= depth_m <= self.max_depth:
                continue

            try:
                x, y, z = self.project_pixel_to_3d(
                    u, v, depth_m, fx, fy, cx, cy
                )
            except ValueError:
                continue

            p = Pose()
            p.position.x = x
            p.position.y = y
            p.position.z = z
            p.orientation.w = 1.0
            camera_poses.poses.append(p)

            cv2.circle(frame, (ui, vi), int(round(radius)), (0, 255, 0), 2)
            cv2.circle(frame, (ui, vi), 3, (0, 0, 255), -1)
            cv2.putText(
                frame,
                f"{depth_m:.2f}m",
                (ui + 5, vi - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

            if transform is None:
                continue

            point = PointStamped()
            point.header = camera_poses.header
            point.point.x = x
            point.point.y = y
            point.point.z = z

            try:
                mapped = do_transform_point(point, transform)
            except Exception:
                continue

            mp = Pose()
            mp.position = mapped.point
            mp.orientation.w = 1.0
            map_poses.poses.append(mp)

            marker = Marker()
            marker.header = map_poses.header
            marker.ns = "tennis_balls"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = mp
            marker.scale.x = 0.12
            marker.scale.y = 0.12
            marker.scale.z = 0.12
            marker.color.a = 0.9
            marker.color.r = 0.45
            marker.color.g = 0.80
            marker.color.b = 0.05
            markers.markers.append(marker)
            marker_id += 1

        self.camera_pub.publish(camera_poses)
        if map_poses.poses:
            self.map_pub.publish(map_poses)
        self.marker_pub.publish(markers)

        debug = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        debug.header = rgb_msg.header
        self.debug_pub.publish(debug)

        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
        mask_msg.header = rgb_msg.header
        self.mask_pub.publish(mask_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BallDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
