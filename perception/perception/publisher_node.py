import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from ultralytics import YOLO
from message_filters import ApproximateTimeSynchronizer, Subscriber
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory


class YoloImagePublisher(Node):
    def __init__(self):
        super().__init__('yolo_image_publisher')

        self.publisher_ = self.create_publisher(Image, 'detected_image', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'detected_markers', 10)
        self.bridge = CvBridge()

        pkg_path = get_package_share_directory('perception')
        model_path = os.path.join(pkg_path, "best.pt")
        self.model = YOLO(model_path)

        self.lower_green = np.array([30, 17, 60])
        self.upper_green = np.array([100, 135, 200])

        self.min_weed_area = 300
        self.max_weed_area = 5000

        self.fx = None
        self.fy = None
        self.ppx = None
        self.ppy = None

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        self.rgb_sub = Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')

        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.05
        )
        self.sync.registerCallback(self.camera_callback)

        self.get_logger().info("Node zagnan, čakam na kamero...")

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.ppx = msg.k[2]
        self.ppy = msg.k[5]

    def pixel_to_3d(self, cx, cy, depth_m):
        x = (cx - self.ppx) * depth_m / self.fx
        y = (cy - self.ppy) * depth_m / self.fy
        z = depth_m
        return x, y, z

    def camera_callback(self, rgb_msg, depth_msg):
        img = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        results = self.model(img)[0]
        weed_locations, _ = self.detect_weeds(img, depth, results)

        # Šparglji iz segmentacijske maske
        asparagus_locations = []
        if results.masks is not None:
            for mask in results.masks.xy:
                # Najnižja točka maske = dno špargelja
                lowest = max(mask, key=lambda p: p[1])
                cx = int(lowest[0])
                cy = int(lowest[1])

                depth_value = depth[cy, cx]
                if depth_value == 0:
                    continue
                depth_m = float(depth_value) / 1000.0
                asparagus_locations.append((cx, cy, depth_m))

        annotated_img = self.draw_results(img, results, weed_locations)

        self.get_logger().info(
            f"Sparglji: {len(asparagus_locations)} | Plevel: {len(weed_locations)}"
        )

        msg = self.bridge.cv2_to_imgmsg(annotated_img, encoding='bgr8')
        self.publisher_.publish(msg)

        self.publish_markers(weed_locations, asparagus_locations)

    def detect_weeds(self, img, depth, yolo_results):
        masked = img.copy()

        for box in yolo_results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cv2.rectangle(masked, (x1, y1), (x2, y2), (0, 0, 0), -1)

        height = masked.shape[0]
        masked[0:height//4, :] = (0, 0, 0)

        hsv = cv2.cvtColor(masked, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(hsv, self.lower_green, self.upper_green)

        kernel = np.ones((5, 5), np.uint8)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        weed_locations = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if not (self.min_weed_area < area < self.max_weed_area):
                continue

            x, y, w, h = cv2.boundingRect(cnt)

            aspect_ratio = max(w, h) / min(w, h)
            if aspect_ratio > 3.0:
                continue

            hull = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            solidity = area / hull_area if hull_area > 0 else 0
            if solidity < 0.5:
                continue

            cx = x + w // 2
            cy = y + h // 2

            depth_value = depth[cy, cx]
            if depth_value == 0:
                continue
            depth_m = float(depth_value) / 1000.0

            weed_locations.append((cx, cy, w, h, depth_m))

        return weed_locations, green_mask

    def draw_results(self, img, yolo_results, weed_locations):
        annotated = yolo_results.plot()

        # najnizja tocka spargljev
        if yolo_results.masks is not None:
            for mask in yolo_results.masks.xy:
                pts = np.array(mask, dtype=np.int32)
                cv2.polylines(annotated, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

                lowest = max(mask, key=lambda p: p[1])
                cv2.circle(annotated, (int(lowest[0]), int(lowest[1])), 8, (0, 255, 0), -1)

        # Plevel
        for (cx, cy, w, h, depth_m) in weed_locations:
            x1 = cx - w // 2
            y1 = cy - h // 2
            cv2.rectangle(annotated, (x1, y1), (x1 + w, y1 + h), (0, 0, 255), 2)
            cv2.circle(annotated, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(annotated, f"plevel {depth_m:.2f}m", (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        cv2.imshow("Anotated", annotated)
        cv2.waitKey(1)
        return annotated

    def publish_markers(self, weed_locations, asparagus_locations):
        if self.fx is None:
            return

        marker_array = MarkerArray()
        marker_id = 0

        # Plevel - rdeca
        for (cx, cy, w, h, depth_m) in weed_locations:
            marker = Marker()
            marker.header.frame_id = "camera_color_optical_frame"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            x, y, z = self.pixel_to_3d(cx, cy, depth_m)
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.01
            marker.scale.y = 0.01
            marker.scale.z = 0.01

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker.lifetime.sec = 1
            marker_array.markers.append(marker)
            marker_id += 1

        # Šparglji - zelena
        for (cx, cy, depth_m) in asparagus_locations:
            marker = Marker()
            marker.header.frame_id = "camera_color_optical_frame"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            x, y, z = self.pixel_to_3d(cx, cy, depth_m)
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.02
            marker.scale.y = 0.02
            marker.scale.z = 0.02

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker.lifetime.sec = 1
            marker_array.markers.append(marker)
            marker_id += 1

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = YoloImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
