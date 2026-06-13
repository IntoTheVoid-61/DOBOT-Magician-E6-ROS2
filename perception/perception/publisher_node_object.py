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
from dobot_msgs_fb.srv import ApproachObject
from geometry_msgs.msg import Pose
import random


class YoloImagePublisher(Node):
    def __init__(self):
        super().__init__("yolo_image_publisher")

        self.publisher_ = self.create_publisher(Image, "detected_image", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "detected_markers", 10)
        self.declare_parameter("debug_param", False)
        self.debug_param = (
            self.get_parameter("debug_param").get_parameter_value()._bool_value
        )
        # HSV parametri za zeleno
        self.declare_parameter("lower_green", [42, 33, 0]) # H-min, S-min, V_min
        self.declare_parameter("upper_green", [84, 255, 152]) # H_max S_max V_max

        lower = (
            self.get_parameter("lower_green").get_parameter_value().integer_array_value
        )
        upper = (
            self.get_parameter("upper_green").get_parameter_value().integer_array_value
        )

        self.lower_green = np.array(lower).astype(np.uint8)
        self.upper_green = np.array(upper).astype(np.uint8)

        self.bridge = CvBridge()

        pkg_path = get_package_share_directory("perception")
        model_path = os.path.join(pkg_path, 'models',"best.pt")
        self.model = YOLO(model_path)

        self.min_weed_area = 2500
        self.max_weed_area = 15000

        # radij pobiranja
        self.min_radius = 0.3
        self.max_radius = 0.8

        self.fx = None
        self.fy = None
        self.ppx = None
        self.ppy = None

        pkg_share = os.path.join(get_package_share_directory("perception"))
        config_file_path = os.path.join(pkg_share, "config", "calibration.yaml")
        fs = cv2.FileStorage(config_file_path, cv2.FILE_STORAGE_READ)
        self.T_cam2base = fs.getNode("T").mat()
        fs.release()
        self.get_logger().info(f"Kalibracija naložena:\n{self.T_cam2base}")

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/camera/camera/color/camera_info",
            self.camera_info_callback,
            10,
        )

        self.srv = self.create_service(
            ApproachObject, "approach_object", self.service_callback
        )
        self.get_logger().info("Service 'approach_object' pripravljen.")

        self.rgb_sub = Subscriber(self, Image, "/camera/camera/color/image_raw")
        self.depth_sub = Subscriber(
            self, Image, "/camera/camera/aligned_depth_to_color/image_raw"
        )

        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=10, slop=0.05
        )
        self.sync.registerCallback(self.camera_callback)

        self.get_logger().info(
            f"Node zagnan, čakam na kamero... Radij za detekcije: min={self.min_radius}m, max={self.max_radius}m"
        )

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

    def cam_to_base(self, x_cam, y_cam, z_cam):
        p_cam = np.array([x_cam, y_cam, z_cam, 1.0])
        p_base = self.T_cam2base @ p_cam
        return p_base[0], p_base[1], p_base[2]

    def service_callback(self, request, response):
        # id=0 -> plevel (rumena), id=1 -> spargelj (zelena)
        if request.object_id == 0:
            if not self.all_weeds:
                response.success = False
                response.message = "Ni zaznane rumene (plevel)"
                return response

            # znotraj radija
            detekcije_v_radiju = []
            for x, y, z in self.all_weeds:
                razdalja = np.sqrt(x**2 + y**2 + z**2)  # Razdalja od baze (0,0,0)
                if self.min_radius <= razdalja <= self.max_radius:
                    detekcije_v_radiju.append((x, y, z, razdalja))

            if not detekcije_v_radiju:
                response.success = False
                response.message = f"Ni rumenih detekcij znotraj radija [{self.min_radius}m, {self.max_radius}m]"
                return response

            # rndom choice
            x, y, z, razdalja = random.choice(detekcije_v_radiju)
            response.radius = 0.01
            response.height = 0.05
            self.get_logger().info(
                f"Izbrana naključna rumena detekcija na razdalji {razdalja:.3f}m"
            )

        elif request.object_id == 1:
            # print(f"vse zelene :{self.vse_zelene}")
            if not self.all_asparagous:
                response.success = False
                response.message = "Ni zaznane zelene (spargelj)"
                return response

            detekcije_v_radiju = []
            for x, y, z in self.all_asparagous:
                razdalja = np.sqrt(x**2 + y**2 + z**2)  # Razdalja od baze (0,0,0)
                if self.min_radius <= razdalja <= self.max_radius:
                    detekcije_v_radiju.append((x, y, z, razdalja))

            if not detekcije_v_radiju:
                response.success = False
                response.message = f"Ni zelenih detekcij znotraj radija [{self.min_radius}m, {self.max_radius}m]"
                return response

            x, y, z, razdalja = random.choice(detekcije_v_radiju)
            response.radius = 0.005
            response.height = 0.2
            self.get_logger().info(
                f"Izbrana naključna zelena detekcija na razdalji {razdalja:.3f}m"
            )

        else:
            response.success = False
            response.message = f"Neznan object_id: {request.object_id}"
            return response

        response.success = True
        response.message = f"OK, object_id={request.object_id}"
        response.x = float(x)
        response.y = float(y)
        response.z = float(z)
        self.get_logger().info(
            f"Service odgovor: id={request.object_id} x={x:.3f} y={y:.3f} z={z:.3f}"
        )
        return response

    def camera_callback(self, rgb_msg, depth_msg):

        self.all_asparagous = []
        self.all_weeds = []

        if self.fx is None:
            self.get_logger().warn("Camera info še ni prišel, preskakujem frame...")
            return

        img = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

        results = self.model(img)[0]
        weed_locations, _ = self.detect_weeds(img, depth, results)

        # sparglji iz segmentacijske maske
        asparagus_locations = []
        if results.masks is not None:
            for mask in results.masks.xy:
                # dno sparglja
                lowest = max(mask, key=lambda p: p[1])
                cx = int(lowest[0])
                cy = int(lowest[1])

                # zaradi nekih errorjev pri indeksu na globinski sliki
                if cy >= depth.shape[0] or cx >= depth.shape[1] or cy < 0 or cx < 0:
                    continue
                depth_value = depth[cy, cx]
                if depth_value == 0:
                    continue
                depth_m = float(depth_value) / 1000.0
                asparagus_locations.append((cx, cy, depth_m))

        for cx, cy, depth_m in asparagus_locations:
            x_cam, y_cam, z_cam = self.pixel_to_3d(cx, cy, depth_m)
            x, y, z = self.cam_to_base(x_cam, y_cam, z_cam)
            self.all_asparagous.append((x, y, z))

        for cx, cy, w, h, depth_m in weed_locations:
            x_cam, y_cam, z_cam = self.pixel_to_3d(cx, cy, depth_m)
            x, y, z = self.cam_to_base(x_cam, y_cam, z_cam)
            self.all_weeds.append((x, y, z))

        spargljji_v_radiju = sum(
            1
            for (x, y, z) in self.all_asparagous
            if self.min_radius <= np.sqrt(x**2 + y**2 + z**2) <= self.max_radius
        )
        plevel_v_radiju = sum(
            1
            for (x, y, z) in self.all_weeds
            if self.min_radius <= np.sqrt(x**2 + y**2 + z**2) <= self.max_radius
        )
        self.get_logger().info(
            f"rumene: {len(self.all_weeds)} (v radiju [{self.min_radius}-{self.max_radius}m]: {plevel_v_radiju}) | "
            f"zelene: {len(self.all_asparagous)} (v radiju: {spargljji_v_radiju})"
        )

        annotated_img = self.draw_results(img, results, weed_locations)

        msg = self.bridge.cv2_to_imgmsg(annotated_img, encoding="bgr8")
        self.publisher_.publish(msg)

        self.publish_markers(weed_locations, asparagus_locations)

    def detect_weeds(self, img, depth, yolo_results):
        masked = img.copy()

        for box in yolo_results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cv2.rectangle(masked, (x1, y1), (x2, y2), (0, 0, 0), -1)

        height = masked.shape[0]
        masked[0 : height // 4, :] = (0, 0, 0)

        hsv = cv2.cvtColor(masked, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(hsv, self.lower_green, self.upper_green)

        kernel = np.ones((5, 5), np.uint8)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if self.debug_param == True:
            cv2.imshow("yolo", masked)
            cv2.imshow("green", green_mask)
            cv2.waitKey(1)

        weed_locations = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if not (self.min_weed_area < area < self.max_weed_area):
                continue

            x, y, w, h = cv2.boundingRect(cnt)

            aspect_ratio = max(w, h) / min(w, h)
            if aspect_ratio == 0:
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
                cv2.polylines(
                    annotated, [pts], isClosed=True, color=(0, 255, 0), thickness=2
                )

                lowest = max(mask, key=lambda p: p[1])
                cv2.circle(
                    annotated, (int(lowest[0]), int(lowest[1])), 8, (0, 255, 0), -1
                )

        # Plevel
        for cx, cy, w, h, depth_m in weed_locations:
            x1 = cx - w // 2
            y1 = cy - h // 2
            cv2.rectangle(annotated, (x1, y1), (x1 + w, y1 + h), (0, 0, 255), 2)
            cv2.circle(annotated, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(
                annotated,
                f"plevel {depth_m:.2f}m",
                (x1, y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 0, 255),
                1,
            )
            area = w * h
            aspect_ratio = max(w, h) / min(w, h) if min(w, h) > 0 else 0
            cv2.putText(
                annotated,
                f"area={area} ar={aspect_ratio:.1f}",
                (x1, y1 - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 0, 255),
                1,
            )

        # cv2.imshow("Anotated", annotated)
        # cv2.waitKey(1)
        return annotated

    def publish_markers(self, weed_locations, asparagus_locations):
        if self.fx is None or self.T_cam2base is None:
            return

        marker_array = MarkerArray()
        marker_id = 0

        for cx, cy, depth_m in asparagus_locations:
            x_cam, y_cam, z_cam = self.pixel_to_3d(cx, cy, depth_m)
            x, y, z = self.cam_to_base(x_cam, y_cam, z_cam)

            marker = Marker()
            marker.header.frame_id = "camera_link"  # popravi na base_link !!
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.02
            marker.scale.y = 0.02
            marker.scale.z = 0.02
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime.sec = 1
            marker_array.markers.append(marker)
            marker_id += 1

        for cx, cy, w, h, depth_m in weed_locations:
            x_cam, y_cam, z_cam = self.pixel_to_3d(cx, cy, depth_m)
            x, y, z = self.cam_to_base(x_cam, y_cam, z_cam)

            marker = Marker()
            marker.header.frame_id = "camera_link"  # popravi na base_link !!
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
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


if __name__ == "__main__":
    main()
