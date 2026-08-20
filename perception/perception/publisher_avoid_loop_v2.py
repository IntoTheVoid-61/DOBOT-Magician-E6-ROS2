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
from dobot_msgs_fb.srv import RemoveWeeds
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
        self.declare_parameter("lower_green", [7, 79, 0]) # H-min, S-min, V_min
        self.declare_parameter("upper_green", [61, 255, 255]) # H_max S_max V_max

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

        # Enako kot v preverjenem HSV tunerju: ohranimo tudi manjse dele
        # listov, nato pa bliznje dele iste rastline zdruzimo v eno zaznavo.
        self.min_weed_area = 5000
        self.max_weed_area = 20000 # ohranjeno zaradi zdruzljivosti
        self.weed_merge_distance_px = 20

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
            RemoveWeeds, "remove_weeds", self.service_callback
        )
        self.get_logger().info("Service 'remove_weed' pripravljen.")

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

    def service_callback(self,request,response):
        
        if len(self.all_weeds) > 0:
            response.response = True

            weeds_flat = [] # flattened list

            for x,y,z in self.all_weeds:
                #y = y + 0.03 # quick fix
                weeds_flat.extend([x, y, z])

            response.weeds = weeds_flat


            #weed = random.choice(self.all_weeds)
            #wx, wy, wz = weed
            #response.weed_x = float(wx)
            #response.weed_y = float(wy)
            #response.weed_z = float(wz)
            #response.weed_height = 0.025
            #response.weed_radius = 0.02

            asparagus_flat = [] # flattened list

            for x,y,z in self.all_asparagous:
                asparagus_height = 0.15
                asparagus_radius = 0.005

                asparagus_flat.extend([x, y, z, asparagus_height, asparagus_radius])

            response.asparagus = asparagus_flat

            #response.message = (
            #    f"weed selected: 1 | asparagus detected: {len(self.all_asparagous)}"
            #)

            return response

        
        # if weeds not detected response->false
        else:
            response.response = False
            response.message = "Did not detect weeds"
            response.asparagus = []
            response.weeds = []
            return response



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
        #self.get_logger().info(
        #    f"rumene: {len(self.all_weeds)} (v radiju [{self.min_radius}-{self.max_radius}m]: {plevel_v_radiju}) | "
        #    f"zelene: {len(self.all_asparagous)} (v radiju: {spargljji_v_radiju})"
        #)

        annotated_img = self.draw_results(img, results, weed_locations)

        msg = self.bridge.cv2_to_imgmsg(annotated_img, encoding="bgr8")
        self.publisher_.publish(msg)

        self.publish_markers(weed_locations, asparagus_locations)

    def create_asparagus_mask(self, image_shape, yolo_results):
        """Maska dejanskih YOLO segmentov spargljev, ne celih okvirjev."""
        asparagus_mask = np.zeros(image_shape[:2], dtype=np.uint8)

        if yolo_results.masks is None:
            return asparagus_mask

        for polygon in yolo_results.masks.xy:
            points = np.rint(polygon).astype(np.int32)
            if len(points) >= 3:
                cv2.fillPoly(asparagus_mask, [points], 255)

        return asparagus_mask

    def clean_small_segments(self, mask):
        """Enako ciscenje majhnih lis kot v HSV tunerju."""
        cleaned = np.zeros_like(mask)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        for contour in contours:
            if cv2.contourArea(contour) >= self.min_weed_area:
                cv2.drawContours(cleaned, [contour], -1, 255, thickness=-1)

        return cleaned

    @staticmethod
    def rectangle_distance(rect_a, rect_b):
        """Najkrajsa razdalja med pravokotnikoma v slikovnih pikah."""
        ax, ay, aw, ah = rect_a
        bx, by, bw, bh = rect_b

        dx = max(ax - (bx + bw), bx - (ax + aw), 0)
        dy = max(ay - (by + bh), by - (ay + ah), 0)
        return float(np.hypot(dx, dy))

    def group_weed_contours(self, contours):
        """Bliznje dele listov zdruzi v eno fizicno rastlino."""
        if not contours:
            return []

        rectangles = [cv2.boundingRect(contour) for contour in contours]
        parents = list(range(len(contours)))

        def find(index):
            while parents[index] != index:
                parents[index] = parents[parents[index]]
                index = parents[index]
            return index

        def union(first, second):
            first_root = find(first)
            second_root = find(second)
            if first_root != second_root:
                parents[second_root] = first_root

        for first in range(len(contours)):
            for second in range(first + 1, len(contours)):
                distance = self.rectangle_distance(
                    rectangles[first], rectangles[second]
                )
                if distance <= self.weed_merge_distance_px:
                    union(first, second)

        groups = {}
        for index, contour in enumerate(contours):
            groups.setdefault(find(index), []).append(contour)

        return list(groups.values())

    @staticmethod
    def median_depth_from_mask(depth, object_mask):
        """Vrne robustno globino iz vseh veljavnih pikslov rastline."""
        values = depth[object_mask > 0]
        if values.size == 0:
            return None

        values = values[np.isfinite(values)]
        values = values[values > 0]
        if values.size == 0:
            return None

        median_depth = float(np.median(values))

        # RealSense praviloma objavlja uint16 v milimetrih. Podprta je tudi
        # morebitna plavajoca globinska slika, podana neposredno v metrih.
        if np.issubdtype(depth.dtype, np.floating) and median_depth < 20.0:
            return median_depth

        return median_depth / 1000.0

    def detect_weeds(self, img, depth, yolo_results):
        # Segmentacijski postopek je prenesen iz realsense_hsv_weed_v2:
        # originalna RGB slika -> HSV -> inRange -> open/close 3x3 ->
        # odstranitev samo zelo majhnih izoliranih lis.
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(hsv, self.lower_green, self.upper_green)

        kernel = np.ones((3, 3), np.uint8)
        green_mask = cv2.morphologyEx(
            green_mask, cv2.MORPH_OPEN, kernel
        )
        green_mask = cv2.morphologyEx(
            green_mask, cv2.MORPH_CLOSE, kernel
        )

        # Sparglje moramo se vedno izlociti, vendar odstranimo samo dejanske
        # segmentacijske maske. Stari pravokotniki so brisali tudi bliznji plevel.
        asparagus_mask = self.create_asparagus_mask(img.shape, yolo_results)
        green_mask[asparagus_mask > 0] = 0
        green_mask = self.clean_small_segments(green_mask)

        contours, _ = cv2.findContours(
            green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        contours = [
            contour
            for contour in contours
            if cv2.contourArea(contour) >= self.min_weed_area
        ]

        if self.debug_param == True:
            cv2.imshow("green", green_mask)
            cv2.waitKey(1)

        weed_locations = []

        # Tuner obrobi posamezne zelene dele. Tukaj bliznje dele dodatno
        # zdruzimo, da en plevel ne postane vec koordinat za remove_weeds.
        for group in self.group_weed_contours(contours):
            group_mask = np.zeros_like(green_mask)
            cv2.drawContours(group_mask, group, -1, 255, thickness=-1)

            points = np.vstack(group)
            x, y, w, h = cv2.boundingRect(points)
            cx = x + w // 2
            cy = y + h // 2

            depth_m = self.median_depth_from_mask(depth, group_mask)
            if depth_m is None:
                continue

            # Popolnoma enaka oblika izhoda kot prej, zato preostali program
            # in vsi drugi ROS paketi ostanejo nespremenjeni.
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