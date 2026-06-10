import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import CameraInfo
from dobot_msgs_fb.srv import ApproachObject # changed
from geometry_msgs.msg import Pose
import random
from ament_index_python.packages import get_package_share_directory
import os

class TestDetekcija(Node):
    def __init__(self):
        super().__init__('Test_detekcija_topic')

        self.publisher_ = self.create_publisher(Image, 'detected_image', 10)
        self.bridge = CvBridge()

        self.rgb_sub = Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        # Parametra za radij (v metrih) - MIN in MAX
        self.min_radius = 0.3
        self.max_radius = 0.8

        self.fx = None
        self.fy = None
        self.ppx = None
        self.ppy = None

        #calib_path = '/home/matija/workspace/calibration.yaml'
        pkg_share = os.path.join(get_package_share_directory("perception"))
        config_file_path = os.path.join(pkg_share, "config", "calibration.yaml")
        #print(config_file_path)
        fs = cv2.FileStorage(config_file_path, cv2.FILE_STORAGE_READ)
        self.T_cam2base = fs.getNode('T').mat()
        fs.release()
        self.get_logger().info(f"Kalibracija naložena:\n{self.T_cam2base}")

        self.zelena_l = np.array([65, 45, 57])
        self.zelena_h = np.array([83, 240, 175])
        self.rumena_l = np.array([2, 113, 100])
        self.rumena_h = np.array([10, 255, 205])
        self.min_weed_area = 500
        self.max_weed_area = 10000

        self.vse_rumene = []
        self.vse_zelene = []  

        self.marker_pub = self.create_publisher(MarkerArray, 'detected_markers', 10)

        self.srv = self.create_service(ApproachObject, 'approach_object', self.service_callback)
        self.get_logger().info("Service 'approach_object' pripravljen.")

        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.05
        )
        self.sync.registerCallback(self.camera_callback)
        self.get_logger().info(f"Node zagnan, čakam na kamero... Radij za detekcije: min={self.min_radius}m, max={self.max_radius}m")

    def service_callback(self, request, response):
        # id=0 -> plevel (rumena), id=1 -> spargelj (zelena)
        if request.object_id == 0:
            if not self.vse_rumene:
                response.success = False
                response.message = "Ni zaznane rumene (plevel)"
                return response
            
            # Filtriramo detekcije znotraj radija (med min_radius in max_radius)
            detekcije_v_radiju = []
            for (x, y, z) in self.vse_rumene:
                razdalja = np.sqrt(x**2 + y**2 + z**2)  # Razdalja od baze (0,0,0)
                if self.min_radius <= razdalja <= self.max_radius:
                    detekcije_v_radiju.append((x, y, z, razdalja))
            
            if not detekcije_v_radiju:
                response.success = False
                response.message = f"Ni rumenih detekcij znotraj radija [{self.min_radius}m, {self.max_radius}m]"
                return response
            
            # Naključno izberemo eno detekcijo
            x, y, z, razdalja = random.choice(detekcije_v_radiju)
            response.radius = 0.01
            response.height = 0.05
            self.get_logger().info(f"Izbrana naključna rumena detekcija na razdalji {razdalja:.3f}m")

        elif request.object_id == 1:
            #print(f"vse zelene :{self.vse_zelene}")
            if not self.vse_zelene:
                response.success = False
                response.message = "Ni zaznane zelene (spargelj)"
                return response
            
            # Filtriramo detekcije znotraj radija (med min_radius in max_radius)
            detekcije_v_radiju = []
            for (x, y, z) in self.vse_zelene:
                razdalja = np.sqrt(x**2 + y**2 + z**2)  # Razdalja od baze (0,0,0)
                if self.min_radius <= razdalja <= self.max_radius:
                    detekcije_v_radiju.append((x, y, z, razdalja))
            
            if not detekcije_v_radiju:
                response.success = False
                response.message = f"Ni zelenih detekcij znotraj radija [{self.min_radius}m, {self.max_radius}m]"
                return response
            
            # Naključno izberemo eno detekcijo
            x, y, z, razdalja = random.choice(detekcije_v_radiju)
            response.radius = 0.005
            response.height = 0.2
            self.get_logger().info(f"Izbrana naključna zelena detekcija na razdalji {razdalja:.3f}m")

        else:
            response.success = False
            response.message = f"Neznan object_id: {request.object_id}"
            return response

        response.success = True
        response.message = f"OK, object_id={request.object_id}"
        response.x = float(x)
        response.y = float(y)
        response.z = float(z)
        self.get_logger().info(f"Service odgovor: id={request.object_id} x={x:.3f} y={y:.3f} z={z:.3f}")
        return response

    def camera_callback(self, rgb_msg, depth_msg):
        img = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        rumene_lokacije, _ = self.zaznaj_rumeno(img, depth)
        zelene_lokacije, _ = self.zaznaj_zeleno(img, depth, rumene_lokacije)

        # Počistimo stare sezname in dodamo nove detekcije
        self.vse_rumene = []
        self.vse_zelene = []
        
        # Pretvorimo lokacije v 3D koordinate
        for (cx, cy, w, h, depth_m) in rumene_lokacije:
            x_cam, y_cam, z_cam = self.pixel_to_3d(cx, cy, depth_m)
            x, y, z = self.cam_to_base(x_cam, y_cam, z_cam)
            self.vse_rumene.append((x, y, z))
        
        for (cx, cy, w, h, depth_m) in zelene_lokacije:
            x_cam, y_cam, z_cam = self.pixel_to_3d(cx, cy, depth_m)
            x, y, z = self.cam_to_base(x_cam, y_cam, z_cam)
            self.vse_zelene.append((x, y, z))

        anotated_img = self.draw_results(img, rumene_lokacije, zelene_lokacije)
        
        # Izpišemo statistiko znotraj radija
        rumene_v_radiju = sum(1 for (x,y,z) in self.vse_rumene 
                              if self.min_radius <= np.sqrt(x**2 + y**2 + z**2) <= self.max_radius)
        zelene_v_radiju = sum(1 for (x,y,z) in self.vse_zelene 
                              if self.min_radius <= np.sqrt(x**2 + y**2 + z**2) <= self.max_radius)
        
        self.get_logger().info(f"rumene: {len(self.vse_rumene)} (v radiju [{self.min_radius}-{self.max_radius}m]: {rumene_v_radiju}) | "
                               f"zelene: {len(self.vse_zelene)} (v radiju: {zelene_v_radiju})")

        msg = self.bridge.cv2_to_imgmsg(anotated_img, encoding='bgr8')
        self.publisher_.publish(msg)
        self.publish_markers(rumene_lokacije, zelene_lokacije)

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

    def zaznaj_rumeno(self, img, depth):
        masked = img.copy()
        hsv = cv2.cvtColor(masked, cv2.COLOR_BGR2HSV)
        rumena_maska = cv2.inRange(hsv, self.rumena_l, self.rumena_h)
        kernel = np.ones((5, 5), np.uint8)
        rumena_maska = cv2.morphologyEx(rumena_maska, cv2.MORPH_OPEN, kernel)
        rumena_maska = cv2.morphologyEx(rumena_maska, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(rumena_maska, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        lokacije_rumene = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if not (self.min_weed_area < area < self.max_weed_area):
                continue
            x, y, w, h = cv2.boundingRect(cnt)
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
            lokacije_rumene.append((cx, cy, w, h, depth_m))
        return lokacije_rumene, rumena_maska

    def zaznaj_zeleno(self, img, depth, lokacije_rumene):
        masked = img.copy()
        for (cx, cy, w, h, depth_m) in lokacije_rumene:
            x1 = cx - w // 2
            y1 = cy - h // 2
            cv2.rectangle(masked, (x1, y1), (x1 + w, y1 + h), (0, 0, 0), -1)
        hsv = cv2.cvtColor(masked, cv2.COLOR_BGR2HSV)
        zelena_maska = cv2.inRange(hsv, self.zelena_l, self.zelena_h)
        kernel = np.ones((5, 5), np.uint8)
        zelena_maska = cv2.morphologyEx(zelena_maska, cv2.MORPH_OPEN, kernel)
        zelena_maska = cv2.morphologyEx(zelena_maska, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(zelena_maska, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        lokacije_zelene = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if not (self.min_weed_area < area < self.max_weed_area):
                continue
            x, y, w, h = cv2.boundingRect(cnt)
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
            lokacije_zelene.append((cx, cy, w, h, depth_m))
        return lokacije_zelene, zelena_maska

    def draw_results(self, img, rumene_lokacije, zelene_lokacije):
        for (cx, cy, w, h, depth_m) in rumene_lokacije:
            x1 = cx - w // 2
            y1 = cy - h // 2
            cv2.rectangle(img, (x1, y1), (x1 + w, y1 + h), (0, 255, 255), 2)
            cv2.circle(img, (cx, cy), 5, (0, 255, 255), -1)
            cv2.putText(img, f"rumena {depth_m:.2f}m", (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        for (cx, cy, w, h, depth_m) in zelene_lokacije:
            x1 = cx - w // 2
            y1 = cy - h // 2
            cv2.rectangle(img, (x1, y1), (x1 + w, y1 + h), (0, 255, 0), 2)
            cv2.circle(img, (cx, cy), 5, (0, 255, 0), -1)
            cv2.putText(img, f"zelena {depth_m:.2f}m", (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        return img

    def publish_markers(self, rumene_lokacije, zelene_lokacije):
        if self.fx is None or self.T_cam2base is None:
            return

        marker_array = MarkerArray()
        marker_id = 0

        for (cx, cy, w, h, depth_m) in rumene_lokacije:
            x_cam, y_cam, z_cam = self.pixel_to_3d(cx, cy, depth_m)
            x, y, z = self.cam_to_base(x_cam, y_cam, z_cam)

            marker = Marker()
            marker.header.frame_id = "base_link"
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

        for (cx, cy, w, h, depth_m) in zelene_lokacije:
            x_cam, y_cam, z_cam = self.pixel_to_3d(cx, cy, depth_m)
            x, y, z = self.cam_to_base(x_cam, y_cam, z_cam)

            marker = Marker()
            marker.header.frame_id = "base_link"
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
    node = TestDetekcija()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()