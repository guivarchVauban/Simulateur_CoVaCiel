from vehicle import Driver
from controller import Lidar, GPS, InertialUnit
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import time
import numpy as np
import math
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
 

print("=== TT02 CONTROLLER STARTED ===", flush=True)

# --- Webots Driver ---
driver = Driver()
basicTimeStep = int(driver.getBasicTimeStep())
sensorTimeStep = 4 * basicTimeStep

# --- Lidar ---
lidar = driver.getDevice("RpLidarA2")
gps = driver.getDevice("gps")
imu = driver.getDevice("imu")

assert lidar is not None, "Lidar device not found"
assert gps is not None, "GPS device not found"
assert imu is not None, "IMU device not found"

lidar.enable(sensorTimeStep)
gps.enable(sensorTimeStep)
imu.enable(sensorTimeStep)



# --- Limits ---
MAX_SPEED_KMH = 28
MAX_STEER_DEG = 65

# --- ROS Node ---
class TT02Ros(Node):
    def __init__(self):
        super().__init__("tt02_ros_controller")
        self.v = 0.0
        self.w = 0.0
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_cb, 10)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.frame_id = "lidar_link"   # tu peux mettre "base_link" au début
        self.last_scan_time = 0.0
        self.scan_period = 0.1  # 10 Hz
        # Odometry variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now().nanoseconds * 1e-9
        self.origin_set = False
        self.x0 = 0.0
        self.y0 = 0.0
        self.yaw0 = 0.0
        self.prev_x = None
        self.prev_y = None


        # Timer to publish TF continuously
        # self.tf_timer = self.create_timer(0.1, self.publish_tf)

    def normalize_angle(self, a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a


    def publish_odom(self, current_time):
        """
        POC odom from Webots GPS only:
        - Use GPS X/Z as ROS x/y (ground plane)
        - Initialize a local origin at first valid GPS sample
        - Use IMU for yaw
        - Publish /odom (TF should use self.x,self.y,self.theta)
        """

        p = gps.getValues()  # [x, y, z] in Webots world frame

        if p is None or len(p) < 3:
            return

        # Webots world -> ROS 2D
        # IMPORTANT: ground plane in Webots is X-Y, with Z up.
        x_w = float(p[0])      # Webots X -> ROS X (forward)
        y_w = float(p[1])      # Webots Y -> ROS Y
        z_wb = float(p[2])    # Webots Z (up)

        
        
        # Wait until GPS is valid (Webots returns NaN for a few ticks)
        if not (math.isfinite(x_w) and math.isfinite(y_w)):
            return

        # Set local origin once
        if not getattr(self, "origin_set", False):
            self.x0 = x_w
            self.y0 = y_w
            imu_values = imu.getRollPitchYaw()
            if imu_values is not None and len(imu_values) >= 3:
                self.yaw0 = float(imu_values[2])
            self.origin_set = True

            # init previous point for yaw estimation
            self.prev_x = 0.0
            self.prev_y = 0.0

            # start at zero
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            # You can publish immediately, but returning avoids any first-step jump
            # return

        # Relative pose in odom
        x = x_w - self.x0
        y = y_w - self.y0

        # Get yaw from IMU
        imu_values = imu.getRollPitchYaw()
        if imu_values is not None and len(imu_values) >= 3:
            yaw = float(imu_values[2])
            new_theta = self.normalize_angle(yaw - self.yaw0)
            # Log si changement brusque
            if hasattr(self, 'prev_theta') and abs(new_theta - self.prev_theta) > 0.1:
                print(f"Theta jump: {self.prev_theta:.3f} -> {new_theta:.3f}", flush=True)
            self.theta = new_theta
            self.prev_theta = self.theta

            # Filtre exponentiel pour stabiliser theta
            if not hasattr(self, 'prev_theta'):
                self.prev_theta = new_theta
            alpha = 0.9  # Ajustez entre 0.8-0.95 pour plus/moins de lissage
            self.theta = alpha * self.prev_theta + (1 - alpha) * new_theta
            self.prev_theta = self.theta

        # Update stored pose
        self.x = x
        self.y = y
        self.prev_x = x
        self.prev_y = y

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        # Twist left unset (or you can derive it from dx/dy over dt if you want)
        self.odom_pub.publish(odom)

        # TF is published by timer

    def publish_tf(self, current_time):
            # ----------------------------
        # 1) odom -> base_footprint
        # ----------------------------
        t_odom_fp = TransformStamped()
        t_odom_fp.header.stamp = current_time.to_msg()
        t_odom_fp.header.frame_id = "odom"
        t_odom_fp.child_frame_id = "base_footprint"

        t_odom_fp.transform.translation.x = float(self.x)
        t_odom_fp.transform.translation.y = float(self.y)
        t_odom_fp.transform.translation.z = 0.0

        # yaw only
        t_odom_fp.transform.rotation.x = 0.0
        t_odom_fp.transform.rotation.y = 0.0
        t_odom_fp.transform.rotation.z = math.sin(self.theta / 2.0)
        t_odom_fp.transform.rotation.w = math.cos(self.theta / 2.0)

        self.tf_broadcaster.sendTransform(t_odom_fp)

        # ----------------------------
        # 2) base_footprint -> base_link
        # (identité 2D, ou mets ici z si ton base_link est surélevé)
        # ----------------------------
        t_fp_bl = TransformStamped()
        t_fp_bl.header.stamp = current_time.to_msg()
        t_fp_bl.header.frame_id = "base_footprint"
        t_fp_bl.child_frame_id = "base_link"

        t_fp_bl.transform.translation.x = 0.0
        t_fp_bl.transform.translation.y = 0.0
        t_fp_bl.transform.translation.z = 0.0  # ex: 0.05 si base_link est 5 cm au-dessus du sol

        t_fp_bl.transform.rotation.x = 0.0
        t_fp_bl.transform.rotation.y = 0.0
        t_fp_bl.transform.rotation.z = 0.0
        t_fp_bl.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t_fp_bl)

        # ----------------------------
        # 3) base_link -> lidar_link
        # (offset du lidar si tu veux)
        # ----------------------------
        t_bl_lidar = TransformStamped()
        t_bl_lidar.header.stamp = current_time.to_msg()
        t_bl_lidar.header.frame_id = "base_link"
        t_bl_lidar.child_frame_id = "lidar_link"

        # Mets ici la vraie position du lidar par rapport à base_link
        t_bl_lidar.transform.translation.x = 0.2   # ex: 0.10 si le lidar est 10 cm devant
        t_bl_lidar.transform.translation.y = 0.0
        t_bl_lidar.transform.translation.z = 0.079   # ex: 0.20 si le lidar est à 20 cm de hauteur

        # Rotation 180° autour de Z pour retourner l'orientation (avant derrière)
        t_bl_lidar.transform.rotation.x = 0.0
        t_bl_lidar.transform.rotation.y = 0.0
        t_bl_lidar.transform.rotation.z = 1.0
        t_bl_lidar.transform.rotation.w = 0.0

        self.tf_broadcaster.sendTransform(t_bl_lidar)
    
    
    def cmd_vel_cb(self, msg): # callback pour cmd_vel
        self.v = msg.linear.x
        self.w = msg.angular.z

rclpy.init()
node = TT02Ros()


def set_vitesse_m_s(v_m_s):
    speed_kmh = max(-MAX_SPEED_KMH, min(MAX_SPEED_KMH, v_m_s * 3.6))
    driver.setCruisingSpeed(speed_kmh)

def set_direction_rad(w, v):
    if abs(v) < 0.01:
        angle = 0.0
    else:
        # Pour rendre intuitif : w > 0 braque à gauche, indépendamment de la direction
        angle = math.atan(w * 0.25 / abs(v))

    angle = max(-math.radians(MAX_STEER_DEG),
                min(math.radians(MAX_STEER_DEG), angle))
    driver.setSteeringAngle(angle)

print("Controller ready, entering main loop.", flush=True)
step_count = 0
SPIN_EVERY_N_STEPS = 1  
while driver.step() != -1:
    step_count += 1
    print("GPS:", gps.getValues(), "IMU:", imu.getRollPitchYaw(), flush=True)

    if step_count % SPIN_EVERY_N_STEPS == 0:
        rclpy.spin_once(node, timeout_sec=0.0)
        current_time = node.get_clock().now()
        node.publish_odom(current_time)
        node.publish_tf(current_time)
        
        now = current_time.nanoseconds * 1e-9
        if now - node.last_scan_time >= node.scan_period:
            node.last_scan_time = now

            ranges = lidar.getRangeImage()
            n = len(ranges)

            msg = LaserScan()
            msg.header.stamp = current_time.to_msg()
            msg.header.frame_id = node.frame_id

            # Important: remplir d'abord ces champs
            msg.range_min = float(lidar.getMinRange())
            msg.range_max = float(lidar.getMaxRange())

            msg.scan_time = float(node.scan_period)
            msg.time_increment = 0.0

            # Angles robustes (si tu veux forcer 360°)
            msg.angle_min = -math.pi
            msg.angle_max =  math.pi
            msg.angle_increment = (msg.angle_max - msg.angle_min) / max(1, (n - 1))

            # Nettoyage ranges
            cleaned = []
            for r in ranges:
                if r is None or not math.isfinite(r):
                    cleaned.append(float("inf"))
                elif r < msg.range_min or r > msg.range_max:
                    cleaned.append(float("inf"))
                else:
                    cleaned.append(float(r))

            # Inverser le sens pour corriger l'orientation
            cleaned = cleaned[::-1]

            msg.ranges = cleaned
            msg.intensities = []

            node.scan_pub.publish(msg)

    print(f"CMD_VEL: v={node.v}, w={node.w}", flush=True)
    print("GPS:", gps.getValues(), flush=True)
    print("IMU:", imu.getRollPitchYaw(), flush=True)

    set_vitesse_m_s(node.v)
    set_direction_rad(node.w, node.v)

node.destroy_node()
rclpy.shutdown()
