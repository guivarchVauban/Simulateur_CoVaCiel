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
 


# --- Webots Driver ---
driver = Driver()
basicTimeStep = int(driver.getBasicTimeStep())
sensorTimeStep = 4 * basicTimeStep

# --- Lidar ---
lidar = Lidar("RpLidarA2")
lidar.enable(sensorTimeStep)

gps = GPS("gps")
gps.enable(sensorTimeStep)

imu = InertialUnit("imu")
imu.enable(sensorTimeStep)

# --- Limits ---
MAX_SPEED_KMH = 28
MAX_STEER_DEG = 32

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

        # Timer to publish TF continuously
        # self.tf_timer = self.create_timer(0.1, self.publish_tf)

    def publish_odom(self, current_time):
    # Pose vraie Webots
        p = gps.getValues()  # [x, y, z] dans repère Webots
        rpy = imu.getRollPitchYaw()
        yaw = rpy[2]

        # ⚠️ Convention : Webots est souvent X avant, Z haut, Y gauche/droite.
        # Selon ton monde, tu devras peut-être mapper (x,y) -> (x, y) ou (x, -y)
        x = float(p[0])
        y = float(p[1])   # parfois il faut mettre -p[2] ou -p[1] selon ton modèle
        self.x, self.y, self.theta = x, y, yaw

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

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
        t_bl_lidar.transform.translation.x = 0.0   # ex: 0.10 si le lidar est 10 cm devant
        t_bl_lidar.transform.translation.y = 0.0
        t_bl_lidar.transform.translation.z = 0.0   # ex: 0.20 si le lidar est à 20 cm de hauteur

        t_bl_lidar.transform.rotation.x = 0.0
        t_bl_lidar.transform.rotation.y = 0.0
        t_bl_lidar.transform.rotation.z = 0.0
        t_bl_lidar.transform.rotation.w = 1.0

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
        angle = math.atan(w * 0.25 / v)

    angle = max(-math.radians(MAX_STEER_DEG),
                min(math.radians(MAX_STEER_DEG), angle))
    driver.setSteeringAngle(angle)

print("TT02 ROS controller started")
step_count = 0
SPIN_EVERY_N_STEPS = 1  
while driver.step() != -1:
    step_count += 1
    if step_count % SPIN_EVERY_N_STEPS == 0:
        rclpy.spin_once(node, timeout_sec=0.0)
        current_time = node.get_clock().now()
        node.publish_odom(current_time)
        node.publish_tf(current_time)
        
        now = current_time.nanoseconds * 1e-9
        if now - node.last_scan_time >= node.scan_period:
            node.last_scan_time = now

            ranges = lidar.getRangeImage()  # liste de distances

            msg = LaserScan()
            msg.header.stamp = current_time.to_msg()
            msg.header.frame_id = node.frame_id

            # Paramètres géométriques lidar depuis Webots
            msg.angle_min = -lidar.getFov() / 2.0
            msg.angle_max =  lidar.getFov() / 2.0
            msg.angle_increment = lidar.getFov() / max(1, (lidar.getHorizontalResolution() - 1))

            msg.time_increment = 0.0
            msg.scan_time = node.scan_period

            msg.range_min = lidar.getMinRange()
            msg.range_max = lidar.getMaxRange()

            # Webots renvoie parfois inf ou des valeurs hors range, on nettoie
            cleaned = []
            for r in ranges:
                if r is None:
                    cleaned.append(float("inf"))
                elif r < msg.range_min or r > msg.range_max:
                    cleaned.append(float("inf"))
                else:
                    cleaned.append(float(r))

            msg.ranges = cleaned
            msg.intensities = []  # pas dispo par défaut

            node.scan_pub.publish(msg)


    set_vitesse_m_s(node.v)
    set_direction_rad(node.w, node.v)

node.destroy_node()
rclpy.shutdown()
