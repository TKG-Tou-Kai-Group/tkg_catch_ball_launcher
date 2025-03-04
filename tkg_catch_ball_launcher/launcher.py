import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64, Int64, Empty, Bool
from sensor_msgs.msg import Image, CameraInfo, LaserScan
import numpy as np
import cv2
from cv_bridge import CvBridge

from tkg_catch_ball_launcher.extract_object_region import ExtractObject
from tkg_catch_ball_launcher.calc_launch_angle import calc_best_launch_angle

class CatchBallLauncher(Node):
    DEBUG = True

    def __init__(self):
        super().__init__('auto')

        # 砲塔の旋回中心からLidarまでの位置
        self.POSE_OFFSET_X = 0.3
        self.POSE_OFFSET_Y = 0.0
        # エアガンの安全基準0.98Jより逆算（もともとは3.5J/cm^2以下から来ているので、ややこじつけ感）
        # 参考URL: https://d-stagegunnet.jp/hpgen/HPB/entries/7.html
        # E = 1/2 * m * v^2 = 1/2 * m * (2 * math.pi * radius * rpm / 60)^2
        # 0.98 = 0.5 * 0.03 *(22 * math.pi * 0.02 * rpm / 60)^2
        # rpm = 3859.3
        self.TARGET_RPM = 3800
        # 射出時減速分
        self.TARGET_RPM_ADD = 500

        # どの範囲で遊ぶかを指定
        self.PLAY_FIELD_START_DISTANCE = 2.0 # ロボットが検知を始める距離
        self.PLAY_FIELD_END_DISTANCE = 4.0   # ロボットが検知を終わる距離
        self.PLAY_FIELD_WIDTH = 0.6          # ロボットが検知できる範囲の横幅

        self.AUTO_MODE_INIT = 0
        self.AUTO_MODE_TRACKING = 1
        self.auto_mode = self.AUTO_MODE_INIT

        self.attack_wait_counter = 0

        self.hammer_pub = self.create_publisher(Empty, "/hammer", 10)
        self.mazemaze_pub = self.create_publisher(Empty, "/mazemaze", 10)
        self.roller_pub = self.create_publisher(Float64, "/roller", 10)
        self.control_pub = self.create_publisher(Bool, "/control", 10)
        self.yaw_pub = self.create_publisher(Float64, "/yaw", 10)
        self.pitch_pub = self.create_publisher(Float64, "/pitch", 10)

        self.object_image_pub = self.create_publisher(Image, "/object_image", 10)

        self.camera_mat = None
        self.dist_coeffs = None
        self.extractor = None
        self.image = None
        self.depth_image = None
        self.bridge = CvBridge()
        self.camera_image_sub = self.create_subscription(Image,'/camera/camera/color/image_raw',self.image_callback,10)
        self.depth_image_sub = self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_image_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo,'/camera/camera/color/camera_info',self.camera_info_callback,10)

        self.motor0_rpm = 0
        self.motor1_rpm = 0
        self.subscription0 = self.create_subscription(Float64, '/can_node/c620_0/rpm', self.motor0_callback, 10)
        self.subscription1 = self.create_subscription(Float64, '/can_node/c620_1/rpm', self.motor1_callback, 10)

        self.scan = None
        self.clusters_sub = self.create_subscription(LaserScan,'/scan',self.scan_callback,10)

        self.turret_data = Vector3()
        self.turret_sub = self.create_subscription(Vector3,'/current_turret_pose',self.turret_callback,10)

        self.timer = self.create_timer(0.04, self.timer_callback)

    def turret_callback(self, msg):
        self.turret_data = msg

    def image_callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def depth_image_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def camera_info_callback(self, msg):
        self.camera_mat = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def motor0_callback(self, msg):
        self.motor0_rpm = msg.data

    def motor1_callback(self, msg):
        self.motor1_rpm = msg.data

    def scan_callback(self, msg):
        self.scan = msg

    def timer_callback(self):
        current_yaw = self.turret_data.z
        current_pitch = self.turret_data.y

        if self.auto_mode == self.AUTO_MODE_INIT:
            self.control_pub.publish(Bool(data=False))
            if self.extractor is None and self.camera_mat is not None and self.dist_coeffs is not None:
                self.extractor = ExtractObject(self.camera_mat, self.dist_coeffs)

            if self.extractor is not None and self.image is not None and self.depth_image is not None and self.scan is not None:
                self.auto_mode = self.AUTO_MODE_TRACKING

        elif self.auto_mode == self.AUTO_MODE_TRACKING:
            self.get_logger().info(f"TRACKING_MODE:")
            self.control_pub.publish(Bool(data=True))

            current_rpm = (abs(self.motor0_rpm) + abs(self.motor1_rpm)) / 2.0

            # ボール発射後に5秒の待機時間を設けるためのカウンタ処理
            if self.attack_wait_counter > 0:
                self.attack_wait_counter += 1
                if self.attack_wait_counter > int(125):
                    self.attack_wait_counter = 0

            # 所定の検知範囲以外は30mに置き換える
            filtered_ranges = [r if self.PLAY_FIELD_START_DISTANCE <= r <= self.PLAY_FIELD_END_DISTANCE else 30.0 for r in self.scan.ranges]

            x_min_range = 30.0
            y_min_range = 30.0
            min_range = 30.0
            for i in range(len(filtered_ranges)):
                angle = self.scan.angle_min + i * self.scan.angle_increment
                x = filtered_ranges[i] * math.cos(angle) + self.POSE_OFFSET_X
                y = filtered_ranges[i] * math.sin(angle) + self.POSE_OFFSET_Y
                if y < -abs(self.PLAY_FIELD_WIDTH/2) or y > abs(self.PLAY_FIELD_WIDTH/2):
                    continue
                if x_min_range > x:
                    x_min_range = x
                    y_min_range = y
                    # オフセットの変化分があるので、self.scan.ranges[i]ではない
                    min_range = math.sqrt(x ** 2 + y ** 2)
            x = x_min_range
            y = y_min_range

            # 検知範囲で相手を見つけたとき
            if self.PLAY_FIELD_START_DISTANCE <= min_range <= self.PLAY_FIELD_END_DISTANCE:

                # 相手の方向に向く処理
                target_yaw = math.atan2(y, x)
                target_pitch = 0.0
                self.yaw_pub.publish(Float64(data=float(target_yaw)))
                # 相手の方向に向くことができたとき
                if abs(target_yaw - current_yaw) <= math.radians(5.0):
                    # 画像上の相手領域の抜き出し
                    # extract_imageがextract関数の引数で渡したサイズ（幅x高さ）で切り抜いた画像
                    # result_imageが元の画像に切り抜いた範囲を示す四角形を描画したもの
                    extract_image, result_image = self.extractor.extract(self.image, x, y, current_pitch, current_yaw, (128, 256))
                    # 切り抜いた画像上の点を元の画像上の点に変換する関数
                    # （今回は対象物の地上高0.3m付近をしめす一番下の中点の位置を取得）
                    (result_x, result_y) = self.extractor.convert_croped_point_to_image_point(128//2, 256)
                    (_, result_y_top) = self.extractor.convert_croped_point_to_image_point(128//2, 0)

                    image_h, image_w, _ = self.image.shape
                    if result_x < 10 or result_x > image_w - 10:
                        return

                    depth_image_h, depth_image_w = self.depth_image.shape
                    # デプス画像上のx座標に変換
                    depth_result_x = result_x * depth_image_w / image_w
                    depth_detect_y = -1
                    for i in range(int(depth_image_h - result_y_top)):
                        current_y = int(i + result_y_top)
                        for v in range(10):
                            # 最も近い物体の前後0.4m以内で最も高い場所（おそらく頭部）の位置を算出
                            if self.depth_image[current_y][int(depth_result_x + 10*(v - 5))] / 1000.0 > min_range - 0.4 and self.depth_image[current_y][int(depth_result_x + 10*(v - 5))] / 1000.0 < min_range + 0.4:
                                depth_detect_y = current_y
                                break
                        if not depth_detect_y == -1:
                            break
                    if depth_detect_y == -1:
                        return
                    detect_y = depth_detect_y * image_h / depth_image_h

                    # 相手の頭部と地上高0.3mの点の中点くらいの高さを目標地点に設定
                    # 画像上の点と距離情報から目標地点の三次元座標を取得
                    target_position = self.extractor.convert_image_point_to_target_point(result_x, (result_y + detect_y) / 2, min_range)
                    cv2.drawMarker(result_image, (int(result_x), int((result_y + detect_y) // 2)), (0, 255, 0), thickness=3)
                    ros_image = self.bridge.cv2_to_imgmsg(result_image, encoding='bgr8')
                    self.object_image_pub.publish(ros_image)

                    if self.TARGET_RPM >= 3800:
                        target_pitch = calc_best_launch_angle(self.TARGET_RPM, min_range, target_position[2])
                    else:
                        # 色々計算してみたところ、3000RPM程度ではそこまで高く投げ上げられないので、砲塔高さから0m(地面から0.36m)付近に投げるようにする
                        target_pitch = calc_best_launch_angle(self.TARGET_RPM, min_range, 0.0)
                    if target_pitch > math.radians(28):
                        target_pitch = math.radians(27.9)
                    if target_pitch > math.radians(-8):
                        self.pitch_pub.publish(Float64(data=float(target_pitch)))
                        #self.roller_pub.publish(Float64(data=float(self.TARGET_RPM + self.TARGET_RPM_ADD)))

                    # 目標地点への発射姿勢・発射速度に到達したとき
                    if abs(target_pitch - current_pitch) <= math.radians(5.0) and abs(current_rpm - self.TARGET_RPM - self.TARGET_RPM_ADD) <= 200:
                        if self.attack_wait_counter == 0:
                            self.hammer_pub.publish(Empty())
                            self.mazemaze_pub.publish(Empty())

                            # 発車後待ちカウンタをスタート
                            self.attack_wait_counter += 1

            # 検知範囲内に相手が見つからないとき
            else:
                self.roller_pub.publish(Float64(data=float(0.0)))

def main():
    rclpy.init()
    node = CatchBallLauncher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
