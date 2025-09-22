#!/usr/bin/env python3
import math
import serial
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

class EbimuPublisher(Node):
    """
    EBIMU 시리얼 -> /bhr_b3/imu 퍼블리시 (SI 단위, z드리프트 완화 옵션 포함)
    - accel_scale:   g -> m/s^2 (default 9.80665)
    - gyro_in_deg:   True면 deg/s 입력을 rad/s로 변환
    - invert_accel_sign: True면 가속도 부호 반전(장치 정의에 맞춰 선택)
    - use_imu_orientation: False면 쿼터니언 무시(orientation unknown으로 전달)
    - reliability:   reliable / best_effort 선택
    - stamp_offset_sec: 타임스탬프에 상수 오프셋(±초) 추가 (IMU↔LiDAR 시간정렬용)
    - accel_bias_si: [bx,by,bz] m/s^2 단위 상수 바이어스 제거 (필요시)
    - frame_id:      기본 imu_link (정적TF로 base_link↔imu_link 연결 권장)
    """
    def __init__(self):
        super().__init__('ebimu_publisher')

        # === 파라미터 ===
        self.port     = self.declare_parameter('port', '/dev/ttyUSB0').get_parameter_value().string_value
        self.baud     = self.declare_parameter('baud', 115200).get_parameter_value().integer_value
        self.frame_id = self.declare_parameter('frame_id', 'base_link').get_parameter_value().string_value

        self.accel_scale = self.declare_parameter('accel_scale', 9.80665).get_parameter_value().double_value
        self.gyro_in_deg = self.declare_parameter('gyro_in_deg', True).get_parameter_value().bool_value
        self.invert_accel_sign = self.declare_parameter('invert_accel_sign', True).get_parameter_value().bool_value
        self.use_imu_orientation = self.declare_parameter('use_imu_orientation', False).get_parameter_value().bool_value

        # 타임오프셋(초). IMU time이 LiDAR보다 "뒤"면 양수, "앞"이면 음수 쪽을 시도.
        self.stamp_offset_sec = self.declare_parameter('stamp_offset_sec', 0.0).get_parameter_value().double_value

        # 상수 바이어스 보정(m/s^2). 정지 평균에서 (x,y,z) 편차를 측정해서 넣으면 좋음.
        self.accel_bias_si = list(self.declare_parameter('accel_bias_si', [0.0, 0.0, 0.0])
                                  .get_parameter_value().double_array_value)

        # 공분산(필요 시 튜닝)
        self.ori_cov = list(self.declare_parameter('orientation_cov',
                         [0.0007, 0.0, 0.0,
                          0.0, 0.0007, 0.0,
                          0.0, 0.0, 0.0007]).get_parameter_value().double_array_value)
        self.gyr_cov = list(self.declare_parameter('gyro_cov',
                         [0.001, 0.0, 0.0,
                          0.0, 0.001, 0.0,
                          0.0, 0.0, 0.001]).get_parameter_value().double_array_value)
        self.acc_cov = list(self.declare_parameter('accel_cov',
                         [0.005, 0.0, 0.0,
                          0.0, 0.005, 0.0,
                          0.0, 0.0, 0.005]).get_parameter_value().double_array_value)

        # QoS
        rel_str = self.declare_parameter('reliability', 'reliable').get_parameter_value().string_value.lower()
        depth   = self.declare_parameter('depth', 100).get_parameter_value().integer_value
        qos = QoSProfile(depth=depth)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.RELIABLE if rel_str == 'reliable' else ReliabilityPolicy.BEST_EFFORT

        self.imu_pub = self.create_publisher(Imu, '/imu/data', qos)

        # 시리얼
        try:
            self.ser = serial.Serial(port=self.port, baudrate=self.baud, timeout=1)
            self.get_logger().info(f"Connected EBIMU: {self.port} @ {self.baud}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial Port Error: {e}")
            raise

        # 기기 설정
        self.setup_imu()

        # 100Hz
        self.timer = self.create_timer(0.005, self.timer_callback)

    def setup_imu(self):
        cmds = [
            "<sof2>",  # Quaternion ON
            "<sog1>",  # Angular velocity ON
            "<soa1>",  # Linear acceleration ON
            "<sem1>",  # Magnetometer ON
            "<sot0>",  # Temperature OFF
            "<sod0>",  # Distance OFF
            "<sor10>"  # Output 100Hz
        ]
        for c in cmds:
            try:
                self.ser.write(c.encode())
                _ = self.ser.readline()
            except serial.SerialException as e:
                self.get_logger().warn(f"IMU setup serial error: {e}")

    def timer_callback(self):
        try:
            line = self.ser.read_until(expected=b'\n').decode('utf-8', errors='ignore').strip()
        except serial.SerialException as e:
            self.get_logger().warn(f"Serial read error: {e}")
            return

        if not line or ',' not in line or '*' not in line:
            return

        parts = line.split(',')
        if '*' in parts[0]:
            parts[0] = parts[0].split('*')[-1]
        if len(parts) < 10:
            return

        try:
            imu = Imu()

            # 타임스탬프(+오프셋)
            now = self.get_clock().now()
            if abs(self.stamp_offset_sec) > 0.0:
                now = now + Duration(seconds=self.stamp_offset_sec)
            imu.header.stamp = now.to_msg()
            imu.header.frame_id = self.frame_id

            # Orientation
            if self.use_imu_orientation:
                imu.orientation = Quaternion(
                    x=float(parts[2]),
                    y=float(parts[1]),
                    z=float(parts[0]),
                    w=float(parts[3])
                )
                imu.orientation_covariance = self.ori_cov
            else:
                imu.orientation.x = 0.0
                imu.orientation.y = 0.0
                imu.orientation.z = 0.0
                imu.orientation.w = 1.0
                imu.orientation_covariance = [-1.0, 0.0, 0.0,
                                              0.0, -1.0, 0.0,
                                              0.0, 0.0, -1.0]

            # Gyro
            gx = float(parts[4]); gy = float(parts[5]); gz = float(parts[6])
            if self.gyro_in_deg:
                gx = math.radians(gx); gy = math.radians(gy); gz = math.radians(gz)
            imu.angular_velocity.x = gx
            imu.angular_velocity.y = gy
            imu.angular_velocity.z = gz
            imu.angular_velocity_covariance = self.gyr_cov

            # Accel (g→SI) + 부호 + 상수 바이어스 보정
            ax = float(parts[7]); ay = float(parts[8]); az = float(parts[9])
            sign = -1.0 if self.invert_accel_sign else 1.0
            ax_si = sign * ax * self.accel_scale - self.accel_bias_si[0]
            ay_si = sign * ay * self.accel_scale - self.accel_bias_si[1]
            az_si = sign * az * self.accel_scale - self.accel_bias_si[2]
            imu.linear_acceleration.x = ax_si
            imu.linear_acceleration.y = ay_si
            imu.linear_acceleration.z = -az_si
            imu.linear_acceleration_covariance = self.acc_cov

            # (옵션) 정지 상태에서 노름 모니터
            # a_norm = math.sqrt(ax_si*ax_si + ay_si*ay_si + az_si*az_si)
            # if abs(a_norm - 9.80665) > 3.0:
            #     self.get_logger().debug(f"Accel norm={a_norm:.2f}")

            self.imu_pub.publish(imu)

        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"Parse error: {e} | raw: {parts}")

def main(args=None):
    rclpy.init(args=args)
    node = EbimuPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
