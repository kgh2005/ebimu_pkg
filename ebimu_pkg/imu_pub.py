import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import serial
import re
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

class EbimuPublisher(Node):

    def __init__(self):
        super().__init__('ebimu_publisher')
        qos_profile = QoSProfile(depth=10)

        # IMU 데이터 퍼블리셔 설정
        self.imu_pub = self.create_publisher(Imu, '/imu/data', qos_profile)

        # 시리얼 포트 설정
        self.comport_num = "/dev/ttyUSB0"
        self.comport_baudrate = 115200

        try:
            self.ser = serial.Serial(port=self.comport_num, baudrate=self.comport_baudrate, timeout=1)
            self.get_logger().info("Connected to IMU Serial Port")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial Port Error: {e}")
            return

        # IMU 초기화 메시지 전송
        self.setup_imu()

        # 타이머 설정 (10ms 간격 = 100Hz)
        self.timer_period = 0.01
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # 이전 데이터 저장
        self.prev_str = ""

    def setup_imu(self):
        """ IMU의 데이터 출력을 설정하는 함수 """
        setup_cmds = [
            "<sof2>",  # Quaternion ON
            "<sog1>",  # Angular velocity ON
            "<soa1>",  # Linear acceleration ON
            "<sem1>",  # Magnetometer ON
            "<sot0>",  # Temperature OFF
            "<sod0>",  # Distance OFF
            "<sor10>"  # Output rate 100Hz 설정
        ]

        for cmd in setup_cmds:
            self.ser.write(cmd.encode())
            self.get_logger().info(f"IMU 설정: {cmd}")
            self.ser.readline()  # 응답 읽기

    def timer_callback(self):
        self.ser.reset_input_buffer()
        ser_data = self.ser.read_until(expected=b'\n').decode('utf-8', errors='ignore').strip()

        # 원시 데이터 디버깅 출력
        self.get_logger().debug(f"Raw IMU string: {ser_data}")

        # 쉼표 개수로 대략적인 유효성 확인
        if ',' not in ser_data or '*' not in ser_data:
            self.get_logger().warn(f"Malformed IMU data (missing * or ,): {ser_data}")
            return

        str_list = ser_data.split(',')

        if '*' in str_list[0]:
            str_list[0] = str_list[0].split('*')[-1]  # 마지막 부분 사용
        else:
            self.get_logger().warn("Received IMU data does not contain '*'")
            return

        # 필드 부족 여부 체크 (최소 10개)
        if len(str_list) < 10:
            self.get_logger().warn(f"Incomplete IMU data: {str_list}")
            return

        self.get_logger().info(f"RAW: {ser_data}")

        try:
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"

            # Quaternion 순서 변경
            imu_msg.orientation = Quaternion(
                x=float(str_list[2]),
                y=float(str_list[1]),
                z=float(str_list[0]),
                w=float(str_list[3])
            )
            imu_msg.orientation_covariance = [0.0007, 0.0, 0.0, 0.0, 0.0007, 0.0, 0.0, 0.0, 0.0007]

            # 가속도 (중력 영향 포함)
            imu_msg.linear_acceleration.x = -float(str_list[7])
            imu_msg.linear_acceleration.y = -float(str_list[8])
            imu_msg.linear_acceleration.z = -float(str_list[9])
            imu_msg.linear_acceleration_covariance = [0.005, 0.0, 0.0, 0.0, 0.005, 0.0, 0.0, 0.0, 0.005]

            # 각속도 (도 → 라디안 변환)
            imu_msg.angular_velocity.x = math.radians(float(str_list[4]))
            imu_msg.angular_velocity.y = math.radians(float(str_list[5]))
            imu_msg.angular_velocity.z = math.radians(float(str_list[6]))
            imu_msg.angular_velocity_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]

            self.imu_pub.publish(imu_msg)

        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"IMU 데이터 파싱 오류: {e}, 원문: {str_list}")



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
