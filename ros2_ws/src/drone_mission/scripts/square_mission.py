#!/usr/bin/env python3
"""
Square Mission - ROS 2 Version (Foxy & Humble)
=================================================
Misi navigasi otonom membentuk pola persegi sempurna (2x2 meter)
pada ketinggian tetap menggunakan rclpy dan MAVROS2.

Logic Requirement: "Distance-Based Waypoint"
- TIDAK menggunakan time.sleep()
- Menghitung jarak Euclidean antara posisi drone saat ini dengan target
- Threshold: jarak < 0.3 meter
- Frekuensi setpoint: 10Hz - 20Hz
- Safety: berhenti jika mode berubah manual (misal STABILIZE)

Kompatibel dengan ROS 2 Foxy & Humble + MAVROS2 + ArduPilot SITL
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, StreamRate


class SquareMission(Node):
    def __init__(self):
        super().__init__('square_mission_node')

        # ---------- Parameters ----------
        self.declare_parameter('square_size', 2.0)
        self.declare_parameter('altitude', 2.0)
        self.declare_parameter('threshold', 0.3)
        self.declare_parameter('rate', 20)

        self.square_size = self.get_parameter('square_size').value
        self.altitude = self.get_parameter('altitude').value
        self.threshold = self.get_parameter('threshold').value
        self.rate_hz = self.get_parameter('rate').value

        # ---------- State Variables ----------
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.pose_received = False
        self.mission_active = True

        # ---------- QoS ----------
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # ---------- Subscribers ----------
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_cb, qos_reliable)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_cb, qos_profile)

        # ---------- Publisher ----------
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)

        # ---------- Service Clients ----------
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')
        self.stream_rate_client = self.create_client(StreamRate, '/mavros/set_stream_rate')

        self.get_logger().info('Square Mission Node telah siap.')
        self.get_logger().info(
            f'Ukuran persegi: {self.square_size:.1f} m | '
            f'Ketinggian: {self.altitude:.1f} m | '
            f'Threshold: {self.threshold:.2f} m')

    # ======================= CALLBACKS =======================

    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb(self, msg):
        self.current_pose = msg
        self.pose_received = True

    # ======================= HELPERS =======================

    def spin_once(self, timeout_sec=0.0):
        rclpy.spin_once(self, timeout_sec=timeout_sec)

    def _wait_for_service(self, client, name, timeout=5.0):
        self.get_logger().info(f'Menunggu service {name} ...')
        while not client.wait_for_service(timeout_sec=timeout):
            if not rclpy.ok():
                return False
            self.get_logger().warn(f'Service {name} belum tersedia ...')
        return True

    def _call_service_sync(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is not None:
            return future.result()
        return None

    def euclidean_distance(self, target):
        dx = self.current_pose.pose.position.x - target.pose.position.x
        dy = self.current_pose.pose.position.y - target.pose.position.y
        dz = self.current_pose.pose.position.z - target.pose.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def create_pose(self, x, y, z):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.w = 1.0
        return pose

    def is_mode_guided(self):
        return self.current_state.mode == 'GUIDED'

    def is_armed(self):
        return self.current_state.armed

    def safety_check(self):
        if not self.is_mode_guided() and self.mission_active:
            self.get_logger().warn(
                f'Mode berubah ke {self.current_state.mode}! Menghentikan misi.')
            self.mission_active = False
            return False
        return True

    # ======================= MISSION METHODS =======================

    def wait_for_connection(self):
        self.get_logger().info('Menunggu koneksi FCU ...')
        while rclpy.ok() and not self.current_state.connected:
            self.spin_once(timeout_sec=0.05)
        self.get_logger().info('FCU terhubung!')

    def request_data_stream(self):
        self.get_logger().info('Requesting data stream dari FCU ...')
        if not self._wait_for_service(self.stream_rate_client, '/mavros/set_stream_rate'):
            return
        req = StreamRate.Request()
        req.stream_id = 0
        req.message_rate = 10
        req.on_off = True
        resp = self._call_service_sync(self.stream_rate_client, req)
        if resp is not None:
            self.get_logger().info('Data stream berhasil di-request!')
        else:
            self.get_logger().warn('Gagal request stream!')

    def wait_for_pose(self):
        self.get_logger().info('Menunggu data pose ...')
        while rclpy.ok() and not self.pose_received:
            self.spin_once(timeout_sec=0.05)
        p = self.current_pose.pose.position
        self.get_logger().info(f'Data pose diterima! Posisi: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})')

    def set_guided_mode(self):
        if not self._wait_for_service(self.set_mode_client, '/mavros/set_mode'):
            return False

        self.get_logger().info('Mencoba set mode GUIDED ...')
        last_request = self.get_clock().now()
        interval = rclpy.duration.Duration(seconds=2.0)

        while rclpy.ok():
            self.spin_once(timeout_sec=0.05)
            if self.current_state.mode != 'GUIDED':
                now = self.get_clock().now()
                if (now - last_request) > interval:
                    req = SetMode.Request()
                    req.custom_mode = 'GUIDED'
                    resp = self._call_service_sync(self.set_mode_client, req)
                    if resp and resp.mode_sent:
                        self.get_logger().info('Request GUIDED terkirim ...')
                    else:
                        self.get_logger().warn('Service call gagal.')
                    last_request = self.get_clock().now()
            else:
                self.get_logger().info('Mode GUIDED aktif!')
                return True
        return False

    def arm_drone(self):
        if not self._wait_for_service(self.arming_client, '/mavros/cmd/arming'):
            return False

        self.get_logger().info('Mencoba ARM drone ...')
        last_request = self.get_clock().now()
        interval = rclpy.duration.Duration(seconds=2.0)

        while rclpy.ok():
            self.spin_once(timeout_sec=0.05)
            if not self.current_state.armed:
                now = self.get_clock().now()
                if (now - last_request) > interval:
                    req = CommandBool.Request()
                    req.value = True
                    resp = self._call_service_sync(self.arming_client, req)
                    if resp and resp.success:
                        self.get_logger().info('Request ARM terkirim ...')
                    else:
                        self.get_logger().warn('Service call gagal.')
                    last_request = self.get_clock().now()
            else:
                self.get_logger().info('Drone berhasil di-ARM!')
                return True
        return False

    def takeoff(self):
        if not self._wait_for_service(self.takeoff_client, '/mavros/cmd/takeoff'):
            return False

        self.get_logger().info(f'Takeoff ke ketinggian {self.altitude:.1f} m ...')
        last_request = self.get_clock().now()
        interval = rclpy.duration.Duration(seconds=2.0)

        while rclpy.ok():
            self.spin_once(timeout_sec=0.05)
            now = self.get_clock().now()
            if (now - last_request) > interval:
                req = CommandTOL.Request()
                req.altitude = float(self.altitude)
                resp = self._call_service_sync(self.takeoff_client, req)
                if resp and resp.success:
                    self.get_logger().info('Takeoff command diterima!')
                    break
                else:
                    self.get_logger().warn('Takeoff gagal, coba lagi ...')
                last_request = self.get_clock().now()

        self.get_logger().info(f'Menunggu drone mencapai ketinggian {self.altitude:.1f} m ...')
        while rclpy.ok():
            self.spin_once(timeout_sec=0.05)
            current_alt = self.current_pose.pose.position.z
            if current_alt >= self.altitude * 0.90:
                self.get_logger().info(f'Ketinggian tercapai! ({current_alt:.2f} m)')
                return True
        return False

    def navigate_to_waypoint(self, target, wp_name=''):
        self.get_logger().info(
            f'Navigasi ke waypoint {wp_name}: '
            f'({target.pose.position.x:.2f}, {target.pose.position.y:.2f}, '
            f'{target.pose.position.z:.2f})')

        while rclpy.ok():
            self.spin_once(timeout_sec=0.05)
            if not self.safety_check():
                return False

            target.header.stamp = self.get_clock().now().to_msg()
            self.setpoint_pub.publish(target)

            distance = self.euclidean_distance(target)
            if distance < self.threshold:
                self.get_logger().info(
                    f'Waypoint {wp_name} tercapai! (jarak: {distance:.3f} m)')
                return True
        return False

    def land_drone(self):
        self.get_logger().info('Misi selesai! Memulai pendaratan ...')
        if not self._wait_for_service(self.set_mode_client, '/mavros/set_mode'):
            return

        while rclpy.ok():
            self.spin_once(timeout_sec=0.05)
            if self.current_state.mode != 'LAND':
                req = SetMode.Request()
                req.custom_mode = 'LAND'
                resp = self._call_service_sync(self.set_mode_client, req)
                if resp and resp.mode_sent:
                    self.get_logger().info('Mode LAND berhasil diaktifkan!')
                    break
                else:
                    self.get_logger().warn('Set mode LAND gagal.')
            else:
                break

        self.get_logger().info('Menunggu drone mendarat ...')
        while rclpy.ok() and self.current_state.armed:
            self.spin_once(timeout_sec=0.05)
        self.get_logger().info('Drone telah mendarat dengan selamat!')

    # ======================= MAIN MISSION =======================

    def generate_square_waypoints(self):
        s = self.square_size
        z = self.altitude
        x0 = self.current_pose.pose.position.x
        y0 = self.current_pose.pose.position.y

        self.get_logger().info(f'Posisi awal referensi: ({x0:.2f}, {y0:.2f})')

        waypoints = [
            (self.create_pose(x0 + s, y0,     z), 'WP1'),
            (self.create_pose(x0 + s, y0 + s, z), 'WP2'),
            (self.create_pose(x0,     y0 + s, z), 'WP3'),
            (self.create_pose(x0,     y0,     z), 'WP4'),
        ]
        return waypoints

    def run(self):
        self.get_logger().info('=' * 50)
        self.get_logger().info('  SQUARE MISSION - 2x2m Autonomous Navigation')
        self.get_logger().info('  ArduPilot + MAVROS (GUIDED mode) [ROS 2 Python]')
        self.get_logger().info('=' * 50)

        # 1. Tunggu koneksi FCU
        self.wait_for_connection()

        # 2. Request data stream
        self.request_data_stream()

        # 3. Tunggu data pose
        self.wait_for_pose()

        # 4. Set GUIDED mode
        if not self.set_guided_mode():
            self.get_logger().error('Gagal set GUIDED mode!')
            return

        # 5. Arm drone
        if not self.arm_drone():
            self.get_logger().error('Gagal arm drone!')
            return

        # 6. Takeoff
        if not self.takeoff():
            self.get_logger().error('Takeoff gagal!')
            return

        if not self.is_mode_guided():
            self.get_logger().warn(
                f'Mode berubah setelah takeoff: {self.current_state.mode}')
            self.set_guided_mode()

        self.mission_active = True

        # 7. Generate waypoints persegi
        waypoints = self.generate_square_waypoints()

        self.get_logger().info('-' * 40)
        self.get_logger().info(
            f'Memulai navigasi pola persegi {self.square_size:.0f}x{self.square_size:.0f} meter ...')
        self.get_logger().info(f'Jumlah waypoint: {len(waypoints)}')
        self.get_logger().info('-' * 40)

        # 8. Navigasi ke setiap waypoint
        for i, (wp, name) in enumerate(waypoints):
            self.get_logger().info(
                f'[{i+1}/{len(waypoints)}] Target: {name} '
                f'({wp.pose.position.x:.2f}, {wp.pose.position.y:.2f}, '
                f'{wp.pose.position.z:.2f})')

            success = self.navigate_to_waypoint(wp, name)

            if not success:
                if not self.mission_active:
                    self.get_logger().warn(
                        'Misi dihentikan karena mode berubah secara manual.')
                else:
                    self.get_logger().warn('Misi dihentikan (node shutdown).')
                return

        # 9. Semua waypoint tercapai - LAND
        self.get_logger().info('=' * 40)
        self.get_logger().info('Semua waypoint telah tercapai!')
        self.get_logger().info('=' * 40)
        self.land_drone()

        self.get_logger().info('=' * 50)
        self.get_logger().info('  MISI SELESAI DENGAN SUKSES!')
        self.get_logger().info('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    try:
        mission = SquareMission()
        mission.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
