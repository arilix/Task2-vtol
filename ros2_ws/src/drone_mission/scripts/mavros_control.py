#!/usr/bin/env python3
"""
MAVROS Drone Control - ROS 2 Version (Foxy & Humble)
=======================================================
Script Python menggunakan rclpy dan MAVROS untuk kontrol drone
secara otonom melalui ArduPilot SITL.

Fitur:
  - Koneksi FCU & data stream request
  - Set mode (GUIDED, LAND, LOITER, RTL, STABILIZE, dll.)
  - Arm / Disarm
  - Takeoff & Land
  - Navigasi waypoint (distance-based, tanpa time.sleep)
  - Hover di posisi tertentu
  - Safety check (auto-stop jika mode berubah manual)
  - Monitoring state & posisi real-time

Penggunaan:
  ros2 launch drone_mission mavros_control.launch.py
  atau
  ros2 run drone_mission mavros_control.py

Kompatibel dengan ROS 2 Foxy & Humble + MAVROS2 + ArduPilot SITL
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, StreamRate, CommandLong
from sensor_msgs.msg import NavSatFix, BatteryState


class MAVROSControl(Node):
    """
    Kelas utama untuk kontrol drone melalui MAVROS di ROS 2.
    Menyediakan fungsi-fungsi dasar: arm, disarm, set mode,
    takeoff, land, navigate, hover, dan monitoring.
    """

    def __init__(self):
        super().__init__('mavros_control_node')

        # ===================== PARAMETERS =====================
        self.declare_parameter('altitude', 2.0)
        self.declare_parameter('threshold', 0.3)
        self.declare_parameter('rate', 20)
        self.declare_parameter('mission_type', 'hover')
        self.declare_parameter('hover_duration', 10.0)
        self.declare_parameter('square_size', 2.0)
        self.declare_parameter('circle_radius', 2.0)
        self.declare_parameter('circle_points', 12)

        self.altitude = self.get_parameter('altitude').value
        self.threshold = self.get_parameter('threshold').value
        self.rate_hz = self.get_parameter('rate').value
        self.mission_type = self.get_parameter('mission_type').value
        self.hover_duration = self.get_parameter('hover_duration').value

        # ===================== STATE VARIABLES =====================
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.current_velocity = TwistStamped()
        self.global_position = NavSatFix()
        self.battery = BatteryState()
        self.pose_received = False
        self.mission_active = True

        # ===================== QOS =====================
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

        # ===================== SUBSCRIBERS =====================
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self._state_cb, qos_reliable)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self._pose_cb, qos_profile)
        self.vel_sub = self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local', self._vel_cb, qos_profile)
        self.gps_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self._gps_cb, qos_profile)
        self.battery_sub = self.create_subscription(
            BatteryState, '/mavros/battery', self._battery_cb, qos_profile)

        # ===================== PUBLISHERS =====================
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
        self.vel_pub = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)

        # ===================== SERVICE CLIENTS =====================
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')
        self.stream_rate_client = self.create_client(StreamRate, '/mavros/set_stream_rate')
        self.command_client = self.create_client(CommandLong, '/mavros/cmd/command')

        self.get_logger().info('[MAVROS] Control node siap.')
        self.get_logger().info(
            f'[MAVROS] Altitude: {self.altitude:.1f} m | '
            f'Threshold: {self.threshold:.2f} m | Rate: {self.rate_hz} Hz')

    # ======================== CALLBACKS ========================

    def _state_cb(self, msg):
        self.current_state = msg

    def _pose_cb(self, msg):
        self.current_pose = msg
        self.pose_received = True

    def _vel_cb(self, msg):
        self.current_velocity = msg

    def _gps_cb(self, msg):
        self.global_position = msg

    def _battery_cb(self, msg):
        self.battery = msg

    # ======================== UTILITY ========================

    def spin_once(self, timeout_sec=0.0):
        """Spin sekali untuk proses callback."""
        rclpy.spin_once(self, timeout_sec=timeout_sec)

    def sleep_rate(self):
        """Sleep sesuai rate_hz dengan spin."""
        rate = self.create_rate(self.rate_hz)
        rate.sleep()

    def euclidean_distance(self, target):
        """Hitung jarak Euclidean 3D ke target PoseStamped."""
        dx = self.current_pose.pose.position.x - target.pose.position.x
        dy = self.current_pose.pose.position.y - target.pose.position.y
        dz = self.current_pose.pose.position.z - target.pose.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def euclidean_distance_2d(self, target):
        """Hitung jarak Euclidean 2D (horizontal) ke target PoseStamped."""
        dx = self.current_pose.pose.position.x - target.pose.position.x
        dy = self.current_pose.pose.position.y - target.pose.position.y
        return math.sqrt(dx * dx + dy * dy)

    def create_pose(self, x, y, z):
        """Buat PoseStamped message."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.w = 1.0
        return pose

    def get_current_position(self):
        """Ambil posisi lokal saat ini (x, y, z)."""
        p = self.current_pose.pose.position
        return p.x, p.y, p.z

    def get_gps_position(self):
        """Ambil posisi GPS (lat, lon, alt)."""
        return (self.global_position.latitude,
                self.global_position.longitude,
                self.global_position.altitude)

    def get_battery_info(self):
        """Ambil info baterai (voltage, percentage)."""
        return self.battery.voltage, self.battery.percentage

    def print_status(self):
        """Print status drone ke log."""
        x, y, z = self.get_current_position()
        lat, lon, alt = self.get_gps_position()
        volt, pct = self.get_battery_info()
        self.get_logger().info('--- Drone Status ---')
        self.get_logger().info(f'  Mode     : {self.current_state.mode}')
        self.get_logger().info(f'  Armed    : {self.current_state.armed}')
        self.get_logger().info(f'  Connected: {self.current_state.connected}')
        self.get_logger().info(f'  Lokal    : ({x:.2f}, {y:.2f}, {z:.2f})')
        self.get_logger().info(f'  GPS      : ({lat:.6f}, {lon:.6f}, {alt:.2f})')
        self.get_logger().info(f'  Baterai  : {volt:.2f}V ({pct*100:.0f}%)' if pct > 0
                               else f'  Baterai  : {volt:.2f}V (N/A)')
        self.get_logger().info('--------------------')

    def is_mode(self, mode_name):
        return self.current_state.mode == mode_name

    def is_armed(self):
        return self.current_state.armed

    def is_connected(self):
        return self.current_state.connected

    def safety_check(self):
        """Cek mode masih GUIDED, jika tidak hentikan misi."""
        if not self.is_mode('GUIDED') and self.mission_active:
            self.get_logger().warn(
                f'[SAFETY] Mode berubah ke {self.current_state.mode}! Menghentikan misi.')
            self.mission_active = False
            return False
        return True

    # ======================== SERVICE HELPERS ========================

    def _wait_for_service(self, client, name, timeout=5.0):
        """Tunggu service tersedia."""
        self.get_logger().info(f'[MAVROS] Menunggu service {name} ...')
        while not client.wait_for_service(timeout_sec=timeout):
            if not rclpy.ok():
                return False
            self.get_logger().warn(f'[MAVROS] Service {name} belum tersedia, menunggu ...')
        return True

    def _call_service_sync(self, client, request):
        """Panggil service secara sinkron."""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is not None:
            return future.result()
        return None

    # ======================== CORE FUNCTIONS ========================

    def wait_for_connection(self):
        """Tunggu sampai FCU terhubung."""
        self.get_logger().info('[MAVROS] Menunggu koneksi FCU ...')
        while rclpy.ok() and not self.current_state.connected:
            self.spin_once(timeout_sec=0.05)
        self.get_logger().info('[MAVROS] FCU terhubung!')

    def request_data_stream(self, stream_rate=10):
        """Request data stream dari ArduPilot."""
        self.get_logger().info(f'[MAVROS] Requesting data stream {stream_rate} Hz ...')
        if not self._wait_for_service(self.stream_rate_client, '/mavros/set_stream_rate'):
            return
        req = StreamRate.Request()
        req.stream_id = 0
        req.message_rate = stream_rate
        req.on_off = True
        resp = self._call_service_sync(self.stream_rate_client, req)
        if resp is not None:
            self.get_logger().info('[MAVROS] Data stream OK!')
        else:
            self.get_logger().warn('[MAVROS] Data stream gagal!')

    def wait_for_pose(self):
        """Tunggu data pose pertama masuk."""
        self.get_logger().info('[MAVROS] Menunggu data pose ...')
        while rclpy.ok() and not self.pose_received:
            self.spin_once(timeout_sec=0.05)
        x, y, z = self.get_current_position()
        self.get_logger().info(f'[MAVROS] Pose diterima: ({x:.2f}, {y:.2f}, {z:.2f})')

    def set_mode(self, mode_name):
        """Set flight mode (GUIDED, LAND, LOITER, RTL, STABILIZE, dll.)."""
        if not self._wait_for_service(self.set_mode_client, '/mavros/set_mode'):
            return False

        self.get_logger().info(f'[MAVROS] Set mode -> {mode_name} ...')
        last_req_time = self.get_clock().now()
        interval = rclpy.duration.Duration(seconds=2.0)

        while rclpy.ok():
            self.spin_once(timeout_sec=0.05)
            if self.current_state.mode != mode_name:
                now = self.get_clock().now()
                if (now - last_req_time) > interval:
                    req = SetMode.Request()
                    req.custom_mode = mode_name
                    resp = self._call_service_sync(self.set_mode_client, req)
                    if resp and resp.mode_sent:
                        self.get_logger().info(f'[MAVROS] Request {mode_name} terkirim.')
                    else:
                        self.get_logger().warn('[MAVROS] Set mode gagal.')
                    last_req_time = self.get_clock().now()
            else:
                self.get_logger().info(f'[MAVROS] Mode {mode_name} aktif!')
                return True
        return False

    def arm(self):
        """Arm drone."""
        if not self._wait_for_service(self.arming_client, '/mavros/cmd/arming'):
            return False

        self.get_logger().info('[MAVROS] ARM drone ...')
        last_req_time = self.get_clock().now()
        interval = rclpy.duration.Duration(seconds=2.0)

        while rclpy.ok():
            self.spin_once(timeout_sec=0.05)
            if not self.current_state.armed:
                now = self.get_clock().now()
                if (now - last_req_time) > interval:
                    req = CommandBool.Request()
                    req.value = True
                    resp = self._call_service_sync(self.arming_client, req)
                    if resp and resp.success:
                        self.get_logger().info('[MAVROS] ARM request terkirim.')
                    else:
                        self.get_logger().warn('[MAVROS] ARM gagal.')
                    last_req_time = self.get_clock().now()
            else:
                self.get_logger().info('[MAVROS] Drone ARMED!')
                return True
        return False

    def disarm(self):
        """Disarm drone."""
        self.get_logger().info('[MAVROS] DISARM drone ...')
        req = CommandBool.Request()
        req.value = False
        resp = self._call_service_sync(self.arming_client, req)
        if resp and resp.success:
            self.get_logger().info('[MAVROS] Drone DISARMED!')
            return True
        self.get_logger().warn('[MAVROS] DISARM gagal.')
        return False

    def takeoff(self, altitude=None):
        """Takeoff ke ketinggian tertentu."""
        alt = altitude if altitude else self.altitude
        if not self._wait_for_service(self.takeoff_client, '/mavros/cmd/takeoff'):
            return False

        self.get_logger().info(f'[MAVROS] Takeoff -> {alt:.1f} m ...')
        last_req_time = self.get_clock().now()
        interval = rclpy.duration.Duration(seconds=2.0)

        while rclpy.ok():
            self.spin_once(timeout_sec=0.05)
            now = self.get_clock().now()
            if (now - last_req_time) > interval:
                req = CommandTOL.Request()
                req.altitude = float(alt)
                resp = self._call_service_sync(self.takeoff_client, req)
                if resp and resp.success:
                    self.get_logger().info('[MAVROS] Takeoff command diterima!')
                    break
                else:
                    self.get_logger().warn('[MAVROS] Takeoff gagal, coba lagi ...')
                last_req_time = self.get_clock().now()

        # Tunggu mencapai ketinggian
        self.get_logger().info(f'[MAVROS] Menunggu ketinggian {alt:.1f} m ...')
        while rclpy.ok():
            self.spin_once(timeout_sec=0.05)
            current_alt = self.current_pose.pose.position.z
            if current_alt >= alt * 0.90:
                self.get_logger().info(f'[MAVROS] Ketinggian tercapai! ({current_alt:.2f} m)')
                return True
        return False

    def land(self):
        """Landing menggunakan mode LAND ArduPilot."""
        self.get_logger().info('[MAVROS] Landing ...')
        self.set_mode('LAND')

        self.get_logger().info('[MAVROS] Menunggu drone mendarat ...')
        while rclpy.ok() and self.current_state.armed:
            self.spin_once(timeout_sec=0.05)
        self.get_logger().info('[MAVROS] Drone mendarat dengan selamat!')
        return True

    def rtl(self):
        """Return to Launch."""
        self.get_logger().info('[MAVROS] Return to Launch (RTL) ...')
        return self.set_mode('RTL')

    def loiter(self):
        """Set mode LOITER."""
        self.get_logger().info('[MAVROS] LOITER ...')
        return self.set_mode('LOITER')

    # ======================== NAVIGATION ========================

    def goto(self, x, y, z, label=''):
        """
        Navigasi ke posisi (x, y, z) menggunakan distance-based waypoint.
        TANPA time.sleep() - cek jarak Euclidean terus menerus.
        """
        target = self.create_pose(x, y, z)
        tag = label if label else f'({x:.2f}, {y:.2f}, {z:.2f})'
        self.get_logger().info(f'[NAV] Menuju {tag} ...')

        while rclpy.ok():
            self.spin_once(timeout_sec=0.05)
            if not self.safety_check():
                return False

            target.header.stamp = self.get_clock().now().to_msg()
            self.setpoint_pub.publish(target)

            dist = self.euclidean_distance(target)
            if dist < self.threshold:
                self.get_logger().info(f'[NAV] {tag} tercapai! (jarak: {dist:.3f} m)')
                return True
        return False

    def goto_relative(self, dx, dy, dz, label=''):
        """Navigasi relatif dari posisi saat ini."""
        x0, y0, z0 = self.get_current_position()
        return self.goto(x0 + dx, y0 + dy, z0 + dz, label)

    def hover_at(self, x, y, z, duration=10.0):
        """Hover di posisi tertentu selama duration detik."""
        target = self.create_pose(x, y, z)
        self.get_logger().info(
            f'[NAV] Hover di ({x:.2f}, {y:.2f}, {z:.2f}) selama {duration:.1f} detik ...')

        start = self.get_clock().now()
        while rclpy.ok():
            self.spin_once(timeout_sec=0.05)
            if not self.safety_check():
                return False

            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed >= duration:
                self.get_logger().info(f'[NAV] Hover selesai! ({elapsed:.1f} detik)')
                return True

            target.header.stamp = self.get_clock().now().to_msg()
            self.setpoint_pub.publish(target)
        return False

    def hover_here(self, duration=10.0):
        """Hover di posisi saat ini selama duration detik."""
        x, y, z = self.get_current_position()
        return self.hover_at(x, y, z, duration)

    def send_velocity(self, vx, vy, vz, duration=1.0):
        """Kirim perintah velocity selama duration detik."""
        vel = TwistStamped()
        vel.header.frame_id = 'map'
        vel.twist.linear.x = float(vx)
        vel.twist.linear.y = float(vy)
        vel.twist.linear.z = float(vz)

        self.get_logger().info(
            f'[NAV] Velocity ({vx:.2f}, {vy:.2f}, {vz:.2f}) selama {duration:.1f} detik')

        start = self.get_clock().now()
        while rclpy.ok():
            self.spin_once(timeout_sec=0.05)
            if not self.safety_check():
                return False
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed >= duration:
                return True
            vel.header.stamp = self.get_clock().now().to_msg()
            self.vel_pub.publish(vel)
        return False

    # ======================== MISSION PATTERNS ========================

    def mission_hover(self):
        """Misi sederhana: takeoff, hover, land."""
        self.get_logger().info('=' * 50)
        self.get_logger().info('  MISI: HOVER')
        self.get_logger().info('=' * 50)

        x, y, _ = self.get_current_position()
        z = self.altitude

        if not self.goto(x, y, z, 'HOVER_POINT'):
            return False
        if not self.hover_at(x, y, z, self.hover_duration):
            return False
        return True

    def mission_square(self, size=2.0):
        """Misi persegi: navigasi pola kotak size x size meter."""
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'  MISI: SQUARE ({size:.1f} x {size:.1f} m)')
        self.get_logger().info('=' * 50)

        x0, y0, _ = self.get_current_position()
        z = self.altitude

        waypoints = [
            (x0 + size, y0,        z, 'WP1-Depan'),
            (x0 + size, y0 + size, z, 'WP2-Kanan'),
            (x0,        y0 + size, z, 'WP3-Belakang'),
            (x0,        y0,        z, 'WP4-Home'),
        ]

        for i, (wx, wy, wz, name) in enumerate(waypoints):
            self.get_logger().info(
                f'[{i+1}/{len(waypoints)}] -> {name} ({wx:.2f}, {wy:.2f}, {wz:.2f})')
            if not self.goto(wx, wy, wz, name):
                self.get_logger().warn(f'Misi square dihentikan di {name}')
                return False

        self.get_logger().info('Pola persegi selesai!')
        return True

    def mission_circle(self, radius=2.0, points=12):
        """Misi lingkaran: navigasi pola lingkaran."""
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'  MISI: CIRCLE (r={radius:.1f} m, {points} titik)')
        self.get_logger().info('=' * 50)

        x0, y0, _ = self.get_current_position()
        z = self.altitude

        for i in range(points + 1):
            angle = 2.0 * math.pi * i / points
            wx = x0 + radius * math.cos(angle)
            wy = y0 + radius * math.sin(angle)
            name = f'C{i % points}'

            self.get_logger().info(
                f'[{i+1}/{points+1}] -> {name} ({wx:.2f}, {wy:.2f}, {z:.2f})')
            if not self.goto(wx, wy, z, name):
                self.get_logger().warn(f'Misi circle dihentikan di {name}')
                return False

        self.get_logger().info('Pola lingkaran selesai!')
        return True

    def mission_waypoints(self, waypoint_list):
        """Misi custom waypoints. waypoint_list: list of (x, y, z) tuples."""
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'  MISI: CUSTOM WAYPOINTS ({len(waypoint_list)} titik)')
        self.get_logger().info('=' * 50)

        for i, (wx, wy, wz) in enumerate(waypoint_list):
            name = f'WP{i+1}'
            self.get_logger().info(
                f'[{i+1}/{len(waypoint_list)}] -> {name} ({wx:.2f}, {wy:.2f}, {wz:.2f})')
            if not self.goto(wx, wy, wz, name):
                self.get_logger().warn(f'Misi dihentikan di {name}')
                return False

        self.get_logger().info('Custom waypoints selesai!')
        return True

    # ======================== MAIN RUN ========================

    def preflight(self):
        """Prosedur pre-flight: koneksi, stream, pose, GUIDED, arm, takeoff."""
        self.wait_for_connection()
        self.request_data_stream()
        self.wait_for_pose()
        self.print_status()

        if not self.set_mode('GUIDED'):
            self.get_logger().error('Gagal set GUIDED mode!')
            return False
        if not self.arm():
            self.get_logger().error('Gagal arm drone!')
            return False
        if not self.takeoff():
            self.get_logger().error('Takeoff gagal!')
            return False

        if not self.is_mode('GUIDED'):
            self.get_logger().warn('Mode berubah setelah takeoff, re-set GUIDED ...')
            self.set_mode('GUIDED')

        self.mission_active = True
        return True

    def run(self):
        """Entry point utama: preflight -> misi -> land."""
        self.get_logger().info('=' * 55)
        self.get_logger().info('   MAVROS DRONE CONTROL (ROS 2 Python)')
        self.get_logger().info('   ArduPilot + MAVROS (GUIDED mode)')
        self.get_logger().info(f'   Misi: {self.mission_type.upper()}')
        self.get_logger().info('=' * 55)

        if not self.preflight():
            self.get_logger().error('Pre-flight gagal! Membatalkan misi.')
            return

        mission_ok = False
        if self.mission_type == 'hover':
            mission_ok = self.mission_hover()
        elif self.mission_type == 'square':
            size = self.get_parameter('square_size').value
            mission_ok = self.mission_square(size)
        elif self.mission_type == 'circle':
            radius = self.get_parameter('circle_radius').value
            points = self.get_parameter('circle_points').value
            mission_ok = self.mission_circle(radius, points)
        else:
            self.get_logger().warn(
                f"Tipe misi '{self.mission_type}' tidak dikenal. Hover saja.")
            mission_ok = self.mission_hover()

        if self.mission_active:
            self.land()

        if mission_ok:
            self.get_logger().info('=' * 55)
            self.get_logger().info('   MISI SELESAI DENGAN SUKSES!')
            self.get_logger().info('=' * 55)
        else:
            self.get_logger().warn('=' * 55)
            self.get_logger().warn('   MISI DIHENTIKAN / GAGAL')
            self.get_logger().warn('=' * 55)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MAVROSControl()
        node.run()
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
