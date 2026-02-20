#!/usr/bin/env python3
"""
MAVROS Drone Control - Reusable Drone Control Interface
=========================================================
Script Python menggunakan rospy dan MAVROS untuk kontrol drone
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
  roslaunch drone_mission mavros_control.launch
  atau
  rosrun drone_mission mavros_control.py

Digunakan dengan ArduPilot SITL + MAVROS (apm.launch)
"""

import rospy
import math
import sys
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import (
    CommandBool, CommandBoolRequest,
    SetMode, SetModeRequest,
    CommandTOL, CommandTOLRequest,
    StreamRate, StreamRateRequest,
    CommandLong, CommandLongRequest,
)
from sensor_msgs.msg import NavSatFix, BatteryState


class MAVROSControl:
    """
    Kelas utama untuk kontrol drone melalui MAVROS.
    Menyediakan fungsi-fungsi dasar: arm, disarm, set mode,
    takeoff, land, navigate, hover, dan monitoring.
    """

    def __init__(self):
        rospy.init_node('mavros_control_node', anonymous=True)

        # ===================== PARAMETERS =====================
        self.altitude = rospy.get_param('~altitude', 2.0)           # meter
        self.threshold = rospy.get_param('~threshold', 0.3)         # meter
        self.rate_hz = rospy.get_param('~rate', 20)                 # Hz
        self.mission_type = rospy.get_param('~mission_type', 'hover')
        self.hover_duration = rospy.get_param('~hover_duration', 10.0)  # detik

        # ===================== STATE VARIABLES =====================
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.current_velocity = TwistStamped()
        self.global_position = NavSatFix()
        self.battery = BatteryState()
        self.pose_received = False
        self.mission_active = True

        # ===================== SUBSCRIBERS =====================
        self.state_sub = rospy.Subscriber(
            '/mavros/state', State, self._state_cb)
        self.pose_sub = rospy.Subscriber(
            '/mavros/local_position/pose', PoseStamped, self._pose_cb)
        self.vel_sub = rospy.Subscriber(
            '/mavros/local_position/velocity_local', TwistStamped, self._vel_cb)
        self.gps_sub = rospy.Subscriber(
            '/mavros/global_position/global', NavSatFix, self._gps_cb)
        self.battery_sub = rospy.Subscriber(
            '/mavros/battery', BatteryState, self._battery_cb)

        # ===================== PUBLISHER =====================
        self.setpoint_pub = rospy.Publisher(
            '/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.vel_pub = rospy.Publisher(
            '/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

        # ===================== SERVICE CLIENTS =====================
        rospy.loginfo("[MAVROS] Menunggu services ...")

        rospy.wait_for_service('/mavros/cmd/arming')
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        rospy.wait_for_service('/mavros/cmd/takeoff')
        self.takeoff_client = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

        rospy.wait_for_service('/mavros/cmd/land')
        self.land_client = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)

        rospy.wait_for_service('/mavros/set_stream_rate')
        self.stream_rate_client = rospy.ServiceProxy('/mavros/set_stream_rate', StreamRate)

        rospy.wait_for_service('/mavros/cmd/command')
        self.command_client = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)

        # ===================== RATE =====================
        self.rate = rospy.Rate(self.rate_hz)

        rospy.loginfo("[MAVROS] Control node siap.")
        rospy.loginfo("[MAVROS] Altitude: %.1f m | Threshold: %.2f m | Rate: %d Hz",
                       self.altitude, self.threshold, self.rate_hz)

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
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
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
        rospy.loginfo("--- Drone Status ---")
        rospy.loginfo("  Mode     : %s", self.current_state.mode)
        rospy.loginfo("  Armed    : %s", self.current_state.armed)
        rospy.loginfo("  Connected: %s", self.current_state.connected)
        rospy.loginfo("  Lokal    : (%.2f, %.2f, %.2f)", x, y, z)
        rospy.loginfo("  GPS      : (%.6f, %.6f, %.2f)", lat, lon, alt)
        rospy.loginfo("  Baterai  : %.2fV (%.0f%%)", volt, pct * 100 if pct > 0 else 0)
        rospy.loginfo("--------------------")

    def is_mode(self, mode_name):
        """Cek apakah mode saat ini sesuai."""
        return self.current_state.mode == mode_name

    def is_armed(self):
        return self.current_state.armed

    def is_connected(self):
        return self.current_state.connected

    def safety_check(self):
        """Cek mode masih GUIDED, jika tidak hentikan misi."""
        if not self.is_mode("GUIDED") and self.mission_active:
            rospy.logwarn("[SAFETY] Mode berubah ke %s! Menghentikan misi.",
                          self.current_state.mode)
            self.mission_active = False
            return False
        return True

    # ======================== CORE FUNCTIONS ========================

    def wait_for_connection(self):
        """Tunggu sampai FCU terhubung."""
        rospy.loginfo("[MAVROS] Menunggu koneksi FCU ...")
        while not rospy.is_shutdown() and not self.current_state.connected:
            self.rate.sleep()
        rospy.loginfo("[MAVROS] FCU terhubung!")

    def request_data_stream(self, stream_rate=10):
        """Request data stream dari ArduPilot."""
        rospy.loginfo("[MAVROS] Requesting data stream %d Hz ...", stream_rate)
        try:
            req = StreamRateRequest()
            req.stream_id = 0          # ALL streams
            req.message_rate = stream_rate
            req.on_off = True
            self.stream_rate_client(req)
            rospy.loginfo("[MAVROS] Data stream OK!")
        except rospy.ServiceException as e:
            rospy.logwarn("[MAVROS] Data stream gagal: %s", str(e))

    def wait_for_pose(self):
        """Tunggu data pose pertama masuk."""
        rospy.loginfo("[MAVROS] Menunggu data pose ...")
        while not rospy.is_shutdown() and not self.pose_received:
            self.rate.sleep()
        x, y, z = self.get_current_position()
        rospy.loginfo("[MAVROS] Pose diterima: (%.2f, %.2f, %.2f)", x, y, z)

    def set_mode(self, mode_name):
        """Set flight mode (GUIDED, LAND, LOITER, RTL, STABILIZE, dll.)."""
        mode_req = SetModeRequest()
        mode_req.custom_mode = mode_name
        last_req_time = rospy.Time(0)

        rospy.loginfo("[MAVROS] Set mode -> %s ...", mode_name)
        while not rospy.is_shutdown():
            if self.current_state.mode != mode_name:
                if (rospy.Time.now() - last_req_time) > rospy.Duration(2.0):
                    try:
                        resp = self.set_mode_client(mode_req)
                        if resp.mode_sent:
                            rospy.loginfo("[MAVROS] Request %s terkirim.", mode_name)
                    except rospy.ServiceException as e:
                        rospy.logwarn("[MAVROS] Set mode gagal: %s", str(e))
                    last_req_time = rospy.Time.now()
            else:
                rospy.loginfo("[MAVROS] Mode %s aktif!", mode_name)
                return True
            self.rate.sleep()
        return False

    def arm(self):
        """Arm drone."""
        arm_req = CommandBoolRequest()
        arm_req.value = True
        last_req_time = rospy.Time(0)

        rospy.loginfo("[MAVROS] ARM drone ...")
        while not rospy.is_shutdown():
            if not self.current_state.armed:
                if (rospy.Time.now() - last_req_time) > rospy.Duration(2.0):
                    try:
                        resp = self.arming_client(arm_req)
                        if resp.success:
                            rospy.loginfo("[MAVROS] ARM request terkirim.")
                    except rospy.ServiceException as e:
                        rospy.logwarn("[MAVROS] ARM gagal: %s", str(e))
                    last_req_time = rospy.Time.now()
            else:
                rospy.loginfo("[MAVROS] Drone ARMED!")
                return True
            self.rate.sleep()
        return False

    def disarm(self):
        """Disarm drone."""
        arm_req = CommandBoolRequest()
        arm_req.value = False
        rospy.loginfo("[MAVROS] DISARM drone ...")
        try:
            resp = self.arming_client(arm_req)
            if resp.success:
                rospy.loginfo("[MAVROS] Drone DISARMED!")
                return True
        except rospy.ServiceException as e:
            rospy.logwarn("[MAVROS] DISARM gagal: %s", str(e))
        return False

    def takeoff(self, altitude=None):
        """Takeoff ke ketinggian tertentu menggunakan ArduPilot takeoff service."""
        alt = altitude if altitude else self.altitude
        rospy.loginfo("[MAVROS] Takeoff -> %.1f m ...", alt)

        takeoff_req = CommandTOLRequest()
        takeoff_req.altitude = alt

        last_req_time = rospy.Time(0)
        while not rospy.is_shutdown():
            if (rospy.Time.now() - last_req_time) > rospy.Duration(2.0):
                try:
                    resp = self.takeoff_client(takeoff_req)
                    if resp.success:
                        rospy.loginfo("[MAVROS] Takeoff command diterima!")
                        break
                except rospy.ServiceException as e:
                    rospy.logwarn("[MAVROS] Takeoff gagal: %s", str(e))
                last_req_time = rospy.Time.now()
            self.rate.sleep()

        # Tunggu mencapai ketinggian
        rospy.loginfo("[MAVROS] Menunggu ketinggian %.1f m ...", alt)
        while not rospy.is_shutdown():
            current_alt = self.current_pose.pose.position.z
            if current_alt >= alt * 0.90:
                rospy.loginfo("[MAVROS] Ketinggian tercapai! (%.2f m)", current_alt)
                return True
            self.rate.sleep()
        return False

    def land(self):
        """Landing menggunakan mode LAND ArduPilot."""
        rospy.loginfo("[MAVROS] Landing ...")
        self.set_mode("LAND")

        # Tunggu sampai disarmed (sudah mendarat)
        rospy.loginfo("[MAVROS] Menunggu drone mendarat ...")
        while not rospy.is_shutdown() and self.current_state.armed:
            self.rate.sleep()
        rospy.loginfo("[MAVROS] Drone mendarat dengan selamat!")
        return True

    def rtl(self):
        """Return to Launch."""
        rospy.loginfo("[MAVROS] Return to Launch (RTL) ...")
        return self.set_mode("RTL")

    def loiter(self):
        """Set mode LOITER (hover di posisi saat ini)."""
        rospy.loginfo("[MAVROS] LOITER ...")
        return self.set_mode("LOITER")

    # ======================== NAVIGATION ========================

    def goto(self, x, y, z, label=""):
        """
        Navigasi ke posisi (x, y, z) menggunakan distance-based waypoint.
        TANPA time.sleep() - cek jarak Euclidean terus menerus.
        """
        target = self.create_pose(x, y, z)
        tag = label if label else "(%.2f, %.2f, %.2f)" % (x, y, z)
        rospy.loginfo("[NAV] Menuju %s ...", tag)

        while not rospy.is_shutdown():
            if not self.safety_check():
                return False

            target.header.stamp = rospy.Time.now()
            self.setpoint_pub.publish(target)

            dist = self.euclidean_distance(target)
            if dist < self.threshold:
                rospy.loginfo("[NAV] %s tercapai! (jarak: %.3f m)", tag, dist)
                return True

            self.rate.sleep()
        return False

    def goto_relative(self, dx, dy, dz, label=""):
        """Navigasi relatif dari posisi saat ini."""
        x0, y0, z0 = self.get_current_position()
        return self.goto(x0 + dx, y0 + dy, z0 + dz, label)

    def hover_at(self, x, y, z, duration=10.0):
        """Hover di posisi tertentu selama duration detik."""
        target = self.create_pose(x, y, z)
        rospy.loginfo("[NAV] Hover di (%.2f, %.2f, %.2f) selama %.1f detik ...",
                       x, y, z, duration)

        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if not self.safety_check():
                return False

            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed >= duration:
                rospy.loginfo("[NAV] Hover selesai! (%.1f detik)", elapsed)
                return True

            target.header.stamp = rospy.Time.now()
            self.setpoint_pub.publish(target)
            self.rate.sleep()
        return False

    def hover_here(self, duration=10.0):
        """Hover di posisi saat ini selama duration detik."""
        x, y, z = self.get_current_position()
        return self.hover_at(x, y, z, duration)

    def send_velocity(self, vx, vy, vz, duration=1.0):
        """Kirim perintah velocity selama duration detik."""
        vel = TwistStamped()
        vel.header.frame_id = "map"
        vel.twist.linear.x = vx
        vel.twist.linear.y = vy
        vel.twist.linear.z = vz

        rospy.loginfo("[NAV] Velocity (%.2f, %.2f, %.2f) selama %.1f detik",
                       vx, vy, vz, duration)

        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if not self.safety_check():
                return False
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed >= duration:
                return True
            vel.header.stamp = rospy.Time.now()
            self.vel_pub.publish(vel)
            self.rate.sleep()
        return False

    # ======================== MISSION PATTERNS ========================

    def mission_hover(self):
        """Misi sederhana: takeoff, hover, land."""
        rospy.loginfo("=" * 50)
        rospy.loginfo("  MISI: HOVER")
        rospy.loginfo("=" * 50)

        x, y, _ = self.get_current_position()
        z = self.altitude

        # Navigasi ke titik hover dan tahan posisi
        if not self.goto(x, y, z, "HOVER_POINT"):
            return
        if not self.hover_at(x, y, z, self.hover_duration):
            return

    def mission_square(self, size=2.0):
        """
        Misi persegi: navigasi pola kotak size x size meter.

            HOME -------> WP1
              ^             |
              |             v
            WP4 <------- WP2
        """
        rospy.loginfo("=" * 50)
        rospy.loginfo("  MISI: SQUARE (%.1f x %.1f m)", size, size)
        rospy.loginfo("=" * 50)

        x0, y0, _ = self.get_current_position()
        z = self.altitude

        waypoints = [
            (x0 + size, y0,        z, "WP1-Depan"),
            (x0 + size, y0 + size, z, "WP2-Kanan"),
            (x0,        y0 + size, z, "WP3-Belakang"),
            (x0,        y0,        z, "WP4-Home"),
        ]

        for i, (wx, wy, wz, name) in enumerate(waypoints):
            rospy.loginfo("[%d/%d] -> %s (%.2f, %.2f, %.2f)",
                           i + 1, len(waypoints), name, wx, wy, wz)
            if not self.goto(wx, wy, wz, name):
                rospy.logwarn("Misi square dihentikan di %s", name)
                return False

        rospy.loginfo("Pola persegi selesai!")
        return True

    def mission_circle(self, radius=2.0, points=12):
        """
        Misi lingkaran: navigasi pola lingkaran.
        Membagi lingkaran menjadi N titik.
        """
        rospy.loginfo("=" * 50)
        rospy.loginfo("  MISI: CIRCLE (r=%.1f m, %d titik)", radius, points)
        rospy.loginfo("=" * 50)

        x0, y0, _ = self.get_current_position()
        z = self.altitude

        for i in range(points + 1):  # +1 untuk kembali ke titik awal
            angle = 2.0 * math.pi * i / points
            wx = x0 + radius * math.cos(angle)
            wy = y0 + radius * math.sin(angle)
            name = "C%d" % (i % points)

            rospy.loginfo("[%d/%d] -> %s (%.2f, %.2f, %.2f)",
                           i + 1, points + 1, name, wx, wy, z)
            if not self.goto(wx, wy, z, name):
                rospy.logwarn("Misi circle dihentikan di %s", name)
                return False

        rospy.loginfo("Pola lingkaran selesai!")
        return True

    def mission_waypoints(self, waypoint_list):
        """
        Misi custom waypoints.
        waypoint_list: list of (x, y, z) tuples
        """
        rospy.loginfo("=" * 50)
        rospy.loginfo("  MISI: CUSTOM WAYPOINTS (%d titik)", len(waypoint_list))
        rospy.loginfo("=" * 50)

        for i, (wx, wy, wz) in enumerate(waypoint_list):
            name = "WP%d" % (i + 1)
            rospy.loginfo("[%d/%d] -> %s (%.2f, %.2f, %.2f)",
                           i + 1, len(waypoint_list), name, wx, wy, wz)
            if not self.goto(wx, wy, wz, name):
                rospy.logwarn("Misi dihentikan di %s", name)
                return False

        rospy.loginfo("Custom waypoints selesai!")
        return True

    # ======================== MAIN RUN ========================

    def preflight(self):
        """Prosedur pre-flight: koneksi, stream, pose, GUIDED, arm, takeoff."""
        # 1. Koneksi FCU
        self.wait_for_connection()

        # 2. Request data stream
        self.request_data_stream()

        # 3. Tunggu pose
        self.wait_for_pose()

        # 4. Print status awal
        self.print_status()

        # 5. Set GUIDED mode
        if not self.set_mode("GUIDED"):
            rospy.logerr("Gagal set GUIDED mode!")
            return False

        # 6. Arm
        if not self.arm():
            rospy.logerr("Gagal arm drone!")
            return False

        # 7. Takeoff
        if not self.takeoff():
            rospy.logerr("Takeoff gagal!")
            return False

        # Pastikan masih GUIDED setelah takeoff
        if not self.is_mode("GUIDED"):
            rospy.logwarn("Mode berubah setelah takeoff, re-set GUIDED ...")
            self.set_mode("GUIDED")

        self.mission_active = True
        return True

    def run(self):
        """Entry point utama: preflight -> misi -> land."""
        rospy.loginfo("=" * 55)
        rospy.loginfo("   MAVROS DRONE CONTROL")
        rospy.loginfo("   ArduPilot + MAVROS (GUIDED mode)")
        rospy.loginfo("   Misi: %s", self.mission_type.upper())
        rospy.loginfo("=" * 55)

        # Pre-flight
        if not self.preflight():
            rospy.logerr("Pre-flight gagal! Membatalkan misi.")
            return

        # Jalankan misi berdasarkan parameter
        mission_ok = False
        if self.mission_type == 'hover':
            self.mission_hover()
            mission_ok = True
        elif self.mission_type == 'square':
            size = rospy.get_param('~square_size', 2.0)
            mission_ok = self.mission_square(size)
        elif self.mission_type == 'circle':
            radius = rospy.get_param('~circle_radius', 2.0)
            points = rospy.get_param('~circle_points', 12)
            mission_ok = self.mission_circle(radius, points)
        else:
            rospy.logwarn("Tipe misi '%s' tidak dikenal. Hover saja.", self.mission_type)
            self.mission_hover()
            mission_ok = True

        # Landing
        if self.mission_active:
            self.land()

        # Status akhir
        if mission_ok:
            rospy.loginfo("=" * 55)
            rospy.loginfo("   MISI SELESAI DENGAN SUKSES!")
            rospy.loginfo("=" * 55)
        else:
            rospy.logwarn("=" * 55)
            rospy.logwarn("   MISI DIHENTIKAN / GAGAL")
            rospy.logwarn("=" * 55)


def main():
    try:
        ctrl = MAVROSControl()
        ctrl.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node dihentikan oleh user.")
    except Exception as e:
        rospy.logerr("Error: %s", str(e))
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
