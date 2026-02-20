#!/usr/bin/env python3
"""
Square Mission - Autonomous 2x2m Square Pattern Navigation (ArduPilot)
=======================================================================
Mengembangkan script Python menggunakan rospy dan MAVROS untuk melakukan
misi navigasi otonom membentuk pola persegi sempurna (2x2 meter) pada
ketinggian tetap. Setelah kembali ke titik awal, ubah mode ke LAND secara otomatis.

Logic Requirement: "Distance-Based Waypoint"
- TIDAK menggunakan time.sleep()
- Menghitung jarak Euclidean antara posisi drone saat ini dengan target
- Threshold: jarak < 0.3 meter
- Frekuensi setpoint: 10Hz - 20Hz
- Safety: berhenti jika mode berubah manual (misal STABILIZE)

Digunakan dengan ArduPilot SITL + MAVROS (apm.launch)
"""

import rospy
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest
from mavros_msgs.srv import CommandTOL, CommandTOLRequest
from mavros_msgs.srv import StreamRate, StreamRateRequest


class SquareMission:
    def __init__(self):
        rospy.init_node('square_mission_node', anonymous=True)

        # ---------- Parameters ----------
        self.square_size = rospy.get_param('~square_size', 2.0)      # meter
        self.altitude = rospy.get_param('~altitude', 2.0)            # meter
        self.threshold = rospy.get_param('~threshold', 0.3)          # meter
        self.rate_hz = rospy.get_param('~rate', 20)                  # Hz (10-20)

        # ---------- State Variables ----------
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.pose_received = False
        self.mission_active = True

        # ---------- Subscribers ----------
        self.state_sub = rospy.Subscriber(
            '/mavros/state', State, self.state_cb)
        self.pose_sub = rospy.Subscriber(
            '/mavros/local_position/pose', PoseStamped, self.pose_cb)

        # ---------- Publisher ----------
        self.setpoint_pub = rospy.Publisher(
            '/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        # ---------- Service Clients ----------
        rospy.loginfo("Menunggu service /mavros/cmd/arming ...")
        rospy.wait_for_service('/mavros/cmd/arming')
        self.arming_client = rospy.ServiceProxy(
            '/mavros/cmd/arming', CommandBool)

        rospy.loginfo("Menunggu service /mavros/set_mode ...")
        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode_client = rospy.ServiceProxy(
            '/mavros/set_mode', SetMode)

        rospy.loginfo("Menunggu service /mavros/cmd/takeoff ...")
        rospy.wait_for_service('/mavros/cmd/takeoff')
        self.takeoff_client = rospy.ServiceProxy(
            '/mavros/cmd/takeoff', CommandTOL)

        rospy.loginfo("Menunggu service /mavros/cmd/land ...")
        rospy.wait_for_service('/mavros/cmd/land')
        self.land_client = rospy.ServiceProxy(
            '/mavros/cmd/land', CommandTOL)

        rospy.loginfo("Menunggu service /mavros/set_stream_rate ...")
        rospy.wait_for_service('/mavros/set_stream_rate')
        self.stream_rate_client = rospy.ServiceProxy(
            '/mavros/set_stream_rate', StreamRate)

        # ---------- Rate ----------
        self.rate = rospy.Rate(self.rate_hz)

        rospy.loginfo("Square Mission Node telah siap.")
        rospy.loginfo("Ukuran persegi: %.1f m | Ketinggian: %.1f m | Threshold: %.2f m",
                       self.square_size, self.altitude, self.threshold)

    # ======================= CALLBACKS =======================

    def state_cb(self, msg):
        """Callback untuk menerima state drone dari MAVROS."""
        self.current_state = msg

    def pose_cb(self, msg):
        """Callback untuk menerima posisi lokal drone."""
        self.current_pose = msg
        self.pose_received = True

    # ======================= HELPER METHODS =======================

    def euclidean_distance(self, target):
        """Menghitung jarak Euclidean 3D antara posisi saat ini dan target."""
        dx = self.current_pose.pose.position.x - target.pose.position.x
        dy = self.current_pose.pose.position.y - target.pose.position.y
        dz = self.current_pose.pose.position.z - target.pose.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def create_pose(self, x, y, z):
        """Membuat PoseStamped message."""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        return pose

    def is_mode_guided(self):
        """Cek apakah drone dalam mode GUIDED (ArduPilot)."""
        return self.current_state.mode == "GUIDED"

    def is_armed(self):
        """Cek apakah drone sudah armed."""
        return self.current_state.armed

    def safety_check(self):
        """
        Safety check: jika mode berubah dari GUIDED secara manual
        (misal ke STABILIZE), hentikan pengiriman perintah.
        """
        if not self.is_mode_guided() and self.mission_active:
            rospy.logwarn("Mode berubah ke %s! Menghentikan misi.", self.current_state.mode)
            self.mission_active = False
            return False
        return True

    # ======================= MISSION METHODS =======================

    def wait_for_connection(self):
        """Menunggu koneksi FCU."""
        rospy.loginfo("Menunggu koneksi FCU ...")
        while not rospy.is_shutdown() and not self.current_state.connected:
            self.rate.sleep()
        rospy.loginfo("FCU terhubung!")

    def request_data_stream(self):
        """Request semua stream data dari ArduPilot pada 10Hz."""
        rospy.loginfo("Requesting data stream dari FCU ...")
        try:
            req = StreamRateRequest()
            req.stream_id = 0       # ALL streams
            req.message_rate = 10   # 10 Hz
            req.on_off = True
            self.stream_rate_client(req)
            rospy.loginfo("Data stream berhasil di-request!")
        except rospy.ServiceException as e:
            rospy.logwarn("Gagal request stream: %s", str(e))

    def wait_for_pose(self):
        """Menunggu data pose pertama diterima."""
        rospy.loginfo("Menunggu data pose ...")
        while not rospy.is_shutdown() and not self.pose_received:
            self.rate.sleep()
        rospy.loginfo("Data pose diterima! Posisi: (%.2f, %.2f, %.2f)",
                       self.current_pose.pose.position.x,
                       self.current_pose.pose.position.y,
                       self.current_pose.pose.position.z)

    def set_guided_mode(self):
        """Set flight mode ke GUIDED (ArduPilot)."""
        mode_req = SetModeRequest()
        mode_req.custom_mode = "GUIDED"
        last_request = rospy.Time(0)

        rospy.loginfo("Mencoba set mode GUIDED ...")
        while not rospy.is_shutdown():
            if self.current_state.mode != "GUIDED":
                if (rospy.Time.now() - last_request) > rospy.Duration(2.0):
                    try:
                        resp = self.set_mode_client(mode_req)
                        if resp.mode_sent:
                            rospy.loginfo("Request GUIDED terkirim ...")
                    except rospy.ServiceException as e:
                        rospy.logwarn("Service call gagal: %s", str(e))
                    last_request = rospy.Time.now()
            else:
                rospy.loginfo("Mode GUIDED aktif!")
                return True
            self.rate.sleep()
        return False

    def arm_drone(self):
        """Arm the drone."""
        arm_req = CommandBoolRequest()
        arm_req.value = True
        last_request = rospy.Time(0)

        rospy.loginfo("Mencoba ARM drone ...")
        while not rospy.is_shutdown():
            if not self.current_state.armed:
                if (rospy.Time.now() - last_request) > rospy.Duration(2.0):
                    try:
                        resp = self.arming_client(arm_req)
                        if resp.success:
                            rospy.loginfo("Request ARM terkirim ...")
                    except rospy.ServiceException as e:
                        rospy.logwarn("Service call gagal: %s", str(e))
                    last_request = rospy.Time.now()
            else:
                rospy.loginfo("Drone berhasil di-ARM!")
                return True
            self.rate.sleep()
        return False

    def takeoff(self):
        """Takeoff ke ketinggian target menggunakan ArduPilot takeoff service."""
        rospy.loginfo("Takeoff ke ketinggian %.1f m ...", self.altitude)

        takeoff_req = CommandTOLRequest()
        takeoff_req.altitude = self.altitude

        last_request = rospy.Time(0)
        while not rospy.is_shutdown():
            if (rospy.Time.now() - last_request) > rospy.Duration(2.0):
                try:
                    resp = self.takeoff_client(takeoff_req)
                    if resp.success:
                        rospy.loginfo("Takeoff command diterima!")
                        break
                except rospy.ServiceException as e:
                    rospy.logwarn("Takeoff gagal: %s", str(e))
                last_request = rospy.Time.now()
            self.rate.sleep()

        # Tunggu drone mencapai ketinggian target
        rospy.loginfo("Menunggu drone mencapai ketinggian %.1f m ...", self.altitude)
        while not rospy.is_shutdown():
            current_alt = self.current_pose.pose.position.z
            if current_alt >= self.altitude * 0.90:  # 90% dari target
                rospy.loginfo("Ketinggian tercapai! (%.2f m)", current_alt)
                return True
            self.rate.sleep()
        return False

    def navigate_to_waypoint(self, target, wp_name=""):
        """
        Navigate ke waypoint menggunakan distance-based logic.
        TIDAK menggunakan time.sleep().
        Terus publish setpoint dan cek jarak hingga threshold terpenuhi.
        """
        rospy.loginfo("Navigasi ke waypoint %s: (%.2f, %.2f, %.2f)",
                       wp_name, target.pose.position.x,
                       target.pose.position.y, target.pose.position.z)

        while not rospy.is_shutdown():
            # Safety check - berhenti jika mode berubah dari GUIDED
            if not self.safety_check():
                return False

            # Update timestamp dan publish setpoint
            target.header.stamp = rospy.Time.now()
            self.setpoint_pub.publish(target)

            # Hitung jarak Euclidean
            distance = self.euclidean_distance(target)

            # Cek threshold
            if distance < self.threshold:
                rospy.loginfo("Waypoint %s tercapai! (jarak: %.3f m)", wp_name, distance)
                return True

            # Jaga frekuensi 10-20Hz
            self.rate.sleep()

        return False

    def land_drone(self):
        """Set mode ke LAND untuk mendarat (ArduPilot)."""
        rospy.loginfo("Misi selesai! Memulai pendaratan ...")
        mode_req = SetModeRequest()
        mode_req.custom_mode = "LAND"

        while not rospy.is_shutdown():
            if self.current_state.mode != "LAND":
                try:
                    resp = self.set_mode_client(mode_req)
                    if resp.mode_sent:
                        rospy.loginfo("Mode LAND berhasil diaktifkan!")
                        break
                except rospy.ServiceException as e:
                    rospy.logwarn("Set mode LAND gagal: %s", str(e))
            else:
                break
            self.rate.sleep()

        # Tunggu drone mendarat (armed = False)
        rospy.loginfo("Menunggu drone mendarat ...")
        while not rospy.is_shutdown() and self.current_state.armed:
            self.rate.sleep()

        rospy.loginfo("Drone telah mendarat dengan selamat!")

    # ======================= MAIN MISSION =======================

    def generate_square_waypoints(self):
        """
        Generate waypoint untuk pola persegi 2x2 meter.

        Pola persegi (dilihat dari atas):
            HOME (x0,y0)  --->  WP1 (x0+s, y0)
                ^                      |
                |                      v
            WP4 (x0, y0+s)  <---  WP2 (x0+s, y0+s)

        Lalu kembali ke HOME
        """
        s = self.square_size
        z = self.altitude

        # Ambil posisi awal saat ini sebagai referensi
        x0 = self.current_pose.pose.position.x
        y0 = self.current_pose.pose.position.y

        rospy.loginfo("Posisi awal referensi: (%.2f, %.2f)", x0, y0)

        waypoints = [
            (x0 + s, y0,     z, "WP1"),  # maju
            (x0 + s, y0 + s, z, "WP2"),  # ke samping
            (x0,     y0 + s, z, "WP3"),  # mundur
            (x0,     y0,     z, "WP4"),  # kembali ke awal
        ]

        return [(self.create_pose(x, y, zz), name) for x, y, zz, name in waypoints]

    def run(self):
        """Menjalankan seluruh misi."""
        rospy.loginfo("=" * 50)
        rospy.loginfo("  SQUARE MISSION - 2x2m Autonomous Navigation")
        rospy.loginfo("  ArduPilot + MAVROS (GUIDED mode)")
        rospy.loginfo("=" * 50)

        # 1. Tunggu koneksi FCU
        self.wait_for_connection()

        # 2. Request data stream dari ArduPilot
        self.request_data_stream()

        # 3. Tunggu data pose
        self.wait_for_pose()

        # 3. Set GUIDED mode
        if not self.set_guided_mode():
            rospy.logerr("Gagal set GUIDED mode!")
            return

        # 4. Arm drone
        if not self.arm_drone():
            rospy.logerr("Gagal arm drone!")
            return

        # 5. Takeoff
        if not self.takeoff():
            rospy.logerr("Takeoff gagal!")
            return

        # Setelah takeoff, pastikan masih GUIDED
        if not self.is_mode_guided():
            rospy.logwarn("Mode berubah setelah takeoff: %s", self.current_state.mode)
            self.set_guided_mode()

        self.mission_active = True

        # 6. Generate waypoints persegi
        waypoints = self.generate_square_waypoints()

        rospy.loginfo("-" * 40)
        rospy.loginfo("Memulai navigasi pola persegi %dx%d meter ...",
                       self.square_size, self.square_size)
        rospy.loginfo("Jumlah waypoint: %d", len(waypoints))
        rospy.loginfo("-" * 40)

        # 7. Navigasi ke setiap waypoint
        for i, (wp, name) in enumerate(waypoints):
            rospy.loginfo("[%d/%d] Target: %s (%.2f, %.2f, %.2f)",
                           i + 1, len(waypoints), name,
                           wp.pose.position.x, wp.pose.position.y,
                           wp.pose.position.z)

            success = self.navigate_to_waypoint(wp, name)

            if not success:
                if not self.mission_active:
                    rospy.logwarn("Misi dihentikan karena mode berubah secara manual.")
                else:
                    rospy.logwarn("Misi dihentikan (node shutdown).")
                return

        # 8. Semua waypoint tercapai - LAND
        rospy.loginfo("=" * 40)
        rospy.loginfo("Semua waypoint telah tercapai!")
        rospy.loginfo("=" * 40)
        self.land_drone()

        rospy.loginfo("=" * 50)
        rospy.loginfo("  MISI SELESAI DENGAN SUKSES!")
        rospy.loginfo("=" * 50)


def main():
    try:
        mission = SquareMission()
        mission.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node dihentikan oleh user.")
    except Exception as e:
        rospy.logerr("Error: %s", str(e))


if __name__ == '__main__':
    main()
