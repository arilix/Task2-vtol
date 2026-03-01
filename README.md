# Drone Mission — MAVROS Autonomous Navigation

> Paket ROS untuk navigasi otonom drone menggunakan **MAVROS** dan **ArduPilot SITL**.  
> Tersedia dalam **ROS 1 (Noetic)** dan **ROS 2 (Foxy & Humble)**, masing-masing dengan versi **Python** dan **C++**.

---

## Daftar Isi

- [Fitur](#fitur)
- [Arsitektur](#arsitektur)
- [Struktur Direktori](#struktur-direktori)
- [Prasyarat](#prasyarat)
- [Instalasi & Build](#instalasi--build)
  - [ROS 1 (Noetic)](#ros-1-noetic)
  - [ROS 2 (Foxy / Humble)](#ros-2-foxy--humble)
- [Cara Menjalankan](#cara-menjalankan)
  - [ROS 1](#menjalankan-ros-1)
  - [ROS 2](#menjalankan-ros-2)
- [Node & Script](#node--script)
  - [mavros_control](#mavros_control)
  - [square_mission](#square_mission)
- [Parameter](#parameter)
- [Launch Files](#launch-files)
- [Logika Navigasi](#logika-navigasi)
- [Lisensi](#lisensi)

---

## Fitur

| Fitur | Keterangan |
|---|---|
| **Multi-ROS** | Mendukung ROS 1 Noetic dan ROS 2 Foxy/Humble |
| **Multi-Bahasa** | Setiap node tersedia dalam Python dan C++ |
| **Distance-Based Waypoint** | Navigasi tanpa `sleep()` — berdasarkan jarak Euclidean |
| **Misi Beragam** | Hover, Square, Circle, Custom Waypoints |
| **Safety Check** | Otomatis berhenti jika mode berubah dari GUIDED |
| **Configurable** | Semua parameter dapat diatur melalui launch arguments |
| **ArduPilot SITL** | Siap digunakan untuk simulasi ArduPilot |

---

## Arsitektur

```
┌──────────────┐     MAVLink      ┌──────────────┐    setpoint     ┌──────────────┐
│  ArduPilot   │◄────────────────►│    MAVROS    │◄──────────────► │ drone_mission│
│    SITL      │  tcp://127.0.0   │  (ROS Bridge)│  /mavros/...    │   (Node)     │
└──────────────┘   .1:5762        └──────────────┘                 └──────────────┘
```

**Alur misi tipikal:**
1. Tunggu koneksi FCU
2. Request data stream
3. Tunggu data pose
4. Set mode **GUIDED**
5. **ARM** drone
6. **Takeoff** ke ketinggian target
7. Navigasi waypoint-by-waypoint (distance-based)
8. **LAND** setelah semua waypoint tercapai

---

## Struktur Direktori

### ROS 1 — `ros1_ws/`

```
ros1_ws/
└── src/
    ├── CMakeLists.txt                         # catkin toplevel
    └── drone_mission/
        ├── CMakeLists.txt                     # Build: catkin, C++11
        ├── package.xml                        # Format 2
        ├── launch/
        │   ├── mavros_control.launch          # Python + MAVROS
        │   ├── mavros_control_cpp.launch      # C++ + MAVROS
        │   ├── square_mission.launch          # Python square + MAVROS
        │   ├── square_mission_cpp.launch      # C++ square + MAVROS
        │   └── mission_only.launch            # Python square tanpa MAVROS
        ├── scripts/
        │   ├── mavros_control.py              # Python: kontrol lengkap
        │   └── square_mission.py              # Python: misi persegi
        └── src/
            ├── mavros_control_node.cpp        # C++: kontrol lengkap
            └── square_mission_node.cpp        # C++: misi persegi
```

### ROS 2 — `ros2_ws/`

```
ros2_ws/
└── src/
    └── drone_mission/
        ├── CMakeLists.txt                     # Build: ament_cmake, C++14
        ├── package.xml                        # Format 3
        ├── launch/
        │   ├── mavros_control.launch.py       # Python + launch args
        │   ├── mavros_control_cpp.launch.py   # C++ + launch args
        │   ├── square_mission.launch.py       # Python square
        │   └── square_mission_cpp.launch.py   # C++ square
        ├── scripts/
        │   ├── mavros_control.py              # rclpy: kontrol lengkap
        │   └── square_mission.py              # rclpy: misi persegi
        └── src/
            ├── mavros_control_node.cpp        # rclcpp: kontrol lengkap
            └── square_mission_node.cpp        # rclcpp: misi persegi
```

---

## Prasyarat

### Umum
- **Ubuntu 20.04** (Noetic/Foxy) atau **Ubuntu 22.04** (Humble)
- **ArduPilot SITL** — simulator autopilot
- **Python 3**

### ROS 1 (Noetic)
- `ros-noetic-desktop-full`
- `ros-noetic-mavros`
- `ros-noetic-mavros-extras`

```bash
# Install MAVROS geographiclib datasets
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

### ROS 2 (Foxy)
- `ros-foxy-desktop`
- `ros-foxy-mavros`
- `ros-foxy-mavros-extras`

### ROS 2 (Humble)
- `ros-humble-desktop`
- `ros-humble-mavros`
- `ros-humble-mavros-extras`

---

## Instalasi & Build

### ROS 1 (Noetic)

> **Build system:** `catkin_make` — build tool bawaan ROS 1 yang berbasis CMake.  
> Hasil build tersimpan di folder `devel/` dan `build/`.

```bash
# Source ROS environment agar perintah ROS tersedia di terminal
source /opt/ros/noetic/setup.bash

# Masuk ke workspace catkin
cd ~/tutorial/ros1_ws

# Build semua package dalam workspace menggunakan catkin_make
# catkin_make akan mengompilasi source C++ dan menyiapkan script Python
catkin_make

# Source workspace agar package yang sudah di-build dapat digunakan
source devel/setup.bash
```

### ROS 2 (Foxy / Humble)

> **Build system:** `colcon build` — build tool standar ROS 2 yang mendukung `ament_cmake` dan `ament_python`.  
> Hasil build tersimpan di folder `install/`, `build/`, dan `log/`.

```bash
# Source ROS 2 environment (pilih salah satu sesuai distro yang terinstall)
source /opt/ros/foxy/setup.bash     # Foxy
# atau
source /opt/ros/humble/setup.bash   # Humble

# Masuk ke workspace ROS 2
cd ~/tutorial/ros2_ws

# Build semua package menggunakan colcon (pengganti catkin di ROS 2)
# colcon menggunakan ament_cmake sebagai build type untuk package C++
colcon build

# Source workspace agar package yang sudah di-build dapat digunakan
source install/setup.bash
```

---

## Cara Menjalankan

### Persiapan: Jalankan ArduPilot SITL

```bash
# Terminal 1 — ArduPilot SITL
cd ~/ardupilot
sim_vehicle.py -v ArduCopter --console --map
```

### Menjalankan ROS 1

```bash
# Terminal 2 — Source workspace
source ~/tutorial/ros1_ws/devel/setup.bash

# Misi hover (Python)
roslaunch drone_mission mavros_control.launch mission_type:=hover

# Misi persegi (Python)
roslaunch drone_mission square_mission.launch

# Misi persegi (C++)
roslaunch drone_mission square_mission_cpp.launch

# Misi lengkap (C++) dengan parameter kustom
roslaunch drone_mission mavros_control_cpp.launch \
    mission_type:=circle \
    altitude:=3.0 \
    circle_radius:=3.0 \
    circle_points:=24

# Jika MAVROS sudah berjalan terpisah
roslaunch drone_mission mission_only.launch
```

### Menjalankan ROS 2

```bash
# Terminal 2 — Source workspace
source ~/tutorial/ros2_ws/install/setup.bash

# MAVROS2 harus sudah berjalan terlebih dahulu
# ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5762

# Misi hover (Python)
ros2 launch drone_mission mavros_control.launch.py mission_type:=hover

# Misi persegi (Python)
ros2 launch drone_mission square_mission.launch.py

# Misi persegi (C++)
ros2 launch drone_mission square_mission_cpp.launch.py

# Misi lingkaran (C++) dengan parameter kustom
ros2 launch drone_mission mavros_control_cpp.launch.py \
    mission_type:=circle \
    altitude:=3.0 \
    circle_radius:=3.0 \
    circle_points:=24
```

---

## Node & Script

### mavros_control

Node kontrol drone lengkap (*reusable*) yang mendukung berbagai tipe misi.

| Bahasa | ROS 1 | ROS 2 |
|---|---|---|
| Python | `scripts/mavros_control.py` | `scripts/mavros_control.py` |
| C++ | `src/mavros_control_node.cpp` | `src/mavros_control_node.cpp` |

**Class:** `MAVROSControl`

**Tipe Misi:**
- **`hover`** — Takeoff, hover selama durasi tertentu, lalu landing
- **`square`** — Navigasi pola persegi (default 2×2 m)
- **`circle`** — Navigasi pola lingkaran (default radius 2 m, 12 titik)
- **`waypoints`** — Navigasi ke daftar waypoint kustom

**Metode Utama:**

| Metode | Deskripsi |
|---|---|
| `preflight()` | Urutan pre-flight lengkap: koneksi → stream → pose → GUIDED → ARM → takeoff |
| `run()` | Entry point: preflight → misi → land |
| `goto(x, y, z)` | Navigasi ke koordinat absolut (distance-based) |
| `goto_relative(dx, dy, dz)` | Navigasi relatif terhadap posisi saat ini |
| `hover_at(x, y, z, duration)` | Tahan posisi selama durasi tertentu |
| `send_velocity(vx, vy, vz, duration)` | Kirim perintah kecepatan |
| `safety_check()` | Hentikan misi jika mode berubah dari GUIDED |

**Topik ROS (Subscribe):**

| Topik | Tipe | Keterangan |
|---|---|---|
| `/mavros/state` | `mavros_msgs/State` | Status koneksi & mode |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | Posisi lokal drone |
| `/mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` | Kecepatan lokal |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | Posisi GPS global |
| `/mavros/battery` | `sensor_msgs/BatteryState` | Status baterai |

**Topik ROS (Publish):**

| Topik | Tipe | Keterangan |
|---|---|---|
| `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | Setpoint posisi |
| `/mavros/setpoint_velocity/cmd_vel` | `geometry_msgs/TwistStamped` | Setpoint kecepatan |

**Service Clients:**

| Service | Tipe | Keterangan |
|---|---|---|
| `/mavros/cmd/arming` | `CommandBool` | ARM / DISARM |
| `/mavros/set_mode` | `SetMode` | Ubah flight mode |
| `/mavros/cmd/takeoff` | `CommandTOL` | Perintah takeoff |
| `/mavros/cmd/land` | `CommandTOL` | Perintah landing |
| `/mavros/set_stream_rate` | `StreamRate` | Request data stream |

---

### square_mission

Node khusus untuk navigasi pola persegi. Lebih sederhana dan fokus dibanding `mavros_control`.

| Bahasa | ROS 1 | ROS 2 |
|---|---|---|
| Python | `scripts/square_mission.py` | `scripts/square_mission.py` |
| C++ | `src/square_mission_node.cpp` | `src/square_mission_node.cpp` |

**Class:** `SquareMission`

**Alur Misi:**
```
Koneksi FCU → Data Stream → Pose → GUIDED → ARM → Takeoff
    → WP1 (+x, y)
    → WP2 (+x, +y)
    → WP3 (x, +y)
    → WP4 (x, y)       ← kembali ke awal
    → LAND
```

**Pola Persegi (default 2×2 m):**
```
  WP3 ←────── WP2
   │            ↑
   │   2m × 2m  │
   ↓            │
  WP4 ──────→ WP1
  (start)
```

---

## Parameter

### mavros_control

| Parameter | Tipe | Default | Keterangan |
|---|---|---|---|
| `mission_type` | string | `"hover"` | Tipe misi: `hover`, `square`, `circle` |
| `altitude` | double | `2.0` | Ketinggian terbang (m) |
| `threshold` | double | `0.3` | Jarak threshold waypoint (m) |
| `rate` | int | `20` | Frekuensi publish setpoint (Hz) |
| `hover_duration` | double | `10.0` | Durasi hover (detik) |
| `square_size` | double | `2.0` | Ukuran sisi persegi (m) |
| `circle_radius` | double | `2.0` | Radius lingkaran (m) |
| `circle_points` | int | `12` | Jumlah titik dalam lingkaran |

### square_mission

| Parameter | Tipe | Default | Keterangan |
|---|---|---|---|
| `square_size` | double | `2.0` | Ukuran sisi persegi (m) |
| `altitude` | double | `2.0` | Ketinggian terbang (m) |
| `threshold` | double | `0.3` | Jarak threshold waypoint (m) |
| `rate` | int | `20` | Frekuensi publish setpoint (Hz) |

---

## Launch Files

### ROS 1

| Launch File | Deskripsi | Termasuk MAVROS |
|---|---|---|
| `mavros_control.launch` | Kontrol lengkap (Python) | ✅ Ya |
| `mavros_control_cpp.launch` | Kontrol lengkap (C++) | ✅ Ya |
| `square_mission.launch` | Misi persegi (Python) | ✅ Ya |
| `square_mission_cpp.launch` | Misi persegi (C++) | ✅ Ya |
| `mission_only.launch` | Misi persegi tanpa MAVROS | ❌ Tidak |

**FCU URL default:** `tcp://127.0.0.1:5762`

### ROS 2

| Launch File | Deskripsi |
|---|---|
| `mavros_control.launch.py` | Kontrol lengkap (Python) |
| `mavros_control_cpp.launch.py` | Kontrol lengkap (C++) |
| `square_mission.launch.py` | Misi persegi (Python) |
| `square_mission_cpp.launch.py` | Misi persegi (C++) |

> **Catatan:** Pada ROS 2, jalankan MAVROS2 terlebih dahulu secara terpisah sebelum menjalankan node misi.

---

## Logika Navigasi

### Distance-Based Waypoint (Bukan Sleep-Based)

Semua node menggunakan pendekatan **distance-based** untuk navigasi:

```
while jarak(posisi_sekarang, target) > threshold:
    publish setpoint ke target
    cek safety (mode masih GUIDED?)
    sleep(1/rate)    ← hanya delay loop, bukan timing navigasi
```

**Mengapa distance-based?**
- ❌ `sleep(5)` → drone belum tentu sampai, atau sudah lewat / overshoot
- ✅ Distance check → waypoint dianggap tercapai **hanya jika** jarak < threshold

**Rumus jarak:**

$$d = \sqrt{(x_1 - x_2)^2 + (y_1 - y_2)^2 + (z_1 - z_2)^2}$$

**Konfigurasi navigasi:**
- **Threshold:** 0.3 meter (default) — waypoint dianggap tercapai jika jarak < 0.3 m
- **Rate:** 20 Hz (default) — setpoint dikirim 20 kali per detik
- **Safety:** Misi otomatis berhenti jika mode berubah dari GUIDED (misalnya pilot mengambil alih kontrol manual)

---

## Perbedaan ROS 1 vs ROS 2

| Aspek | ROS 1 (Noetic) | ROS 2 (Foxy/Humble) |
|---|---|---|
| **Python API** | `rospy` | `rclpy` |
| **C++ API** | `roscpp` | `rclcpp` |
| **C++ Standard** | C++11 | C++14 |
| **Build System** | `catkin_make` | `colcon build` / `ament_cmake` |
| **Package Format** | format 2 | format 3 |
| **Launch Format** | XML (`.launch`) | Python (`.launch.py`) |
| **QoS** | Tidak ada | `SensorDataQoS`, `ReliableQoS` |
| **Service Calls** | Synchronous | Async + `spin_until_future_complete` |
| **Logging** | `rospy.loginfo()` | `self.get_logger().info()` |
| **Parameter** | `rospy.get_param()` | `declare_parameter()` + `get_parameter()` |

---

## Lisensi

BSD 2-Clause License — Lihat file [LICENSE](LICENSE) untuk detail.

---

## Penulis

**arilix** — Drone Mission Package untuk pembelajaran MAVROS + ArduPilot SITL.
