/**
 * @file mavros_control_node.cpp
 * @brief MAVROS Drone Control - Reusable Drone Control Interface (C++ Version)
 *
 * Menggunakan roscpp dan MAVROS untuk kontrol drone secara otonom
 * melalui ArduPilot SITL.
 *
 * Fitur:
 *   - Koneksi FCU & data stream request
 *   - Set mode (GUIDED, LAND, LOITER, RTL, STABILIZE, dll.)
 *   - Arm / Disarm
 *   - Takeoff & Land
 *   - Navigasi waypoint (distance-based, tanpa sleep)
 *   - Hover di posisi tertentu
 *   - Safety check (auto-stop jika mode berubah manual)
 *   - Monitoring state & posisi real-time
 *
 * Penggunaan:
 *   roslaunch drone_mission mavros_control_cpp.launch
 *   rosrun drone_mission mavros_control_node
 *
 * Digunakan dengan ArduPilot SITL + MAVROS (apm.launch)
 */

#include <ros/ros.h>
#include <cmath>
#include <string>
#include <vector>
#include <tuple>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/StreamRate.h>
#include <mavros_msgs/CommandLong.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>


class MAVROSControl
{
public:
    MAVROSControl(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh), pose_received_(false), mission_active_(true)
    {
        // ===================== PARAMETERS =====================
        pnh_.param<double>("altitude", altitude_, 2.0);
        pnh_.param<double>("threshold", threshold_, 0.3);
        pnh_.param<int>("rate", rate_hz_, 20);
        pnh_.param<std::string>("mission_type", mission_type_, "hover");
        pnh_.param<double>("hover_duration", hover_duration_, 10.0);

        // ===================== SUBSCRIBERS =====================
        state_sub_   = nh_.subscribe("/mavros/state", 10,
                                     &MAVROSControl::stateCb, this);
        pose_sub_    = nh_.subscribe("/mavros/local_position/pose", 10,
                                     &MAVROSControl::poseCb, this);
        vel_sub_     = nh_.subscribe("/mavros/local_position/velocity_local", 10,
                                     &MAVROSControl::velCb, this);
        gps_sub_     = nh_.subscribe("/mavros/global_position/global", 10,
                                     &MAVROSControl::gpsCb, this);
        battery_sub_ = nh_.subscribe("/mavros/battery", 10,
                                     &MAVROSControl::batteryCb, this);

        // ===================== PUBLISHERS =====================
        setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            "/mavros/setpoint_position/local", 10);
        vel_pub_      = nh_.advertise<geometry_msgs::TwistStamped>(
            "/mavros/setpoint_velocity/cmd_vel", 10);

        // ===================== SERVICE CLIENTS =====================
        ROS_INFO("[MAVROS] Menunggu services ...");

        ros::service::waitForService("/mavros/cmd/arming");
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

        ros::service::waitForService("/mavros/set_mode");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

        ros::service::waitForService("/mavros/cmd/takeoff");
        takeoff_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");

        ros::service::waitForService("/mavros/cmd/land");
        land_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");

        ros::service::waitForService("/mavros/set_stream_rate");
        stream_rate_client_ = nh_.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");

        ros::service::waitForService("/mavros/cmd/command");
        command_client_ = nh_.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");

        // ===================== RATE =====================
        rate_ = new ros::Rate(rate_hz_);

        ROS_INFO("[MAVROS] Control node siap.");
        ROS_INFO("[MAVROS] Altitude: %.1f m | Threshold: %.2f m | Rate: %d Hz",
                 altitude_, threshold_, rate_hz_);
    }

    ~MAVROSControl()
    {
        delete rate_;
    }

    // ======================== CALLBACKS ========================

    void stateCb(const mavros_msgs::State::ConstPtr& msg)
    {
        current_state_ = *msg;
    }

    void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        current_pose_ = *msg;
        pose_received_ = true;
    }

    void velCb(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
        current_velocity_ = *msg;
    }

    void gpsCb(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
        global_position_ = *msg;
    }

    void batteryCb(const sensor_msgs::BatteryState::ConstPtr& msg)
    {
        battery_ = *msg;
    }

    // ======================== UTILITY ========================

    double euclideanDistance(const geometry_msgs::PoseStamped& target) const
    {
        double dx = current_pose_.pose.position.x - target.pose.position.x;
        double dy = current_pose_.pose.position.y - target.pose.position.y;
        double dz = current_pose_.pose.position.z - target.pose.position.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    double euclideanDistance2D(const geometry_msgs::PoseStamped& target) const
    {
        double dx = current_pose_.pose.position.x - target.pose.position.x;
        double dy = current_pose_.pose.position.y - target.pose.position.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    geometry_msgs::PoseStamped createPose(double x, double y, double z) const
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        pose.pose.orientation.w = 1.0;
        return pose;
    }

    void getCurrentPosition(double& x, double& y, double& z) const
    {
        x = current_pose_.pose.position.x;
        y = current_pose_.pose.position.y;
        z = current_pose_.pose.position.z;
    }

    void getGPSPosition(double& lat, double& lon, double& alt) const
    {
        lat = global_position_.latitude;
        lon = global_position_.longitude;
        alt = global_position_.altitude;
    }

    void getBatteryInfo(double& voltage, double& percentage) const
    {
        voltage = battery_.voltage;
        percentage = battery_.percentage;
    }

    void printStatus()
    {
        double x, y, z, lat, lon, alt, volt, pct;
        getCurrentPosition(x, y, z);
        getGPSPosition(lat, lon, alt);
        getBatteryInfo(volt, pct);
        ROS_INFO("--- Drone Status ---");
        ROS_INFO("  Mode     : %s", current_state_.mode.c_str());
        ROS_INFO("  Armed    : %s", current_state_.armed ? "True" : "False");
        ROS_INFO("  Connected: %s", current_state_.connected ? "True" : "False");
        ROS_INFO("  Lokal    : (%.2f, %.2f, %.2f)", x, y, z);
        ROS_INFO("  GPS      : (%.6f, %.6f, %.2f)", lat, lon, alt);
        ROS_INFO("  Baterai  : %.2fV (%.0f%%)", volt, pct > 0 ? pct * 100.0 : 0.0);
        ROS_INFO("--------------------");
    }

    bool isMode(const std::string& mode_name) const
    {
        return current_state_.mode == mode_name;
    }

    bool isArmed() const
    {
        return current_state_.armed;
    }

    bool isConnected() const
    {
        return current_state_.connected;
    }

    bool safetyCheck()
    {
        if (!isMode("GUIDED") && mission_active_)
        {
            ROS_WARN("[SAFETY] Mode berubah ke %s! Menghentikan misi.",
                     current_state_.mode.c_str());
            mission_active_ = false;
            return false;
        }
        return true;
    }

    // ======================== CORE FUNCTIONS ========================

    void waitForConnection()
    {
        ROS_INFO("[MAVROS] Menunggu koneksi FCU ...");
        while (ros::ok() && !current_state_.connected)
        {
            ros::spinOnce();
            rate_->sleep();
        }
        ROS_INFO("[MAVROS] FCU terhubung!");
    }

    void requestDataStream(int stream_rate = 10)
    {
        ROS_INFO("[MAVROS] Requesting data stream %d Hz ...", stream_rate);
        mavros_msgs::StreamRate srv;
        srv.request.stream_id = 0;
        srv.request.message_rate = stream_rate;
        srv.request.on_off = true;
        if (stream_rate_client_.call(srv))
        {
            ROS_INFO("[MAVROS] Data stream OK!");
        }
        else
        {
            ROS_WARN("[MAVROS] Data stream gagal!");
        }
    }

    void waitForPose()
    {
        ROS_INFO("[MAVROS] Menunggu data pose ...");
        while (ros::ok() && !pose_received_)
        {
            ros::spinOnce();
            rate_->sleep();
        }
        double x, y, z;
        getCurrentPosition(x, y, z);
        ROS_INFO("[MAVROS] Pose diterima: (%.2f, %.2f, %.2f)", x, y, z);
    }

    bool setMode(const std::string& mode_name)
    {
        mavros_msgs::SetMode mode_req;
        mode_req.request.custom_mode = mode_name;
        ros::Time last_req_time(0);

        ROS_INFO("[MAVROS] Set mode -> %s ...", mode_name.c_str());
        while (ros::ok())
        {
            ros::spinOnce();
            if (current_state_.mode != mode_name)
            {
                if ((ros::Time::now() - last_req_time) > ros::Duration(2.0))
                {
                    if (set_mode_client_.call(mode_req) && mode_req.response.mode_sent)
                    {
                        ROS_INFO("[MAVROS] Request %s terkirim.", mode_name.c_str());
                    }
                    else
                    {
                        ROS_WARN("[MAVROS] Set mode gagal.");
                    }
                    last_req_time = ros::Time::now();
                }
            }
            else
            {
                ROS_INFO("[MAVROS] Mode %s aktif!", mode_name.c_str());
                return true;
            }
            rate_->sleep();
        }
        return false;
    }

    bool arm()
    {
        mavros_msgs::CommandBool arm_req;
        arm_req.request.value = true;
        ros::Time last_req_time(0);

        ROS_INFO("[MAVROS] ARM drone ...");
        while (ros::ok())
        {
            ros::spinOnce();
            if (!current_state_.armed)
            {
                if ((ros::Time::now() - last_req_time) > ros::Duration(2.0))
                {
                    if (arming_client_.call(arm_req) && arm_req.response.success)
                    {
                        ROS_INFO("[MAVROS] ARM request terkirim.");
                    }
                    else
                    {
                        ROS_WARN("[MAVROS] ARM gagal.");
                    }
                    last_req_time = ros::Time::now();
                }
            }
            else
            {
                ROS_INFO("[MAVROS] Drone ARMED!");
                return true;
            }
            rate_->sleep();
        }
        return false;
    }

    bool disarm()
    {
        mavros_msgs::CommandBool arm_req;
        arm_req.request.value = false;
        ROS_INFO("[MAVROS] DISARM drone ...");
        if (arming_client_.call(arm_req) && arm_req.response.success)
        {
            ROS_INFO("[MAVROS] Drone DISARMED!");
            return true;
        }
        ROS_WARN("[MAVROS] DISARM gagal.");
        return false;
    }

    bool takeoff(double alt = -1.0)
    {
        if (alt < 0) alt = altitude_;
        ROS_INFO("[MAVROS] Takeoff -> %.1f m ...", alt);

        mavros_msgs::CommandTOL takeoff_req;
        takeoff_req.request.altitude = alt;

        ros::Time last_req_time(0);
        while (ros::ok())
        {
            ros::spinOnce();
            if ((ros::Time::now() - last_req_time) > ros::Duration(2.0))
            {
                if (takeoff_client_.call(takeoff_req) && takeoff_req.response.success)
                {
                    ROS_INFO("[MAVROS] Takeoff command diterima!");
                    break;
                }
                else
                {
                    ROS_WARN("[MAVROS] Takeoff gagal, coba lagi ...");
                }
                last_req_time = ros::Time::now();
            }
            rate_->sleep();
        }

        // Tunggu mencapai ketinggian
        ROS_INFO("[MAVROS] Menunggu ketinggian %.1f m ...", alt);
        while (ros::ok())
        {
            ros::spinOnce();
            double current_alt = current_pose_.pose.position.z;
            if (current_alt >= alt * 0.90)
            {
                ROS_INFO("[MAVROS] Ketinggian tercapai! (%.2f m)", current_alt);
                return true;
            }
            rate_->sleep();
        }
        return false;
    }

    bool land()
    {
        ROS_INFO("[MAVROS] Landing ...");
        setMode("LAND");

        ROS_INFO("[MAVROS] Menunggu drone mendarat ...");
        while (ros::ok() && current_state_.armed)
        {
            ros::spinOnce();
            rate_->sleep();
        }
        ROS_INFO("[MAVROS] Drone mendarat dengan selamat!");
        return true;
    }

    bool rtl()
    {
        ROS_INFO("[MAVROS] Return to Launch (RTL) ...");
        return setMode("RTL");
    }

    bool loiter()
    {
        ROS_INFO("[MAVROS] LOITER ...");
        return setMode("LOITER");
    }

    // ======================== NAVIGATION ========================

    bool goTo(double x, double y, double z, const std::string& label = "")
    {
        geometry_msgs::PoseStamped target = createPose(x, y, z);
        std::string tag = label.empty() ?
            ("(" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ")") : label;
        ROS_INFO("[NAV] Menuju %s ...", tag.c_str());

        while (ros::ok())
        {
            ros::spinOnce();
            if (!safetyCheck()) return false;

            target.header.stamp = ros::Time::now();
            setpoint_pub_.publish(target);

            double dist = euclideanDistance(target);
            if (dist < threshold_)
            {
                ROS_INFO("[NAV] %s tercapai! (jarak: %.3f m)", tag.c_str(), dist);
                return true;
            }
            rate_->sleep();
        }
        return false;
    }

    bool goToRelative(double dx, double dy, double dz, const std::string& label = "")
    {
        double x0, y0, z0;
        getCurrentPosition(x0, y0, z0);
        return goTo(x0 + dx, y0 + dy, z0 + dz, label);
    }

    bool hoverAt(double x, double y, double z, double duration = 10.0)
    {
        geometry_msgs::PoseStamped target = createPose(x, y, z);
        ROS_INFO("[NAV] Hover di (%.2f, %.2f, %.2f) selama %.1f detik ...",
                 x, y, z, duration);

        ros::Time start_time = ros::Time::now();
        while (ros::ok())
        {
            ros::spinOnce();
            if (!safetyCheck()) return false;

            double elapsed = (ros::Time::now() - start_time).toSec();
            if (elapsed >= duration)
            {
                ROS_INFO("[NAV] Hover selesai! (%.1f detik)", elapsed);
                return true;
            }
            target.header.stamp = ros::Time::now();
            setpoint_pub_.publish(target);
            rate_->sleep();
        }
        return false;
    }

    bool hoverHere(double duration = 10.0)
    {
        double x, y, z;
        getCurrentPosition(x, y, z);
        return hoverAt(x, y, z, duration);
    }

    bool sendVelocity(double vx, double vy, double vz, double duration = 1.0)
    {
        geometry_msgs::TwistStamped vel;
        vel.header.frame_id = "map";
        vel.twist.linear.x = vx;
        vel.twist.linear.y = vy;
        vel.twist.linear.z = vz;

        ROS_INFO("[NAV] Velocity (%.2f, %.2f, %.2f) selama %.1f detik",
                 vx, vy, vz, duration);

        ros::Time start_time = ros::Time::now();
        while (ros::ok())
        {
            ros::spinOnce();
            if (!safetyCheck()) return false;

            double elapsed = (ros::Time::now() - start_time).toSec();
            if (elapsed >= duration) return true;

            vel.header.stamp = ros::Time::now();
            vel_pub_.publish(vel);
            rate_->sleep();
        }
        return false;
    }

    // ======================== MISSION PATTERNS ========================

    bool missionHover()
    {
        ROS_INFO("==================================================");
        ROS_INFO("  MISI: HOVER");
        ROS_INFO("==================================================");

        double x, y, z_unused;
        getCurrentPosition(x, y, z_unused);
        double z = altitude_;

        if (!goTo(x, y, z, "HOVER_POINT")) return false;
        if (!hoverAt(x, y, z, hover_duration_)) return false;
        return true;
    }

    bool missionSquare(double size = 2.0)
    {
        ROS_INFO("==================================================");
        ROS_INFO("  MISI: SQUARE (%.1f x %.1f m)", size, size);
        ROS_INFO("==================================================");

        double x0, y0, z_unused;
        getCurrentPosition(x0, y0, z_unused);
        double z = altitude_;

        struct Waypoint { double x, y, z; std::string name; };
        std::vector<Waypoint> waypoints = {
            {x0 + size, y0,        z, "WP1-Depan"},
            {x0 + size, y0 + size, z, "WP2-Kanan"},
            {x0,        y0 + size, z, "WP3-Belakang"},
            {x0,        y0,        z, "WP4-Home"},
        };

        for (size_t i = 0; i < waypoints.size(); ++i)
        {
            const auto& wp = waypoints[i];
            ROS_INFO("[%zu/%zu] -> %s (%.2f, %.2f, %.2f)",
                     i + 1, waypoints.size(), wp.name.c_str(), wp.x, wp.y, wp.z);
            if (!goTo(wp.x, wp.y, wp.z, wp.name))
            {
                ROS_WARN("Misi square dihentikan di %s", wp.name.c_str());
                return false;
            }
        }
        ROS_INFO("Pola persegi selesai!");
        return true;
    }

    bool missionCircle(double radius = 2.0, int points = 12)
    {
        ROS_INFO("==================================================");
        ROS_INFO("  MISI: CIRCLE (r=%.1f m, %d titik)", radius, points);
        ROS_INFO("==================================================");

        double x0, y0, z_unused;
        getCurrentPosition(x0, y0, z_unused);
        double z = altitude_;

        for (int i = 0; i <= points; ++i)
        {
            double angle = 2.0 * M_PI * i / points;
            double wx = x0 + radius * std::cos(angle);
            double wy = y0 + radius * std::sin(angle);
            std::string name = "C" + std::to_string(i % points);

            ROS_INFO("[%d/%d] -> %s (%.2f, %.2f, %.2f)",
                     i + 1, points + 1, name.c_str(), wx, wy, z);
            if (!goTo(wx, wy, z, name))
            {
                ROS_WARN("Misi circle dihentikan di %s", name.c_str());
                return false;
            }
        }
        ROS_INFO("Pola lingkaran selesai!");
        return true;
    }

    bool missionWaypoints(const std::vector<std::tuple<double, double, double>>& wps)
    {
        ROS_INFO("==================================================");
        ROS_INFO("  MISI: CUSTOM WAYPOINTS (%zu titik)", wps.size());
        ROS_INFO("==================================================");

        for (size_t i = 0; i < wps.size(); ++i)
        {
            double wx = std::get<0>(wps[i]);
            double wy = std::get<1>(wps[i]);
            double wz = std::get<2>(wps[i]);
            std::string name = "WP" + std::to_string(i + 1);

            ROS_INFO("[%zu/%zu] -> %s (%.2f, %.2f, %.2f)",
                     i + 1, wps.size(), name.c_str(), wx, wy, wz);
            if (!goTo(wx, wy, wz, name))
            {
                ROS_WARN("Misi dihentikan di %s", name.c_str());
                return false;
            }
        }
        ROS_INFO("Custom waypoints selesai!");
        return true;
    }

    // ======================== MAIN RUN ========================

    bool preflight()
    {
        // 1. Koneksi FCU
        waitForConnection();

        // 2. Request data stream
        requestDataStream();

        // 3. Tunggu pose
        waitForPose();

        // 4. Print status awal
        printStatus();

        // 5. Set GUIDED mode
        if (!setMode("GUIDED"))
        {
            ROS_ERROR("Gagal set GUIDED mode!");
            return false;
        }

        // 6. Arm
        if (!arm())
        {
            ROS_ERROR("Gagal arm drone!");
            return false;
        }

        // 7. Takeoff
        if (!takeoff())
        {
            ROS_ERROR("Takeoff gagal!");
            return false;
        }

        // Pastikan masih GUIDED setelah takeoff
        if (!isMode("GUIDED"))
        {
            ROS_WARN("Mode berubah setelah takeoff, re-set GUIDED ...");
            setMode("GUIDED");
        }

        mission_active_ = true;
        return true;
    }

    void run()
    {
        ROS_INFO("=======================================================");
        ROS_INFO("   MAVROS DRONE CONTROL (C++)");
        ROS_INFO("   ArduPilot + MAVROS (GUIDED mode)");
        ROS_INFO("   Misi: %s", mission_type_.c_str());
        ROS_INFO("=======================================================");

        // Pre-flight
        if (!preflight())
        {
            ROS_ERROR("Pre-flight gagal! Membatalkan misi.");
            return;
        }

        // Jalankan misi berdasarkan parameter
        bool mission_ok = false;
        if (mission_type_ == "hover")
        {
            mission_ok = missionHover();
        }
        else if (mission_type_ == "square")
        {
            double size;
            pnh_.param<double>("square_size", size, 2.0);
            mission_ok = missionSquare(size);
        }
        else if (mission_type_ == "circle")
        {
            double radius;
            int points;
            pnh_.param<double>("circle_radius", radius, 2.0);
            pnh_.param<int>("circle_points", points, 12);
            mission_ok = missionCircle(radius, points);
        }
        else
        {
            ROS_WARN("Tipe misi '%s' tidak dikenal. Hover saja.", mission_type_.c_str());
            mission_ok = missionHover();
        }

        // Landing
        if (mission_active_)
        {
            land();
        }

        // Status akhir
        if (mission_ok)
        {
            ROS_INFO("=======================================================");
            ROS_INFO("   MISI SELESAI DENGAN SUKSES!");
            ROS_INFO("=======================================================");
        }
        else
        {
            ROS_WARN("=======================================================");
            ROS_WARN("   MISI DIHENTIKAN / GAGAL");
            ROS_WARN("=======================================================");
        }
    }

private:
    // ROS
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Rate* rate_;

    // Subscribers
    ros::Subscriber state_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber vel_sub_;
    ros::Subscriber gps_sub_;
    ros::Subscriber battery_sub_;

    // Publishers
    ros::Publisher setpoint_pub_;
    ros::Publisher vel_pub_;

    // Service Clients
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient takeoff_client_;
    ros::ServiceClient land_client_;
    ros::ServiceClient stream_rate_client_;
    ros::ServiceClient command_client_;

    // State variables
    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::TwistStamped current_velocity_;
    sensor_msgs::NavSatFix global_position_;
    sensor_msgs::BatteryState battery_;
    bool pose_received_;
    bool mission_active_;

    // Parameters
    double altitude_;
    double threshold_;
    int rate_hz_;
    std::string mission_type_;
    double hover_duration_;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mavros_control_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    try
    {
        MAVROSControl ctrl(nh, pnh);
        ctrl.run();
    }
    catch (const ros::Exception& e)
    {
        ROS_ERROR("ROS Exception: %s", e.what());
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Error: %s", e.what());
    }

    return 0;
}
