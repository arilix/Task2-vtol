/**
 * @file mavros_control_node.cpp
 * @brief MAVROS Drone Control - ROS 2 Version (Foxy & Humble)
 *
 * Menggunakan rclcpp dan MAVROS2 untuk kontrol drone secara otonom
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
 * Kompatibel dengan ROS 2 Foxy & Humble + MAVROS2 + ArduPilot SITL
 */

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <string>
#include <vector>
#include <tuple>
#include <chrono>
#include <functional>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/stream_rate.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

using namespace std::chrono_literals;


class MAVROSControl : public rclcpp::Node
{
public:
    MAVROSControl()
        : Node("mavros_control_node"), pose_received_(false), mission_active_(true)
    {
        // ===================== PARAMETERS =====================
        this->declare_parameter<double>("altitude", 2.0);
        this->declare_parameter<double>("threshold", 0.3);
        this->declare_parameter<int>("rate", 20);
        this->declare_parameter<std::string>("mission_type", "hover");
        this->declare_parameter<double>("hover_duration", 10.0);
        this->declare_parameter<double>("square_size", 2.0);
        this->declare_parameter<double>("circle_radius", 2.0);
        this->declare_parameter<int>("circle_points", 12);

        altitude_       = this->get_parameter("altitude").as_double();
        threshold_      = this->get_parameter("threshold").as_double();
        rate_hz_        = this->get_parameter("rate").as_int();
        mission_type_   = this->get_parameter("mission_type").as_string();
        hover_duration_ = this->get_parameter("hover_duration").as_double();

        // ===================== SUBSCRIBERS =====================
        auto qos_reliable = rclcpp::QoS(10).reliable();
        auto qos_sensor   = rclcpp::SensorDataQoS();

        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", qos_reliable,
            std::bind(&MAVROSControl::stateCb, this, std::placeholders::_1));
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", qos_sensor,
            std::bind(&MAVROSControl::poseCb, this, std::placeholders::_1));
        vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/mavros/local_position/velocity_local", qos_sensor,
            std::bind(&MAVROSControl::velCb, this, std::placeholders::_1));
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/mavros/global_position/global", qos_sensor,
            std::bind(&MAVROSControl::gpsCb, this, std::placeholders::_1));
        battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "/mavros/battery", qos_sensor,
            std::bind(&MAVROSControl::batteryCb, this, std::placeholders::_1));

        // ===================== PUBLISHERS =====================
        setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mavros/setpoint_position/local", 10);
        vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/mavros/setpoint_velocity/cmd_vel", 10);

        // ===================== SERVICE CLIENTS =====================
        arming_client_      = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        set_mode_client_    = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
        takeoff_client_     = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
        land_client_        = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");
        stream_rate_client_ = this->create_client<mavros_msgs::srv::StreamRate>("/mavros/set_stream_rate");
        command_client_     = this->create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");

        RCLCPP_INFO(this->get_logger(), "[MAVROS] Control node siap.");
        RCLCPP_INFO(this->get_logger(),
            "[MAVROS] Altitude: %.1f m | Threshold: %.2f m | Rate: %d Hz",
            altitude_, threshold_, rate_hz_);
    }

    // ======================== CALLBACKS ========================

    void stateCb(const mavros_msgs::msg::State::SharedPtr msg) { current_state_ = *msg; }
    void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) { current_pose_ = *msg; pose_received_ = true; }
    void velCb(const geometry_msgs::msg::TwistStamped::SharedPtr msg) { current_velocity_ = *msg; }
    void gpsCb(const sensor_msgs::msg::NavSatFix::SharedPtr msg) { global_position_ = *msg; }
    void batteryCb(const sensor_msgs::msg::BatteryState::SharedPtr msg) { battery_ = *msg; }

    // ======================== UTILITY ========================

    void spinOnce()
    {
        rclcpp::spin_some(this->get_node_base_interface());
    }

    double euclideanDistance(const geometry_msgs::msg::PoseStamped& target) const
    {
        double dx = current_pose_.pose.position.x - target.pose.position.x;
        double dy = current_pose_.pose.position.y - target.pose.position.y;
        double dz = current_pose_.pose.position.z - target.pose.position.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    geometry_msgs::msg::PoseStamped createPose(double x, double y, double z) const
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = rclcpp::Clock().now();
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

    void printStatus()
    {
        double x, y, z;
        getCurrentPosition(x, y, z);
        RCLCPP_INFO(this->get_logger(), "--- Drone Status ---");
        RCLCPP_INFO(this->get_logger(), "  Mode     : %s", current_state_.mode.c_str());
        RCLCPP_INFO(this->get_logger(), "  Armed    : %s", current_state_.armed ? "True" : "False");
        RCLCPP_INFO(this->get_logger(), "  Connected: %s", current_state_.connected ? "True" : "False");
        RCLCPP_INFO(this->get_logger(), "  Lokal    : (%.2f, %.2f, %.2f)", x, y, z);
        RCLCPP_INFO(this->get_logger(), "  GPS      : (%.6f, %.6f, %.2f)",
            global_position_.latitude, global_position_.longitude, global_position_.altitude);
        RCLCPP_INFO(this->get_logger(), "  Baterai  : %.2fV (%.0f%%)",
            battery_.voltage, battery_.percentage > 0 ? battery_.percentage * 100.0 : 0.0);
        RCLCPP_INFO(this->get_logger(), "--------------------");
    }

    bool isMode(const std::string& mode) const { return current_state_.mode == mode; }
    bool isArmed() const { return current_state_.armed; }
    bool isConnected() const { return current_state_.connected; }

    bool safetyCheck()
    {
        if (!isMode("GUIDED") && mission_active_)
        {
            RCLCPP_WARN(this->get_logger(),
                "[SAFETY] Mode berubah ke %s! Menghentikan misi.", current_state_.mode.c_str());
            mission_active_ = false;
            return false;
        }
        return true;
    }

    // ======================== SERVICE HELPERS ========================

    template <typename ServiceT>
    bool waitForService(typename rclcpp::Client<ServiceT>::SharedPtr client,
                        const std::string& name, int timeout_sec = 5)
    {
        RCLCPP_INFO(this->get_logger(), "[MAVROS] Menunggu service %s ...", name.c_str());
        while (!client->wait_for_service(std::chrono::seconds(timeout_sec)))
        {
            if (!rclcpp::ok())
                return false;
            RCLCPP_WARN(this->get_logger(), "[MAVROS] Service %s belum tersedia ...", name.c_str());
        }
        return true;
    }

    template <typename ServiceT>
    typename ServiceT::Response::SharedPtr callServiceSync(
        typename rclcpp::Client<ServiceT>::SharedPtr client,
        typename ServiceT::Request::SharedPtr request)
    {
        auto future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(
                this->get_node_base_interface(), future, 5s) == rclcpp::FutureReturnCode::SUCCESS)
        {
            return future.get();
        }
        return nullptr;
    }

    // ======================== CORE FUNCTIONS ========================

    void waitForConnection()
    {
        RCLCPP_INFO(this->get_logger(), "[MAVROS] Menunggu koneksi FCU ...");
        rclcpp::Rate rate(rate_hz_);
        while (rclcpp::ok() && !current_state_.connected)
        {
            spinOnce();
            rate.sleep();
        }
        RCLCPP_INFO(this->get_logger(), "[MAVROS] FCU terhubung!");
    }

    void requestDataStream(int stream_rate = 10)
    {
        RCLCPP_INFO(this->get_logger(), "[MAVROS] Requesting data stream %d Hz ...", stream_rate);
        if (!waitForService<mavros_msgs::srv::StreamRate>(
                stream_rate_client_, "/mavros/set_stream_rate"))
            return;

        auto req = std::make_shared<mavros_msgs::srv::StreamRate::Request>();
        req->stream_id = 0;
        req->message_rate = stream_rate;
        req->on_off = true;
        auto resp = callServiceSync<mavros_msgs::srv::StreamRate>(stream_rate_client_, req);
        if (resp)
            RCLCPP_INFO(this->get_logger(), "[MAVROS] Data stream OK!");
        else
            RCLCPP_WARN(this->get_logger(), "[MAVROS] Data stream gagal!");
    }

    void waitForPose()
    {
        RCLCPP_INFO(this->get_logger(), "[MAVROS] Menunggu data pose ...");
        rclcpp::Rate rate(rate_hz_);
        while (rclcpp::ok() && !pose_received_)
        {
            spinOnce();
            rate.sleep();
        }
        double x, y, z;
        getCurrentPosition(x, y, z);
        RCLCPP_INFO(this->get_logger(), "[MAVROS] Pose diterima: (%.2f, %.2f, %.2f)", x, y, z);
    }

    bool setMode(const std::string& mode_name)
    {
        if (!waitForService<mavros_msgs::srv::SetMode>(set_mode_client_, "/mavros/set_mode"))
            return false;

        RCLCPP_INFO(this->get_logger(), "[MAVROS] Set mode -> %s ...", mode_name.c_str());
        auto last_req = this->now();
        rclcpp::Rate rate(rate_hz_);

        while (rclcpp::ok())
        {
            spinOnce();
            if (current_state_.mode != mode_name)
            {
                if ((this->now() - last_req).seconds() > 2.0)
                {
                    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                    req->custom_mode = mode_name;
                    auto resp = callServiceSync<mavros_msgs::srv::SetMode>(set_mode_client_, req);
                    if (resp && resp->mode_sent)
                        RCLCPP_INFO(this->get_logger(), "[MAVROS] Request %s terkirim.", mode_name.c_str());
                    else
                        RCLCPP_WARN(this->get_logger(), "[MAVROS] Set mode gagal.");
                    last_req = this->now();
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "[MAVROS] Mode %s aktif!", mode_name.c_str());
                return true;
            }
            rate.sleep();
        }
        return false;
    }

    bool arm()
    {
        if (!waitForService<mavros_msgs::srv::CommandBool>(arming_client_, "/mavros/cmd/arming"))
            return false;

        RCLCPP_INFO(this->get_logger(), "[MAVROS] ARM drone ...");
        auto last_req = this->now();
        rclcpp::Rate rate(rate_hz_);

        while (rclcpp::ok())
        {
            spinOnce();
            if (!current_state_.armed)
            {
                if ((this->now() - last_req).seconds() > 2.0)
                {
                    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
                    req->value = true;
                    auto resp = callServiceSync<mavros_msgs::srv::CommandBool>(arming_client_, req);
                    if (resp && resp->success)
                        RCLCPP_INFO(this->get_logger(), "[MAVROS] ARM request terkirim.");
                    else
                        RCLCPP_WARN(this->get_logger(), "[MAVROS] ARM gagal.");
                    last_req = this->now();
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "[MAVROS] Drone ARMED!");
                return true;
            }
            rate.sleep();
        }
        return false;
    }

    bool disarm()
    {
        RCLCPP_INFO(this->get_logger(), "[MAVROS] DISARM drone ...");
        auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        req->value = false;
        auto resp = callServiceSync<mavros_msgs::srv::CommandBool>(arming_client_, req);
        if (resp && resp->success)
        {
            RCLCPP_INFO(this->get_logger(), "[MAVROS] Drone DISARMED!");
            return true;
        }
        RCLCPP_WARN(this->get_logger(), "[MAVROS] DISARM gagal.");
        return false;
    }

    bool takeoff(double alt = -1.0)
    {
        if (alt < 0) alt = altitude_;
        if (!waitForService<mavros_msgs::srv::CommandTOL>(takeoff_client_, "/mavros/cmd/takeoff"))
            return false;

        RCLCPP_INFO(this->get_logger(), "[MAVROS] Takeoff -> %.1f m ...", alt);
        auto last_req = this->now();
        rclcpp::Rate rate(rate_hz_);

        while (rclcpp::ok())
        {
            spinOnce();
            if ((this->now() - last_req).seconds() > 2.0)
            {
                auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
                req->altitude = alt;
                auto resp = callServiceSync<mavros_msgs::srv::CommandTOL>(takeoff_client_, req);
                if (resp && resp->success)
                {
                    RCLCPP_INFO(this->get_logger(), "[MAVROS] Takeoff command diterima!");
                    break;
                }
                RCLCPP_WARN(this->get_logger(), "[MAVROS] Takeoff gagal, coba lagi ...");
                last_req = this->now();
            }
            rate.sleep();
        }

        RCLCPP_INFO(this->get_logger(), "[MAVROS] Menunggu ketinggian %.1f m ...", alt);
        while (rclcpp::ok())
        {
            spinOnce();
            double current_alt = current_pose_.pose.position.z;
            if (current_alt >= alt * 0.90)
            {
                RCLCPP_INFO(this->get_logger(), "[MAVROS] Ketinggian tercapai! (%.2f m)", current_alt);
                return true;
            }
            rate.sleep();
        }
        return false;
    }

    bool land()
    {
        RCLCPP_INFO(this->get_logger(), "[MAVROS] Landing ...");
        setMode("LAND");

        RCLCPP_INFO(this->get_logger(), "[MAVROS] Menunggu drone mendarat ...");
        rclcpp::Rate rate(rate_hz_);
        while (rclcpp::ok() && current_state_.armed)
        {
            spinOnce();
            rate.sleep();
        }
        RCLCPP_INFO(this->get_logger(), "[MAVROS] Drone mendarat dengan selamat!");
        return true;
    }

    bool rtl()  { return setMode("RTL"); }
    bool loiter() { return setMode("LOITER"); }

    // ======================== NAVIGATION ========================

    bool goTo(double x, double y, double z, const std::string& label = "")
    {
        auto target = createPose(x, y, z);
        std::string tag = label.empty() ?
            "(" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ")" : label;
        RCLCPP_INFO(this->get_logger(), "[NAV] Menuju %s ...", tag.c_str());

        rclcpp::Rate rate(rate_hz_);
        while (rclcpp::ok())
        {
            spinOnce();
            if (!safetyCheck()) return false;

            target.header.stamp = this->now();
            setpoint_pub_->publish(target);

            double dist = euclideanDistance(target);
            if (dist < threshold_)
            {
                RCLCPP_INFO(this->get_logger(), "[NAV] %s tercapai! (jarak: %.3f m)", tag.c_str(), dist);
                return true;
            }
            rate.sleep();
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
        auto target = createPose(x, y, z);
        RCLCPP_INFO(this->get_logger(),
            "[NAV] Hover di (%.2f, %.2f, %.2f) selama %.1f detik ...", x, y, z, duration);

        auto start = this->now();
        rclcpp::Rate rate(rate_hz_);
        while (rclcpp::ok())
        {
            spinOnce();
            if (!safetyCheck()) return false;

            double elapsed = (this->now() - start).seconds();
            if (elapsed >= duration)
            {
                RCLCPP_INFO(this->get_logger(), "[NAV] Hover selesai! (%.1f detik)", elapsed);
                return true;
            }
            target.header.stamp = this->now();
            setpoint_pub_->publish(target);
            rate.sleep();
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
        geometry_msgs::msg::TwistStamped vel;
        vel.header.frame_id = "map";
        vel.twist.linear.x = vx;
        vel.twist.linear.y = vy;
        vel.twist.linear.z = vz;

        RCLCPP_INFO(this->get_logger(),
            "[NAV] Velocity (%.2f, %.2f, %.2f) selama %.1f detik", vx, vy, vz, duration);

        auto start = this->now();
        rclcpp::Rate rate(rate_hz_);
        while (rclcpp::ok())
        {
            spinOnce();
            if (!safetyCheck()) return false;
            if ((this->now() - start).seconds() >= duration) return true;
            vel.header.stamp = this->now();
            vel_pub_->publish(vel);
            rate.sleep();
        }
        return false;
    }

    // ======================== MISSION PATTERNS ========================

    bool missionHover()
    {
        RCLCPP_INFO(this->get_logger(), "==================================================");
        RCLCPP_INFO(this->get_logger(), "  MISI: HOVER");
        RCLCPP_INFO(this->get_logger(), "==================================================");

        double x, y, z_unused;
        getCurrentPosition(x, y, z_unused);
        double z = altitude_;

        if (!goTo(x, y, z, "HOVER_POINT")) return false;
        if (!hoverAt(x, y, z, hover_duration_)) return false;
        return true;
    }

    bool missionSquare(double size = 2.0)
    {
        RCLCPP_INFO(this->get_logger(), "==================================================");
        RCLCPP_INFO(this->get_logger(), "  MISI: SQUARE (%.1f x %.1f m)", size, size);
        RCLCPP_INFO(this->get_logger(), "==================================================");

        double x0, y0, z_unused;
        getCurrentPosition(x0, y0, z_unused);
        double z = altitude_;

        struct WP { double x, y, z; std::string name; };
        std::vector<WP> waypoints = {
            {x0 + size, y0,        z, "WP1-Depan"},
            {x0 + size, y0 + size, z, "WP2-Kanan"},
            {x0,        y0 + size, z, "WP3-Belakang"},
            {x0,        y0,        z, "WP4-Home"},
        };

        for (size_t i = 0; i < waypoints.size(); ++i)
        {
            const auto& wp = waypoints[i];
            RCLCPP_INFO(this->get_logger(), "[%zu/%zu] -> %s (%.2f, %.2f, %.2f)",
                i + 1, waypoints.size(), wp.name.c_str(), wp.x, wp.y, wp.z);
            if (!goTo(wp.x, wp.y, wp.z, wp.name))
            {
                RCLCPP_WARN(this->get_logger(), "Misi square dihentikan di %s", wp.name.c_str());
                return false;
            }
        }
        RCLCPP_INFO(this->get_logger(), "Pola persegi selesai!");
        return true;
    }

    bool missionCircle(double radius = 2.0, int points = 12)
    {
        RCLCPP_INFO(this->get_logger(), "==================================================");
        RCLCPP_INFO(this->get_logger(), "  MISI: CIRCLE (r=%.1f m, %d titik)", radius, points);
        RCLCPP_INFO(this->get_logger(), "==================================================");

        double x0, y0, z_unused;
        getCurrentPosition(x0, y0, z_unused);
        double z = altitude_;

        for (int i = 0; i <= points; ++i)
        {
            double angle = 2.0 * M_PI * i / points;
            double wx = x0 + radius * std::cos(angle);
            double wy = y0 + radius * std::sin(angle);
            std::string name = "C" + std::to_string(i % points);

            RCLCPP_INFO(this->get_logger(), "[%d/%d] -> %s (%.2f, %.2f, %.2f)",
                i + 1, points + 1, name.c_str(), wx, wy, z);
            if (!goTo(wx, wy, z, name))
            {
                RCLCPP_WARN(this->get_logger(), "Misi circle dihentikan di %s", name.c_str());
                return false;
            }
        }
        RCLCPP_INFO(this->get_logger(), "Pola lingkaran selesai!");
        return true;
    }

    // ======================== MAIN RUN ========================

    bool preflight()
    {
        waitForConnection();
        requestDataStream();
        waitForPose();
        printStatus();

        if (!setMode("GUIDED")) { RCLCPP_ERROR(this->get_logger(), "Gagal set GUIDED!"); return false; }
        if (!arm()) { RCLCPP_ERROR(this->get_logger(), "Gagal arm!"); return false; }
        if (!takeoff()) { RCLCPP_ERROR(this->get_logger(), "Takeoff gagal!"); return false; }

        if (!isMode("GUIDED"))
        {
            RCLCPP_WARN(this->get_logger(), "Mode berubah setelah takeoff, re-set GUIDED ...");
            setMode("GUIDED");
        }
        mission_active_ = true;
        return true;
    }

    void run()
    {
        RCLCPP_INFO(this->get_logger(), "=======================================================");
        RCLCPP_INFO(this->get_logger(), "   MAVROS DRONE CONTROL (ROS 2 C++)");
        RCLCPP_INFO(this->get_logger(), "   ArduPilot + MAVROS (GUIDED mode)");
        RCLCPP_INFO(this->get_logger(), "   Misi: %s", mission_type_.c_str());
        RCLCPP_INFO(this->get_logger(), "=======================================================");

        if (!preflight())
        {
            RCLCPP_ERROR(this->get_logger(), "Pre-flight gagal! Membatalkan misi.");
            return;
        }

        bool mission_ok = false;
        if (mission_type_ == "hover")
        {
            mission_ok = missionHover();
        }
        else if (mission_type_ == "square")
        {
            double size = this->get_parameter("square_size").as_double();
            mission_ok = missionSquare(size);
        }
        else if (mission_type_ == "circle")
        {
            double radius = this->get_parameter("circle_radius").as_double();
            int points = this->get_parameter("circle_points").as_int();
            mission_ok = missionCircle(radius, points);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Tipe misi '%s' tidak dikenal. Hover saja.", mission_type_.c_str());
            mission_ok = missionHover();
        }

        if (mission_active_) land();

        if (mission_ok)
        {
            RCLCPP_INFO(this->get_logger(), "=======================================================");
            RCLCPP_INFO(this->get_logger(), "   MISI SELESAI DENGAN SUKSES!");
            RCLCPP_INFO(this->get_logger(), "=======================================================");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "=======================================================");
            RCLCPP_WARN(this->get_logger(), "   MISI DIHENTIKAN / GAGAL");
            RCLCPP_WARN(this->get_logger(), "=======================================================");
        }
    }

private:
    // Subscribers
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;

    // Service Clients
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
    rclcpp::Client<mavros_msgs::srv::StreamRate>::SharedPtr stream_rate_client_;
    rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_client_;

    // State
    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::TwistStamped current_velocity_;
    sensor_msgs::msg::NavSatFix global_position_;
    sensor_msgs::msg::BatteryState battery_;
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
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MAVROSControl>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
