/**
 * @file square_mission_node.cpp
 * @brief Square Mission - ROS 2 Version (Foxy & Humble)
 *
 * Misi navigasi otonom membentuk pola persegi sempurna (2x2 meter)
 * pada ketinggian tetap menggunakan rclcpp dan MAVROS2.
 *
 * Logic Requirement: "Distance-Based Waypoint"
 * - TIDAK menggunakan sleep()
 * - Menghitung jarak Euclidean antara posisi drone saat ini dengan target
 * - Threshold: jarak < 0.3 meter
 * - Frekuensi setpoint: 10Hz - 20Hz
 * - Safety: berhenti jika mode berubah manual
 *
 * Kompatibel dengan ROS 2 Foxy & Humble + MAVROS2 + ArduPilot SITL
 */

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <string>
#include <vector>
#include <utility>
#include <chrono>
#include <functional>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/stream_rate.hpp>

using namespace std::chrono_literals;


class SquareMission : public rclcpp::Node
{
public:
    SquareMission()
        : Node("square_mission_node"), pose_received_(false), mission_active_(true)
    {
        // ---------- Parameters ----------
        this->declare_parameter<double>("square_size", 2.0);
        this->declare_parameter<double>("altitude", 2.0);
        this->declare_parameter<double>("threshold", 0.3);
        this->declare_parameter<int>("rate", 20);

        square_size_ = this->get_parameter("square_size").as_double();
        altitude_    = this->get_parameter("altitude").as_double();
        threshold_   = this->get_parameter("threshold").as_double();
        rate_hz_     = this->get_parameter("rate").as_int();

        // ---------- Subscribers ----------
        auto qos_reliable = rclcpp::QoS(10).reliable();
        auto qos_sensor   = rclcpp::SensorDataQoS();

        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", qos_reliable,
            std::bind(&SquareMission::stateCb, this, std::placeholders::_1));
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", qos_sensor,
            std::bind(&SquareMission::poseCb, this, std::placeholders::_1));

        // ---------- Publisher ----------
        setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mavros/setpoint_position/local", 10);

        // ---------- Service Clients ----------
        arming_client_      = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        set_mode_client_    = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
        takeoff_client_     = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
        land_client_        = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");
        stream_rate_client_ = this->create_client<mavros_msgs::srv::StreamRate>("/mavros/set_stream_rate");

        RCLCPP_INFO(this->get_logger(), "Square Mission Node telah siap.");
        RCLCPP_INFO(this->get_logger(),
            "Ukuran persegi: %.1f m | Ketinggian: %.1f m | Threshold: %.2f m",
            square_size_, altitude_, threshold_);
    }

    // ======================= CALLBACKS =======================

    void stateCb(const mavros_msgs::msg::State::SharedPtr msg) { current_state_ = *msg; }
    void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_ = *msg;
        pose_received_ = true;
    }

    // ======================= HELPERS =======================

    void spinOnce() { rclcpp::spin_some(this->get_node_base_interface()); }

    template <typename ServiceT>
    bool waitForService(typename rclcpp::Client<ServiceT>::SharedPtr client,
                        const std::string& name, int timeout_sec = 5)
    {
        RCLCPP_INFO(this->get_logger(), "Menunggu service %s ...", name.c_str());
        while (!client->wait_for_service(std::chrono::seconds(timeout_sec)))
        {
            if (!rclcpp::ok()) return false;
            RCLCPP_WARN(this->get_logger(), "Service %s belum tersedia ...", name.c_str());
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
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        pose.pose.orientation.w = 1.0;
        return pose;
    }

    bool isModeGuided() const { return current_state_.mode == "GUIDED"; }
    bool isArmed() const { return current_state_.armed; }

    bool safetyCheck()
    {
        if (!isModeGuided() && mission_active_)
        {
            RCLCPP_WARN(this->get_logger(),
                "Mode berubah ke %s! Menghentikan misi.", current_state_.mode.c_str());
            mission_active_ = false;
            return false;
        }
        return true;
    }

    // ======================= MISSION METHODS =======================

    void waitForConnection()
    {
        RCLCPP_INFO(this->get_logger(), "Menunggu koneksi FCU ...");
        rclcpp::Rate rate(rate_hz_);
        while (rclcpp::ok() && !current_state_.connected) { spinOnce(); rate.sleep(); }
        RCLCPP_INFO(this->get_logger(), "FCU terhubung!");
    }

    void requestDataStream()
    {
        RCLCPP_INFO(this->get_logger(), "Requesting data stream dari FCU ...");
        if (!waitForService<mavros_msgs::srv::StreamRate>(stream_rate_client_, "/mavros/set_stream_rate"))
            return;
        auto req = std::make_shared<mavros_msgs::srv::StreamRate::Request>();
        req->stream_id = 0;
        req->message_rate = 10;
        req->on_off = true;
        auto resp = callServiceSync<mavros_msgs::srv::StreamRate>(stream_rate_client_, req);
        if (resp)
            RCLCPP_INFO(this->get_logger(), "Data stream berhasil di-request!");
        else
            RCLCPP_WARN(this->get_logger(), "Gagal request stream!");
    }

    void waitForPose()
    {
        RCLCPP_INFO(this->get_logger(), "Menunggu data pose ...");
        rclcpp::Rate rate(rate_hz_);
        while (rclcpp::ok() && !pose_received_) { spinOnce(); rate.sleep(); }
        RCLCPP_INFO(this->get_logger(), "Data pose diterima! Posisi: (%.2f, %.2f, %.2f)",
            current_pose_.pose.position.x, current_pose_.pose.position.y,
            current_pose_.pose.position.z);
    }

    bool setGuidedMode()
    {
        if (!waitForService<mavros_msgs::srv::SetMode>(set_mode_client_, "/mavros/set_mode"))
            return false;

        RCLCPP_INFO(this->get_logger(), "Mencoba set mode GUIDED ...");
        auto last_req = this->now();
        rclcpp::Rate rate(rate_hz_);

        while (rclcpp::ok())
        {
            spinOnce();
            if (current_state_.mode != "GUIDED")
            {
                if ((this->now() - last_req).seconds() > 2.0)
                {
                    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                    req->custom_mode = "GUIDED";
                    auto resp = callServiceSync<mavros_msgs::srv::SetMode>(set_mode_client_, req);
                    if (resp && resp->mode_sent)
                        RCLCPP_INFO(this->get_logger(), "Request GUIDED terkirim ...");
                    else
                        RCLCPP_WARN(this->get_logger(), "Service call gagal.");
                    last_req = this->now();
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Mode GUIDED aktif!");
                return true;
            }
            rate.sleep();
        }
        return false;
    }

    bool armDrone()
    {
        if (!waitForService<mavros_msgs::srv::CommandBool>(arming_client_, "/mavros/cmd/arming"))
            return false;

        RCLCPP_INFO(this->get_logger(), "Mencoba ARM drone ...");
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
                        RCLCPP_INFO(this->get_logger(), "Request ARM terkirim ...");
                    else
                        RCLCPP_WARN(this->get_logger(), "Service call gagal.");
                    last_req = this->now();
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Drone berhasil di-ARM!");
                return true;
            }
            rate.sleep();
        }
        return false;
    }

    bool takeoff()
    {
        if (!waitForService<mavros_msgs::srv::CommandTOL>(takeoff_client_, "/mavros/cmd/takeoff"))
            return false;

        RCLCPP_INFO(this->get_logger(), "Takeoff ke ketinggian %.1f m ...", altitude_);
        auto last_req = this->now();
        rclcpp::Rate rate(rate_hz_);

        while (rclcpp::ok())
        {
            spinOnce();
            if ((this->now() - last_req).seconds() > 2.0)
            {
                auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
                req->altitude = altitude_;
                auto resp = callServiceSync<mavros_msgs::srv::CommandTOL>(takeoff_client_, req);
                if (resp && resp->success)
                {
                    RCLCPP_INFO(this->get_logger(), "Takeoff command diterima!");
                    break;
                }
                RCLCPP_WARN(this->get_logger(), "Takeoff gagal, coba lagi ...");
                last_req = this->now();
            }
            rate.sleep();
        }

        RCLCPP_INFO(this->get_logger(), "Menunggu drone mencapai ketinggian %.1f m ...", altitude_);
        while (rclcpp::ok())
        {
            spinOnce();
            double current_alt = current_pose_.pose.position.z;
            if (current_alt >= altitude_ * 0.90)
            {
                RCLCPP_INFO(this->get_logger(), "Ketinggian tercapai! (%.2f m)", current_alt);
                return true;
            }
            rate.sleep();
        }
        return false;
    }

    bool navigateToWaypoint(geometry_msgs::msg::PoseStamped& target, const std::string& wp_name)
    {
        RCLCPP_INFO(this->get_logger(), "Navigasi ke waypoint %s: (%.2f, %.2f, %.2f)",
            wp_name.c_str(), target.pose.position.x,
            target.pose.position.y, target.pose.position.z);

        rclcpp::Rate rate(rate_hz_);
        while (rclcpp::ok())
        {
            spinOnce();
            if (!safetyCheck()) return false;

            target.header.stamp = this->now();
            setpoint_pub_->publish(target);

            double distance = euclideanDistance(target);
            if (distance < threshold_)
            {
                RCLCPP_INFO(this->get_logger(), "Waypoint %s tercapai! (jarak: %.3f m)",
                    wp_name.c_str(), distance);
                return true;
            }
            rate.sleep();
        }
        return false;
    }

    void landDrone()
    {
        RCLCPP_INFO(this->get_logger(), "Misi selesai! Memulai pendaratan ...");
        if (!waitForService<mavros_msgs::srv::SetMode>(set_mode_client_, "/mavros/set_mode"))
            return;

        rclcpp::Rate rate(rate_hz_);
        while (rclcpp::ok())
        {
            spinOnce();
            if (current_state_.mode != "LAND")
            {
                auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                req->custom_mode = "LAND";
                auto resp = callServiceSync<mavros_msgs::srv::SetMode>(set_mode_client_, req);
                if (resp && resp->mode_sent)
                {
                    RCLCPP_INFO(this->get_logger(), "Mode LAND berhasil diaktifkan!");
                    break;
                }
                RCLCPP_WARN(this->get_logger(), "Set mode LAND gagal.");
            }
            else break;
            rate.sleep();
        }

        RCLCPP_INFO(this->get_logger(), "Menunggu drone mendarat ...");
        while (rclcpp::ok() && current_state_.armed) { spinOnce(); rate.sleep(); }
        RCLCPP_INFO(this->get_logger(), "Drone telah mendarat dengan selamat!");
    }

    // ======================= MAIN MISSION =======================

    std::vector<std::pair<geometry_msgs::msg::PoseStamped, std::string>> generateSquareWaypoints()
    {
        double s = square_size_;
        double z = altitude_;
        double x0 = current_pose_.pose.position.x;
        double y0 = current_pose_.pose.position.y;

        RCLCPP_INFO(this->get_logger(), "Posisi awal referensi: (%.2f, %.2f)", x0, y0);

        std::vector<std::pair<geometry_msgs::msg::PoseStamped, std::string>> waypoints;
        waypoints.push_back({createPose(x0 + s, y0,     z), "WP1"});
        waypoints.push_back({createPose(x0 + s, y0 + s, z), "WP2"});
        waypoints.push_back({createPose(x0,     y0 + s, z), "WP3"});
        waypoints.push_back({createPose(x0,     y0,     z), "WP4"});
        return waypoints;
    }

    void run()
    {
        RCLCPP_INFO(this->get_logger(), "==================================================");
        RCLCPP_INFO(this->get_logger(), "  SQUARE MISSION - 2x2m Autonomous Navigation");
        RCLCPP_INFO(this->get_logger(), "  ArduPilot + MAVROS (GUIDED mode) [ROS 2 C++]");
        RCLCPP_INFO(this->get_logger(), "==================================================");

        waitForConnection();
        requestDataStream();
        waitForPose();

        if (!setGuidedMode()) { RCLCPP_ERROR(this->get_logger(), "Gagal set GUIDED!"); return; }
        if (!armDrone()) { RCLCPP_ERROR(this->get_logger(), "Gagal arm drone!"); return; }
        if (!takeoff()) { RCLCPP_ERROR(this->get_logger(), "Takeoff gagal!"); return; }

        if (!isModeGuided())
        {
            RCLCPP_WARN(this->get_logger(), "Mode berubah setelah takeoff: %s", current_state_.mode.c_str());
            setGuidedMode();
        }

        mission_active_ = true;
        auto waypoints = generateSquareWaypoints();

        RCLCPP_INFO(this->get_logger(), "----------------------------------------");
        RCLCPP_INFO(this->get_logger(), "Memulai navigasi pola persegi %.0fx%.0f meter ...",
            square_size_, square_size_);
        RCLCPP_INFO(this->get_logger(), "Jumlah waypoint: %zu", waypoints.size());
        RCLCPP_INFO(this->get_logger(), "----------------------------------------");

        for (size_t i = 0; i < waypoints.size(); ++i)
        {
            auto& wp   = waypoints[i].first;
            auto& name = waypoints[i].second;

            RCLCPP_INFO(this->get_logger(), "[%zu/%zu] Target: %s (%.2f, %.2f, %.2f)",
                i + 1, waypoints.size(), name.c_str(),
                wp.pose.position.x, wp.pose.position.y, wp.pose.position.z);

            bool success = navigateToWaypoint(wp, name);
            if (!success)
            {
                if (!mission_active_)
                    RCLCPP_WARN(this->get_logger(), "Misi dihentikan karena mode berubah secara manual.");
                else
                    RCLCPP_WARN(this->get_logger(), "Misi dihentikan (node shutdown).");
                return;
            }
        }

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Semua waypoint telah tercapai!");
        RCLCPP_INFO(this->get_logger(), "========================================");
        landDrone();

        RCLCPP_INFO(this->get_logger(), "==================================================");
        RCLCPP_INFO(this->get_logger(), "  MISI SELESAI DENGAN SUKSES!");
        RCLCPP_INFO(this->get_logger(), "==================================================");
    }

private:
    // Subscribers
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;

    // Service Clients
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
    rclcpp::Client<mavros_msgs::srv::StreamRate>::SharedPtr stream_rate_client_;

    // State
    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseStamped current_pose_;
    bool pose_received_;
    bool mission_active_;

    // Parameters
    double square_size_;
    double altitude_;
    double threshold_;
    int rate_hz_;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SquareMission>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
