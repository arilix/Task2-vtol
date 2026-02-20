/**
 * @file square_mission_node.cpp
 * @brief Square Mission - Autonomous 2x2m Square Pattern Navigation (C++ Version)
 *
 * Menggunakan roscpp dan MAVROS untuk misi navigasi otonom membentuk
 * pola persegi sempurna (2x2 meter) pada ketinggian tetap.
 * Setelah kembali ke titik awal, ubah mode ke LAND secara otomatis.
 *
 * Logic Requirement: "Distance-Based Waypoint"
 * - TIDAK menggunakan sleep()
 * - Menghitung jarak Euclidean antara posisi drone saat ini dengan target
 * - Threshold: jarak < 0.3 meter
 * - Frekuensi setpoint: 10Hz - 20Hz
 * - Safety: berhenti jika mode berubah manual (misal STABILIZE)
 *
 * Digunakan dengan ArduPilot SITL + MAVROS (apm.launch)
 */

#include <ros/ros.h>
#include <cmath>
#include <string>
#include <vector>
#include <utility>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/StreamRate.h>


class SquareMission
{
public:
    SquareMission(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh), pose_received_(false), mission_active_(true)
    {
        // ---------- Parameters ----------
        pnh_.param<double>("square_size", square_size_, 2.0);
        pnh_.param<double>("altitude", altitude_, 2.0);
        pnh_.param<double>("threshold", threshold_, 0.3);
        pnh_.param<int>("rate", rate_hz_, 20);

        // ---------- Subscribers ----------
        state_sub_ = nh_.subscribe("/mavros/state", 10,
                                    &SquareMission::stateCb, this);
        pose_sub_  = nh_.subscribe("/mavros/local_position/pose", 10,
                                    &SquareMission::poseCb, this);

        // ---------- Publisher ----------
        setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            "/mavros/setpoint_position/local", 10);

        // ---------- Service Clients ----------
        ROS_INFO("Menunggu service /mavros/cmd/arming ...");
        ros::service::waitForService("/mavros/cmd/arming");
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

        ROS_INFO("Menunggu service /mavros/set_mode ...");
        ros::service::waitForService("/mavros/set_mode");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

        ROS_INFO("Menunggu service /mavros/cmd/takeoff ...");
        ros::service::waitForService("/mavros/cmd/takeoff");
        takeoff_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");

        ROS_INFO("Menunggu service /mavros/cmd/land ...");
        ros::service::waitForService("/mavros/cmd/land");
        land_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");

        ROS_INFO("Menunggu service /mavros/set_stream_rate ...");
        ros::service::waitForService("/mavros/set_stream_rate");
        stream_rate_client_ = nh_.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");

        // ---------- Rate ----------
        rate_ = new ros::Rate(rate_hz_);

        ROS_INFO("Square Mission Node telah siap.");
        ROS_INFO("Ukuran persegi: %.1f m | Ketinggian: %.1f m | Threshold: %.2f m",
                 square_size_, altitude_, threshold_);
    }

    ~SquareMission()
    {
        delete rate_;
    }

    // ======================= CALLBACKS =======================

    void stateCb(const mavros_msgs::State::ConstPtr& msg)
    {
        current_state_ = *msg;
    }

    void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        current_pose_ = *msg;
        pose_received_ = true;
    }

    // ======================= HELPER METHODS =======================

    double euclideanDistance(const geometry_msgs::PoseStamped& target) const
    {
        double dx = current_pose_.pose.position.x - target.pose.position.x;
        double dy = current_pose_.pose.position.y - target.pose.position.y;
        double dz = current_pose_.pose.position.z - target.pose.position.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    geometry_msgs::PoseStamped createPose(double x, double y, double z) const
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        pose.pose.orientation.w = 1.0;
        return pose;
    }

    bool isModeGuided() const
    {
        return current_state_.mode == "GUIDED";
    }

    bool isArmed() const
    {
        return current_state_.armed;
    }

    bool safetyCheck()
    {
        if (!isModeGuided() && mission_active_)
        {
            ROS_WARN("Mode berubah ke %s! Menghentikan misi.",
                     current_state_.mode.c_str());
            mission_active_ = false;
            return false;
        }
        return true;
    }

    // ======================= MISSION METHODS =======================

    void waitForConnection()
    {
        ROS_INFO("Menunggu koneksi FCU ...");
        while (ros::ok() && !current_state_.connected)
        {
            ros::spinOnce();
            rate_->sleep();
        }
        ROS_INFO("FCU terhubung!");
    }

    void requestDataStream()
    {
        ROS_INFO("Requesting data stream dari FCU ...");
        mavros_msgs::StreamRate srv;
        srv.request.stream_id = 0;
        srv.request.message_rate = 10;
        srv.request.on_off = true;
        if (stream_rate_client_.call(srv))
        {
            ROS_INFO("Data stream berhasil di-request!");
        }
        else
        {
            ROS_WARN("Gagal request stream!");
        }
    }

    void waitForPose()
    {
        ROS_INFO("Menunggu data pose ...");
        while (ros::ok() && !pose_received_)
        {
            ros::spinOnce();
            rate_->sleep();
        }
        ROS_INFO("Data pose diterima! Posisi: (%.2f, %.2f, %.2f)",
                 current_pose_.pose.position.x,
                 current_pose_.pose.position.y,
                 current_pose_.pose.position.z);
    }

    bool setGuidedMode()
    {
        mavros_msgs::SetMode mode_req;
        mode_req.request.custom_mode = "GUIDED";
        ros::Time last_request(0);

        ROS_INFO("Mencoba set mode GUIDED ...");
        while (ros::ok())
        {
            ros::spinOnce();
            if (current_state_.mode != "GUIDED")
            {
                if ((ros::Time::now() - last_request) > ros::Duration(2.0))
                {
                    if (set_mode_client_.call(mode_req) && mode_req.response.mode_sent)
                    {
                        ROS_INFO("Request GUIDED terkirim ...");
                    }
                    else
                    {
                        ROS_WARN("Service call gagal.");
                    }
                    last_request = ros::Time::now();
                }
            }
            else
            {
                ROS_INFO("Mode GUIDED aktif!");
                return true;
            }
            rate_->sleep();
        }
        return false;
    }

    bool armDrone()
    {
        mavros_msgs::CommandBool arm_req;
        arm_req.request.value = true;
        ros::Time last_request(0);

        ROS_INFO("Mencoba ARM drone ...");
        while (ros::ok())
        {
            ros::spinOnce();
            if (!current_state_.armed)
            {
                if ((ros::Time::now() - last_request) > ros::Duration(2.0))
                {
                    if (arming_client_.call(arm_req) && arm_req.response.success)
                    {
                        ROS_INFO("Request ARM terkirim ...");
                    }
                    else
                    {
                        ROS_WARN("Service call gagal.");
                    }
                    last_request = ros::Time::now();
                }
            }
            else
            {
                ROS_INFO("Drone berhasil di-ARM!");
                return true;
            }
            rate_->sleep();
        }
        return false;
    }

    bool takeoff()
    {
        ROS_INFO("Takeoff ke ketinggian %.1f m ...", altitude_);

        mavros_msgs::CommandTOL takeoff_req;
        takeoff_req.request.altitude = altitude_;

        ros::Time last_request(0);
        while (ros::ok())
        {
            ros::spinOnce();
            if ((ros::Time::now() - last_request) > ros::Duration(2.0))
            {
                if (takeoff_client_.call(takeoff_req) && takeoff_req.response.success)
                {
                    ROS_INFO("Takeoff command diterima!");
                    break;
                }
                else
                {
                    ROS_WARN("Takeoff gagal, coba lagi ...");
                }
                last_request = ros::Time::now();
            }
            rate_->sleep();
        }

        // Tunggu drone mencapai ketinggian target
        ROS_INFO("Menunggu drone mencapai ketinggian %.1f m ...", altitude_);
        while (ros::ok())
        {
            ros::spinOnce();
            double current_alt = current_pose_.pose.position.z;
            if (current_alt >= altitude_ * 0.90)
            {
                ROS_INFO("Ketinggian tercapai! (%.2f m)", current_alt);
                return true;
            }
            rate_->sleep();
        }
        return false;
    }

    bool navigateToWaypoint(geometry_msgs::PoseStamped& target, const std::string& wp_name)
    {
        ROS_INFO("Navigasi ke waypoint %s: (%.2f, %.2f, %.2f)",
                 wp_name.c_str(), target.pose.position.x,
                 target.pose.position.y, target.pose.position.z);

        while (ros::ok())
        {
            ros::spinOnce();

            // Safety check
            if (!safetyCheck()) return false;

            // Update timestamp dan publish setpoint
            target.header.stamp = ros::Time::now();
            setpoint_pub_.publish(target);

            // Hitung jarak Euclidean
            double distance = euclideanDistance(target);

            // Cek threshold
            if (distance < threshold_)
            {
                ROS_INFO("Waypoint %s tercapai! (jarak: %.3f m)", wp_name.c_str(), distance);
                return true;
            }
            rate_->sleep();
        }
        return false;
    }

    void landDrone()
    {
        ROS_INFO("Misi selesai! Memulai pendaratan ...");
        mavros_msgs::SetMode mode_req;
        mode_req.request.custom_mode = "LAND";

        while (ros::ok())
        {
            ros::spinOnce();
            if (current_state_.mode != "LAND")
            {
                if (set_mode_client_.call(mode_req) && mode_req.response.mode_sent)
                {
                    ROS_INFO("Mode LAND berhasil diaktifkan!");
                    break;
                }
                else
                {
                    ROS_WARN("Set mode LAND gagal.");
                }
            }
            else
            {
                break;
            }
            rate_->sleep();
        }

        // Tunggu drone mendarat (armed = false)
        ROS_INFO("Menunggu drone mendarat ...");
        while (ros::ok() && current_state_.armed)
        {
            ros::spinOnce();
            rate_->sleep();
        }
        ROS_INFO("Drone telah mendarat dengan selamat!");
    }

    // ======================= MAIN MISSION =======================

    /**
     * Generate waypoint untuk pola persegi.
     *
     * Pola persegi (dilihat dari atas):
     *     HOME (x0,y0)  --->  WP1 (x0+s, y0)
     *         ^                      |
     *         |                      v
     *     WP4 (x0, y0+s)  <---  WP2 (x0+s, y0+s)
     *
     * Lalu kembali ke HOME
     */
    std::vector<std::pair<geometry_msgs::PoseStamped, std::string>> generateSquareWaypoints()
    {
        double s = square_size_;
        double z = altitude_;
        double x0 = current_pose_.pose.position.x;
        double y0 = current_pose_.pose.position.y;

        ROS_INFO("Posisi awal referensi: (%.2f, %.2f)", x0, y0);

        std::vector<std::pair<geometry_msgs::PoseStamped, std::string>> waypoints;
        waypoints.push_back({createPose(x0 + s, y0,     z), "WP1"});  // maju
        waypoints.push_back({createPose(x0 + s, y0 + s, z), "WP2"});  // ke samping
        waypoints.push_back({createPose(x0,     y0 + s, z), "WP3"});  // mundur
        waypoints.push_back({createPose(x0,     y0,     z), "WP4"});  // kembali ke awal

        return waypoints;
    }

    void run()
    {
        ROS_INFO("==================================================");
        ROS_INFO("  SQUARE MISSION - 2x2m Autonomous Navigation");
        ROS_INFO("  ArduPilot + MAVROS (GUIDED mode) [C++]");
        ROS_INFO("==================================================");

        // 1. Tunggu koneksi FCU
        waitForConnection();

        // 2. Request data stream dari ArduPilot
        requestDataStream();

        // 3. Tunggu data pose
        waitForPose();

        // 4. Set GUIDED mode
        if (!setGuidedMode())
        {
            ROS_ERROR("Gagal set GUIDED mode!");
            return;
        }

        // 5. Arm drone
        if (!armDrone())
        {
            ROS_ERROR("Gagal arm drone!");
            return;
        }

        // 6. Takeoff
        if (!takeoff())
        {
            ROS_ERROR("Takeoff gagal!");
            return;
        }

        // Setelah takeoff, pastikan masih GUIDED
        if (!isModeGuided())
        {
            ROS_WARN("Mode berubah setelah takeoff: %s", current_state_.mode.c_str());
            setGuidedMode();
        }

        mission_active_ = true;

        // 7. Generate waypoints persegi
        auto waypoints = generateSquareWaypoints();

        ROS_INFO("----------------------------------------");
        ROS_INFO("Memulai navigasi pola persegi %.0fx%.0f meter ...",
                 square_size_, square_size_);
        ROS_INFO("Jumlah waypoint: %zu", waypoints.size());
        ROS_INFO("----------------------------------------");

        // 8. Navigasi ke setiap waypoint
        for (size_t i = 0; i < waypoints.size(); ++i)
        {
            auto& wp = waypoints[i].first;
            const auto& name = waypoints[i].second;

            ROS_INFO("[%zu/%zu] Target: %s (%.2f, %.2f, %.2f)",
                     i + 1, waypoints.size(), name.c_str(),
                     wp.pose.position.x, wp.pose.position.y,
                     wp.pose.position.z);

            bool success = navigateToWaypoint(wp, name);

            if (!success)
            {
                if (!mission_active_)
                    ROS_WARN("Misi dihentikan karena mode berubah secara manual.");
                else
                    ROS_WARN("Misi dihentikan (node shutdown).");
                return;
            }
        }

        // 9. Semua waypoint tercapai - LAND
        ROS_INFO("========================================");
        ROS_INFO("Semua waypoint telah tercapai!");
        ROS_INFO("========================================");
        landDrone();

        ROS_INFO("==================================================");
        ROS_INFO("  MISI SELESAI DENGAN SUKSES!");
        ROS_INFO("==================================================");
    }

private:
    // ROS
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Rate* rate_;

    // Subscribers
    ros::Subscriber state_sub_;
    ros::Subscriber pose_sub_;

    // Publisher
    ros::Publisher setpoint_pub_;

    // Service Clients
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient takeoff_client_;
    ros::ServiceClient land_client_;
    ros::ServiceClient stream_rate_client_;

    // State variables
    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped current_pose_;
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
    ros::init(argc, argv, "square_mission_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    try
    {
        SquareMission mission(nh, pnh);
        mission.run();
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
