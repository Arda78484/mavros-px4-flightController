// uav_control_node.cpp
// Offboard local position control for fixed-wing UAV using ROS 2 Humble + MAVROS v1.12.3
// Mission: Arm → takeoff to offset local pose → fly to target local pose → loiter indefinitely

#include <rclcpp/rclcpp.hpp>
#include <string>  
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


using namespace std::chrono_literals;

class UAVControlNode : public rclcpp::Node {
public:
  UAVControlNode()
  : Node("uav_control_node")
  {
    // Subscribers
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "mavros/state", rclcpp::QoS(10),
      std::bind(&UAVControlNode::state_cb, this, std::placeholders::_1));

    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "mavros/global_position/global",
      rclcpp::SensorDataQoS(),
      std::bind(&UAVControlNode::gps_cb, this, std::placeholders::_1)
    );
    local_pos_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "mavros/local_position/pose", rclcpp::SensorDataQoS(),
      std::bind(&UAVControlNode::local_pos_cb, this, std::placeholders::_1));
    local_sp_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "mavros/setpoint_position/local", 10);

    // Service clients
    arm_client_  = create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
    mode_client_ = create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
    takeoff_client_ = create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");

    // Wait for services
    RCLCPP_INFO(get_logger(), "Waiting for arm and mode services...");
    arm_client_->wait_for_service();
    mode_client_->wait_for_service();
    takeoff_client_->wait_for_service();
    RCLCPP_INFO(get_logger(), "FCU services ready.");

    // Create a main loop timer at 100 Hz
    timer_ = create_wall_timer(
      10ms,
      std::bind(&UAVControlNode::control_loop, this)
    );

    desired_climb_ = 50.0;

    Waypoint wp1{ 47.400382, 8.564178, desired_climb_, 0.0 };
    Waypoint wp2{ 47.381555, 8.562375, desired_climb_, M_PI/2 };
    waypoints_.push_back(wp1);
    waypoints_.push_back(wp2);
  }

private:

  enum class MissionPhase { INIT, TAKEOFF, OFFBOARD_INIT, WAYPOINTS, FINISH };

  void control_loop() {
    switch (phase_) {
      case MissionPhase::INIT:
        // 1) Wait for FCU
        if (!fcu_connected_) {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "INIT: waiting for FCU connection...");
          return;
        }
        // 2) Wait for GPS fix
        if (!gps_ready_) {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "INIT: waiting for GPS fix...");
          return;
        }
        // All good—move to TAKEOFF
        RCLCPP_INFO(get_logger(), "INIT complete → TAKEOFF");
        phase_ = MissionPhase::TAKEOFF;
        break;

      case MissionPhase::TAKEOFF: {
        if (!takeoff_sent_) {
          // 1) Arm
          auto arm_req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
          arm_req->value = true;
          arm_client_->async_send_request(arm_req);
          RCLCPP_INFO(get_logger(), "Arming vehicle…");

          // 2) Switch to AUTO.TAKEOFF
          set_mode("AUTO.TAKEOFF");

          // 3) Send the takeoff command with GPS lat/lon + altitude
          auto tol_req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
          tol_req->latitude  = current_lat_;                   // take off from current
          tol_req->longitude = current_lon_;
          tol_req->altitude  = current_alt_ + desired_climb_;  // AMSL
          tol_req->min_pitch = 0.0;
          tol_req->yaw       = 0.0;
          takeoff_client_->async_send_request(tol_req);
          RCLCPP_INFO(get_logger(),
                      "Takeoff requested → lat: %.6f, lon: %.6f, alt: %.2f m AMSL",
                      tol_req->latitude,
                      tol_req->longitude,
                      tol_req->altitude);
          
          target_altitude_ = current_alt_ + desired_climb_;

          takeoff_sent_ = true;
        }

        if (current_alt_ >= target_altitude_ - 0.5) {
          RCLCPP_INFO(get_logger(),
            "Climb complete (%.2f m) → switching to OFFBOARD_INIT", current_alt_);
          phase_ = MissionPhase::OFFBOARD_INIT;
        }

        // Log current altitude and error
        double alt_error = target_altitude_ - current_alt_;
        RCLCPP_INFO(
          get_logger(),
          "Takeoff → cur_alt=%.2f m, target_alt=%.2f m, error=%.2f m",
          current_alt_, target_altitude_, alt_error
        );
        break;
      }

      case MissionPhase::OFFBOARD_INIT: {
        // 1) Flood the first waypoint @10 Hz so PX4 sees ≥2 Hz setpoints
        publishWaypoint(waypoints_[current_wp_idx_]);
        if (++offb_init_count_ > 20) {
          set_mode("OFFBOARD");
          phase_ = MissionPhase::WAYPOINTS;
        }

        const auto &wp = waypoints_[current_wp_idx_];
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "OFFBOARD_INIT → flooding WP %zu @ [%.2f, %.2f, %.2f]",
          current_wp_idx_+1, wp.x, wp.y, wp.z
        );
        break;
      }

      case MissionPhase::WAYPOINTS: {
        // 2) Continuously publish current waypoint
        publishWaypoint(waypoints_[current_wp_idx_]);

        // 3) Distance check
        if (distanceToWP(waypoints_[current_wp_idx_]) < 1.0) {
          if (++current_wp_idx_ >= waypoints_.size()) { // Note: changed to >=
            RCLCPP_INFO(get_logger(), "Final waypoint reached. Entering mission finish/loiter phase.");
            // CAPTURE THE LOITER POINT
            loiter_sp_.header.frame_id = "map";
            loiter_sp_.pose.position.x = current_x_;
            loiter_sp_.pose.position.y = current_y_;
            loiter_sp_.pose.position.z = current_z_;
            // You can also set a desired orientation if needed
            loiter_sp_.pose.orientation.w = 1.0;

            phase_ = MissionPhase::FINISH; // Switch to the next phase
          } else {
            RCLCPP_INFO(get_logger(), "Advancing to waypoint %zu", current_wp_idx_+1);
          }
        }

        const auto &wp = waypoints_[current_wp_idx_];
        double err = distanceToWP(wp);
        RCLCPP_INFO(
          get_logger(),
          "WP %zu → target [%.2f, %.2f, %.2f], pos [%.2f, %.2f, %.2f], err=%.2f m",
          current_wp_idx_+1,
          wp.x, wp.y, wp.z,
          current_x_, current_y_, current_z_,
          err
        );
        break;
      }

      case MissionPhase::FINISH: {
        RCLCPP_INFO(get_logger(), "Mission complete.");
        loiter_sp_.header.stamp = now();
        local_sp_pub_->publish(loiter_sp_);

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Mission complete. Loitering at [%.2f, %.2f, %.2f]",
            loiter_sp_.pose.position.x,
            loiter_sp_.pose.position.y,
            loiter_sp_.pose.position.z
        );
      }
    }
  }

  void state_cb(const mavros_msgs::msg::State::SharedPtr msg);
  void gps_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void local_pos_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);


  // Members
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  local_sp_pub_;

  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;

  rclcpp::TimerBase::SharedPtr timer_;

  mavros_msgs::msg::State current_state_;
  geometry_msgs::msg::PoseStamped takeoff_sp_, target_sp_, loiter_sp_;

  struct Waypoint { double x,y,z,yaw; };
  std::vector<Waypoint> waypoints_;
  size_t current_wp_idx_{0};
  int offb_init_count_{0};

  bool fcu_connected_{false};
  bool gps_ready_{false};
  bool offboard_started_{false};
  int init_count_{0};
  MissionPhase phase_{MissionPhase::INIT};
  bool takeoff_sent_{false};
  double current_lat_{0.0}, current_lon_{0.0}, current_alt_{0.0};
  double desired_climb_;
  double target_altitude_ = 0.0;

  double current_x_{0.0}, current_y_{0.0}, current_z_{0.0};
  bool local_pos_received_{false};

  void publishWaypoint(const Waypoint &wp) {
    geometry_msgs::msg::PoseStamped sp;
    sp.header.frame_id = "map";
    sp.header.stamp = now();
    sp.pose.position.x = wp.x;
    sp.pose.position.y = wp.y;
    sp.pose.position.z = wp.z;
    // convert wp.yaw → quaternion if you care about heading
    sp.pose.orientation.w = 1.0;
    local_sp_pub_->publish(sp);
  }

  double distanceToWP(const Waypoint &wp) {
    double dx = current_x_ - wp.x;
    double dy = current_y_ - wp.y;
    double dz = current_z_ - wp.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
  }

  void set_mode(const std::string &mode) {
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = mode;
    mode_client_->async_send_request(req);
    RCLCPP_INFO(get_logger(), "%s mode requested", mode.c_str());
  }
};

void UAVControlNode::state_cb(const mavros_msgs::msg::State::SharedPtr msg) {
  current_state_ = *msg;
  fcu_connected_ = msg->connected;
}

void UAVControlNode::gps_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  current_lat_ = msg->latitude;
  current_lon_ = msg->longitude;
  current_alt_ = msg->altitude;
  // STATUS_FIX = 0 (no fix), 1 = GPS fix, 2 = DGPS fix etc.
  gps_ready_ = (msg->status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX);
}

void UAVControlNode::local_pos_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_x_ = msg->pose.position.x;
  current_y_ = msg->pose.position.y;
  current_z_ = msg->pose.position.z;
}


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UAVControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
