// ROS 2 node wrapping dc::DriveController.
//
// Subscriptions
//   odom          nav_msgs/Odometry       pose of base_link (rear axle), twist in base_link
//   path          nav_msgs/Path           reference path; pose.position.z carries the speed reference (m/s)
//   steering      std_msgs/Float64        measured road-wheel angle (rad), optional
//   safety/estop  std_msgs/Bool           heartbeat from the safety supervisor (>= 50 Hz), data = e-stop
//   planner/cmd   std_msgs/Float64        target speed from the planner (>= 20 Hz); NaN = disable
// Publications
//   actuator_cmd  ackermann_msgs/AckermannDriveStamped  steering_angle + acceleration (= throttle - brake)
//   actuator_raw  std_msgs/Float64MultiArray            [steer, throttle, brake]
//   state         std_msgs/UInt8                        dc::DriveState
//
// ROS messages are converted into the same CAN frames the vehicle bus
// would carry, so heartbeat/E2E supervision runs through the identical code
// path as on the CAN interface.
#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "drivecontrol/controller.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"

using namespace std::chrono_literals;

class DriveControlNode : public rclcpp::Node {
 public:
  DriveControlNode() : Node("drivecontrol") {
    dc::DriveConfig cfg;
    cfg.use_can = true;
    cfg.lateral = declare_parameter("lateral", std::string("stanley")) == "pure_pursuit" ? dc::LateralMode::PurePursuit
                                                                                       : dc::LateralMode::Stanley;
    cfg.vehicle.wheelbase = declare_parameter("wheelbase", 2.7);
    cfg.limits.steer_max = declare_parameter("steer_max", 0.52);
    cfg.limits.steer_rate_max = declare_parameter("steer_rate_max", 0.6);
    cfg.stanley.k = declare_parameter("stanley_k", 1.5);
    cfg.pure_pursuit.k_v = declare_parameter("pp_k_v", 0.5);
    rate_hz_ = declare_parameter("rate_hz", 100.0);
    ctrl_ = std::make_unique<dc::DriveController>(cfg);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr m) {
      const auto& q = m->pose.pose.orientation;
      state_.x = m->pose.pose.position.x;
      state_.y = m->pose.pose.position.y;
      state_.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      state_.v = m->twist.twist.linear.x;
      state_.yaw_rate = m->twist.twist.angular.z;
      state_.valid = true;
      last_odom_ = now();
    });
    steer_sub_ = create_subscription<std_msgs::msg::Float64>(
        "steering", 10, [this](std_msgs::msg::Float64::SharedPtr m) { state_.steer = m->data; });
    path_sub_ = create_subscription<nav_msgs::msg::Path>("path", 1, [this](nav_msgs::msg::Path::SharedPtr m) {
      std::vector<double> x, y, v;
      for (const auto& p : m->poses) {
        x.push_back(p.pose.position.x);
        y.push_back(p.pose.position.y);
        v.push_back(p.pose.position.z);
      }
      if (x.size() >= 2) ctrl_->set_path(dc::Path(x, y, v));
    });
    estop_sub_ = create_subscription<std_msgs::msg::Bool>("safety/estop", 10, [this](std_msgs::msg::Bool::SharedPtr m) {
      ctrl_->on_can(dc::encode(dc::EStopMsg{m->data}, ctr_estop_++), seconds());
    });
    cmd_sub_ = create_subscription<std_msgs::msg::Float64>("planner/cmd", 10, [this](std_msgs::msg::Float64::SharedPtr m) {
      dc::CommandMsg c;
      c.enable = std::isfinite(m->data);
      c.target_speed = c.enable ? m->data : 0.0;
      ctrl_->on_can(dc::encode(c, ctr_cmd_++), seconds());
    });

    ack_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("actuator_cmd", 10);
    raw_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("actuator_raw", 10);
    state_pub_ = create_publisher<std_msgs::msg::UInt8>("state", 10);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz_), [this]() { tick(); });
  }

 private:
  double seconds() { return now().seconds(); }

  void tick() {
    const double t = seconds();
    // stale odometry is treated as an invalid state -> emergency stop
    state_.valid = (now() - last_odom_).seconds() < 0.1;
    const dc::ActuatorCommand c = ctrl_->step(state_, t, 1.0 / rate_hz_);
    dc::CanFrame f;
    while (ctrl_->pop_tx(f)) {
      // the CAN frames could be forwarded to a socketcan bridge here
    }
    ackermann_msgs::msg::AckermannDriveStamped a;
    a.header.stamp = now();
    a.drive.steering_angle = static_cast<float>(c.steer);
    a.drive.acceleration = static_cast<float>(c.throttle - c.brake);
    ack_pub_->publish(a);
    std_msgs::msg::Float64MultiArray raw;
    raw.data = {c.steer, c.throttle, c.brake};
    raw_pub_->publish(raw);
    std_msgs::msg::UInt8 s;
    s.data = static_cast<uint8_t>(ctrl_->state());
    state_pub_->publish(s);
  }

  std::unique_ptr<dc::DriveController> ctrl_;
  dc::VehicleState state_;
  rclcpp::Time last_odom_{0, 0, RCL_ROS_TIME};
  double rate_hz_ = 100.0;
  uint8_t ctr_estop_ = 0, ctr_cmd_ = 0;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steer_sub_, cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ack_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr raw_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriveControlNode>());
  rclcpp::shutdown();
  return 0;
}
