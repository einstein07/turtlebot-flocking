#include "turtlebot_flocking/flocking.h"

#include <rclcpp/qos.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <limits>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace turtlebot_flocking
{

/****************************************/
/*    Parameter Loading Functions       */
/****************************************/

void WheelTurningParams::load_from_parameters(std::shared_ptr<rclcpp::Node> node) {
  auto rad_to_deg = [](double radians) {
    return radians * 180.0 / M_PI;
  };
  auto deg_to_rad = [](double degrees) {
    return degrees * M_PI / 180.0;
  };

  std::map<std::string, rclcpp::ParameterValue> defaults{
    {"hard_turn_on_angle_threshold", rclcpp::ParameterValue(rad_to_deg(hard_turn_on_angle_threshold))},
    {"soft_turn_on_angle_threshold", rclcpp::ParameterValue(rad_to_deg(soft_turn_on_angle_threshold))},
    {"no_turn_angle_threshold", rclcpp::ParameterValue(rad_to_deg(no_turn_angle_threshold))},
    {"max_speed", rclcpp::ParameterValue(max_speed)}
  };

  node->declare_parameters("wheel_turning", defaults);

  double hard_turn_deg = rad_to_deg(hard_turn_on_angle_threshold);
  double soft_turn_deg = rad_to_deg(soft_turn_on_angle_threshold);
  double no_turn_deg = rad_to_deg(no_turn_angle_threshold);
  double max_speed_param = max_speed;

  node->get_parameter("wheel_turning.hard_turn_on_angle_threshold", hard_turn_deg);
  node->get_parameter("wheel_turning.soft_turn_on_angle_threshold", soft_turn_deg);
  node->get_parameter("wheel_turning.no_turn_angle_threshold", no_turn_deg);
  node->get_parameter("wheel_turning.max_speed", max_speed_param);

  hard_turn_on_angle_threshold = deg_to_rad(hard_turn_deg);
  soft_turn_on_angle_threshold = deg_to_rad(soft_turn_deg);
  no_turn_angle_threshold = deg_to_rad(no_turn_deg);
  max_speed = max_speed_param;

  turning_mechanism = TurningMechanism::NO_TURN;
}

void FlockingInteractionParams::load_from_parameters(std::shared_ptr<rclcpp::Node> node) {
  const double default_ratio = interaction_cutoff / target_distance;

  std::map<std::string, rclcpp::ParameterValue> defaults{
    {"target_distance", rclcpp::ParameterValue(target_distance)},
    {"gain", rclcpp::ParameterValue(gain)},
    {"exponent", rclcpp::ParameterValue(exponent)},
    {"interaction_range", rclcpp::ParameterValue(interaction_cutoff)}
  };

  node->declare_parameters("flocking", defaults);

  node->get_parameter("flocking.target_distance", target_distance);
  node->get_parameter("flocking.gain", gain);
  node->get_parameter("flocking.exponent", exponent);

  if (node->has_parameter("flocking.interaction_range")) {
    node->get_parameter("flocking.interaction_range", interaction_cutoff);
  } else {
    interaction_cutoff = default_ratio * target_distance;
  }

  if (interaction_cutoff <= 0.0) {
    interaction_cutoff = default_ratio * target_distance;
  }
}

double FlockingInteractionParams::generalizedLennardJones(double distance) const {
  double normDistExp = std::pow(target_distance / distance, exponent);
  return -gain / distance * (normDistExp * normDistExp - normDistExp);
}

namespace {
double quaternionToYaw(const geometry_msgs::msg::Quaternion & q) {
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

Vector2 rotateWorldToBase(const Vector2 & world_vec, double yaw) {
  double cos_yaw = std::cos(yaw);
  double sin_yaw = std::sin(yaw);
  return {
    cos_yaw * world_vec.x + sin_yaw * world_vec.y,
   -sin_yaw * world_vec.x + cos_yaw * world_vec.y
  };
}
}  // namespace

/****************************************/
/*        FlockingController Class      */
/****************************************/

FlockingController::FlockingController(std::shared_ptr<rclcpp::Node> node)
: node_(std::move(node)),
  robot_ns_(node_->get_namespace()),
  initialized_(false),
  have_pose_(false),
  sync_with_clock_(false),
  control_period_(20ms),
  goal_position_(0.0,10.0),
  goal_gain_(1000.0),
  wheel_separation_(0.160),
  wheel_radius_(0.033),
  printed_(false),
  last_clock_time_(0, 0, RCL_ROS_TIME)
{
  if (robot_ns_ == "/") {
    robot_ns_.clear();
  }
  std::string robot_namespace_param = robot_ns_;
  node_->declare_parameter("robot_namespace", robot_namespace_param);
  node_->get_parameter("robot_namespace", robot_namespace_param);
  if (!robot_namespace_param.empty()) {
    robot_ns_ = robot_namespace_param;
  }
  if (!robot_ns_.empty() && robot_ns_.front() != '/') {
    robot_ns_.insert(robot_ns_.begin(), '/');
  }

  node_->declare_parameter("sync_with_argos_clock", sync_with_clock_);
  node_->get_parameter("sync_with_argos_clock", sync_with_clock_);
  int control_period_ms = static_cast<int>(control_period_.count());
  node_->declare_parameter("control_period_ms", control_period_ms);
  node_->get_parameter("control_period_ms", control_period_ms);
  control_period_ms = std::max(control_period_ms, 1);
  control_period_ = std::chrono::milliseconds(control_period_ms);

  std::string clock_topic = topic_with_namespace("/argos3_clock");
  node_->declare_parameter("clock_topic", clock_topic);
  node_->get_parameter("clock_topic", clock_topic);

  wheel_params_.load_from_parameters(node_);
  flocking_params_.load_from_parameters(node_);

  node_->declare_parameter("wheel_separation", wheel_separation_);
  node_->declare_parameter("wheel_radius", wheel_radius_);
  node_->get_parameter("wheel_separation", wheel_separation_);
  node_->get_parameter("wheel_radius", wheel_radius_);

  double goal_x = goal_position_.x;
  double goal_y = goal_position_.y;
  node_->declare_parameter("goal.x", goal_x);
  node_->declare_parameter("goal.y", goal_y);
  node_->declare_parameter("goal.gain", goal_gain_);
  node_->get_parameter("goal.x", goal_x);
  node_->get_parameter("goal.y", goal_y);
  node_->get_parameter("goal.gain", goal_gain_);
  goal_position_ = Vector2(goal_x, goal_y);
  goal_gain_ = std::max(goal_gain_, 0.0);

  std::string default_lidar_topic = topic_with_namespace("/scan");
  std::string default_odom_topic = topic_with_namespace("/odom");
  node_->declare_parameter("lidar_topic", default_lidar_topic);
  node_->declare_parameter("odom_topic", default_odom_topic);

  std::string lidar_topic = default_lidar_topic;
  std::string odom_topic = default_odom_topic;
  node_->get_parameter("lidar_topic", lidar_topic);
  node_->get_parameter("odom_topic", odom_topic);

  // Legacy ARGoS-specific LiDAR subscription retained for reference:
  // lidar_sub_ = node_->create_subscription<argos3_ros2_bridge::msg::LidarList>(
  //   lidar_topic,
  //   rclcpp::SensorDataQoS(),
  //   std::bind(&FlockingController::lidar_sensor_callback, this, _1));

  scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    lidar_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&FlockingController::lidar_sensor_callback, this, _1));

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&FlockingController::odom_callback, this, _1));

  cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(topic_with_namespace("/cmd_vel"), 10);

  if (sync_with_clock_) {
    clock_sub_ = node_->create_subscription<rosgraph_msgs::msg::Clock>(
      clock_topic,
      rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      std::bind(&FlockingController::clock_callback, this, _1));
  } else {
    timer_ = node_->create_wall_timer(control_period_, std::bind(&FlockingController::timer_callback, this));
  }
}

/****************************************/
/*         Sensor Callbacks             */
/****************************************/

// void FlockingController::lidar_sensor_callback(const argos3_ros2_bridge::msg::LidarList & msg) {
//   current_lidar_msg_ = msg;
//   initialized_ = true;
// }

void FlockingController::lidar_sensor_callback(const sensor_msgs::msg::LaserScan & msg) {
  current_scan_ = msg;
  initialized_ = true;
}

void FlockingController::odom_callback(const nav_msgs::msg::Odometry & msg) {
  current_odom_ = msg;
  have_pose_ = true;
}

/****************************************/
/*         Control Loop Step            */
/****************************************/

void FlockingController::timer_callback() {
  perform_control_step();
}

void FlockingController::clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg) {
  if (!msg) {
    return;
  }
  if (msg->clock == last_clock_time_) {
    return;
  }
  last_clock_time_ = msg->clock;
  perform_control_step();
}

void FlockingController::perform_control_step() {
  if (!printed_) {
    auto domain_id = node_->get_node_base_interface()->get_context()->get_domain_id();
    RCLCPP_INFO(node_->get_logger(), "Controller running on ROS_DOMAIN_ID: %zu", domain_id);
    printed_ = true;
  }

  if (!initialized_ || !have_pose_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "Waiting for initialization...");
    return;
  }

  Vector2 goal_vector = vectorToGoal();
  Vector2 flocking_vector = lidarFlockingVector();
  setWheelSpeedsFromVector(goal_vector + flocking_vector);
}

/****************************************/
/*         Vector Calculations          */
/****************************************/

Vector2 FlockingController::vectorToGoal() const {
  if (!have_pose_) {
    return Vector2(0.0, 0.0);
  }

  Vector2 robot_position(
    current_odom_.pose.pose.position.x,
    current_odom_.pose.pose.position.y);

  Vector2 to_goal(goal_position_.x - robot_position.x,
                  goal_position_.y - robot_position.y);
  double distance = to_goal.length();
  if (distance < 1e-4) {
    return Vector2(0.0, 0.0);
  }

  double yaw = quaternionToYaw(current_odom_.pose.pose.orientation);
  Vector2 goal_in_base = rotateWorldToBase(to_goal.normalized(), yaw);

  double speed = std::min(goal_gain_ * distance, wheel_params_.max_speed);
  return goal_in_base * speed;
}

Vector2 FlockingController::lidarFlockingVector() const {
  if (current_scan_.ranges.empty()) {
    return Vector2(0.0, 0.0);
  }

  Vector2 accum(0.0, 0.0);
  int count = 0;
  double angle = current_scan_.angle_min;
  for (const auto & range : current_scan_.ranges) {
    double distance = range;
    if (!std::isfinite(distance)) {
      angle += current_scan_.angle_increment;
      continue;
    }
    if (distance < current_scan_.range_min) {
      distance = current_scan_.range_min;
    }
    if (distance <= 0.0 || distance > flocking_params_.interaction_cutoff || distance > current_scan_.range_max) {
      angle += current_scan_.angle_increment;
      continue;
    }
    if (distance < 1e-3) {
      distance = 1e-3;
    }
    double lj = flocking_params_.generalizedLennardJones(distance);
    accum += Vector2(lj * std::cos(angle),
                     lj * std::sin(angle));
    ++count;
    angle += current_scan_.angle_increment;
  }

  if (count == 0) {
    return Vector2(0.0, 0.0);
  }

  accum = accum / static_cast<double>(count);
  double length = accum.length();
  if (length > wheel_params_.max_speed) {
    accum = accum.normalized() * wheel_params_.max_speed;
  }
  return accum;
}

/****************************************/
/*     Wheel Speed Computation          */
/****************************************/

void FlockingController::setWheelSpeedsFromVector(const Vector2 & heading) {
  double heading_length = heading.length();
  if (heading_length <= 1e-6) {
    geometry_msgs::msg::Twist twist;
    cmd_pub_->publish(twist);
    return;
  }

  double heading_angle = std::atan2(heading.y, heading.x);
  double base_speed = std::min(heading_length, wheel_params_.max_speed);
  double abs_angle = std::fabs(heading_angle);

  if (wheel_params_.turning_mechanism == TurningMechanism::HARD_TURN) {
    if (abs_angle <= wheel_params_.soft_turn_on_angle_threshold){
      wheel_params_.turning_mechanism = TurningMechanism::SOFT_TURN;
    }
  }
  if (wheel_params_.turning_mechanism == TurningMechanism::SOFT_TURN) {
    if (abs_angle > wheel_params_.hard_turn_on_angle_threshold){
      wheel_params_.turning_mechanism = TurningMechanism::HARD_TURN;
    }
    else if (abs_angle <= wheel_params_.no_turn_angle_threshold){
      wheel_params_.turning_mechanism = TurningMechanism::NO_TURN;
    }
  }
  if (wheel_params_.turning_mechanism == TurningMechanism::NO_TURN) {
    if (abs_angle > wheel_params_.hard_turn_on_angle_threshold){
      wheel_params_.turning_mechanism = TurningMechanism::HARD_TURN;
    }
    else if (abs_angle > wheel_params_.no_turn_angle_threshold){
      wheel_params_.turning_mechanism = TurningMechanism::SOFT_TURN;
    }
  }

  double speed1, speed2;
  switch (wheel_params_.turning_mechanism) {
    case TurningMechanism::NO_TURN:
      speed1 = base_speed;
      speed2 = base_speed;
      break;
    case TurningMechanism::SOFT_TURN: {
      double speed_factor = (wheel_params_.hard_turn_on_angle_threshold - abs_angle) /
                            wheel_params_.hard_turn_on_angle_threshold;
      speed_factor = std::clamp(speed_factor, 0.0, 1.0);
      speed1 = base_speed - base_speed * (1.0 - speed_factor);
      speed2 = base_speed + base_speed * (1.0 - speed_factor);
      break;
    }
    case TurningMechanism::HARD_TURN:
      speed1 = -wheel_params_.max_speed;
      speed2 = wheel_params_.max_speed;
      break;
  }

  double left_speed, right_speed;
  if (heading_angle > 0) {
    left_speed  = speed1;
    right_speed = speed2;
  } else {
    left_speed  = speed2;
    right_speed = speed1;
  }

  geometry_msgs::msg::Twist twist;
  twist.linear.x = wheel_radius_ * (left_speed + right_speed) / 2.0;
  twist.angular.z = wheel_radius_ * (right_speed - left_speed) / wheel_separation_;
  // print out the wheel speeds for debugging
  RCLCPP_DEBUG(node_->get_logger(), "Left wheel speed: %.2f, Right wheel speed: %.2f", left_speed, right_speed);
  cmd_pub_->publish(twist);
}

std::string FlockingController::topic_with_namespace(const std::string &suffix) const {
  if (robot_ns_.empty()) {
    return suffix.empty() ? std::string() : suffix;
  }
  if (suffix.empty()) {
    return robot_ns_;
  }
  if (!suffix.empty() && suffix.front() == '/') {
    return robot_ns_ + suffix;
  }
  return robot_ns_ + '/' + suffix;
}

}  // namespace turtlebot_flocking

/**************************
 * Main function
 *************************/
int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("argos_ros_node");
  turtlebot_flocking::FlockingController controller(node);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;

}
