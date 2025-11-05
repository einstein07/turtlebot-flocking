#include "turtlebot_flocking/flocking.h"

#include <rclcpp/qos.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace turtlebot_flocking
{

/****************************************/
/*    Parameter Loading Functions       */
/****************************************/

void WheelTurningParams::load_from_parameters(std::shared_ptr<rclcpp::Node> node, const std::string & ns) {
  node->declare_parameter(ns + ".hard_turn_on_angle_threshold", hard_turn_on_angle_threshold);
  node->declare_parameter(ns + ".soft_turn_on_angle_threshold", soft_turn_on_angle_threshold);
  node->declare_parameter(ns + ".no_turn_angle_threshold", no_turn_angle_threshold);
  node->declare_parameter(ns + ".max_speed", max_speed);

  node->get_parameter(ns + ".hard_turn_on_angle_threshold", hard_turn_on_angle_threshold);
  hard_turn_on_angle_threshold = hard_turn_on_angle_threshold * M_PI / 180.0;
  node->get_parameter(ns + ".soft_turn_on_angle_threshold", soft_turn_on_angle_threshold);
  soft_turn_on_angle_threshold = soft_turn_on_angle_threshold * M_PI / 180.0;
  node->get_parameter(ns + ".no_turn_angle_threshold", no_turn_angle_threshold);
  no_turn_angle_threshold = no_turn_angle_threshold * M_PI / 180.0;
  node->get_parameter(ns + ".max_speed", max_speed);

  turning_mechanism = TurningMechanism::NO_TURN;
}

void FlockingInteractionParams::load_from_parameters(std::shared_ptr<rclcpp::Node> node, const std::string & ns) {
  node->declare_parameter(ns + ".target_distance", target_distance);
  node->declare_parameter(ns + ".gain", gain);
  node->declare_parameter(ns + ".exponent", exponent);

  node->get_parameter(ns + ".target_distance", target_distance);
  node->get_parameter(ns + ".gain", gain);
  node->get_parameter(ns + ".exponent", exponent);

  double default_range = interaction_cutoff * target_distance;
  node->declare_parameter(ns + ".interaction_range", default_range);
  node->get_parameter(ns + ".interaction_range", interaction_cutoff);
  if (interaction_cutoff <= 0.0) {
    interaction_cutoff = default_range;
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
  ns_(node_->get_namespace()),
  initialized_(false),
  have_pose_(false),
  goal_position_(0.0,10.0),
  goal_gain_(1000.0),
  wheel_separation_(0.160),
  wheel_radius_(0.033),
  printed_(false)
{
  wheel_params_.load_from_parameters(node_, "wheel_turning");
  flocking_params_.load_from_parameters(node_, "flocking");

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

  std::string ns_prefix = ns_;
  if (ns_prefix == "/") {
    ns_prefix.clear();
  }

  std::string default_lidar_topic = ns_prefix + "/lidarList";
  std::string default_odom_topic = ns_prefix + "/position";
  node_->declare_parameter("lidar_topic", default_lidar_topic);
  node_->declare_parameter("odom_topic", default_odom_topic);

  std::string lidar_topic = default_lidar_topic;
  std::string odom_topic = default_odom_topic;
  node_->get_parameter("lidar_topic", lidar_topic);
  node_->get_parameter("odom_topic", odom_topic);

  lidar_sub_ = node_->create_subscription<argos3_ros2_bridge::msg::LidarList>(
    lidar_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&FlockingController::lidar_sensor_callback, this, _1));

  odom_sub_ = node_->create_subscription<argos3_ros2_bridge::msg::Position>(
    odom_topic,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&FlockingController::odom_callback, this, _1));

  cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  timer_ = node_->create_wall_timer(20ms, std::bind(&FlockingController::timer_callback, this));
}

/****************************************/
/*         Sensor Callbacks             */
/****************************************/

void FlockingController::lidar_sensor_callback(const argos3_ros2_bridge::msg::LidarList & msg) {
  current_lidar_msg_ = msg;
  initialized_ = true;
}

void FlockingController::odom_callback(const argos3_ros2_bridge::msg::Position & msg) {
  current_odom_ = msg;
  have_pose_ = true;
}

/****************************************/
/*         Control Loop Step            */
/****************************************/

void FlockingController::timer_callback() {
  if (!printed_) {
    auto domain_id = node_->get_node_base_interface()->get_context()->get_domain_id();
    RCLCPP_INFO(node_->get_logger(), "Controller running on ROS_DOMAIN_ID: %u", domain_id);
    printed_ = true;
  }

  if (!initialized_ || !have_pose_) {
    // print deubugging info
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
    current_odom_.position.x,
    current_odom_.position.y);

  Vector2 to_goal(goal_position_.x - robot_position.x,
                  goal_position_.y - robot_position.y);
  double distance = to_goal.length();
  if (distance < 1e-4) {
    return Vector2(0.0, 0.0);
  }

  double yaw = quaternionToYaw(current_odom_.orientation);
  Vector2 goal_in_base = rotateWorldToBase(to_goal.normalized(), yaw);

  double speed = std::min(goal_gain_ * distance, wheel_params_.max_speed);
  return goal_in_base * speed;
}

Vector2 FlockingController::lidarFlockingVector() const {
  if (current_lidar_msg_.lidars.empty()) {
    return Vector2(0.0, 0.0);
  }

  Vector2 accum(0.0, 0.0);
  int count = 0;
  for (const auto & reading : current_lidar_msg_.lidars) {
    double distance = reading.value;
    if (distance <= 0.0 || distance > flocking_params_.interaction_cutoff) {
      continue;
    }
    if (distance < 1e-3) {
      distance = 1e-3;
    }
    double lj = flocking_params_.generalizedLennardJones(distance);
    accum += Vector2(lj * std::cos(reading.angle),
                     lj * std::sin(reading.angle));
    ++count;
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