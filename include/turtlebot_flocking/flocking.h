/***
 * TurtleBot Flocking Controller
 * @author Sindiso Mkhatshwa <sindisomkhat>
 * @file flocking.h
 */
#ifndef TURTLEBOT_FLOCKING_FLOCKING_H_
#define TURTLEBOT_FLOCKING_FLOCKING_H_
#include <cmath>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
// #include "argos3_ros2_bridge/msg/lidar_list.hpp"

namespace turtlebot_flocking
{

  /* 
 * Simple 2D vector class to support polar/cartesian conversions.
 */
struct Vector2 {
  double x;
  double y;
  Vector2(double _x = 0.0, double _y = 0.0) : x(_x), y(_y) {}
  double length() const { return std::hypot(x, y); }
  Vector2 normalized() const {
    double l = length();
    return (l > 0) ? Vector2(x / l, y / l) : Vector2(0.0, 0.0);
  }
  double angle() const { return std::atan2(y, x); }
  Vector2 operator+(const Vector2& other) const {
    return Vector2(x + other.x, y + other.y);
  }
  Vector2& operator+=(const Vector2& other) {
    x += other.x;
    y += other.y;
    return *this;
  }
  Vector2 operator/(double d) const { return Vector2(x / d, y / d); }
  Vector2 operator*(double d) const { return Vector2(x * d, y * d); }
};

/*
 * Turning mechanism state.
 */
enum class TurningMechanism {
  NO_TURN = 0,
  SOFT_TURN,
  HARD_TURN
};

/*
 * Wheel turning parameters.
 */
struct WheelTurningParams {
  TurningMechanism turning_mechanism;
  double hard_turn_on_angle_threshold; // radians
  double soft_turn_on_angle_threshold; // radians
  double no_turn_angle_threshold;      // radians
  double max_speed;                    // maximum wheel speed

  WheelTurningParams()
  : turning_mechanism(TurningMechanism::NO_TURN),
    hard_turn_on_angle_threshold(0.5),
    soft_turn_on_angle_threshold(0.3),
    no_turn_angle_threshold(0.1),
    max_speed(1.0) {}

  void load_from_parameters(std::shared_ptr<rclcpp::Node> node);
};

/*
 * Flocking interaction parameters.
 */
struct FlockingInteractionParams {
  double target_distance; ///< Preferred distance between robots
  double gain;          ///< Gain for the Lennard-Jones potential
  double exponent;      ///< Exponent for the potential
  double interaction_cutoff; ///< Only consider neighbors nearer than this distance

  FlockingInteractionParams()
  : target_distance(1.0),
    gain(1.0),
    exponent(2.0),
    interaction_cutoff(0.3) {}

  void load_from_parameters(std::shared_ptr<rclcpp::Node> node);
  double generalizedLennardJones(double distance) const;
};


/*
 * The ROS2 controller node for TurtleBot3 flocking using LiDAR sensing.
 */
class FlockingController
{
public:
  explicit FlockingController(std::shared_ptr<rclcpp::Node> node);

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::string ns_;
  bool initialized_;
  bool have_pose_;

  // Timer callback for the control loop.
  void timer_callback();
  void lidar_sensor_callback(const sensor_msgs::msg::LaserScan & msg);
  void odom_callback(const nav_msgs::msg::Odometry & msg);

  Vector2 vectorToGoal() const;
  Vector2 lidarFlockingVector() const;
  void setWheelSpeedsFromVector(const Vector2 & heading);

  // Subscribers and publisher.
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Latest sensor data.
  sensor_msgs::msg::LaserScan current_scan_;
  nav_msgs::msg::Odometry current_odom_;

  // Goal configuration.
  Vector2 goal_position_;
  double goal_gain_;

  // Controller parameters.
  WheelTurningParams wheel_params_;
  FlockingInteractionParams flocking_params_;
  double wheel_separation_; // Distance between wheels for kinematics.
  double wheel_radius_;		// Wheel radius
  bool printed_;

};

}  // namespace turtlebot_flocking

#endif  // TURTLEBOT_FLOCKING_FLOCKING_H_
