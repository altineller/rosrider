#ifndef ROSRIDER_FAKENODE__ROSRIDER_FAKENODE_HPP_
#define ROSRIDER_FAKENODE__ROSRIDER_FAKENODE_HPP_

#include <tf2/LinearMath/Quaternion.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#define LEFT 0
#define RIGHT 1

class ROSRiderFake : public rclcpp::Node {

public:
  ROSRiderFake();
  ~ROSRiderFake();

private:

  rclcpp::Time last_cmd_vel_time;
  rclcpp::Time prev_update_time;

  rclcpp::TimerBase::SharedPtr update_timer;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;

  nav_msgs::msg::Odometry odom;
  sensor_msgs::msg::JointState joint_states;

  double wheel_speed_cmd[2];
  double goal_linear_velocity;
  double goal_angular_velocity;
  double cmd_vel_timeout;
  double last_position[2];
  double last_velocity[2];
  float odom_pose[3];
  float odom_vel[3];

  double wheel_seperation;
  double wheel_radius;

  // Function prototypes
  void init_parameters();
  void init_variables();
  void command_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg);
  void update_callback();
  bool update_odometry(const rclcpp::Duration & diff_time);
  void update_joint_state();
  void update_tf(geometry_msgs::msg::TransformStamped & odom_tf);
};
#endif
