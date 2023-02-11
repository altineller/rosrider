#include "rosrider_gazebo/rosrider_drive.hpp"

#include <memory>

using namespace std::chrono_literals;

ROSRiderDrive::ROSRiderDrive(): Node("rosrider_drive_node") {

  scan_data_[0] = 0.0;
  scan_data_[1] = 0.0;
  scan_data_[2] = 0.0;

  robot_pose_ = 0.0;
  prev_robot_pose_ = 0.0;

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

  // Initialise subscribers
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(), std::bind(&ROSRiderDrive::scan_callback, this, std::placeholders::_1));
    
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", qos, std::bind(&ROSRiderDrive::odom_callback, this, std::placeholders::_1));

  update_timer_ = this->create_wall_timer(10ms, std::bind(&ROSRiderDrive::update_callback, this));

  RCLCPP_INFO(this->get_logger(), "simulation node has been initialised");
  
}

ROSRiderDrive::~ROSRiderDrive() {
  RCLCPP_INFO(this->get_logger(), "simulation node has been terminated");
}

void ROSRiderDrive::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  robot_pose_ = yaw;
}

void ROSRiderDrive::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  uint16_t scan_angle[3] = {0, 30, 330};
  for (int num = 0; num < 3; num++) {
    if (std::isinf(msg->ranges.at(scan_angle[num]))) {
      scan_data_[num] = msg->range_max;
    } else {
      scan_data_[num] = msg->ranges.at(scan_angle[num]);
    }
  }
}

void ROSRiderDrive::update_cmd_vel(double linear, double angular) {
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear;
  cmd_vel.angular.z = angular;
  cmd_vel_pub_->publish(cmd_vel);
}

void ROSRiderDrive::update_callback() {

  static uint8_t robot_state_num = 0;
  double escape_range = 30.0 * DEG2RAD;
  double check_forward_dist = 0.7;
  double check_side_dist = 0.6;

  switch (robot_state_num) {
    case GET_DIRECTION:
      if (scan_data_[CENTER] > check_forward_dist) {
        if (scan_data_[LEFT] < check_side_dist) {
          prev_robot_pose_ = robot_pose_;
          robot_state_num = RIGHT_TURN;
        } else if (scan_data_[RIGHT] < check_side_dist) {
          prev_robot_pose_ = robot_pose_;
          robot_state_num = LEFT_TURN;
        } else {
          robot_state_num = DRIVE_FORWARD;
        }
      }

      if (scan_data_[CENTER] < check_forward_dist) {
        prev_robot_pose_ = robot_pose_;
        robot_state_num = RIGHT_TURN;
      }
      break;

    case DRIVE_FORWARD:
      update_cmd_vel(LINEAR_VELOCITY, 0.0);
      robot_state_num = GET_DIRECTION;
      break;

    case RIGHT_TURN:
      if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range) {
        robot_state_num = GET_DIRECTION;
      } else {
        update_cmd_vel(0.0, -1 * ANGULAR_VELOCITY);
      }
      break;

    case LEFT_TURN:
      if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range) {
        robot_state_num = GET_DIRECTION;
      } else {
        update_cmd_vel(0.0, ANGULAR_VELOCITY);
      }
      break;

    default:
      robot_state_num = GET_DIRECTION;
      break;
  }
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ROSRiderDrive>());
  rclcpp::shutdown();
  return 0;
}
