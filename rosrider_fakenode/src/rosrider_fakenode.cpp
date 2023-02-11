#include "rosrider_fakenode/rosrider_fakenode.hpp"

#include <memory>
#include <string>

using namespace std::chrono_literals;

ROSRiderFake::ROSRiderFake(): Node("rosrider_fakenode") {

  init_parameters();
  init_variables();

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
  joint_states_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", qos);
  tf_pub = this->create_publisher<tf2_msgs::msg::TFMessage>("tf", qos);

  cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", qos, \
  std::bind(&ROSRiderFake::command_velocity_callback, this, std::placeholders::_1));

  update_timer = this->create_wall_timer(10ms, std::bind(&ROSRiderFake::update_callback, this));

  RCLCPP_INFO(this->get_logger(), "fakenode has been initialised");
  
}

ROSRiderFake::~ROSRiderFake() {
  RCLCPP_INFO(this->get_logger(), "fakenode has been terminated");
}

void ROSRiderFake::init_parameters() {

  this->declare_parameter<std::string>("joint_states_frame");
  this->declare_parameter<std::string>("odom_frame");
  this->declare_parameter<std::string>("base_frame");
  this->declare_parameter<double>("wheels.separation");
  this->declare_parameter<double>("wheels.radius");

  // Get parameters from yaml
  this->get_parameter_or<std::string>("joint_states_frame", joint_states.header.frame_id, "base_footprint");
  this->get_parameter_or<std::string>("odom_frame", odom.header.frame_id, "odom");
  this->get_parameter_or<std::string>("base_frame", odom.child_frame_id, "base_footprint");
  this->get_parameter_or<double>("wheels.separation", wheel_seperation, 0.0);
  this->get_parameter_or<double>("wheels.radius", wheel_radius, 0.0);
}

void ROSRiderFake::init_variables() {

  wheel_speed_cmd[LEFT] = 0.0;
  wheel_speed_cmd[RIGHT] = 0.0;
  goal_linear_velocity = 0.0;
  goal_angular_velocity = 0.0;
  cmd_vel_timeout = 1.0;
  last_position[LEFT] = 0.0;
  last_position[RIGHT] = 0.0;
  last_velocity[LEFT] = 0.0;
  last_velocity[RIGHT] = 0.0;

  // TODO P3
  // double pcov[36] = { 0.1,   0,   0,   0,   0, 0,
  //                       0, 0.1,   0,   0,   0, 0,
  //                       0,   0, 1e6,   0,   0, 0,
  //                       0,   0,   0, 1e6,   0, 0,
  //                       0,   0,   0,   0, 1e6, 0,
  //                       0,   0,   0,   0,   0, 0.2};
  // memcpy(&(odom_.pose.covariance), pcov, sizeof(double)*36);
  // memcpy(&(odom_.twist.covariance), pcov, sizeof(double)*36);

  odom_pose[0] = 0.0;
  odom_pose[1] = 0.0;
  odom_pose[2] = 0.0;
  odom_vel[0] = 0.0;
  odom_vel[1] = 0.0;
  odom_vel[2] = 0.0;

  joint_states.name.push_back("wheel_left_joint");
  joint_states.name.push_back("wheel_right_joint");
  joint_states.position.resize(2, 0.0);
  joint_states.velocity.resize(2, 0.0);
  joint_states.effort.resize(2, 0.0);

  prev_update_time = this->now();
  last_cmd_vel_time = this->now();

}

void ROSRiderFake::command_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg) {

  last_cmd_vel_time = this->now();

  goal_linear_velocity = cmd_vel_msg->linear.x;
  goal_angular_velocity = cmd_vel_msg->angular.z;

  wheel_speed_cmd[LEFT] = goal_linear_velocity - (goal_angular_velocity * wheel_seperation / 2);
  wheel_speed_cmd[RIGHT] = goal_linear_velocity + (goal_angular_velocity * wheel_seperation / 2);

}

void ROSRiderFake::update_callback() {

  rclcpp::Time time_now = this->now();
  rclcpp::Duration duration(time_now - prev_update_time);
  prev_update_time = time_now;

  // zero-ing after timeout (stop the robot if no cmd_vel)
  if ((time_now - last_cmd_vel_time).nanoseconds() / 1e9 > cmd_vel_timeout) {
    wheel_speed_cmd[LEFT] = 0.0;
    wheel_speed_cmd[RIGHT] = 0.0;
  }

  // odom
  update_odometry(duration);
  odom.header.stamp = time_now;
  odom_pub->publish(odom);

  // joint_states
  update_joint_state();
  joint_states.header.stamp = time_now;
  joint_states_pub->publish(joint_states);

  // tf
  geometry_msgs::msg::TransformStamped odom_tf;
  update_tf(odom_tf);
  tf2_msgs::msg::TFMessage odom_tf_msg;
  odom_tf_msg.transforms.push_back(odom_tf);
  tf_pub->publish(odom_tf_msg);
}

bool ROSRiderFake::update_odometry(const rclcpp::Duration & duration) {

  double wheel_l, wheel_r;  // rotation value of wheel [rad]
  double delta_s, delta_theta;
  double v[2], w[2];
  double step_time = duration.nanoseconds() / 1e9;  // [sec]

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = 0.0;

  // v = translational velocity [m/s]
  // w = rotational velocity [rad/s]
  v[LEFT] = wheel_speed_cmd[LEFT];
  w[LEFT] = v[LEFT] / wheel_radius;  // w = v / r
  v[RIGHT] = wheel_speed_cmd[RIGHT];
  w[RIGHT] = v[RIGHT] / wheel_radius;

  last_velocity[LEFT] = w[LEFT];
  last_velocity[RIGHT] = w[RIGHT];

  wheel_l = w[LEFT] * step_time;
  wheel_r = w[RIGHT] * step_time;

  if(isnan(wheel_l)) { wheel_l = 0.0; }
  if(isnan(wheel_r)) { wheel_r = 0.0; }

  last_position[LEFT] += wheel_l;
  last_position[RIGHT] += wheel_r;

  delta_s = wheel_radius * (wheel_r + wheel_l) / 2.0;
  delta_theta = wheel_radius * (wheel_r - wheel_l) / wheel_seperation;

  // compute odometric pose
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2] += delta_theta;

  // compute odometric instantaneouse velocity
  odom_vel[0] = delta_s / step_time;     // v
  odom_vel[1] = 0.0;
  odom_vel[2] = delta_theta / step_time;  // w

  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;

  tf2::Quaternion q;
  q.setRPY(0, 0, odom_pose[2]);

  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  // We should update the twist of the odometry
  odom.twist.twist.linear.x = odom_vel[0];
  odom.twist.twist.angular.z = odom_vel[2];

  return true;

}

void ROSRiderFake::update_joint_state() {
  joint_states.position[LEFT] = last_position[LEFT];
  joint_states.position[RIGHT] = last_position[RIGHT];
  joint_states.velocity[LEFT] = last_velocity[LEFT];
  joint_states.velocity[RIGHT] = last_velocity[RIGHT];
}

void ROSRiderFake::update_tf(geometry_msgs::msg::TransformStamped & odom_tf) {
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = odom.child_frame_id;
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation = odom.pose.pose.orientation;
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ROSRiderFake>());
  rclcpp::shutdown();
  return 0;
}
