#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <chrono>
#include <rclcpp/publisher.hpp>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "example_interfaces/msg/float64.hpp"

#include "algorithm.hpp"

class VectorFieldNode: public rclcpp::Node
{
public:
  VectorFieldNode()
  : Node("VectorFieldNode")
  {
    // Twist Publisher and Timer
    twist_publisher = this->create_publisher
      <geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10);     
    publish_timer = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&VectorFieldNode::apply_vector_field, this));

    // Publisher for potential
    potential_publisher = this->create_publisher
      <example_interfaces::msg::Float64>(
        "/potential",
        10);     

    // Odom subscriber
    auto sub_callback = std::bind(
      &VectorFieldNode::odom_callback,
      this,
      std::placeholders::_1);
    odom_subscriber = this->create_subscription
      <nav_msgs::msg::Odometry>(
        "/odom",
        10,
        sub_callback);     

    // Trajectory Parameters
    this->declare_parameter("radius", 1.0);
    this->declare_parameter("xc", 0.0);
    this->declare_parameter("yc", 0.0);
    
    // Debug parameter
    this->declare_parameter("debug", false);

    // Initialize t0
    t0 = this->get_clock()->now();
  }

private:

  // ----------- Private Functions ----------------
  
  // Callback for odometry/position
  void odom_callback(nav_msgs::msg::Odometry::UniquePtr odom) 
  {
    pos_x = odom->pose.pose.position.x; 
    pos_y = odom->pose.pose.position.y; 
    pos_t = quaternion_to_yaw(odom->pose.pose.orientation);
  }

  void apply_vector_field()
  {

    // Circle values
    double radius = this->get_parameter("radius").as_double();
    double xc = this->get_parameter("xc").as_double();
    double yc = this->get_parameter("yc").as_double();

    // Getting linear inputs
    // std::array<double, 3> ret_arr = circle_trajectory(xc, yc, radius,
    //                                                   pos_x, pos_y);
    
    double now = this->get_clock()->now().seconds() - t0.seconds();
    std::array<double, 3> ret_arr = moving_trajectory(xc, yc, radius,
                                                      pos_x, pos_y,
                                                      0.1, now);
    double ux = ret_arr[0];
    double uy = ret_arr[1];

    double norm = std::sqrt(ux*ux + uy*uy);
    if (norm > 1) 
    {
      ux = ux/norm; 
      uy = uy/norm;
    }

    // Converting to v and w
    auto input = geometry_msgs::msg::Twist();
    input.linear.x = cos(pos_t) * ux + sin(pos_t) * uy;
    input.angular.z = -sin(pos_t) * ux / dist + cos(pos_t) * uy / dist;

    // Publish the message
    twist_publisher->publish(input);
    
    // Creates potential message and publishes it
    auto pot_msg = example_interfaces::msg::Float64();
    pot_msg.data = ret_arr[2];
    RCLCPP_INFO(this->get_logger(),
                "v: %lf",
                pot_msg.data);
    potential_publisher->publish(pot_msg);

    // Debug messages
    if (this->get_parameter("debug").as_bool()) {
      RCLCPP_INFO(this->get_logger(),
                  "x: %f, y: %f, t: %f",
                  pos_x, pos_y, pos_t);
      RCLCPP_INFO(this->get_logger(),
                  "Holonomic input: ux=%.2f, uy=%.2f",
                  ux, uy);
      RCLCPP_INFO(this->get_logger(),
                  "Publishing: v=%.2f, w=%.2f",
                  input.linear.x, input.angular.z);
    }
  }

  double quaternion_to_yaw(const geometry_msgs::msg::Quaternion & q)
  {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);

    return std::atan2(siny_cosp, cosy_cosp);
  }

  // ----------- Private Variables ----------------

  rclcpp::TimerBase::SharedPtr publish_timer;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher;
  rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr potential_publisher;
  
  // Position variables of the robot
  double pos_x;
  double pos_y;
  double pos_t;

  // Virtual point distance for control
  const double dist = 0.1;

  // Node start instant
  rclcpp::Time t0;
};

int main (int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VectorFieldNode>());
  rclcpp::shutdown();
  return 0;
}
