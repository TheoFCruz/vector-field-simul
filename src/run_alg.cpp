#include <cmath>
#include <functional>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/publisher.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

class VectorFieldNode: public rclcpp::Node
{
public:
  VectorFieldNode()
  : Node("VectorFieldNode")
  {
    // Twist Publisher
    twist_publisher = this->create_publisher
      <geometry_msgs::msg::Twist>(
        "/cmd_vel",
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
  }

private:
  // Callback for odometry/position
  void odom_callback(nav_msgs::msg::Odometry::UniquePtr odom) 
  {
    pos_x = odom->pose.pose.position.x; 
    pos_y = odom->pose.pose.position.y; 
    pos_t = quaternion_to_yaw(odom->pose.pose.orientation);

    // RCLCPP_INFO(this->get_logger(),
    //             "x: %f, y: %f, t: %f",
    //             pos_x, pos_y, pos_t);
  }

  void apply_vector_field()
  {
    // TODO: Apply vector field algorithm
    float ux = 0;
    float uy = 0;
    
    auto input = geometry_msgs::msg::Twist();
    input.linear.x = cos(pos_t) * ux + sin(pos_t) * uy;
    input.angular.z = -sin(pos_t) * ux / dist + cos(pos_t) * uy / dist;

    // TODO: Publish the message
  }

  float quaternion_to_yaw(const geometry_msgs::msg::Quaternion & q)
  {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);

    return std::atan2(siny_cosp, cosy_cosp);
  }


  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
  
  // Position variables of the robot
  float pos_x;
  float pos_y;
  float pos_t;

  // Virtual point distance for control
  const float dist = 0.05;
};

int main (int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VectorFieldNode>());
  rclcpp::shutdown();
  return 0;
}
