#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>

#include <string>
#include <vector>
#include <utility>
#include <set>
#include <cstdio>
#include <fstream>

const double math_pi = 3.141592;
const double wheel_base = 0.4168;

geometry_msgs::msg::Pose2D able_odom;
nav_msgs::msg::Odometry odom;
geometry_msgs::msg::TransformStamped odom_trans;
float dt;
rclcpp::Time current_time, last_time;

float velR = 0.0, velL = 0.0;

void VelRCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    velL = msg->data;
}

void VelLCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    velR = msg->data;
}

void CalcAblePosition()
{
    current_time = rclcpp::Clock().now();
    dt = (current_time - last_time).seconds();
    last_time = current_time;
    
    //... [rest of the function remains same]

    // If you want to ensure theta remains between 0 and 2*pi or -pi and pi, uncomment the section.
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("mobile_localization_node");

    rclcpp::Rate loop_rate(10);

    auto pub_able_odom = node->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    auto odom_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    auto sub_mobile_vel_R = node->create_subscription<std_msgs::msg::Float32>("/mobile/velR", 10, VelRCallback);
    auto sub_mobile_vel_L = node->create_subscription<std_msgs::msg::Float32>("/mobile/velL", 10, VelLCallback);

    able_odom.x = 0.0;
    able_odom.y = 0.0;
    able_odom.theta = 0.0;
    
    while (rclcpp::ok())
    {
        CalcAblePosition();
        odom_broadcaster->sendTransform(odom_trans);
        pub_able_odom->publish(odom);
        
        loop_rate.sleep();
        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
    return 0;
}
