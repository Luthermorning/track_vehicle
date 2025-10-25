#include "std_msgs/Float64.h"
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <ros/ros.h>

std_msgs::Float64 left_vel;
std_msgs::Float64 right_vel;

ros::Publisher pub_lf;
ros::Publisher pub_lb;
ros::Publisher pub_rf;
ros::Publisher pub_rb;

double base_link_lenth = 2.881;

class Turn {
private:
    // double wheel_radius;
    ros::NodeHandle nh;

public:
    Turn()
    {
        pub_lf = nh.advertise<std_msgs::Float64>("/track_vehicle/left_crawler_sprocket_velocity_controller/command", 1);
        pub_lb = nh.advertise<std_msgs::Float64>("/track_vehicle/right_crawler_sprocket_velocity_controller/command", 1);

        // ros::NodeHandle nh_private("~");
        // nh_private.param("wheel_radius", wheel_radius, 0.46);

        left_vel.data = 0;
        right_vel.data = 0;
    }
    ~Turn() { }
};

void doturn_base(const geometry_msgs::Twist::ConstPtr& msg)
{
    left_vel.data = msg->linear.x - (msg->angular.z / 2) * base_link_lenth;
    right_vel.data = msg->linear.x + (msg->angular.z / 2) * base_link_lenth;

    ROS_INFO("left is %.2f and right is %.2f\n", left_vel.data, right_vel.data);
    // ROS_INFO("%.2f\n", base_link_lenth);

    pub_lf.publish(left_vel);
    pub_lb.publish(right_vel);
}

int main(int argc, char* argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "turn", ros::init_options::AnonymousName);

    Turn turn;
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<geometry_msgs::Twist>("track_vehicle_cmd_vel", 10, doturn_base);

    ros::spin();
}