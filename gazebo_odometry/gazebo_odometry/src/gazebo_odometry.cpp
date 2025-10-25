#include <gazebo_msgs/GetModelState.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gazebo_odometry");
    ros::NodeHandle n;

    ros::ServiceClient states_client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state"); // 客户端获取机器人模型状态
    ros::Publisher odometry_pub = n.advertise<nav_msgs::Odometry>("/odom", 10); // 定义里程计

    static tf::TransformBroadcaster odom_br; // tf广播
    tf::Transform transform;
    tf::Quaternion quaternion;
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        gazebo_msgs::GetModelState model_states;
        nav_msgs::Odometry odom;

        model_states.request.model_name = "my"; // gazebo 查看模型名字
        model_states.request.relative_entity_name = "world"; // 参考坐标系
        states_client.call(model_states); // 客户端发送请求

        odom.pose.pose = model_states.response.pose;
        odom.twist.twist = model_states.response.twist;
        bool success = model_states.response.success;
        odometry_pub.publish(odom); // 发布里程计

        tf::quaternionMsgToTF(odom.pose.pose.orientation, quaternion); // quaternion msg转tf
        //    quaternion.setRPY(0, 0, tf::getYaw(odom.pose.pose.orientation));
        transform.setOrigin(tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));
        transform.setRotation(quaternion);
        odom_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "center_unit")); // 广播 odom到base_link的tf

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
