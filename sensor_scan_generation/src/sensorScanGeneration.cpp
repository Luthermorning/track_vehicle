#include <math.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using namespace std;

// 全局变量定义
// 两个激光雷达的数据指针:laserCloudIn和laserCLoudInSensorFrame
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCLoudInSensorFrame(new pcl::PointCloud<pcl::PointXYZ>());

// 机器人的xyz坐标和roll、pitch和yaw的角度值。
double robotX = 0;
double robotY = 0;
double robotZ = 0;
double roll = 0;
double pitch = 0;
double yaw = 0;

bool newTransformToMap = false;

// pubOdometryPointer和tfBroadcasterPointer是ros::Publisher的指针
nav_msgs::Odometry odometryIn; // odometryIn是一个odom的消息类型
ros::Publisher* pubOdometryPointer = NULL;
tf::StampedTransform transformToMap; // transformToMap是一个tf的消息类型
tf::TransformBroadcaster* tfBroadcasterPointer = NULL;

ros::Publisher pubLaserCloud;

// 该回调函数主要是将激光雷达信息进行了逆变换，转化到map的坐标系下，然后发布了两个话题和一个tf的转换关系
void laserCloudAndOdometryHandler(const nav_msgs::Odometry::ConstPtr& odometry,
    const sensor_msgs::PointCloud2ConstPtr& laserCloud2)
{
    // 清空上次数据
    laserCloudIn->clear();
    laserCLoudInSensorFrame->clear();

    // 读入点云数据
    pcl::fromROSMsg(*laserCloud2, *laserCloudIn);

    // 读入odom数据
    odometryIn = *odometry;

    // 创建一个transformToMap变换，将里程计信息应用于地图坐标系的点云数据。这个变换包括了位置和方向的信息。
    transformToMap.setOrigin(
        tf::Vector3(odometryIn.pose.pose.position.x, odometryIn.pose.pose.position.y, odometryIn.pose.pose.position.z));
    transformToMap.setRotation(tf::Quaternion(odometryIn.pose.pose.orientation.x, odometryIn.pose.pose.orientation.y,
        odometryIn.pose.pose.orientation.z, odometryIn.pose.pose.orientation.w));

    int laserCloudInNum = laserCloudIn->points.size();

    pcl::PointXYZ p1;
    tf::Vector3 vec;

    /*将点云数据从地图坐标系转换到传感器坐标系。
     *对于每个点，使用transformToMap变换将其
     *从地图坐标系变换到传感器坐标系，然后将变换
     *后的点云数据存储在 laserCloudInSensorFrame中。
     */
    for (int i = 0; i < laserCloudInNum; i++) {
        p1 = laserCloudIn->points[i];
        vec.setX(p1.x);
        vec.setY(p1.y);
        vec.setZ(p1.z);

        // 将点从地图坐标系转换到传感器坐标系
        vec = transformToMap.inverse() * vec;

        p1.x = vec.x();
        p1.y = vec.y();
        p1.z = vec.z();

        laserCLoudInSensorFrame->points.push_back(p1);
    }

    // 发布map到sensor_at_scan的odom
    odometryIn.header.stamp = laserCloud2->header.stamp;
    odometryIn.header.frame_id = "map";
    odometryIn.child_frame_id = "sensor_at_scan";
    pubOdometryPointer->publish(odometryIn);

    // 发布map到sensor_at_scan的tf关系
    transformToMap.stamp_ = laserCloud2->header.stamp;
    transformToMap.frame_id_ = "map";
    transformToMap.child_frame_id_ = "sensor_at_scan";
    tfBroadcasterPointer->sendTransform(transformToMap);

    // 发布到sensor_at_scan话题上的点云
    sensor_msgs::PointCloud2 scan_data;
    pcl::toROSMsg(*laserCLoudInSensorFrame, scan_data);
    scan_data.header.stamp = laserCloud2->header.stamp;
    scan_data.header.frame_id = "sensor_at_scan";
    pubLaserCloud.publish(scan_data);
}

// 从主函数中可以看出，程序对/state_estimation和/registered_scan的话题进行了基于时间的接受
// 并且触发了laserCloudAndOdometryHandler的回调函数
// 从而发布出两个话题/state_estimation_at_scan和/sensor_scan
int main(int argc, char** argv)
{
    ros::init(argc, argv, "sensor_scan");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");

    // ROS的消息时间同步
    message_filters::Subscriber<nav_msgs::Odometry> subOdometry;
    message_filters::Subscriber<sensor_msgs::PointCloud2> subLaserCloud;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> syncPolicy;
    typedef message_filters::Synchronizer<syncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;
    // CMU设置了loam_interface功能包进行与SLAM算法的转换，从而将下面两个话题发布出来
    subOdometry.subscribe(nh, "/state_estimation", 1); // 由仿真车辆发布
    subLaserCloud.subscribe(nh, "/registered_scan", 1); // 由仿真车辆发布
    sync_.reset(new Sync(syncPolicy(100), subOdometry, subLaserCloud));
    sync_->registerCallback(boost::bind(laserCloudAndOdometryHandler, _1, _2));

    ros::Publisher pubOdometry = nh.advertise<nav_msgs::Odometry>("/state_estimation_at_scan", 5);
    pubOdometryPointer = &pubOdometry;

    tf::TransformBroadcaster tfBroadcaster;
    tfBroadcasterPointer = &tfBroadcaster;

    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/sensor_scan", 2);

    ros::spin();

    return 0;
}
