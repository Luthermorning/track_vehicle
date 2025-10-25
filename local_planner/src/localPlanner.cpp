#include <math.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

const double PI = 3.1415926;

#define PLOTPATHSET 1 // 显示路径集的控制量

string pathFolder; // 使用matlab生成路径集合的文件路径
double vehicleLength = 0.6; // 车辆的长度，单位m
double vehicleWidth = 0.6; // 车辆的宽度，单位m
double sensorOffsetX = 0; // 传感器坐标系与车体中心的偏移量
double sensorOffsetY = 0; // 传感器坐标系与车体中心的偏移量
bool twoWayDrive = true; // 双向驱动
double laserVoxelSize = 0.05; // 下采样平面体素栅格叶大小 0.05
double terrainVoxelSize = 0.2; // 下采样地面体素栅格叶大小 0.2
bool useTerrainAnalysis = false; // 是否使用地面分割后的点云信息
bool checkObstacle = true; // 是否进行障碍物检测
bool checkRotObstacle = false; // 是否检测旋转时障碍物
double adjacentRange = 3.5; // 裁剪点云时的距离
double obstacleHeightThre = 0.2; // 障碍物高度阈值
double groundHeightThre = 0.1; // 地面高度阈值
double costHeightThre = 0.1; // 计算路径惩罚得分的权重
double costScore = 0.02; // 最小惩罚得分
bool useCost = false;
const int laserCloudStackNum = 1; // 缓存的激光点云帧数量
int laserCloudCount = 0; // 当laserCloudStackNum = 1时,暂时没用到
int pointPerPathThre = 2; // 每条路径需要有几个被遮挡的点
double minRelZ = -0.5; // 未使用地面分割时，裁剪点云时的最小高度
double maxRelZ = 0.25; // 未使用地面分割时，裁剪点云时的最大高度
double maxSpeed = 1.0; // 最大速度
double dirWeight = 0.02; // 计算得分时转向角度的权重
double dirThre = 90.0; // 最大转向角度
bool dirToVehicle = false; // 是否以车辆为主方向计算被遮挡的路径
double pathScale = 1.0; // 路径尺度
double minPathScale = 0.75; // 最小路径尺度
double pathScaleStep = 0.25; // 路径尺度的调整步长
bool pathScaleBySpeed = true; // 是否根据速度调整路径尺度
double minPathRange = 1.0; // 最小路径距离
double pathRangeStep = 0.5; // 路径范围的调整步长
bool pathRangeBySpeed = true; // 是否根据速度调整路径的范围
bool pathCropByGoal = true; // 是否根据目标点+ goalClearRange 筛选点云数据
bool autonomyMode = false;
double autonomySpeed = 1.0;
double joyToSpeedDelay = 2.0;
double joyToCheckObstacleDelay = 5.0;
double goalClearRange = 0.5; // 当 pathCropByGoal = true 时,点云距离超过目标点+该值则不被处理
double goalX = 0; // 局部路径目标点
double goalY = 0; // 局部路径目标点

float joySpeed = 0; // 车辆的线速度
float joySpeedRaw = 0; // 车辆的角速度
float joyDir = 0; // 车辆目标位置和车辆位置的相对夹角

const int pathNum = 343; // 总共生成的路径数
const int groupNum = 7; // 路径的组数
float gridVoxelSize = 0.02; // 障碍物检测的一个单元格大小
float searchRadius = 0.45; // 车辆的半径
float gridVoxelOffsetX = 3.2; // Ｘ方向的偏移量
float gridVoxelOffsetY = 4.5; // Ｙ方向的偏移量
const int gridVoxelNumX = 161; // 3.2 / 0.2 = 160
const int gridVoxelNumY = 451; // 4.5 / 0.2 = 451
const int gridVoxelNum = gridVoxelNumX * gridVoxelNumY;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudStack[laserCloudStackNum];
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr boundaryCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr addedObstacles(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZ>::Ptr startPaths[groupNum];
#if PLOTPATHSET == 1
pcl::PointCloud<pcl::PointXYZI>::Ptr paths[pathNum];
pcl::PointCloud<pcl::PointXYZI>::Ptr freePaths(new pcl::PointCloud<pcl::PointXYZI>());
#endif

int pathList[pathNum] = { 0 };
float endDirPathList[pathNum] = { 0 };
int clearPathList[36 * pathNum] = { 0 };
float pathPenaltyList[36 * pathNum] = { 0 };
float clearPathPerGroupScore[36 * groupNum] = { 0 };
std::vector<int> correspondences[gridVoxelNum];

bool newLaserCloud = false;
bool newTerrainCloud = false;

double odomTime = 0;
double joyTime = 0;

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;

pcl::VoxelGrid<pcl::PointXYZI> laserDwzFilter, terrainDwzFilter;

// 将车辆的odom的信息转换到车辆坐标系下，只进行了坐标系的转换
void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
    odomTime = odom->header.stamp.toSec();

    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

    vehicleRoll = roll;
    vehiclePitch = pitch;
    vehicleYaw = yaw;
    vehicleX = odom->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
    vehicleY = odom->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
    vehicleZ = odom->pose.pose.position.z;
}

// 对传输过来的点云信息进行滤波，主要完成小于adjacentRange的点云信息的滤波，然后将点云信息进行下采样
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloud2)
{
    if (!useTerrainAnalysis) {
        laserCloud->clear();
        pcl::fromROSMsg(*laserCloud2, *laserCloud);

        pcl::PointXYZI point;
        laserCloudCrop->clear();
        int laserCloudSize = laserCloud->points.size();
        for (int i = 0; i < laserCloudSize; i++) {
            point = laserCloud->points[i];

            float pointX = point.x;
            float pointY = point.y;
            float pointZ = point.z;

            float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));
            if (dis < adjacentRange) {
                point.x = pointX;
                point.y = pointY;
                point.z = pointZ;
                laserCloudCrop->push_back(point);
            }
        }

        laserCloudDwz->clear();
        laserDwzFilter.setInputCloud(laserCloudCrop);
        laserDwzFilter.filter(*laserCloudDwz);

        newLaserCloud = true;
    }
}

// 主要完成对于地面点云信息的滤波，也是完成小于adjacentRange的点云信息的滤波，然后将点云信息进行下采样
void terrainCloudHandler(const sensor_msgs::PointCloud2ConstPtr& terrainCloud2)
{
    if (useTerrainAnalysis) {
        terrainCloud->clear();
        pcl::fromROSMsg(*terrainCloud2, *terrainCloud);

        pcl::PointXYZI point;
        terrainCloudCrop->clear();
        int terrainCloudSize = terrainCloud->points.size();
        for (int i = 0; i < terrainCloudSize; i++) {
            point = terrainCloud->points[i];

            float pointX = point.x;
            float pointY = point.y;
            float pointZ = point.z;

            float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));
            if (dis < adjacentRange && (point.intensity > obstacleHeightThre || useCost)) {
                point.x = pointX;
                point.y = pointY;
                point.z = pointZ;
                terrainCloudCrop->push_back(point);
            }
        }

        terrainCloudDwz->clear();
        terrainDwzFilter.setInputCloud(terrainCloudCrop);
        terrainDwzFilter.filter(*terrainCloudDwz);

        newTerrainCloud = true;
    }
}
// 主要完成关于手柄摇杆的参数配置，主要包括线速度、角速度、自主 / 遥控模式、障碍物检测的配置
void joystickHandler(const sensor_msgs::Joy::ConstPtr& joy)
{
    joyTime = ros::Time::now().toSec();

    joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
    joySpeed = joySpeedRaw;
    if (joySpeed > 1.0)
        joySpeed = 1.0;
    if (joy->axes[4] == 0)
        joySpeed = 0;

    if (joySpeed > 0) {
        joyDir = atan2(joy->axes[3], joy->axes[4]) * 180 / PI;
        if (joy->axes[4] < 0)
            joyDir *= -1;
    }

    if (joy->axes[4] < 0 && !twoWayDrive)
        joySpeed = 0;

    if (joy->axes[2] > -0.1) {
        autonomyMode = false;
    } else {
        autonomyMode = true;
    }

    if (joy->axes[5] > -0.1) {
        checkObstacle = true;
    } else {
        checkObstacle = false;
    }
}

// 主要完成当前目标点的配置
void goalHandler(const geometry_msgs::PointStamped::ConstPtr& goal)
{
    goalX = goal->point.x;
    goalY = goal->point.y;
}

// 主要完成对于车速的配置和滤波
void speedHandler(const std_msgs::Float32::ConstPtr& speed)
{
    double speedTime = ros::Time::now().toSec();

    if (autonomyMode && speedTime - joyTime > joyToSpeedDelay && joySpeedRaw == 0) {
        joySpeed = speed->data / maxSpeed;

        if (joySpeed < 0)
            joySpeed = 0;
        else if (joySpeed > 1.0)
            joySpeed = 1.0;
    }
}

// 主要完成对于区域边界的解析，从而判定当前的主要区域边界
void boundaryHandler(const geometry_msgs::PolygonStamped::ConstPtr& boundary)
{
    boundaryCloud->clear();
    pcl::PointXYZI point, point1, point2;
    int boundarySize = boundary->polygon.points.size();

    if (boundarySize >= 1) {
        point2.x = boundary->polygon.points[0].x;
        point2.y = boundary->polygon.points[0].y;
        point2.z = boundary->polygon.points[0].z;
    }

    for (int i = 0; i < boundarySize; i++) {
        point1 = point2;

        point2.x = boundary->polygon.points[i].x;
        point2.y = boundary->polygon.points[i].y;
        point2.z = boundary->polygon.points[i].z;

        if (point1.z == point2.z) {
            float disX = point1.x - point2.x;
            float disY = point1.y - point2.y;
            float dis = sqrt(disX * disX + disY * disY);

            int pointNum = int(dis / terrainVoxelSize) + 1;
            for (int pointID = 0; pointID < pointNum; pointID++) {
                point.x = float(pointID) / float(pointNum) * point1.x + (1.0 - float(pointID) / float(pointNum)) * point2.x;
                point.y = float(pointID) / float(pointNum) * point1.y + (1.0 - float(pointID) / float(pointNum)) * point2.y;
                point.z = 0;
                point.intensity = 100.0;

                for (int j = 0; j < pointPerPathThre; j++) {
                    boundaryCloud->push_back(point);
                }
            }
        }
    }
}

// 主要完成对于障碍物点云信息的检测，将障碍物的点云回波强度设置为200
void addedObstaclesHandler(const sensor_msgs::PointCloud2ConstPtr& addedObstacles2)
{
    addedObstacles->clear();
    pcl::fromROSMsg(*addedObstacles2, *addedObstacles);

    int addedObstaclesSize = addedObstacles->points.size();
    for (int i = 0; i < addedObstaclesSize; i++) {
        addedObstacles->points[i].intensity = 200.0;
    }
}

// 主要完成障碍物检测数据时间的检测，可能由于ROS的时间精度不够，程序全部采用了高精度时间戳，从而需要对各种数据进行时间的检测
void checkObstacleHandler(const std_msgs::Bool::ConstPtr& checkObs)
{
    double checkObsTime = ros::Time::now().toSec();

    if (autonomyMode && checkObsTime - joyTime > joyToCheckObstacleDelay) {
        checkObstacle = checkObs->data;
    }
}

// 这是一个打开文件，读取文件内容的程序，后续的文件都依赖于它进行读取
int readPlyHeader(FILE* filePtr)
{
    char str[50];
    int val, pointNum;
    string strCur, strLast;
    while (strCur != "end_header") {
        val = fscanf(filePtr, "%s", str);
        if (val != 1) {
            printf("\nError reading input files, exit.\n\n");
            exit(1);
        }

        strLast = strCur;
        strCur = string(str);

        if (strCur == "vertex" && strLast == "element") {
            val = fscanf(filePtr, "%d", &pointNum);
            if (val != 1) {
                printf("\nError reading input files, exit.\n\n");
                exit(1);
            }
        }
    }

    return pointNum;
}

// 函数主要打开startPaths.ply文件，将里面的初始路径数据读入
void readStartPaths()
{
    string fileName = pathFolder + "/startPaths.ply";

    FILE* filePtr = fopen(fileName.c_str(), "r");
    if (filePtr == NULL) {
        printf("\nCannot read input files, exit.\n\n");
        exit(1);
    }

    int pointNum = readPlyHeader(filePtr);

    pcl::PointXYZ point;
    int val1, val2, val3, val4, groupID;
    for (int i = 0; i < pointNum; i++) {
        val1 = fscanf(filePtr, "%f", &point.x);
        val2 = fscanf(filePtr, "%f", &point.y);
        val3 = fscanf(filePtr, "%f", &point.z);
        val4 = fscanf(filePtr, "%d", &groupID);

        if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1) {
            printf("\nError reading input files, exit.\n\n");
            exit(1);
        }

        if (groupID >= 0 && groupID < groupNum) {
            startPaths[groupID]->push_back(point);
        }
    }

    fclose(filePtr);
}

// 函数主要打开Paths.ply文件，将里面的路径数据读入
#if PLOTPATHSET == 1
void readPaths()
{
    string fileName = pathFolder + "/paths.ply";

    FILE* filePtr = fopen(fileName.c_str(), "r");
    if (filePtr == NULL) {
        printf("\nCannot read input files, exit.\n\n");
        exit(1);
    }

    int pointNum = readPlyHeader(filePtr);

    pcl::PointXYZI point;
    int pointSkipNum = 30;
    int pointSkipCount = 0;
    int val1, val2, val3, val4, val5, pathID;
    for (int i = 0; i < pointNum; i++) {
        val1 = fscanf(filePtr, "%f", &point.x);
        val2 = fscanf(filePtr, "%f", &point.y);
        val3 = fscanf(filePtr, "%f", &point.z);
        val4 = fscanf(filePtr, "%d", &pathID);
        val5 = fscanf(filePtr, "%f", &point.intensity);

        if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
            printf("\nError reading input files, exit.\n\n");
            exit(1);
        }

        if (pathID >= 0 && pathID < pathNum) {
            pointSkipCount++;
            if (pointSkipCount > pointSkipNum) {
                paths[pathID]->push_back(point);
                pointSkipCount = 0;
            }
        }
    }

    fclose(filePtr);
}
#endif

// 函数主要打开pathList.ply文件，将里面的开始路径数据读入
void readPathList()
{
    string fileName = pathFolder + "/pathList.ply";

    FILE* filePtr = fopen(fileName.c_str(), "r");
    if (filePtr == NULL) {
        printf("\nCannot read input files, exit.\n\n");
        exit(1);
    }

    if (pathNum != readPlyHeader(filePtr)) {
        printf("\nIncorrect path number, exit.\n\n");
        exit(1);
    }

    int val1, val2, val3, val4, val5, pathID, groupID;
    float endX, endY, endZ;
    for (int i = 0; i < pathNum; i++) {
        val1 = fscanf(filePtr, "%f", &endX);
        val2 = fscanf(filePtr, "%f", &endY);
        val3 = fscanf(filePtr, "%f", &endZ);
        val4 = fscanf(filePtr, "%d", &pathID);
        val5 = fscanf(filePtr, "%d", &groupID);

        if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
            printf("\nError reading input files, exit.\n\n");
            exit(1);
        }

        if (pathID >= 0 && pathID < pathNum && groupID >= 0 && groupID < groupNum) {
            pathList[pathID] = groupID;
            endDirPathList[pathID] = 2.0 * atan2(endY, endX) * 180 / PI;
        }
    }

    fclose(filePtr);
}

// 函数主要打开correspondences.txt文件，将里面的开始路径数据读入
void readCorrespondences()
{
    string fileName = pathFolder + "/correspondences.txt";

    FILE* filePtr = fopen(fileName.c_str(), "r");
    if (filePtr == NULL) {
        printf("\nCannot read input files, exit.\n\n");
        exit(1);
    }

    int val1, gridVoxelID, pathID;
    for (int i = 0; i < gridVoxelNum; i++) {
        val1 = fscanf(filePtr, "%d", &gridVoxelID);
        if (val1 != 1) {
            printf("\nError reading input files, exit.\n\n");
            exit(1);
        }

        while (1) {
            val1 = fscanf(filePtr, "%d", &pathID);
            if (val1 != 1) {
                printf("\nError reading input files, exit.\n\n");
                exit(1);
            }

            if (pathID != -1) {
                if (gridVoxelID >= 0 && gridVoxelID < gridVoxelNum && pathID >= 0 && pathID < pathNum) {
                    correspondences[gridVoxelID].push_back(pathID);
                }
            } else {
                break;
            }
        }
    }

    fclose(filePtr);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "localPlanner");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");

    nhPrivate.getParam("pathFolder", pathFolder);
    nhPrivate.getParam("vehicleLength", vehicleLength);
    nhPrivate.getParam("vehicleWidth", vehicleWidth);
    nhPrivate.getParam("sensorOffsetX", sensorOffsetX);
    nhPrivate.getParam("sensorOffsetY", sensorOffsetY);
    nhPrivate.getParam("twoWayDrive", twoWayDrive);
    nhPrivate.getParam("laserVoxelSize", laserVoxelSize);
    nhPrivate.getParam("terrainVoxelSize", terrainVoxelSize);
    nhPrivate.getParam("useTerrainAnalysis", useTerrainAnalysis);
    nhPrivate.getParam("checkObstacle", checkObstacle);
    nhPrivate.getParam("checkRotObstacle", checkRotObstacle);
    nhPrivate.getParam("adjacentRange", adjacentRange);
    nhPrivate.getParam("obstacleHeightThre", obstacleHeightThre);
    nhPrivate.getParam("groundHeightThre", groundHeightThre);
    nhPrivate.getParam("costHeightThre", costHeightThre);
    nhPrivate.getParam("costScore", costScore);
    nhPrivate.getParam("useCost", useCost);
    nhPrivate.getParam("pointPerPathThre", pointPerPathThre);
    nhPrivate.getParam("minRelZ", minRelZ);
    nhPrivate.getParam("maxRelZ", maxRelZ);
    nhPrivate.getParam("maxSpeed", maxSpeed);
    nhPrivate.getParam("dirWeight", dirWeight);
    nhPrivate.getParam("dirThre", dirThre);
    nhPrivate.getParam("dirToVehicle", dirToVehicle);
    nhPrivate.getParam("pathScale", pathScale);
    nhPrivate.getParam("minPathScale", minPathScale);
    nhPrivate.getParam("pathScaleStep", pathScaleStep);
    nhPrivate.getParam("pathScaleBySpeed", pathScaleBySpeed);
    nhPrivate.getParam("minPathRange", minPathRange);
    nhPrivate.getParam("pathRangeStep", pathRangeStep);
    nhPrivate.getParam("pathRangeBySpeed", pathRangeBySpeed);
    nhPrivate.getParam("pathCropByGoal", pathCropByGoal);
    nhPrivate.getParam("autonomyMode", autonomyMode);
    nhPrivate.getParam("autonomySpeed", autonomySpeed);
    nhPrivate.getParam("joyToSpeedDelay", joyToSpeedDelay);
    nhPrivate.getParam("joyToCheckObstacleDelay", joyToCheckObstacleDelay);
    nhPrivate.getParam("goalClearRange", goalClearRange);
    nhPrivate.getParam("goalX", goalX);
    nhPrivate.getParam("goalY", goalY);

    ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("/state_estimation", 5, odometryHandler);

    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/registered_scan", 5, laserCloudHandler);

    ros::Subscriber subTerrainCloud = nh.subscribe<sensor_msgs::PointCloud2>("/terrain_map", 5, terrainCloudHandler);

    ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy>("/joy", 5, joystickHandler);

    ros::Subscriber subGoal = nh.subscribe<geometry_msgs::PointStamped>("/way_point", 5, goalHandler);

    ros::Subscriber subSpeed = nh.subscribe<std_msgs::Float32>("/speed", 5, speedHandler);

    ros::Subscriber subBoundary = nh.subscribe<geometry_msgs::PolygonStamped>("/navigation_boundary", 5, boundaryHandler);

    ros::Subscriber subAddedObstacles = nh.subscribe<sensor_msgs::PointCloud2>("/added_obstacles", 5, addedObstaclesHandler);

    ros::Subscriber subCheckObstacle = nh.subscribe<std_msgs::Bool>("/check_obstacle", 5, checkObstacleHandler);

    ros::Publisher pubPath = nh.advertise<nav_msgs::Path>("/path", 5);
    nav_msgs::Path path;

#if PLOTPATHSET == 1
    ros::Publisher pubFreePaths = nh.advertise<sensor_msgs::PointCloud2>("/free_paths", 2);
#endif

    // ros::Publisher pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2> ("/stacked_scans", 2);

    printf("\nReading path files.\n");

    if (autonomyMode) {
        joySpeed = autonomySpeed / maxSpeed;

        if (joySpeed < 0)
            joySpeed = 0;
        else if (joySpeed > 1.0)
            joySpeed = 1.0;
    }

    for (int i = 0; i < laserCloudStackNum; i++) {
        laserCloudStack[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    }
    for (int i = 0; i < groupNum; i++) {
        startPaths[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
    }
#if PLOTPATHSET == 1
    for (int i = 0; i < pathNum; i++) {
        paths[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    }
#endif
    for (int i = 0; i < gridVoxelNum; i++) {
        correspondences[i].resize(0);
    }

    laserDwzFilter.setLeafSize(laserVoxelSize, laserVoxelSize, laserVoxelSize);
    terrainDwzFilter.setLeafSize(terrainVoxelSize, terrainVoxelSize, terrainVoxelSize);

    readStartPaths();
#if PLOTPATHSET == 1
    readPaths();
#endif
    readPathList();
    readCorrespondences();

    printf("\nInitialization complete.\n\n");

    ros::Rate rate(100);
    bool status = ros::ok();
    while (status) {
        ros::spinOnce();

        if (newLaserCloud || newTerrainCloud) {
            // 首先主要进行的重置障碍物信息
            if (newLaserCloud) {
                newLaserCloud = false;

                laserCloudStack[laserCloudCount]->clear();
                *laserCloudStack[laserCloudCount] = *laserCloudDwz;
                laserCloudCount = (laserCloudCount + 1) % laserCloudStackNum;

                plannerCloud->clear(); // 可能是返回的点云信息是有障碍物的点云信息
                for (int i = 0; i < laserCloudStackNum; i++) {
                    *plannerCloud += *laserCloudStack[i];
                }
            }

            if (newTerrainCloud) {
                newTerrainCloud = false;

                plannerCloud->clear();
                *plannerCloud = *terrainCloudDwz;
            }

            float sinVehicleRoll = sin(vehicleRoll);
            float cosVehicleRoll = cos(vehicleRoll);
            float sinVehiclePitch = sin(vehiclePitch);
            float cosVehiclePitch = cos(vehiclePitch);
            float sinVehicleYaw = sin(vehicleYaw);
            float cosVehicleYaw = cos(vehicleYaw);

            // 将点云信息的坐标系为车辆坐标系, 这是因为在仿真中，点云信息输出的坐标系是全局坐标系。
            // 对于实际的车辆而言，点云数据进行标定之后就将点云坐标系直接转变为车辆坐标系，所以这部分内容需要删改 pcl::PointXYZI point;
            pcl::PointXYZI point;
            plannerCloudCrop->clear(); // 用于规划的点云信息
            int plannerCloudSize = plannerCloud->points.size(); // 设置点云的大小
            for (int i = 0; i < plannerCloudSize; i++) {
                float pointX1 = plannerCloud->points[i].x - vehicleX;
                float pointY1 = plannerCloud->points[i].y - vehicleY;
                float pointZ1 = plannerCloud->points[i].z - vehicleZ;

                point.x = pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
                point.y = -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
                point.z = pointZ1;
                point.intensity = plannerCloud->points[i].intensity;

                float dis = sqrt(point.x * point.x + point.y * point.y);
                if (dis < adjacentRange && ((point.z > minRelZ && point.z < maxRelZ) || useTerrainAnalysis)) {
                    plannerCloudCrop->push_back(point); // 修改了点云的坐标系，改为了车辆坐标系
                }
            }

            // 将计算得到的边界信息转变为车辆坐标系下
            int boundaryCloudSize = boundaryCloud->points.size();
            for (int i = 0; i < boundaryCloudSize; i++) {
                point.x = ((boundaryCloud->points[i].x - vehicleX) * cosVehicleYaw
                    + (boundaryCloud->points[i].y - vehicleY) * sinVehicleYaw);
                point.y = (-(boundaryCloud->points[i].x - vehicleX) * sinVehicleYaw
                    + (boundaryCloud->points[i].y - vehicleY) * cosVehicleYaw);
                point.z = boundaryCloud->points[i].z;
                point.intensity = boundaryCloud->points[i].intensity;

                float dis = sqrt(point.x * point.x + point.y * point.y);
                if (dis < adjacentRange) {
                    plannerCloudCrop->push_back(point);
                }
            }

            // 同时将障碍物信息的坐标系转变为车辆坐标系
            int addedObstaclesSize = addedObstacles->points.size();
            for (int i = 0; i < addedObstaclesSize; i++) {
                point.x = ((addedObstacles->points[i].x - vehicleX) * cosVehicleYaw
                    + (addedObstacles->points[i].y - vehicleY) * sinVehicleYaw);
                point.y = (-(addedObstacles->points[i].x - vehicleX) * sinVehicleYaw
                    + (addedObstacles->points[i].y - vehicleY) * cosVehicleYaw);
                point.z = addedObstacles->points[i].z;
                point.intensity = addedObstacles->points[i].intensity;

                float dis = sqrt(point.x * point.x + point.y * point.y);
                if (dis < adjacentRange) {
                    plannerCloudCrop->push_back(point);
                }
            }

            // 设置路径的相关参数，这里对于预先生成的路径参数进行了缩放，从而满足了不同速度下的路径参数
            float pathRange = adjacentRange; // 设置了点云探索的边界值
            if (pathRangeBySpeed)
                pathRange = adjacentRange * joySpeed;
            if (pathRange < minPathRange)
                pathRange = minPathRange;
            float relativeGoalDis = adjacentRange; // 将点云探索的边界值赋予相对的目标距离

            // 将目标点转换为车辆坐标系
            if (autonomyMode) {
                float relativeGoalX = ((goalX - vehicleX) * cosVehicleYaw + (goalY - vehicleY) * sinVehicleYaw);
                float relativeGoalY = (-(goalX - vehicleX) * sinVehicleYaw + (goalY - vehicleY) * cosVehicleYaw);

                relativeGoalDis = sqrt(relativeGoalX * relativeGoalX + relativeGoalY * relativeGoalY);
                joyDir = atan2(relativeGoalY, relativeGoalX) * 180 / PI;

                if (!twoWayDrive) {
                    if (joyDir > 90.0)
                        joyDir = 90.0;
                    else if (joyDir < -90.0)
                        joyDir = -90.0;
                }
            }

            // 设置参考路径，主要配置一下车辆的半径、路径的缩放、高度差等
            bool pathFound = false;
            float defPathScale = pathScale;
            if (pathScaleBySpeed)
                pathScale = defPathScale * joySpeed;
            if (pathScale < minPathScale)
                pathScale = minPathScale;

            while (pathScale >= minPathScale && pathRange >= minPathRange) {
                for (int i = 0; i < 36 * pathNum; i++) {
                    clearPathList[i] = 0;
                    pathPenaltyList[i] = 0;
                }
                for (int i = 0; i < 36 * groupNum; i++) {
                    clearPathPerGroupScore[i] = 0;
                }

                float minObsAngCW = -180.0;
                float minObsAngCCW = 180.0;
                float diameter = sqrt(vehicleLength / 2.0 * vehicleLength / 2.0 + vehicleWidth / 2.0 * vehicleWidth / 2.0);
                float angOffset = atan2(vehicleWidth, vehicleLength) * 180.0 / PI; // 车辆的夹角
                int plannerCloudCropSize = plannerCloudCrop->points.size();
                for (int i = 0; i < plannerCloudCropSize; i++) {
                    float x = plannerCloudCrop->points[i].x / pathScale; // 将雷达的探测距离缩短，v越大，探测效果越小
                    float y = plannerCloudCrop->points[i].y / pathScale; // 将雷达的探测距离缩短，v越大，探测效果越小
                    float h = plannerCloudCrop->points[i].intensity; // 高度差
                    float dis = sqrt(x * x + y * y);

                    // 判断条件
                    // 1.点云距离小于探测缩放后的距离
                    // 2.点云距离小于（相对目标距离+目标距离）/缩放比例　 || 　!路径剪短
                    // 3.进行障碍物检测
                    if (dis < pathRange / pathScale && (dis <= (relativeGoalDis + goalClearRange) / pathScale || !pathCropByGoal) && checkObstacle) {
                        // 程序中将360度分割成36份，每份10度。所以开启了36次循环
                        for (int rotDir = 0; rotDir < 36; rotDir++) {
                            float rotAng = (10.0 * rotDir - 180.0) * PI / 180; // 10度角一区分，进行360度环扫
                            float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0)); // 计算了当前角度与环视角度的差值
                            if (angDiff > 180.0) {
                                angDiff = 360.0 - angDiff; // 差值取正
                            }
                            // 接下来排除一些不需要的角度
                            // dirThre = 90 ,表示与当前方向的垂直方向
                            // dirToVehicle  车辆前进方向
                            // 排除了三种情况下的路径，排除了三种情况下的状态
                            // 1.与目标连线的方向夹角大于转向角度的阈值　且　目标点不在前进方向的方向上的所有路径:
                            // 2.当前角度大于车辆转向角度阈值　且　目标点与车辆连线夹角绝对值小于90度　且　车辆向前运动的左右侧的路径;
                            // 3.当前角度大于车辆转向角度的阈值　且　目标点与车辆连线夹角绝对值小于90度　且　车辆朝向为方向的左右侧的路径。
                            if ((angDiff > dirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) || ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 && dirToVehicle)) {
                                continue;
                            }

                            float x2 = cos(rotAng) * x + sin(rotAng) * y;
                            float y2 = -sin(rotAng) * x + cos(rotAng) * y;

                            // 因为在matlab中的体素网格生成时，Ｙ轴是一条斜线，所以ｙ轴需要有一个缩放的比例，这个比例与ｘ轴的位置相关，是一个线性关系
                            float scaleY = x2 / gridVoxelOffsetX + searchRadius / gridVoxelOffsetY * (gridVoxelOffsetX - x2) / gridVoxelOffsetX;

                            int indX = int((gridVoxelOffsetX + gridVoxelSize / 2 - x2) / gridVoxelSize);
                            int indY = int((gridVoxelOffsetY + gridVoxelSize / 2 - y2 / scaleY) / gridVoxelSize);
                            if (indX >= 0 && indX < gridVoxelNumX && indY >= 0 && indY < gridVoxelNumY) {
                                int ind = gridVoxelNumY * indX + indY; // 得到索引序号
                                int blockedPathByVoxelNum = correspondences[ind].size(); // 当前序号的体素网格,占据了多少条路径
                                for (int j = 0; j < blockedPathByVoxelNum; j++) {
                                    // if(地面分割高度 > 障碍物高度门槛 || ！使用地面分割)
                                    if (h > obstacleHeightThre || !useTerrainAnalysis) {
                                        clearPathList[pathNum * rotDir + correspondences[ind][j]]++;
                                    } else {
                                        // 在使用了地面分割且激光点分割后高度小于障碍物高度阈值obstacleHeightThre时
                                        // 并且 当前点云高度大于原有地面分割高度值,且大于地面高度阈值groundHeightThre
                                        if (pathPenaltyList[pathNum * rotDir + correspondences[ind][j]] < h && h > groundHeightThre) {
                                            pathPenaltyList[pathNum * rotDir + correspondences[ind][j]] = h;
                                        }
                                    }
                                }
                            }
                        }
                    }

                    // 由目标点在车辆转向阈值之外的侧面，所以车辆需要进行原地转向，从而实现车辆向目标点位置转向。
                    // CW即顺时针旋转（Clock Wise）的方向 与CW反方向旋转时为CCW （Counter Clock Wise）
                    if (dis < diameter / pathScale && (fabs(x) > vehicleLength / pathScale / 2.0 || fabs(y) > vehicleWidth / pathScale / 2.0) && (h > obstacleHeightThre || !useTerrainAnalysis) && checkRotObstacle) {
                        float angObs = atan2(y, x) * 180.0 / PI;
                        if (angObs > 0) {
                            if (minObsAngCCW > angObs - angOffset)
                                minObsAngCCW = angObs - angOffset;
                            if (minObsAngCW < angObs + angOffset - 180.0)
                                minObsAngCW = angObs + angOffset - 180.0;
                        } else {
                            if (minObsAngCW < angObs + angOffset)
                                minObsAngCW = angObs + angOffset;
                            if (minObsAngCCW > 180.0 + angObs - angOffset)
                                minObsAngCCW = 180.0 + angObs - angOffset;
                        }
                    }
                }

                if (minObsAngCW > 0)
                    minObsAngCW = 0;
                if (minObsAngCCW < 0)
                    minObsAngCCW = 0;

                for (int i = 0; i < 36 * pathNum; i++) {
                    // 完成所有路径的得分计算
                    int rotDir = int(i / pathNum);
                    float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
                    if (angDiff > 180.0) {
                        angDiff = 360.0 - angDiff;
                    }
                    if ((angDiff > dirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) || ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 && dirToVehicle)) {
                        continue;
                    }

                    if (clearPathList[i] < pointPerPathThre) {
                        float penaltyScore = 1.0 - pathPenaltyList[i] / costHeightThre;
                        if (penaltyScore < costScore)
                            penaltyScore = costScore;

                        float dirDiff = fabs(joyDir - endDirPathList[i % pathNum] - (10.0 * rotDir - 180.0));
                        if (dirDiff > 360.0) {
                            dirDiff -= 360.0;
                        }
                        if (dirDiff > 180.0) {
                            dirDiff = 360.0 - dirDiff;
                        }

                        float rotDirW;
                        if (rotDir < 18)
                            rotDirW = fabs(fabs(rotDir - 9) + 1);
                        else
                            rotDirW = fabs(fabs(rotDir - 27) + 1);
                        float score = (1 - sqrt(sqrt(dirWeight * dirDiff))) * rotDirW * rotDirW * rotDirW * rotDirW * penaltyScore;
                        if (score > 0) {
                            clearPathPerGroupScore[groupNum * rotDir + pathList[i % pathNum]] += score;
                        }
                    }
                }

                // 再选出最优路径后，会选择startPaths组成一条最终的路径。
                float maxScore = 0;
                int selectedGroupID = -1;
                for (int i = 0; i < 36 * groupNum; i++) {
                    int rotDir = int(i / groupNum);
                    float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
                    float rotDeg = 10.0 * rotDir;
                    if (rotDeg > 180.0)
                        rotDeg -= 360.0;
                    if (maxScore < clearPathPerGroupScore[i] && ((rotAng * 180.0 / PI > minObsAngCW && rotAng * 180.0 / PI < minObsAngCCW) || (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && twoWayDrive) || !checkRotObstacle)) {
                        maxScore = clearPathPerGroupScore[i]; // 设置最大的得分情况
                        selectedGroupID = i; // 选出了最大得分的那一组
                    }
                }

                // 因为startPaths表示第一次采样的路径，而且还需要保证该路径小于设定的pathRange。
                // 然后将freePaths会输出clearPathList[i]<pointPerPathThre的路径
                if (selectedGroupID >= 0) {
                    int rotDir = int(selectedGroupID / groupNum);
                    float rotAng = (10.0 * rotDir - 180.0) * PI / 180;

                    selectedGroupID = selectedGroupID % groupNum;
                    int selectedPathLength = startPaths[selectedGroupID]->points.size();
                    path.poses.resize(selectedPathLength);
                    for (int i = 0; i < selectedPathLength; i++) {
                        float x = startPaths[selectedGroupID]->points[i].x;
                        float y = startPaths[selectedGroupID]->points[i].y;
                        float z = startPaths[selectedGroupID]->points[i].z;
                        float dis = sqrt(x * x + y * y);

                        if (dis <= pathRange / pathScale && dis <= relativeGoalDis / pathScale) {
                            path.poses[i].pose.position.x = pathScale * (cos(rotAng) * x - sin(rotAng) * y);
                            path.poses[i].pose.position.y = pathScale * (sin(rotAng) * x + cos(rotAng) * y);
                            path.poses[i].pose.position.z = pathScale * z;
                        } else {
                            path.poses.resize(i);
                            break;
                        }
                    }

                    path.header.stamp = ros::Time().fromSec(odomTime);
                    path.header.frame_id = "vehicle";
                    pubPath.publish(path);

#if PLOTPATHSET == 1
                    freePaths->clear();
                    for (int i = 0; i < 36 * pathNum; i++) {
                        int rotDir = int(i / pathNum);
                        float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
                        float rotDeg = 10.0 * rotDir;
                        if (rotDeg > 180.0)
                            rotDeg -= 360.0;
                        float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
                        if (angDiff > 180.0) {
                            angDiff = 360.0 - angDiff;
                        }
                        if ((angDiff > dirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) || ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 && dirToVehicle) || !((rotAng * 180.0 / PI > minObsAngCW && rotAng * 180.0 / PI < minObsAngCCW) || (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && twoWayDrive) || !checkRotObstacle)) {
                            continue;
                        }

                        if (clearPathList[i] < pointPerPathThre) {
                            int freePathLength = paths[i % pathNum]->points.size();
                            for (int j = 0; j < freePathLength; j++) {
                                point = paths[i % pathNum]->points[j];

                                float x = point.x;
                                float y = point.y;
                                float z = point.z;

                                float dis = sqrt(x * x + y * y);
                                if (dis <= pathRange / pathScale && (dis <= (relativeGoalDis + goalClearRange) / pathScale || !pathCropByGoal)) {
                                    point.x = pathScale * (cos(rotAng) * x - sin(rotAng) * y);
                                    point.y = pathScale * (sin(rotAng) * x + cos(rotAng) * y);
                                    point.z = pathScale * z;
                                    point.intensity = 1.0;

                                    freePaths->push_back(point);
                                }
                            }
                        }
                    }

                    sensor_msgs::PointCloud2 freePaths2;
                    pcl::toROSMsg(*freePaths, freePaths2);
                    freePaths2.header.stamp = ros::Time().fromSec(odomTime);
                    freePaths2.header.frame_id = "vehicle";
                    pubFreePaths.publish(freePaths2);
#endif
                }

                if (selectedGroupID < 0) {
                    if (pathScale >= minPathScale + pathScaleStep) {
                        pathScale -= pathScaleStep;
                        pathRange = adjacentRange * pathScale / defPathScale;
                    } else {
                        pathRange -= pathRangeStep;
                    }
                } else {
                    pathFound = true;
                    break;
                }
            }
            pathScale = defPathScale;

            if (!pathFound) {
                path.poses.resize(1);
                path.poses[0].pose.position.x = 0;
                path.poses[0].pose.position.y = 0;
                path.poses[0].pose.position.z = 0;

                path.header.stamp = ros::Time().fromSec(odomTime);
                path.header.frame_id = "vehicle";
                pubPath.publish(path);

#if PLOTPATHSET == 1
                freePaths->clear();
                sensor_msgs::PointCloud2 freePaths2;
                pcl::toROSMsg(*freePaths, freePaths2);
                freePaths2.header.stamp = ros::Time().fromSec(odomTime);
                freePaths2.header.frame_id = "vehicle";
                pubFreePaths.publish(freePaths2);
#endif
            }

            /*sensor_msgs::PointCloud2 plannerCloud2;
            pcl::toROSMsg(*plannerCloudCrop, plannerCloud2);
            plannerCloud2.header.stamp = ros::Time().fromSec(odomTime);
            plannerCloud2.header.frame_id = "vehicle";
            pubLaserCloud.publish(plannerCloud2);*/
        }

        status = ros::ok();
        rate.sleep();
    }

    return 0;
}
