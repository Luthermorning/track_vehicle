#include "std_msgs/Float64.h"
#include <boost/thread/thread.hpp> //多线程处理头文件
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <ros/ros.h>
#include <sys/poll.h>
#include <termios.h> //终端处理头文件

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64

#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57

double base_link_lenth = 2.881;

class track_vehicle_teleop {
private:
    // 按shift后速度会明显提升
    struct VelFast {
        double x_fast;
        double yaw_fast;
    };
    VelFast vel_fast;

    struct VelSimple {
        double x_simple;
        double yaw_simple;
    };
    VelSimple vel_simple;

    geometry_msgs::Twist cmdvel_twist;
    std_msgs::Float64 left_vel;
    std_msgs::Float64 right_vel;
    ros::NodeHandle nh;
    // ros::Publisher pub;
    ros::Publisher pub_lf;
    ros::Publisher pub_lb;

public:
    // 利用构造函数对参数服务器and参数进行一次初始化
    track_vehicle_teleop()
    {
        // pub = nh.advertise<geometry_msgs::Twist>("track_vehicle_cmd_vel", 10);
        pub_lf = nh.advertise<std_msgs::Float64>("/track_vehicle/left_crawler_sprocket_velocity_controller/command", 1);
        pub_lb = nh.advertise<std_msgs::Float64>("/track_vehicle/right_crawler_sprocket_velocity_controller/command", 1);

        ros::NodeHandle nh_private("~");
        nh_private.param("x_fast", vel_fast.x_fast, 10.0);
        nh_private.param("yaw_fast", vel_fast.yaw_fast, 10.0);
        nh_private.param("x_simple", vel_simple.x_simple, 5.0);
        nh_private.param("yaw_simple", vel_simple.yaw_simple, 5.0);
    }
    ~track_vehicle_teleop() { }

    void KeyBoardUse();

    void StopExcavator();
};

struct termios cooked, raw; // 配置linux终端的两个结构体 cooked用来表示经过处理后的 raw用来保存原始的
void track_vehicle_teleop::KeyBoardUse()
{
    tcgetattr(STDIN_FILENO, &cooked); // 获取终端键盘输入的数据
    memcpy(&raw, &cooked, sizeof(struct termios));

    // ICANON 表示规范模式，即在输入中使用行缓冲，而不是一个字符一个字符地读取。
    // ECHO 表示回显模式，即输入的字符将被显示在终端上。
    // 将这两个标志位清除（设为0）后，关闭了规范模式和回显模式，使得输入字符不再等待回车键确认，且输入字符不会被显示在终端上。
    raw.c_lflag &= ~(ICANON | ECHO);
    // c_cc 表示控制字符数组
    raw.c_cc[VEOL] = 1; // VEOL 是一个控制字符，表示行结束符。在这里，将 VEOL 设置为1，通常表示Ctrl+A。
    raw.c_cc[VEOF] = 2; // VEOF 是一个控制字符，表示文件结束符。在这里，将 VEOF 设置为2，通常表示Ctrl+B。
    tcsetattr(STDIN_FILENO, TCSANOW, &raw); // TCSANOW：是一个标志，表示立即（now）应用设置。这意味着终端的设置将立即生效。

    puts("---用键盘进行控制---");
    puts("---使用 WASD 按键控制底盘---");
    puts("---同时按下Shift键可以进行加速---");
    puts("---咳咳 轻易别用Shift---");

    // 异步检查键盘是否有数据键入
    struct pollfd ufd;
    ufd.fd = STDIN_FILENO;
    ufd.events = POLLIN;

    int flag_poll;
    char jianzhi;
    bool dirty = false;
    int turn; // 左右方向
    int speed; // 前后方向
    double max_liner = 0;
    double max_yaw = 0;
    while (1) {
        boost::this_thread::interruption_point(); // 检测是否有中断进来

        flag_poll = poll(&ufd, 1, 200);
        if (flag_poll < 0) {
            perror("poll():");
            return;
        } else if (flag_poll > 0) {
            if (read(STDIN_FILENO, &jianzhi, 1) < 0) {
                perror("read():");
                return;
            }
        } else {
            if (dirty == true) {
                StopExcavator();
                dirty = false;
            }
            continue;
        }
        // ROS_INFO("%d\n", jianzhi);

        switch (jianzhi) {
        case KEYCODE_W:
            max_liner = vel_simple.x_simple;
            speed = -1;
            turn = 0;
            dirty = true;
            break;

        case KEYCODE_S:
            max_liner = vel_simple.x_simple;
            speed = 1;
            turn = 0;
            dirty = true;
            break;

        case KEYCODE_A:
            max_yaw = vel_simple.yaw_simple;
            speed = 0;
            turn = 1;
            dirty = true;
            break;

        case KEYCODE_D:
            max_yaw = vel_simple.yaw_simple;
            speed = 0;
            turn = -1;
            dirty = true;
            break;

        case KEYCODE_W_CAP:
            max_liner = vel_fast.x_fast;
            speed = -1;
            turn = 0;
            dirty = true;
            break;

        case KEYCODE_S_CAP:
            max_liner = vel_fast.x_fast;
            speed = 1;
            turn = 0;
            dirty = true;
            break;

        case KEYCODE_A_CAP:
            max_yaw = vel_fast.yaw_fast;
            speed = 0;
            turn = 1;
            dirty = true;
            break;

        case KEYCODE_D_CAP:
            max_yaw = vel_fast.yaw_fast;
            speed = 0;
            turn = -1;
            dirty = true;
            break;

        default:
            max_liner = 0;
            max_yaw = 0;
            speed = 0;
            turn = 0;
            dirty = false;
            break;
        }
        cmdvel_twist.linear.x = speed * max_liner;
        cmdvel_twist.angular.z = turn * max_yaw;
        // ROS_INFO("%.2f and %.2f\n", cmdvel_twist.linear.x, cmdvel_twist.angular.z);
        // pub.publish(cmdvel_twist);

        left_vel.data = cmdvel_twist.linear.x - (cmdvel_twist.angular.z / 2) * base_link_lenth;
        right_vel.data = cmdvel_twist.linear.x + (cmdvel_twist.angular.z / 2) * base_link_lenth;
        pub_lf.publish(left_vel);
        pub_lb.publish(right_vel);
    }
}

void track_vehicle_teleop::StopExcavator()
{
    cmdvel_twist.linear.x = 0;
    cmdvel_twist.angular.z = 0;
    // pub.publish(cmdvel_twist);
    left_vel.data = cmdvel_twist.linear.x - (cmdvel_twist.angular.z / 2) * base_link_lenth;
    right_vel.data = cmdvel_twist.linear.x + (cmdvel_twist.angular.z / 2) * base_link_lenth;
    pub_lf.publish(left_vel);
    pub_lb.publish(right_vel);
}

int main(int argc, char* argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "track_vehicle_teleop", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    track_vehicle_teleop et;

    boost::thread t = boost::thread(boost::bind(&track_vehicle_teleop::KeyBoardUse, &et));

    // 保证可以一直键盘输入不退出
    ros::spin();

    // 保证按^c时线程安全退出
    t.interrupt(); // 中断线程 t，发出线程中断的请求。这是为了确保在程序结束时，线程可以安全地退出。
    t.join(); // 等待线程 t 完成。这确保在主线程结束之前，子线程 t 已经完成或被中断。
    et.StopExcavator();

    // 将终端设置还原为之前保存的值，以确保在程序结束时终端状态不会被修改。这个操作通常是在程序结束时执行的，以确保将终端还原到原始状态。
    tcsetattr(STDIN_FILENO, TCSANOW, &cooked);

    return 0;
}