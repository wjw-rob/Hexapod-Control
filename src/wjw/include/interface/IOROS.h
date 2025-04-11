 
// #ifdef COMPILE_WITH_ROS

#ifndef IOROS_H
#define IOROS_H

#include "ros/ros.h"
#include "interface/IOInterface.h"
#include "std_msgs/Float64.h"

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>//用于处理摇杆控制的消息类型
#include <sensor_msgs/Imu.h>//IMU（惯性测量单元）的传感器消息类型
#include <sensor_msgs/JointState.h>//用于表示机器人的关节状态（如角度、速度等）
#include <nav_msgs/Odometry.h>//里程计消息类型，用于表示机器人在环境中的位置信息
#include "std_msgs/Float64.h"
//geometry_msgs 系列：处理机器人位置、速度、力矩、姿态等信息的标准 ROS 消息类型
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PointStamped.h>

// #include "interface/GazeboSim.h"
// #include "unitree_legged_msgs/LowCmd.h"
// #include "unitree_legged_msgs/LowState.h"
// #include "unitree_legged_msgs/MotorCmd.h"
// #include "unitree_legged_msgs/MotorState.h"
// #include <sensor_msgs/Imu.h>
#include <string>
// #include "common/unitreeRobot.h"//换成#include "common/hexpodRobot.h"不行吗？ （换了编译也没报错）
#include "common/hexpodRobot.h"
#include"common/protection.h"//电机保护程序

// extern 关键字，它们在此文件中被声明为外部变量，意味着这些变量在其他地方（IOROS.cpp）已经定义过，并且在当前文件中可以引用
extern Vec3 ODE_P;
extern Vec3 ODE_V;
extern Vec36 sub_joint_p_local_temp_origin;//可能代表的是 关节位置、局部坐标系下的关节位置，并且是一个暂存的 原始数据（由 temp 和 origin 关键词暗示）
// class IOROS : public IOInterface, public GazeboSim{
class IOROS : public IOInterface{
public:
IOROS();
~IOROS();
void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);

Eigen::Matrix<double, 3, 1> retSimOdeBodyP();//获取机器人的位置 （ODE_P）

Eigen::Matrix<double, 3, 1> retSimOdeBodyV();//获取机器人的速度 （ODE_V）

private:
void sendCmd(const LowlevelCmd *cmd, LowlevelState *state);//sendCmd() 负责发送控制命令 与sendRecv（）区别？
void recvState(LowlevelState *state);//recvState() 负责接收机器人的状态信息

unsigned int wait_count;//可能用于控制等待次数，确保数据的同步或避免数据丢失

DataUnusualProtect ___dataUnuProtect[7]; //lcc 20240807:保护程序

//  0~12: RF LF RB LB 
//  0~18: RF LF RM LM RB LB 
ros::Publisher pub_joint_cmd[NUM_DOF_W];//18    pub_joint_cmd 是一个发布器数组，用于发布机器人的关节命令
std_msgs::Float64 pub_joint_msg[NUM_DOF_W];//pub_joint_msg 是一个消息数组，用于存储各关节的命令消息（类型为 Float64）
//订阅
ros::Subscriber sub_joint_states;//接收订阅机器人的关节状态
ros::Subscriber sub_imu_msg;//接收订阅 IMU 数据（惯性测量单元的数据）
ros::Subscriber sub_nav_odometry;//接收订阅机器人的里程计信息

// GazeboSim gs;

// ros::NodeHandle _nm;
// ros::Subscriber _servo_sub[12], _imu_sub;
// ros::Publisher _servo_pub[12];
// unitree_legged_msgs::LowCmd _lowCmd;
// unitree_legged_msgs::LowState _lowState;
// std::string _robot_name;

//repeated functions for multi-thread
ros::NodeHandle nh;//ROS节点的句柄（ros::NodeHandle nh），用于管理ROS节点的订阅和发布

//这两个函数用于初始化接收和发送数据的功能。initSend() 和 initRecv()的具体实现应负责设置 ROS 的发布器和订阅器
void initSend();
void initRecv();

Vec4 quaternion_offset;//四元数偏移量，可能用于调整机器人的姿态，确保机器人在实际应用中的旋转是正确的

};

#endif  // IOROS_H

// #endif  // COMPILE_WITH_ROS