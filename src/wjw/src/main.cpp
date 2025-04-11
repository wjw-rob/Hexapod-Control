/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
// 包含必要的库文件，用于控制和信号处理
#include <iostream>                        
#include <unistd.h>                     // 提供对 POSIX 操作系统 API 的访问
#include <csignal>                      // 提供信号处理功能，如处理 Ctrl+C 终止信号
#include <sched.h>                      // 提供调度功能，如设置实时进程调度策略

#include <iomanip>     //改改改了了了(2025.2.18)    //解决编译器无法识别 std::setprecision 问题

#include "control/ControlFrame.h"       
#include "control/CtrlComponents.h"
#include "Gait/WaveGenerator.h"         //引入步态生成器头文件，用于生成机器人步态模式


#include "interface/KeyBoard.h"         //引入键盘接口头文件，允许通过键盘控制机器人的行为
#include "interface/IOROS.h"

#include <thread>                       // 
#include <chrono>                       //

#include "ros/ros.h"                    //
#include "std_msgs/Float64MultiArray.h" //引入 ROS 消息类型 Float64MultiArray，用于传输 一组浮动数值
#include "std_msgs/Float64.h"           //引入 ROS 消息类型 Float64，用于传输 单个浮动数值


#include "interface/IOInterface.h"  // 改改改了了了(2025.2.18) 相对路径，基于 include 文件夹



#if USE_A_REAL_HEXAPOD == false

class RosTopicMsgPub
{
protected:
    ros::Publisher msgPub;                                  //定义一个 ROS 发布者对象，用于向指定话题发布消息
    std_msgs::Float64 msgTemp;                              //定义一个临时消息变量，用于存储单个 Float64 类型的消息
    std_msgs::Float64MultiArray msgTempArray;               //定义一个临时消息变量，用于存储 Float64MultiArray 类型的消息
    ros::NodeHandle node;                                   //定义一个 ROS 节点句柄，管理节点与 ROS 网络的通信
public:
    RosTopicMsgPub(std::string topicName){                  //构造函数，初始化发布者对象 msgPub，并指定话题名称
        msgPub=node.advertise<std_msgs::Float64MultiArray>(topicName,1000);
        // std::cout<<topicName<<std::endl;
    }

    void msgPubRun(double * msg_array){                     //该函数用于将 msg_array 数组的数据发布到 ROS 话题，最多发布三个数值
        msgTempArray.data = {msg_array[0], msg_array[1], msg_array[2]};
        msgPub.publish(msgTempArray);
    }

    void msgPubRun(Eigen::Matrix<double,3,1> msg_matrix){   //该函数用于将 3x1 的 Eigen::Matrix 矩阵发布到 ROS 话题
        msgTempArray.data = {msg_matrix(0), msg_matrix(1), msg_matrix(2)};
        msgPub.publish(msgTempArray);
    }

    void msgPubRun(Eigen::Matrix<double,4,1> msg_matrix){   //该函数用于将 4x1 的 Eigen::Matrix 矩阵发布到 ROS 话题
        msgTempArray.data = {msg_matrix(0), msg_matrix(1), msg_matrix(2), msg_matrix(3)};
        msgPub.publish(msgTempArray); 
    }

    void msgPubRun(Eigen::Matrix<double,6,1> msg_matrix) {  //该函数用于将 6x1 的 Eigen::Matrix 矩阵发布到 ROS 话题
        msgTempArray.data = {msg_matrix(0), msg_matrix(1), msg_matrix(2), msg_matrix(3), msg_matrix(4), msg_matrix(5)};
        msgPub.publish(msgTempArray);
    }
    
    void msgPubRun(Eigen::Matrix<double,18,1> msg_matrix) { //该函数用于将 18x1 的 Eigen::Matrix 矩阵发布到 ROS 话题
        msgTempArray.data={
            msg_matrix(0),msg_matrix(1),msg_matrix(2),msg_matrix(3),msg_matrix(4),msg_matrix(5),
            msg_matrix(6),msg_matrix(7),msg_matrix(8),msg_matrix(9),msg_matrix(10),msg_matrix(11),
            msg_matrix(12),msg_matrix(13),msg_matrix(14),msg_matrix(15),msg_matrix(16),msg_matrix(17),
            };
        msgPub.publish(msgTempArray);
    }
};
#endif




bool running = true;                    // 全局变量，控制主循环的运行状态

// over watch the ctrl+c command 捕获 Ctrl+C 终止信号的处理函数
// 当捕捉到信号（例如 Ctrl+C 发送的 SIGINT 信号）时，调用该函数 
void ShutDown(int sig)
{
    // 输出信息，告知用户控制程序将停止
    std::cout << "stop the controller" << std::endl;

    // 设置全局变量 running 为 false，以终止主循环
    // 主循环的运行状态依赖于 running 变量的值
    running = false;
    exit(0);
}

// 设置进程调度策略为实时调度
void setProcessScheduler()
{
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1)
    {
        std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }
}

//主函数
int main(int argc, char **argv)
{
    /*  设置为实时进程  */ //如果线程启动失败，那么可以启用超级权限,需要以 root 用户或者通过 sudo 运行程序
    setProcessScheduler();
    /* 设置输出格式  */ //std::setprecision(4) 会设置输出时显示的总有效数字位数（包含小数点前后的数字），并且会影响小数点后的数字。如果同时使用了 std::fixed，它会指定小数点后显示的数字位数。
    std::cout << std::fixed << std::setprecision(4);


/*#ifdef RUN_ROS：

    #ifdef 是一个条件编译指令，表示“如果定义了宏 RUN_ROS，则执行后续代码块”。
    如果代码编译时定义了 RUN_ROS 这个宏，接下来的代码（直到 #endif）会被包含在编译过程中。如果没有定义 RUN_ROS，这些代码会被忽略。

ros::init(argc, argv, "unitree_gazebo_servo");：

    ros::init 是 ROS 中用于初始化一个 ROS 节点的函数。在这段代码中，如果 RUN_ROS 被定义了，就会初始化一个名为 "unitree_gazebo_servo" 的 ROS 节点。
    该函数的参数是：
        argc 和 argv：命令行参数，通常会传入 main 函数的参数。
        "unitree_gazebo_servo"：ROS 节点的名称。

#endif：

    #endif 标志着 #ifdef 条件编译块的结束。如果 RUN_ROS 被定义了，#endif 后面的代码才会继续编译。*/
    // #ifdef RUN_ROS
    ros::init(argc, argv, "wjw1_node");//改改改了了了（2025.2.18）
    // #endif 

    IOInterface *ioInter;
    CtrlPlatform ctrlPlat;                          //创建CtrlPlatform类下变量名ctrlPlat，用于判断是仿真还是真实平台
    
    //动态绑定，实现仿真与真实接口之间的快速转换
    #if USE_A_REAL_HEXAPOD == true
        // ioInter = new IOSDK();                      //创建一个 IOROK 类的实例，并将其指针赋值给 ioInter，用于控制仿真中的机器人。
        // ctrlPlat = CtrlPlatform::REALROBOT;         //ctrlPlat 设置为 REALROBOT，表示控制平台是实际的机器人。
    #else
        ioInter = new IOROS();                      //创建一个 IOROS 类的实例，并将其指针赋值给 ioInter，用于控制仿真中的机器人。
        ctrlPlat = CtrlPlatform::GAZEBO;            //ctrlPlat 设置为 GAZEBO，表示控制平台是 Gazebo 仿真环境。
    #endif 
    //

    //创建 CtrlComponents 类的实例，并进行必要的初始化 同时设置 running 标志来控制主循环
    CtrlComponents *ctrlComp = new CtrlComponents(ioInter);
    ctrlComp->ctrlPlatform = ctrlPlat;
    ctrlComp->dt = 0.002; // run at 500hz           // 设置控制周期 ctrlComp 用于处理机器人的控制逻辑，dt 表示控制周期（例如 500Hz）
    // ctrlComp->dt = 0.00225; // run 
    // ctrlComp->dt = 0.0025; // lcc
    // ctrlComp->dt = 0.003; // lcc
    ctrlComp->running = &running;

    // #if IS_THIS_A_HEXAPOD
        ctrlComp->sixlegdogModel = new SixLegDogRobot();  //创建 SixLegDogRobot 类的实例并赋值给 ctrlComp->sixlegdogModel
    // #else
        // ctrlComp->robotModel = new A1Robot();
        // ctrlComp->robotModel = new Go1Robot();
    // #endif

    Vec6 _bias;//定义了一个 6 维 列向量（6行1列）  _bias 为WaveGenerator里的相位偏移?
    _bias << 0, 0.5, 0.5, 0, 0, 0.5;//使用 Eigen 库的流式操作符 << 来初始化其元素

    #if USE_A_REAL_HEXAPOD == false
    // ctrlComp->waveGen = new WaveGenerator(0.45, 0.5, _bias); // Trot
    ctrlComp->waveGen = new WaveGenerator(0.55, 0.5, _bias);    // Trot 通过 WaveGenerator 初始化参数（周期、占空比、相位偏移）
    // ctrlComp->waveGen = new WaveGenerator(0.6, 0.5, _bias); // Trot
    // ctrlComp->waveGen = new WaveGenerator(0.8, 0.5, _bias); // Trot
    #else
    // ctrlComp->waveGen = new WaveGenerator(1, 0.5, _bias); // Trotss
    ctrlComp->waveGen = new WaveGenerator(0.75, 0.5, _bias); // Trot
    #endif

    ctrlComp->geneObj();//调用 ctrlComp（即 CtrlComponents 类的实例）中的 geneObj() 函数，初始化与机器人控制相关的各个模块
    ControlFrame ctrlFrame(ctrlComp);//ControlFrame 类通常是用于封装整个机器人的控制框架，负责调度和管理机器人的各个控制模块；ctrlComp 是控制组件的实例，包含了机器人控制所需的各个部分，如控制算法、步态生成器、状态估计器等
    signal(SIGINT, ShutDown);

    #if USE_A_REAL_HEXAPOD == false
        //ROS通信:当在仿真平台运行时，通过 ROS 发布各种消息 通过 RosTopicMsgPub 类将数据（例如位置、速度、RPY角度）发布到 ROS 话题，以便其他节点进行订阅和处理
        RosTopicMsgPub COM("COM");//位置
        RosTopicMsgPub VEL("VEL");//速度
        RosTopicMsgPub RPY("RPY");//RPY角度
        while (running){   

            ctrlFrame.run();

        }
    #else
        std::atomic<bool> control_execute{};
        control_execute.store(true, std::memory_order_release);

        std::thread spi_can_run([&]() {
            Vec36 init_motor_set_q;
            Vec36 dou_dong_angle;
            bool get_motor_response_flag = false;
            long long startTime;
            while (control_execute.load(std::memory_order_acquire)  && running) {

               
            }
        });

        //IMU数据接收：imu_recv 线程负责接收和处理 IMU 数据
        std::thread imu_recv([&]() {
            long long startTime;
            while (control_execute.load(std::memory_order_acquire)  && running) {
                // startTime = getSystemTime();
                imu_run();
                absoluteWait(startTime, (long long)(ctrlComp->dt * 1000000));
            }
        });

        //主控制线程：main_thread 线程负责运行主控制循环
        std::thread main_thread([&]() {
            while (control_execute.load(std::memory_order_acquire) && running) {
                ctrlFrame.run();
            }
        });

        //数据保存线程：save_data2txt 线程定期保存数据到文本文件，用于记录日志和调试
        std::thread save_data2txt([&]() {

            #if TXT_FLAGE
            ofstream CoT_motor_t;
            ofstream RPY_act_des_slopDegree;
            ofstream CoG_PosActDes_VelActDes;
            ofstream foot_hip_z;
            ofstream foot_forceZ_forceSum;
            CoT_motor_t.open("./CoT_motor_t.txt", ios::out | ios::trunc);
            RPY_act_des_slopDegree.open("./RPY_act_des_slopDegree.txt", ios::out | ios::trunc);
            CoG_PosActDes_VelActDes.open("./CoG_PosActDes_VelActDes.txt", ios::out | ios::trunc);
            foot_hip_z.open("./foot_hip_z.txt", ios::out | ios::trunc);
            foot_forceZ_forceSum.open("./foot_forceZ_forceSum.txt", ios::out | ios::trunc);
            #endif

            while (control_execute.load(std::memory_order_acquire) && running) {
            usleep(10000);
            #if TXT_FLAGE
            if( DTAT_SAVE2TXT == true ){


                if ( ! CoT_motor_t) { cout << "CoT_motor_t 文件不能打开" <<endl; }
                    CoT_motor_t <<  
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[0].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[1].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[2].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[3].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[4].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[5].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[6].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[7].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[8].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[9].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[10].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[11].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[12].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[13].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[14].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[15].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[16].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[17].tauEst << "\n";

                Vec3 RPY;    
                RPY = rotMatToRPY(ctrlFrame._FSMController->_ctrlComp->lowState->getRotMat());
                if ( ! RPY_act_des_slopDegree) { cout << "RPY_act_des_slopDegree 文件不能打开" <<endl; }
                    RPY_act_des_slopDegree <<  
                    RPY(0) <<" "<< 
                    RPY(1) <<" "<< 
                    RPY(2) <<" "<< 
                    RPY_DES(0) <<" "<< 
                    RPY_DES(1) <<" "<< 
                    RPY_DES(2) <<" "<< 
                    TERRIAN_EST_DEGREE(0) <<" "<< 
                    TERRIAN_EST_DEGREE(1) << "\n";

                Vec3 vel, pos;    
                pos = ctrlFrame._FSMController->_ctrlComp->estimator->getPosition();
                vel = ctrlFrame._FSMController->_ctrlComp->estimator->getVelocity();
                if ( ! CoG_PosActDes_VelActDes) { cout << "CoG_PosActDes_VelActDes 文件不能打开" <<endl; }
                    CoG_PosActDes_VelActDes <<  
                    pos(0) <<" "<< 
                    pos(1) <<" "<< 
                    pos(2) <<" "<< 
                    POS_WORLD_DES(0) <<" "<< 
                    POS_WORLD_DES(1) <<" "<< 
                    POS_WORLD_DES(2) <<" "<< 
                    vel(0) <<" "<< 
                    vel(1) <<" "<< 
                    vel(2) <<" "<< 
                    VEL_WORLD_DES(0) <<" "<< 
                    VEL_WORLD_DES(1) <<" "<< 
                    VEL_WORLD_DES(2) << "\n";

                Vec36 foot_hip_pos;
                foot_hip_pos = ctrlFrame._FSMController->_ctrlComp->sixlegdogModel->getFeet2BPositions(
                    *(ctrlFrame._FSMController->_ctrlComp->lowState),
                    FrameType::BODY );
                if ( ! foot_hip_z) { cout << "foot_hip_z 文件不能打开" <<endl; }
                    foot_hip_z <<  
                    foot_hip_pos(2) <<" "<< 
                    foot_hip_pos(5) <<" "<< 
                    foot_hip_pos(8) <<" "<< 
                    foot_hip_pos(11) <<" "<< 
                    foot_hip_pos(14) <<" "<< 
                    foot_hip_pos(17) << "\n";
                    
                Vec36 _footTipForceEst;
                _footTipForceEst = ctrlFrame._FSMController->_ctrlComp->sixlegdogModel->calcForceByTauEst( 
                    vec36ToVec18(
                        ctrlFrame._FSMController->_ctrlComp->lowState->getQ_Hex()), 
                        ctrlFrame._FSMController->_ctrlComp->lowState->getTau_Hex()
                        );

                Vec6 forceSum;
                forceSum(0) = sqrt(pow(_footTipForceEst(0), 2) + pow(_footTipForceEst(1), 2) + pow(_footTipForceEst(2), 2) ); 
                forceSum(1) = sqrt(pow(_footTipForceEst(3), 2) + pow(_footTipForceEst(4), 2) + pow(_footTipForceEst(5), 2) ); 
                forceSum(2) = sqrt(pow(_footTipForceEst(6), 2) + pow(_footTipForceEst(7), 2) + pow(_footTipForceEst(8), 2) ); 
                forceSum(3) = sqrt(pow(_footTipForceEst(9), 2) + pow(_footTipForceEst(10), 2) + pow(_footTipForceEst(11), 2) ); 
                forceSum(4) = sqrt(pow(_footTipForceEst(12), 2) + pow(_footTipForceEst(13), 2) + pow(_footTipForceEst(14), 2) ); 
                forceSum(5) = sqrt(pow(_footTipForceEst(15), 2) + pow(_footTipForceEst(16), 2) + pow(_footTipForceEst(17), 2) ); 

                if ( ! foot_forceZ_forceSum) { cout << "foot_forceZ_forceSum 文件不能打开" <<endl; }
                    foot_forceZ_forceSum <<  
                    _footTipForceEst(2) <<" "<< 
                    _footTipForceEst(5) <<" "<< 
                    _footTipForceEst(8) <<" "<< 
                    _footTipForceEst(11) <<" "<< 
                    _footTipForceEst(14) <<" "<< 
                    _footTipForceEst(17) <<" "<< 
                    forceSum(0) <<" "<< 
                    forceSum(1) <<" "<< 
                    forceSum(2) <<" "<< 
                    forceSum(3) <<" "<< 
                    forceSum(4) <<" "<< 
                    forceSum(5) << "\n";
            }
            #endif
            }
        });

        // spi_can_run.join();
        imu_recv.join();
        main_thread.join();
        #if TXT_FLAGE
        save_data2txt.join();
        #endif
    #endif

    return 0;

}
