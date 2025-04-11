#ifndef CMDPANEL_H
#define CMDPANEL_H

#include <stdio.h>
#include <math.h>
#include <Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <stdio.h>
// #include "message/unitree_joystick.h"            //这个好像没有用？wjw(2025.3.3)  //可能是一个与机器人控制器（如操纵杆或遥控器）相关的头文件，用于处理输入的用户命令
#include "common/enumClass.h"
#include <pthread.h>
#include <Eigen/Dense>
#include <eigen3/Eigen/Core>
#include "control/neural_bezier_curve.h"                //用于控制曲线（如贝塞尔曲线）生成的控制文件，可能用于机器人运动规划或路径控制
// #include "interface/KeyBoard.h"

//这一段六足里好像没用？
#ifdef COMPILE_WITH_REAL_ROBOT
    #ifdef ROBOT_TYPE_A1
        #include "unitree_legged_sdk/unitree_legged_sdk.h"
    #endif  // ROBOT_TYPE_A1
    #ifdef ROBOT_TYPE_Go1
        #include "unitree_legged_sdk/unitree_legged_sdk.h"
    #endif  // ROBOT_TYPE_Go1
#endif  // COMPILE_WITH_REAL_ROBOT
//

// bool USVLCC_SETZERO;

    // int _add_count=0;
    // float _data_last,_set_last;
    // bool _conver_done_flag=true;

// UserValue指用户能够直接控制的输入变量
class UserValue{        //存储用户输入的原始控制值（如摇杆、按钮）
public:
    UserValue(){
        lx = 0;
        ly = 0;
        rx = 0;
        ry = 0;
        L2 = 0;
    }
    float lx;
    float ly;
    float rx;
    float ry;
    float L2;
    void setZero(){
        lx = 0;
        ly = 0;
        rx = 0;
        ry = 0;
        L2 = 0;
    }
};

class UserValue_lcc         //UserValue_lcc：扩展了 UserValue 类，对用户输入进行线性变换或滤波处理。增加了对输入的线性转换（linear_trans）的支持  但这还没找到在哪应用？
{
private:
    /* data */
    linear_trans lt_lx;     //linear_trans 可能用于对摇杆输入或按钮输入进行归一化、比例缩放或其他类型的转换
    linear_trans lt_ly;     // linear_trans类在 #include "control/neural_bezier_curve.h" 中
    linear_trans lt_l2;
    linear_trans lt_rx;
    linear_trans lt_ry;
public:
    float lx;
    float ly;
    float rx;
    float ry;
    float L2;
    UserValue_lcc(/* args */);
    ~UserValue_lcc();

    void setZero();

};

extern UserValue_lcc userValue_lcc;


/*
 @author:lcc
 @date: 20250601
*/
struct UserFunctionMode{
    bool function_test;
    bool state_reset;
    Eigen::Matrix<double,1,6>  LEG_LIFT_TRIGGER;//20230907cheet 腿部动作触发标志（如抬起、放下、特殊动作）
    Eigen::Matrix<double,1,6>  LEG_DOWM_TRIGGER;//20230907cheet
    Eigen::Matrix<double,1,6>  LEG_MKAN_TRIGGER;//20230907cheet
    int life_reaction_off_on=0,dowm_reaction_off_on=0,mkan_reaction_off_on=0,berzier_shape_off_on = 0;
    double set_pitch;//用户设定的俯仰角（可能用于姿态调整）

    bool motor_enable_flag; // 进入闭环
    bool motor_disenable_flag;//退出闭环

    UserFunctionMode(){
        setZero();
    }
    void setZero(){
        function_test = false;
        state_reset = false;
        LEG_LIFT_TRIGGER.setZero();
        LEG_DOWM_TRIGGER.setZero();
        LEG_MKAN_TRIGGER.setZero();
        set_pitch = 0;

        motor_enable_flag = false;
        motor_disenable_flag = false;
    }
};

/*这段代码定义了一个名为 CmdPanel 的类，表示机器人控制面板的抽象基类。
它包含了一些成员变量和方法，用于处理用户命令、用户值和用户功能模式的管理，
并提供一些默认实现。代码中还包含一些宏条件编译部分，用于根据不同平台进行适配。*/
class CmdPanel{
public:
    CmdPanel(){}
    virtual ~CmdPanel(){}
    UserCommand getUserCmd(){return userCmd;}   // 获取用户命令 返回 userCmd 类型是 UserCommand
    UserValue getUserValue(){return userValue;} // 获取用户值
    UserFunctionMode getUserFunctionMode(){return userFunctionMode;}    // 获取用户功能模式 类型是 UserFunctionMode，代表用户当前选择的功能模式
    void setPassive(){userCmd = UserCommand::PASSIVE_1;}    // 设置为被动模式，即初始状态
    void setZero(){userValue.setZero();}    // 设置为零状态

    UserFunctionMode userFunctionMode; // lcc 20250601 // 用户功能模式
    // UserValue_lcc userValue_lcc;

/*这段代码表明，如果在编译时定义了 COMPILE_WITH_REAL_ROBOT 宏，那么类 CmdPanel 会包含 receiveHandle 方法。
这个方法接受一个 LowState 类型的指针，可能用于接收和处理来自真实机器人的状态。
UNITREE_LEGGED_SDK::LowState 可能是一个包含机器人状态信息的结构体或类，它包含了机器人当前的状态信息。*/
#ifdef COMPILE_WITH_REAL_ROBOT
    virtual void receiveHandle(UNITREE_LEGGED_SDK::LowState *lowState){};
#endif  // COMPILE_WITH_REAL_ROBOT
protected:
    virtual void* run(void *arg){return NULL;}
    UserCommand userCmd;    // 用户命令
    UserValue userValue;    // 用户值
};


#endif  // CMDPANEL_H


/*CmdPanel 类是机器人控制系统中的一个核心部分，主要负责 接收用户输入 并将其  转换为机器人的控制命令、设置控制模式  等。它提供了用户命令、用户值和功能模式的接口，并且能够根据不同的平台（如仿真或真实机器人）进行适配。
UserValue 和 UserValue_lcc 类用于存储和处理用户输入的值，支持摇杆等输入设备的数据转换。
UserFunctionMode 结构体则用于存储用户当前选择的功能模式，如电机的闭环控制、腿部动作触发等。
通过条件编译，CmdPanel 类可以在真实机器人平台上接收并处理机器人的状态信息，实现仿真和实际机器人的兼容*/