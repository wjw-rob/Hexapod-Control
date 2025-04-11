 
#ifndef ENUMCLASS_H
#define ENUMCLASS_H

#include <iostream>
#include <sstream>

//lcc 20240607
#define NUM_LEG_W 6
#define LEG_DOF_W 3
#define NUM_DOF_W 18
#define IS_THIS_A_HEXAPOD true
// #define IS_THIS_A_HEXAPOD false

// #define USE_A_REAL_HEXAPOD true //真实下释放
#define USE_A_REAL_HEXAPOD false   //仿真下释放

// #define ONLY_POSITION_CTRL true
#define ONLY_POSITION_CTRL false

// #define NUM_LEG 4
// #define LEG_DOF 3
// #define NUM_DOF 12

#define TERRIANESTI_FOURLEG true
// #define TERRIANESTI_FOURLEG false

// #define PCONTROL_REFLEX_LIFE_DOWM true
#define PCONTROL_REFLEX_LIFE_DOWM false

#define TXT_FLAGE true
// #define TXT_FLAGE false

//enum class定义枚举类
enum class CtrlPlatform{//平台：仿真还是真实
    GAZEBO,
    REALROBOT,
};

enum class RobotType{
    A1,
    Go1
};

// 指的是用户能够控制的几种模式 由KeyBoard里按键实现对应的控制模式
enum class UserCommand{
    // EXIT,                            // 退出命令
    NONE,                               // 默认值，表示没有命令
    //START,      // position           // 启动命令
    // L2_A,       // fixedStand        // 固定站立
    // L2_B,       // passive           // 阻尼模式
    // L2_X,       // freeStand         // 自由站立
#ifdef COMPILE_WITH_MOVE_BASE
    L2_Y,       // move_base            // 移动基座命令（如果定义了 COMPILE_WITH_MOVE_BASE）
#endif  // COMPILE_WITH_MOVE_BASE
    BALANCE_TEST0,                      // 平衡测试命令
    SWING_TEST9,                        // 摆动测试命令
    SETP_TEST8,                         // 步态测试命令

    PASSIVE_1,                          // 1 阻尼模式 
    FIXEDSTAND_2,                       // 2 固定站立模式命令
    FREESTAND_3,                        // 3 自由站立模式命令 (不好用)
    QP_4,                               // 4 QP控制命令
    POSITION_5,                         // 5 位置控制命令
    A1MPC_6,                            // 6 MPC控制命令(不好用)             //lcc 20240416
    POSREFLEX_7,                        // 7 位置反射控制命令（不知道怎么切换）         //lcc 20240627
    SQUAT_C,                            // c 蹲下命令                //lcc 20240808
    FORCE_POS_8,                        // 8 力位混合控制命令         //lcc 20240827, 力位混合
    MPC_FORCE_POS_0                     // 0 MPC+力位混合控制命令(不好用)     //lcc 20240903, MPC+力位混合
};

//FrameType 用来表示坐标系的类型
enum class FrameType{
    BODY,       //机身坐标系
    HIP,        //髋关节坐标系（单腿）
    GLOBAL      //世界坐标系
};

//WaveStatus（步态状态）
enum class WaveStatus{
    STANCE_ALL,     // 所有腿部处于站立状态
    SWING_ALL,      // 悬挂 所有腿部处于摆动状态，通常用于步态规划中，机器人的腿在空中摆动以实现步态
    WAVE_ALL        // 所有腿部处于波动状态
};

//FSMMode有限状态机（FSM）的不同工作模式
enum class FSMMode{
    NORMAL,     // 正常模式
    CHANGE      // 切换模式
};

//枚举类 FSMStateName，表示有限状态机的不同状态
//这个类与前面的UserCommand有什么区别？
enum class FSMStateName{
    // EXIT,
    INVALID,            // 无效状态
    PASSIVE,            // 阻尼状态，机器人处于完全不动或待机状态
    FIXEDSTAND,         // 固定站立状态，机器人站立并保持固定位置
    FREESTAND,          // 自由站立状态，机器人站立但可以自由移动
    POSITION,           // 位置控制状态，机器人按目标位置进行控制
#ifdef COMPILE_WITH_MOVE_BASE
    MOVE_BASE,       // move_base状态，表示移动基座模式
#endif  // COMPILE_WITH_MOVE_BASE
    BALANCETEST,        // 平衡测试状态，机器人进行平衡测试
    SWINGTEST,          // 摆动测试状态，机器人进行摆动测试
    STEPTEST,           // 步态测试状态，机器人进行步态测试
    A1MPC,              // 机器人使用MPC控制的状态 lcc 20240416
    QP,                 //四元数规划（QP）控制的状态 lcc 20240523
    POSREFLEX,          //位置反射控制状态 lcc 20240627 
    SQUAT,              //蹲下控制状态 lcc 20240808
    FORCE_POS,          //力位混合控制状态 lcc 20240827
    MPC_FORCE_POS       //MPC + 力位混合控制状态 lcc 20240903
};

#endif  // ENUMCLASS_H