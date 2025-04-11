 
#ifndef CTRLCOMPONENTS_H
#define CTRLCOMPONENTS_H

#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "interface/IOInterface.h"
#include "interface/CmdPanel.h"
// #include "common/unitreeRobot.h"
#include "common/hexpodRobot.h"
#include "Gait/WaveGenerator.h"
#include "control/Estimator.h"
#include "control/BalanceCtrl.h"
// #include "control/ConvexMpc.h"
#include <string>
#include <iostream>

#include"common/protection.h"

#ifdef COMPILE_DEBUG
#include "common/PyPlot.h"
#endif  // COMPILE_DEBUG
 
struct CtrlComponents{
public:
    /*CtrlComponents(IOInterface *ioInter): ioInter(ioInter)：构造函数，
    通过传入 IOInterface 类型的指针 ioInter 来初始化该类的成员变量。ioInter 负责与外部接口（如仿真或硬件接口）进行通信*/
    CtrlComponents(IOInterface *ioInter):ioInter(ioInter){
        
        lowCmd = new LowlevelCmd();
        lowState = new LowlevelState();
        contact = new VecInt4;
        phase = new Vec4;
        contact_hex = new VecInt6;
        phase_hex = new Vec6;
        Apla = new Vec3; // lcc
        *contact = VecInt4(0, 0, 0, 0);
        *phase = Vec4(0.5, 0.5, 0.5, 0.5);
        // contact_hex(new VecInt6(0, 0, 0, 0, 0, 0));//lcc注释掉的
        // *contact_hex = VecInt6(0, 0, 0, 0, 0, 0);
        // *phase_hex = Vec6(0.5, 0.5, 0.5, 0.5, 0.5, 0.5);
        
    }
    ~CtrlComponents(){
        delete lowCmd;
        delete lowState;
        delete ioInter;
        // delete robotModel;
        delete sixlegdogModel; //wjw （2025.2.26）加上吗？编译也没报错
        delete waveGen;
        delete estimator;
        delete balCtrl;
        #ifdef COMPILE_DEBUG
                delete plot;
        #endif  // COMPILE_DEBUG
    }
    LowlevelCmd *lowCmd;                                            //存储发送到底层电机的控制指令（位置、速度、力矩、PID增益）
    LowlevelState *lowState;                                        //接收底层电机的实时反馈数据（关节角度、速度、力矩等）
    IOInterface *ioInter;                                           //硬件抽象层接口，用于与真实机器人或仿真器通信（如发送指令、接收状态）
    // QuadrupedRobot *robotModel;
    HexapodRobot *sixlegdogModel;                                   //lcc 20240608 ////六足机器人运动学与动力学模型，用于逆运动学计算和状态估计
    WaveGenerator *waveGen;                                         //步态生成器，计算各腿的摆动/支撑相位（phase_hex）和接触状态（contact_hex）
    Estimator *estimator;                                           //状态估计器，融合传感器数据（IMU、关节编码器）计算机器人位姿、速度等
    BalanceCtrl *balCtrl;                                           //平衡控制器，通过调整足端力或关节力矩维持机器人动态平衡
    // ConvexMpc *convMpc; //lcc

    //terrian estimator. by lcc
    Vec3 *Apla;

    #ifdef COMPILE_DEBUG
        // if (plot != nullptr) delete plot; wjw（2025.2.26）deepseek建议
        PyPlot *plot;
    #endif  // COMPILE_DEBUG

    /* contact = 1 -> stand; contact = 0 -> swing */
    VecInt4 *contact;
    Vec4 *phase;
    Vec6 *phase_hex;                                                //六维向量，表示各腿的步态相位（0~1周期）
    VecInt6 *contact_hex;                                           //六维向量，标记各腿是否接触地面（1-支撑，0-摆动）
    DataUnusualProtect _dataUnuProtect[6]; //lcc 20240807:保护程序   //数据异常保护模块，检测关节超限、通信超时等异常状态

    double dt;                                                      //控制周期（如0.002秒，对应500Hz控制频率）
    bool *running;                                                  //全局运行标志，控制主循环启停（如急停时设为false）
    CtrlPlatform ctrlPlatform;      //平台：仿真还是真实

    void sendRecv(){//发送控制指令（lowCmd）并接收传感器数据（lowState）
        ioInter->sendRecv(lowCmd, lowState);
    }

    void runWaveGen(){
        // waveGen->calcContactPhase(*phase, *contact, _waveStatus);
        waveGen->calcContactPhase(*phase_hex, *contact_hex, _waveStatus);//根据当前步态状态（_waveStatus）更新各腿的相位和接触状态
    }

    //步态状态设置：
    void setAllStance(){
        _waveStatus = WaveStatus::STANCE_ALL;// 所有腿支撑
    }

    void setAllSwing(){
        _waveStatus = WaveStatus::SWING_ALL;// 所有腿摆动
    }

    void setStartWave(){
        _waveStatus = WaveStatus::WAVE_ALL;// 启动波浪步态
    }
    //

/*定义了一个 geneObj 函数，目的是 初始化 机器人的一些关键控制模块，如估计器（Estimator）、平衡控制器（BalanceCtrl）以及可能的调试功能（如 PyPlot）*/
    void geneObj(){
        //estimator = new Estimator(robotModel, lowState, contact, phase, dt);
        estimator = new Estimator(/*robotModel,*/ sixlegdogModel, lowState, contact_hex, phase_hex, dt);//创建了一个 Estimator 对象，并将其指针赋值给 estimator 状态估计器：推测机器人当前的质心位置、速度、姿态等状态
        //balCtrl = new BalanceCtrl(robotModel);
        balCtrl = new BalanceCtrl(sixlegdogModel);//创建了一个 BalanceCtrl 对象，并将其指针赋值给 balCtrl。BalanceCtrl 是机器人的平衡控制器，用于确保机器人在行走或运动过程中保持平衡
        
        //这部分代码是条件编译代码块，仅在定义了 COMPILE_DEBUG 宏时才会执行。它用于调试和可视化
        #ifdef COMPILE_DEBUG
                plot = new PyPlot();
                // balCtrl->setPyPlot(plot);//将 plot（即 PyPlot 实例）传递给 BalanceCtrl，允许平衡控制器将其数据可视化
                // estimator->setPyPlot(plot);//将 plot 传递给 Estimator，使得状态估计器的输出也可以通过图形进行可视化
        #endif  // COMPILE_DEBUG
    }

private:
    WaveStatus _waveStatus = WaveStatus::SWING_ALL;

};

#endif  // CTRLCOMPONENTS_H