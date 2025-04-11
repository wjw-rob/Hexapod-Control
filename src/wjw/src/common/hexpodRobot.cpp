#include "common/hexpodRobot.h"
#include <iostream>
// #include"common/unitreeLeg.h"(wjw:不需要包含unitreeLeg吗？这里计算用的函数是在unitreeLeg里定义的)

Vec3 HexapodRobot::getX(LowlevelState &state){//获取六足机器人中第 0 条腿的足端在机体坐标系（FrameType::BODY）下的位置
    return getFootPosition(state, 0, FrameType::BODY);
}

Vec36 HexapodRobot::getVecXP(LowlevelState &state){
    Vec3 x = getX(state);
    Vec36 vecXP, qLegs;
    qLegs = state.getQ_Hex();//调用 state 对象的 getQ_Hex 方法，获取六足机器人所有腿的关节角度信息，并存储在 qLegs 变量中

    for(int i(0); i < 6; ++i){//遍历机器人的 6 条腿 对于每条腿，调用 _Legs[i]->calcPEe2B(qLegs.col(i)) 方法，根据该腿的关节角度计算其足端在机体坐标系下的位置
        //用计算得到的足端位置减去第 0 条腿的足端位置 x，得到该腿足端相对于第 0 条腿足端的相对位置，并将结果存储在 vecXP 向量的对应列中
        vecXP.col(i) = _Legs[i]->calcPEe2B(qLegs.col(i)) - x;
    }
    return vecXP;//用在机器人站立上
}

// Inverse Kinematics 逆运动学
Vec18 HexapodRobot::getQ(const Vec36 &vecP, FrameType frame){
    Vec18 q;
    // for(int i(0); i < 6; ++i){
    //     q.segment(3*i, 3) = _Legs[i]->calcQ(vecP.col(i), frame);
    // }
    for(int i(0); i < 6; ++i){
        
        q.segment(3*i, 3) = _Legs[i]->invKinematic(i, vecP.col(i), frame);
    }

    #if USE_A_REAL_HEXAPOD == true  //对部分关节角度取负，这可能是为了适应实际机器人硬件的安装方式或者运动方向的定义
        q(1) = -q(1);
        q(2) = -q(2);
        q(4) = -q(4);
        q(5) = -q(5);
        q(7) = -q(7);
        q(8) = -q(8);
        q(10) = -q(10);
        q(11) = -q(11);
        q(13) = -q(13);
        q(14) = -q(14);
        q(16) = -q(16);
        q(17) = -q(17);
    #endif

    return q;
}

Vec18 HexapodRobot::getQd(const Vec36 &pos, const Vec36 &vel, FrameType frame){
    Vec18 qd;
    for(int i(0); i < 6; ++i){
        qd.segment(3*i, 3) = _Legs[i]->calcQd(pos.col(i), vel.col(i), frame);
    }
    return qd;
}

Vec18 HexapodRobot::getTau(const Vec18 &q, const Vec36 feetForce){
    Vec18 tau;
    for(int i(0); i < 6; ++i){
        tau.segment(3*i, 3) = _Legs[i]->calcTau(q.segment(3*i, 3), feetForce.col(i));
    }

    #if USE_A_REAL_HEXAPOD == true
        // tau(1) = -tau(1);
        // tau(2) = -tau(2);
        // tau(4) = -tau(4);
        // tau(5) = -tau(5);
        // tau(7) = -tau(7);
        // tau(8) = -tau(8);
        // tau(10) = -tau(10);
        // tau(11) = -tau(11);
        // tau(13) = -tau(13);
        // tau(14) = -tau(14);
        // tau(16) = -tau(16);
        // tau(17) = -tau(17);
    #endif

    return tau;
}

Vec36 HexapodRobot::calcForceByTauEst(const Vec18 &q, const Vec36 feetForce)//lcc
{
    Vec36 f;
    for(int i(0); i < 6; ++i){
        f.block<3, 1>(0, i) = ( _Legs[i]->calcJaco( q.segment(3*i, 3) ).transpose() ).inverse() * feetForce.block<3, 1>(0, i);
    }
    return f;
}

// Forward Kinematics 正运动学
Vec3 HexapodRobot::getFootPosition(LowlevelState &state, int id, FrameType frame){
    Vec36 qLegs= state.getQ_Hex();//获取六足机器人所有腿部的关节角度信息，并存储在 qLegs

    if(frame == FrameType::BODY){  //FrameType里三个在哪里确定？
        return _Legs[id]->calcPEe2B(qLegs.col(id));//calc P Ee 2B 足端在机身系下位置
    }else if(frame == FrameType::HIP){
        return _Legs[id]->calcPEe2H(qLegs.col(id));//calc P Ee 2H 足端在髋关节系下位置
    }else{
        std::cout << "[ERROR] The frame of function: getFootPosition can only be BODY or HIP." << std::endl;
        exit(-1);
    }
}

// Forward derivative Kinematics 正向运动学微分 通过已知的关节角度和关节角速度来求解足端的速度
Vec3 HexapodRobot::getFootVelocity(LowlevelState &state, int id){
    Vec36 qLegs = state.getQ_Hex();
    Vec36 qdLegs= state.getQd_Hex();
    // std::cout << "qLegs:\n" <<qLegs<< std::endl;
    // std::cout << "qdLegs:\n" <<qdLegs<< std::endl;
    return _Legs[id]->calcVEe(qLegs.col(id), qdLegs.col(id));//calc V Ee 足端速度
}

// Forward Kinematics 获得所有足端位置
Vec36 HexapodRobot::getFeet2BPositions(LowlevelState &state, FrameType frame){
    Vec36 feetPos;
    if(frame == FrameType::GLOBAL){
        for(int i(0); i<6; ++i){
            feetPos.col(i) = getFootPosition(state, i, FrameType::BODY);
        }
        feetPos = state.getRotMat() * feetPos;
    }
    else if((frame == FrameType::BODY) || (frame == FrameType::HIP)){
        for(int i(0); i<6; ++i){
            feetPos.col(i) = getFootPosition(state, i, frame);
        }
    }
    else{
        std::cout << "[ERROR] Frame error of function getFeet2BPositions" << std::endl;
        exit(-1);
    }
    return feetPos;
}

//获得所有足端速度
Vec36 HexapodRobot::getFeet2BVelocities(LowlevelState &state, FrameType frame){
    Vec36 feetVel;
    for(int i(0); i<6; ++i){
        feetVel.col(i) = getFootVelocity(state, i);
    }

    if(frame == FrameType::GLOBAL){
        // #if USE_A_REAL_HEXAPOD == true
        return state.getRotMat() * feetVel;
        // #else
        // Vec36 feetPos = getFeet2BPositions(state, FrameType::BODY);
        // feetVel += skew(state.getGyro()) * feetPos;
        // return state.getRotMat() * feetVel;
        // #endif
    }
    else if((frame == FrameType::BODY) || (frame == FrameType::HIP)){
        return feetVel;
    }
    else{
        std::cout << "[ERROR] Frame error of function getFeet2BVelocities" << std::endl;
        exit(-1);
    }   
}

//雅可比
Mat3 HexapodRobot::getJaco(LowlevelState &state, int legID){
    return _Legs[legID]->calcJaco(state.getQ_Hex().col(legID));
}

SixLegDogRobot::SixLegDogRobot(){

    #if USE_A_REAL_HEXAPOD == true
    //传入腿部编号和髋关节相对于机身中心的偏移向量 pHip2B
    _Legs[0] = new SixLegDogLeg(0, Vec3( 0.155, -0.075, 0));//rf  //这里输入的是 pHip2B 20240524
    _Legs[1] = new SixLegDogLeg(1, Vec3( 0.155,  0.075, 0));//lf    机身中心到髋关节坐标系{0}的向量（偏移量）（wjw添）
    _Legs[2] = new SixLegDogLeg(2, Vec3(0.0, -0.075, 0));//rm
    _Legs[3] = new SixLegDogLeg(3, Vec3(0.0,  0.075, 0));//lm
    _Legs[4] = new SixLegDogLeg(4, Vec3(-0.155, -0.075, 0));//rb
    _Legs[5] = new SixLegDogLeg(5, Vec3(-0.155,  0.075, 0));//lb
    //lcc 实际上，这个理想位置就是：body系下 x和y值就是(足端+小腿+大腿)平面,即LX和LY+L1。z方向的值还不知道怎么确定
    //hip下的投影
    // _feetPosNormalStand <<  0.155 + 0.076,  0.155 + 0.076, 0.0, 0.0, -0.155 - 0.076, -0.155 - 0.076, 
    //                        -0.075 - 0.12,  0.075 + 0.12, -0.075 - 0.12,  0.075 + 0.12, -0.075 - 0.12,  0.075 + 0.12,
    //                        -0.21, -0.21, -0.21, -0.21, -0.21, -0.21;
    //腿部正常站立足端位置设置？
    _feetPosNormalStand <<  0.155 + 0.076,  0.155 + 0.076, 0.0, 0.0, -0.155 - 0.076, -0.155 - 0.076, //x
                           -0.075 - 0.14,  0.075 + 0.14, -0.075 - 0.16,  0.075 + 0.16, -0.075 - 0.14,  0.075 + 0.14,//y
                           -0.22, -0.22, -0.22, -0.22, -0.22, -0.22;//z

    // _feetPosNormalStand <<  0.155 + 0.076,  0.155 + 0.076, 0.0, 0.0, -0.155 - 0.076, -0.155 - 0.076, 
    //                        -0.075 - 0.13,  0.075 + 0.13, -0.075 - 0.13,  0.075 + 0.13, -0.075 - 0.13,  0.075 + 0.13,
    //                        -0.24, -0.24, -0.24, -0.24, -0.24, -0.24;

    // _feetPosNormalStand <<  0.155 + 0.076,  0.155 + 0.076, 0.0, 0.0, -0.155 - 0.076, -0.155 - 0.076, 
    //                        -0.075 - 0.13,  0.075 + 0.13, -0.075 - 0.13,  0.075 + 0.13, -0.075 - 0.13,  0.075 + 0.13,
    //                        -0.27, -0.27, -0.27, -0.27, -0.27, -0.27;
    //腿部蹲伏足端位置设置？
    _feetPosNormalSquat <<  0.155 + 0.076,  0.155 + 0.076, 0.0, 0.0, -0.155 - 0.076, -0.155 - 0.076, 
                           -0.075 - 0.34,  0.075 + 0.34, -0.075 - 0.34,  0.075 + 0.34, -0.075 - 0.34,  0.075 + 0.34,
                           -0.025, -0.025, -0.025, -0.025, -0.025, -0.025;

    _robVelLimitX << -0.1, 0.1;
    _robVelLimitY << -0.2, 0.2; // 力位混合，最快 vy=0.3m/s
    _robVelLimitYaw << -0.2, 0.2;

    _mass =15;
    // _mass =20;
    _pcb << 0.0, 0.0, 0.07;

    // ixx="0.026264"
    // ixy="0"
    // ixz="0"
    // iyy="0.069583"
    // iyz="0"
    // izz="0.090006" />

    // _Ib = Vec3(0.26264, 0.69583, 0.90006).asDiagonal(); //原始的转动惯量
    _Ib = Vec3(1, 1, 1).asDiagonal();//lcc 20240611: 修改后的转的惯量 -> 我发现换上这个以后，往前走也不会沉头了，效果好了很多
    
    #else
    _Legs[0] = new SixLegDogLeg(0, Vec3( 0.14185, -0.0655, 0));//rf  //这里输入的是 pHip2B 20240524
    _Legs[1] = new SixLegDogLeg(1, Vec3( 0.14185,  0.0655, 0));//lf
    _Legs[2] = new SixLegDogLeg(2, Vec3(0.0, -0.0655, 0));//rm
    _Legs[3] = new SixLegDogLeg(3, Vec3(0.0,  0.0655, 0));//lm
    _Legs[4] = new SixLegDogLeg(4, Vec3(-0.14185, -0.0655, 0));//rb
    _Legs[5] = new SixLegDogLeg(5, Vec3(-0.14185,  0.0655, 0));//lb
    //lcc 实际上，这个理想位置就是：body系下 x和y值就是(足端+小腿+大腿)平面,即LX和LY+L1。z方向的值还不知道怎么确定 （wjw：z方向修改怎么没变化？）
    //hip下的投影
    // _feetPosNormalStand <<  0.14185 + 0.07725,  0.14185 + 0.07725, 0.0, 0.0, -0.14185 - 0.07725, -0.14185 - 0.07725, 
    //                        -0.0655 - 0.12,  0.0655 + 0.12, -0.0655 - 0.12,  0.0655 + 0.12, -0.0655 - 0.12,  0.0655 + 0.12,
    //                        -0.20, -0.20, -0.20, -0.20, -0.20, -0.20;

    // _feetPosNormalStand <<  0.14185 + 0.07725,  0.14185 + 0.07725, 0.0, 0.0, -0.14185 - 0.07725, -0.14185 - 0.07725, 
    //                        -0.0655 - 0.13,  0.0655 + 0.13, -0.0655 - 0.13,  0.0655 + 0.13, -0.0655 - 0.13,  0.0655 + 0.13,
    //                        -0.24, -0.24, -0.24, -0.24, -0.24, -0.24;//设置正常站立状态下的足端位置信息（设置完后会根据足端位置求出关节角度？）


     _feetPosNormalStand <<  0.14185 + 0.07725,  0.14185 + 0.07725, 0.0, 0.0, -0.14185 - 0.07725, -0.14185 - 0.07725, 
                           -0.0655 - 0.13,  0.0655 + 0.13, -0.0655 - 0.13,  0.0655 + 0.13, -0.0655 - 0.13,  0.0655 + 0.13,
                           -0.24, -0.24, -0.24, -0.24, -0.24, -0.24;//设置正常站立状态下的足端位置信息



    _feetPosNormalSquat <<  0.14185 + 0.07725,  0.14185 + 0.07725, 0.0, 0.0, -0.14185 - 0.07725, -0.14185 - 0.07725, 
                           -0.0655 - 0.34,  0.0655 + 0.34, -0.0655 - 0.34,  0.0655 + 0.34, -0.0655 - 0.34,  0.0655 + 0.34,
                           -0.025, -0.025, -0.025, -0.025, -0.025, -0.025;//存储蹲伏状态下的足端位置信息

    _robVelLimitX << -0.1, 0.1;
    _robVelLimitY << -0.2, 0.2; // 力位混合，最快 vy=0.3m/s
    _robVelLimitYaw << -0.2, 0.2;

    // _mass =16;
    _mass =16.8;
    // _mass =20;
    _pcb << 0.0, 0.0, 0.0;

    // ixx="0.026264"
    // ixy="0"
    // ixz="0"
    // iyy="0.069583"
    // iyz="0"
    // izz="0.090006" />

    // _Ib = Vec3(0.26264, 0.69583, 0.90006).asDiagonal(); //原始的转动惯量
    _Ib = Vec3(1, 1, 1).asDiagonal();//lcc 20240611: 修改后的转的惯量 -> 我发现换上这个以后，往前走也不会沉头了，效果好了很多
    #endif
}

