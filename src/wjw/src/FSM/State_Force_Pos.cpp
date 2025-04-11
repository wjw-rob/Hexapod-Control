#include "FSM/State_Force_Pos.h"
#include "interface/KeyBoard.h"
#include <iomanip>

State_Force_Pos::State_Force_Pos(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::FORCE_POS, "force_pos"), 
              _est(ctrlComp->estimator), _phase(ctrlComp->phase), _Apla( ctrlComp->Apla),
              _contact(ctrlComp->contact), _sixlegdogModel(ctrlComp->sixlegdogModel), 
              _balCtrl(ctrlComp->balCtrl), _phase_hex(ctrlComp->phase_hex), _contact_hex(ctrlComp->contact_hex)
              {
    // _gait = new GaitGenerator(ctrlComp);
    _gait_P = new GaitGenerator_P(ctrlComp);

    #if USE_A_REAL_HEXAPOD == true
    _gaitHeight = 0.175;
    #else
    _gaitHeight = 0.1;
    #endif
    root_euler_d.setZero();

// #ifdef ROBOT_TYPE_Go1
    // // _Kpp = Vec3(70, 70, 70).asDiagonal();
    // _Kdp = Vec3(10, 10, 10).asDiagonal();
    // _kpw = 780; 
    // _Kdw = Vec3(70, 70, 70).asDiagonal();
    // _KpSwing = Vec3(400, 400, 400).asDiagonal();
    // _KdSwing = Vec3(10, 10, 10).asDiagonal();
// #endif

    //lcc tuning 20240604
    // _Kpp = Vec3(100, 30, 100).asDiagonal();
    _Kpp = Vec3(30, 30, 100).asDiagonal();
    _Kdp = Vec3(20, 20, 20).asDiagonal();
    _kpw = 1000; 
    _Kdw = Vec3(70, 70, 70).asDiagonal();

    _KpSwing = Vec3(400, 400, 400).asDiagonal();
    _KdSwing = Vec3(10, 10, 10).asDiagonal();

// #ifdef ROBOT_TYPE_A1
    // _Kpp = Vec3(20, 20, 100).asDiagonal();
    // _Kdp = Vec3(20, 20, 20).asDiagonal();
    // _kpw = 400;
    // _Kdw = Vec3(50, 50, 50).asDiagonal();
    // _KpSwing = Vec3(400, 400, 400).asDiagonal();
    // _KdSwing = Vec3(10, 10, 10).asDiagonal();
// #endif

    // _vxLim = _sixlegdogModel->getRobVelLimitX();
    // _vyLim = _sixlegdogModel->getRobVelLimitY();
    // _wyawLim = _sixlegdogModel->getRobVelLimitYaw();

    _vxLim << -0.1, 0.1;
    _vyLim << -0.275, 0.275; 
    _wyawLim << -0.15, 0.15;

    _contact_te = new VecInt4;
    _spf = new SupportFeetEndP(ctrlComp);
    _spt = new SupportTrajectory(ctrlComp);

    _rowMax = 20 * M_PI / 180;
    _rowMin = -_rowMax;
    _pitchMax = 15 * M_PI / 180;
    _pitchMin = -_pitchMax;
    _yawMax = 20 * M_PI / 180;
    _yawMin = -_yawMax;
    _heightMax = 0.18;
    _heightMin = -_heightMax + 0.05;

    adj_RPY_P.setZero();
    adj_RPY_P_past.setZero();

    _posFeet2BGoal_P_Increment.setZero();
    terian_FootHold = new Vec1_6;
    (*terian_FootHold).setZero();
}

State_Force_Pos::~State_Force_Pos(){
    // delete _gait;
    delete _gait_P;
}

void State_Force_Pos::enter(){

    // printf(" \n enter -> qp \n ");
    /* 一开始，设置期望的位置为实际位置；速度设置为0； */
    _pcd = _est->getPosition(); //一开始，将实际位置设置为目标位置。_pcd-> world系下，机身目标位置。
    // _pcd(2) = -_sixlegdogModel->getFeetPosIdeal()(2, 0);
    // body_h = -_sixlegdogModel->getFeetPosIdeal()(2, 0) + 0.0944;//lcc 20240604
    // _pcd(2) = -_sixlegdogModel->getFeetPosIdeal()(2, 0) + 0.0944;//lcc 20240604
    body_h = -_sixlegdogModel->getFeetPosIdeal()(2, 0) + 0.0;//lcc 20240604
    _pcd(2) = -_sixlegdogModel->getFeetPosIdeal()(2, 0) + 0.0;//lcc 20240604

    // std::cout<<" _pcd :\n"<< _pcd.transpose() <<std::endl;

    _vCmdBody.setZero();
    _yawCmd = _lowState->getYaw();
    _Rd = rotz(_yawCmd);
    _wCmdGlobal.setZero();

    /* 将所有UserCommand和UserValue设置为0 */
    _ctrlComp->ioInter->zeroCmdPanel();
    /* 将目标全局速度->setZero，因为落足点是根据速度来定的 */
    // _gait->restart();
    _gait_P->restart();

    for(int i=0; i<18; i++){
        _lowCmd->motorCmd[i].q = _lowState->motorState[i].q;
    }
    _initVecOX = _ctrlComp->sixlegdogModel->getX(*_lowState); // P_b0_(0)
    _initVecXP = _ctrlComp->sixlegdogModel->getVecXP(*_lowState); // P_si

    _initFeetPos = _sixlegdogModel->getFeet2BPositions(*_lowState, FrameType::HIP);//QP

    // userValue_lcc.setZero();
    USVLCC_SETZERO = true; 

    for(int i=0; i<NUM_DOF_W; i++){ //lcc 20240809
        _lowCmd->motorCmd[i].dq = 0;
        _lowCmd->motorCmd[i].Kp = 30;
        _lowCmd->motorCmd[i].Kd = 2;
        _lowCmd->motorCmd[i].tau = 0;
    }
}

void State_Force_Pos::exit(){
    _ctrlComp->ioInter->zeroCmdPanel();
    _ctrlComp->setAllSwing();
}

FSMStateName State_Force_Pos::checkChange(){
    if(_lowState->userCmd == UserCommand::PASSIVE_1){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::FIXEDSTAND_2){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::FORCE_POS;
    }
}

void State_Force_Pos::run(){
    // Rob State
    _posBody = _est->getPosition();
    _velBody = _est->getVelocity();
    _yaw = _lowState->getYaw();
    _dYaw = _lowState->getDYaw();
    _posFeet2BGlobal = _est->getPosFeet2BGlobal();
    _posFeetGlobal = _est->getFeetPos();
    _velFeetGlobal = _est->getFeetVel();
    _B2G_RotMat = _lowState->getRotMat();//机身 到 世界 的变化矩阵
    _G2B_RotMat = _B2G_RotMat.transpose();//世界 到 机身 的变化矩阵

    // std::cout<<" _posBody :"<< _posBody.transpose() <<std::endl;
    // std::cout<<" _velBody :"<< _velBody.transpose() <<std::endl;
    // std::cout<<" _yaw :\n"<< _yaw*180/3.1415926 <<std::endl;
    // std::cout<<" getQuat :\n"<< _lowState->imu.getQuat() <<std::endl;
    // std::cout<<" getRotMat :\n"<< _lowState->getRotMat() <<std::endl;
    // std::cout<<" RPY :\n"<< rotMatToRPY(_lowState->getRotMat()).transpose()*180/3.1415926 <<std::endl;
    // std::cout<<" _yaw :\n"<< _yaw <<std::endl;
    // std::cout<<" getAcc :\n"<< _lowState->getAcc().transpose() <<std::endl;

    // Vec36 _footTipForceEst;
    // _footTipForceEst = _sixlegdogModel->calcForceByTauEst( vec36ToVec18(_lowState->getQ_Hex()), _lowState->getTau_Hex());
    // std::cout<<"_footTipForceEst:"<< _footTipForceEst <<std::endl;

    #if TERRIANESTI_FOURLEG
        (*_contact_te)(0) = (*_contact_hex)(0); 
        (*_contact_te)(1) = (*_contact_hex)(1); 
        (*_contact_te)(2) = (*_contact_hex)(4); 
        (*_contact_te)(3) = (*_contact_hex)(5);
        _posFeet2BGlobal_te.block< 3, 1>( 0, 0) =  _posFeet2BGlobal.block< 3, 1>( 0, 0); 
        _posFeet2BGlobal_te.block< 3, 1>( 0, 1) =  _posFeet2BGlobal.block< 3, 1>( 0, 1); 
        _posFeet2BGlobal_te.block< 3, 1>( 0, 2) =  _posFeet2BGlobal.block< 3, 1>( 0, 4); 
        _posFeet2BGlobal_te.block< 3, 1>( 0, 3) =  _posFeet2BGlobal.block< 3, 1>( 0, 5); 
        terr.terrain_adaptation( _posBody, _yawCmd, root_euler_d, _contact_te, _posFeet2BGlobal_te, _Apla);//lcc
    #endif

    // std::cout<<" root_euler_d :"<< root_euler_d.transpose()*180/3.1415926 <<std::endl;
    // std::cout<<" _Apla :"<< (*_Apla).transpose() <<std::endl;

    /* 将键盘输入的_userValue转换为 需要的控制量：body系下的 目标速度、角速度 */
    getUserCmd();
    /* 继续，得到world系下的：机身目标速度、速度、期望姿态角yaw、dyaw */
    calcCmd();

    /* setGait -> 设置世界系下的目标: vxyGoalGlobal, dYawGoal, gaitHeight*/
    // _gait->setGait(_vCmdGlobal.segment(0,2), _wCmdGlobal(2), _gaitHeight);
    /* run传的是指针，地址绑定，直接得到：世界系下足端目标位置和速度 */
    // _gait->run(_posFeetGlobalGoal, _velFeetGlobalGoal);





    // Vec36 pIncrement = Vec36::Zero(); // 位置增量，暂时设为零向量
    // Vec1_6 footHold = Vec1_6::Zero();
    // _gait_P->run(_posFeetGlobalGoal, _velFeetGlobalGoal, pIncrement, &footHold); // 传递地址
    _gait_P->run(_posFeetGlobalGoal, _velFeetGlobalGoal, _posFeet2BGoal_P_Increment, terian_FootHold); 


    // _posFeet2BGoal_P_Increment.setZero();
    // terian_FootHold = new Vec1_6;
    // (*terian_FootHold).setZero();

    // _gait_P->run(_posFeetGlobalGoal, _velFeetGlobalGoal,_posFeet2BGoal_P_Increment, terian_FootHold);//(4.9 wjw)


    calcTau(); // QP力矩控制->_tau
    calcP(); //位置控制
    #if USE_A_REAL_HEXAPOD == true
    _torqueCtrl();// 位置控制 + 位置反馈 的力矩控制->torque18
    #else
    _torqueCtrl();// 位置控制 + 位置反馈 的力矩控制
    #endif
    
    if(checkStepOrNot()){
        _ctrlComp->setStartWave();
    }else{
        _ctrlComp->setAllStance();
    }

    #if USE_A_REAL_HEXAPOD == true
    Vec18 tau_send;
    tau_send = (_tau * 0.5 + torque18 * 0.3) * 1;
    _lowCmd->setTau( tau_send ); //lcc 20240602

    // std::cout<<" _forceFeetBody: \n"<< _forceFeetBody <<std::endl;
    float kkk = 0.1;
    for(int i(0); i<6; ++i){
        if((*_contact_hex)(i) == 0){
            // _lowCmd->setLegGain(i, 100 * kkk, 3 * kkk);//swing
            _lowCmd->setLegGain(i, 150 * kkk, 1.5);//swing
        }else{
            // if( i == 2 || i == 3)
            // _lowCmd->setLegGain(i, 400 * kkk, 4 * kkk * 1);//stand,mid leg
            // else
            // _lowCmd->setLegGain(i, 100 * kkk, 3 * kkk);//stand
            _lowCmd->setLegGain(i, 100 * kkk, 1);//stand
        }
    }

    #else
    Vec18 tau_send;
    tau_send = _tau * 1 + torque18 * 1;
    // tau_send = _tau * 0 + torque18 * 1;
    _lowCmd->setTau( tau_send ); //lcc 20240602
    #endif
}

bool State_Force_Pos::checkStepOrNot(){
    if( (fabs(_vCmdBody(0)) > 0.01) ||
        (fabs(_vCmdBody(1)) > 0.01) ||
        (fabs(_posError(0)) > 0.04) ||
        (fabs(_posError(1)) > 0.04) ||
        (fabs(_velError(0)) > 0.025) ||
        (fabs(_velError(1)) > 0.025) ||
        (fabs(_dYawCmd) > 0.01) ){
        return true;
    }
    else{
        return false;
    }
}

void State_Force_Pos::getUserCmd(){
    /* Movement */
    _vCmdBody(0) =  invNormalize(userValue_lcc.ly, _vxLim(0), _vxLim(1));
    _vCmdBody(1) = -invNormalize(userValue_lcc.lx, _vyLim(0), _vyLim(1));
    _vCmdBody(2) = 0;
    
    /* Turning */
    _dYawCmd = -invNormalize(userValue_lcc.rx, _wyawLim(0), _wyawLim(1));
    _dYawCmd = 0.9*_dYawCmdPast + (1-0.9) * _dYawCmd;
    _dYawCmdPast = _dYawCmd;
}

void State_Force_Pos::calcCmd(){
    /* Movement */
    _vCmdGlobal = _B2G_RotMat * _vCmdBody; //将机身速度映射到world系

    if ( _posBody(0) != 0 ||  _posBody(1) != 0 ||  _posBody(2) != 0){
        //origin
        _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * _ctrlComp->dt, Vec2(_posBody(0) - 0.05, _posBody(0) + 0.05));
        _pcd(1) = saturation(_pcd(1) + _vCmdGlobal(1) * _ctrlComp->dt, Vec2(_posBody(1) - 0.05, _posBody(1) + 0.05));
        // //lcc 20240914
        // _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * _ctrlComp->dt, Vec2(_posBody(0) - 1, _posBody(0) + 1));
        // _pcd(1) = saturation(_pcd(1) + _vCmdGlobal(1) * _ctrlComp->dt, Vec2(_posBody(1) - 1, _posBody(1) + 1));
    }
    else{
        //lcc 20240621： 摆脱状态估计的依赖->_posbody
        _pcd(0) = _pcd(0) +_vCmdGlobal(0) * _ctrlComp->dt;
        _pcd(1) = _pcd(1) +_vCmdGlobal(1) * _ctrlComp->dt;
    }

    POS_WORLD_DES = _pcd;
    POS_WORLD_DES(0) = POS_WORLD_DES(0) + _vCmdGlobal(0) * _ctrlComp->dt;
    POS_WORLD_DES(1) = POS_WORLD_DES(1) + _vCmdGlobal(1) * _ctrlComp->dt;

    RPY_DES = root_euler_d;
    VEL_WORLD_DES = _vCmdGlobal;

    //lcc 20240624:在位置控制中，如果使用速度限制，会导致崩溃。
    if ( _velBody(0) != 0 ||  _velBody(1) != 0 ||  _velBody(2) != 0){
        //origin
        _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(_velBody(0)-0.2, _velBody(0)+0.2));
        _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2(_velBody(1)-0.2, _velBody(1)+0.2));
        _vCmdGlobal(2) = 0;
    }
    else
    {
        //lcc 20240621： 摆脱状态估计的依赖->_velbody
        _vCmdGlobal(0) = saturation(_vCmdGlobal(0), _vxLim);
        _vCmdGlobal(1) = saturation(_vCmdGlobal(1), _vyLim);
        _vCmdGlobal(2) = 0;
    }


    /* Turning */
    _yawCmd = _yawCmd + _dYawCmd * _ctrlComp->dt;

    // _Rd = rotz(_yawCmd);
    Eigen::Matrix3d eye3;
    eye3.setIdentity();
    // _Rd = rpyToRotMat(0, 0.2, _yawCmd)* eye3;//lcc
    // _yawCmd = saturation(_yawCmd, Vec2(_lowState->getYaw()-0.05, _lowState->getYaw()+0.05));
    // _dYawCmd = saturation(_dYawCmd, Vec2(_lowState->getDYaw()-0.2, _lowState->getDYaw()+0.2));
    _Rd = rpyToRotMat(root_euler_d(0), root_euler_d(1), _yawCmd)* eye3;//lcc
    // std::cout<<" root_euler_d :\n"<< root_euler_d.transpose() <<std::endl;
    _wCmdGlobal(2) = _dYawCmd;
}

void State_Force_Pos::calcTau(){
    _posError = _pcd - _posBody;
    /*--------------lcc start 20240604----------------*/
    Vec6 leg_deep;
    int leg_deep_num;
    leg_deep_num = 0;
    leg_deep.setZero();
    for (int i = 0; i < 6; i++)
    {   
        if((*_contact_hex)(i) == 1) //stand
        {
            leg_deep(i) = _ctrlComp->sixlegdogModel->getFootPosition(*_lowState, i, FrameType::BODY)(2);
            leg_deep_num ++;
        }
    }
    if( (*_phase_hex)(0) > 0.3 && (*_phase_hex)(0) <= 0.8 )
    {
        // body_h = -(leg_deep(0) + leg_deep(1) + leg_deep(2) + leg_deep(3) + leg_deep(4) + leg_deep(5) )/leg_deep_num + 0.0944;
        body_h = -(leg_deep(0) + leg_deep(1) + leg_deep(2) + leg_deep(3) + leg_deep(4) + leg_deep(5) )/leg_deep_num ;
    }
    _posError(2) = _pcd(2) - body_h; // 使用接地腿的高度平均值作为机身实际高度，摆脱世界系下机身高度状态估计不准的难题
    // std::cout<<" body_h :"<< body_h <<std::endl;
    // std::cout<<" _pcd :\n"<< _pcd.transpose() <<std::endl;
    // std::cout<<" _posBody :\n"<< _posBody.transpose() <<std::endl;
    /*--------------lcc end 20240604----------------*/

    _velError = _vCmdGlobal - _velBody;

    #if USE_A_REAL_HEXAPOD == true
    // _Kpp = Vec3(20, 20, 100).asDiagonal();
    // _Kdp = Vec3(20, 20, 20).asDiagonal();
    // _kpw = 400;
    // _Kdw = Vec3(50, 50, 50).asDiagonal();
    // _KpSwing = Vec3(400, 400, 400).asDiagonal();
    // _KdSwing = Vec3(10, 10, 10).asDiagonal();

    // _Kpp = Vec3(30, 30, 100).asDiagonal();
    // _Kdp = Vec3(20, 20, 20).asDiagonal();
    // _kpw = 1000; 
    // _Kdw = Vec3(70, 70, 70).asDiagonal();

    // _KpSwing = Vec3(400, 400, 400).asDiagonal();
    // _KdSwing = Vec3(10, 10, 10).asDiagonal();

    _Kpp = Vec3(60, 60, 120).asDiagonal();
    _Kdp = Vec3(6, 6, 6).asDiagonal();
    // _kpw = 120;
    // _Kdw = Vec3(0.175, 0.175, 0.175).asDiagonal();
    _kpw = 125;
    _Kdw = Vec3(0.2, 0.2, 0.2).asDiagonal();
    #endif

    //lcc 20240827: 重大发现！！！ 这部用getGyro似乎比用getGyroGlobal更好；
    _ddPcd = _Kpp * _posError + _Kdp * _velError;
    // _dWbd  = _kpw*rotMatToExp(_Rd*_G2B_RotMat) + _Kdw * ( _wCmdGlobal - _lowState->getGyroGlobal()); //lcc 20240827: yaw>=90直接崩
    _dWbd  = _kpw*rotMatToExp(_Rd*_G2B_RotMat) + _Kdw * ( _wCmdGlobal - _lowState->getGyro()); //lcc 20240827: 在yaw=[95,-95]间灵活运动
    // _dWbd  = _kpw*rotMatToExp(_Rd*_G2B_RotMat) + _Kdw * ( _wCmdGlobal ); //lcc 20240827: 在yaw=[95,-95]间灵活运动

    _ddPcd(0) = saturation(_ddPcd(0), Vec2(-3, 3));
    _ddPcd(1) = saturation(_ddPcd(1), Vec2(-3, 3));
    _ddPcd(2) = saturation(_ddPcd(2), Vec2(-5, 5));

    _dWbd(0) = saturation(_dWbd(0), Vec2(-40, 40));
    _dWbd(1) = saturation(_dWbd(1), Vec2(-40, 40));
    _dWbd(2) = saturation(_dWbd(2), Vec2(-10, 10));

    /* 对于 (*_contact)(i) == 1-> 处于stand的腿。使用QP来获得支撑力  */
    _forceFeetGlobal = - _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact_hex);

    #if USE_A_REAL_HEXAPOD == true
    _KpSwing = Vec3(0, 0, 0).asDiagonal();
    _KdSwing = Vec3(0, 0, 0).asDiagonal();
    if( (*_phase_hex)(0) >= 0.85 ){
        _KpSwing = Vec3(0, 0, 0).asDiagonal();
        _KdSwing = Vec3(0, 0, 0).asDiagonal();
    }
    else if( (*_phase_hex)(0) >= 0.75 &&  (*_phase_hex)(0) < 0.85){
        _KpSwing = Vec3(15, 15, 15).asDiagonal();
        _KdSwing = Vec3(0.1, 0.1, 0.1).asDiagonal();
    }
    else{
        _KpSwing = Vec3(15, 15, 15).asDiagonal();
        _KdSwing = Vec3(0.1, 0.1, 0.1).asDiagonal();
    }
    #endif
    
    /* 对于 (*_contact)(i) == 0-> 处于swing的腿。使用PD来获得摆动力  */
    for(int i(0); i<6; ++i){
        if((*_contact_hex)(i) == 0){
            _forceFeetGlobal.col(i) = _KpSwing*(_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing*(_velFeetGlobalGoal.col(i)-_velFeetGlobal.col(i));
        }
    }

    // std::cout<<" _posFeetGlobalGoal: \n"<< _posFeetGlobalGoal <<std::endl;
    // std::cout<<" _posFeetGlobal: \n"<< _posFeetGlobal <<std::endl;
    // std::cout<<" _velFeetGlobalGoal: \n"<< _velFeetGlobalGoal <<std::endl;
    // std::cout<<" _velFeetGlobal: \n"<< _velFeetGlobal <<std::endl;

    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;//将足端力从world系转换到body系

    //lcc 20240617
    #if USE_A_REAL_HEXAPOD == true
    _forceFeetBody.block< 1, 6>( 0, 0) = _forceFeetBody.block< 1, 6>( 0, 0) * 1.5;
    _forceFeetBody.block< 2, 6>( 0, 0) = _forceFeetBody.block< 2, 6>( 0, 0) * 1.2;
    #else
    _forceFeetBody.block< 1, 6>( 0, 0) = _forceFeetBody.block< 1, 6>( 0, 0) * 5; //x方向的力
    _forceFeetBody.block< 2, 6>( 0, 0) = _forceFeetBody.block< 2, 6>( 0, 0) * 1; //y方向的力
    #endif
    _q = vec36ToVec18(_lowState->getQ_Hex());
    _tau = _sixlegdogModel->getTau(_q, _forceFeetBody);
    // std::cout<<" _forceFeetBody: \n"<< _forceFeetBody <<std::endl;
}

void State_Force_Pos::calcP(){
    //lcc 20240624: 位置控制的摆动轨迹
    _gait_P->setGait(_vCmdBody.segment(0,2), _wCmdGlobal(2), _gaitHeight);
    // _gait_P->run(_posSwingLeg_P, _velSwingLeg_P);
    _gait_P->run(_posSwingLeg_P, _velSwingLeg_P, _posFeet2BGoal_P_Increment, terian_FootHold);
    //lcc 20240624: 位置控制的支撑轨迹
    _spt->setGait(_vCmdBody.segment(0,2), _wCmdGlobal(2), 0);
    // _spt->run(_posSupportLeg_P, _velSupportLeg_P);
    _spt->run(_posSupportLeg_P, _velSupportLeg_P, _posFeet2BGoal_P_Increment, terian_FootHold);
    for(int i(0); i<6; ++i){  
        if((*_contact_hex)(i) == 1){  //stand
            // _posFeet2BGoal_P.col(i) = _G2B_RotMat * (_posSupportLeg_P.col(i) - _posBody);
            _posFeet2BGoal_P.col(i) = 1 * (_posSupportLeg_P.col(i) );
            _velFeet2BGoal.col(i) = _G2B_RotMat * (_velSupportLeg_P.col(i) - _velBody); 
        }
        else if((*_contact_hex)(i) == 0){ //swing
                // _posFeet2BGoal_P.col(i) = _G2B_RotMat * (_posSwingLeg_P.col(i) - _posBody);
                _posFeet2BGoal_P.col(i) = 1 * (_posSwingLeg_P.col(i) );
                _velFeet2BGoal.col(i) = _G2B_RotMat * (_velSwingLeg_P.col(i) - _velBody); 
        }
    }

    //基于足端位置的姿态调整
    _initVecOX = _ctrlComp->sixlegdogModel->getFootPosition(*_lowState, 0, FrameType::BODY); // P_b0_(0)
    Vec3 x = _initVecOX;
    Vec36 vecXP, qLegs;
    qLegs = _lowState->getQ_Hex();
    for(int i(0); i < 6; ++i){
        vecXP.col(i) = _ctrlComp->sixlegdogModel->getFootPosition(*_lowState, i, FrameType::BODY) - x;
    }
    _initVecXP = vecXP;
    float row, pitch, yaw, height;
    Vec3 rpy;
    rpy = rotMatToRPY(_ctrlComp->lowState->getRotMat());
    adj_RPY_P << root_euler_d(0) - rpy(0), root_euler_d(1) - rpy(1), 0;
    // adj_RPY_P << root_euler_d(0) , root_euler_d(1) , 0;
    // std::cout<<" adj_RPY_P: \n"<< adj_RPY_P.transpose() <<std::endl;
    adj_RPY_P = 0.0 * adj_RPY_P_past + (1 - 0.0) * adj_RPY_P;
    // std::cout<<" adj_RPY_P affter: \n"<< adj_RPY_P.transpose() <<std::endl;
    adj_RPY_P_past = adj_RPY_P;
    row = invNormalize(adj_RPY_P(0), _rowMin, _rowMax);
    pitch = invNormalize(adj_RPY_P(1), _pitchMin, _pitchMax);
    yaw = -invNormalize(adj_RPY_P(2), _yawMin, _yawMax);
    // height = invNormalize(userValue_lcc.ry, _heightMin, _heightMax) ;
    height = 0;
    for(int i(0); i < 6; ++i){
        if((*_contact_hex)(i) == 1){  //stand
        _posFeet2BGoal_P.col(i) = _posFeet2BGoal_P.col(i) + (_calcOP(row, pitch, yaw, height).col(i) - _ctrlComp->sixlegdogModel->getFeet2BPositions(*_lowState,FrameType::BODY ).col(i));
        }
    }
    _ctrlComp->lowCmd->setQ( _ctrlComp->sixlegdogModel->getQ( _posFeet2BGoal_P, FrameType::BODY) );
}

Vec36 State_Force_Pos::_calcOP(float row, float pitch, float yaw, float height){
    Vec3 vecXO = -_initVecOX;
    vecXO(2) += height;
    RotMat rotM = rpyToRotMat(row, pitch, yaw);
    HomoMat Tsb = homoMatrix(vecXO, rotM);
    HomoMat Tbs = homoMatrixInverse(Tsb);
    Vec4 tempVec6;
    Vec36 vecOP;
    for(int i(0); i<6; ++i){
        tempVec6 = Tbs * homoVec(_initVecXP.col(i));
        vecOP.col(i) = noHomoVec(tempVec6);
    }
    return vecOP;
}

void State_Force_Pos::_torqueCtrl(){
    #if USE_A_REAL_HEXAPOD == true
        // _Kp = Vec3(500, 500, 500).asDiagonal();
        // _Kd = Vec3( 15,  15, 15).asDiagonal() ;
        // _Kp = Vec3(400, 400, 400).asDiagonal();
        // _Kd = Vec3( 10,  10, 10).asDiagonal() ;
        _Kp = Vec3(100, 100, 100).asDiagonal();
        _Kd = Vec3( 1,  1, 1).asDiagonal() ;
        // _Kp = Vec3(35, 35, 35).asDiagonal();
        // _Kd = Vec3( 1,  1, 1).asDiagonal() ;
    #else
        // _Kp = Vec3(5000, 5000, 5000).asDiagonal();
        // _Kd = Vec3( 200,  200, 200).asDiagonal();
        _Kp = Vec3(3500, 3500, 3500).asDiagonal();
        _Kd = Vec3( 120,  120, 120).asDiagonal();
    #endif
    Vec36 pos36;
    Vec36 vel36;
    Vec36 _targetPos36_o1;
    Vec36 _targetPos36_o2;
    Vec36 _targetPos36_o3;
    Vec36 force36_o1;
    Vec36 force36_o2;
    Vec36 force36_o3;
    Vec18 torque18_o1;
    Vec18 torque18_o2;
    Vec18 torque18_o3;
    Vec18 _q;

    pos36.setZero();
    vel36.setZero();
    force36_o1.setZero();
    force36_o2.setZero();
    force36_o3.setZero();
    _targetPos36_o1.setZero();
    _targetPos36_o2.setZero();
    _targetPos36_o3.setZero();
    torque18_o1.setZero();
    torque18_o2.setZero();
    torque18_o3.setZero();
    _q.setZero();
    torque18.setZero();

    pos36 = _ctrlComp->sixlegdogModel->getFeet2BPositions(*_lowState,FrameType::BODY );
    vel36 = _ctrlComp->sixlegdogModel->getFeet2BVelocities(*_lowState,FrameType::BODY );

    for(int i(0); i<6; ++i){
        _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _posBody);
        _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody); 
    }
    // _posFeet2BGoal.block< 1, 6>( 2, 0) = -_posFeet2BGoal.block< 1, 6>( 2, 0);
    // std::cout<<" _posBody: \n"<< _posBody <<std::endl;
    // std::cout<<" _posFeet2BGoal calcQQd: \n"<< _posFeet2BGoal <<std::endl;

    _targetPos36_o3 = _posFeet2BGoal_P; //位置控制 计算得到的P
    _targetPos36_o2 = _posFeet2BGoal;
    _targetPos36_o1 = _ctrlComp->sixlegdogModel->_feetPosNormalStand;
    // std::cout<<" _posFeet2BGoal: \n"<< _posFeet2BGoal <<std::endl;
    // std::cout<<" _feetPosNormalStand: \n"<< _ctrlComp->sixlegdogModel->_feetPosNormalStand <<std::endl;
    for (int i = 0; i < 6; ++i){
        force36_o1.block< 3, 1>( 0, i) =  _Kp*(_targetPos36_o1.block< 3, 1>( 0, i) - pos36.block< 3, 1>( 0, i) ) + _Kd*(-vel36.block< 3, 1>( 0, i) );
        force36_o2.block< 3, 1>( 0, i) =  _Kp*(_targetPos36_o2.block< 3, 1>( 0, i) - pos36.block< 3, 1>( 0, i) ) + _Kd*(-vel36.block< 3, 1>( 0, i) );
        force36_o3.block< 3, 1>( 0, i) =  _Kp*(_targetPos36_o3.block< 3, 1>( 0, i) - pos36.block< 3, 1>( 0, i) ) + _Kd*(-vel36.block< 3, 1>( 0, i) );
    }

    #if USE_A_REAL_HEXAPOD == true
    /* 对于 (*_contact)(i) == 0-> 处于swing的腿。  */
    for(int i(0); i<6; ++i){
        if((*_contact_hex)(i) == 0){
            if( (*_phase_hex)(i) >= 0.85 ){
                    force36_o1.col(i) = force36_o1.col(i) * 1;
                    force36_o2.col(i) = force36_o2.col(i) * 1;
                    force36_o3.col(i) = force36_o3.col(i) * 1;
            }
            else if( (*_phase_hex)(i) >= 0.75 &&  (*_phase_hex)(i) < 0.85){
                    force36_o1.col(i) = force36_o1.col(i) * 2;
                    force36_o2.col(i) = force36_o2.col(i) * 2;
                    force36_o3.col(i) = force36_o3.col(i) * 2;
            }
            else{
                    force36_o1.col(i) = force36_o1.col(i) * 3;
                    force36_o2.col(i) = force36_o2.col(i) * 3;
                    force36_o3.col(i) = force36_o3.col(i) * 3;
            }
        }
    }
    #endif

    // std::cout<<" force36_o3: \n"<< force36_o3 <<std::endl;

    _q = vec36ToVec18(_lowState->getQ_Hex()); 
    torque18_o1 = _ctrlComp->sixlegdogModel->getTau( _q, force36_o1);
    torque18_o2 = _ctrlComp->sixlegdogModel->getTau( _q, force36_o2);
    torque18_o3 = _ctrlComp->sixlegdogModel->getTau( _q, force36_o3);
    #if USE_A_REAL_HEXAPOD == true
    torque18 = (torque18_o1 * 0.005 + torque18_o2 * 0.05 + torque18_o3 * 1) *0.3; //力矩控制
    #else
    // torque18 = (torque18_o1 * 0.0005 + torque18_o2 * 0.005 + torque18_o3 * 1.5) * 0.1; //力位混合
    torque18 = (torque18_o1 * 0.0005 + torque18_o2 * 0.005 + torque18_o3 * 1.5) * 0.1; //力位混合
    // torque18 = torque18_o1 * 0.0 + torque18_o2 * 0.0 + torque18_o3 * 1; //位置控制
    #endif
}
