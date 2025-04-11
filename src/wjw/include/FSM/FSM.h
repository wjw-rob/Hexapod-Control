 
#ifndef FSM_H
#define FSM_H

// FSM States
#include <fstream> // c++文件操作
#include "FSM/FSMState.h"
#include "FSM/State_FixedStand.h"
#include "FSM/State_Passive.h"
// #include "FSM/State_FreeStand.h"
#include "FSM/State_Position.h"
// #include "FSM/State_MPC.h"
// #include "FSM/State_BalanceTest.h"
// #include "FSM/State_SwingTest.h"
// #include "FSM/State_StepTest.h"
// #include "FSM/State_MPC.h"
// #include "FSM/State_QP.h"
#include "common/enumClass.h"
#include "control/CtrlComponents.h"
// #include "FSM/State_PosReflex.h"
// #include "FSM/State_FixedSquat.h"
#include "FSM/State_Force_Pos.h"
// #include "FSM/State_MPC_Force_Pos.h"

struct FSMStateList{//成员初始化在FSM.cpp中
    FSMState *invalid;
    State_Passive *passive;
    State_FixedStand *fixedStand;
    // State_FreeStand *freeStand;
    State_Position *position;
    // State_A1MPC *a1mpc; // lcc 20240416
    // State_QP *qp;// lcc 20240523
    // State_BalanceTest *balanceTest;
    // State_SwingTest *swingTest;
    // State_StepTest *stepTest;
    // State_PosReflex *posReflex;//lcc 20240627
    // State_FixedSquat *fixedSquat;//lcc 20240808
    State_Force_Pos *force_pos;//lcc 20240827
    // State_MPC_Force_Pos *mpc_force_pos;//lcc 20240903
// #ifdef COMPILE_WITH_MOVE_BASE
    // State_move_base *moveBase;
// #endif  // COMPILE_WITH_MOVE_BASE

    void deletePtr(){
        delete invalid;
        delete passive;
        delete fixedStand;
        // delete freeStand;
        delete position;
        // delete a1mpc;// lcc 2024041s#ifdef COMPILE_WITH_MOVE_BASE
    // _stateList.moveBase = new State_move_base(_ctrlComp);
        // delete posReflex;// lcc 20240627
        // delete qp;// lcc 20240523
        delete force_pos;// lcc 20240827
        // delete mpc_force_pos;// lcc 20240903
        // delete balanceTest;
        // delete swingTest;
        // delete stepTest;
// #ifdef COMPILE_WITH_MOVE_BASE
        // delete moveBase;
// #endif  // COMPILE_WITH_MOVE_BASE
    }
};

class FSM{
public:
    FSM(CtrlComponents *ctrlComp);
    ~FSM();
    void initialize();
    void run();
    void fsm_run();

// private:
    FSMState* getNextState(FSMStateName stateName);
    bool checkSafty();
    bool initialized = false;//(wjw)
    CtrlComponents *_ctrlComp;// 结构体，包含所有控制组件
    FSMState *_currentState;
    FSMState *_nextState;
    FSMStateName _nextStateName;
    FSMStateList _stateList;// 结构体，包含所有状态
    FSMMode _mode;
    long long _startTime;
    int count;

    bool fsm_first_start = true;

};


#endif  // FSM_H