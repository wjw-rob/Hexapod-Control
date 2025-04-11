 
#ifndef GAITGENERATOR_P_H_
#define GAITGENERATOR_P_H_

#include "Gait/SupportFeetEndP.h"
#include "Gait/WaveGenerator.h"
// #include "Gait/FeetEndCal.h"

#ifdef COMPILE_DEBUG
#include <common/PyPlot.h>
#endif  // COMPILE_DEBUG

/*cycloid gait 与 GaitGenerator 没什么区别（wjw 3.19）*/
class GaitGenerator_P{
public:
    GaitGenerator_P(CtrlComponents *ctrlComp);
    ~GaitGenerator_P();
    void setGait(Vec2 vxyGoalGlobal, float dYawGoal, float gaitHeight);
    // void run(Vec34 &feetPos, Vec34 &feetVel);
    void run(Vec36 &feetPos, Vec36 &feetVel, Vec36 P_Increment, Vec1_6 *terian_FootHold);
    Vec3 getFootPos(int i);
    Vec3 getFootVel(int i);
    void restart();
    void useCPG(Eigen::Matrix<double,1,6> cpg_phase, Eigen::Matrix<double,1,6> cpg_contact);//加的，lcc说cpg用不了，那这一部分的代码是不是没用？可以删？（wjw 3.19）
private:
    float cycloidXYPosition(float startXY, float endXY, float phase);
    float cycloidXYVelocity(float startXY, float endXY, float phase);
    float cycloidZPosition(float startZ, float height, float phase, float end);
    float cycloidZVelocity(float height, float phase, float end);

    WaveGenerator *_waveG;
    Estimator *_est;
    SupportFeetEndP *_feetCal;
    // QuadrupedRobot *_robModel;
    HexapodRobot *_robModel;//lcc加的 跟我修改的一样 那是不是GaitGenerator_P 可以直接替代 GaitGenerator？（wjw 3.19）
    LowlevelState *_state;
    float _gaitHeight;
    Vec2 _vxyGoal;
    float _dYawGoal;

    #if IS_THIS_A_HEXAPOD
    Vec6 *_phase, _phasePast;
    VecInt6 *_contact;
    Vec36 _startP, _endP, _idealP, _pastP;
    bool _firstRun;
    Vec6 endPz;
    #else
    Vec4 *_phase, _phasePast;
    VecInt4 *_contact;
    Vec34 _startP, _endP, _idealP, _pastP;
    bool _firstRun;
    Vec4 endPz;
    #endif

#ifdef COMPILE_DEBUG
    PyPlot _testGaitPlot;
#endif  // COMPILE_DEBUG

};

#endif  // GAITGENERATOR_H