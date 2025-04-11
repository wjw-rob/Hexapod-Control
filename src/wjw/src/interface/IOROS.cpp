#include "interface/IOROS.h"
#include "interface/KeyBoard.h"
#include <iostream>
#include <unistd.h>
#include <csignal>

Vec3 ODE_P;
Vec3 ODE_V;

void RosShutDown(int sig){//用户按下 Ctrl + C 时发送的信号）时，会输出一条 ROS 信息并调用 ros::shutdown() 来关闭 ROS 节点
	ROS_INFO("ROS interface shutting down!");
	ros::shutdown();
}

//IOROS::IOROS():IOInterface(){} 表示在构造 IOROS 类的对象时，首先调用其基类 IOInterface 的默认构造函数。
//这是一种初始化基类的常见方式，尤其是当基类有一个显式的构造函数时。
IOROS::IOROS():IOInterface(){
    std::cout << "The control interface for ROS Gazebo simulation" << std::endl;

    // start subscriber
    initRecv();//调用 initRecv() 启动订阅者，创建一个异步 Spinner 并启动，等待 300 毫秒让订阅者启动
    ros::AsyncSpinner subSpinner(4); // 4 threads 异步多线程 
    subSpinner.start();
    usleep(300000);     //wait for subscribers start
    // initialize publisher
    initSend();//调用 initSend() 初始化发布者   

    signal(SIGINT, RosShutDown);

    // 由于 KeyBoard 类继承自 CmdPanel 类，因此 KeyBoard 对象也被视为一种 CmdPanel 对象。
    // 这就是继承的基本概念，子类对象可以赋值给父类指针或引用。
    cmdPanel = new KeyBoard();//创建一个 KeyBoard 对象，用于处理用户的控制命令

    quaternion_offset << 0, 0, 0, 0;//初始化四元数偏移量 quaternion_offset
}

IOROS::~IOROS(){//析构函数中删除 cmdPanel 对象并关闭 ROS 节点
    delete cmdPanel;
    ros::shutdown();
}

/*该函数用于发送命令和接收状态信息。
调用 sendCmd 和 recvState 函数分别发送命令和接收状态。
从 cmdPanel 对象中获取用户命令、用户值和用户功能模式，并赋值给 state 结构体的相应成员。
获取 ODE 相关的位置和速度信息并赋值给全局变量 ODE_P 和 ODE_V*/
void IOROS::sendRecv(const LowlevelCmd *cmd, LowlevelState *state){
    
    sendCmd(cmd, state);//发送给电机命令
    recvState(state);//接收电机状态、IMU状态信息 

    //从 cmdPanel 获取用户命令、用户值和用户功能模式，并更新到 state 中
    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue();
    state->userFunctionMode = cmdPanel->getUserFunctionMode();// lcc 20250601
    //

    // state->userFunctionMode_p->function_test = cmdPanel->getUserFunctionMode().function_test;
    // state->userFunctionMode_p = &cmdPanel->getUserFunctionMode();
    // retSimOdeBodyP();
    // retSimOdeBodyV();
    ODE_P = retSimOdeBodyP();//获取 ODE 相关的位置和速度信息并赋值给全局变量 ODE_P 和 ODE_V
    ODE_V = retSimOdeBodyV();


}


//初始化多个 ROS 发布者，用于向每个电机发送控制命令。每个 pub_joint_cmd[i] 对应一个电机的控制命令，类型是 std_msgs::Float64
//nh.advertise：用于发布到指定的 ROS 话题（如 /hexapod_description2/rfj1_jointcc/command），并设置队列长度为 1000
void IOROS::initSend(){
    // ROS publisher 话题发布        ""里的是模型关节编号
    pub_joint_cmd[0] = nh.advertise<std_msgs::Float64>("/hexapod_description2/rfj1_jointcc/command", 1000);
    pub_joint_cmd[1] = nh.advertise<std_msgs::Float64>("/hexapod_description2/rfj2_jointcc/command", 1000);
    pub_joint_cmd[2] = nh.advertise<std_msgs::Float64>("/hexapod_description2/rfj3_jointcc/command", 1000);

    pub_joint_cmd[3] = nh.advertise<std_msgs::Float64>("/hexapod_description2/lfj5_jointcc/command", 1000);
    pub_joint_cmd[4] = nh.advertise<std_msgs::Float64>("/hexapod_description2/lfj6_jointcc/command", 1000);
    pub_joint_cmd[5] = nh.advertise<std_msgs::Float64>("/hexapod_description2/lfj7_jointcc/command", 1000);

    pub_joint_cmd[6] = nh.advertise<std_msgs::Float64>("/hexapod_description2/rmj9_jointcc/command", 1000);
    pub_joint_cmd[7] = nh.advertise<std_msgs::Float64>("/hexapod_description2/rmj10_jointcc/command", 1000);
    pub_joint_cmd[8] = nh.advertise<std_msgs::Float64>("/hexapod_description2/rmj11_jointcc/command", 1000);

    pub_joint_cmd[9] = nh.advertise<std_msgs::Float64>("/hexapod_description2/lmj13_jointcc/command", 1000);
    pub_joint_cmd[10] = nh.advertise<std_msgs::Float64>("/hexapod_description2/lmj14_jointcc/command", 1000);
    pub_joint_cmd[11] = nh.advertise<std_msgs::Float64>("/hexapod_description2/lmj15_jointcc/command", 1000);

    pub_joint_cmd[12] = nh.advertise<std_msgs::Float64>("/hexapod_description2/rbj17_jointcc/command", 1000);
    pub_joint_cmd[13] = nh.advertise<std_msgs::Float64>("/hexapod_description2/rbj18_jointcc/command", 1000);
    pub_joint_cmd[14] = nh.advertise<std_msgs::Float64>("/hexapod_description2/rbj19_jointcc/command", 1000);

    pub_joint_cmd[15] = nh.advertise<std_msgs::Float64>("/hexapod_description2/lbj21_jointcc/command", 1000);
    pub_joint_cmd[16] = nh.advertise<std_msgs::Float64>("/hexapod_description2/lbj22_jointcc/command", 1000);
    pub_joint_cmd[17] = nh.advertise<std_msgs::Float64>("/hexapod_description2/lbj23_jointcc/command", 1000);

    printf(" IOROS ros inti.\n");
}

// void IOROS::doMsg_yobotics_joint_states(const sensor_msgs::JointState::ConstPtr& sub_joint_states)
Eigen::Matrix<double, LEG_DOF_W, NUM_LEG_W> sub_joint_p_local, sub_joint_p_local_temp;//3*6 存储机器人的 关节位置 数据
Eigen::Matrix<double, LEG_DOF_W, NUM_LEG_W> sub_joint_v_local, sub_joint_v_local_temp;//速度
Eigen::Matrix<double, LEG_DOF_W, NUM_LEG_W> sub_joint_t_local, sub_joint_t_local_temp;//关节力矩（扭矩））
int sub_joint_state_seq_local;//整型 存储关节状态消息的序列号。用于标记关节状态消息的顺序，帮助处理不同的消息
Vec36 sub_joint_p_local_temp_origin;//用于存储原始的关节位置数据

void doMsg_yobotics_joint_states(const sensor_msgs::JointState::ConstPtr& sub_joint_states)
{
    sub_joint_state_seq_local = sub_joint_states->header.seq;
    for (int i = 0; i < NUM_DOF_W; i++)
    {
        sub_joint_p_local_temp(i) = sub_joint_states->position[i];
        sub_joint_v_local_temp(i) = sub_joint_states->velocity[i];
        sub_joint_t_local_temp(i) = sub_joint_states->effort[i];
    }


   //  0~18: RF LF RM LM RB LB 
    sub_joint_p_local.block<3,1>(0,0) = sub_joint_p_local_temp.block<3,1>(0,4);//rf
    sub_joint_p_local.block<3,1>(0,1) = sub_joint_p_local_temp.block<3,1>(0,1);//lf
    sub_joint_p_local.block<3,1>(0,2) = sub_joint_p_local_temp.block<3,1>(0,5);//rm
    sub_joint_p_local.block<3,1>(0,3) = sub_joint_p_local_temp.block<3,1>(0,2);//lm
    sub_joint_p_local.block<3,1>(0,4) = sub_joint_p_local_temp.block<3,1>(0,3);//rb
    sub_joint_p_local.block<3,1>(0,5) = sub_joint_p_local_temp.block<3,1>(0,0);//lb
    sub_joint_p_local_temp_origin = sub_joint_p_local;

    // std::cout<<"sub_joint_p_local_temp_origin:  \n"<< sub_joint_p_local_temp_origin * 180/3.1415926 <<std::endl;


   //  0~18: RF LF RM LM RB LB 
    sub_joint_v_local.block<3,1>(0,0) = sub_joint_v_local_temp.block<3,1>(0,4);
    sub_joint_v_local.block<3,1>(0,1) = sub_joint_v_local_temp.block<3,1>(0,1);
    sub_joint_v_local.block<3,1>(0,2) = sub_joint_v_local_temp.block<3,1>(0,5);//rm
    sub_joint_v_local.block<3,1>(0,3) = sub_joint_v_local_temp.block<3,1>(0,2);
    sub_joint_v_local.block<3,1>(0,4) = sub_joint_v_local_temp.block<3,1>(0,3);
    sub_joint_v_local.block<3,1>(0,5) = sub_joint_v_local_temp.block<3,1>(0,0);

   //  0~18: RF LF RM LM RB LB 
    sub_joint_t_local.block<3,1>(0,0) = sub_joint_t_local_temp.block<3,1>(0,4);
    sub_joint_t_local.block<3,1>(0,1) = sub_joint_t_local_temp.block<3,1>(0,1);
    sub_joint_t_local.block<3,1>(0,2) = sub_joint_t_local_temp.block<3,1>(0,5);//rm
    sub_joint_t_local.block<3,1>(0,3) = sub_joint_t_local_temp.block<3,1>(0,2);
    sub_joint_t_local.block<3,1>(0,4) = sub_joint_t_local_temp.block<3,1>(0,3);
    sub_joint_t_local.block<3,1>(0,5) = sub_joint_t_local_temp.block<3,1>(0,0);


    //旋转方向调整->p
    sub_joint_p_local(0) = -sub_joint_p_local(0);
    sub_joint_p_local(1) = -sub_joint_p_local(1);
    sub_joint_p_local(5) = -sub_joint_p_local(5);
    sub_joint_p_local(6) = -sub_joint_p_local(6);
    sub_joint_p_local(8) = -sub_joint_p_local(8);
    sub_joint_p_local(10) = -sub_joint_p_local(10);
    sub_joint_p_local(12) = -sub_joint_p_local(12);
    sub_joint_p_local(14) = -sub_joint_p_local(14);
    sub_joint_p_local(16) = -sub_joint_p_local(16);
    // sub_joint_p_local = sub_joint_p_local - p_offset;

    // //旋转方向调整->v
    sub_joint_v_local(0) = -sub_joint_v_local(0);
    sub_joint_v_local(1) = -sub_joint_v_local(1);
    sub_joint_v_local(5) = -sub_joint_v_local(5);
    sub_joint_v_local(6) = -sub_joint_v_local(6);
    sub_joint_v_local(8) = -sub_joint_v_local(8);
    sub_joint_v_local(10) = -sub_joint_v_local(10);
    sub_joint_v_local(12) = -sub_joint_v_local(12);
    sub_joint_v_local(14) = -sub_joint_v_local(14);
    sub_joint_v_local(16) = -sub_joint_v_local(16);

    // // //旋转方向调整->t
    sub_joint_t_local(0) = -sub_joint_t_local(0);
    sub_joint_t_local(1) = -sub_joint_t_local(1);
    sub_joint_t_local(5) = -sub_joint_t_local(5);
    sub_joint_t_local(6) = -sub_joint_t_local(6);
    sub_joint_t_local(8) = -sub_joint_t_local(8);
    sub_joint_t_local(10) = -sub_joint_t_local(10);
    sub_joint_t_local(12) = -sub_joint_t_local(12);
    sub_joint_t_local(14) = -sub_joint_t_local(14);
    sub_joint_t_local(16) = -sub_joint_t_local(16);
}

Eigen::Matrix<double,4,1> sub_imu_orie_local;
Eigen::Matrix<double,3,1> sub_imu_ang_v_local;
Eigen::Matrix<double,3,1> sub_imu_lin_a_local;
int sub_imu_seq_local;
void doMsg_yobotics_imu_msg(const sensor_msgs::Imu::ConstPtr& sub_imu_msg){          //doaMsg回调函数
    sub_imu_seq_local = sub_imu_msg->header.seq;

    sub_imu_orie_local << sub_imu_msg->orientation.x, sub_imu_msg->orientation.y, sub_imu_msg->orientation.z, sub_imu_msg->orientation.w;
    sub_imu_ang_v_local << sub_imu_msg->angular_velocity.x, sub_imu_msg->angular_velocity.y, sub_imu_msg->angular_velocity.z;
    sub_imu_lin_a_local << sub_imu_msg->linear_acceleration.x, sub_imu_msg->linear_acceleration.y, sub_imu_msg->linear_acceleration.z;

    // std::cout << sub_imu_ang_v_local << std::endl;
}

Eigen::Matrix<double,3,1> sub_odo_pos_local;
Eigen::Matrix<double,3,1> sub_odo_lin_twist_local;
void doMsg_nav_odometry(const nav_msgs::Odometry::ConstPtr& sub_nav_odometry){
    sub_odo_pos_local << sub_nav_odometry->pose.pose.position.x, sub_nav_odometry->pose.pose.position.y, 
                    sub_nav_odometry->pose.pose.position.z;
    sub_odo_lin_twist_local << sub_nav_odometry->twist.twist.linear.x, sub_nav_odometry->twist.twist.linear.y, 
                         sub_nav_odometry->twist.twist.linear.z;
    // std::cout << sub_odo_pos_local << std::endl;
}

//消息订阅
void IOROS::initRecv(){
    sub_joint_states = nh.subscribe<sensor_msgs::JointState>("/hexapod_description2/joint_states",1,doMsg_yobotics_joint_states);
    sub_imu_msg = nh.subscribe<sensor_msgs::Imu>("/imu",1,doMsg_yobotics_imu_msg);
    sub_nav_odometry = nh.subscribe<nav_msgs::Odometry>("/POS_COM",1,doMsg_nav_odometry);
}

Eigen::Matrix<double, 3, 1> IOROS::retSimOdeBodyP(){
    return sub_odo_pos_local;
}

Eigen::Matrix<double, 3, 1> IOROS::retSimOdeBodyV(){
    return sub_odo_lin_twist_local;
}

void IOROS::sendCmd(const LowlevelCmd *lowCmd, LowlevelState *state){
    std_msgs::Float64 pub_data[NUM_DOF_W];
    Vec18 motor_q;
    for(int x; x < NUM_DOF_W; ++x)
        motor_q(x) = lowCmd->motorCmd[x].q;

    motor_q(1) = -motor_q(1);
    motor_q(2) = -motor_q(2);

    motor_q(4) = -motor_q(4);
    motor_q(5) = -motor_q(5);

    motor_q(7) = -motor_q(7);
    motor_q(8) = -motor_q(8);

    motor_q(10) = -motor_q(10);
    motor_q(11) = -motor_q(11);

    motor_q(13) = -motor_q(13);
    motor_q(14) = -motor_q(14);

    motor_q(16) = -motor_q(16);
    motor_q(17) = -motor_q(17);

    Vec36 motor_cmd_q_36;
    motor_cmd_q_36 = vec18ToVec36(motor_q);
    double radd;
    radd = 3.1415926/180;


    // std::cout<<"pub_data_msg:  \n"<< pub_data_msg * 180/3.1415926 <<std::endl;

    if( wait_count >= 150 )
    {
        #if ONLY_POSITION_CTRL == true
        motor_cmd_q_36.block<3,1>(0,1)=___dataUnuProtect[1].sendDataConPro(1,motor_cmd_q_36.block<3,1>(0,1),20*radd);
        motor_cmd_q_36.block<3,1>(0,0)=___dataUnuProtect[6].sendDataConPro(0,motor_cmd_q_36.block<3,1>(0,0),20*radd);
        motor_cmd_q_36.block<3,1>(0,2)=___dataUnuProtect[2].sendDataConPro(2,motor_cmd_q_36.block<3,1>(0,2),20*radd);
        motor_cmd_q_36.block<3,1>(0,3)=___dataUnuProtect[3].sendDataConPro(3,motor_cmd_q_36.block<3,1>(0,3),20*radd);
        motor_cmd_q_36.block<3,1>(0,4)=___dataUnuProtect[4].sendDataConPro(4,motor_cmd_q_36.block<3,1>(0,4),20*radd);
        motor_cmd_q_36.block<3,1>(0,5)=___dataUnuProtect[5].sendDataConPro(5,motor_cmd_q_36.block<3,1>(0,5),20*radd);
        #endif
        
        for(int m(0); m < NUM_DOF_W; ++m){
            #if ONLY_POSITION_CTRL == true
            pub_data[m].data = motor_cmd_q_36(m);
            #else
            pub_data[m].data = lowCmd->motorCmd[m].tau;
            #endif
        }
        //旋转方向调整->t
        pub_data[0].data = -pub_data[0].data; 
        pub_data[1].data = -pub_data[1].data; 
        pub_data[5].data = -pub_data[5].data; 
        pub_data[6].data = -pub_data[6].data; 
        pub_data[8].data = -pub_data[8].data; 
        pub_data[10].data = -pub_data[10].data; 
        pub_data[12].data = -pub_data[12].data; 
        pub_data[14].data = -pub_data[14].data; 
        pub_data[16].data = -pub_data[16].data; 

        Vec36 pub_data_msg;
        for(int x; x < NUM_DOF_W; ++x)
            pub_data_msg(x) = lowCmd->motorCmd[x].q;

        pub_data_msg(1) = -pub_data_msg(1);
        pub_data_msg(2) = -pub_data_msg(2);

        pub_data_msg(4) = -pub_data_msg(4);
        pub_data_msg(5) = -pub_data_msg(5);

        pub_data_msg(7) = -pub_data_msg(7);
        pub_data_msg(8) = -pub_data_msg(8);

        pub_data_msg(10) = -pub_data_msg(10);
        pub_data_msg(11) = -pub_data_msg(11);

        pub_data_msg(13) = -pub_data_msg(13);
        pub_data_msg(14) = -pub_data_msg(14);

        pub_data_msg(16) = -pub_data_msg(16);
        pub_data_msg(17) = -pub_data_msg(17);

        pub_data_msg(0) = -pub_data_msg(0);
        pub_data_msg(1) = -pub_data_msg(1);
        pub_data_msg(5) = -pub_data_msg(5);
        pub_data_msg(6) = -pub_data_msg(6);
        pub_data_msg(8) = -pub_data_msg(8);
        pub_data_msg(10) = -pub_data_msg(10);
        pub_data_msg(12) = -pub_data_msg(12);
        pub_data_msg(14) = -pub_data_msg(14);
        pub_data_msg(16) = -pub_data_msg(16);

        #if ONLY_POSITION_CTRL == true
        ___dataUnuProtect[6].velLimAndDifFroDesPosAndActPos(0, 3,
                                                        pub_data_msg.block<3, 1>(0, 0), 
                                                        sub_joint_p_local_temp_origin.block<3, 1>(0, 0), 30 * radd,
                                                        sub_joint_v_local_temp.block<3, 1>(0, 0), 9);
        ___dataUnuProtect[1].velLimAndDifFroDesPosAndActPos(1,3,
                                                        pub_data_msg.block<3, 1>(0, 1), 
                                                        sub_joint_p_local_temp_origin.block<3, 1>(0, 1), 30 * radd,
                                                        sub_joint_v_local_temp.block<3, 1>(0, 1), 9);
        ___dataUnuProtect[2].velLimAndDifFroDesPosAndActPos(2, 3,
                                                        pub_data_msg.block<3, 1>(0, 2), 
                                                        sub_joint_p_local_temp_origin.block<3, 1>(0, 2), 30 * radd,
                                                        sub_joint_v_local_temp.block<3, 1>(0, 2), 9);
        ___dataUnuProtect[3].velLimAndDifFroDesPosAndActPos(3, 3,
                                                        pub_data_msg.block<3, 1>(0, 3), 
                                                        sub_joint_p_local_temp_origin.block<3, 1>(0, 3), 30 * radd,
                                                        sub_joint_v_local_temp.block<3, 1>(0, 3), 9);
        ___dataUnuProtect[4].velLimAndDifFroDesPosAndActPos(4, 3,
                                                        pub_data_msg.block<3, 1>(0, 4), 
                                                        sub_joint_p_local_temp_origin.block<3, 1>(0, 4), 30 * radd,
                                                        sub_joint_v_local_temp.block<3, 1>(0, 4), 9);
        ___dataUnuProtect[5].velLimAndDifFroDesPosAndActPos(5, 3,
                                                        pub_data_msg.block<3, 1>(0, 5), 
                                                        sub_joint_p_local_temp_origin.block<3, 1>(0, 5), 30 * radd,
                                                        sub_joint_v_local_temp.block<3, 1>(0, 5), 9);
        #endif
        // 如果有false,那么pub_data就不会执行
        if (___dataUnuProtect[5].diff_val_flag == false or ___dataUnuProtect[4].diff_val_flag == false 
        or ___dataUnuProtect[3].diff_val_flag == false or ___dataUnuProtect[2].diff_val_flag == false 
        or ___dataUnuProtect[1].diff_val_flag == false or ___dataUnuProtect[6].diff_val_flag == false)
        {
                ___dataUnuProtect[5].diff_val_flag = false;
                ___dataUnuProtect[4].diff_val_flag = false;
                ___dataUnuProtect[3].diff_val_flag = false;
                ___dataUnuProtect[2].diff_val_flag = false;
                ___dataUnuProtect[1].diff_val_flag = false;
                ___dataUnuProtect[6].diff_val_flag = false;
                // printf("\n   ------------diff_val_flag:%d --------------\n",___dataUnuProtect[5].diff_val_flag);
                exit(0);
        }
        else if(___dataUnuProtect[5].vel_lim_flag==false or ___dataUnuProtect[4].vel_lim_flag==false 
        or ___dataUnuProtect[3].vel_lim_flag==false or ___dataUnuProtect[2].vel_lim_flag==false 
        or ___dataUnuProtect[1].vel_lim_flag==false or ___dataUnuProtect[6].vel_lim_flag==false)
        {
                ___dataUnuProtect[5].vel_lim_flag=false;___dataUnuProtect[4].vel_lim_flag=false;
                ___dataUnuProtect[3].vel_lim_flag=false;___dataUnuProtect[2].vel_lim_flag=false;
                ___dataUnuProtect[1].vel_lim_flag=false;___dataUnuProtect[6].vel_lim_flag=false;
                // printf("\n   ------------vel_lim_flag:%d --------------\n",___dataUnuProtect[5].vel_lim_flag);
                exit(0);
        }
        else
        {
            for(int m(0); m < NUM_DOF_W; ++m){
                pub_joint_cmd[m].publish(pub_data[m]);
            }
        }
    }
    wait_count++;

    ros::spinOnce();
}

void IOROS::recvState(LowlevelState *state){
    for(int i(0); i < NUM_DOF_W; ++i){
        state->motorState[i].q = sub_joint_p_local(i);
        state->motorState[i].dq = sub_joint_v_local(i);
        state->motorState[i].tauEst = sub_joint_t_local(i);
    }
    for(int i(0); i < 3; ++i){
        state->imu.accelerometer[i] = sub_imu_lin_a_local(i);
        state->imu.gyroscope[i] = sub_imu_ang_v_local(i);
    }

    //lcc 20240827: 重大发现！！！
    state->imu.gyroscope[0] = state->imu.gyroscope[0];
    state->imu.gyroscope[1] = state->imu.gyroscope[1];
    // state->imu.gyroscope[0] = 0;
    // state->imu.gyroscope[1] = 0;



    state->imu.quaternion[0] = sub_imu_orie_local[3];
    state->imu.quaternion[1] = sub_imu_orie_local[0];
    state->imu.quaternion[2] = sub_imu_orie_local[1];
    state->imu.quaternion[3] = sub_imu_orie_local[2];
    


    state->imu.quaternion[0] = sub_imu_orie_local[3] + quaternion_offset(0);
    state->imu.quaternion[1] = sub_imu_orie_local[0] + quaternion_offset(1);
    state->imu.quaternion[2] = sub_imu_orie_local[1] + quaternion_offset(2);
    state->imu.quaternion[3] = sub_imu_orie_local[2] + quaternion_offset(3);


}