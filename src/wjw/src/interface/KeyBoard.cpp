#include "interface/KeyBoard.h"
// #include "interface/ "
#include <iostream>
// #include "interface/IOSDK.h"

bool KEY_M = false;
bool USVLCC_SETZERO = false;
bool FORCE_PROTECT_CHANGE = false;//力保护的启停？true的时候在力控上
bool DTAT_SAVE2TXT = false;

std::mutex MTX_MOTORCMD;
std::mutex MTX_MOTORSTATES;
std::mutex MTX_IMU;

std::mutex MTX_SPICMD;
std::mutex MTX_SPIREC;

Vec3 RPY_DES;
Vec3 POS_WORLD_DES;
Vec3 VEL_WORLD_DES;


KeyBoard::KeyBoard(){
    userCmd = UserCommand::NONE;
    userValue.setZero();

    tcgetattr( fileno( stdin ), &_oldSettings );//调用 tcgetattr 函数获取标准输入（键盘）的当前终端设置，并将其保存到 _oldSettings 结构体中
    _newSettings = _oldSettings;//将 _oldSettings 的内容复制到 _newSettings 中，以便后续进行修改
    _newSettings.c_lflag &= (~ICANON & ~ECHO);//修改 _newSettings 中的本地标志位 c_lflag。~ICANON 表示关闭规范模式，使得输入不需要等待换行符就可以被处理；~ECHO 表示关闭回显功能，即输入的字符不会在屏幕上显示出来
    tcsetattr( fileno( stdin ), TCSANOW, &_newSettings );//调用 tcsetattr 函数将修改后的终端设置 _newSettings 应用到标准输入上，TCSANOW 表示立即生效

    /*pthread_create 是一个 POSIX 线程库函数，用于创建一个新的线程。
    &_tid：传入线程标识符的地址，线程创建成功后，线程的标识符会被存储在 _tid 中。
    NULL：线程的属性设置，这里使用默认属性。
    runKeyBoard：线程的入口函数，该函数会在新线程中执行。
    (void*)this：传递给线程入口函数的参数，将当前 KeyBoard 对象的指针作为参数传递给 runKeyBoard 函数，以便在该函数中可以访问 KeyBoard 对象的成员*/
    pthread_create(&_tid, NULL, runKeyBoard, (void*)this);
    //打印出一系列可用的命令选项，提示用户可以通过输入相应的字符或数字来选择不同的命令
    printf(" KeyBoard checkCmd:\n"
           " 1->PASSIVE_1       ( ----1 阻尼模式---- );\n"
           " 2->FIXEDSTAND_2    ( ----2 固定站立模式命令---- );\n"
           " c->FIXEDSQUAT_c    ( ----c 蹲下命令---- );\n"
           " 3->FREESTAND_3     ( ----3 自由站立模式命令 (不要用)---- );\n"
           " 4->QP_4            ( ----4 QP控制命令--- );\n"
           " 5->POSITION_5      ( ----5 位置控制命令---- );\n"
           " 6->A1MPC_6(Ban);\n"
           " 7->POSREFLEX_7(Ban);\n"
           " 8->FORCE_POS       ( ----8 力位混合控制命令---- );\n"
           " 9->SWING_TEST9;\n"
           " 0->MPC_FOREC_POS0  ( ----0 MPC+力位混合控制命令(不好用)---- )\n");
    //打印出地形估计相关的信息
    printf(" TERRIANESTI_FOURLEG: %d \n",TERRIANESTI_FOURLEG);
    
}

KeyBoard::~KeyBoard(){
    pthread_cancel(_tid);//pthread_cancel 是 POSIX 线程库中的一个函数，用于向指定的线程发送取消请求
    pthread_join(_tid, NULL);//pthread_join 函数用于等待指定线程的结束，并回收其资源
    tcsetattr( fileno( stdin ), TCSANOW, &_oldSettings );
}

UserCommand KeyBoard::checkCmd(){
    // printf("\n-a-gf-asg-\n");
    switch (_c){
    case '1':
        return UserCommand::PASSIVE_1;
    case 'c':
        FORCE_PROTECT_CHANGE = false;
        return UserCommand::SQUAT_C;
    case '2':
        FORCE_PROTECT_CHANGE = false;
        return UserCommand::FIXEDSTAND_2;
    case '3':
        FORCE_PROTECT_CHANGE = false;
        return UserCommand::FREESTAND_3;
    #if USE_A_REAL_HEXAPOD == false
    case '4':
        // printf(" \n keyboard->QP_4  \n ");
        return UserCommand::QP_4;
    #endif
    case '5':
        FORCE_PROTECT_CHANGE = false;
        return UserCommand::POSITION_5;

    #ifdef COMPILE_WITH_MOVE_BASE//（这个宏是在哪定义的？）
        case '5':
            return UserCommand::L2_Y;
    #endif  // COMPILE_WITH_MOVE_BASE
    #if USE_A_REAL_HEXAPOD == false
    case '6':
        return UserCommand::A1MPC_6;
    #endif;
    // case '7':
    //     return UserCommand::POSREFLEX_7;
    // case '0':
    //     return UserCommand::BALANCE_TEST0;
    case '0':
        FORCE_PROTECT_CHANGE = true;
        return UserCommand::MPC_FORCE_POS_0;
    case '9':
        return UserCommand::SWING_TEST9;
    // case '8':
    //     return UserCommand::SETP_TEST8;
    case '8':
        FORCE_PROTECT_CHANGE = true;
        // printf(" FORCE_PROTECT_CHANGE:%d\n",FORCE_PROTECT_CHANGE);
        return UserCommand::FORCE_POS_8;
    case ' ':
        {   
            USVLCC_SETZERO = true;
            // userValue.LTsetZero = true;
            // userValue.setZero();
            // printf(" space\n");
        }
        return UserCommand::NONE;
    default:
        return UserCommand::NONE;
    }
}

void KeyBoard::changeValue(){
    switch (_c){
    // case 'w':case 'W':
    case 'w':
        userValue.ly = min<float>(userValue.ly+sensitivityLeft, 1.0);
        userValue_lcc.ly = min<float>(userValue_lcc.ly+sensitivityLeft, 1.0);
    break;
    // case 's':case 'S':
    case 's':
        userValue.ly = max<float>(userValue.ly-sensitivityLeft, -1.0);
        userValue_lcc.ly = max<float>(userValue_lcc.ly-sensitivityLeft, -1.0);
        break;
    // case 'd':case 'D':
    case 'd':
        userValue.lx = min<float>(userValue.lx+sensitivityLeft, 1.0);
        userValue_lcc.lx = min<float>(userValue_lcc.lx+sensitivityLeft, 1.0);
        break;
    // case 'a':case 'A':
    case 'a':
        userValue.lx = max<float>(userValue.lx-sensitivityLeft, -1.0);
        userValue_lcc.lx = max<float>(userValue_lcc.lx-sensitivityLeft, -1.0);
        break;

    // case 'i':case 'I':
    case 'i':
        userValue.ry = min<float>(userValue.ry+sensitivityRight, 1.0);
        userValue_lcc.ry = min<float>(userValue_lcc.ry+sensitivityRight, 1.0);
        break;
    // case 'k':case 'K':
    case 'k':
        userValue.ry = max<float>(userValue.ry-sensitivityRight, -1.0);
        userValue_lcc.ry = max<float>(userValue_lcc.ry-sensitivityRight, -1.0);
        break;
    // case 'l':case 'L':
    case 'l':
        userValue.rx = min<float>(userValue.rx+sensitivityRight, 1.0);
        userValue_lcc.rx = min<float>(userValue_lcc.rx+sensitivityRight, 1.0);
        break;
    // case 'j':case 'J':
    case 'j':
        userValue.rx = max<float>(userValue.rx-sensitivityRight, -1.0);
        userValue_lcc.rx = max<float>(userValue_lcc.rx-sensitivityRight, -1.0);
        break;
    default:
        break;
    }
}

// lcc 20250601
int life_reaction_off_on_flag=0,dowm_reaction_off_on_flag=0,mkan_reaction_off_on_flag=0, berzier_shape_off_on_flag = 0;
float raddd = 3.14159/180;
void KeyBoard::changeFunctionModeValue(){
    switch (_c){
        case 'M':case 'm':{ //进入闭环
        if( KEY_M == false )
            KEY_M = true;
        else if( KEY_M == true )
            KEY_M = false;
        std::cout<<"KEY_M:  "<< KEY_M <<std::endl;
        }
        break; 
        #if USE_A_REAL_HEXAPOD == true   //以下为真实下与电机有关的操作（wjw）
        case 'p':case 'P':{ //进入闭环
                // if( userFunctionMode.motor_enable_flag == false )
                //     userFunctionMode.motor_enable_flag = true;
                // else if( userFunctionMode.motor_enable_flag == true )
                //     userFunctionMode.motor_enable_flag = false;
                // std::cout<<"motor_enable_flag:  "<< userFunctionMode.motor_enable_flag <<std::endl;

                MOTOR_ENABLE_FLAG = true;
                MOTOR_DISABEL_FLAG = false;
            }
            break;
        case 'o':case 'O':{ //退出闭环
                // std::cout<<"getQ_Hex:  \n"<< _ctrlComp->lowState->getQ_Hex() * 180/3.1415926 <<std::endl;
                // spi_2.exit_close_loop();

                MOTOR_DISABEL_FLAG = true;
                MOTOR_DATA_LOAD = false;

                // if( userFunctionMode.motor_disenable_flag == false )
                //     userFunctionMode.motor_disenable_flag = true;
                // else if( userFunctionMode.motor_disenable_flag == true )
                //     userFunctionMode.motor_disenable_flag = false;
                // std::cout<<"motor_disenable_flag:  "<< userFunctionMode.motor_disenable_flag <<std::endl;
            }
            break;
        case '[':case '{':{ //电机连续加载数据
                MOTOR_READY_FLAG = true;
                MOTOR_DATA_LOAD = false;
                MOTOR_DISABEL_FLAG = false;
            }
            break;
        case ']':case '}':{ //进入程序算法，点击获得程序的控制数据
                MOTOR_DATA_LOAD = true;
                MOTOR_READY_FLAG = false;
                printf(" \n  ----------------- fsm_run -------------------- \n ");
            }
            break;
        case '-':case '_':{ //退出闭环
                DOU_DONG_ANGEL = DOU_DONG_ANGEL -1 * raddd;
                if( DOU_DONG_ANGEL >= 2.5 * raddd)
                    DOU_DONG_ANGEL = 2.5 * raddd;
                else if( DOU_DONG_ANGEL <= -2.5 * raddd )
                    DOU_DONG_ANGEL = -2.5 * raddd;
            }
            break;
        case '=':case '+':{ //退出闭环
                DOU_DONG_ANGEL = DOU_DONG_ANGEL +1 * raddd;
                if( DOU_DONG_ANGEL >= 2.5 * raddd)
                    DOU_DONG_ANGEL = 2.5 * raddd;
                else if( DOU_DONG_ANGEL <= -2.5 * raddd )
                    DOU_DONG_ANGEL = -2.5 * raddd;
            }
            break;
        case 't':case 'T':{

                if( DTAT_SAVE2TXT == false )
                    DTAT_SAVE2TXT = true;
                else if( DTAT_SAVE2TXT == true )
                    DTAT_SAVE2TXT = false;
                std::cout<<"DTAT_SAVE2TXT:  "<< DTAT_SAVE2TXT <<std::endl;

                // if( TEST_FLAG == false )
                //     TEST_FLAG = true;
                // else if( TEST_FLAG == true )
                //     TEST_FLAG = false;

                // if( userFunctionMode.function_test == false )
                //     userFunctionMode.function_test = true;
                // else if( userFunctionMode.function_test == true )
                //     userFunctionMode.function_test = false;
                // std::cout<<"function_test:  "<< userFunctionMode.function_test <<std::endl;
                // if( userFunctionMode.state_reset == false )
                //     userFunctionMode.state_reset = true;
                // else if( userFunctionMode.state_reset == true )
                //     userFunctionMode.state_reset = false;
                // std::cout<<"state_reset:  "<< userFunctionMode.state_reset <<std::endl;
        }
        break;
        #endif
        /******************20230906自适应cheet按键 现在报废了（wjw） ******************/
        #if PCONTROL_REFLEX_LIFE_DOWM == true//通过按键手动控制
        case 'Q':   {   // lf
                    if (userFunctionMode.life_reaction_off_on==1){
                        if ( userFunctionMode.LEG_LIFT_TRIGGER(1) == 0 )
                            userFunctionMode.LEG_LIFT_TRIGGER(1)=1;
                        else if ( userFunctionMode.LEG_LIFT_TRIGGER(1) == 1 )
                            userFunctionMode.LEG_LIFT_TRIGGER(1)=0;
                        printf("\n LEG_LIFT_TRIGGER 1:%f \n",userFunctionMode.LEG_LIFT_TRIGGER(1));
                    }
                    if (userFunctionMode.dowm_reaction_off_on==1){
                        if ( userFunctionMode.LEG_DOWM_TRIGGER(1) == 0 )
                            userFunctionMode.LEG_DOWM_TRIGGER(1)=1;
                        else if ( userFunctionMode.LEG_DOWM_TRIGGER(1) == 1 )
                            userFunctionMode.LEG_DOWM_TRIGGER(1)=0;
                        printf("\n LEG_DOWM_TRIGGER 1:%f \n",userFunctionMode.LEG_DOWM_TRIGGER(1));
                    }
                }
                break;
        case 'W':   {  // rf
                    if (userFunctionMode.life_reaction_off_on==1){
                        if ( userFunctionMode.LEG_LIFT_TRIGGER(0) == 0 )
                            userFunctionMode.LEG_LIFT_TRIGGER(0)=1;
                        else if ( userFunctionMode.LEG_LIFT_TRIGGER(0) == 1 )
                            userFunctionMode.LEG_LIFT_TRIGGER(0)=0;
                        printf("\n LEG_LIFT_TRIGGER 0:%f \n",userFunctionMode.LEG_LIFT_TRIGGER(0));
                    }
                    if (userFunctionMode.dowm_reaction_off_on==1){
                        if ( userFunctionMode.LEG_DOWM_TRIGGER(0) == 0 )
                            userFunctionMode.LEG_DOWM_TRIGGER(0)=1;
                        else if ( userFunctionMode.LEG_DOWM_TRIGGER(0) == 1 )
                            userFunctionMode.LEG_DOWM_TRIGGER(0)=0;
                        printf("\n LEG_DOWM_TRIGGER 0:%f \n",userFunctionMode.LEG_DOWM_TRIGGER(0));
                    }
                }
                break;
        case 'A':  {   //lm
                    if (userFunctionMode.life_reaction_off_on==1){
                        if ( userFunctionMode.LEG_LIFT_TRIGGER(3) == 0 )
                            userFunctionMode.LEG_LIFT_TRIGGER(3)=1;
                        else if ( userFunctionMode.LEG_LIFT_TRIGGER(3) == 1 )
                            userFunctionMode.LEG_LIFT_TRIGGER(3)=0;
                        printf("\n LEG_LIFT_TRIGGER 3:%f \n",userFunctionMode.LEG_LIFT_TRIGGER(3));
                    }
                    if (userFunctionMode.dowm_reaction_off_on==1){
                        if ( userFunctionMode.LEG_DOWM_TRIGGER(3) == 0 )
                            userFunctionMode.LEG_DOWM_TRIGGER(3)=1;
                        else if ( userFunctionMode.LEG_DOWM_TRIGGER(3) == 1 )
                            userFunctionMode.LEG_DOWM_TRIGGER(3)=0;
                        printf("\n LEG_DOWM_TRIGGER 3:%f \n",userFunctionMode.LEG_DOWM_TRIGGER(3));
                    }
                }
                break;
        case 'S': {   //rm
                    if (userFunctionMode.life_reaction_off_on==1){
                        if ( userFunctionMode.LEG_LIFT_TRIGGER(2) == 0 )
                            userFunctionMode.LEG_LIFT_TRIGGER(2)=1;
                        else if ( userFunctionMode.LEG_LIFT_TRIGGER(2) == 1 )
                            userFunctionMode.LEG_LIFT_TRIGGER(2)=0;
                        printf("\n LEG_LIFT_TRIGGER 2:%f \n",userFunctionMode.LEG_LIFT_TRIGGER(2));
                    }
                    if (userFunctionMode.dowm_reaction_off_on==1){
                        if ( userFunctionMode.LEG_DOWM_TRIGGER(2) == 0 )
                            userFunctionMode.LEG_DOWM_TRIGGER(2)=1;
                        else if ( userFunctionMode.LEG_DOWM_TRIGGER(2) == 1 )
                            userFunctionMode.LEG_DOWM_TRIGGER(2)=0;
                        printf("\n LEG_DOWM_TRIGGER 2:%f \n",userFunctionMode.LEG_DOWM_TRIGGER(2));
                    }
                }
                break;
        case 'Z':   {   //lb
                    if (userFunctionMode.life_reaction_off_on==1){
                        if ( userFunctionMode.LEG_LIFT_TRIGGER(5) == 0 )
                            userFunctionMode.LEG_LIFT_TRIGGER(5)=1;
                        else if ( userFunctionMode.LEG_LIFT_TRIGGER(5) == 1 )
                            userFunctionMode.LEG_LIFT_TRIGGER(5)=0;
                        printf("\n LEG_LIFT_TRIGGER 5:%f \n",userFunctionMode.LEG_LIFT_TRIGGER(5));
                    }
                    if (userFunctionMode.dowm_reaction_off_on==1){
                        if ( userFunctionMode.LEG_DOWM_TRIGGER(5) == 0 )
                            userFunctionMode.LEG_DOWM_TRIGGER(5)=1;
                        else if ( userFunctionMode.LEG_DOWM_TRIGGER(5) == 1 )
                            userFunctionMode.LEG_DOWM_TRIGGER(5)=0;
                        printf("\n LEG_DOWM_TRIGGER 5:%f \n",userFunctionMode.LEG_DOWM_TRIGGER(5));
                    }
                }
                break;
        case 'X': {   //rb
                    if (userFunctionMode.life_reaction_off_on==1){
                        if ( userFunctionMode.LEG_LIFT_TRIGGER(4) == 0 )
                            userFunctionMode.LEG_LIFT_TRIGGER(4)=1;
                        else if ( userFunctionMode.LEG_LIFT_TRIGGER(4) == 1 )
                            userFunctionMode.LEG_LIFT_TRIGGER(4)=0;
                        printf("\n LEG_LIFT_TRIGGER 4:%f \n",userFunctionMode.LEG_LIFT_TRIGGER(4));
                    }
                    if (userFunctionMode.dowm_reaction_off_on==1){
                        if ( userFunctionMode.LEG_DOWM_TRIGGER(4) == 0 )
                            userFunctionMode.LEG_DOWM_TRIGGER(4)=1;
                        else if ( userFunctionMode.LEG_DOWM_TRIGGER(4) == 1 )
                            userFunctionMode.LEG_DOWM_TRIGGER(4)=0;
                        printf("\n LEG_DOWM_TRIGGER 4:%f \n",userFunctionMode.LEG_DOWM_TRIGGER(4));
                    }
                }
                break;
        case '!':  { // lift_reaction开关  通过按键控制决定是否启用这个反应行为
                    if(life_reaction_off_on_flag==0)
                    {   
                        life_reaction_off_on_flag=1;
                        userFunctionMode.life_reaction_off_on=1;
                    }   
                    else if(life_reaction_off_on_flag==1)
                    {
                        life_reaction_off_on_flag=0;
                        userFunctionMode.life_reaction_off_on=0;
                    }
                    printf("life_reaction_off_on: %d\n ",userFunctionMode.life_reaction_off_on);
                    printf("dowm_reaction_off_on: %d\n ",userFunctionMode.dowm_reaction_off_on);
                    printf("mkan_reaction_off_on: %d\n ",userFunctionMode.mkan_reaction_off_on);
                    std::cout<<" LEG_DOWM_TRIGGER "<<std::endl;
                    std::cout<< userFunctionMode.LEG_DOWM_TRIGGER <<std::endl;
                }
                break;
        case '@': {  //dowm_reaction开关
                    if(dowm_reaction_off_on_flag==0)
                    {   
                        dowm_reaction_off_on_flag=1;
                        userFunctionMode.dowm_reaction_off_on=1;
                    }   
                    else if(dowm_reaction_off_on_flag==1)
                    {
                        dowm_reaction_off_on_flag=0;
                        userFunctionMode.dowm_reaction_off_on=0;
                    }
                    printf("life_reaction_off_on: %d\n ",userFunctionMode.life_reaction_off_on);
                    printf("dowm_reaction_off_on: %d\n ",userFunctionMode.dowm_reaction_off_on);
                    printf("mkan_reaction_off_on: %d\n ",userFunctionMode.mkan_reaction_off_on);
                    std::cout<<" LEG_DOWM_TRIGGER "<<std::endl;
                    std::cout<< userFunctionMode.LEG_DOWM_TRIGGER <<std::endl;
                }
                break;
        case '#': { //dowm_reaction开关
                    if(mkan_reaction_off_on_flag==0)
                    {   
                        mkan_reaction_off_on_flag=1;
                        userFunctionMode.mkan_reaction_off_on=1;
                    }   
                    else if(mkan_reaction_off_on_flag==1)
                    {
                        mkan_reaction_off_on_flag=0;
                        userFunctionMode.mkan_reaction_off_on=0;
                    }

                    printf("life_reaction_off_on: %d\n ",userFunctionMode.life_reaction_off_on);
                    printf("dowm_reaction_off_on: %d\n ",userFunctionMode.dowm_reaction_off_on);
                    printf("mkan_reaction_off_on: %d\n ",userFunctionMode.mkan_reaction_off_on);
                    std::cout<<" LEG_DOWM_TRIGGER "<<std::endl;
                    std::cout<< userFunctionMode.LEG_DOWM_TRIGGER <<std::endl;
                    userFunctionMode.LEG_DOWM_TRIGGER.setZero();
                }
                case '~': {   //全部复原
                    userFunctionMode.LEG_LIFT_TRIGGER.setZero();
                    userFunctionMode.LEG_DOWM_TRIGGER.setZero();
                    userFunctionMode.berzier_shape_off_on=0;
                    userFunctionMode.mkan_reaction_off_on=0;
                    userFunctionMode.dowm_reaction_off_on=0;
                    userFunctionMode.life_reaction_off_on=0;
                    userFunctionMode.set_pitch = 0;
                    printf("life_reaction_off_on: %d\n ",userFunctionMode.life_reaction_off_on);
                    printf("dowm_reaction_off_on: %d\n ",userFunctionMode.dowm_reaction_off_on);
                    printf("mkan_reaction_off_on: %d\n ",userFunctionMode.mkan_reaction_off_on);
                    printf("set_pitch:%f\n",userFunctionMode.set_pitch*3.1415/180);
                }
                case 'v':   {  
                    if(dowm_reaction_off_on_flag==0 || dowm_reaction_off_on_flag==1){   
                        userFunctionMode.set_pitch=userFunctionMode.set_pitch+1*3.1415/180;
                        printf("set_pitch:%f\n",userFunctionMode.set_pitch*3.1415/180);
                    }   
                }
                break;
                case 'b':   {  
                    if(dowm_reaction_off_on_flag==0 || dowm_reaction_off_on_flag==1){
                        userFunctionMode.set_pitch=userFunctionMode.set_pitch-1*3.1415/180;
                        printf("set_pitch:%f\n",userFunctionMode.set_pitch*3.1415/180);  
                    }   
                }
                break;
            #endif
        default:
        break;
    }
}


void* KeyBoard::runKeyBoard(void *arg){
    ((KeyBoard*)arg)->run(NULL);
    return NULL;
}

void* KeyBoard::run(void *arg){
    while(1){
        FD_ZERO(&set);
        FD_SET( fileno( stdin ), &set );

        res = select( fileno( stdin )+1, &set, NULL, NULL, NULL);

        if(res > 0){
            ret = read( fileno( stdin ), &_c, 1 );
            userCmd = checkCmd();
            if(userCmd == UserCommand::NONE)
                changeValue();
            changeFunctionModeValue();// lcc 20250601
            _c = '\0';
        }
        usleep(1000);
    }
    return NULL;
}