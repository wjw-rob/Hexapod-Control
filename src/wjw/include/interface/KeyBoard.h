 
#ifndef KEYBOARD_H
#define KEYBOARD_H

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include "interface/CmdPanel.h"
#include "common/mathTools.h"

extern bool KEY_M;
extern bool USVLCC_SETZERO;
extern bool FORCE_PROTECT_CHANGE;
extern bool DTAT_SAVE2TXT;

extern Vec3 RPY_DES;
extern Vec3 POS_WORLD_DES;
extern Vec3 VEL_WORLD_DES;

#include <mutex>
#include <thread>

extern std::mutex MTX_IMU;
extern std::mutex MTX_MOTORCMD;
extern std::mutex MTX_MOTORCMD_2;
extern std::mutex MTX_MOTORSTATES;

extern std::mutex MTX_SPICMD;
extern std::mutex MTX_SPIREC;

//继承自 CmdPanel 类，实现一个基于键盘输入的命令面板，用于接收用户的键盘输入并将其转换为机器人的控制命令
class KeyBoard : public CmdPanel{
public:
    KeyBoard();
    ~KeyBoard();
private:
    static void* runKeyBoard(void *arg);//这是一个静态成员函数，通常用于创建线程时作为线程的入口函数
    void* run(void *arg);
    UserCommand checkCmd();//用于检查用户输入的命令，并将其转换为 UserCommand 类型的对象
    void changeValue();//根据用户输入更改某些参数的值
    void changeFunctionModeValue(); // lcc 20250601 更改机器人的功能模式

    pthread_t _tid;//用于存储线程的标识符
    float sensitivityLeft = 0.05;//分别表示左右方向的灵敏度，用于控制机器人在左右方向上的运动速度
    float sensitivityRight = 0.05;
    struct termios _oldSettings, _newSettings;//是一个用于存储终端设置的结构体
    fd_set set;//文件描述符集合，用于 select 函数，用于监听键盘输入的文件描述符是否有数据可读
    int res;//用于存储系统调用的返回值
    int ret;//用于存储系统调用的返回值
    char _c;//用于存储从键盘读取的字符
};

#endif  // KEYBOARD_H