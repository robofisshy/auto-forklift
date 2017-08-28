/************************************************************************
* Copyright(c) 2017  stardraw
* All rights reserved.
*
* File:	pubinc.h
* Brief: 声明公用数据和函数
* Version: 1.0
* Author: Yu Yiqi
* Email: zju_yyq@163.com
* Date:	2017/2/17 9:05
* History:
************************************************************************/

#ifndef STARDRAW_FORKLIFT_PUB_INC_H
#define STARDRAW_FORKLIFT_PUB_INC_H

#include <iostream>
#include <vector>
#include <boost/thread.hpp>

using namespace std;

#define pi 3.141592653

#define edge_x 50.0       //定义电子地图上x、y轴的计量单位
#define edge_y 30.0

/*** signal函数，检测到Ctrl+C，跳出循环，安全退出程序 ***/
extern bool quit;				    //quit flag	
extern bool isObstacle;
extern bool noMarker;				    //no Marker flag	
extern double lift_encoder;                         //拉线编码器返回值
extern double lift_velocity;                        //货叉速度
enum carry_action{
    _stop=6,
    _down=7,
    _up=8
};
/*** obstacle_detect ***/
enum alarm_state{
    SAFE=3,
    WARN=4,
    DANGER=5
};
extern enum alarm_state forward_alarm;
extern enum alarm_state back_alarm;
extern enum carry_action carry;
extern void handle_signal(int sig);

extern boost::shared_mutex poseupdate;
extern boost::shared_mutex readCMD;

/*** 实际坐标值 ***/
struct position{
    double x;
    double y;
    double theta;
};

struct pose{
    position position_;
    long time;
};

extern struct pose currentPosition;

/*** 每一段前进方向 ***/
enum direction{
    Straight=12,
    Right,
    Left
};
/*** 全局路径规划生成
 * s_p：每一段起点 g_p：每一段终点 flag：每一段的方向 ***/
struct traj_point{
    struct position s_p;
    struct position g_p;
    enum direction flag;
};
/*** 每一段进行差值
 * 得到x、y标，速度***/
struct inter_point{
    double x;
    double y;
    double v;
};
/*** 叉车行为 ***/
enum action{
    CEASE=40,
    PREPARE,
    MOVE
};
/*** 叉车状态 ***/
enum working_state{
    STOP=30,
    STANDBY,
    WORKING,
    ARRIVE
};
/*** 地图状态 ***/
enum map_state{
    BLANK=9,
    UNDONE=10,
    COMPLETE=11
};
/*** 控制台状态 ***/
enum terminal_state{
    WAITING=20,
    MAPPING
};
/*** 叉车上传信息 ***/
struct Info{
    struct position curr_pos;
    enum working_state curr_work_state;
    struct position next_point;
};
/*** 终端下发命令 ***/
struct Cmd{
    struct inter_point traj[1000];
    enum action fork_action;
    enum map_state map_state;
    int station_s;
    int station_g;
};
/*** 速度 ***/
struct speed{
    double v;
    double w;
};
/*** security_state ***/
struct Security{
    char radar_forward;
    char radar_backward;
    char cl_switch;
    char buf[10];
};
/*** lift pid parameter***/
struct pid{
    double err;
    double e_sum;
    double e_last;
    double e_pre;
    double p;
    double i;
    double d;
};
extern map<int,double> station_height;
#endif //STARDRAW_FORKLIFT_PUB_INC_H
