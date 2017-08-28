/************************************************************************
* Copyright(c) 2017  stardraw
* All rights reserved.
*
* File:	public_inc.h
* Brief: 声明全局变量
* Version: 1.0
* Author: Yu Yiqi
* Email: zju_yyq@163.com
* Date:	2017/2/17 9:05
* History:
************************************************************************/

#ifndef STARDRAW_FORKLIFT_PUBLIC_INC_H
#define STARDRAW_FORKLIFT_PUBLIC_INC_H

#include "../pub_inc.h"
#include "../location_base/LocationDetector.h"      //需要放在引用controlcan.h的头文件之前,否则opencv里面的定义会和controlcan.h的冲突
#include "../motor_control/motor_control.h"
#include "../multi_thread/Multi_thread.h"
#include "../detect/detect.h"


class Forklift{
public:
    Forklift(map_state _m,working_state _w);
    void start();
public:
    Multi_thread mthread;
    LocationDetector m_locationDetector;
    Detector detector;
    MotorControl motor;

    struct traj_point next_point;
    struct position curr_pos;
    struct inter_point trajectory[1000];
	
    int station_s;
    int station_g;    
	
    enum working_state cur_work_state;
    enum action m_action;
    enum map_state _map_state;

};
#endif //STARDRAW_FORKLIFT_PUBLIC_INC_H
