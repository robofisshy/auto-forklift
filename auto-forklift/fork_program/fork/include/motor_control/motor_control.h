/************************************************************************
* Copyright(c) 2017  stardraw
* All rights reserved.
*
* File:	motor_Control.h
* Brief: 电机（前进电机和转向电机）控制,by CAN-BUS and DA
* Version: 1.0
* Author: Yu Yiqi
* Email: zju_yyq@163.com
* Date:	2017/2/17 9:05
* History:
************************************************************************/

#ifndef MOVE_CONTROL_CMD_FORKLIFT_H
#define MOVE_CONTROL_CMD_FORKLIFT_H

#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include "controlcan.h"
#include "jetsonGPIO.h"
#include "../pub_inc.h"

#define DA_VREF 2.5
#define UP_VELOCITY_MAX 0.22              //空载起升最大速度:0.22m/s
#define DOWN_VELOCITY_MAX 0.37            //空载下降最大速度:0.37m/s

enum move_action{
    _forward=0,
    _backward
};

class MotorControl{
    friend class Multi_thread;
public:
    MotorControl();
    void start();
    bool Can_Init();
    bool GPIO_Init();
    void DA_convert(unsigned int Data);
    bool Move(float linear_vel);
    bool Turn(float angle_dst);
    bool Carry();
    bool Carrystop();
    void close();
/*** motion control thread ***/
    static void *motion_control_thread(void *);
private:
    VCI_INIT_CONFIG config;
    VCI_CAN_OBJ send;

    jetsonTX1GPIONumber DA_CLK;
    jetsonTX1GPIONumber DA_CS;
    jetsonTX1GPIONumber DA_DIN;

/****** obstacle_detect radar ******/
    jetsonTX1GPIONumber WARN1;
    jetsonTX1GPIONumber WARN2;
    jetsonTX1GPIONumber DANGER;

    jetsonTX1GPIONumber Area_set0;
    jetsonTX1GPIONumber Area_set1;
/*** motion control thread_t ***/
    pthread_t motion_control_thread_t;
public:
    move_action m_move_action;
    carry_action m_carry_action;
    struct speed m_speed;
};

#endif //MOVE_CONTROL_CMD_FORKLIFT_H
