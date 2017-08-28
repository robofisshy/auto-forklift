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
#include "jetsonGPIO.h"

//BYTE forward[7]={0x30,0x0,0x0,0x0,0x01,0x03,0x0};
//BYTE backward[7]={0x30,0x0,0x0,0x0,0x02,0x03,0x0};
//BYTE up[7]={0x0,0x0,0x0,0x20,0x10,0x03,0x0};
//BYTE down[7]={0x0,0x0,0x0,0x20,0x20,0x03,0x0};

#define DA_VREF 2.5

enum move_action{
    _forward=0,
    _backward
};
enum carry_action{
    _stop=6,
    _down=7,
    _up=8
};

class MotorControl{
    friend class Multi_thread;
public:
    MotorControl();
    bool GPIO_Init();
    void DA_convert(unsigned int Data);
    bool Turn(float angle_dst);
    void close();
public:

    jetsonTX1GPIONumber DA_CLK;
    jetsonTX1GPIONumber DA_CS;
    jetsonTX1GPIONumber DA_DIN;

/****** obstacle_detect radar ******/
    jetsonTX1GPIONumber WARN0;
    jetsonTX1GPIONumber WARN1;
    jetsonTX1GPIONumber DANGER;

    jetsonTX1GPIONumber Area_set0;
    jetsonTX1GPIONumber Area_set1;
public:
    move_action m_move_action;
    carry_action m_carry_action;
};

#endif //MOVE_CONTROL_CMD_FORKLIFT_H
