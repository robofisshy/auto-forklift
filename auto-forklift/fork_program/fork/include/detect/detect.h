//
// Created by yyq on 17-4-18.
//

#ifndef DETECT_DETECT_H
#define DETECT_DETECT_H
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "../pub_inc.h"

class Detector{
public:
    Detector();
    void start();
    static void *sensor_detect_thread(void*);
    static void *back_detect_thread(void*);
    void openUart();
public:
    struct Security _s;
    enum alarm_state _a;   
    bool isSafe;
    int i2c_fd;
    int uart_fd;
};
#endif //DETECT_DETECT_H
