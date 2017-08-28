/************************************************************************
* Copyright(c) 2017  stardraw
* All rights reserved.
*
* File:	Multi_thread.h
* Brief: 多线程
* Version: 1.0
* Author: Yu Yiqi
* Email: zju_yyq@163.com
* Date:	2017/2/17 9:05
* History:
************************************************************************/
#ifndef STARDRAW_FORKLIFT_MULTI_THREAD_H
#define STARDRAW_FORKLIFT_MULTI_THREAD_H

#include <iostream>
#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include "../pub_inc.h"
#include "../motor_control/motor_control.h"
#include "../detect/detect.h"

#define PORT "3333"
#define IP_SERVER "10.10.2.103"
#define WIFI_SERVER "192.168.1.103"
//#define WIFI_SERVER "192.168.1.100"
#define MAXSLEEP 64
#define BUFSIZE 1024

class Multi_thread{
public:
    Multi_thread();
    void start();
    static void *socket_tx_thread(void *);
    static void *socket_rx_thread(void *);
    void socket_Init();
    int connect_retry(int domain,int type,int protocol,const struct sockaddr *addr, socklen_t alen);

public:
    struct Info m_Info;
    struct Cmd m_Cmd;

private:
    struct addrinfo *ailist,*aip;
    struct addrinfo hint;

    bool sock_state;
    int sockfd,err;

//    bool isObstacle;
};

#endif //STARDRAW_FORKLIFT_MULTI_THREAD_H
