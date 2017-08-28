/************************************************************************
* Copyright(c) 2017  stardraw
* All rights reserved.
*
* File:	Communication.h
* Brief: 通信类，包含两个socket线程
* Version: 1.0
* Author: Yu Yiqi
* Email: zju_yyq@163.com
* Date:	2017/3/2 10:05
* History:
************************************************************************/

#ifndef TERMINAL_COMMUNICATION_H
#define TERMINAL_COMMUNICATION_H

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
#include <arpa/inet.h>

#include "../../include/pub_inc.h"
#include "../../include/Socket/Socket.h"

#define PORT "3333"
#define IP "10.10.2.103"
#define WIFI_IP "192.168.0.119"
#define MAXSLEEP 64
#define BUFSIZE 1024
#define QLEN 10

class Listener{
public:
    Listener();
    void start();
    static void *monitor(void *);

    void socket_Init();
    int initserver(int type,const struct sockaddr* addr,socklen_t alen,int qlen);

    shared_ptr<Socket> m_socket[MAX_FORK];
    //Socket *m_socket[5];
private:

    void *tret_s;
    void *tret_m;

    struct addrinfo *ailist,*aip;
    struct addrinfo hint;

    bool sock_state;
    int sockfd,err,new_fd;
    socklen_t sin_size;

    int connect_num;                    //记录叉车连接数量
};

#endif //TERMINAL_COMMUNICATION_H
