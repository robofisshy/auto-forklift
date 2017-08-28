//
// Created by yyq on 17-5-3.
//

#ifndef TERMINAL_MULTIFORK_SOCKET_H
#define TERMINAL_MULTIFORK_SOCKET_H

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
#include "../pub_inc.h"

class Socket{
public:
    Socket(int _fd,char *_addr,struct addrinfo *_aip);
    //static void *socket_tx_thread(void *);
    //static void *socket_rx_thread(void *);
    //void *socket_tx_thread();
    //void *socket_rx_thread();

public:
    int fd;
    char *addr;
    struct addrinfo *aip;
    socklen_t sin_size;
};
#endif //TERMINAL_MULTIFORK_SOCKET_H
