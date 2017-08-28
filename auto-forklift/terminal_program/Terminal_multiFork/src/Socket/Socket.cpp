//
// Created by yyq on 17-5-3.
//
#include "../../include/Socket/Socket.h"

void *midCall(void *ptr);

struct Info fork_Info[MAX_FORK];
struct Cmd Terminal_Cmd[MAX_FORK];
struct pipe_info{
    int fd;                 //socket通信文件符
    int num;                //叉车编号
    struct addrinfo *aip;   //地址信息
    socklen_t sin_size;     //struct sockaddr 结构体长度 accept()函数使用
};
struct pipe_info m_info;

pthread_t _tx_t[MAX_FORK];
pthread_t _rx_t[MAX_FORK];
void *socket_rx_thread(void *tmp);
void *socket_tx_thread(void *tmp);
Socket::Socket(int _fd,char *_addr,struct addrinfo *_aip)
{
    fd=_fd;
    addr=_addr;
    aip=_aip;
    sin_size = sizeof(struct sockaddr);
    int size=sizeof(Terminal_Cmd[0]);
    socklen_t optlen=sizeof(Terminal_Cmd[0]);
    setsockopt(fd,SOL_SOCKET,SO_RCVBUF, (char *)& size, optlen);

    Socket *tx=this;
    Socket *rx=this;

    m_info.fd=_fd;
    m_info.num=fork_address[addr];
    m_info.aip=aip;
    m_info.sin_size=sin_size;

    /**** 创建socket发送和接收线程 ****/
    pthread_create(&_rx_t[m_info.num], NULL, socket_rx_thread,&m_info);
    pthread_create(&_tx_t[m_info.num], NULL, socket_tx_thread,&m_info);
    //boost::thread socket_rx(boost::bind(socket_rx_thread,(void*)&m_info));
    //boost::thread socket_tx(boost::bind(socket_tx_thread,(void*)&m_info));
}

void *socket_rx_thread(void *tmp)
{
    struct pipe_info* p=(struct pipe_info*)tmp;
    int n;
    int fd=p->fd;
    int num=p->num;
    char recv_msg[sizeof(fork_Info[num])];
    bool recv_state=true;
    sleep(1);
    while(!quit)
    {
        if((n=recv(fd,recv_msg,sizeof(fork_Info[num]),MSG_WAITALL))<=0)
        {
            recv_state=false;
            if(n!=0)
                fprintf(stderr,"Write Error:%s\n",strerror(errno));
            close(fd);
            if((fd=accept(fd,(struct sockaddr *)(&p->aip->ai_addr),&p->sin_size))<0)
            {
                fprintf(stderr,"Accept error:%s\n\a",strerror(errno));
                exit(1);
            }
            cout<<"error"<<endl;
        }
        if(recv_state)
        {
            boost::unique_lock<boost::mutex> lock(dispatch);
            memset(&fork_Info[num],0,sizeof(fork_Info));
            memcpy(&fork_Info[num],recv_msg,sizeof(fork_Info));
        }
        recv_state=true;
    }
    delete p;
    p=NULL;
}
void *socket_tx_thread(void* tmp)
{
    struct pipe_info* p=(struct pipe_info*)tmp;
    int fd=p->fd;
    int num=p->num;
    int n;
    sleep(1);
    while(!quit)
    {
        {
            boost::unique_lock<boost::mutex> lock(pathClear);
            if((n=send(fd,(char*) &Terminal_Cmd[num],sizeof(Terminal_Cmd[0]),0))<0)
                fprintf(stderr,"Connect error %s\n",strerror(errno));
        }
        usleep(500000);
    }
    delete p;
    p=NULL;
}