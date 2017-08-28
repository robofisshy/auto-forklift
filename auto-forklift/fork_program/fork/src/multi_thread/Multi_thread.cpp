//
// Created by yyq on 17-2-14.
//
#include "../../include/multi_thread/Multi_thread.h"
#include <fstream>

using namespace std;

Multi_thread::Multi_thread() {
    socket_Init();
}
void Multi_thread::start()
{
    sock_state=true;
    for(aip=ailist;aip!=NULL;aip=aip->ai_next)
    {
        if((sockfd=connect_retry(aip->ai_family,SOCK_STREAM,0,aip->ai_addr,aip->ai_addrlen))<=0)
        {
            fprintf(stderr,"Connect error %s",strerror(errno));
            exit(1);
        }
        else{
            int size=sizeof(m_Cmd);
            socklen_t optlen=sizeof(m_Cmd);
            setsockopt(sockfd,SOL_SOCKET,SO_RCVBUF, (char *)& size, optlen);
            cout<<"Connect successful"<<endl;
            Multi_thread *tx=this;
            Multi_thread *rx=this;
            /**** 创建socket发送和接收线程 ****/
            boost::thread socket_rx(boost::bind(socket_rx_thread,rx));
            boost::thread socket_tx(boost::bind(socket_tx_thread,tx));
        }
    }
}
void* Multi_thread::socket_rx_thread(void *tmp)
{
    int n=0;
    bool recv_state=true;
    Multi_thread *p=(Multi_thread*) tmp;
    char recv_msg[sizeof(p->m_Cmd)];
    memset(&p->m_Cmd, 0, sizeof(p->m_Cmd));
    int nn=sizeof(p->m_Cmd);
    while(!quit) {
        if(p->sock_state)
        {
            if ((n = recv(p->sockfd, recv_msg, sizeof(p->m_Cmd), MSG_WAITALL)) <= 0) {
                recv_state = false;
                if (n != 0)
                {
                    fprintf(stderr, "Receive Error:%s\n", strerror(errno));
                    sleep(1);
                }
            }
            if (recv_state) {
                /**** 将收到的数据装入m_Terminal_cmd的结构体 ****/
                boost::unique_lock<boost::shared_mutex> lock(readCMD);
                memset(&p->m_Cmd, 0, sizeof(p->m_Cmd));
                memcpy(&p->m_Cmd, recv_msg, sizeof(p->m_Cmd));
            }
            recv_state = true;
        }
    }
}
void* Multi_thread::socket_tx_thread(void *tmp) {
    int n;
    Multi_thread *p=(Multi_thread*) tmp;
    //signal(SIGPIPE, SIG_IGN);
    while(!quit)
    {
        if((n=send(p->sockfd,(char*) &p->m_Info,sizeof(p->m_Info),0))<0) {
            fprintf(stderr, "Send error %s\n", strerror(errno));
            close(p->sockfd);
            p->sock_state=false;            //关闭socket fd,置sock_state为false,避免socket_rx_thread读取失败
            if((p->sockfd=p->connect_retry(p->aip->ai_family,SOCK_STREAM,0,p->aip->ai_addr,p->aip->ai_addrlen))<=0)
            {
                fprintf(stderr,"Connect error %s",strerror(errno));
                exit(1);
            }
            p->sock_state=true;
        }
        usleep(50000);
    }
}
void Multi_thread::socket_Init()
{
    memset(&hint,0,sizeof(hint));
    hint.ai_flags=0;
    hint.ai_socktype=SOCK_STREAM;
    hint.ai_addr=NULL;
    hint.ai_next=NULL;
    if((err=getaddrinfo(WIFI_SERVER,PORT,&hint,&ailist))!=0)
        printf("getaddrinfo error:%s\n",gai_strerror(err));
}
int Multi_thread::connect_retry(int domain,int type,int protocol, const struct sockaddr *addr, socklen_t alen)
{
    int numsec,fd;
    for(numsec=1;numsec<=MAXSLEEP;numsec<<=1)
    {
        if(quit==true)
            break;
        if((fd=socket(domain,type,protocol))<0)
        {
            fprintf(stderr,"Socket Error:%s\a\n",strerror(errno));
            return(-1);
        }
        if(connect(fd,addr,alen)==0){
            return (fd);
        }
        close(fd);
        if(numsec<=MAXSLEEP/2)
            sleep(numsec);
    }
}
