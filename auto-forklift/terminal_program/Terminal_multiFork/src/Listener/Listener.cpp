//
// Created by yyq on 17-3-2.
//
#include "../../include/Listener/Listener.h"

int connected_fork=0;        //记录连接的叉车数量

vector<int> fork_queue;

Listener::Listener()
{
    socket_Init();
}
void Listener::start()
{
   Listener *m=this;
   boost::thread _monitor(bind(monitor,m));
}
void * Listener::monitor(void *tmp)
{
    Listener *p=(Listener*)tmp;

    char* addr;         //叉车IP
    unsigned short int port;

    for(p->aip=p->ailist;p->aip!=NULL;p->aip=p->aip->ai_next)
    {
        if((p->sockfd=p->initserver(SOCK_STREAM,p->aip->ai_addr,p->aip->ai_addrlen,QLEN))>=0) {
            p->sin_size = sizeof(struct sockaddr);
            while(connected_fork<MAX_FORK && !quit)
            {
                if ((p->new_fd = accept(p->sockfd, (struct sockaddr *) &(p->aip->ai_addr), &p->sin_size)) < 0) {
                    fprintf(stderr, "Accept error:%s\n\a", strerror(errno));
                    exit(1);
                } else {
                    /*** get the address of client ***/
                    addr=inet_ntoa(((struct sockaddr_in *) &p->aip->ai_addr)->sin_addr);
                    port=((struct sockaddr_in *) &p->aip->ai_addr)->sin_port;
                    cout<<"Forklift connect:"<<addr<<","<<port<<",fd:"<<p->new_fd<<endl;

                    p->m_socket[fork_address[addr]]=shared_ptr<Socket>(new Socket(p->new_fd,addr,p->aip));
                    //p->m_socket[fork_port[port]]=shared_ptr<Socket>(new Socket(p->new_fd,addr,p->aip));
                    fork_queue.push_back(fork_address[addr]);
                    connected_fork++;
                }
            }
        }
    }
}
void Listener::socket_Init()
{
    memset(&hint,0,sizeof(hint));
    hint.ai_flags=AI_CANONNAME;
    hint.ai_socktype=SOCK_STREAM;
    hint.ai_family=AF_INET;
    hint.ai_canonname=NULL;
    hint.ai_addr=NULL;
    hint.ai_next=NULL;
    if((err=getaddrinfo(IP,PORT,&hint,&ailist))!=0)
    {
        printf("getaddrinfo err:%s\n",gai_strerror(err));
        exit(1);
    }
}
int Listener::initserver(int type,const struct sockaddr* addr,socklen_t alen,int qlen)
{
    int fd;
    int err=0;
    if((fd=socket(addr->sa_family,type,0))<0)
    {
        perror("socket error");
        return (-1);
    }
    int on=1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on) );
    if(bind(fd,addr,alen)<0)
    {
        perror("bind error");
        return (-1);
    }
    if(type==SOCK_STREAM||type==SOCK_SEQPACKET)
    {
        if(listen(fd,qlen)<0) {
            perror("listen error");
            return (-1);
        }
    }
    return fd;
}