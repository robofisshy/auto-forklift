//
// Created by yyq on 17-4-18.
//
#include "../../include/detect/detect.h"
#include <iostream>
#include <sys/time.h>

using namespace std;

enum alarm_state forward_alarm=SAFE;
enum alarm_state back_alarm=SAFE;
double lift_encoder;

Detector::Detector()
{
    /*** open i2c_device ***/
    i2c_fd=open("/dev/i2c-1",O_RDWR);
    ioctl(i2c_fd,I2C_SLAVE,0x04);

//    openUart();
}
void Detector::start()
{
    Detector *f=this;
    Detector *b=this;
    boost::thread sensor_detect(boost::bind(sensor_detect_thread,f));
//    boost::thread back_detect(boost::bind(back_detect_thread,b));
}
//forward_radar and encoder
void* Detector::sensor_detect_thread(void *tmp)
{
    Detector *p=(Detector*)tmp;
    char msg[sizeof(_s)];
    char cmd[]="security_state";
    while(!quit)
    {
        /*** 写控制字 ***/
        write(p->i2c_fd,cmd,sizeof(cmd));
        usleep(1000);
        /*** 读取检测状态 ***/
        read(p->i2c_fd,msg,sizeof(p->_s));
        memset(&p->_s,0,sizeof(p->_s));
        memcpy(&p->_s,msg,sizeof(p->_s));
        if(p->_s.radar_forward=='d')
        {
//            cout<<"Forward_DANGER"<<endl;
            forward_alarm=DANGER;
        }
        if(p->_s.radar_forward=='s')
        {
//            cout<<"Forward_SAFE"<<endl;
            forward_alarm=SAFE;
        }
        if(p->_s.radar_forward=='w')
        {
//            cout<<"Forward_WARNING"<<endl;
            forward_alarm=WARN;
        }
//	lift_encoder=atof(p->_s.buf);
//	cout<<"height: "<<lift_encoder<<endl;
	usleep(100000);
    }
}
void* Detector::back_detect_thread(void *tmp)
{
    Detector *p=(Detector*) tmp;
    int nread,i;
    unsigned char buff[1];
    unsigned char buff6[6];
    unsigned char buff2[2];
    unsigned char buff4[4];
    unsigned char buff8[8];
    unsigned char buff548[548];
    int dist_count=0;
    int plane=-1;
    int count=0;
    int distance=0;
    int flag=0;
    struct timeval start;
    struct timeval end;
    gettimeofday(&start,NULL);

    while(!quit)
    {
        flag=0;
        count=0;
        nread=read(p->uart_fd,buff,1);
        if(buff[0]==0xFC)
        {
            nread=read(p->uart_fd,buff,1);
            if(buff[0]==0xFD)
            {   usleep(100);
                nread=read(p->uart_fd,buff,1);
                if(buff[0]==0xFE)
                {  usleep(100);
                    nread=read(p->uart_fd,buff,1);
                    if(buff[0]==0xFF)
                    {  usleep(100);
                        nread=read(p->uart_fd,buff4,4);
                        if(buff4[0]==0x2D&&buff4[1]==0x02&&buff4[2]==0x5B&&buff4[3]==0xC3)
                        {  usleep(100);
                            nread=read(p->uart_fd,buff6,6);
                            usleep(100);
                            nread=read(p->uart_fd,buff,1);
                            plane=buff[0];
                            while(buff[0]==0)//||buff[0]==1||buff[0]==2||buff[0]==3)
                            {
                                //count=count+1;
                                for(dist_count=0;dist_count<274;dist_count++)
                                {
                                    usleep(200);
                                    nread=read(p->uart_fd,buff2,2);
                                    //printf("nread :%d;distance is %d  ",nread,buff2[0]+buff2[1]*16*16);
                                    if(dist_count>=95&&dist_count<=180)
                                    {
                                        distance=buff2[0]+buff2[1]*16*16;
                                        if(distance<=1000)
                                        {
                                            flag=1;
					    back_alarm=DANGER;
				//	    printf("*****Back_DANGER*********\n");
                                        }
					else
					{
					   back_alarm=SAFE;
                               	      //     printf("oooooBack_SAFEooooooooo\n");
                                        }
                               	    }
				}
//				cout<<"STATE:"<<back_alarm<<endl;
                               // if(flag==1&&count==3)
                               // {
                               //     back_alarm=DANGER;
                               //     printf("*****Back_DANGER*********\n");
                               // }
                                //if(flag==0&&count==3)
                               // {
                               //     back_alarm=SAFE;
                               //     printf("oooooBack_SAFEooooooooo\n");
                               // }
                                usleep(200);
                               // nread=read(p->uart_fd,buff,1);
                               // plane=buff[0];
				
				gettimeofday(&end,NULL);
  //                              cout<<"Time cost:"<<end.tv_sec-start.tv_sec+(end.tv_usec-start.tv_usec)/1000000.0<<endl;
                                start=end;

                                //if(count==4)
                                //{
                                   break;
                               // }
				
                            }
                        }
                    }
                }
            }
        }
    }
}
void Detector::openUart()
{
    /*** open uart_device ***/
    uart_fd=open( "/dev/ttyUSB0", O_RDWR|O_NOCTTY|O_NDELAY);
    if(fcntl(uart_fd, F_SETFL, 0)<0)
    {
        printf("fcntl failed!\n");
    }
    if(isatty(STDIN_FILENO)==0)
    {
        printf("standard input is not a terminal device\n");
    }
    struct termios newtio,oldtio;
    if(tcgetattr( uart_fd,&oldtio)  !=  0)
    {
        perror("SetupSerial 1");
    }
    bzero( &newtio, sizeof( newtio ) );
    newtio.c_cflag  |=  CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    newtio.c_cflag |=CS8;
	
    newtio.c_cflag &=~PARENB;

    cfsetispeed(&newtio, B460800);
    cfsetospeed(&newtio, B460800);

    newtio.c_cflag &= ~CSTOPB;

    newtio.c_cc[VTIME]  = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(uart_fd,TCIFLUSH);
    if((tcsetattr(uart_fd,TCSANOW,&newtio))!=0)
    {
        perror("com set error");
    }
    printf("set done!\n");
}
