//
// Created by yyq on 17-1-17.
//
#include "../../include/motor_control/motor_control.h"
#include <stdio.h>
#include <fstream>

using namespace std;

enum carry_action carry=_stop;

MotorControl::MotorControl() {
    if(VCI_OpenDevice(VCI_USBCAN2,0,0)!=1)
    {
        cout<<"open deivce error"<<endl;
        exit(1);
    }
    config.AccCode=0;
    config.AccMask=0xffffffff;
    config.Filter=1;
    config.Mode=0;

    /*250 Kbps  0x01  0x1C*/
    config.Timing0=0x01;
    config.Timing1=0x1C;

    Can_Init();
    GPIO_Init();
}
void MotorControl::start() {
    /*** start motion_control thread ***/
    MotorControl *m=this;
    boost::thread motion_control(boost::bind(motion_control_thread,m));
}
void* MotorControl::motion_control_thread(void *tmp)
{
#if 1
    MotorControl *p=(MotorControl*) tmp;
    /**** 初始化电机，转动电机摆正位置 ****/
    p->m_speed.v=0;
    p->m_speed.w=1.1513;
    p->Turn(p->m_speed.w);
    double last_w=p->m_speed.w;                 //上一时刻角度值，当检测到障碍物时另角度值等于上一时刻角度
    while(!quit)
    {
        if(p->m_speed.v>0)
            p->m_move_action=_forward;
        else
            p->m_move_action=_backward;
	if(p->m_move_action==_forward)
        {
		if(forward_alarm==SAFE)
	//	if(1)
        	{
            		if(carry==_up || carry==_down)
            		{
                		p->m_carry_action=carry;
                		p->Carry();
            		}
            	else
            	{
                	p->Move(abs(p->m_speed.v));
            	}	
            	last_w=p->m_speed.w;
            	usleep(1000);
       		}
        	else if(forward_alarm==WARN)
        	{
            		p->Move(abs(p->m_speed.v)/3);
            		usleep(1000);
        	}
        	else{
            		p->Move(0);
            		usleep(1000);
        	}
	}
	else {
		//if(1)
		if(back_alarm==SAFE)
                 {
                        if(carry==_up || carry==_down)
                        {
                                p->m_carry_action=carry;
                                p->Carry();
                        }
                        else
                        {
                                p->Move(abs(p->m_speed.v));
                        }
                        last_w=p->m_speed.w;
                        usleep(1000);
                }
                else if(back_alarm==WARN)
                {
                        p->Move(abs(p->m_speed.v)/3);
                        usleep(1000);
                }
                else{
			p->Move(0);
                        usleep(1000);
                }
        }
    }  	
    p->close();
#endif
}
bool MotorControl::Can_Init() {
    if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
    {
        cout<<"init CAN error"<<endl;
        VCI_CloseDevice(VCI_USBCAN2,0);
    }

    if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
    {
        cout<<"Start CAN error"<<endl;
        VCI_CloseDevice(VCI_USBCAN2,0);
    }

    send.ID=0x18D;
    send.SendType=2;
    send.RemoteFlag=0;
    send.ExternFlag=0;
    send.DataLen=7;

    return true;
}
bool MotorControl::GPIO_Init() {
    DA_CS  = gpio63 ;      // DA CLK --Output
    DA_CLK = gpio186 ;      // DA CS  --Output
    DA_DIN = gpio219  ;      // DA DIN --Output
    
    gpioExport(DA_CLK) ;
    gpioExport(DA_CS ) ;
    gpioExport(DA_DIN) ;
    gpioSetDirection(DA_CLK,outputPin) ;
    gpioSetDirection(DA_CS ,outputPin) ;
    gpioSetDirection(DA_DIN,outputPin) ;

    return true;
}
bool MotorControl::Move(float linear_vel) {
    //The max velocity is 1.667 m/s
    BYTE temp[7];
    temp[1]=temp[2]=temp[3]=0x0;        //The first 3 bytes has no affection
    temp[5]=0x03; temp[6]=0x0;          //Each action have the same Byte5 Byte6
    switch (m_move_action){
        case _backward:
            temp[4]=0x01;               //Backward  command
            temp[0]=(linear_vel/1.667)*255; //Set the value of the first byte
            break;
        case _forward:
            temp[4]=0x02;               //Forward command
            temp[0]=(linear_vel/1.667)*255;
    }
    int i;
    memcpy(send.Data,temp,7);
    /***Send linear_vel cmd by Can Bus**/
    if (VCI_Transmit(VCI_USBCAN2, 0, 0, &send, 1) > 0) {
	return true;
    }
    else
        return false;
}
bool MotorControl::Carry(){
    //Up and Down with a constant speed	
    BYTE temp[7];
    unsigned short vel;
    temp[0]=0x0;  temp[1]=0x0;
    //temp[2]=0xff; 
    temp[3]=0x0;
    temp[5]=0x03; temp[6]=0x0;          //Each action have the same Byte5 Byte6
    switch (m_carry_action){
        case _up:
            temp[4]=0x04;               //Lift-up  command
	    if(abs(lift_velocity)>UP_VELOCITY_MAX)
            {
                //temp[2]=0xff; temp[3]=0xff;
		vel=(0.015/UP_VELOCITY_MAX)*256;
                temp[2]=vel&0xff;       //8 bits low

            } else
            {
		vel=(0.015/UP_VELOCITY_MAX)*256;
               // vel=(abs(lift_velocity)/UP_VELOCITY_MAX)*65535;
                temp[2]=vel&0xff;	//8 bits low
            }
            break;
        case _down:
            temp[4]=0x08;               //Lift-down command
	    if(abs(lift_velocity)>DOWN_VELOCITY_MAX)
            {
		vel=(0.03/DOWN_VELOCITY_MAX)*256;
                temp[2]=vel&0xff;       //8 bits low
                //temp[2]=0xff; temp[3]=0xff;
            } else
            {
                vel=(0.03/DOWN_VELOCITY_MAX)*256;
		// vel=(abs(lift_velocity)/DOWN_VELOCITY_MAX)*65535;
                temp[2]=vel&0xff;       //8 bits low
	    }
    }
    memcpy(send.Data,temp,7);
    /***Send linear_vel cmd by Can Bus**/
    if (VCI_Transmit(VCI_USBCAN2, 0, 0, &send, 1) > 0) {
        return true;
    }
    else
        return false;	
}
bool MotorControl::Carrystop()
{
    BYTE temp[7];
    temp[0]=0x0;  temp[1]=0x0;
    temp[2]=0x0; temp[3]=0x0;
    temp[5]=0x03; temp[6]=0x0;          //Each action have the same Byte5 Byte6
    switch (m_carry_action){
        case _up:
            temp[4]=0x10;               //Lift-up  command
            break;
        case _down:
            temp[4]=0x20;               //Lift-down command
    }
    memcpy(send.Data,temp,7);
    /***Send linear_vel cmd by Can Bus**/
    if (VCI_Transmit(VCI_USBCAN2, 0, 0, &send, 1) > 0) {
        return true;
    }
    else
        return false;
}
bool MotorControl::Turn(float angle_dst) {
    /***According to angle_dst,calculate the DIN value**/
    float sin_ch = sin(angle_dst)*2+2.5 ;                               //Waiting Modify
    float cos_ch = cos(angle_dst)*2+2.5 ;
    unsigned int Temp=0x4000;                                       //DA fast mode

    unsigned int DA_sin = round(sin_ch*pow(2,12)/(2*DA_VREF));      //Use A-Chanel to output
    unsigned int DA_cos = round(cos_ch*pow(2,12)/(2*DA_VREF));      //Use B-Chanel to output

    DA_convert(Temp|0x1000|(0x0fff&DA_cos));                //B-Chanel Output
    DA_convert(Temp|0x8000|(0x0fff&DA_sin));                //A-Chanel Output

    return true;
}

void MotorControl::DA_convert(unsigned int Data) {
    unsigned int Dig=0;
    unsigned int i=0;
    gpioSetValue(DA_CLK,high);
    gpioSetValue(DA_CS ,low);                               //Select
    for(i=0;i<16;i++)
    {
        Dig=Data&0x8000;
        if(Dig)
        {
            gpioSetValue(DA_DIN,high);
        }
        else
        {
            gpioSetValue(DA_DIN,low);
        }
        gpioSetValue(DA_CLK,low);
        usleep(1);
        Data<<=1;
        gpioSetValue(DA_CLK,high);
        usleep(1);
    }
    gpioSetValue(DA_CLK,high);
    gpioSetValue(DA_CS,high);
}
void MotorControl::close(){
    cout << "GPIO closed." << endl;
    gpioUnexport(DA_CLK);          // unexport the DA_CLK
    gpioUnexport(DA_CS);      	   // unexport the DA_CS
    gpioUnexport(DA_DIN);      	   // unexport the DA_DIN

//   gpioUnexport(WARN1);          // unexport the Obstacle_radar WARN0
//   gpioUnexport(WARN2);          // unexport the Obstacle_radar WARN1
//   gpioUnexport(DANGER);         // unexport the Obstacle_radar DANGER
  // gpioUnexport(Area_set0);      // unexport the Obstacle_radar Area_set0
  // gpioUnexport(Area_set1);      // unexport the Obstacle_radar Area_set1

}
