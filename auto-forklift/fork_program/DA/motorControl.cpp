//
// Created by yyq on 17-1-17.
//
#include "motor_control.h"
#include <stdio.h>
#include <fstream>

using namespace std;

MotorControl::MotorControl() {
    GPIO_Init();
}
bool MotorControl::GPIO_Init() {
    DA_CS  = gpio63 ;      // DA CLK --Output
    DA_CLK = gpio186 ;      // DA CS  --Output
    DA_DIN = gpio219  ;      // DA DIN --Output
   //radar 
    WARN0 = gpio36;
    WARN1 = gpio37;
    DANGER= gpio38;
    Area_set0 = gpio184;
    Area_set1 = gpio187;    
   
   //radar
   gpioExport(WARN0) ;
   gpioExport(WARN1 ) ;
    gpioExport(DANGER) ;
    //gpioExport(Area_set0) ;
    gpioExport(Area_set1) ;
    gpioSetDirection(WARN0 ,inputPin) ;
    gpioSetDirection(WARN1 ,inputPin) ;
    gpioSetDirection(DANGER,inputPin) ;
    //gpioSetDirection(Area_set0,inputPin) ;
    gpioSetDirection(Area_set1,outputPin) ;

   //DA
    gpioExport(DA_CLK) ;
    gpioExport(DA_CS ) ;
    gpioExport(DA_DIN) ;
    gpioSetDirection(DA_CLK,outputPin) ;
    gpioSetDirection(DA_CS ,outputPin) ;
    gpioSetDirection(DA_DIN,outputPin) ;

    return true;
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

   gpioUnexport(WARN0);          // unexport the Obstacle_radar WARN0
   gpioUnexport(WARN1);          // unexport the Obstacle_radar WARN1
   gpioUnexport(DANGER);         // unexport the Obstacle_radar DANGER
   //gpioUnexport(Area_set0);      // unexport the Obstacle_radar Area_set0
   gpioUnexport(Area_set1);      // unexport the Obstacle_radar Area_set1

}
