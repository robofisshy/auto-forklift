////
//// Created by yyq on 17-2-20.
////
#include "../../include/forklift/forklift.h"

map<int,double> station_height;

Forklift::Forklift(map_state _m,working_state _w)
{
    /*** 工位高度简单初始化、需要建库 ***/
    station_height[0]=0.179;
    station_height[1]=0.692;

    _map_state=_m;
    cur_work_state=_w;
    motor.m_speed.v=0;
    motor.m_speed.w=1.1513;
    mthread.m_Info.curr_work_state=_w;
}
void Forklift::start()
{
    mthread.start();
    detector.start();
    motor.start();
}
