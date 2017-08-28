//
// Created by yyq on 17-3-22.
//

#ifndef STARDRAW_FORKLIFT_STATE_MACHINE_H
#define STARDRAW_FORKLIFT_STATE_MACHINE_H

#include "../pub_inc.h"

class State_Machine{
public:
    State_Machine(){};

    bool decide(enum working_state _w,enum action _a);
};
bool State_Machine::decide(enum working_state _w,enum action _a)
{

}

#endif //STARDRAW_FORKLIFT_STATE_MACHINE_H
