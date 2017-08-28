//
// Created by yyq on 17-3-2.
//

#ifndef TERMINAL_TERMINAL_H
#define TERMINAL_TERMINAL_H
#include "../Listener/Listener.h"
#include "../Map/Map.h"

using namespace cv;

class Terminal{
public:
    Terminal(){};
    void init();
    void start();
    bool plan();
    bool pathPlan(struct position origin,vector<vector<struct inter_point>> &traj);
    static void * getCmd(void*);
    static void * plan_thread(void*);
public:
    Listener m_listener;
    Fork_Map m_Map;
    working_state fork_state;
    map_state _map_state;
    terminal_state cur_state;
    terminal_state next_state;

    pthread_t input_t;

    int target;
    int lift_pick;
    int lift_put;
    //struct _cmd cmd;

    Mat img;
    bool map_complete[MAX_FORK];

    vector<struct traj_point> trajectory;
};
#endif //TERMINAL_TERMINAL_H
