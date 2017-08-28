//
// Created by yyq on 17-3-16.
//

#ifndef STARDRAW_FORKLIFT_PATHTRACK_H
#define STARDRAW_FORKLIFT_PATHTRACK_H

#include "../forklift/forklift.h"

bool work(Forklift &_fork,vector<vector<inter_point> > traj);

/*** 运动控制函数，跟踪规划好的轨迹 ***/
bool pathTrack(Forklift &_fork,vector<inter_point> &_traj, double accuracy);
bool pathTrack(Forklift &_fork, double accuracy);
/*** 读取当前位置 ***/
struct pose getpose();

void add33();

#endif //STARDRAW_FORKLIFT_PATHTRACK_H
