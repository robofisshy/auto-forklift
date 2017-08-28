/************************************************************************
* Copyright(c) 2017  stardraw
* All rights reserved.
*
* File:	pubinc.h
* Brief: 声明公用数据和函数
* Version: 1.0
* Author: Yu Yiqi
* Email: zju_yyq@163.com
* Date:	2017/2/17 9:05
* History:
************************************************************************/

#ifndef STARDRAW_FORKLIFT_PUB_INC_H
#define STARDRAW_FORKLIFT_PUB_INC_H

#include <iostream>
#include <vector>
#include <boost/thread.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

using namespace std;
//using namespace boost;

#define X_radius 300      //倒车时x方向的转弯距离
#define Y_radius 300      //倒车时y方向的转弯距离

#define pi 3.141592653

#define edge_x 50.0       //定义电子地图上x、y轴的计量单位
#define edge_y 30.0

#define MAX_FORK 5        //定义叉车连接的最大数量

/*** 地图相关变量 ***/
typedef boost::property<boost::edge_weight_t, int> EdgeWeightPro;                 //为边添加权重属性
typedef boost::property<boost::vertex_global_t, struct position> VertexPro;       //为顶点添加位置属性
typedef boost::adjacency_list<boost::listS,boost::vecS,boost::directedS,VertexPro,EdgeWeightPro> Graph;

typedef boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Graph>::edge_descriptor edge_descriptor;

extern boost::mutex pathPlan;
extern boost::mutex pathClear;
extern boost::mutex dispatch;
//extern boost::mutex terminalMutex;
/*** signal函数，检测到Ctrl+C，跳出循环，安全退出程序 ***/
extern bool quit;
void handle_signal(int sig);

/*** 每一段前进方向 ***/
enum direction{
    Straight=12,
    Right,
    Left
};
/*** 叉车行为 ***/
enum action{
    CEASE=40,
    PREPARE,
    MOVE
};
/*** 叉车状态 ***/
enum working_state{
    STOP=30,
    STANDBY,
    WORKING,
    ARRIVE
};
/*** 地图状态 ***/
enum map_state{
    BLANK=9,
    UNDONE=10,
    COMPLETE=11
};
/*** 控制台状态 ***/
enum terminal_state{
    WAITING=20,
    MAPPING
};
/*** 任务状态 ***/
enum cmd_state{
    WAIT=50,
    EXECUTE,
    FINISH
};
/*** 实际坐标值 ***/
struct position{
    double x;
    double y;
    double theta;
};
/*** 全局路径规划生成
 * s_p：每一段起点 g_p：每一段终点 flag：每一段的方向 ***/
struct traj_point{
    struct position s_p;
    struct position g_p;
    enum direction flag;
};
/*** 任务信息 ***/
struct track_info{
    edge_descriptor edge;               //边序号
    enum direction flag;    //方向
    double time_w;
    double in_time;
    double out_time;
};
/*** 每一段进行差值
 * 得到x、y标，速度***/
struct inter_point{
    double x;
    double y;
    double v;
};
/*** 叉车上传信息 ***/
struct Info{
    struct position curr_pos;
    enum working_state curr_work_state;
    struct position next_point;
};
/*** 终端下发命令 ***/
struct Cmd{
    struct inter_point traj[1000];
    enum action fork_action;
    enum map_state map_state;
};
/*** 速度 ***/
struct speed{
    double v;
    double w;
};
/*** 控制台状态结构体，存储控制台的当前状态和下一状态 ***/
struct terminal_state_struct{
    terminal_state cur_state;
    terminal_state next_state;
};
/*** 控制台任务结构体，存储任务的起点和终点以及任务当前状态 ***/
struct _cmd{
    int pick;
    int put;
    enum cmd_state state;
};
/*** 定义叉车颜色结构体，每台叉车以不同颜色显示 ***/
struct color{
    double b;
    double g;
    double r;
};
/*** 控制台状态结构体数组，存储每个叉车对应控制台的当前状态和下一状态 ***/
extern struct terminal_state_struct terminal_queue[MAX_FORK];
/*** 命令队列 ***/
extern vector<struct _cmd> cmd_queue;
/*** 任务队列 ***/
extern vector<vector<struct track_info>> task_queue;
/*** 定义Info结构体数组,存储每个叉车上传的信息 ***/
extern struct Info fork_Info[MAX_FORK];
/*** 定义Cmd结构体数组,存储在终端下发给每个叉车的信息 ***/
extern struct Cmd  Terminal_Cmd[MAX_FORK];
/*** fork_address存储IP对应的叉车编号 ***/
extern map<string,int> fork_address;
/*** fork_port存储端口号对应的叉车编号 (调试用,模拟多台叉车)***/
extern map<unsigned short int,int> fork_port;
/*** 当前连接叉车的数量 ***/
extern int connected_fork;
/*** 当前处于决策的叉车号 ***/
extern int current_num;
/*** 存储叉车号vector ***/
extern vector<int> fork_queue;
/*** 存储叉车任务状态 （true为有任务 false为无任务）***/
extern bool task[MAX_FORK];
/*** 每个叉车对应的颜色 ***/
extern struct color fork_color[MAX_FORK];
/*** 存储时间窗冲突时，原始的路径权值 ***/
extern map<edge_descriptor,int> origin_weight;

#endif //STARDRAW_FORKLIFT_PUB_INC_H
