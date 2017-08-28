/************************************************************************
* Copyright(c) 2017  stardraw
* All rights reserved.
*
* File:	Map.h
* Brief: 地图类（读取地图数据、建图、路径规划）
* Version: 1.0
* Author: Yu Yiqi
* Email: zju_yyq@163.com
* Date:	2017/2/26 9:05
* History:
************************************************************************/

#ifndef DIJKSTRA_TEST_MAP_H_H
#define DIJKSTRA_TEST_MAP_H_H

#include <iostream>
#include <fstream>
#include <stack>
#include "../pub_inc.h"
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

#define DIS_THREADHOLD 50        //如果实际起始点与规划起点的距离小于该阈值，则使用实际起始点替换规划起点；否则构造结构体，插入轨迹前端

using namespace boost;
using namespace std;
using namespace cv;

struct Edge{
    int origin;
    int end;
    double weight;
};
struct Curve{
    double r;
    double theta;
};

class Fork_Map{
public:
    Fork_Map();
    void Map_Init();
    bool plan(struct position _cur,int _target,vector<struct inter_point> &traj);       //路径规划、插值 入口函数
    vector<struct inter_point> reverse_plan(struct position _cur,int _target);          //倒车规划

    bool dijkstra(struct position _cur,int _target,vector<struct traj_point> &tmp);               //规划全局最短路径
    int find_nearest_vertex(struct position _cur,int _target);                          //找到地图上距实际位置最近的节点
    bool interpolate(vector<struct traj_point> _traj,vector<struct inter_point> &_inter_traj);            //为每一段全局路径做局部插值
    vector<struct inter_point> reverse_interpolate(vector<struct traj_point> _traj);    //倒车点至货架位置轨迹规划
    vector<struct inter_point> reverse_interpolate(int _start,int _target);   //新版倒车插补

    void readParam(); //读取地图数据

    double getX_theta(struct position p1,struct position p2);
    double getX_theta(struct inter_point p1,struct inter_point p2);                     //p2 指向 P1
    double getDistance(struct position p1,struct position p2);

    struct Curve getCurve(struct position p1,struct position p2,struct position p3,struct position p4);

    int dispatch(int _start);                          //根据用户输入的命令,选择距任务执行点最近的待命叉车

public:
    Graph m_Map;
    enum map_state dijk_map;
    //Vertex m_Vertex;
    map<int,int> reverse;
    Mat map_image;

    map<int,struct position> node;                            //顶点
    map<int,struct position> station;                         //工位

    map<int,struct Edge> m_edge;
    property_map < Graph,vertex_global_t>::type vertexprop;         //地图顶点属性
    property_map < Graph,edge_weight_t>::type edgeprop;         //地图顶点属性
    vector<int> path;
private:
    vector<vector<double>> edge_w;
    vector<struct position> m_Point;
    vector<int> point_num;

    vector<vertex_descriptor> parent;
};

#endif //DIJKSTRA_TEST_MAP_H_H
