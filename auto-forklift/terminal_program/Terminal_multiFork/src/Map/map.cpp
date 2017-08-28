/************************************************************************
* Copyright(c) 2017  stardraw
* All rights reserved.
*
* File:	map.cpp
* Brief: Fork_Map类函数实现
* Version: 1.0
* Author: Yu Yiqi
* Email: zju_yyq@163.com
* Date:	2017/2/17 9:05
* History:
************************************************************************/
#include "../../include/Map/Map.h"

using namespace cv;

vector<vector<struct track_info>> task_queue;       //声明任务队列
map<edge_descriptor,int> origin_weight;             //存储路径原始权值

Fork_Map::Fork_Map()
{
    readParam();
    task_queue.resize(5);
}
/**** 初始化地图，添加点和边 ****/
void Fork_Map::Map_Init()
{
    int i;
    int temp;
    Mat img(1200,1200,CV_8UC3,Scalar(255,255,255));
    map_image=img;
    Point pp;
    for(i=0;i<node.size();i++)
    {
        pp.x=node[i].x/10.0;
        pp.y=node[i].y/10.0;
        circle(map_image,pp,1,Scalar(0,0,0),3);
        temp=add_vertex(node[i],m_Map);
        point_num.push_back(temp);
    }

    for(i=0;i<m_edge.size();i++)
    {
        add_edge(m_edge[i].origin,m_edge[i].end,m_edge[i].weight,m_Map).first;
    }
    vertexprop = get(vertex_global, m_Map);     // 获得顶点的属性
    parent.resize(num_vertices(m_Map));
}
/*** 寻找地图中距实际起始位置最近的节点 ***/
int Fork_Map::find_nearest_vertex(struct position _cur,int _target)
{
    double min_dis=99999;
    double dis;
    int near_num;

    graph_traits < Graph >::vertex_iterator vi, vend;   //迭代器，遍历顶点
    property_map < Graph,vertex_global_t>::type vertexprop = get(vertex_global, m_Map);     // 获得顶点的属性

    for(tie(vi,vend)=vertices(m_Map);vi!=vend;++vi)
    {
        dis=sqrt((vertexprop[*vi].x-_cur.x)*(vertexprop[*vi].x-_cur.x)+(vertexprop[*vi].y-_cur.y)*(vertexprop[*vi].y-_cur.y));
        if(dis<=min_dis)
        {
            if(dis==min_dis)
            {
                /*** 如果实际位置距离地图多个节点间距离相等，判断与target的距离（可能有问题，待修改） ***/
                double dis1=sqrt(pow((vertexprop[_target].x-vertexprop[near_num].x),2)+pow((vertexprop[_target].y-vertexprop[near_num].y),2));
                double dis2=sqrt(pow((vertexprop[_target].x-vertexprop[*vi].x),2)+pow((vertexprop[_target].y-vertexprop[*vi].y),2));
                if(dis2<dis1)
                {
                    near_num=*vi;
                    continue;
                }
            }
            min_dis=dis;
            near_num=*vi;
        }
    }
    return near_num;
}
int Fork_Map::dispatch(int _start)
{
    int i=0;
    int num=5555;
    int min_dis=10000;
    {
        boost::unique_lock<boost::mutex> lock(pathClear);
        for(i;i<MAX_FORK;i++)
        {
            if(fork_Info[i].curr_work_state==STANDBY)                   //当前待命的叉车
            {
                int s=find_nearest_vertex(fork_Info[i].curr_pos,_start);
                vector<double> dist(num_vertices(m_Map));               //存储最短距离
                dijkstra_shortest_paths(m_Map,s,predecessor_map(&parent[0]).distance_map(&dist[0]));
                if(dist[_start]<min_dis)
                {
                    min_dis=dist[_start];
                    num=i;
                }
            }
        }
        //cout<<min_dis<<endl;
    }
    return num;
}
bool Fork_Map::plan(struct position _cur,int _target,vector<struct inter_point> &traj)
{
    /*** 得到全局路径 ***/
    vector<struct traj_point> tmp;
    if(dijkstra(_cur,_target,tmp))
    {
        /*** 局部路径插补 ***/
        if(interpolate(tmp,traj))
            return true;
        else
            return false;
    }
    else
        return false;
}
vector<struct inter_point> Fork_Map::reverse_plan(struct position _cur,int _target)
{
    /*** 得到全局路径 ***/
    vector<struct traj_point> tmp;
    dijkstra(_cur,_target,tmp);
    /*** 局部路径插补 ***/
    vector<struct inter_point> _trajectory=reverse_interpolate(tmp);
    return _trajectory;
}
/*** 全局路径规划，使用dijkstra算法寻找最优路径 ***/
bool Fork_Map::dijkstra(struct position _cur,int _target,vector<struct traj_point> &temp_traj)
{
    timeval begin;
    /*** 找到地图中与实际起始位置最近的顶点 ***/
    int start=find_nearest_vertex(_cur,_target);

    if(start==_target)
    {
        cout<<"起点终点相同，无需规划"<<endl;
        return true;
    }

    point_num.resize(num_vertices(m_Map));

    vertex_descriptor s = vertex(start, m_Map);       //设定出发点
    vector<double> dist(num_vertices(m_Map));           //存储最短距离
    vector<struct track_info> current_task;            //存储当前任务信息

    struct track_info  _track_info;
    struct traj_point _temp;

    /**** ***求取最短路径** ******/
    dijkstra_shortest_paths(m_Map, s, predecessor_map(&parent[0]).distance_map(&dist[0]));

    std::cout << "distances and parents:" << std::endl;
    graph_traits < Graph >::vertex_iterator vi, vend;   //迭代器，遍历顶点

    for (tie(vi, vend) = vertices(m_Map); vi != vend; ++vi)
    {
        cout << "distance(" << point_num[*vi] << ") = " << dist[*vi] <<" ("<<vertexprop[*vi].x<<", "<<vertexprop[*vi].y<<"), ";
        cout << "parent(" << point_num[*vi] << ") = " << point_num[parent[*vi]] <<"(" << vertexprop[parent[*vi]].x<<","<<vertexprop[parent[*vi]].y<<")"<<endl;
    }
    /*** 判断是否有能到达的路径 ***/
    if(dist[_target]>65536)
    {
        cout<<"距离为:"<<dist[_target]<<",道路堵塞，等一段时间"<<endl;
        return false;
    }
    /**** 得到_origin至_target的最短路径所经过的点(此操作后，vector开始项为终点) ****/
    while(point_num[_target]!=point_num[start])
    {
        path.push_back(point_num[_target]);
        _target=parent[_target];
    }
    path.push_back(point_num[_target]);

    /**** 倒序排列 把轨迹点赋值给traj向量 ****/
    vector<int>::reverse_iterator iter;
    edge_descriptor pass_edge;
    for(iter=path.rbegin();iter!=path.rend()-1;iter++)
    {
        cout<<*iter<<" ";
        _temp.s_p=vertexprop[*iter];
        _temp.g_p=vertexprop[*(iter+1)];

        pass_edge=edge(*iter,*(iter+1),m_Map).first;    //根据起点和终点得到边

        double current_angle=_temp.s_p.theta;                        //当前段起始点方向（水平或垂直）
        double t=getX_theta(_temp.g_p,_temp.s_p)-current_angle*1.57;      //起点指向终点，与x轴夹角。减去起始点与x轴夹角
        if(t>pi)                                                     //角度值转化
            t=t-2*pi;
        if(t<-pi)
            t=t+2*pi;
        if(((pi/8<t)&&(t<pi/2))||((-0.875*pi<t)&&(t<-pi/2)))
        {
            _temp.flag=Left;
        }
        else if(((-pi/2<t)&&(t<-pi/8)) || ((pi/2<t)&&(t<0.875*pi)))
        {
            _temp.flag=Right;
        }
        else
        {
            _temp.flag=Straight;
        }
        temp_traj.push_back(_temp);
        /*** 将该路径所经过边及其方向存入task_info结构体 ***/
        _track_info.edge=pass_edge;
        _track_info.flag=_temp.flag;
        current_task.push_back(_track_info);
    }
    /**** 用实际起始点替换规划轨迹中的起始点 ****/
    {
        temp_traj.front().s_p.x=_cur.x;
        temp_traj.front().s_p.y=_cur.y;

        double current_angle=temp_traj.front().s_p.theta;                        //当前段起始点方向（水平或垂直）
        double t=getX_theta(temp_traj.front().g_p,temp_traj.front().s_p)-current_angle*1.57;      //起点指向终点，与x轴夹角。减去起始点与x轴夹角
        if(t>pi)            //角度值转化
            t=t-2*pi;
        if(t<-pi)
            t=t+2*pi;
        if(((pi/8<t)&&(t<pi/2))||((-0.875*pi<t)&&(t<-pi/2)))
            temp_traj.front().flag=Left;
        else if(((-pi/2<t)&&(t<-pi/8)) || ((pi/2<t)&&(t<0.875*pi)))
            temp_traj.front().flag=Right;
        else
            temp_traj.front().flag=Straight;
    }
    /*** get time of now,set as the start-time of folk ***/
    gettimeofday(&begin, NULL);
    double move_time=begin.tv_sec*1000000+begin.tv_usec;
    vector<struct track_info>::iterator iter_track;
    /*** fill the time_window of each edge ***/
    for(iter_track=current_task.begin();iter_track!=current_task.end();iter_track++)
    {
        switch(iter_track->flag)
        {
            case Left:
                iter_track->in_time=move_time;
                move_time+=30*1000000;//转弯时间设定为30s
                iter_track->time_w=30*1000000;
                iter_track->out_time=move_time;
                break;
            case Right:
                iter_track->in_time=move_time;
                move_time+=30*1000000;//转弯时间设定为30s
                iter_track->time_w=30*1000000;
                iter_track->out_time=move_time;
                break;
            case Straight:
                iter_track->in_time=move_time;
                move_time+=50*1000000;//转弯时间设定为50s
                iter_track->time_w=50*1000000;
                iter_track->out_time=move_time;
        }
    }
    /*** 与task_queue已有任务的路径匹对，判断是否有时间窗重叠 ***/
    edgeprop=get(edge_weight,m_Map);
    for(int i=0;i<current_task.size();i++)
    {
        for (int j = 0; j < task_queue.size(); j++)
        {
            for(int k=0;k < task_queue[j].size();k++)
            {
                if(task_queue[j][k].edge==current_task[i].edge)
                {
                    //如果当前任务的某一段时间窗与其它任务的某一段时间窗重叠
                    if((task_queue[j][k].in_time<current_task[i].in_time)&&(task_queue[j][k].out_time>current_task[i].in_time))
                    {
                        //在地图中将该边禁用（权重设为无穷大），返回false重新规划
                        origin_weight[current_task[i].edge]=edgeprop[current_task[i].edge];
                        edgeprop[current_task[i].edge]=66000;
                        return false;
                    }
                }
            }
        }
    }

    /*** 规划好的路径存入task_queue ***/
    task_queue[current_num].insert(task_queue[current_num].end(),current_task.begin(),current_task.end());

    cout<<path.front()<<endl;

    path.clear();
    parent.clear();

    dijk_map=COMPLETE;
    return true;
}
/*** 对全局路径规划得到的每一段进行局部插补 ***/
bool Fork_Map::interpolate(vector<struct traj_point> _traj,vector<struct inter_point> &inter_traj)
{
    struct inter_point temp_p;

    vector<struct traj_point>::iterator iter;

    double l_inter=100; //插值点间距
    double dist;		//直线距离
    double theta;       //起点指向终点,与x轴夹角
    double dx(0.0);
    double dy(0.0);
    double d_theta,turn_radius;
    int k(0);//右转,y递减,k为负数; 左转,y递增,k为正数

    int num_curve=27;    //divided int 27 points  转弯固定分为17个点
    int num_straight;    //Due to dist and l_inter

    /*** 一个for循环，将_traj最短路径分段做局部插补，存在all_inter向量里 ***/
    for(iter=_traj.begin();iter!=_traj.end();iter++)
    {
        dist=getDistance(iter->s_p,iter->g_p);
        theta=getX_theta(iter->g_p,iter->s_p);          //起点指向终点
        switch(iter->flag)
        {
            case Straight:
                num_straight=dist/l_inter;
                dx=l_inter*cos(theta);
                dy=l_inter*sin(theta);
                for(int i=0;i<num_straight+1;i++)
                {
                    temp_p.x=iter->s_p.x+i*dx;
                    temp_p.y=iter->s_p.y+i*dy;
                    if((iter+1)!=_traj.end())
                    {
                        if((iter+1)->flag!=Straight)    //如果下一段是弯道，提前减速
                        {
                            if(i>=num_straight-4)       //点之间间隔0.3m，提前4个点减速--1.2米
                                temp_p.v=0.6;
                            else
                                temp_p.v=1;
                        }
                        else
                            temp_p.v=1;                 //如果下一段是直线，不减速
                    } else{                             //如果本段是最后一段，与下一段弯道一样提前减速
                            if(i>=num_straight-4)
                                temp_p.v=0.3;
                            else
                                temp_p.v=1;
                    }
                    //temp_p.v
                    inter_traj.push_back(temp_p);
                }
                temp_p.x=iter->g_p.x;                   //每段最后一点速度，与上一点相等
                temp_p.y=iter->g_p.y;
                temp_p.v=inter_traj.back().v;
                inter_traj.push_back(temp_p);
                break;
            case Right:
                d_theta=pi/2/(num_curve-1);
                turn_radius=dist/1.414;
                theta=theta-(pi/4);
                k=1;
                for(int i=0;i<num_curve;i++)
                {
                    dx=turn_radius*(1-cos(d_theta*i));
                    dy=k*turn_radius*sin(d_theta*i);
                    //坐标变换,从理想坐标系到实际坐标系变换
                    temp_p.x= iter->s_p.x+dx*cos(theta)-dy*sin(theta);
                    temp_p.y= iter->s_p.y+dx*sin(theta)+dy*cos(theta);
                    temp_p.v=0.6;
                    inter_traj.push_back(temp_p);
                }
                break;
            case Left:
                d_theta=pi/2/(num_curve-1);
                turn_radius=dist/1.414;
                theta=theta+(pi/4);
                k=-1;
                for(int i=0;i<num_curve;i++)
                {
                    dx=turn_radius*(1-cos(d_theta*i));
                    dy=k*turn_radius*sin(d_theta*i);
                    //坐标变换,从理想坐标系到实际坐标系变换
                    temp_p.x= iter->s_p.x+dx*cos(theta)-dy*sin(theta);
                    temp_p.y= iter->s_p.y+dx*sin(theta)+dy*cos(theta);
                    temp_p.v=0.6;
                    inter_traj.push_back(temp_p);
                }
        }
    }
    return true;
}
vector<struct inter_point> Fork_Map::reverse_interpolate(int _start,int _target)
{
    double theta;

    double lx,ly;                      //起始点和终点，x方向和y方向距离
    double dx,dy;                      //x方向和y方向插补间距
    double dist;
    double d_theta,turn_radius;
    int k(0);//右转,y递减,k为负数; 左转,y递增,k为正数

    int num_curve=17;                  //divided int 17 points  转弯固定分为17个点
    double l_inter=50;                 //插值点间距
    int num_straight;                  //Due to dist and l_inter
    enum direction flag;

    struct inter_point temp_p={node[_start].x,node[_start].y,-0.3};
    vector<struct inter_point> inter_traj;       //插值后的整条轨迹，由起点到终点
    struct inter_point target={station[_target].x,station[_target].y,-0.3};
    struct inter_point turn_start,turn_end;
    switch((int)node[_start].theta)
    {
        case 0:
            lx=node[_start].x-station[_target].x;
            ly=node[_start].y-station[_target].y;
            if(abs(lx)>300)
            {
                dist=abs(lx)-300;
                if(lx>0)
                    theta=pi;
                else
                    theta=0;
                num_straight = dist / l_inter;
                dx = l_inter * cos(theta);
                dy = l_inter * sin(theta);
                for (int i = 0; i < num_straight + 1; i++) {
                    temp_p.x=node[_start].x+i*dx;
                    temp_p.y=node[_start].y+i*dy;
                    temp_p.v=-0.3;

                    inter_traj.push_back(temp_p);
                }
            }
            turn_start=temp_p;
            /*** 判断左传还是右转 ***/
            {
                double t = getX_theta(target, turn_start) - node[_start].theta*1.57;      //起点指向终点，与x轴夹角。减去起始点与x轴夹角
                if (t > pi)            //角度值转化
                    t = t - 2 * pi;
                if (t < -pi)
                    t = t + 2 * pi;
                if (((pi / 8 < t) && (t < pi / 2)) || ((-0.875 * pi < t) && (t < -pi / 2)))
                    flag = Left;
                else if (((-pi / 2 < t) && (t < -pi / 8)) || ((pi / 2 < t) && (t < 0.875 * pi)))
                    flag = Right;
            }
            switch(flag)
            {
                case Right:
                    dist = 300*1.414;
                    if(target.y>turn_start.y)
                        theta=0.75*pi;
                    else
                        theta = -pi/4;          //起点指向终点
                    d_theta = pi / 2 / (num_curve - 1);
                    turn_radius = dist / 1.414;
                    theta = theta - (pi / 4);
                    k = 1;
                    for (int i = 0; i < num_curve; i++) {
                        dx = turn_radius * (1 - cos(d_theta * i));
                        dy = k * turn_radius * sin(d_theta * i);
                        //坐标变换,从理想坐标系到实际坐标系变换
                        temp_p.x = turn_start.x + dx * cos(theta) - dy * sin(theta);
                        temp_p.y = turn_start.y + dx * sin(theta) + dy * cos(theta);
                        temp_p.v = -0.3;
                        inter_traj.push_back(temp_p);
                    }
                    break;
                case Left:
                    dist = 300*1.414;
                    if(target.y>turn_start.y)
                        theta=pi/4;
                    else
                        theta =-0.75*pi;          //起点指向终点
                    d_theta = pi / 2 / (num_curve - 1);
                    turn_radius = dist / 1.414;
                    theta = theta + (pi / 4);
                    k = -1;
                    for (int i = 0; i < num_curve; i++) {
                        dx = turn_radius * (1 - cos(d_theta * i));
                        dy = k * turn_radius * sin(d_theta * i);
                        //坐标变换,从理想坐标系到实际坐标系变换
                        temp_p.x = turn_start.x + dx * cos(theta) - dy * sin(theta);
                        temp_p.y = turn_start.y + dx * sin(theta) + dy * cos(theta);
                        temp_p.v = -0.3;
                        inter_traj.push_back(temp_p);
                    }
            }
            turn_end=temp_p;
            if(abs(ly)>300)
            {
                dist=abs(ly)-300;
                if(ly>0)
                    theta=-pi/2;
                else
                    theta=pi/2;
                num_straight = dist / l_inter;
                dx = l_inter * cos(theta);
                dy = l_inter * sin(theta);
                for(int i = 0; i < num_straight + 1; i++) {
                    temp_p.x=turn_end.x+i*dx;
                    temp_p.y=turn_end.y+i*dy;
                    temp_p.v=-0.3;

                    inter_traj.push_back(temp_p);
                }
            }
            break;
        case 1:
            ;
    }
    return inter_traj;
}
/*** 倒车点至货架位置轨迹规划 （与interpolate函数基本相似，但是速度取负值--倒车）***/
vector<struct inter_point> Fork_Map::reverse_interpolate(vector<struct traj_point> _traj)
{
    property_map < Graph,vertex_global_t>::type vertexprop = get(vertex_global, m_Map);     // 获得顶点的属性

    vector<struct inter_point> inter_traj;       //插值后的整条轨迹，由起点到终点

    vector<struct traj_point>::iterator iter;

    struct inter_point temp_p;

    double l_inter=100; //插值点间距
    double dist;		//直线距离
    double theta;       //起点指向终点,与x轴夹角
    double dx(0.0);
    double dy(0.0);
    double d_theta,turn_radius;
    int k(0);//右转,y递减,k为负数; 左转,y递增,k为正数

    int num_curve=27;   //divided int 17 points  转弯固定分为17个点
    int num_straight;   //Due to dist and l_inter

    for(iter=_traj.begin();iter!=_traj.end();iter++) {
        dist = getDistance(iter->s_p, iter->g_p);
        theta = getX_theta(iter->g_p, iter->s_p);          //起点指向终点
        switch (iter->flag) {
            case Straight:
                num_straight = dist / l_inter;
                dx = l_inter * cos(theta);
                dy = l_inter * sin(theta);
                for (int i = 0; i < num_straight + 1; i++) {
                    temp_p.x=iter->s_p.x+i*dx;
                    temp_p.y=iter->s_p.y+i*dy;
                    temp_p.v=-0.3;

                    inter_traj.push_back(temp_p);
                }
                temp_p.x = iter->g_p.x;                   //每段最后一点速度，与上一点相等
                temp_p.y = iter->g_p.y;
                temp_p.v = inter_traj.back().v;
                inter_traj.push_back(temp_p);
                break;
            case Right:
                d_theta = pi / 2 / (num_curve - 1);
                turn_radius = dist / 1.414;
                theta = theta - (pi / 4);
                k = 1;
                for (int i = 0; i < num_curve; i++) {
                    dx = turn_radius * (1 - cos(d_theta * i));
                    dy = k * turn_radius * sin(d_theta * i);
                    //坐标变换,从理想坐标系到实际坐标系变换
                    temp_p.x = iter->s_p.x + dx * cos(theta) - dy * sin(theta);
                    temp_p.y = iter->s_p.y + dx * sin(theta) + dy * cos(theta);
                    temp_p.v = -0.3;
                    inter_traj.push_back(temp_p);
                }
                break;
            case Left:
                d_theta = pi / 2 / (num_curve - 1);
                turn_radius = dist / 1.414;
                theta = theta + (pi / 4);
                k = -1;
                for (int i = 0; i < num_curve; i++) {
                    dx = turn_radius * (1 - cos(d_theta * i));
                    dy = k * turn_radius * sin(d_theta * i);
                    //坐标变换,从理想坐标系到实际坐标系变换
                    temp_p.x = iter->s_p.x + dx * cos(theta) - dy * sin(theta);
                    temp_p.y = iter->s_p.y + dx * sin(theta) + dy * cos(theta);
                    temp_p.v = -0.3;
                    inter_traj.push_back(temp_p);
                }
        }
    }
    return inter_traj;
}
/**** 读取地图数据 ****/
void Fork_Map::readParam()
{
    string filename="/home/yyq/ClionProjects/Terminal_multiFork/include/Map/map_data.txt";
    ifstream fin(filename.c_str());
    if(!fin)
    {
        cerr<<"parameters doesn't exist"<<endl;
        return ;
    }
    int station_num=0;
    while(!fin.eof()) {
        string str;
        getline(fin, str);
        if (str[0] == '#')
            continue;
        else if(str[0]=='w'){                               //存储工位信息（包括工位倒车点）
            int pos1 = str.find(":");
            station_num = atoi(str.substr(2, pos1).c_str());
            struct position cor;
            int pos2 = str.find(",");
            int pos3 = str.find(",",pos2+1);
            int pos4 = str.find(":",pos3+1);
            cor.x = atof(str.substr(pos1 + 1, pos2).c_str());
            cor.y = atof(str.substr(pos2 + 1, pos3).c_str());
            cor.theta=atof(str.substr(pos3 + 1, pos4).c_str());
            reverse[station_num]=atof(str.substr(pos4 + 1, str.length()).c_str());
            station[station_num] = cor;
        }
        else if (str[0] == 'p') {                           //存取道路节点信息
            int pos1 = str.find(":");
            int node_num = atoi(str.substr(2, pos1).c_str());
            struct position cor;
            int pos2 = str.find(",");
            int pos3=str.find(",",pos2+1);
            cor.x = atof(str.substr(pos1 + 1, pos2).c_str());
            cor.y = atof(str.substr(pos2 + 1, pos3).c_str());
            cor.theta=atof(str.substr(pos3 + 1, str.length()).c_str());
            node[node_num] = cor;
        }
            else if (str[0] == 'e') {                       //存取边信息
            struct Edge e;
            int pos0 = str.find(":");
            int edge_num = atoi(str.substr(2, pos0).c_str());
            int pos1 = str.find(",");
            e.origin = atoi(str.substr(pos0 + 1, pos1).c_str());
            int pos2 = str.find(",", pos1 + 1);
            e.end = atoi(str.substr(pos1 + 1, pos2).c_str());
            e.weight = atof(str.substr(pos2 + 1, str.length()).c_str());
            m_edge[edge_num] = e;
        }
        int pos = str.find(":");
        if (pos == -1)
            continue;
    }
}
double Fork_Map::getDistance(struct position p1,struct position p2)
{
    double dis=sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
    return dis;
}
double Fork_Map::getX_theta(struct inter_point p1,struct inter_point p2)//p2 指向 P1
{
    double lx = p1.x - p2.x;
    double ly = p1.y - p2.y;
    double theta1(0);
    if (ly < 0)
        theta1 = -acos(lx / sqrt(lx*lx + ly*ly)); //arccos 的范围为0—pi（3.14）  _theta1范围为-180度到180度
    else if (ly>0)
        theta1 = acos(lx / sqrt(lx*lx + ly*ly)); //arccos 的范围为0—pi（3.14）
    else
    {
        if (lx < 0)
            theta1 = 3.1415;
        else
            theta1 = 0;
    }
    return theta1;
}
double Fork_Map::getX_theta(struct position p1,struct position p2)//p2 指向 P1
{
    double lx = p1.x - p2.x;
    double ly = p1.y - p2.y;
    double theta1(0);
    if (ly < 0)
        theta1 = -acos(lx / sqrt(lx*lx + ly*ly)); //arccos 的范围为0—pi（3.14）  _theta1范围为-180度到180度
    else if (ly>0)
        theta1 = acos(lx / sqrt(lx*lx + ly*ly)); //arccos 的范围为0—pi（3.14）
    else
    {
        if (lx < 0)
            theta1 = 3.1415;
        else
            theta1 = 0;
    }
    return theta1;
}
struct Curve Fork_Map::getCurve(struct position p1, struct position p2, struct position p3, struct position p4)
{
    double theta;
    struct Curve tmp;
    double intersection_x,intersection_y;       //交点x,y
    double d1=sqrt((p1.x-p2.x,2)+pow(p1.y-p2.y,2));
    double d2=sqrt((p4.x-p3.x,2)+pow(p4.y-p3.y,2));
    theta=acos(((p1.x-p2.x)*(p4.x-p3.x)+(p1.y-p2.y)*(p4.y-p3.y))/(d1*d2));

    double m1,n1;       //y=m1*x+n1   以p1、p2列出直线方程
    double m2,n2;       //y=m2*x+n2   以p3、p4列出直线方程
    if(p1.x!=p2.x)
    {
        m1=(p1.y-p2.y)/(p1.x-p2.x);
        n1=p1.y-m1*p1.x;
    }
    if(p3.x!=p4.x)
    {
        m2=(p3.y-p4.y)/(p3.x-p4.x);
        n2=p3.y-m1*p3.x;
    }
    intersection_x=(n2-n1)/(m1-m2);
    intersection_y=m1*intersection_x+n1;

    double l=sqrt(pow(intersection_x-p1.x,2)+pow(intersection_y-p1.y,2));
    tmp.r=l/tan(theta/2);
    tmp.theta=theta;
    return tmp;
}