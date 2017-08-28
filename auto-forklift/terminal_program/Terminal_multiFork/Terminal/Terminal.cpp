//
// Created by yyq on 17-3-2.
//
#include "../../include/Terminal/Terminal.h"
#include <fstream>

using namespace std;
using namespace cv;

map<string,int> fork_address;
map<unsigned short int,int> fork_port;
vector<struct _cmd> cmd_queue;
struct color fork_color[MAX_FORK];

int current_num=1;

/**** 初始化，读入地图数据 ****/
void Terminal::init()
{
    /*** 存储叉车IP与叉车号的对应关系 ***/
    fork_address["10.10.2.162"]=0;
    fork_address["10.10.2.80"]=4;
    /*** 存储叉车port与叉车号的对应关系 ***/
    fork_port[7330]=0;
    fork_port[7334]=4;

    /*** 设定叉车对应的颜色 ***/
    fork_color[0]={0,255,0};
    fork_color[4]={200,0,0};

    m_Map.Map_Init();
    for(int i=0;i<MAX_FORK;i++)
    {
        Terminal_Cmd[i].fork_action=CEASE;
        Terminal_Cmd[i].map_state=BLANK;
        map_complete[i]=false;
        memset(&Terminal_Cmd[i].traj,0,sizeof(struct inter_point)*1000);
    }

    cur_state=WAITING;
    next_state=WAITING;

    Terminal *p=this;
    boost::thread _plan(bind(plan_thread,p));
}
void *Terminal::plan_thread(void* tmp)
{
    Terminal *p=(Terminal*)tmp;
    int error_num=0;
    while(!quit)
    {
        if(p->cur_state==WAITING)
        {
            for(int i=0;i<cmd_queue.size();i++)
            {
                if(cmd_queue[i].state==WAIT)
                {
                    p->lift_pick=cmd_queue[i].pick;
                    p->lift_put=cmd_queue[i].put;
                    /*** 选取距离起点最近的待命叉车 赋值给current_num ***/
                    current_num=p->m_Map.dispatch(p->m_Map.reverse[p->lift_pick]);
                    if(current_num==5555)
                    {
                        cout<<"没有待命叉车，请稍等"<<endl;
                        sleep(1);
                        break;
                    }
                    while(error_num<3)
                    {
                        if(p->plan())
                        {
                            break;
                        }
                        else
                        {
                            error_num++;
                            cout<<"路径冲突，规划失败,重新规划"<<endl;
                            sleep(1);
                        }
                    }
                    if(error_num<3)
                        cmd_queue[i].state=EXECUTE;
                    /*** 将改变权值的路径复原 ***/
                    map<edge_descriptor,int>::iterator iter;
                    for(iter=origin_weight.begin();iter!=origin_weight.end();iter++)
                    {
                        p->m_Map.edgeprop[iter->first]=iter->second;
                    }
                    error_num=0;
                }
            }
        }
    }
}
/**** 规划路径图 ****/
bool Terminal::plan()
{
    vector<vector<struct inter_point>> temp(6);
    vector<struct inter_point>::iterator iter;
    struct inter_point temp_point;

    int i=0,j=0;

    ///*** 选取距离起点最近的待命叉车 赋值给current_num ***/
    //current_num=m_Map.dispatch(m_Map.reverse[lift_pick]);

    struct position origin=fork_Info[current_num].curr_pos;        //以该叉车的当前位置为起点，做路径规划
    if(!pathPlan(origin,temp))
    {
        cout<<"plan false"<<endl;
        return false;
    }

    namedWindow("trajectory");
/*** 画出轨迹图 判断是否正确 ***/
#if 1
    vector<struct inter_point>::iterator iter1;

    Mat img;
    m_Map.map_image.copyTo(img);
    Point pp;
    pp.x=m_Map.station[lift_pick].x;
    pp.y=m_Map.station[lift_pick].y;
    circle(m_Map.map_image,pp,1,Scalar(0,0,0),5);
    pp.x=m_Map.station[lift_put].x;
    pp.y=m_Map.station[lift_put].y;
    circle(m_Map.map_image,pp,1,Scalar(0,0,0),5);
    for(j;j<6;j++)
    {
        for(iter1=temp[j].begin();iter1!=temp[j].end()-1;iter1++)
        {
            pp.x=iter1->x/10.0;
            pp.y=iter1->y/10.0;
            circle(m_Map.map_image,pp,1,Scalar(0,0,255),1);
        }
    }
    //task[current_num]=true;
    map_complete[current_num]=true;

#endif
#if 0   //存点的轨迹为一个txt
    vector<struct inter_point>::iterator iter1;
    ofstream traj("trajectory.txt",ios::app);
    for(j;j<temp.size();j++)
    {
        for(iter1=temp[j].begin();iter1!=temp[j].end();iter1++)
        {

            traj<<iter1->x<<" ";
            traj<<iter1->y<<"\n";

        }
    }
    traj.close();
#endif
#if 1
    for(j=0;j<temp.size();j++)
    {
        for(iter=temp[j].begin();iter!=temp[j].end();iter++)
        {
            Terminal_Cmd[current_num].traj[i].x=iter->x;
            Terminal_Cmd[current_num].traj[i].y=iter->y;
            Terminal_Cmd[current_num].traj[i].v=iter->v;
            i++;
        }
    }
#endif
    /**** 更新地图状态：COMPLETE 更新叉车下一工作状态：RUNNING ****/
    _map_state=m_Map.dijk_map;
    Terminal_Cmd[current_num].map_state=m_Map.dijk_map;
    Terminal_Cmd[current_num].fork_action=MOVE;
    /**** 置cur_state为WAITING，可执行下一次任务分配 ****/
    cur_state=WAITING;
    next_state=WAITING;
    temp.clear();
    return true;
}
bool Terminal::pathPlan(struct position _origin,vector<vector<struct inter_point>> &temp)
{
    vector<struct inter_point>::reverse_iterator iter;

    //vector<vector<struct inter_point>> temp(6);
    struct inter_point end_flag={0xABCD,0xABCD,0xABCD};
    property_map < Graph,vertex_global_t>::type vertexprop = get(vertex_global, m_Map.m_Map);     // 获得节点属性
    /*** 完整流程，取货+下货--路径规划(分为6段)***/
    if(!m_Map.plan(_origin,m_Map.reverse[lift_pick],temp[0]))                     //当前位置->取货倒车点
    {
        return false;
    }
    temp[0].push_back(end_flag);
    temp[1]=m_Map.reverse_interpolate(m_Map.reverse[lift_pick],lift_pick);        //取货倒车点->取货点
    for(iter=temp[1].rbegin();iter!=temp[1].rend();iter++)
    {
        iter->v=0.6;                                                            //与倒车轨迹相反，速度变成正向
        temp[2].push_back(*iter);                                               //取货点->取货倒车点
    }
    temp[1].push_back(end_flag);
    temp[2].push_back(end_flag);
    if(!m_Map.plan(vertexprop[m_Map.reverse[lift_pick]],m_Map.reverse[lift_put],temp[3])) //取货倒车点->卸货倒车点
    {
        return false;
    }
    temp[3].push_back(end_flag);
    temp[4]=m_Map.reverse_interpolate(m_Map.reverse[lift_put],lift_put);        //卸货倒车点->卸货点
    for(iter=temp[4].rbegin();iter!=temp[4].rend();iter++)
    {
        iter->v=0.6;                                                            //与倒车轨迹相反，速度变成正向
        temp[5].push_back(*iter);                                               //卸货点->卸货倒车点
    }
    temp[4].push_back(end_flag);
    temp[5].push_back(end_flag);
    return true;
}
/**** 开启socket通信线程和输入命令读取线程 ****/
void Terminal::start()
{
    m_listener.start();

    /**** 开启输入命令读取线程 ****/
    pthread_create(&input_t,NULL,getCmd,NULL);
}
void* Terminal::getCmd(void *tmp)
{
    string _target;
    string _cmd_word;
    string input_Cmd;
    struct _cmd cmd;

    while(!quit)
    {
        _target.clear();
        _cmd_word.clear();
        cin>>input_Cmd;

        int pos1=input_Cmd.find("t");
        int lift_pick=atoi(input_Cmd.substr(0, pos1).c_str());
        _cmd_word=input_Cmd.substr(pos1, 2).c_str();
        int lift_put=atoi(input_Cmd.substr(pos1+2).c_str());

        if(_cmd_word=="to")
        {
            /*** task压入task_queue向量中 ***/
            cmd.pick=lift_pick;
            cmd.put=lift_put;
            cmd.state=WAIT;
            cmd_queue.push_back(cmd);
        }
    }
}