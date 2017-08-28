#include "include/Terminal/Terminal.h"
#include <signal.h>

using namespace std;

boost::mutex pathClear;
boost::mutex dispatch;

bool quit=false;
//position cur_pos{0,0.1,0};
struct inter_point clear_traj[1000]={0,0,0};

void handle_signal(int sig)
{
    quit=true;
}
void decide(Terminal &t,int num)
{
//    cout<<num<<endl;
    switch(t.cur_state){
        case WAITING:
            if(t.fork_state==ARRIVE)
            {
                Terminal_Cmd[num].map_state=BLANK;
                Terminal_Cmd[num].fork_action=PREPARE;
                t.map_complete[num]=false;
                task_queue[num].clear();
                /*** 删除执行完成的指令 ***/
//                vector<struct _cmd>::iterator iter;
//                for(iter=cmd_queue.begin();iter!=cmd_queue.end();iter++)
//                {
//                    if(iter->state==EXECUTE)
//                        cmd_queue.erase(iter);
//                }
                cout<<"Arrive target,please input next command"<<endl;
                sleep(1);
                //destroyWindow("trajectory");
            }
            if(t.fork_state==WORKING)
            {
                usleep(500);
                {
                    boost::unique_lock<boost::mutex> lock(pathClear);
                    Terminal_Cmd[num].map_state=COMPLETE;

                   // memset(t.m_Com.m_Cmd.traj,0,sizeof(struct inter_point)*1000);
                }

            }

            break;
        case MAPPING:
            ;
    }
    //t.cur_state=t.next_state;
}
int main()
{
    signal(SIGINT,handle_signal);

    Terminal _t;
    _t.init();
    _t.start();

    while(!quit)
    {
        Point pos_;
        Point track_;
        Mat img;
        _t.m_Map.map_image.copyTo(img);
        for(int i=0;i<connected_fork;i++)
        {
            int num=fork_queue[i];
            _t.fork_state=fork_Info[num].curr_work_state;

            //if(_t.cur_state==WAITING)
            //    if(_t.next_state==MAPPING&&_t.fork_state==STANDBY)
            //        _t.plan();

            pos_.x=fork_Info[num].curr_pos.x/10.0;
            pos_.y=fork_Info[num].curr_pos.y/10.0;

            //cout<<"curr_pos:"<<pos_.x<<" "<<pos_.y<<endl;


            track_.x=fork_Info[num].next_point.x/10.0;
            track_.y=fork_Info[num].next_point.y/10.0;

            //cout<<"track_pos:"<<track_.x<<" "<<track_.y<<endl;
            //Mat img;
            //_t.m_Map.map_image.copyTo(img);
            if(_t.map_complete[num])
            {
                if(!((pos_.x==0)&&(pos_.y==0)))
                    circle(img,pos_,1,Scalar(fork_color[num].b,fork_color[num].g,fork_color[num].r),8);
                if(!((track_.x==0)&&(track_.y==0)))
                    circle(img,track_,1,Scalar(0,255,0),2);
            }
            //if(task[num]==true)
            decide(_t,num);
            usleep(500);
        }
        imshow("trajectory",img);
        waitKey(5);
    }
    return 0;
}