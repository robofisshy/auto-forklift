#include <iostream>
#include <signal.h>
#include <boost/thread.hpp>

#include "../include/motor_control/pathTrack.h"
#include "../include/forklift/forklift.h"
#include "../include/utils/utils.h"

bool quit=false;
bool noMarker=false;

position start_point={10,10,5};
boost::shared_mutex poseupdate;
boost::shared_mutex readCMD;
struct pose currentPosition={0,0,0};

void handle_signal(int sig)
{
    quit=true;
}
bool camera_set(VideoCapture& pCap)
{
   if(pCap.get(CV_CAP_PROP_FRAME_WIDTH)!=1920)
   {
          pCap.set(CV_CAP_PROP_FRAME_WIDTH,1920);
          pCap.set(CV_CAP_PROP_FRAME_HEIGHT,1080);

   }
    Mat frame;
    if(!pCap.isOpened())
    {
        return false;
    }
    bool bSuccess = pCap.read(frame);
    unsigned int test_n;
    test_n=0;
    while(!bSuccess)
    {
        bSuccess = pCap.read(frame);
        test_n++;
        if(test_n>100)
        {
            cout<<"camera error"<<endl;
            return false;
        }
    }
    return true;
}
void *location(void *m)
{
    Mat frame;
    vector <Point2f> ROI_P;
    ROI_P.clear();
    Forklift *_fork=(Forklift*)m;
    //打开摄像头并设置摄像头参数
    VideoCapture pCap(-1);
    if(camera_set(pCap))
    {
        while(!quit) {
            pCap >> frame;
	    noMarker=false;
            if (!_fork->m_locationDetector.PositionGet(frame, ROI_P)) {
            //    cout<<"******forklift position is not update******"<<endl;
                noMarker=true;
		continue;
            }
            ROI_P = _fork->m_locationDetector.ROI_Points;
            {
                boost::unique_lock<boost::shared_mutex> lock(poseupdate);
                currentPosition.position_ = _fork->m_locationDetector.current_position;
                currentPosition.time = utils::getCurrentTime();
//                cout<<currentPosition.position_.x<<","<<currentPosition.position_.y<<endl;
            }
	    cout<<currentPosition.position_.x<<" "<<currentPosition.position_.y<<" "<<currentPosition.position_.theta<<endl;
        }
        pCap.release();
    }
}

void *monitor(void *tmp)
{
    while(1);
}
void stop(Forklift &_fork)
{
    _fork.motor.m_speed.v=0;
    _fork.motor.m_speed.w=1.3083;
}
/**** 根据当前状态和下一状态进行决断**
 **** 使用有限状态机模型  **********/
void decision(Forklift &_fork)
{
    switch (_fork.cur_work_state){
        case WORKING:
            if(_fork._map_state==COMPLETE)
            {
                if(_fork.m_action==MOVE)
                {
                    /*** 完整的工作流程分为4段轨迹 ***/
                    memcpy(_fork.trajectory,_fork.mthread.m_Cmd.traj,sizeof(_fork.mthread.m_Cmd.traj));
                    vector<vector <struct inter_point> > traj(4);

                    int i=0,j,flag=0;
                    while(i<4)
                    {
                        for(j=flag;_fork.trajectory[j].x!=0xABCD && _fork.trajectory[j].y!=0xABCD;j++)
                        {
                            traj[i].push_back(_fork.trajectory[j]);
                        }
                        i++;
                        flag=j+1;
                    }
                    if(work(_fork, traj))
                    {
                        cout<<"Success"<<endl;
                        _fork.cur_work_state=ARRIVE;
                    }

                }
                if(_fork.m_action==CEASE)
                {
                    stop(_fork);
                }
            }
            if(_fork.m_action==CEASE)
            {
                stop(_fork);
                _fork.cur_work_state=STANDBY;
            }
            break;
        case ARRIVE:
            stop(_fork);
            if(_fork.m_action==PREPARE)
            {
                _fork.cur_work_state=STANDBY;
            }
            _fork._map_state=BLANK;
            usleep(50000);
            break;
        case STANDBY:
	    //cout<<"*****----Im STANDBY----*****"<<endl;
            if(_fork._map_state==COMPLETE)
            {
                if(_fork.m_action==MOVE)
                {
                    _fork.cur_work_state=WORKING;
                }
                if(_fork.m_action==CEASE)
                {
                    stop(_fork);
                }
            }
            if(_fork.m_action==CEASE)
            {
                stop(_fork);
            }
            break;
        case STOP:
            stop(_fork);
    }
}

int main() {
    signal(SIGINT,handle_signal);
    void* tret;
    pthread_t monitor_t;
    //实例化叉车对象
    Forklift forklift(BLANK,STANDBY);

    //start函数启动socket接收线程和发送线程、及电机控制线程
    forklift.start();
    Forklift *m=&forklift;
    sleep(1);
   // pthread_create(&monitor_t,NULL,monitor,m);
    boost::thread location_thread(boost::bind(location,m));
    int n=sizeof(forklift.mthread.m_Cmd);
    while(!quit)
    {
        {
            boost::shared_lock<boost::shared_mutex> lock(readCMD);
            forklift.curr_pos=start_point;

            forklift.m_action=forklift.mthread.m_Cmd.fork_action;
	    forklift.station_s=forklift.mthread.m_Cmd.station_s;
	    forklift.station_g=forklift.mthread.m_Cmd.station_g;
            forklift._map_state=forklift.mthread.m_Cmd.map_state;

            forklift.mthread.m_Info.curr_pos=currentPosition.position_;
            /**** 将当前状态赋值给m_Info类成员，于socket_tx线程中汇报至终端 ****/
            forklift.mthread.m_Info.curr_work_state=forklift.cur_work_state;
	   // cout<<forklift.mthread.m_Info.curr_pos.x<<" "<<forklift.mthread.m_Info.curr_pos.y<<endl;
        }

        //根据终端命令（状态和目标点）,做出决断
        decision(forklift);
        usleep(50000);
    }
    location_thread.join();

    return 0;
}
