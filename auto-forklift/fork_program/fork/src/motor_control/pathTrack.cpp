//
// Created by yyq on 17-3-16.
//
#include "../../include/motor_control/pathTrack.h"
#include <opencv2/opencv.hpp>
#include "../../include/utils/utils.h"
#define d_lookahear_min 300
#define d_lookahear_max 2000

using namespace cv;
struct pid lift_pid={0.0,       //err
                     0.0,       //e_sum
                     0.0,       //e_last
                     0.0,       //e_pre
                     1.5,       //p
                     0.0,       //i
                     0.0};      //d
double lift_velocity;
int noMarker_times=0;
pose getpose()
{
    boost::unique_lock<boost::shared_mutex> lock(poseupdate);
    return currentPosition;
}
bool work(Forklift &_fork,vector<vector<inter_point> > traj)
{
    int i=0;
    double threhold=2;
    double height_s=station_height[_fork.station_s];
    double height_g=station_height[_fork.station_g];    

    for(i;i<traj.size();i++)
    {
       if( pathTrack(_fork,traj[i],100))
	{
#if 0
		if(i==0)
		{
			while(abs(lift_encoder-height_s*1000)>threhold && !quit)
                	{
                    		lift_pid.err=height_s-(lift_encoder/1000.0);
                    		lift_pid.e_sum+=lift_pid.err;
                    		/*** pid计算出货叉速度 ***/
                    		lift_velocity=lift_pid.p*lift_pid.err+lift_pid.i*lift_pid.e_sum+lift_pid.d*(lift_pid.e_last-lift_pid.e_pre);
                    		lift_pid.e_pre=lift_pid.e_last;
                    		lift_pid.e_last=lift_pid.err;

                    		if(lift_velocity>0)
                        		carry=_up;
                    		else
                        		carry=_down;
				//cout<<"velocity:"<<lift_velocity<<endl;
                    		usleep(1000);
                	}
                	carry=_stop;
//			carry=_up;
//			sleep(2);
//			carry=_stop;
		}
		if(i==1)
		{
			lift_velocity=0.15;
			carry=_up;
			sleep(1);
			carry=_stop;
		}
		if(i==2)
		{
			while(abs(lift_encoder-(height_g+0.05)*1000)>threhold && !quit)
                	{	
                    		lift_pid.err=(height_g+0.05)-(lift_encoder/1000.0);
                    		lift_pid.e_sum+=lift_pid.err;
                    		/*** pid计算出货叉速度 ***/
                    		lift_velocity=lift_pid.p*lift_pid.err+lift_pid.i*lift_pid.e_sum+lift_pid.d*(lift_pid.e_last-lift_pid.e_pre);
                   		lift_pid.e_pre=lift_pid.e_last;
                    		lift_pid.e_last=lift_pid.err;

                    		if(lift_velocity>0)
                        		carry=_up;
                    		else
                        		carry=_down;
                    		usleep(100000);
                	}
                	carry=_stop;
//			carry=_down;
//			sleep(5);
//			carry=_stop;
		}
		if(i==3)
		{
			while(abs(lift_encoder-height_g*1000)>threhold && !quit)
                        {
                                lift_pid.err=height_g-(lift_encoder/1000.0);
                                lift_pid.e_sum+=lift_pid.err;
                                /*** pid计算出货叉速度 ***/
                                lift_velocity=lift_pid.p*lift_pid.err+lift_pid.i*lift_pid.e_sum+lift_pid.d*(lift_pid.e_last-lift_pid.e_pre);
                                lift_pid.e_pre=lift_pid.e_last;
                                lift_pid.e_last=lift_pid.err;

                                if(lift_velocity>0)
                                        carry=_up;
                                else
                                        carry=_down;
                                usleep(100000);
                        }
                        carry=_stop;
		}
#endif
	}
//	else
//	{
//		break;
//	}
	usleep(50000);
    }
    return true;
}
vector<inter_point> optimal(vector<inter_point> in, vector<double>& segs)
{
    vector<inter_point> out;
    out.push_back(in.front());
    for(int i=1; i<in.size(); i++)
    {
        double dis = utils::getDistance(Point2f(in[i-1].x,in[i-1].y),Point2f(in[i].x,in[i].y));
        //除去太接近的点(重复点)
        if(dis>=25)
        {
            out.push_back(in[i]);
            segs.push_back(dis);
        }
    }
    return out;
}
double Dot2( double x1, double y1, double x2, double y2) {
    return (x1*x2 + y1*y2); // cross product
}
double Dist(double x1, double y1, double x2, double y2) {
    double diff_x = (x2 - x1);
    double diff_y = (y2 - y1);
    return sqrt( diff_x*diff_x + diff_y*diff_y );
}
//获取p到so-s1最近点，若p向线段做垂线，在外，则取s0，或者s1，若在内，则取垂足，计算距离
double DistP2S( pose current_position, inter_point s0, inter_point s1, inter_point& Pb){
    double vx,vy, wx, wy;
    double c1, c2, di, b;

    vx = s1.x - s0.x;
    vy = s1.y - s0.y;

    wx = current_position.position_.x - s0.x;
    wy = current_position.position_.y - s0.y;

    c1 = Dot2( wx, wy, vx, vy );

    if ( c1 <= 0.0 ) {
        di = Dist(current_position.position_.x, current_position.position_.y, s0.x, s0.y);
        Pb.x = s0.x;
        Pb.y = s0.y;
        return di;
    }
    c2 = Dot2(vx,vy, vx, vy);
    if ( c2 <= c1 ) {
        //printf("kanban::DistP2S: c2 <= c1\n");
        di = Dist(current_position.position_.x, current_position.position_.y, s1.x, s1.y);
        Pb.x = s1.x;
        Pb.y = s1.y;
        return di;
    }
    b = c1 / c2;
    Pb.x = s0.x + b * vx;
    Pb.y = s0.y + b * vy;

    di = Dist(current_position.position_.x, current_position.position_.y, Pb.x, Pb.y);
    return di;
}

void UpdateLookAhead(double dLinearSpeed, double& dLookAhead){
    double aux_lookahead = fabs(dLinearSpeed*1200);
    if(dLinearSpeed>0)
	aux_lookahead = 1600;
    if(dLinearSpeed<0)
        aux_lookahead = 600;
    double desired_lookahead = 0.0;
    double inc = 150;	// incremento del lookahead
    if(aux_lookahead < d_lookahear_min)
        desired_lookahead = d_lookahear_min;
    else if(aux_lookahead > d_lookahear_max)
        desired_lookahead = d_lookahear_max;
    else{
        desired_lookahead = aux_lookahead;
    }

    if((desired_lookahead -0.001) > dLookAhead){
        dLookAhead+= inc;
    }else if((desired_lookahead +0.001) < dLookAhead)
        dLookAhead-= inc;
}

bool pathTrack(Forklift &_fork,vector<inter_point> &_traj, double accuracy)
{
    double accel_lim_v(0.05);
    double accel_lim_w(0.1);
    double last_v(0.0);
    double last_w(1.1513);
    double alpha_cmd(1.1513);
    double velocity_cmd(0.0);
    double kp(0.0);
    double ki(0.0);
    double kv(0.001);
    double ka(2);
    double kb(-3);
    double dLookAhead(300);
    double LocationAccuracy(accuracy);
    double last_alpha_error(0.0);
    double cross_track_error(0.0);
    double turn_alpha(0.0);
    double dis_forklift_trackpoint;
    vector<double>segs;
    Point2d Goal_Point(_traj.back().x, _traj.back().y);
    pose forklift_pose;
    int find_near_num(0);
    int track_num(0);
    int last_track_num(0);
    inter_point vertical_point;
    //轨迹点优化
    vector<inter_point> traj_opt = optimal(_traj,segs);
        
   _fork.motor.m_speed.v=0;
   _fork.motor.m_speed.w = 1.1513;
   _fork.motor.Turn(alpha_cmd);
    sleep(1);

    while(!quit)
    {
        //1:获取叉车定位数据,并检验定位数据时间
        forklift_pose = getpose();
    //    cout<<"Current position "<<forklift_pose.position_.x<<","<<forklift_pose.position_.y<<" , "<<forklift_pose.position_.theta<<endl;
        _fork.mthread.m_Info.curr_pos.x=forklift_pose.position_.x;
        _fork.mthread.m_Info.curr_pos.y=forklift_pose.position_.y;
	    _fork.mthread.m_Info.curr_pos.theta=forklift_pose.position_.theta;
	long time_diff = utils::getCurrentTime() - currentPosition.time;
#if 1
        if((time_diff > 2000)||(time_diff * last_v > 1000))
        //if(noMarker)
	{
            _fork.motor.m_speed.v = 0;
            _fork.motor.m_speed.w = 1.1513;
//	    _fork.motor.m_speed.v=0.3;
  //          _fork.motor.m_speed.w=last_w;
	    // noMarker_times++;
            //cout<<"pose data is old"<<endl;
            sleep(1);
	    //cout<<"no marker,v=last_v"<<endl;
	    //if(noMarker_times>100)
	    //{
	    //	_fork.motor.m_speed.v=0;
	    //	noMarker_times=0;
	    //	usleep(500000);		
	    // }	
            continue;
        }
#endif
	noMarker_times=0;
        //更新前向跟踪距离
        UpdateLookAhead(last_v, dLookAhead);
      //  cout<<"lookadead = "<<dLookAhead<<endl;
        //2:获取跟踪点,并计算跟踪方向
        double last_dis=DBL_MAX;
        // intfind_near_num(0);
        double track_alpha(0.0);
        //get nearest point
        for(int i=find_near_num;i<traj_opt.size();i++)
        {
            double dis = utils::getDistance(Point2f(forklift_pose.position_.x, forklift_pose.position_.y), Point2f(traj_opt[i].x,traj_opt[i].y));
            if(dis<last_dis)
            {
                last_dis=dis;
               find_near_num=i;
               // cout<<"The shortest dis: "<<dis<<endl;
            } else
                break;
        }
        if(last_dis>3000)
        {
            cout<<"path tracking error too big, please check it out "<<endl;
            _fork.motor.m_speed.v =0;
            return false;
        }
        inter_point lateral;
        if(find_near_num>1)
        {
            double d1(DBL_MAX),d2(DBL_MAX);
            inter_point lateral1,lateral2;
            d1 = DistP2S(forklift_pose,traj_opt[find_near_num-1],traj_opt[find_near_num], lateral1);
            if(find_near_num<traj_opt.size()-1)
                d2 = DistP2S(forklift_pose,traj_opt[find_near_num],traj_opt[find_near_num+1], lateral2);
            if(d1>d2)
            {
                cross_track_error = d2;
                lateral = lateral2;
            } else{
                cross_track_error = d1;
                lateral = lateral1;
            }

        } else{
            cross_track_error = last_dis;
            lateral = traj_opt[find_near_num];
        }
        //get tracking point
        double d=0;
        track_num = find_near_num;
        while ( (d < dLookAhead) && (track_num < traj_opt.size()-1 ) ) {
            // searched point on this segment
            d = d + segs[track_num];
            track_num++;
        }
        if(track_num<last_track_num)
             track_num = last_track_num;
        last_track_num = track_num;
        dis_forklift_trackpoint = utils::getDistance(Point2f(forklift_pose.position_.x, forklift_pose.position_.y), Point2f(traj_opt[track_num].x,traj_opt[track_num].y));

        if(track_num>1)
            track_alpha=utils::getX_theta(traj_opt[track_num],traj_opt[track_num-1]);
        else
            track_alpha=utils::getX_theta(traj_opt[track_num+1],traj_opt[track_num]);
       // cout<<"Tracking point "<<traj_opt[track_num].x<<","<<traj_opt[track_num].y<<endl;
       // cout<<"track_num: "<<track_num<<endl;
	    _fork.mthread.m_Info.next_point.x=traj_opt[track_num].x;
	    _fork.mthread.m_Info.next_point.y=traj_opt[track_num].y;
        //3:计算控制转角
        double alpha_forklift_trackpoint=utils::getX_theta(traj_opt[track_num],Point2f(forklift_pose.position_.x, forklift_pose.position_.y));//p后 指向 P前
        double alpha= utils::check_theta(alpha_forklift_trackpoint- forklift_pose.position_.theta);
        double alpha_error = utils::check_theta(track_alpha - forklift_pose.position_.theta);//转角误差
	    if(dis_forklift_trackpoint<1000)
		    dis_forklift_trackpoint = 1000;
        if(dis_forklift_trackpoint>2000)
		    dis_forklift_trackpoint = 2000;

	//    cout<<"alpha_fork_track = "<<alpha_forklift_trackpoint<<endl;
	//    cout<<"puresuit alpha = "<<alpha<<endl;
        velocity_cmd = traj_opt[track_num].v;
        turn_alpha = atan(2*1357*sin(alpha)/dis_forklift_trackpoint);

        turn_alpha = turn_alpha + kp*alpha_error + ki*(alpha_error-last_alpha_error) ;
        last_alpha_error = alpha_error;

        if((abs(turn_alpha))>1.221)
        {
            turn_alpha=alpha>0?1.221:(-1.221);
        }
        //if(turn_alpha>0)
        //    cout<<"turn left"<<endl;
        //else
        //    cout<<"turn right"<<endl;
        alpha_cmd = 1.1513 - turn_alpha;
        alpha_cmd = (alpha_cmd-last_w) > 0 ? min(last_w+accel_lim_w, alpha_cmd ) : max(last_w-accel_lim_w , alpha_cmd);
        velocity_cmd = (velocity_cmd-last_v)>0?min(last_v + accel_lim_v,velocity_cmd):max(last_v - accel_lim_v,velocity_cmd);

        //4:判断是否到达目的地,输出控制命令
        double dist_to_goal=utils::getDistance(Point2f(forklift_pose.position_.x, forklift_pose.position_.y), Goal_Point);
        if(dist_to_goal>LocationAccuracy)
        {
            if(abs(alpha_cmd-last_w)<0.05)
                alpha_cmd = last_w;
            _fork.motor.m_speed.v = velocity_cmd;
            _fork.motor.m_speed.w =  alpha_cmd;
            if(forward_alarm==SAFE||forward_alarm==WARN)
            	_fork.motor.Turn(alpha_cmd);
	//    cout<<"speed = "<<velocity_cmd<<endl;
            last_w = alpha_cmd;
            last_v = velocity_cmd;
        }
        else
        {
            _fork.motor.m_speed.v=0;
            _fork.motor.m_speed.w = 1.1513;
	    _fork.motor.Turn(1.1513);
            last_v=0;
            last_w = 1.1513;
            return true;
        }
        usleep(100000);//20Hz
    }

}
