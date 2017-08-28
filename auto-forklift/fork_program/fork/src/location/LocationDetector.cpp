#include "../../include/location_base/LocationDetector.h"
#include <string>

LocationDetector::LocationDetector()
{
	string landmark="/opt/project/fork/src/location/landmark_map.txt";
	L=689.0;
	u0=935;
	v0=530;
	vector<Point2f> p_buf1(100);   //存储实际世界坐标
	vector<Point2f> lc_buf1(100);  //信标中心图像坐标
	vector<double>theta_buf1(100,0);     //实际theta值
	vector<double>theta1_buf1(100,0);   //信标特征在图像中的角度
	vector<int> id1(100,0);

	p_buf=p_buf1;
	lc_buf=lc_buf1;
	theta_buf=theta_buf1;
	theta1_buf=theta1_buf1;
	id=id1;

	std::ifstream landmark_map(landmark.c_str());
	if(!landmark_map.is_open())
	{
		cerr << "Failed to open settings file at: " << landmark << endl;
		exit(-1);
	}
	else
	{
		std::string line;
		//std::getline(landmark_map, line);
		while (std::getline(landmark_map, line))
		{
			double temp_double;
			int temp_int;
			std::istringstream in(line);
			for(int j=0; j<7;j++){
				if(j==0) {in>>temp_int;if(temp_int==13) cout<<"get 13"<<endl;id[temp_int]=temp_int;}
				else{
					in>>temp_double;
					if(j==1) p_buf[temp_int].x = temp_double;
					if(j==2) p_buf[temp_int].y = temp_double;
					if(j==5) theta_buf[temp_int] = temp_double;
					if(j==3) lc_buf[temp_int].x = temp_double;
					if(j==4) lc_buf[temp_int].y = temp_double;
					if(j==6) theta1_buf[temp_int] = temp_double;
				}
			}
                 
		}
               
	}
}
LocationDetector::~LocationDetector(void)
{
}
double LocationDetector::max(double a,double b)
{
	return a > b? a :b;
}
double LocationDetector::min(double a,double b)
{
	return a > b? b :a;
}
bool LocationDetector::PositionGet(const Mat& _frame, const vector<Point2f>& _ROI_P)
{
	MarkerDetector markCapture;
	Mat frameROI;
	vector <Marker> ROI_markers;
	vector <Point2f> vertex0;
	bool p_flag(false);
	Point2f lp ;
	Point2f lc ;
	double theta_x ;
	double theta3;
	Point2f rc ;
	double theta1 ;
	double k;
	double theta;

	if(_ROI_P.size()>0)
	{
		for(unsigned int i=0;i<_ROI_P.size();i++)
		{
			//cout<<ROI_P[i]<<endl;
			Mat frameROI2(_frame,Rect(max(_ROI_P[i].x-200,0.0),max(_ROI_P[i].y-200,0.0),min(_frame.cols-_ROI_P[i].x+200,400.0),min(_frame.rows-_ROI_P[i].y+200,400.0)));
			frameROI2.copyTo(frameROI);
			/*imshow("frameROI", frameROI);
			waitKey(33);*/
			markCapture.processFrame(frameROI);
			for(unsigned int j=0; j<markCapture.m_markers.size(); j++)
			{
				ROI_markers.push_back(markCapture.m_markers[j]);
				vertex0.push_back(Point2f(max(_ROI_P[i].x-200,0),max(_ROI_P[i].y-200,0)));
			}
		}
		ROI_Points.clear();
		//cout<<"1_size="<<ROI_markers.size()<<endl;
		if(ROI_markers.size()>0)
		{
			p_flag=true;
			int loc_num(-1);
			int loc_id(0);
			double min_dis(DBL_MAX);
			for(unsigned int i=0; i<ROI_markers.size(); i++)
			{
				ROI_Points.push_back(Point2f((ROI_markers[i].m_points[0].x+ROI_markers[i].m_points[2].x)/2+vertex0[i].x,
											 (ROI_markers[i].m_points[0].y+ROI_markers[i].m_points[2].y)/2+vertex0[i].y));
				double dis=sqrt((ROI_markers[i].m_points[0].x+vertex0[i].x-u0)*(ROI_markers[i].m_points[0].x+vertex0[i].x-u0)+
								(ROI_markers[i].m_points[0].y+vertex0[i].y-v0)*(ROI_markers[i].m_points[0].y+vertex0[i].y-v0));
				if (dis<min_dis)
				{
					min_dis=dis;
					loc_id=ROI_markers[i].id ;
					loc_num=i;
				}
			}
			//location
			if(id[loc_id]==0 || loc_id>id.size())
				p_flag=false;
			else
			{
				lp =p_buf[loc_id];
				lc = lc_buf[loc_id];
				theta_x = theta1_buf[loc_id];
				//cout<<theta_x<<endl;
				theta3= theta_buf[loc_id];

				rc = Point2f(ROI_markers[loc_num].m_points[0].x+vertex0[loc_num].x,ROI_markers[loc_num].m_points[0].y+vertex0[loc_num].y);
				if(((rc.x/(double)_frame.cols)<0.3)||((rc.x/(double)_frame.cols)>0.7)||((rc.y/(double)_frame.rows)<0.2)||((rc.y/(double)_frame.rows)>0.8))
					ROI_Points.clear();//if the distance between closest points to the center and the center point is large enough,reprocess the whole picture.
				theta1 = ROI_markers[loc_num].Ltheta;
				k=ROI_markers[loc_num].k;

				theta = -(theta1-theta_x);
				while (theta<-3.14)
					theta = theta+6.28;
				while (theta>3.14)
					theta = theta-6.28;
	//		cout<<"loc id = "<<loc_id<<endl;
			}
		}
	}//if(_ROI_P.size()>0)
	if(!p_flag)
	{
		markCapture.processFrame(_frame);
		int numMarker = markCapture.m_markers.size();
		//std::cout << numMarker<< std::endl;
		if (numMarker <=0)
		{
			std::cout << "no marker!" << std::endl;
			return false;
		}
		int loc_num(-1);
		int loc_id(0);
		double min_dis(DBL_MAX);
		// show marker
		for(unsigned int i=0; i<markCapture.m_markers.size(); i++)
		{
			p_flag=true;
			ROI_Points.push_back(Point2f((markCapture.m_markers[i].m_points[0].x+markCapture.m_markers[i].m_points[2].x)/2,
										 (markCapture.m_markers[i].m_points[0].y+markCapture.m_markers[i].m_points[2].y)/2));
			double dis=sqrt((markCapture.m_markers[i].m_points[0].x-u0)*(markCapture.m_markers[i].m_points[0].x-u0)+
							(markCapture.m_markers[i].m_points[0].y-v0)*(markCapture.m_markers[i].m_points[0].y-v0));
			if (dis<min_dis)
			{
				min_dis=dis;
				loc_id=markCapture.m_markers[i].id ;
				loc_num=i;
			}
		}
		if(id[loc_id]==0 || loc_id>id.size())
			p_flag=false;
		else
		{
			lp =p_buf[loc_id];
			lc = lc_buf[loc_id];
			theta_x = theta1_buf[loc_id];
			//cout<<theta_x<<endl;
			theta3= theta_buf[loc_id];

			rc = markCapture.m_markers[loc_num].m_points[0];
			theta1 = markCapture.m_markers[loc_num].Ltheta;
			k=markCapture.m_markers[loc_num].k;

			theta = -(theta1-theta_x);
			while (theta<-3.14)
				theta = theta+6.28;
			while (theta>3.14)
				theta = theta-6.28;
		}
	}//if(!p_flag) process the whole picture
	if(p_flag)
	{
		double pwx1 = k * ((lc.x - u0) - (rc.x - u0) * cos(theta) + (rc.y - v0) * sin(theta));
		double pwy1 = k * ((lc.y - v0) - (rc.x - u0) * sin(theta) - (rc.y - v0) * cos(theta));
		//cout<<"pwx1="<<pwx1<<"pwy1="<<pwy1<<endl;
		double pwx = pwx1 * cos(theta3) - pwy1 * sin(theta3) + lp.x;
		double pwy = pwx1 * sin(theta3) + pwy1 * cos(theta3) + lp.y;
		double p_angle = (theta + theta3);
		pwx=pwx-L*cos(p_angle);
		pwy=pwy-L*sin(p_angle);

		current_position.x = pwx;
		current_position.y = pwy;
		current_position.theta = p_angle;
		return true;
	}
	return false;
}

