#include "../../include/location_base/MarkerDetector.h"

MarkerDetector::MarkerDetector(void)
{
	m_minContourLengthAllowed = 7000.0f;
	_maxContourPointsAllowed = 1000.0f;
	m_camMat = (Mat_<float>(3,3) << 1026.09176, 0, 953.90595,
									0, 1026.53527, 539.40119,
									0, 0, 1);
	m_distCoeff = (Mat_<float>(4,1) << -0.00628, 0.03946, 0.00005, 0.00183);
	
	m_markerSize = Size(360, 360);
	// 默认marker的size为100*100,markercorner在2d空间为100*100的矩形
	m_markerCorners2d.push_back(Point2f(0, 0));
	m_markerCorners2d.push_back(Point2f(m_markerSize.width-1, 0));
	m_markerCorners2d.push_back(Point2f(m_markerSize.width-1, m_markerSize.height-1));
	m_markerCorners2d.push_back(Point2f(0, m_markerSize.height-1));

	// 3d corner所在坐标为以marker中心为原点 exhibition room 360.0mm,2# building 400.0mm,accurate position 80mm
	m_markerCorners3d.push_back(cv::Point3f(-40.f,40.f,0.f));
	m_markerCorners3d.push_back(cv::Point3f(40.f,40.f,0.f));
	m_markerCorners3d.push_back(cv::Point3f(40.f,-40.f,0.f));
	m_markerCorners3d.push_back(cv::Point3f(-40.f,-40.f,0.f));

    //二值化参数
    threshold_para_good = 0;
    for(int i=0; i<11; i++)
       thershold_paras.push_back(20+i*20);
}

MarkerDetector::~MarkerDetector(void)
{
}

void MarkerDetector::processFrame(const Mat& _frame)
{
	m_markers.clear();
    if(threshold_para_good!=0)
        if(!findMarkers(_frame, m_markers,threshold_para_good))
            threshold_para_good = 0;
    if(threshold_para_good == 0)
    for(int i=0; i<11; i++)
    {
        if(findMarkers(_frame, m_markers,thershold_paras[i]))
        {
            threshold_para_good = thershold_paras[i];
            break;
        }

    }
	//estimatePosition(m_markers);
}

void MarkerDetector::getTransformations(void)
{
}

bool MarkerDetector::findMarkers(const Mat& _frame, vector<Marker>& _detectedMarkers, int threshold_para)
{
	// 转为灰度图
	cvtColor(_frame, m_imgGray, CV_BGR2GRAY);
	_frame.copyTo(color);

	threshold(m_imgGray, m_imgThreshold, threshold_para, 255, cv::THRESH_BINARY_INV);
// 	adaptiveThreshold(m_imgGray, m_imgThreshold, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 121, 5);

	/********************* 检测边缘**********************/ 
	findMarkerContours(m_imgThreshold, m_contours, m_imgGray.cols/8);

	/********************* 筛选contours，选择那些又4点围成的contour，得到候选marker**********************/ 
	findMarkerCandidates(m_contours, _detectedMarkers);

	/********************* 检测marker，根据marker的id信息筛选**********************/ 
	detectMarkers(m_imgGray, _detectedMarkers);

	if(_detectedMarkers.size()<=0)
		return false;
	else
		return true;
}

// void MarkerDetector::performThreshold(const Mat& _imgGray, Mat& _thresholdImg)
// {
// }

void MarkerDetector::findMarkerContours(const Mat& _imgThreshold, vector<vector<Point> >& _contours, int _minContourPointsAllowed)
{
	Mat imgTemp = _imgThreshold.clone();

	vector<vector<Point> > allContours;
	findContours(imgTemp, allContours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	// 做一个筛选,如果一个contour的点的个数比较少,不是一个好contour
	_contours.clear();
	for (size_t i=0; i<allContours.size(); i++)
	{
		int contourSize = allContours[i].size();
		if (contourSize > _minContourPointsAllowed && contourSize < _maxContourPointsAllowed)
		{
			//cout<<"contoursize="<<contourSize<<endl;
			_contours.push_back(allContours[i]);
		}
	}
}

void MarkerDetector::findMarkerCandidates(const vector<vector<Point> >& _contours, vector<Marker>& _detectedMarkers)
{
	vector<Point> approxCurve;
	vector<Marker> markerPossible;
	
	for (size_t i=0; i<_contours.size(); i++)
	{
		// 得到近似多边形
		approxPolyDP(_contours[i], approxCurve, double(_contours[i].size())*0.01, true);
		// 我们只关心那些4边形的轮廓，因为只有他们才可能是marker
		if (approxCurve.size() != 4)
			continue;
		// 并且是凸4边形
		if (!isContourConvex(approxCurve))
			continue;
		// 找个4个边最小的边
		float minDist = FLT_MAX;
		float minRatio = FLT_MAX;
		float maxRatio = FLT_MIN;
		Point vecTemp = approxCurve[0] - approxCurve[1];
		float distSquared0 = vecTemp.dot(vecTemp);
		for (int i=0; i<4; i++)
		{
			Point vecTemp = approxCurve[i] - approxCurve[(i+1)%4];
			float distSquared = vecTemp.dot(vecTemp);
			minDist=std::min(minDist, distSquared);
			float ratio=distSquared0/distSquared;
			minRatio = std::min(minRatio, ratio);
			maxRatio = std::max(maxRatio, ratio);
		}
		/*cout<<"minDist="<<minRatio<<endl;
		cout<<"maxDist="<<minDist<<endl;*/
		// 如果这个最小边不够长，这个contour不是marker
		if (minRatio > 0.6 && maxRatio < 1.4 && minDist>m_minContourLengthAllowed)
		{
			Marker markerTemp;
			for (int i=0; i<4; i++)
			{
				markerTemp.m_points.push_back(Point2f(approxCurve[i].x, approxCurve[i].y));
			}
			markerPossible.push_back(markerTemp);
		}
	}
	// 把得到的markerPossible按照顺时针排序
	for (size_t i=0; i<markerPossible.size(); i++)
	{
		// 检测第三个点在一二点组成的线的左边还是右边
		
		Point v1 = markerPossible[i].m_points[1] - markerPossible[i].m_points[0];
		Point v2 = markerPossible[i].m_points[2] - markerPossible[i].m_points[0];
		double theta = (v1.x * v2.y) - (v1.y * v2.x);
		//cout<<theta<<endl;
		// 左边为逆时针，右边顺时针，变换2、4点顺序
		if (theta < 0.0)
		{
			std::swap(markerPossible[i].m_points[1], markerPossible[i].m_points[3]);
		}
		/*circle(color, markerPossible[i].m_points[0], 3, Scalar(255,0,0), 2, 1);
		circle(color, markerPossible[i].m_points[1], 3, Scalar(0,255,0), 2, 22);
		imshow("circle:",color);
		waitKey(0);*/
	}
	// 找到corner非常接近的contour,放到tooNearCandidates
	vector<pair<int, int> > tooNearCandidates;
	for (size_t i=0; i<markerPossible.size(); i++)
	{
		for (size_t j=i+1; j<markerPossible.size(); j++)
		{
			float distSquared = 0.0f;
			for (int k=0; k<4; k++)
			{
				Point vec = markerPossible[i].m_points[k] - markerPossible[j].m_points[k];
				distSquared += vec.dot(vec);
			}
			//cout<<distSquared<<endl;
			if (distSquared < 2200)
			{
				tooNearCandidates.push_back(pair<int, int>(i,j));
			}
		}
	}
	// 选择tooNearCadidates中周长更小的作为移除的对象
	vector<bool> markerRemoveIndex(markerPossible.size(), false);
	for (size_t i=0; i<tooNearCandidates.size(); i++)
	{
		float length1 = markerPossible[tooNearCandidates[i].first].calPerimeter();
		float length2 = markerPossible[tooNearCandidates[i].second].calPerimeter();
		markerRemoveIndex[(length1>length2) ? tooNearCandidates[i].second : tooNearCandidates[i].first] = true;
	}
	// 去掉markerRemoveIndex得到最终的候选者
	_detectedMarkers.clear();
	for (size_t i=0; i<markerPossible.size(); i++)
	{
		if (!markerRemoveIndex[i])
		{
			_detectedMarkers.push_back(markerPossible[i]);
			//cout<<"周长："<<endl<<markerPossible[i].calPerimeter()<<endl;
		}
	}

}

void MarkerDetector::detectMarkers(const Mat& _imgGray, vector<Marker>& _detectedMarkers)
{
	Mat canonicalImg;	// 去除投影变换恢复的3维视图
	vector<Marker> goodMarkers;

	// 根据id，验证marker
	for (size_t i=0; i<_detectedMarkers.size(); i++)
	{
		// 得到当前marker的透视变换矩阵M
		Mat M = getPerspectiveTransform(_detectedMarkers[i].m_points, m_markerCorners2d);
		// 将当前的marker变换为正交投影
		
		warpPerspective(_imgGray, canonicalImg, M, m_markerSize);

		// 解码marker
		int nRotations;
		int idMarker = Marker::decode(canonicalImg, nRotations);
		//cout<<"idMarker:"<<endl<<idMarker<<endl;
		if (idMarker != -1)
		{
			_detectedMarkers[i].id = idMarker;
			// 将对应的corner point也进行旋转
			//cout<<"*********"<<endl<<_detectedMarkers[i].m_points<<endl;
			
			//circle(frame, markCapture.m_markers[i].m_points[1], 3, Scalar(0,255,255), 2, 8);
			std::rotate(_detectedMarkers[i].m_points.begin(), _detectedMarkers[i].m_points.begin()+4-nRotations, _detectedMarkers[i].m_points.end());
			//cout<<"*********"<<endl<<_detectedMarkers[i].m_points<<endl;
			goodMarkers.push_back(_detectedMarkers[i]);
			//cout<<M<<endl;
		}
	}

	// 用subpixel提高marker corner的精度
	if (goodMarkers.size() > 0)
	{
		// 把各corner整理为一个数组，因为cornerSubPix函数很慢，尽力不要多次调用
		vector<Point2f> preciseCorners(4*goodMarkers.size());

		for (size_t i=0; i<goodMarkers.size(); i++)
		{
			for (int j=0; j<4; j++)
			{
				preciseCorners[4*i+j] = goodMarkers[i].m_points[j];
			}
		}

		cornerSubPix(_imgGray, preciseCorners, Size(5,5), Size(-1,-1), cv::TermCriteria(CV_TERMCRIT_ITER, 30, 0.1));

		for (size_t i=0; i<goodMarkers.size(); i++)
		{
			for (int j=0; j<4; j++)
			{
				goodMarkers[i].m_points[j] = preciseCorners[4*i+j];
			}
		}
	}

	_detectedMarkers = goodMarkers;
	for (size_t i=0; i<goodMarkers.size(); i++)
	{
		_detectedMarkers[i].Llocation=_detectedMarkers[i].m_points[0];
		double lx = _detectedMarkers[i].m_points[1].x - _detectedMarkers[i].m_points[0].x;
		double ly = _detectedMarkers[i].m_points[1].y - _detectedMarkers[i].m_points[0].y;
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
		_detectedMarkers[i].Ltheta=theta1;
		_detectedMarkers[i].k=360.0/sqrt(lx*lx+ly*ly);
		//cout<<"l=="<<sqrt(lx*lx+ly*ly)<<endl;
		//cout<<"k=="<<_detectedMarkers[i].k<<endl;
		/*line(color,_detectedMarkers[i].m_points[1],_detectedMarkers[i].m_points[0],Scalar(0,100,250),2, CV_AA);
		circle(color,_detectedMarkers[i].m_points[0],4,Scalar(255,100,0), CV_FILLED, CV_AA);
		imshow("666",color);
		waitKey(0);*/
	}
}

void MarkerDetector::estimatePosition(vector<Marker>& _detectedMarkers)
{
	for (size_t i=0; i<_detectedMarkers.size(); i++)
	{
		Mat Rvec;
// 		Mat_<float> Tvec;
		Vec3f Tvec;
		Mat raux, taux;
		// 建立一个3d to 2d的映射		
		solvePnP(m_markerCorners3d, _detectedMarkers[i].m_points, m_camMat, m_distCoeff, raux, taux);
		cout<<"raux"<<raux<<endl;
		raux.convertTo(Rvec, CV_32F);
		taux.convertTo(Tvec, CV_32F);
		
// 		Mat_<float> rotMat(3,3);
		Matx33f rotMat;
		Rodrigues(Rvec, rotMat);

		cout<<"rotMat"<<endl<<rotMat<<endl;
		cout<<"Tvec"<<endl<<Tvec<<endl;

		double rt[16]={0,0,0,0,
						0,0,0,0,
						0,0,0,0,
						0,0,0,1};
		Mat rtM(4,4,CV_64FC1, rt);
		for(int ii=0; ii<3; ii++)
		{
			for(int jj=0; jj<3; jj++)
			{
				rtM.at<double>(ii,jj)=rotMat(ii,jj);
			}
		}
		rtM.at<double>(0,3)=Tvec[0];
		rtM.at<double>(1,3)=Tvec[1];
		rtM.at<double>(2,3)=Tvec[2];
		//cout<<"rtM"<<endl<<rtM<<endl;
		Mat rtMv(4,4,CV_64FC1);
		rtMv=rtM.inv();
		cout<<"rtMv"<<endl<<rtMv<<endl;
		
		ofstream ofs("1.txt",ios::app);
		if(ofs==NULL)
		{
			cout << "create file failed" << endl;
				
		}
		ofs << rtMv.at<double>(0,3) << " ";
		ofs << rtMv.at<double>(1,3) << "\n";
		ofs.close();

		_detectedMarkers[i].m_rotation = rotMat.t();
		_detectedMarkers[i].m_translation = -Tvec;
	}
}
