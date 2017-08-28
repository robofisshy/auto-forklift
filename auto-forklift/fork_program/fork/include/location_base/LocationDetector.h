/************************************************************************
* Copyright(c) 2017  stardraw
* All rights reserved.
*
* File:	LocationDetector.h
* Brief: 检测坐标
* Version: 1.0
* Author: Li Yuehua
* Email: 570208816@qq.com
* Date:	2017/2/16 13:45
* History:
************************************************************************/
#ifndef STARDRAW_FORKLIFT_LOCATION_DETECTOR_H
#define STARDRAW_FORKLIFT_LOCATION_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <algorithm>
#include <math.h>
#include <fstream>
#include <stdio.h>
#include "MarkerDetector.h"
#include "../pub_inc.h"


using namespace cv;
using namespace std;

class LocationDetector
{
public:
	position current_position;
	vector<Point2f> ROI_Points;
	LocationDetector();
	~LocationDetector(void);
	bool PositionGet(const Mat& _frame, const vector<Point2f>& _ROI_P);

private:

	double max(double a,double b);
	double min(double a,double b);
	int u0;
	int v0;
	vector<Point2f>p_buf;   //存储实际世界坐标
	vector<Point2f>lc_buf;  //信标中心图像坐标
	vector<double>theta_buf;     //实际theta值
	vector<double>theta1_buf;   //信标特征在图像中的角度
	vector<int> id;
	double L;//folk parameter
};

#endif
