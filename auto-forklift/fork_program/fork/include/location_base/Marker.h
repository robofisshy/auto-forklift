/************************************************************************
* Copyright(c) 2017  stardraw
* All rights reserved.
*
* File:	Marker.h
* Brief: 标签类,实现marker图和对应ID之间的转换
* Version: 1.0
* Author: Li Yuehua
* Email: 570208816@qq.com
* Date:	2017/2/16 13:45
* History:
************************************************************************/
#ifndef STARDRAW_FORKLIFT_MARKER_H
#define STARDRAW_FORKLIFT_MARKER_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Marker.h"
#include <iostream>

using namespace cv;
using namespace std;

class Marker
{
public:
	Marker(void);
	~Marker(void);
	static int decode(Mat& _input, int& _numRotation);
	float calPerimeter();
private:
	static Mat rotate(const Mat& _input);
	static int hammDistMarker(const Mat& _code);
	static int code2ID(const Mat& _code);
private:
	static const int m_idVerify[4][5];	// marker 设计的id
public:
	int id;	// marker 解码的id
	double Ltheta;
	Point2f Llocation;
	double k;
	vector<Point2f> m_points;	// marker的contour信息
	Matx33f m_rotation;
	Vec3f m_translation;
};

#endif