#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "Marker.h"
#include <iostream> 
//#include<vector>
//#include<algorithm>
#include <math.h>
#include<fstream>
#include <stdio.h>
using namespace cv;
using namespace std; 
using std::pair;

class MarkerDetector
{
public:
	double duration;
	MarkerDetector(void);
	~MarkerDetector(void);
	void processFrame(const Mat& _frame);
	void getTransformations(void);
private:
	bool findMarkers(const Mat& _frame, vector<Marker>& _detectedMarkers, int threshold_para);
// 	void performThreshold(const Mat& _imgGray, Mat& _thresholdImg);
	void findMarkerContours(const Mat& _imgThreshold, vector<vector<Point> >& _contours, int _minContourPointsAllowed);
	void findMarkerCandidates(const vector<vector<Point> >& _contours, vector<Marker>& _detectedMarkers);
	void detectMarkers(const Mat& _imgGray, vector<Marker>& _detectedMarkers);
	void estimatePosition(vector<Marker>& _detectedMarkers);
private:
	float m_minContourLengthAllowed, _maxContourPointsAllowed;	// contour最小边长的阈值 = 100
	Size m_markerSize;
	vector<Point2f> m_markerCorners2d;	// marker 4个角点的正交投影标准值
	vector<Point3f> m_markerCorners3d;	
	Mat m_camMat;
	Mat color;
	Mat m_distCoeff;
	double m_area;
public:
	Mat m_imgGray;
	Mat m_imgThreshold;
	vector<vector<Point> > m_contours;
    int threshold_para_good;
    vector<int> thershold_paras;
public:
	vector<Marker> m_markers;
};
