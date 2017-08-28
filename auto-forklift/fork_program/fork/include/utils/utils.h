//
// Created by chen on 17-3-9.
//

#ifndef L_MOVE_UTILS_H
#define L_MOVE_UTILS_H
#include "../../include/location_base/LocationDetector.h"
#include "../../include/location_base/MarkerDetector.h"
#include "../pub_inc.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <math.h>
#include <bitset>
#include <string>
#include <pthread.h>
#include <signal.h>
#include <sys/time.h>
using namespace cv;
using namespace std;

class utils
{
public:
    static double getDistance(Point2f p1,Point2f p2);
    static double getX_theta(Point2f p1,Point2f p2);
    static double getX_theta(struct inter_point p1,struct inter_point p2);
    static double getX_theta(struct inter_point p1,Point2f p2);
    static double check_theta(double alpha, int direction);
    static double check_theta(double alpha);
    static long getCurrentTime();
};
#endif //L_MOVE_UTILS_H
