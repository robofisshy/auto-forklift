//
// Created by chen on 17-3-9.
//
#include "../../include/utils/utils.h"

#define pi 3.141592653

double utils::getDistance(Point2f p1,Point2f p2)
{
    double dis=sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
    return dis;
}
double utils::getX_theta(Point2f p1,Point2f p2)//p2 指向 P1
{
    double lx = p1.x - p2.x;
    double ly = p1.y - p2.y;
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
    return theta1;
}
double utils::getX_theta(struct inter_point p1,struct inter_point p2)//p2 指向 P1
{
    double lx = p1.x - p2.x;
    double ly = p1.y - p2.y;
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
    return theta1;
}
double utils::getX_theta(struct inter_point p1,Point2f p2)//p2 指向 P1
{
    double lx = p1.x - p2.x;
    double ly = p1.y - p2.y;
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
    return theta1;
}


double utils::check_theta(double alpha, int direction)
{
    direction = 1;
    double alpha_(alpha);
    while (alpha_<-3.14)
        alpha_ = alpha+6.28;
    while (alpha_>3.14)
        alpha_ = alpha-6.28;
    if((alpha_>1.57) || (alpha_<-1.57))
    {
        alpha_=alpha_>0?(pi-alpha_):(-pi-alpha_);
        direction = -1;
    }
    return alpha_;
}
double utils::check_theta(double alpha)
{
    double alpha_(alpha);
    while (alpha_<-3.14)
        alpha_ = alpha+6.28;
    while (alpha_>3.14)
        alpha_ = alpha-6.28;
    if((alpha_>1.57) || (alpha_<-1.57))
    {
        alpha_=alpha_>0?(pi-alpha_):(-pi-alpha_);
    }
    return alpha_;
}
long utils::getCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}
