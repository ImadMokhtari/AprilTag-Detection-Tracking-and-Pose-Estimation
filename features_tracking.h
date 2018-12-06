#include <opencv2/highgui.hpp>
#include<opencv2/calib3d.hpp>
#include<opencv2/video/tracking.hpp>
#include<iostream>


#ifndef FEATURES_TRACKING_H
#define FEATURES_TRACKING_H

using namespace std;
using namespace cv;

class Features_Tracking
{
public:
    vector<Point2f> OpticalFlow_Homograhpy(Mat prevgray,Mat src_gray,vector<Point2f> corners,vector<Point2f> corners0,Mat& H);//,TermCriteria termcrit(), Size  winSize());
    void Show_OpticalFlow(int r,Mat src,vector<Point2f> cnrs,vector<Point2f> nextcorners);
    void Show_Detection(Mat,vector<Point>);
    void Show_Tracking(Mat src, vector<Point>,Mat H);
    Features_Tracking();
};
#endif // FEATURES_TRACKING_H
