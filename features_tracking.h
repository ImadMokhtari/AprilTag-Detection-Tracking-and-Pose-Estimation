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
    vector<Mat> Show_Tracking(Mat src, vector<Point>,Mat H);//,vector<Point>next_edges);

    vector<Point2f> OpticalFlow_tracking_box(Mat src,Mat prevgray,Mat src_gray,vector<Point2f> edges);
    Features_Tracking();
};
#endif // FEATURES_TRACKING_H