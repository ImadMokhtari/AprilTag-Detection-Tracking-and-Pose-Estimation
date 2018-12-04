#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include<opencv2/videoio.hpp>
#include<opencv2/calib3d/calib3d.hpp>
#include<opencv2/calib3d.hpp>
#include<opencv2/video/tracking.hpp>
#include<iostream>


#ifndef FEATURES_TRACKING_H
#define FEATURES_TRACKING_H
using namespace std;
using namespace cv;


class Features_Tracking
{
//private:

   // Mat src_gray,prevgray,src;
    //vector<Point2f> corners0,corners;
public:
    vector<Point2f> OpticalFlow_Homograhpy(Mat prevgray,Mat src_gray,vector<Point2f> corners,vector<Point2f> corners0);//,TermCriteria termcrit(), Size  winSize());
    void Show_OpticalFlow(int r,Mat src,vector<Point2f> cnrs,vector<Point2f> nextcorners);
    Features_Tracking();
};

#endif // FEATURES_TRACKING_H
