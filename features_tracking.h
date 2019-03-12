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

private:

    vector<Point2f> next_edges,next_corners,corners0;
    TermCriteria termcrit;
    Size  winSize;
    vector<int> OutLiersMask;
    vector<uchar> status;
    vector<float> err;
    vector <Point3f> pred_edge;
    Mat pred;
    vector<Mat> next_points;

public:
    vector<Point2f> OpticalFlow_Homograhpy(Mat prevgray,Mat src_gray,vector<Point2f> corners,vector<Point2f> corners0,Mat& H);//,TermCriteria termcrit(), Size  winSize());
    void Show_OpticalFlow(int r,Mat src,vector<Point2f> cnrs,vector<Point2f> nextcorners);
    void Show_Detection(Mat,vector<Point2f>);
    vector<Point2f> Show_Tracking_Homography(Mat src, vector<Point2f>,Mat H);//,vector<Point>next_edges);
    vector<Point2f> Next_with_Homography(vector<Point2f>,Mat H);
    vector<Point2f> OpticalFlow_tracking_box(Mat src,Mat prevgray,Mat src_gray,vector<Point2f> edges);
    vector<Point2f> OpticalFlow_tracking_box_previous(Mat prevgray,Mat src_gray,vector<Point2f> edges);

    Features_Tracking();

};
#endif // FEATURES_TRACKING_H
